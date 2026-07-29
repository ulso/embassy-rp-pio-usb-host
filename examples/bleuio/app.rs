//! Embassy USB-host integration example for a CDC-ACM BleuIO dongle.
//!
//! The former wire-level M5i implementation is retained in
//! `legacy_m5i.rs`. This application exercises the reusable PIO host
//! controller, Embassy enumeration, the generic CDC-ACM class, and the
//! transport-independent BleuIO protocol client.

#[path = "protocol.rs"]
mod bleuio_protocol;

use bleuio_protocol::BleuIo;
use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_futures::select::{Either, select};
use embassy_rp::bind_interrupts;
use embassy_rp::clocks::ClockConfig;
use embassy_rp::dma::InterruptHandler as DmaInterruptHandler;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{DMA_CH0, PIO0, PIO1};
use embassy_rp::pio::InterruptHandler as PioInterruptHandler;
use embassy_rp_pio_usb_host::cdc_acm::allocate_from_enumeration;
use embassy_rp_pio_usb_host::host::{DeviceEvent, PipeError, Speed, UsbHostController};
use embassy_rp_pio_usb_host::pio_host::PioHostState;
use embassy_rp_pio_usb_host::pio_host::rp2040::{HandshakeObservationDiagnostic, Rp2040PioEngine};
use embassy_rp_pio_usb_host::usb::{CdcLineCoding, PID_ACK, PID_NAK, PID_STALL, SYNC};
use embassy_rp_pio_usb_host::{AttachDetector, BusEvent, DeviceSpeed};
use embassy_time::{Duration, Ticker, Timer, with_timeout};
use embassy_usb_host::{BusController, BusRoute, BusState, EnumerationError};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
    PIO1_IRQ_0 => PioInterruptHandler<PIO1>;
    DMA_IRQ_0 => DmaInterruptHandler<DMA_CH0>;
});

const SYS_CLOCK_HZ: u32 = 120_000_000;
const ATTACH_DEBOUNCE_SAMPLES: u16 = 100;
const CONFIG_DESCRIPTOR_CAPACITY: usize = 512;
const SESSION_TIMEOUT: Duration = Duration::from_secs(10);
const LED_TICK: Duration = Duration::from_millis(100);

#[derive(Clone, Copy)]
enum LedPattern {
    /// Two pulses: enumeration, CDC-ACM, and the BLE scan succeeded.
    Success,
    /// Coded enumeration failure; see [`enumeration_error_pattern`].
    EnumerationError(u8),
    /// A long marker followed by stage, failure, length, prefix, edge-PC,
    /// GPIO-input, and GPIO-override groups.
    BadResponseDiagnostic {
        site: u8,
        detail: u8,
        length: u8,
        prefix: u8,
        edge: u8,
        input: u8,
        input_override: u8,
    },
    /// Five pulses: CDC allocation or the BleuIO exchange failed.
    SessionError,
}

impl LedPattern {
    const fn pulse_count(self) -> u8 {
        match self {
            Self::Success => 2,
            Self::EnumerationError(pulses) => pulses,
            Self::BadResponseDiagnostic { site, .. } => site,
            Self::SessionError => 5,
        }
    }

    fn level(self, phase: u8) -> Level {
        if let Self::BadResponseDiagnostic {
            site,
            detail,
            length,
            prefix,
            edge,
            input,
            input_override,
        } = self
        {
            let diagnostic_phase = phase % 160;
            let in_marker = diagnostic_phase < 5;
            let in_site = in_pulse_group(diagnostic_phase, 10, site);
            let in_detail = in_pulse_group(diagnostic_phase, 31, detail);
            let in_length = in_pulse_group(diagnostic_phase, 52, length);
            let in_prefix = in_pulse_group(diagnostic_phase, 73, prefix);
            let in_edge = in_pulse_group(diagnostic_phase, 94, edge);
            let in_input = in_pulse_group(diagnostic_phase, 115, input);
            let in_input_override = in_pulse_group(diagnostic_phase, 136, input_override);
            return Level::from(
                in_marker
                    || in_site
                    || in_detail
                    || in_length
                    || in_prefix
                    || in_edge
                    || in_input
                    || in_input_override,
            );
        }

        let pulse_count = self.pulse_count();
        let pulse_phase = phase % 20;
        Level::from(pulse_phase < pulse_count * 2 && pulse_phase.is_multiple_of(2))
    }
}

fn in_pulse_group(phase: u8, start: u8, pulses: u8) -> bool {
    let group_phase = phase.saturating_sub(start);
    phase >= start && group_phase < pulses * 2 && group_phase.is_multiple_of(2)
}

fn observation_codes(observation: Option<HandshakeObservationDiagnostic>) -> (u8, u8, u8, u8, u8) {
    let Some(observation) = observation else {
        return (1, 1, 8, 8, 8);
    };

    // One-based length encoding keeps zero bytes visible. Code eight means
    // seven or more bytes, which is already invalid for a handshake.
    let length = observation.len.min(7) + 1;
    let prefix = if observation.len == 0 {
        1
    } else if observation.bytes[0] != SYNC {
        2
    } else if observation.len == 1 {
        3
    } else {
        match observation.bytes[1] {
            PID_ACK => 4,
            PID_NAK => 5,
            PID_STALL => 6,
            pid if (pid >> 4) == ((!pid) & 0x0f) => 7,
            _ => 8,
        }
    };
    let edge = match observation.edge_program_counter {
        15 => 1,
        16 => 2,
        17 => 3,
        18..=24 => 4,
        25..=26 => 5,
        27..=29 => 6,
        30..=31 => 7,
        _ => 8,
    };
    let input = match observation.input_snapshot {
        // Expected full-speed idle J with both peripheral inputs inverted.
        0x12 => 1,
        // Full-speed idle J without the expected input inversion.
        0x11 => 2,
        // Correctly inverted SE0, K, and SE1 line states.
        0x03 => 3,
        0x21 => 4,
        0x30 => 5,
        // Other complete non-inverted line-state snapshots.
        0x00 | 0x22 | 0x33 => 6,
        // Pad and peripheral samples disagree on only one input.
        0x01 | 0x02 | 0x10 | 0x13 | 0x20 | 0x23 | 0x31 | 0x32 => 7,
        _ => 8,
    };
    let input_override = match observation.input_override_snapshot {
        // Both D+ and D- use INVERT.
        0x05 => 1,
        // Both D+ and D- use NORMAL.
        0x00 => 2,
        // Only D+ or only D- uses INVERT.
        0x01 => 3,
        0x04 => 4,
        // At least one input is forced LOW or HIGH.
        override_bits if override_bits & 0x0a != 0 => 5,
        _ => 8,
    };
    (length, prefix, edge, input, input_override)
}

fn enumeration_error_pattern(
    error: &EnumerationError,
    bad_response_code: Option<(u8, u8, u8, u8, u8, u8, u8)>,
) -> LedPattern {
    if matches!(error, EnumerationError::Transfer(PipeError::BadResponse))
        && let Some((site, detail, length, prefix, edge, input, input_override)) = bad_response_code
    {
        return LedPattern::BadResponseDiagnostic {
            site,
            detail,
            length,
            prefix,
            edge,
            input,
            input_override,
        };
    }

    let pulses = match error {
        EnumerationError::NoPipe => 3,
        EnumerationError::InvalidDescriptor | EnumerationError::ConfigBufferTooSmall(_) => 4,
        EnumerationError::RequestFailed => 6,
        EnumerationError::Transfer(PipeError::Timeout) => 7,
        EnumerationError::Transfer(PipeError::BadResponse) => 8,
        EnumerationError::Transfer(_) => 9,
    };
    LedPattern::EnumerationError(pulses)
}

/// Monitor the physical root port and keep idle full-speed frames alive.
///
/// Pin samples are deliberately ignored while the host drives reset. Feeding
/// reset SE0 into the debouncer would make the returning full-speed J state
/// look like a second attachment.
async fn root_port_monitor<'d>(host_state: &PioHostState<Rp2040PioEngine<'d>>) {
    let mut detector = AttachDetector::new(ATTACH_DEBOUNCE_SAMPLES);
    let mut ticker = Ticker::every(Duration::from_millis(1));
    let mut connected = false;

    loop {
        ticker.next().await;

        let Some(line_state) = host_state.root_line_state_if_not_resetting() else {
            continue;
        };

        if let Some(event) = detector.update(line_state) {
            match event {
                BusEvent::Attached(DeviceSpeed::Full) => {
                    info!("full-speed root device attached; resetting");
                    match host_state.reset_and_report_connected(Speed::Full).await {
                        Ok(()) => {
                            connected = true;
                            // Do not catch up the ticker ticks spent in reset.
                            ticker = Ticker::every(Duration::from_millis(1));
                            info!("root reset complete");
                        }
                        Err(_) => {
                            connected = false;
                            warn!("root reset failed");
                        }
                    }
                }
                BusEvent::Attached(DeviceSpeed::Low) => {
                    warn!("low-speed root device is not supported");
                    if connected && host_state.report_disconnected_if_not_resetting() {
                        connected = false;
                    }
                }
                BusEvent::Detached => {
                    if connected && host_state.report_disconnected_if_not_resetting() {
                        connected = false;
                        info!("root device detached");
                    }
                }
                BusEvent::Invalid => {
                    warn!("stable SE1 detected on the root port");
                    if connected && host_state.report_disconnected_if_not_resetting() {
                        connected = false;
                    }
                }
            }
        }

        if connected {
            // A busy transfer owns the engine and emits its own SOFs. The
            // adapter therefore treats a failed try-lock as successful idle
            // service.
            let _ = host_state.service_frame().await;
        }
    }
}

async fn wait_for_root_removal<'d, C>(
    controller: &mut BusController<'d, C>,
    status_led: &mut Output<'_>,
    pattern: LedPattern,
) where
    C: UsbHostController<'d>,
{
    let mut phase = 0_u8;
    let mut ticker = Ticker::every(LED_TICK);

    loop {
        status_led.set_level(pattern.level(phase));
        match select(controller.wait_for_device_event(), ticker.next()).await {
            Either::First(event) => match event {
                DeviceEvent::Disconnected | DeviceEvent::Overcurrent => break,
                DeviceEvent::Connected(_) => {}
                _ => {}
            },
            Either::Second(_) => phase = (phase + 1) % 160,
        }
    }

    status_led.set_low();
}

pub async fn run(_spawner: Spawner) {
    let clocks = ClockConfig::system_freq(SYS_CLOCK_HZ).expect("valid 120 MHz PLL setup");
    let peripherals = embassy_rp::init(embassy_rp::config::Config::new(clocks));

    let mut status_led = Output::new(peripherals.PIN_13, Level::Low);
    let mut vbus_enable = Output::new(peripherals.PIN_18, Level::Low);

    let engine = Rp2040PioEngine::new(
        peripherals.PIO0,
        peripherals.PIO1,
        peripherals.DMA_CH0,
        peripherals.PIN_16,
        peripherals.PIN_17,
        Irqs,
        Irqs,
        Irqs,
    );
    let host_state = PioHostState::new(engine);
    let bus_state = BusState::new();
    let controller = host_state
        .controller()
        .expect("one Embassy root controller");
    let (mut controller, bus_handle) = embassy_usb_host::bus(controller, &bus_state);
    let application_host_state = &host_state;

    // GPIO18 is a logic input to the board's current-limited 5 V load switch.
    // It must never drive USB VBUS directly.
    Timer::after_millis(100).await;
    vbus_enable.set_high();
    info!("USB VBUS enabled; LED off means waiting for a device");

    let application = async move {
        // Keep ownership of the output for the lifetime of the application.
        let _vbus_enable = vbus_enable;

        loop {
            status_led.set_low();
            let speed = controller.wait_for_connection().await;
            if speed != Speed::Full {
                warn!("unsupported root-device speed");
                continue;
            }

            // Solid LED: attachment accepted; enumeration/session in progress.
            status_led.set_high();
            info!("enumerating full-speed root device");

            // Give the independent root-port runner a visible pre-enumeration
            // window in analyzer traces. It should emit about 100 idle SOFs;
            // their presence separates frame-service failures from EP0
            // enumeration failures without changing the class-driver path.
            Timer::after_millis(100).await;
            application_host_state.clear_bad_response_diagnostic().await;

            let mut configuration = [0_u8; CONFIG_DESCRIPTOR_CAPACITY];
            let (enumeration, configuration_len) = match bus_handle
                .enumerate(BusRoute::Direct(speed), &mut configuration)
                .await
            {
                Ok(result) => result,
                Err(error) => {
                    let bad_response_code = application_host_state
                        .take_bad_response_diagnostic()
                        .await
                        .map(|diagnostic| {
                            let (length, prefix, edge, input, input_override) =
                                observation_codes(diagnostic.handshake_observation);
                            (
                                diagnostic.site.diagnostic_code(),
                                diagnostic
                                    .handshake_failure
                                    .map(|failure| failure.diagnostic_code())
                                    .unwrap_or(0),
                                length,
                                prefix,
                                edge,
                                input,
                                input_override,
                            )
                        });
                    let pattern = enumeration_error_pattern(&error, bad_response_code);
                    match pattern {
                        LedPattern::BadResponseDiagnostic {
                            site,
                            detail,
                            length,
                            prefix,
                            edge,
                            input,
                            input_override,
                        } => warn!(
                            "enumeration failed; diagnostic stage {}, detail {}, length {}, prefix {}, edge {}, input {}, override {}",
                            site, detail, length, prefix, edge, input, input_override
                        ),
                        _ => warn!(
                            "enumeration failed; {}-pulse LED until detach",
                            pattern.pulse_count()
                        ),
                    }
                    wait_for_root_removal(&mut controller, &mut status_led, pattern).await;

                    // embassy-usb-host 0.1 does not release its private
                    // address lease on every enumeration error path. This
                    // backend currently supports one direct root device and
                    // no hubs, so physical root detach safely releases the
                    // entire address space.
                    for address in 1_u8..=127 {
                        bus_handle.free_address(address);
                    }
                    continue;
                }
            };

            let address = enumeration.device_address;
            info!(
                "device {} configured; allocating generic CDC-ACM host",
                address
            );

            {
                let (pattern, _live_client) = match allocate_from_enumeration(
                    &bus_handle,
                    &configuration[..configuration_len],
                    &enumeration,
                ) {
                    Ok(mut cdc) => {
                        let result = with_timeout(SESSION_TIMEOUT, async move {
                            // Preserve the order verified by the legacy M5i
                            // capture before entering the generic stream client.
                            cdc.set_control_line_state(true, true)
                                .await
                                .map_err(|_| ())?;
                            cdc.set_line_coding(CdcLineCoding::eight_n_one(115_200))
                                .await
                                .map_err(|_| ())?;
                            cdc.reset_data_toggles();

                            let mut bleuio = BleuIo::new(cdc);
                            bleuio.attention().await.map_err(|_| ())?;
                            bleuio.set_central().await.map_err(|_| ())?;
                            let scan = bleuio.gap_scan().await.map_err(|_| ())?;
                            Ok::<_, ()>((scan, bleuio))
                        })
                        .await;

                        match result {
                            Ok(Ok((scan, bleuio))) => {
                                info!(
                                    "BLE scan succeeded: device index {}, address type {}, RSSI {}",
                                    scan.index, scan.address_type, scan.rssi
                                );
                                (LedPattern::Success, Some(bleuio))
                            }
                            Ok(Err(())) => {
                                warn!("BleuIO command exchange failed");
                                (LedPattern::SessionError, None)
                            }
                            Err(_) => {
                                warn!("BleuIO session timed out");
                                (LedPattern::SessionError, None)
                            }
                        }
                    }
                    Err(_) => {
                        warn!("CDC-ACM discovery or pipe allocation failed");
                        (LedPattern::SessionError, None)
                    }
                };

                match pattern {
                    LedPattern::Success => info!("two-pulse LED until detach"),
                    LedPattern::EnumerationError(_) => {}
                    LedPattern::BadResponseDiagnostic { .. } => {}
                    LedPattern::SessionError => warn!("five-pulse LED until detach"),
                }
                wait_for_root_removal(&mut controller, &mut status_led, pattern).await;
            }

            // The inner scope has dropped every pipe before the address is
            // returned to Embassy.
            bus_handle.free_address(address);
            info!("root address {} released", address);
        }
    };

    // Both futures borrow stack-local host/bus state. `run` never returns, so
    // no static allocation is required.
    join(root_port_monitor(&host_state), application).await;
}
