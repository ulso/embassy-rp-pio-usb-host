#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]

//! Product-neutral CDC-ACM acceptance firmware for the Adafruit Feather
//! RP2040 USB Host.
//!
//! This binary performs standard Embassy enumeration, allocates the first
//! descriptor-compliant CDC-ACM function, and only exercises controls
//! advertised by that function. Product protocols belong in named Cargo
//! examples such as `examples/bleuio.rs`.

#[cfg(target_os = "none")]
use defmt::{info, warn};
#[cfg(target_os = "none")]
use defmt_rtt as _;
#[cfg(target_os = "none")]
use embassy_executor::Spawner;
#[cfg(target_os = "none")]
use embassy_futures::join::join;
#[cfg(target_os = "none")]
use embassy_futures::select::{Either, select};
#[cfg(target_os = "none")]
use embassy_rp::bind_interrupts;
#[cfg(target_os = "none")]
use embassy_rp::clocks::ClockConfig;
#[cfg(target_os = "none")]
use embassy_rp::dma::InterruptHandler as DmaInterruptHandler;
#[cfg(target_os = "none")]
use embassy_rp::gpio::{Level, Output};
#[cfg(target_os = "none")]
use embassy_rp::peripherals::{DMA_CH0, PIO0, PIO1};
#[cfg(target_os = "none")]
use embassy_rp::pio::InterruptHandler as PioInterruptHandler;
#[cfg(target_os = "none")]
use embassy_rp_pio_usb_host::cdc_acm::allocate_from_enumeration;
#[cfg(target_os = "none")]
use embassy_rp_pio_usb_host::host::{DeviceEvent, Speed, UsbHostController};
#[cfg(target_os = "none")]
use embassy_rp_pio_usb_host::pio_host::PioHostState;
#[cfg(target_os = "none")]
use embassy_rp_pio_usb_host::pio_host::rp2040::Rp2040PioEngine;
#[cfg(target_os = "none")]
use embassy_rp_pio_usb_host::{AttachDetector, BusEvent, DeviceSpeed};
#[cfg(target_os = "none")]
use embassy_time::{Duration, Ticker, Timer, with_timeout};
#[cfg(target_os = "none")]
use embassy_usb_host::{BusController, BusRoute, BusState};
#[cfg(target_os = "none")]
use panic_probe as _;

#[cfg(target_os = "none")]
bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
    PIO1_IRQ_0 => PioInterruptHandler<PIO1>;
    DMA_IRQ_0 => DmaInterruptHandler<DMA_CH0>;
});

#[cfg(target_os = "none")]
const SYS_CLOCK_HZ: u32 = 120_000_000;
#[cfg(target_os = "none")]
const ATTACH_DEBOUNCE_SAMPLES: u16 = 100;
#[cfg(target_os = "none")]
const CONFIG_DESCRIPTOR_CAPACITY: usize = 512;
#[cfg(target_os = "none")]
const CONTROL_PROBE_TIMEOUT: Duration = Duration::from_secs(2);
#[cfg(target_os = "none")]
const LED_TICK: Duration = Duration::from_millis(100);

#[cfg(target_os = "none")]
#[derive(Clone, Copy)]
enum LedPattern {
    /// Two pulses: a generic CDC-ACM session is ready.
    Ready,
    /// Three pulses: standard USB enumeration failed.
    EnumerationError,
    /// Four pulses: no usable CDC-ACM function could be allocated.
    CdcAllocationError,
    /// Five pulses: advertised CDC-ACM standard controls failed.
    StandardControlError,
}

#[cfg(target_os = "none")]
impl LedPattern {
    const fn pulse_count(self) -> u8 {
        match self {
            Self::Ready => 2,
            Self::EnumerationError => 3,
            Self::CdcAllocationError => 4,
            Self::StandardControlError => 5,
        }
    }

    fn level(self, phase: u8) -> Level {
        let pulse_count = self.pulse_count();
        let pulse_phase = phase % 20;
        Level::from(pulse_phase < pulse_count * 2 && pulse_phase.is_multiple_of(2))
    }
}

/// Monitor the physical root port and keep an idle full-speed bus alive.
///
/// Samples are deliberately omitted while reset drives SE0. Passing those
/// samples through the debouncer would turn reset recovery into a false
/// second attachment.
#[cfg(target_os = "none")]
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
                            // Do not catch up ticker ticks spent in reset.
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
            // Busy transfers own the engine and service their own frame edge.
            let _ = host_state.service_frame().await;
        }
    }
}

#[cfg(target_os = "none")]
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
            Either::Second(_) => phase = (phase + 1) % 20,
        }
    }

    status_led.set_low();
}

#[cfg(target_os = "none")]
async fn run() {
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

    // GPIO18 is a logic input to the board's current-limited 5 V load switch.
    // It must never drive USB VBUS directly.
    Timer::after_millis(100).await;
    vbus_enable.set_high();
    info!("USB VBUS enabled; LED off means waiting for a device");

    let application = async move {
        // Keep the load-switch enable asserted for the application's lifetime.
        let _vbus_enable = vbus_enable;

        loop {
            status_led.set_low();
            let speed = controller.wait_for_connection().await;
            if speed != Speed::Full {
                warn!("unsupported root-device speed");
                continue;
            }

            // Solid LED: attachment accepted; enumeration/probe in progress.
            status_led.set_high();
            info!("enumerating full-speed root device");

            let mut configuration = [0_u8; CONFIG_DESCRIPTOR_CAPACITY];
            let (enumeration, configuration_len) = match bus_handle
                .enumerate(BusRoute::Direct(speed), &mut configuration)
                .await
            {
                Ok(result) => result,
                Err(_) => {
                    warn!("enumeration failed; three-pulse LED until detach");
                    wait_for_root_removal(
                        &mut controller,
                        &mut status_led,
                        LedPattern::EnumerationError,
                    )
                    .await;

                    // embassy-usb-host 0.1 does not release its private
                    // address lease on every enumeration error path. With one
                    // direct root device and no hubs, detach safely releases
                    // the complete address space.
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
                match allocate_from_enumeration(
                    &bus_handle,
                    &configuration[..configuration_len],
                    &enumeration,
                ) {
                    Ok(mut cdc) => {
                        cdc.reset_data_toggles();

                        let pattern = if cdc.function().supports_line_requests() {
                            // Round-trip the device's current line coding
                            // instead of assuming a product-specific baud
                            // rate. Deasserting DTR/RTS leaves this neutral
                            // probe without an open application session.
                            let standard_controls = with_timeout(CONTROL_PROBE_TIMEOUT, async {
                                let coding = cdc.get_line_coding().await.map_err(|_| ())?;
                                cdc.set_line_coding(coding).await.map_err(|_| ())?;
                                cdc.set_control_line_state(false, false)
                                    .await
                                    .map_err(|_| ())?;
                                Ok::<_, ()>(coding)
                            })
                            .await;

                            match standard_controls {
                                Ok(Ok(coding)) => {
                                    info!(
                                        "CDC-ACM ready; retained {} bit/s, {} data bits",
                                        coding.data_terminal_rate, coding.data_bits
                                    );
                                    LedPattern::Ready
                                }
                                Ok(Err(())) => {
                                    warn!("advertised CDC-ACM standard controls failed");
                                    LedPattern::StandardControlError
                                }
                                Err(_) => {
                                    warn!("CDC-ACM standard-control probe timed out");
                                    LedPattern::StandardControlError
                                }
                            }
                        } else {
                            info!("CDC-ACM ready; line requests are not advertised");
                            LedPattern::Ready
                        };

                        match pattern {
                            LedPattern::Ready => info!("two-pulse LED until detach"),
                            LedPattern::StandardControlError => {
                                warn!("five-pulse LED until detach")
                            }
                            LedPattern::EnumerationError | LedPattern::CdcAllocationError => {}
                        }

                        // Keep all class pipes allocated and the generic CDC
                        // session alive until the physical device is removed.
                        wait_for_root_removal(&mut controller, &mut status_led, pattern).await;
                    }
                    Err(_) => {
                        warn!("no usable CDC-ACM function; four-pulse LED until detach");
                        wait_for_root_removal(
                            &mut controller,
                            &mut status_led,
                            LedPattern::CdcAllocationError,
                        )
                        .await;
                    }
                }
            }

            // Every pipe is out of scope before the address returns to Embassy.
            bus_handle.free_address(address);
            info!("root address {} released", address);
        }
    };

    // Both futures borrow stack-local host/bus state. `run` never returns.
    join(root_port_monitor(&host_state), application).await;
}

#[cfg(target_os = "none")]
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    run().await;
}

#[cfg(not(target_os = "none"))]
fn main() {}
