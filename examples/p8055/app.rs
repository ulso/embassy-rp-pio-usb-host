//! First low-speed HID acceptance example for a Velleman K8055/P8055.

#[path = "protocol.rs"]
mod p8055_protocol;

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
use embassy_rp_pio_usb_host::hid::allocate_from_enumeration;
use embassy_rp_pio_usb_host::host::{DeviceEvent, Speed, UsbHostController};
use embassy_rp_pio_usb_host::pio_host::PioHostState;
use embassy_rp_pio_usb_host::pio_host::rp2040::Rp2040PioEngine;
use embassy_rp_pio_usb_host::{AttachDetector, BusEvent, DeviceSpeed};
use embassy_time::{Duration, Ticker, Timer, with_timeout};
use embassy_usb_host::{BusController, BusRoute, BusState};
use p8055_protocol::{InputReport, OutputState, REPORT_LEN, is_original_k8055};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
    PIO1_IRQ_0 => PioInterruptHandler<PIO1>;
    DMA_IRQ_0 => DmaInterruptHandler<DMA_CH0>;
});

const SYS_CLOCK_HZ: u32 = 120_000_000;
const ATTACH_DEBOUNCE_SAMPLES: u16 = 100;
const CONFIG_DESCRIPTOR_CAPACITY: usize = 128;
const REPORT_DESCRIPTOR_CAPACITY: usize = 256;
const TRANSFER_TIMEOUT: Duration = Duration::from_secs(2);
const LED_TICK: Duration = Duration::from_millis(100);

#[derive(Clone, Copy)]
enum LedPattern {
    /// Two pulses: low-speed enumeration and one HID IN/OUT exchange worked.
    Success,
    /// Three pulses: standard USB enumeration failed.
    EnumerationError,
    /// Four pulses: the enumerated device is not an original K8055/P8055.
    WrongDevice,
    /// Five pulses: no usable raw HID interface could be allocated.
    HidAllocationError,
    /// Six pulses: the HID report descriptor could not be read.
    ReportDescriptorError,
    /// Seven pulses: the safe all-off output report failed.
    OutputError,
    /// Eight pulses: no valid eight-byte input report was received.
    InputError,
    /// Nine pulses: this example was started with a non-low-speed device.
    WrongSpeed,
}

impl LedPattern {
    const fn pulse_count(self) -> u8 {
        match self {
            Self::Success => 2,
            Self::EnumerationError => 3,
            Self::WrongDevice => 4,
            Self::HidAllocationError => 5,
            Self::ReportDescriptorError => 6,
            Self::OutputError => 7,
            Self::InputError => 8,
            Self::WrongSpeed => 9,
        }
    }

    fn level(self, phase: u8) -> Level {
        let pulse_phase = phase % 24;
        Level::from(pulse_phase < self.pulse_count() * 2 && pulse_phase.is_multiple_of(2))
    }
}

fn embassy_speed(speed: DeviceSpeed) -> Speed {
    match speed {
        DeviceSpeed::Full => Speed::Full,
        DeviceSpeed::Low => Speed::Low,
    }
}

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
                BusEvent::Attached(device_speed) => {
                    let speed = embassy_speed(device_speed);
                    info!("root device attached; resetting at {:?}", speed);
                    match host_state.reset_and_report_connected(speed).await {
                        Ok(()) => {
                            connected = true;
                            ticker = Ticker::every(Duration::from_millis(1));
                            info!("root reset complete");
                        }
                        Err(_) => {
                            connected = false;
                            warn!("root reset failed");
                        }
                    }
                }
                BusEvent::Detached => {
                    if connected && host_state.report_disconnected_if_not_resetting() {
                        connected = false;
                        info!("root device detached");
                    }
                }
                BusEvent::Invalid => {
                    warn!("stable SE1 detected");
                    if connected && host_state.report_disconnected_if_not_resetting() {
                        connected = false;
                    }
                }
            }
        }

        if connected {
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
                _ => {}
            },
            Either::Second(_) => phase = (phase + 1) % 24,
        }
    }
    status_led.set_low();
}

async fn run_application<'d, C>(
    controller: &mut BusController<'d, C>,
    bus_handle: &embassy_usb_host::BusHandle<'d, C::Allocator>,
    status_led: &mut Output<'_>,
) where
    C: UsbHostController<'d>,
{
    loop {
        status_led.set_low();
        let speed = controller.wait_for_connection().await;
        if speed != Speed::Low {
            warn!("P8055 example requires a low-speed root device");
            wait_for_root_removal(controller, status_led, LedPattern::WrongSpeed).await;
            continue;
        }

        status_led.set_high();
        info!("enumerating low-speed root device");
        let mut configuration = [0_u8; CONFIG_DESCRIPTOR_CAPACITY];
        let (enumeration, configuration_len) = match bus_handle
            .enumerate(BusRoute::Direct(speed), &mut configuration)
            .await
        {
            Ok(result) => result,
            Err(_) => {
                warn!("low-speed enumeration failed");
                wait_for_root_removal(controller, status_led, LedPattern::EnumerationError).await;
                for address in 1_u8..=127 {
                    bus_handle.free_address(address);
                }
                continue;
            }
        };

        let address = enumeration.device_address;
        let device = enumeration.device_desc;
        let pattern = if !is_original_k8055(device.vendor_id, device.product_id) {
            warn!("enumerated low-speed device is not an original K8055");
            LedPattern::WrongDevice
        } else {
            match allocate_from_enumeration(
                bus_handle,
                &configuration[..configuration_len],
                &enumeration,
            ) {
                Err(_) => {
                    warn!("raw HID allocation failed");
                    LedPattern::HidAllocationError
                }
                Ok(mut hid) => {
                    hid.reset_data_toggles();
                    let mut report_descriptor = [0_u8; REPORT_DESCRIPTOR_CAPACITY];
                    match with_timeout(
                        TRANSFER_TIMEOUT,
                        hid.get_report_descriptor(&mut report_descriptor),
                    )
                    .await
                    {
                        Err(_) | Ok(Err(_)) => {
                            warn!("HID report descriptor read failed");
                            LedPattern::ReportDescriptorError
                        }
                        Ok(Ok(descriptor)) => {
                            info!("HID report descriptor has {} bytes", descriptor.len());

                            // The original K8055 needs an initial command
                            // before its first input report. Command zero is
                            // the documented safe state: all outputs off.
                            let reset = OutputState::reset_report();
                            match with_timeout(TRANSFER_TIMEOUT, hid.write_output_report(&reset))
                                .await
                            {
                                Err(_) | Ok(Err(_)) => {
                                    warn!("safe K8055 reset report failed");
                                    LedPattern::OutputError
                                }
                                Ok(Ok(())) => {
                                    let mut raw_input = [0_u8; REPORT_LEN];
                                    match with_timeout(
                                        TRANSFER_TIMEOUT,
                                        hid.read_input_report(&mut raw_input),
                                    )
                                    .await
                                    {
                                        Ok(Ok(REPORT_LEN)) => {
                                            match InputReport::parse(&raw_input) {
                                                Ok(input) => {
                                                    info!(
                                                        "K8055 ready: status {}, digital {}, analog {} {}",
                                                        input.status(),
                                                        input.digital_inputs(),
                                                        input.analog_input_1(),
                                                        input.analog_input_2()
                                                    );
                                                    LedPattern::Success
                                                }
                                                Err(_) => LedPattern::InputError,
                                            }
                                        }
                                        _ => {
                                            warn!("K8055 interrupt-IN report failed");
                                            LedPattern::InputError
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        };

        wait_for_root_removal(controller, status_led, pattern).await;
        bus_handle.free_address(address);
        info!("root address {} released", address);
    }
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
    let controller = host_state.controller().expect("one root controller");
    let (mut controller, bus_handle) = embassy_usb_host::bus(controller, &bus_state);

    Timer::after_millis(100).await;
    vbus_enable.set_high();
    info!("USB VBUS enabled; waiting for a low-speed K8055/P8055");

    let application = async move {
        let _vbus_enable = vbus_enable;
        run_application(&mut controller, &bus_handle, &mut status_led).await;
    };
    join(root_port_monitor(&host_state), application).await;
}
