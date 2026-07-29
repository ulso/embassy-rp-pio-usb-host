//! Legacy, hardware-verified M5i application retained as a wire-level reference.

#[path = "protocol.rs"]
mod bleuio_protocol;

use defmt::{error, info, warn};
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::clocks::ClockConfig;
use embassy_rp::dma::{self, InterruptHandler as DmaInterruptHandler};
use embassy_rp::gpio::{Drive, Level, Output, Pull, SlewRate};
use embassy_rp::pac;
use embassy_rp::peripherals::{DMA_CH0, PIO0, PIO1};
use embassy_rp::pio::{
    Config as PioConfig, Direction, FifoJoin, InterruptHandler as PioInterruptHandler, IrqFlags,
    Pin as PioPin, Pio, ShiftDirection, StateMachine,
};
use embassy_rp::pio_programs::clock_divider::calculate_pio_clock_divider;
use embassy_time::{Duration, Instant, Ticker, Timer};
use fixed::traits::ToFixed;

use bleuio_protocol::{
    ATTENTION_COMMAND, AtResponseAccumulator as CdcAtResponseAccumulator,
    AtResponseStatus as CdcAtResponseStatus, CENTRAL_ROLE_COMMAND, GAP_SCAN_COMMAND,
    ScanAccumulator as BleuIoScanAccumulator, ScanResult as BleuIoScanResult,
    ScanStatus as BleuIoScanStatus,
};
use embassy_rp_pio_usb_host::usb::{
    CdcAcmDataState, CdcAcmFunction, CdcLineCoding, ConfigurationDescriptorHeader,
    DeviceDescriptor, DeviceDescriptorHeader, EndpointDescriptor, InDataDisposition,
    MAX_DECODED_BYTES, PID_ACK, PID_DATA0, PID_DATA1, PID_IN, PID_NAK, PID_OUT, PID_SETUP, PID_SOF,
    PID_STALL, ParsedPacket, RawDataPacket, SYNC, SetupRequest, classify_in_data, crc16_data,
    parse_packet, sof_packet, status_data1_packet, token_packet,
};
use embassy_rp_pio_usb_host::{AttachDetector, BusEvent, DeviceSpeed, LineState};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
    PIO1_IRQ_0 => PioInterruptHandler<PIO1>;
    DMA_IRQ_0 => DmaInterruptHandler<DMA_CH0>;
});

const SYS_CLOCK_HZ: u32 = 120_000_000;
const USB_TX_PIO_HZ: u32 = 48_000_000;
const USB_EDGE_PIO_HZ: u32 = 96_000_000;
const ATTACH_DEBOUNCE_SAMPLES: u16 = 100;
const USB_RESET_MS: u64 = 20;
const USB_RESET_RECOVERY_MS: u64 = 10;
const RX_IRQ_MASK: u8 = 0b0001_1110;
const TX_EOP_TIMEOUT_US: u64 = 100;
const TX_EOP_GUARD_CYCLES: u32 = SYS_CLOCK_HZ / 2_000_000;
// RX FIFO draining, CRC validation and packet classification already extend
// past the physical EOP. Do not add a fixed delay before releasing the ACK:
// the Beagle M5a capture showed missed host ACKs, and Pico-PIO-USB likewise
// proceeds directly from RX completion and CRC validation to its handshake.
// Keep the zero-cycle call in place as an explicit A/B timing point.
const RX_ACK_EOP_GUARD_CYCLES: u32 = 0;
const TX_IRQ_POLL_BUDGET: u32 = 100_000;
const RX_PACKET_POLL_BUDGET: u32 = 20_000;
const CONTROL_SETUP_RETRY_LIMIT: u8 = 3;
const CONTROL_STAGE_RETRY_LIMIT: u16 = 128;
const CONTROL_STATUS_NAK_RETRY_LIMIT: u8 = 4;
const CONTROL_TRANSFER_RETRY_LIMIT: u8 = 3;
const CDC_RESPONSE_POLL_LIMIT: u16 = 1_000;
const BLE_SCAN_POLL_LIMIT: u16 = 3_000;
const MAX_CONFIGURATION_DESCRIPTOR_BYTES: usize = 128;
const MAX_BULK_PACKET_BYTES: usize = 64;
const MAX_TOKEN_DATA_PAIR_BYTES: usize = 18;
const CRC16_USB_RESIDUE: u16 = 0xb001;

const fn make_crc16_table() -> [u16; 256] {
    let mut table = [0_u16; 256];
    let mut index = 0;
    while index < table.len() {
        let mut crc = index as u16;
        let mut bit = 0;
        while bit < 8 {
            crc = if crc & 1 != 0 {
                (crc >> 1) ^ 0xa001
            } else {
                crc >> 1
            };
            bit += 1;
        }
        table[index] = crc;
        index += 1;
    }
    table
}

// The IN transaction runs while XIP flash latency is unavailable for useful
// work. Keep its byte-at-a-time CRC lookup table in SRAM with the function.
#[unsafe(link_section = ".data.ram_crc16")]
static USB_CRC16_TABLE: [u16; 256] = make_crc16_table();

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum TxError {
    EopMissing,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum ReceiveResult {
    Handshake(u8),
    NoStart,
    InvalidPacket,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum InReceiveResult {
    Data { len: u8 },
    UnexpectedToggle,
    Nak,
    Stall,
    NoResponse,
    InvalidPacket,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
struct RxObservation {
    len: u8,
    irq_flags: u8,
    bytes: [u8; 4],
    program_counter: u8,
    edge_program_counter: u8,
    input_snapshot: u8,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum LoopbackResult {
    SawStart,
    ArmWait,
    EdgeWait,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum PioPulseResult {
    SawStart,
    PinsDidNotReachK,
    ReceiverMissed,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum PacketLoopbackResult {
    ValidSof,
    NoStart,
    InvalidPacket,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum ProbeResult {
    BleScanFound,
    TransportError(TxError),
    RxPulseArmWait,
    RxPulseEdgeWait,
    TxPioPulsePinsFailed,
    TxPioPulseReceiverMissed,
    TxPacketLoopbackFailed,
    TxPacketLoopbackInvalid,
    NoResponse,
    InvalidResponse,
    NonAck,
    ControlReadFailed,
    ConfigurationUnsupported,
    BulkTransferFailed,
    CdcResponseInvalid,
    BleScanInvalid,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum LedPattern {
    BleScanFound,
    Diagnostic(u16),
}

impl ProbeResult {
    const fn led_pattern(self) -> LedPattern {
        match self {
            Self::BleScanFound => LedPattern::BleScanFound,
            Self::TransportError(TxError::EopMissing) => LedPattern::Diagnostic(2),
            Self::ConfigurationUnsupported => LedPattern::Diagnostic(3),
            Self::RxPulseArmWait => LedPattern::Diagnostic(4),
            Self::RxPulseEdgeWait => LedPattern::Diagnostic(5),
            Self::TxPioPulsePinsFailed => LedPattern::Diagnostic(6),
            Self::TxPioPulseReceiverMissed => LedPattern::Diagnostic(7),
            Self::TxPacketLoopbackFailed => LedPattern::Diagnostic(8),
            Self::TxPacketLoopbackInvalid => LedPattern::Diagnostic(9),
            Self::NoResponse => LedPattern::Diagnostic(10),
            Self::InvalidResponse => LedPattern::Diagnostic(11),
            Self::NonAck => LedPattern::Diagnostic(12),
            Self::ControlReadFailed
            | Self::BulkTransferFailed
            | Self::CdcResponseInvalid
            | Self::BleScanInvalid => LedPattern::Diagnostic(13),
        }
    }
}

fn diagnostic_led_level(pattern: LedPattern, frame: u16) -> Level {
    let on = match pattern {
        LedPattern::BleScanFound => {
            // Six deliberately slow, separated pulses followed by a long
            // pause make M5i distinguishable from the faster M5h burst.
            let phase = frame % 2_500;
            phase < 100
                || (250..350).contains(&phase)
                || (500..600).contains(&phase)
                || (750..850).contains(&phase)
                || (1_000..1_100).contains(&phase)
                || (1_250..1_350).contains(&phase)
        }
        LedPattern::Diagnostic(pulses) => {
            // Five-second diagnostic cycle:
            //   500 ms long marker, 500 ms dark, then N countable pulses.
            let phase = frame % 5_000;
            if phase < 500 {
                true
            } else if phase < 1_000 {
                false
            } else {
                let count_phase = phase - 1_000;
                count_phase < pulses * 300 && count_phase % 300 < 150
            }
        }
    };
    Level::from(on)
}

fn snapshot_irq_flags() -> u8 {
    pac::PIO1.irq().read().irq()
}

fn record_rx_observation(
    observation: &mut RxObservation,
    sm: &StateMachine<'_, PIO1, 0>,
    edge_sm: &StateMachine<'_, PIO1, 1>,
    bytes: &[u8; 4],
    len: usize,
) {
    observation.len = len.min(u8::MAX as usize) as u8;
    observation.irq_flags = snapshot_irq_flags();
    observation.bytes = *bytes;
    observation.program_counter = sm.get_addr();
    observation.edge_program_counter = edge_sm.get_addr();
    let dp_peripheral = pac::IO_BANK0.gpio(16).status().read().intoperi() as u8;
    let dm_peripheral = pac::IO_BANK0.gpio(17).status().read().intoperi() as u8;
    let dp_pad = pac::IO_BANK0.gpio(16).status().read().infrompad() as u8;
    let dm_pad = pac::IO_BANK0.gpio(17).status().read().infrompad() as u8;
    observation.input_snapshot =
        dp_peripheral | (dm_peripheral << 1) | (dp_pad << 4) | (dm_pad << 5);
}

fn drain_rx_fifo(sm: &mut StateMachine<'_, PIO1, 0>, bytes: &mut [u8; 4], len: &mut usize) {
    while let Some(word) = sm.rx().try_pull() {
        if *len < bytes.len() {
            bytes[*len] = (word >> 24) as u8;
        }
        *len += 1;
    }
}

fn rx_diagnostic_packet(observation: RxObservation) -> [u8; 15] {
    let payload = [
        b'R',
        b'X',
        observation.len,
        observation.irq_flags,
        observation.bytes[0],
        observation.bytes[1],
        observation.bytes[2],
        observation.bytes[3],
        observation.program_counter,
        observation.edge_program_counter,
        observation.input_snapshot,
    ];
    let crc = crc16_data(&payload);
    [
        SYNC,
        PID_DATA0,
        payload[0],
        payload[1],
        payload[2],
        payload[3],
        payload[4],
        payload[5],
        payload[6],
        payload[7],
        payload[8],
        payload[9],
        payload[10],
        crc as u8,
        (crc >> 8) as u8,
    ]
}

fn read_line_state() -> LineState {
    let dp = pac::IO_BANK0.gpio(16).status().read().infrompad() as u32;
    let dm = pac::IO_BANK0.gpio(17).status().read().infrompad() as u32;
    LineState::from_pio_sample(dp | (dm << 1))
}

async fn reset_full_speed_bus<'d>(
    sm: &mut StateMachine<'d, PIO0, 0>,
    dp: &PioPin<'d, PIO0>,
    dm: &PioPin<'d, PIO0>,
) {
    sm.set_enable(false);
    sm.clear_fifos();
    sm.set_pins(Level::Low, &[dp, dm]);
    sm.set_pin_dirs(Direction::Out, &[dp, dm]);

    Timer::after_millis(USB_RESET_MS).await;

    sm.set_pins(Level::High, &[dp]);
    sm.set_pins(Level::Low, &[dm]);
    sm.set_pin_dirs(Direction::In, &[dp, dm]);
    // Preserve the TX program counter at IRQ WAIT 0 across reset, matching
    // Pico-PIO-USB. The first packet releases that wait by clearing IRQ 0.
    sm.set_enable(true);

    Timer::after_millis(USB_RESET_RECOVERY_MS).await;
}

#[allow(clippy::too_many_arguments)]
async fn transmit_full_speed(
    sm: &mut StateMachine<'_, PIO0, 0>,
    _tx_dma: &mut dma::Channel<'_>,
    tx_irq_flags: &IrqFlags<'_, PIO0>,
    rx_irq_flags: &IrqFlags<'_, PIO1>,
    packet: &[u8],
    arm_receiver_at_eop: bool,
) -> Result<(), TxError> {
    // The current Pico-PIO-USB transmitter consumes ordinary USB packet
    // bytes. PIO performs LSB-first serialization, NRZI and bit stuffing.
    assert!(!packet.is_empty());
    assert!(packet.len() <= MAX_TOKEN_DATA_PAIR_BYTES);

    // Preload enough bytes to give the CPU ample time to refill the FIFO, but
    // release the SM quickly enough to satisfy USB's token-to-data timing.
    // Replication makes the selected byte independent of the OSR lane.
    let preload_len = packet.len().min(4);
    for &byte in &packet[..preload_len] {
        sm.tx().push(u32::from(byte) * 0x0101_0101);
    }

    // Clear EOP and completion. Clearing completion releases the transmitter
    // from its IRQ WAIT 0 between packets.
    tx_irq_flags.clear_all(0b11);

    for &byte in &packet[preload_len..] {
        while sm.tx().full() {}
        sm.tx().push(u32::from(byte) * 0x0101_0101);
    }

    // USB full-speed devices may begin their response only two bit times
    // after EOP, and must begin within 16 bit times (about 1.33 us). An
    // async IRQ wake-up can therefore resume too late. Poll the PIO flag
    // here, as Pico-PIO-USB does, so RX is armed while TX is still in EOP.
    let eop_deadline = Instant::now() + Duration::from_micros(TX_EOP_TIMEOUT_US);
    while !tx_irq_flags.check(1) {
        if Instant::now() >= eop_deadline {
            return Err(TxError::EopMissing);
        }
    }
    tx_irq_flags.clear(1);

    if arm_receiver_at_eop {
        // PIO1 SM1 was parked at IRQ WAIT 2 before transmission. One register
        // write releases it at the start of EOP, matching Pico-PIO-USB's
        // latency-critical receive handoff.
        rx_irq_flags.clear_all(RX_IRQ_MASK);
    }

    // IRQ 1 is raised at the first EOP instruction. Its program path is
    // deterministic: two SE0 bits, one J bit, then pin release. A synchronous
    // 0.5 us guard is longer than that path while keeping the token-to-data
    // inter-packet delay inside the full-speed response window.
    cortex_m::asm::delay(TX_EOP_GUARD_CYCLES);

    Ok(())
}

#[inline(always)]
fn wait_for_tx_irq(tx_irq_flags: &IrqFlags<'_, PIO0>, irq: u8) -> Result<(), TxError> {
    let mut budget = TX_IRQ_POLL_BUDGET;
    while !tx_irq_flags.check(irq) {
        if budget == 0 {
            return Err(TxError::EopMissing);
        }
        budget -= 1;
    }
    Ok(())
}

#[inline(always)]
fn inline_delay_cycles(cycles: u32) {
    // Keep this delay inside RAM-resident callers. cortex_m::asm::delay()
    // otherwise calls an out-of-line shim in XIP flash on thumbv6m.
    let remaining = 1 + cycles / 2;
    unsafe {
        core::arch::asm!(
            "1:",
            "subs {remaining}, #1",
            "bne 1b",
            remaining = inout(reg) remaining => _,
            options(nomem, nostack),
        );
    }
}

#[inline(always)]
fn crc16_update_ram(crc: u16, byte: u8) -> u16 {
    let index = usize::from((crc as u8) ^ byte);
    (crc >> 8) ^ USB_CRC16_TABLE[index]
}

#[inline(always)]
fn drain_in_fifo(
    sm: &mut StateMachine<'_, PIO1, 0>,
    raw: &mut [u8; MAX_DECODED_BYTES],
    len: &mut usize,
    crc: &mut u16,
) {
    while let Some(word) = sm.rx().try_pull() {
        let byte = (word >> 24) as u8;
        if *len < raw.len() {
            raw[*len] = byte;
        }
        if *len >= 2 {
            *crc = crc16_update_ram(*crc, byte);
        }
        *len += 1;
    }
}

#[inline(never)]
#[unsafe(link_section = ".data.ram_func")]
fn transmit_token_data_pair(
    sm: &mut StateMachine<'_, PIO0, 0>,
    tx_irq_flags: &IrqFlags<'_, PIO0>,
    rx_irq_flags: &IrqFlags<'_, PIO1>,
    token: &[u8],
    data: &[u8],
) -> Result<(), TxError> {
    assert!(!token.is_empty() && token.len() <= 8);
    assert!(!data.is_empty() && data.len() <= MAX_TOKEN_DATA_PAIR_BYTES);

    for &byte in token {
        sm.tx().push(u32::from(byte) * 0x0101_0101);
    }
    tx_irq_flags.clear_all(0b11);

    wait_for_tx_irq(tx_irq_flags, 1)?;
    tx_irq_flags.clear(1);

    // The token has already branched into its EOP path, so bytes placed in
    // the FIFO now cannot be merged into it. Hide data-packet preload latency under
    // EOP and then release the next packet shortly after TX-complete.
    let preload_len = data.len().min(4);
    for &byte in &data[..preload_len] {
        sm.tx().push(u32::from(byte) * 0x0101_0101);
    }

    wait_for_tx_irq(tx_irq_flags, 0)?;
    tx_irq_flags.clear_all(0b11);

    for &byte in &data[preload_len..] {
        while sm.tx().full() {}
        sm.tx().push(u32::from(byte) * 0x0101_0101);
    }

    wait_for_tx_irq(tx_irq_flags, 1)?;
    tx_irq_flags.clear(1);

    // Release the parked edge detector at the beginning of the data packet's EOP so it
    // is waiting before the device's earliest legal handshake response.
    rx_irq_flags.clear_all(RX_IRQ_MASK);
    inline_delay_cycles(TX_EOP_GUARD_CYCLES);

    Ok(())
}

/// Send an IN token, receive one device response, and ACK valid DATA in time.
///
/// The ACK is placed in the TX FIFO while the token is still in EOP, but the
/// TX state machine remains parked on IRQ0 until the device packet has passed
/// its PID and streaming CRC checks. The complete critical path stays in SRAM.
#[allow(clippy::too_many_arguments)]
#[inline(never)]
#[unsafe(link_section = ".data.ram_func")]
fn receive_data_packet(
    tx_sm: &mut StateMachine<'_, PIO0, 0>,
    tx_irq_flags: &IrqFlags<'_, PIO0>,
    rx_sm: &mut StateMachine<'_, PIO1, 0>,
    rx_irq_flags: &IrqFlags<'_, PIO1>,
    token: &[u8; 4],
    payload: &mut [u8],
    raw: &mut [u8; MAX_DECODED_BYTES],
    max_payload_len: usize,
    expected_pid: u8,
) -> Result<InReceiveResult, TxError> {
    for &byte in token {
        tx_sm.tx().push(u32::from(byte) * 0x0101_0101);
    }
    tx_irq_flags.clear_all(0b11);

    wait_for_tx_irq(tx_irq_flags, 1)?;
    tx_irq_flags.clear(1);

    // Arm the parked edge detector at the beginning of the IN token's EOP.
    rx_irq_flags.clear_all(RX_IRQ_MASK);

    // The token can no longer consume FIFO data after entering EOP. Queue the
    // host ACK now, then leave IRQ0 asserted so it cannot start prematurely.
    tx_sm.tx().push(u32::from(SYNC) * 0x0101_0101);
    tx_sm.tx().push(u32::from(PID_ACK) * 0x0101_0101);
    if let Err(error) = wait_for_tx_irq(tx_irq_flags, 0) {
        tx_sm.clear_fifos();
        return Err(error);
    }

    let mut len = 0_usize;
    let mut crc = 0xffff_u16;
    let mut budget = RX_PACKET_POLL_BUDGET;
    let mut saw_start = false;
    let mut saw_eop = false;
    let mut rx_error = false;

    loop {
        drain_in_fifo(rx_sm, raw, &mut len, &mut crc);
        saw_start |= len != 0 || rx_irq_flags.check(3);

        if rx_irq_flags.check(1) {
            rx_error = true;
            break;
        }
        if rx_irq_flags.check(2) {
            saw_eop = true;
            break;
        }
        if budget == 0 {
            break;
        }
        budget -= 1;
    }

    if saw_eop {
        // IRQ2 is raised near the beginning of physical EOP. Wait through the
        // two SE0 bits and J recovery before allowing the host ACK to start.
        inline_delay_cycles(RX_ACK_EOP_GUARD_CYCLES);
        drain_in_fifo(rx_sm, raw, &mut len, &mut crc);
    }

    let pid = raw[1];
    let valid_pid = (pid >> 4) == ((!pid) & 0x0f);
    let actual_payload_len = if len >= 4 { len - 4 } else { usize::MAX };
    let wire_valid_data = !rx_error
        && saw_eop
        && raw[0] == SYNC
        && valid_pid
        && matches!(pid, PID_DATA0 | PID_DATA1)
        && crc == CRC16_USB_RESIDUE;
    let data_disposition = classify_in_data(
        wire_valid_data,
        expected_pid,
        max_payload_len,
        payload.len(),
        pid,
        actual_payload_len,
    );

    if data_disposition != InDataDisposition::Reject {
        // ACK an in-range expected packet and every CRC-valid duplicate that
        // fits the endpoint receive buffer. A duplicate can be longer than the
        // request's final remaining bytes, so its length limit differs.
        tx_irq_flags.clear(0);
        wait_for_tx_irq(tx_irq_flags, 1)?;
        tx_irq_flags.clear(1);
        wait_for_tx_irq(tx_irq_flags, 0)?;

        if data_disposition == InDataDisposition::Accept {
            let mut index = 0;
            while index < actual_payload_len {
                // Keep this explicit byte loop inside the SRAM function.
                // A normal slice copy is lowered to an XIP-flash memcpy shim.
                unsafe {
                    let byte = core::ptr::read_volatile(raw.as_ptr().add(index + 2));
                    core::ptr::write_volatile(payload.as_mut_ptr().add(index), byte);
                }
                index += 1;
            }
            return Ok(InReceiveResult::Data {
                len: actual_payload_len as u8,
            });
        }
        return Ok(InReceiveResult::UnexpectedToggle);
    }

    // A handshake or invalid response must not release the queued ACK on a
    // later transaction.
    tx_sm.clear_fifos();

    if !saw_start && len == 0 {
        return Ok(InReceiveResult::NoResponse);
    }
    if !rx_error && saw_eop && len == 2 && raw[0] == SYNC && valid_pid {
        return Ok(match pid {
            PID_NAK => InReceiveResult::Nak,
            PID_STALL => InReceiveResult::Stall,
            _ => InReceiveResult::InvalidPacket,
        });
    }
    Ok(InReceiveResult::InvalidPacket)
}

fn prepare_receive(
    sm: &mut StateMachine<'_, PIO1, 0>,
    reset_instruction: u16,
    clear_x_instruction: u16,
) {
    sm.set_enable(false);
    sm.clear_fifos();
    sm.restart();
    unsafe {
        sm.exec_instr(reset_instruction);
        sm.exec_instr(clear_x_instruction);
    }
    sm.set_enable(true);
}

fn start_receive(
    edge_sm: &mut StateMachine<'_, PIO1, 1>,
    irq_flags: &IrqFlags<'_, PIO1>,
    edge_start_instruction: u16,
) {
    edge_sm.set_enable(false);
    edge_sm.clear_fifos();
    irq_flags.clear_all(RX_IRQ_MASK);
    edge_sm.restart();
    unsafe { edge_sm.exec_instr(edge_start_instruction) };
    edge_sm.set_enable(true);
}

fn park_receive(
    edge_sm: &mut StateMachine<'_, PIO1, 1>,
    irq_flags: &IrqFlags<'_, PIO1>,
    edge_reset_instruction: u16,
) {
    edge_sm.set_enable(false);
    edge_sm.clear_fifos();
    irq_flags.clear_all(RX_IRQ_MASK);
    edge_sm.restart();
    unsafe { edge_sm.exec_instr(edge_reset_instruction) };
    edge_sm.set_enable(true);
}

fn receive_handshake(
    sm: &mut StateMachine<'_, PIO1, 0>,
    edge_sm: &mut StateMachine<'_, PIO1, 1>,
    irq_flags: &IrqFlags<'_, PIO1>,
    observation: &mut RxObservation,
) -> ReceiveResult {
    *observation = RxObservation::default();
    let mut bytes = [0_u8; 4];
    let mut len = 0;

    let start_deadline = Instant::now() + Duration::from_micros(5);
    while sm.rx().empty() && !irq_flags.check(2) {
        if irq_flags.check(1) {
            sm.set_enable(false);
            edge_sm.set_enable(false);
            drain_rx_fifo(sm, &mut bytes, &mut len);
            record_rx_observation(observation, sm, edge_sm, &bytes, len);
            return ReceiveResult::InvalidPacket;
        }
        if Instant::now() >= start_deadline {
            let edge_saw_start = irq_flags.check(3);
            sm.set_enable(false);
            edge_sm.set_enable(false);
            drain_rx_fifo(sm, &mut bytes, &mut len);
            record_rx_observation(observation, sm, edge_sm, &bytes, len);
            return if edge_saw_start {
                ReceiveResult::InvalidPacket
            } else {
                ReceiveResult::NoStart
            };
        }
    }

    let packet_deadline = Instant::now() + Duration::from_micros(20);

    loop {
        drain_rx_fifo(sm, &mut bytes, &mut len);

        if irq_flags.check(1) {
            sm.set_enable(false);
            edge_sm.set_enable(false);
            drain_rx_fifo(sm, &mut bytes, &mut len);
            record_rx_observation(observation, sm, edge_sm, &bytes, len);
            return ReceiveResult::InvalidPacket;
        }

        if irq_flags.check(2) {
            sm.set_enable(false);
            edge_sm.set_enable(false);
            drain_rx_fifo(sm, &mut bytes, &mut len);
            record_rx_observation(observation, sm, edge_sm, &bytes, len);
            break;
        }

        if Instant::now() >= packet_deadline {
            sm.set_enable(false);
            edge_sm.set_enable(false);
            drain_rx_fifo(sm, &mut bytes, &mut len);
            record_rx_observation(observation, sm, edge_sm, &bytes, len);
            return ReceiveResult::InvalidPacket;
        }
    }

    if len > bytes.len() {
        return ReceiveResult::InvalidPacket;
    }

    match parse_packet(&bytes[..len]) {
        Ok(ParsedPacket::Handshake { pid }) => ReceiveResult::Handshake(pid),
        _ => ReceiveResult::InvalidPacket,
    }
}

#[allow(clippy::too_many_arguments)]
fn verify_rx_k_pulse(
    rx_sm: &mut StateMachine<'_, PIO1, 0>,
    edge_sm: &mut StateMachine<'_, PIO1, 1>,
    irq_flags: &IrqFlags<'_, PIO1>,
    rx_reset_instruction: u16,
    rx_clear_x_instruction: u16,
    edge_start_instruction: u16,
) -> LoopbackResult {
    prepare_receive(rx_sm, rx_reset_instruction, rx_clear_x_instruction);
    start_receive(edge_sm, irq_flags, edge_start_instruction);

    // Force a raw full-speed K state at the pads through IO_BANK0 overrides.
    // This bypasses SM0, side-set, PIO output latches, and DMA entirely while
    // leaving the inverted peripheral input path visible to SM2.
    pac::IO_BANK0.gpio(16).ctrl().modify(|w| {
        w.set_outover(pac::io::vals::Outover::LOW);
        w.set_oeover(pac::io::vals::Oeover::ENABLE);
    });
    pac::IO_BANK0.gpio(17).ctrl().modify(|w| {
        w.set_outover(pac::io::vals::Outover::HIGH);
        w.set_oeover(pac::io::vals::Oeover::ENABLE);
    });
    cortex_m::asm::delay(SYS_CLOCK_HZ / 500_000);

    // Inspect while K is still actively driven. After release SM2 may already
    // have returned to address 15, making its final PC ambiguous.
    let result = if irq_flags.check(3) || edge_sm.rx().try_pull().is_some() || !rx_sm.rx().empty() {
        LoopbackResult::SawStart
    } else {
        match edge_sm.get_addr() {
            15 => LoopbackResult::ArmWait,
            16 => LoopbackResult::EdgeWait,
            _ => LoopbackResult::SawStart,
        }
    };

    // Disable the forced drivers before returning output control to PIO.
    pac::IO_BANK0
        .gpio(16)
        .ctrl()
        .modify(|w| w.set_oeover(pac::io::vals::Oeover::DISABLE));
    pac::IO_BANK0
        .gpio(17)
        .ctrl()
        .modify(|w| w.set_oeover(pac::io::vals::Oeover::DISABLE));
    pac::IO_BANK0.gpio(16).ctrl().modify(|w| {
        w.set_outover(pac::io::vals::Outover::NORMAL);
        w.set_oeover(pac::io::vals::Oeover::NORMAL);
    });
    pac::IO_BANK0.gpio(17).ctrl().modify(|w| {
        w.set_outover(pac::io::vals::Outover::NORMAL);
        w.set_oeover(pac::io::vals::Oeover::NORMAL);
    });
    cortex_m::asm::delay(SYS_CLOCK_HZ / 1_000_000);
    rx_sm.set_enable(false);

    // The synthetic pulse has no USB EOP. Leave SM2 stopped; start_receive()
    // performs a complete deterministic restart for the following test.
    edge_sm.set_enable(false);
    edge_sm.clear_fifos();
    irq_flags.clear_all(RX_IRQ_MASK);

    result
}

#[allow(clippy::too_many_arguments)]
fn verify_tx_pio_k_pulse(
    tx_sm: &mut StateMachine<'_, PIO0, 0>,
    rx_sm: &mut StateMachine<'_, PIO1, 0>,
    edge_sm: &mut StateMachine<'_, PIO1, 1>,
    irq_flags: &IrqFlags<'_, PIO1>,
    force_k_instruction: u16,
    release_instruction: u16,
    idle_instruction: u16,
    rx_reset_instruction: u16,
    rx_clear_x_instruction: u16,
    edge_start_instruction: u16,
) -> PioPulseResult {
    prepare_receive(rx_sm, rx_reset_instruction, rx_clear_x_instruction);
    start_receive(edge_sm, irq_flags, edge_start_instruction);

    // Exercise the same PIO side-set and SET PINDIRS mappings as the real TX
    // program, but hold K long enough for both the CPU and SM2 to observe it.
    tx_sm.set_enable(false);
    tx_sm.clear_fifos();
    tx_sm.restart();
    unsafe { tx_sm.exec_instr(force_k_instruction) };
    cortex_m::asm::delay(SYS_CLOCK_HZ / 500_000);

    let pins_reached_k = read_line_state() == LineState::JLowSpeed;
    let receiver_saw_start =
        irq_flags.check(3) || edge_sm.rx().try_pull().is_some() || !rx_sm.rx().empty();

    unsafe {
        tx_sm.exec_instr(release_instruction);
        tx_sm.exec_instr(idle_instruction);
    }
    tx_sm.set_enable(true);
    cortex_m::asm::delay(SYS_CLOCK_HZ / 500_000);
    rx_sm.set_enable(false);

    // The pulse deliberately has no EOP. Leave SM2 stopped; start_receive()
    // performs a complete deterministic restart for the following test.
    edge_sm.set_enable(false);
    edge_sm.clear_fifos();
    irq_flags.clear_all(RX_IRQ_MASK);

    if !pins_reached_k {
        PioPulseResult::PinsDidNotReachK
    } else if receiver_saw_start {
        PioPulseResult::SawStart
    } else {
        PioPulseResult::ReceiverMissed
    }
}

#[allow(clippy::too_many_arguments)]
async fn verify_rx_packet_loopback(
    tx_sm: &mut StateMachine<'_, PIO0, 0>,
    tx_dma: &mut dma::Channel<'_>,
    tx_irq_flags: &IrqFlags<'_, PIO0>,
    rx_sm: &mut StateMachine<'_, PIO1, 0>,
    edge_sm: &mut StateMachine<'_, PIO1, 1>,
    rx_irq_flags: &IrqFlags<'_, PIO1>,
    edge_start_instruction: u16,
    rx_reset_instruction: u16,
    rx_clear_x_instruction: u16,
) -> Result<PacketLoopbackResult, TxError> {
    let packet = sof_packet(0);

    prepare_receive(rx_sm, rx_reset_instruction, rx_clear_x_instruction);
    start_receive(edge_sm, rx_irq_flags, edge_start_instruction);
    transmit_full_speed(tx_sm, tx_dma, tx_irq_flags, rx_irq_flags, &packet, false).await?;

    let started = rx_irq_flags.check(3)
        || edge_sm.rx().try_pull().is_some()
        || !rx_sm.rx().empty()
        || rx_irq_flags.check(2);
    if !started {
        rx_sm.set_enable(false);
        return Ok(PacketLoopbackResult::NoStart);
    }

    let eop_deadline = Instant::now() + Duration::from_micros(20);
    while !rx_irq_flags.check(2) {
        if Instant::now() >= eop_deadline {
            rx_sm.set_enable(false);
            return Ok(PacketLoopbackResult::InvalidPacket);
        }
    }

    let mut bytes = [0_u8; 4];
    let mut len = 0;
    while let Some(word) = rx_sm.rx().try_pull() {
        if len < bytes.len() {
            bytes[len] = (word >> 24) as u8;
        }
        len += 1;
    }
    rx_sm.set_enable(false);

    let valid = len == bytes.len()
        && matches!(
            parse_packet(&bytes),
            Ok(ParsedPacket::Token {
                pid: PID_SOF,
                value: 0
            })
        );
    Ok(if valid {
        PacketLoopbackResult::ValidSof
    } else {
        PacketLoopbackResult::InvalidPacket
    })
}

fn setup_data_packet(request: SetupRequest) -> [u8; 12] {
    let bytes = request.to_bytes();
    let crc = crc16_data(&bytes);
    [
        SYNC,
        PID_DATA0,
        bytes[0],
        bytes[1],
        bytes[2],
        bytes[3],
        bytes[4],
        bytes[5],
        bytes[6],
        bytes[7],
        crc as u8,
        (crc >> 8) as u8,
    ]
}

#[allow(clippy::too_many_arguments)]
async fn advance_probe_frame(
    tx_sm: &mut StateMachine<'_, PIO0, 0>,
    tx_dma: &mut dma::Channel<'_>,
    tx_irq_flags: &IrqFlags<'_, PIO0>,
    rx_irq_flags: &IrqFlags<'_, PIO1>,
    frame_number: &mut u16,
) -> Result<(), TxError> {
    Timer::after_millis(1).await;
    let sof = sof_packet(*frame_number);
    *frame_number = (*frame_number + 1) & 0x07ff;
    transmit_full_speed(tx_sm, tx_dma, tx_irq_flags, rx_irq_flags, &sof, false).await
}

#[allow(clippy::too_many_arguments)]
async fn receive_control_in_packet(
    address: u8,
    expected_pid: u8,
    max_payload_len: usize,
    payload: &mut [u8; 8],
    probe_frame_number: &mut u16,
    tx_sm: &mut StateMachine<'_, PIO0, 0>,
    tx_dma: &mut dma::Channel<'_>,
    tx_irq_flags: &IrqFlags<'_, PIO0>,
    rx_sm: &mut StateMachine<'_, PIO1, 0>,
    edge_sm: &mut StateMachine<'_, PIO1, 1>,
    rx_irq_flags: &IrqFlags<'_, PIO1>,
    edge_reset_instruction: u16,
    rx_reset_instruction: u16,
    rx_clear_x_instruction: u16,
) -> Result<usize, ProbeResult> {
    let in_token = token_packet(PID_IN, address, 0);
    // Keep the larger receive scratch outside the SRAM timing function so its
    // initialization cannot introduce a flash-resident memset on the ACK path.
    let mut raw = [0_u8; MAX_DECODED_BYTES];
    let mut attempt = 0_u16;

    loop {
        prepare_receive(rx_sm, rx_reset_instruction, rx_clear_x_instruction);
        park_receive(edge_sm, rx_irq_flags, edge_reset_instruction);
        let result = receive_data_packet(
            tx_sm,
            tx_irq_flags,
            rx_sm,
            rx_irq_flags,
            &in_token,
            payload,
            &mut raw,
            max_payload_len,
            expected_pid,
        );
        rx_sm.set_enable(false);
        edge_sm.set_enable(false);

        match result {
            Ok(InReceiveResult::Data { len }) => return Ok(len as usize),
            Ok(InReceiveResult::Stall) => return Err(ProbeResult::ControlReadFailed),
            Ok(
                InReceiveResult::UnexpectedToggle
                | InReceiveResult::Nak
                | InReceiveResult::NoResponse
                | InReceiveResult::InvalidPacket,
            ) => {}
            Err(error) => return Err(ProbeResult::TransportError(error)),
        }

        attempt += 1;
        if attempt >= CONTROL_STAGE_RETRY_LIMIT {
            return Err(ProbeResult::ControlReadFailed);
        }

        advance_probe_frame(
            tx_sm,
            tx_dma,
            tx_irq_flags,
            rx_irq_flags,
            probe_frame_number,
        )
        .await
        .map_err(ProbeResult::TransportError)?;
    }
}

#[allow(clippy::too_many_arguments)]
async fn read_device_descriptor_prefix(
    address: u8,
    probe_frame_number: &mut u16,
    tx_sm: &mut StateMachine<'_, PIO0, 0>,
    tx_dma: &mut dma::Channel<'_>,
    tx_irq_flags: &IrqFlags<'_, PIO0>,
    rx_sm: &mut StateMachine<'_, PIO1, 0>,
    edge_sm: &mut StateMachine<'_, PIO1, 1>,
    rx_irq_flags: &IrqFlags<'_, PIO1>,
    edge_reset_instruction: u16,
    rx_reset_instruction: u16,
    rx_clear_x_instruction: u16,
    observation: &mut RxObservation,
) -> Result<DeviceDescriptorHeader, ProbeResult> {
    // Complete a descriptor-prefix control read:
    // SETUP/DATA0/ACK, IN/DATA1/ACK, OUT/DATA1-ZLP/ACK.
    let mut transfer_attempt = 0_u8;
    'transfer: loop {
        let data_packet = setup_data_packet(SetupRequest::get_device_descriptor_prefix());
        let setup_token = token_packet(PID_SETUP, address, 0);
        let mut setup_attempt = 0_u8;
        loop {
            prepare_receive(rx_sm, rx_reset_instruction, rx_clear_x_instruction);
            park_receive(edge_sm, rx_irq_flags, edge_reset_instruction);
            if let Err(error) = transmit_token_data_pair(
                tx_sm,
                tx_irq_flags,
                rx_irq_flags,
                &setup_token,
                &data_packet,
            ) {
                *observation = RxObservation::default();
                rx_sm.set_enable(false);
                edge_sm.set_enable(false);
                return Err(ProbeResult::TransportError(error));
            }

            let result = match receive_handshake(rx_sm, edge_sm, rx_irq_flags, observation) {
                ReceiveResult::Handshake(PID_ACK) => break,
                ReceiveResult::Handshake(PID_STALL) => {
                    return Err(ProbeResult::ControlReadFailed);
                }
                ReceiveResult::Handshake(PID_NAK) => ProbeResult::NonAck,
                ReceiveResult::Handshake(_) => ProbeResult::InvalidResponse,
                ReceiveResult::NoStart => ProbeResult::NoResponse,
                ReceiveResult::InvalidPacket => ProbeResult::InvalidResponse,
            };

            setup_attempt += 1;
            if setup_attempt >= CONTROL_SETUP_RETRY_LIMIT {
                return Err(result);
            }
            advance_probe_frame(
                tx_sm,
                tx_dma,
                tx_irq_flags,
                rx_irq_flags,
                probe_frame_number,
            )
            .await
            .map_err(ProbeResult::TransportError)?;
        }

        let mut descriptor_prefix = [0_u8; 8];
        let descriptor_prefix_len = receive_control_in_packet(
            address,
            PID_DATA1,
            descriptor_prefix.len(),
            &mut descriptor_prefix,
            probe_frame_number,
            tx_sm,
            tx_dma,
            tx_irq_flags,
            rx_sm,
            edge_sm,
            rx_irq_flags,
            edge_reset_instruction,
            rx_reset_instruction,
            rx_clear_x_instruction,
        )
        .await?;

        // Finish the control read even if the descriptor contents later prove
        // invalid; the device has already committed the DATA1 transaction.
        let status_token = token_packet(PID_OUT, address, 0);
        let status_data = status_data1_packet();
        let mut status_attempt = 0_u16;
        let mut status_nak_count = 0_u8;
        loop {
            prepare_receive(rx_sm, rx_reset_instruction, rx_clear_x_instruction);
            park_receive(edge_sm, rx_irq_flags, edge_reset_instruction);
            if let Err(error) = transmit_token_data_pair(
                tx_sm,
                tx_irq_flags,
                rx_irq_flags,
                &status_token,
                &status_data,
            ) {
                rx_sm.set_enable(false);
                edge_sm.set_enable(false);
                return Err(ProbeResult::TransportError(error));
            }

            let status_was_nak = match receive_handshake(rx_sm, edge_sm, rx_irq_flags, observation)
            {
                ReceiveResult::Handshake(PID_ACK) => break,
                ReceiveResult::Handshake(PID_NAK) => true,
                ReceiveResult::Handshake(PID_STALL) => {
                    return Err(ProbeResult::ControlReadFailed);
                }
                ReceiveResult::Handshake(_) => {
                    return Err(ProbeResult::InvalidResponse);
                }
                ReceiveResult::NoStart | ReceiveResult::InvalidPacket => {
                    status_nak_count = 0;
                    false
                }
            };

            if status_was_nak {
                status_nak_count += 1;
                if status_nak_count >= CONTROL_STATUS_NAK_RETRY_LIMIT {
                    transfer_attempt += 1;
                    if transfer_attempt >= CONTROL_TRANSFER_RETRY_LIMIT {
                        return Err(ProbeResult::ControlReadFailed);
                    }
                    advance_probe_frame(
                        tx_sm,
                        tx_dma,
                        tx_irq_flags,
                        rx_irq_flags,
                        probe_frame_number,
                    )
                    .await
                    .map_err(ProbeResult::TransportError)?;
                    continue 'transfer;
                }
            }

            status_attempt += 1;
            if status_attempt >= CONTROL_STAGE_RETRY_LIMIT {
                return Err(ProbeResult::ControlReadFailed);
            }

            advance_probe_frame(
                tx_sm,
                tx_dma,
                tx_irq_flags,
                rx_irq_flags,
                probe_frame_number,
            )
            .await
            .map_err(ProbeResult::TransportError)?;
        }

        return DeviceDescriptorHeader::parse(&descriptor_prefix[..descriptor_prefix_len])
            .map_err(|_| ProbeResult::ControlReadFailed);
    }
}

#[allow(clippy::too_many_arguments)]
async fn control_read_ep0(
    address: u8,
    max_packet_size0: u8,
    request: SetupRequest,
    destination: &mut [u8],
    probe_frame_number: &mut u16,
    tx_sm: &mut StateMachine<'_, PIO0, 0>,
    tx_dma: &mut dma::Channel<'_>,
    tx_irq_flags: &IrqFlags<'_, PIO0>,
    rx_sm: &mut StateMachine<'_, PIO1, 0>,
    edge_sm: &mut StateMachine<'_, PIO1, 1>,
    rx_irq_flags: &IrqFlags<'_, PIO1>,
    edge_reset_instruction: u16,
    rx_reset_instruction: u16,
    rx_clear_x_instruction: u16,
    observation: &mut RxObservation,
) -> Result<usize, ProbeResult> {
    // The current SRAM receive path is intentionally sized for the target
    // dongle's eight-byte endpoint zero packets.
    let requested_length = request.length as usize;
    if max_packet_size0 != 8
        || request.request_type & 0x80 == 0
        || requested_length == 0
        || requested_length > destination.len()
    {
        return Err(ProbeResult::ControlReadFailed);
    }

    let mut transfer_attempt = 0_u8;
    'transfer: loop {
        let data_packet = setup_data_packet(request);
        let setup_token = token_packet(PID_SETUP, address, 0);
        let mut setup_attempt = 0_u8;
        loop {
            prepare_receive(rx_sm, rx_reset_instruction, rx_clear_x_instruction);
            park_receive(edge_sm, rx_irq_flags, edge_reset_instruction);
            if let Err(error) = transmit_token_data_pair(
                tx_sm,
                tx_irq_flags,
                rx_irq_flags,
                &setup_token,
                &data_packet,
            ) {
                *observation = RxObservation::default();
                rx_sm.set_enable(false);
                edge_sm.set_enable(false);
                return Err(ProbeResult::TransportError(error));
            }

            let result = match receive_handshake(rx_sm, edge_sm, rx_irq_flags, observation) {
                ReceiveResult::Handshake(PID_ACK) => break,
                ReceiveResult::Handshake(PID_STALL) => {
                    return Err(ProbeResult::ControlReadFailed);
                }
                ReceiveResult::Handshake(PID_NAK) => ProbeResult::NonAck,
                ReceiveResult::Handshake(_) => ProbeResult::InvalidResponse,
                ReceiveResult::NoStart => ProbeResult::NoResponse,
                ReceiveResult::InvalidPacket => ProbeResult::InvalidResponse,
            };

            setup_attempt += 1;
            if setup_attempt >= CONTROL_SETUP_RETRY_LIMIT {
                return Err(result);
            }
            advance_probe_frame(
                tx_sm,
                tx_dma,
                tx_irq_flags,
                rx_irq_flags,
                probe_frame_number,
            )
            .await
            .map_err(ProbeResult::TransportError)?;
        }

        destination[..requested_length].fill(0);
        let mut packet_payload = [0_u8; 8];
        let mut received_length = 0_usize;
        let mut expected_pid = PID_DATA1;

        while received_length < requested_length {
            let remaining = requested_length - received_length;
            let max_payload_len = remaining.min(max_packet_size0 as usize);
            let received_len = receive_control_in_packet(
                address,
                expected_pid,
                max_payload_len,
                &mut packet_payload,
                probe_frame_number,
                tx_sm,
                tx_dma,
                tx_irq_flags,
                rx_sm,
                edge_sm,
                rx_irq_flags,
                edge_reset_instruction,
                rx_reset_instruction,
                rx_clear_x_instruction,
            )
            .await?;

            destination[received_length..received_length + received_len]
                .copy_from_slice(&packet_payload[..received_len]);
            received_length += received_len;

            if received_length == requested_length || received_len < max_packet_size0 as usize {
                break;
            }

            expected_pid = if expected_pid == PID_DATA1 {
                PID_DATA0
            } else {
                PID_DATA1
            };

            // Keep the deliberately simple probe to one data-stage IN
            // transaction per frame, matching the verified reference capture.
            advance_probe_frame(
                tx_sm,
                tx_dma,
                tx_irq_flags,
                rx_irq_flags,
                probe_frame_number,
            )
            .await
            .map_err(ProbeResult::TransportError)?;
        }

        // Complete the status stage before validating descriptor contents. A
        // standards-compliant short packet ends the data stage even when it
        // makes the final descriptor too short to parse.
        let status_token = token_packet(PID_OUT, address, 0);
        let status_data = status_data1_packet();
        let mut status_attempt = 0_u16;
        let mut status_nak_count = 0_u8;
        loop {
            prepare_receive(rx_sm, rx_reset_instruction, rx_clear_x_instruction);
            park_receive(edge_sm, rx_irq_flags, edge_reset_instruction);
            if let Err(error) = transmit_token_data_pair(
                tx_sm,
                tx_irq_flags,
                rx_irq_flags,
                &status_token,
                &status_data,
            ) {
                rx_sm.set_enable(false);
                edge_sm.set_enable(false);
                return Err(ProbeResult::TransportError(error));
            }

            let status_was_nak = match receive_handshake(rx_sm, edge_sm, rx_irq_flags, observation)
            {
                ReceiveResult::Handshake(PID_ACK) => break,
                ReceiveResult::Handshake(PID_NAK) => true,
                ReceiveResult::Handshake(PID_STALL) => {
                    return Err(ProbeResult::ControlReadFailed);
                }
                ReceiveResult::Handshake(_) => {
                    return Err(ProbeResult::InvalidResponse);
                }
                ReceiveResult::NoStart | ReceiveResult::InvalidPacket => {
                    status_nak_count = 0;
                    false
                }
            };

            if status_was_nak {
                status_nak_count += 1;
                if status_nak_count >= CONTROL_STATUS_NAK_RETRY_LIMIT {
                    transfer_attempt += 1;
                    if transfer_attempt >= CONTROL_TRANSFER_RETRY_LIMIT {
                        return Err(ProbeResult::ControlReadFailed);
                    }
                    advance_probe_frame(
                        tx_sm,
                        tx_dma,
                        tx_irq_flags,
                        rx_irq_flags,
                        probe_frame_number,
                    )
                    .await
                    .map_err(ProbeResult::TransportError)?;
                    continue 'transfer;
                }
            }

            status_attempt += 1;
            if status_attempt >= CONTROL_STAGE_RETRY_LIMIT {
                return Err(ProbeResult::ControlReadFailed);
            }
            advance_probe_frame(
                tx_sm,
                tx_dma,
                tx_irq_flags,
                rx_irq_flags,
                probe_frame_number,
            )
            .await
            .map_err(ProbeResult::TransportError)?;
        }

        return Ok(received_length);
    }
}

#[allow(clippy::too_many_arguments)]
async fn read_complete_device_descriptor(
    address: u8,
    max_packet_size0: u8,
    probe_frame_number: &mut u16,
    tx_sm: &mut StateMachine<'_, PIO0, 0>,
    tx_dma: &mut dma::Channel<'_>,
    tx_irq_flags: &IrqFlags<'_, PIO0>,
    rx_sm: &mut StateMachine<'_, PIO1, 0>,
    edge_sm: &mut StateMachine<'_, PIO1, 1>,
    rx_irq_flags: &IrqFlags<'_, PIO1>,
    edge_reset_instruction: u16,
    rx_reset_instruction: u16,
    rx_clear_x_instruction: u16,
    observation: &mut RxObservation,
) -> Result<DeviceDescriptor, ProbeResult> {
    let mut descriptor_bytes = [0_u8; 18];
    let descriptor_len = control_read_ep0(
        address,
        max_packet_size0,
        SetupRequest::get_device_descriptor(descriptor_bytes.len() as u16),
        &mut descriptor_bytes,
        probe_frame_number,
        tx_sm,
        tx_dma,
        tx_irq_flags,
        rx_sm,
        edge_sm,
        rx_irq_flags,
        edge_reset_instruction,
        rx_reset_instruction,
        rx_clear_x_instruction,
        observation,
    )
    .await?;

    DeviceDescriptor::parse(&descriptor_bytes[..descriptor_len])
        .map_err(|_| ProbeResult::ControlReadFailed)
}

#[allow(clippy::too_many_arguments)]
async fn discover_cdc_acm_configuration(
    address: u8,
    max_packet_size0: u8,
    probe_frame_number: &mut u16,
    tx_sm: &mut StateMachine<'_, PIO0, 0>,
    tx_dma: &mut dma::Channel<'_>,
    tx_irq_flags: &IrqFlags<'_, PIO0>,
    rx_sm: &mut StateMachine<'_, PIO1, 0>,
    edge_sm: &mut StateMachine<'_, PIO1, 1>,
    rx_irq_flags: &IrqFlags<'_, PIO1>,
    edge_reset_instruction: u16,
    rx_reset_instruction: u16,
    rx_clear_x_instruction: u16,
    observation: &mut RxObservation,
) -> Result<CdcAcmFunction, ProbeResult> {
    let mut header_bytes = [0_u8; 9];
    let header_len = control_read_ep0(
        address,
        max_packet_size0,
        SetupRequest::get_configuration_descriptor(0, header_bytes.len() as u16),
        &mut header_bytes,
        probe_frame_number,
        tx_sm,
        tx_dma,
        tx_irq_flags,
        rx_sm,
        edge_sm,
        rx_irq_flags,
        edge_reset_instruction,
        rx_reset_instruction,
        rx_clear_x_instruction,
        observation,
    )
    .await?;
    if header_len != header_bytes.len() {
        return Err(ProbeResult::ConfigurationUnsupported);
    }

    let header = ConfigurationDescriptorHeader::parse(&header_bytes)
        .map_err(|_| ProbeResult::ConfigurationUnsupported)?;
    let total_length = header.total_length as usize;
    if total_length > MAX_CONFIGURATION_DESCRIPTOR_BYTES {
        return Err(ProbeResult::ConfigurationUnsupported);
    }

    advance_probe_frame(
        tx_sm,
        tx_dma,
        tx_irq_flags,
        rx_irq_flags,
        probe_frame_number,
    )
    .await
    .map_err(ProbeResult::TransportError)?;

    let mut configuration_bytes = [0_u8; MAX_CONFIGURATION_DESCRIPTOR_BYTES];
    let configuration_len = control_read_ep0(
        address,
        max_packet_size0,
        SetupRequest::get_configuration_descriptor(0, header.total_length),
        &mut configuration_bytes,
        probe_frame_number,
        tx_sm,
        tx_dma,
        tx_irq_flags,
        rx_sm,
        edge_sm,
        rx_irq_flags,
        edge_reset_instruction,
        rx_reset_instruction,
        rx_clear_x_instruction,
        observation,
    )
    .await?;
    if configuration_len != total_length {
        return Err(ProbeResult::ConfigurationUnsupported);
    }

    let complete_header = ConfigurationDescriptorHeader::parse(&configuration_bytes)
        .map_err(|_| ProbeResult::ConfigurationUnsupported)?;
    if complete_header != header {
        return Err(ProbeResult::ConfigurationUnsupported);
    }

    CdcAcmFunction::discover(&configuration_bytes[..configuration_len])
        .map_err(|_| ProbeResult::ConfigurationUnsupported)
}

#[allow(clippy::too_many_arguments)]
async fn control_write_ep0(
    address: u8,
    request: SetupRequest,
    payload: &[u8],
    probe_frame_number: &mut u16,
    tx_sm: &mut StateMachine<'_, PIO0, 0>,
    tx_dma: &mut dma::Channel<'_>,
    tx_irq_flags: &IrqFlags<'_, PIO0>,
    rx_sm: &mut StateMachine<'_, PIO1, 0>,
    edge_sm: &mut StateMachine<'_, PIO1, 1>,
    rx_irq_flags: &IrqFlags<'_, PIO1>,
    edge_reset_instruction: u16,
    rx_reset_instruction: u16,
    rx_clear_x_instruction: u16,
    observation: &mut RxObservation,
) -> Result<(), ProbeResult> {
    if request.request_type & 0x80 != 0
        || request.length as usize != payload.len()
        || payload.len() > 8
    {
        return Err(ProbeResult::ControlReadFailed);
    }

    let setup_data = setup_data_packet(request);
    let setup_token = token_packet(PID_SETUP, address, 0);
    let mut setup_attempt = 0_u8;

    loop {
        prepare_receive(rx_sm, rx_reset_instruction, rx_clear_x_instruction);
        park_receive(edge_sm, rx_irq_flags, edge_reset_instruction);
        transmit_token_data_pair(tx_sm, tx_irq_flags, rx_irq_flags, &setup_token, &setup_data)
            .map_err(ProbeResult::TransportError)?;

        let result = match receive_handshake(rx_sm, edge_sm, rx_irq_flags, observation) {
            ReceiveResult::Handshake(PID_ACK) => break,
            ReceiveResult::Handshake(PID_STALL) => {
                return Err(ProbeResult::ControlReadFailed);
            }
            ReceiveResult::Handshake(PID_NAK) => ProbeResult::NonAck,
            ReceiveResult::Handshake(_) => ProbeResult::InvalidResponse,
            ReceiveResult::NoStart => ProbeResult::NoResponse,
            ReceiveResult::InvalidPacket => ProbeResult::InvalidResponse,
        };

        setup_attempt += 1;
        if setup_attempt >= CONTROL_SETUP_RETRY_LIMIT {
            return Err(result);
        }
        advance_probe_frame(
            tx_sm,
            tx_dma,
            tx_irq_flags,
            rx_irq_flags,
            probe_frame_number,
        )
        .await
        .map_err(ProbeResult::TransportError)?;
    }

    if !payload.is_empty() {
        // A control-write data stage starts at DATA1. Retrying the same PID
        // after NAK or a lost ACK lets the device recognize a duplicate
        // without applying the request twice.
        let data_packet =
            RawDataPacket::new(PID_DATA1, payload).map_err(|_| ProbeResult::ControlReadFailed)?;
        let out_token = token_packet(PID_OUT, address, 0);
        let mut data_attempt = 0_u16;

        loop {
            prepare_receive(rx_sm, rx_reset_instruction, rx_clear_x_instruction);
            park_receive(edge_sm, rx_irq_flags, edge_reset_instruction);
            transmit_token_data_pair(
                tx_sm,
                tx_irq_flags,
                rx_irq_flags,
                &out_token,
                data_packet.as_bytes(),
            )
            .map_err(ProbeResult::TransportError)?;

            let result = match receive_handshake(rx_sm, edge_sm, rx_irq_flags, observation) {
                ReceiveResult::Handshake(PID_ACK) => break,
                ReceiveResult::Handshake(PID_STALL) => {
                    return Err(ProbeResult::ControlReadFailed);
                }
                ReceiveResult::Handshake(PID_NAK) => ProbeResult::NonAck,
                ReceiveResult::Handshake(_) => ProbeResult::InvalidResponse,
                ReceiveResult::NoStart => ProbeResult::NoResponse,
                ReceiveResult::InvalidPacket => ProbeResult::InvalidResponse,
            };

            data_attempt += 1;
            if data_attempt >= CONTROL_STAGE_RETRY_LIMIT {
                return Err(result);
            }
            advance_probe_frame(
                tx_sm,
                tx_dma,
                tx_irq_flags,
                rx_irq_flags,
                probe_frame_number,
            )
            .await
            .map_err(ProbeResult::TransportError)?;
        }
    }

    // A control-write status stage is an IN transaction carrying a DATA1
    // ZLP. The requested state change completes with the host ACK.
    let mut unused_payload = [0_u8; 8];
    let status_len = receive_control_in_packet(
        address,
        PID_DATA1,
        0,
        &mut unused_payload,
        probe_frame_number,
        tx_sm,
        tx_dma,
        tx_irq_flags,
        rx_sm,
        edge_sm,
        rx_irq_flags,
        edge_reset_instruction,
        rx_reset_instruction,
        rx_clear_x_instruction,
    )
    .await?;

    if status_len == 0 {
        Ok(())
    } else {
        Err(ProbeResult::ControlReadFailed)
    }
}

#[allow(clippy::too_many_arguments)]
async fn bulk_out_packet(
    address: u8,
    endpoint: EndpointDescriptor,
    payload: &[u8],
    data_state: &mut CdcAcmDataState,
    probe_frame_number: &mut u16,
    tx_sm: &mut StateMachine<'_, PIO0, 0>,
    tx_dma: &mut dma::Channel<'_>,
    tx_irq_flags: &IrqFlags<'_, PIO0>,
    rx_sm: &mut StateMachine<'_, PIO1, 0>,
    edge_sm: &mut StateMachine<'_, PIO1, 1>,
    rx_irq_flags: &IrqFlags<'_, PIO1>,
    edge_reset_instruction: u16,
    rx_reset_instruction: u16,
    rx_clear_x_instruction: u16,
    observation: &mut RxObservation,
) -> Result<(), ProbeResult> {
    if endpoint.is_in()
        || endpoint.transfer_type() != 0x02
        || endpoint.number() == 0
        || payload.len() > endpoint.max_packet_size as usize
        || payload.len() + 4 > MAX_TOKEN_DATA_PAIR_BYTES
    {
        return Err(ProbeResult::BulkTransferFailed);
    }

    let data_packet = RawDataPacket::new(data_state.bulk_out_pid(), payload)
        .map_err(|_| ProbeResult::BulkTransferFailed)?;
    let out_token = token_packet(PID_OUT, address, endpoint.number());
    let mut attempt = 0_u16;

    loop {
        prepare_receive(rx_sm, rx_reset_instruction, rx_clear_x_instruction);
        park_receive(edge_sm, rx_irq_flags, edge_reset_instruction);
        transmit_token_data_pair(
            tx_sm,
            tx_irq_flags,
            rx_irq_flags,
            &out_token,
            data_packet.as_bytes(),
        )
        .map_err(ProbeResult::TransportError)?;

        let result = match receive_handshake(rx_sm, edge_sm, rx_irq_flags, observation) {
            ReceiveResult::Handshake(PID_ACK) => {
                *data_state = data_state.after_bulk_out_ack();
                return Ok(());
            }
            ReceiveResult::Handshake(PID_STALL) => {
                return Err(ProbeResult::BulkTransferFailed);
            }
            ReceiveResult::Handshake(PID_NAK) => ProbeResult::NonAck,
            ReceiveResult::Handshake(_) => ProbeResult::InvalidResponse,
            ReceiveResult::NoStart => ProbeResult::NoResponse,
            ReceiveResult::InvalidPacket => ProbeResult::InvalidResponse,
        };

        attempt += 1;
        if attempt >= CONTROL_STAGE_RETRY_LIMIT {
            return Err(result);
        }
        advance_probe_frame(
            tx_sm,
            tx_dma,
            tx_irq_flags,
            rx_irq_flags,
            probe_frame_number,
        )
        .await
        .map_err(ProbeResult::TransportError)?;
    }
}

#[allow(clippy::too_many_arguments)]
async fn bulk_in_ok_response(
    address: u8,
    endpoint: EndpointDescriptor,
    data_state: &mut CdcAcmDataState,
    probe_frame_number: &mut u16,
    tx_sm: &mut StateMachine<'_, PIO0, 0>,
    tx_dma: &mut dma::Channel<'_>,
    tx_irq_flags: &IrqFlags<'_, PIO0>,
    rx_sm: &mut StateMachine<'_, PIO1, 0>,
    edge_sm: &mut StateMachine<'_, PIO1, 1>,
    rx_irq_flags: &IrqFlags<'_, PIO1>,
    edge_reset_instruction: u16,
    rx_reset_instruction: u16,
    rx_clear_x_instruction: u16,
) -> Result<usize, ProbeResult> {
    if !endpoint.is_in()
        || endpoint.transfer_type() != 0x02
        || endpoint.number() == 0
        || endpoint.max_packet_size == 0
        || endpoint.max_packet_size as usize > MAX_BULK_PACKET_BYTES
    {
        return Err(ProbeResult::BulkTransferFailed);
    }

    let in_token = token_packet(PID_IN, address, endpoint.number());
    let endpoint_packet_size = endpoint.max_packet_size as usize;
    let mut payload = [0_u8; MAX_BULK_PACKET_BYTES];
    let mut response = CdcAtResponseAccumulator::new();
    // Both buffers are initialized before entering the SRAM timing function.
    // The timing path overwrites every byte covered by its returned length.
    let mut raw = [0_u8; MAX_DECODED_BYTES];
    let mut attempt = 0_u16;

    loop {
        prepare_receive(rx_sm, rx_reset_instruction, rx_clear_x_instruction);
        park_receive(edge_sm, rx_irq_flags, edge_reset_instruction);
        let result = receive_data_packet(
            tx_sm,
            tx_irq_flags,
            rx_sm,
            rx_irq_flags,
            &in_token,
            &mut payload[..endpoint_packet_size],
            &mut raw,
            endpoint_packet_size,
            data_state.bulk_in_pid(),
        );
        rx_sm.set_enable(false);
        edge_sm.set_enable(false);
        attempt += 1;

        let failure = match result {
            Ok(InReceiveResult::Data { len }) => {
                *data_state = data_state.after_bulk_in_ack();
                if len != 0 {
                    if response.is_empty() {
                        info!(
                            "CDC response started with {} byte(s) from bulk IN endpoint {}",
                            len,
                            endpoint.number()
                        );
                    }
                    match response.push(&payload[..len as usize]) {
                        Ok(CdcAtResponseStatus::Ok) => return Ok(response.len()),
                        Ok(CdcAtResponseStatus::Pending) => ProbeResult::CdcResponseInvalid,
                        Ok(CdcAtResponseStatus::Error) | Err(_) => {
                            return Err(ProbeResult::CdcResponseInvalid);
                        }
                    }
                } else {
                    // A valid ZLP consumes a data toggle but carries no CDC
                    // bytes and is not an application response boundary.
                    ProbeResult::CdcResponseInvalid
                }
            }
            Ok(InReceiveResult::UnexpectedToggle) => ProbeResult::InvalidResponse,
            Ok(InReceiveResult::Nak) => ProbeResult::NonAck,
            Ok(InReceiveResult::Stall) => return Err(ProbeResult::BulkTransferFailed),
            Ok(InReceiveResult::NoResponse) => ProbeResult::NoResponse,
            Ok(InReceiveResult::InvalidPacket) => ProbeResult::InvalidResponse,
            Err(error) => return Err(ProbeResult::TransportError(error)),
        };

        if attempt >= CDC_RESPONSE_POLL_LIMIT {
            return Err(failure);
        }

        advance_probe_frame(
            tx_sm,
            tx_dma,
            tx_irq_flags,
            rx_irq_flags,
            probe_frame_number,
        )
        .await
        .map_err(ProbeResult::TransportError)?;
    }
}

#[allow(clippy::too_many_arguments)]
async fn bulk_in_scan_result(
    address: u8,
    endpoint: EndpointDescriptor,
    data_state: &mut CdcAcmDataState,
    probe_frame_number: &mut u16,
    tx_sm: &mut StateMachine<'_, PIO0, 0>,
    tx_dma: &mut dma::Channel<'_>,
    tx_irq_flags: &IrqFlags<'_, PIO0>,
    rx_sm: &mut StateMachine<'_, PIO1, 0>,
    edge_sm: &mut StateMachine<'_, PIO1, 1>,
    rx_irq_flags: &IrqFlags<'_, PIO1>,
    edge_reset_instruction: u16,
    rx_reset_instruction: u16,
    rx_clear_x_instruction: u16,
) -> Result<BleuIoScanResult, ProbeResult> {
    if !endpoint.is_in()
        || endpoint.transfer_type() != 0x02
        || endpoint.number() == 0
        || endpoint.max_packet_size == 0
        || endpoint.max_packet_size as usize > MAX_BULK_PACKET_BYTES
    {
        return Err(ProbeResult::BulkTransferFailed);
    }

    let in_token = token_packet(PID_IN, address, endpoint.number());
    let endpoint_packet_size = endpoint.max_packet_size as usize;
    let mut payload = [0_u8; MAX_BULK_PACKET_BYTES];
    let mut scan = BleuIoScanAccumulator::new();
    // Both buffers are initialized before entering the SRAM timing function.
    // The timing path overwrites every byte covered by its returned length.
    let mut raw = [0_u8; MAX_DECODED_BYTES];
    let mut attempt = 0_u16;

    loop {
        prepare_receive(rx_sm, rx_reset_instruction, rx_clear_x_instruction);
        park_receive(edge_sm, rx_irq_flags, edge_reset_instruction);
        let result = receive_data_packet(
            tx_sm,
            tx_irq_flags,
            rx_sm,
            rx_irq_flags,
            &in_token,
            &mut payload[..endpoint_packet_size],
            &mut raw,
            endpoint_packet_size,
            data_state.bulk_in_pid(),
        );
        rx_sm.set_enable(false);
        edge_sm.set_enable(false);
        attempt += 1;

        match result {
            Ok(InReceiveResult::Data { len }) => {
                // receive_data_packet() has already emitted the host ACK.
                // Advance first, then parse only this newly accepted payload;
                // a valid wrong-toggle duplicate never reaches this branch.
                *data_state = data_state.after_bulk_in_ack();
                if len != 0 {
                    match scan.push(&payload[..len as usize]) {
                        Ok(BleuIoScanStatus::Complete) => {
                            return scan.first_result().ok_or(ProbeResult::BleScanInvalid);
                        }
                        Ok(BleuIoScanStatus::Pending) => {}
                        Err(_) => return Err(ProbeResult::BleScanInvalid),
                    }
                }
            }
            Ok(InReceiveResult::UnexpectedToggle)
            | Ok(InReceiveResult::Nak)
            | Ok(InReceiveResult::NoResponse)
            | Ok(InReceiveResult::InvalidPacket) => {}
            Ok(InReceiveResult::Stall) => return Err(ProbeResult::BulkTransferFailed),
            Err(error) => return Err(ProbeResult::TransportError(error)),
        }

        if attempt >= BLE_SCAN_POLL_LIMIT {
            return Err(ProbeResult::BleScanInvalid);
        }

        advance_probe_frame(
            tx_sm,
            tx_dma,
            tx_irq_flags,
            rx_irq_flags,
            probe_frame_number,
        )
        .await
        .map_err(ProbeResult::TransportError)?;
    }
}

#[allow(clippy::too_many_arguments)]
async fn cdc_acm_command_expect_ok(
    address: u8,
    bulk_out_endpoint: EndpointDescriptor,
    bulk_in_endpoint: EndpointDescriptor,
    command: &[u8],
    data_state: &mut CdcAcmDataState,
    probe_frame_number: &mut u16,
    tx_sm: &mut StateMachine<'_, PIO0, 0>,
    tx_dma: &mut dma::Channel<'_>,
    tx_irq_flags: &IrqFlags<'_, PIO0>,
    rx_sm: &mut StateMachine<'_, PIO1, 0>,
    edge_sm: &mut StateMachine<'_, PIO1, 1>,
    rx_irq_flags: &IrqFlags<'_, PIO1>,
    edge_reset_instruction: u16,
    rx_reset_instruction: u16,
    rx_clear_x_instruction: u16,
    observation: &mut RxObservation,
) -> Result<usize, ProbeResult> {
    if !command.ends_with(b"\r\n") {
        return Err(ProbeResult::CdcResponseInvalid);
    }

    // Keep at most one host transaction in each frame. This SOF also
    // separates successive application commands without resetting either
    // endpoint's DATA-toggle state.
    advance_probe_frame(
        tx_sm,
        tx_dma,
        tx_irq_flags,
        rx_irq_flags,
        probe_frame_number,
    )
    .await
    .map_err(ProbeResult::TransportError)?;

    bulk_out_packet(
        address,
        bulk_out_endpoint,
        command,
        data_state,
        probe_frame_number,
        tx_sm,
        tx_dma,
        tx_irq_flags,
        rx_sm,
        edge_sm,
        rx_irq_flags,
        edge_reset_instruction,
        rx_reset_instruction,
        rx_clear_x_instruction,
        observation,
    )
    .await?;

    advance_probe_frame(
        tx_sm,
        tx_dma,
        tx_irq_flags,
        rx_irq_flags,
        probe_frame_number,
    )
    .await
    .map_err(ProbeResult::TransportError)?;

    bulk_in_ok_response(
        address,
        bulk_in_endpoint,
        data_state,
        probe_frame_number,
        tx_sm,
        tx_dma,
        tx_irq_flags,
        rx_sm,
        edge_sm,
        rx_irq_flags,
        edge_reset_instruction,
        rx_reset_instruction,
        rx_clear_x_instruction,
    )
    .await
}

#[allow(clippy::too_many_arguments)]
async fn cdc_acm_timed_gap_scan(
    address: u8,
    bulk_out_endpoint: EndpointDescriptor,
    bulk_in_endpoint: EndpointDescriptor,
    data_state: &mut CdcAcmDataState,
    probe_frame_number: &mut u16,
    tx_sm: &mut StateMachine<'_, PIO0, 0>,
    tx_dma: &mut dma::Channel<'_>,
    tx_irq_flags: &IrqFlags<'_, PIO0>,
    rx_sm: &mut StateMachine<'_, PIO1, 0>,
    edge_sm: &mut StateMachine<'_, PIO1, 1>,
    rx_irq_flags: &IrqFlags<'_, PIO1>,
    edge_reset_instruction: u16,
    rx_reset_instruction: u16,
    rx_clear_x_instruction: u16,
    observation: &mut RxObservation,
) -> Result<BleuIoScanResult, ProbeResult> {
    advance_probe_frame(
        tx_sm,
        tx_dma,
        tx_irq_flags,
        rx_irq_flags,
        probe_frame_number,
    )
    .await
    .map_err(ProbeResult::TransportError)?;

    bulk_out_packet(
        address,
        bulk_out_endpoint,
        GAP_SCAN_COMMAND,
        data_state,
        probe_frame_number,
        tx_sm,
        tx_dma,
        tx_irq_flags,
        rx_sm,
        edge_sm,
        rx_irq_flags,
        edge_reset_instruction,
        rx_reset_instruction,
        rx_clear_x_instruction,
        observation,
    )
    .await?;

    advance_probe_frame(
        tx_sm,
        tx_dma,
        tx_irq_flags,
        rx_irq_flags,
        probe_frame_number,
    )
    .await
    .map_err(ProbeResult::TransportError)?;

    bulk_in_scan_result(
        address,
        bulk_in_endpoint,
        data_state,
        probe_frame_number,
        tx_sm,
        tx_dma,
        tx_irq_flags,
        rx_sm,
        edge_sm,
        rx_irq_flags,
        edge_reset_instruction,
        rx_reset_instruction,
        rx_clear_x_instruction,
    )
    .await
}

#[allow(clippy::too_many_arguments)]
async fn set_device_address(
    new_address: u8,
    probe_frame_number: &mut u16,
    tx_sm: &mut StateMachine<'_, PIO0, 0>,
    tx_dma: &mut dma::Channel<'_>,
    tx_irq_flags: &IrqFlags<'_, PIO0>,
    rx_sm: &mut StateMachine<'_, PIO1, 0>,
    edge_sm: &mut StateMachine<'_, PIO1, 1>,
    rx_irq_flags: &IrqFlags<'_, PIO1>,
    edge_reset_instruction: u16,
    rx_reset_instruction: u16,
    rx_clear_x_instruction: u16,
    observation: &mut RxObservation,
) -> Result<(), ProbeResult> {
    let request =
        SetupRequest::set_address(new_address).map_err(|_| ProbeResult::ControlReadFailed)?;
    control_write_ep0(
        0,
        request,
        &[],
        probe_frame_number,
        tx_sm,
        tx_dma,
        tx_irq_flags,
        rx_sm,
        edge_sm,
        rx_irq_flags,
        edge_reset_instruction,
        rx_reset_instruction,
        rx_clear_x_instruction,
        observation,
    )
    .await
}

#[allow(clippy::too_many_arguments)]
async fn probe_default_control_endpoint(
    probe_frame_number: &mut u16,
    tx_sm: &mut StateMachine<'_, PIO0, 0>,
    tx_dma: &mut dma::Channel<'_>,
    tx_irq_flags: &IrqFlags<'_, PIO0>,
    rx_sm: &mut StateMachine<'_, PIO1, 0>,
    edge_sm: &mut StateMachine<'_, PIO1, 1>,
    rx_irq_flags: &IrqFlags<'_, PIO1>,
    edge_reset_instruction: u16,
    rx_reset_instruction: u16,
    rx_clear_x_instruction: u16,
    observation: &mut RxObservation,
) -> ProbeResult {
    let descriptor_at_zero = match read_device_descriptor_prefix(
        0,
        probe_frame_number,
        tx_sm,
        tx_dma,
        tx_irq_flags,
        rx_sm,
        edge_sm,
        rx_irq_flags,
        edge_reset_instruction,
        rx_reset_instruction,
        rx_clear_x_instruction,
        observation,
    )
    .await
    {
        Ok(descriptor) => descriptor,
        Err(result) => return result,
    };
    info!(
        "M4a descriptor at address 0: USB BCD {}, class {}, MPS0 {}",
        descriptor_at_zero.usb_version_bcd,
        descriptor_at_zero.device_class,
        descriptor_at_zero.max_packet_size0
    );

    const DEVICE_ADDRESS: u8 = 1;
    if let Err(result) = set_device_address(
        DEVICE_ADDRESS,
        probe_frame_number,
        tx_sm,
        tx_dma,
        tx_irq_flags,
        rx_sm,
        edge_sm,
        rx_irq_flags,
        edge_reset_instruction,
        rx_reset_instruction,
        rx_clear_x_instruction,
        observation,
    )
    .await
    {
        return result;
    }

    // USB 2.0 allows the device 2 ms of recovery after the status ACK before
    // it must accept requests using the new address. SOFs continue meanwhile.
    for _ in 0..2 {
        if let Err(error) = advance_probe_frame(
            tx_sm,
            tx_dma,
            tx_irq_flags,
            rx_irq_flags,
            probe_frame_number,
        )
        .await
        {
            return ProbeResult::TransportError(error);
        }
    }

    let descriptor_at_one = match read_device_descriptor_prefix(
        DEVICE_ADDRESS,
        probe_frame_number,
        tx_sm,
        tx_dma,
        tx_irq_flags,
        rx_sm,
        edge_sm,
        rx_irq_flags,
        edge_reset_instruction,
        rx_reset_instruction,
        rx_clear_x_instruction,
        observation,
    )
    .await
    {
        Ok(descriptor) => descriptor,
        Err(result) => return result,
    };

    if descriptor_at_one != descriptor_at_zero {
        return ProbeResult::ControlReadFailed;
    }

    info!("M4b verified: device responds at address 1");

    if let Err(error) = advance_probe_frame(
        tx_sm,
        tx_dma,
        tx_irq_flags,
        rx_irq_flags,
        probe_frame_number,
    )
    .await
    {
        return ProbeResult::TransportError(error);
    }

    let complete_descriptor = match read_complete_device_descriptor(
        DEVICE_ADDRESS,
        descriptor_at_one.max_packet_size0,
        probe_frame_number,
        tx_sm,
        tx_dma,
        tx_irq_flags,
        rx_sm,
        edge_sm,
        rx_irq_flags,
        edge_reset_instruction,
        rx_reset_instruction,
        rx_clear_x_instruction,
        observation,
    )
    .await
    {
        Ok(descriptor) => descriptor,
        Err(result) => return result,
    };

    if complete_descriptor.header() != descriptor_at_one {
        return ProbeResult::ControlReadFailed;
    }

    info!(
        "M4c verified: full device descriptor, VID {}, PID {}",
        complete_descriptor.vendor_id, complete_descriptor.product_id
    );
    if complete_descriptor.num_configurations == 0 {
        return ProbeResult::ConfigurationUnsupported;
    }

    if let Err(error) = advance_probe_frame(
        tx_sm,
        tx_dma,
        tx_irq_flags,
        rx_irq_flags,
        probe_frame_number,
    )
    .await
    {
        return ProbeResult::TransportError(error);
    }

    let cdc_acm = match discover_cdc_acm_configuration(
        DEVICE_ADDRESS,
        complete_descriptor.max_packet_size0,
        probe_frame_number,
        tx_sm,
        tx_dma,
        tx_irq_flags,
        rx_sm,
        edge_sm,
        rx_irq_flags,
        edge_reset_instruction,
        rx_reset_instruction,
        rx_clear_x_instruction,
        observation,
    )
    .await
    {
        Ok(function) => function,
        Err(result) => return result,
    };

    info!(
        "M5a CDC-ACM discovered: config {}, control IF {}, data IF {}, bulk OUT {}, bulk IN {}",
        cdc_acm.configuration.configuration_value,
        cdc_acm.control_interface,
        cdc_acm.data_interface,
        cdc_acm.bulk_out_endpoint.address,
        cdc_acm.bulk_in_endpoint.address
    );

    if let Err(error) = advance_probe_frame(
        tx_sm,
        tx_dma,
        tx_irq_flags,
        rx_irq_flags,
        probe_frame_number,
    )
    .await
    {
        return ProbeResult::TransportError(error);
    }

    if let Err(result) = control_write_ep0(
        DEVICE_ADDRESS,
        SetupRequest::set_configuration(cdc_acm.configuration.configuration_value),
        &[],
        probe_frame_number,
        tx_sm,
        tx_dma,
        tx_irq_flags,
        rx_sm,
        edge_sm,
        rx_irq_flags,
        edge_reset_instruction,
        rx_reset_instruction,
        rx_clear_x_instruction,
        observation,
    )
    .await
    {
        return result;
    }

    info!(
        "M5b verified: configuration {} selected",
        cdc_acm.configuration.configuration_value
    );

    if !cdc_acm.supports_line_requests() {
        return ProbeResult::ConfigurationUnsupported;
    }

    if let Err(error) = advance_probe_frame(
        tx_sm,
        tx_dma,
        tx_irq_flags,
        rx_irq_flags,
        probe_frame_number,
    )
    .await
    {
        return ProbeResult::TransportError(error);
    }

    if let Err(result) = control_write_ep0(
        DEVICE_ADDRESS,
        SetupRequest::set_control_line_state(cdc_acm.control_interface, true, true),
        &[],
        probe_frame_number,
        tx_sm,
        tx_dma,
        tx_irq_flags,
        rx_sm,
        edge_sm,
        rx_irq_flags,
        edge_reset_instruction,
        rx_reset_instruction,
        rx_clear_x_instruction,
        observation,
    )
    .await
    {
        return result;
    }

    info!(
        "M5c verified: DTR and RTS asserted on CDC control interface {}",
        cdc_acm.control_interface
    );

    if let Err(error) = advance_probe_frame(
        tx_sm,
        tx_dma,
        tx_irq_flags,
        rx_irq_flags,
        probe_frame_number,
    )
    .await
    {
        return ProbeResult::TransportError(error);
    }

    let line_coding = CdcLineCoding::eight_n_one(115_200).to_bytes();
    if let Err(result) = control_write_ep0(
        DEVICE_ADDRESS,
        SetupRequest::set_line_coding(cdc_acm.control_interface),
        &line_coding,
        probe_frame_number,
        tx_sm,
        tx_dma,
        tx_irq_flags,
        rx_sm,
        edge_sm,
        rx_irq_flags,
        edge_reset_instruction,
        rx_reset_instruction,
        rx_clear_x_instruction,
        observation,
    )
    .await
    {
        return result;
    }

    info!(
        "M5d verified: CDC line coding set to 115200 8N1 on interface {}",
        cdc_acm.control_interface
    );

    let mut cdc_data_state = CdcAcmDataState::new();
    let at_response_len = match cdc_acm_command_expect_ok(
        DEVICE_ADDRESS,
        cdc_acm.bulk_out_endpoint,
        cdc_acm.bulk_in_endpoint,
        ATTENTION_COMMAND,
        &mut cdc_data_state,
        probe_frame_number,
        tx_sm,
        tx_dma,
        tx_irq_flags,
        rx_sm,
        edge_sm,
        rx_irq_flags,
        edge_reset_instruction,
        rx_reset_instruction,
        rx_clear_x_instruction,
        observation,
    )
    .await
    {
        Ok(len) => len,
        Err(result) => return result,
    };

    info!(
        "M5e/M5g verified: AT accepted and {} CDC response bytes ended in OK on bulk IN endpoint {}",
        at_response_len,
        cdc_acm.bulk_in_endpoint.number()
    );

    let central_response_len = match cdc_acm_command_expect_ok(
        DEVICE_ADDRESS,
        cdc_acm.bulk_out_endpoint,
        cdc_acm.bulk_in_endpoint,
        CENTRAL_ROLE_COMMAND,
        &mut cdc_data_state,
        probe_frame_number,
        tx_sm,
        tx_dma,
        tx_irq_flags,
        rx_sm,
        edge_sm,
        rx_irq_flags,
        edge_reset_instruction,
        rx_reset_instruction,
        rx_clear_x_instruction,
        observation,
    )
    .await
    {
        Ok(len) => len,
        Err(result) => return result,
    };

    info!(
        "M5h verified: persistent CDC session entered BLE central role with {} response bytes",
        central_response_len
    );

    let scan_result = match cdc_acm_timed_gap_scan(
        DEVICE_ADDRESS,
        cdc_acm.bulk_out_endpoint,
        cdc_acm.bulk_in_endpoint,
        &mut cdc_data_state,
        probe_frame_number,
        tx_sm,
        tx_dma,
        tx_irq_flags,
        rx_sm,
        edge_sm,
        rx_irq_flags,
        edge_reset_instruction,
        rx_reset_instruction,
        rx_clear_x_instruction,
        observation,
    )
    .await
    {
        Ok(result) => result,
        Err(result) => return result,
    };

    info!(
        "M5i verified: GAP scan found device index {}, address type {}, RSSI {}",
        scan_result.index, scan_result.address_type, scan_result.rssi
    );
    ProbeResult::BleScanFound
}

pub async fn run(_spawner: Spawner) {
    let clocks = ClockConfig::system_freq(SYS_CLOCK_HZ).expect("valid 120 MHz PLL setup");
    let peripherals = embassy_rp::init(embassy_rp::config::Config::new(clocks));
    let mut tx_dma = dma::Channel::new(peripherals.DMA_CH0, Irqs);

    let mut status_led = Output::new(peripherals.PIN_13, Level::Low);

    // This pin is a logic signal for an external, current-limited 5 V load
    // switch. It must never be connected directly to USB VBUS.
    let mut vbus_enable = Output::new(peripherals.PIN_18, Level::Low);
    Timer::after_millis(100).await;
    vbus_enable.set_high();

    let mut pio_tx = Pio::new(peripherals.PIO0, Irqs);
    let mut pio_rx = Pio::new(peripherals.PIO1, Irqs);
    let mut dp = pio_tx.common.make_pio_pin(peripherals.PIN_16);
    let mut dm = pio_tx.common.make_pio_pin(peripherals.PIN_17);
    dp.set_pull(Pull::Down);
    dm.set_pull(Pull::Down);
    // Match Pico-PIO-USB's full-speed pad configuration. The Feather has
    // 22-ohm series resistors on both host data lines.
    dp.set_slew_rate(SlewRate::Fast);
    dm.set_slew_rate(SlewRate::Fast);
    dp.set_drive_strength(Drive::_12mA);
    dm.set_drive_strength(Drive::_12mA);

    // The RX programs are written for inverted inputs: FS idle becomes 0b10
    // and SE0 becomes 0b11. Output signalling is unaffected by INOVER.
    pac::IO_BANK0
        .gpio(16)
        .ctrl()
        .modify(|w| w.set_inover(pac::io::vals::Inover::INVERT));
    pac::IO_BANK0
        .gpio(17)
        .ctrl()
        .modify(|w| w.set_inover(pac::io::vals::Inover::INVERT));

    // Current Pico-PIO-USB full-speed transmitter. Unlike the historical
    // OUT-PC implementation, this consumes ordinary USB bytes and performs
    // NRZI encoding and bit stuffing in PIO.
    let tx_program = pio::pio_asm!(
        ".origin 0",
        ".side_set 2 opt",
        "start:",
        "set y, 5 side 0b01",
        "set pindirs, 0b11",
        ".wrap_target",
        "check_eop1:",
        "jmp !osre load_bit1",
        "nop [1]",
        "send_eop:",
        "irq 1 side 0b00 [3]",
        "nop [3]",
        "nop side 0b01",
        "set pindirs, 0b00 [3]",
        "irq wait 0",
        "jmp start",
        "load_bit1:",
        "out x, 1",
        "jmp !x low1",
        "high1:",
        "jmp y-- check_eop1 side 0b01",
        "nop [2]",
        "low1:",
        "set y, 5 side 0b10",
        "check_eop2:",
        "jmp !osre load_bit2",
        "jmp send_eop [1]",
        "load_bit2:",
        "out x, 1",
        "jmp !x low2",
        "high2:",
        "jmp y-- check_eop2 side 0b10",
        "nop [2]",
        "low2:",
        "set y, 5 side 0b01",
        ".wrap",
    );
    let tx_program = pio_tx.common.load_program(&tx_program.program);

    let idle_instruction = pio::pio_asm!(".side_set 2 opt", "jmp 0 side 0b01")
        .program
        .code[0];
    let force_k_instruction = pio::pio_asm!(".side_set 2 opt", "set pindirs, 0b11 side 0b10")
        .program
        .code[0];
    let release_instruction = pio::pio_asm!(".side_set 2 opt", "set pindirs, 0b00 side 0b01")
        .program
        .code[0];

    // Current Pico-PIO-USB NRZI decoder. It consumes timing triggers from
    // IRQ 4, removes stuffed bits, and pushes decoded bytes.
    let rx_program = pio::pio_asm!(
        ".origin 0",
        ".wrap_target",
        "set y, 6",
        "irq_wait:",
        "wait 1 irq 4",
        "jmp pin pin_high",
        "pin_low:",
        "jmp !y flip",
        "jmp !x k1",
        "in null, 1",
        "jmp flip",
        "k1:",
        "in osr, 1",
        "jmp y-- irq_wait",
        "pin_high:",
        "jmp !y flip",
        "jmp !x j1",
        "in x, 1",
        "jmp y-- irq_wait",
        "j1:",
        "in null, 1",
        "flip:",
        "mov x, ~x",
        ".wrap",
    );
    let rx_program = pio_rx.common.load_program(&rx_program.program);

    // Current Pico-PIO-USB edge/EOP detector. At 96 MHz each path supplies
    // one decoder trigger per 12 Mbit/s bit and resynchronizes on transitions.
    let edge_program = pio::pio_asm!(
        ".origin 15",
        "eop:",
        "irq wait 2",
        "start:",
        "jmp pin start",
        "irq 3 [1]",
        ".wrap_target",
        "pin_still_low:",
        "irq 4 [1]",
        "pin_low:",
        "jmp pin pin_went_high",
        "pin_went_low:",
        "jmp pin pin_went_high",
        "jmp pin pin_went_high",
        "jmp pin pin_went_high",
        "jmp pin pin_went_high",
        "jmp pin pin_went_high",
        ".wrap",
        "pin_still_high:",
        "mov x, isr [2]",
        "jmp x-- eop",
        "pin_went_high:",
        "mov isr, null",
        "in pins, 1",
        "irq 4",
        "jmp pin pin_still_high",
        "jmp pin pin_went_low",
    );
    let edge_program = pio_rx.common.load_program(&edge_program.program);

    // The RX programs fill all 32 PIO1 instruction slots. Hardware
    // diagnostics showed SM1 following the edge program's unrelocated local
    // jump target 1 and escaping into the decoder. Install the exact fully
    // relocated image used by Pico-PIO-USB: decoder at 0..14 and edge/EOP at
    // 15..31.
    const RX_PROGRAM_IMAGE: [u16; 32] = [
        0xe046, 0x20c4, 0x00c9, 0x006e, 0x0027, 0x4061, 0x000e, 0x40e1, 0x0081, 0x006e, 0x002d,
        0x4021, 0x0081, 0x4061, 0xa029, 0xc022, 0x00d0, 0xc103, 0xc104, 0x00db, 0x00db, 0x00db,
        0x00db, 0x00db, 0x00db, 0xa226, 0x004f, 0xa0c3, 0x4001, 0xc004, 0x00d9, 0x0014,
    ];
    for (address, instruction) in RX_PROGRAM_IMAGE.iter().copied().enumerate() {
        pac::PIO1
            .instr_mem(address)
            .write(|w| w.set_instr_mem(instruction));
    }

    let rx_reset_instruction = pio::pio_asm!("jmp 0").program.code[0];
    let rx_clear_x_instruction = pio::pio_asm!("set x, 0").program.code[0];
    let rx_fill_osr_instruction = pio::pio_asm!("mov osr, ~null").program.code[0];
    let edge_start_instruction = pio::pio_asm!("jmp 16").program.code[0];
    let edge_reset_instruction = pio::pio_asm!("jmp 15").program.code[0];

    let mut tx_sm = pio_tx.sm0;
    tx_sm.set_pin_dirs(Direction::In, &[&dp, &dm]);
    tx_sm.set_pins(Level::High, &[&dp]);
    tx_sm.set_pins(Level::Low, &[&dm]);

    let mut tx_config = PioConfig::default();
    tx_config.set_out_pins(&[&dp, &dm]);
    tx_config.set_set_pins(&[&dp, &dm]);
    tx_config.fifo_join = FifoJoin::TxOnly;
    tx_config.shift_out.direction = ShiftDirection::Right;
    tx_config.shift_out.auto_fill = true;
    tx_config.shift_out.threshold = 8;
    tx_config.clock_divider = calculate_pio_clock_divider(USB_TX_PIO_HZ).to_fixed();
    tx_config.use_program(&tx_program, &[&dp, &dm]);
    tx_sm.set_enable(false);
    tx_sm.set_config(&tx_config);
    {
        // Exact register image produced by usb_tx_fs_program_init() in the
        // locally verified Pico-PIO-USB control firmware.
        let hw = pac::PIO0.sm(0);
        hw.clkdiv()
            .write_value(pac::pio::regs::SmClkdiv(0x0002_8000));
        hw.execctrl()
            .write_value(pac::pio::regs::SmExecctrl(0x4001_5100));
        hw.shiftctrl()
            .write_value(pac::pio::regs::SmShiftctrl(0x500e_0000));
        hw.pinctrl()
            .write_value(pac::pio::regs::SmPinctrl(0x6820_4210));
    }
    tx_sm.clear_fifos();
    tx_sm.restart();
    tx_sm.clkdiv_restart();
    unsafe { tx_sm.exec_instr(idle_instruction) };
    tx_sm.set_enable(true);

    let mut rx_sm = pio_rx.sm0;
    let mut rx_config = PioConfig::default();
    let mut rx_pins = rx_config.get_pins();
    rx_pins.in_base = 16;
    unsafe { rx_config.set_pins(rx_pins) };
    let mut rx_exec = rx_config.get_exec();
    rx_exec.jmp_pin = 16;
    unsafe { rx_config.set_exec(rx_exec) };
    rx_config.fifo_join = FifoJoin::RxOnly;
    rx_config.shift_in.direction = ShiftDirection::Right;
    rx_config.shift_in.auto_fill = true;
    rx_config.shift_in.threshold = 8;
    rx_config.use_program(&rx_program, &[]);
    rx_sm.set_config(&rx_config);
    {
        // Exact usb_rx_fs_program_init() register image from the working
        // Pico-PIO-USB reference, eliminating HAL-default ambiguity.
        let hw = pac::PIO1.sm(0);
        hw.clkdiv()
            .write_value(pac::pio::regs::SmClkdiv(0x0001_0000));
        hw.execctrl()
            .write_value(pac::pio::regs::SmExecctrl(0x1000_e000));
        hw.shiftctrl()
            .write_value(pac::pio::regs::SmShiftctrl(0x808d_0000));
        hw.pinctrl()
            .write_value(pac::pio::regs::SmPinctrl(0x0008_0000));
    }
    rx_sm.set_enable(false);
    unsafe { rx_sm.exec_instr(rx_fill_osr_instruction) };

    let mut edge_sm = pio_rx.sm1;
    let mut edge_config = PioConfig::default();
    let mut edge_pins = edge_config.get_pins();
    edge_pins.in_base = 16;
    unsafe { edge_config.set_pins(edge_pins) };
    let mut edge_exec = edge_config.get_exec();
    edge_exec.jmp_pin = 17;
    unsafe { edge_config.set_exec(edge_exec) };
    edge_config.shift_in.direction = ShiftDirection::Left;
    edge_config.shift_in.auto_fill = false;
    edge_config.shift_in.threshold = 8;
    edge_config.clock_divider = calculate_pio_clock_divider(USB_EDGE_PIO_HZ).to_fixed();
    edge_config.use_program(&edge_program, &[]);
    edge_sm.set_config(&edge_config);
    {
        // Exact eop_detect_fs_program_init() register image from the same
        // reference firmware.
        let hw = pac::PIO1.sm(1);
        hw.clkdiv()
            .write_value(pac::pio::regs::SmClkdiv(0x0001_4000));
        hw.execctrl()
            .write_value(pac::pio::regs::SmExecctrl(0x1101_8900));
        hw.shiftctrl()
            .write_value(pac::pio::regs::SmShiftctrl(0x0088_0000));
        hw.pinctrl()
            .write_value(pac::pio::regs::SmPinctrl(0x0008_0000));
    }
    unsafe { edge_sm.exec_instr(edge_start_instruction) };
    edge_sm.set_enable(true);

    info!("M5i ready: enter central role and run one-second BLE GAP scan");

    let mut detector = AttachDetector::new(ATTACH_DEBOUNCE_SAMPLES);
    let tx_irq_flags = pio_tx.irq_flags;
    let rx_irq_flags = pio_rx.irq_flags;
    let mut host_active = false;
    let mut frame_number = 0_u16;
    let mut heartbeat_frames = 0_u16;
    let mut led_pattern = LedPattern::Diagnostic(1);
    let mut sof_ticker = Ticker::every(Duration::from_millis(1));

    loop {
        if !host_active {
            Timer::after_millis(1).await;
            let line_state = read_line_state();
            let Some(event) = detector.update(line_state) else {
                continue;
            };

            match event {
                BusEvent::Attached(DeviceSpeed::Full) => {
                    info!("full-speed device attached");
                    status_led.set_high();

                    reset_full_speed_bus(&mut tx_sm, &dp, &dm).await;

                    if cfg!(feature = "analyzer-capture") {
                        // Produce one clean reset followed only by a fixed
                        // SOF0 once per millisecond. This gives an external
                        // analyzer a stable waveform without synthetic pulse
                        // tests or SETUP transactions in the capture.
                        frame_number = 0;
                        heartbeat_frames = 0;
                        led_pattern = LedPattern::Diagnostic(9);
                        sof_ticker = Ticker::every(Duration::from_millis(1));
                        host_active = true;
                        info!("analyzer capture mode: fixed SOF0 stream");
                        continue;
                    }

                    let mut preflight_failure = None;
                    let pulse_ok = match verify_rx_k_pulse(
                        &mut rx_sm,
                        &mut edge_sm,
                        &rx_irq_flags,
                        rx_reset_instruction,
                        rx_clear_x_instruction,
                        edge_start_instruction,
                    ) {
                        LoopbackResult::SawStart => true,
                        LoopbackResult::ArmWait => {
                            preflight_failure = Some(ProbeResult::RxPulseArmWait);
                            false
                        }
                        LoopbackResult::EdgeWait => {
                            preflight_failure = Some(ProbeResult::RxPulseEdgeWait);
                            false
                        }
                    };

                    let pio_pulse_ok = if pulse_ok {
                        // Restore a standards-compliant bus state after the
                        // synthetic K pulse before testing the real TX path.
                        reset_full_speed_bus(&mut tx_sm, &dp, &dm).await;

                        match verify_tx_pio_k_pulse(
                            &mut tx_sm,
                            &mut rx_sm,
                            &mut edge_sm,
                            &rx_irq_flags,
                            force_k_instruction,
                            release_instruction,
                            idle_instruction,
                            rx_reset_instruction,
                            rx_clear_x_instruction,
                            edge_start_instruction,
                        ) {
                            PioPulseResult::SawStart => true,
                            PioPulseResult::PinsDidNotReachK => {
                                preflight_failure = Some(ProbeResult::TxPioPulsePinsFailed);
                                false
                            }
                            PioPulseResult::ReceiverMissed => {
                                preflight_failure = Some(ProbeResult::TxPioPulseReceiverMissed);
                                false
                            }
                        }
                    } else {
                        false
                    };

                    let packet_loopback_ok = if pio_pulse_ok {
                        // Undo the deliberately long PIO-driven K pulse
                        // before exercising the timed DMA/NRZI packet path.
                        reset_full_speed_bus(&mut tx_sm, &dp, &dm).await;

                        match verify_rx_packet_loopback(
                            &mut tx_sm,
                            &mut tx_dma,
                            &tx_irq_flags,
                            &mut rx_sm,
                            &mut edge_sm,
                            &rx_irq_flags,
                            edge_start_instruction,
                            rx_reset_instruction,
                            rx_clear_x_instruction,
                        )
                        .await
                        {
                            Ok(PacketLoopbackResult::ValidSof) => true,
                            Ok(PacketLoopbackResult::NoStart) => {
                                preflight_failure = Some(ProbeResult::TxPacketLoopbackFailed);
                                false
                            }
                            Ok(PacketLoopbackResult::InvalidPacket) => {
                                preflight_failure = Some(ProbeResult::TxPacketLoopbackInvalid);
                                false
                            }
                            Err(error) => {
                                preflight_failure = Some(ProbeResult::TransportError(error));
                                false
                            }
                        }
                    } else {
                        false
                    };

                    if !packet_loopback_ok {
                        warn!("RX loopback failed; continuing with external control probe");
                    }
                    let mut rx_observation = RxObservation::default();
                    // verify_rx_packet_loopback() placed SOF0 on the bus.
                    // Keep one monotonically increasing frame counter through
                    // control retries, address recovery and steady-state SOF.
                    frame_number = 1;
                    let mut probe_result = probe_default_control_endpoint(
                        &mut frame_number,
                        &mut tx_sm,
                        &mut tx_dma,
                        &tx_irq_flags,
                        &mut rx_sm,
                        &mut edge_sm,
                        &rx_irq_flags,
                        edge_reset_instruction,
                        rx_reset_instruction,
                        rx_clear_x_instruction,
                        &mut rx_observation,
                    )
                    .await;

                    if probe_result != ProbeResult::BleScanFound && preflight_failure.is_some() {
                        warn!("control probe and one preflight diagnostic both failed");
                    }

                    if probe_result != ProbeResult::BleScanFound {
                        // Emit one deliberately unaddressed DATA0 packet for
                        // the external analyzer. Payload:
                        // "RX", FIFO length, IRQ flags, four raw bytes,
                        // decoder PC, edge-detector PC and pad/peripheral
                        // input levels.
                        // No device will consume it because it has no token.
                        Timer::after_micros(100).await;
                        let diagnostic_packet = rx_diagnostic_packet(rx_observation);
                        if let Err(error) = transmit_full_speed(
                            &mut tx_sm,
                            &mut tx_dma,
                            &tx_irq_flags,
                            &rx_irq_flags,
                            &diagnostic_packet,
                            false,
                        )
                        .await
                        {
                            probe_result = ProbeResult::TransportError(error);
                        }
                    }

                    heartbeat_frames = 0;
                    led_pattern = probe_result.led_pattern();
                    sof_ticker = Ticker::every(Duration::from_millis(1));
                    host_active = true;
                    if probe_result == ProbeResult::BleScanFound {
                        info!("M5i verified: timed GAP scan returned a structured BLE device");
                    } else {
                        warn!("M5i CDC/BLE scan probe failed; LED shows diagnostic code");
                    }
                }
                BusEvent::Attached(DeviceSpeed::Low) => {
                    warn!("low-speed device attached; M3 is full-speed only");
                    status_led.set_high();
                }
                BusEvent::Detached => {
                    info!("device detached");
                    status_led.set_low();
                }
                BusEvent::Invalid => {
                    error!("stable SE1 bus state; check wiring");
                    status_led.set_low();
                }
            }
            continue;
        }

        sof_ticker.next().await;

        if detector.update(read_line_state()) == Some(BusEvent::Detached) {
            info!("device detached; SOF scheduler stopped");
            host_active = false;
            status_led.set_low();
        }

        if !host_active {
            continue;
        }

        let scheduled_frame = if cfg!(feature = "analyzer-capture") {
            0
        } else {
            frame_number
        };
        let sof = sof_packet(scheduled_frame);
        let sof_tx_result = transmit_full_speed(
            &mut tx_sm,
            &mut tx_dma,
            &tx_irq_flags,
            &rx_irq_flags,
            &sof,
            false,
        )
        .await;
        if let Err(error) = sof_tx_result {
            led_pattern = ProbeResult::TransportError(error).led_pattern();
        }
        frame_number = (frame_number + 1) & 0x07ff;
        // 20 s is a common multiple of the 5 s diagnostic cycle and the
        // 2.5 s BLE-scan six-pulse heartbeat, so neither pattern jumps.
        heartbeat_frames = (heartbeat_frames + 1) % 20_000;
        status_led.set_level(diagnostic_led_level(led_pattern, heartbeat_frames));
    }
}
