//! RP2040 PIO implementation of the serialised host packet engine.
//!
//! The PIO programs, state-machine assignments and explicit register images
//! preserve the analyzer-verified full-speed implementation developed during
//! the incremental board bring-up. The low-speed profile swaps the transmitter
//! image, J/K pin roles and PIO clocks in place while retaining the same
//! packet engine. Both the neutral firmware and named examples use this shared
//! backend; the former monolithic implementation remains a historical
//! reference.

use core::time::Duration as CoreDuration;

use embassy_rp::Peri;
use embassy_rp::dma::{self, InterruptHandler as DmaInterruptHandler};
use embassy_rp::gpio::{Drive, Level, Pull, SlewRate};
use embassy_rp::interrupt::typelevel::Binding;
use embassy_rp::pac;
use embassy_rp::peripherals::{DMA_CH0, PIN_16, PIN_17, PIO0, PIO1};
use embassy_rp::pio::{
    Config as PioConfig, Direction, FifoJoin, Instance as PioInstance,
    InterruptHandler as PioInterruptHandler, IrqFlags, Pin as PioPin, Pio, ShiftDirection,
    StateMachine,
};
use embassy_rp::pio_programs::clock_divider::calculate_pio_clock_divider;
use embassy_time::{Duration, Instant, Timer};
use fixed::traits::ToFixed;

use super::{PioHostState, PioPacketEngine, PipeTarget, TransactionOutcome};
use crate::LineState;
use crate::host::{EndpointType, PipeError, Speed, TimeoutConfig};
use crate::usb::{
    DataToggle, InDataDisposition, MAX_DECODED_BYTES, PID_ACK, PID_DATA0, PID_DATA1, PID_IN,
    PID_NAK, PID_OUT, PID_SETUP, PID_STALL, ParsedPacket, RawDataPacket, SYNC, classify_in_data,
    crc16_data, parse_packet, sof_packet, token_packet,
};

const SYS_CLOCK_HZ: u32 = 120_000_000;
const USB_TX_PIO_HZ: u32 = 48_000_000;
const USB_EDGE_PIO_HZ: u32 = 96_000_000;
const USB_RESET_MS: u64 = 20;
const USB_RESET_RECOVERY_MS: u64 = 10;
const RX_IRQ_MASK: u8 = 0b0001_1110;
const TX_EOP_TIMEOUT_US: u64 = 100;
const FULL_SPEED_TX_EOP_GUARD_CYCLES: u32 = SYS_CLOCK_HZ / 2_000_000;
const LOW_SPEED_TX_EOP_GUARD_CYCLES: u32 = SYS_CLOCK_HZ / 333_333;
const FULL_SPEED_RX_ACK_EOP_GUARD_CYCLES: u32 = 0;
const LOW_SPEED_RX_ACK_EOP_GUARD_CYCLES: u32 = SYS_CLOCK_HZ / 500_000;
const TX_IRQ_POLL_BUDGET: u32 = 100_000;
const RX_PACKET_POLL_BUDGET: u32 = 20_000;
const MAX_WIRE_PACKET_BYTES: usize = MAX_DECODED_BYTES;
const CRC16_USB_RESIDUE: u16 = 0xb001;
const CONTROL_SETUP_BAD_RESPONSE_RETRY_LIMIT: u8 = 3;

const TX_FULL_SPEED_PROGRAM_IMAGE: [u16; 22] = [
    0xf445, 0xe083, 0x00ea, 0xa142, 0xd301, 0xa342, 0xb442, 0xe380, 0xc020, 0x0000, 0x6021, 0x002e,
    0x1482, 0xa242, 0xf845, 0x00f1, 0x0104, 0x6021, 0x0035, 0x188f, 0xa242, 0xf445,
];

const TX_LOW_SPEED_PROGRAM_IMAGE: [u16; 22] = [
    0xf845, 0xe083, 0x00ea, 0xa142, 0xd301, 0xa342, 0xb842, 0xe380, 0xc020, 0x0000, 0x6021, 0x002e,
    0x1882, 0xa242, 0xf445, 0x00f1, 0x0104, 0x6021, 0x0035, 0x148f, 0xa242, 0xf845,
];

#[derive(Clone, Copy)]
struct WireProfile {
    speed: Speed,
    tx_program: &'static [u16; 22],
    tx_clkdiv_image: u32,
    rx_execctrl_image: u32,
    rx_pinctrl_image: u32,
    edge_clkdiv_image: u32,
    edge_execctrl_image: u32,
    edge_pinctrl_image: u32,
    tx_eop_guard_cycles: u32,
    rx_ack_eop_guard_cycles: u32,
    handshake_start_timeout_us: u64,
    handshake_packet_timeout_us: u64,
}

impl WireProfile {
    const FULL: Self = Self {
        speed: Speed::Full,
        tx_program: &TX_FULL_SPEED_PROGRAM_IMAGE,
        tx_clkdiv_image: 0x0002_8000,
        rx_execctrl_image: 0x1000_e000,
        rx_pinctrl_image: 0x0008_0000,
        edge_clkdiv_image: 0x0001_4000,
        edge_execctrl_image: 0x1101_8900,
        edge_pinctrl_image: 0x0008_0000,
        tx_eop_guard_cycles: FULL_SPEED_TX_EOP_GUARD_CYCLES,
        rx_ack_eop_guard_cycles: FULL_SPEED_RX_ACK_EOP_GUARD_CYCLES,
        handshake_start_timeout_us: 5,
        handshake_packet_timeout_us: 20,
    };

    const LOW: Self = Self {
        speed: Speed::Low,
        tx_program: &TX_LOW_SPEED_PROGRAM_IMAGE,
        tx_clkdiv_image: 0x0014_0000,
        rx_execctrl_image: 0x1100_e000,
        rx_pinctrl_image: 0x0008_8000,
        edge_clkdiv_image: 0x000a_0000,
        edge_execctrl_image: 0x1001_8900,
        edge_pinctrl_image: 0x0008_8000,
        tx_eop_guard_cycles: LOW_SPEED_TX_EOP_GUARD_CYCLES,
        rx_ack_eop_guard_cycles: LOW_SPEED_RX_ACK_EOP_GUARD_CYCLES,
        handshake_start_timeout_us: 20,
        handshake_packet_timeout_us: 100,
    };

    const fn for_speed(speed: Speed) -> Option<Self> {
        match speed {
            Speed::Full => Some(Self::FULL),
            Speed::Low => Some(Self::LOW),
            Speed::High => None,
        }
    }
}

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

// Keep the lookup table in SRAM with the receive/ACK critical path.
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
    input_override_snapshot: u8,
}

/// Wire stage that first produced [`PipeError::BadResponse`] after a clear.
///
/// This diagnostic is kept outside Embassy's portable error type so the
/// controller remains API-compatible while hardware bring-up can still
/// distinguish failures in the SETUP, DATA and status stages.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BadResponseSite {
    ControlInContract,
    ControlInSetup,
    ControlInData,
    ControlInStatus,
    ControlOutContract,
    ControlOutSetup,
    ControlOutData,
    ControlOutStatus,
}

impl BadResponseSite {
    /// Stable one-based code used by analyzer-oriented examples.
    pub const fn diagnostic_code(self) -> u8 {
        match self {
            Self::ControlInContract => 1,
            Self::ControlInSetup => 2,
            Self::ControlInData => 3,
            Self::ControlInStatus => 4,
            Self::ControlOutContract => 5,
            Self::ControlOutSetup => 6,
            Self::ControlOutData => 7,
            Self::ControlOutStatus => 8,
        }
    }
}

/// Additional classification of an invalid handshake observation.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum HandshakeFailure {
    RxDecoderError,
    FalseStart,
    IncompletePacket,
    WrongLength,
    InvalidSync,
    InvalidPidComplement,
    UnexpectedPid,
    Unknown,
}

impl HandshakeFailure {
    /// Stable one-based code used by analyzer-oriented examples.
    pub const fn diagnostic_code(self) -> u8 {
        match self {
            Self::RxDecoderError => 1,
            Self::FalseStart => 2,
            Self::IncompletePacket => 3,
            Self::WrongLength => 4,
            Self::InvalidSync => 5,
            Self::InvalidPidComplement => 6,
            Self::UnexpectedPid => 7,
            Self::Unknown => 8,
        }
    }
}

/// First `BadResponse` source and any available handshake classification.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct BadResponseDiagnostic {
    pub site: BadResponseSite,
    pub handshake_failure: Option<HandshakeFailure>,
    pub handshake_observation: Option<HandshakeObservationDiagnostic>,
}

/// Raw receiver state captured when a handshake packet could not be decoded.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct HandshakeObservationDiagnostic {
    pub len: u8,
    pub irq_flags: u8,
    pub bytes: [u8; 4],
    pub rx_program_counter: u8,
    pub edge_program_counter: u8,
    pub input_snapshot: u8,
    pub input_override_snapshot: u8,
}

impl From<RxObservation> for HandshakeObservationDiagnostic {
    fn from(observation: RxObservation) -> Self {
        Self {
            len: observation.len,
            irq_flags: observation.irq_flags,
            bytes: observation.bytes,
            rx_program_counter: observation.program_counter,
            edge_program_counter: observation.edge_program_counter,
            input_snapshot: observation.input_snapshot,
            input_override_snapshot: observation.input_override_snapshot,
        }
    }
}

/// Hardware owner for the verified PIO0 TX and PIO1 RX state machines.
///
/// This first backend intentionally fixes the resources to the Feather
/// RP2040 USB Host wiring: PIO0 SM0 for TX, PIO1 SM0/SM1 for RX, GPIO16/17
/// for D+/D-, and DMA channel 0. Generalising resource selection is deferred
/// until after analyzer equivalence has been established.
pub struct Rp2040PioEngine<'d> {
    tx_sm: StateMachine<'d, PIO0, 0>,
    rx_sm: StateMachine<'d, PIO1, 0>,
    edge_sm: StateMachine<'d, PIO1, 1>,
    tx_dma: dma::Channel<'d>,
    dp: PioPin<'d, PIO0>,
    dm: PioPin<'d, PIO0>,
    tx_irq_flags: IrqFlags<'d, PIO0>,
    rx_irq_flags: IrqFlags<'d, PIO1>,
    tx_idle_instruction: u16,
    tx_full_speed_idle_instruction: u16,
    tx_low_speed_idle_instruction: u16,
    rx_reset_instruction: u16,
    rx_clear_x_instruction: u16,
    edge_start_instruction: u16,
    edge_reset_instruction: u16,
    profile: WireProfile,
    frame_number: u16,
    next_frame: Instant,
    bus_ready_at: Instant,
    first_bad_response: Option<BadResponseDiagnostic>,
}

impl<'d> Rp2040PioEngine<'d> {
    /// Configure the full-speed baseline PIO programs and register images used
    /// by the working GPIO16/GPIO17 application. A low-speed attachment swaps
    /// the speed-specific image and register fields before bus reset.
    ///
    /// Panics before touching the PIO blocks unless `clk_sys` is exactly
    /// 120 MHz. The verified register images and SRAM polling budgets are
    /// clock-specific.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        pio0: Peri<'d, PIO0>,
        pio1: Peri<'d, PIO1>,
        dma_ch0: Peri<'d, DMA_CH0>,
        dp_pin: Peri<'d, PIN_16>,
        dm_pin: Peri<'d, PIN_17>,
        pio0_irq: impl Binding<
            <PIO0 as embassy_rp::pio::Instance>::Interrupt,
            PioInterruptHandler<PIO0>,
        > + 'd,
        pio1_irq: impl Binding<
            <PIO1 as embassy_rp::pio::Instance>::Interrupt,
            PioInterruptHandler<PIO1>,
        > + 'd,
        dma_irq: impl Binding<
            <DMA_CH0 as dma::ChannelInstance>::Interrupt,
            DmaInterruptHandler<DMA_CH0>,
        > + 'd,
    ) -> Self {
        assert_eq!(
            embassy_rp::clocks::clk_sys_freq(),
            SYS_CLOCK_HZ,
            "Rp2040PioEngine requires a 120 MHz clk_sys"
        );
        let tx_dma = dma::Channel::new(dma_ch0, dma_irq);
        let mut pio_tx = Pio::new(pio0, pio0_irq);
        let mut pio_rx = Pio::new(pio1, pio1_irq);
        let mut dp = pio_tx.common.make_pio_pin(dp_pin);
        let mut dm = pio_tx.common.make_pio_pin(dm_pin);
        dp.set_pull(Pull::Down);
        dm.set_pull(Pull::Down);
        dp.set_slew_rate(SlewRate::Fast);
        dm.set_slew_rate(SlewRate::Fast);
        dp.set_drive_strength(Drive::_12mA);
        dm.set_drive_strength(Drive::_12mA);

        // RX is written for inverted inputs. Output signalling is unaffected.
        configure_rx_input_inversion();

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
        let low_speed_idle_instruction = pio::pio_asm!(".side_set 2 opt", "jmp 0 side 0b10")
            .program
            .code[0];

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

        // embassy-rp 0.10.0's default SealedInstance::state() gives PIO0 and
        // PIO1 the same five-user counter. Constructing both blocks resets
        // that shared count to five, although ten Common/SM handles exist.
        // Dropping all seven unused handles would therefore reach zero early
        // and deconfigure GPIO16/17 (FUNCSEL=NULL, INOVER=NORMAL).
        //
        // Suppress Drop for exactly the five unused SM handles while allowing
        // both Common handles to drop. The resulting count is three, matching
        // tx_sm, rx_sm, and edge_sm retained below, so final engine teardown
        // still reaches zero and releases the pins normally.
        forget_unused_state_machine(pio_tx.sm1);
        forget_unused_state_machine(pio_tx.sm2);
        forget_unused_state_machine(pio_tx.sm3);
        forget_unused_state_machine(pio_rx.sm2);
        forget_unused_state_machine(pio_rx.sm3);

        let tx_irq_flags = pio_tx.irq_flags;
        let rx_irq_flags = pio_rx.irq_flags;

        Self {
            tx_sm,
            rx_sm,
            edge_sm,
            tx_dma,
            dp,
            dm,
            tx_irq_flags,
            rx_irq_flags,
            tx_idle_instruction: idle_instruction,
            tx_full_speed_idle_instruction: idle_instruction,
            tx_low_speed_idle_instruction: low_speed_idle_instruction,
            rx_reset_instruction,
            rx_clear_x_instruction,
            edge_start_instruction,
            edge_reset_instruction,
            profile: WireProfile::FULL,
            frame_number: 0,
            next_frame: Instant::now() + Duration::from_millis(1),
            bus_ready_at: Instant::now(),
            first_bad_response: None,
        }
    }

    /// Sample the physical GPIO16/GPIO17 bus state.
    pub fn line_state(&self) -> LineState {
        root_line_state()
    }

    fn configure_wire_speed(&mut self, speed: Speed) -> Result<(), PipeError> {
        let Some(profile) = WireProfile::for_speed(speed) else {
            return Err(PipeError::BadResponse);
        };
        if self.profile.speed == profile.speed {
            return Ok(());
        }

        self.tx_sm.set_enable(false);
        self.rx_sm.set_enable(false);
        self.edge_sm.set_enable(false);
        self.tx_sm.clear_fifos();
        self.rx_sm.clear_fifos();
        self.edge_sm.clear_fifos();
        self.tx_irq_flags.clear_all(0b11);
        self.rx_irq_flags.clear_all(RX_IRQ_MASK);

        for (address, instruction) in profile.tx_program.iter().copied().enumerate() {
            pac::PIO0
                .instr_mem(address)
                .write(|w| w.set_instr_mem(instruction));
        }

        pac::PIO0
            .sm(0)
            .clkdiv()
            .write_value(pac::pio::regs::SmClkdiv(profile.tx_clkdiv_image));
        pac::PIO1
            .sm(0)
            .execctrl()
            .write_value(pac::pio::regs::SmExecctrl(profile.rx_execctrl_image));
        pac::PIO1
            .sm(0)
            .pinctrl()
            .write_value(pac::pio::regs::SmPinctrl(profile.rx_pinctrl_image));
        pac::PIO1
            .sm(1)
            .clkdiv()
            .write_value(pac::pio::regs::SmClkdiv(profile.edge_clkdiv_image));
        pac::PIO1
            .sm(1)
            .execctrl()
            .write_value(pac::pio::regs::SmExecctrl(profile.edge_execctrl_image));
        pac::PIO1
            .sm(1)
            .pinctrl()
            .write_value(pac::pio::regs::SmPinctrl(profile.edge_pinctrl_image));

        self.tx_idle_instruction = match speed {
            Speed::Full => self.tx_full_speed_idle_instruction,
            Speed::Low => self.tx_low_speed_idle_instruction,
            Speed::High => return Err(PipeError::BadResponse),
        };
        self.profile = profile;

        self.tx_sm.set_pins(Level::Low, &[&self.dp, &self.dm]);
        match speed {
            Speed::Full => self.tx_sm.set_pins(Level::High, &[&self.dp]),
            Speed::Low => self.tx_sm.set_pins(Level::High, &[&self.dm]),
            Speed::High => return Err(PipeError::BadResponse),
        }
        self.tx_sm
            .set_pin_dirs(Direction::In, &[&self.dp, &self.dm]);
        self.tx_sm.restart();
        self.tx_sm.clkdiv_restart();
        unsafe { self.tx_sm.exec_instr(self.tx_idle_instruction) };
        self.tx_sm.set_enable(true);

        self.rx_sm.restart();
        self.rx_sm.clkdiv_restart();
        self.edge_sm.restart();
        self.edge_sm.clkdiv_restart();
        unsafe { self.edge_sm.exec_instr(self.edge_start_instruction) };
        self.edge_sm.set_enable(true);
        Ok(())
    }

    async fn reset(&mut self, speed: Speed) -> Result<(), PipeError> {
        self.configure_wire_speed(speed)?;
        self.frame_number = 0;
        reset_bus(
            &mut self.tx_sm,
            &self.dp,
            &self.dm,
            self.profile.speed,
            &mut self.bus_ready_at,
        )
        .await;
        self.next_frame = Instant::now() + Duration::from_millis(1);
        Ok(())
    }

    async fn advance_frame(&mut self) -> Result<(), PipeError> {
        Timer::at(self.bus_ready_at).await;
        let scheduled_frame = self.next_frame;
        Timer::at(scheduled_frame).await;
        self.transmit_frame_marker().await?;
        self.schedule_frame_after(scheduled_frame);
        Ok(())
    }

    async fn transmit_frame_marker(&mut self) -> Result<(), PipeError> {
        let result = match self.profile.speed {
            Speed::Full => {
                let sof = sof_packet(self.frame_number);
                self.frame_number = (self.frame_number + 1) & 0x07ff;
                transmit_packet(
                    &mut self.tx_sm,
                    &mut self.tx_dma,
                    &self.tx_irq_flags,
                    &self.rx_irq_flags,
                    &sof,
                    false,
                    self.profile.tx_eop_guard_cycles,
                )
                .await
            }
            Speed::Low => transmit_low_speed_keep_alive(
                &mut self.tx_sm,
                &self.tx_irq_flags,
                self.profile.tx_eop_guard_cycles,
            ),
            Speed::High => return Err(PipeError::BadResponse),
        };
        if let Err(error) = result {
            self.recover_tx_state();
            return Err(map_tx_error(error));
        }
        Ok(())
    }

    fn schedule_frame_after(&mut self, scheduled_frame: Instant) {
        let nominal_next = scheduled_frame + Duration::from_millis(1);
        self.next_frame = if nominal_next > Instant::now() {
            nominal_next
        } else {
            Instant::now() + Duration::from_millis(1)
        };
    }

    fn schedule_retry_frame_from_now(&mut self) {
        // Match the analyzer-verified Pico-PIO-USB behavior: a failed status
        // attempt is followed by a full millisecond of device recovery time,
        // then an SOF and the retry. Reusing the old absolute frame deadline
        // can make the interval slightly shorter than one millisecond.
        self.next_frame = Instant::now() + Duration::from_millis(1);
    }

    async fn transmit_due_sof_if_needed(&mut self) -> Result<(), PipeError> {
        // Attachment and speed are owned by PioHostState's debounced root-port
        // monitor. Sampling D+ and D- again here would make one un-debounced,
        // non-atomic GPIO observation override that authoritative state and
        // can suppress every SOF before a transfer even reaches the wire.
        if Instant::now() < self.bus_ready_at {
            return Ok(());
        }
        if Instant::now() < self.next_frame {
            return Ok(());
        }
        let scheduled_frame = self.next_frame;
        self.transmit_frame_marker().await?;
        self.schedule_frame_after(scheduled_frame);
        Ok(())
    }

    fn ensure_expected_speed_attached(&self) -> Result<(), PipeError> {
        match (self.profile.speed, root_line_state()) {
            (Speed::Full, LineState::JFullSpeed) | (Speed::Low, LineState::JLowSpeed) => Ok(()),
            (_, LineState::Se0) => Err(PipeError::Disconnected),
            _ => Err(PipeError::BadResponse),
        }
    }

    fn record_bad_response(
        &mut self,
        site: BadResponseSite,
        handshake_failure: Option<HandshakeFailure>,
    ) {
        if self.first_bad_response.is_none() {
            self.first_bad_response = Some(BadResponseDiagnostic {
                site,
                handshake_failure,
                handshake_observation: None,
            });
        }
    }

    fn record_handshake_bad_response(
        &mut self,
        site: BadResponseSite,
        observation: &RxObservation,
    ) {
        if self.first_bad_response.is_none() {
            self.first_bad_response = Some(BadResponseDiagnostic {
                site,
                handshake_failure: Some(classify_handshake_failure(observation)),
                handshake_observation: Some((*observation).into()),
            });
        }
    }

    fn record_bad_response_result<T>(
        &mut self,
        result: Result<T, PipeError>,
        site: BadResponseSite,
    ) -> Result<T, PipeError> {
        if matches!(result, Err(PipeError::BadResponse)) {
            self.record_bad_response(site, None);
        }
        result
    }

    fn record_handshake_bad_response_result<T>(
        &mut self,
        result: Result<T, PipeError>,
        site: BadResponseSite,
        observation: &RxObservation,
    ) -> Result<T, PipeError> {
        if matches!(result, Err(PipeError::BadResponse)) {
            self.record_handshake_bad_response(site, observation);
        }
        result
    }

    fn recover_tx_state(&mut self) {
        self.tx_sm.set_enable(false);
        self.tx_sm.clear_fifos();
        self.tx_irq_flags.clear_all(0b11);
        self.tx_sm.set_pins(Level::Low, &[&self.dp, &self.dm]);
        match self.profile.speed {
            Speed::Full => self.tx_sm.set_pins(Level::High, &[&self.dp]),
            Speed::Low => self.tx_sm.set_pins(Level::High, &[&self.dm]),
            Speed::High => {}
        }
        self.tx_sm
            .set_pin_dirs(Direction::In, &[&self.dp, &self.dm]);
        self.tx_sm.restart();
        self.tx_sm.clkdiv_restart();
        unsafe { self.tx_sm.exec_instr(self.tx_idle_instruction) };
        self.tx_sm.set_enable(true);
    }

    async fn wait_transaction_frame(&mut self, interval_ms: u8) -> Result<(), PipeError> {
        let frames = usize::from(interval_ms.max(1));
        for _ in 0..frames {
            // Pipe operations validate the debounced connection and its
            // generation both before and after locking this engine.
            self.advance_frame().await?;
        }
        Ok(())
    }

    async fn control_setup_stage(
        &mut self,
        target: PipeTarget,
        setup_payload: &[u8],
        deadline: Instant,
        site: BadResponseSite,
        observation: &mut RxObservation,
    ) -> Result<(), PipeError> {
        let mut bad_response_attempts = 0_u8;
        loop {
            *observation = RxObservation::default();
            let result = self
                .out_once(
                    target,
                    PID_SETUP,
                    PID_DATA0,
                    setup_payload,
                    observation,
                    true,
                )
                .await;

            match result {
                Ok(OutResponse::Ack) => return Ok(()),
                Ok(OutResponse::Nak | OutResponse::NoResponse) => {
                    if Instant::now() >= deadline {
                        return Err(PipeError::Timeout);
                    }
                }
                Err(PipeError::BadResponse) => {
                    bad_response_attempts += 1;
                    if bad_response_attempts >= CONTROL_SETUP_BAD_RESPONSE_RETRY_LIMIT
                        || Instant::now() >= deadline
                    {
                        self.record_handshake_bad_response(site, observation);
                        return Err(PipeError::BadResponse);
                    }
                }
                Err(error) => return Err(error),
            }
        }
    }

    async fn out_once(
        &mut self,
        target: PipeTarget,
        token_pid: u8,
        data_pid: u8,
        payload: &[u8],
        observation: &mut RxObservation,
        frame_scheduled: bool,
    ) -> Result<OutResponse, PipeError> {
        Timer::at(self.bus_ready_at).await;
        if frame_scheduled {
            self.wait_transaction_frame(1).await?;
        } else {
            self.transmit_due_sof_if_needed().await?;
        }
        let token = token_packet(
            token_pid,
            target.device_address,
            target.endpoint.addr.index() as u8,
        );
        let data_packet =
            RawDataPacket::new(data_pid, payload).map_err(|_| PipeError::BufferOverflow)?;

        // Synchronize with the edge detector's IRQ2 park point, but leave the
        // byte decoder disabled while the host owns the bus. The SRAM TX
        // handoff enables RX at the start of the DATA0 EOP immediately before
        // releasing the edge detector.
        park_receive(
            &mut self.edge_sm,
            &self.rx_irq_flags,
            self.edge_reset_instruction,
        )?;
        prepare_receive_disabled(
            &mut self.rx_sm,
            self.rx_reset_instruction,
            self.rx_clear_x_instruction,
        );
        let transmit_result = transmit_token_data_pair(
            &mut self.tx_sm,
            &self.tx_irq_flags,
            &self.rx_irq_flags,
            &token,
            data_packet.as_bytes(),
            self.profile.tx_eop_guard_cycles,
        );
        if let Err(error) = transmit_result {
            self.rx_sm.set_enable(false);
            self.edge_sm.set_enable(false);
            self.recover_tx_state();
            return Err(map_tx_error(error));
        }

        match receive_handshake(
            &mut self.rx_sm,
            &mut self.edge_sm,
            &self.rx_irq_flags,
            observation,
            self.profile.handshake_start_timeout_us,
            self.profile.handshake_packet_timeout_us,
        ) {
            ReceiveResult::Handshake(PID_ACK) => Ok(OutResponse::Ack),
            ReceiveResult::Handshake(PID_NAK) => Ok(OutResponse::Nak),
            ReceiveResult::Handshake(PID_STALL) => Err(PipeError::Stall),
            ReceiveResult::Handshake(_) | ReceiveResult::InvalidPacket => {
                Err(PipeError::BadResponse)
            }
            ReceiveResult::NoStart => Ok(OutResponse::NoResponse),
        }
    }

    async fn in_once(
        &mut self,
        target: PipeTarget,
        expected_pid: u8,
        max_payload_len: usize,
        payload: &mut [u8; 64],
        raw: &mut [u8; MAX_DECODED_BYTES],
        frame_scheduled: bool,
    ) -> Result<InReceiveResult, PipeError> {
        Timer::at(self.bus_ready_at).await;
        if frame_scheduled {
            self.wait_transaction_frame(1).await?;
        } else {
            self.transmit_due_sof_if_needed().await?;
        }
        let token = token_packet(
            PID_IN,
            target.device_address,
            target.endpoint.addr.index() as u8,
        );
        let receive_capacity = target.endpoint.max_packet_size as usize;
        let Some(receive_payload) = payload.get_mut(..receive_capacity) else {
            return Err(PipeError::BufferOverflow);
        };
        // See out_once: the edge IRQs must be cleared before RX starts waiting
        // on IRQ4, especially after an independently serviced idle SOF.
        park_receive(
            &mut self.edge_sm,
            &self.rx_irq_flags,
            self.edge_reset_instruction,
        )?;
        prepare_receive(
            &mut self.rx_sm,
            self.rx_reset_instruction,
            self.rx_clear_x_instruction,
        );
        // USB full-speed permits only a very short turnaround from the
        // device's EOP to our ACK. The Embassy time-driver interrupt may
        // otherwise preempt this polling path while a longer DATA packet is
        // being received, leaving an otherwise valid packet unacknowledged.
        // Keep only the analyzer-verified token/RX/ACK routine atomic; all
        // frame scheduling and retry waits remain interruptible.
        let result = cortex_m::interrupt::free(|_| {
            receive_data_packet(
                &mut self.tx_sm,
                &self.tx_irq_flags,
                &mut self.rx_sm,
                &self.rx_irq_flags,
                &token,
                receive_payload,
                raw,
                max_payload_len,
                expected_pid,
                self.profile.rx_ack_eop_guard_cycles,
            )
        });
        self.rx_sm.set_enable(false);
        self.edge_sm.set_enable(false);
        let response = match result {
            Ok(response) => response,
            Err(error) => {
                self.recover_tx_state();
                return Err(map_tx_error(error));
            }
        };
        Ok(response)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum OutResponse {
    Ack,
    Nak,
    NoResponse,
}

fn map_tx_error(_error: TxError) -> PipeError {
    PipeError::Timeout
}

fn timeout_deadline(timeout: CoreDuration) -> Instant {
    let micros = timeout.as_micros().min(u128::from(u64::MAX)) as u64;
    Instant::now() + Duration::from_micros(micros)
}

fn setup_data_packet(setup: &[u8; 8]) -> [u8; 12] {
    let crc = crc16_data(setup);
    [
        SYNC,
        PID_DATA0,
        setup[0],
        setup[1],
        setup[2],
        setup[3],
        setup[4],
        setup[5],
        setup[6],
        setup[7],
        crc as u8,
        (crc >> 8) as u8,
    ]
}

/// Sample GPIO16/GPIO17 without acquiring the packet-engine mutex.
///
/// Prefer [`PioHostState::root_line_state_if_not_resetting`] in a root-port
/// runner; a separate reset check plus this raw sample has a check/sample race.
/// After debouncing a real SE0 the runner should call
/// [`super::PioHostState::report_disconnected_if_not_resetting`] so the
/// host-driven SE0 during bus reset cannot become a false detach event.
pub fn root_line_state() -> LineState {
    let dp = pac::IO_BANK0.gpio(16).status().read().infrompad() as u32;
    let dm = pac::IO_BANK0.gpio(17).status().read().infrompad() as u32;
    LineState::from_pio_sample(dp | (dm << 1))
}

/// Backwards-compatible descriptive alias for [`root_line_state`].
pub fn sample_root_line_state() -> LineState {
    root_line_state()
}

impl<'d> PioHostState<Rp2040PioEngine<'d>> {
    /// Atomically suppress line sampling while the host drives bus reset.
    ///
    /// A root-port runner should prefer this over separately checking
    /// [`PioHostState::is_reset_in_progress`] and calling [`root_line_state`].
    /// `None` means it must skip its attach/detach debouncer update entirely.
    pub fn root_line_state_if_not_resetting(&self) -> Option<LineState> {
        self.shared.lock(|shared| {
            if shared.borrow().reset_in_progress {
                None
            } else {
                Some(root_line_state())
            }
        })
    }

    /// Clear the first-fault diagnostic immediately before a traced operation.
    pub async fn clear_bad_response_diagnostic(&self) {
        self.engine.lock().await.first_bad_response = None;
    }

    /// Take the first `BadResponse` wire stage recorded since the last clear.
    pub async fn take_bad_response_diagnostic(&self) -> Option<BadResponseDiagnostic> {
        self.engine.lock().await.first_bad_response.take()
    }
}

impl PioPacketEngine for Rp2040PioEngine<'_> {
    async fn bus_reset(&mut self, speed: Speed) -> Result<(), PipeError> {
        self.reset(speed).await?;
        self.ensure_expected_speed_attached()
    }

    async fn service_frame(&mut self) -> Result<(), PipeError> {
        self.transmit_due_sof_if_needed().await
    }

    async fn control_in(
        &mut self,
        target: PipeTarget,
        setup: &[u8; 8],
        buffer: &mut [u8],
        timeout: TimeoutConfig,
    ) -> Result<usize, PipeError> {
        if target.endpoint.ep_type != EndpointType::Control
            || target.endpoint.addr.index() != 0
            || setup[0] & 0x80 == 0
        {
            self.record_bad_response(BadResponseSite::ControlInContract, None);
            return Err(PipeError::BadResponse);
        }

        let requested = u16::from_le_bytes([setup[6], setup[7]]) as usize;
        if requested > buffer.len() {
            return Err(PipeError::BufferOverflow);
        }
        let selected_timeout = if requested == 0 {
            timeout.no_data_timeout
        } else {
            timeout.data_timeout
        };
        let deadline = timeout_deadline(selected_timeout);
        let setup_packet = setup_data_packet(setup);
        let mut observation = RxObservation::default();

        self.control_setup_stage(
            target,
            &setup_packet[2..10],
            deadline,
            BadResponseSite::ControlInSetup,
            &mut observation,
        )
        .await?;

        let mut received = 0_usize;
        let mut expected = DataToggle::Data1;
        let mut packet = [0_u8; 64];
        let mut raw = [0_u8; MAX_DECODED_BYTES];
        let max_packet_size = target.endpoint.max_packet_size as usize;

        while received < requested {
            let remaining = requested - received;
            let expected_len = remaining.min(max_packet_size);
            let packet_len = loop {
                let result = self
                    .in_once(
                        target,
                        expected.pid(),
                        expected_len,
                        &mut packet,
                        &mut raw,
                        true,
                    )
                    .await;
                match self.record_bad_response_result(result, BadResponseSite::ControlInData)? {
                    InReceiveResult::Data { len } => break len as usize,
                    InReceiveResult::UnexpectedToggle | InReceiveResult::Nak => {}
                    InReceiveResult::Stall => return Err(PipeError::Stall),
                    InReceiveResult::NoResponse => {}
                    InReceiveResult::InvalidPacket => {
                        self.record_bad_response(BadResponseSite::ControlInData, None);
                        return Err(PipeError::BadResponse);
                    }
                }
                if Instant::now() >= deadline {
                    return Err(PipeError::Timeout);
                }
            };

            buffer[received..received + packet_len].copy_from_slice(&packet[..packet_len]);
            received += packet_len;
            expected = expected.after_ack();
            if packet_len < max_packet_size {
                break;
            }
        }

        // Control-IN status is OUT/DATA1 with a zero-length payload.
        // Start it immediately after the final host ACK. The verified
        // Pico-PIO-USB sequence commonly receives one early NAK and retries
        // that status transaction in the following frame; delaying the first
        // attempt by a frame makes this device NAK every later retry.
        let mut wait_for_status_frame = false;
        loop {
            let result = self
                .out_once(
                    target,
                    PID_OUT,
                    PID_DATA1,
                    &[],
                    &mut observation,
                    wait_for_status_frame,
                )
                .await;
            wait_for_status_frame = true;
            match self.record_handshake_bad_response_result(
                result,
                BadResponseSite::ControlInStatus,
                &observation,
            )? {
                OutResponse::Ack => return Ok(received),
                OutResponse::Nak | OutResponse::NoResponse => {
                    if Instant::now() >= deadline {
                        return Err(PipeError::Timeout);
                    }
                    self.schedule_retry_frame_from_now();
                }
            }
        }
    }

    async fn control_out(
        &mut self,
        target: PipeTarget,
        setup: &[u8; 8],
        data: &[u8],
        timeout: TimeoutConfig,
    ) -> Result<(), PipeError> {
        if target.endpoint.ep_type != EndpointType::Control
            || target.endpoint.addr.index() != 0
            || setup[0] & 0x80 != 0
        {
            self.record_bad_response(BadResponseSite::ControlOutContract, None);
            return Err(PipeError::BadResponse);
        }
        let requested = u16::from_le_bytes([setup[6], setup[7]]) as usize;
        if requested != data.len() {
            return Err(PipeError::BufferOverflow);
        }
        let selected_timeout = if data.is_empty() {
            timeout.no_data_timeout
        } else {
            timeout.data_timeout
        };
        let deadline = timeout_deadline(selected_timeout);
        let setup_packet = setup_data_packet(setup);
        let mut observation = RxObservation::default();

        self.control_setup_stage(
            target,
            &setup_packet[2..10],
            deadline,
            BadResponseSite::ControlOutSetup,
            &mut observation,
        )
        .await?;

        let max_packet_size = target.endpoint.max_packet_size as usize;
        let mut toggle = DataToggle::Data1;
        for packet in data.chunks(max_packet_size) {
            loop {
                let result = self
                    .out_once(
                        target,
                        PID_OUT,
                        toggle.pid(),
                        packet,
                        &mut observation,
                        true,
                    )
                    .await;
                match self.record_handshake_bad_response_result(
                    result,
                    BadResponseSite::ControlOutData,
                    &observation,
                )? {
                    OutResponse::Ack => {
                        toggle = toggle.after_ack();
                        break;
                    }
                    OutResponse::Nak | OutResponse::NoResponse => {
                        if Instant::now() >= deadline {
                            return Err(PipeError::Timeout);
                        }
                    }
                }
            }
        }

        // Control-OUT status is IN/DATA1 and must be a ZLP.
        let mut packet = [0_u8; 64];
        let mut raw = [0_u8; MAX_DECODED_BYTES];
        let mut wait_for_status_frame = false;
        loop {
            let result = self
                .in_once(
                    target,
                    PID_DATA1,
                    0,
                    &mut packet,
                    &mut raw,
                    wait_for_status_frame,
                )
                .await;
            wait_for_status_frame = true;
            match self.record_bad_response_result(result, BadResponseSite::ControlOutStatus)? {
                InReceiveResult::Data { len: 0 } => return Ok(()),
                InReceiveResult::Data { .. } | InReceiveResult::InvalidPacket => {
                    self.record_bad_response(BadResponseSite::ControlOutStatus, None);
                    return Err(PipeError::BadResponse);
                }
                InReceiveResult::UnexpectedToggle | InReceiveResult::Nak => {}
                InReceiveResult::Stall => return Err(PipeError::Stall),
                InReceiveResult::NoResponse => {}
            }
            if Instant::now() >= deadline {
                return Err(PipeError::Timeout);
            }
            self.schedule_retry_frame_from_now();
        }
    }

    async fn request_in_once(
        &mut self,
        target: PipeTarget,
        data_toggle: &mut DataToggle,
        buffer: &mut [u8],
    ) -> Result<TransactionOutcome<usize>, PipeError> {
        if !matches!(
            target.endpoint.ep_type,
            EndpointType::Bulk | EndpointType::Interrupt
        ) || !target.endpoint.addr.is_in()
        {
            return Err(PipeError::BadResponse);
        }
        let mut packet = [0_u8; 64];
        let mut raw = [0_u8; MAX_DECODED_BYTES];
        match self
            .in_once(
                target,
                data_toggle.pid(),
                buffer.len().min(target.endpoint.max_packet_size as usize),
                &mut packet,
                &mut raw,
                false,
            )
            .await?
        {
            InReceiveResult::Data { len } => {
                let len = len as usize;
                *data_toggle = data_toggle.after_ack();
                if len > buffer.len() {
                    return Err(PipeError::BufferOverflow);
                }
                buffer[..len].copy_from_slice(&packet[..len]);
                Ok(TransactionOutcome::Complete(len))
            }
            InReceiveResult::UnexpectedToggle | InReceiveResult::Nak => Ok(TransactionOutcome::Nak),
            InReceiveResult::Stall => Err(PipeError::Stall),
            InReceiveResult::NoResponse => Ok(TransactionOutcome::NoResponse),
            InReceiveResult::InvalidPacket => Err(PipeError::BadResponse),
        }
    }

    async fn request_out_once(
        &mut self,
        target: PipeTarget,
        data_toggle: &mut DataToggle,
        packet: &[u8],
    ) -> Result<TransactionOutcome<()>, PipeError> {
        if !matches!(
            target.endpoint.ep_type,
            EndpointType::Bulk | EndpointType::Interrupt
        ) || !target.endpoint.addr.is_out()
        {
            return Err(PipeError::BadResponse);
        }

        let mut observation = RxObservation::default();
        match self
            .out_once(
                target,
                PID_OUT,
                data_toggle.pid(),
                packet,
                &mut observation,
                false,
            )
            .await?
        {
            OutResponse::Ack => {
                *data_toggle = data_toggle.after_ack();
                Ok(TransactionOutcome::Complete(()))
            }
            OutResponse::Nak => Ok(TransactionOutcome::Nak),
            OutResponse::NoResponse => Ok(TransactionOutcome::NoResponse),
        }
    }
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
    let dp_status = pac::IO_BANK0.gpio(16).status().read();
    let dm_status = pac::IO_BANK0.gpio(17).status().read();
    let dp_peripheral = dp_status.intoperi() as u8;
    let dm_peripheral = dm_status.intoperi() as u8;
    let dp_pad = dp_status.infrompad() as u8;
    let dm_pad = dm_status.infrompad() as u8;
    observation.input_snapshot =
        dp_peripheral | (dm_peripheral << 1) | (dp_pad << 4) | (dm_pad << 5);
    let dp_inover = pac::IO_BANK0.gpio(16).ctrl().read().inover().to_bits();
    let dm_inover = pac::IO_BANK0.gpio(17).ctrl().read().inover().to_bits();
    observation.input_override_snapshot = dp_inover | (dm_inover << 2);
}

fn classify_handshake_failure(observation: &RxObservation) -> HandshakeFailure {
    const RX_ERROR: u8 = 1 << 1;
    const SAW_EOP: u8 = 1 << 2;
    const SAW_START: u8 = 1 << 3;

    if observation.irq_flags & RX_ERROR != 0 {
        return HandshakeFailure::RxDecoderError;
    }
    if observation.irq_flags & SAW_EOP == 0 {
        if observation.len == 0 && observation.irq_flags & SAW_START != 0 {
            return HandshakeFailure::FalseStart;
        }
        return HandshakeFailure::IncompletePacket;
    }
    if observation.len != 2 {
        return HandshakeFailure::WrongLength;
    }
    if observation.bytes[0] != SYNC {
        return HandshakeFailure::InvalidSync;
    }

    let pid = observation.bytes[1];
    if (pid >> 4) != ((!pid) & 0x0f) {
        return HandshakeFailure::InvalidPidComplement;
    }
    if !matches!(pid, PID_ACK | PID_NAK | PID_STALL) {
        return HandshakeFailure::UnexpectedPid;
    }
    HandshakeFailure::Unknown
}

fn drain_rx_fifo(sm: &mut StateMachine<'_, PIO1, 0>, bytes: &mut [u8; 4], len: &mut usize) {
    while let Some(word) = sm.rx().try_pull() {
        if *len < bytes.len() {
            bytes[*len] = (word >> 24) as u8;
        }
        *len += 1;
    }
}

struct BusResetGuard<'a, 'd> {
    sm: &'a mut StateMachine<'d, PIO0, 0>,
    dp: &'a PioPin<'d, PIO0>,
    dm: &'a PioPin<'d, PIO0>,
    speed: Speed,
    bus_ready_at: &'a mut Instant,
    driving_reset: bool,
}

impl<'a, 'd> BusResetGuard<'a, 'd> {
    fn begin(
        sm: &'a mut StateMachine<'d, PIO0, 0>,
        dp: &'a PioPin<'d, PIO0>,
        dm: &'a PioPin<'d, PIO0>,
        speed: Speed,
        bus_ready_at: &'a mut Instant,
    ) -> Self {
        *bus_ready_at =
            Instant::now() + Duration::from_millis(USB_RESET_MS + USB_RESET_RECOVERY_MS);
        sm.set_enable(false);
        sm.clear_fifos();
        sm.set_pins(Level::Low, &[dp, dm]);
        sm.set_pin_dirs(Direction::Out, &[dp, dm]);
        Self {
            sm,
            dp,
            dm,
            speed,
            bus_ready_at,
            driving_reset: true,
        }
    }

    fn release_bus(&mut self) {
        if !self.driving_reset {
            return;
        }
        self.sm.set_pins(Level::Low, &[self.dp, self.dm]);
        match self.speed {
            Speed::Full => self.sm.set_pins(Level::High, &[self.dp]),
            Speed::Low => self.sm.set_pins(Level::High, &[self.dm]),
            Speed::High => {}
        }
        self.sm.set_pin_dirs(Direction::In, &[self.dp, self.dm]);
        // Preserve the TX program counter at IRQ WAIT 0 across reset, matching
        // Pico-PIO-USB. The first packet releases that wait by clearing IRQ 0.
        self.sm.set_enable(true);
        *self.bus_ready_at = Instant::now() + Duration::from_millis(USB_RESET_RECOVERY_MS);
        self.driving_reset = false;
    }

    fn recovery_deadline(&self) -> Instant {
        *self.bus_ready_at
    }
}

impl Drop for BusResetGuard<'_, '_> {
    fn drop(&mut self) {
        // Dropping a reset future during either timer must never leave both
        // bus pins driven low.
        self.release_bus();
    }
}

async fn reset_bus<'d>(
    sm: &mut StateMachine<'d, PIO0, 0>,
    dp: &PioPin<'d, PIO0>,
    dm: &PioPin<'d, PIO0>,
    speed: Speed,
    bus_ready_at: &mut Instant,
) {
    let mut reset = BusResetGuard::begin(sm, dp, dm, speed, bus_ready_at);
    Timer::after_millis(USB_RESET_MS).await;
    reset.release_bus();

    Timer::at(reset.recovery_deadline()).await;
}

#[allow(clippy::too_many_arguments)]
async fn transmit_packet(
    sm: &mut StateMachine<'_, PIO0, 0>,
    _tx_dma: &mut dma::Channel<'_>,
    tx_irq_flags: &IrqFlags<'_, PIO0>,
    rx_irq_flags: &IrqFlags<'_, PIO1>,
    packet: &[u8],
    arm_receiver_at_eop: bool,
    eop_guard_cycles: u32,
) -> Result<(), TxError> {
    // The current Pico-PIO-USB transmitter consumes ordinary USB packet
    // bytes. PIO performs LSB-first serialization, NRZI and bit stuffing.
    assert!(!packet.is_empty());
    assert!(packet.len() <= MAX_WIRE_PACKET_BYTES);

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
    cortex_m::asm::delay(eop_guard_cycles);

    Ok(())
}

fn transmit_low_speed_keep_alive(
    sm: &mut StateMachine<'_, PIO0, 0>,
    tx_irq_flags: &IrqFlags<'_, PIO0>,
    eop_guard_cycles: u32,
) -> Result<(), TxError> {
    // Releasing IRQ0 with an empty TX FIFO takes the low-speed PIO program
    // directly through its EOP path: two low-speed bit times of SE0, one J,
    // then pin release. No SYNC/PID/CRC bytes are emitted.
    sm.clear_fifos();
    tx_irq_flags.clear_all(0b11);
    wait_for_tx_irq(tx_irq_flags, 1)?;
    tx_irq_flags.clear(1);
    inline_delay_cycles(eop_guard_cycles);
    wait_for_tx_irq(tx_irq_flags, 0)?;
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
    eop_guard_cycles: u32,
) -> Result<(), TxError> {
    assert!(!token.is_empty() && token.len() <= 8);
    assert!(!data.is_empty() && data.len() <= MAX_WIRE_PACKET_BYTES);

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

    // PIO1 SM0 is owned by this fixed backend and was reset with its enable
    // bit clear. Enable it at the first DATA EOP instruction, then release the
    // already parked edge detector. Both writes occur before the transmitter
    // releases the pins and before the device's earliest legal response.
    enable_rx_sm0_at_eop();
    rx_irq_flags.clear_all(RX_IRQ_MASK);
    inline_delay_cycles(eop_guard_cycles);

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
    ack_eop_guard_cycles: u32,
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
        inline_delay_cycles(ack_eop_guard_cycles);
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
    prepare_receive_disabled(sm, reset_instruction, clear_x_instruction);
    sm.set_enable(true);
}

fn prepare_receive_disabled(
    sm: &mut StateMachine<'_, PIO1, 0>,
    reset_instruction: u16,
    clear_x_instruction: u16,
) {
    sm.set_enable(false);

    // GPIO CTRL can be rewritten by pin-function setup outside the PIO
    // blocks. Reassert the polarity immediately before every transaction,
    // while the byte decoder is disabled and the edge detector is parked on
    // IRQ2, so the receiver always sees Pico-PIO-USB's inverted D+/D- image.
    configure_rx_input_inversion();

    sm.clear_fifos();
    sm.restart();
    unsafe {
        sm.exec_instr(reset_instruction);
        sm.exec_instr(clear_x_instruction);
    }
}

fn forget_unused_state_machine<'d, PIO: PioInstance, const SM: usize>(
    mut sm: StateMachine<'d, PIO, SM>,
) {
    sm.set_enable(false);
    core::mem::forget(sm);
}

#[inline(always)]
fn configure_rx_input_inversion() {
    pac::IO_BANK0
        .gpio(16)
        .ctrl()
        .modify(|w| w.set_inover(pac::io::vals::Inover::INVERT));
    pac::IO_BANK0
        .gpio(17)
        .ctrl()
        .modify(|w| w.set_inover(pac::io::vals::Inover::INVERT));
}

#[inline(always)]
fn enable_rx_sm0_at_eop() {
    // RP2040 peripheral registers expose an atomic SET alias 0x2000 bytes
    // above their normal address. Embassy's equivalent helper is private, so
    // use the alias directly to avoid a read/modify/write and an out-of-line
    // call inside this SRAM-resident timing path.
    const ATOMIC_SET_OFFSET: usize = 0x2000;
    unsafe {
        let ctrl_set = (pac::PIO1.ctrl().as_ptr() as *mut u8).add(ATOMIC_SET_OFFSET) as *mut u32;
        ctrl_set.write_volatile(1_u32 << 0);
    }
}

fn park_receive(
    edge_sm: &mut StateMachine<'_, PIO1, 1>,
    irq_flags: &IrqFlags<'_, PIO1>,
    edge_reset_instruction: u16,
) -> Result<(), PipeError> {
    edge_sm.set_enable(false);
    edge_sm.clear_fifos();
    irq_flags.clear_all(RX_IRQ_MASK);
    edge_sm.restart();
    unsafe { edge_sm.exec_instr(edge_reset_instruction) };
    edge_sm.set_enable(true);

    // `irq wait 2` first raises IRQ2 and only then stalls. Do not let TX
    // begin until that state is observable; otherwise clearing IRQ2 at the
    // host data EOP can race ahead of the detector reaching its park point.
    let mut budget = RX_PACKET_POLL_BUDGET;
    while !irq_flags.check(2) {
        if budget == 0 {
            edge_sm.set_enable(false);
            return Err(PipeError::Timeout);
        }
        budget -= 1;
    }
    Ok(())
}

fn receive_handshake(
    sm: &mut StateMachine<'_, PIO1, 0>,
    edge_sm: &mut StateMachine<'_, PIO1, 1>,
    irq_flags: &IrqFlags<'_, PIO1>,
    observation: &mut RxObservation,
    start_timeout_us: u64,
    packet_timeout_us: u64,
) -> ReceiveResult {
    *observation = RxObservation::default();
    let mut bytes = [0_u8; 4];
    let mut len = 0;

    let start_deadline = Instant::now() + Duration::from_micros(start_timeout_us);
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

    let packet_deadline = Instant::now() + Duration::from_micros(packet_timeout_us);

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
