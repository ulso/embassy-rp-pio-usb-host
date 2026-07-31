//! Embassy USB-host adapter for a serialised RP2040 PIO packet engine.
//!
//! This module establishes the ownership, lifetime and concurrency boundary
//! needed by [`embassy_usb_driver::host`]. The target-gated `rp2040` module
//! contains the timing-critical PIO implementation.
//!
//! A packet engine owns its state machines, pins and DMA channel. The
//! application places [`PioHostState`] in storage that lives for `'d`;
//! controllers, allocators and pipes then share that state by reference. This
//! matches the Embassy requirement that allocator storage must not be borrowed
//! from the controller value itself.

use core::cell::RefCell;
use core::marker::PhantomData;

use embassy_sync::blocking_mutex::Mutex as BlockingMutex;
#[cfg(target_os = "none")]
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex as HostRawMutex;
#[cfg(not(target_os = "none"))]
use embassy_sync::blocking_mutex::raw::NoopRawMutex as HostRawMutex;
use embassy_sync::mutex::Mutex as AsyncMutex;
use embassy_sync::signal::Signal;

use crate::host::{
    DeviceEvent, Direction as UsbDirection, EndpointInfo, EndpointType, HostError, PipeError,
    Speed, SplitInfo, TimeoutConfig, UsbHostAllocator, UsbHostController, UsbPipe, pipe,
};
use crate::usb::DataToggle;

#[cfg(target_os = "none")]
pub mod rp2040;

const NON_RESPONSE_RETRY_LIMIT: u16 = 128;

async fn yield_to_other_pipes() {
    let mut yielded = false;
    core::future::poll_fn(|cx| {
        if yielded {
            core::task::Poll::Ready(())
        } else {
            yielded = true;
            cx.waker().wake_by_ref();
            core::task::Poll::Pending
        }
    })
    .await;
}

#[cfg(any(target_os = "none", test))]
fn next_transaction_delay_ms(target: PipeTarget, retry: bool) -> u64 {
    if target.endpoint.ep_type == EndpointType::Interrupt {
        u64::from(target.endpoint.interval_ms.max(1))
    } else if retry {
        1
    } else {
        0
    }
}

/// Immutable metadata carried by one logical host pipe.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct PipeTarget {
    /// Device address used in USB tokens.
    pub device_address: u8,
    /// Endpoint address, type, packet size and polling interval.
    pub endpoint: EndpointInfo,
    /// Hub routing metadata, when supported by the packet engine.
    pub split: Option<SplitInfo>,
}

/// Result of one non-control wire transaction.
///
/// NAK and missing-response outcomes are returned to the adapter so it can
/// drop the engine mutex between retries and fairly multiplex concurrent
/// logical pipes onto one physical PIO engine.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TransactionOutcome<T> {
    /// The transaction was acknowledged and produced the enclosed result.
    Complete(T),
    /// The device returned NAK.
    Nak,
    /// No valid device response began in the response window.
    NoResponse,
}

/// Maximum number of leading payload bytes retained for an IN-toggle diagnostic.
pub const IN_DIAGNOSTIC_PAYLOAD_PREFIX_CAPACITY: usize = 8;

/// Most recently observed CRC-valid IN packet with the opposite DATA toggle.
///
/// The RP2040 backend records this only after the packet has been ACKed and the
/// timing-critical receive routine has returned.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct UnexpectedToggleDiagnostic {
    /// DATA0 or DATA1 PID expected by the host pipe.
    pub expected_pid: u8,
    /// DATA0 or DATA1 PID decoded from the device packet.
    pub actual_pid: u8,
    /// Complete payload length of the duplicate packet.
    pub payload_len: u8,
    /// Number of valid bytes in [`Self::payload_prefix`].
    pub payload_prefix_len: u8,
    /// Leading duplicate-packet payload bytes, zero-filled after the valid prefix.
    pub payload_prefix: [u8; IN_DIAGNOSTIC_PAYLOAD_PREFIX_CAPACITY],
}

/// Cumulative non-control IN diagnostics since the last take or bus reset.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct InTransactionDiagnostics {
    /// Number of completed non-control IN wire observations.
    pub attempt_count: u32,
    /// Number of expected DATA packets accepted by their logical pipe.
    pub accepted_data_count: u32,
    /// Number of CRC-valid wrong-toggle packets that were ACKed and discarded.
    pub unexpected_toggle_count: u32,
    /// Number of NAK handshakes returned by the device.
    pub nak_count: u32,
    /// Number of attempts in which no valid response began.
    pub no_response_count: u32,
    /// Number of invalid packets or STALL handshakes.
    pub invalid_or_stall_count: u32,
    /// Number of expected zero-length DATA packets that were ACKed.
    ///
    /// ZLPs advance the endpoint toggle but are otherwise invisible to byte
    /// counters, so retaining this count is important when reconstructing the
    /// expected DATA0/DATA1 sequence.
    pub accepted_zlp_count: u32,
    /// Most recently observed wrong-toggle packet.
    pub latest_unexpected_toggle: Option<UnexpectedToggleDiagnostic>,
}

/// Cumulative progress markers for non-control IN pipe requests.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct InPipeProgressDiagnostics {
    /// Requests that entered the pipe retry loop.
    pub starts: u32,
    /// Requests whose per-pipe transaction deadline was already satisfied.
    pub deadline_ready: u32,
    /// Requests that acquired the shared physical packet engine.
    pub engine_acquired: u32,
    /// Requests whose engine transaction returned.
    pub engine_returned: u32,
    /// Requests whose post-transaction frame service returned.
    pub service_returned: u32,
}

#[cfg(target_os = "none")]
static IN_PIPE_PROGRESS: cortex_m::interrupt::Mutex<RefCell<InPipeProgressDiagnostics>> =
    cortex_m::interrupt::Mutex::new(RefCell::new(InPipeProgressDiagnostics {
        starts: 0,
        deadline_ready: 0,
        engine_acquired: 0,
        engine_returned: 0,
        service_returned: 0,
    }));

#[cfg(target_os = "none")]
enum InPipeProgressEvent {
    Start,
    DeadlineReady,
    EngineAcquired,
    EngineReturned,
    ServiceReturned,
}

#[cfg(target_os = "none")]
fn record_in_pipe_progress(event: InPipeProgressEvent) {
    cortex_m::interrupt::free(|critical_section| {
        let mut progress = IN_PIPE_PROGRESS.borrow(critical_section).borrow_mut();
        let counter = match event {
            InPipeProgressEvent::Start => &mut progress.starts,
            InPipeProgressEvent::DeadlineReady => &mut progress.deadline_ready,
            InPipeProgressEvent::EngineAcquired => &mut progress.engine_acquired,
            InPipeProgressEvent::EngineReturned => &mut progress.engine_returned,
            InPipeProgressEvent::ServiceReturned => &mut progress.service_returned,
        };
        *counter = counter.wrapping_add(1);
    });
}

/// Return non-blocking progress counters for the RP2040 IN pipe path.
#[cfg(target_os = "none")]
pub fn snapshot_in_pipe_progress_diagnostics() -> InPipeProgressDiagnostics {
    cortex_m::interrupt::free(|critical_section| {
        *IN_PIPE_PROGRESS.borrow(critical_section).borrow()
    })
}

#[cfg(target_os = "none")]
fn reset_in_pipe_progress_diagnostics() {
    cortex_m::interrupt::free(|critical_section| {
        *IN_PIPE_PROGRESS.borrow(critical_section).borrow_mut() =
            InPipeProgressDiagnostics::default();
    });
}

#[cfg(any(test, target_os = "none"))]
impl InTransactionDiagnostics {
    fn record_attempt(&mut self) {
        self.attempt_count = self.attempt_count.saturating_add(1);
    }

    fn record_accepted_data(&mut self) {
        self.accepted_data_count = self.accepted_data_count.saturating_add(1);
    }

    fn record_nak(&mut self) {
        self.nak_count = self.nak_count.saturating_add(1);
    }

    fn record_no_response(&mut self) {
        self.no_response_count = self.no_response_count.saturating_add(1);
    }

    fn record_invalid_or_stall(&mut self) {
        self.invalid_or_stall_count = self.invalid_or_stall_count.saturating_add(1);
    }

    fn record_accepted_zlp(&mut self) {
        self.accepted_zlp_count = self.accepted_zlp_count.saturating_add(1);
    }

    fn record_unexpected_toggle(&mut self, expected_pid: u8, actual_pid: u8, payload: &[u8]) {
        let prefix_len = payload.len().min(IN_DIAGNOSTIC_PAYLOAD_PREFIX_CAPACITY);
        let mut payload_prefix = [0; IN_DIAGNOSTIC_PAYLOAD_PREFIX_CAPACITY];
        payload_prefix[..prefix_len].copy_from_slice(&payload[..prefix_len]);
        self.unexpected_toggle_count = self.unexpected_toggle_count.saturating_add(1);
        self.latest_unexpected_toggle = Some(UnexpectedToggleDiagnostic {
            expected_pid,
            actual_pid,
            payload_len: payload.len() as u8,
            payload_prefix_len: prefix_len as u8,
            payload_prefix,
        });
    }

    fn take(&mut self) -> Self {
        core::mem::take(self)
    }
}

/// The packet-transfer contract implemented by the timing-critical PIO layer.
///
/// The adapter serialises individual wire transactions through one async
/// mutex, so an RP2040 implementation may own one physical TX/RX engine while
/// exposing multiple logical Embassy pipes. Non-control NAK retry, interrupt
/// `bInterval` scheduling and multi-packet OUT packetisation happen in the
/// adapter with the mutex released.
///
/// # Required transfer semantics
///
/// Implementations and the adapter divide the [`UsbPipe`] contract as follows:
///
/// - control methods execute and retry the complete transfer while holding the
///   engine because endpoint zero is bus-exclusive during enumeration;
/// - non-control `*_once` methods execute exactly one immediate transaction
///   and return NAK/no-response to the adapter;
/// - the adapter retries bulk NAKs indefinitely, schedules interrupt polls at
///   `bInterval`, packetises OUT buffers and adds a requested terminating ZLP;
/// - the engine advances `data_toggle` only after ACK;
/// - leave the engine and toggle in a reusable state if an async operation is
///   cancelled.
///
/// For the RP2040 PIO implementation, cancellation is easiest to guarantee by
/// awaiting only between complete wire transactions. The existing
/// SRAM-resident token/data/handshake paths are synchronous and should remain
/// so.
#[allow(async_fn_in_trait)]
pub trait PioPacketEngine {
    /// Perform a root-port bus reset at `speed` and verify that the device
    /// settled back to the corresponding attached line state after reset
    /// recovery.
    async fn bus_reset(&mut self, speed: Speed) -> Result<(), PipeError>;

    /// Perform due idle-frame maintenance for the active bus speed.
    ///
    /// This must never wait for a future frame deadline: transmit an overdue
    /// full-speed SOF or low-speed keep-alive immediately, or return `Ok(())`
    /// when none is due. The adapter calls it both from the idle runner and
    /// once after every wire operation while it still owns the engine, closing
    /// races at frame boundaries.
    async fn service_frame(&mut self) -> Result<(), PipeError>;

    /// Execute one complete control-IN transfer.
    async fn control_in(
        &mut self,
        target: PipeTarget,
        setup: &[u8; 8],
        buffer: &mut [u8],
        timeout: TimeoutConfig,
    ) -> Result<usize, PipeError>;

    /// Execute one complete control-OUT transfer.
    async fn control_out(
        &mut self,
        target: PipeTarget,
        setup: &[u8; 8],
        data: &[u8],
        timeout: TimeoutConfig,
    ) -> Result<(), PipeError>;

    /// Attempt one non-control IN transaction.
    async fn request_in_once(
        &mut self,
        target: PipeTarget,
        data_toggle: &mut DataToggle,
        buffer: &mut [u8],
    ) -> Result<TransactionOutcome<usize>, PipeError>;

    /// Attempt one non-control OUT transaction containing one packet.
    async fn request_out_once(
        &mut self,
        target: PipeTarget,
        data_toggle: &mut DataToggle,
        packet: &[u8],
    ) -> Result<TransactionOutcome<()>, PipeError>;
}

#[derive(Clone, Copy)]
struct PortState {
    connected: bool,
    speed: Option<Speed>,
    connection_generation: u32,
    reset_generation: u32,
}

impl PortState {
    const fn new() -> Self {
        Self {
            connected: false,
            speed: None,
            connection_generation: 0,
            reset_generation: 0,
        }
    }

    fn after_connection(self, speed: Speed) -> Self {
        Self {
            connected: true,
            speed: Some(speed),
            connection_generation: self.connection_generation.wrapping_add(1),
            reset_generation: self.reset_generation.wrapping_add(1),
        }
    }

    fn after_disconnection(self) -> Self {
        Self {
            connected: false,
            speed: None,
            connection_generation: self.connection_generation.wrapping_add(1),
            reset_generation: self.reset_generation,
        }
    }

    fn after_bus_reset(self) -> Self {
        Self {
            reset_generation: self.reset_generation.wrapping_add(1),
            ..self
        }
    }
}

#[derive(Clone, Copy)]
struct PendingEvents {
    destructive: Option<DeviceEvent>,
    connected: Option<Speed>,
}

impl PendingEvents {
    const fn new() -> Self {
        Self {
            destructive: None,
            connected: None,
        }
    }

    fn push(&mut self, event: DeviceEvent) {
        match event {
            DeviceEvent::Connected(speed) => self.connected = Some(speed),
            DeviceEvent::Disconnected => {
                if self.destructive != Some(DeviceEvent::Overcurrent) {
                    self.destructive = Some(DeviceEvent::Disconnected);
                }
                self.connected = None;
            }
            DeviceEvent::Overcurrent => {
                self.destructive = Some(DeviceEvent::Overcurrent);
                self.connected = None;
            }
            _ => {
                if self.destructive != Some(DeviceEvent::Overcurrent) {
                    self.destructive = Some(event);
                }
                self.connected = None;
            }
        }
    }

    fn take(&mut self) -> Option<DeviceEvent> {
        self.destructive
            .take()
            .or_else(|| self.connected.take().map(DeviceEvent::Connected))
    }
}

struct SharedState {
    port: PortState,
    pending_events: PendingEvents,
    controller_taken: bool,
    reset_in_progress: bool,
}

impl SharedState {
    const fn new() -> Self {
        Self {
            port: PortState::new(),
            pending_events: PendingEvents::new(),
            controller_taken: false,
            reset_in_progress: false,
        }
    }
}

struct ResetActivity<'a, E> {
    state: &'a PioHostState<E>,
}

impl<E> Drop for ResetActivity<'_, E> {
    fn drop(&mut self) {
        self.state
            .shared
            .lock(|shared| shared.borrow_mut().reset_in_progress = false);
    }
}

/// User-owned storage shared by the controller, allocator and all pipes.
///
/// Put this value in storage that outlives every host handle, normally a
/// `StaticCell` in embedded firmware. For RP2040 firmware, `E` is normally
/// `rp2040::Rp2040PioEngine`.
pub struct PioHostState<E> {
    engine: AsyncMutex<HostRawMutex, E>,
    shared: BlockingMutex<HostRawMutex, RefCell<SharedState>>,
    event_wake: Signal<HostRawMutex, ()>,
}

impl<E> PioHostState<E> {
    /// Create disconnected host state around a packet engine.
    pub const fn new(engine: E) -> Self {
        Self {
            engine: AsyncMutex::new(engine),
            shared: BlockingMutex::new(RefCell::new(SharedState::new())),
            event_wake: Signal::new(),
        }
    }

    /// Acquire the single controller handle consumed by `embassy-usb-host`.
    ///
    /// Allocators are cloneable, but event consumption and bus reset require
    /// exactly one controller. A second call returns an error.
    pub fn controller(&self) -> Result<PioHostController<'_, E>, HostError> {
        let acquired = self.shared.lock(|shared| {
            let mut shared = shared.borrow_mut();
            if shared.controller_taken {
                false
            } else {
                shared.controller_taken = true;
                true
            }
        });
        if acquired {
            Ok(PioHostController { state: self })
        } else {
            Err(HostError::Other(
                "the RP2040 PIO backend supports exactly one host controller",
            ))
        }
    }

    /// Reset an attached device, publish the settled speed, and make the port
    /// available to allocators.
    ///
    /// A line-monitor runner calls this only after a debounced attachment.
    /// Performing reset before signaling preserves the
    /// [`UsbHostController::wait_for_device_event`] contract.
    pub async fn reset_and_report_connected(&self, speed: Speed) -> Result<(), PipeError>
    where
        E: PioPacketEngine,
    {
        let connection_generation = self.port_state().connection_generation;
        let mut engine = self.engine.lock().await;
        self.mark_bus_reset();
        let reset_activity = self.begin_reset_activity();
        if let Err(error) = engine.bus_reset(speed).await {
            drop(reset_activity);
            if error == PipeError::Disconnected {
                self.report_disconnected();
            }
            drop(engine);
            return Err(error);
        }
        let connected = self.shared.lock(|shared| {
            let mut shared = shared.borrow_mut();
            if shared.port.connection_generation != connection_generation {
                return false;
            }
            shared.port = shared.port.after_connection(speed);
            shared.pending_events.push(DeviceEvent::Connected(speed));
            true
        });
        drop(reset_activity);
        drop(engine);
        if connected {
            self.event_wake.signal(());
            Ok(())
        } else {
            Err(PipeError::Disconnected)
        }
    }

    /// Service one idle USB frame.
    ///
    /// Firmware should call this from a 1 ms runner while a low- or full-speed
    /// device is connected. If a pipe operation owns the engine, idle service
    /// is skipped; concrete transfer methods continue speed-appropriate frame
    /// maintenance while retrying NAKs.
    pub async fn service_frame(&self) -> Result<(), PipeError>
    where
        E: PioPacketEngine,
    {
        if !matches!(self.port_state().speed, Some(Speed::Low | Speed::Full)) {
            return Err(PipeError::Disconnected);
        }
        let Ok(mut engine) = self.engine.try_lock() else {
            return Ok(());
        };
        if !matches!(self.port_state().speed, Some(Speed::Low | Speed::Full)) {
            return Err(PipeError::Disconnected);
        }
        engine.service_frame().await
    }

    /// Report that the root device has detached.
    ///
    /// Every pipe from the previous connection is invalidated, including
    /// address-zero pipes.
    pub fn report_disconnected(&self) {
        self.shared.lock(|shared| {
            let mut shared = shared.borrow_mut();
            shared.port = shared.port.after_disconnection();
            shared.pending_events.push(DeviceEvent::Disconnected);
        });
        self.event_wake.signal(());
    }

    /// Report a sampled detach unless the host is currently driving reset.
    ///
    /// Root-port monitors should sample the pins first, then call this method
    /// for a debounced SE0. The reset check and state/event update are atomic,
    /// avoiding a race with reset beginning between a separate flag check and
    /// [`Self::report_disconnected`].
    pub fn report_disconnected_if_not_resetting(&self) -> bool {
        let reported = self.shared.lock(|shared| {
            let mut shared = shared.borrow_mut();
            if shared.reset_in_progress {
                return false;
            }
            shared.port = shared.port.after_disconnection();
            shared.pending_events.push(DeviceEvent::Disconnected);
            true
        });
        if reported {
            self.event_wake.signal(());
        }
        reported
    }

    /// Report an overcurrent condition on the root port.
    pub fn report_overcurrent(&self) {
        self.shared.lock(|shared| {
            let mut shared = shared.borrow_mut();
            shared.port = shared.port.after_disconnection();
            shared.pending_events.push(DeviceEvent::Overcurrent);
        });
        self.event_wake.signal(());
    }

    /// Return whether the host is intentionally driving root-port reset.
    ///
    /// A lock-free GPIO line monitor must skip its attach/detach detector
    /// update entirely while this is true because bus reset itself is a
    /// host-driven SE0. Merely suppressing the resulting event would still
    /// corrupt the detector's stable state.
    pub fn is_reset_in_progress(&self) -> bool {
        self.shared.lock(|shared| shared.borrow().reset_in_progress)
    }

    fn port_state(&self) -> PortState {
        self.shared.lock(|shared| shared.borrow().port)
    }

    fn take_pending_event(&self) -> Option<DeviceEvent> {
        self.shared
            .lock(|shared| shared.borrow_mut().pending_events.take())
    }

    fn mark_bus_reset(&self) {
        #[cfg(target_os = "none")]
        reset_in_pipe_progress_diagnostics();
        self.shared.lock(|shared| {
            let mut shared = shared.borrow_mut();
            shared.port = shared.port.after_bus_reset();
        });
    }

    fn begin_reset_activity(&self) -> ResetActivity<'_, E> {
        self.shared
            .lock(|shared| shared.borrow_mut().reset_in_progress = true);
        ResetActivity { state: self }
    }
}

/// Bus-level handle implementing the official Embassy host-controller trait.
pub struct PioHostController<'d, E> {
    state: &'d PioHostState<E>,
}

impl<'d, E> UsbHostController<'d> for PioHostController<'d, E>
where
    E: PioPacketEngine + 'd,
{
    type Allocator = PioHostAllocator<'d, E>;

    fn allocator(&self) -> Self::Allocator {
        PioHostAllocator { state: self.state }
    }

    async fn wait_for_device_event(&mut self) -> DeviceEvent {
        loop {
            if let Some(event) = self.state.take_pending_event() {
                return event;
            }
            self.state.event_wake.wait().await;
        }
    }

    async fn bus_reset(&mut self) {
        let mut engine = self.state.engine.lock().await;
        let Some(speed) = self.state.port_state().speed else {
            return;
        };
        self.state.mark_bus_reset();
        let reset_activity = self.state.begin_reset_activity();
        let result = engine.bus_reset(speed).await;
        drop(reset_activity);
        if result == Err(PipeError::Disconnected) {
            self.state.report_disconnected();
        }
        drop(engine);
    }
}

/// Cloneable logical-pipe allocator sharing a user-owned [`PioHostState`].
pub struct PioHostAllocator<'d, E> {
    state: &'d PioHostState<E>,
}

impl<E> Clone for PioHostAllocator<'_, E> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<E> Copy for PioHostAllocator<'_, E> {}

impl<'d, E> UsbHostAllocator<'d> for PioHostAllocator<'d, E>
where
    E: PioPacketEngine + 'd,
{
    type Pipe<T: pipe::Type, D: pipe::Direction> = PioHostPipe<'d, E, T, D>;

    fn alloc_pipe<T: pipe::Type, D: pipe::Direction>(
        &self,
        device_address: u8,
        endpoint: &EndpointInfo,
        split: Option<SplitInfo>,
    ) -> Result<Self::Pipe<T, D>, HostError> {
        let port = self.state.port_state();
        if !port.connected {
            return Err(HostError::NoSuchDevice);
        }
        let speed = port.speed.ok_or(HostError::NoSuchDevice)?;
        validate_pipe::<T, D>(device_address, endpoint, split, speed)?;

        Ok(PioHostPipe {
            state: self.state,
            target: PipeTarget {
                device_address,
                endpoint: *endpoint,
                split,
            },
            data_toggle: DataToggle::Data0,
            timeout: TimeoutConfig::default(),
            connection_generation: port.connection_generation,
            reset_generation: port.reset_generation,
            #[cfg(target_os = "none")]
            next_transaction: embassy_time::Instant::now(),
            _marker: PhantomData,
        })
    }
}

fn validate_pipe<T: pipe::Type, D: pipe::Direction>(
    device_address: u8,
    endpoint: &EndpointInfo,
    split: Option<SplitInfo>,
    speed: Speed,
) -> Result<(), HostError> {
    if device_address > 127 {
        return Err(HostError::NoSuchDevice);
    }
    if split.is_some() {
        return Err(HostError::Other(
            "split transactions are not implemented by the RP2040 PIO backend",
        ));
    }
    if endpoint.ep_type != T::ep_type() {
        return Err(HostError::Other(
            "pipe type does not match endpoint transfer type",
        ));
    }
    if endpoint.addr.index() > 15 {
        return Err(HostError::InvalidDescriptor);
    }

    match speed {
        Speed::Low => validate_low_speed_pipe::<D>(endpoint),
        Speed::Full => validate_full_speed_pipe::<D>(endpoint),
        Speed::High => Err(HostError::Other(
            "high-speed devices are not supported by the RP2040 PIO backend",
        )),
    }
}

fn validate_low_speed_pipe<D: pipe::Direction>(endpoint: &EndpointInfo) -> Result<(), HostError> {
    match endpoint.ep_type {
        EndpointType::Control => {
            if endpoint.addr.index() != 0 || !D::is_in() || !D::is_out() {
                return Err(HostError::Other(
                    "a control pipe must be bidirectional endpoint zero",
                ));
            }
            if endpoint.max_packet_size != 8 {
                return Err(HostError::InvalidDescriptor);
            }
        }
        EndpointType::Interrupt => {
            validate_non_control_direction::<D>(endpoint)?;
            if endpoint.addr.index() == 0
                || endpoint.max_packet_size == 0
                || endpoint.max_packet_size > 8
                || endpoint.interval_ms < 10
            {
                return Err(HostError::InvalidDescriptor);
            }
        }
        EndpointType::Bulk => {
            return Err(HostError::Other(
                "bulk transfers are not valid for low-speed USB devices",
            ));
        }
        EndpointType::Isochronous => {
            return Err(HostError::Other(
                "isochronous transfers are not implemented by the RP2040 PIO backend",
            ));
        }
    }

    Ok(())
}

fn validate_full_speed_pipe<D: pipe::Direction>(endpoint: &EndpointInfo) -> Result<(), HostError> {
    match endpoint.ep_type {
        EndpointType::Control => {
            if endpoint.addr.index() != 0 || !D::is_in() || !D::is_out() {
                return Err(HostError::Other(
                    "a control pipe must be bidirectional endpoint zero",
                ));
            }
            if !matches!(endpoint.max_packet_size, 8 | 16 | 32 | 64) {
                return Err(HostError::InvalidDescriptor);
            }
        }
        EndpointType::Bulk => {
            validate_non_control_direction::<D>(endpoint)?;
            if endpoint.addr.index() == 0 || !matches!(endpoint.max_packet_size, 8 | 16 | 32 | 64) {
                return Err(HostError::InvalidDescriptor);
            }
        }
        EndpointType::Interrupt => {
            validate_non_control_direction::<D>(endpoint)?;
            if endpoint.addr.index() == 0
                || endpoint.max_packet_size == 0
                || endpoint.max_packet_size > 64
                || endpoint.interval_ms == 0
            {
                return Err(HostError::InvalidDescriptor);
            }
        }
        EndpointType::Isochronous => {
            return Err(HostError::Other(
                "isochronous transfers are not implemented by the RP2040 PIO backend",
            ));
        }
    }

    Ok(())
}

fn validate_non_control_direction<D: pipe::Direction>(
    endpoint: &EndpointInfo,
) -> Result<(), HostError> {
    let direction_matches = match endpoint.addr.direction() {
        UsbDirection::In => D::is_in() && !D::is_out(),
        UsbDirection::Out => D::is_out() && !D::is_in(),
    };
    if direction_matches {
        Ok(())
    } else {
        Err(HostError::Other(
            "pipe direction does not match endpoint direction",
        ))
    }
}

/// One logical Embassy pipe multiplexed onto the shared packet engine.
pub struct PioHostPipe<'d, E, T: pipe::Type, D: pipe::Direction> {
    state: &'d PioHostState<E>,
    target: PipeTarget,
    data_toggle: DataToggle,
    timeout: TimeoutConfig,
    connection_generation: u32,
    reset_generation: u32,
    #[cfg(target_os = "none")]
    next_transaction: embassy_time::Instant,
    _marker: PhantomData<(T, D)>,
}

impl<E, T: pipe::Type, D: pipe::Direction> PioHostPipe<'_, E, T, D> {
    /// Return the immutable routing metadata for this pipe.
    pub const fn target(&self) -> PipeTarget {
        self.target
    }

    fn check_valid(&self) -> Result<(), PipeError> {
        let port = self.state.port_state();
        if !port.connected || port.connection_generation != self.connection_generation {
            return Err(PipeError::Disconnected);
        }
        if self.target.device_address != 0 && port.reset_generation != self.reset_generation {
            return Err(PipeError::Disconnected);
        }
        Ok(())
    }

    async fn wait_until_transaction_due(&self) {
        #[cfg(target_os = "none")]
        if self.next_transaction > embassy_time::Instant::now() {
            embassy_time::Timer::at(self.next_transaction).await;
        }
    }

    fn schedule_after_attempt(&mut self, retry: bool) {
        #[cfg(target_os = "none")]
        {
            let delay_ms = next_transaction_delay_ms(self.target, retry);
            self.next_transaction =
                embassy_time::Instant::now() + embassy_time::Duration::from_millis(delay_ms);
        }
        #[cfg(not(target_os = "none"))]
        let _ = retry;
    }

    async fn request_in_packet(&mut self, buffer: &mut [u8]) -> Result<usize, PipeError>
    where
        E: PioPacketEngine,
    {
        let mut no_response_count = 0_u16;
        loop {
            #[cfg(target_os = "none")]
            record_in_pipe_progress(InPipeProgressEvent::Start);
            self.wait_until_transaction_due().await;
            #[cfg(target_os = "none")]
            record_in_pipe_progress(InPipeProgressEvent::DeadlineReady);
            self.check_valid()?;
            let mut engine = self.state.engine.lock().await;
            #[cfg(target_os = "none")]
            record_in_pipe_progress(InPipeProgressEvent::EngineAcquired);
            self.check_valid()?;
            let result = engine
                .request_in_once(self.target, &mut self.data_toggle, buffer)
                .await;
            #[cfg(target_os = "none")]
            record_in_pipe_progress(InPipeProgressEvent::EngineReturned);
            let _ = engine.service_frame().await;
            #[cfg(target_os = "none")]
            record_in_pipe_progress(InPipeProgressEvent::ServiceReturned);
            drop(engine);
            self.check_valid()?;
            let outcome = result?;
            match outcome {
                TransactionOutcome::Complete(len) => {
                    self.schedule_after_attempt(false);
                    return if len <= buffer.len() {
                        Ok(len)
                    } else {
                        Err(PipeError::BufferOverflow)
                    };
                }
                TransactionOutcome::Nak => {
                    no_response_count = 0;
                    // A successful bulk packet may be followed immediately
                    // within the same frame, but a NAK must yield until the
                    // next 1 ms retry slot. Interrupt endpoints retain their
                    // descriptor interval through next_transaction_delay_ms.
                    self.schedule_after_attempt(true);
                }
                TransactionOutcome::NoResponse => {
                    no_response_count += 1;
                    if no_response_count >= NON_RESPONSE_RETRY_LIMIT {
                        return Err(PipeError::Timeout);
                    }
                    self.schedule_after_attempt(true);
                }
            }
            yield_to_other_pipes().await;
        }
    }
}

impl<'d, E, T, D> UsbPipe<T, D> for PioHostPipe<'d, E, T, D>
where
    E: PioPacketEngine + 'd,
    T: pipe::Type,
    D: pipe::Direction,
{
    async fn control_in(&mut self, setup: &[u8; 8], buffer: &mut [u8]) -> Result<usize, PipeError>
    where
        T: pipe::IsControl,
        D: pipe::IsIn,
    {
        self.check_valid()?;
        let mut engine = self.state.engine.lock().await;
        self.check_valid()?;
        let result = engine
            .control_in(self.target, setup, buffer, self.timeout)
            .await;
        let _ = engine.service_frame().await;
        drop(engine);
        self.check_valid()?;
        result
    }

    async fn control_out(&mut self, setup: &[u8; 8], data: &[u8]) -> Result<(), PipeError>
    where
        T: pipe::IsControl,
        D: pipe::IsOut,
    {
        self.check_valid()?;
        let mut engine = self.state.engine.lock().await;
        self.check_valid()?;
        let result = engine
            .control_out(self.target, setup, data, self.timeout)
            .await;
        let _ = engine.service_frame().await;
        drop(engine);
        self.check_valid()?;
        result
    }

    async fn request_in(&mut self, buffer: &mut [u8]) -> Result<usize, PipeError>
    where
        D: pipe::IsIn,
    {
        if buffer.is_empty() {
            return Ok(0);
        }

        let max_packet_size = self.target.endpoint.max_packet_size as usize;
        let accumulate = self.target.endpoint.ep_type == EndpointType::Bulk;
        let buffer_len = buffer.len();
        let mut received = 0_usize;

        loop {
            let packet_len = self.request_in_packet(&mut buffer[received..]).await?;
            received += packet_len;

            if !accumulate || received == buffer_len || packet_len < max_packet_size {
                return Ok(received);
            }

            // A full bulk packet can be followed by another packet in the
            // same transfer. Yield at the transaction boundary so unrelated
            // pipes can acquire the single physical PIO engine.
            yield_to_other_pipes().await;
        }
    }

    async fn request_out(
        &mut self,
        data: &[u8],
        ensure_transaction_end: bool,
    ) -> Result<(), PipeError>
    where
        D: pipe::IsOut,
    {
        let max_packet_size = self.target.endpoint.max_packet_size as usize;
        let data_packet_count = data.len().div_ceil(max_packet_size);
        let packet_count = if data.is_empty() {
            1
        } else if ensure_transaction_end && data.len().is_multiple_of(max_packet_size) {
            data_packet_count + 1
        } else {
            data_packet_count
        };

        for packet_index in 0..packet_count {
            let start = (packet_index * max_packet_size).min(data.len());
            let end = (start + max_packet_size).min(data.len());
            let packet = &data[start..end];
            let mut no_response_count = 0_u16;
            loop {
                self.wait_until_transaction_due().await;
                self.check_valid()?;
                let mut engine = self.state.engine.lock().await;
                self.check_valid()?;
                let result = engine
                    .request_out_once(self.target, &mut self.data_toggle, packet)
                    .await;
                let _ = engine.service_frame().await;
                drop(engine);
                self.check_valid()?;
                let outcome = result?;
                match outcome {
                    TransactionOutcome::Complete(()) => {
                        self.schedule_after_attempt(false);
                        break;
                    }
                    TransactionOutcome::Nak => {
                        no_response_count = 0;
                        self.schedule_after_attempt(true);
                    }
                    TransactionOutcome::NoResponse => {
                        no_response_count += 1;
                        if no_response_count >= NON_RESPONSE_RETRY_LIMIT {
                            return Err(PipeError::Timeout);
                        }
                        self.schedule_after_attempt(true);
                    }
                }
                yield_to_other_pipes().await;
            }
            if packet_index + 1 < packet_count {
                yield_to_other_pipes().await;
            }
        }
        Ok(())
    }

    fn set_timeout(&mut self, timeout: TimeoutConfig)
    where
        T: pipe::IsControl,
    {
        self.timeout = timeout;
    }

    fn reset_data_toggle(&mut self)
    where
        T: pipe::IsBulkOrInterrupt,
    {
        self.data_toggle = DataToggle::Data0;
    }
}

#[cfg(test)]
mod tests {
    use core::future::Future;
    use core::pin::{Pin, pin};
    use core::sync::atomic::{AtomicBool, Ordering};
    use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};

    use super::*;
    use crate::host::Direction;

    static ATTACH_RESET_GATE: AtomicBool = AtomicBool::new(false);
    static CONTROLLER_RESET_GATE: AtomicBool = AtomicBool::new(false);

    #[derive(Default)]
    struct FakeEngine {
        resets: usize,
        reset_speeds: [Option<Speed>; 4],
        frames: usize,
        in_calls: usize,
        out_calls: usize,
        in_naks: usize,
        in_packet_lengths: &'static [usize],
        in_packet_index: usize,
        in_error: Option<PipeError>,
        frame_error: Option<PipeError>,
        reset_gate: Option<&'static AtomicBool>,
        reset_error: Option<PipeError>,
    }

    impl PioPacketEngine for FakeEngine {
        async fn bus_reset(&mut self, speed: Speed) -> Result<(), PipeError> {
            if let Some(slot) = self.reset_speeds.get_mut(self.resets) {
                *slot = Some(speed);
            }
            self.resets += 1;
            if let Some(reset_gate) = self.reset_gate {
                core::future::poll_fn(|cx| {
                    if reset_gate.load(Ordering::Relaxed) {
                        Poll::Ready(())
                    } else {
                        cx.waker().wake_by_ref();
                        Poll::Pending
                    }
                })
                .await;
            }
            self.reset_error.map_or(Ok(()), Err)
        }

        async fn service_frame(&mut self) -> Result<(), PipeError> {
            self.frames += 1;
            self.frame_error.map_or(Ok(()), Err)
        }

        async fn control_in(
            &mut self,
            _target: PipeTarget,
            _setup: &[u8; 8],
            buffer: &mut [u8],
            _timeout: TimeoutConfig,
        ) -> Result<usize, PipeError> {
            self.in_calls += 1;
            buffer[..2].copy_from_slice(b"OK");
            Ok(2)
        }

        async fn control_out(
            &mut self,
            _target: PipeTarget,
            _setup: &[u8; 8],
            _data: &[u8],
            _timeout: TimeoutConfig,
        ) -> Result<(), PipeError> {
            self.out_calls += 1;
            Ok(())
        }

        async fn request_in_once(
            &mut self,
            _target: PipeTarget,
            data_toggle: &mut DataToggle,
            buffer: &mut [u8],
        ) -> Result<TransactionOutcome<usize>, PipeError> {
            self.in_calls += 1;
            if let Some(error) = self.in_error {
                return Err(error);
            }
            if self.in_naks != 0 {
                self.in_naks -= 1;
                return Ok(TransactionOutcome::Nak);
            }

            let (packet_len, fill) = if self.in_packet_lengths.is_empty() {
                (1, 0x5a)
            } else {
                let Some(&packet_len) = self.in_packet_lengths.get(self.in_packet_index) else {
                    return Err(PipeError::BadResponse);
                };
                self.in_packet_index += 1;
                (packet_len, self.in_packet_index as u8)
            };
            if packet_len > buffer.len() {
                return Err(PipeError::BufferOverflow);
            }
            buffer[..packet_len].fill(fill);
            *data_toggle = data_toggle.after_ack();
            Ok(TransactionOutcome::Complete(packet_len))
        }

        async fn request_out_once(
            &mut self,
            _target: PipeTarget,
            data_toggle: &mut DataToggle,
            _packet: &[u8],
        ) -> Result<TransactionOutcome<()>, PipeError> {
            self.out_calls += 1;
            *data_toggle = data_toggle.after_ack();
            Ok(TransactionOutcome::Complete(()))
        }
    }

    fn noop_waker() -> Waker {
        const VTABLE: RawWakerVTable = RawWakerVTable::new(|_| raw_waker(), |_| {}, |_| {}, |_| {});

        const fn raw_waker() -> RawWaker {
            RawWaker::new(core::ptr::null(), &VTABLE)
        }

        // SAFETY: the no-op vtable never dereferences the null data pointer.
        unsafe { Waker::from_raw(raw_waker()) }
    }

    fn poll_once<F: Future>(future: Pin<&mut F>) -> Poll<F::Output> {
        let waker = noop_waker();
        let mut context = Context::from_waker(&waker);
        future.poll(&mut context)
    }

    fn block_on<F: Future>(future: F) -> F::Output {
        let mut future = pin!(future);
        match poll_once(future.as_mut()) {
            Poll::Ready(output) => output,
            Poll::Pending => panic!("fake engine futures must complete immediately"),
        }
    }

    fn bulk_endpoint(direction: Direction) -> EndpointInfo {
        EndpointInfo {
            addr: embassy_usb_driver::EndpointAddress::from_parts(2, direction),
            ep_type: EndpointType::Bulk,
            max_packet_size: 64,
            interval_ms: 0,
        }
    }

    fn interrupt_endpoint(direction: Direction) -> EndpointInfo {
        EndpointInfo {
            addr: embassy_usb_driver::EndpointAddress::from_parts(3, direction),
            ep_type: EndpointType::Interrupt,
            max_packet_size: 8,
            interval_ms: 1,
        }
    }

    fn control_endpoint(max_packet_size: u16) -> EndpointInfo {
        EndpointInfo {
            addr: embassy_usb_driver::EndpointAddress::from_parts(0, Direction::In),
            ep_type: EndpointType::Control,
            max_packet_size,
            interval_ms: 0,
        }
    }

    #[test]
    fn allocator_builds_exact_embassy_pipe_types() {
        let state = PioHostState::new(FakeEngine::default());
        block_on(state.reset_and_report_connected(Speed::Full)).unwrap();
        let controller = state.controller().unwrap();
        let allocator = controller.allocator();

        let input = allocator
            .alloc_pipe::<pipe::Bulk, pipe::In>(1, &bulk_endpoint(Direction::In), None)
            .unwrap();
        let output = allocator
            .alloc_pipe::<pipe::Bulk, pipe::Out>(1, &bulk_endpoint(Direction::Out), None)
            .unwrap();

        assert_eq!(input.target().device_address, 1);
        assert_eq!(input.target().endpoint.addr.index(), 2);
        assert_eq!(output.target().endpoint.addr.direction(), Direction::Out);
    }

    #[test]
    fn attach_and_controller_reset_forward_the_connected_speed() {
        let state = PioHostState::new(FakeEngine::default());
        block_on(state.reset_and_report_connected(Speed::Low)).unwrap();
        {
            let engine = state.engine.try_lock().unwrap();
            assert_eq!(engine.resets, 1);
            assert_eq!(engine.reset_speeds[0], Some(Speed::Low));
        }

        let mut controller = state.controller().unwrap();
        block_on(controller.bus_reset());

        let engine = state.engine.try_lock().unwrap();
        assert_eq!(engine.resets, 2);
        assert_eq!(
            &engine.reset_speeds[..2],
            &[Some(Speed::Low), Some(Speed::Low)]
        );
    }

    #[test]
    fn idle_frame_service_runs_for_low_and_full_speed_only() {
        for speed in [Speed::Low, Speed::Full] {
            let state = PioHostState::new(FakeEngine::default());
            block_on(state.reset_and_report_connected(speed)).unwrap();

            assert_eq!(block_on(state.service_frame()), Ok(()));
            assert_eq!(state.engine.try_lock().unwrap().frames, 1);
        }

        let high_speed = PioHostState::new(FakeEngine::default());
        block_on(high_speed.reset_and_report_connected(Speed::High)).unwrap();
        assert_eq!(
            block_on(high_speed.service_frame()),
            Err(PipeError::Disconnected)
        );
        assert_eq!(high_speed.engine.try_lock().unwrap().frames, 0);

        let disconnected = PioHostState::new(FakeEngine::default());
        assert_eq!(
            block_on(disconnected.service_frame()),
            Err(PipeError::Disconnected)
        );
    }

    #[test]
    fn low_speed_allocator_accepts_ep0_and_interrupt_limits() {
        let state = PioHostState::new(FakeEngine::default());
        block_on(state.reset_and_report_connected(Speed::Low)).unwrap();
        let allocator = state.controller().unwrap().allocator();

        assert!(
            allocator
                .alloc_pipe::<pipe::Control, pipe::InOut>(0, &control_endpoint(8), None)
                .is_ok()
        );

        let interrupt = EndpointInfo {
            interval_ms: 10,
            ..interrupt_endpoint(Direction::In)
        };
        assert!(
            allocator
                .alloc_pipe::<pipe::Interrupt, pipe::In>(1, &interrupt, None)
                .is_ok()
        );
    }

    #[test]
    fn low_speed_allocator_rejects_noncompliant_routes() {
        let state = PioHostState::new(FakeEngine::default());
        block_on(state.reset_and_report_connected(Speed::Low)).unwrap();
        let allocator = state.controller().unwrap().allocator();

        assert!(matches!(
            allocator.alloc_pipe::<pipe::Control, pipe::InOut>(0, &control_endpoint(16), None),
            Err(HostError::InvalidDescriptor)
        ));

        let too_large = EndpointInfo {
            max_packet_size: 9,
            interval_ms: 10,
            ..interrupt_endpoint(Direction::In)
        };
        assert!(matches!(
            allocator.alloc_pipe::<pipe::Interrupt, pipe::In>(1, &too_large, None),
            Err(HostError::InvalidDescriptor)
        ));

        let too_frequent = EndpointInfo {
            interval_ms: 9,
            ..interrupt_endpoint(Direction::In)
        };
        assert!(matches!(
            allocator.alloc_pipe::<pipe::Interrupt, pipe::In>(1, &too_frequent, None),
            Err(HostError::InvalidDescriptor)
        ));

        assert!(matches!(
            allocator.alloc_pipe::<pipe::Bulk, pipe::In>(1, &bulk_endpoint(Direction::In), None),
            Err(HostError::Other(_))
        ));

        let isochronous = EndpointInfo {
            addr: embassy_usb_driver::EndpointAddress::from_parts(4, Direction::In),
            ep_type: EndpointType::Isochronous,
            max_packet_size: 8,
            interval_ms: 10,
        };
        assert!(matches!(
            allocator.alloc_pipe::<pipe::Isochronous, pipe::In>(1, &isochronous, None),
            Err(HostError::Other(_))
        ));

        let split = SplitInfo::new(2, 1, crate::host::SplitSpeed::Low);
        assert!(matches!(
            allocator.alloc_pipe::<pipe::Control, pipe::InOut>(
                0,
                &control_endpoint(8),
                Some(split)
            ),
            Err(HostError::Other(_))
        ));
    }

    #[test]
    fn high_speed_allocator_is_rejected() {
        let state = PioHostState::new(FakeEngine::default());
        block_on(state.reset_and_report_connected(Speed::High)).unwrap();
        let allocator = state.controller().unwrap().allocator();

        assert!(matches!(
            allocator.alloc_pipe::<pipe::Control, pipe::InOut>(0, &control_endpoint(64), None),
            Err(HostError::Other(_))
        ));
    }

    #[test]
    fn pipes_delegate_and_retain_independent_toggles() {
        let state = PioHostState::new(FakeEngine::default());
        block_on(state.reset_and_report_connected(Speed::Full)).unwrap();
        let allocator = state.controller().unwrap().allocator();
        let mut input = allocator
            .alloc_pipe::<pipe::Bulk, pipe::In>(1, &bulk_endpoint(Direction::In), None)
            .unwrap();
        let mut output = allocator
            .alloc_pipe::<pipe::Bulk, pipe::Out>(1, &bulk_endpoint(Direction::Out), None)
            .unwrap();

        let mut packet = [0; 64];
        assert_eq!(block_on(input.request_in(&mut packet)), Ok(1));
        assert_eq!(packet[0], 0x5a);
        assert_eq!(block_on(output.request_out(b"AT\r\n", false)), Ok(()));
        assert_eq!(input.data_toggle, DataToggle::Data1);
        assert_eq!(output.data_toggle, DataToggle::Data1);

        input.reset_data_toggle();
        assert_eq!(input.data_toggle, DataToggle::Data0);
        assert_eq!(output.data_toggle, DataToggle::Data1);
    }

    #[test]
    fn empty_in_buffer_is_a_no_op() {
        let state = PioHostState::new(FakeEngine::default());
        block_on(state.reset_and_report_connected(Speed::Full)).unwrap();
        let allocator = state.controller().unwrap().allocator();
        let mut input = allocator
            .alloc_pipe::<pipe::Bulk, pipe::In>(1, &bulk_endpoint(Direction::In), None)
            .unwrap();
        let mut empty = [];

        assert_eq!(block_on(input.request_in(&mut empty)), Ok(0));
        assert_eq!(input.data_toggle, DataToggle::Data0);
        assert_eq!(state.engine.try_lock().unwrap().in_calls, 0);
    }

    #[test]
    fn bulk_in_accumulates_full_packets_until_the_buffer_is_full() {
        let state = PioHostState::new(FakeEngine {
            in_packet_lengths: &[8, 8],
            ..FakeEngine::default()
        });
        block_on(state.reset_and_report_connected(Speed::Full)).unwrap();
        let allocator = state.controller().unwrap().allocator();
        let endpoint = EndpointInfo {
            max_packet_size: 8,
            ..bulk_endpoint(Direction::In)
        };
        let mut input = allocator
            .alloc_pipe::<pipe::Bulk, pipe::In>(1, &endpoint, None)
            .unwrap();
        let mut data = [0; 16];

        {
            let mut request = pin!(input.request_in(&mut data));
            assert!(poll_once(request.as_mut()).is_pending());
            assert!(state.engine.try_lock().is_ok());
            assert_eq!(poll_once(request.as_mut()), Poll::Ready(Ok(16)));
        }

        assert_eq!(&data[..8], &[1; 8]);
        assert_eq!(&data[8..], &[2; 8]);
        assert_eq!(input.data_toggle, DataToggle::Data0);
        let engine = state.engine.try_lock().unwrap();
        assert_eq!(engine.in_calls, 2);
        assert_eq!(engine.in_packet_index, 2);
        assert_eq!(engine.frames, 2);
    }

    #[test]
    fn bulk_in_stops_on_a_short_packet() {
        let state = PioHostState::new(FakeEngine {
            in_packet_lengths: &[8, 3, 8],
            ..FakeEngine::default()
        });
        block_on(state.reset_and_report_connected(Speed::Full)).unwrap();
        let allocator = state.controller().unwrap().allocator();
        let endpoint = EndpointInfo {
            max_packet_size: 8,
            ..bulk_endpoint(Direction::In)
        };
        let mut input = allocator
            .alloc_pipe::<pipe::Bulk, pipe::In>(1, &endpoint, None)
            .unwrap();
        let mut data = [0xcc; 24];

        {
            let mut request = pin!(input.request_in(&mut data));
            assert!(poll_once(request.as_mut()).is_pending());
            assert_eq!(poll_once(request.as_mut()), Poll::Ready(Ok(11)));
        }

        assert_eq!(&data[..8], &[1; 8]);
        assert_eq!(&data[8..11], &[2; 3]);
        assert_eq!(&data[11..], &[0xcc; 13]);
        assert_eq!(input.data_toggle, DataToggle::Data0);
        let engine = state.engine.try_lock().unwrap();
        assert_eq!(engine.in_calls, 2);
        assert_eq!(engine.in_packet_index, 2);
    }

    #[test]
    fn bulk_in_stops_on_a_zero_length_packet() {
        let state = PioHostState::new(FakeEngine {
            in_packet_lengths: &[8, 0, 8],
            ..FakeEngine::default()
        });
        block_on(state.reset_and_report_connected(Speed::Full)).unwrap();
        let allocator = state.controller().unwrap().allocator();
        let endpoint = EndpointInfo {
            max_packet_size: 8,
            ..bulk_endpoint(Direction::In)
        };
        let mut input = allocator
            .alloc_pipe::<pipe::Bulk, pipe::In>(1, &endpoint, None)
            .unwrap();
        let mut data = [0xcc; 16];

        {
            let mut request = pin!(input.request_in(&mut data));
            assert!(poll_once(request.as_mut()).is_pending());
            assert_eq!(poll_once(request.as_mut()), Poll::Ready(Ok(8)));
        }

        assert_eq!(&data[..8], &[1; 8]);
        assert_eq!(&data[8..], &[0xcc; 8]);
        assert_eq!(input.data_toggle, DataToggle::Data0);
        let engine = state.engine.try_lock().unwrap();
        assert_eq!(engine.in_calls, 2);
        assert_eq!(engine.in_packet_index, 2);
    }

    #[test]
    fn interrupt_in_returns_exactly_one_packet() {
        let state = PioHostState::new(FakeEngine {
            in_packet_lengths: &[8, 8],
            ..FakeEngine::default()
        });
        block_on(state.reset_and_report_connected(Speed::Full)).unwrap();
        let allocator = state.controller().unwrap().allocator();
        let mut input = allocator
            .alloc_pipe::<pipe::Interrupt, pipe::In>(1, &interrupt_endpoint(Direction::In), None)
            .unwrap();
        let mut data = [0; 16];

        assert_eq!(block_on(input.request_in(&mut data)), Ok(8));
        assert_eq!(&data[..8], &[1; 8]);
        assert_eq!(&data[8..], &[0; 8]);
        assert_eq!(input.data_toggle, DataToggle::Data1);
        let engine = state.engine.try_lock().unwrap();
        assert_eq!(engine.in_calls, 1);
        assert_eq!(engine.in_packet_index, 1);
    }

    #[test]
    fn detach_between_bulk_in_packets_aborts_before_another_wire_call() {
        let state = PioHostState::new(FakeEngine {
            in_packet_lengths: &[8, 8],
            ..FakeEngine::default()
        });
        block_on(state.reset_and_report_connected(Speed::Full)).unwrap();
        let allocator = state.controller().unwrap().allocator();
        let endpoint = EndpointInfo {
            max_packet_size: 8,
            ..bulk_endpoint(Direction::In)
        };
        let mut input = allocator
            .alloc_pipe::<pipe::Bulk, pipe::In>(1, &endpoint, None)
            .unwrap();
        let mut data = [0; 16];
        {
            let mut request = pin!(input.request_in(&mut data));

            assert!(poll_once(request.as_mut()).is_pending());
            state.report_disconnected();
            assert_eq!(
                poll_once(request.as_mut()),
                Poll::Ready(Err(PipeError::Disconnected))
            );
        }

        assert_eq!(&data[..8], &[1; 8]);
        assert_eq!(&data[8..], &[0; 8]);
        assert_eq!(input.data_toggle, DataToggle::Data1);
        let engine = state.engine.try_lock().unwrap();
        assert_eq!(engine.in_calls, 1);
        assert_eq!(engine.in_packet_index, 1);
    }

    #[test]
    fn cancelling_between_bulk_in_packets_leaves_the_pipe_reusable() {
        let state = PioHostState::new(FakeEngine {
            in_packet_lengths: &[8, 8],
            ..FakeEngine::default()
        });
        block_on(state.reset_and_report_connected(Speed::Full)).unwrap();
        let allocator = state.controller().unwrap().allocator();
        let endpoint = EndpointInfo {
            max_packet_size: 8,
            ..bulk_endpoint(Direction::In)
        };
        let mut input = allocator
            .alloc_pipe::<pipe::Bulk, pipe::In>(1, &endpoint, None)
            .unwrap();
        let mut cancelled_data = [0; 16];

        {
            let mut request = pin!(input.request_in(&mut cancelled_data));
            assert!(poll_once(request.as_mut()).is_pending());
        }

        assert_eq!(&cancelled_data[..8], &[1; 8]);
        assert_eq!(input.data_toggle, DataToggle::Data1);

        let mut resumed_data = [0; 8];
        assert_eq!(block_on(input.request_in(&mut resumed_data)), Ok(8));
        assert_eq!(resumed_data, [2; 8]);
        assert_eq!(input.data_toggle, DataToggle::Data0);
        let engine = state.engine.try_lock().unwrap();
        assert_eq!(engine.in_calls, 2);
        assert_eq!(engine.in_packet_index, 2);
    }

    #[test]
    fn detach_and_reset_invalidate_the_required_pipes() {
        let state = PioHostState::new(FakeEngine::default());
        block_on(state.reset_and_report_connected(Speed::Full)).unwrap();
        assert_eq!(state.engine.try_lock().unwrap().resets, 1);
        let mut controller = state.controller().unwrap();
        let allocator = controller.allocator();
        let ep0 = EndpointInfo {
            addr: embassy_usb_driver::EndpointAddress::from_parts(0, Direction::In),
            ep_type: EndpointType::Control,
            max_packet_size: 8,
            interval_ms: 0,
        };
        let mut control = allocator
            .alloc_pipe::<pipe::Control, pipe::InOut>(0, &ep0, None)
            .unwrap();
        let mut bulk = allocator
            .alloc_pipe::<pipe::Bulk, pipe::Out>(1, &bulk_endpoint(Direction::Out), None)
            .unwrap();

        block_on(controller.bus_reset());
        assert_eq!(state.engine.try_lock().unwrap().resets, 2);
        assert_eq!(block_on(control.control_out(&[0; 8], &[])), Ok(()));
        assert_eq!(
            block_on(bulk.request_out(b"x", false)),
            Err(PipeError::Disconnected)
        );

        state.report_disconnected();
        assert_eq!(
            block_on(control.control_out(&[0; 8], &[])),
            Err(PipeError::Disconnected)
        );
    }

    #[test]
    fn allocator_rejects_unsupported_or_mismatched_routes() {
        let state = PioHostState::new(FakeEngine::default());
        block_on(state.reset_and_report_connected(Speed::Full)).unwrap();
        let allocator = state.controller().unwrap().allocator();
        let input = bulk_endpoint(Direction::In);

        assert!(matches!(
            allocator.alloc_pipe::<pipe::Bulk, pipe::Out>(1, &input, None),
            Err(HostError::Other(_))
        ));

        let split = SplitInfo::new(2, 1, crate::host::SplitSpeed::Full);
        assert!(matches!(
            allocator.alloc_pipe::<pipe::Bulk, pipe::In>(1, &input, Some(split)),
            Err(HostError::Other(_))
        ));

        let reserved_address = EndpointInfo {
            addr: embassy_usb_driver::EndpointAddress::from(0x91),
            ..input
        };
        assert!(matches!(
            allocator.alloc_pipe::<pipe::Bulk, pipe::In>(1, &reserved_address, None),
            Err(HostError::InvalidDescriptor)
        ));
    }

    #[test]
    fn controller_is_singleton_and_destructive_events_are_not_overwritten() {
        let state = PioHostState::new(FakeEngine::default());
        let mut controller = state.controller().unwrap();
        assert!(matches!(state.controller(), Err(HostError::Other(_))));

        block_on(state.reset_and_report_connected(Speed::Full)).unwrap();
        state.report_overcurrent();
        state.report_disconnected();
        block_on(state.reset_and_report_connected(Speed::Full)).unwrap();

        assert_eq!(
            block_on(controller.wait_for_device_event()),
            DeviceEvent::Overcurrent
        );
        assert_eq!(
            block_on(controller.wait_for_device_event()),
            DeviceEvent::Connected(Speed::Full)
        );
    }

    #[test]
    fn detach_during_reset_cannot_publish_a_stale_connected_event() {
        ATTACH_RESET_GATE.store(false, Ordering::Relaxed);
        let state = PioHostState::new(FakeEngine {
            reset_gate: Some(&ATTACH_RESET_GATE),
            ..FakeEngine::default()
        });
        let mut controller = state.controller().unwrap();
        {
            let mut cancelled_reset = pin!(state.reset_and_report_connected(Speed::Full));
            assert!(poll_once(cancelled_reset.as_mut()).is_pending());
            assert!(state.is_reset_in_progress());
            assert!(!state.report_disconnected_if_not_resetting());
        }
        assert!(!state.is_reset_in_progress());

        let mut reset = pin!(state.reset_and_report_connected(Speed::Full));

        assert!(poll_once(reset.as_mut()).is_pending());
        assert!(state.is_reset_in_progress());
        state.report_disconnected();
        ATTACH_RESET_GATE.store(true, Ordering::Relaxed);
        assert_eq!(
            poll_once(reset.as_mut()),
            Poll::Ready(Err(PipeError::Disconnected))
        );
        assert!(!state.is_reset_in_progress());
        assert!(!state.port_state().connected);
        assert_eq!(
            block_on(controller.wait_for_device_event()),
            DeviceEvent::Disconnected
        );
    }

    #[test]
    fn idle_sof_skips_a_busy_engine_instead_of_waiting() {
        let state = PioHostState::new(FakeEngine::default());
        block_on(state.reset_and_report_connected(Speed::Full)).unwrap();

        let engine = block_on(state.engine.lock());
        assert_eq!(block_on(state.service_frame()), Ok(()));
        drop(engine);
        assert_eq!(state.engine.try_lock().unwrap().frames, 0);

        assert_eq!(block_on(state.service_frame()), Ok(()));
        assert_eq!(state.engine.try_lock().unwrap().frames, 1);
    }

    #[test]
    fn nak_retry_releases_engine_for_another_pipe() {
        let state = PioHostState::new(FakeEngine::default());
        block_on(state.reset_and_report_connected(Speed::Full)).unwrap();
        state.engine.try_lock().unwrap().in_naks = 1;
        let allocator = state.controller().unwrap().allocator();
        let mut input = allocator
            .alloc_pipe::<pipe::Bulk, pipe::In>(1, &bulk_endpoint(Direction::In), None)
            .unwrap();
        let mut output = allocator
            .alloc_pipe::<pipe::Bulk, pipe::Out>(1, &bulk_endpoint(Direction::Out), None)
            .unwrap();
        let mut byte = [0; 64];
        {
            let mut pending_input = pin!(input.request_in(&mut byte));
            assert!(poll_once(pending_input.as_mut()).is_pending());
            assert_eq!(block_on(output.request_out(b"x", false)), Ok(()));
            assert_eq!(poll_once(pending_input.as_mut()), Poll::Ready(Ok(1)));
        }
        assert_eq!(byte[0], 0x5a);
        let engine = state.engine.try_lock().unwrap();
        assert_eq!(engine.in_calls, 2);
        assert_eq!(engine.out_calls, 1);
    }

    #[test]
    fn detach_while_waiting_for_engine_prevents_stale_wire_call() {
        let state = PioHostState::new(FakeEngine::default());
        block_on(state.reset_and_report_connected(Speed::Full)).unwrap();
        let allocator = state.controller().unwrap().allocator();
        let mut input = allocator
            .alloc_pipe::<pipe::Bulk, pipe::In>(1, &bulk_endpoint(Direction::In), None)
            .unwrap();
        let mut byte = [0; 64];
        let engine = block_on(state.engine.lock());
        let mut request = pin!(input.request_in(&mut byte));

        assert!(poll_once(request.as_mut()).is_pending());
        state.report_disconnected();
        drop(engine);
        assert_eq!(
            poll_once(request.as_mut()),
            Poll::Ready(Err(PipeError::Disconnected))
        );
        assert_eq!(state.engine.try_lock().unwrap().in_calls, 0);
    }

    #[test]
    fn failed_controller_reset_still_invalidates_addressed_pipes() {
        let state = PioHostState::new(FakeEngine::default());
        block_on(state.reset_and_report_connected(Speed::Full)).unwrap();
        let mut controller = state.controller().unwrap();
        let allocator = controller.allocator();
        let mut bulk = allocator
            .alloc_pipe::<pipe::Bulk, pipe::Out>(1, &bulk_endpoint(Direction::Out), None)
            .unwrap();
        state.engine.try_lock().unwrap().reset_error = Some(PipeError::BadResponse);

        block_on(controller.bus_reset());
        assert_eq!(
            block_on(bulk.request_out(b"x", false)),
            Err(PipeError::Disconnected)
        );
    }

    #[test]
    fn cancelling_controller_reset_still_invalidates_addressed_pipes() {
        CONTROLLER_RESET_GATE.store(false, Ordering::Relaxed);
        let state = PioHostState::new(FakeEngine::default());
        block_on(state.reset_and_report_connected(Speed::Full)).unwrap();
        let mut controller = state.controller().unwrap();
        let allocator = controller.allocator();
        let mut bulk = allocator
            .alloc_pipe::<pipe::Bulk, pipe::Out>(1, &bulk_endpoint(Direction::Out), None)
            .unwrap();
        state.engine.try_lock().unwrap().reset_gate = Some(&CONTROLLER_RESET_GATE);

        {
            let mut reset = pin!(controller.bus_reset());
            assert!(poll_once(reset.as_mut()).is_pending());
            assert!(state.is_reset_in_progress());
        }
        assert!(!state.is_reset_in_progress());
        assert_eq!(
            block_on(bulk.request_out(b"x", false)),
            Err(PipeError::Disconnected)
        );
    }

    #[test]
    fn short_in_buffer_is_allowed_and_overflow_does_not_advance_toggle() {
        let state = PioHostState::new(FakeEngine::default());
        block_on(state.reset_and_report_connected(Speed::Full)).unwrap();
        let allocator = state.controller().unwrap().allocator();
        let mut input = allocator
            .alloc_pipe::<pipe::Bulk, pipe::In>(1, &bulk_endpoint(Direction::In), None)
            .unwrap();
        let mut short = [0; 1];

        assert_eq!(block_on(input.request_in(&mut short)), Ok(1));
        assert_eq!(short, [0x5a]);
        input.reset_data_toggle();
        state.engine.try_lock().unwrap().in_error = Some(PipeError::BufferOverflow);
        assert_eq!(
            block_on(input.request_in(&mut short)),
            Err(PipeError::BufferOverflow)
        );
        assert_eq!(state.engine.try_lock().unwrap().in_calls, 2);
        assert_eq!(input.data_toggle, DataToggle::Data0);
    }

    #[test]
    fn retry_schedule_respects_interrupt_interval_outside_the_engine() {
        let interrupt = PipeTarget {
            device_address: 1,
            endpoint: EndpointInfo {
                addr: embassy_usb_driver::EndpointAddress::from_parts(3, Direction::In),
                ep_type: EndpointType::Interrupt,
                max_packet_size: 16,
                interval_ms: 37,
            },
            split: None,
        };
        let bulk = PipeTarget {
            endpoint: bulk_endpoint(Direction::In),
            ..interrupt
        };

        assert_eq!(next_transaction_delay_ms(interrupt, true), 37);
        assert_eq!(next_transaction_delay_ms(interrupt, false), 37);
        assert_eq!(next_transaction_delay_ms(bulk, true), 1);
        assert_eq!(next_transaction_delay_ms(bulk, false), 0);
    }

    #[test]
    fn post_transaction_sof_error_does_not_hide_an_acked_transfer() {
        let state = PioHostState::new(FakeEngine::default());
        block_on(state.reset_and_report_connected(Speed::Full)).unwrap();
        state.engine.try_lock().unwrap().frame_error = Some(PipeError::Timeout);
        let allocator = state.controller().unwrap().allocator();
        let mut output = allocator
            .alloc_pipe::<pipe::Bulk, pipe::Out>(1, &bulk_endpoint(Direction::Out), None)
            .unwrap();

        assert_eq!(block_on(output.request_out(b"x", false)), Ok(()));
        assert_eq!(output.data_toggle, DataToggle::Data1);
        assert_eq!(state.engine.try_lock().unwrap().frames, 1);
    }

    #[test]
    fn in_transaction_diagnostics_retain_counts_and_latest_prefix() {
        let mut diagnostics = InTransactionDiagnostics::default();

        diagnostics.record_attempt();
        diagnostics.record_accepted_data();
        diagnostics.record_accepted_zlp();
        diagnostics.record_attempt();
        diagnostics.record_accepted_data();
        diagnostics.record_accepted_zlp();
        diagnostics.record_attempt();
        diagnostics.record_unexpected_toggle(
            crate::usb::PID_DATA1,
            crate::usb::PID_DATA0,
            b"0123456789",
        );
        diagnostics.record_attempt();
        diagnostics.record_unexpected_toggle(crate::usb::PID_DATA0, crate::usb::PID_DATA1, b"AT");
        diagnostics.record_attempt();
        diagnostics.record_nak();
        diagnostics.record_attempt();
        diagnostics.record_no_response();
        diagnostics.record_attempt();
        diagnostics.record_invalid_or_stall();

        assert_eq!(diagnostics.attempt_count, 7);
        assert_eq!(diagnostics.accepted_data_count, 2);
        assert_eq!(diagnostics.accepted_zlp_count, 2);
        assert_eq!(diagnostics.unexpected_toggle_count, 2);
        assert_eq!(diagnostics.nak_count, 1);
        assert_eq!(diagnostics.no_response_count, 1);
        assert_eq!(diagnostics.invalid_or_stall_count, 1);
        assert_eq!(
            diagnostics.attempt_count,
            diagnostics.accepted_data_count
                + diagnostics.unexpected_toggle_count
                + diagnostics.nak_count
                + diagnostics.no_response_count
                + diagnostics.invalid_or_stall_count
        );
        assert_eq!(
            diagnostics.latest_unexpected_toggle,
            Some(UnexpectedToggleDiagnostic {
                expected_pid: crate::usb::PID_DATA0,
                actual_pid: crate::usb::PID_DATA1,
                payload_len: 2,
                payload_prefix_len: 2,
                payload_prefix: [b'A', b'T', 0, 0, 0, 0, 0, 0],
            })
        );
    }

    #[test]
    fn taking_in_transaction_diagnostics_clears_the_accumulator() {
        let mut diagnostics = InTransactionDiagnostics::default();
        diagnostics.record_accepted_zlp();
        diagnostics.record_unexpected_toggle(
            crate::usb::PID_DATA1,
            crate::usb::PID_DATA0,
            b"0123456789",
        );

        let taken = diagnostics.take();

        assert_eq!(taken.accepted_zlp_count, 1);
        assert_eq!(taken.unexpected_toggle_count, 1);
        let latest = taken.latest_unexpected_toggle.unwrap();
        assert_eq!(latest.payload_len, 10);
        assert_eq!(latest.payload_prefix_len, 8);
        assert_eq!(latest.payload_prefix, *b"01234567");
        assert_eq!(diagnostics, InTransactionDiagnostics::default());
    }
}
