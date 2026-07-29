//! Controller-independent CDC-ACM host class.
//!
//! The class uses the upstream Embassy host-pipe contract directly while
//! retaining this crate's stricter CDC Union/interface descriptor discovery.
//! USB packetization, retry and DATA-toggle state remain pipe concerns.

use crate::host::{
    Direction, EndpointAddress, EndpointInfo, EndpointType, HostError, PipeError, SplitInfo,
    TimeoutConfig, UsbHostAllocator, UsbPipe, pipe,
};
use crate::usb::{
    CdcAcmFunction, CdcLineCoding, CdcLineCodingError, ConfigurationError, SetupRequest,
};

/// Receive-packet capacity used by the ergonomic full-speed class aliases and
/// allocation helpers.
pub const DEFAULT_RX_PACKET_CAPACITY: usize = 64;

/// Receive-packet capacity required by a high-speed USB bulk endpoint.
pub const HIGH_SPEED_RX_PACKET_CAPACITY: usize = 512;

/// A [`CdcAcmHost`] whose three pipes came from an Embassy host allocator.
pub type AllocatedCdcAcmHostWithRxCapacity<'d, A, const RX_PACKET_CAPACITY: usize> = CdcAcmHost<
    <A as UsbHostAllocator<'d>>::Pipe<pipe::Control, pipe::InOut>,
    <A as UsbHostAllocator<'d>>::Pipe<pipe::Bulk, pipe::In>,
    <A as UsbHostAllocator<'d>>::Pipe<pipe::Bulk, pipe::Out>,
    RX_PACKET_CAPACITY,
>;

/// A full-speed [`CdcAcmHost`] using the ergonomic 64-byte packet buffer.
pub type AllocatedCdcAcmHost<'d, A> =
    AllocatedCdcAcmHostWithRxCapacity<'d, A, DEFAULT_RX_PACKET_CAPACITY>;

/// A CDC-ACM host sized for one 512-byte high-speed bulk-IN packet.
pub type HighSpeedAllocatedCdcAcmHost<'d, A> =
    AllocatedCdcAcmHostWithRxCapacity<'d, A, HIGH_SPEED_RX_PACKET_CAPACITY>;

/// Error while discovering a CDC-ACM function and allocating its pipes.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CdcAcmCreateError {
    /// The configuration descriptor is malformed or has no usable CDC-ACM
    /// function.
    Configuration(ConfigurationError),
    /// A USB device address is only seven bits wide.
    InvalidDeviceAddress,
    /// Endpoint zero has an invalid full-speed maximum packet size.
    InvalidControlMaxPacketSize,
    /// The selected bulk-IN endpoint maximum packet size is invalid or does
    /// not fit in the class instance's configured receive buffer.
    UnsupportedBulkInMaxPacketSize,
    /// The host controller could not allocate one of the required pipes.
    Allocation(HostError),
}

impl From<ConfigurationError> for CdcAcmCreateError {
    fn from(error: ConfigurationError) -> Self {
        Self::Configuration(error)
    }
}

impl From<HostError> for CdcAcmCreateError {
    fn from(error: HostError) -> Self {
        Self::Allocation(error)
    }
}

impl core::fmt::Display for CdcAcmCreateError {
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Configuration(_) => {
                formatter.write_str("invalid or unsupported CDC-ACM configuration")
            }
            Self::InvalidDeviceAddress => formatter.write_str("invalid USB device address"),
            Self::InvalidControlMaxPacketSize => {
                formatter.write_str("invalid endpoint-zero maximum packet size")
            }
            Self::UnsupportedBulkInMaxPacketSize => {
                formatter.write_str("invalid or unsupported bulk-IN maximum packet size")
            }
            Self::Allocation(_) => formatter.write_str("could not allocate CDC-ACM host pipes"),
        }
    }
}

impl core::error::Error for CdcAcmCreateError {}

/// Discover a CDC-ACM function and allocate all pipes required by its class
/// driver.
///
/// `configuration` must contain the complete active configuration
/// descriptor. The strict parser follows the CDC Union Functional Descriptor,
/// so endpoints from unrelated functions in a composite device cannot be
/// paired accidentally.
pub fn allocate_cdc_acm_host<'d, A>(
    allocator: &A,
    configuration: &[u8],
    device_address: u8,
    control_max_packet_size: u16,
    split: Option<SplitInfo>,
) -> Result<AllocatedCdcAcmHost<'d, A>, CdcAcmCreateError>
where
    A: UsbHostAllocator<'d>,
{
    allocate_cdc_acm_host_with_rx_capacity::<A, DEFAULT_RX_PACKET_CAPACITY>(
        allocator,
        configuration,
        device_address,
        control_max_packet_size,
        split,
    )
}

/// Discover a CDC-ACM function and allocate it with an explicitly sized
/// internal bulk-IN packet buffer.
///
/// A high-speed controller can use
/// `allocate_cdc_acm_host_with_rx_capacity::<_, 512>` for a function whose
/// bulk endpoints advertise a 512-byte maximum packet size.
pub fn allocate_cdc_acm_host_with_rx_capacity<'d, A, const RX_PACKET_CAPACITY: usize>(
    allocator: &A,
    configuration: &[u8],
    device_address: u8,
    control_max_packet_size: u16,
    split: Option<SplitInfo>,
) -> Result<AllocatedCdcAcmHostWithRxCapacity<'d, A, RX_PACKET_CAPACITY>, CdcAcmCreateError>
where
    A: UsbHostAllocator<'d>,
{
    validate_allocation_inputs(device_address, control_max_packet_size)?;

    let function = CdcAcmFunction::discover(configuration)?;
    allocate_cdc_acm_function_with_rx_capacity::<A, RX_PACKET_CAPACITY>(
        allocator,
        function,
        device_address,
        control_max_packet_size,
        split,
    )
}

/// Select a specific CDC-ACM communications interface and allocate its pipes.
///
/// This is useful for composite devices containing multiple CDC-ACM
/// functions. Candidate-local errors in other functions do not prevent the
/// selected function from being allocated.
pub fn allocate_cdc_acm_host_for_control_interface<'d, A>(
    allocator: &A,
    configuration: &[u8],
    control_interface: u8,
    device_address: u8,
    control_max_packet_size: u16,
    split: Option<SplitInfo>,
) -> Result<AllocatedCdcAcmHost<'d, A>, CdcAcmCreateError>
where
    A: UsbHostAllocator<'d>,
{
    allocate_cdc_acm_host_for_control_interface_with_rx_capacity::<A, DEFAULT_RX_PACKET_CAPACITY>(
        allocator,
        configuration,
        control_interface,
        device_address,
        control_max_packet_size,
        split,
    )
}

/// Select a specific CDC-ACM communications interface and allocate it with an
/// explicitly sized internal bulk-IN packet buffer.
pub fn allocate_cdc_acm_host_for_control_interface_with_rx_capacity<
    'd,
    A,
    const RX_PACKET_CAPACITY: usize,
>(
    allocator: &A,
    configuration: &[u8],
    control_interface: u8,
    device_address: u8,
    control_max_packet_size: u16,
    split: Option<SplitInfo>,
) -> Result<AllocatedCdcAcmHostWithRxCapacity<'d, A, RX_PACKET_CAPACITY>, CdcAcmCreateError>
where
    A: UsbHostAllocator<'d>,
{
    validate_allocation_inputs(device_address, control_max_packet_size)?;

    let function = CdcAcmFunction::discover_control_interface(configuration, control_interface)?;
    allocate_cdc_acm_function_with_rx_capacity::<A, RX_PACKET_CAPACITY>(
        allocator,
        function,
        device_address,
        control_max_packet_size,
        split,
    )
}

/// Allocate pipes for an already selected and descriptor-validated CDC-ACM
/// function.
///
/// The default [`CdcAcmHost`] retains one complete bulk-IN packet in a
/// 64-byte internal buffer. Use [`allocate_cdc_acm_function_with_rx_capacity`]
/// when a controller supports a larger endpoint packet size.
pub fn allocate_cdc_acm_function<'d, A>(
    allocator: &A,
    function: CdcAcmFunction,
    device_address: u8,
    control_max_packet_size: u16,
    split: Option<SplitInfo>,
) -> Result<AllocatedCdcAcmHost<'d, A>, CdcAcmCreateError>
where
    A: UsbHostAllocator<'d>,
{
    allocate_cdc_acm_function_with_rx_capacity::<A, DEFAULT_RX_PACKET_CAPACITY>(
        allocator,
        function,
        device_address,
        control_max_packet_size,
        split,
    )
}

/// Allocate pipes for an already selected CDC-ACM function with an explicitly
/// sized internal bulk-IN packet buffer.
///
/// Capacity is validated before any pipe is allocated. A 512-byte capacity
/// is the standard choice for a high-speed bulk endpoint.
pub fn allocate_cdc_acm_function_with_rx_capacity<'d, A, const RX_PACKET_CAPACITY: usize>(
    allocator: &A,
    function: CdcAcmFunction,
    device_address: u8,
    control_max_packet_size: u16,
    split: Option<SplitInfo>,
) -> Result<AllocatedCdcAcmHostWithRxCapacity<'d, A, RX_PACKET_CAPACITY>, CdcAcmCreateError>
where
    A: UsbHostAllocator<'d>,
{
    validate_allocation_inputs(device_address, control_max_packet_size)?;
    validate_rx_packet_capacity::<RX_PACKET_CAPACITY>(function)?;

    let control_info = EndpointInfo {
        addr: EndpointAddress::from_parts(0, Direction::In),
        ep_type: EndpointType::Control,
        max_packet_size: control_max_packet_size,
        interval_ms: 0,
    };
    let bulk_in_info = endpoint_info(function.bulk_in_endpoint);
    let bulk_out_info = endpoint_info(function.bulk_out_endpoint);

    let control =
        allocator.alloc_pipe::<pipe::Control, pipe::InOut>(device_address, &control_info, split)?;
    let bulk_in =
        allocator.alloc_pipe::<pipe::Bulk, pipe::In>(device_address, &bulk_in_info, split)?;
    let bulk_out =
        allocator.alloc_pipe::<pipe::Bulk, pipe::Out>(device_address, &bulk_out_info, split)?;

    Ok(CdcAcmHost::from_validated_parts(
        function, control, bulk_in, bulk_out,
    ))
}

const fn validate_rx_packet_capacity<const RX_PACKET_CAPACITY: usize>(
    function: CdcAcmFunction,
) -> Result<(), CdcAcmCreateError> {
    if rx_packet_capacity_supports::<RX_PACKET_CAPACITY>(function) {
        Ok(())
    } else {
        Err(CdcAcmCreateError::UnsupportedBulkInMaxPacketSize)
    }
}

const fn rx_packet_capacity_supports<const RX_PACKET_CAPACITY: usize>(
    function: CdcAcmFunction,
) -> bool {
    let max_packet_size = function.bulk_in_endpoint.max_packet_size;
    matches!(max_packet_size, 8 | 16 | 32 | 64 | 512)
        && max_packet_size as usize <= RX_PACKET_CAPACITY
}

fn validate_allocation_inputs(
    device_address: u8,
    control_max_packet_size: u16,
) -> Result<(), CdcAcmCreateError> {
    if device_address > 0x7f {
        return Err(CdcAcmCreateError::InvalidDeviceAddress);
    }
    if !matches!(control_max_packet_size, 8 | 16 | 32 | 64) {
        return Err(CdcAcmCreateError::InvalidControlMaxPacketSize);
    }
    Ok(())
}

#[cfg(feature = "embassy-usb-host")]
/// Allocate this stricter CDC-ACM class directly from Embassy enumeration
/// output.
pub fn allocate_from_enumeration<'d, A>(
    allocator: &A,
    configuration: &[u8],
    enumeration: &embassy_usb_host::handler::EnumerationInfo,
) -> Result<AllocatedCdcAcmHost<'d, A>, CdcAcmCreateError>
where
    A: UsbHostAllocator<'d>,
{
    allocate_from_enumeration_with_rx_capacity::<A, DEFAULT_RX_PACKET_CAPACITY>(
        allocator,
        configuration,
        enumeration,
    )
}

#[cfg(feature = "embassy-usb-host")]
/// Allocate this class directly from Embassy enumeration output with an
/// explicitly sized internal bulk-IN packet buffer.
pub fn allocate_from_enumeration_with_rx_capacity<'d, A, const RX_PACKET_CAPACITY: usize>(
    allocator: &A,
    configuration: &[u8],
    enumeration: &embassy_usb_host::handler::EnumerationInfo,
) -> Result<AllocatedCdcAcmHostWithRxCapacity<'d, A, RX_PACKET_CAPACITY>, CdcAcmCreateError>
where
    A: UsbHostAllocator<'d>,
{
    allocate_cdc_acm_host_with_rx_capacity::<A, RX_PACKET_CAPACITY>(
        allocator,
        configuration,
        enumeration.device_address,
        enumeration.device_desc.max_packet_size0 as u16,
        enumeration.split(),
    )
}

#[cfg(feature = "embassy-usb-host")]
/// Select a specific CDC-ACM function and allocate it directly from Embassy
/// enumeration output.
pub fn allocate_from_enumeration_for_control_interface<'d, A>(
    allocator: &A,
    configuration: &[u8],
    control_interface: u8,
    enumeration: &embassy_usb_host::handler::EnumerationInfo,
) -> Result<AllocatedCdcAcmHost<'d, A>, CdcAcmCreateError>
where
    A: UsbHostAllocator<'d>,
{
    allocate_from_enumeration_for_control_interface_with_rx_capacity::<A, DEFAULT_RX_PACKET_CAPACITY>(
        allocator,
        configuration,
        control_interface,
        enumeration,
    )
}

#[cfg(feature = "embassy-usb-host")]
/// Select a specific CDC-ACM function from Embassy enumeration output and
/// allocate it with an explicitly sized internal bulk-IN packet buffer.
pub fn allocate_from_enumeration_for_control_interface_with_rx_capacity<
    'd,
    A,
    const RX_PACKET_CAPACITY: usize,
>(
    allocator: &A,
    configuration: &[u8],
    control_interface: u8,
    enumeration: &embassy_usb_host::handler::EnumerationInfo,
) -> Result<AllocatedCdcAcmHostWithRxCapacity<'d, A, RX_PACKET_CAPACITY>, CdcAcmCreateError>
where
    A: UsbHostAllocator<'d>,
{
    allocate_cdc_acm_host_for_control_interface_with_rx_capacity::<A, RX_PACKET_CAPACITY>(
        allocator,
        configuration,
        control_interface,
        enumeration.device_address,
        enumeration.device_desc.max_packet_size0 as u16,
        enumeration.split(),
    )
}

fn endpoint_info(endpoint: crate::usb::EndpointDescriptor) -> EndpointInfo {
    EndpointInfo {
        addr: EndpointAddress::from(endpoint.address),
        ep_type: EndpointType::Bulk,
        max_packet_size: endpoint.max_packet_size,
        interval_ms: endpoint.interval,
    }
}

/// Error returned by the generic CDC-ACM host.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CdcAcmError {
    /// The underlying host pipe failed a transfer.
    Transfer(PipeError),
    /// The ACM functional descriptor does not advertise line requests.
    LineRequestsUnsupported,
    /// The ACM functional descriptor does not advertise SEND_BREAK.
    SendBreakUnsupported,
    /// A locally supplied or device-returned line coding is invalid.
    InvalidLineCoding(CdcLineCodingError),
}

impl From<PipeError> for CdcAcmError {
    fn from(error: PipeError) -> Self {
        Self::Transfer(error)
    }
}

impl From<CdcLineCodingError> for CdcAcmError {
    fn from(error: CdcLineCodingError) -> Self {
        Self::InvalidLineCoding(error)
    }
}

impl core::fmt::Display for CdcAcmError {
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Transfer(_) => formatter.write_str("CDC-ACM USB transfer failed"),
            Self::LineRequestsUnsupported => {
                formatter.write_str("CDC-ACM line requests are not supported")
            }
            Self::SendBreakUnsupported => {
                formatter.write_str("CDC-ACM SEND_BREAK is not supported")
            }
            Self::InvalidLineCoding(_) => formatter.write_str("invalid CDC-ACM line coding"),
        }
    }
}

impl core::error::Error for CdcAcmError {}

impl embedded_io_async::Error for CdcAcmError {
    fn kind(&self) -> embedded_io_async::ErrorKind {
        match self {
            Self::Transfer(PipeError::Disconnected) => embedded_io_async::ErrorKind::NotConnected,
            Self::Transfer(PipeError::Timeout) => embedded_io_async::ErrorKind::TimedOut,
            Self::Transfer(PipeError::BufferOverflow) => embedded_io_async::ErrorKind::OutOfMemory,
            _ => embedded_io_async::ErrorKind::Other,
        }
    }
}

/// A configured CDC-ACM function backed by three Embassy host pipes.
///
/// The caller allocates the pipes from the descriptor-selected endpoint zero,
/// bulk-IN and bulk-OUT endpoint metadata before constructing this value.
pub struct CdcAcmHost<C, I, O, const RX_PACKET_CAPACITY: usize = DEFAULT_RX_PACKET_CAPACITY> {
    function: CdcAcmFunction,
    control: C,
    bulk_in: I,
    bulk_out: O,
    rx_packet: [u8; RX_PACKET_CAPACITY],
    rx_start: usize,
    rx_end: usize,
}

impl<C, I, O> CdcAcmHost<C, I, O> {
    /// Bind already allocated pipes using the default 64-byte packet buffer.
    ///
    /// This source-compatible convenience constructor asserts that the
    /// descriptor-selected bulk-IN packet fits. New generic code that handles
    /// runtime descriptors directly should prefer [`Self::try_new`].
    pub const fn new(function: CdcAcmFunction, control: C, bulk_in: I, bulk_out: O) -> Self {
        assert!(
            rx_packet_capacity_supports::<DEFAULT_RX_PACKET_CAPACITY>(function),
            "CDC-ACM bulk-IN packet size is invalid or exceeds the default receive buffer"
        );
        Self::from_validated_parts(function, control, bulk_in, bulk_out)
    }
}

impl<C, I, O, const RX_PACKET_CAPACITY: usize> CdcAcmHost<C, I, O, RX_PACKET_CAPACITY> {
    /// Bind already allocated pipes after validating that the selected
    /// bulk-IN endpoint packet fits this class instance's receive buffer.
    pub fn try_new(
        function: CdcAcmFunction,
        control: C,
        bulk_in: I,
        bulk_out: O,
    ) -> Result<Self, CdcAcmCreateError> {
        match validate_rx_packet_capacity::<RX_PACKET_CAPACITY>(function) {
            Ok(()) => Ok(Self::from_validated_parts(
                function, control, bulk_in, bulk_out,
            )),
            Err(error) => Err(error),
        }
    }

    const fn from_validated_parts(
        function: CdcAcmFunction,
        control: C,
        bulk_in: I,
        bulk_out: O,
    ) -> Self {
        Self {
            function,
            control,
            bulk_in,
            bulk_out,
            rx_packet: [0; RX_PACKET_CAPACITY],
            rx_start: 0,
            rx_end: 0,
        }
    }

    /// Internal receive-packet capacity selected for this class instance.
    pub const fn rx_packet_capacity(&self) -> usize {
        RX_PACKET_CAPACITY
    }
}

impl<C, I, O, const RX_PACKET_CAPACITY: usize> CdcAcmHost<C, I, O, RX_PACKET_CAPACITY>
where
    C: UsbPipe<pipe::Control, pipe::InOut>,
    I: UsbPipe<pipe::Bulk, pipe::In>,
    O: UsbPipe<pipe::Bulk, pipe::Out>,
{
    /// The strictly descriptor-validated CDC-ACM function.
    pub const fn function(&self) -> CdcAcmFunction {
        self.function
    }

    /// Configure endpoint-zero transfer deadlines.
    pub fn set_control_timeout(&mut self, timeout: TimeoutConfig) {
        self.control.set_timeout(timeout);
    }

    /// Reset both non-control host-side DATA toggles to DATA0.
    ///
    /// This is normally called immediately after the device configuration or
    /// data-interface alternate setting changes.
    pub fn reset_data_toggles(&mut self) {
        self.bulk_in.reset_data_toggle();
        self.bulk_out.reset_data_toggle();
        self.rx_start = 0;
        self.rx_end = 0;
    }

    /// Set baud rate, stop bits, parity and data bits.
    pub async fn set_line_coding(&mut self, coding: CdcLineCoding) -> Result<(), CdcAcmError> {
        self.require_line_requests()?;
        coding.validate()?;
        let setup = SetupRequest::set_line_coding(self.function.control_interface).to_bytes();
        let data = coding.to_bytes();
        self.control.control_out(&setup, &data).await?;
        Ok(())
    }

    /// Read the line coding currently selected by the device.
    pub async fn get_line_coding(&mut self) -> Result<CdcLineCoding, CdcAcmError> {
        self.require_line_requests()?;
        let setup = SetupRequest::get_line_coding(self.function.control_interface).to_bytes();
        let mut data = [0_u8; CdcLineCoding::ENCODED_LEN];
        let received = self.control.control_in(&setup, &mut data).await?;
        if received != data.len() {
            return Err(CdcLineCodingError::InvalidLength.into());
        }
        Ok(CdcLineCoding::from_bytes(&data)?)
    }

    /// Set the CDC DTR and RTS control signals.
    pub async fn set_control_line_state(
        &mut self,
        dtr: bool,
        rts: bool,
    ) -> Result<(), CdcAcmError> {
        self.require_line_requests()?;
        let setup = SetupRequest::set_control_line_state(self.function.control_interface, dtr, rts)
            .to_bytes();
        self.control.control_out(&setup, &[]).await?;
        Ok(())
    }

    /// Start, stop or time a serial break condition.
    ///
    /// `duration_ms == 0` stops a break and `u16::MAX` requests an
    /// indefinite break, as defined by CDC PSTN.
    pub async fn send_break(&mut self, duration_ms: u16) -> Result<(), CdcAcmError> {
        if !self.function.supports_send_break() {
            return Err(CdcAcmError::SendBreakUnsupported);
        }
        let setup =
            SetupRequest::send_break(self.function.control_interface, duration_ms).to_bytes();
        self.control.control_out(&setup, &[]).await?;
        Ok(())
    }

    /// Read bytes from the CDC stream.
    ///
    /// USB bulk packets are received into the configured internal packet
    /// buffer, so callers may supply a destination smaller than the endpoint
    /// maximum packet size. Unconsumed packet bytes remain available to the
    /// next call. USB zero-length packets are transfer delimiters, not stream
    /// EOF, and are skipped. An empty destination succeeds without issuing a
    /// USB transaction or consuming buffered data.
    pub async fn read(&mut self, buffer: &mut [u8]) -> Result<usize, CdcAcmError> {
        if buffer.is_empty() {
            return Ok(0);
        }

        if self.rx_start != self.rx_end {
            return Ok(self.copy_buffered_read(buffer));
        }

        loop {
            let packet_size = usize::from(self.function.bulk_in_endpoint.max_packet_size);
            if packet_size > self.rx_packet.len() {
                return Err(PipeError::BufferOverflow.into());
            }
            let received = self
                .bulk_in
                .request_in(&mut self.rx_packet[..packet_size])
                .await?;
            if received > packet_size {
                return Err(PipeError::BufferOverflow.into());
            }
            if received == 0 {
                continue;
            }

            self.rx_start = 0;
            self.rx_end = received;
            return Ok(self.copy_buffered_read(buffer));
        }
    }

    /// Write an arbitrary byte sequence to the CDC bulk-OUT endpoint.
    ///
    /// Packetization is delegated to `UsbPipe::request_out`. CDC is a byte
    /// stream, so this does not request an extra terminating ZLP for a full
    /// final packet. An empty source succeeds without issuing a USB transfer.
    pub async fn write(&mut self, data: &[u8]) -> Result<usize, CdcAcmError> {
        if data.is_empty() {
            return Ok(0);
        }
        self.bulk_out.request_out(data, false).await?;
        Ok(data.len())
    }

    /// Borrow the endpoint-zero pipe.
    pub const fn control_pipe(&self) -> &C {
        &self.control
    }

    /// Borrow the bulk-IN pipe.
    pub const fn bulk_in_pipe(&self) -> &I {
        &self.bulk_in
    }

    /// Borrow the bulk-OUT pipe.
    pub const fn bulk_out_pipe(&self) -> &O {
        &self.bulk_out
    }

    /// Recover the function description and all three owned pipes.
    ///
    /// Any bytes already received into the class driver's internal stream
    /// buffer are discarded.
    pub fn into_parts(self) -> (CdcAcmFunction, C, I, O) {
        (self.function, self.control, self.bulk_in, self.bulk_out)
    }

    fn copy_buffered_read(&mut self, buffer: &mut [u8]) -> usize {
        let count = buffer.len().min(self.rx_end - self.rx_start);
        buffer[..count].copy_from_slice(&self.rx_packet[self.rx_start..self.rx_start + count]);
        self.rx_start += count;
        if self.rx_start == self.rx_end {
            self.rx_start = 0;
            self.rx_end = 0;
        }
        count
    }

    fn require_line_requests(&self) -> Result<(), CdcAcmError> {
        if self.function.supports_line_requests() {
            Ok(())
        } else {
            Err(CdcAcmError::LineRequestsUnsupported)
        }
    }
}

impl<C, I, O, const RX_PACKET_CAPACITY: usize> embedded_io_async::ErrorType
    for CdcAcmHost<C, I, O, RX_PACKET_CAPACITY>
where
    C: UsbPipe<pipe::Control, pipe::InOut>,
    I: UsbPipe<pipe::Bulk, pipe::In>,
    O: UsbPipe<pipe::Bulk, pipe::Out>,
{
    type Error = CdcAcmError;
}

impl<C, I, O, const RX_PACKET_CAPACITY: usize> embedded_io_async::Read
    for CdcAcmHost<C, I, O, RX_PACKET_CAPACITY>
where
    C: UsbPipe<pipe::Control, pipe::InOut>,
    I: UsbPipe<pipe::Bulk, pipe::In>,
    O: UsbPipe<pipe::Bulk, pipe::Out>,
{
    async fn read(&mut self, buffer: &mut [u8]) -> Result<usize, Self::Error> {
        CdcAcmHost::read(self, buffer).await
    }
}

impl<C, I, O, const RX_PACKET_CAPACITY: usize> embedded_io_async::Write
    for CdcAcmHost<C, I, O, RX_PACKET_CAPACITY>
where
    C: UsbPipe<pipe::Control, pipe::InOut>,
    I: UsbPipe<pipe::Bulk, pipe::In>,
    O: UsbPipe<pipe::Bulk, pipe::Out>,
{
    async fn write(&mut self, data: &[u8]) -> Result<usize, Self::Error> {
        CdcAcmHost::write(self, data).await
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use core::future::Future;
    use core::pin::pin;
    use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};
    use core::time::Duration;

    use super::*;
    use crate::host::SplitSpeed;
    use crate::usb::{
        CDC_ACM_CAPABILITY_LINE_REQUESTS, CDC_ACM_CAPABILITY_SEND_BREAK, CdcParity, CdcStopBits,
        ConfigurationDescriptorHeader, EndpointDescriptor,
    };

    const MAX_FAKE_BYTES: usize = HIGH_SPEED_RX_PACKET_CAPACITY;
    const CDC_ACM_CONFIGURATION: [u8; 75] = [
        0x09, 0x02, 0x4b, 0x00, 0x02, 0x01, 0x00, 0x80, 0x32, // Configuration.
        0x08, 0x0b, 0x00, 0x02, 0x02, 0x02, 0x01, 0x00, // IAD.
        0x09, 0x04, 0x00, 0x00, 0x01, 0x02, 0x02, 0x01, 0x00, // Control IF.
        0x05, 0x24, 0x00, 0x10, 0x01, // CDC 1.10 header.
        0x05, 0x24, 0x01, 0x00, 0x01, // Call management.
        0x04, 0x24, 0x02, 0x06, // ACM capabilities.
        0x05, 0x24, 0x06, 0x00, 0x01, // Union: control 0, data 1.
        0x07, 0x05, 0x81, 0x03, 0x08, 0x00, 0x10, // Notification IN.
        0x09, 0x04, 0x01, 0x00, 0x02, 0x0a, 0x00, 0x00, 0x00, // Data IF.
        0x07, 0x05, 0x02, 0x02, 0x40, 0x00, 0x00, // Bulk OUT.
        0x07, 0x05, 0x82, 0x02, 0x40, 0x00, 0x00, // Bulk IN.
    ];

    const DUAL_CDC_ACM_CONFIGURATION: [u8; 102] = [
        0x09, 0x02, 0x66, 0x00, 0x04, 0x01, 0x00, 0x80, 0x32, // Configuration.
        0x09, 0x04, 0x00, 0x00, 0x00, 0x02, 0x02, 0x01, 0x00, // Control IF 0.
        0x05, 0x24, 0x00, 0x10, 0x01, // CDC header.
        0x04, 0x24, 0x02, 0x02, // ACM.
        0x06, 0x24, 0x06, 0x00, 0x02, 0x01, // Union: master 0; subs 2, 1.
        0x09, 0x04, 0x01, 0x00, 0x02, 0x0a, 0x00, 0x00, 0x00, // Data IF 1.
        0x07, 0x05, 0x01, 0x02, 0x40, 0x00, 0x00, // Bulk OUT 1.
        0x07, 0x05, 0x81, 0x02, 0x40, 0x00, 0x00, // Bulk IN 1.
        0x09, 0x04, 0x02, 0x00, 0x00, 0x02, 0x02, 0x01, 0x00, // Control IF 2.
        0x05, 0x24, 0x00, 0x10, 0x01, // CDC header.
        0x04, 0x24, 0x02, 0x06, // ACM.
        0x05, 0x24, 0x06, 0x02, 0x03, // Union: control 2, data 3.
        0x09, 0x04, 0x03, 0x00, 0x02, 0x0a, 0x00, 0x00, 0x00, // Data IF 3.
        0x07, 0x05, 0x03, 0x02, 0x40, 0x00, 0x00, // Bulk OUT 3.
        0x07, 0x05, 0x83, 0x02, 0x40, 0x00, 0x00, // Bulk IN 3.
    ];

    #[derive(Clone)]
    struct FakeAllocator;

    struct FakeAllocatedPipe<T: pipe::Type, D: pipe::Direction> {
        address: u8,
        endpoint: EndpointInfo,
        split: Option<SplitInfo>,
        _marker: core::marker::PhantomData<(T, D)>,
    }

    impl<'d> UsbHostAllocator<'d> for FakeAllocator {
        type Pipe<T: pipe::Type, D: pipe::Direction> = FakeAllocatedPipe<T, D>;

        fn alloc_pipe<T: pipe::Type, D: pipe::Direction>(
            &self,
            address: u8,
            endpoint: &EndpointInfo,
            split: Option<SplitInfo>,
        ) -> Result<Self::Pipe<T, D>, HostError> {
            Ok(FakeAllocatedPipe {
                address,
                endpoint: *endpoint,
                split,
                _marker: core::marker::PhantomData,
            })
        }
    }

    impl<T: pipe::Type, D: pipe::Direction> UsbPipe<T, D> for FakeAllocatedPipe<T, D> {
        async fn control_in(
            &mut self,
            _setup: &[u8; 8],
            _buffer: &mut [u8],
        ) -> Result<usize, PipeError>
        where
            T: pipe::IsControl,
            D: pipe::IsIn,
        {
            Ok(0)
        }

        async fn control_out(&mut self, _setup: &[u8; 8], _data: &[u8]) -> Result<(), PipeError>
        where
            T: pipe::IsControl,
            D: pipe::IsOut,
        {
            Ok(())
        }

        async fn request_in(&mut self, _buffer: &mut [u8]) -> Result<usize, PipeError>
        where
            D: pipe::IsIn,
        {
            Ok(0)
        }

        async fn request_out(
            &mut self,
            _data: &[u8],
            _ensure_transaction_end: bool,
        ) -> Result<(), PipeError>
        where
            D: pipe::IsOut,
        {
            Ok(())
        }

        fn set_timeout(&mut self, _timeout: TimeoutConfig)
        where
            T: pipe::IsControl,
        {
        }

        fn reset_data_toggle(&mut self)
        where
            T: pipe::IsBulkOrInterrupt,
        {
        }
    }

    struct FakeControl {
        timeout: TimeoutConfig,
        calls: usize,
        setup: [u8; 8],
        data: [u8; MAX_FAKE_BYTES],
        data_len: usize,
        in_data: [u8; CdcLineCoding::ENCODED_LEN],
        in_data_len: usize,
        fail: bool,
    }

    impl FakeControl {
        fn new() -> Self {
            Self {
                timeout: TimeoutConfig::default(),
                calls: 0,
                setup: [0; 8],
                data: [0; MAX_FAKE_BYTES],
                data_len: 0,
                in_data: CdcLineCoding::eight_n_one(115_200).to_bytes(),
                in_data_len: CdcLineCoding::ENCODED_LEN,
                fail: false,
            }
        }
    }

    impl UsbPipe<pipe::Control, pipe::InOut> for FakeControl {
        async fn control_in(
            &mut self,
            setup: &[u8; 8],
            buffer: &mut [u8],
        ) -> Result<usize, PipeError> {
            self.calls += 1;
            self.setup = *setup;
            if self.fail {
                return Err(PipeError::Timeout);
            }
            if self.in_data_len > buffer.len() {
                return Err(PipeError::BufferOverflow);
            }
            buffer[..self.in_data_len].copy_from_slice(&self.in_data[..self.in_data_len]);
            Ok(self.in_data_len)
        }

        async fn control_out(&mut self, setup: &[u8; 8], data: &[u8]) -> Result<(), PipeError> {
            self.calls += 1;
            self.setup = *setup;
            self.data_len = data.len();
            self.data[..data.len()].copy_from_slice(data);
            if self.fail {
                Err(PipeError::Timeout)
            } else {
                Ok(())
            }
        }

        async fn request_in(&mut self, _buffer: &mut [u8]) -> Result<usize, PipeError> {
            unreachable!("control pipe used as a non-control IN pipe")
        }

        async fn request_out(
            &mut self,
            _data: &[u8],
            _ensure_transaction_end: bool,
        ) -> Result<(), PipeError> {
            unreachable!("control pipe used as a non-control OUT pipe")
        }

        fn set_timeout(&mut self, timeout: TimeoutConfig) {
            self.timeout = timeout;
        }

        fn reset_data_toggle(&mut self) {
            unreachable!("control pipes do not expose a resettable data toggle")
        }
    }

    struct FakeBulkIn {
        calls: usize,
        resets: usize,
        zero_reads_before_data: usize,
        last_buffer_len: usize,
        response: [u8; MAX_FAKE_BYTES],
        response_len: usize,
        fail: bool,
    }

    impl FakeBulkIn {
        fn new(response: &[u8]) -> Self {
            let mut bytes = [0; MAX_FAKE_BYTES];
            bytes[..response.len()].copy_from_slice(response);
            Self {
                calls: 0,
                resets: 0,
                zero_reads_before_data: 0,
                last_buffer_len: 0,
                response: bytes,
                response_len: response.len(),
                fail: false,
            }
        }
    }

    impl UsbPipe<pipe::Bulk, pipe::In> for FakeBulkIn {
        async fn control_in(
            &mut self,
            _setup: &[u8; 8],
            _buffer: &mut [u8],
        ) -> Result<usize, PipeError> {
            unreachable!("bulk pipe used for control-IN")
        }

        async fn control_out(&mut self, _setup: &[u8; 8], _data: &[u8]) -> Result<(), PipeError> {
            unreachable!("bulk pipe used for control-OUT")
        }

        async fn request_in(&mut self, buffer: &mut [u8]) -> Result<usize, PipeError> {
            self.calls += 1;
            self.last_buffer_len = buffer.len();
            if self.fail {
                return Err(PipeError::Timeout);
            }
            if self.zero_reads_before_data != 0 {
                self.zero_reads_before_data -= 1;
                return Ok(0);
            }
            if self.response_len > buffer.len() {
                return Err(PipeError::BufferOverflow);
            }
            buffer[..self.response_len].copy_from_slice(&self.response[..self.response_len]);
            Ok(self.response_len)
        }

        async fn request_out(
            &mut self,
            _data: &[u8],
            _ensure_transaction_end: bool,
        ) -> Result<(), PipeError> {
            unreachable!("bulk-IN pipe used for OUT")
        }

        fn set_timeout(&mut self, _timeout: TimeoutConfig) {
            unreachable!("bulk pipes do not use control timeouts")
        }

        fn reset_data_toggle(&mut self) {
            self.resets += 1;
        }
    }

    struct FakeBulkOut {
        calls: usize,
        resets: usize,
        data: [u8; MAX_FAKE_BYTES],
        data_len: usize,
        ensure_transaction_end: bool,
        fail: bool,
    }

    impl FakeBulkOut {
        fn new() -> Self {
            Self {
                calls: 0,
                resets: 0,
                data: [0; MAX_FAKE_BYTES],
                data_len: 0,
                ensure_transaction_end: false,
                fail: false,
            }
        }
    }

    impl UsbPipe<pipe::Bulk, pipe::Out> for FakeBulkOut {
        async fn control_in(
            &mut self,
            _setup: &[u8; 8],
            _buffer: &mut [u8],
        ) -> Result<usize, PipeError> {
            unreachable!("bulk pipe used for control-IN")
        }

        async fn control_out(&mut self, _setup: &[u8; 8], _data: &[u8]) -> Result<(), PipeError> {
            unreachable!("bulk pipe used for control-OUT")
        }

        async fn request_in(&mut self, _buffer: &mut [u8]) -> Result<usize, PipeError> {
            unreachable!("bulk-OUT pipe used for IN")
        }

        async fn request_out(
            &mut self,
            data: &[u8],
            ensure_transaction_end: bool,
        ) -> Result<(), PipeError> {
            self.calls += 1;
            self.data_len = data.len();
            self.data[..data.len()].copy_from_slice(data);
            self.ensure_transaction_end = ensure_transaction_end;
            if self.fail {
                Err(PipeError::Timeout)
            } else {
                Ok(())
            }
        }

        fn set_timeout(&mut self, _timeout: TimeoutConfig) {
            unreachable!("bulk pipes do not use control timeouts")
        }

        fn reset_data_toggle(&mut self) {
            self.resets += 1;
        }
    }

    fn function(capabilities: u8) -> CdcAcmFunction {
        CdcAcmFunction {
            configuration: ConfigurationDescriptorHeader {
                total_length: 67,
                num_interfaces: 2,
                configuration_value: 1,
                string_index: 0,
                attributes: 0x80,
                max_power_ma: 100,
            },
            control_interface: 3,
            data_interface: 4,
            cdc_version_bcd: 0x0110,
            acm_capabilities: capabilities,
            notification_endpoint: None,
            bulk_out_endpoint: EndpointDescriptor {
                address: 0x02,
                attributes: 0x02,
                max_packet_size: 64,
                interval: 0,
            },
            bulk_in_endpoint: EndpointDescriptor {
                address: 0x81,
                attributes: 0x02,
                max_packet_size: 64,
                interval: 0,
            },
        }
    }

    fn host() -> CdcAcmHost<FakeControl, FakeBulkIn, FakeBulkOut> {
        CdcAcmHost::new(
            function(CDC_ACM_CAPABILITY_LINE_REQUESTS | CDC_ACM_CAPABILITY_SEND_BREAK),
            FakeControl::new(),
            FakeBulkIn::new(b"response"),
            FakeBulkOut::new(),
        )
    }

    fn block_on<F: Future>(future: F) -> F::Output {
        const VTABLE: RawWakerVTable = RawWakerVTable::new(|_| raw_waker(), |_| {}, |_| {}, |_| {});

        const fn raw_waker() -> RawWaker {
            RawWaker::new(core::ptr::null(), &VTABLE)
        }

        // SAFETY: the no-op vtable never dereferences the null data pointer.
        let waker = unsafe { Waker::from_raw(raw_waker()) };
        let mut context = Context::from_waker(&waker);
        let mut future = pin!(future);
        match future.as_mut().poll(&mut context) {
            Poll::Ready(output) => output,
            Poll::Pending => panic!("fake pipe futures must complete immediately"),
        }
    }

    #[test]
    fn constructor_retains_strictly_discovered_function() {
        let host = host();
        assert_eq!(host.function().control_interface, 3);
        assert_eq!(host.function().bulk_in_endpoint.address, 0x81);
        assert_eq!(host.function().bulk_out_endpoint.address, 0x02);
    }

    #[test]
    fn allocator_constructor_uses_descriptor_selected_endpoints() {
        let host =
            allocate_cdc_acm_host(&FakeAllocator, &CDC_ACM_CONFIGURATION, 11, 8, None).unwrap();

        assert_eq!(host.function().control_interface, 0);
        assert_eq!(host.function().data_interface, 1);
        assert_eq!(host.control_pipe().address, 11);
        assert_eq!(host.control_pipe().endpoint.ep_type, EndpointType::Control);
        assert_eq!(host.control_pipe().endpoint.addr.index(), 0);
        assert_eq!(host.control_pipe().endpoint.max_packet_size, 8);
        assert_eq!(
            host.bulk_in_pipe().endpoint.addr,
            EndpointAddress::from(0x82)
        );
        assert_eq!(host.bulk_in_pipe().endpoint.ep_type, EndpointType::Bulk);
        assert_eq!(host.bulk_in_pipe().endpoint.max_packet_size, 64);
        assert_eq!(
            host.bulk_out_pipe().endpoint.addr,
            EndpointAddress::from(0x02)
        );
        assert_eq!(host.bulk_out_pipe().endpoint.max_packet_size, 64);
        assert_eq!(host.control_pipe().split, None);
        assert_eq!(host.bulk_in_pipe().split, None);
        assert_eq!(host.bulk_out_pipe().split, None);
    }

    #[test]
    fn allocator_can_select_one_function_from_a_composite_device() {
        let mut broken_first = DUAL_CDC_ACM_CONFIGURATION;
        broken_first[26] = 0x12;

        let host = allocate_cdc_acm_host_for_control_interface(
            &FakeAllocator,
            &broken_first,
            2,
            11,
            8,
            None,
        )
        .unwrap();

        assert_eq!(host.function().control_interface, 2);
        assert_eq!(host.function().data_interface, 3);
        assert_eq!(
            host.bulk_in_pipe().endpoint.addr,
            EndpointAddress::from(0x83)
        );
        assert_eq!(
            host.bulk_out_pipe().endpoint.addr,
            EndpointAddress::from(0x03)
        );
    }

    #[test]
    fn default_allocator_rejects_bulk_in_packets_larger_than_its_buffer() {
        let mut high_speed = function(CDC_ACM_CAPABILITY_LINE_REQUESTS);
        high_speed.bulk_in_endpoint.max_packet_size = 512;

        assert!(matches!(
            allocate_cdc_acm_function(&FakeAllocator, high_speed, 1, 64, None),
            Err(CdcAcmCreateError::UnsupportedBulkInMaxPacketSize)
        ));
    }

    #[test]
    fn fallible_constructor_validates_receive_packet_capacity() {
        let mut invalid = function(CDC_ACM_CAPABILITY_LINE_REQUESTS);
        invalid.bulk_in_endpoint.max_packet_size = 7;
        assert!(matches!(
            CdcAcmHost::<FakeControl, FakeBulkIn, FakeBulkOut>::try_new(
                invalid,
                FakeControl::new(),
                FakeBulkIn::new(b""),
                FakeBulkOut::new(),
            ),
            Err(CdcAcmCreateError::UnsupportedBulkInMaxPacketSize)
        ));

        let mut high_speed = function(CDC_ACM_CAPABILITY_LINE_REQUESTS);
        high_speed.bulk_in_endpoint.max_packet_size = 512;
        high_speed.bulk_out_endpoint.max_packet_size = 512;

        assert!(matches!(
            CdcAcmHost::<FakeControl, FakeBulkIn, FakeBulkOut>::try_new(
                high_speed,
                FakeControl::new(),
                FakeBulkIn::new(b""),
                FakeBulkOut::new(),
            ),
            Err(CdcAcmCreateError::UnsupportedBulkInMaxPacketSize)
        ));

        let host = CdcAcmHost::<
            FakeControl,
            FakeBulkIn,
            FakeBulkOut,
            HIGH_SPEED_RX_PACKET_CAPACITY,
        >::try_new(
            high_speed,
            FakeControl::new(),
            FakeBulkIn::new(b""),
            FakeBulkOut::new(),
        )
        .unwrap();
        assert_eq!(host.rx_packet_capacity(), HIGH_SPEED_RX_PACKET_CAPACITY);
    }

    #[test]
    fn explicit_high_speed_allocator_accepts_a_512_byte_bulk_endpoint() {
        let mut high_speed = function(CDC_ACM_CAPABILITY_LINE_REQUESTS);
        high_speed.bulk_in_endpoint.max_packet_size = 512;
        high_speed.bulk_out_endpoint.max_packet_size = 512;

        let host = allocate_cdc_acm_function_with_rx_capacity::<_, HIGH_SPEED_RX_PACKET_CAPACITY>(
            &FakeAllocator,
            high_speed,
            1,
            64,
            None,
        )
        .unwrap();

        assert_eq!(host.rx_packet_capacity(), HIGH_SPEED_RX_PACKET_CAPACITY);
        assert_eq!(host.bulk_in_pipe().endpoint.max_packet_size, 512);
        assert_eq!(host.bulk_out_pipe().endpoint.max_packet_size, 512);
    }

    #[test]
    fn allocator_constructor_validates_ep0_and_configuration() {
        assert!(
            allocate_cdc_acm_host(&FakeAllocator, &CDC_ACM_CONFIGURATION, 0x7f, 8, None).is_ok()
        );
        assert!(matches!(
            allocate_cdc_acm_host(&FakeAllocator, &CDC_ACM_CONFIGURATION, 0x80, 8, None),
            Err(CdcAcmCreateError::InvalidDeviceAddress)
        ));
        assert!(matches!(
            allocate_cdc_acm_host(&FakeAllocator, &CDC_ACM_CONFIGURATION, 1, 7, None),
            Err(CdcAcmCreateError::InvalidControlMaxPacketSize)
        ));
        assert!(matches!(
            allocate_cdc_acm_host(&FakeAllocator, &CDC_ACM_CONFIGURATION[..9], 1, 8, None),
            Err(CdcAcmCreateError::Configuration(
                ConfigurationError::DescriptorOverrun
            ))
        ));
    }

    #[test]
    fn allocator_constructor_propagates_split_to_every_pipe() {
        let split = SplitInfo::new(5, 2, SplitSpeed::Full);
        let host =
            allocate_cdc_acm_host(&FakeAllocator, &CDC_ACM_CONFIGURATION, 11, 8, Some(split))
                .unwrap();

        assert_eq!(host.control_pipe().split, Some(split));
        assert_eq!(host.bulk_in_pipe().split, Some(split));
        assert_eq!(host.bulk_out_pipe().split, Some(split));
    }

    #[test]
    fn line_coding_uses_the_control_interface_and_exact_payload() {
        let mut host = host();
        assert_eq!(
            block_on(host.set_line_coding(CdcLineCoding::eight_n_one(115_200))),
            Ok(())
        );

        let control = host.control_pipe();
        assert_eq!(control.calls, 1);
        assert_eq!(control.setup, [0x21, 0x20, 0, 0, 3, 0, 7, 0]);
        assert_eq!(
            &control.data[..control.data_len],
            &[0x00, 0xc2, 0x01, 0x00, 0, 0, 8]
        );
    }

    #[test]
    fn invalid_local_line_coding_does_not_touch_the_control_pipe() {
        let mut host = host();
        let invalid = CdcLineCoding::new(115_200, CdcStopBits::One, CdcParity::None, 9);

        assert_eq!(
            block_on(host.set_line_coding(invalid)),
            Err(CdcAcmError::InvalidLineCoding(
                CdcLineCodingError::InvalidDataBits
            ))
        );
        assert_eq!(host.control_pipe().calls, 0);
    }

    #[test]
    fn control_line_state_serializes_dtr_and_rts_without_data() {
        let mut host = host();
        assert_eq!(block_on(host.set_control_line_state(true, true)), Ok(()));

        let control = host.control_pipe();
        assert_eq!(control.calls, 1);
        assert_eq!(control.setup, [0x21, 0x22, 3, 0, 3, 0, 0, 0]);
        assert_eq!(control.data_len, 0);
    }

    #[test]
    fn get_line_coding_uses_control_in_and_validates_the_response() {
        let mut host = host();
        assert_eq!(
            block_on(host.get_line_coding()),
            Ok(CdcLineCoding::eight_n_one(115_200))
        );
        assert_eq!(host.control_pipe().setup, [0xa1, 0x21, 0, 0, 3, 0, 7, 0]);

        host.control.in_data_len = 6;
        assert_eq!(
            block_on(host.get_line_coding()),
            Err(CdcAcmError::InvalidLineCoding(
                CdcLineCodingError::InvalidLength
            ))
        );

        host.control.in_data_len = 7;
        host.control.in_data[5] = 5;
        assert_eq!(
            block_on(host.get_line_coding()),
            Err(CdcAcmError::InvalidLineCoding(
                CdcLineCodingError::InvalidParity
            ))
        );

        host.control.in_data[5] = 0;
        host.control.in_data[6] = 9;
        assert_eq!(
            block_on(host.get_line_coding()),
            Err(CdcAcmError::InvalidLineCoding(
                CdcLineCodingError::InvalidDataBits
            ))
        );
    }

    #[test]
    fn send_break_checks_capability_and_serializes_duration() {
        let mut host = host();
        assert_eq!(block_on(host.send_break(250)), Ok(()));
        assert_eq!(host.control_pipe().setup, [0x21, 0x23, 0xfa, 0, 3, 0, 0, 0]);

        let mut unsupported = CdcAcmHost::new(
            function(CDC_ACM_CAPABILITY_LINE_REQUESTS),
            FakeControl::new(),
            FakeBulkIn::new(b""),
            FakeBulkOut::new(),
        );
        assert_eq!(
            block_on(unsupported.send_break(1)),
            Err(CdcAcmError::SendBreakUnsupported)
        );
        assert_eq!(unsupported.control_pipe().calls, 0);
    }

    #[test]
    fn unsupported_line_requests_do_not_touch_control_pipe() {
        let mut host = CdcAcmHost::new(
            function(0),
            FakeControl::new(),
            FakeBulkIn::new(b""),
            FakeBulkOut::new(),
        );

        assert_eq!(
            block_on(host.set_line_coding(CdcLineCoding::eight_n_one(9_600))),
            Err(CdcAcmError::LineRequestsUnsupported)
        );
        assert_eq!(
            block_on(host.set_control_line_state(true, false)),
            Err(CdcAcmError::LineRequestsUnsupported)
        );
        assert_eq!(host.control_pipe().calls, 0);
    }

    #[test]
    fn read_delegates_one_successful_transfer() {
        let mut host = host();
        let mut buffer = [0_u8; 32];
        assert_eq!(block_on(host.read(&mut buffer)), Ok(8));
        assert_eq!(&buffer[..8], b"response");
        assert_eq!(host.bulk_in_pipe().calls, 1);
    }

    #[test]
    fn read_requests_exactly_one_endpoint_packet_from_the_bulk_pipe() {
        let mut descriptor = function(CDC_ACM_CAPABILITY_LINE_REQUESTS);
        descriptor.bulk_in_endpoint.max_packet_size = 8;
        descriptor.bulk_out_endpoint.max_packet_size = 8;
        let mut host = CdcAcmHost::new(
            descriptor,
            FakeControl::new(),
            FakeBulkIn::new(b"12345678"),
            FakeBulkOut::new(),
        );
        let mut buffer = [0_u8; 16];

        assert_eq!(block_on(host.read(&mut buffer)), Ok(8));
        assert_eq!(&buffer[..8], b"12345678");
        assert_eq!(host.bulk_in_pipe().last_buffer_len, 8);
    }

    #[test]
    fn high_speed_read_buffers_and_replays_one_512_byte_packet() {
        let mut descriptor = function(CDC_ACM_CAPABILITY_LINE_REQUESTS);
        descriptor.bulk_in_endpoint.max_packet_size = 512;
        descriptor.bulk_out_endpoint.max_packet_size = 512;

        let mut response = [0_u8; HIGH_SPEED_RX_PACKET_CAPACITY];
        for (index, byte) in response.iter_mut().enumerate() {
            *byte = index as u8;
        }

        let mut host = CdcAcmHost::<
            FakeControl,
            FakeBulkIn,
            FakeBulkOut,
            HIGH_SPEED_RX_PACKET_CAPACITY,
        >::try_new(
            descriptor,
            FakeControl::new(),
            FakeBulkIn::new(&response),
            FakeBulkOut::new(),
        )
        .unwrap();
        let mut first = [0_u8; 300];
        let mut second = [0_u8; 300];

        assert_eq!(block_on(host.read(&mut first)), Ok(first.len()));
        assert_eq!(&first, &response[..first.len()]);
        assert_eq!(
            host.bulk_in_pipe().last_buffer_len,
            HIGH_SPEED_RX_PACKET_CAPACITY
        );
        assert_eq!(host.bulk_in_pipe().calls, 1);

        assert_eq!(
            block_on(host.read(&mut second)),
            Ok(HIGH_SPEED_RX_PACKET_CAPACITY - first.len())
        );
        assert_eq!(
            &second[..HIGH_SPEED_RX_PACKET_CAPACITY - first.len()],
            &response[first.len()..]
        );
        assert_eq!(host.bulk_in_pipe().calls, 1);
    }

    #[test]
    fn read_ignores_usb_zero_length_packets_instead_of_reporting_eof() {
        let mut host = host();
        host.bulk_in.zero_reads_before_data = 2;
        let mut buffer = [0_u8; 32];

        assert_eq!(block_on(host.read(&mut buffer)), Ok(8));
        assert_eq!(&buffer[..8], b"response");
        assert_eq!(host.bulk_in_pipe().calls, 3);
    }

    #[test]
    fn small_reads_retain_the_rest_of_one_usb_packet() {
        let mut host = host();
        let mut buffer = [0_u8; 3];

        assert_eq!(block_on(host.read(&mut buffer)), Ok(3));
        assert_eq!(&buffer, b"res");
        assert_eq!(host.bulk_in_pipe().calls, 1);

        assert_eq!(block_on(host.read(&mut buffer)), Ok(3));
        assert_eq!(&buffer, b"pon");
        assert_eq!(host.bulk_in_pipe().calls, 1);

        assert_eq!(block_on(host.read(&mut buffer)), Ok(2));
        assert_eq!(&buffer[..2], b"se");
        assert_eq!(host.bulk_in_pipe().calls, 1);
    }

    #[test]
    fn write_delegates_arbitrary_length_to_transport_transfer() {
        let mut host = host();
        let data = [0x5a; 200];
        assert_eq!(block_on(host.write(&data)), Ok(data.len()));

        let pipe = host.bulk_out_pipe();
        assert_eq!(pipe.calls, 1);
        assert_eq!(pipe.data_len, data.len());
        assert_eq!(&pipe.data[..pipe.data_len], &data);
        assert!(!pipe.ensure_transaction_end);
    }

    #[test]
    fn empty_read_and_write_are_no_ops() {
        let mut host = host();
        let mut empty = [];
        assert_eq!(block_on(host.read(&mut empty)), Ok(0));
        assert_eq!(block_on(host.write(&[])), Ok(0));
        assert_eq!(host.bulk_in_pipe().calls, 0);
        assert_eq!(host.bulk_out_pipe().calls, 0);
    }

    #[test]
    fn transport_errors_are_preserved() {
        let mut host = host();
        host.control.fail = true;
        assert_eq!(
            block_on(host.set_control_line_state(true, true)),
            Err(CdcAcmError::Transfer(PipeError::Timeout))
        );

        host.bulk_in.fail = true;
        let mut buffer = [0; 8];
        assert_eq!(
            block_on(host.read(&mut buffer)),
            Err(CdcAcmError::Transfer(PipeError::Timeout))
        );

        host.bulk_out.fail = true;
        assert_eq!(
            block_on(host.write(b"AT\r\n")),
            Err(CdcAcmError::Transfer(PipeError::Timeout))
        );
    }

    #[test]
    fn timeout_and_toggle_reset_are_delegated() {
        let mut host = host();
        let mut timeout = TimeoutConfig::default();
        timeout.data_timeout = Duration::from_secs(2);
        timeout.no_data_timeout = Duration::from_millis(250);
        host.set_control_timeout(timeout);
        host.reset_data_toggles();

        assert_eq!(host.control_pipe().timeout, timeout);
        assert_eq!(host.bulk_in_pipe().resets, 1);
        assert_eq!(host.bulk_out_pipe().resets, 1);
    }

    #[test]
    fn into_parts_returns_ownership_without_state_loss() {
        let mut host = host();
        block_on(host.write(b"AT\r\n")).unwrap();
        let (function, control, bulk_in, bulk_out) = host.into_parts();

        assert_eq!(function.control_interface, 3);
        assert_eq!(control.calls, 0);
        assert_eq!(bulk_in.calls, 0);
        assert_eq!(&bulk_out.data[..bulk_out.data_len], b"AT\r\n");
    }
}
