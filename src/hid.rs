//! Controller-independent raw USB HID host class.
//!
//! This module deliberately stops at HID transport. It discovers one HID
//! interface, owns endpoint zero plus its interrupt pipes, and exposes raw
//! reports and the standard HID class requests. Product report formats (for
//! example the Velleman K8055/P8055 eight-byte protocol) belong in a layer
//! above [`HidHost`].

use crate::host::{
    Direction, EndpointAddress, EndpointInfo, EndpointType, HostError, PipeError, SplitInfo,
    TimeoutConfig, UsbHostAllocator, UsbPipe, pipe,
};
use crate::usb::{HidConfigurationError, HidInterface, SetupRequest};

/// A [`HidHost`] whose pipes came from an Embassy host allocator.
pub type AllocatedHidHost<'d, A> = HidHost<
    <A as UsbHostAllocator<'d>>::Pipe<pipe::Control, pipe::InOut>,
    <A as UsbHostAllocator<'d>>::Pipe<pipe::Interrupt, pipe::In>,
    <A as UsbHostAllocator<'d>>::Pipe<pipe::Interrupt, pipe::Out>,
>;

/// HID report type used by GET_REPORT and SET_REPORT.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum HidReportType {
    Input = 1,
    Output = 2,
    Feature = 3,
}

/// Protocol selected for a boot-subclass HID interface.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum HidProtocol {
    Boot = 0,
    Report = 1,
}

impl TryFrom<u8> for HidProtocol {
    type Error = HidError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(Self::Boot),
            1 => Ok(Self::Report),
            _ => Err(HidError::InvalidProtocol),
        }
    }
}

/// Error while discovering a HID interface and allocating its pipes.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum HidCreateError {
    /// The configuration descriptor is malformed or has no usable HID
    /// interface.
    Configuration(HidConfigurationError),
    /// A USB device address is only seven bits wide.
    InvalidDeviceAddress,
    /// Endpoint zero has an invalid full-speed/low-speed maximum packet size.
    InvalidControlMaxPacketSize,
    /// The selected interrupt-IN endpoint has an unsupported packet size.
    UnsupportedInterruptInMaxPacketSize,
    /// The host controller could not allocate one of the required pipes.
    Allocation(HostError),
}

impl From<HidConfigurationError> for HidCreateError {
    fn from(error: HidConfigurationError) -> Self {
        Self::Configuration(error)
    }
}

impl From<HostError> for HidCreateError {
    fn from(error: HostError) -> Self {
        Self::Allocation(error)
    }
}

impl core::fmt::Display for HidCreateError {
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Configuration(_) => {
                formatter.write_str("invalid or unsupported HID configuration")
            }
            Self::InvalidDeviceAddress => formatter.write_str("invalid USB device address"),
            Self::InvalidControlMaxPacketSize => {
                formatter.write_str("invalid endpoint-zero maximum packet size")
            }
            Self::UnsupportedInterruptInMaxPacketSize => {
                formatter.write_str("invalid or unsupported HID interrupt-IN packet size")
            }
            Self::Allocation(_) => formatter.write_str("could not allocate HID host pipes"),
        }
    }
}

impl core::error::Error for HidCreateError {}

/// Discover the first valid HID interface and allocate its pipes.
pub fn allocate_hid_host<'d, A>(
    allocator: &A,
    configuration: &[u8],
    device_address: u8,
    control_max_packet_size: u16,
    split: Option<SplitInfo>,
) -> Result<AllocatedHidHost<'d, A>, HidCreateError>
where
    A: UsbHostAllocator<'d>,
{
    validate_allocation_inputs(device_address, control_max_packet_size)?;
    let interface = HidInterface::discover(configuration)?;
    allocate_hid_interface(
        allocator,
        interface,
        device_address,
        control_max_packet_size,
        split,
    )
}

/// Select a specific HID interface and allocate its pipes.
pub fn allocate_hid_host_for_interface<'d, A>(
    allocator: &A,
    configuration: &[u8],
    interface_number: u8,
    device_address: u8,
    control_max_packet_size: u16,
    split: Option<SplitInfo>,
) -> Result<AllocatedHidHost<'d, A>, HidCreateError>
where
    A: UsbHostAllocator<'d>,
{
    validate_allocation_inputs(device_address, control_max_packet_size)?;
    let interface = HidInterface::discover_interface(configuration, interface_number)?;
    allocate_hid_interface(
        allocator,
        interface,
        device_address,
        control_max_packet_size,
        split,
    )
}

/// Allocate pipes for an already selected and descriptor-validated HID
/// interface.
pub fn allocate_hid_interface<'d, A>(
    allocator: &A,
    interface: HidInterface,
    device_address: u8,
    control_max_packet_size: u16,
    split: Option<SplitInfo>,
) -> Result<AllocatedHidHost<'d, A>, HidCreateError>
where
    A: UsbHostAllocator<'d>,
{
    validate_allocation_inputs(device_address, control_max_packet_size)?;
    validate_interrupt_in(interface)?;

    let control_info = EndpointInfo {
        addr: EndpointAddress::from_parts(0, Direction::In),
        ep_type: EndpointType::Control,
        max_packet_size: control_max_packet_size,
        interval_ms: 0,
    };
    let interrupt_in_info = interrupt_endpoint_info(interface.interrupt_in_endpoint);

    let control =
        allocator.alloc_pipe::<pipe::Control, pipe::InOut>(device_address, &control_info, split)?;
    let interrupt_in = allocator.alloc_pipe::<pipe::Interrupt, pipe::In>(
        device_address,
        &interrupt_in_info,
        split,
    )?;
    let interrupt_out = match interface.interrupt_out_endpoint {
        Some(endpoint) => {
            let endpoint_info = interrupt_endpoint_info(endpoint);
            Some(allocator.alloc_pipe::<pipe::Interrupt, pipe::Out>(
                device_address,
                &endpoint_info,
                split,
            )?)
        }
        None => None,
    };

    Ok(HidHost::from_validated_parts(
        interface,
        control,
        interrupt_in,
        interrupt_out,
    ))
}

#[cfg(feature = "embassy-usb-host")]
/// Allocate the first valid HID interface directly from Embassy enumeration
/// output.
pub fn allocate_from_enumeration<'d, A>(
    allocator: &A,
    configuration: &[u8],
    enumeration: &embassy_usb_host::handler::EnumerationInfo,
) -> Result<AllocatedHidHost<'d, A>, HidCreateError>
where
    A: UsbHostAllocator<'d>,
{
    allocate_hid_host(
        allocator,
        configuration,
        enumeration.device_address,
        enumeration.device_desc.max_packet_size0 as u16,
        enumeration.split(),
    )
}

#[cfg(feature = "embassy-usb-host")]
/// Select and allocate one HID interface directly from Embassy enumeration
/// output.
pub fn allocate_from_enumeration_for_interface<'d, A>(
    allocator: &A,
    configuration: &[u8],
    interface_number: u8,
    enumeration: &embassy_usb_host::handler::EnumerationInfo,
) -> Result<AllocatedHidHost<'d, A>, HidCreateError>
where
    A: UsbHostAllocator<'d>,
{
    allocate_hid_host_for_interface(
        allocator,
        configuration,
        interface_number,
        enumeration.device_address,
        enumeration.device_desc.max_packet_size0 as u16,
        enumeration.split(),
    )
}

fn validate_allocation_inputs(
    device_address: u8,
    control_max_packet_size: u16,
) -> Result<(), HidCreateError> {
    if device_address > 0x7f {
        return Err(HidCreateError::InvalidDeviceAddress);
    }
    if !matches!(control_max_packet_size, 8 | 16 | 32 | 64) {
        return Err(HidCreateError::InvalidControlMaxPacketSize);
    }
    Ok(())
}

fn validate_interrupt_in(interface: HidInterface) -> Result<(), HidCreateError> {
    let max_packet_size = interface.interrupt_in_endpoint.max_packet_size;
    if max_packet_size == 0 || max_packet_size > 64 {
        Err(HidCreateError::UnsupportedInterruptInMaxPacketSize)
    } else {
        Ok(())
    }
}

fn interrupt_endpoint_info(endpoint: crate::usb::EndpointDescriptor) -> EndpointInfo {
    EndpointInfo {
        addr: EndpointAddress::from(endpoint.address),
        ep_type: EndpointType::Interrupt,
        max_packet_size: endpoint.max_packet_size,
        interval_ms: endpoint.interval,
    }
}

/// Error returned by the raw HID class driver.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum HidError {
    /// The underlying host pipe failed a transfer.
    Transfer(PipeError),
    /// The caller's input buffer cannot hold one complete endpoint packet.
    InputBufferTooSmall,
    /// The interface has no interrupt-OUT endpoint.
    InterruptOutUnavailable,
    /// An output report is longer than one interrupt-OUT endpoint packet.
    OutputReportTooLong,
    /// A control request payload is longer than USB's 16-bit length field.
    RequestTooLong,
    /// The report-descriptor buffer is shorter than the advertised length.
    ReportDescriptorBufferTooSmall,
    /// A fixed-length HID control response had an unexpected length.
    UnexpectedControlResponseLength,
    /// GET_PROTOCOL returned a value other than boot or report protocol.
    InvalidProtocol,
}

impl From<PipeError> for HidError {
    fn from(error: PipeError) -> Self {
        Self::Transfer(error)
    }
}

impl core::fmt::Display for HidError {
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Transfer(_) => formatter.write_str("HID USB transfer failed"),
            Self::InputBufferTooSmall => {
                formatter.write_str("HID input buffer is smaller than one endpoint packet")
            }
            Self::InterruptOutUnavailable => {
                formatter.write_str("HID interface has no interrupt-OUT endpoint")
            }
            Self::OutputReportTooLong => {
                formatter.write_str("HID output report exceeds one endpoint packet")
            }
            Self::RequestTooLong => formatter.write_str("HID control request is too long"),
            Self::ReportDescriptorBufferTooSmall => {
                formatter.write_str("HID report-descriptor buffer is too small")
            }
            Self::UnexpectedControlResponseLength => {
                formatter.write_str("unexpected HID control-response length")
            }
            Self::InvalidProtocol => formatter.write_str("invalid HID protocol value"),
        }
    }
}

impl core::error::Error for HidError {}

/// A configured raw HID interface backed by Embassy host pipes.
pub struct HidHost<C, I, O> {
    interface: HidInterface,
    control: C,
    interrupt_in: I,
    interrupt_out: Option<O>,
}

impl<C, I, O> HidHost<C, I, O> {
    /// Bind already allocated pipes after validating the interrupt-IN packet
    /// size.
    pub fn try_new(
        interface: HidInterface,
        control: C,
        interrupt_in: I,
        interrupt_out: Option<O>,
    ) -> Result<Self, HidCreateError> {
        validate_interrupt_in(interface)?;
        Ok(Self::from_validated_parts(
            interface,
            control,
            interrupt_in,
            interrupt_out,
        ))
    }

    const fn from_validated_parts(
        interface: HidInterface,
        control: C,
        interrupt_in: I,
        interrupt_out: Option<O>,
    ) -> Self {
        Self {
            interface,
            control,
            interrupt_in,
            interrupt_out,
        }
    }
}

impl<C, I, O> HidHost<C, I, O>
where
    C: UsbPipe<pipe::Control, pipe::InOut>,
    I: UsbPipe<pipe::Interrupt, pipe::In>,
    O: UsbPipe<pipe::Interrupt, pipe::Out>,
{
    /// Return the strictly descriptor-validated HID interface.
    pub const fn interface(&self) -> HidInterface {
        self.interface
    }

    /// Configure endpoint-zero transfer deadlines.
    pub fn set_control_timeout(&mut self, timeout: TimeoutConfig) {
        self.control.set_timeout(timeout);
    }

    /// Reset all allocated interrupt endpoint DATA toggles to DATA0.
    pub fn reset_data_toggles(&mut self) {
        self.interrupt_in.reset_data_toggle();
        if let Some(interrupt_out) = self.interrupt_out.as_mut() {
            interrupt_out.reset_data_toggle();
        }
    }

    /// Fetch the complete HID report descriptor.
    pub async fn get_report_descriptor<'a>(
        &mut self,
        buffer: &'a mut [u8],
    ) -> Result<&'a [u8], HidError> {
        let expected = usize::from(self.interface.report_descriptor_len);
        if buffer.len() < expected {
            return Err(HidError::ReportDescriptorBufferTooSmall);
        }
        let setup = SetupRequest::get_hid_report_descriptor(
            self.interface.interface_number,
            self.interface.report_descriptor_len,
        )
        .to_bytes();
        let received = self
            .control
            .control_in(&setup, &mut buffer[..expected])
            .await?;
        if received != expected {
            return Err(HidError::UnexpectedControlResponseLength);
        }
        Ok(&buffer[..received])
    }

    /// Read one raw input report from interrupt-IN.
    pub async fn read_input_report(&mut self, buffer: &mut [u8]) -> Result<usize, HidError> {
        let packet_size = usize::from(self.interface.interrupt_in_endpoint.max_packet_size);
        if buffer.len() < packet_size {
            return Err(HidError::InputBufferTooSmall);
        }
        let received = self
            .interrupt_in
            .request_in(&mut buffer[..packet_size])
            .await?;
        if received > packet_size {
            return Err(PipeError::BufferOverflow.into());
        }
        Ok(received)
    }

    /// Send one raw output report through interrupt-OUT.
    pub async fn write_output_report(&mut self, report: &[u8]) -> Result<(), HidError> {
        let endpoint = self
            .interface
            .interrupt_out_endpoint
            .ok_or(HidError::InterruptOutUnavailable)?;
        if report.len() > usize::from(endpoint.max_packet_size) {
            return Err(HidError::OutputReportTooLong);
        }
        let interrupt_out = self
            .interrupt_out
            .as_mut()
            .ok_or(HidError::InterruptOutUnavailable)?;
        interrupt_out.request_out(report, false).await?;
        Ok(())
    }

    /// Read an input, output or feature report through endpoint zero.
    pub async fn get_report(
        &mut self,
        report_type: HidReportType,
        report_id: u8,
        buffer: &mut [u8],
    ) -> Result<usize, HidError> {
        let length = u16::try_from(buffer.len()).map_err(|_| HidError::RequestTooLong)?;
        let setup = SetupRequest::get_hid_report(
            self.interface.interface_number,
            report_type as u8,
            report_id,
            length,
        )
        .to_bytes();
        Ok(self.control.control_in(&setup, buffer).await?)
    }

    /// Send an input, output or feature report through endpoint zero.
    pub async fn set_report(
        &mut self,
        report_type: HidReportType,
        report_id: u8,
        report: &[u8],
    ) -> Result<(), HidError> {
        let length = u16::try_from(report.len()).map_err(|_| HidError::RequestTooLong)?;
        let setup = SetupRequest::set_hid_report(
            self.interface.interface_number,
            report_type as u8,
            report_id,
            length,
        )
        .to_bytes();
        self.control.control_out(&setup, report).await?;
        Ok(())
    }

    /// Read the idle duration for one report ID, in units of four
    /// milliseconds.
    pub async fn get_idle(&mut self, report_id: u8) -> Result<u8, HidError> {
        let setup =
            SetupRequest::get_hid_idle(self.interface.interface_number, report_id).to_bytes();
        let mut response = [0_u8; 1];
        let received = self.control.control_in(&setup, &mut response).await?;
        if received != response.len() {
            return Err(HidError::UnexpectedControlResponseLength);
        }
        Ok(response[0])
    }

    /// Set the idle duration for one report ID, in units of four
    /// milliseconds. Report ID zero applies to all reports.
    pub async fn set_idle(&mut self, report_id: u8, duration_4ms: u8) -> Result<(), HidError> {
        let setup =
            SetupRequest::set_hid_idle(self.interface.interface_number, report_id, duration_4ms)
                .to_bytes();
        self.control.control_out(&setup, &[]).await?;
        Ok(())
    }

    /// Read the currently selected boot/report protocol.
    pub async fn get_protocol(&mut self) -> Result<HidProtocol, HidError> {
        let setup = SetupRequest::get_hid_protocol(self.interface.interface_number).to_bytes();
        let mut response = [0_u8; 1];
        let received = self.control.control_in(&setup, &mut response).await?;
        if received != response.len() {
            return Err(HidError::UnexpectedControlResponseLength);
        }
        HidProtocol::try_from(response[0])
    }

    /// Select boot or report protocol.
    pub async fn set_protocol(&mut self, protocol: HidProtocol) -> Result<(), HidError> {
        let setup = SetupRequest::set_hid_protocol(self.interface.interface_number, protocol as u8)
            .to_bytes();
        self.control.control_out(&setup, &[]).await?;
        Ok(())
    }

    /// Borrow the endpoint-zero pipe.
    pub const fn control_pipe(&self) -> &C {
        &self.control
    }

    /// Borrow the interrupt-IN pipe.
    pub const fn interrupt_in_pipe(&self) -> &I {
        &self.interrupt_in
    }

    /// Borrow the optional interrupt-OUT pipe.
    pub const fn interrupt_out_pipe(&self) -> Option<&O> {
        self.interrupt_out.as_ref()
    }

    /// Recover the interface description and all owned pipes.
    pub fn into_parts(self) -> (HidInterface, C, I, Option<O>) {
        (
            self.interface,
            self.control,
            self.interrupt_in,
            self.interrupt_out,
        )
    }
}

#[cfg(test)]
mod tests {
    use core::future::Future;
    use core::pin::pin;
    use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};

    use super::*;
    use crate::host::{SplitSpeed, pipe};
    const P8055_REPORT_DESCRIPTOR_LEN: usize = 29;
    const P8055_CONFIGURATION: [u8; 41] = [
        0x09, 0x02, 0x29, 0x00, 0x01, 0x01, 0x00, 0x80, 0x32, // Configuration.
        0x09, 0x04, 0x00, 0x00, 0x02, 0x03, 0x00, 0x00, 0x00, // HID interface.
        0x09, 0x21, 0x00, 0x01, 0x00, 0x01, 0x22, 0x1d, 0x00, // HID 1.00.
        0x07, 0x05, 0x81, 0x03, 0x08, 0x00, 0x0a, // Interrupt IN.
        0x07, 0x05, 0x01, 0x03, 0x08, 0x00, 0x0a, // Interrupt OUT.
    ];
    const WISPY1_REPORT_DESCRIPTOR_LEN: usize = 48;
    const WISPY1_CONFIGURATION: [u8; 34] = [
        0x09, 0x02, 0x22, 0x00, 0x01, 0x01, 0x03, 0x80, 0x31, // Configuration.
        0x09, 0x04, 0x00, 0x00, 0x01, 0x03, 0x01, 0x01, 0x00, // HID boot keyboard.
        0x09, 0x21, 0x11, 0x01, 0x00, 0x01, 0x22, 0x30, 0x00, // HID 1.11.
        0x07, 0x05, 0x81, 0x03, 0x08, 0x00, 0x0a, // Interrupt IN.
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
        out_data: [u8; 64],
        out_len: usize,
        in_data: [u8; 64],
        in_len: usize,
        fail: bool,
    }

    impl FakeControl {
        fn new(in_data: &[u8]) -> Self {
            let mut response = [0; 64];
            response[..in_data.len()].copy_from_slice(in_data);
            Self {
                timeout: TimeoutConfig::default(),
                calls: 0,
                setup: [0; 8],
                out_data: [0; 64],
                out_len: 0,
                in_data: response,
                in_len: in_data.len(),
                fail: false,
            }
        }

        fn set_response(&mut self, response: &[u8]) {
            self.in_data[..response.len()].copy_from_slice(response);
            self.in_len = response.len();
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
            if self.in_len > buffer.len() {
                return Err(PipeError::BufferOverflow);
            }
            buffer[..self.in_len].copy_from_slice(&self.in_data[..self.in_len]);
            Ok(self.in_len)
        }

        async fn control_out(&mut self, setup: &[u8; 8], data: &[u8]) -> Result<(), PipeError> {
            self.calls += 1;
            self.setup = *setup;
            self.out_len = data.len();
            self.out_data[..data.len()].copy_from_slice(data);
            if self.fail {
                Err(PipeError::Timeout)
            } else {
                Ok(())
            }
        }

        async fn request_in(&mut self, _buffer: &mut [u8]) -> Result<usize, PipeError> {
            unreachable!("control pipe used as interrupt-IN")
        }

        async fn request_out(
            &mut self,
            _data: &[u8],
            _ensure_transaction_end: bool,
        ) -> Result<(), PipeError> {
            unreachable!("control pipe used as interrupt-OUT")
        }

        fn set_timeout(&mut self, timeout: TimeoutConfig) {
            self.timeout = timeout;
        }

        fn reset_data_toggle(&mut self) {
            unreachable!("control pipe has no class data toggle")
        }
    }

    struct FakeInterruptIn {
        calls: usize,
        resets: usize,
        last_buffer_len: usize,
        report: [u8; 64],
        report_len: usize,
        fail: bool,
    }

    impl FakeInterruptIn {
        fn new(report: &[u8]) -> Self {
            let mut bytes = [0; 64];
            bytes[..report.len()].copy_from_slice(report);
            Self {
                calls: 0,
                resets: 0,
                last_buffer_len: 0,
                report: bytes,
                report_len: report.len(),
                fail: false,
            }
        }
    }

    impl UsbPipe<pipe::Interrupt, pipe::In> for FakeInterruptIn {
        async fn control_in(
            &mut self,
            _setup: &[u8; 8],
            _buffer: &mut [u8],
        ) -> Result<usize, PipeError> {
            unreachable!("interrupt-IN pipe used as control")
        }

        async fn control_out(&mut self, _setup: &[u8; 8], _data: &[u8]) -> Result<(), PipeError> {
            unreachable!("interrupt-IN pipe used as control")
        }

        async fn request_in(&mut self, buffer: &mut [u8]) -> Result<usize, PipeError> {
            self.calls += 1;
            self.last_buffer_len = buffer.len();
            if self.fail {
                return Err(PipeError::Timeout);
            }
            if self.report_len > buffer.len() {
                return Err(PipeError::BufferOverflow);
            }
            buffer[..self.report_len].copy_from_slice(&self.report[..self.report_len]);
            Ok(self.report_len)
        }

        async fn request_out(
            &mut self,
            _data: &[u8],
            _ensure_transaction_end: bool,
        ) -> Result<(), PipeError> {
            unreachable!("interrupt-IN pipe used for OUT")
        }

        fn set_timeout(&mut self, _timeout: TimeoutConfig) {
            unreachable!("interrupt pipe has no control timeout")
        }

        fn reset_data_toggle(&mut self) {
            self.resets += 1;
        }
    }

    struct FakeInterruptOut {
        calls: usize,
        resets: usize,
        report: [u8; 64],
        report_len: usize,
        ensure_transaction_end: bool,
        fail: bool,
    }

    impl FakeInterruptOut {
        fn new() -> Self {
            Self {
                calls: 0,
                resets: 0,
                report: [0; 64],
                report_len: 0,
                ensure_transaction_end: false,
                fail: false,
            }
        }
    }

    impl UsbPipe<pipe::Interrupt, pipe::Out> for FakeInterruptOut {
        async fn control_in(
            &mut self,
            _setup: &[u8; 8],
            _buffer: &mut [u8],
        ) -> Result<usize, PipeError> {
            unreachable!("interrupt-OUT pipe used as control")
        }

        async fn control_out(&mut self, _setup: &[u8; 8], _data: &[u8]) -> Result<(), PipeError> {
            unreachable!("interrupt-OUT pipe used as control")
        }

        async fn request_in(&mut self, _buffer: &mut [u8]) -> Result<usize, PipeError> {
            unreachable!("interrupt-OUT pipe used for IN")
        }

        async fn request_out(
            &mut self,
            data: &[u8],
            ensure_transaction_end: bool,
        ) -> Result<(), PipeError> {
            self.calls += 1;
            self.report_len = data.len();
            self.report[..data.len()].copy_from_slice(data);
            self.ensure_transaction_end = ensure_transaction_end;
            if self.fail {
                Err(PipeError::Timeout)
            } else {
                Ok(())
            }
        }

        fn set_timeout(&mut self, _timeout: TimeoutConfig) {
            unreachable!("interrupt pipe has no control timeout")
        }

        fn reset_data_toggle(&mut self) {
            self.resets += 1;
        }
    }

    fn p8055_interface() -> HidInterface {
        HidInterface::discover(&P8055_CONFIGURATION).unwrap()
    }

    fn host(
        control_response: &[u8],
        input_report: &[u8],
        with_out: bool,
    ) -> HidHost<FakeControl, FakeInterruptIn, FakeInterruptOut> {
        HidHost::try_new(
            p8055_interface(),
            FakeControl::new(control_response),
            FakeInterruptIn::new(input_report),
            with_out.then(FakeInterruptOut::new),
        )
        .unwrap()
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
    fn p8055_configuration_discovers_raw_hid_transport() {
        let interface = p8055_interface();
        assert_eq!(interface.interface_number, 0);
        assert_eq!(interface.interface_subclass, 0);
        assert_eq!(interface.interface_protocol, 0);
        assert_eq!(interface.hid_version_bcd, 0x0100);
        assert_eq!(
            usize::from(interface.report_descriptor_len),
            P8055_REPORT_DESCRIPTOR_LEN
        );
        assert_eq!(interface.interrupt_in_endpoint.address, 0x81);
        assert_eq!(interface.interrupt_in_endpoint.max_packet_size, 8);
        assert_eq!(interface.interrupt_in_endpoint.interval, 10);
        let output = interface.interrupt_out_endpoint.unwrap();
        assert_eq!(output.address, 0x01);
        assert_eq!(output.max_packet_size, 8);
        assert_eq!(output.interval, 10);
    }

    #[test]
    fn wispy1_configuration_discovers_feature_report_hid_without_interrupt_out() {
        let interface = HidInterface::discover(&WISPY1_CONFIGURATION).unwrap();
        assert_eq!(interface.interface_number, 0);
        assert_eq!(interface.interface_subclass, 1);
        assert_eq!(interface.interface_protocol, 1);
        assert_eq!(interface.hid_version_bcd, 0x0111);
        assert_eq!(
            usize::from(interface.report_descriptor_len),
            WISPY1_REPORT_DESCRIPTOR_LEN
        );
        assert_eq!(interface.interrupt_in_endpoint.address, 0x81);
        assert_eq!(interface.interrupt_in_endpoint.max_packet_size, 8);
        assert_eq!(interface.interrupt_in_endpoint.interval, 10);
        assert_eq!(interface.interrupt_out_endpoint, None);

        let split = SplitInfo::new(3, 1, SplitSpeed::Low);
        let host =
            allocate_hid_host(&FakeAllocator, &WISPY1_CONFIGURATION, 1, 8, Some(split)).unwrap();
        assert_eq!(host.control_pipe().endpoint.max_packet_size, 8);
        assert_eq!(host.interrupt_in_pipe().endpoint.interval_ms, 10);
        assert!(host.interrupt_out_pipe().is_none());
    }

    #[test]
    fn allocator_preserves_address_endpoints_interval_and_split() {
        let split = SplitInfo::new(5, 2, SplitSpeed::Low);
        let host =
            allocate_hid_host(&FakeAllocator, &P8055_CONFIGURATION, 7, 8, Some(split)).unwrap();

        assert_eq!(host.control_pipe().address, 7);
        assert_eq!(host.control_pipe().endpoint.addr.index(), 0);
        assert_eq!(host.control_pipe().endpoint.ep_type, EndpointType::Control);
        assert_eq!(host.control_pipe().endpoint.max_packet_size, 8);
        assert_eq!(host.control_pipe().split, Some(split));

        assert_eq!(host.interrupt_in_pipe().address, 7);
        assert_eq!(
            host.interrupt_in_pipe().endpoint.addr,
            EndpointAddress::from(0x81)
        );
        assert_eq!(
            host.interrupt_in_pipe().endpoint.ep_type,
            EndpointType::Interrupt
        );
        assert_eq!(host.interrupt_in_pipe().endpoint.max_packet_size, 8);
        assert_eq!(host.interrupt_in_pipe().endpoint.interval_ms, 10);
        assert_eq!(host.interrupt_in_pipe().split, Some(split));

        let output = host.interrupt_out_pipe().unwrap();
        assert_eq!(output.address, 7);
        assert_eq!(output.endpoint.addr, EndpointAddress::from(0x01));
        assert_eq!(output.endpoint.ep_type, EndpointType::Interrupt);
        assert_eq!(output.endpoint.max_packet_size, 8);
        assert_eq!(output.endpoint.interval_ms, 10);
        assert_eq!(output.split, Some(split));
    }

    #[test]
    fn input_reads_exactly_one_endpoint_packet_and_checks_capacity_first() {
        let input = [0x30, 0x01, 17, 29, 0x34, 0x12, 0x78, 0x56];
        let mut host = host(&[], &input, true);
        let mut short = [0; 7];
        assert_eq!(
            block_on(host.read_input_report(&mut short)),
            Err(HidError::InputBufferTooSmall)
        );
        assert_eq!(host.interrupt_in_pipe().calls, 0);

        let mut report = [0; 16];
        assert_eq!(block_on(host.read_input_report(&mut report)), Ok(8));
        assert_eq!(&report[..8], &input);
        assert_eq!(host.interrupt_in_pipe().last_buffer_len, 8);
        assert_eq!(host.interrupt_in_pipe().calls, 1);
    }

    #[test]
    fn output_is_one_raw_interrupt_packet_without_a_synthetic_report_id() {
        let mut host = host(&[], &[], true);
        let report = [5, 0x55, 1, 2, 0, 0, 0, 0];
        assert_eq!(block_on(host.write_output_report(&report)), Ok(()));

        let output = host.interrupt_out_pipe().unwrap();
        assert_eq!(output.calls, 1);
        assert_eq!(output.report_len, 8);
        assert_eq!(&output.report[..8], &report);
        assert!(!output.ensure_transaction_end);
    }

    #[test]
    fn missing_or_oversize_interrupt_output_is_rejected_without_io() {
        let mut no_output = host(&[], &[], false);
        assert_eq!(
            block_on(no_output.write_output_report(&[0; 8])),
            Err(HidError::InterruptOutUnavailable)
        );

        let mut host = host(&[], &[], true);
        assert_eq!(
            block_on(host.write_output_report(&[0; 9])),
            Err(HidError::OutputReportTooLong)
        );
        assert_eq!(host.interrupt_out_pipe().unwrap().calls, 0);
    }

    #[test]
    fn report_descriptor_request_uses_interface_and_advertised_length() {
        let descriptor = [0x5a; P8055_REPORT_DESCRIPTOR_LEN];
        let mut host = host(&descriptor, &[], true);
        let mut short = [0; P8055_REPORT_DESCRIPTOR_LEN - 1];
        assert!(matches!(
            block_on(host.get_report_descriptor(&mut short)),
            Err(HidError::ReportDescriptorBufferTooSmall)
        ));
        assert_eq!(host.control_pipe().calls, 0);

        let mut buffer = [0; P8055_REPORT_DESCRIPTOR_LEN];
        assert_eq!(
            block_on(host.get_report_descriptor(&mut buffer)),
            Ok(descriptor.as_slice())
        );
        assert_eq!(
            host.control_pipe().setup,
            [0x81, 0x06, 0x00, 0x22, 0x00, 0x00, 0x1d, 0x00]
        );
    }

    #[test]
    fn report_idle_and_protocol_control_requests_are_exact() {
        let mut host = host(&[0xaa, 0xbb], &[], true);
        let mut response = [0; 2];
        assert_eq!(
            block_on(host.get_report(HidReportType::Feature, 4, &mut response)),
            Ok(2)
        );
        assert_eq!(response, [0xaa, 0xbb]);
        assert_eq!(
            host.control_pipe().setup,
            [0xa1, 0x01, 0x04, 0x03, 0x00, 0x00, 0x02, 0x00]
        );

        assert_eq!(
            block_on(host.set_report(HidReportType::Output, 0, &[5; 8])),
            Ok(())
        );
        assert_eq!(
            host.control_pipe().setup,
            [0x21, 0x09, 0x00, 0x02, 0x00, 0x00, 0x08, 0x00]
        );
        assert_eq!(&host.control_pipe().out_data[..8], &[5; 8]);

        host.control.set_response(&[25]);
        assert_eq!(block_on(host.get_idle(7)), Ok(25));
        assert_eq!(
            host.control_pipe().setup,
            [0xa1, 0x02, 0x07, 0x00, 0x00, 0x00, 0x01, 0x00]
        );
        assert_eq!(block_on(host.set_idle(0, 25)), Ok(()));
        assert_eq!(
            host.control_pipe().setup,
            [0x21, 0x0a, 0x00, 0x19, 0x00, 0x00, 0x00, 0x00]
        );

        host.control.set_response(&[1]);
        assert_eq!(block_on(host.get_protocol()), Ok(HidProtocol::Report));
        assert_eq!(
            host.control_pipe().setup,
            [0xa1, 0x03, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00]
        );
        assert_eq!(block_on(host.set_protocol(HidProtocol::Boot)), Ok(()));
        assert_eq!(
            host.control_pipe().setup,
            [0x21, 0x0b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        );
    }

    #[test]
    fn transport_errors_toggles_and_owned_parts_are_preserved() {
        let mut host = host(&[], &[], true);
        host.interrupt_in.fail = true;
        let mut report = [0; 8];
        assert_eq!(
            block_on(host.read_input_report(&mut report)),
            Err(HidError::Transfer(PipeError::Timeout))
        );

        host.reset_data_toggles();
        assert_eq!(host.interrupt_in_pipe().resets, 1);
        assert_eq!(host.interrupt_out_pipe().unwrap().resets, 1);

        let (interface, control, input, output) = host.into_parts();
        assert_eq!(interface, p8055_interface());
        assert_eq!(control.calls, 0);
        assert_eq!(input.calls, 1);
        assert_eq!(output.unwrap().resets, 1);
    }
}
