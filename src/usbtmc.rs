//! Controller-independent USB Test and Measurement Class host support.
//!
//! USBTMC transports instrument commands in framed bulk messages. This module
//! discovers USBTMC and USBTMC-USB488 interfaces, allocates their control and
//! bulk pipes, and provides a bounded request/response API suitable for SCPI.

use crate::host::{
    Direction, EndpointAddress, EndpointInfo, EndpointType, HostError, PipeError, SplitInfo,
    TimeoutConfig, UsbHostAllocator, UsbPipe, pipe,
};
use crate::usb::{
    ConfigurationDescriptorHeader, ConfigurationError, DESCRIPTOR_TYPE_ENDPOINT,
    DESCRIPTOR_TYPE_INTERFACE, DescriptorIter, EndpointDescriptor, SetupRequest,
};

/// USB interface class assigned to application-specific devices.
pub const USBTMC_INTERFACE_CLASS: u8 = 0xfe;
/// USB interface subclass assigned to test and measurement devices.
pub const USBTMC_INTERFACE_SUBCLASS: u8 = 0x03;
/// Plain USBTMC interface protocol.
pub const USBTMC_PROTOCOL: u8 = 0x00;
/// USBTMC interface implementing the USB488 extension.
pub const USBTMC_USB488_PROTOCOL: u8 = 0x01;

/// Full-speed USBTMC bulk packet capacity.
pub const DEFAULT_PACKET_CAPACITY: usize = 64;
/// Default maximum outbound USBTMC message, including its 12-byte header.
pub const DEFAULT_OUT_MESSAGE_CAPACITY: usize = 512;

const MSGID_DEV_DEP_MSG_OUT: u8 = 1;
const MSGID_DEV_DEP_MSG_IN: u8 = 2;
const REQUEST_GET_CAPABILITIES: u8 = 7;
const REQUEST_REN_CONTROL: u8 = 0xa0;
const USBTMC_STATUS_SUCCESS: u8 = 1;
const USB488_CAPABILITY_SIMPLE: u8 = 1 << 1;
const HEADER_LEN: usize = 12;
const ATTRIBUTE_EOM: u8 = 1;

/// A USBTMC class instance whose pipes came from an Embassy host allocator.
pub type AllocatedUsbtmcHost<'d, A> = UsbtmcHost<
    <A as UsbHostAllocator<'d>>::Pipe<pipe::Control, pipe::InOut>,
    <A as UsbHostAllocator<'d>>::Pipe<pipe::Bulk, pipe::In>,
    <A as UsbHostAllocator<'d>>::Pipe<pipe::Bulk, pipe::Out>,
>;

/// A descriptor-validated USBTMC interface.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct UsbtmcInterface {
    pub configuration: ConfigurationDescriptorHeader,
    pub interface_number: u8,
    pub protocol: u8,
    pub bulk_out_endpoint: EndpointDescriptor,
    pub bulk_in_endpoint: EndpointDescriptor,
    pub interrupt_in_endpoint: Option<EndpointDescriptor>,
}

impl UsbtmcInterface {
    /// Discover the first alternate-setting-zero USBTMC interface.
    pub fn discover(bytes: &[u8]) -> Result<Self, UsbtmcConfigurationError> {
        let configuration = ConfigurationDescriptorHeader::parse(bytes)?;
        let mut selected = None;
        let mut current_selected = false;
        let mut bulk_in = None;
        let mut bulk_out = None;
        let mut interrupt_in = None;

        for descriptor in DescriptorIter::new(bytes)? {
            let descriptor = descriptor?;
            match descriptor[1] {
                DESCRIPTOR_TYPE_INTERFACE => {
                    if descriptor.len() != 9 {
                        return Err(UsbtmcConfigurationError::InvalidInterfaceDescriptor);
                    }
                    current_selected = descriptor[3] == 0
                        && descriptor[5] == USBTMC_INTERFACE_CLASS
                        && descriptor[6] == USBTMC_INTERFACE_SUBCLASS
                        && matches!(descriptor[7], USBTMC_PROTOCOL | USBTMC_USB488_PROTOCOL);
                    if current_selected {
                        if selected.is_some() {
                            break;
                        }
                        selected = Some((descriptor[2], descriptor[7]));
                        bulk_in = None;
                        bulk_out = None;
                        interrupt_in = None;
                    }
                }
                DESCRIPTOR_TYPE_ENDPOINT if current_selected => {
                    let endpoint = parse_endpoint(descriptor)?;
                    match (endpoint.attributes & 0x03, endpoint.is_in()) {
                        (0x02, true) if bulk_in.replace(endpoint).is_none() => {}
                        (0x02, false) if bulk_out.replace(endpoint).is_none() => {}
                        (0x03, true) if interrupt_in.replace(endpoint).is_none() => {}
                        _ => {
                            return Err(UsbtmcConfigurationError::InvalidEndpointDescriptor);
                        }
                    }
                }
                _ => {}
            }
        }

        let (interface_number, protocol) =
            selected.ok_or(UsbtmcConfigurationError::MissingInterface)?;
        Ok(Self {
            configuration,
            interface_number,
            protocol,
            bulk_out_endpoint: bulk_out.ok_or(UsbtmcConfigurationError::MissingBulkOutEndpoint)?,
            bulk_in_endpoint: bulk_in.ok_or(UsbtmcConfigurationError::MissingBulkInEndpoint)?,
            interrupt_in_endpoint: interrupt_in,
        })
    }

    pub const fn supports_usb488(self) -> bool {
        self.protocol == USBTMC_USB488_PROTOCOL
    }
}

fn parse_endpoint(bytes: &[u8]) -> Result<EndpointDescriptor, UsbtmcConfigurationError> {
    if bytes.len() != 7 || bytes[1] != DESCRIPTOR_TYPE_ENDPOINT {
        return Err(UsbtmcConfigurationError::InvalidEndpointDescriptor);
    }
    let raw_max_packet_size = u16::from_le_bytes([bytes[4], bytes[5]]);
    let endpoint = EndpointDescriptor {
        address: bytes[2],
        attributes: bytes[3],
        max_packet_size: raw_max_packet_size & 0x07ff,
        interval: bytes[6],
    };
    if endpoint.address & 0x70 != 0
        || endpoint.number() == 0
        || !matches!(endpoint.attributes & 0x03, 0x02 | 0x03)
        || !matches!(endpoint.max_packet_size, 8 | 16 | 32 | 64 | 512)
        || raw_max_packet_size & 0xf800 != 0
    {
        return Err(UsbtmcConfigurationError::InvalidEndpointDescriptor);
    }
    Ok(endpoint)
}

/// Error while discovering a USBTMC interface.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum UsbtmcConfigurationError {
    Configuration(ConfigurationError),
    MissingInterface,
    InvalidInterfaceDescriptor,
    InvalidEndpointDescriptor,
    MissingBulkInEndpoint,
    MissingBulkOutEndpoint,
}

impl From<ConfigurationError> for UsbtmcConfigurationError {
    fn from(error: ConfigurationError) -> Self {
        Self::Configuration(error)
    }
}

/// Error while allocating a USBTMC class instance.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum UsbtmcCreateError {
    Configuration(UsbtmcConfigurationError),
    InvalidDeviceAddress,
    InvalidControlMaxPacketSize,
    UnsupportedBulkInMaxPacketSize,
    Allocation(HostError),
}

impl From<UsbtmcConfigurationError> for UsbtmcCreateError {
    fn from(error: UsbtmcConfigurationError) -> Self {
        Self::Configuration(error)
    }
}

impl From<HostError> for UsbtmcCreateError {
    fn from(error: HostError) -> Self {
        Self::Allocation(error)
    }
}

/// Discover a USBTMC interface and allocate its endpoint-zero and bulk pipes.
pub fn allocate_usbtmc_host<'d, A>(
    allocator: &A,
    configuration: &[u8],
    device_address: u8,
    control_max_packet_size: u16,
    split: Option<SplitInfo>,
) -> Result<AllocatedUsbtmcHost<'d, A>, UsbtmcCreateError>
where
    A: UsbHostAllocator<'d>,
{
    if device_address > 0x7f {
        return Err(UsbtmcCreateError::InvalidDeviceAddress);
    }
    if !matches!(control_max_packet_size, 8 | 16 | 32 | 64) {
        return Err(UsbtmcCreateError::InvalidControlMaxPacketSize);
    }
    let interface = UsbtmcInterface::discover(configuration)?;
    if interface.bulk_in_endpoint.max_packet_size as usize > DEFAULT_PACKET_CAPACITY {
        return Err(UsbtmcCreateError::UnsupportedBulkInMaxPacketSize);
    }

    let control_info = EndpointInfo {
        addr: EndpointAddress::from_parts(0, Direction::In),
        ep_type: EndpointType::Control,
        max_packet_size: control_max_packet_size,
        interval_ms: 0,
    };
    let bulk_in_info = endpoint_info(interface.bulk_in_endpoint);
    let bulk_out_info = endpoint_info(interface.bulk_out_endpoint);
    let control =
        allocator.alloc_pipe::<pipe::Control, pipe::InOut>(device_address, &control_info, split)?;
    let bulk_in =
        allocator.alloc_pipe::<pipe::Bulk, pipe::In>(device_address, &bulk_in_info, split)?;
    let bulk_out =
        allocator.alloc_pipe::<pipe::Bulk, pipe::Out>(device_address, &bulk_out_info, split)?;

    Ok(UsbtmcHost::new(interface, control, bulk_in, bulk_out))
}

#[cfg(feature = "embassy-usb-host")]
/// Discover and allocate USBTMC directly from Embassy enumeration output.
pub fn allocate_from_enumeration<'d, A>(
    allocator: &A,
    configuration: &[u8],
    enumeration: &embassy_usb_host::handler::EnumerationInfo,
) -> Result<AllocatedUsbtmcHost<'d, A>, UsbtmcCreateError>
where
    A: UsbHostAllocator<'d>,
{
    allocate_usbtmc_host(
        allocator,
        configuration,
        enumeration.device_address,
        enumeration.device_desc.max_packet_size0 as u16,
        enumeration.split(),
    )
}

fn endpoint_info(endpoint: EndpointDescriptor) -> EndpointInfo {
    EndpointInfo {
        addr: EndpointAddress::from(endpoint.address),
        ep_type: EndpointType::Bulk,
        max_packet_size: endpoint.max_packet_size,
        interval_ms: endpoint.interval,
    }
}

/// Parsed response to the USBTMC `GET_CAPABILITIES` request.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct UsbtmcCapabilities {
    pub usbtmc_version_bcd: u16,
    pub interface_capabilities: u8,
    pub device_capabilities: u8,
    pub usb488_version_bcd: u16,
    pub usb488_interface_capabilities: u8,
    pub usb488_device_capabilities: u8,
}

impl UsbtmcCapabilities {
    /// Whether the USB488 interface accepts `REN_CONTROL`, `GO_TO_LOCAL`, and
    /// `LOCAL_LOCKOUT` class requests.
    pub const fn supports_remote_enable(self) -> bool {
        self.usb488_interface_capabilities & USB488_CAPABILITY_SIMPLE != 0
    }
}

/// Error returned by a USBTMC host.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum UsbtmcError {
    Transfer(PipeError),
    CommandTooLong,
    ResponseBufferTooSmall,
    InvalidCapabilities,
    InvalidControlStatus,
    InvalidMessage,
    TagMismatch,
}

impl From<PipeError> for UsbtmcError {
    fn from(error: PipeError) -> Self {
        Self::Transfer(error)
    }
}

/// A USBTMC interface backed by endpoint zero and two bulk pipes.
pub struct UsbtmcHost<
    C,
    I,
    O,
    const PACKET_CAPACITY: usize = DEFAULT_PACKET_CAPACITY,
    const OUT_MESSAGE_CAPACITY: usize = DEFAULT_OUT_MESSAGE_CAPACITY,
> {
    interface: UsbtmcInterface,
    control: C,
    bulk_in: I,
    bulk_out: O,
    packet: [u8; PACKET_CAPACITY],
    outgoing: [u8; OUT_MESSAGE_CAPACITY],
    next_tag: u8,
}

impl<C, I, O, const PACKET_CAPACITY: usize, const OUT_MESSAGE_CAPACITY: usize>
    UsbtmcHost<C, I, O, PACKET_CAPACITY, OUT_MESSAGE_CAPACITY>
{
    pub const fn new(interface: UsbtmcInterface, control: C, bulk_in: I, bulk_out: O) -> Self {
        assert!(
            interface.bulk_in_endpoint.max_packet_size as usize <= PACKET_CAPACITY,
            "USBTMC bulk-IN packet exceeds receive capacity"
        );
        Self {
            interface,
            control,
            bulk_in,
            bulk_out,
            packet: [0; PACKET_CAPACITY],
            outgoing: [0; OUT_MESSAGE_CAPACITY],
            next_tag: 1,
        }
    }

    pub const fn interface(&self) -> UsbtmcInterface {
        self.interface
    }
}

impl<C, I, O, const PACKET_CAPACITY: usize, const OUT_MESSAGE_CAPACITY: usize>
    UsbtmcHost<C, I, O, PACKET_CAPACITY, OUT_MESSAGE_CAPACITY>
where
    C: UsbPipe<pipe::Control, pipe::InOut>,
    I: UsbPipe<pipe::Bulk, pipe::In>,
    O: UsbPipe<pipe::Bulk, pipe::Out>,
{
    pub fn set_control_timeout(&mut self, timeout: TimeoutConfig) {
        self.control.set_timeout(timeout);
    }

    pub fn reset_data_toggles(&mut self) {
        self.bulk_in.reset_data_toggle();
        self.bulk_out.reset_data_toggle();
    }

    /// Read the mandatory 24-byte USBTMC capability block.
    pub async fn get_capabilities(&mut self) -> Result<UsbtmcCapabilities, UsbtmcError> {
        let setup = SetupRequest {
            request_type: 0xa1,
            request: REQUEST_GET_CAPABILITIES,
            value: 0,
            index: self.interface.interface_number as u16,
            length: 24,
        }
        .to_bytes();
        let mut bytes = [0_u8; 24];
        let received = self.control.control_in(&setup, &mut bytes).await?;
        if received != bytes.len() || bytes[0] != 1 {
            return Err(UsbtmcError::InvalidCapabilities);
        }
        Ok(UsbtmcCapabilities {
            usbtmc_version_bcd: u16::from_le_bytes([bytes[2], bytes[3]]),
            interface_capabilities: bytes[4],
            device_capabilities: bytes[5],
            usb488_version_bcd: u16::from_le_bytes([bytes[12], bytes[13]]),
            usb488_interface_capabilities: bytes[14],
            usb488_device_capabilities: bytes[15],
        })
    }

    /// Assert or deassert USB488 Remote Enable.
    ///
    /// Call this only when [`UsbtmcCapabilities::supports_remote_enable`]
    /// reports support. Asserting REN lets USB488 instruments enter remote
    /// operation when they receive program messages.
    pub async fn remote_enable(&mut self, enabled: bool) -> Result<(), UsbtmcError> {
        let setup = build_remote_enable_setup(self.interface.interface_number, enabled);
        let mut status = [0_u8; 1];
        let received = self.control.control_in(&setup, &mut status).await?;
        if received != status.len() || status[0] != USBTMC_STATUS_SUCCESS {
            return Err(UsbtmcError::InvalidControlStatus);
        }
        Ok(())
    }

    /// Send one SCPI/program message and request one response message.
    ///
    /// The returned byte count excludes USBTMC headers and alignment padding.
    pub async fn exchange(
        &mut self,
        command: &[u8],
        response: &mut [u8],
    ) -> Result<usize, UsbtmcError> {
        self.write(command).await?;
        self.read(response).await
    }

    /// Send one complete device-dependent message with EOM asserted.
    pub async fn write(&mut self, command: &[u8]) -> Result<(), UsbtmcError> {
        if command.len() + HEADER_LEN + 3 > OUT_MESSAGE_CAPACITY {
            return Err(UsbtmcError::CommandTooLong);
        }
        let tag = self.take_tag();
        let padded_command_len = command.len().next_multiple_of(4);
        let outgoing_len = HEADER_LEN + padded_command_len;
        self.outgoing[..outgoing_len].fill(0);
        self.outgoing[0] = MSGID_DEV_DEP_MSG_OUT;
        self.outgoing[1] = tag;
        self.outgoing[2] = !tag;
        self.outgoing[4..8].copy_from_slice(&(command.len() as u32).to_le_bytes());
        self.outgoing[8] = ATTRIBUTE_EOM;
        self.outgoing[HEADER_LEN..HEADER_LEN + command.len()].copy_from_slice(command);
        self.bulk_out
            .request_out(&self.outgoing[..outgoing_len], false)
            .await?;
        Ok(())
    }

    /// Request and read one complete device-dependent response message.
    pub async fn read(&mut self, response: &mut [u8]) -> Result<usize, UsbtmcError> {
        let tag = self.take_tag();
        let request = build_request_in(tag, response.len());
        self.bulk_out.request_out(&request, false).await?;
        self.read_response(tag, response).await
    }

    async fn read_response(&mut self, tag: u8, response: &mut [u8]) -> Result<usize, UsbtmcError> {
        let packet_size = self.interface.bulk_in_endpoint.max_packet_size as usize;
        let first_len = self
            .bulk_in
            .request_in(&mut self.packet[..packet_size])
            .await?;
        if first_len < HEADER_LEN
            || self.packet[0] != MSGID_DEV_DEP_MSG_IN
            || self.packet[2] != !self.packet[1]
        {
            return Err(UsbtmcError::InvalidMessage);
        }
        if self.packet[1] != tag {
            return Err(UsbtmcError::TagMismatch);
        }
        let announced = u32::from_le_bytes([
            self.packet[4],
            self.packet[5],
            self.packet[6],
            self.packet[7],
        ]) as usize;
        if announced > response.len() {
            return Err(UsbtmcError::ResponseBufferTooSmall);
        }

        let minimum_wire_len = HEADER_LEN + announced;
        let maximum_wire_len = HEADER_LEN + announced.next_multiple_of(4);
        if first_len > maximum_wire_len {
            return Err(UsbtmcError::InvalidMessage);
        }

        let mut copied = (first_len - HEADER_LEN).min(announced);
        let mut wire_received = first_len;
        let mut last_packet_len = first_len;
        response[..copied].copy_from_slice(&self.packet[HEADER_LEN..HEADER_LEN + copied]);
        while copied < announced {
            let received = self
                .bulk_in
                .request_in(&mut self.packet[..packet_size])
                .await?;
            if received == 0 || wire_received + received > maximum_wire_len {
                return Err(UsbtmcError::InvalidMessage);
            }
            let count = received.min(announced - copied);
            response[copied..copied + count].copy_from_slice(&self.packet[..count]);
            copied += count;
            wire_received += received;
            last_packet_len = received;
        }
        if wire_received < minimum_wire_len {
            return Err(UsbtmcError::InvalidMessage);
        }

        // USBTMC allows up to three alignment bytes after the announced data,
        // but real instruments may terminate the USB transfer with a short
        // packet immediately after the payload instead. Only a full final
        // packet needs a following ZLP to mark the transaction boundary.
        if last_packet_len == packet_size {
            let received = self
                .bulk_in
                .request_in(&mut self.packet[..packet_size])
                .await?;
            if received != 0 {
                return Err(UsbtmcError::InvalidMessage);
            }
        }
        Ok(copied)
    }

    fn take_tag(&mut self) -> u8 {
        let tag = self.next_tag;
        self.next_tag = if tag == u8::MAX { 1 } else { tag + 1 };
        tag
    }
}

fn build_remote_enable_setup(interface_number: u8, enabled: bool) -> [u8; 8] {
    SetupRequest {
        request_type: 0xa1,
        request: REQUEST_REN_CONTROL,
        value: u16::from(enabled),
        index: interface_number as u16,
        length: 1,
    }
    .to_bytes()
}

fn build_request_in(tag: u8, capacity: usize) -> [u8; HEADER_LEN] {
    let mut request = [0_u8; HEADER_LEN];
    request[0] = MSGID_DEV_DEP_MSG_IN;
    request[1] = tag;
    request[2] = !tag;
    request[4..8].copy_from_slice(&(capacity.min(u32::MAX as usize) as u32).to_le_bytes());
    request
}

#[cfg(test)]
mod tests {
    use core::future::Future;
    use core::pin::pin;
    use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};

    use super::*;

    const KEYSIGHT_34450A_CONFIGURATION: &[u8] = &[
        9, 2, 39, 0, 1, 1, 0, 0xc0, 50, 9, 4, 0, 0, 3, 0xfe, 3, 1, 0, 7, 5, 1, 2, 64, 0, 1, 7, 5,
        0x82, 2, 64, 0, 1, 7, 5, 0x84, 3, 8, 0, 1,
    ];

    struct FakeControl;

    impl UsbPipe<pipe::Control, pipe::InOut> for FakeControl {
        async fn control_in(
            &mut self,
            _setup: &[u8; 8],
            _buffer: &mut [u8],
        ) -> Result<usize, PipeError> {
            unreachable!()
        }

        async fn control_out(&mut self, _setup: &[u8; 8], _data: &[u8]) -> Result<(), PipeError> {
            unreachable!()
        }

        async fn request_in(&mut self, _buffer: &mut [u8]) -> Result<usize, PipeError> {
            unreachable!()
        }

        async fn request_out(
            &mut self,
            _data: &[u8],
            _ensure_transaction_end: bool,
        ) -> Result<(), PipeError> {
            unreachable!()
        }

        fn set_timeout(&mut self, _timeout: TimeoutConfig) {}

        fn reset_data_toggle(&mut self) {
            unreachable!()
        }
    }

    struct FakeBulkIn {
        response: [u8; DEFAULT_PACKET_CAPACITY],
        response_len: usize,
        calls: usize,
    }

    impl FakeBulkIn {
        fn new(response: &[u8]) -> Self {
            let mut bytes = [0; DEFAULT_PACKET_CAPACITY];
            bytes[..response.len()].copy_from_slice(response);
            Self {
                response: bytes,
                response_len: response.len(),
                calls: 0,
            }
        }
    }

    impl UsbPipe<pipe::Bulk, pipe::In> for FakeBulkIn {
        async fn control_in(
            &mut self,
            _setup: &[u8; 8],
            _buffer: &mut [u8],
        ) -> Result<usize, PipeError> {
            unreachable!()
        }

        async fn control_out(&mut self, _setup: &[u8; 8], _data: &[u8]) -> Result<(), PipeError> {
            unreachable!()
        }

        async fn request_in(&mut self, buffer: &mut [u8]) -> Result<usize, PipeError> {
            assert_eq!(self.calls, 0, "short response must finish in one packet");
            self.calls += 1;
            buffer[..self.response_len].copy_from_slice(&self.response[..self.response_len]);
            Ok(self.response_len)
        }

        async fn request_out(
            &mut self,
            _data: &[u8],
            _ensure_transaction_end: bool,
        ) -> Result<(), PipeError> {
            unreachable!()
        }

        fn set_timeout(&mut self, _timeout: TimeoutConfig) {
            unreachable!()
        }

        fn reset_data_toggle(&mut self) {}
    }

    struct FakeBulkOut;

    impl UsbPipe<pipe::Bulk, pipe::Out> for FakeBulkOut {
        async fn control_in(
            &mut self,
            _setup: &[u8; 8],
            _buffer: &mut [u8],
        ) -> Result<usize, PipeError> {
            unreachable!()
        }

        async fn control_out(&mut self, _setup: &[u8; 8], _data: &[u8]) -> Result<(), PipeError> {
            unreachable!()
        }

        async fn request_in(&mut self, _buffer: &mut [u8]) -> Result<usize, PipeError> {
            unreachable!()
        }

        async fn request_out(
            &mut self,
            _data: &[u8],
            _ensure_transaction_end: bool,
        ) -> Result<(), PipeError> {
            unreachable!()
        }

        fn set_timeout(&mut self, _timeout: TimeoutConfig) {
            unreachable!()
        }

        fn reset_data_toggle(&mut self) {}
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
    fn discovers_keysight_34450a_interface() {
        let interface = UsbtmcInterface::discover(KEYSIGHT_34450A_CONFIGURATION).unwrap();
        assert_eq!(interface.interface_number, 0);
        assert!(interface.supports_usb488());
        assert_eq!(interface.bulk_out_endpoint.address, 0x01);
        assert_eq!(interface.bulk_in_endpoint.address, 0x82);
        assert_eq!(interface.interrupt_in_endpoint.unwrap().address, 0x84);
    }

    #[test]
    fn request_in_header_uses_tag_inverse_and_capacity() {
        assert_eq!(
            build_request_in(7, 256),
            [2, 7, 0xf8, 0, 0, 1, 0, 0, 0, 0, 0, 0]
        );
    }

    #[test]
    fn usb488_remote_enable_capability_and_request_are_exact() {
        let mut capabilities = UsbtmcCapabilities {
            usbtmc_version_bcd: 0x0100,
            interface_capabilities: 0,
            device_capabilities: 0,
            usb488_version_bcd: 0x0100,
            usb488_interface_capabilities: 0,
            usb488_device_capabilities: 0,
        };
        assert!(!capabilities.supports_remote_enable());
        capabilities.usb488_interface_capabilities = USB488_CAPABILITY_SIMPLE;
        assert!(capabilities.supports_remote_enable());
        assert_eq!(
            build_remote_enable_setup(3, true),
            [0xa1, 0xa0, 1, 0, 3, 0, 1, 0]
        );
        assert_eq!(
            build_remote_enable_setup(3, false),
            [0xa1, 0xa0, 0, 0, 3, 0, 1, 0]
        );
    }

    #[test]
    fn accepts_short_response_without_optional_alignment_padding() {
        let interface = UsbtmcInterface::discover(KEYSIGHT_34450A_CONFIGURATION).unwrap();
        let packet = [
            MSGID_DEV_DEP_MSG_IN,
            4,
            !4,
            0,
            5,
            0,
            0,
            0,
            ATTRIBUTE_EOM,
            0,
            0,
            0,
            b'+',
            b'1',
            b'2',
            b'8',
            b'\n',
        ];
        let mut host: UsbtmcHost<
            FakeControl,
            FakeBulkIn,
            FakeBulkOut,
            DEFAULT_PACKET_CAPACITY,
            DEFAULT_OUT_MESSAGE_CAPACITY,
        > = UsbtmcHost::new(
            interface,
            FakeControl,
            FakeBulkIn::new(&packet),
            FakeBulkOut,
        );
        let mut response = [0; 16];

        assert_eq!(block_on(host.read_response(4, &mut response)), Ok(5));
        assert_eq!(&response[..5], b"+128\n");
        assert_eq!(host.bulk_in.calls, 1);
    }
}
