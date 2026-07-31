//! Controller-independent FTDI USB-to-serial host class.
//!
//! FTDI UART devices use vendor requests rather than CDC-ACM requests and
//! prepend a two-byte modem/line-status header to every bulk-IN packet. This
//! module turns that wire protocol into an asynchronous byte stream while
//! leaving USB packet retry and DATA-toggle handling to the host pipes.

use crate::host::{
    Direction, EndpointAddress, EndpointInfo, EndpointType, HostError, PipeError, SplitInfo,
    TimeoutConfig, UsbHostAllocator, UsbPipe, pipe,
};
use crate::usb::{
    ConfigurationDescriptorHeader, ConfigurationError, DESCRIPTOR_TYPE_ENDPOINT,
    DESCRIPTOR_TYPE_INTERFACE, DescriptorIter, DeviceDescriptor, EndpointDescriptor, SetupRequest,
};

/// FTDI's assigned USB vendor identifier.
pub const FTDI_VENDOR_ID: u16 = 0x0403;
/// FT232AM, FT232BM and FT232R default product identifier.
pub const FTDI_PRODUCT_ID_FT232: u16 = 0x6001;
/// FT2232C/D/H default product identifier.
pub const FTDI_PRODUCT_ID_FT2232: u16 = 0x6010;
/// FT4232H default product identifier.
pub const FTDI_PRODUCT_ID_FT4232H: u16 = 0x6011;
/// FT232H default product identifier.
pub const FTDI_PRODUCT_ID_FT232H: u16 = 0x6014;
/// FT230X-family default product identifier.
pub const FTDI_PRODUCT_ID_FT230X: u16 = 0x6015;

/// Full-speed FTDI bulk packet capacity.
pub const DEFAULT_RX_PACKET_CAPACITY: usize = 64;
/// High-speed FTDI bulk packet capacity.
pub const HIGH_SPEED_RX_PACKET_CAPACITY: usize = 512;

const FTDI_INTERFACE_CLASS: u8 = 0xff;
const REQUEST_RESET: u8 = 0x00;
const REQUEST_MODEM_CTRL: u8 = 0x01;
const REQUEST_FLOW_CTRL: u8 = 0x02;
const REQUEST_SET_BAUD: u8 = 0x03;
const REQUEST_SET_DATA: u8 = 0x04;
const RESET_SIO: u16 = 0;
const PURGE_TX: u16 = 1;
const PURGE_RX: u16 = 2;

/// An FTDI class instance whose pipes came from an Embassy host allocator.
pub type AllocatedFtdiHostWithRxCapacity<'d, A, const RX_PACKET_CAPACITY: usize> = FtdiHost<
    <A as UsbHostAllocator<'d>>::Pipe<pipe::Control, pipe::InOut>,
    <A as UsbHostAllocator<'d>>::Pipe<pipe::Bulk, pipe::In>,
    <A as UsbHostAllocator<'d>>::Pipe<pipe::Bulk, pipe::Out>,
    RX_PACKET_CAPACITY,
>;

/// A full-speed FTDI class instance with a 64-byte receive packet.
pub type AllocatedFtdiHost<'d, A> =
    AllocatedFtdiHostWithRxCapacity<'d, A, DEFAULT_RX_PACKET_CAPACITY>;

/// An FTDI class instance sized for a 512-byte high-speed endpoint.
pub type HighSpeedAllocatedFtdiHost<'d, A> =
    AllocatedFtdiHostWithRxCapacity<'d, A, HIGH_SPEED_RX_PACKET_CAPACITY>;

/// FTDI silicon generation inferred from `bcdDevice`.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FtdiChip {
    Ft232Am,
    Ft232Bm,
    Ft2232C,
    Ft232R,
    Ft2232H,
    Ft4232H,
    Ft232H,
    Ft230X,
}

impl FtdiChip {
    /// Infer the silicon generation using the same `bcdDevice` scheme as the
    /// FTDI D2XX/libftdi family. Unknown revisions conservatively use BM
    /// divider rules, as libftdi does.
    pub const fn detect(device: DeviceDescriptor) -> Self {
        match device.device_version_bcd {
            0x0200 if device.serial_number_string_index != 0 => Self::Ft232Am,
            0x0200 | 0x0400 => Self::Ft232Bm,
            0x0500 => Self::Ft2232C,
            0x0600 => Self::Ft232R,
            0x0700 => Self::Ft2232H,
            0x0800 => Self::Ft4232H,
            0x0900 => Self::Ft232H,
            0x1000 => Self::Ft230X,
            _ => Self::Ft232Bm,
        }
    }

    const fn is_multi_channel(self) -> bool {
        matches!(self, Self::Ft2232C | Self::Ft2232H | Self::Ft4232H)
    }

    const fn is_h_type(self) -> bool {
        matches!(self, Self::Ft2232H | Self::Ft4232H | Self::Ft232H)
    }
}

/// A descriptor-validated FTDI vendor interface.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct FtdiInterface {
    pub configuration: ConfigurationDescriptorHeader,
    pub interface_number: u8,
    pub interface_subclass: u8,
    pub interface_protocol: u8,
    pub bulk_out_endpoint: EndpointDescriptor,
    pub bulk_in_endpoint: EndpointDescriptor,
}

/// Error while discovering an FTDI interface.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FtdiConfigurationError {
    Configuration(ConfigurationError),
    NotFtdiDevice,
    UnsupportedProduct,
    MissingInterface,
    InvalidInterfaceDescriptor,
    InvalidEndpointDescriptor,
    MissingBulkInEndpoint,
    MissingBulkOutEndpoint,
}

impl From<ConfigurationError> for FtdiConfigurationError {
    fn from(error: ConfigurationError) -> Self {
        Self::Configuration(error)
    }
}

impl FtdiInterface {
    /// Discover the first alternate-setting-zero FTDI vendor interface.
    pub fn discover(
        device: DeviceDescriptor,
        configuration: &[u8],
    ) -> Result<Self, FtdiConfigurationError> {
        Self::discover_interface(device, configuration, None)
    }

    /// Discover a specific alternate-setting-zero FTDI vendor interface.
    pub fn discover_number(
        device: DeviceDescriptor,
        configuration: &[u8],
        interface_number: u8,
    ) -> Result<Self, FtdiConfigurationError> {
        Self::discover_interface(device, configuration, Some(interface_number))
    }

    fn discover_interface(
        device: DeviceDescriptor,
        bytes: &[u8],
        requested: Option<u8>,
    ) -> Result<Self, FtdiConfigurationError> {
        if device.vendor_id != FTDI_VENDOR_ID {
            return Err(FtdiConfigurationError::NotFtdiDevice);
        }
        if !matches!(
            device.product_id,
            FTDI_PRODUCT_ID_FT232
                | FTDI_PRODUCT_ID_FT2232
                | FTDI_PRODUCT_ID_FT4232H
                | FTDI_PRODUCT_ID_FT232H
                | FTDI_PRODUCT_ID_FT230X
        ) {
            return Err(FtdiConfigurationError::UnsupportedProduct);
        }

        let configuration = ConfigurationDescriptorHeader::parse(bytes)?;
        let mut current: Option<(u8, u8, u8, u8, u8)> = None;
        let mut selected: Option<(u8, u8, u8, u8)> = None;
        let mut endpoints_seen = 0_u8;
        let mut bulk_in = None;
        let mut bulk_out = None;

        for descriptor in DescriptorIter::new(bytes)? {
            let descriptor = descriptor?;
            match descriptor[1] {
                DESCRIPTOR_TYPE_INTERFACE => {
                    if descriptor.len() != 9 {
                        return Err(FtdiConfigurationError::InvalidInterfaceDescriptor);
                    }
                    let interface = (
                        descriptor[2],
                        descriptor[3],
                        descriptor[4],
                        descriptor[5],
                        descriptor[6],
                    );
                    current = Some(interface);
                    let wanted = interface.1 == 0
                        && interface.3 == FTDI_INTERFACE_CLASS
                        && requested.is_none_or(|number| number == interface.0);
                    if wanted {
                        if selected.is_some() {
                            break;
                        }
                        selected = Some((interface.0, interface.2, interface.4, descriptor[7]));
                        endpoints_seen = 0;
                        bulk_in = None;
                        bulk_out = None;
                    }
                }
                DESCRIPTOR_TYPE_ENDPOINT => {
                    let Some(interface) = current else {
                        continue;
                    };
                    if selected.map(|item| item.0) != Some(interface.0) || interface.1 != 0 {
                        continue;
                    }
                    let endpoint = parse_bulk_endpoint(descriptor)?;
                    endpoints_seen = endpoints_seen
                        .checked_add(1)
                        .ok_or(FtdiConfigurationError::InvalidEndpointDescriptor)?;
                    let slot = if endpoint.is_in() {
                        &mut bulk_in
                    } else {
                        &mut bulk_out
                    };
                    if slot.replace(endpoint).is_some() {
                        return Err(FtdiConfigurationError::InvalidEndpointDescriptor);
                    }
                }
                _ => {}
            }
        }

        let (interface_number, endpoint_count, interface_subclass, interface_protocol) =
            selected.ok_or(FtdiConfigurationError::MissingInterface)?;
        if endpoint_count != 2 || endpoints_seen != endpoint_count {
            return Err(FtdiConfigurationError::InvalidEndpointDescriptor);
        }
        Ok(Self {
            configuration,
            interface_number,
            interface_subclass,
            interface_protocol,
            bulk_out_endpoint: bulk_out.ok_or(FtdiConfigurationError::MissingBulkOutEndpoint)?,
            bulk_in_endpoint: bulk_in.ok_or(FtdiConfigurationError::MissingBulkInEndpoint)?,
        })
    }
}

fn parse_bulk_endpoint(bytes: &[u8]) -> Result<EndpointDescriptor, FtdiConfigurationError> {
    if bytes.len() != 7 || bytes[1] != DESCRIPTOR_TYPE_ENDPOINT {
        return Err(FtdiConfigurationError::InvalidEndpointDescriptor);
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
        || endpoint.attributes != 0x02
        || !matches!(endpoint.max_packet_size, 8 | 16 | 32 | 64 | 512)
        || raw_max_packet_size & 0xf800 != 0
    {
        return Err(FtdiConfigurationError::InvalidEndpointDescriptor);
    }
    Ok(endpoint)
}

/// Error while allocating an FTDI class instance.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FtdiCreateError {
    Configuration(FtdiConfigurationError),
    InvalidDeviceAddress,
    InvalidControlMaxPacketSize,
    UnsupportedBulkInMaxPacketSize,
    Allocation(HostError),
}

impl From<FtdiConfigurationError> for FtdiCreateError {
    fn from(error: FtdiConfigurationError) -> Self {
        Self::Configuration(error)
    }
}

impl From<HostError> for FtdiCreateError {
    fn from(error: HostError) -> Self {
        Self::Allocation(error)
    }
}

/// Discover the first FTDI interface and allocate its pipes.
pub fn allocate_ftdi_host<'d, A>(
    allocator: &A,
    device: DeviceDescriptor,
    configuration: &[u8],
    device_address: u8,
    split: Option<SplitInfo>,
) -> Result<AllocatedFtdiHost<'d, A>, FtdiCreateError>
where
    A: UsbHostAllocator<'d>,
{
    allocate_ftdi_host_with_rx_capacity::<A, DEFAULT_RX_PACKET_CAPACITY>(
        allocator,
        device,
        configuration,
        device_address,
        split,
    )
}

/// Discover the first FTDI interface and allocate explicitly sized pipes.
pub fn allocate_ftdi_host_with_rx_capacity<'d, A, const RX_PACKET_CAPACITY: usize>(
    allocator: &A,
    device: DeviceDescriptor,
    configuration: &[u8],
    device_address: u8,
    split: Option<SplitInfo>,
) -> Result<AllocatedFtdiHostWithRxCapacity<'d, A, RX_PACKET_CAPACITY>, FtdiCreateError>
where
    A: UsbHostAllocator<'d>,
{
    validate_allocation(device_address, device.max_packet_size0 as u16)?;
    let interface = FtdiInterface::discover(device, configuration)?;
    allocate_ftdi_interface_with_rx_capacity::<A, RX_PACKET_CAPACITY>(
        allocator,
        device,
        interface,
        device_address,
        split,
    )
}

#[cfg(feature = "embassy-usb-host")]
/// Discover and allocate an FTDI interface directly from Embassy enumeration
/// output.
pub fn allocate_from_enumeration<'d, A>(
    allocator: &A,
    configuration: &[u8],
    enumeration: &embassy_usb_host::handler::EnumerationInfo,
) -> Result<AllocatedFtdiHost<'d, A>, FtdiCreateError>
where
    A: UsbHostAllocator<'d>,
{
    let descriptor = enumeration.device_desc;
    let device = DeviceDescriptor {
        usb_version_bcd: descriptor.bcd_usb,
        device_class: descriptor.device_class,
        device_subclass: descriptor.device_subclass,
        device_protocol: descriptor.device_protocol,
        max_packet_size0: descriptor.max_packet_size0,
        vendor_id: descriptor.vendor_id,
        product_id: descriptor.product_id,
        device_version_bcd: descriptor.bcd_device,
        manufacturer_string_index: descriptor.manufacturer,
        product_string_index: descriptor.product,
        serial_number_string_index: descriptor.serial_number,
        num_configurations: descriptor.num_configurations,
    };
    allocate_ftdi_host(
        allocator,
        device,
        configuration,
        enumeration.device_address,
        enumeration.split(),
    )
}

/// Allocate pipes for an already selected FTDI interface.
pub fn allocate_ftdi_interface_with_rx_capacity<'d, A, const RX_PACKET_CAPACITY: usize>(
    allocator: &A,
    device: DeviceDescriptor,
    interface: FtdiInterface,
    device_address: u8,
    split: Option<SplitInfo>,
) -> Result<AllocatedFtdiHostWithRxCapacity<'d, A, RX_PACKET_CAPACITY>, FtdiCreateError>
where
    A: UsbHostAllocator<'d>,
{
    validate_allocation(device_address, device.max_packet_size0 as u16)?;
    if interface.bulk_in_endpoint.max_packet_size as usize > RX_PACKET_CAPACITY {
        return Err(FtdiCreateError::UnsupportedBulkInMaxPacketSize);
    }
    let control_info = EndpointInfo {
        addr: EndpointAddress::from_parts(0, Direction::In),
        ep_type: EndpointType::Control,
        max_packet_size: device.max_packet_size0 as u16,
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
    Ok(FtdiHost::new(device, interface, control, bulk_in, bulk_out))
}

fn validate_allocation(device_address: u8, control_mps: u16) -> Result<(), FtdiCreateError> {
    if device_address > 0x7f {
        return Err(FtdiCreateError::InvalidDeviceAddress);
    }
    if !matches!(control_mps, 8 | 16 | 32 | 64) {
        return Err(FtdiCreateError::InvalidControlMaxPacketSize);
    }
    Ok(())
}

fn endpoint_info(endpoint: EndpointDescriptor) -> EndpointInfo {
    EndpointInfo {
        addr: EndpointAddress::from(endpoint.address),
        ep_type: EndpointType::Bulk,
        max_packet_size: endpoint.max_packet_size,
        interval_ms: endpoint.interval,
    }
}

/// UART parity.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum FtdiParity {
    #[default]
    None,
    Odd,
    Even,
    Mark,
    Space,
}

/// UART stop-bit selection.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum FtdiStopBits {
    #[default]
    One,
    OnePointFive,
    Two,
}

/// FTDI flow-control mode.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum FtdiFlowControl {
    #[default]
    Disabled,
    RtsCts,
    DtrDsr,
    XonXoff {
        xon: u8,
        xoff: u8,
    },
}

/// Latest status bytes received from an FTDI bulk-IN packet.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct FtdiStatus {
    pub modem: u8,
    pub line: u8,
}

/// Error returned by an FTDI host.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FtdiError {
    Transfer(PipeError),
    InvalidBaudRate,
    UnsupportedBaudRate { requested: u32, actual: u32 },
    InvalidDataBits,
}

impl core::fmt::Display for FtdiError {
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Transfer(_) => formatter.write_str("FTDI USB transfer failed"),
            Self::InvalidBaudRate => formatter.write_str("FTDI baud rate must be nonzero"),
            Self::UnsupportedBaudRate { .. } => {
                formatter.write_str("requested FTDI baud rate is not achievable")
            }
            Self::InvalidDataBits => formatter.write_str("FTDI UART supports 7 or 8 data bits"),
        }
    }
}

impl core::error::Error for FtdiError {}

impl From<PipeError> for FtdiError {
    fn from(error: PipeError) -> Self {
        Self::Transfer(error)
    }
}

impl embedded_io_async::Error for FtdiError {
    fn kind(&self) -> embedded_io_async::ErrorKind {
        match self {
            Self::Transfer(PipeError::Disconnected) => embedded_io_async::ErrorKind::NotConnected,
            Self::Transfer(PipeError::Timeout) => embedded_io_async::ErrorKind::TimedOut,
            Self::Transfer(PipeError::BufferOverflow) => embedded_io_async::ErrorKind::OutOfMemory,
            _ => embedded_io_async::ErrorKind::Other,
        }
    }
}

/// A configured FTDI UART backed by endpoint-zero and two bulk pipes.
pub struct FtdiHost<C, I, O, const RX_PACKET_CAPACITY: usize = DEFAULT_RX_PACKET_CAPACITY> {
    device: DeviceDescriptor,
    interface: FtdiInterface,
    chip: FtdiChip,
    control: C,
    bulk_in: I,
    bulk_out: O,
    rx_packet: [u8; RX_PACKET_CAPACITY],
    rx_start: usize,
    rx_end: usize,
    status: FtdiStatus,
    baud_rate: u32,
}

impl<C, I, O, const RX_PACKET_CAPACITY: usize> FtdiHost<C, I, O, RX_PACKET_CAPACITY> {
    pub const fn new(
        device: DeviceDescriptor,
        interface: FtdiInterface,
        control: C,
        bulk_in: I,
        bulk_out: O,
    ) -> Self {
        assert!(
            interface.bulk_in_endpoint.max_packet_size as usize <= RX_PACKET_CAPACITY,
            "FTDI bulk-IN packet exceeds receive capacity"
        );
        Self {
            device,
            interface,
            chip: FtdiChip::detect(device),
            control,
            bulk_in,
            bulk_out,
            rx_packet: [0; RX_PACKET_CAPACITY],
            rx_start: 0,
            rx_end: 0,
            status: FtdiStatus { modem: 0, line: 0 },
            baud_rate: 0,
        }
    }

    pub const fn device(&self) -> DeviceDescriptor {
        self.device
    }

    pub const fn interface(&self) -> FtdiInterface {
        self.interface
    }

    pub const fn chip(&self) -> FtdiChip {
        self.chip
    }

    pub const fn status(&self) -> FtdiStatus {
        self.status
    }

    pub const fn baud_rate(&self) -> u32 {
        self.baud_rate
    }

    fn usb_index(&self) -> u16 {
        if self.chip.is_multi_channel() {
            self.interface.interface_number as u16 + 1
        } else {
            0
        }
    }
}

impl<C, I, O, const RX_PACKET_CAPACITY: usize> FtdiHost<C, I, O, RX_PACKET_CAPACITY>
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
        self.rx_start = 0;
        self.rx_end = 0;
    }

    /// Reset the FTDI SIO engine.
    pub async fn reset(&mut self) -> Result<(), FtdiError> {
        self.vendor_out(REQUEST_RESET, RESET_SIO, 0).await?;
        self.rx_start = 0;
        self.rx_end = 0;
        Ok(())
    }

    pub async fn purge_rx(&mut self) -> Result<(), FtdiError> {
        self.vendor_out(REQUEST_RESET, PURGE_RX, self.usb_index())
            .await?;
        self.rx_start = 0;
        self.rx_end = 0;
        Ok(())
    }

    pub async fn purge_tx(&mut self) -> Result<(), FtdiError> {
        self.vendor_out(REQUEST_RESET, PURGE_TX, self.usb_index())
            .await
    }

    /// Configure a conventional 8N1 UART with flow control disabled.
    pub async fn configure_8n1(&mut self, baud_rate: u32) -> Result<u32, FtdiError> {
        self.reset().await?;
        self.purge_tx().await?;
        self.purge_rx().await?;
        let actual = self.set_baud_rate(baud_rate).await?;
        self.set_line_properties(8, FtdiStopBits::One, FtdiParity::None, false)
            .await?;
        self.set_flow_control(FtdiFlowControl::Disabled).await?;
        Ok(actual)
    }

    /// Set the closest supported baud rate and return the actual rate.
    pub async fn set_baud_rate(&mut self, requested: u32) -> Result<u32, FtdiError> {
        let divisor = convert_baud_rate(requested, self.chip, self.usb_index())
            .ok_or(FtdiError::InvalidBaudRate)?;
        let difference = divisor.actual.abs_diff(requested);
        if difference as u64 * 20 > requested as u64 {
            return Err(FtdiError::UnsupportedBaudRate {
                requested,
                actual: divisor.actual,
            });
        }
        self.vendor_out(REQUEST_SET_BAUD, divisor.value, divisor.index)
            .await?;
        self.baud_rate = divisor.actual;
        Ok(divisor.actual)
    }

    pub async fn set_line_properties(
        &mut self,
        data_bits: u8,
        stop_bits: FtdiStopBits,
        parity: FtdiParity,
        break_enabled: bool,
    ) -> Result<(), FtdiError> {
        if !matches!(data_bits, 7 | 8) {
            return Err(FtdiError::InvalidDataBits);
        }
        let parity = match parity {
            FtdiParity::None => 0,
            FtdiParity::Odd => 1,
            FtdiParity::Even => 2,
            FtdiParity::Mark => 3,
            FtdiParity::Space => 4,
        };
        let stop = match stop_bits {
            FtdiStopBits::One => 0,
            FtdiStopBits::OnePointFive => 1,
            FtdiStopBits::Two => 2,
        };
        let value =
            data_bits as u16 | (parity << 8) | (stop << 11) | (u16::from(break_enabled) << 14);
        self.vendor_out(REQUEST_SET_DATA, value, self.usb_index())
            .await
    }

    pub async fn set_flow_control(&mut self, flow: FtdiFlowControl) -> Result<(), FtdiError> {
        let (value, index) = match flow {
            FtdiFlowControl::Disabled => (0, self.usb_index()),
            FtdiFlowControl::RtsCts => (0, 0x0100 | self.usb_index()),
            FtdiFlowControl::DtrDsr => (0, 0x0200 | self.usb_index()),
            FtdiFlowControl::XonXoff { xon, xoff } => (
                u16::from(xon) | (u16::from(xoff) << 8),
                0x0400 | self.usb_index(),
            ),
        };
        self.vendor_out(REQUEST_FLOW_CTRL, value, index).await
    }

    pub async fn set_dtr_rts(&mut self, dtr: bool, rts: bool) -> Result<(), FtdiError> {
        let dtr = if dtr { 0x0101 } else { 0x0100 };
        let rts = if rts { 0x0202 } else { 0x0200 };
        self.vendor_out(REQUEST_MODEM_CTRL, dtr | rts, self.usb_index())
            .await
    }

    /// Read serial payload, stripping the two FTDI status bytes belonging to
    /// each received USB packet.
    pub async fn read(&mut self, buffer: &mut [u8]) -> Result<usize, FtdiError> {
        if buffer.is_empty() {
            return Ok(0);
        }
        if self.rx_start != self.rx_end {
            return Ok(self.copy_buffered(buffer));
        }
        loop {
            let packet_size = self.interface.bulk_in_endpoint.max_packet_size as usize;
            let received = self
                .bulk_in
                .request_in(&mut self.rx_packet[..packet_size])
                .await?;
            if received > packet_size {
                return Err(PipeError::BufferOverflow.into());
            }
            if received < 2 {
                continue;
            }
            self.status = FtdiStatus {
                modem: self.rx_packet[0],
                line: self.rx_packet[1],
            };
            if received == 2 {
                continue;
            }
            self.rx_start = 2;
            self.rx_end = received;
            return Ok(self.copy_buffered(buffer));
        }
    }

    pub async fn write(&mut self, data: &[u8]) -> Result<usize, FtdiError> {
        if data.is_empty() {
            return Ok(0);
        }
        self.bulk_out.request_out(data, false).await?;
        Ok(data.len())
    }

    pub const fn control_pipe(&self) -> &C {
        &self.control
    }

    pub const fn bulk_in_pipe(&self) -> &I {
        &self.bulk_in
    }

    pub const fn bulk_out_pipe(&self) -> &O {
        &self.bulk_out
    }

    /// Recover the descriptors and all three owned pipes.
    ///
    /// Any serial payload retained in the internal receive buffer is
    /// discarded.
    pub fn into_parts(self) -> (DeviceDescriptor, FtdiInterface, C, I, O) {
        (
            self.device,
            self.interface,
            self.control,
            self.bulk_in,
            self.bulk_out,
        )
    }

    async fn vendor_out(&mut self, request: u8, value: u16, index: u16) -> Result<(), FtdiError> {
        let setup = SetupRequest {
            request_type: 0x40,
            request,
            value,
            index,
            length: 0,
        }
        .to_bytes();
        self.control.control_out(&setup, &[]).await?;
        Ok(())
    }

    fn copy_buffered(&mut self, buffer: &mut [u8]) -> usize {
        let count = buffer.len().min(self.rx_end - self.rx_start);
        buffer[..count].copy_from_slice(&self.rx_packet[self.rx_start..self.rx_start + count]);
        self.rx_start += count;
        if self.rx_start == self.rx_end {
            self.rx_start = 0;
            self.rx_end = 0;
        }
        count
    }
}

impl<C, I, O, const RX_PACKET_CAPACITY: usize> embedded_io_async::ErrorType
    for FtdiHost<C, I, O, RX_PACKET_CAPACITY>
where
    C: UsbPipe<pipe::Control, pipe::InOut>,
    I: UsbPipe<pipe::Bulk, pipe::In>,
    O: UsbPipe<pipe::Bulk, pipe::Out>,
{
    type Error = FtdiError;
}

impl<C, I, O, const RX_PACKET_CAPACITY: usize> embedded_io_async::Read
    for FtdiHost<C, I, O, RX_PACKET_CAPACITY>
where
    C: UsbPipe<pipe::Control, pipe::InOut>,
    I: UsbPipe<pipe::Bulk, pipe::In>,
    O: UsbPipe<pipe::Bulk, pipe::Out>,
{
    async fn read(&mut self, buffer: &mut [u8]) -> Result<usize, Self::Error> {
        FtdiHost::read(self, buffer).await
    }
}

impl<C, I, O, const RX_PACKET_CAPACITY: usize> embedded_io_async::Write
    for FtdiHost<C, I, O, RX_PACKET_CAPACITY>
where
    C: UsbPipe<pipe::Control, pipe::InOut>,
    I: UsbPipe<pipe::Bulk, pipe::In>,
    O: UsbPipe<pipe::Bulk, pipe::Out>,
{
    async fn write(&mut self, data: &[u8]) -> Result<usize, Self::Error> {
        FtdiHost::write(self, data).await
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct BaudDivisor {
    actual: u32,
    value: u16,
    index: u16,
}

const FRACTION_CODE: [u32; 8] = [0, 3, 2, 4, 1, 5, 6, 7];

fn convert_baud_rate(baud: u32, chip: FtdiChip, usb_index: u16) -> Option<BaudDivisor> {
    if baud == 0 {
        return None;
    }
    let (actual, encoded) = if chip == FtdiChip::Ft232Am {
        clock_bits_am(baud)
    } else if chip.is_h_type() && baud as u64 * 10 > 120_000_000_u64 / 0x3fff {
        let (actual, encoded) = clock_bits(baud, 120_000_000, 10);
        (actual, encoded | 0x20000)
    } else {
        clock_bits(baud, 48_000_000, 16)
    };
    if actual == 0 {
        return None;
    }
    let value = (encoded & 0xffff) as u16;
    let index = match chip {
        FtdiChip::Ft2232H | FtdiChip::Ft4232H | FtdiChip::Ft232H => {
            ((encoded >> 8) as u16 & 0xff00) | usb_index
        }
        FtdiChip::Ft2232C | FtdiChip::Ft230X => ((encoded >> 16) as u16) << 8 | usb_index,
        _ => (encoded >> 16) as u16,
    };
    Some(BaudDivisor {
        actual,
        value,
        index,
    })
}

fn clock_bits(baud: u32, clock: u32, predivider: u32) -> (u32, u32) {
    if baud >= clock / predivider {
        return (clock / predivider, 0);
    }
    if baud >= clock / (predivider + predivider / 2) {
        return (clock / (predivider + predivider / 2), 1);
    }
    if baud >= clock / (2 * predivider) {
        return (clock / (2 * predivider), 2);
    }
    let divisor = clock * 16 / predivider / baud;
    let best = (divisor / 2 + (divisor & 1)).min(0x1ffff);
    let twice_actual = clock * 16 / predivider / best;
    let actual = twice_actual / 2 + (twice_actual & 1);
    let encoded = (best >> 3) | (FRACTION_CODE[(best & 7) as usize] << 14);
    (actual, encoded)
}

fn clock_bits_am(baud: u32) -> (u32, u32) {
    const DOWN: [i32; 8] = [0, 0, 0, 1, 0, 3, 2, 1];
    const UP: [i32; 8] = [0, 0, 0, 1, 0, 1, 2, 3];
    let requested = baud as i32;
    let divisor = 24_000_000 / requested;
    let divisor = divisor - DOWN[(divisor & 7) as usize];
    let mut best = 8;
    let mut best_baud = 3_000_000;
    let mut best_difference = i32::MAX;
    for increment in 0..2 {
        let mut candidate = divisor + increment;
        if candidate <= 8 {
            candidate = 8;
        } else if divisor < 16 {
            candidate = 16;
        } else {
            candidate = (candidate + UP[(candidate & 7) as usize]).min(0x1fff8);
        }
        let actual = (24_000_000 + candidate / 2) / candidate;
        let difference = (actual - requested).abs();
        if difference < best_difference {
            best = candidate;
            best_baud = actual;
            best_difference = difference;
        }
    }
    let mut encoded = (best as u32 >> 3) | (FRACTION_CODE[best as usize & 7] << 14);
    if encoded == 1 {
        encoded = 0;
    } else if encoded == 0x4001 {
        encoded = 1;
    }
    (best_baud as u32, encoded)
}

#[cfg(test)]
mod tests {
    use core::future::Future;
    use core::pin::pin;
    use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};

    use super::*;

    const FT232R_DEVICE: DeviceDescriptor = DeviceDescriptor {
        usb_version_bcd: 0x0200,
        device_class: 0,
        device_subclass: 0,
        device_protocol: 0,
        max_packet_size0: 8,
        vendor_id: FTDI_VENDOR_ID,
        product_id: FTDI_PRODUCT_ID_FT232,
        device_version_bcd: 0x0600,
        manufacturer_string_index: 1,
        product_string_index: 2,
        serial_number_string_index: 3,
        num_configurations: 1,
    };

    const CONFIGURATION: [u8; 32] = [
        9, 2, 32, 0, 1, 1, 0, 0x80, 50, // configuration
        9, 4, 0, 0, 2, 0xff, 0xff, 0xff, 0, // vendor interface
        7, 5, 0x81, 2, 64, 0, 0, // bulk IN
        7, 5, 0x02, 2, 64, 0, 0, // bulk OUT
    ];

    struct FakeControl {
        setups: [[u8; 8]; 8],
        calls: usize,
    }

    impl FakeControl {
        const fn new() -> Self {
            Self {
                setups: [[0; 8]; 8],
                calls: 0,
            }
        }
    }

    impl UsbPipe<pipe::Control, pipe::InOut> for FakeControl {
        async fn control_in(
            &mut self,
            _setup: &[u8; 8],
            _buffer: &mut [u8],
        ) -> Result<usize, PipeError> {
            unreachable!()
        }

        async fn control_out(&mut self, setup: &[u8; 8], data: &[u8]) -> Result<(), PipeError> {
            assert!(data.is_empty());
            self.setups[self.calls] = *setup;
            self.calls += 1;
            Ok(())
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
        packet: [u8; DEFAULT_RX_PACKET_CAPACITY],
        packet_len: usize,
        calls: usize,
        resets: usize,
    }

    impl FakeBulkIn {
        fn new(packet: &[u8]) -> Self {
            let mut bytes = [0; DEFAULT_RX_PACKET_CAPACITY];
            bytes[..packet.len()].copy_from_slice(packet);
            Self {
                packet: bytes,
                packet_len: packet.len(),
                calls: 0,
                resets: 0,
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
            self.calls += 1;
            buffer[..self.packet_len].copy_from_slice(&self.packet[..self.packet_len]);
            Ok(self.packet_len)
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

        fn reset_data_toggle(&mut self) {
            self.resets += 1;
        }
    }

    struct FakeBulkOut {
        bytes: [u8; DEFAULT_RX_PACKET_CAPACITY],
        len: usize,
        calls: usize,
        resets: usize,
    }

    impl FakeBulkOut {
        const fn new() -> Self {
            Self {
                bytes: [0; DEFAULT_RX_PACKET_CAPACITY],
                len: 0,
                calls: 0,
                resets: 0,
            }
        }
    }

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
            data: &[u8],
            ensure_transaction_end: bool,
        ) -> Result<(), PipeError> {
            assert!(!ensure_transaction_end);
            self.bytes[..data.len()].copy_from_slice(data);
            self.len = data.len();
            self.calls += 1;
            Ok(())
        }

        fn set_timeout(&mut self, _timeout: TimeoutConfig) {
            unreachable!()
        }

        fn reset_data_toggle(&mut self) {
            self.resets += 1;
        }
    }

    fn host(packet: &[u8]) -> FtdiHost<FakeControl, FakeBulkIn, FakeBulkOut> {
        FtdiHost::new(
            FT232R_DEVICE,
            FtdiInterface::discover(FT232R_DEVICE, &CONFIGURATION).unwrap(),
            FakeControl::new(),
            FakeBulkIn::new(packet),
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
    fn discovers_ft232r_interface() {
        let interface = FtdiInterface::discover(FT232R_DEVICE, &CONFIGURATION).unwrap();
        assert_eq!(interface.interface_number, 0);
        assert_eq!(interface.bulk_in_endpoint.address, 0x81);
        assert_eq!(interface.bulk_out_endpoint.address, 0x02);
        assert_eq!(FtdiChip::detect(FT232R_DEVICE), FtdiChip::Ft232R);
    }

    #[test]
    fn rejects_non_ftdi_device() {
        let mut device = FT232R_DEVICE;
        device.vendor_id = 0x1234;
        assert_eq!(
            FtdiInterface::discover(device, &CONFIGURATION),
            Err(FtdiConfigurationError::NotFtdiDevice)
        );
    }

    #[test]
    fn ft232r_baud_divisors_are_accurate() {
        for baud in [300, 9_600, 115_200, 1_000_000, 3_000_000] {
            let divisor = convert_baud_rate(baud, FtdiChip::Ft232R, 0).unwrap();
            assert!(divisor.actual.abs_diff(baud) as u64 * 20 <= baud as u64);
        }
        assert_eq!(
            convert_baud_rate(3_000_000, FtdiChip::Ft232R, 0)
                .unwrap()
                .value,
            0
        );
    }

    #[test]
    fn zero_baud_is_rejected() {
        assert_eq!(convert_baud_rate(0, FtdiChip::Ft232R, 0), None);
    }

    #[test]
    fn configure_115200_8n1_emits_exact_vendor_requests() {
        let mut host = host(&[0x01, 0x60]);
        assert_eq!(block_on(host.configure_8n1(115_200)), Ok(115_385));
        let control = host.control_pipe();
        assert_eq!(control.calls, 6);
        assert_eq!(control.setups[0], [0x40, 0x00, 0, 0, 0, 0, 0, 0]);
        assert_eq!(control.setups[1], [0x40, 0x00, 1, 0, 0, 0, 0, 0]);
        assert_eq!(control.setups[2], [0x40, 0x00, 2, 0, 0, 0, 0, 0]);
        assert_eq!(control.setups[3][..2], [0x40, 0x03]);
        assert_eq!(control.setups[4], [0x40, 0x04, 8, 0, 0, 0, 0, 0]);
        assert_eq!(control.setups[5], [0x40, 0x02, 0, 0, 0, 0, 0, 0]);
        assert_eq!(host.baud_rate(), 115_385);
    }

    #[test]
    fn read_strips_status_and_buffers_remaining_payload() {
        let mut host = host(&[0x31, 0x60, b'a', b'b', b'c']);
        let mut first = [0; 2];
        assert_eq!(block_on(host.read(&mut first)), Ok(2));
        assert_eq!(&first, b"ab");
        assert_eq!(
            host.status(),
            FtdiStatus {
                modem: 0x31,
                line: 0x60
            }
        );
        assert_eq!(host.bulk_in_pipe().calls, 1);

        let mut second = [0; 2];
        assert_eq!(block_on(host.read(&mut second)), Ok(1));
        assert_eq!(second[0], b'c');
        assert_eq!(host.bulk_in_pipe().calls, 1);
    }

    #[test]
    fn write_is_a_raw_bulk_payload() {
        let mut host = host(&[0x01, 0x60]);
        assert_eq!(block_on(host.write(b"loopback")), Ok(8));
        assert_eq!(host.bulk_out_pipe().calls, 1);
        assert_eq!(&host.bulk_out_pipe().bytes[..8], b"loopback");
    }
}
