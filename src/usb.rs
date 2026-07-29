// The TX encoding and RX decoding models are adapted from Pico-PIO-USB,
// Copyright (c) 2021 sekigon-gonnoc, under the MIT License.
// See THIRD_PARTY_NOTICES.md.

/// USB full-speed SYNC byte, transmitted least-significant bit first.
pub const SYNC: u8 = 0x80;

/// USB SOF packet identifier.
pub const PID_SOF: u8 = 0xa5;

/// USB OUT token packet identifier.
pub const PID_OUT: u8 = 0xe1;

/// USB IN token packet identifier.
pub const PID_IN: u8 = 0x69;

/// USB SETUP token packet identifier.
pub const PID_SETUP: u8 = 0x2d;

/// USB DATA0 packet identifier.
pub const PID_DATA0: u8 = 0xc3;

/// USB DATA1 packet identifier.
pub const PID_DATA1: u8 = 0x4b;

/// USB ACK handshake packet identifier.
pub const PID_ACK: u8 = 0xd2;

/// USB NAK handshake packet identifier.
pub const PID_NAK: u8 = 0x5a;

/// USB STALL handshake packet identifier.
pub const PID_STALL: u8 = 0x1e;

/// Standard USB SET_ADDRESS request code.
pub const REQUEST_SET_ADDRESS: u8 = 0x05;

/// Standard USB SET_CONFIGURATION request code.
pub const REQUEST_SET_CONFIGURATION: u8 = 0x09;

/// CDC-ACM SET_LINE_CODING request code.
pub const CDC_REQUEST_SET_LINE_CODING: u8 = 0x20;

/// CDC-ACM GET_LINE_CODING request code.
pub const CDC_REQUEST_GET_LINE_CODING: u8 = 0x21;

/// CDC-ACM SET_CONTROL_LINE_STATE request code.
pub const CDC_REQUEST_SET_CONTROL_LINE_STATE: u8 = 0x22;

/// CDC-ACM SEND_BREAK request code.
pub const CDC_REQUEST_SEND_BREAK: u8 = 0x23;

/// HID GET_REPORT class-request code.
pub const HID_REQUEST_GET_REPORT: u8 = 0x01;

/// HID GET_IDLE class-request code.
pub const HID_REQUEST_GET_IDLE: u8 = 0x02;

/// HID GET_PROTOCOL class-request code.
pub const HID_REQUEST_GET_PROTOCOL: u8 = 0x03;

/// HID SET_REPORT class-request code.
pub const HID_REQUEST_SET_REPORT: u8 = 0x09;

/// HID SET_IDLE class-request code.
pub const HID_REQUEST_SET_IDLE: u8 = 0x0a;

/// HID SET_PROTOCOL class-request code.
pub const HID_REQUEST_SET_PROTOCOL: u8 = 0x0b;

/// CDC-ACM capability bit for line-coding and control-line requests.
pub const CDC_ACM_CAPABILITY_LINE_REQUESTS: u8 = 0x02;

/// CDC-ACM capability bit for the SEND_BREAK request.
pub const CDC_ACM_CAPABILITY_SEND_BREAK: u8 = 0x04;

/// Maximum encoded size needed by the packets implemented through M3.
pub const MAX_ENCODED_BYTES: usize = 48;

/// Maximum raw packet size for a 64-byte endpoint payload.
pub const MAX_DECODED_BYTES: usize = 68;

/// USB device descriptor type.
pub const DESCRIPTOR_TYPE_DEVICE: u8 = 0x01;

/// USB configuration descriptor type.
pub const DESCRIPTOR_TYPE_CONFIGURATION: u8 = 0x02;

/// USB interface descriptor type.
pub const DESCRIPTOR_TYPE_INTERFACE: u8 = 0x04;

/// USB endpoint descriptor type.
pub const DESCRIPTOR_TYPE_ENDPOINT: u8 = 0x05;

/// USB class-specific interface descriptor type.
pub const DESCRIPTOR_TYPE_CLASS_INTERFACE: u8 = 0x24;

/// HID class descriptor type.
pub const DESCRIPTOR_TYPE_HID: u8 = 0x21;

/// HID report descriptor type.
pub const DESCRIPTOR_TYPE_HID_REPORT: u8 = 0x22;

/// USB Communications Device Class.
pub const USB_CLASS_COMMUNICATIONS: u8 = 0x02;

/// USB Abstract Control Model subclass.
pub const USB_SUBCLASS_ACM: u8 = 0x02;

/// USB CDC data-interface class.
pub const USB_CLASS_CDC_DATA: u8 = 0x0a;

/// USB Human Interface Device class.
pub const USB_CLASS_HID: u8 = 0x03;

/// CDC header functional-descriptor subtype.
pub const CDC_DESCRIPTOR_SUBTYPE_HEADER: u8 = 0x00;

/// CDC call-management functional-descriptor subtype.
pub const CDC_DESCRIPTOR_SUBTYPE_CALL_MANAGEMENT: u8 = 0x01;

/// CDC ACM functional-descriptor subtype.
pub const CDC_DESCRIPTOR_SUBTYPE_ACM: u8 = 0x02;

/// CDC union functional-descriptor subtype.
pub const CDC_DESCRIPTOR_SUBTYPE_UNION: u8 = 0x06;

/// Call Management bit indicating that call management uses a CDC Data
/// interface identified by `bDataInterface`.
pub const CDC_CALL_MANAGEMENT_CAPABILITY_DATA_INTERFACE: u8 = 0x02;

/// Call Management bit indicating that the device handles call management
/// itself.
pub const CDC_CALL_MANAGEMENT_CAPABILITY_HANDLES_CALLS: u8 = 0x01;

/// How a validated IN DATA packet affects the receive state.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum InDataDisposition {
    Accept,
    Duplicate,
    Reject,
}

/// Classify an IN DATA packet after its wire framing and CRC checks.
///
/// A wrong-toggle packet is a possible retransmission after a lost host ACK.
/// It must be ACKed and discarded even when it is longer than the request's
/// final remaining bytes, provided it still fits the endpoint receive buffer.
#[inline(always)]
pub const fn classify_in_data(
    wire_valid: bool,
    expected_pid: u8,
    max_expected_payload_len: usize,
    receive_capacity: usize,
    received_pid: u8,
    received_payload_len: usize,
) -> InDataDisposition {
    if !wire_valid
        || !matches!(expected_pid, PID_DATA0 | PID_DATA1)
        || !matches!(received_pid, PID_DATA0 | PID_DATA1)
        || max_expected_payload_len > receive_capacity
        || received_payload_len > receive_capacity
    {
        return InDataDisposition::Reject;
    }

    if received_pid != expected_pid {
        return InDataDisposition::Duplicate;
    }
    if received_payload_len <= max_expected_payload_len {
        InDataDisposition::Accept
    } else {
        InDataDisposition::Reject
    }
}

/// Number of stop bits encoded in a CDC-ACM line-coding request.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum CdcStopBits {
    One = 0,
    OnePointFive = 1,
    Two = 2,
}

/// Parity mode encoded in a CDC-ACM line-coding request.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum CdcParity {
    None = 0,
    Odd = 1,
    Even = 2,
    Mark = 3,
    Space = 4,
}

/// Error while validating a local CDC line coding or decoding a seven-byte
/// device response.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CdcLineCodingError {
    InvalidLength,
    InvalidStopBits,
    InvalidParity,
    InvalidDataBits,
}

/// The seven-byte payload used by CDC-ACM line-coding requests.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CdcLineCoding {
    pub data_terminal_rate: u32,
    pub stop_bits: CdcStopBits,
    pub parity: CdcParity,
    pub data_bits: u8,
}

impl CdcLineCoding {
    pub const ENCODED_LEN: usize = 7;

    /// Construct an arbitrary CDC-ACM line coding.
    pub const fn new(
        data_terminal_rate: u32,
        stop_bits: CdcStopBits,
        parity: CdcParity,
        data_bits: u8,
    ) -> Self {
        Self {
            data_terminal_rate,
            stop_bits,
            parity,
            data_bits,
        }
    }

    /// Construct an eight-data-bit, no-parity, one-stop-bit line coding.
    pub const fn eight_n_one(data_terminal_rate: u32) -> Self {
        Self::new(data_terminal_rate, CdcStopBits::One, CdcParity::None, 8)
    }

    /// Validate values that remain representable through the public fields.
    pub const fn validate(self) -> Result<(), CdcLineCodingError> {
        if !matches!(self.data_bits, 5 | 6 | 7 | 8 | 16) {
            return Err(CdcLineCodingError::InvalidDataBits);
        }
        Ok(())
    }

    /// Parse the seven-byte payload returned by GET_LINE_CODING.
    pub fn from_bytes(bytes: &[u8]) -> Result<Self, CdcLineCodingError> {
        if bytes.len() != Self::ENCODED_LEN {
            return Err(CdcLineCodingError::InvalidLength);
        }
        let stop_bits = match bytes[4] {
            0 => CdcStopBits::One,
            1 => CdcStopBits::OnePointFive,
            2 => CdcStopBits::Two,
            _ => return Err(CdcLineCodingError::InvalidStopBits),
        };
        let parity = match bytes[5] {
            0 => CdcParity::None,
            1 => CdcParity::Odd,
            2 => CdcParity::Even,
            3 => CdcParity::Mark,
            4 => CdcParity::Space,
            _ => return Err(CdcLineCodingError::InvalidParity),
        };
        let coding = Self::new(
            u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]),
            stop_bits,
            parity,
            bytes[6],
        );
        coding.validate()?;
        Ok(coding)
    }

    /// Serialize the CDC line-coding payload in little-endian wire order.
    pub const fn to_bytes(self) -> [u8; Self::ENCODED_LEN] {
        let rate = self.data_terminal_rate.to_le_bytes();
        [
            rate[0],
            rate[1],
            rate[2],
            rate[3],
            self.stop_bits as u8,
            self.parity as u8,
            self.data_bits,
        ]
    }
}

/// A USB setup request.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SetupRequest {
    pub request_type: u8,
    pub request: u8,
    pub value: u16,
    pub index: u16,
    pub length: u16,
}

/// Error while constructing a standard USB setup request.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SetupRequestError {
    InvalidAddress,
}

impl SetupRequest {
    /// Request a caller-selected number of bytes from the device descriptor.
    pub const fn get_device_descriptor(length: u16) -> Self {
        Self {
            request_type: 0x80,
            request: 0x06,
            value: (DESCRIPTOR_TYPE_DEVICE as u16) << 8,
            index: 0,
            length,
        }
    }

    /// Request the first eight bytes of the device descriptor.
    pub const fn get_device_descriptor_prefix() -> Self {
        Self::get_device_descriptor(8)
    }

    /// Request bytes from a configuration descriptor by zero-based index.
    pub const fn get_configuration_descriptor(index: u8, length: u16) -> Self {
        Self {
            request_type: 0x80,
            request: 0x06,
            value: ((DESCRIPTOR_TYPE_CONFIGURATION as u16) << 8) | index as u16,
            index: 0,
            length,
        }
    }

    /// Request that a device adopt a seven-bit USB address.
    pub const fn set_address(address: u8) -> Result<Self, SetupRequestError> {
        if address > 0x7f {
            return Err(SetupRequestError::InvalidAddress);
        }

        Ok(Self {
            request_type: 0x00,
            request: REQUEST_SET_ADDRESS,
            value: address as u16,
            index: 0,
            length: 0,
        })
    }

    /// Request that a device select the given configuration value.
    pub const fn set_configuration(configuration_value: u8) -> Self {
        Self {
            request_type: 0x00,
            request: REQUEST_SET_CONFIGURATION,
            value: configuration_value as u16,
            index: 0,
            length: 0,
        }
    }

    /// Read the HID report descriptor owned by an interface.
    pub const fn get_hid_report_descriptor(interface: u8, length: u16) -> Self {
        Self {
            request_type: 0x81,
            request: 0x06,
            value: (DESCRIPTOR_TYPE_HID_REPORT as u16) << 8,
            index: interface as u16,
            length,
        }
    }

    /// Read one HID input, output or feature report through endpoint zero.
    ///
    /// HID defines report types `1`, `2` and `3` for input, output and feature
    /// reports respectively. A zero `report_id` selects the sole report when
    /// the report descriptor does not use report IDs.
    pub const fn get_hid_report(
        interface: u8,
        report_type: u8,
        report_id: u8,
        length: u16,
    ) -> Self {
        Self {
            request_type: 0xa1,
            request: HID_REQUEST_GET_REPORT,
            value: ((report_type as u16) << 8) | report_id as u16,
            index: interface as u16,
            length,
        }
    }

    /// Write one HID output or feature report through endpoint zero.
    pub const fn set_hid_report(
        interface: u8,
        report_type: u8,
        report_id: u8,
        length: u16,
    ) -> Self {
        Self {
            request_type: 0x21,
            request: HID_REQUEST_SET_REPORT,
            value: ((report_type as u16) << 8) | report_id as u16,
            index: interface as u16,
            length,
        }
    }

    /// Read the HID idle duration for one report ID.
    pub const fn get_hid_idle(interface: u8, report_id: u8) -> Self {
        Self {
            request_type: 0xa1,
            request: HID_REQUEST_GET_IDLE,
            value: report_id as u16,
            index: interface as u16,
            length: 1,
        }
    }

    /// Set the HID idle duration, expressed in four-millisecond units.
    ///
    /// A zero `report_id` applies the duration to every input report.
    pub const fn set_hid_idle(interface: u8, report_id: u8, duration_4ms: u8) -> Self {
        Self {
            request_type: 0x21,
            request: HID_REQUEST_SET_IDLE,
            value: ((duration_4ms as u16) << 8) | report_id as u16,
            index: interface as u16,
            length: 0,
        }
    }

    /// Read whether a boot-capable HID interface uses boot or report protocol.
    pub const fn get_hid_protocol(interface: u8) -> Self {
        Self {
            request_type: 0xa1,
            request: HID_REQUEST_GET_PROTOCOL,
            value: 0,
            index: interface as u16,
            length: 1,
        }
    }

    /// Select boot protocol (`0`) or report protocol (`1`) on a HID interface.
    pub const fn set_hid_protocol(interface: u8, protocol: u8) -> Self {
        Self {
            request_type: 0x21,
            request: HID_REQUEST_SET_PROTOCOL,
            value: protocol as u16,
            index: interface as u16,
            length: 0,
        }
    }

    /// Set the CDC-ACM DTR and RTS state on a communications interface.
    pub const fn set_control_line_state(control_interface: u8, dtr: bool, rts: bool) -> Self {
        Self {
            request_type: 0x21,
            request: CDC_REQUEST_SET_CONTROL_LINE_STATE,
            value: (dtr as u16) | ((rts as u16) << 1),
            index: control_interface as u16,
            length: 0,
        }
    }

    /// Select the CDC-ACM line coding on a communications interface.
    pub const fn set_line_coding(control_interface: u8) -> Self {
        Self {
            request_type: 0x21,
            request: CDC_REQUEST_SET_LINE_CODING,
            value: 0,
            index: control_interface as u16,
            length: CdcLineCoding::ENCODED_LEN as u16,
        }
    }

    /// Read the current CDC-ACM line coding from a communications interface.
    pub const fn get_line_coding(control_interface: u8) -> Self {
        Self {
            request_type: 0xa1,
            request: CDC_REQUEST_GET_LINE_CODING,
            value: 0,
            index: control_interface as u16,
            length: CdcLineCoding::ENCODED_LEN as u16,
        }
    }

    /// Request a timed break condition.
    ///
    /// A zero duration stops a break. `0xffff` requests an indefinite break.
    pub const fn send_break(control_interface: u8, duration_ms: u16) -> Self {
        Self {
            request_type: 0x21,
            request: CDC_REQUEST_SEND_BREAK,
            value: duration_ms,
            index: control_interface as u16,
            length: 0,
        }
    }

    /// Serialize the request in the little-endian USB wire layout.
    pub const fn to_bytes(self) -> [u8; 8] {
        [
            self.request_type,
            self.request,
            self.value as u8,
            (self.value >> 8) as u8,
            self.index as u8,
            (self.index >> 8) as u8,
            self.length as u8,
            (self.length >> 8) as u8,
        ]
    }
}

/// The fields available in the first eight bytes of a USB device descriptor.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct DeviceDescriptorHeader {
    pub usb_version_bcd: u16,
    pub device_class: u8,
    pub device_subclass: u8,
    pub device_protocol: u8,
    pub max_packet_size0: u8,
}

/// Error in a device descriptor prefix.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DescriptorError {
    TooShort,
    InvalidLength,
    InvalidType,
    InvalidMaxPacketSize0,
}

impl DeviceDescriptorHeader {
    /// Parse and validate the first eight bytes of a device descriptor.
    pub fn parse(bytes: &[u8]) -> Result<Self, DescriptorError> {
        if bytes.len() < 8 {
            return Err(DescriptorError::TooShort);
        }
        if bytes[0] != 18 {
            return Err(DescriptorError::InvalidLength);
        }
        if bytes[1] != DESCRIPTOR_TYPE_DEVICE {
            return Err(DescriptorError::InvalidType);
        }
        if !matches!(bytes[7], 8 | 16 | 32 | 64) {
            return Err(DescriptorError::InvalidMaxPacketSize0);
        }

        Ok(Self {
            usb_version_bcd: u16::from_le_bytes([bytes[2], bytes[3]]),
            device_class: bytes[4],
            device_subclass: bytes[5],
            device_protocol: bytes[6],
            max_packet_size0: bytes[7],
        })
    }
}

/// A validated USB device descriptor.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct DeviceDescriptor {
    pub usb_version_bcd: u16,
    pub device_class: u8,
    pub device_subclass: u8,
    pub device_protocol: u8,
    pub max_packet_size0: u8,
    pub vendor_id: u16,
    pub product_id: u16,
    pub device_version_bcd: u16,
    pub manufacturer_string_index: u8,
    pub product_string_index: u8,
    pub serial_number_string_index: u8,
    pub num_configurations: u8,
}

impl DeviceDescriptor {
    /// Return the endpoint-zero fields shared with the eight-byte prefix.
    pub const fn header(&self) -> DeviceDescriptorHeader {
        DeviceDescriptorHeader {
            usb_version_bcd: self.usb_version_bcd,
            device_class: self.device_class,
            device_subclass: self.device_subclass,
            device_protocol: self.device_protocol,
            max_packet_size0: self.max_packet_size0,
        }
    }

    /// Parse and validate a complete USB device descriptor.
    ///
    /// USB device descriptors are 18 bytes long. A longer slice is accepted
    /// so callers can pass a packet payload without first slicing it.
    pub fn parse(bytes: &[u8]) -> Result<Self, DescriptorError> {
        if bytes.len() < 18 {
            return Err(DescriptorError::TooShort);
        }

        let header = DeviceDescriptorHeader::parse(bytes)?;
        Ok(Self {
            usb_version_bcd: header.usb_version_bcd,
            device_class: header.device_class,
            device_subclass: header.device_subclass,
            device_protocol: header.device_protocol,
            max_packet_size0: header.max_packet_size0,
            vendor_id: u16::from_le_bytes([bytes[8], bytes[9]]),
            product_id: u16::from_le_bytes([bytes[10], bytes[11]]),
            device_version_bcd: u16::from_le_bytes([bytes[12], bytes[13]]),
            manufacturer_string_index: bytes[14],
            product_string_index: bytes[15],
            serial_number_string_index: bytes[16],
            num_configurations: bytes[17],
        })
    }
}

/// The fixed fields in the leading nine-byte configuration descriptor.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ConfigurationDescriptorHeader {
    pub total_length: u16,
    pub num_interfaces: u8,
    pub configuration_value: u8,
    pub string_index: u8,
    pub attributes: u8,
    pub max_power_ma: u16,
}

/// Error while validating a configuration descriptor or CDC-ACM function.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ConfigurationError {
    TooShort,
    InvalidLength,
    InvalidType,
    InvalidTotalLength,
    InvalidConfigurationValue,
    InvalidAttributes,
    InvalidDescriptorLength,
    DescriptorOverrun,
    InvalidInterfaceDescriptor,
    InvalidFunctionalDescriptor,
    InvalidEndpointDescriptor,
    MissingControlInterface,
    MissingCdcHeader,
    MissingAcmDescriptor,
    MissingUnionDescriptor,
    InvalidUnionDescriptor,
    MissingDataInterface,
    AmbiguousDataInterface,
    MissingBulkInEndpoint,
    MissingBulkOutEndpoint,
}

impl ConfigurationDescriptorHeader {
    /// Parse and validate the leading standard configuration descriptor.
    pub fn parse(bytes: &[u8]) -> Result<Self, ConfigurationError> {
        if bytes.len() < 9 {
            return Err(ConfigurationError::TooShort);
        }
        if bytes[0] != 9 {
            return Err(ConfigurationError::InvalidLength);
        }
        if bytes[1] != DESCRIPTOR_TYPE_CONFIGURATION {
            return Err(ConfigurationError::InvalidType);
        }

        let total_length = u16::from_le_bytes([bytes[2], bytes[3]]);
        if total_length < 9 {
            return Err(ConfigurationError::InvalidTotalLength);
        }
        if bytes[5] == 0 {
            return Err(ConfigurationError::InvalidConfigurationValue);
        }
        if bytes[7] & 0x80 == 0 || bytes[7] & 0x1f != 0 {
            return Err(ConfigurationError::InvalidAttributes);
        }

        Ok(Self {
            total_length,
            num_interfaces: bytes[4],
            configuration_value: bytes[5],
            string_index: bytes[6],
            attributes: bytes[7],
            max_power_ma: bytes[8] as u16 * 2,
        })
    }
}

/// Iterates over the descriptors bounded by a configuration's wTotalLength.
pub struct DescriptorIter<'a> {
    bytes: &'a [u8],
    offset: usize,
    total_length: usize,
    failed: bool,
}

impl<'a> DescriptorIter<'a> {
    pub fn new(bytes: &'a [u8]) -> Result<Self, ConfigurationError> {
        let header = ConfigurationDescriptorHeader::parse(bytes)?;
        let total_length = header.total_length as usize;
        if bytes.len() < total_length {
            return Err(ConfigurationError::DescriptorOverrun);
        }

        Ok(Self {
            bytes,
            offset: 0,
            total_length,
            failed: false,
        })
    }
}

impl<'a> Iterator for DescriptorIter<'a> {
    type Item = Result<&'a [u8], ConfigurationError>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.failed || self.offset == self.total_length {
            return None;
        }
        if self.total_length - self.offset < 2 {
            self.failed = true;
            return Some(Err(ConfigurationError::DescriptorOverrun));
        }

        let descriptor_length = self.bytes[self.offset] as usize;
        if descriptor_length < 2 {
            self.failed = true;
            return Some(Err(ConfigurationError::InvalidDescriptorLength));
        }
        if descriptor_length > self.total_length - self.offset {
            self.failed = true;
            return Some(Err(ConfigurationError::DescriptorOverrun));
        }

        let start = self.offset;
        self.offset += descriptor_length;
        Some(Ok(&self.bytes[start..self.offset]))
    }
}

/// A validated non-control USB endpoint.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct EndpointDescriptor {
    pub address: u8,
    pub attributes: u8,
    pub max_packet_size: u16,
    pub interval: u8,
}

impl EndpointDescriptor {
    pub const fn number(self) -> u8 {
        self.address & 0x0f
    }

    pub const fn is_in(self) -> bool {
        self.address & 0x80 != 0
    }

    pub const fn transfer_type(self) -> u8 {
        self.attributes & 0x03
    }

    fn parse(bytes: &[u8]) -> Result<Self, ConfigurationError> {
        if bytes.len() != 7 || bytes[1] != DESCRIPTOR_TYPE_ENDPOINT {
            return Err(ConfigurationError::InvalidEndpointDescriptor);
        }

        let raw_max_packet_size = u16::from_le_bytes([bytes[4], bytes[5]]);
        let endpoint = Self {
            address: bytes[2],
            attributes: bytes[3],
            max_packet_size: raw_max_packet_size & 0x07ff,
            interval: bytes[6],
        };
        if endpoint.address & 0x70 != 0
            || endpoint.number() == 0
            || endpoint.max_packet_size == 0
            || raw_max_packet_size & 0xf800 != 0
        {
            return Err(ConfigurationError::InvalidEndpointDescriptor);
        }
        Ok(endpoint)
    }

    fn is_supported_bulk(self) -> bool {
        self.transfer_type() == 0x02
            && self.attributes & 0xfc == 0
            && matches!(self.max_packet_size, 8 | 16 | 32 | 64 | 512)
    }

    fn is_supported_interrupt_in(self) -> bool {
        self.is_in()
            && self.transfer_type() == 0x03
            && self.attributes & 0xfc == 0
            && self.max_packet_size <= 64
            && self.interval != 0
    }

    fn is_supported_hid_interrupt(self) -> bool {
        self.transfer_type() == 0x03
            && self.attributes & 0xfc == 0
            && self.max_packet_size <= 1024
            && self.interval != 0
    }
}

/// The interfaces and endpoints needed by a first CDC-ACM class driver.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CdcAcmFunction {
    pub configuration: ConfigurationDescriptorHeader,
    pub control_interface: u8,
    pub data_interface: u8,
    pub cdc_version_bcd: u16,
    pub acm_capabilities: u8,
    pub notification_endpoint: Option<EndpointDescriptor>,
    pub bulk_out_endpoint: EndpointDescriptor,
    pub bulk_in_endpoint: EndpointDescriptor,
}

#[derive(Clone, Copy)]
struct InterfaceDescriptor {
    number: u8,
    alternate_setting: u8,
    endpoint_count: u8,
    class: u8,
    subclass: u8,
    protocol: u8,
}

impl InterfaceDescriptor {
    fn parse(bytes: &[u8]) -> Result<Self, ConfigurationError> {
        if bytes.len() != 9 || bytes[1] != DESCRIPTOR_TYPE_INTERFACE {
            return Err(ConfigurationError::InvalidInterfaceDescriptor);
        }

        Ok(Self {
            number: bytes[2],
            alternate_setting: bytes[3],
            endpoint_count: bytes[4],
            class: bytes[5],
            subclass: bytes[6],
            protocol: bytes[7],
        })
    }

    const fn is_acm_control(self) -> bool {
        self.alternate_setting == 0
            && self.class == USB_CLASS_COMMUNICATIONS
            && self.subclass == USB_SUBCLASS_ACM
    }

    const fn is_hid(self) -> bool {
        self.alternate_setting == 0 && self.class == USB_CLASS_HID
    }
}

/// Error while discovering and validating one HID interface.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum HidConfigurationError {
    /// The enclosing USB configuration descriptor is malformed.
    Configuration(ConfigurationError),
    /// No alternate-setting-zero HID interface was found.
    MissingInterface,
    /// The selected HID interface descriptor is malformed or duplicated.
    InvalidInterfaceDescriptor,
    /// The selected interface has no HID class descriptor.
    MissingHidDescriptor,
    /// The HID class descriptor is malformed.
    InvalidHidDescriptor,
    /// More than one HID class descriptor belongs to the selected interface.
    DuplicateHidDescriptor,
    /// The HID class descriptor does not identify a report descriptor.
    MissingReportDescriptor,
    /// The HID class descriptor identifies more than one report descriptor.
    DuplicateReportDescriptor,
    /// An endpoint descriptor belonging to the HID interface is malformed,
    /// unsupported or inconsistent with `bNumEndpoints`.
    InvalidEndpointDescriptor,
    /// The HID interface has no interrupt-IN endpoint.
    MissingInterruptInEndpoint,
    /// The HID interface has more than one interrupt-IN endpoint.
    DuplicateInterruptInEndpoint,
    /// The HID interface has more than one interrupt-OUT endpoint.
    DuplicateInterruptOutEndpoint,
}

impl From<ConfigurationError> for HidConfigurationError {
    fn from(error: ConfigurationError) -> Self {
        Self::Configuration(error)
    }
}

/// A validated alternate-setting-zero HID interface and its report endpoints.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct HidInterface {
    /// USB interface number used as `wIndex` by HID class requests.
    pub interface_number: u8,
    /// HID interface subclass from the standard interface descriptor.
    pub interface_subclass: u8,
    /// HID interface protocol from the standard interface descriptor.
    pub interface_protocol: u8,
    /// HID specification version from the HID class descriptor.
    pub hid_version_bcd: u16,
    /// HID country code from the HID class descriptor.
    pub country_code: u8,
    /// Declared byte length of the HID report descriptor.
    pub report_descriptor_len: u16,
    /// Required interrupt-IN endpoint carrying input reports.
    pub interrupt_in_endpoint: EndpointDescriptor,
    /// Optional interrupt-OUT endpoint carrying output reports.
    pub interrupt_out_endpoint: Option<EndpointDescriptor>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct HidDescriptor {
    hid_version_bcd: u16,
    country_code: u8,
    report_descriptor_len: u16,
}

impl HidDescriptor {
    fn parse(bytes: &[u8]) -> Result<Self, HidConfigurationError> {
        if bytes.len() < 6 || bytes[1] != DESCRIPTOR_TYPE_HID {
            return Err(HidConfigurationError::InvalidHidDescriptor);
        }

        let descriptor_count = bytes[5] as usize;
        let expected_len = 6_usize
            .checked_add(
                descriptor_count
                    .checked_mul(3)
                    .ok_or(HidConfigurationError::InvalidHidDescriptor)?,
            )
            .ok_or(HidConfigurationError::InvalidHidDescriptor)?;
        if bytes.len() != expected_len {
            return Err(HidConfigurationError::InvalidHidDescriptor);
        }

        let mut report_descriptor_len = None;
        for descriptor in bytes[6..].chunks_exact(3) {
            let descriptor_type = descriptor[0];
            let descriptor_len = u16::from_le_bytes([descriptor[1], descriptor[2]]);
            if descriptor_len == 0 {
                return Err(HidConfigurationError::InvalidHidDescriptor);
            }
            if descriptor_type == DESCRIPTOR_TYPE_HID_REPORT
                && report_descriptor_len.replace(descriptor_len).is_some()
            {
                return Err(HidConfigurationError::DuplicateReportDescriptor);
            }
        }

        Ok(Self {
            hid_version_bcd: u16::from_le_bytes([bytes[2], bytes[3]]),
            country_code: bytes[4],
            report_descriptor_len: report_descriptor_len
                .ok_or(HidConfigurationError::MissingReportDescriptor)?,
        })
    }
}

/// Iterates over independently validated HID interfaces in a configuration.
///
/// Candidate-local errors are returned for that interface and iteration then
/// continues, allowing a caller to select a later valid HID interface in a
/// composite device.
pub struct HidInterfaces<'a> {
    bytes: &'a [u8],
    descriptors: DescriptorIter<'a>,
    seen_interfaces: [u32; 8],
}

impl<'a> HidInterfaces<'a> {
    /// Validate the configuration framing and create a HID-interface iterator.
    pub fn new(bytes: &'a [u8]) -> Result<Self, HidConfigurationError> {
        Ok(Self {
            bytes,
            descriptors: DescriptorIter::new(bytes)?,
            seen_interfaces: [0; 8],
        })
    }

    fn mark_seen(&mut self, interface: u8) -> bool {
        let word = usize::from(interface / 32);
        let mask = 1_u32 << (interface % 32);
        let was_seen = self.seen_interfaces[word] & mask != 0;
        self.seen_interfaces[word] |= mask;
        was_seen
    }
}

impl Iterator for HidInterfaces<'_> {
    type Item = Result<HidInterface, HidConfigurationError>;

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            let descriptor = match self.descriptors.next()? {
                Ok(descriptor) => descriptor,
                Err(error) => return Some(Err(error.into())),
            };
            if descriptor[1] != DESCRIPTOR_TYPE_INTERFACE {
                continue;
            }

            let interface = match InterfaceDescriptor::parse(descriptor) {
                Ok(interface) => interface,
                Err(_) => {
                    if descriptor.len() >= 7 && descriptor[3] == 0 && descriptor[5] == USB_CLASS_HID
                    {
                        if self.mark_seen(descriptor[2]) {
                            continue;
                        }
                        return Some(Err(HidConfigurationError::InvalidInterfaceDescriptor));
                    }
                    continue;
                }
            };
            if !interface.is_hid() || self.mark_seen(interface.number) {
                continue;
            }

            return Some(HidInterface::discover_candidate(
                self.bytes,
                interface.number,
            ));
        }
    }
}

impl HidInterface {
    /// Discover the first valid alternate-setting-zero HID interface.
    ///
    /// Malformed candidates are skipped while looking for a later valid HID
    /// interface. If none is valid, the first candidate error is returned.
    pub fn discover(bytes: &[u8]) -> Result<Self, HidConfigurationError> {
        let mut first_error = None;
        for interface in HidInterfaces::new(bytes)? {
            match interface {
                Ok(interface) => return Ok(interface),
                Err(error) => {
                    if first_error.is_none() {
                        first_error = Some(error);
                    }
                }
            }
        }
        Err(first_error.unwrap_or(HidConfigurationError::MissingInterface))
    }

    /// Discover and validate a particular alternate-setting-zero HID interface.
    pub fn discover_interface(
        bytes: &[u8],
        interface_number: u8,
    ) -> Result<Self, HidConfigurationError> {
        DescriptorIter::new(bytes)?;
        Self::discover_candidate(bytes, interface_number)
    }

    fn discover_candidate(
        bytes: &[u8],
        requested_interface: u8,
    ) -> Result<Self, HidConfigurationError> {
        let mut current_interface = None;
        let mut selected_interface = None;
        let mut hid_descriptor = None;
        let mut endpoints_seen = 0_u8;
        let mut interrupt_in_endpoint = None;
        let mut interrupt_out_endpoint = None;

        for descriptor in DescriptorIter::new(bytes)? {
            let descriptor = descriptor?;
            match descriptor[1] {
                DESCRIPTOR_TYPE_INTERFACE => {
                    let interface = match InterfaceDescriptor::parse(descriptor) {
                        Ok(interface) => interface,
                        Err(_) => {
                            current_interface = None;
                            if descriptor.len() >= 4
                                && descriptor[2] == requested_interface
                                && descriptor[3] == 0
                            {
                                return Err(HidConfigurationError::InvalidInterfaceDescriptor);
                            }
                            continue;
                        }
                    };
                    current_interface = Some(interface);
                    if interface.number == requested_interface
                        && interface.alternate_setting == 0
                        && selected_interface.replace(interface).is_some()
                    {
                        return Err(HidConfigurationError::InvalidInterfaceDescriptor);
                    }
                }
                DESCRIPTOR_TYPE_HID => {
                    let Some(interface) = current_interface else {
                        continue;
                    };
                    if interface.number != requested_interface || !interface.is_hid() {
                        continue;
                    }
                    let parsed = HidDescriptor::parse(descriptor)?;
                    if hid_descriptor.replace(parsed).is_some() {
                        return Err(HidConfigurationError::DuplicateHidDescriptor);
                    }
                }
                DESCRIPTOR_TYPE_ENDPOINT => {
                    let Some(interface) = current_interface else {
                        continue;
                    };
                    if interface.number != requested_interface || !interface.is_hid() {
                        continue;
                    }

                    let endpoint = EndpointDescriptor::parse(descriptor)
                        .map_err(|_| HidConfigurationError::InvalidEndpointDescriptor)?;
                    endpoints_seen = endpoints_seen
                        .checked_add(1)
                        .ok_or(HidConfigurationError::InvalidEndpointDescriptor)?;
                    if !endpoint.is_supported_hid_interrupt() {
                        return Err(HidConfigurationError::InvalidEndpointDescriptor);
                    }

                    if endpoint.is_in() {
                        if interrupt_in_endpoint.replace(endpoint).is_some() {
                            return Err(HidConfigurationError::DuplicateInterruptInEndpoint);
                        }
                    } else if interrupt_out_endpoint.replace(endpoint).is_some() {
                        return Err(HidConfigurationError::DuplicateInterruptOutEndpoint);
                    }
                }
                _ => {}
            }
        }

        let interface = selected_interface.ok_or(HidConfigurationError::MissingInterface)?;
        if !interface.is_hid() {
            return Err(HidConfigurationError::MissingInterface);
        }
        if interface.endpoint_count != endpoints_seen || !matches!(interface.endpoint_count, 1 | 2)
        {
            return Err(HidConfigurationError::InvalidEndpointDescriptor);
        }

        let hid_descriptor = hid_descriptor.ok_or(HidConfigurationError::MissingHidDescriptor)?;
        let interrupt_in_endpoint =
            interrupt_in_endpoint.ok_or(HidConfigurationError::MissingInterruptInEndpoint)?;

        Ok(Self {
            interface_number: requested_interface,
            interface_subclass: interface.subclass,
            interface_protocol: interface.protocol,
            hid_version_bcd: hid_descriptor.hid_version_bcd,
            country_code: hid_descriptor.country_code,
            report_descriptor_len: hid_descriptor.report_descriptor_len,
            interrupt_in_endpoint,
            interrupt_out_endpoint,
        })
    }
}

#[derive(Clone, Copy)]
struct DataInterface {
    number: u8,
    bulk_out_endpoint: EndpointDescriptor,
    bulk_in_endpoint: EndpointDescriptor,
}

/// Iterates over the independently validated CDC-ACM functions in a
/// configuration descriptor.
///
/// Candidate-local errors are returned for that function and iteration then
/// continues. This lets a composite-device user select a later valid function
/// even when another CDC-ACM function is malformed.
pub struct CdcAcmFunctions<'a> {
    bytes: &'a [u8],
    descriptors: DescriptorIter<'a>,
    seen_control_interfaces: [u32; 8],
}

impl<'a> CdcAcmFunctions<'a> {
    /// Validate the configuration framing and create a CDC-ACM iterator.
    pub fn new(bytes: &'a [u8]) -> Result<Self, ConfigurationError> {
        Ok(Self {
            bytes,
            descriptors: DescriptorIter::new(bytes)?,
            seen_control_interfaces: [0; 8],
        })
    }

    fn mark_seen(&mut self, interface: u8) -> bool {
        let word = usize::from(interface / 32);
        let mask = 1_u32 << (interface % 32);
        let was_seen = self.seen_control_interfaces[word] & mask != 0;
        self.seen_control_interfaces[word] |= mask;
        was_seen
    }
}

impl Iterator for CdcAcmFunctions<'_> {
    type Item = Result<CdcAcmFunction, ConfigurationError>;

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            let descriptor = match self.descriptors.next()? {
                Ok(descriptor) => descriptor,
                Err(error) => return Some(Err(error)),
            };
            if descriptor[1] != DESCRIPTOR_TYPE_INTERFACE {
                continue;
            }

            let interface = match InterfaceDescriptor::parse(descriptor) {
                Ok(interface) => interface,
                Err(error) => {
                    // A malformed interface descriptor is candidate-local
                    // when it still contains enough fields to identify an ACM
                    // control interface. Other malformed interface
                    // descriptors belong to unrelated functions.
                    if descriptor.len() >= 7
                        && descriptor[3] == 0
                        && descriptor[5] == USB_CLASS_COMMUNICATIONS
                        && descriptor[6] == USB_SUBCLASS_ACM
                    {
                        if self.mark_seen(descriptor[2]) {
                            continue;
                        }
                        return Some(Err(error));
                    }
                    continue;
                }
            };
            if !interface.is_acm_control() || self.mark_seen(interface.number) {
                continue;
            }

            return Some(CdcAcmFunction::discover_candidate(
                self.bytes,
                interface.number,
            ));
        }
    }
}

impl CdcAcmFunction {
    /// Whether the ACM functional descriptor advertises line requests.
    pub const fn supports_line_requests(self) -> bool {
        self.acm_capabilities & CDC_ACM_CAPABILITY_LINE_REQUESTS != 0
    }

    /// Whether the ACM functional descriptor advertises SEND_BREAK.
    pub const fn supports_send_break(self) -> bool {
        self.acm_capabilities & CDC_ACM_CAPABILITY_SEND_BREAK != 0
    }

    /// Discover the first valid alternate-setting-zero CDC-ACM function.
    ///
    /// Malformed candidates are skipped while looking for a later valid
    /// function. If no candidate is valid, the first candidate error is
    /// returned.
    pub fn discover(bytes: &[u8]) -> Result<Self, ConfigurationError> {
        let mut first_error = None;
        for function in CdcAcmFunctions::new(bytes)? {
            match function {
                Ok(function) => return Ok(function),
                Err(error) => {
                    if first_error.is_none() {
                        first_error = Some(error);
                    }
                }
            }
        }

        Err(first_error.unwrap_or(ConfigurationError::MissingControlInterface))
    }

    /// Discover the CDC-ACM function whose communications interface has the
    /// requested number.
    ///
    /// Candidate-local descriptor errors in other functions do not affect
    /// this explicit selection.
    pub fn discover_control_interface(
        bytes: &[u8],
        control_interface: u8,
    ) -> Result<Self, ConfigurationError> {
        // Validate the configuration framing before candidate-local parsing.
        DescriptorIter::new(bytes)?;
        Self::discover_candidate(bytes, control_interface)
    }

    fn discover_candidate(
        bytes: &[u8],
        requested_control_interface: u8,
    ) -> Result<Self, ConfigurationError> {
        let configuration = ConfigurationDescriptorHeader::parse(bytes)?;
        let mut current_interface = None;
        let mut control_descriptor = None;
        let mut control_endpoints_seen = 0_u8;
        let mut notification_endpoint = None;
        let mut first_functional_descriptor_seen = false;
        let mut cdc_version_bcd = None;
        let mut acm_capabilities = None;
        let mut union_descriptor = None;
        let mut call_management = None;

        for descriptor in DescriptorIter::new(bytes)? {
            let descriptor = descriptor?;
            match descriptor[1] {
                DESCRIPTOR_TYPE_INTERFACE => {
                    let interface = match InterfaceDescriptor::parse(descriptor) {
                        Ok(interface) => interface,
                        Err(error) => {
                            current_interface = None;
                            if descriptor.len() >= 4
                                && descriptor[2] == requested_control_interface
                                && descriptor[3] == 0
                            {
                                return Err(error);
                            }
                            continue;
                        }
                    };
                    current_interface = Some(interface);
                    if interface.number == requested_control_interface
                        && interface.alternate_setting == 0
                        && control_descriptor.replace(interface).is_some()
                    {
                        return Err(ConfigurationError::InvalidInterfaceDescriptor);
                    }
                }
                DESCRIPTOR_TYPE_CLASS_INTERFACE => {
                    let Some(interface) = current_interface else {
                        continue;
                    };
                    if interface.number != requested_control_interface
                        || interface.alternate_setting != 0
                        || !interface.is_acm_control()
                    {
                        continue;
                    }
                    if descriptor.len() < 3 {
                        return Err(ConfigurationError::InvalidFunctionalDescriptor);
                    }
                    if !first_functional_descriptor_seen {
                        first_functional_descriptor_seen = true;
                        if descriptor[2] != CDC_DESCRIPTOR_SUBTYPE_HEADER {
                            return Err(ConfigurationError::InvalidFunctionalDescriptor);
                        }
                    }

                    match descriptor[2] {
                        CDC_DESCRIPTOR_SUBTYPE_HEADER => {
                            if descriptor.len() != 5 || cdc_version_bcd.is_some() {
                                return Err(ConfigurationError::InvalidFunctionalDescriptor);
                            }
                            cdc_version_bcd =
                                Some(u16::from_le_bytes([descriptor[3], descriptor[4]]));
                        }
                        CDC_DESCRIPTOR_SUBTYPE_CALL_MANAGEMENT => {
                            if descriptor.len() != 5
                                || call_management.is_some()
                                || descriptor[3] & !0x03 != 0
                                || descriptor[3] & 0x03
                                    == CDC_CALL_MANAGEMENT_CAPABILITY_DATA_INTERFACE
                            {
                                return Err(ConfigurationError::InvalidFunctionalDescriptor);
                            }
                            call_management = Some((descriptor[3], descriptor[4]));
                        }
                        CDC_DESCRIPTOR_SUBTYPE_ACM => {
                            if descriptor.len() != 4
                                || acm_capabilities.is_some()
                                || descriptor[3] & 0xf0 != 0
                            {
                                return Err(ConfigurationError::InvalidFunctionalDescriptor);
                            }
                            acm_capabilities = Some(descriptor[3]);
                        }
                        CDC_DESCRIPTOR_SUBTYPE_UNION => {
                            if descriptor.len() < 5 || union_descriptor.is_some() {
                                return Err(ConfigurationError::InvalidFunctionalDescriptor);
                            }
                            union_descriptor = Some(descriptor);
                        }
                        _ => {}
                    }
                }
                DESCRIPTOR_TYPE_ENDPOINT => {
                    let Some(interface) = current_interface else {
                        continue;
                    };
                    if interface.number != requested_control_interface
                        || interface.alternate_setting != 0
                        || !interface.is_acm_control()
                    {
                        continue;
                    }

                    let endpoint = EndpointDescriptor::parse(descriptor)?;
                    control_endpoints_seen = control_endpoints_seen
                        .checked_add(1)
                        .ok_or(ConfigurationError::InvalidEndpointDescriptor)?;
                    if !endpoint.is_supported_interrupt_in()
                        || notification_endpoint.replace(endpoint).is_some()
                    {
                        return Err(ConfigurationError::InvalidEndpointDescriptor);
                    }
                }
                _ => {}
            }
        }

        let control_descriptor =
            control_descriptor.ok_or(ConfigurationError::MissingControlInterface)?;
        if !control_descriptor.is_acm_control() {
            return Err(ConfigurationError::MissingControlInterface);
        }
        if control_descriptor.endpoint_count > 1
            || control_descriptor.endpoint_count != control_endpoints_seen
        {
            return Err(ConfigurationError::InvalidEndpointDescriptor);
        }

        let cdc_version_bcd = cdc_version_bcd.ok_or(ConfigurationError::MissingCdcHeader)?;
        let acm_capabilities = acm_capabilities.ok_or(ConfigurationError::MissingAcmDescriptor)?;
        let union_descriptor =
            union_descriptor.ok_or(ConfigurationError::MissingUnionDescriptor)?;
        if union_descriptor[3] != requested_control_interface {
            return Err(ConfigurationError::InvalidUnionDescriptor);
        }

        let subordinate_interfaces = &union_descriptor[4..];
        for (index, subordinate) in subordinate_interfaces.iter().copied().enumerate() {
            if subordinate == requested_control_interface
                || subordinate_interfaces[..index].contains(&subordinate)
            {
                return Err(ConfigurationError::InvalidUnionDescriptor);
            }
            if !Self::interface_exists(bytes, subordinate)? {
                return Err(ConfigurationError::InvalidUnionDescriptor);
            }
        }

        let managed_data_interface = call_management.and_then(|(capabilities, data_interface)| {
            (capabilities & 0x03
                == (CDC_CALL_MANAGEMENT_CAPABILITY_HANDLES_CALLS
                    | CDC_CALL_MANAGEMENT_CAPABILITY_DATA_INTERFACE))
                .then_some(data_interface)
        });
        let data_interface = if let Some(call_management_interface) = managed_data_interface {
            if !subordinate_interfaces.contains(&call_management_interface) {
                return Err(ConfigurationError::InvalidUnionDescriptor);
            }
            Self::discover_data_interface(bytes, call_management_interface)?
                .ok_or(ConfigurationError::MissingDataInterface)?
        } else {
            let mut selected = None;
            for subordinate in subordinate_interfaces.iter().copied() {
                let Some(candidate) = Self::discover_data_interface(bytes, subordinate)? else {
                    continue;
                };
                if selected.replace(candidate).is_some() {
                    return Err(ConfigurationError::AmbiguousDataInterface);
                }
            }
            selected.ok_or(ConfigurationError::MissingDataInterface)?
        };

        if notification_endpoint.is_some_and(|notification| {
            notification.address == data_interface.bulk_in_endpoint.address
        }) {
            return Err(ConfigurationError::InvalidEndpointDescriptor);
        }

        Ok(Self {
            configuration,
            control_interface: requested_control_interface,
            data_interface: data_interface.number,
            cdc_version_bcd,
            acm_capabilities,
            notification_endpoint,
            bulk_out_endpoint: data_interface.bulk_out_endpoint,
            bulk_in_endpoint: data_interface.bulk_in_endpoint,
        })
    }

    fn discover_data_interface(
        bytes: &[u8],
        requested_data_interface: u8,
    ) -> Result<Option<DataInterface>, ConfigurationError> {
        let mut current_interface = None;
        let mut data_descriptor = None;
        let mut endpoints_seen = 0_u8;
        let mut bulk_out_endpoint = None;
        let mut bulk_in_endpoint = None;

        for descriptor in DescriptorIter::new(bytes)? {
            let descriptor = descriptor?;
            match descriptor[1] {
                DESCRIPTOR_TYPE_INTERFACE => {
                    let interface = match InterfaceDescriptor::parse(descriptor) {
                        Ok(interface) => interface,
                        Err(error) => {
                            current_interface = None;
                            if descriptor.len() >= 4
                                && descriptor[2] == requested_data_interface
                                && descriptor[3] == 0
                            {
                                return Err(error);
                            }
                            continue;
                        }
                    };
                    current_interface = Some(interface);
                    if interface.number == requested_data_interface
                        && interface.alternate_setting == 0
                        && data_descriptor.replace(interface).is_some()
                    {
                        return Err(ConfigurationError::InvalidInterfaceDescriptor);
                    }
                }
                DESCRIPTOR_TYPE_ENDPOINT => {
                    let Some(interface) = current_interface else {
                        continue;
                    };
                    if interface.number != requested_data_interface
                        || interface.alternate_setting != 0
                    {
                        continue;
                    }

                    let endpoint = EndpointDescriptor::parse(descriptor)?;
                    endpoints_seen = endpoints_seen
                        .checked_add(1)
                        .ok_or(ConfigurationError::InvalidEndpointDescriptor)?;
                    if !endpoint.is_supported_bulk() {
                        return Err(ConfigurationError::InvalidEndpointDescriptor);
                    }
                    let slot = if endpoint.is_in() {
                        &mut bulk_in_endpoint
                    } else {
                        &mut bulk_out_endpoint
                    };
                    if slot.replace(endpoint).is_some() {
                        return Err(ConfigurationError::InvalidEndpointDescriptor);
                    }
                }
                _ => {}
            }
        }

        let Some(data_descriptor) = data_descriptor else {
            return Ok(None);
        };
        if data_descriptor.class != USB_CLASS_CDC_DATA {
            return Ok(None);
        }
        if data_descriptor.subclass != 0 {
            return Err(ConfigurationError::InvalidInterfaceDescriptor);
        }
        if data_descriptor.endpoint_count != endpoints_seen {
            return Err(ConfigurationError::InvalidEndpointDescriptor);
        }

        let bulk_out_endpoint =
            bulk_out_endpoint.ok_or(ConfigurationError::MissingBulkOutEndpoint)?;
        let bulk_in_endpoint = bulk_in_endpoint.ok_or(ConfigurationError::MissingBulkInEndpoint)?;
        if data_descriptor.endpoint_count != 2 {
            return Err(ConfigurationError::InvalidEndpointDescriptor);
        }

        Ok(Some(DataInterface {
            number: requested_data_interface,
            bulk_out_endpoint,
            bulk_in_endpoint,
        }))
    }

    fn interface_exists(bytes: &[u8], requested_interface: u8) -> Result<bool, ConfigurationError> {
        for descriptor in DescriptorIter::new(bytes)? {
            let descriptor = descriptor?;
            if descriptor[1] != DESCRIPTOR_TYPE_INTERFACE {
                continue;
            }

            match InterfaceDescriptor::parse(descriptor) {
                Ok(interface)
                    if interface.number == requested_interface
                        && interface.alternate_setting == 0 =>
                {
                    return Ok(true);
                }
                Err(error)
                    if descriptor.len() >= 4
                        && descriptor[2] == requested_interface
                        && descriptor[3] == 0 =>
                {
                    return Err(error);
                }
                _ => {}
            }
        }
        Ok(false)
    }
}

/// A two-bit command consumed by the USB TX PIO program.
///
/// The numeric values are PIO instruction addresses. `J` and `K` select the
/// next NRZI line state, while `Se0` and `Release` implement EOP.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum TxSymbol {
    Se0 = 0,
    K = 1,
    Release = 2,
    J = 3,
}

/// Error returned when a packet cannot fit in the fixed M2 TX buffer.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct EncodeError;

/// A differential USB line state during a packet.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum WireState {
    J,
    K,
}

/// Error detected while decoding NRZI wire states.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum NrziError {
    BitStuff,
    TruncatedByte,
    PacketTooLong,
}

/// Bytes recovered from NRZI wire states.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct DecodedBytes {
    bytes: [u8; MAX_DECODED_BYTES],
    len: usize,
}

impl DecodedBytes {
    /// Return the decoded bytes.
    pub fn as_bytes(&self) -> &[u8] {
        &self.bytes[..self.len]
    }
}

/// Decode USB NRZI states and remove stuffed zero bits.
///
/// The first state is interpreted relative to the full-speed idle J state.
pub fn decode_nrzi(states: &[WireState]) -> Result<DecodedBytes, NrziError> {
    let mut result = DecodedBytes {
        bytes: [0; MAX_DECODED_BYTES],
        len: 0,
    };
    let mut previous = WireState::J;
    let mut consecutive_ones = 0_u8;
    let mut bit_index = 0_u8;

    for &state in states {
        let bit_is_one = state == previous;
        previous = state;

        if consecutive_ones == 6 {
            if bit_is_one {
                return Err(NrziError::BitStuff);
            }
            consecutive_ones = 0;
            continue;
        }

        if result.len >= result.bytes.len() {
            return Err(NrziError::PacketTooLong);
        }

        if bit_is_one {
            result.bytes[result.len] |= 1 << bit_index;
            consecutive_ones += 1;
        } else {
            consecutive_ones = 0;
        }

        bit_index += 1;
        if bit_index == 8 {
            bit_index = 0;
            result.len += 1;
        }
    }

    if bit_index != 0 {
        return Err(NrziError::TruncatedByte);
    }

    Ok(result)
}

/// Error found while validating decoded USB packet bytes.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PacketError {
    TooShort,
    InvalidSync,
    InvalidPid,
    InvalidLength,
    InvalidCrc5,
    InvalidCrc16,
    UnsupportedPid,
}

/// A validated USB packet.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ParsedPacket<'a> {
    Handshake { pid: u8 },
    Token { pid: u8, value: u16 },
    Data { pid: u8, payload: &'a [u8] },
}

/// Error while constructing an unencoded USB data packet.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DataPacketError {
    InvalidPid,
    PayloadTooLong,
}

/// Raw SYNC/PID/payload/CRC bytes for a USB DATA0 or DATA1 packet.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RawDataPacket {
    bytes: [u8; MAX_DECODED_BYTES],
    len: usize,
}

/// The DATA0/DATA1 sequence state for one non-control endpoint direction.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum DataToggle {
    #[default]
    Data0,
    Data1,
}

impl DataToggle {
    /// Return the packet identifier for the current sequence state.
    pub const fn pid(self) -> u8 {
        match self {
            Self::Data0 => PID_DATA0,
            Self::Data1 => PID_DATA1,
        }
    }

    /// Advance the sequence after, and only after, a successful ACK.
    pub const fn after_ack(self) -> Self {
        match self {
            Self::Data0 => Self::Data1,
            Self::Data1 => Self::Data0,
        }
    }
}

/// Persistent DATA-toggle state for the two CDC-ACM bulk directions.
///
/// USB maintains independent sequence state per endpoint direction. A command
/// session must therefore retain both toggles across application command
/// boundaries instead of restarting either direction at DATA0.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct CdcAcmDataState {
    bulk_out: DataToggle,
    bulk_in: DataToggle,
}

impl CdcAcmDataState {
    pub const fn new() -> Self {
        Self {
            bulk_out: DataToggle::Data0,
            bulk_in: DataToggle::Data0,
        }
    }

    pub const fn bulk_out_pid(self) -> u8 {
        self.bulk_out.pid()
    }

    pub const fn bulk_in_pid(self) -> u8 {
        self.bulk_in.pid()
    }

    pub const fn after_bulk_out_ack(mut self) -> Self {
        self.bulk_out = self.bulk_out.after_ack();
        self
    }

    pub const fn after_bulk_in_ack(mut self) -> Self {
        self.bulk_in = self.bulk_in.after_ack();
        self
    }
}

impl RawDataPacket {
    /// Build a data packet for a payload of up to 64 bytes.
    pub fn new(pid: u8, payload: &[u8]) -> Result<Self, DataPacketError> {
        if !matches!(pid, PID_DATA0 | PID_DATA1) {
            return Err(DataPacketError::InvalidPid);
        }
        if payload.len() > MAX_DECODED_BYTES - 4 {
            return Err(DataPacketError::PayloadTooLong);
        }

        let mut packet = Self {
            bytes: [0; MAX_DECODED_BYTES],
            len: payload.len() + 4,
        };
        packet.bytes[0] = SYNC;
        packet.bytes[1] = pid;
        packet.bytes[2..2 + payload.len()].copy_from_slice(payload);
        let crc = crc16_data(payload);
        packet.bytes[2 + payload.len()] = crc as u8;
        packet.bytes[3 + payload.len()] = (crc >> 8) as u8;
        Ok(packet)
    }

    /// Return the meaningful packet bytes.
    pub fn as_bytes(&self) -> &[u8] {
        &self.bytes[..self.len]
    }
}

/// Validate and classify decoded USB bytes.
pub fn parse_packet(bytes: &[u8]) -> Result<ParsedPacket<'_>, PacketError> {
    if bytes.len() < 2 {
        return Err(PacketError::TooShort);
    }
    if bytes[0] != SYNC {
        return Err(PacketError::InvalidSync);
    }

    let pid = bytes[1];
    if (pid >> 4) != ((!pid) & 0x0f) {
        return Err(PacketError::InvalidPid);
    }

    match pid {
        PID_ACK | PID_NAK | PID_STALL => {
            if bytes.len() != 2 {
                return Err(PacketError::InvalidLength);
            }
            Ok(ParsedPacket::Handshake { pid })
        }
        PID_OUT | PID_IN | PID_SETUP | PID_SOF => {
            if bytes.len() != 4 {
                return Err(PacketError::InvalidLength);
            }
            let value = u16::from(bytes[2]) | (u16::from(bytes[3] & 0x07) << 8);
            if crc5_token(value) != bytes[3] >> 3 {
                return Err(PacketError::InvalidCrc5);
            }
            Ok(ParsedPacket::Token { pid, value })
        }
        PID_DATA0 | PID_DATA1 => {
            if bytes.len() < 4 {
                return Err(PacketError::InvalidLength);
            }
            let payload = &bytes[2..bytes.len() - 2];
            let received_crc =
                u16::from(bytes[bytes.len() - 2]) | (u16::from(bytes[bytes.len() - 1]) << 8);
            if crc16_data(payload) != received_crc {
                return Err(PacketError::InvalidCrc16);
            }
            Ok(ParsedPacket::Data { pid, payload })
        }
        _ => Err(PacketError::UnsupportedPid),
    }
}

/// USB bytes encoded as four two-bit PIO commands per byte.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct EncodedPacket {
    bytes: [u8; MAX_ENCODED_BYTES],
    len: usize,
    symbols: usize,
}

impl EncodedPacket {
    /// Encode raw USB packet bytes, including SYNC and PID.
    ///
    /// Data bits are sent least-significant bit first. The encoder applies
    /// NRZI, inserts a zero after six consecutive ones, and appends a
    /// standards-compliant two-bit SE0 plus one-bit J EOP sequence.
    pub fn encode(raw: &[u8]) -> Result<Self, EncodeError> {
        let mut packet = Self {
            bytes: [0; MAX_ENCODED_BYTES],
            len: 0,
            symbols: 0,
        };
        let mut current_state_is_j = true;
        let mut consecutive_ones = 0_u8;

        for &byte in raw {
            for bit_index in 0..8 {
                let bit_is_one = byte & (1 << bit_index) != 0;
                let symbol = if bit_is_one {
                    consecutive_ones += 1;
                    if current_state_is_j {
                        TxSymbol::K
                    } else {
                        TxSymbol::J
                    }
                } else {
                    consecutive_ones = 0;
                    current_state_is_j = !current_state_is_j;
                    if current_state_is_j {
                        TxSymbol::K
                    } else {
                        TxSymbol::J
                    }
                };
                packet.push(symbol)?;

                if consecutive_ones == 6 {
                    current_state_is_j = !current_state_is_j;
                    packet.push(if current_state_is_j {
                        TxSymbol::K
                    } else {
                        TxSymbol::J
                    })?;
                    consecutive_ones = 0;
                }
            }
        }

        packet.push(TxSymbol::Se0)?;
        packet.push(TxSymbol::Release)?;

        while !packet.symbols.is_multiple_of(4) {
            packet.push(TxSymbol::K)?;
        }
        packet.len = packet.symbols / 4;

        Ok(packet)
    }

    fn push(&mut self, symbol: TxSymbol) -> Result<(), EncodeError> {
        let byte_index = self.symbols / 4;
        if byte_index >= self.bytes.len() {
            return Err(EncodeError);
        }

        self.bytes[byte_index] = (self.bytes[byte_index] << 2) | symbol as u8;
        self.symbols += 1;
        Ok(())
    }

    /// Return the packed bytes to feed to the PIO TX FIFO.
    pub fn as_bytes(&self) -> &[u8] {
        &self.bytes[..self.len]
    }

    /// Return the number of meaningful and padding PIO commands.
    pub const fn symbol_count(&self) -> usize {
        self.symbols
    }

    #[cfg(test)]
    fn symbol(&self, index: usize) -> TxSymbol {
        assert!(index < self.symbols);
        let byte = self.bytes[index / 4];
        let shift = 6 - 2 * (index % 4);
        match (byte >> shift) & 0b11 {
            0 => TxSymbol::Se0,
            1 => TxSymbol::K,
            2 => TxSymbol::Release,
            _ => TxSymbol::J,
        }
    }
}

/// Build the four raw bytes of a Start-of-Frame token.
pub const fn sof_packet(frame_number: u16) -> [u8; 4] {
    let frame = frame_number & 0x07ff;
    let crc = crc5_token(frame);
    [
        SYNC,
        PID_SOF,
        frame as u8,
        ((frame >> 8) as u8 & 0x07) | (crc << 3),
    ]
}

/// Build a token packet for a device address and endpoint.
pub const fn token_packet(pid: u8, address: u8, endpoint: u8) -> [u8; 4] {
    let value = ((endpoint as u16 & 0x0f) << 7) | (address as u16 & 0x7f);
    let crc = crc5_token(value);
    [
        SYNC,
        pid,
        value as u8,
        ((value >> 8) as u8 & 0x07) | (crc << 3),
    ]
}

/// Build the zero-length DATA1 packet used by a control-read status stage.
pub const fn status_data1_packet() -> [u8; 4] {
    [SYNC, PID_DATA1, 0, 0]
}

/// Calculate the USB CRC-5 over the 11 token bits.
pub const fn crc5_token(mut token: u16) -> u8 {
    token &= 0x07ff;
    let mut crc = 0x1f_u8;
    let mut bit = 0;

    while bit < 11 {
        let mix = (crc ^ token as u8) & 1;
        crc >>= 1;
        if mix != 0 {
            crc ^= 0x14;
        }
        token >>= 1;
        bit += 1;
    }

    crc ^ 0x1f
}

/// Calculate the USB CRC-16 over a data payload.
pub fn crc16_data(data: &[u8]) -> u16 {
    let mut crc = 0xffff_u16;

    for &byte in data {
        crc ^= byte as u16;
        for _ in 0..8 {
            crc = if crc & 1 != 0 {
                (crc >> 1) ^ 0xa001
            } else {
                crc >> 1
            };
        }
    }

    crc ^ 0xffff
}

#[cfg(test)]
mod tests {
    use super::*;

    fn encode_wire_states(bytes: &[u8]) -> ([WireState; 96], usize) {
        let mut states = [WireState::J; 96];
        let mut len = 0;
        let mut state = WireState::J;
        let mut ones = 0_u8;

        for &byte in bytes {
            for bit_index in 0..8 {
                let one = byte & (1 << bit_index) != 0;
                if !one {
                    state = match state {
                        WireState::J => WireState::K,
                        WireState::K => WireState::J,
                    };
                    ones = 0;
                } else {
                    ones += 1;
                }
                states[len] = state;
                len += 1;

                if ones == 6 {
                    state = match state {
                        WireState::J => WireState::K,
                        WireState::K => WireState::J,
                    };
                    states[len] = state;
                    len += 1;
                    ones = 0;
                }
            }
        }

        (states, len)
    }

    const CDC_ACM_CONFIGURATION: [u8; 75] = [
        0x09, 0x02, 0x4b, 0x00, 0x02, 0x01, 0x00, 0x80, 0x32, // Configuration.
        0x08, 0x0b, 0x00, 0x02, 0x02, 0x02, 0x01, 0x00, // IAD.
        0x09, 0x04, 0x00, 0x00, 0x01, 0x02, 0x02, 0x01, 0x00, // Control IF.
        0x05, 0x24, 0x00, 0x10, 0x01, // CDC 1.10 header.
        0x05, 0x24, 0x01, 0x00, 0x01, // Call management.
        0x04, 0x24, 0x02, 0x02, // ACM capabilities.
        0x05, 0x24, 0x06, 0x00, 0x01, // Union: control 0, data 1.
        0x07, 0x05, 0x81, 0x03, 0x08, 0x00, 0x10, // Notification IN.
        0x09, 0x04, 0x01, 0x00, 0x02, 0x0a, 0x00, 0x00, 0x00, // Data IF.
        0x07, 0x05, 0x02, 0x02, 0x40, 0x00, 0x00, // Bulk OUT.
        0x07, 0x05, 0x82, 0x02, 0x40, 0x00, 0x00, // Bulk IN.
    ];

    // Captured from the target VID:PID 2dcf:6002 dongle with the independent
    // reference host firmware and a Beagle USB 480.
    const BLE_DONGLE_CONFIGURATION: [u8; 67] = [
        0x09, 0x02, 0x43, 0x00, 0x02, 0x01, 0x04, 0x80, 0x32, // Configuration.
        0x09, 0x04, 0x00, 0x00, 0x01, 0x02, 0x02, 0x01, 0x05, // Control IF.
        0x05, 0x24, 0x00, 0x10, 0x01, // CDC 1.10 header.
        0x05, 0x24, 0x01, 0x03, 0x01, // Call management.
        0x04, 0x24, 0x02, 0x06, // ACM capabilities.
        0x05, 0x24, 0x06, 0x00, 0x01, // Union: control 0, data 1.
        0x07, 0x05, 0x83, 0x03, 0x40, 0x00, 0x01, // Notification IN.
        0x09, 0x04, 0x01, 0x00, 0x02, 0x0a, 0x00, 0x00, 0x06, // Data IF.
        0x07, 0x05, 0x81, 0x02, 0x40, 0x00, 0x01, // Bulk IN.
        0x07, 0x05, 0x02, 0x02, 0x40, 0x00, 0x01, // Bulk OUT.
    ];

    const DUAL_CDC_ACM_CONFIGURATION: [u8; 102] = [
        0x09, 0x02, 0x66, 0x00, 0x04, 0x01, 0x00, 0x80, 0x32, // Configuration.
        0x09, 0x04, 0x00, 0x00, 0x00, 0x02, 0x02, 0x01, 0x00, // Control IF 0.
        0x05, 0x24, 0x00, 0x10, 0x01, // CDC 1.10 header.
        0x04, 0x24, 0x02, 0x02, // ACM capabilities.
        0x06, 0x24, 0x06, 0x00, 0x02, 0x01, // Union: master 0; subordinates 2, 1.
        0x09, 0x04, 0x01, 0x00, 0x02, 0x0a, 0x00, 0x00, 0x00, // Data IF 1.
        0x07, 0x05, 0x01, 0x02, 0x40, 0x00, 0x00, // Bulk OUT 1.
        0x07, 0x05, 0x81, 0x02, 0x40, 0x00, 0x00, // Bulk IN 1.
        0x09, 0x04, 0x02, 0x00, 0x00, 0x02, 0x02, 0x01, 0x00, // Control IF 2.
        0x05, 0x24, 0x00, 0x10, 0x01, // CDC 1.10 header.
        0x04, 0x24, 0x02, 0x06, // ACM capabilities.
        0x05, 0x24, 0x06, 0x02, 0x03, // Union: control 2, data 3.
        0x09, 0x04, 0x03, 0x00, 0x02, 0x0a, 0x00, 0x00, 0x00, // Data IF 3.
        0x07, 0x05, 0x03, 0x02, 0x40, 0x00, 0x00, // Bulk OUT 3.
        0x07, 0x05, 0x83, 0x02, 0x40, 0x00, 0x00, // Bulk IN 3.
    ];

    const HID_IN_OUT_CONFIGURATION: [u8; 41] = [
        0x09, 0x02, 0x29, 0x00, 0x01, 0x01, 0x00, 0x80, 0x32, // Configuration.
        0x09, 0x04, 0x03, 0x00, 0x02, 0x03, 0x00, 0x00, 0x07, // HID IF 3.
        0x09, 0x21, 0x11, 0x01, 0x21, 0x01, 0x22, 0x34, 0x12, // HID; report=0x1234.
        0x07, 0x05, 0x84, 0x03, 0x08, 0x00, 0x0a, // Interrupt IN.
        0x07, 0x05, 0x04, 0x03, 0x08, 0x00, 0x05, // Interrupt OUT.
    ];

    const HID_IN_ONLY_CONFIGURATION: [u8; 34] = [
        0x09, 0x02, 0x22, 0x00, 0x01, 0x01, 0x00, 0x80, 0x32, // Configuration.
        0x09, 0x04, 0x01, 0x00, 0x01, 0x03, 0x01, 0x02, 0x00, // Boot mouse IF 1.
        0x09, 0x21, 0x10, 0x01, 0x00, 0x01, 0x22, 0x34, 0x00, // HID; report=52.
        0x07, 0x05, 0x81, 0x03, 0x08, 0x00, 0x08, // Interrupt IN.
    ];

    const DUAL_HID_CONFIGURATION: [u8; 58] = [
        0x09, 0x02, 0x3a, 0x00, 0x02, 0x01, 0x00, 0x80, 0x32, // Configuration.
        0x09, 0x04, 0x00, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, // Broken HID IF 0.
        0x08, 0x21, 0x11, 0x01, 0x00, 0x01, 0x22, 0x08, // Truncated HID descriptor.
        0x07, 0x05, 0x81, 0x03, 0x08, 0x00, 0x01, // Interrupt IN 0.
        0x09, 0x04, 0x02, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, // Valid HID IF 2.
        0x09, 0x21, 0x11, 0x01, 0x00, 0x01, 0x22, 0x08, 0x00, // HID; report=8.
        0x07, 0x05, 0x83, 0x03, 0x08, 0x00, 0x04, // Interrupt IN 2.
    ];

    #[test]
    fn crc5_matches_known_zero_token() {
        assert_eq!(crc5_token(0), 0x02);
    }

    #[test]
    fn crc16_matches_usb_check_value() {
        assert_eq!(crc16_data(b"123456789"), 0xb4c8);
    }

    #[test]
    fn sof_zero_contains_pid_frame_and_crc() {
        assert_eq!(sof_packet(0), [0x80, 0xa5, 0x00, 0x10]);
        assert_eq!(sof_packet(0x0800), sof_packet(0));
    }

    #[test]
    fn encoder_inserts_zero_after_six_ones() {
        let encoded = EncodedPacket::encode(&[0xff]).unwrap();
        let expected = [
            TxSymbol::K,
            TxSymbol::K,
            TxSymbol::K,
            TxSymbol::K,
            TxSymbol::K,
            TxSymbol::K,
            TxSymbol::J,
            TxSymbol::J,
            TxSymbol::J,
        ];

        for (index, symbol) in expected.into_iter().enumerate() {
            assert_eq!(encoded.symbol(index), symbol);
        }
    }

    #[test]
    fn encoder_appends_eop_and_pads_to_fifo_byte() {
        let encoded = EncodedPacket::encode(&[0x00]).unwrap();

        assert_eq!(encoded.symbol_count(), 12);
        assert_eq!(encoded.symbol(8), TxSymbol::Se0);
        assert_eq!(encoded.symbol(9), TxSymbol::Release);
        assert_eq!(encoded.symbol(10), TxSymbol::K);
        assert_eq!(encoded.symbol(11), TxSymbol::K);
        assert_eq!(encoded.as_bytes().len(), 3);
    }

    #[test]
    fn encoded_sof_fits_tx_buffer() {
        let encoded = EncodedPacket::encode(&sof_packet(0x07ff)).unwrap();
        assert!(encoded.as_bytes().len() <= MAX_ENCODED_BYTES);
    }

    #[test]
    fn encoded_sof_zero_matches_pico_pio_usb_layout() {
        let encoded = EncodedPacket::encode(&sof_packet(0)).unwrap();
        assert_eq!(
            encoded.as_bytes(),
            [0xdd, 0xdf, 0xd7, 0x5f, 0x77, 0x77, 0x77, 0xdd, 0x25]
        );
    }

    #[test]
    fn nrzi_decoder_removes_stuffed_zero() {
        let (states, len) = encode_wire_states(&[0xff, 0x80, 0x3f]);
        let decoded = decode_nrzi(&states[..len]).unwrap();
        assert_eq!(decoded.as_bytes(), [0xff, 0x80, 0x3f]);
    }

    #[test]
    fn nrzi_decoder_rejects_missing_stuffed_zero() {
        let (mut states, len) = encode_wire_states(&[0xff]);
        states[6] = states[5];
        assert_eq!(decode_nrzi(&states[..len]), Err(NrziError::BitStuff));
    }

    #[test]
    fn parser_validates_handshake_pid_complement() {
        assert_eq!(
            parse_packet(&[SYNC, PID_ACK]),
            Ok(ParsedPacket::Handshake { pid: PID_ACK })
        );
        assert_eq!(
            parse_packet(&[SYNC, PID_ACK ^ 0x10]),
            Err(PacketError::InvalidPid)
        );
    }

    #[test]
    fn parser_validates_token_crc5() {
        let token = token_packet(PID_IN, 5, 2);
        assert_eq!(
            parse_packet(&token),
            Ok(ParsedPacket::Token {
                pid: PID_IN,
                value: (2 << 7) | 5,
            })
        );

        let mut corrupt = token;
        corrupt[3] ^= 0x08;
        assert_eq!(parse_packet(&corrupt), Err(PacketError::InvalidCrc5));
    }

    #[test]
    fn parser_validates_data_crc16() {
        let payload = [1, 2, 3, 4];
        let crc = crc16_data(&payload);
        let packet = [SYNC, PID_DATA1, 1, 2, 3, 4, crc as u8, (crc >> 8) as u8];
        assert_eq!(
            parse_packet(&packet),
            Ok(ParsedPacket::Data {
                pid: PID_DATA1,
                payload: &payload,
            })
        );

        let mut corrupt = packet;
        corrupt[2] ^= 1;
        assert_eq!(parse_packet(&corrupt), Err(PacketError::InvalidCrc16));
    }

    #[test]
    fn get_device_descriptor_request_serializes_first_eight_bytes() {
        assert_eq!(
            SetupRequest::get_device_descriptor_prefix().to_bytes(),
            [0x80, 0x06, 0x00, 0x01, 0x00, 0x00, 0x08, 0x00]
        );
    }

    #[test]
    fn get_complete_device_descriptor_request_serializes_eighteen_bytes() {
        let bytes = SetupRequest::get_device_descriptor(18).to_bytes();
        assert_eq!(bytes, [0x80, 0x06, 0x00, 0x01, 0x00, 0x00, 0x12, 0x00]);
        assert_eq!(crc16_data(&bytes), 0xf4e0);
    }

    #[test]
    fn get_configuration_descriptor_requests_serialize_header_and_full_length() {
        let header = SetupRequest::get_configuration_descriptor(0, 9).to_bytes();
        assert_eq!(header, [0x80, 0x06, 0x00, 0x02, 0x00, 0x00, 0x09, 0x00]);
        assert_eq!(crc16_data(&header), 0x04ae);

        let full = SetupRequest::get_configuration_descriptor(0, 75).to_bytes();
        assert_eq!(full, [0x80, 0x06, 0x00, 0x02, 0x00, 0x00, 0x4b, 0x00]);
        assert_eq!(crc16_data(&full), 0xa49e);
    }

    #[test]
    fn in_data_classification_handles_lengths_and_duplicates() {
        use InDataDisposition::{Accept, Duplicate, Reject};

        assert_eq!(
            classify_in_data(true, PID_DATA1, 8, 8, PID_DATA1, 8),
            Accept
        );
        assert_eq!(
            classify_in_data(true, PID_DATA0, 8, 8, PID_DATA0, 8),
            Accept
        );
        assert_eq!(
            classify_in_data(true, PID_DATA1, 2, 8, PID_DATA1, 2),
            Accept
        );
        assert_eq!(
            classify_in_data(true, PID_DATA1, 2, 8, PID_DATA0, 8),
            Duplicate
        );
        assert_eq!(
            classify_in_data(true, PID_DATA1, 2, 8, PID_DATA1, 8),
            Reject
        );
        assert_eq!(
            classify_in_data(false, PID_DATA1, 8, 8, PID_DATA1, 8),
            Reject
        );
        assert_eq!(
            classify_in_data(true, PID_DATA0, 64, 64, PID_DATA0, 0),
            Accept
        );
        assert_eq!(
            classify_in_data(true, PID_DATA0, 64, 64, PID_DATA0, 64),
            Accept
        );
        assert_eq!(
            classify_in_data(true, PID_DATA0, 64, 64, PID_DATA0, 65),
            Reject
        );
        assert_eq!(
            classify_in_data(true, PID_DATA0, 8, 8, PID_DATA1, 64),
            Reject
        );
    }

    #[test]
    fn set_address_one_serializes_wire_bytes() {
        let bytes = SetupRequest::set_address(1).unwrap().to_bytes();
        assert_eq!(bytes, [0x00, 0x05, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00]);
        assert_eq!(crc16_data(&bytes), 0x25eb);
    }

    #[test]
    fn set_address_validates_seven_bit_range() {
        assert_eq!(
            SetupRequest::set_address(0).unwrap().to_bytes(),
            [0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        );
        assert_eq!(
            SetupRequest::set_address(127).unwrap().to_bytes(),
            [0x00, 0x05, 0x7f, 0x00, 0x00, 0x00, 0x00, 0x00]
        );
        assert_eq!(
            SetupRequest::set_address(128),
            Err(SetupRequestError::InvalidAddress)
        );
        assert_eq!(
            SetupRequest::set_address(255),
            Err(SetupRequestError::InvalidAddress)
        );
    }

    #[test]
    fn set_configuration_one_serializes_wire_bytes_and_crc() {
        let bytes = SetupRequest::set_configuration(1).to_bytes();
        assert_eq!(bytes, [0x00, 0x09, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00]);
        assert_eq!(crc16_data(&bytes), 0x2527);
    }

    #[test]
    fn set_configuration_preserves_full_configuration_value_range() {
        assert_eq!(
            SetupRequest::set_configuration(0).to_bytes(),
            [0x00, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        );
        assert_eq!(
            SetupRequest::set_configuration(u8::MAX).to_bytes(),
            [0x00, 0x09, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00]
        );
    }

    #[test]
    fn hid_report_descriptor_and_report_requests_serialize_interface_recipient() {
        assert_eq!(
            SetupRequest::get_hid_report_descriptor(3, 0x1234).to_bytes(),
            [0x81, 0x06, 0x00, 0x22, 0x03, 0x00, 0x34, 0x12]
        );
        assert_eq!(
            SetupRequest::get_hid_report(3, 1, 7, 8).to_bytes(),
            [0xa1, 0x01, 0x07, 0x01, 0x03, 0x00, 0x08, 0x00]
        );
        assert_eq!(
            SetupRequest::set_hid_report(3, 2, 0, 8).to_bytes(),
            [0x21, 0x09, 0x00, 0x02, 0x03, 0x00, 0x08, 0x00]
        );
    }

    #[test]
    fn hid_idle_and_protocol_requests_serialize_exact_values() {
        assert_eq!(
            SetupRequest::get_hid_idle(3, 7).to_bytes(),
            [0xa1, 0x02, 0x07, 0x00, 0x03, 0x00, 0x01, 0x00]
        );
        assert_eq!(
            SetupRequest::set_hid_idle(3, 7, 25).to_bytes(),
            [0x21, 0x0a, 0x07, 0x19, 0x03, 0x00, 0x00, 0x00]
        );
        assert_eq!(
            SetupRequest::get_hid_protocol(3).to_bytes(),
            [0xa1, 0x03, 0x00, 0x00, 0x03, 0x00, 0x01, 0x00]
        );
        assert_eq!(
            SetupRequest::set_hid_protocol(3, 1).to_bytes(),
            [0x21, 0x0b, 0x01, 0x00, 0x03, 0x00, 0x00, 0x00]
        );
    }

    #[test]
    fn set_control_line_state_serializes_interface_flags_and_crc() {
        let bytes = SetupRequest::set_control_line_state(0, true, true).to_bytes();
        assert_eq!(bytes, [0x21, 0x22, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00]);
        assert_eq!(crc16_data(&bytes), 0x117e);

        assert_eq!(
            SetupRequest::set_control_line_state(7, false, false).to_bytes(),
            [0x21, 0x22, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00]
        );
        assert_eq!(
            SetupRequest::set_control_line_state(7, true, false).to_bytes(),
            [0x21, 0x22, 0x01, 0x00, 0x07, 0x00, 0x00, 0x00]
        );
        assert_eq!(
            SetupRequest::set_control_line_state(7, false, true).to_bytes(),
            [0x21, 0x22, 0x02, 0x00, 0x07, 0x00, 0x00, 0x00]
        );
    }

    #[test]
    fn set_line_coding_serializes_interface_length_and_crc() {
        let request = SetupRequest::set_line_coding(0);
        let bytes = request.to_bytes();
        assert_eq!(bytes, [0x21, 0x20, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00]);
        assert_eq!(request.length, CdcLineCoding::ENCODED_LEN as u16);
        assert_eq!(crc16_data(&bytes), 0xd25f);

        assert_eq!(
            SetupRequest::set_line_coding(7).to_bytes(),
            [0x21, 0x20, 0x00, 0x00, 0x07, 0x00, 0x07, 0x00]
        );
    }

    #[test]
    fn get_line_coding_and_send_break_serialize_class_requests() {
        assert_eq!(
            SetupRequest::get_line_coding(7).to_bytes(),
            [0xa1, 0x21, 0x00, 0x00, 0x07, 0x00, 0x07, 0x00]
        );
        assert_eq!(
            SetupRequest::send_break(7, 250).to_bytes(),
            [0x21, 0x23, 0xfa, 0x00, 0x07, 0x00, 0x00, 0x00]
        );
        assert_eq!(
            SetupRequest::send_break(7, u16::MAX).to_bytes(),
            [0x21, 0x23, 0xff, 0xff, 0x07, 0x00, 0x00, 0x00]
        );
    }

    #[test]
    fn cdc_line_coding_serializes_115200_eight_n_one() {
        let bytes = CdcLineCoding::eight_n_one(115_200).to_bytes();
        assert_eq!(bytes, [0x00, 0xc2, 0x01, 0x00, 0x00, 0x00, 0x08]);
        assert_eq!(crc16_data(&bytes), 0x1bc8);

        assert_eq!(
            CdcLineCoding::eight_n_one(0x1234_5678).to_bytes(),
            [0x78, 0x56, 0x34, 0x12, 0x00, 0x00, 0x08]
        );
        assert_eq!(
            crc16_data(&CdcLineCoding::eight_n_one(0x1234_5678).to_bytes()),
            0xb4a5
        );
    }

    #[test]
    fn cdc_line_coding_supports_non_default_serial_formats() {
        let coding = CdcLineCoding::new(9_600, CdcStopBits::Two, CdcParity::Even, 7);

        assert_eq!(coding.data_terminal_rate, 9_600);
        assert_eq!(coding.stop_bits, CdcStopBits::Two);
        assert_eq!(coding.parity, CdcParity::Even);
        assert_eq!(coding.data_bits, 7);
        assert_eq!(
            coding.to_bytes(),
            [0x80, 0x25, 0x00, 0x00, 0x02, 0x02, 0x07]
        );
        assert_eq!(CdcLineCoding::from_bytes(&coding.to_bytes()), Ok(coding));
        assert_eq!(
            CdcLineCoding::from_bytes(&coding.to_bytes()[..6]),
            Err(CdcLineCodingError::InvalidLength)
        );

        let mut invalid = coding.to_bytes();
        invalid[4] = 3;
        assert_eq!(
            CdcLineCoding::from_bytes(&invalid),
            Err(CdcLineCodingError::InvalidStopBits)
        );
        invalid[4] = 0;
        invalid[5] = 5;
        assert_eq!(
            CdcLineCoding::from_bytes(&invalid),
            Err(CdcLineCodingError::InvalidParity)
        );
        invalid[5] = 0;
        invalid[6] = 9;
        assert_eq!(
            CdcLineCoding::from_bytes(&invalid),
            Err(CdcLineCodingError::InvalidDataBits)
        );
    }

    #[test]
    fn cdc_line_coding_accepts_every_specified_data_width() {
        for data_bits in [5, 6, 7, 8, 16] {
            let coding = CdcLineCoding::new(115_200, CdcStopBits::One, CdcParity::None, data_bits);
            assert_eq!(CdcLineCoding::from_bytes(&coding.to_bytes()), Ok(coding));
        }

        for data_bits in [0, 4, 9, 15, 17, u8::MAX] {
            let bytes = CdcLineCoding::new(115_200, CdcStopBits::One, CdcParity::None, data_bits)
                .to_bytes();
            assert_eq!(
                CdcLineCoding::from_bytes(&bytes),
                Err(CdcLineCodingError::InvalidDataBits)
            );
        }
    }

    #[test]
    fn raw_data_packet_builds_line_coding_data1_and_rejects_invalid_inputs() {
        let payload = CdcLineCoding::eight_n_one(115_200).to_bytes();
        let packet = RawDataPacket::new(PID_DATA1, &payload).unwrap();
        assert_eq!(
            packet.as_bytes(),
            &[
                SYNC, PID_DATA1, 0x00, 0xc2, 0x01, 0x00, 0x00, 0x00, 0x08, 0xc8, 0x1b
            ]
        );
        assert_eq!(
            parse_packet(packet.as_bytes()),
            Ok(ParsedPacket::Data {
                pid: PID_DATA1,
                payload: &payload,
            })
        );
        assert_eq!(
            RawDataPacket::new(PID_ACK, &payload),
            Err(DataPacketError::InvalidPid)
        );
        assert_eq!(
            RawDataPacket::new(PID_DATA1, &[]).unwrap().as_bytes(),
            &[SYNC, PID_DATA1, 0x00, 0x00]
        );
        assert_eq!(
            RawDataPacket::new(PID_DATA0, &[0; 64])
                .unwrap()
                .as_bytes()
                .len(),
            MAX_DECODED_BYTES
        );
        assert_eq!(
            RawDataPacket::new(PID_DATA1, &[0; 65]),
            Err(DataPacketError::PayloadTooLong)
        );
    }

    #[test]
    fn first_cdc_bulk_out_packet_uses_discovered_endpoint_and_data0() {
        let function = CdcAcmFunction::discover(&BLE_DONGLE_CONFIGURATION).unwrap();
        let endpoint = function.bulk_out_endpoint;
        assert_eq!(endpoint.address, 0x02);
        assert_eq!(endpoint.number(), 2);
        assert!(!endpoint.is_in());
        assert_eq!(endpoint.transfer_type(), 0x02);
        assert_eq!(endpoint.max_packet_size, 64);

        let payload = b"AT\r\n";
        assert!(payload.len() <= endpoint.max_packet_size as usize);
        assert_eq!(crc16_data(payload), 0xa02e);

        let data = RawDataPacket::new(DataToggle::default().pid(), payload).unwrap();
        assert_eq!(
            data.as_bytes(),
            &[SYNC, PID_DATA0, 0x41, 0x54, 0x0d, 0x0a, 0x2e, 0xa0]
        );
        assert_eq!(
            parse_packet(data.as_bytes()),
            Ok(ParsedPacket::Data {
                pid: PID_DATA0,
                payload,
            })
        );

        let token = token_packet(PID_OUT, 1, endpoint.number());
        assert_eq!(token, [SYNC, PID_OUT, 0x01, 0xc1]);
        assert_eq!(
            parse_packet(&token),
            Ok(ParsedPacket::Token {
                pid: PID_OUT,
                value: 0x0101,
            })
        );
    }

    #[test]
    fn first_cdc_bulk_in_poll_uses_discovered_endpoint_and_data0() {
        let function = CdcAcmFunction::discover(&BLE_DONGLE_CONFIGURATION).unwrap();
        let endpoint = function.bulk_in_endpoint;
        assert_eq!(endpoint.address, 0x81);
        assert_eq!(endpoint.number(), 1);
        assert!(endpoint.is_in());
        assert_eq!(endpoint.transfer_type(), 0x02);
        assert_eq!(endpoint.max_packet_size, 64);

        let token = token_packet(PID_IN, 1, endpoint.number());
        assert_eq!(token, [SYNC, PID_IN, 0x81, 0x58]);
        assert_eq!(
            parse_packet(&token),
            Ok(ParsedPacket::Token {
                pid: PID_IN,
                value: 0x0081,
            })
        );

        let captured_response = b"A";
        assert_eq!(crc16_data(captured_response), 0x8f80);
        let data = RawDataPacket::new(DataToggle::default().pid(), captured_response).unwrap();
        assert_eq!(data.as_bytes(), &[SYNC, PID_DATA0, 0x41, 0x80, 0x8f]);
        assert_eq!(
            parse_packet(data.as_bytes()),
            Ok(ParsedPacket::Data {
                pid: PID_DATA0,
                payload: captured_response,
            })
        );
    }

    #[test]
    fn data_toggle_advances_only_when_caller_records_ack() {
        let bulk_out_toggle = DataToggle::default();
        let bulk_in_toggle = DataToggle::default();
        assert_eq!(bulk_out_toggle.pid(), PID_DATA0);
        assert_eq!(bulk_in_toggle.pid(), PID_DATA0);

        let after_first_ack = bulk_out_toggle.after_ack();
        assert_eq!(after_first_ack.pid(), PID_DATA1);
        assert_eq!(after_first_ack.after_ack(), DataToggle::Data0);
        assert_eq!(bulk_in_toggle.pid(), PID_DATA0);
    }

    #[test]
    fn cdc_session_retains_independent_toggles_across_commands() {
        let state = CdcAcmDataState::new();
        assert_eq!(state.bulk_out_pid(), PID_DATA0);
        assert_eq!(state.bulk_in_pid(), PID_DATA0);

        // The first AT command consumes one acknowledged bulk-OUT packet.
        let state = state.after_bulk_out_ack();
        assert_eq!(state.bulk_out_pid(), PID_DATA1);
        assert_eq!(state.bulk_in_pid(), PID_DATA0);

        // The captured M5g response consumed DATA0, DATA1 and DATA0.
        let state = state
            .after_bulk_in_ack()
            .after_bulk_in_ack()
            .after_bulk_in_ack();
        assert_eq!(state.bulk_out_pid(), PID_DATA1);
        assert_eq!(state.bulk_in_pid(), PID_DATA1);

        let command = b"AT+CENTRAL\r\n";
        assert_eq!(crc16_data(command), 0xe3a9);
        let packet = RawDataPacket::new(state.bulk_out_pid(), command).unwrap();
        assert_eq!(
            packet.as_bytes(),
            &[
                SYNC, PID_DATA1, b'A', b'T', b'+', b'C', b'E', b'N', b'T', b'R', b'A', b'L', b'\r',
                b'\n', 0xa9, 0xe3,
            ]
        );

        let state = state.after_bulk_out_ack();
        assert_eq!(state.bulk_out_pid(), PID_DATA0);
        assert_eq!(state.bulk_in_pid(), PID_DATA1);

        let command = b"AT+GAPSCAN=1\r\n";
        assert_eq!(crc16_data(command), 0xaed7);
        let packet = RawDataPacket::new(state.bulk_out_pid(), command).unwrap();
        assert_eq!(
            packet.as_bytes(),
            &[
                SYNC, PID_DATA0, b'A', b'T', b'+', b'G', b'A', b'P', b'S', b'C', b'A', b'N', b'=',
                b'1', b'\r', b'\n', 0xd7, 0xae,
            ]
        );
        assert_eq!(packet.as_bytes().len(), 18);
    }

    #[test]
    fn device_descriptor_header_parses_valid_prefix() {
        let prefix = [0x12, 0x01, 0x00, 0x02, 0x02, 0x00, 0x00, 0x40];

        assert_eq!(
            DeviceDescriptorHeader::parse(&prefix),
            Ok(DeviceDescriptorHeader {
                usb_version_bcd: 0x0200,
                device_class: 0x02,
                device_subclass: 0,
                device_protocol: 0,
                max_packet_size0: 64,
            })
        );
    }

    #[test]
    fn device_descriptor_header_rejects_invalid_prefixes() {
        assert_eq!(
            DeviceDescriptorHeader::parse(&[0x12; 7]),
            Err(DescriptorError::TooShort)
        );

        let mut prefix = [0x12, 0x01, 0x00, 0x02, 0x02, 0x00, 0x00, 0x40];
        prefix[0] = 17;
        assert_eq!(
            DeviceDescriptorHeader::parse(&prefix),
            Err(DescriptorError::InvalidLength)
        );

        prefix[0] = 18;
        prefix[1] = 2;
        assert_eq!(
            DeviceDescriptorHeader::parse(&prefix),
            Err(DescriptorError::InvalidType)
        );

        prefix[1] = DESCRIPTOR_TYPE_DEVICE;
        prefix[7] = 9;
        assert_eq!(
            DeviceDescriptorHeader::parse(&prefix),
            Err(DescriptorError::InvalidMaxPacketSize0)
        );
    }

    #[test]
    fn device_descriptor_parses_realistic_mps0_eight_descriptor() {
        let bytes = [
            0x12, 0x01, 0x00, 0x02, 0x02, 0x02, 0x00, 0x08, 0xcf, 0x2d, 0x02, 0x60, 0x00, 0x01,
            0x01, 0x02, 0x03, 0x01,
        ];
        let expected = DeviceDescriptor {
            usb_version_bcd: 0x0200,
            device_class: 0x02,
            device_subclass: 0x02,
            device_protocol: 0,
            max_packet_size0: 8,
            vendor_id: 0x2dcf,
            product_id: 0x6002,
            device_version_bcd: 0x0100,
            manufacturer_string_index: 1,
            product_string_index: 2,
            serial_number_string_index: 3,
            num_configurations: 1,
        };

        assert_eq!(DeviceDescriptor::parse(&bytes), Ok(expected));
        assert_eq!(
            expected.header(),
            DeviceDescriptorHeader {
                usb_version_bcd: 0x0200,
                device_class: 0x02,
                device_subclass: 0x02,
                device_protocol: 0,
                max_packet_size0: 8,
            }
        );

        let mut longer_payload = [0; 19];
        longer_payload[..18].copy_from_slice(&bytes);
        longer_payload[18] = 0xaa;
        assert_eq!(DeviceDescriptor::parse(&longer_payload), Ok(expected));
    }

    #[test]
    fn device_descriptor_rejects_invalid_length_type_and_mps0() {
        let mut bytes = [
            0x12, 0x01, 0x00, 0x02, 0x02, 0x02, 0x00, 0x08, 0xcf, 0x2d, 0x02, 0x60, 0x00, 0x01,
            0x01, 0x02, 0x03, 0x01,
        ];

        assert_eq!(
            DeviceDescriptor::parse(&bytes[..17]),
            Err(DescriptorError::TooShort)
        );

        bytes[0] = 17;
        assert_eq!(
            DeviceDescriptor::parse(&bytes),
            Err(DescriptorError::InvalidLength)
        );

        bytes[0] = 18;
        bytes[1] = 2;
        assert_eq!(
            DeviceDescriptor::parse(&bytes),
            Err(DescriptorError::InvalidType)
        );

        bytes[1] = DESCRIPTOR_TYPE_DEVICE;
        bytes[7] = 9;
        assert_eq!(
            DeviceDescriptor::parse(&bytes),
            Err(DescriptorError::InvalidMaxPacketSize0)
        );
    }

    #[test]
    fn configuration_header_parses_and_validates_fixed_fields() {
        assert_eq!(
            ConfigurationDescriptorHeader::parse(&CDC_ACM_CONFIGURATION),
            Ok(ConfigurationDescriptorHeader {
                total_length: 75,
                num_interfaces: 2,
                configuration_value: 1,
                string_index: 0,
                attributes: 0x80,
                max_power_ma: 100,
            })
        );

        assert_eq!(
            ConfigurationDescriptorHeader::parse(&CDC_ACM_CONFIGURATION[..8]),
            Err(ConfigurationError::TooShort)
        );

        let mut malformed = CDC_ACM_CONFIGURATION;
        malformed[0] = 8;
        assert_eq!(
            ConfigurationDescriptorHeader::parse(&malformed),
            Err(ConfigurationError::InvalidLength)
        );
        malformed[0] = 9;
        malformed[1] = DESCRIPTOR_TYPE_INTERFACE;
        assert_eq!(
            ConfigurationDescriptorHeader::parse(&malformed),
            Err(ConfigurationError::InvalidType)
        );
        malformed[1] = DESCRIPTOR_TYPE_CONFIGURATION;
        malformed[2] = 8;
        malformed[3] = 0;
        assert_eq!(
            ConfigurationDescriptorHeader::parse(&malformed),
            Err(ConfigurationError::InvalidTotalLength)
        );
        malformed[2] = 75;
        malformed[5] = 0;
        assert_eq!(
            ConfigurationDescriptorHeader::parse(&malformed),
            Err(ConfigurationError::InvalidConfigurationValue)
        );
        malformed[5] = 1;
        malformed[7] = 0;
        assert_eq!(
            ConfigurationDescriptorHeader::parse(&malformed),
            Err(ConfigurationError::InvalidAttributes)
        );
        malformed[7] = 0x81;
        assert_eq!(
            ConfigurationDescriptorHeader::parse(&malformed),
            Err(ConfigurationError::InvalidAttributes)
        );
    }

    #[test]
    fn descriptor_iterator_rejects_zero_length_and_truncation() {
        assert!(matches!(
            DescriptorIter::new(&CDC_ACM_CONFIGURATION[..74]),
            Err(ConfigurationError::DescriptorOverrun)
        ));

        let mut zero_length = CDC_ACM_CONFIGURATION;
        zero_length[17] = 0;
        let mut descriptors = DescriptorIter::new(&zero_length).unwrap();
        assert_eq!(descriptors.next().unwrap().unwrap()[1], 0x02);
        assert_eq!(descriptors.next().unwrap().unwrap()[1], 0x0b);
        assert_eq!(
            descriptors.next(),
            Some(Err(ConfigurationError::InvalidDescriptorLength))
        );
        assert_eq!(descriptors.next(), None);

        let mut one_byte_length = CDC_ACM_CONFIGURATION;
        one_byte_length[17] = 1;
        assert_eq!(
            DescriptorIter::new(&one_byte_length).unwrap().nth(2),
            Some(Err(ConfigurationError::InvalidDescriptorLength))
        );

        let mut overrun = CDC_ACM_CONFIGURATION;
        overrun[68] = 8;
        assert_eq!(
            DescriptorIter::new(&overrun).unwrap().last(),
            Some(Err(ConfigurationError::DescriptorOverrun))
        );
    }

    #[test]
    fn hid_discovery_retains_descriptor_and_interrupt_endpoint_metadata() {
        assert_eq!(
            HidInterface::discover(&HID_IN_OUT_CONFIGURATION),
            Ok(HidInterface {
                interface_number: 3,
                interface_subclass: 0,
                interface_protocol: 0,
                hid_version_bcd: 0x0111,
                country_code: 0x21,
                report_descriptor_len: 0x1234,
                interrupt_in_endpoint: EndpointDescriptor {
                    address: 0x84,
                    attributes: 0x03,
                    max_packet_size: 8,
                    interval: 10,
                },
                interrupt_out_endpoint: Some(EndpointDescriptor {
                    address: 0x04,
                    attributes: 0x03,
                    max_packet_size: 8,
                    interval: 5,
                }),
            })
        );
    }

    #[test]
    fn hid_discovery_accepts_an_in_only_boot_interface() {
        let interface = HidInterface::discover(&HID_IN_ONLY_CONFIGURATION).unwrap();
        assert_eq!(interface.interface_number, 1);
        assert_eq!(interface.interface_subclass, 1);
        assert_eq!(interface.interface_protocol, 2);
        assert_eq!(interface.hid_version_bcd, 0x0110);
        assert_eq!(interface.report_descriptor_len, 52);
        assert_eq!(interface.interrupt_in_endpoint.address, 0x81);
        assert_eq!(interface.interrupt_in_endpoint.interval, 8);
        assert_eq!(interface.interrupt_out_endpoint, None);
    }

    #[test]
    fn hid_iterator_and_selector_isolate_a_broken_composite_candidate() {
        let mut interfaces = HidInterfaces::new(&DUAL_HID_CONFIGURATION).unwrap();
        assert_eq!(
            interfaces.next(),
            Some(Err(HidConfigurationError::InvalidHidDescriptor))
        );
        let second = interfaces.next().unwrap().unwrap();
        assert_eq!(second.interface_number, 2);
        assert_eq!(second.report_descriptor_len, 8);
        assert_eq!(second.interrupt_in_endpoint.address, 0x83);
        assert!(interfaces.next().is_none());

        assert_eq!(HidInterface::discover(&DUAL_HID_CONFIGURATION), Ok(second));
        assert_eq!(
            HidInterface::discover_interface(&DUAL_HID_CONFIGURATION, 0),
            Err(HidConfigurationError::InvalidHidDescriptor)
        );
        assert_eq!(
            HidInterface::discover_interface(&DUAL_HID_CONFIGURATION, 2),
            Ok(second)
        );
        assert_eq!(
            HidInterface::discover_interface(&DUAL_HID_CONFIGURATION, 7),
            Err(HidConfigurationError::MissingInterface)
        );
    }

    #[test]
    fn hid_descriptor_requires_exact_framing_and_one_report_descriptor() {
        assert_eq!(
            HidDescriptor::parse(&[0x08, 0x21, 0x11, 0x01, 0x00, 0x01, 0x22, 0x08]),
            Err(HidConfigurationError::InvalidHidDescriptor)
        );
        assert_eq!(
            HidDescriptor::parse(&[0x09, 0x21, 0x11, 0x01, 0x00, 0x01, 0x23, 0x08, 0x00]),
            Err(HidConfigurationError::MissingReportDescriptor)
        );
        assert_eq!(
            HidDescriptor::parse(&[
                0x0c, 0x21, 0x11, 0x01, 0x00, 0x02, 0x22, 0x08, 0x00, 0x22, 0x10, 0x00,
            ]),
            Err(HidConfigurationError::DuplicateReportDescriptor)
        );
        assert_eq!(
            HidDescriptor::parse(&[0x09, 0x21, 0x11, 0x01, 0x00, 0x01, 0x22, 0x00, 0x00]),
            Err(HidConfigurationError::InvalidHidDescriptor)
        );
    }

    #[test]
    fn hid_discovery_rejects_missing_or_duplicate_required_descriptors() {
        let mut missing_hid = HID_IN_ONLY_CONFIGURATION;
        missing_hid[19] = DESCRIPTOR_TYPE_CLASS_INTERFACE;
        assert_eq!(
            HidInterface::discover(&missing_hid),
            Err(HidConfigurationError::MissingHidDescriptor)
        );

        let mut missing_report = HID_IN_ONLY_CONFIGURATION;
        missing_report[24] = 0x23;
        assert_eq!(
            HidInterface::discover(&missing_report),
            Err(HidConfigurationError::MissingReportDescriptor)
        );

        let mut duplicate_in = HID_IN_OUT_CONFIGURATION;
        duplicate_in[36] = 0x85;
        assert_eq!(
            HidInterface::discover(&duplicate_in),
            Err(HidConfigurationError::DuplicateInterruptInEndpoint)
        );

        let mut duplicate_out = HID_IN_OUT_CONFIGURATION;
        duplicate_out[29] = 0x05;
        assert_eq!(
            HidInterface::discover(&duplicate_out),
            Err(HidConfigurationError::DuplicateInterruptOutEndpoint)
        );

        let mut no_input = HID_IN_ONLY_CONFIGURATION;
        no_input[29] = 0x01;
        assert_eq!(
            HidInterface::discover(&no_input),
            Err(HidConfigurationError::MissingInterruptInEndpoint)
        );
    }

    #[test]
    fn hid_discovery_rejects_invalid_endpoint_contracts_and_nonzero_alt() {
        let mut wrong_type = HID_IN_ONLY_CONFIGURATION;
        wrong_type[30] = 0x02;
        assert_eq!(
            HidInterface::discover(&wrong_type),
            Err(HidConfigurationError::InvalidEndpointDescriptor)
        );

        let mut zero_interval = HID_IN_ONLY_CONFIGURATION;
        zero_interval[33] = 0;
        assert_eq!(
            HidInterface::discover(&zero_interval),
            Err(HidConfigurationError::InvalidEndpointDescriptor)
        );

        let mut oversized_packet = HID_IN_ONLY_CONFIGURATION;
        oversized_packet[31] = 0x01;
        oversized_packet[32] = 0x04;
        assert_eq!(
            HidInterface::discover(&oversized_packet),
            Err(HidConfigurationError::InvalidEndpointDescriptor)
        );

        let mut endpoint_count_mismatch = HID_IN_ONLY_CONFIGURATION;
        endpoint_count_mismatch[13] = 2;
        assert_eq!(
            HidInterface::discover(&endpoint_count_mismatch),
            Err(HidConfigurationError::InvalidEndpointDescriptor)
        );

        let mut alternate_setting_one = HID_IN_ONLY_CONFIGURATION;
        alternate_setting_one[12] = 1;
        assert_eq!(
            HidInterface::discover(&alternate_setting_one),
            Err(HidConfigurationError::MissingInterface)
        );
    }

    #[test]
    fn hid_discovery_preserves_configuration_framing_errors() {
        assert_eq!(
            HidInterface::discover(&HID_IN_ONLY_CONFIGURATION[..33]),
            Err(HidConfigurationError::Configuration(
                ConfigurationError::DescriptorOverrun
            ))
        );
    }

    #[test]
    fn cdc_acm_discovery_finds_union_interfaces_and_endpoints() {
        assert_eq!(
            CdcAcmFunction::discover(&CDC_ACM_CONFIGURATION),
            Ok(CdcAcmFunction {
                configuration: ConfigurationDescriptorHeader {
                    total_length: 75,
                    num_interfaces: 2,
                    configuration_value: 1,
                    string_index: 0,
                    attributes: 0x80,
                    max_power_ma: 100,
                },
                control_interface: 0,
                data_interface: 1,
                cdc_version_bcd: 0x0110,
                acm_capabilities: 0x02,
                notification_endpoint: Some(EndpointDescriptor {
                    address: 0x81,
                    attributes: 0x03,
                    max_packet_size: 8,
                    interval: 16,
                }),
                bulk_out_endpoint: EndpointDescriptor {
                    address: 0x02,
                    attributes: 0x02,
                    max_packet_size: 64,
                    interval: 0,
                },
                bulk_in_endpoint: EndpointDescriptor {
                    address: 0x82,
                    attributes: 0x02,
                    max_packet_size: 64,
                    interval: 0,
                },
            })
        );
    }

    #[test]
    fn cdc_acm_iterator_and_selector_handle_multiple_functions() {
        let mut functions = CdcAcmFunctions::new(&DUAL_CDC_ACM_CONFIGURATION).unwrap();

        let first = functions.next().unwrap().unwrap();
        assert_eq!(first.control_interface, 0);
        assert_eq!(first.data_interface, 1);
        assert_eq!(first.bulk_out_endpoint.address, 0x01);
        assert_eq!(first.bulk_in_endpoint.address, 0x81);

        let second = functions.next().unwrap().unwrap();
        assert_eq!(second.control_interface, 2);
        assert_eq!(second.data_interface, 3);
        assert_eq!(second.bulk_out_endpoint.address, 0x03);
        assert_eq!(second.bulk_in_endpoint.address, 0x83);
        assert!(functions.next().is_none());

        assert_eq!(
            CdcAcmFunction::discover(&DUAL_CDC_ACM_CONFIGURATION)
                .unwrap()
                .control_interface,
            0
        );
        assert_eq!(
            CdcAcmFunction::discover_control_interface(&DUAL_CDC_ACM_CONFIGURATION, 2).unwrap(),
            second
        );
        assert_eq!(
            CdcAcmFunction::discover_control_interface(&DUAL_CDC_ACM_CONFIGURATION, 4),
            Err(ConfigurationError::MissingControlInterface)
        );
    }

    #[test]
    fn cdc_acm_selection_isolates_a_broken_unrelated_function() {
        let mut broken_first = DUAL_CDC_ACM_CONFIGURATION;
        broken_first[26] = 0x12;

        let mut functions = CdcAcmFunctions::new(&broken_first).unwrap();
        assert_eq!(
            functions.next(),
            Some(Err(ConfigurationError::InvalidFunctionalDescriptor))
        );
        let second = functions.next().unwrap().unwrap();
        assert_eq!(second.control_interface, 2);
        assert!(functions.next().is_none());

        assert_eq!(
            CdcAcmFunction::discover(&broken_first)
                .unwrap()
                .control_interface,
            2
        );
        assert_eq!(
            CdcAcmFunction::discover_control_interface(&broken_first, 0),
            Err(ConfigurationError::InvalidFunctionalDescriptor)
        );
        assert_eq!(
            CdcAcmFunction::discover_control_interface(&broken_first, 2),
            Ok(second)
        );
    }

    #[test]
    fn cdc_acm_discovery_uses_every_union_subordinate_and_detects_ambiguity() {
        // The valid data interface is the second subordinate in the first
        // function's Union Functional Descriptor.
        let first =
            CdcAcmFunction::discover_control_interface(&DUAL_CDC_ACM_CONFIGURATION, 0).unwrap();
        assert_eq!(first.data_interface, 1);

        let mut ambiguous = DUAL_CDC_ACM_CONFIGURATION;
        ambiguous[31] = 3;
        assert_eq!(
            CdcAcmFunction::discover_control_interface(&ambiguous, 0),
            Err(ConfigurationError::AmbiguousDataInterface)
        );

        let mut duplicate_subordinate = DUAL_CDC_ACM_CONFIGURATION;
        duplicate_subordinate[31] = 1;
        assert_eq!(
            CdcAcmFunction::discover_control_interface(&duplicate_subordinate, 0),
            Err(ConfigurationError::InvalidUnionDescriptor)
        );
    }

    #[test]
    fn cdc_acm_discovery_rejects_duplicate_required_functional_descriptors() {
        let mut duplicate_header = [0_u8; 107];
        duplicate_header[..23].copy_from_slice(&DUAL_CDC_ACM_CONFIGURATION[..23]);
        duplicate_header[23..28].copy_from_slice(&[5, 0x24, 0x00, 0x10, 0x01]);
        duplicate_header[28..].copy_from_slice(&DUAL_CDC_ACM_CONFIGURATION[23..]);
        duplicate_header[2] = 107;
        assert_eq!(
            CdcAcmFunction::discover_control_interface(&duplicate_header, 0),
            Err(ConfigurationError::InvalidFunctionalDescriptor)
        );

        let mut duplicate_acm = [0_u8; 106];
        duplicate_acm[..27].copy_from_slice(&DUAL_CDC_ACM_CONFIGURATION[..27]);
        duplicate_acm[27..31].copy_from_slice(&[4, 0x24, 0x02, 0x02]);
        duplicate_acm[31..].copy_from_slice(&DUAL_CDC_ACM_CONFIGURATION[27..]);
        duplicate_acm[2] = 106;
        assert_eq!(
            CdcAcmFunction::discover_control_interface(&duplicate_acm, 0),
            Err(ConfigurationError::InvalidFunctionalDescriptor)
        );

        let mut duplicate_union = [0_u8; 108];
        duplicate_union[..33].copy_from_slice(&DUAL_CDC_ACM_CONFIGURATION[..33]);
        duplicate_union[33..39].copy_from_slice(&[6, 0x24, 0x06, 0x00, 0x02, 0x01]);
        duplicate_union[39..].copy_from_slice(&DUAL_CDC_ACM_CONFIGURATION[33..]);
        duplicate_union[2] = 108;
        assert_eq!(
            CdcAcmFunction::discover_control_interface(&duplicate_union, 0),
            Err(ConfigurationError::InvalidFunctionalDescriptor)
        );
    }

    #[test]
    fn cdc_acm_discovery_requires_data_subclass_zero_and_accepts_protocols_and_hs_bulk_mps() {
        let mut wrong_subclass = DUAL_CDC_ACM_CONFIGURATION;
        wrong_subclass[39] = 1;
        assert_eq!(
            CdcAcmFunction::discover_control_interface(&wrong_subclass, 0),
            Err(ConfigurationError::InvalidInterfaceDescriptor)
        );

        let mut wrong_protocol = DUAL_CDC_ACM_CONFIGURATION;
        wrong_protocol[40] = 1;
        assert_eq!(
            CdcAcmFunction::discover_control_interface(&wrong_protocol, 0)
                .unwrap()
                .data_interface,
            1
        );

        let mut high_speed = DUAL_CDC_ACM_CONFIGURATION;
        high_speed[46..48].copy_from_slice(&512_u16.to_le_bytes());
        high_speed[53..55].copy_from_slice(&512_u16.to_le_bytes());
        let function = CdcAcmFunction::discover_control_interface(&high_speed, 0).unwrap();
        assert_eq!(function.bulk_out_endpoint.max_packet_size, 512);
        assert_eq!(function.bulk_in_endpoint.max_packet_size, 512);
    }

    #[test]
    fn cdc_acm_call_management_uses_data_interface_only_when_d0_and_d1_are_set() {
        let mut configuration = [0_u8; 107];
        configuration[..23].copy_from_slice(&DUAL_CDC_ACM_CONFIGURATION[..23]);
        configuration[23..28].copy_from_slice(&[
            5, 0x24, 0x01, 0x00, 0x03, // D1 clear: bDataInterface is ignored.
        ]);
        configuration[28..].copy_from_slice(&DUAL_CDC_ACM_CONFIGURATION[23..]);
        configuration[2] = 107;
        configuration[36] = 3;

        assert_eq!(
            CdcAcmFunction::discover_control_interface(&configuration, 0),
            Err(ConfigurationError::AmbiguousDataInterface)
        );

        configuration[26] = CDC_CALL_MANAGEMENT_CAPABILITY_HANDLES_CALLS;
        assert_eq!(
            CdcAcmFunction::discover_control_interface(&configuration, 0),
            Err(ConfigurationError::AmbiguousDataInterface)
        );

        configuration[26] = CDC_CALL_MANAGEMENT_CAPABILITY_DATA_INTERFACE;
        assert_eq!(
            CdcAcmFunction::discover_control_interface(&configuration, 0),
            Err(ConfigurationError::InvalidFunctionalDescriptor)
        );

        configuration[26] = CDC_CALL_MANAGEMENT_CAPABILITY_HANDLES_CALLS
            | CDC_CALL_MANAGEMENT_CAPABILITY_DATA_INTERFACE;
        assert_eq!(
            CdcAcmFunction::discover_control_interface(&configuration, 0)
                .unwrap()
                .data_interface,
            3
        );

        configuration[26] = 0x04;
        assert_eq!(
            CdcAcmFunction::discover_control_interface(&configuration, 0),
            Err(ConfigurationError::InvalidFunctionalDescriptor)
        );
    }

    #[test]
    fn cdc_acm_discovery_accepts_captured_ble_dongle_configuration() {
        let function = CdcAcmFunction::discover(&BLE_DONGLE_CONFIGURATION).unwrap();

        assert_eq!(function.configuration.total_length, 67);
        assert_eq!(function.configuration.configuration_value, 1);
        assert_eq!(function.control_interface, 0);
        assert_eq!(function.data_interface, 1);
        assert_eq!(function.cdc_version_bcd, 0x0110);
        assert_eq!(function.acm_capabilities, 0x06);
        assert!(function.supports_line_requests());
        assert!(function.supports_send_break());
        assert_eq!(function.notification_endpoint.unwrap().address, 0x83);
        assert_eq!(function.bulk_in_endpoint.address, 0x81);
        assert_eq!(function.bulk_out_endpoint.address, 0x02);
    }

    #[test]
    fn cdc_acm_line_request_capability_is_exposed() {
        assert!(
            CdcAcmFunction::discover(&CDC_ACM_CONFIGURATION)
                .unwrap()
                .supports_line_requests()
        );

        let mut without_line_requests = CDC_ACM_CONFIGURATION;
        without_line_requests[39] = 0;
        assert!(
            !CdcAcmFunction::discover(&without_line_requests)
                .unwrap()
                .supports_line_requests()
        );
    }

    #[test]
    fn cdc_acm_fixture_has_expected_ep0_packetization() {
        let expected_pids = [
            PID_DATA1, PID_DATA0, PID_DATA1, PID_DATA0, PID_DATA1, PID_DATA0, PID_DATA1, PID_DATA0,
            PID_DATA1, PID_DATA0,
        ];
        let expected_crc = [
            0x3d02, 0x5ad5, 0x11f6, 0x3cce, 0x8550, 0xa268, 0x2ff4, 0x7da7, 0xfd8f, 0xeb8f,
        ];
        let expected_lengths = [8, 8, 8, 8, 8, 8, 8, 8, 8, 3];

        let mut packet_count = 0;
        for (index, payload) in CDC_ACM_CONFIGURATION.chunks(8).enumerate() {
            assert_eq!(
                expected_pids[index],
                if index % 2 == 0 { PID_DATA1 } else { PID_DATA0 }
            );
            assert_eq!(payload.len(), expected_lengths[index]);
            assert_eq!(crc16_data(payload), expected_crc[index]);
            packet_count += 1;
        }
        assert_eq!(packet_count, expected_pids.len());
    }

    #[test]
    fn cdc_acm_discovery_accepts_configuration_without_iad() {
        let mut without_iad = [0_u8; 67];
        without_iad[..9].copy_from_slice(&CDC_ACM_CONFIGURATION[..9]);
        without_iad[2] = 67;
        without_iad[3] = 0;
        without_iad[9..].copy_from_slice(&CDC_ACM_CONFIGURATION[17..]);

        let function = CdcAcmFunction::discover(&without_iad).unwrap();
        assert_eq!(function.control_interface, 0);
        assert_eq!(function.data_interface, 1);
        assert_eq!(function.bulk_out_endpoint.address, 0x02);
        assert_eq!(function.bulk_in_endpoint.address, 0x82);
    }

    #[test]
    fn cdc_acm_discovery_accepts_an_absent_notification_endpoint() {
        let mut without_notification = [0_u8; 68];
        without_notification[..45].copy_from_slice(&CDC_ACM_CONFIGURATION[..45]);
        without_notification[45..].copy_from_slice(&CDC_ACM_CONFIGURATION[52..]);
        without_notification[2] = 68;
        without_notification[3] = 0;
        without_notification[21] = 0;

        let function = CdcAcmFunction::discover(&without_notification).unwrap();
        assert_eq!(function.notification_endpoint, None);
        assert_eq!(function.bulk_out_endpoint.address, 0x02);
        assert_eq!(function.bulk_in_endpoint.address, 0x82);
    }

    #[test]
    fn cdc_acm_discovery_ignores_endpoints_on_unrelated_interfaces() {
        let mut composite = [0_u8; 93];
        composite[..75].copy_from_slice(&CDC_ACM_CONFIGURATION);
        composite[2] = 93;
        composite[3] = 0;
        composite[4] = 3;
        composite[75..84].copy_from_slice(&[0x09, 0x04, 0x02, 0x00, 0x01, 0x01, 0x02, 0x00, 0x00]);
        composite[84..].copy_from_slice(&[0x09, 0x05, 0x84, 0x01, 0x40, 0x00, 0x01, 0x00, 0x00]);

        let function = CdcAcmFunction::discover(&composite).unwrap();
        assert_eq!(function.control_interface, 0);
        assert_eq!(function.data_interface, 1);
        assert_eq!(function.bulk_in_endpoint.address, 0x82);
    }

    #[test]
    fn cdc_acm_discovery_reports_missing_bulk_directions() {
        let mut without_bulk_in = [0_u8; 68];
        without_bulk_in.copy_from_slice(&CDC_ACM_CONFIGURATION[..68]);
        without_bulk_in[2] = 68;
        without_bulk_in[3] = 0;
        without_bulk_in[56] = 1;
        assert_eq!(
            CdcAcmFunction::discover(&without_bulk_in),
            Err(ConfigurationError::MissingBulkInEndpoint)
        );

        let mut without_bulk_out = [0_u8; 68];
        without_bulk_out[..61].copy_from_slice(&CDC_ACM_CONFIGURATION[..61]);
        without_bulk_out[61..].copy_from_slice(&CDC_ACM_CONFIGURATION[68..]);
        without_bulk_out[2] = 68;
        without_bulk_out[3] = 0;
        without_bulk_out[56] = 1;
        assert_eq!(
            CdcAcmFunction::discover(&without_bulk_out),
            Err(ConfigurationError::MissingBulkOutEndpoint)
        );
    }

    #[test]
    fn cdc_acm_discovery_rejects_union_and_endpoint_mismatches() {
        let mut wrong_union = CDC_ACM_CONFIGURATION;
        wrong_union[44] = 2;
        assert_eq!(
            CdcAcmFunction::discover(&wrong_union),
            Err(ConfigurationError::InvalidUnionDescriptor)
        );

        let mut wrong_bulk_type = CDC_ACM_CONFIGURATION;
        wrong_bulk_type[64] = 0x03;
        assert_eq!(
            CdcAcmFunction::discover(&wrong_bulk_type),
            Err(ConfigurationError::InvalidEndpointDescriptor)
        );

        let mut invalid_notification_direction = CDC_ACM_CONFIGURATION;
        invalid_notification_direction[47] = 0x03;
        assert_eq!(
            CdcAcmFunction::discover(&invalid_notification_direction),
            Err(ConfigurationError::InvalidEndpointDescriptor)
        );

        let mut reserved_packet_size_bits = CDC_ACM_CONFIGURATION;
        reserved_packet_size_bits[66] = 0x08;
        assert_eq!(
            CdcAcmFunction::discover(&reserved_packet_size_bits),
            Err(ConfigurationError::InvalidEndpointDescriptor)
        );

        let mut reserved_attributes = CDC_ACM_CONFIGURATION;
        reserved_attributes[64] = 0x82;
        assert_eq!(
            CdcAcmFunction::discover(&reserved_attributes),
            Err(ConfigurationError::InvalidEndpointDescriptor)
        );

        let mut duplicate_in_address = CDC_ACM_CONFIGURATION;
        duplicate_in_address[47] = 0x82;
        assert_eq!(
            CdcAcmFunction::discover(&duplicate_in_address),
            Err(ConfigurationError::InvalidEndpointDescriptor)
        );

        let mut wrong_endpoint_count = CDC_ACM_CONFIGURATION;
        wrong_endpoint_count[21] = 0;
        assert_eq!(
            CdcAcmFunction::discover(&wrong_endpoint_count),
            Err(ConfigurationError::InvalidEndpointDescriptor)
        );
    }

    #[test]
    fn cdc_acm_discovery_rejects_noncanonical_known_descriptor_lengths() {
        let mut long_interface = CDC_ACM_CONFIGURATION;
        long_interface[17] = 10;
        assert_eq!(
            CdcAcmFunction::discover(&long_interface),
            Err(ConfigurationError::InvalidInterfaceDescriptor)
        );

        let mut short_cdc_header = CDC_ACM_CONFIGURATION;
        short_cdc_header[26] = 4;
        assert_eq!(
            CdcAcmFunction::discover(&short_cdc_header),
            Err(ConfigurationError::InvalidFunctionalDescriptor)
        );

        let mut long_endpoint = [0_u8; 76];
        long_endpoint[..52].copy_from_slice(&CDC_ACM_CONFIGURATION[..52]);
        long_endpoint[52] = 0;
        long_endpoint[53..].copy_from_slice(&CDC_ACM_CONFIGURATION[52..]);
        long_endpoint[2] = 76;
        long_endpoint[3] = 0;
        long_endpoint[45] = 8;
        assert_eq!(
            CdcAcmFunction::discover(&long_endpoint),
            Err(ConfigurationError::InvalidEndpointDescriptor)
        );
    }

    #[test]
    fn parser_reassembles_captured_three_packet_device_descriptor() {
        let packets: [&[u8]; 3] = [
            &[
                SYNC, PID_DATA1, 0x12, 0x01, 0x00, 0x02, 0x02, 0x02, 0x00, 0x08, 0xf7, 0x9f,
            ],
            &[
                SYNC, PID_DATA0, 0xcf, 0x2d, 0x02, 0x60, 0x00, 0x01, 0x01, 0x02, 0x5e, 0x9d,
            ],
            &[SYNC, PID_DATA1, 0x03, 0x01, 0x3f, 0x7f],
        ];
        let expected_pids = [PID_DATA1, PID_DATA0, PID_DATA1];
        let mut descriptor_bytes = [0_u8; 18];
        let mut offset = 0;

        for (packet, expected_pid) in packets.into_iter().zip(expected_pids) {
            let ParsedPacket::Data { pid, payload } = parse_packet(packet).unwrap() else {
                panic!("expected DATA packet");
            };
            assert_eq!(pid, expected_pid);
            descriptor_bytes[offset..offset + payload.len()].copy_from_slice(payload);
            offset += payload.len();
        }

        assert_eq!(offset, descriptor_bytes.len());
        assert_eq!(
            DeviceDescriptor::parse(&descriptor_bytes),
            Ok(DeviceDescriptor {
                usb_version_bcd: 0x0200,
                device_class: 0x02,
                device_subclass: 0x02,
                device_protocol: 0,
                max_packet_size0: 8,
                vendor_id: 0x2dcf,
                product_id: 0x6002,
                device_version_bcd: 0x0100,
                manufacturer_string_index: 1,
                product_string_index: 2,
                serial_number_string_index: 3,
                num_configurations: 1,
            })
        );
    }

    #[test]
    fn parser_accepts_known_data1_device_descriptor_prefix() {
        // CRC-16 over the eight-byte prefix is 0x6956, sent least significant
        // byte first.
        let packet = [
            SYNC, PID_DATA1, 0x12, 0x01, 0x00, 0x02, 0x02, 0x00, 0x00, 0x40, 0x56, 0x69,
        ];

        let ParsedPacket::Data { pid, payload } = parse_packet(&packet).unwrap() else {
            panic!("expected DATA packet");
        };
        assert_eq!(pid, PID_DATA1);
        assert_eq!(
            DeviceDescriptorHeader::parse(payload),
            Ok(DeviceDescriptorHeader {
                usb_version_bcd: 0x0200,
                device_class: 0x02,
                device_subclass: 0,
                device_protocol: 0,
                max_packet_size0: 64,
            })
        );
    }

    #[test]
    fn parser_accepts_data1_zero_length_status_packet() {
        let packet = status_data1_packet();

        assert_eq!(packet, [SYNC, PID_DATA1, 0, 0]);
        assert_eq!(
            parse_packet(&packet),
            Ok(ParsedPacket::Data {
                pid: PID_DATA1,
                payload: &[],
            })
        );
    }

    #[test]
    fn parser_rejects_corrupt_zero_length_status_crc() {
        let mut packet = status_data1_packet();
        packet[2] = 1;

        assert_eq!(parse_packet(&packet), Err(PacketError::InvalidCrc16));
    }
}
