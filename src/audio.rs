//! Minimal USB Audio Class 1.0 capture support.
//!
//! The first implementation deliberately covers the useful RP2040-sized
//! subset: full-speed, mono PCM, 16-bit, 48 kHz and one isochronous IN packet
//! per USB frame. Playback, feedback endpoints and asynchronous sample-rate
//! adaptation remain out of scope.

use crate::host::{
    Direction, EndpointAddress, EndpointInfo, EndpointType, HostError, PipeError, SplitInfo,
    TimeoutConfig, UsbHostAllocator, UsbPipe, pipe,
};
use crate::usb::{ConfigurationDescriptorHeader, ConfigurationError, DescriptorIter};

const DESCRIPTOR_TYPE_INTERFACE: u8 = 0x04;
const DESCRIPTOR_TYPE_ENDPOINT: u8 = 0x05;
const DESCRIPTOR_TYPE_CLASS_INTERFACE: u8 = 0x24;
const AUDIO_CLASS: u8 = 0x01;
const AUDIO_STREAMING_SUBCLASS: u8 = 0x02;
const AS_GENERAL: u8 = 0x01;
const FORMAT_TYPE: u8 = 0x02;
const FORMAT_TYPE_I: u8 = 0x01;
const FORMAT_TAG_PCM: u16 = 0x0001;

/// Sample rate selected by the initial capture implementation.
pub const CAPTURE_SAMPLE_RATE_HZ: u32 = 48_000;
/// Largest supported audio payload per full-speed frame.
pub const CAPTURE_PACKET_CAPACITY: usize = 100;

/// One descriptor-selected UAC1 capture alternate setting.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct AudioInputInterface {
    pub configuration: ConfigurationDescriptorHeader,
    pub interface_number: u8,
    pub alternate_setting: u8,
    pub endpoint_address: u8,
    pub max_packet_size: u16,
    pub interval: u8,
    pub channels: u8,
    pub subframe_size: u8,
    pub bit_resolution: u8,
}

impl AudioInputInterface {
    /// Find the first mono 16-bit PCM capture interface advertising 48 kHz.
    pub fn discover(configuration: &[u8]) -> Result<Self, AudioConfigurationError> {
        let header = ConfigurationDescriptorHeader::parse(configuration)
            .map_err(AudioConfigurationError::Configuration)?;
        let descriptors =
            DescriptorIter::new(configuration).map_err(AudioConfigurationError::Configuration)?;

        let mut interface_number = 0;
        let mut alternate_setting = 0;
        let mut candidate = false;
        let mut pcm_stream = false;
        let mut format_valid = false;
        let mut channels = 0;
        let mut subframe_size = 0;
        let mut bit_resolution = 0;

        for descriptor in descriptors {
            let descriptor = descriptor.map_err(AudioConfigurationError::Configuration)?;
            match descriptor[1] {
                DESCRIPTOR_TYPE_INTERFACE if descriptor.len() == 9 => {
                    interface_number = descriptor[2];
                    alternate_setting = descriptor[3];
                    candidate = alternate_setting != 0
                        && descriptor[4] == 1
                        && descriptor[5] == AUDIO_CLASS
                        && descriptor[6] == AUDIO_STREAMING_SUBCLASS
                        && descriptor[7] == 0;
                    pcm_stream = false;
                    format_valid = false;
                }
                DESCRIPTOR_TYPE_CLASS_INTERFACE
                    if candidate && descriptor.len() >= 7 && descriptor[2] == AS_GENERAL =>
                {
                    pcm_stream =
                        u16::from_le_bytes([descriptor[5], descriptor[6]]) == FORMAT_TAG_PCM;
                }
                DESCRIPTOR_TYPE_CLASS_INTERFACE
                    if candidate
                        && pcm_stream
                        && descriptor.len() >= 11
                        && descriptor[2] == FORMAT_TYPE
                        && descriptor[3] == FORMAT_TYPE_I =>
                {
                    channels = descriptor[4];
                    subframe_size = descriptor[5];
                    bit_resolution = descriptor[6];
                    let frequency_count = descriptor[7] as usize;
                    let expected_len = 8 + frequency_count * 3;
                    let supports_48k = frequency_count != 0
                        && descriptor.len() == expected_len
                        && descriptor[8..].chunks_exact(3).any(|frequency| {
                            u32::from(frequency[0])
                                | (u32::from(frequency[1]) << 8)
                                | (u32::from(frequency[2]) << 16)
                                == CAPTURE_SAMPLE_RATE_HZ
                        });
                    format_valid =
                        channels == 1 && subframe_size == 2 && bit_resolution == 16 && supports_48k;
                }
                DESCRIPTOR_TYPE_ENDPOINT if candidate && pcm_stream && format_valid => {
                    if descriptor.len() != 9 {
                        continue;
                    }
                    let address = descriptor[2];
                    let attributes = descriptor[3];
                    let max_packet_size =
                        u16::from_le_bytes([descriptor[4], descriptor[5]]) & 0x07ff;
                    let interval = descriptor[6];
                    if address & 0x80 != 0
                        && address & 0x70 == 0
                        && address & 0x0f != 0
                        && attributes & 0x03 == 0x01
                        && max_packet_size != 0
                        && max_packet_size as usize <= CAPTURE_PACKET_CAPACITY
                        && interval == 1
                    {
                        return Ok(Self {
                            configuration: header,
                            interface_number,
                            alternate_setting,
                            endpoint_address: address,
                            max_packet_size,
                            interval,
                            channels,
                            subframe_size,
                            bit_resolution,
                        });
                    }
                }
                _ => {}
            }
        }

        Err(AudioConfigurationError::MissingCaptureInterface)
    }
}

/// UAC1 discovery failure.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AudioConfigurationError {
    Configuration(ConfigurationError),
    MissingCaptureInterface,
}

/// UAC1 discovery or pipe-allocation failure.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AudioCreateError {
    Configuration(AudioConfigurationError),
    InvalidControlMaxPacketSize,
    Allocation(HostError),
}

/// UAC1 control or capture transfer failure.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AudioError {
    Transfer(PipeError),
    BufferTooSmall,
    InvalidPacketLength,
}

impl From<PipeError> for AudioError {
    fn from(error: PipeError) -> Self {
        Self::Transfer(error)
    }
}

/// Allocated UAC1 mono capture class.
pub type AllocatedAudioInput<'d, A> = AudioInputHost<
    <A as UsbHostAllocator<'d>>::Pipe<pipe::Control, pipe::InOut>,
    <A as UsbHostAllocator<'d>>::Pipe<pipe::Isochronous, pipe::In>,
>;

/// Discover and allocate a UAC1 mono capture class.
pub fn allocate_audio_input<'d, A>(
    allocator: &A,
    configuration: &[u8],
    device_address: u8,
    control_max_packet_size: u16,
    split: Option<SplitInfo>,
) -> Result<AllocatedAudioInput<'d, A>, AudioCreateError>
where
    A: UsbHostAllocator<'d>,
{
    if !matches!(control_max_packet_size, 8 | 16 | 32 | 64) {
        return Err(AudioCreateError::InvalidControlMaxPacketSize);
    }
    let interface =
        AudioInputInterface::discover(configuration).map_err(AudioCreateError::Configuration)?;
    let control = allocator
        .alloc_pipe::<pipe::Control, pipe::InOut>(
            device_address,
            &EndpointInfo {
                addr: EndpointAddress::from_parts(0, Direction::In),
                ep_type: EndpointType::Control,
                max_packet_size: control_max_packet_size,
                interval_ms: 0,
            },
            split,
        )
        .map_err(AudioCreateError::Allocation)?;
    let input = allocator
        .alloc_pipe::<pipe::Isochronous, pipe::In>(
            device_address,
            &EndpointInfo {
                addr: EndpointAddress::from(interface.endpoint_address),
                ep_type: EndpointType::Isochronous,
                max_packet_size: interface.max_packet_size,
                interval_ms: interface.interval,
            },
            split,
        )
        .map_err(AudioCreateError::Allocation)?;
    Ok(AudioInputHost {
        interface,
        control,
        input,
    })
}

#[cfg(feature = "embassy-usb-host")]
/// Allocate directly from Embassy enumeration output.
pub fn allocate_from_enumeration<'d, A>(
    allocator: &A,
    configuration: &[u8],
    enumeration: &embassy_usb_host::handler::EnumerationInfo,
) -> Result<AllocatedAudioInput<'d, A>, AudioCreateError>
where
    A: UsbHostAllocator<'d>,
{
    allocate_audio_input(
        allocator,
        configuration,
        enumeration.device_address,
        enumeration.device_desc.max_packet_size0 as u16,
        enumeration.split(),
    )
}

/// Configured UAC1 mono capture stream.
pub struct AudioInputHost<C, I> {
    interface: AudioInputInterface,
    control: C,
    input: I,
}

impl<C, I> AudioInputHost<C, I>
where
    C: UsbPipe<pipe::Control, pipe::InOut>,
    I: UsbPipe<pipe::Isochronous, pipe::In>,
{
    pub const fn interface(&self) -> &AudioInputInterface {
        &self.interface
    }

    pub fn set_control_timeout(&mut self, timeout: TimeoutConfig) {
        self.control.set_timeout(timeout);
    }

    /// Select the capture alternate setting and request 48 kHz sampling.
    pub async fn configure(&mut self) -> Result<(), AudioError> {
        let set_interface = [
            0x01,
            0x0b,
            self.interface.alternate_setting,
            0,
            self.interface.interface_number,
            0,
            0,
            0,
        ];
        self.control.control_out(&set_interface, &[]).await?;

        let set_sample_rate = [
            0x22,
            0x01,
            0,
            0x01,
            self.interface.endpoint_address,
            0,
            3,
            0,
        ];
        self.control
            .control_out(&set_sample_rate, &[0x80, 0xbb, 0x00])
            .await?;
        Ok(())
    }

    /// Read the next frame's mono PCM packet.
    pub async fn read_packet(&mut self, buffer: &mut [u8]) -> Result<usize, AudioError> {
        let capacity = self.interface.max_packet_size as usize;
        if buffer.len() < capacity {
            return Err(AudioError::BufferTooSmall);
        }
        let count = self.input.request_in(&mut buffer[..capacity]).await?;
        if count > capacity || count % self.interface.subframe_size as usize != 0 {
            return Err(AudioError::InvalidPacketLength);
        }
        Ok(count)
    }

    pub fn into_parts(self) -> (AudioInputInterface, C, I) {
        (self.interface, self.control, self.input)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const NAD_CONFIGURATION: [u8; 253] = [
        0x09, 0x02, 0xfd, 0x00, 0x04, 0x01, 0x00, 0x80, 0x32, 0x09, 0x04, 0x00, 0x00, 0x00, 0x01,
        0x01, 0x00, 0x00, 0x0a, 0x24, 0x01, 0x00, 0x01, 0x64, 0x00, 0x02, 0x01, 0x02, 0x0c, 0x24,
        0x02, 0x01, 0x01, 0x01, 0x00, 0x02, 0x03, 0x00, 0x00, 0x00, 0x0c, 0x24, 0x02, 0x02, 0x01,
        0x02, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x09, 0x24, 0x03, 0x06, 0x01, 0x03, 0x00, 0x09,
        0x00, 0x09, 0x24, 0x03, 0x07, 0x01, 0x01, 0x00, 0x08, 0x00, 0x07, 0x24, 0x05, 0x08, 0x01,
        0x0a, 0x00, 0x0a, 0x24, 0x06, 0x09, 0x0f, 0x01, 0x01, 0x02, 0x02, 0x00, 0x09, 0x24, 0x06,
        0x0a, 0x02, 0x01, 0x43, 0x00, 0x00, 0x09, 0x24, 0x06, 0x0d, 0x02, 0x01, 0x03, 0x00, 0x00,
        0x0d, 0x24, 0x04, 0x0f, 0x02, 0x01, 0x0d, 0x02, 0x03, 0x00, 0x00, 0x00, 0x00, 0x09, 0x04,
        0x01, 0x00, 0x00, 0x01, 0x02, 0x00, 0x00, 0x09, 0x04, 0x01, 0x01, 0x01, 0x01, 0x02, 0x00,
        0x00, 0x07, 0x24, 0x01, 0x01, 0x01, 0x01, 0x00, 0x0e, 0x24, 0x02, 0x01, 0x02, 0x02, 0x10,
        0x02, 0x80, 0xbb, 0x00, 0x44, 0xac, 0x00, 0x09, 0x05, 0x01, 0x09, 0xc8, 0x00, 0x01, 0x00,
        0x00, 0x07, 0x25, 0x01, 0x01, 0x01, 0x01, 0x00, 0x09, 0x04, 0x02, 0x00, 0x00, 0x01, 0x02,
        0x00, 0x00, 0x09, 0x04, 0x02, 0x01, 0x01, 0x01, 0x02, 0x00, 0x00, 0x07, 0x24, 0x01, 0x07,
        0x01, 0x01, 0x00, 0x0e, 0x24, 0x02, 0x01, 0x01, 0x02, 0x10, 0x02, 0x80, 0xbb, 0x00, 0x44,
        0xac, 0x00, 0x09, 0x05, 0x82, 0x0d, 0x64, 0x00, 0x01, 0x00, 0x00, 0x07, 0x25, 0x01, 0x01,
        0x00, 0x00, 0x00, 0x09, 0x04, 0x03, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x09, 0x21, 0x00,
        0x01, 0x00, 0x01, 0x22, 0x3c, 0x00, 0x07, 0x05, 0x87, 0x03, 0x04, 0x00, 0x02,
    ];

    #[test]
    fn discovers_nad_mono_capture_alternate_setting() {
        let interface = AudioInputInterface::discover(&NAD_CONFIGURATION).unwrap();
        assert_eq!(interface.interface_number, 2);
        assert_eq!(interface.alternate_setting, 1);
        assert_eq!(interface.endpoint_address, 0x82);
        assert_eq!(interface.max_packet_size, 100);
        assert_eq!(interface.interval, 1);
        assert_eq!(interface.channels, 1);
        assert_eq!(interface.subframe_size, 2);
        assert_eq!(interface.bit_resolution, 16);
    }
}
