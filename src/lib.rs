#![no_std]

//! Reusable USB-host building blocks for an RP2040 PIO controller.
//!
//! [`cdc_acm`] provides a controller-independent CDC-ACM byte stream and
//! [`ftdi`] provides controller-independent FTDI UART streams. [`hid`]
//! provides controller-independent raw HID reports and [`usbtmc`] provides
//! framed test-and-measurement messages over the official Embassy host-pipe
//! traits re-exported by [`host`]. [`pio_host`] adapts a
//! serialized packet engine to those traits; its target-gated RP2040
//! implementation supports one directly connected full- or low-speed device.
//! Product protocols intentionally live above the library, as shown by the
//! named examples.

pub mod cdc_acm;
pub mod ftdi;
pub mod hid;
pub mod host;
pub mod pio_host;
pub mod usb;
pub mod usbtmc;

/// USB bus state sampled from D+ and D-.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LineState {
    /// Both lines are low. For a host this normally means no device is attached.
    Se0,
    /// D+ is high and D- is low.
    JFullSpeed,
    /// D- is high and D+ is low.
    JLowSpeed,
    /// Both lines are high, which is not a valid idle state.
    Se1,
}

impl LineState {
    /// Decode the two least-significant PIO sample bits.
    ///
    /// Bit 0 is D+ and bit 1 is D-.
    pub const fn from_pio_sample(sample: u32) -> Self {
        match sample & 0b11 {
            0b00 => Self::Se0,
            0b01 => Self::JFullSpeed,
            0b10 => Self::JLowSpeed,
            _ => Self::Se1,
        }
    }
}

/// USB speed inferred from the device pull-up resistor.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DeviceSpeed {
    /// 12 Mbit/s signalling, indicated by D+ high at idle.
    Full,
    /// 1.5 Mbit/s signalling, indicated by D- high at idle.
    Low,
}

/// A debounced change on the host bus.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BusEvent {
    /// A device was attached.
    Attached(DeviceSpeed),
    /// The bus returned to the disconnected SE0 state.
    Detached,
    /// The invalid SE1 state remained stable.
    Invalid,
}

/// Debounces PIO line samples before exposing attach/detach events.
pub struct AttachDetector {
    stable: LineState,
    candidate: LineState,
    candidate_samples: u16,
    required_samples: u16,
}

impl AttachDetector {
    /// Create a detector.
    ///
    /// `required_samples` must be greater than zero.
    pub const fn new(required_samples: u16) -> Self {
        assert!(required_samples > 0);
        Self {
            stable: LineState::Se0,
            candidate: LineState::Se0,
            candidate_samples: 0,
            required_samples,
        }
    }

    /// Return the current debounced line state.
    pub const fn stable_state(&self) -> LineState {
        self.stable
    }

    /// Feed one sample and return an event when a new state becomes stable.
    pub fn update(&mut self, sample: LineState) -> Option<BusEvent> {
        if sample == self.stable {
            self.candidate = sample;
            self.candidate_samples = 0;
            return None;
        }

        if sample != self.candidate {
            self.candidate = sample;
            self.candidate_samples = 1;
        } else {
            self.candidate_samples = self.candidate_samples.saturating_add(1);
        }

        if self.candidate_samples < self.required_samples {
            return None;
        }

        self.stable = self.candidate;
        self.candidate_samples = 0;
        Some(match self.stable {
            LineState::Se0 => BusEvent::Detached,
            LineState::JFullSpeed => BusEvent::Attached(DeviceSpeed::Full),
            LineState::JLowSpeed => BusEvent::Attached(DeviceSpeed::Low),
            LineState::Se1 => BusEvent::Invalid,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pio_bits_map_to_usb_line_states() {
        assert_eq!(LineState::from_pio_sample(0b00), LineState::Se0);
        assert_eq!(LineState::from_pio_sample(0b01), LineState::JFullSpeed);
        assert_eq!(LineState::from_pio_sample(0b10), LineState::JLowSpeed);
        assert_eq!(LineState::from_pio_sample(0b11), LineState::Se1);
        assert_eq!(
            LineState::from_pio_sample(0xffff_fffd),
            LineState::JFullSpeed
        );
    }

    #[test]
    fn attach_requires_consecutive_samples() {
        let mut detector = AttachDetector::new(3);

        assert_eq!(detector.update(LineState::JFullSpeed), None);
        assert_eq!(detector.update(LineState::Se0), None);
        assert_eq!(detector.update(LineState::JFullSpeed), None);
        assert_eq!(detector.update(LineState::JFullSpeed), None);
        assert_eq!(
            detector.update(LineState::JFullSpeed),
            Some(BusEvent::Attached(DeviceSpeed::Full))
        );
    }

    #[test]
    fn detector_reports_speed_detach_and_invalid_state() {
        let mut detector = AttachDetector::new(2);

        detector.update(LineState::JLowSpeed);
        assert_eq!(
            detector.update(LineState::JLowSpeed),
            Some(BusEvent::Attached(DeviceSpeed::Low))
        );

        detector.update(LineState::Se1);
        assert_eq!(detector.update(LineState::Se1), Some(BusEvent::Invalid));

        detector.update(LineState::Se0);
        assert_eq!(detector.update(LineState::Se0), Some(BusEvent::Detached));
    }
}
