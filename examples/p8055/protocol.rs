//! Velleman K8055/P8055 report format used by the HID example.
//!
//! These helpers are intentionally above the generic `HidHost`: they describe
//! one product's eight-byte reports, not USB HID itself.

#![allow(dead_code)]

pub const VELLEMAN_VENDOR_ID: u16 = 0x10cf;
pub const K8055_PRODUCT_ID_BASE: u16 = 0x5500;
pub const REPORT_LEN: usize = 8;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ReportError {
    InvalidLength,
}

/// One eight-byte interrupt-IN report from an original K8055/P8055.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct InputReport {
    bytes: [u8; REPORT_LEN],
}

impl InputReport {
    pub fn parse(bytes: &[u8]) -> Result<Self, ReportError> {
        let bytes = <[u8; REPORT_LEN]>::try_from(bytes).map_err(|_| ReportError::InvalidLength)?;
        Ok(Self { bytes })
    }

    pub const fn as_bytes(&self) -> &[u8; REPORT_LEN] {
        &self.bytes
    }

    /// Digital inputs I1..I5 in bits 0..4.
    ///
    /// The original board reports open inputs as one and grounded inputs as
    /// zero. This method preserves that electrical convention.
    pub const fn digital_inputs(&self) -> u8 {
        let raw = self.bytes[0];
        ((raw >> 4) & 0x03) | ((raw << 2) & 0x04) | ((raw >> 3) & 0x18)
    }

    pub const fn status(&self) -> u8 {
        self.bytes[1]
    }

    pub const fn analog_input_1(&self) -> u8 {
        self.bytes[2]
    }

    pub const fn analog_input_2(&self) -> u8 {
        self.bytes[3]
    }

    pub const fn counter_1(&self) -> u16 {
        u16::from_le_bytes([self.bytes[4], self.bytes[5]])
    }

    pub const fn counter_2(&self) -> u16 {
        u16::from_le_bytes([self.bytes[6], self.bytes[7]])
    }
}

/// Shadow of the K8055 outputs used to construct atomic output reports.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct OutputState {
    pub digital_outputs: u8,
    pub analog_output_1: u8,
    pub analog_output_2: u8,
}

impl OutputState {
    pub const fn all_off() -> Self {
        Self {
            digital_outputs: 0,
            analog_output_1: 0,
            analog_output_2: 0,
        }
    }

    /// Device reset command. This makes all digital and analog outputs zero.
    pub const fn reset_report() -> [u8; REPORT_LEN] {
        [0; REPORT_LEN]
    }

    /// Apply all shadowed digital and analog outputs in one report.
    pub const fn apply_report(self) -> [u8; REPORT_LEN] {
        [
            5,
            self.digital_outputs,
            self.analog_output_1,
            self.analog_output_2,
            0,
            0,
            0,
            0,
        ]
    }

    pub const fn reset_counter_1_report(self) -> [u8; REPORT_LEN] {
        [
            3,
            self.digital_outputs,
            self.analog_output_1,
            self.analog_output_2,
            0,
            0,
            0,
            0,
        ]
    }

    pub const fn reset_counter_2_report(self) -> [u8; REPORT_LEN] {
        [
            4,
            self.digital_outputs,
            self.analog_output_1,
            self.analog_output_2,
            0,
            0,
            0,
            0,
        ]
    }

    /// Encode the board's nonlinear debounce-time field from microseconds.
    ///
    /// The nearest supported setting is `115 us * raw²`.
    pub fn debounce_raw_micros(microseconds: u32) -> u8 {
        let mut raw = 0_u16;
        loop {
            let current = 115_u32 * u32::from(raw) * u32::from(raw);
            if raw == u16::from(u8::MAX) {
                return u8::MAX;
            }
            let next_raw = raw + 1;
            let next = 115_u32 * u32::from(next_raw) * u32::from(next_raw);
            let current_error = current.abs_diff(microseconds);
            let next_error = next.abs_diff(microseconds);
            if current_error <= next_error {
                return raw as u8;
            }
            raw = next_raw;
        }
    }

    pub fn set_debounce_1_report(self, microseconds: u32) -> [u8; REPORT_LEN] {
        [
            1,
            self.digital_outputs,
            self.analog_output_1,
            self.analog_output_2,
            0,
            0,
            Self::debounce_raw_micros(microseconds),
            0,
        ]
    }

    pub fn set_debounce_2_report(self, microseconds: u32) -> [u8; REPORT_LEN] {
        [
            2,
            self.digital_outputs,
            self.analog_output_1,
            self.analog_output_2,
            0,
            0,
            0,
            Self::debounce_raw_micros(microseconds),
        ]
    }
}

pub const fn is_original_k8055(vendor_id: u16, product_id: u16) -> bool {
    vendor_id == VELLEMAN_VENDOR_ID
        && product_id >= K8055_PRODUCT_ID_BASE
        && product_id <= K8055_PRODUCT_ID_BASE + 3
}
