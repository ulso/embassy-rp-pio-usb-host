#![allow(dead_code)]

#[path = "../examples/p8055/protocol.rs"]
mod protocol;

use protocol::*;

#[test]
fn identifies_all_four_original_board_addresses_only() {
    for product_id in 0x5500..=0x5503 {
        assert!(is_original_k8055(0x10cf, product_id));
    }
    assert!(!is_original_k8055(0x10cf, 0x5504));
    assert!(!is_original_k8055(0x10ce, 0x5500));
}

#[test]
fn input_report_decodes_the_non_linear_digital_bit_order() {
    for (raw_bit, logical_bit) in [(4, 0), (5, 1), (0, 2), (6, 3), (7, 4)] {
        let mut bytes = [0; REPORT_LEN];
        bytes[0] = 1 << raw_bit;
        let report = InputReport::parse(&bytes).unwrap();
        assert_eq!(report.digital_inputs(), 1 << logical_bit);
    }
}

#[test]
fn input_report_preserves_analog_status_and_little_endian_counters() {
    let report = InputReport::parse(&[0, 3, 17, 29, 0x34, 0x12, 0x78, 0x56]).unwrap();
    assert_eq!(report.status(), 3);
    assert_eq!(report.analog_input_1(), 17);
    assert_eq!(report.analog_input_2(), 29);
    assert_eq!(report.counter_1(), 0x1234);
    assert_eq!(report.counter_2(), 0x5678);
    assert_eq!(report.as_bytes(), &[0, 3, 17, 29, 0x34, 0x12, 0x78, 0x56]);
    assert_eq!(
        InputReport::parse(&[0; REPORT_LEN - 1]),
        Err(ReportError::InvalidLength)
    );
}

#[test]
fn output_reports_are_exactly_eight_wire_bytes_without_report_id() {
    let state = OutputState {
        digital_outputs: 0x55,
        analog_output_1: 17,
        analog_output_2: 29,
    };
    assert_eq!(OutputState::reset_report(), [0; 8]);
    assert_eq!(state.apply_report(), [5, 0x55, 17, 29, 0, 0, 0, 0]);
    assert_eq!(
        state.reset_counter_1_report(),
        [3, 0x55, 17, 29, 0, 0, 0, 0]
    );
    assert_eq!(
        state.reset_counter_2_report(),
        [4, 0x55, 17, 29, 0, 0, 0, 0]
    );
}

#[test]
fn debounce_encoding_is_clamped_and_placed_in_the_correct_channel() {
    let state = OutputState::all_off();
    assert_eq!(OutputState::debounce_raw_micros(0), 0);
    assert_eq!(OutputState::debounce_raw_micros(115), 1);
    assert_eq!(OutputState::debounce_raw_micros(2_875), 5);
    assert_eq!(OutputState::debounce_raw_micros(u32::MAX), 255);
    assert_eq!(state.set_debounce_1_report(2_875), [1, 0, 0, 0, 0, 0, 5, 0]);
    assert_eq!(state.set_debounce_2_report(2_875), [2, 0, 0, 0, 0, 0, 0, 5]);
}
