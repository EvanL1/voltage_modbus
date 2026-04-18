//! Fuzz target: PDU builder (`PduBuilder::build_read_request`)
//!
//! Feeds arbitrary (fc, address, quantity) triples into the PDU builder.
//! Invalid function codes return Err; valid ones build a stack-allocated PDU.
//! A panic or abort is a bug.
//!
//! Run:
//!   cd fuzz && cargo +nightly fuzz run fuzz_pdu_builder -- -max_total_time=60

#![no_main]

use libfuzzer_sys::{arbitrary, fuzz_target};
use voltage_modbus::pdu::PduBuilder;

/// Input struct — `arbitrary` derives a structured fuzzer input from raw bytes.
#[derive(Debug, arbitrary::Arbitrary)]
struct Input {
    fc: u8,
    address: u16,
    quantity: u16,
}

fuzz_target!(|input: Input| {
    // build_read_request returns Err for invalid FCs (anything outside 0x01–0x04).
    // For valid FCs it returns a stack-allocated PDU.  Neither path should panic.
    let result = PduBuilder::build_read_request(input.fc, input.address, input.quantity);

    if let Ok(pdu) = result {
        // Exercise the PDU accessors to make sure nothing panics there either.
        let _ = pdu.as_slice();
        let _ = pdu.len();
        let _ = pdu.function_code();
        let _ = pdu.is_exception();
    }
});
