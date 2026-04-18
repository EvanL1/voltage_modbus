//! Fuzz target: TCP response frame parser (`ModbusResponse::new_from_frame`)
//!
//! Feeds arbitrary bytes through the zero-copy frame parsing path.
//! A returned `Err` is fine; a panic or abort is a bug.
//!
//! Run:
//!   cd fuzz && cargo +nightly fuzz run fuzz_response_frame -- -max_total_time=60

#![no_main]

use libfuzzer_sys::fuzz_target;
use voltage_modbus::protocol::{ModbusFunction, ModbusResponse};

fuzz_target!(|data: &[u8]| {
    // Need at least 4 bytes to derive all parameters.
    if data.len() < 4 {
        return;
    }

    // Interpret bytes[0] as function code (strip exception bit so we get a
    // valid FC; from_u8 will return Err on unknown codes — that's fine).
    let fc_byte = data[0] & 0x7F;
    let slave_id = data[1];

    let Ok(function) = ModbusFunction::from_u8(fc_byte) else {
        return;
    };

    let frame = data.to_vec();
    let frame_len = frame.len();

    // Derive data_start and data_len from the fuzzer bytes, bounded so that
    // `data_offset + data_len <= buffer.len()` (the invariant new_from_frame
    // requires — if it panics instead of being enforced, that's the bug).
    let data_start = (data[2] as usize) % frame_len;
    let max_len = frame_len.saturating_sub(data_start);
    let data_len = if max_len == 0 {
        0
    } else {
        (data[3] as usize) % (max_len + 1)
    };

    // Primary target: zero-copy frame constructor.
    let response =
        ModbusResponse::new_from_frame(frame, slave_id, function, data_start, data_len);

    // Exercise the slice-indexing path (`data()`) which can panic if
    // data_offset + data_len > buffer.len().
    let _ = response.data();
    let _ = response.data_len();
    let _ = response.is_exception();

    // Also exercise the two parsing paths on the response payload.
    let _ = response.parse_registers();
    let _ = response.parse_bits();
});
