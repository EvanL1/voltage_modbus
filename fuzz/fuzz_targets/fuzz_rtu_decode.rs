//! Fuzz target: RTU response frame decoder (`RtuTransport::decode_response_fuzz`)
//!
//! Feeds arbitrary bytes through the RTU CRC-verification and frame-parsing path.
//! A returned `Err` is fine; a panic or abort is a bug.
//!
//! Run:
//!   cd fuzz && cargo +nightly fuzz run fuzz_rtu_decode -- -max_total_time=60

#![no_main]

use libfuzzer_sys::fuzz_target;
use voltage_modbus::transport::RtuTransport;

fuzz_target!(|data: &[u8]| {
    // The RTU decode path requires at least 4 bytes (slave_id, fc, data, CRC×2).
    // Shorter frames are rejected early with Err — that's expected behaviour.
    let transport = RtuTransport::new_for_fuzz();
    // Any result (Ok or Err) is acceptable; panics are bugs.
    let _ = transport.decode_response_fuzz(data.to_vec());
});
