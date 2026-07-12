//! Fuzz target: Device identification response parser
//! (`protocol::DeviceIdentification::parse`)
//!
//! Feeds arbitrary bytes through the FC 0x2B / MEI 0x0E response parser — a
//! nested length-prefixed loop (object count, then per-object id+len+value)
//! that is exactly the shape most prone to truncation/overflow bugs on
//! malformed or adversarial device responses.
//!
//! A returned `Err` is fine; a panic is a bug.
//!
//! Run:
//!   cd fuzz && cargo +nightly fuzz run fuzz_device_identification -- -max_total_time=60

#![no_main]

use libfuzzer_sys::fuzz_target;
use voltage_modbus::protocol::DeviceIdentification;

fuzz_target!(|data: &[u8]| {
    if let Ok(ident) = DeviceIdentification::parse(data) {
        // Exercise the accessors too — they re-index into parsed data.
        let _ = ident.object(0);
        let _ = ident.object(0x80);
        for obj in &ident.objects {
            let _ = obj.as_str();
        }
    }
});
