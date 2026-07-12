//! Fuzz target: RTU length-aware frame reader (`transport::read_rtu_frame_fuzz`)
//!
//! Exercises the per-function-code length-derivation branches added for
//! FC 0x07/0x08/0x0B/0x0C/0x11/0x16/0x2B, including the FC 0x2B per-object
//! read loop driven by device-controlled `mei[5]` (object count) and
//! `obj_header[1]` (per-object length) fields — the kind of length-prefixed
//! loop most prone to off-by-one or truncation bugs.
//!
//! The first 2 fuzzer bytes select the function code (passed directly as the
//! "already read" header, matching how the real caller reads it first); the
//! rest of the input is what the reader consumes via `read_exact` calls.
//! A returned `Err` (including EOF from running out of bytes) is fine; a
//! panic, hang, or excessive allocation is a bug.
//!
//! Run:
//!   cd fuzz && cargo +nightly fuzz run fuzz_rtu_frame_reader -- -max_total_time=60

#![no_main]

use libfuzzer_sys::fuzz_target;
use std::io::Cursor;
use voltage_modbus::transport::read_rtu_frame_fuzz;

fuzz_target!(|data: &[u8]| {
    if data.len() < 2 {
        return;
    }
    let header = [data[0], data[1]];

    let rt = tokio::runtime::Builder::new_current_thread()
        .build()
        .expect("failed to build tokio runtime");
    rt.block_on(async {
        let mut reader = Cursor::new(&data[2..]);
        let _ = read_rtu_frame_fuzz(&mut reader, header).await;
    });
});
