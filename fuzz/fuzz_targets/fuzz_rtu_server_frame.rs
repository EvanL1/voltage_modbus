//! Fuzz target: RTU server frame processing (`ModbusRtuServer::process_frame_fuzz`)
//!
//! Feeds arbitrary bytes as a "request from a master" through the full
//! server-side path: CRC-16 validation, slave-id filtering (including
//! broadcast), function-code dispatch, and register-bank read/write
//! handling. This is the server's untrusted-input boundary — noise on the
//! wire, a corrupted frame, or a hostile peer all land here.
//!
//! `own_slave_id` is fuzzed too so both "addressed to us" and "addressed to
//! another slave" paths get exercised. A panic or hang is a bug; `None`
//! (frame silently ignored) and `Some(response)` are both valid outcomes.
//!
//! Run:
//!   cd fuzz && cargo +nightly fuzz run fuzz_rtu_server_frame -- -max_total_time=60

#![no_main]

use libfuzzer_sys::{arbitrary, fuzz_target};
use voltage_modbus::ModbusRtuServer;

#[derive(Debug, arbitrary::Arbitrary)]
struct Input<'a> {
    own_slave_id: u8,
    frame: &'a [u8],
}

fuzz_target!(|input: Input| {
    let rt = tokio::runtime::Builder::new_current_thread()
        .build()
        .expect("failed to build tokio runtime");
    rt.block_on(async {
        let _ = ModbusRtuServer::process_frame_fuzz(input.frame, input.own_slave_id).await;
    });
});
