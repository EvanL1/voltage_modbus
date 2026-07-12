//! Fuzz target: server-side device identification responder
//! (`server::DeviceIdentity::handle_request_fuzz`)
//!
//! Feeds arbitrary bytes as an FC 0x2B request payload to a populated
//! `DeviceIdentity` (the server-side object store backing
//! `ModbusTcpServer::set_device_identity`). This is a separate code path
//! from the client-side `DeviceIdentification::parse` fuzzed elsewhere —
//! it's the server's untrusted-input entry point, not the client's response
//! parser, and was not reachable through `fuzz_rtu_server_frame` since that
//! target dispatches through a plain `ModbusRegisterBank` service rather
//! than one wrapped with a device identity.
//!
//! A returned `Err` is fine; a panic is a bug.
//!
//! Run:
//!   cd fuzz && cargo +nightly fuzz run fuzz_device_identity_request -- -max_total_time=60

#![no_main]

use libfuzzer_sys::fuzz_target;
use voltage_modbus::DeviceIdentity;

fuzz_target!(|data: &[u8]| {
    let identity = DeviceIdentity::basic("VendorX", "ProductY", "1.2.3");
    let _ = identity.handle_request_fuzz(data);
});
