//! Fuzz target: client-side response validation
//! (`client::validate_response_matches_request_fuzz`)
//!
//! Every response-validation branch — including the ones backing
//! `get_comm_event_log`/`report_server_id`/`diagnostics`, which index into
//! `response.data()` on the assumption this function already bounds-checked
//! it — is normally only reachable through a live transport round trip.
//! This constructs a `(ModbusRequest, ModbusResponse)` pair directly from
//! fuzzer bytes so every function-code branch (mask write, read/write
//! multiple, device identification, all 5 serial diagnostics FCs, and the
//! original read/write FCs) gets exercised without needing a real
//! connection.
//!
//! A returned `Err` is fine; a panic is a bug.
//!
//! Run:
//!   cd fuzz && cargo +nightly fuzz run fuzz_response_validation -- -max_total_time=60

#![no_main]

use libfuzzer_sys::{arbitrary, fuzz_target};
use voltage_modbus::client::validate_response_matches_request_fuzz;
use voltage_modbus::{ModbusFunction, ModbusRequest, ModbusResponse};

const FUNCTIONS: &[ModbusFunction] = &[
    ModbusFunction::ReadCoils,
    ModbusFunction::ReadDiscreteInputs,
    ModbusFunction::ReadHoldingRegisters,
    ModbusFunction::ReadInputRegisters,
    ModbusFunction::WriteSingleCoil,
    ModbusFunction::WriteSingleRegister,
    ModbusFunction::ReadExceptionStatus,
    ModbusFunction::Diagnostics,
    ModbusFunction::GetCommEventCounter,
    ModbusFunction::GetCommEventLog,
    ModbusFunction::WriteMultipleCoils,
    ModbusFunction::WriteMultipleRegisters,
    ModbusFunction::ReportServerId,
    ModbusFunction::MaskWriteRegister,
    ModbusFunction::ReadWriteMultipleRegisters,
    ModbusFunction::ReadDeviceIdentification,
];

#[derive(Debug, arbitrary::Arbitrary)]
struct Input {
    function_idx: u8,
    slave_id: u8,
    address: u16,
    quantity: u16,
    request_data: Vec<u8>,
    response_slave_id: u8,
    response_is_exception: bool,
    response_exception_code: u8,
    response_data: Vec<u8>,
}

fuzz_target!(|input: Input| {
    let function = FUNCTIONS[input.function_idx as usize % FUNCTIONS.len()];

    let request = ModbusRequest {
        slave_id: input.slave_id,
        function,
        address: input.address,
        quantity: input.quantity,
        data: input.request_data,
    };

    let response = if input.response_is_exception {
        ModbusResponse::new_exception(
            input.response_slave_id,
            function,
            input.response_exception_code,
        )
    } else {
        ModbusResponse::new_success(input.response_slave_id, function, input.response_data)
    };

    let _ = validate_response_matches_request_fuzz(&request, &response);
});
