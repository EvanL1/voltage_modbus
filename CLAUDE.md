# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & Test Commands

```bash
# Build (default: TCP only)
cargo build
cargo build --features rtu          # With RTU serial support
cargo build --features "rtu,igw"    # All features

# Test
cargo test                          # All unit + integration tests
cargo test --lib                    # Unit tests only
cargo test --test integration_tests # Integration tests only
cargo test <test_name>              # Single test by name
cargo test -p voltage_modbus <mod>::tests  # Tests in a specific module

# Lint & Check
cargo clippy -- -D warnings
cargo fmt --check
cargo doc --no-deps                 # Build docs (verify doc-tests compile)
```

## Architecture

Single-crate library (`voltage_modbus`) implementing Modbus TCP and RTU protocols using async Tokio.

### Layered Design

```
Application:  ModbusTcpClient / ModbusRtuClient  (user-facing, convenience methods)
                          ↓
Generic:      GenericModbusClient<T: ModbusTransport>  (shared PDU logic for all FC01-FC16)
                          ↓
Transport:    TcpTransport / RtuTransport  (frame encapsulation, CRC, MBAP headers)
```

**Key insight**: TCP and RTU share identical PDU (Protocol Data Unit). They differ only in transport framing — TCP uses MBAP headers with transaction IDs, RTU uses slave ID prefix + CRC-16. `GenericModbusClient<T>` implements all Modbus function codes once; `ModbusTcpClient` and `ModbusRtuClient` are thin wrappers that create the appropriate transport.

### Dual API Naming

Client methods use function-code naming as primary (`read_03`, `write_06`) with semantic aliases (`read_holding_registers`, `write_single_register`). The `ModbusClient` trait defines the interface.

### Module Responsibilities

- **`client.rs`** (1656L): `ModbusClient` trait, `GenericModbusClient<T>`, `ModbusTcpClient`, `ModbusRtuClient`, batch read methods
- **`transport.rs`** (2085L): `ModbusTransport` trait, `TcpTransport` (MBAP framing, reconnection, transaction ID), `RtuTransport` (CRC-16, frame gap timing), `TransportStats`, `PacketCallback`
- **`protocol.rs`**: `ModbusFunction` enum, `ModbusRequest`/`ModbusResponse` structs, `data_utils` module for register/bit conversions
- **`pdu.rs`**: `ModbusPdu` — stack-allocated fixed-size buffer (253 bytes, no heap), `PduBuilder` fluent API for constructing PDUs
- **`error.rs`**: `ModbusError` enum with `thiserror`, classifiable via `is_recoverable()`, `is_transport_error()`, `is_protocol_error()`
- **`codec.rs`**: `ModbusCodec` — encode/decode typed values (f32, f64, i32, u32, string) with configurable byte order
- **`bytes.rs`**: `ByteOrder` enum (BigEndian, LittleEndian, MidBigEndian, MidLittleEndian) for multi-register data types
- **`batcher.rs`**: `CommandBatcher` — write command batching with configurable window and max batch size
- **`value.rs`**: `ModbusValue` enum for typed industrial data values
- **`device_limits.rs`**: `DeviceLimits` — per-device protocol limit configuration
- **`constants.rs`**: Modbus spec constants (MAX_PDU_SIZE=253, MAX_READ_REGISTERS=125, etc.)

### Feature Flags

- **default**: TCP only (no optional deps)
- **`rtu`**: Enables `ModbusRtuClient` and `RtuTransport` via `tokio-serial`
- **`igw`**: IGW integration (optional)

### Zero-Copy Response Parsing

`ModbusResponse` uses buffer+offset tracking (`new_from_frame`) to avoid copying payload data from TCP/RTU frames. The `data()` method returns a slice into the original frame buffer.

## Conventions

- MSRV: Rust 1.85.0, Edition 2021
- All I/O is async via Tokio
- Zero `unsafe` code — pure safe Rust
- Error construction uses factory methods: `ModbusError::timeout(op, ms)`, `ModbusError::frame(msg)`, etc.
- Protocol constants in `constants.rs` are derived from the Modbus spec with calculation comments
- Tests use `MockRtuTransport` in integration tests (no real serial hardware needed)
