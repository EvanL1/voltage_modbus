# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & Test Commands

```bash
# Build (default features = "std": TCP client/server + async runtime)
cargo build
cargo build --features rtu                 # Add RTU serial support
cargo build --features "rtu,igw"           # All std features
cargo build --no-default-features          # no_std build (core modules only: constants, error, pdu, protocol)

# Test
cargo test                                 # All unit + integration tests
cargo test --lib                           # Unit tests only
cargo test --test integration_tests        # Integration tests only
cargo test <test_name>                     # Single test by name
cargo test --features rtu                  # Include RTU tests

# Run examples / demo binary (all require --features std, which is default)
cargo run --example tcp_client
cargo run --bin demo

# Lint & Check
cargo clippy --all-targets -- -D warnings
cargo clippy --no-default-features -- -D warnings  # Verify no_std build stays clean
cargo fmt --check
cargo doc --no-deps                        # Build docs (verify doc-tests compile)
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

- **`client.rs`**: `ModbusClient` trait, `GenericModbusClient<T>`, `ModbusTcpClient`, `ModbusRtuClient`, batch read methods
- **`transport.rs`**: `ModbusTransport` trait, `TcpTransport` (MBAP framing, reconnection, transaction ID, pipelining), `RtuTransport` (CRC-16, frame gap timing), `TransportStats`, `PacketCallback`
- **`server.rs`**: `ModbusTcpServer` — TCP server implementation backed by `RegisterBank`
- **`register_bank.rs`**: `RegisterBank` — server-side storage for coils / discrete inputs / holding / input registers
- **`protocol.rs`**: `ModbusFunction` enum, `ModbusRequest`/`ModbusResponse` structs, `data_utils` for register/bit conversions
- **`pdu.rs`**: `ModbusPdu` — stack-allocated fixed-size buffer (253 bytes, no heap), `PduBuilder` fluent API
- **`error.rs`**: `ModbusError` enum (`thiserror` in std, hand-rolled `Display` in no_std), classifiable via `is_recoverable()`, `is_transport_error()`, `is_protocol_error()`
- **`codec.rs`**: `ModbusCodec` — encode/decode typed values (f32, f64, i32, u32, string) with configurable byte order
- **`bytes.rs`**: `ByteOrder` enum (BigEndian, LittleEndian, MidBigEndian, MidLittleEndian)
- **`batcher.rs`**: `CommandBatcher` — write command batching with configurable window and max batch size
- **`coalescer.rs`**: read-request coalescing — merges overlapping/adjacent read ranges into fewer on-wire requests
- **`value.rs`**: `ModbusValue` enum for typed industrial data values
- **`device_limits.rs`**: `DeviceLimits` — per-device protocol limit configuration
- **`constants.rs`**: Modbus spec constants (MAX_PDU_SIZE=253, MAX_READ_REGISTERS=125, etc.) — `no_std` safe
- **`logging.rs`** / **`utils.rs`**: tracing setup and shared helpers (std only)

### Feature Flags

- **`std`** (default): enables `tokio`, `thiserror`, `bytes`, `chrono` — full async TCP client/server
- **`rtu`**: implies `std`; adds `tokio-serial` for `ModbusRtuClient` / `RtuTransport`
- **`igw`**: implies `std`; optional IGW integration
- **no_std**: `cargo build --no-default-features` — only `constants`, `error`, `pdu`, `protocol` compile. Keep these four modules `alloc`/`core`-only; guard any `std`-dependent code behind `#[cfg(feature = "std")]`.

### Zero-Copy Response Parsing

`ModbusResponse` uses buffer+offset tracking (`new_from_frame`) to avoid copying payload data from TCP/RTU frames. The `data()` method returns a slice into the original frame buffer.

## Conventions

- MSRV: Rust 1.85.0, Edition 2021
- All I/O is async via Tokio
- Zero `unsafe` code — pure safe Rust
- Error construction uses factory methods: `ModbusError::timeout(op, ms)`, `ModbusError::frame(msg)`, etc.
- Protocol constants in `constants.rs` are derived from the Modbus spec with calculation comments
- Tests use `MockRtuTransport` in integration tests (no real serial hardware needed)
