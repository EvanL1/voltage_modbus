# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & Test Commands

```bash
# Build (default features = "std": TCP client/server + async runtime)
cargo build
cargo build --features rtu                 # Add RTU serial support
cargo build --features tls                 # Add Modbus/TCP Security (TLS client)
cargo build --all-features                 # All features (rtu, tls, embedded, defmt)
cargo build --no-default-features          # no_std build (core modules only: constants, error, pdu, protocol)
cargo build --no-default-features --features embedded,defmt  # no_std + alloc embedded transport

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

- **`client.rs`**: `ModbusClient` trait, `GenericModbusClient<T>`, `ModbusTcpClient`, `ModbusRtuClient`, batch read methods, `RetryPolicy` (opt-in retry with exponential backoff for recoverable errors), extended FCs (`write_16` mask write, `read_write_17`, `read_device_identification`)
- **`transport.rs`**: `ModbusTransport` trait, `TcpTransport` (MBAP framing, reconnection, transaction ID, pipelining), `TlsTransport` (`tls`), `RtuOverTcpTransport`, `RtuTransport` / `AsciiTransport` (`rtu`; CRC-16 / LRC, spec t3.5 frame gap, length-aware frame reads), `TransportStats`, `PacketCallback`. All TCP connects go through `connect_tcp()` (timeout-bounded); RTU-over-TCP takes the stream out for each round trip so a cancelled request cannot leave a stale reply. PDU bodies come from the shared `ModbusRequest::encode_pdu()` — add new function codes there, not per-transport
- **`server.rs`**: `ModbusTcpServer` / `ModbusRtuServer`, plus the `ModbusService` trait — servers dispatch raw PDUs to a service; `ModbusRegisterBank` is the default in-memory implementation, `set_service()` swaps in custom logic
- **`rtu_frame.rs`** (`rtu`, private): `is_partial_request()` — lets the RTU server wait for the rest of a request to it split by USB-adapter chunking. RTU server framing is otherwise t3.5 silence + whole-frame CRC: a CRC-guessing frame splitter was tried for 1.1.0 and abandoned after three review rounds (CRC-16 residue makes "frame + 0x00" ambiguous; resync executed phantom broadcast writes). Keep changes here conservative: delaying a frame is acceptable, inventing one is not
- **`register_bank.rs`**: `RegisterBank` — server-side storage for coils / discrete inputs / holding / input registers
- **`protocol.rs`**: `ModbusFunction` enum, `ModbusRequest`/`ModbusResponse` structs, `data_utils` for register/bit conversions
- **`pdu.rs`**: `ModbusPdu` — stack-allocated fixed-size buffer (253 bytes, no heap), `PduBuilder` fluent API
- **`error.rs`**: `ModbusError` enum (`thiserror` in std, hand-rolled `Display` in no_std), classifiable via `is_recoverable()`, `is_transport_error()`, `is_protocol_error()`
- **`codec.rs`**: free functions (`decode_register_value`, `encode_value`, `encode_f64_as_type`, `registers_for_type`) — typed values ↔ registers with configurable byte order
- **`bytes.rs`**: `ByteOrder` enum (BigEndian, LittleEndian, BigEndianSwap, LittleEndianSwap, BigEndian16, LittleEndian16) plus `regs_to_*` helpers
- **`batcher.rs`**: `CommandBatcher` — write command batching with configurable window and max batch size
- **`coalescer.rs`**: read-request coalescing — merges overlapping/adjacent read ranges into fewer on-wire requests
- **`value.rs`**: `ModbusValue` enum for typed industrial data values
- **`device_limits.rs`**: `DeviceLimits` — per-device protocol limit configuration
- **`constants.rs`**: Modbus spec constants (MAX_PDU_SIZE=253, MAX_READ_REGISTERS=125, etc.) — `no_std` safe
- **`logging.rs`**: `CallbackLogger` / packet logging helpers (std only)
- **`embedded.rs`** (`embedded`): `EmbeddedRtuTransport` — no_std + alloc RTU over `embedded-io-async`; callers must bound each request with their own timeout

### Feature Flags

- **`std`** (default): enables `tokio`, `thiserror` — full async TCP client/server
- **`rtu`**: implies `std`; adds `tokio-serial` for `ModbusRtuClient` / `RtuTransport`
- **`tls`**: implies `std`; adds `tokio-rustls` (ring provider) for `ModbusTlsClient` / `TlsTransport` — Modbus/TCP Security, caller supplies the `rustls::ClientConfig`
- **`embedded`**: no_std + alloc; adds the `embedded` module (`embedded-io-async`, `heapless`)
- **`defmt`**: derives `defmt::Format` for the no_std public types
- **no_std**: `cargo build --no-default-features` — only `constants`, `error`, `pdu`, `protocol` compile. Keep these four modules `alloc`/`core`-only; guard any `std`-dependent code behind `#[cfg(feature = "std")]`.

### Zero-Copy Response Parsing

`ModbusResponse` uses buffer+offset tracking (`new_from_frame`) to avoid copying payload data from TCP/RTU frames. The `data()` method returns a slice into the original frame buffer.

## Conventions

- MSRV: Rust 1.85.0, Edition 2021
- All std I/O is async via Tokio (the `embedded` transport uses `embedded-io-async`)
- Semver (since 1.0): public enums and stats structs are `#[non_exhaustive]`; no new `pub` fields — add getters. Breaking changes need a major version. Fuzz entry points are `#[cfg(any(fuzzing, test))]`
- Zero `unsafe` code — pure safe Rust
- Error construction uses factory methods: `ModbusError::timeout(op, ms)`, `ModbusError::frame(msg)`, etc.
- Protocol constants in `constants.rs` are derived from the Modbus spec with calculation comments
- Tests use `MockRtuTransport` in integration tests (no real serial hardware needed)
