# Changelog

All notable changes to Voltage Modbus library will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [1.1.0] - 2026-10-09

### Changed
- **RTU server waits for the rest of a partly-arrived request.** USB-RS485 adapters
  deliver bytes in chunks (FTDI's default latency timer is 16 ms), far longer apart
  than t3.5 at high baud rates, so a request to the server was split at the gap and
  dropped. Now, when the bytes received so far are the beginning of a fixed-layout
  request addressed to this server (or broadcast), the server waits up to 50 ms after
  the last byte for the rest — however long the request, since each chunk renews the
  wait (and never less than the configured `frame_gap`). Everything else is unchanged
  from 1.0.1: frames are delimited by t3.5 silence and processed only whole and
  CRC-valid. If the wait does not end in a CRC-valid request — a glitch byte, a
  truncated request followed by the master's retry — the bytes are split at every gap
  that was waited through, i.e. into exactly the frames 1.0.1 would have processed.
  So the only new frame is a reassembled request; nothing 1.0.1 answered is lost.
  Other slaves' traffic is never waited on. No API or configuration changes.

### Fixed
- The RTU server loop now stops when the port reports EOF; it used to spin at 100 % CPU.

### Known limitations
- If another slave's reply and the master's request to this server reach the server
  in the *same* USB chunk, the merged bytes still fail the CRC and the request is
  dropped (as in 1.0.1). Lower the adapter's latency timer to avoid it, e.g. on Linux
  `echo 1 > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer`.

## [1.0.1] - 2026-10-09

### Fixed
- **Server FC 0x16 / FC 0x17 were not atomic**: Mask Write and Read/Write Multiple
  took the register lock once per step, so a concurrent write from another
  connection could be lost. Each now runs under a single lock. FC 0x17 also checks
  the read range before writing, so an invalid request leaves registers unchanged.
- **RTU server latency and memory**: a request was only answered after a fixed 100 ms
  read timeout; it is now answered once `frame_gap` (t3.5) of silence ends the frame.
  Frame splitting is unchanged from 1.0.0 (a `frame_gap` silence separates frames, so
  back-to-back frames on a multi-drop bus still split correctly). The frame buffer is
  capped at the 256-byte RTU ADU; longer bursts are discarded.
- **TCP accept loop** no longer spins at 100 % CPU on persistent `accept()` errors
  (e.g. out of file descriptors); it backs off 100 ms.
- **TCP/TLS timeouts**: one request now takes at most ~1× the configured timeout
  (header, body and skipped stale frames share one deadline; previously up to ~10×).
  A peer closing or resetting the connection is reported as `ModbusError::Connection`
  instead of `Timeout` and no longer counts in `stats.timeouts`. Both stay
  `is_recoverable()`; code matching `Timeout` specifically to trigger a reconnect
  should also handle `Connection`.
- **Pipelining**: when some replies never arrive, `pipeline` / `pipeline_reads` now
  return the replies that did arrive plus per-entry timeout errors, as documented,
  instead of failing the whole call — including when *no* reply arrives (`Ok` with
  every entry `Err(Timeout)`). Code that detected timeouts with `pipeline(..).is_err()`
  must inspect the entries. Every pipelined reply is validated against its request
  (function code, byte count, slave ID); a bad one fails only its entry.
- **Codec, 16-bit types**: `uint16`/`int16` decode and encode now honor
  `ByteOrder::LittleEndian16` (byte swap), matching `bytes::reg_to_u16`. Other byte
  orders are unchanged.
- **Codec, type aliases**: all aliases (`word`, `short`, `dword`, `long`, `float`,
  `real`, `qword`, `longlong`, `double`, `lreal`, …) now clamp, encode, decode and size
  exactly like their canonical names — e.g. `encode_f64_as_type(1e40, "real")` writes
  `f32::MAX` instead of `+inf`.
- **Embedded transport**: a broadcast (slave 0) request returned never — it now
  returns an ack right after sending. Replies from the wrong slave or with the wrong
  function code are rejected. Callers must still bound each request with their own
  timeout (documented).
- **CommandBatcher**: the batch window now starts at the first command after an idle
  period (it used to flush single commands immediately), and contiguity checks near
  address 65535 no longer overflow.
- **DeviceLimits**: `with_max_*` builders clamp to `1..=` the Modbus spec maximum, so
  `with_max_read_registers(0)` no longer panics later (it now means one item per
  request) and values above the spec (e.g. 200 registers) become the spec maximum (125).
- **ReadCoalescer**: the default limit for coil / discrete-input reads is the spec's
  2000 (was 125). Only direct users of `ReadCoalescer` see this (the client coalesces
  FC03/04 only); devices with a lower coil limit should set it via `with_config`.
- `ModbusRequest::new_write` no longer overflows on ≥ 8192 bytes of data; `validate()`
  rejects such requests.

### Known limitations
- The RTU **server** still delimits request frames by silence. At high baud rates a
  USB-RS485 adapter that delivers bytes in ~16 ms chunks can split a frame (as in
  1.0.0); the RTU *client* is not affected (it reads by frame length). Addressed in
  1.1.0 for requests to the server.

### Changed
- CI runs `cargo-semver-checks` against the latest release; the MSRV job checks all features.

## [1.0.0] - 2026-10-09

First stable release. From here on the public API follows Semantic Versioning; see
the *Stability* section of the README for the exact guarantee. This release also
contains the fixes prepared as 0.7.3, which was never published.

### Security
- Lockfile: `rustls` 0.23.41 → 0.23.45 (RUSTSEC-2026-0285: TLS 1.3 handshake messages
  accepted across encryption level boundaries; affects the `tls` feature). Downstream
  builds resolve their own `rustls` — run `cargo update -p rustls` if your lockfile
  pins an older 0.23.x.

### Breaking changes
- **`#[non_exhaustive]`** on `ModbusError`, `ModbusFunction`, `ModbusException`,
  `ByteOrder`, `ModbusValue`, `LogLevel`, `LoggingMode`; on the statistics structs
  `TransportStats`, `ServerStats`, `RegisterBankStats`; on the config structs
  `ModbusTcpServerConfig`, `ModbusRtuServerConfig`, `RetryPolicy`, `DeviceLimits`; and
  on the returned data structs `CoalescedRead`, `CommEventLog`, `ServerIdReport`,
  `DeviceIdObject`, `DeviceIdentification`. Exhaustive `match`es need a `_` arm.
  Struct literals (including `..Default::default()`) no longer compile outside the
  crate: start from `Default::default()` / the constructor and assign fields; custom
  `ModbusTransport` impls build stats with `TransportStats::default()`.
- **`ModbusServer` is sealed** — it was only ever implemented by `ModbusTcpServer` /
  `ModbusRtuServer`; custom server behavior goes through `ModbusService`.
- **Fields → getters**: `ModbusResponse::exception` is now a method
  (`response.exception()`); `TcpTransport::address` / `TlsTransport::address` are now
  `address()`. The fields are private.
- **Exception responses** surface as `ModbusError::Exception` instead of
  `ModbusError::Protocol` (see *Fixed*).
- **Removed** (unused or superseded):
  - `scheduler` module and the `ScheduledRequest` trait.
  - `ModbusCodec` and its `build_fc05_pdu` / `build_fc06_pdu` / `build_fc15_pdu` /
    `build_fc16_pdu` / `parse_write_response` — use `ModbusRequest` (`encode_pdu()`)
    or `PduBuilder`.
  - `codec::parse_read_response` (returned `Ok(empty)` on truncated PDUs) — use
    `ModbusResponse::parse_registers()` / `parse_bits()`.
  - The crate-level `utils` module: `PerformanceMetrics`, `OperationTimer`,
    `utils::validation`, `utils::format`, `utils::logging` (`client::utils` stays).
  - `protocol::ModbusValue` (a `u16` alias that clashed with `value::ModbusValue`).
  - Deprecated `ModbusError` variants `TimeoutLegacy`, `InvalidFrame`,
    `InvalidDataValue`, `IllegalFunction`, `InternalError` — use `Timeout`, `Frame`,
    `InvalidData`, `InvalidFunction`, `Internal`.
  - The `igw` feature, which enabled a dependency but no code.
- **Fuzz entry points** (`*_fuzz`, `RtuTransport::new_for_fuzz`) only compile under
  `cfg(fuzzing)` (set by cargo-fuzz) and for the crate's own tests.
- Crate-root re-exports of the codec / byte-order helpers (`regs_to_f32`,
  `decode_register_value`, `DEFAULT_*`, …) are no longer `#[doc(hidden)]`; they are
  documented, supported API.

### Migration from 0.7
- `match err { … }` over `ModbusError` (or `ByteOrder`, `ModbusFunction`, …): add `_ => …`.
- `resp.exception` → `resp.exception()`; prefer `resp.is_exception()` /
  `resp.get_exception()`, which also cover codes outside `ModbusException`.
- `transport.address` → `transport.address()`.
- `ModbusTcpServerConfig { bind_address, ..Default::default() }` →
  `let mut cfg = ModbusTcpServerConfig::default(); cfg.bind_address = …;` (same for the
  other config structs; `DeviceLimits` and `RetryPolicy` also have builder methods).
- Code that matched `ModbusError::Protocol` to detect device exceptions: match
  `ModbusError::Exception { code, .. }`.
- If you used anything from the removed list, the replacement is named next to it above.

### Fixed
- **docs.rs build**: 0.7.2's documentation failed to build on docs.rs because nightly removed `feature(doc_auto_cfg)` (merged into `doc_cfg`). Switched to `feature(doc_cfg)`; CI now reproduces the docs.rs build (nightly + `--cfg docsrs`).
- **TCP connect had no timeout**: every `TcpStream::connect` (TCP, TLS, RTU-over-TCP; initial connect and reconnect) now goes through one helper bounded by the transport's configured timeout. Previously a peer that silently drops SYNs (e.g. a powered-off PLC) blocked for the OS connect timeout (75–127 s), holding any `SharedModbusClient` lock throughout.
- **Serial port never reopened after an I/O error**: `RtuTransport` and `AsciiTransport` now drop the port handle on a write or read I/O error, so the next request reopens it. Previously an unplugged and re-plugged USB-RS485 adapter failed forever, and `RetryPolicy` retried against the dead handle.
- **RTU-over-TCP stale reply after cancellation**: if a request future was dropped mid-flight (caller `tokio::time::timeout` / `select!`), the late reply stayed in the socket and the next request returned it as its own data — RTU framing has no transaction ID to catch this. The stream is now taken for the whole round trip and kept only after a complete, CRC-valid frame; a CRC/decode failure also drops the connection, since the stream may be misaligned.
- **Exception codes were lost**: `ModbusResponse::get_exception()` returned `ModbusError::Protocol(String)`, so `is_recoverable()` never retried Acknowledge (0x05) / Server Busy (0x06) and callers could not match on the code. It now returns `ModbusError::Exception { function, code, .. }`. Exception codes outside `ModbusException` (e.g. 0x0C) are now still treated as exceptions instead of being parsed as a normal response.
- **README examples did not compile**: the pipelining example was missing the `ModbusClient` import, the coalescing example called a method that exists only on the generic client (now via `generic_mut()`), and the install snippets pointed at 0.5. README code blocks are now compiled as doctests (`cargo test --all-features`).

### Behavior changes
- Exception responses now surface as `ModbusError::Exception` instead of `ModbusError::Protocol`. The `Display` text still starts with `Modbus exception:` and `is_protocol_error()` is still true, but code that matched the `Protocol` variant for exceptions takes a different branch. With a `RetryPolicy` set, 0x05/0x06 exceptions are now retried as `is_recoverable()` always intended.
- A connect that exceeds the timeout fails with `ModbusError::Timeout` (previously, after the OS gave up, `Connection`). Both are recoverable transport errors. The timeout applies to connect and to each I/O step separately, so a request that reconnects first can take up to 2× the timeout (3× for TLS, which adds the handshake).

### Changed
- Removed the unused direct dependencies `chrono` and `bytes` (`crate::bytes` is the crate's own module).
- `#![forbid(unsafe_code)]` now enforces the "zero unsafe" claim.
- CI also checks the `embedded` + `defmt` build on `thumbv7em-none-eabihf`.

## [0.7.2] - 2026-08-04

### Security
- Updated `bytes` 1.10.1 → 1.12.1 (RUSTSEC-2026-0007: integer overflow in `BytesMut::reserve`) and `crossbeam-epoch` 0.9.18 → 0.9.20 (RUSTSEC-2026-0204: invalid pointer dereference in the `fmt::Pointer` impl for `Atomic`/`Shared`). `crossbeam-epoch` enters the tree only through `criterion`, a dev-dependency, so it never reached the published library build.

### Changed
- Collapsed three `match` arms whose body was a lone `if` into match guards, clearing the `clippy::collapsible_match` errors that Rust 1.97 began reporting: the FC 0x17 read-quantity check and the FC 0x08 payload check in `protocol.rs`, and FC 0x0F/0x10 response formatting in `logging.rs`. Behavior is unchanged — each of the three arms sits immediately before the catch-all it now falls through to, which is also why clippy flagged only these three of the six structurally similar arms.

## [0.7.1] - 2026-07-12

### Fixed
- `validate_response_matches_request`'s Diagnostics (FC 0x08) branch indexed `request.data[0..2]` after only checking the *response*'s length, panicking if `request.data` was shorter than 2 bytes. Not reachable through the public client API (`ModbusRequest::validate()` already rejects a too-short Diagnostics request before this code runs), but a real latent panic found by fuzzing on its first run in CI.

### Added
- Five new fuzz targets covering code added in 0.7.0 that the original fuzz corpus never exercised: the length-aware RTU frame reader (including the FC 0x2B per-object read loop), `DeviceIdentification::parse`, the RTU server's frame-processing entry point, the server-side FC 0x2B `DeviceIdentity` responder, and `validate_response_matches_request` itself. Runs daily in CI (`.github/workflows/fuzz.yml`) with corpus caching and crash-artifact upload.

## [0.7.0] - 2026-07-12

### Added
- **Extended function codes**: FC 0x16 (Mask Write Register), FC 0x17 (Read/Write Multiple Registers), FC 0x2B/MEI 0x0E (Read Device Identification, client + server via `DeviceIdentity`/`set_device_identity`). All PDU bodies now flow through a single shared `ModbusRequest::encode_pdu()` used by every transport.
- **Serial-line diagnostics**: FC 0x07 (Read Exception Status), FC 0x08 (Diagnostics, sub 0x0000 echo test), FC 0x0B (Get Comm Event Counter), FC 0x0C (Get Comm Event Log), FC 0x11 (Report Server ID), with `CommEventLog`/`ServerIdReport` response types.
- **`RetryPolicy`**: opt-in exponential-backoff retry on the client for recoverable errors (timeouts, connection loss, device-busy); default is no retries, so existing behavior is unchanged.
- **`ModbusService` trait**: TCP/RTU servers now dispatch raw PDUs through a pluggable service; `ModbusRegisterBank` remains the default in-memory implementation, `set_service()` swaps in custom backends (live sensors, gateways, simulations).
- **`SharedModbusClient`**: cloneable, task-shareable client handle over one connection (async-mutex serialized); available via `into_shared()` on every client type.
- **`tls` feature**: Modbus/TCP Security client transport (`TlsTransport` + `ModbusTlsClient`) — MBAP over TLS on IANA port 802, via `tokio-rustls` (ring provider). Certificate policy is caller-supplied through `rustls::ClientConfig`.
- RTU server slave address is now configurable (`ModbusRtuServerConfig::slave_id`, previously hardcoded to `1`).

### Fixed
- **RTU server protocol compliance**: validates CRC-16 on incoming frames (previously executed corrupted writes), never answers broadcast frames, replies with Modbus exceptions on processing errors instead of leaving the master to time out, and no longer panics on sub-4-byte noise bursts.
- **RTU framing**: t3.5 inter-frame gap now follows MODBUS over Serial Line V1.02 (fixed 1750µs above 19200 baud, was under-computed at high baud rates); serial reads are now length-aware (derived from function code) instead of relying on inter-byte silence, so bursty USB-serial delivery no longer truncates frames; stale serial input is purged before each request; broadcast writes wait a turnaround delay before the next request.
- Fixed a latent panic in the TCP server's `peer_addr` fallback path.

### Changed
- Broadcast (slave_id = 0) requests are now validated against `is_write_function()` (pure writes only) rather than the inverse of `is_read_function()`, correctly rejecting broadcast for FC 0x17/0x2B which return data.

## [0.6.2] - 2026-05-15

### Added
- `ModbusRequest::new_write_multiple_coils` for FC0F requests where the final packed coil byte is only partially used.

### Fixed
- Generic clients now validate that responses match the request slave id, function code, read byte count, and write echo fields.
- Embedded RTU response reading now handles short Modbus exception responses instead of waiting for the normal success-frame length.
- TCP server request handlers now reject malformed PDUs with trailing bytes.
- `cargo test --no-default-features` now works by gating std-only integration/property tests and std-only doctest snippets.

## [0.6.1] - 2026-04-23

### Fixed
- `ReadCoalescer::coalesce` now handles windows that extend past `u16::MAX` without collapsing quantity to zero.
- `ModbusTcpServer` now returns complete Modbus TCP frames on successful responses, including MBAP header and unit id.
- `RtuOverTcpTransport` now validates raw requests and records bytes/errors/timeouts consistently.
- Server statistics now report accumulated counters from handled requests.
- Register bank and raw request validation now reject address ranges that overflow the 16-bit Modbus address space.

## [0.6.0] - 2026-04-18

### Added
- **Modbus ASCII client** — `ModbusAsciiClient` now exposed publicly (wraps the existing `AsciiTransport`). Gated on `rtu` feature.
- **RTU-over-TCP transport** — `RtuOverTcpTransport` + `ModbusRtuOverTcpClient` for industrial gateways that carry raw RTU frames over TCP (no serial dependencies). Available in the default feature set.
- **`embedded` feature** — `EmbeddedRtuTransport<RW>` over `embedded-io-async::{Read, Write}`, `no_std + alloc`, `heapless` TX buffer. Enables Modbus RTU on RP2040 / ESP32 / STM32 without any tokio or serial deps. Build with `cargo build --no-default-features --features embedded`.
- **`defmt` feature** — derives `defmt::Format` on `ModbusError`, `ModbusFunction`, `ModbusException` for RTT/USB logging on MCUs. Pairs with `embedded`.
- **`ScheduledRequest` trait** — minimal shared surface (`slave_id()` + `function_code()`) implemented by both `BatchCommand` and `ReadRequest`, for uniform routing/logging.
- **Criterion benchmarks** — `benches/throughput.rs` covers PDU builder, byte-order decode (all 4 variants × f32/u32/f64), and read coalescer.
- **cargo-fuzz targets** — `fuzz/fuzz_targets/` with 3 targets (TCP frame parser, RTU decode, PDU builder). 6.8M iterations, 0 crashes.
- **proptest properties** — `tests/proptest_roundtrips.rs` with 15 properties covering byte-order roundtrips, PDU builder invariants, and coalescer invariants.
- **docs.rs feature badges** — `doc_auto_cfg` auto-annotates every feature-gated item. `[package.metadata.docs.rs]` builds with `all-features`.

### Changed
- Transport-layer tracing calls restructured to use key-value fields (`protocol`, `slave_id`, `function_code`, `kind`) instead of interpolated strings — now filterable in `tracing-subscriber`.
- `CRC_MODBUS` constant moved out of the `rtu`-feature gate so both RTU and RTU-over-TCP transports share it.

### Fixed
- Cleaned all pre-existing clippy warnings (`approx_constant`, `bool_assert_comparison`, `byte_char_slices`, `manual_div_ceil`, `useless_vec`, `new_without_default`). `cargo clippy --all-features --all-targets -- -D warnings` now passes.
- Silenced self-referential `deprecated` warnings on `ModbusError` legacy variants (triggered by `defmt::Format` derive expansion). User-facing call-site deprecation warnings are preserved.

## [0.5.1] - 2026-03-26

### Fixed
- **DoS prevention**: Added stale-response counter (max 5) to TCP TID-mismatch loop — a misbehaving server can no longer cause infinite looping (`transport.rs`)
- **Panic safety**: Replaced `unwrap()` on `Option<TcpStream>` in pipeline send and regular request hot paths with `ok_or_else(...)? ` (`transport.rs`)
- **Protocol correctness**: Added overflow guard in `PduBuilder::build_write_multiple_registers` and `build_write_multiple_coils` — byte_count field no longer silently truncates (`pdu.rs`)
- **Protocol correctness**: `data.len() as u8` casts in transport encode functions replaced with checked `u8::try_from` (`transport.rs` TCP, RTU, ASCII paths)
- **no_std**: `bytes.rs` and `value.rs` now use `core::fmt` instead of `std::fmt`
- **Server module**: Fixed pre-existing `log` crate reference (changed to `tracing`), deprecated `ModbusError::InvalidFrame` usages, and `div_ceil` patterns in `server.rs`

### Added
- **Tests**: PDU boundary tests — overflow at 253 bytes, oversized `from_slice`, write-multiple-registers limit
- **Tests**: Pipeline out-of-order response test — verifies TID-based reordering when server replies in reverse order
- **Modules**: `server` and `register_bank` registered in `lib.rs` under `rtu` feature — tests now run with `cargo test --features rtu`
- **CI**: `no-std` job — checks core modules compile for `thumbv7em-none-eabihf`
- **CI**: `msrv` job — verifies compilation on Rust 1.85.0
- **CI**: `security-audit` job — `cargo audit` on every push
- **CI**: `coverage` job — LCOV report via `cargo-llvm-cov`, uploaded to Codecov
- **CI**: `check` job now also runs `cargo check --no-default-features`

### Changed
- `README.md` version examples updated from `0.4` to `0.5`

## [0.4.7] - 2026-01-06

### Changed
- **BREAKING**: `ModbusResponse.data` 从公开字段改为方法 `data() -> &[u8]`
  - 迁移: `response.data` → `response.data()`
- 优化响应解析的内存分配（零拷贝设计）
- 优化 hex 日志格式化，减少临时分配
- 优化 `Vec` 预分配策略，避免多次扩容
- 优化字符串归一化，单次迭代替代多次分配

### Fixed
- 移除 `server.rs` 中多余的变量遮蔽

## [0.4.6] - 2025-01-05

### Added
- RTU 完整配置示例（含校验位）

## [0.4.5] - 2025-01-04

### Changed
- 更新 MSRV 至 1.85.0
- 简化 README

## [0.4.4] - 2025-01-03

### Changed
- 使用 crates.io 的 igw 依赖
- 使用原生 AFIT 替代 async-trait（零成本异步 trait）

## [0.4.3] - 2024-11-29

### Added
- **Examples**: New example programs demonstrating real-world usage:
  - `tcp_client.rs` - Basic TCP client operations
  - `read_meter.rs` - Energy meter reading scenario
  - `batch_read.rs` - Batch reading with `DeviceLimits`
  - `data_types.rs` - Industrial data type handling
- **Documentation**: Enhanced API documentation with examples and protocol limits

### Changed
- Improved module-level documentation for `client.rs`, `value.rs`, and `bytes.rs`
- Examples are now included in the crate distribution

## [0.4.2] - 2024-11-29

### Changed
- **Dual-track API**: Function code names (`read_01`, `write_06`) as primary API, semantic names (`read_coils`, `write_single_register`) as aliases
- **Tightened API surface**: Internal utility functions hidden with `#[doc(hidden)]`
- **Re-exported tokio**: Users can use `voltage_modbus::tokio` directly

### Fixed
- **CI**: Fixed Windows RTU compilation issue (tokio-serial `Sync` trait)
- **CI**: Upgraded GitHub Actions to latest versions (checkout@v4, cache@v4, etc.)

## [0.4.1] - 2024-11-27

### Added
- Initial release to crates.io

## [0.4.0] - 2024-11-27

### Added
- **Industrial Data Types**: New `ModbusValue` enum supporting U16/I16/U32/I32/F32/F64/Bool
- **Byte Order Support**: `ByteOrder` with BigEndian/LittleEndian/BigEndianSwap/LittleEndianSwap
- **ModbusCodec**: Unified encoding/decoding for all industrial data types
- **CommandBatcher**: Write command batching for optimized communication
- **DeviceLimits**: Configurable protocol limits per device
- **Stack-allocated PDU**: Fixed 253-byte array with zero heap allocation
- **CallbackLogger**: Flexible logging system with callback support
- **PerformanceMetrics**: Built-in performance monitoring

### Changed
- Simplified API using function code naming (`read_03`, `write_06`, etc.)
- Generic client architecture for code reuse between TCP and RTU

### Removed
- Server functionality (planned for future release)

## [0.2.0] - 2024-06-04

### Added
- **Modbus ASCII Transport Support** - Complete implementation of Modbus ASCII protocol
  - `AsciiTransport` class with full ASCII frame encoding/decoding
  - LRC (Longitudinal Redundancy Check) error detection
  - Human-readable frame format for debugging and legacy system integration
  - Configurable serial parameters (7/8 data bits, parity, stop bits)
  - Inter-character timeout handling for ASCII frame reception
  - Comprehensive ASCII frame validation and error handling

### Features
- **ASCII Protocol Implementation**
  - ASCII hex encoding/decoding utilities
  - LRC checksum calculation and verification
  - CR/LF frame termination handling
  - Support for all standard Modbus functions in ASCII format
  - Exception response handling in ASCII format

- **Development Tools**
  - `ascii_test` binary for testing and demonstration
  - ASCII frame logger for debugging purposes
  - Example ASCII frames for educational use
  - Complete test suite for ASCII functionality

### Use Cases
- **Debugging**: Human-readable format for protocol troubleshooting
- **Legacy Systems**: Integration with older SCADA systems that only support ASCII
- **Educational**: Learning Modbus protocol structure with readable format
- **Manual Testing**: Ability to type commands manually in serial terminals

### Updated
- Library documentation to include ASCII transport
- Export statements to include `AsciiTransport`
- Main library description to mention TCP/RTU/ASCII support
- Comprehensive test coverage for ASCII functionality

### Technical Details
- ASCII frames use ':' start character and CR/LF termination
- LRC calculated as two's complement of sum of data bytes
- Default configuration: 7 data bits, even parity, 1 stop bit
- Configurable timeouts for overall operation and inter-character delays
- Full compatibility with existing `ModbusTransport` trait

## [0.1.0] - 2024-06-03

### Added
- Initial release of Voltage Modbus library
- **Modbus TCP Transport** - Complete TCP implementation with MBAP header handling
- **Modbus RTU Transport** - Full RTU implementation with CRC-16 validation
- **Protocol Layer** - Support for all standard Modbus function codes (0x01-0x10)
- **Client/Server Architecture** - Async client and server implementations
- **Register Bank** - Thread-safe register storage for server applications
- **Error Handling** - Comprehensive error types and recovery mechanisms
- **Performance Monitoring** - Built-in statistics and metrics
- **Testing Framework** - Complete test suite and example applications

### Features
- Async/await support with Tokio
- Zero-copy operations where possible
- Thread-safe design for concurrent usage
- Configurable timeouts and retry mechanisms
- Comprehensive logging and debugging support
- Production-ready reliability and performance

### Function Codes Supported

- 0x01: Read Coils
- 0x02: Read Discrete Inputs
- 0x03: Read Holding Registers
- 0x04: Read Input Registers
- 0x05: Write Single Coil
- 0x06: Write Single Register
- 0x0F: Write Multiple Coils
- 0x10: Write Multiple Registers

### Documentation

- Complete API reference with examples
- Architecture documentation and diagrams
- Performance benchmarks and optimization guide
- GitHub Pages deployment for live documentation

### Author

- Evan Liu <evan.liu@voltageenergy.com>
