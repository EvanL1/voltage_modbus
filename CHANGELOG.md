# Changelog

All notable changes to Voltage Modbus library will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

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
