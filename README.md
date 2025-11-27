# Voltage Modbus

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Rust](https://img.shields.io/badge/rust-1.70+-blue.svg)](https://www.rust-lang.org)
[![GitHub](https://img.shields.io/badge/github-voltage_modbus-blue.svg)](https://github.com/EvanL1/voltage_modbus)
[![Crates.io](https://img.shields.io/crates/v/voltage_modbus.svg)](https://crates.io/crates/voltage_modbus)
[![docs.rs](https://docs.rs/voltage_modbus/badge.svg)](https://docs.rs/voltage_modbus)

> **High-Performance Modbus TCP/RTU/ASCII Library for Rust**
>
> **Author:** Evan Liu <liuyifanz.1996@gmail.com>
> **Version:** 0.4.0
> **License:** MIT

A comprehensive, high-performance Modbus TCP/RTU/ASCII implementation in pure Rust designed for industrial automation, IoT applications, and smart grid systems.

## ✨ Features

- **🚀 High Performance**: Async/await support with Tokio for maximum throughput
- **🔧 Complete Protocol Support**: Modbus TCP, RTU, and ASCII protocols
- **🛡️ Memory Safe**: Pure Rust implementation with zero unsafe code
- **⚡ Zero-Copy Operations**: Optimized for minimal memory allocations
- **🔄 Concurrent Processing**: Multi-client server support
- **📊 Built-in Monitoring**: Comprehensive statistics and metrics
- **🏭 Production Ready**: Extensive testing and error handling
- **🎯 Smart Architecture**: Generic client design eliminates code duplication
- **🧩 Modular Design**: Clean separation of transport and application layers

## 📋 Supported Function Codes

| Code | Function                 | Client | Server |
| ---- | ------------------------ | ------ | ------ |
| 0x01 | Read Coils               | ✅     | 🚧     |
| 0x02 | Read Discrete Inputs     | ✅     | 🚧     |
| 0x03 | Read Holding Registers   | ✅     | 🚧     |
| 0x04 | Read Input Registers     | ✅     | 🚧     |
| 0x05 | Write Single Coil        | ✅     | 🚧     |
| 0x06 | Write Single Register    | ✅     | 🚧     |
| 0x0F | Write Multiple Coils     | ✅     | 🚧     |
| 0x10 | Write Multiple Registers | ✅     | 🚧     |

> 🚧 Server functionality is planned for a future release.

## 🚀 Quick Start

Add this to your `Cargo.toml`:

```toml
[dependencies]
voltage_modbus = "0.4.0"
tokio = { version = "1.0", features = ["full"] }
```

### Client Examples

#### TCP Client

```rust
use voltage_modbus::{ModbusTcpClient, ModbusClient, ModbusResult};
use std::time::Duration;

#[tokio::main]
async fn main() -> ModbusResult<()> {
    // Connect to Modbus TCP server
    let mut client = ModbusTcpClient::with_timeout("127.0.0.1:502", Duration::from_secs(5)).await?;
    
    // Read holding registers
    let values = client.read_holding_registers(1, 0, 10).await?;
    println!("Read registers: {:?}", values);
    
    // Write single register
    client.write_single_register(1, 100, 0x1234).await?;
    
    client.close().await?;
    Ok(())
}
```

#### RTU Client

```rust
use voltage_modbus::{ModbusRtuClient, ModbusClient, ModbusResult};
use std::time::Duration;

#[tokio::main] 
async fn main() -> ModbusResult<()> {
    // Connect to Modbus RTU device
    let mut client = ModbusRtuClient::with_config(
        "/dev/ttyUSB0",
        9600,
        tokio_serial::DataBits::Eight,
        tokio_serial::StopBits::One,
        tokio_serial::Parity::None,
        Duration::from_secs(1),
    )?;
    
    // Read coils
    let coils = client.read_coils(1, 0, 8).await?;
    println!("Read coils: {:?}", coils);
    
    client.close().await?;
    Ok(())
}
```

### Server Example

> ⚠️ **Note**: Server functionality is not included in this release. The example below is for reference only and will be available in a future version.

```rust
use voltage_modbus::{
    ModbusTcpServer, ModbusTcpServerConfig, ModbusServer, ModbusRegisterBank
};
use std::sync::Arc;
use std::time::Duration;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Create server configuration
    let config = ModbusTcpServerConfig {
        bind_address: "127.0.0.1:502".parse().unwrap(),
        max_connections: 50,
        request_timeout: Duration::from_secs(30),
        register_bank: Some(Arc::new(ModbusRegisterBank::new())),
    };

    // Start server
    let mut server = ModbusTcpServer::with_config(config)?;
    server.start().await?;

    // Server is now running...
    Ok(())
}
```

## 📖 Documentation

- **[API Reference](https://docs.rs/voltage_modbus)** - Complete API documentation
- **[Crates.io](https://crates.io/crates/voltage_modbus)** - Package information
- **[GitHub Repository](https://github.com/EvanL1/voltage_modbus)** - Source code and issues

## 🏗️ Architecture

### Protocol Layer Insight

The library implements a key architectural insight: **Modbus TCP and RTU share identical application layer messages (PDU)**, differing only in transport encapsulation:

```text
TCP Frame: [MBAP Header (7 bytes)] + [PDU (Function Code + Data)]
RTU Frame: [Slave ID (1 byte)] + [PDU (Function Code + Data)] + [CRC (2 bytes)]
```

This enables code reuse through a generic client design:

```text
┌─────────────────────────────────────────────────────────────┐
│                    Application Layer                        │
│  ┌─────────────────┐    ┌─────────────────┐                │
│  │ ModbusTcpClient │    │ ModbusRtuClient │                │
│  └─────────────────┘    └─────────────────┘                │
│           │                       │                        │
│  ┌─────────────────────────────────────────────────────────┐│
│  │           GenericModbusClient<T>                        ││
│  │         (Shared Application Logic)                      ││
│  └─────────────────────────────────────────────────────────┘│
└─────────────────────────────────────────────────────────────┘
                          │
┌─────────────────────────────────────────────────────────────┐
│                   Transport Layer                           │
│  ┌─────────────────┐    ┌─────────────────┐                │
│  │   TcpTransport  │    │   RtuTransport  │                │
│  │  (TCP Sockets)  │    │ (Serial Ports)  │                │
│  └─────────────────┘    └─────────────────┘                │
└─────────────────────────────────────────────────────────────┘
```

### Core Modules

- **`error`** - Error types and result handling
- **`protocol`** - Modbus protocol definitions and message handling  
- **`transport`** - Network transport layer for TCP, RTU, and ASCII communication
- **`client`** - Generic and protocol-specific client implementations
- **`server`** - Modbus server implementations with concurrent support
- **`register_bank`** - Thread-safe register storage for server applications
- **`utils`** - Utility functions, data conversion, and performance monitoring

## 🧪 Examples and Testing

### Run Examples

```bash
# Start the demo server
cargo run --bin server_demo

# Run TCP client demo
cargo run --bin demo

# Test RTU functionality
cargo run --bin rtu_test

# Run performance benchmarks
cargo run --bin performance_test

# Test all function codes
cargo run --bin full_function_test
```

### Test Coverage

```bash
# Run all tests
cargo test

# Run with output
cargo test -- --nocapture

# Integration tests
cargo test --test integration_tests

# Documentation tests
cargo test --doc
```

**Test Results:**
- ✅ 34 unit tests passed
- ✅ 9 integration tests passed  
- ✅ 22 documentation tests passed
- ✅ All Modbus function codes tested
- ✅ Error handling and recovery tested
- ✅ Concurrent client connections tested

## 📈 Performance

### Benchmarks

| Metric                     | Value              |
| -------------------------- | ------------------ |
| **Latency**                | < 1ms (local)      |
| **Throughput**             | 1000+ requests/sec |
| **Concurrent Connections** | 50+ clients        |
| **Memory Usage**           | < 10MB (baseline)  |
| **CPU Usage**              | < 5% (idle)        |

### Optimization Features

- **Async I/O**: Non-blocking operations with Tokio
- **Zero-Copy Operations**: Minimal memory allocations
- **Generic Architecture**: Code reuse eliminates duplication
- **Lock-Free Operations**: Where possible
- **Configurable Timeouts**: Adaptive timeout management

## 🔧 Configuration

### Advanced Client Configuration

```rust
use voltage_modbus::{ModbusTcpClient, ModbusRtuClient};
use std::time::Duration;

// TCP with custom timeout
let mut tcp_client = ModbusTcpClient::with_timeout(
    "192.168.1.100:502", 
    Duration::from_secs(10)
).await?;

// RTU with full configuration
let mut rtu_client = ModbusRtuClient::with_config(
    "/dev/ttyUSB0",
    9600,                                // Baud rate
    tokio_serial::DataBits::Eight,      // Data bits
    tokio_serial::StopBits::One,        // Stop bits  
    tokio_serial::Parity::None,         // Parity
    Duration::from_secs(1),             // Timeout
)?;
```

### Server Configuration

> ⚠️ **Note**: Server functionality is planned for a future release.

```rust
use voltage_modbus::{ModbusTcpServerConfig, ModbusRegisterBank};
use std::sync::Arc;

let config = ModbusTcpServerConfig {
    bind_address: "0.0.0.0:502".parse().unwrap(),
    max_connections: 100,
    request_timeout: Duration::from_secs(30),
    register_bank: Some(Arc::new(ModbusRegisterBank::with_sizes(
        10000, // coils
        10000, // discrete_inputs
        10000, // holding_registers
        10000, // input_registers
    ))),
};
```

## 🛠️ Development

### Building from Source

```bash
git clone https://github.com/EvanL1/voltage_modbus.git
cd voltage-modbus
cargo build --release
```

### Development Tools

```bash
# Check code
cargo check

# Format code
cargo fmt

# Run linter
cargo clippy

# Generate documentation
cargo doc --no-deps --open
```

## 🚀 Installation

### From Crates.io

```bash
cargo add voltage_modbus
```

### From Source

```bash
git clone https://github.com/EvanL1/voltage_modbus.git
cd voltage-modbus
cargo install --path .
```

## 🤝 Contributing

We welcome contributions! Please see our [Contributing Guide](CONTRIBUTING.md) for details.

### Development Setup

1. **Clone the repository**
2. **Install Rust** (latest stable)
3. **Install dependencies**: `cargo build`
4. **Run tests**: `cargo test`
5. **Check formatting**: `cargo fmt --check`
6. **Run linter**: `cargo clippy`

## 📝 Changelog

See [CHANGELOG.md](CHANGELOG.md) for detailed release notes.

### v0.4.0 - Industrial Enhancement

- 🏭 **Industrial Data Types**: New `ModbusValue` enum supporting U16/I16/U32/I32/F32/F64/Bool/String
- 🔄 **Byte Order Support**: `ByteOrder` with BigEndian/LittleEndian/BigEndianSwap/LittleEndianSwap
- 📦 **ModbusCodec**: Unified encoding/decoding for all industrial data types
- ⚡ **CommandBatcher**: Write command batching for optimized communication
- 🎛️ **DeviceLimits**: Configurable protocol limits per device
- 🧱 **Stack-allocated PDU**: Fixed 253-byte array with zero heap allocation
- 🎯 **Simplified API**: Function code naming (`read_03`, `write_06`, etc.)
- 📊 **CallbackLogger**: Flexible logging system with callback support
- 📈 **PerformanceMetrics**: Built-in performance monitoring
- 🚧 **Server**: Temporarily not included (planned for future release)

### v0.3.1

- 🐛 Bug fixes and stability improvements

### v0.2.0

- ✨ **Generic Client Architecture**: Eliminated code duplication between TCP/RTU clients
- 🎯 **Improved API**: Cleaner, more intuitive client interfaces
- 🔧 **Enhanced RTU Support**: Full RTU client implementation
- 📊 **Better Testing**: Comprehensive test coverage
- 🏗️ **Architectural Refinement**: Clean separation of transport and application layers

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- **Modbus Organization** for the protocol specification
- **Tokio Team** for the excellent async runtime
- **Rust Community** for the amazing ecosystem

## 📞 Support

- **Documentation**: https://docs.rs/voltage_modbus
- **Package**: https://crates.io/crates/voltage_modbus
- **Issues**: https://github.com/EvanL1/voltage_modbus/issues
- **Discussions**: https://github.com/EvanL1/voltage_modbus/discussions
- **Email**: liuyifanz.1996@gmail.com

---

**Built with ❤️ by Evan Liu for the Rust and Industrial Automation communities.**
