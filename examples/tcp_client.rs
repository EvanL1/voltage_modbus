//! Basic TCP Client Example
//!
//! This example demonstrates how to connect to a Modbus TCP server
//! and perform basic read/write operations.
//!
//! # Running this example
//!
//! ```bash
//! cargo run --example tcp_client
//! ```
//!
//! Note: This requires a Modbus TCP server running on 127.0.0.1:502.
//! You can use any Modbus simulator for testing.

use std::time::Duration;
use voltage_modbus::{ModbusClient, ModbusResult, ModbusTcpClient};

#[tokio::main]
async fn main() -> ModbusResult<()> {
    // Connect to Modbus TCP server
    let mut client = ModbusTcpClient::from_address("127.0.0.1:502", Duration::from_secs(5)).await?;

    println!("Connected to Modbus server");

    let slave_id = 1;

    // =========================================================================
    // Reading Operations
    // =========================================================================

    // Read holding registers using function code name (FC03)
    // This is the primary API style
    let registers = client.read_03(slave_id, 0, 10).await?;
    println!("Registers 0-9: {:?}", registers);

    // Or use semantic name (alias) - same functionality
    let _registers = client.read_holding_registers(slave_id, 0, 10).await?;

    // Read input registers (FC04)
    let input_regs = client.read_04(slave_id, 0, 5).await?;
    println!("Input registers 0-4: {:?}", input_regs);

    // Read coils (FC01)
    let coils = client.read_01(slave_id, 0, 8).await?;
    println!("Coils 0-7: {:?}", coils);

    // Read discrete inputs (FC02)
    let discrete = client.read_02(slave_id, 0, 8).await?;
    println!("Discrete inputs 0-7: {:?}", discrete);

    // =========================================================================
    // Writing Operations
    // =========================================================================

    // Write single register (FC06)
    client.write_06(slave_id, 100, 0x1234).await?;
    println!("Wrote 0x1234 to register 100");

    // Write single coil (FC05)
    client.write_05(slave_id, 0, true).await?;
    println!("Set coil 0 to ON");

    // Write multiple registers (FC16)
    client
        .write_10(slave_id, 200, &[0x1111, 0x2222, 0x3333])
        .await?;
    println!("Wrote 3 registers starting at address 200");

    // Write multiple coils (FC15)
    client
        .write_0f(slave_id, 10, &[true, false, true, true])
        .await?;
    println!("Wrote 4 coils starting at address 10");

    // =========================================================================
    // Statistics
    // =========================================================================

    // Get transport statistics
    let stats = client.get_stats();
    println!(
        "\nTransport Statistics:\n  Requests sent: {}\n  Responses received: {}",
        stats.requests_sent, stats.responses_received
    );

    // Clean up
    client.close().await?;
    println!("\nConnection closed");

    Ok(())
}
