//! Energy Meter Reading Example
//!
//! Demonstrates how to read voltage, current, and power data
//! from a smart energy meter using Modbus TCP.
//!
//! This is a common real-world scenario in industrial monitoring.
//!
//! # Register Map (example for a typical power meter)
//!
//! | Address | Data Type | Description |
//! |---------|-----------|-------------|
//! | 0x0000  | Float32   | Voltage (V) |
//! | 0x0002  | Float32   | Current (A) |
//! | 0x0004  | Float32   | Active Power (W) |
//! | 0x0006  | Float32   | Power Factor |
//! | 0x0008  | Float32   | Frequency (Hz) |
//! | 0x000A  | Float32   | Total Energy (kWh) |
//!
//! # Running this example
//!
//! ```bash
//! cargo run --example read_meter
//! ```

use std::time::Duration;
use voltage_modbus::{regs_to_f32, ByteOrder, ModbusClient, ModbusResult, ModbusTcpClient};

/// Meter configuration
struct MeterConfig {
    /// IP address and port
    address: &'static str,
    /// Modbus slave ID
    slave_id: u8,
    /// Byte order used by the meter
    byte_order: ByteOrder,
}

/// Meter reading result
#[derive(Debug)]
struct MeterReading {
    voltage: f32,
    current: f32,
    active_power: f32,
    power_factor: f32,
    frequency: f32,
    total_energy: f32,
}

/// Read a Float32 value from two consecutive registers
fn read_f32(registers: &[u16], byte_order: ByteOrder) -> f32 {
    if registers.len() >= 2 {
        regs_to_f32(&[registers[0], registers[1]], byte_order)
    } else {
        0.0
    }
}

/// Read all meter values in a single request (optimized)
async fn read_meter_bulk(
    client: &mut ModbusTcpClient,
    config: &MeterConfig,
) -> ModbusResult<MeterReading> {
    // Read all 12 registers (6 float32 values) in one request
    let registers = client.read_03(config.slave_id, 0x0000, 12).await?;

    Ok(MeterReading {
        voltage: read_f32(&registers[0..2], config.byte_order),
        current: read_f32(&registers[2..4], config.byte_order),
        active_power: read_f32(&registers[4..6], config.byte_order),
        power_factor: read_f32(&registers[6..8], config.byte_order),
        frequency: read_f32(&registers[8..10], config.byte_order),
        total_energy: read_f32(&registers[10..12], config.byte_order),
    })
}

/// Read meter values one by one (for demonstration)
async fn read_meter_individual(
    client: &mut ModbusTcpClient,
    config: &MeterConfig,
) -> ModbusResult<MeterReading> {
    // Read voltage (address 0x0000, 2 registers)
    let regs = client.read_03(config.slave_id, 0x0000, 2).await?;
    let voltage = read_f32(&regs, config.byte_order);

    // Read current (address 0x0002, 2 registers)
    let regs = client.read_03(config.slave_id, 0x0002, 2).await?;
    let current = read_f32(&regs, config.byte_order);

    // Read active power (address 0x0004, 2 registers)
    let regs = client.read_03(config.slave_id, 0x0004, 2).await?;
    let active_power = read_f32(&regs, config.byte_order);

    // Read power factor (address 0x0006, 2 registers)
    let regs = client.read_03(config.slave_id, 0x0006, 2).await?;
    let power_factor = read_f32(&regs, config.byte_order);

    // Read frequency (address 0x0008, 2 registers)
    let regs = client.read_03(config.slave_id, 0x0008, 2).await?;
    let frequency = read_f32(&regs, config.byte_order);

    // Read total energy (address 0x000A, 2 registers)
    let regs = client.read_03(config.slave_id, 0x000A, 2).await?;
    let total_energy = read_f32(&regs, config.byte_order);

    Ok(MeterReading {
        voltage,
        current,
        active_power,
        power_factor,
        frequency,
        total_energy,
    })
}

/// Display meter reading
fn display_reading(reading: &MeterReading) {
    println!("┌────────────────────────────────────┐");
    println!("│       Power Meter Reading          │");
    println!("├────────────────────────────────────┤");
    println!("│  Voltage:      {:>10.2} V       │", reading.voltage);
    println!("│  Current:      {:>10.3} A       │", reading.current);
    println!("│  Active Power: {:>10.1} W       │", reading.active_power);
    println!("│  Power Factor: {:>10.3}         │", reading.power_factor);
    println!("│  Frequency:    {:>10.2} Hz      │", reading.frequency);
    println!("│  Total Energy: {:>10.2} kWh     │", reading.total_energy);
    println!("└────────────────────────────────────┘");
}

#[tokio::main]
async fn main() -> ModbusResult<()> {
    // Configure your meter here
    let config = MeterConfig {
        address: "127.0.0.1:502", // Change to your meter's IP
        slave_id: 1,
        byte_order: ByteOrder::BigEndian, // Most common for power meters
    };

    println!("Connecting to meter at {}...", config.address);

    let mut client = ModbusTcpClient::from_address(config.address, Duration::from_secs(5)).await?;

    println!("Connected!\n");

    // Method 1: Bulk read (recommended - more efficient)
    println!("=== Bulk Read (Optimized) ===\n");
    let reading = read_meter_bulk(&mut client, &config).await?;
    display_reading(&reading);

    println!("\n=== Individual Reads (Demonstration) ===\n");
    // Method 2: Individual reads (for demonstration)
    let reading = read_meter_individual(&mut client, &config).await?;
    display_reading(&reading);

    // Show statistics
    let stats = client.get_stats();
    println!(
        "\nCommunication stats: {} requests, {} responses",
        stats.requests_sent, stats.responses_received
    );

    client.close().await?;
    Ok(())
}
