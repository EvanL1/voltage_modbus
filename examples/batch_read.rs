//! Batch Reading Example
//!
//! Demonstrates how to read large ranges of registers efficiently
//! by splitting requests according to device limits.
//!
//! # Why Batch Reading?
//!
//! The Modbus specification limits the number of registers per read request:
//! - **FC03/FC04**: Max 125 registers per request
//! - **FC01/FC02**: Max 2000 bits per request
//!
//! Some devices have even lower limits. This example shows how to:
//! 1. Configure device-specific limits using `DeviceLimits`
//! 2. Split large read requests into smaller batches
//! 3. Handle inter-request delays for slow devices
//!
//! # Running this example
//!
//! ```bash
//! cargo run --example batch_read
//! ```
//!
//! # New Batch API
//!
//! The `ModbusClient` trait now provides built-in batch read methods:
//! - `read_01_batch` / `read_coils_batch`
//! - `read_02_batch` / `read_discrete_inputs_batch`
//! - `read_03_batch` / `read_holding_registers_batch`
//! - `read_04_batch` / `read_input_registers_batch`
//!
//! These methods automatically handle chunking based on `DeviceLimits`.

use std::time::Duration;
use voltage_modbus::{
    regs_to_f32, ByteOrder, DeviceLimits, ModbusClient, ModbusResult, ModbusTcpClient,
};

#[tokio::main]
async fn main() -> ModbusResult<()> {
    let mut client = ModbusTcpClient::from_address("127.0.0.1:502", Duration::from_secs(5)).await?;

    let slave_id = 1;

    // =========================================================================
    // Example 1: Default limits (Modbus specification)
    // =========================================================================
    println!("=== Example 1: Default Limits (125 registers/request) ===\n");

    let default_limits = DeviceLimits::new();
    println!(
        "DeviceLimits: max_read_registers={}, delay={}ms",
        default_limits.max_read_registers, default_limits.inter_request_delay_ms
    );

    // Using the new trait method: read_holding_registers_batch
    let registers = client
        .read_holding_registers_batch(slave_id, 0, 200, &default_limits)
        .await?;
    println!("Read {} registers total\n", registers.len());

    // =========================================================================
    // Example 2: Conservative limits (for older/slower devices)
    // =========================================================================
    println!("=== Example 2: Conservative Limits (50 registers/request) ===\n");

    let conservative_limits = DeviceLimits::conservative();
    println!(
        "DeviceLimits: max_read_registers={}, delay={}ms",
        conservative_limits.max_read_registers, conservative_limits.inter_request_delay_ms
    );

    let registers = client
        .read_holding_registers_batch(slave_id, 0, 200, &conservative_limits)
        .await?;
    println!("Read {} registers total\n", registers.len());

    // =========================================================================
    // Example 3: Custom limits for specific device
    // =========================================================================
    println!("=== Example 3: Custom Limits (30 registers/request, 5ms delay) ===\n");

    let custom_limits = DeviceLimits::new()
        .with_max_read_registers(30) // Device only supports 30 registers per read
        .with_inter_request_delay_ms(5); // 5ms delay between requests

    println!(
        "DeviceLimits: max_read_registers={}, delay={}ms",
        custom_limits.max_read_registers, custom_limits.inter_request_delay_ms
    );

    let registers = client
        .read_03_batch(slave_id, 0, 100, &custom_limits)
        .await?;
    println!("Read {} registers total\n", registers.len());

    // =========================================================================
    // Example 4: Batch read with Float32 decoding
    // =========================================================================
    println!("=== Example 4: Batch Read with Float Decoding ===\n");

    // Read 10 float values (20 registers) starting at address 1000
    let registers = client
        .read_03_batch(slave_id, 1000, 20, &default_limits)
        .await?;

    println!("Decoded Float32 values (BigEndian):");
    for i in 0..10 {
        let idx = i * 2;
        if idx + 1 < registers.len() {
            let value = regs_to_f32(&[registers[idx], registers[idx + 1]], ByteOrder::BigEndian);
            println!("  Float[{}] @ address {}: {:.4}", i, 1000 + idx, value);
        }
    }

    // =========================================================================
    // Example 5: Batch read coils
    // =========================================================================
    println!("\n=== Example 5: Batch Read Coils ===\n");

    let coil_limits = DeviceLimits::new().with_max_read_coils(500);

    println!(
        "Reading 1000 coils with max {} per request",
        coil_limits.max_read_coils
    );
    // Using the new trait method: read_coils_batch (alias for read_01_batch)
    let coils = client
        .read_coils_batch(slave_id, 0, 1000, &coil_limits)
        .await?;
    println!("Read {} coils total", coils.len());
    println!("First 16 coils: {:?}", &coils[..coils.len().min(16)]);

    // =========================================================================
    // Statistics
    // =========================================================================
    let stats = client.get_stats();
    println!(
        "\nTotal requests: {}, responses: {}",
        stats.requests_sent, stats.responses_received
    );

    client.close().await?;
    Ok(())
}
