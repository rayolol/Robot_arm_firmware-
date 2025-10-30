// utils.rs
use stm32f1xx_hal::{
    pac, 
    serial::{Tx, Rx},
    prelude::*,
};
use tmc2209::reg;
use nb::block;
use crate::Register;
use crate::HalfDuplexSerial;

/// Write to TMC2209 register
pub fn write_register(
    uart: &mut HalfDuplexSerial,
    motor_address: u8,
    register: Register,
    value: u32,
) -> bool {
    // Send the request immediately
    let success = match register {
        Register::GCONF => {
            let req = tmc2209::write_request::<reg::GCONF>(motor_address, reg::GCONF(value));
            uart.send_and_get(req.bytes())
        }
        Register::CHOPCONF => {
            let req = tmc2209::write_request::<reg::CHOPCONF>(motor_address, reg::CHOPCONF(value));
            uart.send_and_get(req.bytes())
        }
        Register::IHOLD_IRUN => {
            let req = tmc2209::write_request::<reg::IHOLD_IRUN>(motor_address, reg::IHOLD_IRUN(value));
            uart.send_and_get(req.bytes())
        }
        Register::GSTAT => {
            let req = tmc2209::write_request::<reg::GSTAT>(motor_address, reg::GSTAT(value));
            uart.send_and_get(req.bytes())
        }
        _ => return false,
    };

    if success.is_err() {
        return false;
    }

    // Wait for transmission complete
    cortex_m::asm::delay(10_800); // ~150µs at 72MHz
    true
}

/// Read from TMC2209 register
/// Read from TMC2209 register
pub fn read_register(
    uart: &mut HalfDuplexSerial,
    motor_address: u8,
    register: Register,
) -> (bool, u32) {

    // Send read request and get response
    let response_bytes = match register {
        Register::GSTAT => {
            let req = tmc2209::read_request::<reg::GSTAT>(motor_address);
            uart.send_and_get(req.bytes())
        }
        Register::DRV_STATUS => {
            let req = tmc2209::read_request::<reg::DRV_STATUS>(motor_address);
            uart.send_and_get(req.bytes())
        }
        Register::IFCNT => {
            let req = tmc2209::read_request::<reg::IFCNT>(motor_address);
            uart.send_and_get(req.bytes())
        }
        _ => return (false, 1),
    };

    // Check if send_and_get succeeded
    let bytes = match response_bytes {
        Ok(b) => b,
        Err(_) => return (false, 2), // Communication error
    };

    // Check if we got enough bytes (TMC2209 response is 8 bytes)
    if bytes.len() < 8 {
        return (false, 1);
    }
    
    // Wait for TMC to prepare response (may not be needed if send_and_get already waits)
    cortex_m::asm::delay(14_400); // ~200µs

    // Validate sync byte (should be 0x05 for TMC2209)
    if bytes[0] != 0x05 {
        return (false, 0); // Invalid sync byte
    }

    // Extract 32-bit value from bytes 3-6
    let value = ((bytes[3] as u32) << 24)
              | ((bytes[4] as u32) << 16)
              | ((bytes[5] as u32) << 8)
              | (bytes[6] as u32);

    (true, value)
}


pub fn configure_motor(
    uart: &mut HalfDuplexSerial,  // Changed from Tx<pac::USART3>
    motor_address: u8,
    current_ma: f32,
    microsteps: u8,
) {
    // Calculate current settings
    let (_vsense, cs) = tmc2209::rms_current_to_vsense_cs(
        0.11,      // Sense resistor (check your board!)
        current_ma,
    );

    // Configure IHOLD_IRUN
    let ihold_irun = (8 << 0) | ((cs as u32) << 8) | (5 << 16);
    write_register(uart, motor_address, Register::IHOLD_IRUN, ihold_irun);

    // Small delay between commands
    cortex_m::asm::delay(7_200); // 100µs

    // Configure CHOPCONF with microsteps
    let chopconf = (3 << 0) | (5 << 4) | ((microsteps as u32) << 24);
    write_register(uart, motor_address, Register::CHOPCONF, chopconf);
}

