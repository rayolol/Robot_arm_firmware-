// drivers/as5600/driver.rs
// AS5600 magnetic rotary encoder driver
use crate::config::{AS5600_I2C_ADDR, ENCODER_RESOLUTION, ENCODER_TO_DEGREES};
// AS5600 REGISTER MAP

// global cached selected channel: -1 = none

// ============================================================================

#[allow(dead_code)]
pub mod register {
    /// Raw angle (12-bit)
    pub const RAW_ANGLE: u8 = 0x0C;
    
    /// Filtered angle (12-bit)
    pub const ANGLE: u8 = 0x0E;
    
    /// AGC (Automatic Gain Control)
    pub const AGC: u8 = 0x1A;
    
    /// Magnitude (CORDIC magnitude)
    pub const MAGNITUDE: u8 = 0x1B;
    
    /// Status register
    pub const STATUS: u8 = 0x0B;
    
    /// Configuration registers
    pub const CONF_LOW: u8 = 0x07;
    pub const CONF_HIGH: u8 = 0x08;
}

// ============================================================================
// STATUS BITS
// ============================================================================

const STATUS_MAGNET_DETECTED: u8 = 1 << 5;
const STATUS_MAGNET_TOO_WEAK: u8 = 1 << 4;
const STATUS_MAGNET_TOO_STRONG: u8 = 1 << 3;

// ============================================================================
// ERROR TYPES
// ============================================================================

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum As5600Error {
    I2cError,
    MagnetNotDetected,
    MagnetTooWeak,
    MagnetTooStrong,
    InvalidData,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MagnetStatus {
    Good,
    NotDetected,
    TooWeak,
    TooStrong,
}

// ============================================================================
// AS5600 DRIVER
// ============================================================================

/// AS5600 magnetic rotary encoder driver
///
/// The AS5600 provides 12-bit absolute angle measurement.
/// Communication is via I2C, typically through a multiplexer
/// when using multiple encoders.
pub struct As5600<I2C> {
    /// Multiplexer channel this encoder is on
    mux_channel: u8,

    /// Last known good angle (for velocity calculation)
    last_angle: u16,

    /// Number of full rotations (for multi-turn tracking)
    rotations: i32,

    /// Marker for I2C type parameter
    _phantom: core::marker::PhantomData<I2C>,
}

impl<I2C> As5600<I2C>
where I2C: embedded_hal::i2c::I2c
{
    /// Create a new AS5600 encoder
    ///
    /// # Arguments
    /// * `mux_channel` - TCA9548A multiplexer channel (0-7)
    pub fn new(mux_channel: u8) -> Self {
        Self {
            mux_channel,
            last_angle: 0,
            rotations: 0,
            _phantom: core::marker::PhantomData,
        }
    }


    /// Get the multiplexer channel
    pub fn mux_channel(&self) -> u8 {
        self.mux_channel
    }        

    /// Read raw 12-bit angle (0-4095)
    ///
    /// This is the unfiltered angle value.
    /// Most applications should use `read_angle()` instead.
    pub fn read_raw_angle(&mut self, i2c: &mut I2C) -> Result<u16, As5600Error>
    {
        self.read_u16(i2c, register::RAW_ANGLE)
    }
    
    /// Read filtered 12-bit angle (0-4095)
    /// 
    /// This is the recommended angle reading for most applications.
    pub fn read_angle(&mut self, i2c: &mut I2C) -> Result<u16, As5600Error>
    {   
        let angle = self.read_u16(i2c, register::ANGLE)?;
        
        // Track rotations for multi-turn applications
        self.update_rotations(angle);
        
        Ok(angle)
    }
    
    /// Read angle in degrees (0.0 - 359.9°)
    pub fn read_angle_degrees(&mut self, i2c: &mut I2C) -> Result<f32, As5600Error>
    {
        let angle = self.read_angle(i2c)?;
        Ok(angle as f32 * ENCODER_TO_DEGREES)
    }
    
    /// Read angle in radians (0.0 - 2π)
    pub fn read_angle_radians(&mut self, i2c: &mut I2C) -> Result<f32, As5600Error>
    {
        let degrees = self.read_angle_degrees(i2c)?;
        Ok(degrees * core::f32::consts::PI / 180.0)
    }
    
    /// Get total rotations including multi-turn tracking
    pub fn total_angle_degrees(&self) -> f32 {
        (self.rotations as f32 * 360.0) + (self.last_angle as f32 * ENCODER_TO_DEGREES)
    }
    
    /// Read magnet status
    pub fn read_magnet_status(&mut self, i2c: &mut I2C) -> Result<MagnetStatus, As5600Error>
    {
        let status = self.read_u8(i2c, register::STATUS)?;
        
        if status & STATUS_MAGNET_TOO_STRONG != 0 {
            Ok(MagnetStatus::TooStrong)
        } else if status & STATUS_MAGNET_TOO_WEAK != 0 {
            Ok(MagnetStatus::TooWeak)
        } else if status & STATUS_MAGNET_DETECTED == 0 {
            Ok(MagnetStatus::NotDetected)
        } else {
            Ok(MagnetStatus::Good)
        }
    }
    
    /// Check if magnet is detected and in good range
    pub fn is_magnet_ok(&mut self, i2c: &mut I2C) -> bool
    {
        matches!(self.read_magnet_status(i2c), Ok(MagnetStatus::Good) | Ok(MagnetStatus::TooWeak) | Ok(MagnetStatus::TooStrong))
    }
    
    /// Read AGC (Automatic Gain Control) value
    /// 
    /// AGC value can be used for diagnostics.
    /// Typical good values are 64-128.
    pub fn read_agc(&mut self, i2c: &mut I2C) -> Result<u8, As5600Error>
    {
        self.read_u8(i2c, register::AGC)
    }
    
    /// Read magnitude value
    /// 
    /// Magnitude indicates the strength of the magnetic field.
    /// Values should typically be in range 1000-3000.
    pub fn read_magnitude(&mut self, i2c: &mut I2C) -> Result<u16, As5600Error>
    {
        self.read_u16(i2c, register::MAGNITUDE)
    }
    
    /// Reset rotation counter
    pub fn reset_rotations(&mut self) {
        self.rotations = 0;
    }
    
    /// Get current rotation count
    pub fn rotations(&self) -> i32 {
        self.rotations
    }
    
    // ========================================================================
    // PRIVATE METHODS
    // ========================================================================
    
    /// Update rotation counter based on angle wraparound
    fn update_rotations(&mut self, new_angle: u16) {
        let diff = new_angle as i32 - self.last_angle as i32;
        
        // Detect wraparound
        if diff < -(ENCODER_RESOLUTION as i32 / 2) {
            // Wrapped forward (0 -> 4095)
            self.rotations += 1;
        } else if diff > (ENCODER_RESOLUTION as i32 / 2) {
            // Wrapped backward (4095 -> 0)
            self.rotations -= 1;
        }
        
        self.last_angle = new_angle;
    }
    
    /// Read 8-bit register
    fn read_u8(&mut self, i2c: &mut I2C, reg: u8) -> Result<u8, As5600Error>
    {
        let mut buffer = [0u8; 1];
        i2c.write_read(AS5600_I2C_ADDR, &[reg], &mut buffer)
            .map_err(|_| As5600Error::I2cError)?;
        Ok(buffer[0])
    }
    
    /// Read 16-bit register (big-endian)
    fn read_u16(&mut self, i2c: &mut I2C, reg: u8) -> Result<u16, As5600Error>
    {

        //TODO: add delays between mux selection and read if needed
        let mut buffer = [0u8; 2];
        i2c.write_read(AS5600_I2C_ADDR, &[reg], &mut buffer)
            .map_err(|_| As5600Error::I2cError)?;
        
        // AS5600 sends MSB first
        let value = ((buffer[0] as u16) << 8) | (buffer[1] as u16);
        
        // Mask to 12 bits
        Ok(value & 0x0FFF)
    }
}

// ============================================================================
// TESTS
// ============================================================================