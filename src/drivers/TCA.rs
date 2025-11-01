
use crate::config::TCA9548A_I2C_ADDR;

use stm32f1xx_hal::gpio::{ErasedPin, Output, PushPull};


pub struct TCA9548A<I2C> {
    /// Marker for I2C type parameter
    _phantom: core::marker::PhantomData<I2C>,
    current_channel: u8,
    rst_mux: ErasedPin<Output<PushPull>>,
    
}


impl<I2C> TCA9548A<I2C>  
where I2C: embedded_hal::i2c::I2c
{
    /// Create a new TCA9548A driver
    pub fn new(rst_mux: ErasedPin<Output<PushPull>>) -> Self {
        Self {
            rst_mux,
            _phantom: core::marker::PhantomData,
            current_channel: 0xFF, // Invalid channel to force initial set
        }
    }

    pub fn select_channel(&mut self, i2c: &mut I2C, channel: u8, delay: fn()) -> Result<(), ()>
    {
        // Ensure TCA is not in reset (reset pin is active LOW, so HIGH = normal operation)
        if self.rst_mux.is_set_low() {
            self.rst_mux.set_high();
            delay(); // Give time for TCA to come out of reset
        }

        if channel > 7 {
            return Err(());
        }

        if self.current_channel == channel {
            return Ok(());
        }

        let data = 1 << channel;
        i2c.write(TCA9548A_I2C_ADDR, &[data]).map_err(|_| ())?;
        self.current_channel = channel;
        delay();

        Ok(())
    }

    pub fn reset(&mut self, delay: fn()) {
        // TCA9548A reset is active LOW
        // Pulse LOW to reset, then return HIGH for normal operation
        self.rst_mux.set_high(); // Ensure starting from HIGH
        delay();
        self.rst_mux.set_low();  // Assert reset (active LOW)
        delay();
        self.rst_mux.set_high(); // Release reset - device now operational
        delay();
    }
}