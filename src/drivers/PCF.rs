

use stm32f1xx_hal::gpio::{ErasedPin, Output, PushPull};

const PCF8574_I2C_ADDR: u8 = 0x20;
pub enum PinMode {
    Input = 0,
    Output = 1,
}
pub struct pcf8574<I2C> {
    _phantom: core::marker::PhantomData<I2C>,
    interrupt: ErasedPin<Output<PushPull>>,
    output_state: u8,
    address: u8,
}

impl <I2C> pcf8574<I2C> 
where I2C: embedded_hal::i2c::I2c
{
    pub fn new(interrupt: ErasedPin<Output<PushPull>>, address: u8, i2c: &mut I2C) -> Self {

        let mut dev = Self {
            _phantom: core::marker::PhantomData,
            interrupt,
            output_state: 0xFF, // set all pins high (inputs)
            address,
        };

        dev.write(i2c).ok();
        dev
    }

    pub fn write(&mut self, i2c: &mut I2C) -> Result<(), ()> {
        i2c.write(self.address, &[self.output_state]).map_err(|_| ())
    }


    pub fn set_output_pin(&mut self, pin: u8, mode: PinMode, i2c: &mut I2C) -> Result<(), ()> {

        if pin > 7 {
            return Err(());
        }
        match mode {
            PinMode::Input => self.output_state |= 1 << pin,
            PinMode::Output => self.output_state &= !(1 << pin),
        }
        self.write(i2c)
    }

    pub fn read_pins(&mut self, i2c: &mut I2C) -> Result<u8, ()> {
        let mut buffer = [0u8; 1];
        i2c.read(self.address, &mut buffer).map_err(|_| ())?;
        Ok(buffer[0])
    }

    pub fn read_pin(&mut self, pin: u8, i2c: &mut I2C) -> Result<bool, ()> {
        if pin > 7 {
            return Err(());
        }
        let pins = self.read_pins(i2c)?;
        Ok((pins & (1 << pin)) != 0)
    }
}



