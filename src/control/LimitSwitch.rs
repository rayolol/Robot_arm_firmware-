pub struct LimitSwitch<I2C, const N: usize> {
    pcf: crate::drivers::PCF::pcf8574<I2C>,
    switches_states: [(u8, bool); N],
}

impl<I2C, const N: usize> LimitSwitch<I2C, N>
where
    I2C: embedded_hal::i2c::I2c,
{
    pub fn new(pcf: crate::drivers::PCF::pcf8574<I2C>) -> Self {
        Self {
            pcf,
            switches_states: [(0, false); N],
        }
    }

    pub fn read_switches(&mut self, i2c: &mut I2C) -> Result<[(u8, bool); N], ()> {
        let pins_state = self.pcf.read_pins(i2c)?;

        for (index, slot) in self.switches_states.iter_mut().enumerate() {
            let state = (pins_state & (1 << index)) != 0;
            *slot = (index as u8, state);
        }

        Ok(self.switches_states)
    }
}
