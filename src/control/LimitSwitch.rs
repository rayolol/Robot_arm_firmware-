

struct LimitSwitch {
    switches_num: usize,
    pcf: pcf8574<I2C>,
    switches_states: [(u8, bool); switches_num],
}


impl LimitSwitch {
    pub fn new(pcf: pcf8574<I2C>, switches_num: usize) -> Self {
        Self {
            pcf,
            switches_num,
            switches_states: [(0, false); switches_num],
        }
    }

    pub fn read_switches(&mut self, i2c: &mut I2C) -> Result<[(u8, bool); switches_num], ()> {
        let pins_state = self.pcf.read_pins(i2c)?;

        for i in 0..self.switches_num {
            let state = (pins_state & (1 << i)) != 0;
            self.switches_states[i] = (i as u8, state);
        }

        Ok(self.switches_states)
    }
}