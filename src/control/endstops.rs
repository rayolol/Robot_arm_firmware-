


pub struct Endstops {
    endstops_addr: [u8; 8],
    endstop_states: [(u8, bool); 6],
}

impl Endstops {
    pub fn new(pcf: pcf8574<I2C>) -> Self {
        Self {
            pcf,
        }
    }
}
