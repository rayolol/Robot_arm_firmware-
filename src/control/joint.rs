use crate::control::motor::Motor;
use crate::control::pid::PidController;
use crate::control::utils::convert_to_steps_f32;
use crate::drivers::AS6500::{As5600, As5600Error};
use crate::factories::I2cBus;

/// Concrete encoder type used by the joints (I2C1 on the STM32F103).
type JointEncoder = As5600<stm32f1xx_hal::i2c::I2c<stm32f1xx_hal::pac::I2C1>>;

pub struct Joint {
    pub id: usize,
    pub name: &'static str,
    pub motor: Motor,
    pub encoder: Option<JointEncoder>,
    pid: Option<PidController>,
    offset: f32,
    closed_loop: bool,
    output_ratio: Option<f32>,
    pub target_angle: f32,
    pub current_angle: f32,
}

impl Joint {
    pub fn new(id: usize, name: &'static str, motor: Motor, encoder: Option<JointEncoder>, pid: Option<PidController>, output_ratio: Option<f32>) -> Self {
        let closed_loop = encoder.is_some() && pid.is_some();

        Self {
            id,
            name,
            motor,
            encoder,
            pid,
            closed_loop,
            output_ratio,
            offset: 0.0,
            target_angle: 0.0,
            current_angle: 0.0,
        }
    }

    fn read_encoder(&mut self, i2c_bus: &mut I2cBus) -> Result<f32, As5600Error> {
        let encoder = self.encoder.as_mut().ok_or(As5600Error::InvalidData)?;
        let channel = encoder.mux_channel();
        i2c_bus
            .tca
            .select_channel(&mut i2c_bus.i2c, channel, || {
                cortex_m::asm::delay(100_000);
            })
            .map_err(|_| As5600Error::I2cError)?;

        let raw = encoder.read_angle_degrees(&mut i2c_bus.i2c)?;
        self.current_angle = Self::normalize_angle(raw - self.offset);
        Ok(self.current_angle)
    }

    fn normalize_angle(angle: f32) -> f32 {
        ((angle % 360.0) + 360.0) % 360.0

    }

    pub fn zero_encoder(&mut self, i2c_bus: &mut I2cBus) -> Result<(), As5600Error> {
        let encoder = match self.encoder.as_mut() {
            Some(enc) => enc,
            None => return Ok(()),
        };

        let channel = encoder.mux_channel();
        i2c_bus
            .tca
            .select_channel(&mut i2c_bus.i2c, channel, || {
                cortex_m::asm::delay(100_000);
            })
            .map_err(|_| As5600Error::I2cError)?;

        self.offset = encoder.read_angle_degrees(&mut i2c_bus.i2c)?;
        encoder.zero_current_position(&mut i2c_bus.i2c)?;
        self.current_angle = 0.0;
        Ok(())
    }

    #[inline(always)]
    pub fn step_position_loop(&mut self, dt: f32, i2c_bus: &mut I2cBus) -> Result<(), As5600Error> {
        if self.closed_loop {
            self.read_encoder(i2c_bus)?;
        }
        self.update_control(dt);
        Ok(())
    }

    pub fn reset_pid(&mut self) {
        if let Some(pid) = self.pid.as_mut() {
            pid.reset();
        }
    }

    fn update_control(&mut self, dt: f32) {
        if dt < 1e-3 {
            return;
        }

        let mut command = if self.closed_loop {
            match self.pid.as_mut() {
                Some(pid) => pid.update(self.target_angle, self.current_angle, dt),
                None => return,
            }
        } else {
            // Open loop: drive straight from the commanded target (degrees/sec).
            self.target_angle
        };

        if let Some(ratio) = self.output_ratio {
            command *= ratio;
        }

        let setpoint = convert_to_steps_f32(command);
        let _ = self.motor.set_velocity(setpoint);
    }

    #[inline(always)]
    pub fn tick(&mut self) -> bool {
        self.motor.tick()
    }
}
