#[derive(PartialEq)]
pub enum ControlLoop {
    Open,
    Closed,
}
#[derive(PartialEq)]
pub enum ControlMode {
    Position(ControlLoop),
    Velocity(ControlLoop),
}


use crate::control::joint::Joint as J;
use crate::drivers::AS6500::As5600Error;
use crate::factories::I2cBus;

pub struct EndEffector {
    pub joint1: J,
    pub joint2: J,
    pub joint3: J,


    pub motor_to_wrist1: f32,
    pub motor_to_wrist2: f32,

    pub control_mode: ControlMode,
}

impl EndEffector {
    pub fn new(joint1: J,
               joint2: J,
               joint3: J,
               motor_to_wrist1: f32,
               motor_to_wrist2: f32,
               control_mode: ControlMode,
    ) -> Self {
        Self {
            joint1,
            joint2,
            joint3,
            motor_to_wrist1,
            motor_to_wrist2,
            control_mode,
        }
    }


    pub fn set_pitch_roll(&mut self, pitch: f32, roll: f32) {
        self.joint1.target_angle = (pitch + roll) / (2.0 * self.motor_to_wrist1);
        self.joint2.target_angle = (pitch - roll) / (2.0 * self.motor_to_wrist2);
    }


    // pub fn set_pitch_roll_speed(&mut self, pitch_speed: f32, roll_speed: f32) {
    //     self.target_vel1 = (pitch_speed + roll_speed) / (2.0 * self.motor_to_wrist1);
    //     self.target_vel2 = (pitch_speed - roll_speed) / (2.0 * self.motor_to_wrist2);
    // }

    // pub fn set_tool_roll_speed(&mut self, speed: f32) {
    //     self.target_vel3 = speed;
    // }

    pub fn tool_roll_angle(&mut self, angle: f32) {
        self.joint3.target_angle = angle;
    }


    #[inline(always)]
    pub fn step_position_loop(&mut self, dt: f32, i2c_bus: &mut I2cBus) -> Result<(), As5600Error> {
       if self.control_mode != ControlMode::Position(ControlLoop::Closed) {
            return Ok(());
        }
        if dt <= 0.0 {
            return Ok(());
        }
        self.joint1.step_position_loop(dt, i2c_bus)?;
        self.joint2.step_position_loop(dt, i2c_bus)?;
        self.joint3.step_position_loop(dt, i2c_bus)?;
        Ok(())
    }

    #[inline(always)]
    pub fn tick_motors(&mut self) -> [bool; 3] {
        let m1_step = self.joint1.tick();
        let m2_step = self.joint2.tick();
        let m3_step = self.joint3.tick();
        [m1_step, m2_step, m3_step]
    }
}


pub struct Robot {
    pub joint1: J,
    pub joint2: J,
    pub joint3: J,
    pub tool: EndEffector,
    pub control_mode: ControlMode,
}

impl Robot {
    pub fn new(joint1: J, joint2: J, joint3: J, tool: EndEffector, control_mode: ControlMode) -> Self {
        Self {
            joint1,
            joint2,
            joint3,
            tool,
            control_mode,
        }
    }

    pub fn set_target(&mut self, target: [f32; 6]) {
        self.joint1.target_angle = target[0];
        self.joint2.target_angle = target[1];
        self.joint3.target_angle = target[2];
        self.tool.tool_roll_angle(target[3]);
        self.tool.set_pitch_roll(target[4], target[5]);

    }

    pub fn step_position_loop(&mut self, dt: f32, i2c_bus: &mut I2cBus) -> Result<(), As5600Error> {
        self.tool.step_position_loop(dt, i2c_bus);
        self.joint1.step_position_loop(dt, i2c_bus)?;
        self.joint2.step_position_loop(dt, i2c_bus)?;
        self.joint3.step_position_loop(dt, i2c_bus)?;
        Ok(())
    }

    #[inline(always)]
    pub fn tick_motors(&mut self) -> [bool; 6] {
        let m1_step = self.joint1.tick();
        let m2_step = self.joint2.tick();
        let m3_step = self.joint3.tick();
        let [m4_step, m5_step, m6_step] = self.tool.tick_motors();
        [m1_step, m2_step, m3_step, m4_step, m5_step, m6_step]
    }


}