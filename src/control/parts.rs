use rtt_target::rprintln;

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




pub struct EndEffector {
    pub motor1: crate::control::motor::Motor,
    pub motor2: crate::control::motor::Motor,
    pub motor3: crate::control::motor::Motor,

    pub pid1: crate::control::pid::PidController,
    pub pid2: crate::control::pid::PidController,
    pub pid3: crate::control::pid::PidController,

    pub motor_to_wrist1: f32,
    pub motor_to_wrist2: f32,

    pub control_mode: ControlMode,

    pub target_angle1: f32,
    pub target_angle2: f32,
    target_vel1: f32,
    target_vel2: f32,
    target_vel3: f32,
    target_angle3: f32,
    encoder_offset1: f32,
    encoder_offset2: f32,
    encoder_offset3: f32,
    encoder_offsets_valid: bool,
}

impl EndEffector {
    pub fn new(motor1: crate::control::motor::Motor,
               motor2: crate::control::motor::Motor,
               motor3: crate::control::motor::Motor,
               pid1: crate::control::pid::PidController,
               pid2: crate::control::pid::PidController,    
               pid3: crate::control::pid::PidController,
               motor_to_wrist1: f32,
               motor_to_wrist2: f32,
               control_mode: ControlMode,
    ) -> Self {
        Self {
            motor1,
            motor2,
            motor3,
            pid1,
            pid2,
            pid3,
            motor_to_wrist1,
            motor_to_wrist2,
            target_angle1: 0.0,
            target_angle2: 0.0,
            target_angle3: 0.0,
            target_vel1: 0.0,
            target_vel2: 0.0,
            target_vel3: 0.0,
            control_mode,
            encoder_offset1: 0.0,
            encoder_offset2: 0.0,
            encoder_offset3: 0.0,
            encoder_offsets_valid: false,
        }
    }


    pub fn set_pitch_roll(&mut self, pitch: f32, roll: f32) {
        self.target_angle1 = (pitch + roll) / (2.0 * self.motor_to_wrist1);
        self.target_angle2 = (pitch - roll) / (2.0 * self.motor_to_wrist2);
    }


    pub fn set_pitch_roll_speed(&mut self, pitch_speed: f32, roll_speed: f32) {
        self.target_vel1 = (pitch_speed + roll_speed) / (2.0 * self.motor_to_wrist1);
        self.target_vel2 = (pitch_speed - roll_speed) / (2.0 * self.motor_to_wrist2);
    }

    pub fn set_tool_roll_speed(&mut self, speed: f32) {
        self.target_vel3 = speed;
    }

    pub fn tool_roll_angle(&mut self, angle: f32) {
        self.target_angle3 = angle;
    }

    pub fn reset_encoder_offsets(&mut self) {
        self.encoder_offsets_valid = false;
    }



    pub fn update_position_control(&mut self, current_angle1: Option<f32>, current_angle2: Option<f32>, current_angle3: Option<f32>, dt: Option<f32>) {

        if self.control_mode == ControlMode::Velocity(ControlLoop::Closed) || self.control_mode == ControlMode::Velocity(ControlLoop::Open) {
            return;
        }
        if current_angle1.is_none() || current_angle2.is_none() || dt.is_none() {
            rprintln!("[pos] missing data: a1={:?} a2={:?} dt_present={}", current_angle1, current_angle2, dt.is_some());
            return;
        }

        let current1 = current_angle1.unwrap();
        let current2 = current_angle2.unwrap();
        let current3 = current_angle3.unwrap_or(0.0);

        if !self.encoder_offsets_valid {
            self.encoder_offset1 = current1 - self.target_angle1;
            self.encoder_offset2 = current2 - self.target_angle2;
            self.encoder_offset3 = current3 - self.target_angle3;
            self.encoder_offsets_valid = true;
            rprintln!(
                "[pos] synced encoder offsets -> ({:.3}, {:.3}, {:.3})",
                self.encoder_offset1,
                self.encoder_offset2,
                self.encoder_offset3
            );
        }

        let adj_angle1 = current1 - self.encoder_offset1;
        let adj_angle2 = current2 - self.encoder_offset2;
        let adj_angle3 = current3 - self.encoder_offset3;

        let mut output1 = self.target_angle1;
        let mut output2 = self.target_angle2;
        let mut output3 = self.target_angle3;
        if self.control_mode == ControlMode::Position(ControlLoop::Closed) {
            rprintln!("Updating position control");
            output1 = self.pid1.update(self.target_angle1, adj_angle1, dt.unwrap());
            output2 = self.pid2.update(self.target_angle2, adj_angle2, dt.unwrap());
            output3 = self.pid3.update(self.target_angle3, adj_angle3, dt.unwrap());
            let steps1 = crate::control::utils::convert_to_steps_f32(output1);
            let steps2 = crate::control::utils::convert_to_steps_f32(output2);
            let steps3 = crate::control::utils::convert_to_steps_f32(output3);
            rprintln!("[pos] target angles( {:.3},{:.3} )setpoints deg({:.3},{:.3}) current ({:.3},{:.3}) -> steps ({:.2}, {:.2})",self.target_angle1, self.target_angle2, output1, output2, adj_angle1, adj_angle2, steps1, steps2);
        
            let _ = self.motor1.set_velocity(steps1);
            let _ = self.motor2.set_velocity(steps2);
            let _ = self.motor3.set_velocity(steps3);
        }
        //TODO : Add open loop position control if needed

    }

    pub fn update_velocity_control(&mut self, current_vel1: Option<f32>, current_vel2: Option<f32>, current_vel3: Option<f32>, dt: Option<f32>) {
        if self.control_mode == ControlMode::Position(ControlLoop::Closed) || self.control_mode == ControlMode::Position(ControlLoop::Open) {
            return;
        }
        if current_vel1.is_none() || current_vel2.is_none() || dt.is_none() {
            return;
        }
        let mut output1 = self.target_vel1;
        let mut output2 = self.target_vel2;
        let mut output3 = self.target_vel3;
        if self.control_mode == ControlMode::Velocity(ControlLoop::Closed) {
            rprintln!("Updating velocity control");
            output1 = self.pid1.update(self.target_vel1, current_vel1.unwrap(), dt.unwrap());
            output2 = self.pid2.update(self.target_vel2, current_vel2.unwrap(), dt.unwrap());
            output3 = self.pid3.update(self.target_vel3, current_vel3.unwrap(), dt.unwrap());
        }
        let steps1 = crate::control::utils::convert_to_steps_f32(output1);
        let steps2 = crate::control::utils::convert_to_steps_f32(output2);
        let steps3 = crate::control::utils::convert_to_steps_f32(output3);
        rprintln!("[vel] targets {:?} -> steps ({:.2}, {:.2}, {:.2})", (output1, output2, output3), steps1, steps2, steps3);
        let _ = self.motor1.set_velocity(steps1);
        let _ = self.motor2.set_velocity(steps2);
        let _ = self.motor3.set_velocity(steps3);
    }

    /// Tick both motors; returns which ones produced a rising edge this cycle.
    #[inline(always)]
    pub fn tick_motors(&mut self) -> [bool; 3] {
        let m1_step = self.motor1.tick();
        let m2_step = self.motor2.tick();
        let m3_step = self.motor3.tick();
        [m1_step, m2_step, m3_step]
    }
}
