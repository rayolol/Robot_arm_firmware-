
use core::f32;
use libm::ceilf;

const MICROSTEPS: f32 = 4.0;
const STEPS_PER_REV: f32 = 200.0;
const STEPS_PER_MICROSTEP: f32 = STEPS_PER_REV * MICROSTEPS;
const STEPS_PER_DEGREE: f32 = STEPS_PER_MICROSTEP / 360.0;

pub fn convert_to_steps(angle_deg: f32) -> i32 {
    ceilf(angle_deg * STEPS_PER_DEGREE) as i32
}

/// Convert angle in degrees to steps without rounding so callers can keep fractional resolution.
pub fn convert_to_steps_f32(angle_deg: f32) -> f32 {
    angle_deg * STEPS_PER_DEGREE
}

pub fn convert_to_angle(steps: i32) -> f32 {
    let degrees_per_step = 360.0 / STEPS_PER_MICROSTEP;
    steps as f32 * degrees_per_step
}


pub fn encoders_to_pitch_roll(angle1: f32, angle2: f32, ratio: f32) -> (f32, f32) {
    let pitch = (angle1 + angle2) * 0.5 * ratio;
    let roll = (angle1 - angle2) * 0.5 * ratio ;
    (pitch, roll)
}
