
#[derive(Clone, PartialEq)]
pub struct PidController {
    id: u8,
    kp: f32,
    ki: f32,
    kd: f32,
    integral_error: f32,
    derivative_error: f32,
    last_error: f32,
    output_min: f32,
    output_max: f32,
    integral_min: f32,
    integral_max: f32,
}

impl PidController {
    pub fn new(kp: f32, ki: f32, kd: f32, id: u8) -> Self {
        Self {
            id,
            kp,
            ki,
            kd,
            integral_error: 0.0,
            last_error: 0.0,
            derivative_error: 0.0,
            output_max: 300.0,
            output_min: -300.0,  // Fixed: should be negative!
            integral_min: -100.0,
            integral_max: 100.0,
        }
    }
    pub fn update(&mut self, setpoint: f32, measurement: f32, dt: f32) -> f32 {
        // Check for invalid inputs
        if !setpoint.is_finite() || !measurement.is_finite() || !dt.is_finite() || dt <= 0.0 {
            return 0.0; // Return safe value on invalid input
        }

        let error = setpoint - measurement;
        self.integral_error += error * dt;
        self.integral_error = self.integral_error.clamp(self.integral_min, self.integral_max);

        self.derivative_error = (error - self.last_error) / dt;

        self.last_error = error;


        let output: f32 = (self.kp * error) + (self.ki * self.integral_error) + (self.kd * self.derivative_error);
        output.clamp(self.output_min, self.output_max)
    }
    pub fn reset(&mut self) {
        self.integral_error = 0.0;
        self.last_error = 0.0;
        self.derivative_error = 0.0;
    }
    pub fn set_limits(&mut self, min: f32, max: f32, int_min: f32, int_max: f32) {
        self.output_min = min;
        self.output_max = max;
        self.integral_min = int_min;
        self.integral_max = int_max;
    }
}