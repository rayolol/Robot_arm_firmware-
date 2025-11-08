use rtt_target::rprintln;
use crate::control::stepgen::stepgen::Stepgen;
use stm32f1xx_hal::gpio::{ErasedPin, Output, PushPull};

fn f32_to_fixed24_8(v: f32) -> i32 { (v * 256.0) as i32 }
#[allow(dead_code)]
fn fixed24_8_to_f32(raw: i32) -> f32 { raw as f32 / 256.0 }

const MOTOR_DEBUG_MASK: u8 = 0b10; // enable verbose logging per motor bitmask (bit 0 -> motor 0, etc.)
const STALL_CYCLE_THRESHOLD: u8 = 8;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MotorType { Tmc(u8), Tb6600 }

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MotorMode { Disabled, OpenLoop, ClosedLoopVelocity, ClosedLoopPosition }

#[derive(Debug, Clone, Copy)]
pub enum MotorTarget { Velocity(f32), Position(i32) }

#[derive(Debug, Clone, Copy)]
pub struct MotorConfig {
    pub steps_per_rev: u32,
    pub microsteps: u16,
    pub max_velocity: f32,
    pub direction_inverted: bool,
}

impl Default for MotorConfig {
    fn default() -> Self {
        Self { steps_per_rev: 200, microsteps: 16, max_velocity: 500.0, direction_inverted: false }
    }
}

pub struct Motor {
    pub id: u8,
    pub name: &'static str,
    pub motor_type: MotorType,
    step_pin: ErasedPin<Output<PushPull>>,
    dir_pin: ErasedPin<Output<PushPull>>,
    config: MotorConfig,
    ramp: Stepgen,
    pub mode: MotorMode,
    pub target: MotorTarget,
    pub target_velocity: f32,
    pub current_position: i32,
    direction_sign: i8,
    steps_emitted: u32,
    last_command_step: u32,
    stall_cycles: u8,
    step_pin_state: bool,
    pulse_active: bool,
    pulse_ticks: u32,
    scheduler_tick_us: u32,
    pub interval_ticks: u32,
    pub remaining_ticks: u32,
    ramp_update_pending: bool,
    pub enabled: bool,
    pub encoder_id: Option<u8>,
}

impl Motor {
    #[inline(always)]
    fn debug_enabled(&self) -> bool {
        ((1u8 << self.id) & MOTOR_DEBUG_MASK) != 0
    }

    pub fn new(
        id: u8,
        name: &'static str,
        motor_type: MotorType,
        step_pin: ErasedPin<Output<PushPull>>,
        dir_pin: ErasedPin<Output<PushPull>>,
        config: MotorConfig,
        scheduler_tick_us: u32,
        encoder_id: Option<u8>,
    ) -> Self {
        // Stepgen ticks per second: convert scheduler_tick_us to ticks/sec
        let ticks_per_second = 1_000_000 / scheduler_tick_us;
        let pulse_ticks = (5 + scheduler_tick_us - 1) / scheduler_tick_us;
        Self {
            id,
            name,
            motor_type,
            step_pin,
            dir_pin,
            config,
            encoder_id,
            ramp: Stepgen::new(ticks_per_second),
            mode: MotorMode::Disabled,
            target: MotorTarget::Velocity(0.0),
            target_velocity: 0.0,
            current_position: 0,
            direction_sign: 1,
            steps_emitted: 0,
            last_command_step: 0,
            stall_cycles: 0,
            step_pin_state: false,
            scheduler_tick_us,
            pulse_active: false,
            pulse_ticks: pulse_ticks.max(1),
            interval_ticks: u32::MAX,
            remaining_ticks: u32::MAX,
            ramp_update_pending: false,
            enabled: false,
        }
    }

    pub fn init(&mut self, acceleration: f32) -> Result<(), ()> {
        rprintln!("[{}] init", self.name);
        self.enabled = false;
        self.remaining_ticks = 0;
        let acc_fixed = f32_to_fixed24_8(acceleration).max(1);
        if let Err(e) = self.ramp.set_acceleration(acc_fixed as u32) {
            rprintln!("[{}] Stepgen set_acceleration error: {:?}", self.name, e);
            return Err(());
        }
        let _ = self.step_pin.set_low();
        let _ = self.dir_pin.set_low();
        Ok(())
    }

    pub fn set_velocity(&mut self, velocity: f32) -> Result<(), ()> {
        let motor_debug = self.debug_enabled();
        if motor_debug { rprintln!("[{}] set_velocity: {}", self.name, velocity); }
        // Check for NaN or infinite values
        if !velocity.is_finite() {
            rprintln!("[{}] ERROR: velocity is NaN or infinite: {}", self.name, velocity);
            return Err(());
        }

        let velocity_clamped = velocity.clamp(-self.config.max_velocity, self.config.max_velocity);
        let prev_target = self.target_velocity;
        let velocity_delta = (prev_target - velocity_clamped).abs();
        let stepgen_active = self.enabled && self.interval_ticks != u32::MAX;
        if stepgen_active && velocity_clamped.abs() >= 0.1 {
            if self.steps_emitted == self.last_command_step {
                self.stall_cycles = self.stall_cycles.saturating_add(1);
                if self.stall_cycles >= STALL_CYCLE_THRESHOLD {
                    rprintln!("[{}] stall detected (no steps). Forcing re-prime.", self.name);
                    self.enabled = false;
                    self.interval_ticks = u32::MAX;
                }
            } else {
                self.last_command_step = self.steps_emitted;
                self.stall_cycles = 0;
            }
        } else {
            self.last_command_step = self.steps_emitted;
            self.stall_cycles = 0;
        }

        let already_running = stepgen_active
            && matches!(self.target, MotorTarget::Velocity(_))
            && velocity_delta < 0.01;

        self.target_velocity = velocity_clamped;
        self.target = MotorTarget::Velocity(velocity_clamped);

        if already_running {
            rprintln!("[{}] set_velocity: unchanged (delta {:.6})", self.name, velocity_delta);
            return Ok(());
        }

        // convert to 24.8 fixed (Stepgen expects fixed<<8)
        let speed_fixed = f32_to_fixed24_8(velocity_clamped).abs() as u32;

        if speed_fixed == 0 {
            if motor_debug { rprintln!("[{}] set_velocity -> stop (speed_fixed=0)", self.name); }
            self.stop();
            return Ok(());
        }

        // set target speed (check result)
        if let Err(e) = self.ramp.set_target_speed(speed_fixed) {
            rprintln!("[{}] set_target_speed ERROR: {:?}", self.name, e);
            return Err(());
        } else {
            rprintln!("[{}] set_target_speed OK (fixed={})", self.name, speed_fixed);
        }

        let forward = velocity_clamped >= 0.0;
        let _ = self.set_direction(forward);

        let need_prime = !self.enabled || self.interval_ticks == u32::MAX;
        if need_prime {
            rprintln!("[{}] priming ramp", self.name);
            // IMPORTANT: tell stepgen how far to run on first start only
            let run_forever_steps = u32::MAX / 4;
            if let Err(e) = self.ramp.set_target_step(run_forever_steps) {
                rprintln!("[{}] set_target_step ERROR: {:?}", self.name, e);
            }

            // Prime the ramp: call next once so Stepgen computes initial delay/speed
            let delay = self.ramp.next().unwrap_or(0);
            if delay == 0 {
                rprintln!("[{}] prime next() returned 0 — ramp not started", self.name);
                self.enabled = false;
                self.interval_ticks = u32::MAX;
                self.remaining_ticks = u32::MAX;
                return Err(());
            } else {
                let delay_ticks = delay >> 8;
                self.set_interval_ticks(delay_ticks, true);
                self.enabled = true;
                if motor_debug {
                    rprintln!("[{}] primed: delay_ticks={} interval_ticks={}", self.name, delay_ticks, self.interval_ticks);
                }
            }
        }

        Ok(())
    }

    fn set_direction(&mut self, forward: bool) -> Result<(), ()> {
        let dir = if self.config.direction_inverted { !forward } else { forward };
        self.direction_sign = if forward { 1 } else { -1 };
        if dir { let _ = self.dir_pin.set_high(); } else { let _ = self.dir_pin.set_low(); }
        Ok(())
    }

    pub fn update_from_ramp(&mut self, reload_remaining: bool) -> bool {
        let delay = self.ramp.next().unwrap_or(0);

        if delay == 0 {
            rprintln!("[{}] update_from_ramp: next() -> 0 (stopped or not configured)", self.name);
            self.enabled = false;
            self.interval_ticks = u32::MAX;
            self.remaining_ticks = u32::MAX;
            return false;
        }

        // delay is 16.8 fixed representing ticks. Convert to ticks integer:
        let delay_ticks = delay >> 8; // number of Stepgen ticks per step
        if delay_ticks == 0 {
            rprintln!("[{}] update_from_ramp: delay_ticks == 0 (too fast)", self.name);
            self.enabled = false;
            return false;
        }

        self.set_interval_ticks(delay_ticks, reload_remaining);
        if self.debug_enabled() {
            rprintln!("[{}] update_from_ramp: delay={} delay_ticks={} interval_ticks={}", self.name, delay, delay_ticks, self.interval_ticks);
        }
        self.enabled = true;
        true
    }


    pub fn set_interval_ticks(&mut self, delay_ticks: u32, reload_remaining: bool) {
        let interval = delay_ticks.max(self.pulse_ticks + 1);
        self.interval_ticks = interval;
        if reload_remaining {
            self.remaining_ticks = self.interval_ticks;
        }
        self.enabled = true;
    }

    #[inline(always)]
    /// Called from the high-rate timer; returns true on the rising edge so callers
    /// can fetch the next ramp interval exactly once per emitted step.
    pub fn tick(&mut self) -> bool {
        if !self.enabled { return false; }
        if self.remaining_ticks > 0 {
            self.remaining_ticks -= 1;
            return false;
        }

        if !self.pulse_active {
            // Rising edge: assert STEP high briefly
            self.pulse_active = true;
            self.step_pin_state = true;
            let _ = self.step_pin.set_high();
            self.current_position = self.current_position.wrapping_add(self.direction_sign as i32);
            self.steps_emitted = self.steps_emitted.wrapping_add(1);
            self.remaining_ticks = self.pulse_ticks.max(1);
            self.ramp_update_pending = true;
            if self.debug_enabled() {
                rprintln!("[{}] STEP high pos={} interval_ticks={} pulse_ticks={}", self.name, self.current_position, self.interval_ticks, self.pulse_ticks);
            }
            true
        } else {
            // End of pulse: drop STEP low and wait for remainder of interval
            self.pulse_active = false;
            self.step_pin_state = false;
            let _ = self.step_pin.set_low();
            let rest = self.interval_ticks.saturating_sub(self.pulse_ticks).max(1);
            self.remaining_ticks = rest;
            if self.ramp_update_pending {
                self.ramp_update_pending = false;
                let _ = self.update_from_ramp(false);
            }
            if self.debug_enabled() {
                rprintln!("[{}] STEP low rest_ticks={}", self.name, rest);
            }
            false
        }
    }

    pub fn stop(&mut self) {
        self.target_velocity = 0.0;
        self.enabled = false;
        self.pulse_active = false;
        self.step_pin_state = false;
        self.ramp_update_pending = false;
        self.last_command_step = self.steps_emitted;
        self.stall_cycles = 0;
        let _ = self.step_pin.set_low();
        self.interval_ticks = u32::MAX;
        self.remaining_ticks = 0;
    }
}
