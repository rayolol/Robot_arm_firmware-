use rtt_target::rprintln;
use crate::control::stepgen::stepgen::Stepgen;
use stm32f1xx_hal::gpio::{ErasedPin, Output, PushPull};

fn f32_to_fixed24_8(v: f32) -> i32 { (v * 256.0) as i32 }
fn fixed24_8_to_f32(raw: i32) -> f32 { raw as f32 / 256.0 }

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
    step_pin_state: bool,
    scheduler_tick_us: u32,
    pub interval_ticks: u32,
    pub remaining_ticks: u32,
    pub enabled: bool,
    pub encoder_id: Option<u8>,
}

impl Motor {
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
            step_pin_state: false,
            scheduler_tick_us,
            interval_ticks: u32::MAX,
            remaining_ticks: u32::MAX,
            enabled: false,
        }
    }

    pub fn init(&mut self, acceleration: f32) -> Result<(), ()> {
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
        // Check for NaN or infinite values
        if !velocity.is_finite() {
            rprintln!("[{}] ERROR: velocity is NaN or infinite: {}", self.name, velocity);
            return Err(());
        }

        let velocity_clamped = velocity.clamp(-self.config.max_velocity, self.config.max_velocity);
        self.target_velocity = velocity_clamped;
        rprintln!("[{}] target vel {}", self.name, velocity_clamped);
        self.target = MotorTarget::Velocity(velocity_clamped);

        // convert to 24.8 fixed (Stepgen expects fixed<<8)
        let speed_fixed = f32_to_fixed24_8(velocity_clamped).abs() as u32;

        // set target speed (check result)
        match self.ramp.set_target_speed(speed_fixed) {
            Ok(()) => {
                rprintln!("[{}] set_target_speed OK (fixed={})", self.name, speed_fixed);
            }
            Err(e) => {
                rprintln!("[{}] set_target_speed ERROR: {:?}", self.name, e);
                return Err(());
            }
        }

        // IMPORTANT: tell stepgen how far to run.
        // Set a very large target so it will keep generating steps until you call stop().
        // Use a large but safe value instead of u32::MAX to avoid edge cases.
        let run_forever_steps = u32::MAX / 4;
        if let Err(e) = self.ramp.set_target_step(run_forever_steps) {
            rprintln!("[{}] set_target_step ERROR: {:?}", self.name, e);
            // Not fatal: continue, but expect next() to return 0 if this fails
        } else {
            rprintln!("[{}] set_target_step = {}", self.name, run_forever_steps);
        }

        // set direction pin (non-ISR)
        let forward = velocity_clamped >= 0.0;
        let _ = self.set_direction(forward);

        // Prime the ramp: call next once so Stepgen computes initial delay/speed
        let delay = self.ramp.next().unwrap_or(0);
        if delay == 0 {
            rprintln!("[{}] prime next() returned 0 — ramp not started", self.name);
            // keep disabled; update_from_ramp will see the same
            self.enabled = false;
            self.interval_ticks = u32::MAX;
            self.remaining_ticks = u32::MAX;
            return Err(());
        } else {
            // compute steps/sec from delay (delay is 16.8 format)
            let speed_raw = self.ramp.current_speed() as i32;
            let steps_per_sec = fixed24_8_to_f32(speed_raw);
            rprintln!("[{}] prime: delay={} (raw_speed={} => {:.3} steps/s)", 
                self.name, delay, speed_raw, steps_per_sec);

            // convert delay to interval_us for your scheduler (delay is in ???units - see below)
            // The crate returns delays in 16.8 fixed format of ticks: actual_delay_ticks = delay >> 8 (ticks)
            // Each tick length = 1 / ticks_per_second = scheduler_tick_us microseconds in your setup
            // We can compute microseconds: delay_ticks = delay >> 8
            // microseconds = delay_ticks * scheduler_tick_us
            let delay_ticks = delay >> 8;
            let interval_us = delay_ticks.saturating_mul(self.scheduler_tick_us);
            self.set_interval_us(interval_us, self.scheduler_tick_us);
            self.enabled = true;
            rprintln!("[{}] primed: delay_ticks={} interval_us={} interval_ticks={}", 
                self.name, delay_ticks, interval_us, self.interval_ticks);
        }

        Ok(())
    }

    fn set_direction(&mut self, forward: bool) -> Result<(), ()> {
        let dir = if self.config.direction_inverted { !forward } else { forward };
        if dir { let _ = self.dir_pin.set_high(); } else { let _ = self.dir_pin.set_low(); }
        Ok(())
    }

    pub fn update_from_ramp(&mut self) -> bool {
        let start = cortex_m::peripheral::DWT::cycle_count();
        let delay = self.ramp.next().unwrap_or(0);
        let cycles = cortex_m::peripheral::DWT::cycle_count().wrapping_sub(start);
        rprintln!("[dbg] next() cycles = {}", cycles);

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

        // Convert to steps/sec for debug: current_speed() returns 24.8 fixed
        let speed_raw = self.ramp.current_speed() as i32;
        let steps_per_sec = fixed24_8_to_f32(speed_raw);
        rprintln!("[{}] update_from_ramp: delay={} delay_ticks={} speed_raw={} {:.3} steps/s",
            self.name, delay, delay_ticks, speed_raw, steps_per_sec);

        // Convert delay_ticks to microseconds using your tick length
        let interval_us = delay_ticks.saturating_mul(self.scheduler_tick_us);
        self.set_interval_us(interval_us, self.scheduler_tick_us);
        self.enabled = true;
        true
    }


    pub fn set_interval_us(&mut self, interval_us: u32, scheduler_tick_us: u32) {
        let ticks = ((interval_us as u64 + (scheduler_tick_us as u64 / 2)) / scheduler_tick_us as u64) as u32;
        self.interval_ticks = ticks.max(1);
        self.remaining_ticks = self.interval_ticks;
        self.enabled = true;
    }

    #[inline(always)]
    pub fn tick(&mut self) {
        if !self.enabled { return; }
        if self.remaining_ticks > 0 { self.remaining_ticks -= 1; return; }

        self.step_pin_state = !self.step_pin_state;
        if self.step_pin_state {
            let _ = self.step_pin.set_high();
            self.current_position = self.current_position.wrapping_add(1);
        } else {
            let _ = self.step_pin.set_low();
        }
        self.remaining_ticks = self.interval_ticks;
    }

    pub fn stop(&mut self) { self.target_velocity = 0.0; self.enabled = false; }
}



