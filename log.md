# Step Generation Debug Log

## Context
- Date: 2025-01-14 (session spanning the entire day)
- Goal: Get both NEMA-17 axes stepping reliably from the STM32F103 RTIC firmware, diagnose why only motor 1 moved, and ensure we have tools to catch future regressions.

## Key Changes
1. **Motor ramp priming fixes**
   - `Motor::init` now leaves motors disabled so the first non-zero velocity command primes Stepgen correctly.
   - `set_velocity` always records the requested value before deciding if it can early-return, and it now forces priming whenever `interval_ticks == u32::MAX`.
   - `stop()` resets interval/remaining ticks to make the next start deterministic.

2. **Step scheduling inside the ISR**
   - Rising/falling edges in `Motor::tick` now handle ramp updates directly, so the TIM2 handler only calls `tick_motors()` and doesn’t re-lock the shared motor state to fetch delays.
   - Position counters now respect direction via `direction_sign`, so closed-loop math sees signed travel.

3. **Diagnostics and stall detection**
   - Added `MOTOR_DEBUG_MASK` plus per-motor step counters. When a motor is commanded to move but we don’t observe new steps for several control cycles, we log a stall warning and automatically re-prime Stepgen.
   - STEP edge logs can be toggled per axis, making it easy to confirm ISR activity without drowning in RTT output.

4. **Control-loop hygiene**
   - Guarded noisy `rprintln!` calls with the new debug mask so TIM2 logs don’t dominate cycles.
   - Confirmed TIM2 priority (5) preempts TIM3/USART and left TIM3 (control loop) at priority 2 so encoder/PID math can’t starve the high-rate ticker.

5. **Operational learnings captured in code comments**
   - Documented that zero-speed commands must stop the ramp instead of erroring.
   - Converted interval math to direct tick counts (`set_interval_ticks`) instead of microseconds to avoid rounding drift.

## Lessons Learned
1. **Always verify hardware mapping first.** We lost time because motor 2’s STEP/DIR pins were miswired; the software was correct, but the driver never saw pulses. Confirm physical wiring whenever one axis misbehaves.
2. **Prime ramps only when the motor is truly idle.** Leaving `enabled = true` after `init` prevented Stepgen from ever starting. Clear state between runs so the first command can configure acceleration/delay data.
3. **Add observability early.** The per-motor debug mask and stall detector immediately showed that motor 2’s STEP ISR was firing, narrowing the problem to wiring/driver config instead of firmware math.
4. **Start conservatively with driver current.** Configure TMC2209 current around 80 % of the motor’s rated value (≈1 A RMS for most NEMA-17s) and adjust upward only after checking temperature/torque.
5. **Control loops will happily spam the same target.** “set_velocity unchanged” messages were expected once the PID settled; you only get motion if the hardware actually sees the pulses.

## Next Steps
1. Wire a simple driver self-test command so we can query DRV_STATUS for both axes at boot.
2. Consider logging encoder deltas next to the step counters to catch cases where pulses happen but the joint is mechanically stalled.
3. Once tuning resumes, disable `MOTOR_DEBUG_MASK` to keep TIM2 ISR lightweight.

