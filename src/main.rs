#![no_std]
#![no_main]
mod control;
mod drivers;
mod config;
mod factories;

use panic_rtt_target as _;
use rtt_target::{rtt_init_print, rprintln};
use rtic::app; 
use stm32f1xx_hal::{
    prelude::*,
    pac,
    timer::{Event},
    gpio::{gpioc::PC13, gpiob::PB1,  Output,Input, PinState, PushPull, PullUp},
    serial::{Serial, Config, Rx, Tx},
    timer::{CounterHz, CounterUs},
    i2c::{I2c, Mode},
    rcc,
};
use control::motor::{Motor as MT, MotorType, MotorConfig};
use drivers::TMC2209::{MotorCommand,MotorResponse, TmcDriver};
use drivers::AS6500::{As5600, As5600Error, MagnetStatus};
use drivers::TCA::TCA9548A;
use factories::*;


const SYSCLK_HZ: u32 = 72_000_000;
const BAUD: u32 = 9_600;
const BIT_CYCLES: u32 = SYSCLK_HZ / BAUD;       // ≈ 7500 cycles per bit (72MHz/9600)
const BYTE_TIME_CYCLES: u32 = BIT_CYCLES * 10;  // 1 start + 8 data + 1 stop ≈ 75_000 cycles ~1.04 ms
const BYTE_CYCLES: u32 = BIT_CYCLES * 10;        // start+8+stop ~75k cycles ~1.04ms
const DEBUG_SPIN_MOTORS: bool = false;

const MICROSTEPS: u16 = 4;
const STEPS_PER_REV: i32 = 200; 
const STEPS_PER_MICROSTEP: i32 = STEPS_PER_REV * MICROSTEPS as i32;

mod halfduplex;
use halfduplex::HalfDuplexSerial;
// use utils::{write_register, read_register, configure_motor}; 

// ============================================================================
// RTIC V1 APP
// ============================================================================

#[app(device = stm32f1xx_hal::pac, peripherals = true, dispatchers = [EXTI0, EXTI1])]
mod app {

    use rtic_monotonics::{Monotonic, systick_monotonic};
    use rtic_sync::{channel::*, make_channel};


    use crate::control::{joint::Joint, parts::{ControlLoop, ControlMode,Robot, EndEffector}};

    use super::*;

    #[shared]
    struct Shared {
        six_dof: Robot,
        limit_switch: crate::control::limit_switch::LimitSwitch::<stm32f1xx_hal::i2c::I2c<stm32f1xx_hal::pac::I2C1>,4>,
        i2c_bus: I2cBus,
    }

    #[local]
    struct Local {
        half_serial: HalfDuplexSerial,
        timer: CounterUs<pac::TIM2>,
        timer3: CounterUs<pac::TIM3>,
        led: PC13<Output<PushPull>>,
        tmcs: [TmcDriver; 4],
        tmc_client: TmcClient,
    }

    systick_monotonic!(Mono);


    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {

     
        let (flash, mut rcc) = setup_clocks(ctx.device.FLASH, ctx.device.RCC);

        let (tmcs, tmc_client, tmc_server) = tmc_stack();

        let (gpioa, gpiob, mut gpioc, afio) = make_gpio(ctx.device.GPIOA, ctx.device.GPIOB, ctx.device.GPIOC, ctx.device.AFIO, &mut rcc);

        let (timer, timer3, delay) = setup_timers(ctx.device.TIM2, ctx.device.TIM3, ctx.device.TIM4, &mut rcc);

        let PinsAndI2c { step_dir, rst_mux, scl, sda } = make_pins(afio, gpiob, gpioa);

        let mut i2c_bus = setup_i2c(ctx.device.I2C1, scl, sda, rst_mux, &mut rcc);

        let half_serial = setup_tmc_uart(delay);

        let (encoders, limit_switch, pids) = make_encoders(0);
    
        let motors = make_motors(step_dir);

        let stages  = [None, None, None, None, None, None];

        let mut joints = factories::make_joints(motors, None, None, stages);

        if let Some(enc) = joints.get_mut(1).and_then(|j| j.encoder.as_mut()) {
            enc.invert();
        }
        let stage1 = 20.0/10.0;
        let stage2 = 32.0/36.0;

        for joint in joints.iter_mut() {
            let Some(encoder) = joint.encoder.as_mut() else {
                continue;
            };

            if let Err(err) = i2c_bus.tca.select_channel(&mut i2c_bus.i2c, encoder.mux_channel(), || { cortex_m::asm::delay(100000); }) {
                rprintln!("Failed to select TCA channel {}: {:?}", encoder.mux_channel(), err);
                continue;
            }
            let mag = encoder.read_magnitude(&mut i2c_bus.i2c);
            let status = encoder.read_magnet_status(&mut i2c_bus.i2c);

            match (mag, status) {
                (Ok(m), Ok(s)) => {
                    rprintln!("Encoder {} magnitude: {}, status: {:?}", encoder.mux_channel(), m, s);
                }
                _ => {
                    rprintln!("Encoder {} read error", encoder.mux_channel());
                }
            }

            if let Err(e) = encoder.zero_current_position(&mut i2c_bus.i2c) {
                rprintln!("Encoder {} zero failed: {:?}", encoder.mux_channel(), e);
            } else {
                rprintln!("Encoder {} zeroed", encoder.mux_channel());
            }
        }

        let [joint0, joint1, joint2, joint3, joint4, joint5] = joints;

        //TODO: move the reduction ratio configuration the joint
        //TODO: refactor the control mode configuration
        let end_effector = EndEffector::new(joint0, joint1, joint2, stage1*stage2, stage1*stage2, ControlMode::Position(ControlLoop::Closed));
        let six_dof = Robot::new(joint3, joint4, joint5, end_effector, ControlMode::Position(ControlLoop::Closed));

        Mono::start(ctx.core.SYST, SYSCLK_HZ);

        rtt_init_print!();

        let led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);

 
    let mut ok = false;
    if i2c_bus.i2c.write(crate::config::TCA9548A_I2C_ADDR, &[]).is_ok() { ok = true; }
    rprintln!(" TCA9548A @ 0x{:02X} -> {}", crate::config::TCA9548A_I2C_ADDR, if ok { "ACK" } else { "NO ACK" });

    // try AS5600 address
    let mut as_ok = false;
    if i2c_bus.i2c.write(crate::config::AS5600_I2C_ADDR, &[]).is_ok() { as_ok = true; }
    rprintln!(" AS5600 @ 0x{:02X} -> {}", crate::config::AS5600_I2C_ADDR, if as_ok { "ACK (direct)" } else { "NO ACK (direct)" });

            unsafe {
                cortex_m::peripheral::NVIC::unmask(pac::Interrupt::TIM3);
                cortex_m::peripheral::NVIC::unmask(pac::Interrupt::TIM2);
            }

        (
            Shared {
                six_dof,
                limit_switch,
                i2c_bus,
            },
            Local {
                half_serial,
                timer,
                timer3,
                led,
                tmcs,
                tmc_client,
            },
    )
    }



    #[task( binds = TIM2, local = [timer], shared = [six_dof], priority = 5)]
    fn tmc_manager(mut ctx: tmc_manager::Context) {
        // Clear the interrupt flag
        ctx.local.timer.clear_interrupt(Event::Update);

        // Tick all motors in the end effector
        ctx.shared.six_dof.lock(|ee| {
            let [m1_step, m2_step, m3_step, m4_step, m5_step, m6_step] = ee.tick_motors();
            if DEBUG_SPIN_MOTORS && (m1_step || m2_step || m3_step || m4_step || m5_step || m6_step) {
                rprintln!("steps: m1={} m2={} m3={}", m1_step, m2_step, m3_step);
            }
        })
    }
    
    #[task(priority = 1)]
    async fn configue_task(_ctx: configue_task::Context, mut command_sender: Sender<'static, (u8, MotorCommand), 8>) {
        for i in 0..4 {
            command_sender.try_send((i, MotorCommand::GetStatus)).unwrap();
            command_sender.try_send((i, MotorCommand::SetCurrent(1000))).unwrap();
            command_sender.try_send((i, MotorCommand::SetMicrosteps(4))).unwrap();
           
        }
    }

    // // Keep this task lower priority than the high-rate TIM2 ticker so long UART exchanges
    // // can't stall the step generator and cause visible jitter.
    #[task(local=[tmcs, half_serial], priority = 1)]
    async fn tmc_command_task(ctx: tmc_command_task::Context, mut command_sender: Receiver<'static, (u8, MotorCommand), 8>, mut response_sender: Sender<'static, (u8, MotorResponse), 8>) {
        loop {
            let command = command_sender.recv().await.ok();

        
            match command {
                Some(cmd) => {
                    rprintln!("got command for motor {}: {:?}", cmd.0, cmd.1);
                    let response = ctx.local.tmcs[cmd.0 as usize].execute(cmd.1, ctx.local.half_serial);
                    response_sender.try_send((cmd.0, response)).ok();
                    rprintln!("response sent {:?}", response);
                }
                None => {
                    rprintln!("no command received in tmc_command_task");
                }
            }
            
        }
        
    }
    
    #[task(priority = 3, local = [homed:[bool; 4] = [false; 4] ], shared = [i2c_bus, limit_switch, six_dof])]
    async fn home_sequence(mut ctx: home_sequence::Context) {
        rprintln!("home_sequence task started");

        while (!ctx.local.homed.iter().all(|&h| h)) {
            rprintln!("homing...");
            ctx.shared.limit_switch.lock(|switch| {
                ctx.shared.i2c_bus.lock(|i2c| {
                    let switches = switch.read_switches(&mut i2c.i2c).unwrap();
             
                    rprintln!("switches: {:?}", switches);
                    for (pin, state) in switches {
                        if state {
                            ctx.local.homed[pin as usize] = true;
                            if pin > 2 {
                                ctx.shared.six_dof.lock(|ee| {
                                ee.tool.joint1.zero_encoder(i2c).unwrap();
                                ee.tool.joint2.zero_encoder(i2c).unwrap();
                            });
                            } else {
                                ctx.shared.six_dof.lock(|ee| {
                                ee.joint1.zero_encoder(i2c).unwrap();
                                ee.joint2.zero_encoder(i2c).unwrap();
                                ee.joint3.zero_encoder(i2c).unwrap();
                            });
                            }
                      
                        }
                    }
                });
                
            })
        }
    }

    // }
    // #[task(binds = USART2, local = [ rx_buffer: [u8; 4] = [0; 4], rx_index: usize = 0], priority = 3)]
    // fn master_serial_task(ctx: master_serial_task::Context) {
    //     // clear interrupt flag

    //     if let Ok(byte) = ctx.local.master_serial.read(){
    //         rprintln!("Received byte: {:02x}", byte);
    //         ctx.local.rx_buffer[*ctx.local.rx_index] = byte;
    //         *ctx.local.rx_index += 1;
    //     }

    //     if *ctx.local.rx_index >= 4 {
    //         rprintln!("Full command received: {:02x?}", &ctx.local.rx_buffer[..*ctx.local.rx_index]);
    //         *ctx.local.rx_index = 0;
    //         ctx.local.master_serial.write(b'O').unwrap();

    //     }
    // }

  
    #[task(shared = [six_dof], priority = 1)]
    async fn motor(mut ctx: motor::Context) {
        let target_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        loop {
            ctx.shared.six_dof.lock(|ee| {
                ee.set_target(target_angles);
            })
        }
    }


    #[task(
        binds = TIM3,
        local = [last_time: u32 = 0, counter: u32 = 0, led, timer3, control_dt_accum: f32 = 0.0],
        shared = [six_dof, i2c_bus],
        priority = 2
    )]
    fn motor_velocity(mut ctx: motor_velocity::Context) {
        ctx.local.timer3.clear_interrupt(Event::Update);

        let now = Mono::now().duration_since_epoch().to_micros() as u32;
        let dt = if *ctx.local.last_time == 0 {
            0.00002
        } else {
            let raw = (now - *ctx.local.last_time) as f32 / 1_000_000.0;
            if raw <= 0.0 { 0.00002 } else { raw }
        };

        ctx.shared.six_dof.lock(|m| {
            if *ctx.local.counter >= 100 {
                m.set_target([90.0, 90.0, 90.0, 90.0 , 90.0, 90.0]);
            }
            let i2cBus = ctx.shared.i2c_bus.lock(|f| {            
                m.step_position_loop(dt, f)
});

        });


        *ctx.local.control_dt_accum += dt;
        *ctx.local.last_time = now;

        // // Read encoders and run the position PID loop.
        // (&mut ctx.shared.six_dof, &mut ctx.shared.i2c_bus).lock(|ee, i2c| {
        //     if let Err(e) = ee.step_position_loop(dt, i2c) {
        //         rprintln!("encoder read/PID update failed: {:?}", e);
        //     }
        // });

        *ctx.local.counter += 1;
        if *ctx.local.counter >= 50 {
            ctx.local.led.toggle();

            *ctx.local.counter = 0;
        }

    }
}
