#![no_std]
#![no_main]
mod control;
mod drivers;
mod config;

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
use control::pid::PidController;    
use drivers::TMC2209::{MotorCommand,MotorResponse, TmcDriver};
use drivers::AS6500::{As5600, As5600Error, MagnetStatus};
use drivers::TCA::TCA9548A;


const SYSCLK_HZ: u32 = 72_000_000;
const BAUD: u32 = 9_600;
const BIT_CYCLES: u32 = SYSCLK_HZ / BAUD;       // ≈ 7500 cycles per bit (72MHz/9600)
const BYTE_TIME_CYCLES: u32 = BIT_CYCLES * 10;  // 1 start + 8 data + 1 stop ≈ 75_000 cycles ~1.04 ms
const BYTE_CYCLES: u32 = BIT_CYCLES * 10;        // start+8+stop ~75k cycles ~1.04ms

mod halfduplex;
use halfduplex::HalfDuplexSerial;
// use utils::{write_register, read_register, configure_motor};

use heapless::{spsc::Queue, LinearMap};
 

// ============================================================================
// RTIC V1 APP
// ============================================================================

#[app(device = stm32f1xx_hal::pac, peripherals = true, dispatchers = [EXTI0, EXTI1])]
mod app {

    use rtic_monotonics::{Monotonic, systick_monotonic};
    use rtic_sync::{channel::*, make_channel};


    use crate::control::parts::{EndEffector, ControlMode, ControlLoop};

    use super::*;

    #[shared]
    struct Shared {
        command_queue: Queue<(u8, MotorCommand), 8>,
        response_queue: Queue<(u8, MotorResponse), 8>,
        encoder_value: LinearMap<u8, Queue<(f32, u16), 8>, 6>,
        encoders: [As5600<stm32f1xx_hal::i2c::I2c<stm32f1xx_hal::pac::I2C1>>; 6],
        end_effector: EndEffector,


    }

    #[local]
    struct Local {
        half_serial: HalfDuplexSerial,
        timer: CounterUs<pac::TIM2>,
        timer3: CounterUs<pac::TIM3>,
        led: PC13<Output<PushPull>>,
        drivers: [TmcDriver; 4],
        i2c: stm32f1xx_hal::i2c::I2c<stm32f1xx_hal::pac::I2C1>,
        sender: Sender<'static, (u8,u32 ), 8>,
        receiver: Receiver<'static, (u8, u32), 8>,
        tca: TCA9548A<stm32f1xx_hal::i2c::I2c<stm32f1xx_hal::pac::I2C1>>,
        master_serial: stm32f1xx_hal::serial::Serial<pac::USART2>,

    }

    systick_monotonic!(Mono);


    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {

        let mut flash = ctx.device.FLASH.constrain();
        let mut rcc = ctx.device.RCC.freeze(
            rcc::Config::hsi()
            .sysclk(72.MHz())
            .pclk1(36.MHz()),
            &mut flash.acr
        );

        let (sender, receiver) = make_channel!((u8, u32), 8);
        
      
        let drivers: [TmcDriver; 4] = core::array::from_fn(|i| TmcDriver::new(i as u8, 200, 256));
        let mut encoders: [As5600<stm32f1xx_hal::i2c::I2c<stm32f1xx_hal::pac::I2C1>>; 6] = core::array::from_fn(|i| As5600::new(i as u8));
        let pids: [control::pid::PidController; 6] = core::array::from_fn(|i| control::pid::PidController::new(1.0, 0.0, 0.0, i as u8));

        rtt_init_print!();
        rprintln!("=====initializing RTIC========");

        let mut gpiob = ctx.device.GPIOB.split(&mut rcc);
        let mut gpioc = ctx.device.GPIOC.split(&mut rcc);
        let mut gpioa = ctx.device.GPIOA.split(&mut rcc);


        let mut _afio = ctx.device.AFIO.constrain(&mut rcc);
        // Disable JTAG to free up PB3, PB4, PA15 for GPIO
        // let (pa15, pb3, pb4) = _afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);
        let led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);

        let step_pin0 = gpiob.pb1.into_push_pull_output(&mut gpiob.crl).erase();
        let dir_pin0 = gpiob.pb12.into_push_pull_output(&mut gpiob.crh).erase();

        let rst_mux = gpiob.pb5.into_push_pull_output(&mut gpiob.crl).erase();

            let sda = gpiob.pb7.into_alternate_open_drain(&mut gpiob.crl);
        let scl = gpiob.pb6.into_alternate_open_drain(&mut gpiob.crl);
        let mut i2c = I2c::new(ctx.device.I2C1, (scl, sda), Mode::Standard { frequency: 10u32.kHz().into() }, &mut rcc);

        let rx2 = gpioa.pa3.into_floating_input(&mut gpioa.crl);
        let tx2 = gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl);

        let mut tca: TCA9548A<stm32f1xx_hal::i2c::I2c<stm32f1xx_hal::pac::I2C1>> = TCA9548A::new(rst_mux);
        rprintln!("Resetting TCA9548A...");
        tca.reset(|| {cortex_m::asm::delay(100000);});
        rprintln!("TCA9548A reset complete");

        
        

        let step_pin1 =  gpiob.pb0.into_push_pull_output(&mut gpiob.crl).erase();
        let dir_pin1 = gpiob.pb13.into_push_pull_output(&mut gpiob.crh).erase();

        // let step_pin2 = gpioa.pa1.into_push_pull_output(&mut gpioa.crl).erase();
        // let dir_pin2 = gpiob.pb14.into_push_pull_output(&mut gpiob.crh).erase();

        // let step_pin3 =gpioa.pa8.into_push_pull_output(&mut gpioa.crh).erase();
        // let dir_pin3 = gpiob.pb15.into_push_pull_output(&mut gpiob.crh).erase();

        // let step_pin4 = gpiob.pb8.into_push_pull_output(&mut gpiob.crh).erase();
        // let dir_pin4 = pb3.into_push_pull_output(&mut gpiob.crl).erase();


        // let step_pin5 = gpiob.pb9.into_push_pull_output(&mut gpiob.crh).erase();
        // let dir_pin5 = pb4.into_push_pull_output(&mut gpiob.crl).erase();


        rprintln!("I2C live probe start...");
for attempt in 0..200 {
    // try write to TCA (select channel 0)
    let channel_byte = 1u8; // select channel 0
    match tca.select_channel(&mut i2c, channel_byte, || {cortex_m::asm::delay(100000);}) {
        Ok(()) => {
            // small settle
            cortex_m::asm::delay(72_000);
            // read a register on AS5600 to confirm device visible (safe read of STATUS)
            let mut buf = [0u8; 1];
            if i2c.write_read(crate::config::AS5600_I2C_ADDR, &[crate::drivers::AS6500::register::STATUS], &mut buf).is_ok() {
                rprintln!("probe {}: TCA OK, AS5600 STATUS {:02X}", attempt, buf[0]);
            } else {
                rprintln!("probe {}: TCA OK, AS5600 no ack", attempt);
            }
        }
        Err(err) => {
            rprintln!("probe {}: TCA write FAILED: {:?}", attempt, err);
        }
    }
    // small pause between attempts
    cortex_m::asm::delay(360_000); // ~5ms
}
rprintln!("I2C live probe end");

        let tx_pin = gpiob.pb10.into_alternate_open_drain(&mut gpiob.crh);
        let rx_pin = gpiob.pb11.into_floating_input(&mut gpiob.crh);

        rprintln!("init mono");
        Mono::start(ctx.core.SYST, SYSCLK_HZ);

            

        rprintln!("init uart");
        let apb1enr = unsafe { &(*pac::RCC::ptr()).apb1enr() };
        apb1enr.modify(|_, w| w.usart3en().set_bit());

        // Configure baud rate manually (must do before HalfDuplexSerial::new enables USART)
        let usart3 = unsafe { &*pac::USART3::ptr() };
        // BRR = PCLK1/baudrate = 36,000,000/9600 = 3750
        usart3.brr().write(|w| unsafe { w.bits(3750) });

        rprintln!("init timer");
       let mut timer = ctx.device.TIM2.counter_us(&mut rcc);
        timer.start(10.micros()).unwrap();

        timer.listen(Event::Update);

        let delay= ctx.device.TIM4.delay_us(&mut rcc);
        rprintln!("init delay");
        
        let cfg = MotorConfig { steps_per_rev: 200, microsteps: 256, max_velocity: 2000.0, direction_inverted: false };

        use rtic_monotonics::fugit::ExtU32;
        let mut timer3 = ctx.device.TIM3.counter_us(&mut rcc);
        timer3.start(20.micros()).unwrap();
        timer3.listen(Event::Update);

            let mut m0 = MT::new(0,"motor 1", MotorType::Tmc(0), step_pin0, dir_pin0, cfg.clone(), 10, Some(0));

            let mut m1 = MT::new(1,"motor 2", MotorType::Tmc(1), step_pin1, dir_pin1, cfg.clone(), 10, Some(1));

            // let mut m2 = MT::new(2,"motor 3",MotorType::Tmc(2), step_pin2, dir_pin2, cfg.clone(), 20);

            // let mut m3 =MT::new(3,"motor 4",MotorType::Tmc(3), step_pin3, dir_pin3, cfg.clone(), 20);

            // let mut m4 =MT::new(4,"motor 5",MotorType::Tb6600, step_pin4, dir_pin4, cfg.clone(), 20);

            // let mut m5 =MT::new(5,"motor 6",MotorType::Tb6600, step_pin5, dir_pin5, cfg.clone(), 20);

            // Initialize motors
            m0.init(500.0).unwrap();
            m1.init(500.0).unwrap();

            // Create EndEffector - it owns the motors
            let end_effector = EndEffector::new(m0, m1, pids[0 as usize].clone(), pids[1 as usize].clone(), 1.0, 1.0, ControlMode::Position(ControlLoop::Closed));



            
    
        let command_queue = Queue::new();
        let response_queue = Queue::new();

        let serial = HalfDuplexSerial::new(100000, delay);

        let config = Config::default()
            .baudrate(115_200.bps());
        let master_serial = Serial::new(ctx.device.USART2,(tx2, rx2),config , &mut rcc);

        // Pre-initialize encoder queues for all encoders
        let mut encoder_value: LinearMap<u8, Queue<(f32, u16), 8>, 6> = LinearMap::new();
        for i in 0..6 {
            let _ = encoder_value.insert(i as u8, Queue::new());
        }

        rprintln!("========initialized========\n");
        // // // quick full-duplex test: configure TX push-pull, RX floating input and simple send/read
        // let usart3 = unsafe { &*pac::USART3::ptr() };
        // usart3.cr3().modify(|_, w| w.hdsel().clear_bit()); // disable half-duplex for this test
        // // ensure UE=1, TE=1, RE=1
        // usart3.cr1().modify(|_, w| w.ue().set_bit().te().set_bit().re().set_bit());
        // cortex_m::asm::delay(BIT_CYCLES as u32 * 10);

        // // write single byte:
        // while usart3.sr().read().txe().bit_is_clear() {}
        // usart3.dr().write(|w| unsafe { w.dr().bits(0x05) });
        // // wait for TC
        // while usart3.sr().read().tc().bit_is_clear() {}
        // // read any incoming bytes for a short window
        // let mut got = Vec::<u8, 16>::new();
        // let mut t = 0u32;
        // while t < 50_000 {
        // if usart3.sr().read().rxne().bit_is_set() {
        //     let b = (usart3.dr().read().bits() & 0xFF) as u8;
        //     got.push(b).ok();
        // }
        // t += 1;
        // }
        // rprintln!("Full-duplex got: {:02X?}", got.as_slice());
         rprintln!("I2C scan:");
    // Try the multiplexer address
    let mut ok = false;
    if i2c.write(crate::config::TCA9548A_I2C_ADDR, &[]).is_ok() { ok = true; }
    rprintln!(" TCA9548A @ 0x{:02X} -> {}", crate::config::TCA9548A_I2C_ADDR, if ok { "ACK" } else { "NO ACK" });

    // try AS5600 address
    let mut as_ok = false;
    if i2c.write(crate::config::AS5600_I2C_ADDR, &[]).is_ok() { as_ok = true; }
    rprintln!(" AS5600 @ 0x{:02X} -> {}", crate::config::AS5600_I2C_ADDR, if as_ok { "ACK (direct)" } else { "NO ACK (direct)" });

            unsafe {
                cortex_m::peripheral::NVIC::unmask(pac::Interrupt::TIM3);
                cortex_m::peripheral::NVIC::unmask(pac::Interrupt::TIM2);
            }

    for encoder in encoders.iter_mut() {
        let mag = encoder.read_magnitude(&mut i2c);
        let status = encoder.read_magnet_status(&mut i2c);

        match (mag, status) {
            (Ok(m), Ok(s)) => {
                rprintln!("Encoder {} magnitude: {}, status: {:?}", encoder.mux_channel(), m, s);
            }
            _ => {
                rprintln!("Encoder {} read error", encoder.mux_channel());
            }
        }
    }



        (
            Shared {
                response_queue,
                command_queue,
                encoder_value,
                encoders,
                end_effector,

            },
            Local {
                half_serial: serial,
                timer,
                timer3,
                led,
                drivers,
                sender,
                receiver,
                i2c,
                tca,
                master_serial,
            },
    )
    }



    #[task( binds = TIM2, local = [timer], shared = [end_effector], priority = 5)]
    fn tmc_manager(mut ctx: tmc_manager::Context) {
        // Clear the interrupt flag
        ctx.local.timer.clear_interrupt(Event::Update);

        // Tick all motors in the end effector
        ctx.shared.end_effector.lock(|ee| {
            let [m1_step, m2_step] = ee.tick_motors();
            if m1_step {
                let _ = ee.motor1.update_from_ramp();
            }
            if m2_step {
                let _ = ee.motor2.update_from_ramp();
            }
        })
    }

    #[task(local=[drivers, half_serial], shared = [command_queue, response_queue], priority = 1)]
    async fn tmc_command_task(mut ctx: tmc_command_task::Context) {
        let command = ctx.shared.command_queue.lock(|q| {
            q.dequeue()
        });
        match command {
            Some(cmd) => {
                let response = ctx.local.drivers[cmd.0 as usize].execute(cmd.1, ctx.local.half_serial);
                ctx.shared.response_queue.lock(|q| {
                    let _ = q.enqueue((cmd.0, response));
                });
            }
            None => {}
        }
    }

    #[task(binds = USART2, local = [master_serial, rx_buffer: [u8; 4] = [0; 4], rx_index: usize = 0], shared = [encoders], priority = 3)]
    fn master_serial_task(ctx: master_serial_task::Context) {
        // clear interrupt flag

        if let Ok(byte) = ctx.local.master_serial.read(){
            rprintln!("Received byte: {:02x}", byte);
            ctx.local.rx_buffer[*ctx.local.rx_index] = byte;
            *ctx.local.rx_index += 1;
        }

        if *ctx.local.rx_index >= 4 {
            rprintln!("Full command received: {:02x?}", &ctx.local.rx_buffer[..*ctx.local.rx_index]);
            *ctx.local.rx_index = 0;
            ctx.local.master_serial.write(b'O').unwrap();

        }
    }


    #[task(binds = TIM3,local = [last_time : u32 = 0,last_angle: u16 = 0, counter: u32 = 0, i2c, led, change_counter: u32 = 0, state: u8 = 0, timer3, sender, receiver, tca],  shared = [end_effector, encoders], priority = 2)]
    fn motor_velocity(mut ctx: motor_velocity::Context) {
        ctx.local.timer3.clear_interrupt(Event::Update);

        let now = Mono::now().duration_since_epoch().to_micros() as u32;
        let dt = if *ctx.local.last_time == 0 {
            0.00002 // 20 microseconds (timer period) on first call
        } else {
            (now - *ctx.local.last_time) as f32 / 1_000_000.0 // Convert microseconds to seconds
        };

        *ctx.local.last_time = now;

          *ctx.local.counter += 1;
        if *ctx.local.counter >= 50 {
            ctx.local.led.toggle();

            *ctx.local.counter = 0;
        }

        let command = ctx.local.receiver.try_recv().ok();
        if let Some((id, angle)) = command {
            rprintln!("got command for encoder {}: angle {}", id, angle);
        }



        *ctx.local.change_counter += 1;

        if *ctx.local.change_counter >= 100 {
                ctx.shared.end_effector.lock(|ee| {
                    // Access motor1 and motor2 directly

                    ee.set_pitch_roll(200.0, 200.0);

                    // Handle motor1
                    if let Err(_) = ctx.local.tca.select_channel(ctx.local.i2c, ee.motor1.encoder_id.unwrap_or(0), || {cortex_m::asm::delay(100000);}) {
                        rprintln!("Failed to select TCA channel {}", ee.motor1.encoder_id.unwrap_or(0));
                    }
                    let (angle1, angle2) = ctx.shared.encoders.lock(|encoders| {
                        if let Err(_) = ctx.local.tca.select_channel(ctx.local.i2c, ee.motor1.encoder_id.unwrap_or(0), || {cortex_m::asm::delay(100000);}) {
                            rprintln!("Failed to select TCA channel {}", ee.motor1.encoder_id.unwrap_or(0));
                        }
                        let angle1 = encoders[ee.motor1.encoder_id.unwrap_or(0) as usize].read_angle_degrees(ctx.local.i2c).unwrap_or(0.0) as f32;
                        if let Err(_) = ctx.local.tca.select_channel(ctx.local.i2c, ee.motor2.encoder_id.unwrap_or(0), || {cortex_m::asm::delay(100000);}) {
                            rprintln!("Failed to select TCA channel {}", ee.motor2.encoder_id.unwrap_or(0));
                        }
                        let angle2 = encoders[ee.motor2.encoder_id.unwrap_or(0) as usize].read_angle_degrees(ctx.local.i2c).unwrap_or(0.0) as f32;

                        (angle1, angle2)
                    });

                    ee.update_position_control(Some(angle1), Some(angle2), Some(dt));
                    
                });
    }
}

    // #[task(priority = 1, local = [i2c], shared = [encoder_value, encoders])]
    // async fn read_encoder(mut ctx: read_encoder::Context, mut sender: Sender<'static, (u8,u32), 8>) {
    //     Mono::delay(10.millis()).await;
    //     ctx.shared.encoders.lock(|encoders| {
    //        for i in 0..encoders.len() {
    //         if !encoders[0].is_magnet_ok(ctx.local.i2c) {
    //             if i == 0 {
    //             rprintln!("Encoder {} magnet not OK", i);
    //             let res = encoders[0].read_magnitude(ctx.local.i2c).ok();
    //             rprintln!("Encoder {} magnitude: {:?}", i, res);
    //             }
    //             continue;
    //         } else {
    //             rprintln!("Encoder {} magnet OK", i);
    //             let res = encoders[0].read_magnitude(ctx.local.i2c).ok();
    //             rprintln!("Encoder {} magnitude: {:?}", i, res);
    //         }
    //         let now = Mono::now().duration_since_epoch().to_micros() as u32;
    //         let encoder = &mut encoders[i];
    //         let angle1 = encoder.read_angle(ctx.local.i2c).unwrap_or(0) as f32;
    //         //rprintln!("Encoder {} angle: {}", i, angle1);
    //         let angle2 = encoder.read_angle(ctx.local.i2c).unwrap_or(0) as f32;
    //         //rprintln!("Encoder {} angle: {}", i, angle2);
    //         let after = Mono::now().duration_since_epoch().to_micros() as u32;
    //         let dt = (after - now) as f32 / 1_000_000.0;
            
    //         sender.try_send((i as u8, angle2 as u32)).ok();
    //         let speed = (angle2 - angle1) / dt; // in counts per second
    //             let key = i as u8;
    //             let sample = (speed as f32, angle2 as u16);
    //         }
    //     });
    // }
}
