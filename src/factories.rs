use stm32f1xx_hal::gpio::{gpiob, gpioa, gpioc, Output, PushPull};
use stm32f1xx_hal::{
    pac, rcc, prelude::*,
    timer::{Event},
    i2c::{I2c, Mode},
    serial::{Serial, Config, Rx, Tx},
    gpio::{ErasedPin},
};
use stm32f1xx_hal::{self as hal, afio};

use crate::drivers::TMC2209::TmcDriver;
use crate::drivers::AS6500::As5600;
use crate::drivers::PCF::pcf8574;
use crate::control::LimitSwitch::LimitSwitch;
use crate::control::pid::PidController;
use crate::drivers::TCA::TCA9548A;
use crate::control::motor::{Motor as MT, MotorType, MotorConfig};



pub fn make_peripherals() -> ([TmcDriver; 4], [As5600<stm32f1xx_hal::i2c::I2c<stm32f1xx_hal::pac::I2C1>>; 6], [PidController; 6], LimitSwitch<stm32f1xx_hal::i2c::I2c<stm32f1xx_hal::pac::I2C1>,4>) {
    let mut drivers: [TmcDriver; 4] = core::array::from_fn(|i| TmcDriver::new(i as u8, 200, 256));

    let mut encoders: [As5600<stm32f1xx_hal::i2c::I2c<stm32f1xx_hal::pac::I2C1>>; 6] = core::array::from_fn(|i| As5600::new(i as u8));
    let pids: [PidController; 6] = core::array::from_fn(|i| PidController::new(1.0, 0.1, 0.1, i as u8));
    let pcf = crate::drivers::PCF::pcf8574::<stm32f1xx_hal::i2c::I2c<stm32f1xx_hal::pac::I2C1>>::new(crate::config::PCF8574_I2C_ADDR);
    let switch = crate::control::LimitSwitch::LimitSwitch::<stm32f1xx_hal::i2c::I2c<stm32f1xx_hal::pac::I2C1>,4>::new(pcf);
    let mut tca: TCA9548A<stm32f1xx_hal::i2c::I2c<stm32f1xx_hal::pac::I2C1>> = TCA9548A::new(rst_mux);


    (drivers, encoders, pids, switch)
}

pub fn make_gpio(gpioa: pac::GPIOA, gpiob: pac::GPIOB, gpioc: pac::GPIOC, afio: pac::AFIO, rcc: &mut rcc::Rcc) -> (gpioa::Parts, gpiob::Parts, gpioc::Parts, afio::Parts) {
    let mut gpioa = gpioa.split(rcc);
    let mut gpiob = gpiob.split(rcc);
    let mut gpioc = gpioc.split(rcc);
    let mut afio = afio.constrain(rcc);
    (gpioa, gpiob, gpioc, afio)
}

pub fn setup_i2c(i2c: pac::I2c1,scl: gpiob::PB6<gpio::Alternate<gpio::OpenDrain>>, sda: gpiob::PB7<gpio::Alternate<gpio::OpenDrain>>,rst_mux: gpiob::PB5<gpio::Output<gpio::PushPull>>, rcc: &mut rcc::Rcc) {
    let mut i2c = I2c::new(i2c, (scl, sda), Mode::Standard { frequency: 10u32.kHz().into() }, rcc);
    let mut tca: TCA9548A<stm32f1xx_hal::i2c::I2c<pac::I2C1>> = TCA9548A::new(rst_mux);

    (i2c, tca)
}

pub fn setup_master_serial() {
  
}

pub fn make_pins(mut gpiob: gpiob::Parts, mut gpioa: gpioa::Parts, mut gpioc: gpioc::Parts) -> [(ErasedPin<Output<PushPull>>, ErasedPin<Output<PushPull>>); 6] {
    let step_pin0 = gpiob.pb1.into_push_pull_output(&mut gpiob.crl).erase();
    let dir_pin0 = gpiob.pb12.into_push_pull_output(&mut gpiob.crh).erase();

    let step_pin1 = gpioa.pa1.into_push_pull_output(&mut gpioa.crl).erase();
    let dir_pin1 = gpiob.pb14.into_push_pull_output(&mut gpiob.crh).erase();

    let step_pin2 = gpioa.pa8.into_push_pull_output(&mut gpioa.crh).erase();
    let dir_pin2 = gpiob.pb15.into_push_pull_output(&mut gpiob.crh).erase();

    let step_pin3 = gpioa.pa9.into_push_pull_output(&mut gpioa.crh).erase();
    let dir_pin3 = gpiob.pb13.into_push_pull_output(&mut gpiob.crh).erase();

    let step_pin4 = gpiob.pb8.into_push_pull_output(&mut gpiob.crh).erase();
    let dir_pin4 = gpiob.pb3.into_push_pull_output(&mut gpiob.crl).erase();

    let step_pin5 = gpiob.pb9.into_push_pull_output(&mut gpiob.crh).erase();
    let dir_pin5 = gpiob.pb4.into_push_pull_output(&mut gpiob.crl).erase();

    let pins = [
        (step_pin0, dir_pin0),
        (step_pin1, dir_pin1),
        (step_pin2, dir_pin2),
        (step_pin3, dir_pin3),
        (step_pin4, dir_pin4),
        (step_pin5, dir_pin5),
    ];

    pins
}


pub fn make_motors(pins : [(ErasedPin<Output<PushPull>>, ErasedPin<Output<PushPull>>); 6]) -> [MT; 6] {
    const MOTOR_NAMES: [&str; 6] = ["motor 0", "motor 1", "motor 2", "motor 3", "motor 4", "motor 5"];
    let cfg = MotorConfig::default();
    let [p0, p1, p2, p3, p4, p5] = pins;
    let pins_arr = [p0, p1, p2, p3, p4, p5];

    let mut motors: [MT; 6] = core::array::from_fn(|i| {
        let (step_pin, dir_pin) = unsafe {
            core::ptr::read(&pins_arr[i])
        };
        MT::new(
            i as u8,
            MOTOR_NAMES[i],
            if i < 4 { MotorType::Tmc(i as u8) } else { MotorType::Tb6600 },
            step_pin,
            dir_pin,
            MotorConfig { direction_inverted: true, ..cfg },
            10,
            if i < 5 { Some(i as u8) } else { None },
        )
    });

    for motor in motors.iter_mut() {
        motor.init(500.0).unwrap();
    }

    motors
}

pub fn setup_clocks(flash: pac::FLASH, rcc: pac::RCC) -> (hal::flash::Parts, hal::rcc::Rcc) {
    let mut flash = flash.constrain();
        let mut rcc = rcc.freeze(
            rcc::Config::hsi()
            .sysclk(72.MHz())
            .pclk1(36.MHz()),
            &mut flash.acr
        );
    (flash, rcc)
}

pub fn setup_timers(tim2: pac::TIM2, tim3: pac::TIM3, tim4: pac::TIM4, rcc: &mut rcc::Rcc) -> (hal::timer::CounterUs<pac::TIM2>, hal::timer::CounterUs<pac::TIM3>, hal::timer::DelayUs<pac::TIM4>) {
    let mut timer2 = tim2.counter_us(&mut rcc);
    timer2.start(10.micros());
    timer2.listen(Event::Update);

    let mut timer3 = tim3.counter_us(&mut rcc);
    timer3.start(1000.micros());
    timer3.listen(Event::Update);

    let delay = tim4.delay_us(&mut rcc);

    (timer2, timer3, delay)
}

