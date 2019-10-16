#![no_std]
#![no_main]
#![feature(proc_macro_hygiene)]

#[cfg(feature = "semihosting")]
extern crate panic_semihosting;

#[cfg(not(feature = "semihosting"))]
extern crate panic_abort;


use arrayvec::ArrayVec;

use onewire::{
    OneWire,
    DeviceSearch,
    ds18b20,
};

use self::ds18b20::{DS18B20, split_temp};

use stm32f1xx_hal::{
    prelude::*,
    pac,
    delay,
    watchdog::{IndependentWatchdog},
    stm32::TIM3,
    gpio::{Alternate, PushPull, gpiob::PB4},
    pwm::{Pins, Pwm, C1},
    i2c::{BlockingI2c, DutyCycle, Mode}
};
use cortex_m_rt::entry;
use cortex_m_semihosting::hio;
use core::fmt::Write;

use pid::Pid;

use embedded_graphics::{fonts::Font6x8,
                        pixelcolor::BinaryColor,
                        prelude::*
};

use ssd1306::{prelude::*,
                Builder
};

struct MyChannels(PB4<Alternate<PushPull>>);

impl Pins<TIM3> for MyChannels
{
  const REMAP: u8 = 0b10;
  const C1: bool = false;
  const C2: bool = true;
  const C3: bool = false;
  const C4: bool = false;
  type Channels = (Pwm<TIM3, C1>);
}

const N_SEN : usize = 2;
const TARGET : f32 = 40.0;
const N_HIS : usize = 256; // roughly 4 mins
const N_OVH : usize = 200; // 200/256 = 80%. If we have been averagely heating at 80%, we are wrong

const KP : f32 = 0.0;
const KI : f32 = 0.0;
const KD : f32 = 0.0;
const LP : f32 = 100.0;
const LI : f32 = 100.0;
const LD : f32 = 100.0;

#[entry]
fn main() -> ! {
    #[cfg(feature = "semihosting")]
    let mut stdout = match hio::hstdout() {
        Ok(fd) => fd,
        Err(()) => panic!("failed to init semihosting stdout"),
    };

// Debug version
#[cfg(feature = "semihosting")]
macro_rules! print {
    ($( $args:expr ),*) => { write!(stdout, $( $args ),* ); }
}

// Non-debug version
#[cfg(not(feature = "semihosting"))]
macro_rules! print {
    ($( $args:expr ),*) => {}
}

    print!("program starting").unwrap();

    //common init
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(72.mhz()).pclk1(36.mhz())
        .pclk2(72.mhz()).freeze(&mut flash.acr);  

    let gpioa = dp.GPIOA.split(&mut rcc.apb2);
    let mut gpiob = dp.GPIOB.split(&mut rcc.apb2);
    let mut _gpioc = dp.GPIOC.split(&mut rcc.apb2);
    let mut afio = dp.AFIO.constrain(&mut rcc.apb2);

    // DISPLAY
    let b8 = gpiob.pb8.into_alternate_open_drain(&mut gpiob.crh);
    let b9 = gpiob.pb9.into_alternate_open_drain(&mut gpiob.crh);

    let i2c = BlockingI2c::i2c1(
        dp.I2C1,
        (b8,b9),
        &mut afio.mapr,
        Mode::Fast {
            frequency: 400_000,
            duty_cycle: DutyCycle::Ratio2to1
        },
        clocks,
        &mut rcc.apb1,
        1000,10,1000,1000
    );

    let mut disp:GraphicsMode<_> = Builder::new().connect_i2c(i2c).into();

    disp.init().unwrap();

    // PWM
    let (_a15, b3, b4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);

    let mut delay = delay::Delay::new(cp.SYST, clocks);

    let b4 = b4.into_alternate_push_pull(&mut gpiob.crl);

    let mut pwm =  dp.TIM3.pwm(
                            MyChannels(b4),
                            &mut afio.mapr,
                            100.hz(),
                            clocks,
                            &mut rcc.apb1
                            );
    
    pwm.set_duty(0);
    pwm.enable();
    let max_duty : u16 = pwm.get_max_duty();

    // sensors
    let mut sen = ArrayVec::<[_; N_SEN]>::new();
    let mut b3 = b3
        .into_open_drain_output(&mut gpiob.crl)
        .downgrade();

    let mut wire = OneWire::new(&mut b3, false);

    if wire.reset(&mut delay).is_err() {
        panic!("missing pullup or error on line");
    }

    let mut search = DeviceSearch::new();
    while let Some(device) = wire.search_next(&mut search, &mut delay).unwrap() {
        match device.address[0] {
            ds18b20::FAMILY_CODE => {
                sen.push(DS18B20::new(device).unwrap());
            },
            _ => {
                panic!("unrecognized device {}", device);
            }
        }
    }

    assert_eq!(sen.len(), N_SEN);
    
    // PID
    let mut pid = Pid::new(KP, KI, KD, LP, LI, LD);
    pid.update_setpoint(TARGET);
    let mut his : [u16; N_HIS] = [0; N_HIS];
    let mut sum_his = 0;
    let mut j = 0;

    // watchdog
    let iwdg = dp.IWDG;
    let mut wd = IndependentWatchdog::new(iwdg);
    wd.start(2000.ms());

    // loops
    loop {
        let mut max = 0.0;
        for i in 0..sen.len() {
                // request sensor to measure temperature
                let resolution = sen[i].measure_temperature(&mut wire, &mut delay).unwrap();
                
                // wait for compeltion, depends on resolution 
                delay.delay_ms(resolution.time_ms());
                
                // read temperature
                let temperature = sen[i].read_temperature(&mut wire, &mut delay).unwrap();

                let (d, f) = split_temp(temperature);
                let t = d as f32 + f as f32 / 10000.0;
                if t > max {
                    max = t;
                }
        }
        let output = pid.next_control_output(max);
        let mut duty = output.output as u16;
        if duty > max_duty {
            duty = max_duty;
        }
        pwm.set_duty(duty);

        // overheat protection
        sum_his -= his[j];
        his[j] = duty;
        sum_his += his[j];

        j += 1;
        if j == N_HIS {
            j = 0;
        }

        if sum_his > max_duty * N_OVH as u16 {
            panic!("overheat protection");
        }

        //draw something
        disp.draw(Font6x8::render_str("holy shit").stroke(Some(BinaryColor::On)).into_iter());
        disp.flush().unwrap();

        //feed the dog so it doesn't bit                
        wd.feed();
    }
}