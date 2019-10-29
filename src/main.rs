#![no_std]
#![no_main]
#![feature(proc_macro_hygiene)]

use cortex_m_semihosting::hio;

use arrayvec::{ArrayVec};

use onewire::{
    OneWire,
    DeviceSearch,
    ds18b20::{
        MeasureResolution,
        DS18B20,
        split_temp
    },
};


use stm32f1xx_hal::{
    prelude::*,
    pac,
    pac::I2C1,
    delay,
    watchdog::{IndependentWatchdog},
    stm32::{TIM3, DWT},
    gpio::{Alternate, PushPull, gpiob::{PB4, PB6, PB7}, OpenDrain},
    pwm::{Pins, Pwm, C1},
    i2c::{BlockingI2c, DutyCycle, Mode},
};
use cortex_m_rt::entry;
use core::fmt::{Write, Arguments};
use core::{f32, i32, u16};
use num_traits::float::FloatCore;

use ssd1306::{prelude::*,
                Builder
};

struct MyChannels(PB4<Alternate<PushPull>>);

impl Pins<TIM3> for MyChannels
{
  const REMAP: u8 = 0b10;
  const C1: bool = true;
  const C2: bool = false;
  const C3: bool = false;
  const C4: bool = false;
  type Channels = (Pwm<TIM3, C1>);
}

const N_SEN : usize = 2;
const TARGET : f32 = 40.0;
const N_HIS : usize = 80;
const N_OVH : usize = 40;
const N_TRY : u32 = 2; // at most try getting temperature twice

const KP : f32 = 0.2;
const KI : f32 = 0.01;
const KD : f32 = 0.8;

const BAL : f32 = 0.10; // balanced at 10% load

use core::sync::atomic::{self, Ordering};
use core::panic::PanicInfo;

static mut PWM : Option<Pwm<TIM3, C1>> = None;
static mut WD : Option<IndependentWatchdog> = None;
static mut DISP : Option<TerminalMode<I2cInterface<BlockingI2c<I2C1,(PB6<Alternate<OpenDrain>>, PB7<Alternate<OpenDrain>>)>>>> = None;
static mut HIO : Option<hio::HStdout> = None;

unsafe fn print(args : Arguments) {
    match &mut DISP {
        Some(disp) => {
            disp.write_fmt(args).unwrap();
        },
        None => {},
    }
    match &mut HIO {
        Some(hio) => {
            hio.write_fmt(args).unwrap();
        },
        None => {},
    }
}

macro_rules! print {
    ($($arg:tt)*) => (unsafe{print(format_args!($($arg)*))});
}

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    print!("{}", info);
    unsafe {
    match &mut PWM {
        Some(pwm) => {
            pwm.set_duty(0);
            pwm.disable();
        },
        None => {},
    };
    

    loop {
        atomic::compiler_fence(Ordering::SeqCst);
        match &mut WD {
            Some(wd) => {wd.feed()},
            None => {}
        }
    }
    }
}

#[entry]
fn main() -> ! {
    //common init
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(64.mhz()).pclk1(32.mhz())
        .pclk2(64.mhz()).freeze(&mut flash.acr);  

    let gpioa = dp.GPIOA.split(&mut rcc.apb2);
    let mut gpiob = dp.GPIOB.split(&mut rcc.apb2);
    let mut _gpioc = dp.GPIOC.split(&mut rcc.apb2);
    let mut afio = dp.AFIO.constrain(&mut rcc.apb2);

    let mut delay = delay::Delay::new(cp.SYST, clocks);

    // watchdog
    let iwdg = dp.IWDG;
    let mut wd = IndependentWatchdog::new(iwdg);
    wd.start(2000.ms());
    unsafe{WD = Some (wd)}
    print!("watchdog init.\n");

    // DISPLAY
    let b6 = gpiob.pb6.into_alternate_open_drain(&mut gpiob.crl);
    let b7 = gpiob.pb7.into_alternate_open_drain(&mut gpiob.crl);

    let i2c = BlockingI2c::i2c1(
        dp.I2C1,
        (b6,b7),
        &mut afio.mapr,
        Mode::Fast {
            frequency: 400_000,
            duty_cycle: DutyCycle::Ratio2to1
        },
        clocks,
        &mut rcc.apb1,
        1000,10,1000,1000
    );

    let mut disp : TerminalMode<_> = Builder::new().connect_i2c(i2c).into();

    disp.init().unwrap();
    disp.clear().unwrap();

    unsafe {DISP = Some (disp)}

    print!("display init.\n");

    //HIO
    #[cfg(feature = "semihosting")]
    match hio::hstdout() {
        Ok(hio) => {unsafe {HIO = Some(hio)}; print!("HIO init.\n")},
        Err(_) => {print!("HIO init failed \n")}
    }

    // PWM
    let (_a15, _b3, b4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);

    let b4 = b4.into_alternate_push_pull(&mut gpiob.crl);

    let mut pwm = dp.TIM3.pwm(
                            MyChannels(b4),
                            &mut afio.mapr,
                            100.hz(),
                            clocks,
                            &mut rcc.apb1
                            );

    pwm.set_duty(0);
    pwm.enable();
    let max_duty : u16 = pwm.get_max_duty();
    let max_duty_f = max_duty as f32;
    unsafe {PWM = Some (pwm)};

    print!("PWM init.\n");

    // sensors
    let mut sen = ArrayVec::<[_; N_SEN]>::new();
    let mut b9 = gpiob.pb9
        .into_open_drain_output(&mut gpiob.crh)
        .downgrade();

    let mut wire = OneWire::new(&mut b9, false);

    if wire.reset(&mut delay).is_err() {
        panic!("missing pullup or error on line");
    }

    let mut search = DeviceSearch::new();
    while let Some(device) = wire.search_next(&mut search, &mut delay).unwrap() {
        sen.push(DS18B20::new::<()>(device).unwrap());
    }


    assert_eq!(sen.len(), N_SEN);

    print!("sensors init.\n");
    
    // history
    let mut loads : [u16; N_HIS] = [0; N_HIS];
    let mut temps : [i32; N_HIS] = [0; N_HIS];
    let mut sum_loads : u32 = 0;
    let mut sum_temps : i32 = 0;
    let mut j = 0;
    let mut tr = 0;

    // PID
    let mut integral = 0.0;
    let mut prev = None;

    //DWT
    let mut dwt = cp.DWT;
    dwt.enable_cycle_counter();
    let freq = clocks.sysclk().0 as f32;

    // loops
    loop {
        //feed the dog so it doesn't bit            
        unsafe  {    
        match &mut WD {
            Some (wd) => {wd.feed()},
            None => {panic!("where did the dog go?")}
        }
        }
        tr += 1;
        if tr > N_TRY {panic!("max try reached")}

        let time = DWT::get_cycle_count();
        let mut temp = i32::MIN;
        let multiplier = 10000;
        let multiplier_f = multiplier as f32;
        let mut err = false; 
        for i in 0..sen.len() {
                // request sensor to measure temperature
                match sen[i].measure_temperature(&mut wire, &mut delay) {
                    Ok (_) => {},
                    Err (_) => {err = true;break}
                }
        }
        if err {continue};

        // wait for compeltion, depends on resolution 
        delay.delay_ms(MeasureResolution::TC.time_ms());
                           
        for i in 0..sen.len() {
                // read temperature
                match sen[i].read_temperature(&mut wire, &mut delay) {
                    Ok (temperature) => {   let (d, f) = split_temp(temperature);
                                            let t = d as i32 * multiplier + f as i32;
                                            if t > temp { temp = t;}
                                        },
                    Err (_) => {err = true; break}
                }
        }
        if err {continue};

        tr = 0;

        let error = TARGET - temp as f32 / multiplier as f32;
        let p = KP * error;

        let (i,d) = match prev {
            None => (0.0, 0.0),
            Some ((prev_time, prev_temp)) => {
                let diff_time_ = time.wrapping_sub(prev_time);
                if diff_time_ == 0 {
                    panic!("diff time is zero!");
                }
                let diff_time = diff_time_ as f32 / freq;
                let diff_temp = temp.checked_sub(prev_temp).unwrap() as f32 / multiplier as f32;

                integral += diff_time * error * KI;
                if integral.abs() > 0.1 {
                    integral = 0.1 * integral.signum();
                }
                let d = -diff_temp / diff_time * KD;
                (integral, d)
            }
        };
        prev = Some((time, temp));
        let output = p + i + d;
        let duty_ : f32 = (output + BAL) * max_duty as f32;
        let duty : u16;

        if duty_ < 0.0 {
            duty = 0;
        } else if duty_ > max_duty as f32 {
            duty = max_duty
        } else {
            duty = duty_ as u16;
        }

        unsafe {
            match &mut PWM {
                Some (pwm) => {pwm.set_duty(duty)},
                None => {panic!("where did the PWM go?")}
            }
        }


        // overheat protection
        sum_loads -= loads[j] as u32;
        loads[j] = duty;
        sum_loads += loads[j] as u32;

        sum_temps -= temps[j] as i32;
        temps[j] = temp;
        sum_temps += temps[j] as i32;

        j += 1;
        if j == N_HIS {
            j = 0;
        }

        if sum_loads > max_duty as u32 * N_OVH as u32 {
            panic!("overheat protection");
        }

        let mut min_temp = i32::MAX;
        let mut max_temp = i32::MIN;
        let mut min_load = u16::MAX;
        let mut max_load = u16::MIN;
        for i in 0..N_HIS {
            if temps[i] < min_temp {
                min_temp = temps[i];
            }
            if loads[i] < min_load{
                min_load = loads[i];
            }
            if temps[i] > max_temp {
                max_temp = temps[i];
            }
            if loads[i] > max_load {
                max_load = loads[i];
            }
        }

        print!("TEMP {:.3} p {:.2} i {:.2} d {:.2} sum {:.2}\n", temp, p, i, d, output);
        print!("TEMP {:.3}/{:.3}/{:.3}\n", min_temp as f32 / multiplier_f, sum_temps as f32 / N_HIS as f32 / multiplier_f, max_temp as f32 / multiplier_f);
        print!("LOAD {:.2}/{:.2}/{:.2}\n", min_load as f32 / max_duty_f, sum_loads as f32 / N_HIS as f32 / max_duty_f, max_load as f32 / max_duty_f);

 
    }
}