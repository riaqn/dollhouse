#![no_std]
#![no_main]

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
};
use cortex_m_rt::entry;

use stm32f1xx_hal::stm32::TIM3;
use stm32f1xx_hal::gpio::gpiob::{PB5};
use stm32f1xx_hal::gpio::{Alternate, PushPull};
use stm32f1xx_hal::pwm::{Pins, Pwm, C1};

struct MyChannels(PB5<Alternate<PushPull>>);

impl Pins<TIM3> for MyChannels
{
  const REMAP: u8 = 0b10;
  const C1: bool = false;
  const C2: bool = true;
  const C3: bool = false;
  const C4: bool = false;
  type Channels = (Pwm<TIM3, C1>);
}

#[entry]
fn main() -> ! {
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let mut afio = dp.AFIO.constrain(&mut rcc.apb2);
    let clocks = rcc.cfgr.sysclk(72.mhz()).pclk1(36.mhz())
        .pclk2(72.mhz()).freeze(&mut flash.acr);  

    let iwdg = dp.IWDG;
    let mut wd = IndependentWatchdog::new(iwdg);
    wd.start(2000.ms());

    let mut gpiob = dp.GPIOB.split(&mut rcc.apb2);
    let mut gpioc = dp.GPIOC.split(&mut rcc.apb2);


    let mut delay = delay::Delay::new(cp.SYST, clocks);
    
    let mut one = gpiob
        .pb9
        .into_open_drain_output(&mut gpiob.crh)
        .downgrade();

    let mut wire = OneWire::new(&mut one, false);

    if wire.reset(&mut delay).is_err() {
        panic!("missing pullup or error on line");
    }

    let b5 = gpiob.pb5.into_alternate_push_pull(&mut gpiob.crl);

    let mut pwm =  dp.TIM3.pwm(
                            MyChannels(b5),
                            &mut afio.mapr,
                            100.hz(),
                            clocks,
                            &mut rcc.apb1
                            );
    
    pwm.set_duty(0);
    pwm.enable();
    
    const N_SEN : usize = 2;
    const TARGET : i16 = 40;
    const N_HIS: usize = 1024;
    
    let max_duty : u16 = pwm.get_max_duty();

    let mut sen = ArrayVec::<[_; N_SEN]>::new();
    let mut his : [u16; N_HIS] = [max_duty / 2; N_HIS];
    let mut j = 0;
    let mut sum_duty : u32 = (max_duty / 2) as u32 * N_HIS as u32;

    // search for devices
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
    
    loop {
        let mut heat = true;
        for i in 0..sen.len() {
                // request sensor to measure temperature
                let resolution = sen[i].measure_temperature(&mut wire, &mut delay).unwrap();
                
                // wait for compeltion, depends on resolution 
                delay.delay_ms(resolution.time_ms());
                
                // read temperature
                let temperature = sen[i].read_temperature(&mut wire, &mut delay).unwrap();

                let (d, _) = split_temp(temperature);
                if d >= TARGET {
                    heat = false;
                }
        }
        let old = his[j];
        let duty : u16 = (sum_duty / N_HIS as u32) as u16 + (max_duty / 8);
        if duty > max_duty {
            panic!("high duty for too long, dangerous");
        }
        if heat {
            pwm.set_duty(duty);
            his[j] = duty;
            sum_duty += duty as u32;
        } else {
            pwm.set_duty(0);
            his[j] = 0;
            sum_duty += 0;
        }
        sum_duty -= old as u32;
        j += 1;
        if j == N_HIS {
            j = 0;
        }
        wd.feed();
    }
}