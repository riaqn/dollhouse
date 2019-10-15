#![no_std]
#![no_main]

extern crate panic_abort;

use arrayvec::ArrayVec;

use embedded_hal::digital::v2::OutputPin;

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

#[entry]
fn main() -> ! {

    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(64.mhz())
        .pclk1(32.mhz()).freeze(&mut flash.acr);  

    let iwdg = dp.IWDG;
    let mut wd = IndependentWatchdog::new(iwdg);
    wd.start(2000.ms());

    let mut gpiob = dp.GPIOB.split(&mut rcc.apb2);
    let mut gpioc = dp.GPIOC.split(&mut rcc.apb2);


    let mut switch = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);

    switch.set_low();
    
    let mut delay = delay::Delay::new(cp.SYST, clocks);
    
    let mut one = gpiob
        .pb9
        .into_open_drain_output(&mut gpiob.crh)
        .downgrade();

    let mut wire = OneWire::new(&mut one, false);

    if wire.reset(&mut delay).is_err() {
        panic!("missing pullup or error on line");
    }

    const N_SENS : usize = 2;
    const TARGET : f32 = 40.0;
    const HISTORY : usize = 1024;

    let mut vec = ArrayVec::<[_; N_SENS]>::new();
    let mut his : [bool; HISTORY] = [false; HISTORY];
    let mut j = 0;
    let mut sum = 0;

    // search for devices
    let mut search = DeviceSearch::new();
    while let Some(device) = wire.search_next(&mut search, &mut delay).unwrap() {
        match device.address[0] {
            ds18b20::FAMILY_CODE => {
                vec.push(DS18B20::new(device).unwrap());
            },
            _ => {
                panic!("unrecognized device {}", device);
            }
        }
    }

    assert_eq!(vec.len(), N_SENS);
    
    loop {
        let mut heat = true;
        for i in 0..vec.len() {
                // request sensor to measure temperature
                let resolution = vec[i].measure_temperature(&mut wire, &mut delay).unwrap();
                
                // wait for compeltion, depends on resolution 
                delay.delay_ms(resolution.time_ms());
                
                // read temperature
                let temperature = vec[i].read_temperature(&mut wire, &mut delay).unwrap();

                let (d, f) = split_temp(temperature);
                let t : f32 = d as f32 + f as f32 / 10000 as f32;
                if t >= TARGET {
                    heat = false;
                }
        }
        let old = his[j];
        if heat {
            switch.set_high();
            his[j] = true;
            sum += 1;
        } else {
            switch.set_low();
            his[j] = false;
            sum += 0;
        }  
        j += 1;
        if j == HISTORY {
            j = 0;
        }
        if old {
            sum -= 1;
        }
        if (sum as f32) / (HISTORY as f32) > 0.9 {
            panic!("heating for too long");
        }
        wd.feed();
    }
}
