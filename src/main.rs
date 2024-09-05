#![deny(unsafe_code)]
#![no_main]
#![no_std]

use defmt_rtt as _;
use panic_probe as _;

use stm32f3xx_hal as hal;
use stm32f3xx_hal::flash::FlashExt;

use cortex_m_rt::entry;
use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::digital::v2::{OutputPin, StatefulOutputPin, ToggleableOutputPin};
use embedded_time::duration::Extensions as DurationExt;
use hal::gpio::GpioExt;
use hal::pac;
use hal::rcc::RccExt;

#[entry]
fn main() -> ! {
    defmt::debug!("entry");
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut flash.acr);
    let mut delay = hal::delay::Delay::new(cp.SYST, clocks);

    let mut gpiob = dp.GPIOB.split(&mut rcc.ahb);
    let mut led = gpiob
        .pb3
        .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);
    defmt::debug!("done initializing");

    led.set_low().unwrap();

    loop {
        defmt::debug!("toggle");
        led.toggle().unwrap();
        delay.delay_ms(1000.milliseconds());
        // Toggle by hand.
        // Uses `StatefulOutputPin` instead of `ToggleableOutputPin`.
        // Logically it is the same.
        if led.is_set_low().unwrap() {
            defmt::debug!("high");
            led.set_high().unwrap();
        } else {
            defmt::debug!("low");
            led.set_low().unwrap();
        }
        delay.delay_ms(1000.milliseconds());
    }
}
