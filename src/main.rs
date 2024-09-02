#![deny(unsafe_code)]
#![no_main]
#![no_std]

use defmt_rtt as _;
use panic_probe as _;

use stm32f3xx_hal as hal;

use cortex_m_rt::entry;
use embedded_hal::digital::v2::{OutputPin, StatefulOutputPin, ToggleableOutputPin};
use hal::gpio::GpioExt;
use hal::pac;
use hal::rcc::RccExt;

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    let mut rcc = dp.RCC.constrain();
    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);

    let mut led = gpioa
        .pa1
        .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);

    led.set_low().unwrap();

    loop {
        led.toggle().unwrap();
        cortex_m::asm::delay(8_000_000);
        // Toggle by hand.
        // Uses `StatefulOutputPin` instead of `ToggleableOutputPin`.
        // Logically it is the same.
        if led.is_set_low().unwrap() {
            led.set_high().unwrap();
        } else {
            led.set_low().unwrap();
        }
        cortex_m::asm::delay(8_000_000);
    }
}
