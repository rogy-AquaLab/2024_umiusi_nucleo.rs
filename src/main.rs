#![deny(unsafe_code)]
#![no_main]
#![no_std]

extern crate alloc;

use core::cell::RefCell;
use core::convert::Infallible;

use defmt_rtt as _;
use panic_probe as _;
use stm32f3xx_hal as hal;

use cortex_m::interrupt::Mutex;
use cortex_m_rt::entry;
use embedded_alloc::LlffHeap;
use embedded_hal::digital::v2::{OutputPin, ToggleableOutputPin};
use embedded_hal::timer::CountDown;
use embedded_time::duration::Extensions as DurationExt;
use hal::flash::FlashExt;
use hal::gpio::GpioExt;
use hal::interrupt;
use hal::pac;
use hal::pac::{NVIC, TIM2};
use hal::rcc::RccExt;
use hal::timer::{Event as TimerEvent, Timer};

#[global_allocator]
static HEAP: LlffHeap = LlffHeap::empty();

type LedType = &'static mut (dyn ToggleableOutputPin<Error = Infallible> + Send + Sync);
static LED: Mutex<RefCell<Option<LedType>>> = Mutex::new(RefCell::new(None));

static TIMER: Mutex<RefCell<Option<Timer<TIM2>>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    defmt::debug!("entry");

    // Initialize the allocator
    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 1024;
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        #[allow(unsafe_code)]
        unsafe {
            HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE);
        }
    }

    let dp = pac::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut flash.acr);
    let mut timer = Timer::new(dp.TIM2, clocks, &mut rcc.apb1);

    let mut gpiob = dp.GPIOB.split(&mut rcc.ahb);
    let mut led = {
        let mut l = gpiob
            .pb3
            .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);
        l.set_low().unwrap();
        l
    };
    defmt::debug!("done initializing");

    let led_ptr = &mut led as *mut _;
    // Box::leakライクにポインタを経由することで`led`のライフタイムを'staticに広げる
    // **これ以降にledへのアクセス(drop含む)がなければ** この操作は安全
    #[allow(unsafe_code)]
    let leaked_led = unsafe { &mut (*led_ptr) as LedType };
    cortex_m::interrupt::free(|cs| {
        LED.borrow(cs).replace(Some(leaked_led));
    });

    #[allow(unsafe_code)]
    unsafe {
        NVIC::unmask(timer.interrupt());
    }
    timer.enable_interrupt(TimerEvent::Update);
    timer.start(1000.milliseconds());
    cortex_m::interrupt::free(move |cs| {
        TIMER.borrow(cs).replace(Some(timer));
    });

    loop {
        cortex_m::asm::wfi();
    }
}

#[interrupt]
fn TIM2() {
    cortex_m::interrupt::free(|cs| {
        defmt::debug!("TIM2 interrupt");
        let mut led = LED.borrow(cs).borrow_mut();
        let led = led.as_mut().unwrap();
        let mut timer = TIMER.borrow(cs).borrow_mut();
        let timer = timer.as_mut().unwrap();
        led.toggle().unwrap();
        timer.clear_event(TimerEvent::Update);
    });
}
