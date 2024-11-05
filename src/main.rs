#![deny(unsafe_code)]
#![no_main]
#![no_std]

extern crate alloc;

use defmt_rtt as _;
use panic_probe as _;
use stm32f3xx_hal::{self as hal};

use core::cell::RefCell;
use core::convert::Infallible;

use alloc::boxed::Box;

use cortex_m::interrupt::Mutex;
use embedded_alloc::LlffHeap;
use embedded_hal::blocking::delay::DelayMs as _;
use embedded_hal::digital::v2::{OutputPin as _, ToggleableOutputPin};
use embedded_hal::timer::CountDown as _;
use embedded_time::duration::Extensions as _;
use embedded_time::rate::Extensions as _;
use hal::delay::Delay;
use hal::dma::{self, DmaExt as _};
use hal::flash::FlashExt as _;
use hal::gpio::GpioExt as _;
use hal::interrupt;
use hal::pac;
use hal::rcc::RccExt as _;
use hal::serial::{self, Serial};
use hal::timer::{self, Timer};

#[derive(Debug)]
enum EchoState<R, RxBuf, RxCh, T, TxBuf, TxCh> {
    Receiving {
        receive: R,
        tx_buffer: TxBuf,
        tx_channel: TxCh,
    },
    Transmitting {
        transmit: T,
        rx_buffer: RxBuf,
        rx_channel: RxCh,
    },
}

impl<Usart, Pins, RxBuf, RxCh, TxBuf, TxCh>
    EchoState<
        dma::Transfer<RxBuf, RxCh, Serial<Usart, Pins>>,
        RxBuf,
        RxCh,
        dma::Transfer<TxBuf, TxCh, Serial<Usart, Pins>>,
        TxBuf,
        TxCh,
    >
where
    Usart: serial::Instance + serial::Dma,
    Serial<Usart, Pins>: dma::OnChannel<RxCh> + dma::OnChannel<TxCh>,
    RxBuf: dma::WriteBuffer<Word = u8> + 'static,
    RxCh: dma::Channel,
    TxBuf: dma::ReadBuffer<Word = u8> + 'static,
    TxCh: dma::Channel,
{
    fn next(self) -> Self {
        match self {
            Self::Receiving {
                receive,
                tx_buffer,
                tx_channel,
            } if receive.is_complete() => {
                let (rx_buffer, rx_channel, serial) = receive.wait();
                let transmit = serial.write_all(tx_buffer, tx_channel);
                Self::Transmitting {
                    transmit,
                    rx_buffer,
                    rx_channel,
                }
            }
            Self::Transmitting {
                transmit,
                rx_buffer,
                rx_channel,
            } if transmit.is_complete() => {
                let (tx_buffer, tx_channel, serial) = transmit.wait();
                let receive = serial.read_exact(rx_buffer, rx_channel);
                Self::Receiving {
                    receive,
                    tx_buffer,
                    tx_channel,
                }
            }
            s => s,
        }
    }
}

#[global_allocator]
static HEAP: LlffHeap = LlffHeap::empty();

type LedType = Box<dyn ToggleableOutputPin<Error = Infallible> + Send + Sync>;
static LED: Mutex<RefCell<Option<LedType>>> = Mutex::new(RefCell::new(None));

static TIMER: Mutex<RefCell<Option<Timer<pac::TIM2>>>> = Mutex::new(RefCell::new(None));

#[expect(unsafe_code)]
unsafe fn init_heap() {
    use core::mem::MaybeUninit;
    const HEAP_SIZE: usize = 1024;
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
    HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE);
}

#[cortex_m_rt::entry]
fn main() -> ! {
    defmt::debug!("entry");

    #[expect(unsafe_code)]
    unsafe {
            HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE);
        }
    }

    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    // TODO: ↓はreleaseビルドでは必要ないかも
    // This is a workaround, so that the debugger will not disconnect
    // imidiatly on asm::wfi();
    // https://github.com/probe-rs/probe-rs/issues/350#issuecomment-740550519
    dp.DBGMCU.cr.modify(|_, w| {
        w.dbg_sleep().set_bit();
        w.dbg_standby().set_bit();
        w.dbg_stop().set_bit()
    });
    dp.RCC.ahbenr.modify(|_, w| w.dma1en().enabled());

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let clocks = rcc
        .cfgr
        .sysclk(8.MHz())
        .pclk1(8.MHz())
        .pclk2(8.MHz())
        .hclk(8.MHz())
        .freeze(&mut flash.acr);
    let mut timer = Timer::new(dp.TIM2, clocks, &mut rcc.apb1);
    let mut delay = Delay::new(cp.SYST, clocks);

    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);
    let mut gpiob = dp.GPIOB.split(&mut rcc.ahb);
    let led = {
        let mut l = gpiob
            .pb3
            .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);
        l.set_low().unwrap();
        Box::new(l)
    };

    cortex_m::interrupt::free(move |cs| {
        LED.borrow(cs).replace(Some(led));
    });

    let tx = gpioa
        .pa2
        .into_af_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);
    let rx = gpioa
        .pa15
        .into_af_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);
    let serial_config = serial::config::Config::default()
        .baudrate(9600.Bd())
        .parity(serial::config::Parity::None)
        .stopbits(serial::config::StopBits::Stop1);
    let serial = Serial::new(dp.USART2, (tx, rx), serial_config, clocks, &mut rcc.apb1);

    let dma1 = dp.DMA1.split(&mut rcc.ahb);
    let tx_channel = dma1.ch7;
    let rx_channel = dma1.ch6;

    timer.enable_interrupt(timer::Event::Update);
    timer.start(1000.milliseconds());
    let interrupt_number = timer.interrupt();
    cortex_m::interrupt::free(move |cs| {
        TIMER.borrow(cs).replace(Some(timer));
    });
    #[allow(unsafe_code)]
    unsafe {
        pac::NVIC::unmask(interrupt_number);
    }
    defmt::debug!("done initializing");

    let message = b"Received!\r\n";
    let rx_buffer = cortex_m::singleton!(: [u8; 1] = [0]).unwrap();
    let mut state = EchoState::Receiving {
        receive: serial.read_exact(rx_buffer, rx_channel),
        tx_buffer: message,
        tx_channel,
    };
    loop {
        state = state.next();
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
        timer.clear_event(timer::Event::Update);
    });
}
