#![no_std]
#![no_main]

use ast1060_pac::Peripherals;
use aspeed_ddk::uart::{Config, UartController};
use ast1060_pac::{Wdt, Wdt1};
use aspeed_ddk::watchdog::WdtController;

use fugit::MillisDurationU32 as MilliSeconds;
use panic_halt as _;

use cortex_m_rt::entry;
use embedded_hal::delay::DelayNs;

use embedded_io::Write;
pub struct DummyDelay;

impl DelayNs for DummyDelay {
    fn delay_ns(&mut self, _ns: u32) {
        for _ in 0.._ns {
            cortex_m::asm::nop();
        }
    }
}

fn test_wdt( uart:&mut UartController<'_>) {
    //instantiates the controller for the hardware watchdog Wdt and Wdt1
    let mut wdt0 = WdtController::<Wdt>::new(); 
    let mut wdt1 = WdtController::<Wdt1>::new();
    let mut delay = DummyDelay {};

    // Start watchdog with a timeout of 2000 milliseconds (2 seconds)
    uart.write_all(b"start wdt\r\n").unwrap();
    wdt0.start(MilliSeconds::millis(5000));
    wdt1.start(MilliSeconds::millis(10000));
    let mut cnt = 0;

    loop {
        delay.delay_ns(2_000_000);
        uart.write_all(b"wdt feed\r\n").unwrap();
        wdt0.feed(); // petting to prevent reset
        wdt1.feed(); // petting to prevent reset
        cnt += 1;
        if cnt > 30 {
            wdt0.stop();
            wdt1.stop();
            uart.write_all(b"stop wdt\r\n").unwrap();
            break;
        }
    }
}


#[entry]
fn main() -> ! {

    let _peripherals = unsafe { Peripherals::steal() };
    let uart = _peripherals.uart;
    let mut delay = DummyDelay {};

    let mut uart_controller = UartController::new(uart, &mut delay);
    unsafe {
        uart_controller.init(Config {
            baud_rate: 115200,
            word_length: aspeed_ddk::uart::WordLength::Eight as u8,
            parity: aspeed_ddk::uart::Parity::None,
            stop_bits: aspeed_ddk::uart::StopBits::One,
            clock: 24_000_000,
        });
    }

    uart_controller.write_all(b"\r\nHello, world!\r\n").unwrap();
    uart_controller.write_all(b"aspeed_ddk!\r\n").unwrap();

    test_wdt(&mut uart_controller);
    // Initialize the peripherals here if needed
    loop {}
}

