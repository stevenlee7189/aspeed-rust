#![no_std]
#![no_main]

use ast1060_pac::Peripherals;
use aspeed_ddk::uart::{Config, UartController};
use aspeed_ddk::digest::{DigestCtrl, DigestOp, HashAlgo, HaceController};

use panic_halt as _;

use cortex_m_rt::entry;
use embedded_hal::delay::DelayNs;

use embedded_io::Write;

pub struct DummyDelay;

impl DelayNs for DummyDelay {
    fn delay_ns(&mut self, _ns: u32) {
        for _ in 0..1000 {
            cortex_m::asm::nop();
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

    let hace = _peripherals.hace;
    let scu = _peripherals.scu;

    writeln!(uart_controller, "\r\nHello, world!\r\n").unwrap();

    let mut hace_controller = HaceController::new(hace, scu);

    unsafe {
        let mut ctx =  hace_controller.init(HashAlgo::SHA256).unwrap() ;
        ctx.update(b"hello_world").unwrap();

        let mut output = [0u8; 32];
        ctx.finalize(&mut output).unwrap();

        // for byte in &output[..32] {
        //     writeln!(uart_controller, "{:02x}", byte).unwrap();
        // }
    }
    // Initialize the peripherals here if needed
    loop {}
}
