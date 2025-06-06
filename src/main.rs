#![no_std]
#![no_main]

use core::sync::atomic::AtomicBool;
// use core::arch::asm;
use ast1060_pac::Peripherals;
use aspeed_ddk::uart::{Config, UartController};
use aspeed_ddk::digest::HaceController;

use aspeed_ddk::hash_test::run_hash_tests;
use panic_halt as _;

use cortex_m_rt::entry;
use embedded_hal::delay::DelayNs;

use embedded_io::Write;
use cortex_m_rt::pre_init;
use core::ptr::{read_volatile, write_volatile};

#[cfg(test)]
mod hash_test;

#[pre_init]
unsafe fn pre_init() {
    let jtag_pinmux_offset : u32 = 0x7e6e2000 + 0x41c;
    let mut reg : u32;
    reg = read_volatile(jtag_pinmux_offset as *const u32);
    reg |= 0x1f << 25;
    write_volatile(jtag_pinmux_offset as *mut u32, reg);
}

pub struct DummyDelay;

impl DelayNs for DummyDelay {
    fn delay_ns(&mut self, _ns: u32) {
        for _ in 0..1000 {
            cortex_m::asm::nop();
        }
    }
}

#[no_mangle]
pub static HALT: AtomicBool = AtomicBool::new(true);

#[macro_export]
macro_rules! debug_halt {
    () => {{
        use core::sync::atomic::{AtomicBool, Ordering};
        use core::arch::asm;

        static HALT: AtomicBool = AtomicBool::new(true);

        while HALT.load(Ordering::SeqCst) {
            unsafe {
                asm!("nop");
            }
        }
    }};
}

#[entry]
fn main() -> ! {

    let _peripherals = unsafe { Peripherals::steal() };
    let uart = _peripherals.uart;
    let mut delay = DummyDelay {};

    // For jlink attach
    // set aspeed_ddk::__cortex_m_rt_main::HALT.v.value = 0 in gdb
    // debug_halt!();
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

    writeln!(uart_controller, "\r\nHello, world!!\r\n").unwrap();

    let mut hace_controller = HaceController::new(hace, scu);

    run_hash_tests(&mut uart_controller, &mut hace_controller);

    loop {}
}

