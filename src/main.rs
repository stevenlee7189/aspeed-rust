//! Prints "Hello, world!" on the host console using semihosting


#![no_std]
#![no_main]

use ast1060_pac::Peripherals;

use panic_halt as _;

use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;

#[entry]
fn main() -> ! {
    hprintln!("Hello, world!");

    let _peripherals = unsafe { Peripherals::steal() };

    // Initialize the peripherals here if needed
    loop {}
}

