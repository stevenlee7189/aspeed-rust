use core::fmt;
use embedded_hal_old::watchdog::{Watchdog, Disable, Enable};
use fugit::MillisDurationU32 as MilliSeconds;
use core::cmp::min;
use core::marker::PhantomData;


#[derive(Debug)]
pub enum WdtError {
    Unknown,
}

//abstracts register base access for different instances
pub trait WdtInstance {
    fn ptr() -> *const ast1060_pac::wdt::RegisterBlock; 
}

//wdt0
impl WdtInstance for ast1060_pac::Wdt {
    fn ptr() -> *const ast1060_pac::wdt::RegisterBlock {
        ast1060_pac::Wdt::ptr()
    }
}

//wdt1
impl WdtInstance for ast1060_pac::Wdt1 {
    fn ptr() -> *const ast1060_pac::wdt::RegisterBlock {
         ast1060_pac::Wdt1::ptr()
    }
}

//wdt2
impl WdtInstance for ast1060_pac::Wdt2 {
    fn ptr() -> *const ast1060_pac::wdt::RegisterBlock {
         ast1060_pac::Wdt2::ptr()
    }
}

//wdt3
impl WdtInstance for ast1060_pac::Wdt3 {
    fn ptr() -> *const ast1060_pac::wdt::RegisterBlock {
         ast1060_pac::Wdt3::ptr()
    }
}

//generic
pub struct WdtController<WDT: WdtInstance> {
    wdt: &'static ast1060_pac::wdt::RegisterBlock,
    _marker: PhantomData<WDT>,
}

impl<WDT: WdtInstance> fmt::Debug for WdtController<WDT> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("WdtController")
    }
}

const WDT_RATE_1MHZ:u32 = 1000000;
const MAX_TIMEOUT_MS: u32 = 4294967;
const RESTART_MAGIC: u16 = 0x4755;

impl<WDT: WdtInstance> WdtController<WDT> {
    /// Creates a new `WdtController` without starting it. 
    pub fn new() -> Self {
        let wdt = unsafe { &*WDT::ptr() };
        Self { wdt, _marker: PhantomData, }
    }
    
    /// Sets the watchdog timer timout period. 
    fn setup(&self, timeout_ms: MilliSeconds) {
        assert!(timeout_ms.to_millis() < MAX_TIMEOUT_MS, "Watchdog timeout too high");

        let actual = min(timeout_ms.to_millis(), MAX_TIMEOUT_MS);

        self.wdt.wdt004().write(|w| {
            unsafe { w.counter_reload_value_reg().bits(actual / 1000 * WDT_RATE_1MHZ) }
        });

        self.wdt.wdt008().write(|w| {
            unsafe { w.restart_reg().bits(RESTART_MAGIC as u16) }
        });
    }

    pub fn start(&self, period: MilliSeconds) {
        self.setup(period);
        self.wdt.wdt014().write(|w| {
            w.clear_timeout_boot_code_sel_and_intsts().set_bit()
        });

        self.wdt.wdt00c().write(|w| {
            w.rst_sys_after_timeout().set_bit();
            w.wdtenbl_sig().set_bit()
        });
    }

    pub fn stop(&self) {
        self.wdt.wdt00c().write(|w| {
            w.wdtenbl_sig().clear_bit()
        });
    }

    pub fn feed(&mut self) {
        self.wdt.wdt008().write(|w| {
            unsafe { w.restart_reg().bits(RESTART_MAGIC) }
        });
    }
}

impl<WDT: WdtInstance> Disable for WdtController<WDT> {
    type Error = WdtError;
    type Target = WdtController<WDT>;

    fn try_disable(self) -> Result<Self::Target,Self::Error>{
        self.stop();
        Ok(self)
    }
}

impl<WDT: WdtInstance> Enable for WdtController<WDT> {
    type Error = WdtError;
    type Target = WdtController<WDT>;
    type Time = MilliSeconds;

    fn try_start<T: Into<Self::Time>>(self, period: T) -> Result<Self::Target,Self::Error> {
        self.start(period.into());
        Ok(self)
    }
}

impl<WDT: WdtInstance> Watchdog for WdtController<WDT> {
    type Error = WdtError;

    fn try_feed(&mut self) -> Result<(), Self::Error>{
        self.feed();
        Ok(())
    }
}
