// Licensed under the Apache-2.0 license

use crate::common::{DummyDelay, Logger};
use core::marker::PhantomData;
use core::fmt::Write;
use embedded_hal::delay::DelayNs;
// use cortex_m::peripheral::NVIC;
// use embedded_hal::delay::DelayNs;

// use ast1060_pac::I3cglobal;
// #![allow(non_upper_case_globals)]

pub const I3C_BUS_I2C_STD_TLOW_MIN_NS:  u32 = 4_700;
pub const I3C_BUS_I2C_STD_THIGH_MIN_NS: u32 = 4_000;
pub const I3C_BUS_I2C_STD_TR_MAX_NS:    u32 = 1_000;
pub const I3C_BUS_I2C_STD_TF_MAX_NS:    u32 =   300;

pub const I3C_BUS_I2C_FM_TLOW_MIN_NS:   u32 = 1_300;
pub const I3C_BUS_I2C_FM_THIGH_MIN_NS:  u32 =   600;
pub const I3C_BUS_I2C_FM_TR_MAX_NS:     u32 =   300;
pub const I3C_BUS_I2C_FM_TF_MAX_NS:     u32 =   300;

pub const I3C_BUS_I2C_FMP_TLOW_MIN_NS:  u32 =   500;
pub const I3C_BUS_I2C_FMP_THIGH_MIN_NS: u32 =   260;
pub const I3C_BUS_I2C_FMP_TR_MAX_NS:    u32 =   120;
pub const I3C_BUS_I2C_FMP_TF_MAX_NS:    u32 =   120;

pub const I3C_BUS_THIGH_MAX_NS:         u32 =    41;

pub const NSEC_PER_SEC:                 u32 = 1_000_000_000;
pub const SDA_TX_HOLD_MIN:u32             = 0b001;
pub const SDA_TX_HOLD_MAX:u32             = 0b111;

pub const SLV_DCR_MASK: u32 = 0x0000_ff00;

pub const I3CG_REG1_SCL_IN_SW_MODE_VAL: u32 = 1 << 23;
pub const I3CG_REG1_SDA_IN_SW_MODE_VAL: u32 = 1 << 27;
pub const I3CG_REG1_SCL_IN_SW_MODE_EN:  u32 = 1 << 28;
pub const I3CG_REG1_SDA_IN_SW_MODE_EN:  u32 = 1 << 29;

pub enum I3cStatus {
    Ok,
    Timeout,
    Busy,
    Pending,
    Invalid,
}

pub struct I3cCmd<'a> {
    pub cmd_lo: u32,
    pub cmd_hi: u32,
    /// Write payload (ignored for reads)
    pub tx: &'a [u8],
    /// Read buffer (ignored for writes)
    pub rx: &'a mut [u8],
    /// Driver can set per-cmd status after execution
    pub status: I3cStatus,
}

pub struct I3cXfer<'a> {
    pub cmds: &'a mut [I3cCmd<'a>],
    pub status: I3cStatus,
}
impl<'a> I3cXfer<'a> {
    pub fn new(cmds: &'a mut [I3cCmd<'a>]) -> Self {
        Self { cmds, status: I3cStatus::Pending }
    }
}

#[derive(Clone, Copy, Default)]
pub struct I3cPriv {
    pub pos: u8,
    pub addr: u8,
    pub ibi_enable: bool,
}

pub struct I3cConfig {
    // Optional: your own “common” higher-level state
    pub common: CommonState,

    pub target_config: Option<&'static mut I3cTargetConfig>,

    // Concurrency
    pub curr_xfer: Option<&'static mut I3cXfer<'static>>,

    // Timing/phy params (ns)
    pub core_period: u32,
    pub i2c_scl_hz: u32,
    pub i3c_scl_hz: u32,
    pub i3c_pp_scl_hi_period_ns: u32,
    pub i3c_pp_scl_lo_period_ns: u32,
    pub i3c_od_scl_hi_period_ns: u32,
    pub i3c_od_scl_lo_period_ns: u32,
    pub sda_tx_hold_ns: u32,
    pub is_secondary: bool,

    // Tables/indices
    pub maxdevs: u16,
    pub datstartaddr: u16,
    pub free_pos: u32,
    pub need_da: u32,

    pub addrs: [u8; 8],
    pub dcr: u32,
    pub privs: [I3cPriv; 8],

    // Target-mode data
    pub sir_allowed_by_sw: bool,
}

#[derive(Default)]
pub struct CommonState {
    _phantom: PhantomData<()>,
}

#[derive(Default)]
pub struct CommonCfg {
    _phantom: PhantomData<()>,
}

#[derive(Clone, Copy)]
pub struct ResetSpec {
    pub id: u32,
    pub active_high: bool,
}

pub struct I3cTargetConfig {
    pub flags: u8,
    pub addr: Option<u8>,
}

pub trait HardwareInterface {
    fn init(&mut self, config: &mut I3cConfig);
    fn i3c_enable(&mut self, config: &I3cConfig);
    fn i3c_disable(&mut self, is_secondary: bool);
    fn core_reset_assert(&mut self, bus: u8);
    fn core_reset_deassert(&mut self, bus: u8);
    fn global_reset_assert(&mut self);
    fn global_reset_deassert(&mut self);
    fn clock_on(&mut self, bus: u8);
    fn set_role(&mut self, is_secondary: bool);
    fn init_clock(&mut self, config: &mut I3cConfig);
    fn get_clock_rate(&self) -> u32;
    fn calc_i2c_clk(&mut self, fscl_hz: u32) -> (u32, u32);
    fn init_pid(&mut self, config: &mut I3cConfig ,bus: u8);
    fn enter_sw_mode(&mut self, bus: u8);
    fn exit_sw_mode(&mut self, bus: u8);
}

pub trait Instance {
    fn ptr() -> *const ast1060_pac::i3c::RegisterBlock;
    fn ptr_global() -> *const ast1060_pac::i3cglobal::RegisterBlock;
    fn scu() -> *const ast1060_pac::scu::RegisterBlock;
    const BUS_NUM: u8;
}

macro_rules! macro_i3c {
    ($I3cx: ident, $x: literal) => {
        impl Instance for ast1060_pac::$I3cx {
            fn ptr() -> *const ast1060_pac::i3c::RegisterBlock {
                ast1060_pac::$I3cx::ptr()
            }

            fn ptr_global() -> *const ast1060_pac::i3cglobal::RegisterBlock {
                ast1060_pac::I3cglobal::ptr()
            }

            fn scu() -> *const ast1060_pac::scu::RegisterBlock {
                ast1060_pac::Scu::ptr()
            }
            const BUS_NUM: u8 = $x;
        }
    };
}

macro_i3c!(I3c, 0);
macro_i3c!(I3c1, 1);

pub struct Ast1060I3c<I3C: Instance, L: Logger> {
    pub i3c: &'static ast1060_pac::i3c::RegisterBlock,
    pub i3cg: &'static ast1060_pac::i3cglobal::RegisterBlock,
    pub scu: &'static ast1060_pac::scu::RegisterBlock,
    pub i3c_config: I3cConfig,
    pub logger: L,
    _marker: PhantomData<I3C>,
}

impl<I3C: Instance, L: Logger> Ast1060I3c<I3C, L> {
    pub fn new(logger: L) -> Self {
        let i3c = unsafe { &*I3C::ptr() };
        let i3cg = unsafe { &*I3C::ptr_global() };
        let scu = unsafe { &*I3C::scu() };
        let i3c_config = I3cConfig::new();
        Self { i3c, i3cg, scu, i3c_config, logger, _marker: PhantomData}
    }
}

impl I3cConfig {
    pub fn new() -> Self {
        Self {
            common: CommonState::default(),
            target_config: None,
            curr_xfer: None,
            core_period: 0,
            i2c_scl_hz: 0,
            i3c_scl_hz: 0,
            i3c_pp_scl_hi_period_ns: 0,
            i3c_pp_scl_lo_period_ns: 0,
            i3c_od_scl_hi_period_ns: 0,
            i3c_od_scl_lo_period_ns: 0,
            sda_tx_hold_ns: 0,
            is_secondary: false,
            maxdevs: 8,
            datstartaddr: 0,
            free_pos: 0,
            need_da: 0,
            addrs: [0; 8],
            dcr: 0,
            privs: [I3cPriv::default(); 8],
            sir_allowed_by_sw: false,
        }
    }
}

macro_rules! i3c_debug {
    ($logger:expr, $($arg:tt)*) => {
        let mut buf: heapless::String<64> = heapless::String::new();
        write!(buf, $($arg)*).unwrap();
        $logger.debug(buf.as_str());
    };
}

#[allow(unused_macros)]
macro_rules! read_i3cg_reg0 {
    ($self:expr, $bus:expr) => {{
        match $bus {
            0 => $self.i3cg.i3c010().read().bits(),
            1 => $self.i3cg.i3c020().read().bits(),
            2 => $self.i3cg.i3c030().read().bits(),
            3 => $self.i3cg.i3c040().read().bits(),
            _ => panic!("invalid I3C bus index: {}", $bus),
        }
    }};
}

macro_rules! read_i3cg_reg1 {
    ($self:expr, $bus:expr) => {{
        match $bus {
            0 => $self.i3cg.i3c014().read().bits(),
            1 => $self.i3cg.i3c024().read().bits(),
            2 => $self.i3cg.i3c034().read().bits(),
            3 => $self.i3cg.i3c044().read().bits(),
            _ => panic!("invalid I3C bus index: {}", $bus),
        }
    }};
}

macro_rules! write_i3cg_reg0 {
    ($self:expr, $bus:expr, |$w:ident| $body:expr) => {{
        match $bus {
            0 => $self.i3cg.i3c010().write(|$w| { $body }),
            1 => $self.i3cg.i3c020().write(|$w| { $body }),
            2 => $self.i3cg.i3c030().write(|$w| { $body }),
            3 => $self.i3cg.i3c040().write(|$w| { $body }),
            _ => panic!("invalid I3C bus index: {}", $bus),
        }
    }};
}

macro_rules! write_i3cg_reg1 {
    ($self:expr, $bus:expr, |$w:ident| $body:expr) => {{
        match $bus {
            0 => $self.i3cg.i3c014().write(|$w| { $body }),
            1 => $self.i3cg.i3c024().write(|$w| { $body }),
            2 => $self.i3cg.i3c034().write(|$w| { $body }),
            3 => $self.i3cg.i3c044().write(|$w| { $body }),
            _ => panic!("invalid I3C bus index: {}", $bus),
        }
    }};
}

#[allow(unused_macros)]
macro_rules! modify_i3cg_reg0 {
    ($self:expr, $bus:expr, |$r:ident, $w:ident| $body:expr) => {{
        match $bus {
            0 => $self.i3cg.i3c010().modify(|$r, $w| { $body }),
            1 => $self.i3cg.i3c020().modify(|$r, $w| { $body }),
            2 => $self.i3cg.i3c030().modify(|$r, $w| { $body }),
            3 => $self.i3cg.i3c040().modify(|$r, $w| { $body }),
            _ => panic!("invalid I3C bus index: {}", $bus),
        }
    }};
}

macro_rules! modify_i3cg_reg1 {
    ($self:expr, $bus:expr, |$r:ident, $w:ident| $body:expr) => {{
        match $bus {
            0 => $self.i3cg.i3c014().modify(|$r, $w| { $body }),
            1 => $self.i3cg.i3c024().modify(|$r, $w| { $body }),
            2 => $self.i3cg.i3c034().modify(|$r, $w| { $body }),
            3 => $self.i3cg.i3c044().modify(|$r, $w| { $body }),
            _ => panic!("invalid I3C bus index: {}", $bus),
        }
    }};
}

pub struct I3cController<H: HardwareInterface, L: Logger> {
    pub hw: H,
    pub config: I3cConfig,
    pub logger: L,
}

impl <I3C: Instance, L: Logger> HardwareInterface for Ast1060I3c<I3C, L> {
    fn init(&mut self, config: &mut I3cConfig) {
        i3c_debug!(self.logger, "i3c init");

        // Global reset is shared, so just need to deassert it
        self.global_reset_deassert();

        write_i3cg_reg1!(self, I3C::BUS_NUM, |w| unsafe {
            w.actmode().bits(1)
                .instid().bits(I3C::BUS_NUM)
                .staticaddr().bits(0x74)
        });

        write_i3cg_reg0!(self, I3C::BUS_NUM, |w|
            w.sdapullupen2k().set_bit()
                .sdapullupen750().clear_bit()
        );
        // TODO: pinctrl
        let mut delay = DummyDelay {};
        self.core_reset_assert(I3C::BUS_NUM);
        self.clock_on(I3C::BUS_NUM);
        self.core_reset_deassert(I3C::BUS_NUM);
        self.i3c_disable(config.is_secondary);

        // Reset controller
        self.i3c.i3cd034().write(|w| {
                w.ibiqueue_sw_rst().set_bit()
                    .rx_buffer_sw_rst().set_bit()
                    .tx_buffer_sw_rst().set_bit()
                    .response_queue_sw_rst().set_bit()
                    .cmd_queue_sw_rst().set_bit()
                    .core_sw_rst().set_bit()
        });
        // TODO: wait controller ready

        let mut timeout = 1_000_000;
        while timeout > 0 {
            let reg_val = self.i3c.i3cd034().read().bits();
            if reg_val == 0 {
                break;
            }
            delay.delay_ns(100_000);
            timeout -= 1;
        }
        self.set_role(config.is_secondary);
        self.init_clock(config);
        // init interrupt mask
        self.i3c.i3cd03c().write(|w| unsafe { w.bits(0xffffffff) });
        if config.is_secondary {
            self.i3c.i3cd040().write(|w| {
                w.transfererrstaten().set_bit()
                    .respreadystatintren().set_bit()
                    .cccupdatedstaten().set_bit()
                    .dynaddrassgnstaten().set_bit()
                    .ibiupdatedstaten().set_bit()
                    .readreqrecvstaten().set_bit()
            });
            self.i3c.i3cd044().write(|w| {
                w.transfererrsignalen().set_bit()
                    .respreadysignalintren().set_bit()
                    .cccupdatedsignalen().set_bit()
                    .dynaddrassgnsignalen().set_bit()
                    .ibiupdatedsignalen().set_bit()
                    .readreqrecvsignalen().set_bit()
            });
        } else {
            self.i3c.i3cd040().write(|w| {
                w.transfererrstaten().set_bit()
                    .respreadystatintren().set_bit()
            });
            self.i3c.i3cd044().write(|w| {
                w.transfererrsignalen().set_bit()
                    .respreadysignalintren().set_bit()
            });
        }
        // TODO: enable interrupt in NVIC according to bus num
        // unsafe {
        //     match I3C::BUS_NUM {
        //         0 => NVIC::unmask(ast1060_pac::Interrupt::i3c),
        //         1 => NVIC::unmask(ast1060_pac::Interrupt::i3c1),
        //         2 => NVIC::unmask(ast1060_pac::Interrupt::i3c2),
        //         3 => NVIC::unmask(ast1060_pac::Interrupt::i3c3),
        //         _ => {}
        //     }
        // }

        config.sir_allowed_by_sw = false;
        // todo: i3c worker init

        // Init hardware queues
        self.i3c.i3cd01c().write(|w| unsafe {
            w.ibidata_threshold_value().bits(31)
        });

        self.i3c.i3cd020().write(|w| unsafe {
            w.rx_buffer_threshold_value().bits(0)
        });

        // Init PID and DCR for target/secondary mode
        self.init_pid(config, I3C::BUS_NUM);

        // Get max device and DAT start addr
        config.datstartaddr = self.i3c.i3cd05c().read().pdevaddrtablestartaddr().bits();
        config.maxdevs = self.i3c.i3cd05c().read().devaddrtabledepth().bits();
        config.free_pos = if config.maxdevs == 32 {
            u32::MAX
        } else {
            (1u32 << config.maxdevs) - 1
        };
        config.need_da = 0;

        self.i3c.i3cd280().write(|w| {
            w.sirreject().set_bit()
                .mrreject().set_bit()
        });

        // Init DAT
        for i in 0..config.maxdevs {
            match i {
                0 => { self.i3c.i3cd280().write(|w| w.sirreject().set_bit().mrreject().set_bit()); }
                1 => { self.i3c.i3cd284().write(|w| w.sirreject().set_bit().mrreject().set_bit()); }
                2 => { self.i3c.i3cd288().write(|w| w.sirreject().set_bit().mrreject().set_bit()); }
                3 => { self.i3c.i3cd28c().write(|w| w.sirreject().set_bit().mrreject().set_bit()); }
                4 => { self.i3c.i3cd290().write(|w| w.sirreject().set_bit().mrreject().set_bit()); }
                5 => { self.i3c.i3cd294().write(|w| w.sirreject().set_bit().mrreject().set_bit()); }
                6 => { self.i3c.i3cd298().write(|w| w.sirreject().set_bit().mrreject().set_bit()); }
                7 => { self.i3c.i3cd29c().write(|w| w.sirreject().set_bit().mrreject().set_bit()); }
                _ => {},
            }
        }

        self.i3c.i3cd02c().write(|w| unsafe {
            w.bits(0xffff_ffff)
        });

        self.i3c.i3cd030().write(|w| unsafe {
            w.bits(0xffff_ffff)
        });
        self.i3c.i3cd000().write(|w| w.hot_join_ack_nack_ctrl().set_bit());

        // TODO: i3c_addr_slot_init

        // 0 - 7 is reserved
        let free_slot = 8;
        if config.is_secondary {
            self.i3c.i3cd004().modify(|_, w| unsafe {
                w.dev_static_addr().bits(free_slot)
                    .static_addr_valid().set_bit()
            });
        } else {
            self.i3c.i3cd004().modify(|_, w| unsafe {
                w.dev_dynamic_addr().bits(free_slot)
                    .dynamic_addr_valid().set_bit()
            });
        }

        // TODO: i3c_addr_slots_mark_i3c
    }

    fn i3c_disable(&mut self, is_secondary: bool) {
        i3c_debug!(self.logger, "i3c disable");
        if self.i3c.i3cd000().read().enbl_i3cctrl().bit_is_clear() {
            return;
        }

        if !is_secondary {
            // enter sw mode
        }
        self.i3c.i3cd000().write(|w| w.enbl_i3cctrl().clear_bit());
    }

    fn core_reset_assert(&mut self, bus: u8) {
        match bus {
            0 => self.scu.scu050().write(|w| w.rst_i3c0ctrl().set_bit()),
            1 => self.scu.scu050().write(|w| w.rst_i3c1ctrl().set_bit()),
            2 => self.scu.scu050().write(|w| w.rst_i3c2ctrl().set_bit()),
            3 => self.scu.scu050().write(|w| w.rst_i3c3ctrl().set_bit()),
            _ => panic!("invalid I3C bus index: {bus}"),
        };
    }

    fn core_reset_deassert(&mut self, bus: u8) {
        let mask = 1u32 << (8 + bus as u32);
        self.scu.scu054().write(|w| unsafe { w.scu050sys_rst_ctrl_clear_reg2().bits(mask) });
    }

    fn global_reset_assert(&mut self) {
        self.scu.scu050().write(|w| w.rst_i3cregdmactrl().set_bit());
    }

    fn global_reset_deassert(&mut self) {
        self.scu.scu054().write(|w| unsafe { w.scu050sys_rst_ctrl_clear_reg2().bits(0x80) });
    }

    fn clock_on(&mut self, bus: u8) {
        let mask = 1u32 << (8 + bus as u32);
        self.scu.scu094().write(|w| unsafe { w.scu090clk_stop_ctrl_clear_reg_set2().bits(mask) });
    }

    fn set_role(&mut self, is_secondary: bool) {
        if is_secondary {
            self.i3c.i3cd0b0().modify(|_, w| unsafe { w.dev_op_mode().bits(1) });
        } else {
            self.i3c.i3cd0b0().modify(|_, w| unsafe { w.dev_op_mode().bits(0) });
        }
    }

    fn init_clock(&mut self, config: &mut I3cConfig) {
        let clk_rate = self.get_clock_rate();
        config.core_period = (1_000_000_000_u32).div_ceil(clk_rate);

        // I2C FM
        let (i2c_hi, i2c_lo) = self.calc_i2c_clk(config.i2c_scl_hz);
        let hcnt: u32 = i2c_hi.div_ceil(config.core_period);
        let lcnt: u32 = i2c_lo.div_ceil(config.core_period);
        self.i3c.i3cd0bc().write(|w| unsafe {
            w.i2cfmhcnt().bits(hcnt as u16)
                .i2cfmlcnt().bits(lcnt as u16)
        });

        // I2C FMP
        let (i2c_hi, i2c_lo) = self.calc_i2c_clk(1_000_000);
        let hcnt: u32 = i2c_hi.div_ceil(config.core_period);
        let lcnt: u32 = i2c_lo.div_ceil(config.core_period);
        self.i3c.i3cd0c0().write(|w| unsafe {
            w.i2cfmphcnt().bits(hcnt as u8)
                .i2cfmplcnt().bits(lcnt as u16)
        });

        // I3C OD
        let hcnt = config.i3c_od_scl_hi_period_ns.div_ceil(config.core_period);
        let lcnt = config.i3c_od_scl_lo_period_ns.div_ceil(config.core_period);
        self.i3c.i3cd0b4().write(|w| unsafe {
            w.i3codhcnt().bits(hcnt as u8)
                .i3codlcnt().bits(lcnt as u8)
        });

        // I3C PP
        let hcnt = config.i3c_pp_scl_hi_period_ns.div_ceil(config.core_period);
        let lcnt = config.i3c_pp_scl_lo_period_ns.div_ceil(config.core_period);
        self.i3c.i3cd0b8().write(|w| unsafe {
            w.i3cpphcnt().bits(hcnt as u8)
                .i3cpplcnt().bits(lcnt as u8)
        });

        // SDA TX hold time
        let lcnt: u32 = (config.sda_tx_hold_ns)
            .div_ceil(config.core_period)
            .clamp(SDA_TX_HOLD_MIN, SDA_TX_HOLD_MAX);

        self.i3c.i3cd0d0().write(|w| unsafe {
            w.sdatxhold().bits(lcnt as u8)
        });
    }

    fn get_clock_rate(&self) -> u32 {
        const HPLL_HZ: u32 = 1_000_000_000;
        const SPLL_HZ: u32 =   480_000_000;

        let r = self.scu.scu310().read();

        let src_hz = if r.i3cclk_source_sel().bit() { HPLL_HZ } else { SPLL_HZ };

        let div = match r.i3cclk_divider_sel().bits() {
            0 => 2,
            n => u32::from(n) + 1,
        };

        src_hz / div
    }

    fn calc_i2c_clk(&mut self, fscl_hz: u32) -> (u32, u32) {
        use core::cmp::max;

        debug_assert!(fscl_hz > 0);
        let period_ns: u32 = (1_000_000_000u32).div_ceil(fscl_hz as u32);

        let (lo_min, hi_min): (u32, u32) = if fscl_hz <= 100_000 {
            (
                (I3C_BUS_I2C_STD_TLOW_MIN_NS  + I3C_BUS_I2C_STD_TF_MAX_NS).div_ceil(period_ns),
                (I3C_BUS_I2C_STD_THIGH_MIN_NS + I3C_BUS_I2C_STD_TR_MAX_NS).div_ceil(period_ns),
            )
        } else if fscl_hz <= 400_000 {
            (
                (I3C_BUS_I2C_FM_TLOW_MIN_NS  + I3C_BUS_I2C_FM_TF_MAX_NS).div_ceil(period_ns),
                (I3C_BUS_I2C_FM_THIGH_MIN_NS + I3C_BUS_I2C_FM_TR_MAX_NS).div_ceil(period_ns),
            )
        } else {
            (
                (I3C_BUS_I2C_FMP_TLOW_MIN_NS  + I3C_BUS_I2C_FMP_TF_MAX_NS).div_ceil(period_ns),
                (I3C_BUS_I2C_FMP_THIGH_MIN_NS + I3C_BUS_I2C_FMP_TR_MAX_NS).div_ceil(period_ns),
            )
        };

        let leftover = period_ns.saturating_sub(lo_min + hi_min);
        let lo = lo_min + leftover / 2;
        let hi = max(period_ns.saturating_sub(lo), hi_min);

        (hi as u32, lo as u32)
    }

    fn init_pid(&mut self, config: &mut I3cConfig ,bus: u8) {
        self.i3c.i3cd070().write(|w| unsafe {
            w.slvmipimfgid().bits(0x3f6)
                .slvpiddcr().clear_bit()
        });

        let rev_id: u32 = self.scu.scu004().read().hw_rev_id().bits().into();
        let reg: u32 = rev_id << 16 | u32::from(bus) << 12;
        self.i3c.i3cd074().write(|w| unsafe { w.bits(reg) });
        let mut reg: u32 = self.scu.scu078().read().bits();
        reg &= !SLV_DCR_MASK;
        reg |= (config.dcr << 8) | 0x66;
        self.i3c.i3cd078().write(|w| unsafe { w.bits(reg) });
    }

    fn enter_sw_mode(&mut self, bus: u8) {
        let mut reg = read_i3cg_reg1!(self, bus);
        reg |= I3CG_REG1_SCL_IN_SW_MODE_VAL | I3CG_REG1_SDA_IN_SW_MODE_VAL;
        modify_i3cg_reg1!(self, bus, |_r, w| unsafe { w.bits(reg) });
        reg |= I3CG_REG1_SCL_IN_SW_MODE_EN | I3CG_REG1_SDA_IN_SW_MODE_EN;
        modify_i3cg_reg1!(self, bus, |_r, w| unsafe { w.bits(reg) });
    }

    fn exit_sw_mode(&mut self, bus: u8) {
        let mut reg = read_i3cg_reg1!(self, bus);
        reg &= !(I3CG_REG1_SCL_IN_SW_MODE_EN | I3CG_REG1_SDA_IN_SW_MODE_EN);
        modify_i3cg_reg1!(self, bus, |_r, w| unsafe { w.bits(reg) });
    }

    fn i3c_enable(&mut self, config: &I3cConfig) {
        i3c_debug!(self.logger, "i3c enable");
        if config.is_secondary {
            self.i3c.i3cd038().write(|w| unsafe { w.bits(0) });
            self.i3c.i3cd000().write(|w| {
                w.enbl_adaption_of_i2ci3cmode().clear_bit()
                    .ibipayloaden().set_bit()
                    .enbl_i3cctrl().set_bit()
            });
            let wait_cnt = &self.i3c.i3cd0d4().read().i3cibifree().bits();
            let wait_ns = u32::from(*wait_cnt) * config.core_period;
            let mut delay = DummyDelay {};
            delay.delay_ns(wait_ns as u32);
        } else {
            self.i3c.i3cd000().write(|w| {
                w.i3cbroadcast_addr_include().set_bit()
                    .enbl_i3cctrl().set_bit()
            });
        }
    }
}
