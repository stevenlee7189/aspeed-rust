// Licensed under the Apache-2.0 license

use crate::common::{DummyDelay, Logger};
use core::marker::PhantomData;
use core::fmt::Write;
use core::sync::atomic::{AtomicPtr, AtomicBool, Ordering};
use core::cell::RefCell;
use embedded_hal::delay::DelayNs;
use heapless::Vec;
use cortex_m::peripheral::NVIC;
use critical_section::Mutex;

#[derive(Clone, Copy)]
struct Handler {
    func: fn(usize),
    ctx: usize,
}

static BUS_HANDLERS: [Mutex<RefCell<Option<Handler>>>; 4] = [
    Mutex::new(RefCell::new(None)),
    Mutex::new(RefCell::new(None)),
    Mutex::new(RefCell::new(None)),
    Mutex::new(RefCell::new(None)),
];

pub fn register_i3c_irq_handler(bus: usize, func: fn(usize), ctx: usize) {
    assert!(bus < 4);
    critical_section::with(|cs| {
        *BUS_HANDLERS[bus].borrow(cs).borrow_mut() = Some(Handler { func, ctx });
    });
}

#[inline]
fn dispatch_irq(bus: usize) {
    critical_section::with(|cs| {
        if let Some(h) = *BUS_HANDLERS[bus].borrow(cs).borrow() {
            (h.func)(h.ctx);
        }
    });
}

#[no_mangle]
pub extern "C" fn i3c() {
    dispatch_irq(0);
}
#[no_mangle]
pub extern "C" fn i3c1() {
}
#[no_mangle]
pub extern "C" fn i3c2() {
}
#[no_mangle]
pub extern "C" fn i3c3() {
}

pub struct Completion {
    done: AtomicBool,
}

impl Completion {
    pub const fn new() -> Self {
        Self { done: AtomicBool::new(false) }
    }

    #[inline]
    pub fn reset(&self) {
        self.done.store(false, Ordering::Release);
    }

    #[inline]
    pub fn complete(&self) {
        self.done.store(true, Ordering::Release);

        cortex_m::asm::sev();
    }

    #[inline]
    pub fn is_completed(&self) -> bool {
        self.done.load(Ordering::Acquire)
    }

    pub fn wait_for_us<D: DelayNs>(&self, timeout_us: u32, delay: &mut D) -> bool {
        let mut left = timeout_us;
        while !self.is_completed() {
            if left == 0 {
                return false;
            }
            delay.delay_us(1);
            left -= 1;
        }
        true
    }
}

pub const I3C_MSG_READ: u8  = 0x1;

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

pub const CM_TFR_STS_MASTER_HALT: u8 = 0xf;
pub const CM_TFR_STS_TARGET_HALT: u8 = 0x6;

pub const COMMAND_QUEUE_PORT: u32 = 0x0c;

// --- single-bit flags ---
pub const COMMAND_PORT_PEC:           u32 = bit(31);
pub const COMMAND_PORT_TOC:           u32 = bit(30);
pub const COMMAND_PORT_READ_TRANSFER: u32 = bit(28);
pub const COMMAND_PORT_SDAP:          u32 = bit(27);
pub const COMMAND_PORT_ROC:           u32 = bit(26);
pub const COMMAND_PORT_DBP:           u32 = bit(25);
pub const COMMAND_PORT_CP:            u32 = bit(15);

// --- field masks ---
pub const COMMAND_PORT_SPEED:     u32 = bits(23, 21);
pub const COMMAND_PORT_DEV_INDEX: u32 = bits(20, 16);
pub const COMMAND_PORT_CMD:       u32 = bits(14, 7);
pub const COMMAND_PORT_TID:       u32 = bits(6, 3);
pub const COMMAND_PORT_ATTR:      u32 = bits(2, 0);
pub const COMMAND_ATTR_XFER_CMD:        u32 = 0;
pub const COMMAND_ATTR_XFER_ARG:        u32 = 1;
pub const COMMAND_ATTR_SHORT_ARG:       u32 = 2;
pub const COMMAND_ATTR_ADDR_ASSGN_CMD:  u32 = 3;
pub const COMMAND_ATTR_SLAVE_DATA_CMD:  u32 = 0;

pub const COMMAND_PORT_ARG_DB:       u32 = bits(23, 16);
pub const COMMAND_PORT_ARG_DATA_LEN: u32 = bits(15,  0);

/// Device Address Table fields
pub const DEV_ADDR_TABLE_LEGACY_I2C_DEV: u32 = bit(31);
pub const DEV_ADDR_TABLE_DYNAMIC_ADDR:   u32 = bits(23, 16);     // GENMASK(23,16)
pub const DEV_ADDR_TABLE_MR_REJECT:      u32 = bit(14);         // BIT(14)
pub const DEV_ADDR_TABLE_SIR_REJECT:     u32 = bit(13);         // BIT(13)
pub const DEV_ADDR_TABLE_IBI_MDB:        u32 = bit(12);         // BIT(12)
pub const DEV_ADDR_TABLE_IBI_PEC:        u32 = bit(11);         // BIT(11)
pub const DEV_ADDR_TABLE_STATIC_ADDR:    u32 = bits(6, 0);     // GENMASK(6,0)


pub const I3C_BCR_IBI_PAYLOAD_HAS_DATA_BYTE: u32 = bit(2);

pub const I3C_CCC_ENTDAA:u32 = 0x7;
pub const I3C_CCC_SETHID:u8 = 0x61;
pub const I3C_CCC_DEVCTRL:u8 = 0x62;
pub const I3C_CCC_SETNEWDA:u8 = 0x88;
pub const I3C_CCC_GETPID:u8 = 0x8d;
pub const I3C_CCC_GETBCR:u8 = 0x8e;
pub const I3C_CCC_GETSTATUS:u8 = 0x90;
pub const I3C_CCC_EVT_INTR: u32 = bit(0);


const MAX_CMDS: usize = 32;

#[repr(u32)]
pub enum SpeedI3c {
    Sdr0   = 0x0,
    Sdr1   = 0x1,
    Sdr2   = 0x2,
    Sdr3   = 0x3,
    Sdr4   = 0x4,
    HdrTs  = 0x5,
    HdrDdr = 0x6,
    I2cFmAsI3c = 0x7, // SPEED_I3C_I2C_FM
}

#[repr(u32)]
pub enum SpeedI2c {
    Fm  = 0x0,
    Fmp = 0x1,
}

#[repr(u32)]
pub enum Tid {
    TargetIbi       = 0x1,
    TargetRdData    = 0x2,
    TargetMasterWr  = 0x8,
    TargetMasterDef = 0xF,
}

pub enum I3cError {
    NoSpace,     // -ENOSPC
}
pub type I3cResult<T> = core::result::Result<T, I3cError>;

pub const COMMAND_PORT_DEV_COUNT: u32 = bits(25, 21);

pub const RESET_CTRL_IBI_QUEUE: u32   = bit(5);
pub const RESET_CTRL_RX_FIFO: u32     = bit(4);
pub const RESET_CTRL_TX_FIFO: u32     = bit(3);
pub const RESET_CTRL_RESP_QUEUE: u32  = bit(2);
pub const RESET_CTRL_CMD_QUEUE: u32   = bit(1);
pub const RESET_CTRL_SOFT: u32        = bit(0);

pub const RESET_CTRL_ALL: u32 =
    RESET_CTRL_IBI_QUEUE
    | RESET_CTRL_RX_FIFO
    | RESET_CTRL_TX_FIFO
    | RESET_CTRL_RESP_QUEUE
    | RESET_CTRL_CMD_QUEUE
    | RESET_CTRL_SOFT;

pub const RESET_CTRL_QUEUES: u32 =
    RESET_CTRL_IBI_QUEUE
    | RESET_CTRL_RX_FIFO
    | RESET_CTRL_TX_FIFO
    | RESET_CTRL_RESP_QUEUE
    | RESET_CTRL_CMD_QUEUE;

pub const RESET_CTRL_XFER_QUEUES: u32 =
    RESET_CTRL_RX_FIFO
    | RESET_CTRL_TX_FIFO
    | RESET_CTRL_RESP_QUEUE
    | RESET_CTRL_CMD_QUEUE;

const fn genmask(msb: u32, lsb: u32) -> u32 {
    let width = msb - lsb + 1;
    if width >= 32 {
        u32::MAX
    } else {
        ((1u32 << width) - 1) << lsb
    }
}

#[inline(always)]
const fn field_get(val: u32, mask: u32, shift: u32) -> u32 {
    (val & mask) >> shift
}

pub const fn bit(n: u32) -> u32 { 1 << n }
pub const fn bits(h: u32, l: u32) -> u32 { ((1u32 << (h - l + 1)) - 1) << l }
pub const fn field_prep(mask: u32, val: u32) -> u32 {
    (val << mask.trailing_zeros()) & mask
}
fn find_lsb_pos(x: u32) -> Option<u32> {
    if x == 0 { None } else { Some(x.trailing_zeros() as u32) }
}


// ---- registers / fields ----
pub const RESPONSE_QUEUE_PORT: u32 = 0x10;
pub const RESPONSE_PORT_ERR_STATUS_SHIFT: u32 = 28;
pub const RESPONSE_PORT_ERR_STATUS_MASK:  u32 = genmask(31, 28);
pub const RESPONSE_PORT_TID_SHIFT: u32 = 24;
pub const RESPONSE_PORT_TID_MASK:  u32 = genmask(27, 24);
pub const RESPONSE_PORT_DATA_LEN_SHIFT: u32 = 0;
pub const RESPONSE_PORT_DATA_LEN_MASK:  u32 = genmask(15, 0);

pub const RESPONSE_NO_ERROR: u32             = 0;
pub const RESPONSE_ERROR_CRC: u32            = 1;
pub const RESPONSE_ERROR_PARITY: u32         = 2;
pub const RESPONSE_ERROR_FRAME: u32          = 3;
pub const RESPONSE_ERROR_IBA_NACK: u32       = 4;
pub const RESPONSE_ERROR_ADDRESS_NACK: u32   = 5;
pub const RESPONSE_ERROR_OVER_UNDER_FLOW: u32= 6;
pub const RESPONSE_ERROR_TRANSF_ABORT: u32   = 8;
pub const RESPONSE_ERROR_I2C_W_NACK_ERR: u32 = 9;
pub const RESPONSE_ERROR_EARLY_TERMINATE: u32= 10;
pub const RESPONSE_ERROR_PEC_ERR: u32        = 12;

pub const INTR_STATUS:     u32 = 0x3c;
pub const INTR_STATUS_EN:  u32 = 0x40;
pub const INTR_SIGNAL_EN:  u32 = 0x44;
pub const INTR_FORCE:      u32 = 0x48;

// Interrupt status bits
pub const INTR_BUSOWNER_UPDATE_STAT: u32 = bit(13);
pub const INTR_IBI_UPDATED_STAT:     u32 = bit(12);
pub const INTR_READ_REQ_RECV_STAT:   u32 = bit(11);
pub const INTR_DEFSLV_STAT:          u32 = bit(10);
pub const INTR_TRANSFER_ERR_STAT:    u32 = bit(9);
pub const INTR_DYN_ADDR_ASSGN_STAT:  u32 = bit(8);
pub const INTR_CCC_UPDATED_STAT:     u32 = bit(6);
pub const INTR_TRANSFER_ABORT_STAT:  u32 = bit(5);
pub const INTR_RESP_READY_STAT:      u32 = bit(4);
pub const INTR_CMD_QUEUE_READY_STAT: u32 = bit(3);
pub const INTR_IBI_THLD_STAT:        u32 = bit(2);
pub const INTR_RX_THLD_STAT:         u32 = bit(1);
pub const INTR_TX_THLD_STAT:         u32 = bit(0);

pub enum I3cStatus {
    Ok,
    Timeout,
    Busy,
    Pending,
    Invalid,
}

#[derive(Debug)]
pub struct I3cCmd<'a> {
    pub cmd_lo: u32,
    pub cmd_hi: u32,
    pub tx: Option<&'a [u8]>,
    pub rx: Option<&'a mut [u8]>,
    pub tx_len: u32,
    pub rx_len: u32,
    pub ret: i32,
}

pub struct I3cMsg<'a> {
    pub buf: Option<&'a mut [u8]>,
    pub actual_len: u32,
    pub num_xfer: u32,
    pub flags: u8,
    pub hdr_mode: u8,
    pub hdr_cmd_mode: u8,
}

pub struct I3cXfer<'cmds, 'buf> {
    pub cmds: &'cmds mut [I3cCmd<'buf>],
    pub ret: i32,
    pub done: Completion,
}

impl<'cmds, 'buf> I3cXfer<'cmds, 'buf> {
    pub fn new(cmds: &'cmds mut [I3cCmd<'buf>]) -> Self {
        Self { cmds, ret: 0, done: Completion::new() }
    }

    pub fn ncmds(&self) -> usize {
        self.cmds.len()
    }
}

#[derive(Clone, Copy, Default)]
pub struct I3cPriv {
    pub pos: u8,
    pub addr: u8,
    pub ibi_enable: bool,
}

#[derive(Clone, Copy, Default)]
pub struct I3cDesc {
    pub pid: u64,
    pub static_addr: u8,
    pub init_dyn_addr: u8,
    pub dynamic_addr: u8,
    pub bcr: u8,
    pub dcr: u8,
    pub maxrd: u8,
    pub maxwr: u8,
    pub max_read_turnaround: u32,
    pub mrl: u16,
    pub mwl: u16,
    pub max_ibi: u8,
    pub i3c_priv_idx: Option<u8>,
}

pub struct I2cDesc {
    pub addr: u16,
    pub lvr: u8,
    pub i3c_priv_idx: Option<u8>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct I3cPid(pub u64);

impl I3cPid {
    pub const fn manuf_id(self) -> u16 { ((self.0 >> 33) & 0x1FFF) as u16 }
    pub const fn has_random_lower32(self) -> bool { (self.0 & (1u64 << 32)) != 0 }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct I3cDeviceId { pub pid: I3cPid }

impl I3cDeviceId {
    pub const fn new(pid: u64) -> Self { Self { pid: I3cPid(pid) } }
}

pub const I3C_MAX_ADDR: u8 = 0x7F;

const WORD_BITS: usize = 32;
const TOTAL_BITS: usize = ((I3C_MAX_ADDR as usize) + 1) * 2;
const NWORDS: usize = (TOTAL_BITS + WORD_BITS - 1) / WORD_BITS;

#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum I3cAddrSlotStatus {
    Free   = 0b00,
    I3cDev = 0b01,
    I2cDev = 0b10,
    Rsvd   = 0b11,
}

impl I3cAddrSlotStatus {
    #[inline]
    fn from_raw(v: u32) -> Self {
        match (v & 0b11) as u8 {
            0 => Self::Free,
            1 => Self::I3cDev,
            2 => Self::I2cDev,
            _ => Self::Rsvd,
        }
    }
}

pub struct I3cAddrSlots {
    words: [u32; NWORDS],
}

impl I3cAddrSlots {
    pub const fn new() -> Self {
        Self { words: [0; NWORDS] }
    }

    #[inline]
    pub fn status(&self, addr: u8) -> I3cAddrSlotStatus {
        if addr > I3C_MAX_ADDR { return I3cAddrSlotStatus::Rsvd; }
        let bitpos = (addr as usize) * 2;
        let idx    = bitpos / WORD_BITS;
        let shift  = bitpos % WORD_BITS;
        I3cAddrSlotStatus::from_raw((self.words[idx] >> shift) & 0b11)
    }

    #[inline]
    pub fn set_status(&mut self, addr: u8, st: I3cAddrSlotStatus) {
        if addr > I3C_MAX_ADDR { return; }
        let bitpos = (addr as usize) * 2;
        let idx    = bitpos / WORD_BITS;
        let shift  = bitpos % WORD_BITS;
        let mask   = !(0b11u32 << shift);
        self.words[idx] = (self.words[idx] & mask) | (((st as u32) & 0b11) << shift);
    }

    #[inline]
    pub fn next_free_from(&self, start_addr: u8) -> u8 {
        let mut addr = start_addr.max(8);
        while addr < I3C_MAX_ADDR {
            if self.status(addr) == I3cAddrSlotStatus::Free {
                return addr;
            }
            addr += 1;
        }
        0
    }

    #[inline]
    pub fn reserve_range(&mut self, start: u8, end_inclusive: u8) {
        let s = start.min(I3C_MAX_ADDR);
        let e = end_inclusive.min(I3C_MAX_ADDR);
        let mut a = s;
        while a <= e {
            self.set_status(a, I3cAddrSlotStatus::Rsvd);
            if a == u8::MAX { break; }
            a += 1;
        }
    }
}

pub struct I3cDevAttachedList<const I3C_MAX: usize, const I2C_MAX: usize> {
    pub addr_slots: I3cAddrSlots,
    pub i3c_devices: Vec<I3cDesc, I3C_MAX>,
    pub i2c_devices: Vec<I2cDesc, I2C_MAX>,
}

impl<const I3C_MAX: usize, const I2C_MAX: usize> I3cDevAttachedList<I3C_MAX, I2C_MAX> {
    pub fn new() -> Self {
        let mut slots = I3cAddrSlots::new();
        slots.reserve_range(0, 7);
        Self {
            addr_slots: slots,
            i3c_devices: Vec::new(),
            i2c_devices: Vec::new(),
        }
    }

    #[inline]
    pub fn slot_status(&self, addr: u8) -> I3cAddrSlotStatus {
        self.addr_slots.status(addr)
    }

    #[inline]
    pub fn next_free_da(&self, start_addr: u8) -> u8 {
        self.addr_slots.next_free_from(start_addr)
    }

    #[inline]
    pub fn mark_i3c(&mut self, addr: u8) {
        self.addr_slots.set_status(addr, I3cAddrSlotStatus::I3cDev);
    }

    #[inline]
    pub fn mark_i2c(&mut self, addr: u8) {
        self.addr_slots.set_status(addr, I3cAddrSlotStatus::I2cDev);
    }

    #[inline]
    pub fn mark_free(&mut self, addr: u8) {
        self.addr_slots.set_status(addr, I3cAddrSlotStatus::Free);
    }
    #[inline]
    pub fn find_index_by_dyn(&self, addr: u8) -> Option<usize> {
        self.i3c_devices
            .iter()
            .position(|d| d.dynamic_addr == addr)
    }

    #[inline]
    pub fn find_i3c_by_dyn(&self, addr: u8) -> Option<&I3cDesc> {
        self.i3c_devices.iter().find(|d| d.dynamic_addr == addr)
    }

    #[inline]
    pub fn find_i3c_by_dyn_mut(&mut self, addr: u8) -> Option<&mut I3cDesc> {
        self.i3c_devices.iter_mut().find(|d| d.dynamic_addr == addr)
    }

    #[inline]
    fn find_index_by_pid(&self, id: I3cDeviceId) -> Option<usize> {
        if id.pid.has_random_lower32() {
            let want = id.pid.manuf_id();
            self.i3c_devices
                .iter()
                .position(|d| I3cPid(d.pid).manuf_id() == want)
        } else {
            self.i3c_devices
                .iter()
                .position(|d| d.pid == id.pid.0)
        }
    }

    #[inline]
    pub fn find_i3c_by_pid(&self, id: I3cDeviceId) -> Option<&I3cDesc> {
        self.find_index_by_pid(id).map(|i| &self.i3c_devices[i])
    }

    #[inline]
    pub fn find_i3c_by_pid_mut(&mut self, id: I3cDeviceId) -> Option<&mut I3cDesc> {
        if let Some(i) = self.find_index_by_pid(id) {
            Some(&mut self.i3c_devices[i])
        } else {
            None
        }
    }
}

pub trait HasI3cPid { fn pid(&self) -> I3cPid; }
impl HasI3cPid for I3cDesc { #[inline] fn pid(&self) -> I3cPid { I3cPid(self.pid) } }

pub fn i3c_dev_list_find<'a, T, I>(iter: I, id: I3cDeviceId) -> Option<&'a T>
where
    I: IntoIterator<Item = &'a T>,
    T: HasI3cPid,
{
    if id.pid.has_random_lower32() {
        let want = id.pid.manuf_id();
        iter.into_iter().find(|d| d.pid().manuf_id() == want)
    } else {
        let want = id.pid.0;
        iter.into_iter().find(|d| d.pid().0 == want)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GetStatusFormat {
    Fmt1,
    Fmt2(GetStatusDefByte),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GetStatusDefByte {
    /// 0x00 - TGTSTAT
    TgtStat,
    /// 0x91 - PRECR
    Precr,
}

impl GetStatusDefByte {
    #[inline]
    fn as_byte(self) -> u8 {
        match self {
            GetStatusDefByte::TgtStat => 0x00,
            GetStatusDefByte::Precr   => 0x91,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GetStatusResp {
    Fmt1 { status: u16 },
    Fmt2 { kind: GetStatusDefByte, raw_u16: u16 },
}

pub struct I3cConfig {
    // Optional: your own “common” higher-level state
    pub common: CommonState,

    pub target_config: Option<&'static mut I3cTargetConfig>,
    pub devs: I3cDevAttachedList<8, 0>,

    // Concurrency
    // pub curr_xfer: Option<&'static mut I3cXfer<'static>>,
    pub curr_xfer: AtomicPtr<()>,

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

#[derive(Debug)]
pub struct CccTargetPayload<'a> {
    /// Target 7‑bit dynamic address (left‑aligned; driver decides if LSB is R/W).
    pub addr: u8,
    /// `false` = write, `true` = read.
    pub rnw: bool,
    /// Data buffer for write (source) or read (destination).
    pub data: Option<&'a mut [u8]>,
    /// Actual bytes transferred (driver fills on return).
    pub num_xfer: usize,
}

#[derive(Debug)]
pub struct Ccc<'a> {
    pub id: u8,
    /// Optional CCC data immediately following the CCC byte.
    pub data: Option<&'a mut [u8]>,
    /// Actual bytes transferred (driver fills on return).
    pub num_xfer: usize,
}

/// One CCC transaction description.
#[derive(Debug)]
pub struct CccPayload<'a, 'b> {
    pub ccc: Option<Ccc<'a>>,
    /// Optional list of direct‑CCC target payloads.
    pub targets: Option<&'b mut [CccTargetPayload<'a>]>,
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
    fn bus_num(&self) -> u8;
    fn enable_irq(&mut self);
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
    fn enter_sw_mode(&mut self);
    fn exit_sw_mode(&mut self);
    fn i3c_toggle_scl_in(&mut self, count:u32);
    fn gen_internal_stop(&mut self);
    fn i3c_bus_init(&mut self, config: &mut I3cConfig);
    fn even_parity(byte: u8) -> bool;
    fn set_ibi_mdb(&mut self, mdb: u8);
    fn exit_halt(&mut self, config: &mut I3cConfig);
    fn enter_halt(&mut self, by_sw: bool, config: &mut I3cConfig);
    fn reset_ctrl(&mut self, reset: u32);
    fn wr_tx_fifo(&mut self, bytes: &[u8]);
    fn rd_fifo<F>(&mut self, read_word: F, out: &mut [u8])
    where
        F: FnMut() -> u32;
    fn drain_fifo<F>(&mut self, read_word: F, len: usize)
    where
        F: FnMut() -> u32;
    fn rd_rx_fifo(&mut self, out: &mut [u8]);
    fn rd_ibi_fifo(&mut self, out: &mut [u8]);
    fn ibi_enable(&mut self, config: &mut I3cConfig, dev_idx: usize) -> i32;
    fn start_xfer(&mut self, config: &mut I3cConfig, xfer: &mut I3cXfer);
    fn end_xfer(&mut self, config: &mut I3cConfig);
    fn get_addr_pos(&mut self, config: &I3cConfig, addr: u8) -> Option<u8>;
    fn detach_i3c_dev(&mut self, config: &mut I3cConfig, pos: u8);
    fn attach_i3c_device(&mut self, config: &mut I3cConfig, target: &mut I3cDesc, addr: u8) -> I3cResult<()>;
    fn do_ccc(&mut self, config: &mut I3cConfig, ccc: &mut CccPayload) -> i32;
    fn do_entdaa(&mut self, config: &mut I3cConfig, index: u32) -> i32;
    fn bytes_to_pid(bytes: &[u8]) -> u64;
    fn handle_unsolicited(&mut self, config: &mut I3cConfig);
    fn do_daa(&mut self, config: &mut I3cConfig) -> i32;
    fn priv_xfer_build_cmds<'a>( &mut self, cmds: &mut [I3cCmd<'a>], msgs: &mut [I3cMsg<'a>], pos: u8,) -> i32;
    fn priv_xfer(&mut self, config: &mut I3cConfig, target: &mut I3cDesc, msgs: &mut [I3cMsg]) -> i32;
    fn target_tx_write(&mut self, buf: &[u8]);
    fn handle_ibi_sir(&mut self, config: &mut I3cConfig);
    fn handle_ibis(&mut self, config: &mut I3cConfig);
    fn i3c_aspeed_isr(&mut self, config: &mut I3cConfig);
    // ccc apis
    fn ccc_do_getbcr(&mut self, config: &mut I3cConfig, dyn_addr: u8) -> Result<u8, i32>;
    fn ccc_do_setnewda( &mut self, config: &mut I3cConfig, curr_da: u8, new_da: u8,) -> i32;
    fn ccc_do_getpid(&mut self, config: &mut I3cConfig, dyn_addr: u8) -> Result<u64, i32>;
    fn ccc_do_events_set(&mut self, config: &mut I3cConfig, da: u8, enable: bool, events: u8) -> i32;
    fn ccc_do_getstatus( &mut self, config: &mut I3cConfig, da: u8, fmt: GetStatusFormat,) -> Result<GetStatusResp, i32>;
    fn ccc_do_getstatus_fmt1(&mut self, config: &mut I3cConfig, da: u8) -> Result<u16, i32>;
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
macro_i3c!(I3c2, 2);
macro_i3c!(I3c3, 3);

pub struct Ast1060I3c<I3C: Instance, L: Logger> {
    pub i3c: &'static ast1060_pac::i3c::RegisterBlock,
    pub i3cg: &'static ast1060_pac::i3cglobal::RegisterBlock,
    pub scu: &'static ast1060_pac::scu::RegisterBlock,
    // pub i3c_config: I3cConfig,
    pub logger: L,
    _marker: PhantomData<I3C>,
}

impl<I3C: Instance, L: Logger> Ast1060I3c<I3C, L> {
    pub fn new(logger: L) -> Self {
        let i3c = unsafe { &*I3C::ptr() };
        let i3cg = unsafe { &*I3C::ptr_global() };
        let scu = unsafe { &*I3C::scu() };
        // let i3c_config = I3cConfig::new();
        // Self { i3c, i3cg, scu, i3c_config, logger, _marker: PhantomData}
        Self { i3c, i3cg, scu, logger, _marker: PhantomData}
    }
}

impl I3cConfig {
    pub fn new() -> Self {
        Self {
            common: CommonState::default(),
            target_config: None,
            devs: I3cDevAttachedList::new(),
            curr_xfer: AtomicPtr::new(core::ptr::null_mut()),
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

macro_rules! i3c_dat_read {
    ($self:expr, $pos:expr) => {{
        match ($pos as u8) {
            0 => $self.i3c.i3cd280().read().bits(),
            1 => $self.i3c.i3cd284().read().bits(),
            2 => $self.i3c.i3cd288().read().bits(),
            3 => $self.i3c.i3cd28c().read().bits(),
            4 => $self.i3c.i3cd290().read().bits(),
            5 => $self.i3c.i3cd294().read().bits(),
            6 => $self.i3c.i3cd298().read().bits(),
            7 => $self.i3c.i3cd29c().read().bits(),
            _ => 0,
        }
    }};
}

macro_rules! i3c_dat_write {
    ($self:expr, $pos:expr, |$w:ident| $body:expr) => {{
        match ($pos as u8) {
            0 => { $self.i3c.i3cd280().write(|$w| { $body }); },
            1 => { $self.i3c.i3cd284().write(|$w| { $body }); },
            2 => { $self.i3c.i3cd288().write(|$w| { $body }); },
            3 => { $self.i3c.i3cd28c().write(|$w| { $body }); },
            4 => { $self.i3c.i3cd290().write(|$w| { $body }); },
            5 => { $self.i3c.i3cd294().write(|$w| { $body }); },
            6 => { $self.i3c.i3cd298().write(|$w| { $body }); },
            7 => { $self.i3c.i3cd29c().write(|$w| { $body }); },
            _ => { /* ignore */ },
        }
    }};
}

#[allow(unused_macros)]
macro_rules! i3c_dat_modify {
    ($self:expr, $pos:expr, |$r:ident, $w:ident| $body:expr) => {{
        match ($pos as u8) {
            0 => { $self.i3c.i3cd280().modify(|$r, $w| { $body }); },
            1 => { $self.i3c.i3cd284().modify(|$r, $w| { $body }); },
            2 => { $self.i3c.i3cd288().modify(|$r, $w| { $body }); },
            3 => { $self.i3c.i3cd28c().modify(|$r, $w| { $body }); },
            4 => { $self.i3c.i3cd290().modify(|$r, $w| { $body }); },
            5 => { $self.i3c.i3cd294().modify(|$r, $w| { $body }); },
            6 => { $self.i3c.i3cd298().modify(|$r, $w| { $body }); },
            7 => { $self.i3c.i3cd29c().modify(|$r, $w| { $body }); },
            _ => { /* ignore */ },
        }
    }};
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PollError {
    Timeout,
}

pub fn poll_with_timeout<F, C, D>(
    mut read_reg: F,
    mut condition: C,
    delay: &mut D,
    delay_ns: u32,
    max_iters: u32,
) -> Result<u32, PollError>
where
    F: FnMut() -> u32,
    C: FnMut(u32) -> bool,
    D: embedded_hal::delay::DelayNs,
{
    for _ in 0..max_iters {
        let val = read_reg();
        if condition(val) {
            return Ok(val);
        }
        delay.delay_ns(delay_ns);
    }
    Err(PollError::Timeout)
}

pub struct I3cController<H: HardwareInterface, L: Logger> {
    pub hw: H,
    pub config: I3cConfig,
    pub logger: L,
}

impl<H: HardwareInterface, L: Logger> I3cController<H, L> {
    pub fn init(&mut self) {
        let ctx = (self as *mut Self) as usize;
        let bus = self.hw.bus_num() as usize;
        register_i3c_irq_handler(bus, Self::irq_trampoline, ctx);

        self.hw.init(&mut self.config);

        self.hw.enable_irq();
    }

    fn irq_trampoline(ctx: usize) {
        let me: &mut Self = unsafe { &mut *(ctx as *mut Self) };
        me.hw.i3c_aspeed_isr(&mut me.config);
    }
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

        let _ = poll_with_timeout(
            || self.i3c.i3cd034().read().bits(),
            |val| val == 0,
            &mut delay,
            100_000,
            1_000_000,
        );

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

        self.i3c.i3cd020().modify(|_, w| unsafe {
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

        // Init DAT
        for i in 0..(config.maxdevs as u8) {
            i3c_dat_write!(self, i, |w| {
                w.sirreject().set_bit()
                    .mrreject().set_bit()
            });
        }

        self.i3c.i3cd02c().write(|w| unsafe {
            w.bits(0xffff_ffff)
        });
        self.i3c.i3cd030().write(|w| unsafe {
            w.bits(0xffff_ffff)
        });
        self.i3c.i3cd000().modify(|_, w| w.hot_join_ack_nack_ctrl().set_bit());

        // TODO: i3c_addr_slot_init
        // TODO: find free slot

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
        self.i3c_enable(config);

        // Perform bus initialization
        if !config.is_secondary {
            self.i3c_bus_init(config);
        }

        // Enable hot-join
        self.i3c.i3cd000().modify(|_, w| w.hot_join_ack_nack_ctrl().clear_bit());
    }

    fn bus_num(&self) -> u8 {
        I3C::BUS_NUM
    }

    fn enable_irq(&mut self) {
        unsafe {
            match I3C::BUS_NUM {
                0 => NVIC::unmask(ast1060_pac::Interrupt::i3c),
                1 => NVIC::unmask(ast1060_pac::Interrupt::i3c1),
                2 => NVIC::unmask(ast1060_pac::Interrupt::i3c2),
                3 => NVIC::unmask(ast1060_pac::Interrupt::i3c3),
                _ => {}
            }
        }
    }

    fn i3c_disable(&mut self, is_secondary: bool) {
        i3c_debug!(self.logger, "i3c disable");
        if self.i3c.i3cd000().read().enbl_i3cctrl().bit_is_clear() {
            return;
        }

        if is_secondary {
            // enter sw mode
            self.enter_sw_mode();
        }
        self.i3c.i3cd000().modify(|_, w| w.enbl_i3cctrl().clear_bit());

        if is_secondary {
            self.i3c_toggle_scl_in(8);
            self.gen_internal_stop();
            self.exit_sw_mode();
        }
    }

    fn core_reset_assert(&mut self, bus: u8) {
        match bus {
            0 => self.scu.scu050().modify(|_, w| w.rst_i3c0ctrl().set_bit()),
            1 => self.scu.scu050().modify(|_, w| w.rst_i3c1ctrl().set_bit()),
            2 => self.scu.scu050().modify(|_, w| w.rst_i3c2ctrl().set_bit()),
            3 => self.scu.scu050().modify(|_, w| w.rst_i3c3ctrl().set_bit()),
            _ => panic!("invalid I3C bus index: {bus}"),
        };
    }

    fn core_reset_deassert(&mut self, bus: u8) {
        let mask = 1u32 << (8 + bus as u32);
        self.scu.scu054().modify(|_, w| unsafe { w.scu050sys_rst_ctrl_clear_reg2().bits(mask) });
    }

    fn global_reset_assert(&mut self) {
        self.scu.scu050().modify(|_, w| w.rst_i3cregdmactrl().set_bit());
    }

    fn global_reset_deassert(&mut self) {
        self.scu.scu054().modify(|_, w| unsafe { w.scu050sys_rst_ctrl_clear_reg2().bits(0x80) });
    }

    fn clock_on(&mut self, bus: u8) {
        let mask = 1u32 << (8 + bus as u32);
        self.scu.scu094().modify(|_, w| unsafe { w.scu090clk_stop_ctrl_clear_reg_set2().bits(mask) });
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

        self.i3c.i3cd0d0().modify(|_, w| unsafe {
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

    fn enter_sw_mode(&mut self) {
        let bus = I3C::BUS_NUM;
        let mut reg = read_i3cg_reg1!(self, bus);
        reg |= I3CG_REG1_SCL_IN_SW_MODE_VAL | I3CG_REG1_SDA_IN_SW_MODE_VAL;
        modify_i3cg_reg1!(self, bus, |_r, w| unsafe { w.bits(reg) });
        reg |= I3CG_REG1_SCL_IN_SW_MODE_EN | I3CG_REG1_SDA_IN_SW_MODE_EN;
        modify_i3cg_reg1!(self, bus, |_r, w| unsafe { w.bits(reg) });
    }

    fn exit_sw_mode(&mut self) {
        let bus = I3C::BUS_NUM;
        let mut reg = read_i3cg_reg1!(self, bus);
        reg &= !(I3CG_REG1_SCL_IN_SW_MODE_EN | I3CG_REG1_SDA_IN_SW_MODE_EN);
        modify_i3cg_reg1!(self, bus, |_r, w| unsafe { w.bits(reg) });
    }

    fn i3c_enable(&mut self, config: &I3cConfig) {
        i3c_debug!(self.logger, "i3c enable");
        if config.is_secondary {
            self.i3c.i3cd038().write(|w| unsafe { w.bits(0) });
            self.enter_sw_mode();
            // Enable hot-join
            self.i3c.i3cd000().modify(|_, w| {
                w.enbl_adaption_of_i2ci3cmode().clear_bit()
                    .ibipayloaden().set_bit()
                    .enbl_i3cctrl().set_bit()
            });
            let wait_cnt = &self.i3c.i3cd0d4().read().i3cibifree().bits();
            let wait_ns = u32::from(*wait_cnt) * config.core_period;
            let mut delay = DummyDelay {};
            delay.delay_ns(wait_ns as u32);
            self.i3c_toggle_scl_in(8);
            if self.i3c.i3cd000().read().enbl_i3cctrl().bit_is_set() {
                self.gen_internal_stop();
            }
            self.exit_sw_mode();
        } else {
            self.i3c.i3cd000().modify(|_, w| {
                w.i3cbroadcast_addr_include().set_bit()
                    .enbl_i3cctrl().set_bit()
            });
        }
    }

    fn i3c_toggle_scl_in(&mut self, count:u32) {
        let bus = I3C::BUS_NUM;
        for _ in 0..count {
            modify_i3cg_reg1!(self, bus, |r, w| unsafe {
                w.bits(r.bits() & !I3CG_REG1_SCL_IN_SW_MODE_VAL)
            });
            modify_i3cg_reg1!(self, bus, |r, w| unsafe {
                w.bits(r.bits() | I3CG_REG1_SCL_IN_SW_MODE_VAL)
            });
        }
    }

    fn gen_internal_stop(&mut self) {
        let bus = I3C::BUS_NUM;
        modify_i3cg_reg1!(self, bus, |r, w| unsafe {
            w.bits(r.bits() & !I3CG_REG1_SCL_IN_SW_MODE_VAL)
        });
        modify_i3cg_reg1!(self, bus, |r, w| unsafe {
            w.bits(r.bits() & !I3CG_REG1_SDA_IN_SW_MODE_VAL)
        });
        modify_i3cg_reg1!(self, bus, |r, w| unsafe {
            w.bits(r.bits() | I3CG_REG1_SCL_IN_SW_MODE_VAL)
        });
        modify_i3cg_reg1!(self, bus, |r, w| unsafe {
            w.bits(r.bits() | I3CG_REG1_SDA_IN_SW_MODE_VAL)
        });
    }

    fn i3c_bus_init(&mut self, _config: &mut I3cConfig) {
    }

    fn even_parity(byte: u8) -> bool {
        let mut parity = false;
        let mut b = byte;

        while b != 0 {
            parity = !parity;
            b &= b - 1;
        }

        !parity
    }

    fn set_ibi_mdb(&mut self, mdb: u8) {
        self.i3c.i3cd000().modify(|_, w| unsafe { w.mdb().bits(mdb) });
    }

    fn exit_halt(&mut self, config: &mut I3cConfig) {
        let state = self.i3c.i3cd054().read().cmtfrstatus().bits();
        let expected = if config.is_secondary {
            CM_TFR_STS_TARGET_HALT
        } else {
            CM_TFR_STS_MASTER_HALT
        };

        if state != expected {
            return;
        }

        self.i3c.i3cd000().modify(|_, w| w.i3cresume().set_bit());

        let ret = poll_with_timeout(
            || u32::from(self.i3c.i3cd054().read().cmtfrstatus().bits()),
            |val| val != u32::from(expected),
            &mut DummyDelay {},
            10000,
            1_000_000,
        );

        if ret.is_err() {
            i3c_debug!(self.logger, "exit_halt: timeout");
        }
    }

    fn enter_halt(&mut self, by_sw: bool, config: &mut I3cConfig) {
        let expected = if config.is_secondary {
            CM_TFR_STS_TARGET_HALT
        } else {
            CM_TFR_STS_MASTER_HALT
        };

        if by_sw {
            self.i3c.i3cd000().modify(|_, w| w.i3cabort().set_bit());
        }

        let ret = poll_with_timeout(
            || u32::from(self.i3c.i3cd054().read().cmtfrstatus().bits()),
            |val| val == u32::from(expected),
            &mut DummyDelay {},
            10000,
            1_000_000,
        );

        if ret.is_err() {
            i3c_debug!(self.logger, "enter_halt: timeout");
        }
    }

    fn reset_ctrl(&mut self, reset: u32) {
        let reg = reset & RESET_CTRL_ALL;

        if reg == 0 {
            return;
        }

        self.i3c.i3cd034().write(|w| unsafe { w.bits(reg) });
        let ret = poll_with_timeout(
            || self.i3c.i3cd034().read().bits(),
            |val| val == 0,
            &mut DummyDelay {},
            10_000,
            1_000_000,
        );

        if ret.is_err() {
            i3c_debug!(self.logger, "reset_ctrl: timeout");
        }
    }

    fn wr_tx_fifo(&mut self, bytes: &[u8]) {
        let mut chunks = bytes.chunks_exact(4);
        for chunk in &mut chunks {
            let word = u32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
            self.i3c.i3cd014().write(|w| unsafe { w.tx_data_port().bits(word) });
        }

        let rem = chunks.remainder();
        if !rem.is_empty() {
            let mut tmp = [0u8; 4];
            tmp[..rem.len()].copy_from_slice(rem);
            let word = u32::from_le_bytes(tmp);
            self.i3c.i3cd014().write(|w| unsafe { w.tx_data_port().bits(word) });
        }
    }

    fn rd_fifo<F>(&mut self, mut read_word: F, out: &mut [u8])
    where
        F: FnMut() -> u32,
    {
        let mut chunks = out.chunks_exact_mut(4);
        for chunk in &mut chunks {
            let val = read_word();
            chunk.copy_from_slice(&val.to_le_bytes());
        }

        let rem = chunks.into_remainder();
        if !rem.is_empty() {
            let val = read_word();
            let bytes = val.to_le_bytes();
            rem.copy_from_slice(&bytes[..rem.len()]);
        }
    }

    fn rd_rx_fifo(&mut self, out: &mut [u8]) {
        self. rd_fifo(|| self.i3c.i3cd014().read().rx_data_port().bits(), out);
    }

    fn rd_ibi_fifo(&mut self, out: &mut [u8]) {
        self.rd_fifo(|| self.i3c.i3cd018().read().bits(), out);
    }

    fn ibi_enable(&mut self, config: &mut I3cConfig, dev_idx: usize) -> i32 {

        let target = config.devs.i3c_devices[dev_idx as usize];
        let pos: u8 = match target.i3c_priv_idx {
            Some(i) => config.privs[i as usize].pos,
            None => return -22, // or your error path
        };

        let tgt_bcr:u32 = target.bcr as u32;
        let mut reg = i3c_dat_read!(self, pos as u32);
        if tgt_bcr & I3C_BCR_IBI_PAYLOAD_HAS_DATA_BYTE != 0 {
            reg |= DEV_ADDR_TABLE_IBI_MDB | DEV_ADDR_TABLE_IBI_PEC;
        }

        i3c_dat_write!(self, pos as u32, |w| unsafe {
            w.bits(reg)
        });

        let mut sir_reject = self.i3c.i3cd030().read().bits();
        sir_reject &= !bit(pos.into());
        self.i3c.i3cd030().write(|w| unsafe { w.bits(sir_reject) });

        self.i3c.i3cd040().modify(|_, w| {
            w.ibithldstaten().set_bit()
        });

        self.i3c.i3cd044().modify(|_, w| {
            w.ibithldsignalen().set_bit()
        });

        let events = I3C_CCC_EVT_INTR as u8;
        self.ccc_do_events_set(config, target.dynamic_addr, true, events);

        0
    }

    fn start_xfer(&mut self, config: &mut I3cConfig, xfer: &mut I3cXfer) {

        let prev = config.curr_xfer.swap(core::ptr::from_mut(xfer).cast::<()>(), Ordering::AcqRel);
        debug_assert!(prev.is_null(), "previous xfer still in flight");

        xfer.ret = -1;
        xfer.done.reset();

        for cmd in xfer.cmds.iter() {
            if let Some(tx) = cmd.tx {
                let take = tx.len().min(cmd.tx_len as usize);
                if take > 0 {
                    self.wr_tx_fifo(&tx[..take]);
                }
            }
        }
        self.i3c.i3cd01c().modify(|_, w| unsafe {
            w.response_buffer_threshold_value().bits(xfer.ncmds() as u8 - 1)
        });

        for cmd in xfer.cmds.iter() {
            self.i3c.i3cd00c().write(|w| unsafe {
                w.bits(cmd.cmd_hi)
            });
            self.i3c.i3cd00c().write(|w| unsafe {
                w.bits(cmd.cmd_lo)
            });
        }
    }

    fn end_xfer(&mut self, config: &mut I3cConfig) {
        let p = config.curr_xfer.swap(core::ptr::null_mut(), Ordering::AcqRel);
        if p.is_null() {
            return;
        }
        let xfer: &mut I3cXfer = unsafe { &mut *(p.cast::<I3cXfer>()) };

        let nresp = self.i3c.i3cd04c().read().respbufblr().bits() as usize;

        for _ in 0..nresp {
            let resp = self.i3c.i3cd010().read().bits();

            let tid    = field_get(resp, RESPONSE_PORT_TID_MASK,        RESPONSE_PORT_TID_SHIFT)   as usize;
            let rx_len = field_get(resp, RESPONSE_PORT_DATA_LEN_MASK,   RESPONSE_PORT_DATA_LEN_SHIFT) as usize;
            let err    = field_get(resp, RESPONSE_PORT_ERR_STATUS_MASK, RESPONSE_PORT_ERR_STATUS_SHIFT) as i32;

            let cmd = &mut xfer.cmds[tid];
            cmd.rx_len = rx_len as u32;
            cmd.ret    = err;

            if rx_len > 0 && err == 0 {
                if let Some(rx_buf) = cmd.rx.as_deref_mut() {
                    self.rd_rx_fifo(&mut rx_buf[..rx_len]);
                }
            }
        }
        let mut ret = 0;
        for i in 0..(nresp as usize) {
            let r = xfer.cmds[i].ret;
            if r != 0 {
                ret = r;
            }
        }

        if ret != 0 {
            self.enter_halt(false, config);
            self.reset_ctrl(RESET_CTRL_QUEUES);
            self.exit_halt(config);
        }

        xfer.ret = ret;
        xfer.done.complete();
    }

    fn get_addr_pos(&mut self, config: &I3cConfig, addr: u8) -> Option<u8> {
        config
            .addrs
            .iter()
            .take(config.maxdevs as usize)
            .position(|&a| a == addr)
            .map(|i| i as u8)
    }

    fn detach_i3c_dev(&mut self, config: &mut I3cConfig, pos: u8) {

        config.free_pos |= 1u32 << pos;
        config.addrs[pos as usize] = 0;

        i3c_dat_write!(self, pos as u32, |w| {
            w.sirreject().set_bit()
                .mrreject().set_bit()
        });
    }

    fn attach_i3c_device(&mut self, config: &mut I3cConfig, target: &mut I3cDesc, addr: u8) -> I3cResult<()> {

        let pos: u32 = match find_lsb_pos(config.free_pos) {
            Some(p) if p < u32::from(config.maxdevs) => p,
            _ => return Err(I3cError::NoSpace),
        };

        let pos_usize = pos as usize;

        // Mark position as used and remember address
        config.free_pos &= !bit(pos);
        config.addrs[pos_usize] = addr;

        // Bind controller private
        config.privs[pos_usize].pos  = pos as u8;
        config.privs[pos_usize].addr = addr;
        target.i3c_priv_idx = Some(pos as u8);

        // Program DAT entry:
        // Byte address with parity (bit7 = even parity of 7-bit addr)
        let mut da_with_parity = addr;
        if Self::even_parity(addr) { da_with_parity |= 1 << 7; }

        i3c_dat_write!(self, pos, |w| unsafe {
            w.sirreject().set_bit()
                .mrreject().set_bit()
                .devdynamicaddr().bits(da_with_parity)
        });

        if target.dynamic_addr == 0 {
            config.need_da |= bit(pos);
        }

        Ok(())
    }

    fn do_ccc<'a, 'b>(&mut self, config: &mut I3cConfig, payload: &mut CccPayload<'a, 'b>) -> i32 {
        // init i3c_cmd to all 0
        let mut cmds = [I3cCmd {
            cmd_lo: 0,
            cmd_hi: 0,
            tx: None,
            rx: None,
            tx_len: 0,
            rx_len: 0,
            ret: 0,
        }];
        let dbp: bool;
        let db: u8;
        let mut rnw: bool = false;
        let mut pos: u8 = 0;
        let id: u8;
        let mut is_broadcast: bool = false;
        let mut ccc_num_xfer: Option<usize> = None;

        {
            let cmd = &mut cmds[0];

            // Take `ccc` out as a short-lived mutable borrow, copy what we need into locals
            let data_len = {
                let ccc = match payload.ccc.as_mut() {
                    Some(c) => c,
                    None => return -22, // EINVAL: missing CCC
                };

                // Peek defining byte (if any) without keeping a borrow alive
                (dbp, db) = match ccc.data.as_deref() {
                    Some(d) if !d.is_empty() => {
                        // For direct CCC, defining byte must be exactly 1 byte;
                        // for broadcast, multiple bytes are allowed (payload write).
                        (true, d[0])
                    }
                    _ => (false, 0),
                };

                id = ccc.id;
                let len = ccc.data.as_deref().map(|d| d.len()).unwrap_or(0);
                len
            };

            if id <= 0x7F {
                // Broadcast CCC
                is_broadcast = true;
                if data_len > 0 {
                    if let Some(ccc_ro) = payload.ccc.as_ref() {
                        if let Some(d) = ccc_ro.data.as_deref() {
                            cmd.tx = Some(d);
                            cmd.tx_len = data_len as u32;
                            ccc_num_xfer = Some(data_len); // optimistic assume all bytes written
                        }
                    }
                }
            } else {
                let addr = {
                    let tp = match payload.targets.as_deref_mut().and_then(|ts| ts.first_mut()) {
                        Some(tp) => tp,
                        None => return -22, // EINVAL: no target
                    };

                    if tp.rnw {
                        if let Some(d) = tp.data.as_deref_mut() {
                            if d.len() == 0 {
                                return -22; // EINVAL: missing read buffer
                            }
                        } else {
                            return -22; // EINVAL: missing read buffer
                        }
                        let len = tp.data.as_deref().map(|d| d.len()).unwrap_or(0);
                        cmd.rx_len = len as u32;
                        cmd.rx = tp.data.as_deref_mut();
                    } else {
                        let d = match tp.data.as_deref() {
                            Some(d) if !d.is_empty() => d,
                            _ => return -22, // EINVAL: missing write data
                        };
                        let len = d.len();
                        cmd.tx_len = len as u32;
                        cmd.tx = Some(d);
                        tp.num_xfer = len; // optimistic assume all bytes written
                    }
                    rnw = tp.rnw;
                    tp.addr
                }; // target borrow ends here

                pos = match self.get_addr_pos(config, addr) {
                    Some(p) => p,
                    None => return -22,
                };
            }
        }

        let cmd = &mut cmds[0];
        cmd.cmd_hi = field_prep(COMMAND_PORT_ATTR, COMMAND_ATTR_XFER_ARG as u32)
            | field_prep(COMMAND_PORT_ARG_DB, db.into());

        if rnw == true {
            cmd.cmd_hi |= field_prep(COMMAND_PORT_ARG_DATA_LEN, cmd.rx_len);
        } else {
            cmd.cmd_hi |= field_prep(COMMAND_PORT_ARG_DATA_LEN, cmd.tx_len);
        }

        cmd.cmd_lo = field_prep(COMMAND_PORT_ATTR, COMMAND_ATTR_XFER_CMD as u32)
            | field_prep(COMMAND_PORT_DEV_INDEX, pos as u32)
            | field_prep(COMMAND_PORT_CMD, id.into())
            | field_prep(COMMAND_PORT_READ_TRANSFER, if rnw { 1 } else { 0 })
            | COMMAND_PORT_CP | COMMAND_PORT_ROC | COMMAND_PORT_TOC;

        if dbp {
            cmd.cmd_lo |= COMMAND_PORT_DBP;
        }

        if id == I3C_CCC_SETHID || id == I3C_CCC_DEVCTRL {
            cmd.cmd_lo |= field_prep(COMMAND_PORT_SPEED, SpeedI3c::I2cFmAsI3c as u32);
        }

        let mut xfer = I3cXfer::new(&mut cmds[..]);
        self.start_xfer(config, &mut xfer);
        let mut delay = DummyDelay {};

        if !xfer.done.wait_for_us(10_000, &mut delay) {
            self.enter_halt(true, config);
            self.reset_ctrl(RESET_CTRL_XFER_QUEUES);
            self.exit_halt(config);
            return -1;
        }

        let ret = xfer.ret;
        if ret == RESPONSE_ERROR_IBA_NACK.try_into().unwrap() {
            return 0
        }

        drop(xfer);

        if let (true, Some(n)) = (is_broadcast, ccc_num_xfer) {
            if let Some(ccc_rw) = payload.ccc.as_mut() {
                ccc_rw.num_xfer = n;
            }
        }

        ret
    }

    fn do_entdaa(&mut self, config: &mut I3cConfig, index: u32) -> i32 {
        let cmd = I3cCmd {
            cmd_lo: field_prep(COMMAND_PORT_ATTR,        COMMAND_ATTR_ADDR_ASSGN_CMD as u32)
                | field_prep(COMMAND_PORT_CMD,         I3C_CCC_ENTDAA as u32)
                | field_prep(COMMAND_PORT_DEV_COUNT,   1)
                | field_prep(COMMAND_PORT_DEV_INDEX,   index as u32)
                | COMMAND_PORT_ROC
                | COMMAND_PORT_TOC,
                cmd_hi: field_prep(COMMAND_PORT_ATTR, COMMAND_ATTR_XFER_ARG as u32),
                tx: None,
                rx: None,
                tx_len: 0,
                rx_len: 0,
                ret: 0,
        };

        let mut cmds = [cmd];
        let mut xfer = I3cXfer::new(&mut cmds[..]);
        xfer.ret = -1;

        self.start_xfer(config, &mut xfer);

        let mut delay = DummyDelay {};

        if !xfer.done.wait_for_us(10_000, &mut delay) {
            self.enter_halt(true, config);
            self.reset_ctrl(RESET_CTRL_XFER_QUEUES);
            self.exit_halt(config);
            return -1;
        }

        xfer.ret
    }


    fn bytes_to_pid(bytes: &[u8]) -> u64 {
        bytes.iter()
            .take(6)
            .fold(0u64, |acc, &b| (acc << 8) | b as u64)
    }

    fn handle_unsolicited(&mut self, _config: &mut I3cConfig) {
        // todo
    }

    fn do_daa(&mut self, config: &mut I3cConfig) -> i32 {
        let mut need_da = config.need_da;
        if need_da == 0 {
            return 0;
        }

        let maxdevs = config.maxdevs as usize;
        let mut pos: usize = 0;

        while need_da != 0 {
            if (need_da & bit(pos as u32)) == 0 {
                pos = (pos + 1) % maxdevs;
                continue;
            }

            let addr = config.addrs[pos];

            let alive = {
                self.ccc_do_getstatus_fmt1(config, addr).is_ok()
            };
            if alive {
                need_da &= !bit(pos as u32);
                pos = (pos + 1) % maxdevs;
                continue;
            }

            let entdaa_ret = self.do_entdaa(config, pos as u32);
            if entdaa_ret != 0 {
                if entdaa_ret == RESPONSE_ERROR_IBA_NACK as i32 || entdaa_ret == -1 {
                    break;
                }
                pos = (pos + 1) % maxdevs;
                continue;
            }

            let pid = {
                let mut pid_buf = [0u8; 6];
                let mut tgt_pl = CccTargetPayload {
                    addr,
                    rnw: true,
                    data: Some(&mut pid_buf[..]),
                    num_xfer: 0,
                };
                let mut payload = CccPayload {
                    ccc: Some(Ccc { id: I3C_CCC_GETPID, data: None, num_xfer: 0 }),
                    targets: Some(core::slice::from_mut(&mut tgt_pl)),
                };
                let _ = self.do_ccc(config, &mut payload);
                Self::bytes_to_pid(&pid_buf)
            };

            let dev_id = I3cDeviceId::new(pid);

            if let Some(idx) = config.devs.find_index_by_pid(dev_id) {
                {
                    let t = &mut config.devs.i3c_devices[idx];
                    i3c_debug!(self.logger,
                        "Device {:012x} DA {:02x} assigned (was {:02x})",
                        pid, addr, t.dynamic_addr);
                    t.dynamic_addr = addr;
                }

                let (expected_da, expected_pos) = {
                    let t = &config.devs.i3c_devices[idx];
                    match t.i3c_priv_idx {
                        Some(pidx) => {
                            let pr = config.privs[pidx as usize];
                            (pr.addr, pr.pos)
                        }
                        None => {
                            need_da &= !bit(pos as u32);
                            pos = (pos + 1) % maxdevs;
                            continue;
                        }
                    }
                };

                if addr != expected_da {
                    config.devs.addr_slots.set_status(expected_da, I3cAddrSlotStatus::Free);

                    let set_ret = {
                        self.ccc_do_setnewda(config, addr, expected_da)
                    };

                    if set_ret == 0 {
                        config.devs.addr_slots.set_status(expected_da, I3cAddrSlotStatus::I3cDev);
                        need_da &= !bit(expected_pos as u32);
                        need_da |= bit(pos as u32);

                        {
                            let t = &mut config.devs.i3c_devices[idx];
                            t.dynamic_addr = expected_da;
                        }

                        i3c_debug!(self.logger,
                            "Device {:012x} new DA {:02x} assigned (was {:02x})",
                            pid, expected_da, addr);
                    } else {
                        i3c_debug!(self.logger,
                            "SETNEWDA failed: expect {:02x} for {:012x}",
                            expected_da, pid);
                    }
                } else {
                    need_da &= !bit(pos as u32);
                }

                let final_da = {
                    let t = &config.devs.i3c_devices[idx];
                    t.dynamic_addr
                };
                let bcr = self.ccc_do_getbcr(config, final_da);
                if let Ok(b) = bcr {
                    let t = &mut config.devs.i3c_devices[idx];
                    t.bcr = b;
                }
                self.ibi_enable(config, idx);
            } else {
                i3c_debug!(self.logger, "Unsolicited device with PID {:012x}", pid);
                continue;
            }

            pos = (pos + 1) % maxdevs;
        }

        0
    }

    fn priv_xfer_build_cmds<'a>(
        &mut self,
        cmds: &mut [I3cCmd<'a>],
        msgs: &mut [I3cMsg<'a>],
        pos: u8,
    ) -> i32 {

        let cmds_len = cmds.len();
        if cmds_len != msgs.len() {
            return -22; // EINVAL
        }

        for i in 0..cmds_len {
            let (is_read, ptr, len) = {
                let m = &mut msgs[i];
                let is_read = (m.flags & I3C_MSG_READ) != 0;

                if is_read {
                    let buf = match m.buf.as_deref_mut() {
                        Some(b) if !b.is_empty() => b,
                        _ => return -22, // EINVAL
                    };
                    (true, buf.as_mut_ptr(), buf.len())
                } else {
                    let buf = match m.buf.as_deref() {
                        Some(b) if !b.is_empty() => b,
                        _ => return -22, // EINVAL
                    };
                    m.num_xfer = buf.len() as u32;
                    (false, buf.as_ptr() as *mut u8, buf.len())
                }
            };

            let cmd = &mut cmds[i];
            *cmd = I3cCmd {
                cmd_hi: field_prep(COMMAND_PORT_ATTR, COMMAND_ATTR_XFER_ARG as u32)
                    | field_prep(COMMAND_PORT_ARG_DATA_LEN, len as u32),
                    cmd_lo: field_prep(COMMAND_PORT_TID, i as u32)
                        | field_prep(COMMAND_PORT_DEV_INDEX, pos as u32)
                        | COMMAND_PORT_ROC,
                        tx: None,
                        rx: None,
                        tx_len: 0,
                        rx_len: 0,
                        ret: 0,
            };

            if is_read {
                let rx_slice: &'a mut [u8] = unsafe { core::slice::from_raw_parts_mut(ptr, len) };
                cmd.rx = Some(rx_slice);
                cmd.rx_len = len as u32;
                cmd.cmd_lo |= COMMAND_PORT_READ_TRANSFER;
            } else {
                let tx_slice: &'a [u8] = unsafe { core::slice::from_raw_parts(ptr as *const u8, len) };
                cmd.tx = Some(tx_slice);
                cmd.tx_len = len as u32;
            }

            let is_last = i + 1 == cmds_len;
            if is_last {
                cmd.cmd_lo |= COMMAND_PORT_TOC;
            }
        }

        0
    }

    fn priv_xfer(&mut self, config: &mut I3cConfig, target: &mut I3cDesc, msgs: &mut [I3cMsg]) -> i32 {
        if msgs.is_empty() {
            return 0
        }

        if target.dynamic_addr == 0 {
            return -22
        }

        let pos: u8 = match target.i3c_priv_idx {
            Some(i) => config.privs[i as usize].pos,
            None => return -22, // or your error path
        };

        if msgs.len() > MAX_CMDS {
            return -22
        }

        let mut cmds: heapless::Vec<I3cCmd, MAX_CMDS> = heapless::Vec::new();

        for _ in 0..msgs.len() {
            cmds.push(I3cCmd {
                cmd_lo: 0,
                cmd_hi: 0,
                tx: None,
                rx: None,
                tx_len: 0,
                rx_len: 0,
                ret: 0,
            }).unwrap();
        }

        let ret = self.priv_xfer_build_cmds(&mut cmds.as_mut_slice(), msgs, pos);
        if ret != 0 {
            return ret;
        }

        let mut xfer = I3cXfer::new(cmds.as_mut_slice());
        self.start_xfer(config, &mut xfer);

        let mut delay = DummyDelay {};
        if !xfer.done.wait_for_us(10_000, &mut delay) {
            self.enter_halt(true, config);
            self.reset_ctrl(RESET_CTRL_XFER_QUEUES);
            self.exit_halt(config);
            return -1;
        }

        for (i, m) in msgs.iter_mut().enumerate() {
            if (m.flags & I3C_MSG_READ) != 0 {
                m.actual_len = xfer.cmds[i].rx_len;
            }
        }

        let ret = xfer.ret;

        ret
    }

    fn target_tx_write(&mut self, buf: &[u8]) {
        self.wr_tx_fifo(buf);
        let _len = buf.len() as u32;
        // self.i3c.i3cd00c().write(|w| unsafe {
        //     w.bits(cmd);
        // });
    }

    fn drain_fifo<F>(&mut self, mut read_word: F, len: usize)
    where
        F: FnMut() -> u32,
    {
        let nwords = (len + 3) >> 2;
        for _ in 0..nwords {
            let _ = read_word();
        }
    }
    fn handle_ibi_sir(&mut self, _config: &mut I3cConfig) {
        // todo
    }
    fn handle_ibis(&mut self, config: &mut I3cConfig) {
        let nibis = self.i3c.i3cd04c().read().ibistatuscnt().bits();

        if nibis == 0 {
            return;
        }

        for _ in 0..nibis {
            let ibi_id = self.i3c.i3cd018().read().ibiidentifier().bits();
            let ibi_addr = ibi_id >> 1 | 0x7E;
            if ibi_addr != 2 && ibi_id & 1 == 1 {
                // sir
                self.handle_ibi_sir(config);
            } else if ibi_addr == 2 && ibi_id & 1 == 0 {
                // hot-join
            } else {
                // normal ibi
                let len = self.i3c.i3cd018().read().in_band_intdata_len().bits() as usize;
                self.drain_fifo(|| self.i3c.i3cd018().read().bits(), len);
            }

        }
    }

    fn i3c_aspeed_isr(&mut self, config: &mut I3cConfig) {
        let status = self.i3c.i3cd03c().read().bits();
        if status == 0 {
            return;
        }

        if config.is_secondary {
            if status & INTR_DYN_ADDR_ASSGN_STAT != 0 {
                let _dyn_addr = self.i3c.i3cd004().read().dev_dynamic_addr().bits();
                //todo
            }

            if (status & INTR_RESP_READY_STAT) != 0 {
                //todo
            }

            if (status & INTR_CCC_UPDATED_STAT) != 0 {
                //todo
            }
        } else {
            if (status & INTR_RESP_READY_STAT) != 0 || (status & INTR_TRANSFER_ERR_STAT) != 0 {
                self.end_xfer(config)
            }

            if (status & INTR_IBI_THLD_STAT) != 0 {
                self.handle_ibis(config);
            }
        }

        self.i3c.i3cd03c().write(|w| unsafe { w.bits(status) } );
    }

    fn ccc_do_events_set(&mut self, config: &mut I3cConfig, da: u8, enable: bool, events: u8) -> i32 {

        if da == 0 {
            return -22
        }

        let mut ev_buf = [events];
        let tgt = CccTargetPayload {
            addr: da,
            rnw: false,
            data: Some(&mut ev_buf[..]),
            num_xfer: 0,
        };

        let mut tgts = [tgt];
        let ccc_id = if enable { i3c_ccc_enec(false) } else { i3c_ccc_disec(false) };
        let ccc = Ccc {
            id: ccc_id,
            data: None,
            num_xfer: 0,
        };

        let mut payload = CccPayload {
            ccc: Some(ccc),
            targets: Some(&mut tgts[..]),
        };

        self.do_ccc(config, &mut payload);

        return 0
    }

    fn ccc_do_getstatus( &mut self, config: &mut I3cConfig, da: u8, fmt: GetStatusFormat,) -> Result<GetStatusResp, i32> {

        let mut data_buf = [0u8; 2];

        let mut defbyte_buf = [0u8; 1];

        let tgt = CccTargetPayload {
            addr: da,
            rnw: true,
            data: Some(&mut data_buf[..]),
            num_xfer: 0,
        };

        let mut ccc = Ccc {
            id: I3C_CCC_GETSTATUS,
            data: None,
            num_xfer: 0,
        };

        let kind_opt = match fmt {
            GetStatusFormat::Fmt1 => None,
            GetStatusFormat::Fmt2(kind) => {
                defbyte_buf[0] = kind.as_byte();
                ccc.data = Some(&mut defbyte_buf[..]);
                Some(kind)
            }
        };

        let mut targets_arr = [tgt];
        let mut payload = CccPayload {
            ccc: Some(ccc),
            targets: Some(&mut targets_arr[..]),
        };

        let ret = self.do_ccc(config, &mut payload);
        if ret != 0 {
            return Err(ret);
        }

        let val = u16::from_be_bytes(data_buf);

        let resp = match kind_opt {
            None => GetStatusResp::Fmt1 { status: val },
            Some(kind) => GetStatusResp::Fmt2 { kind, raw_u16: val },
        };

        Ok(resp)
    }

    fn ccc_do_getpid(&mut self, config: &mut I3cConfig, dyn_addr: u8) -> Result<u64, i32> {
        let mut pid_buf = [0u8; 6];

        let tgt = CccTargetPayload {
            addr: dyn_addr,
            rnw: true,
            data: Some(&mut pid_buf[..]),
            num_xfer: 0,
        };
        let mut tgts = [tgt];

        let ccc = Ccc { id: I3C_CCC_GETPID, data: None, num_xfer: 0 };
        let mut payload = CccPayload { ccc: Some(ccc), targets: Some(&mut tgts[..]) };

        let ret = self.do_ccc(config, &mut payload);
        if ret != 0 {
            return Err(ret);
        }
        Ok(Self::bytes_to_pid(&pid_buf))
    }
    fn ccc_do_setnewda(
        &mut self,
        config: &mut I3cConfig,
        curr_da: u8,
        new_da: u8,
    ) -> i32 {

        if curr_da == 0 || new_da == 0 {
            return -22; // -EINVAL
        }

        let idx = match config.devs.find_index_by_dyn(curr_da) {
            Some(i) => i,
            None => return -22,
        };

        let (old_da, new_slot_free) = {
            let t = &config.devs.i3c_devices[idx];
            if t.dynamic_addr == 0 {
                return -22;
            }
            let status = config.devs.addr_slots.status(new_da);
            (t.dynamic_addr, status == I3cAddrSlotStatus::Free)
        };

        if new_da != old_da && !new_slot_free {
            return -22;
        }

        let mut new_dyn_addr = (new_da & 0x7F) << 1;
        let tgt = CccTargetPayload {
            addr: old_da,
            rnw: false,
            data: Some(core::slice::from_mut(&mut new_dyn_addr)),
            num_xfer: 0,
        };
        let mut tgts = [tgt];
        let ccc = Ccc { id: I3C_CCC_SETNEWDA, data: None, num_xfer: 0 };
        let mut payload = CccPayload { ccc: Some(ccc), targets: Some(&mut tgts[..]) };

        let ret = self.do_ccc(config, &mut payload);
        if ret != 0 {
            return ret;
        }

        {
            let t = &mut config.devs.i3c_devices[idx];
            let prev = t.dynamic_addr;
            if prev != new_da {
                config.devs.addr_slots.set_status(prev, I3cAddrSlotStatus::Free);
                config.devs.addr_slots.set_status(new_da, I3cAddrSlotStatus::I3cDev);
            }
            t.dynamic_addr = new_da;
        }

        0
    }

    fn ccc_do_getstatus_fmt1(&mut self, config: &mut I3cConfig, da: u8) -> Result<u16, i32> {
        match self.ccc_do_getstatus(config, da, GetStatusFormat::Fmt1) {
            Ok(GetStatusResp::Fmt1 { status }) => Ok(status),
            Ok(_) => Err(-22),
            Err(e) => Err(e),
        }
    }

    fn ccc_do_getbcr(&mut self, config: &mut I3cConfig, dyn_addr: u8) -> Result<u8, i32> {
        if dyn_addr == 0 {
            return Err(-22); // -EINVAL
        }

        let mut bcr_buf = [0u8; 1];

        let tgt = CccTargetPayload {
            addr: dyn_addr,
            rnw: true,
            data: Some(&mut bcr_buf[..]),
            num_xfer: 0,
        };
        let mut tgts = [tgt];

        let ccc = Ccc { id: I3C_CCC_GETBCR, data: None, num_xfer: 0 };
        let mut payload = CccPayload { ccc: Some(ccc), targets: Some(&mut tgts[..]) };

        let ret = self.do_ccc(config, &mut payload);
        if ret != 0 {
            return Err(ret);
        }

        Ok(bcr_buf[0])
    }

}
pub const fn i3c_ccc_enec(broadcast: bool) -> u8 {
    if broadcast { 0x00 } else { 0x80 }
}

pub const fn i3c_ccc_disec(broadcast: bool) -> u8 {
    if broadcast { 0x01 } else { 0x81 }
}

