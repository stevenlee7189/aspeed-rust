// Licensed under the Apache-2.0 license

use core::ptr::read_volatile;
use crate::uart::{self, Config, UartController};
use crate::common::{DummyDelay, NoOpLogger, UartLogger};
use crate::pinctrl;
use ast1060_pac::Peripherals;
use embedded_io::Write;
use crate::i3c::ast1060_i3c::{self, i3c, Ast1060I3c};
use crate::i3c::ast1060_i3c::I3cController;
use crate::i3c::ast1060_i3c::I3cConfig;
use crate::i3c::ast1060_i3c::HardwareInterface;
use crate::i3c::ast1060_i3c::i3c_ibi_workq_consumer;
use crate::i3c::ast1060_i3c::IbiWork;

pub fn test_i3c_master(uart: &mut UartController<'_>) {
    let peripherals = unsafe { Peripherals::steal() };
    let mut delay = DummyDelay {};
    let mut dbg_uart = UartController::new(peripherals.uart, &mut delay);

    writeln!(uart, "\r\n####### I3C master test #######\r\n").unwrap();
    unsafe {
        dbg_uart.init(&Config {
            baud_rate: 115_200,
            word_length: uart::WordLength::Eight as u8,
            parity: uart::Parity::None,
            stop_bits: uart::StopBits::One,
            clock: 24_000_000,
        });
    }

    pinctrl::Pinctrl::apply_pinctrl_group(pinctrl::PINCTRL_I3C2);
    let hw = Ast1060I3c::<ast1060_pac::I3c2, UartLogger>::new(UartLogger::new(&mut dbg_uart));

    let mut ctrl = I3cController { hw, config: I3cConfig::new(), logger: NoOpLogger };

    {
        let c = &mut ctrl.config;
        c.init_runtime_fields();
        c.is_secondary = false;
        c.i2c_scl_hz = 1000_000;
        c.i3c_scl_hz = 12_500_000;
        c.i3c_pp_scl_hi_period_ns = 250;
        c.i3c_pp_scl_lo_period_ns = 250;
        c.i3c_od_scl_hi_period_ns = 0;
        c.i3c_od_scl_lo_period_ns = 0;
        c.sda_tx_hold_ns = 20;

    }

    let mut ibi_cons = i3c_ibi_workq_consumer(ctrl.hw.bus_num() as usize);
    let known_pid = 0x07ec_0503_1000u64;
    let ctrl_dev_slot0 = 0;
    // let ctrl_dev_slot1 = 1;
    // let dyn_addr = 8;
    ctrl.init();

    // ctrl.attach_i3c_dev(known_pid, dyn_addr, ctrl_dev_slot0).unwrap();
    let dyn_addr = match ctrl.config.addrbook.alloc_from(8) {
        Some(da) => {
            ctrl.attach_i3c_dev(known_pid, da, ctrl_dev_slot0).unwrap();
            writeln!(uart, "~~~~pre-attached dev at slot 0, dyn addr {}\r", da).unwrap();
            da
        }
        None => {
            writeln!(uart, "no dyn addr\r").unwrap();
            return;
        }
    };

    writeln!(uart, "ctrl dev at slot 0, dyn addr {}\r", dyn_addr).unwrap();
    loop {
        if let Some(work) = ibi_cons.dequeue() {
            match work {
                IbiWork::HotJoin => {
                    writeln!(uart, "[IBI] hotjoin\r").unwrap();
                    let _ = ctrl.hw.do_entdaa(&mut ctrl.config, ctrl_dev_slot0.try_into().unwrap());
                    writeln!(uart, "  entdaa done\r").unwrap();
                    let pid = ctrl.hw.ccc_do_getpid(&mut ctrl.config, dyn_addr);
                    match pid {
                        Ok(pid) => {
                            writeln!(uart, "  dev pid 0x{:x}\r", pid).unwrap();
                        }
                        Err(e) => {
                            writeln!(uart, "  getpid err {}\r", e).unwrap();
                        }
                    }
                    let bcr = ctrl.hw.ccc_do_getbcr(&mut ctrl.config, dyn_addr);
                    match bcr {
                        Ok(bcr) => {
                            writeln!(uart, "  dev bcr 0x{:02x}\r", bcr).unwrap();
                        }
                        Err(e) => {
                            writeln!(uart, "  getbcr err {}\r", e).unwrap();
                        }
                    }
                    let dev_idx = ctrl.config.attached.find_dev_idx_by_addr(dyn_addr).unwrap();
                    ctrl.config.attached.devices[dev_idx].bcr = bcr.unwrap_or(0);
                    let _ = ctrl.hw.ibi_enable(&mut ctrl.config, dyn_addr);
                }
                IbiWork::Sirq { addr, len, data } => {
                    writeln!(uart, "[IBI] SIRQ from 0x{:02x}, len {}\r", addr, len).unwrap();
                    writeln!(uart, "  IBI payload:").unwrap();
                    for i in 0..len {
                        write!(uart, " {:02x}", data[i as usize]).unwrap();
                    }
                    writeln!(uart, "\r").unwrap();
                    let mut rx_buf = [0u8; 128];
                    let mut msgs = [
                        ast1060_i3c::I3cMsg {

                            buf: Some(&mut rx_buf[..]),
                            actual_len: 128,
                            num_xfer: 0,
                            flags: ast1060_i3c::I3C_MSG_READ | ast1060_i3c::I3C_MSG_STOP,
                            hdr_mode: 0,
                            hdr_cmd_mode: 0,
                        }
                    ];
                    let _ = ctrl.hw.priv_xfer(&mut ctrl.config, known_pid, &mut msgs);
                    writeln!(uart, "  read {} bytes\r", msgs[0].actual_len).unwrap();
                    writeln!(uart, "  read data:").unwrap();
                    for i in 0..msgs[0].actual_len {
                        write!(uart, " {:02x}", rx_buf[i as usize]).unwrap();
                    }
                    writeln!(uart, "\r").unwrap();
                }
            }
        }
    }
}

pub fn test_i3c_target(uart: &mut UartController<'_>) {
    let peripherals = unsafe { Peripherals::steal() };
    let mut delay = DummyDelay {};
    let mut dbg_uart = UartController::new(peripherals.uart, &mut delay);

    writeln!(uart, "\r\n####### I3C target test #######\r\n").unwrap();
    unsafe {
        dbg_uart.init(&Config {
            baud_rate: 115_200,
            word_length: uart::WordLength::Eight as u8,
            parity: uart::Parity::None,
            stop_bits: uart::StopBits::One,
            clock: 24_000_000,
        });
    }

    pinctrl::Pinctrl::apply_pinctrl_group(pinctrl::PINCTRL_I3C2);
    let hw = Ast1060I3c::<ast1060_pac::I3c2, UartLogger>::new(UartLogger::new(&mut dbg_uart));

    let mut ctrl = I3cController { hw, config: I3cConfig::new(), logger: NoOpLogger };

    {
        let c = &mut ctrl.config;
        c.init_runtime_fields();
        // Configure as target
        c.is_secondary = true;
        c.i2c_scl_hz = 1000_000;
        c.i3c_scl_hz = 12_500_000;
        c.i3c_pp_scl_hi_period_ns = 250;
        c.i3c_pp_scl_lo_period_ns = 250;
        c.i3c_od_scl_hi_period_ns = 0;
        c.i3c_od_scl_lo_period_ns = 0;
        c.sda_tx_hold_ns = 20;
        c.dcr = 0xcc;

    }
    let mut ibi_cons = i3c_ibi_workq_consumer(ctrl.hw.bus_num() as usize);
    ctrl.init();
    unsafe {
        let reg_base = 0x7e7a_4000 as *mut u32;
        // [7e7a4000] 80000200 00008009 000f40bb 00000000
        // [7e7a4010] 00000000 00000000 00000000 001f0000
        // [7e7a4020] 01010001 00000000 00000000 ffffffff
        let mut preg = 0x7e7a_4000;
        writeln!(uart, "rust I3C2 reg dump:\r").unwrap();
        for i in 0..0xc0 {
            let v = read_volatile(reg_base.add(i));
            if i % 4 == 0 {
                write!(uart, "[{:08x}]", preg);
                preg += 0x10;
            }
            write!(uart, " {:08x}", v).unwrap();
            if i % 4 == 3 {
                writeln!(uart, "\r").unwrap();
            }
        }
    }
    loop {
        if let Some(work) = ibi_cons.dequeue() {
            match work {
                IbiWork::HotJoin => {
                    writeln!(uart, "[IBI] hotjoin\r").unwrap();
                }
                IbiWork::Sirq { addr, len, data } => {
                    writeln!(uart, "[IBI] SIRQ from 0x{:02x}, len {}\r", addr, len).unwrap();
                }
            }
        }
    }
}
