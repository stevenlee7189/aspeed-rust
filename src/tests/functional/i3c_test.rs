// Licensed under the Apache-2.0 license

use core::ptr::read_volatile;
use crate::uart::{self, Config, UartController};
use crate::common::{DummyDelay, NoOpLogger, UartLogger};
use crate::pinctrl;
use ast1060_pac::Peripherals;
use embedded_io::Write;
use crate::i3c::i3c_controller::I3cController;
use crate::i3c::i3c_config::I3cConfig;
use crate::i3c::i3c_config::I3cTargetConfig;
use crate::i3c::ibi_workq::{IbiWork, i3c_ibi_workq_consumer};
use crate::i3c::ast1060_i3c::{Ast1060I3c, I3C_MSG_READ, I3C_MSG_STOP};
use crate::i3c::ast1060_i3c::HardwareInterface;
use crate::i3c::ast1060_i3c::I3cMsg;
use crate::i3c::ccc;
use embedded_hal::delay::DelayNs;

// I3cTarget
use crate::i3c::ast1060_i3c::I3cIbi;
use crate::i3c::ast1060_i3c::I3cIbiType;

pub fn dump_i3c_controller_registers(uart: &mut UartController<'_>, base: u32) {
    // [7e7a4000] 80000200 00008009 000f40bb 00000000
    // [7e7a4010] 00000000 00000000 00000000 001f0000
    // [7e7a4020] 01010001 00000000 00000000 ffffffff
    unsafe {
        let reg_base = base as *mut u32;
        writeln!(uart, "rust I3C reg dump:\r").unwrap();
        for i in 0..0xc0 {
            let v = read_volatile(reg_base.add(i));
            if i % 4 == 0 {
                write!(uart, "[{:08x}]", base + i as u32 * 4).unwrap();
            }
            write!(uart, " {:08x}", v).unwrap();
            if i % 4 == 3 {
                writeln!(uart, "\r").unwrap();
            }
        }
    }
}

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
    ctrl.init();

    let dyn_addr = match ctrl.config.addrbook.alloc_from(8) {
        Some(da) => {
            ctrl.attach_i3c_dev(known_pid, da, ctrl_dev_slot0).unwrap();
            writeln!(uart, "Pre-attached dev at slot 0, dyn addr {}\r", da).unwrap();
            da
        }
        None => {
            writeln!(uart, "no dyn addr\r").unwrap();
            return;
        }
    };

    // dump_i3c_controller_registers(uart, 0x7e7a_4000);
    writeln!(uart, "ctrl dev at slot 0, dyn addr {}\r", dyn_addr).unwrap();
    loop {
        if let Some(work) = ibi_cons.dequeue() {
            match work {
                IbiWork::HotJoin => {
                    writeln!(uart, "[IBI] hotjoin\r").unwrap();
                    let _ = ctrl.hw.do_entdaa(&mut ctrl.config, ctrl_dev_slot0.try_into().unwrap());
                    let pid = ccc::ccc_getpid(&mut ctrl.hw, &mut ctrl.config, dyn_addr);
                    match pid {
                        Ok(pid) => {
                            writeln!(uart, "  dev pid 0x{:x}\r", pid).unwrap();
                        }
                        Err(e) => {
                            writeln!(uart, "  getpid err {}\r", e).unwrap();
                        }
                    }
                    let bcr = ccc::ccc_getbcr(&mut ctrl.hw, &mut ctrl.config, dyn_addr);
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
                        I3cMsg {

                            buf: Some(&mut rx_buf[..]),
                            actual_len: 128,
                            num_xfer: 0,
                            flags: I3C_MSG_READ | I3C_MSG_STOP,
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
                IbiWork::TargetDaAssignment => {
                    writeln!(uart, "[IBI] TargetDaAssignment\r").unwrap();
                }
            }
        }
    }
}

fn crc8_ccitt(mut crc: u8, data: &[u8]) -> u8 {
    for &b in data {
        let mut x = crc ^ b;
        for _ in 0..8 {
            x = if (x & 0x80) != 0 { (x << 1) ^ 0x07 } else { x << 1 };
        }
        crc = x;
    }
    crc
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
        c.i3c_pp_scl_hi_period_ns = 36;
        c.i3c_pp_scl_lo_period_ns = 36;
        c.i3c_od_scl_hi_period_ns = 0;
        c.i3c_od_scl_lo_period_ns = 0;
        c.sda_tx_hold_ns = 20;
        c.dcr = 0xcc;
        c.target_config = Some(I3cTargetConfig::new(0, Some(0), 0xae));
    }
    let mut ibi_cons = i3c_ibi_workq_consumer(ctrl.hw.bus_num() as usize);
    ctrl.init();
    let dyn_addr = 8;
    let dev_idx = 0;
    ctrl.hw.attach_i3c_dev(dev_idx, dyn_addr);
    // Dump I3C2 registers
    dump_i3c_controller_registers(uart, 0x7e7a_4000);
    loop {
        if let Some(work) = ibi_cons.dequeue() {
            match work {
                IbiWork::HotJoin => {
                    // do nothing in target mode
                    writeln!(uart, "[IBI] hotjoin\r").unwrap();
                }
                IbiWork::Sirq { addr, len, data: _ } => {
                    // do nothing in target mode
                    writeln!(uart, "[IBI] SIRQ from 0x{:02x}, len {}\r", addr, len).unwrap();
                }
                IbiWork::TargetDaAssignment => {

                    let mut delay = DummyDelay {};
                    delay.delay_ns(4_000_000_000);
                    writeln!(uart, "[IBI] TargetDaAssignment\r").unwrap();
                    writeln!(uart, "  allow SIR by SW\r").unwrap();
                    ctrl.config.sir_allowed_by_sw = true;
                    let da = ctrl.config.target_config.as_ref().unwrap().addr;
                    let mdb = ctrl.config.target_config.as_ref().unwrap().mdb;
                    let addr_rnw;
                    if let Some(da_val) = da {
                        addr_rnw = (da_val << 1) | 0x1;
                    } else {
                        writeln!(uart, "  no dyn addr\r").unwrap();
                        return;
                    }
                    let mut pec = crc8_ccitt(0, &[addr_rnw]);
                    pec = crc8_ccitt(pec, &[mdb]);
                    writeln!(uart, "  assigned dyn addr 0x{:02x}, mdb 0x{:02x}, pec 0x{:02x}\r", da.unwrap(), mdb, pec).unwrap();

                    let payload = [mdb, pec];
                    let mut data_to_read = [0u8; 16];
                    for (i, b) in data_to_read.iter_mut().enumerate() { *b = i as u8; }

                    let mut ibi = I3cIbi { ibi_type: I3cIbiType::TargetIntr, payload: Some(&payload) };
                    let rc = ctrl.hw.target_pending_read_notify(&mut ctrl.config, &data_to_read, &mut ibi);
                    writeln!(uart, "  pending_read_notify rc {}\r", rc).unwrap();
                }
            }
        }
    }
}
