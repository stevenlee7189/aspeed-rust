// Licensed under the Apache-2.0 license

use core::ptr::{read_volatile, write_volatile};
use crate::uart::{self, Config, UartController};
use crate::common::{DummyDelay, NoOpLogger, UartLogger, Logger};
use crate::pinctrl;
use ast1060_pac::Peripherals;
use embedded_io::Write;
use crate::i3c::ast1060_i3c::Ast1060I3c;
use crate::i3c::ast1060_i3c::I3cController;
use crate::i3c::ast1060_i3c::I3cConfig;
use crate::i3c::ast1060_i3c::I3cDesc;
use crate::i3c::ast1060_i3c::I3cDeviceId;
use crate::i3c::ast1060_i3c::HardwareInterface;
use crate::i3c::ast1060_i3c::i3c_ibi_work_take_consumer;
use crate::i3c::ast1060_i3c::IbiWork;

pub fn enable_ibi_for_pid<H: HardwareInterface, L: Logger>(
    ctrl: &mut I3cController<H, L>,
    pid: u64,
) -> Result<(), i32> {
    let dev_id = I3cDeviceId::new(pid);
    let Some(idx) = ctrl.config.devs.find_index_by_pid(dev_id) else {
        return Err(-1);
    };

    if ctrl.config.devs.i3c_devices[idx].dynamic_addr == 0 {
        let _ = ctrl.hw.do_daa(&mut ctrl.config);
    }

    let Some(idx2) = ctrl.config.devs.find_index_by_pid(dev_id) else {
        return Err(-2);
    };
    // if ctrl.config.devs.i3c_devices[idx2].dynamic_addr == 0 {
    //     return Err(-11);
    // }

    let ret = ctrl.hw.ibi_enable(&mut ctrl.config, idx2);
    if ret != 0 {
        return Err(ret);
    }
    Ok(())
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
        c.is_secondary = false;
        c.i2c_scl_hz = 1000_000;
        c.i3c_scl_hz = 12_500_000;
        c.i3c_pp_scl_hi_period_ns = 250;
        c.i3c_pp_scl_lo_period_ns = 250;
        c.i3c_od_scl_hi_period_ns = 0;
        c.i3c_od_scl_lo_period_ns = 0;
        c.sda_tx_hold_ns = 20;

        const PID: u64 = 0x07ec_0503_1000;
        c.devs.i3c_devices.push(I3cDesc {
            pid: PID,
            static_addr: 0,
            init_dyn_addr: 0,
            dynamic_addr: 0,
            bcr: 4, dcr: 0,
            maxrd: 0, maxwr: 0,
            max_read_turnaround: 0,
            mrl: 0, mwl: 0,
            max_ibi: 1,
            i3c_priv_idx: None,
        }).unwrap();
        // let _ = c.devs.i3c_devices.push(I3cDesc {
        //     pid: 0, static_addr: 0x0, init_dyn_addr: 0, dynamic_addr: 0,
        //     bcr: 0, dcr: 0, maxrd: 0, maxwr: 0, max_read_turnaround: 0,
        //     mrl: 0, mwl: 0, max_ibi: 0, i3c_priv_idx: None,
        // });
    }

    writeln!(uart, "~~~~~~~~~~~~~~~~~~~~~~here\r").unwrap();
    let mut ibi_cons = i3c_ibi_work_take_consumer(ctrl.hw.bus_num() as usize);
    ctrl.init();
    let ret = enable_ibi_for_pid(&mut ctrl, 0x07ec_0503_1000);
    match ret {
        Ok(()) => writeln!(uart, "Enable IBI for PID 0x07ec_0503_1000: OK\r").unwrap(),
        Err(code) => writeln!(uart, "Enable IBI for PID 0x07ec_0503_1000: err {}\r", code).unwrap(),
    }
    unsafe {
        for reg in 0..80 {
            let reg_base = 0x7e7a4000;
            let reg_val: u32;
            reg_val = read_volatile((reg_base + reg * 4) as *const u32);
            // reg as hex
            writeln!(uart, "\r\ni3c reg {:#04x}: {:#010x}", reg * 4, reg_val);
        }
    }
    loop {
        if let Some(work) = ibi_cons.dequeue() {
            match work {
                IbiWork::HotJoin => {
                    writeln!(uart, "[IBI] hotjoin\r").unwrap();
                    let _ = ctrl.hw.do_daa(&mut ctrl.config);
                }
                IbiWork::Sirq { addr, len, data: _ } => {
                    writeln!(uart, "[IBI] SIRQ from 0x{:02x}, len {}\r", addr, len).unwrap();
                }
            }
        }
    }
}
