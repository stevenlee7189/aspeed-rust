// Licensed under the Apache-2.0 license

use paste::paste;
pub struct Pinctrl;

pub struct PinctrlPin {
    pub offset: u32,
    pub bit: u32,
    pub clear: bool,
}

macro_rules! gen_pin_pairs {
    ($reg_name:ident, $offset:expr, $bit:expr) => {
        paste! {
            pub const [<PIN_ $reg_name _ $bit>]: PinctrlPin = PinctrlPin {
                offset: $offset,
                bit: $bit,
                clear: false,
            };

            pub const [<CLR_PIN_ $reg_name _ $bit>]: PinctrlPin = PinctrlPin {
                offset: $offset,
                bit: $bit,
                clear: true,
            };
        }
    };
}

paste! {
    gen_pin_pairs!(SCU410, 0x410, 0);
    gen_pin_pairs!(SCU410, 0x410, 1);
    gen_pin_pairs!(SCU410, 0x410, 2);
    gen_pin_pairs!(SCU410, 0x410, 3);
    gen_pin_pairs!(SCU410, 0x410, 4);
    gen_pin_pairs!(SCU410, 0x410, 5);
    gen_pin_pairs!(SCU410, 0x410, 6);
    gen_pin_pairs!(SCU410, 0x410, 7);
    gen_pin_pairs!(SCU410, 0x410, 8);
    gen_pin_pairs!(SCU410, 0x410, 9);
    gen_pin_pairs!(SCU410, 0x410, 10);
    gen_pin_pairs!(SCU410, 0x410, 11);
    gen_pin_pairs!(SCU410, 0x410, 12);
    gen_pin_pairs!(SCU410, 0x410, 13);
    gen_pin_pairs!(SCU410, 0x410, 14);
    gen_pin_pairs!(SCU410, 0x410, 15);
    gen_pin_pairs!(SCU410, 0x410, 16);
    gen_pin_pairs!(SCU410, 0x410, 17);
    gen_pin_pairs!(SCU410, 0x410, 18);
    gen_pin_pairs!(SCU410, 0x410, 19);
    gen_pin_pairs!(SCU410, 0x410, 20);
    gen_pin_pairs!(SCU410, 0x410, 21);
    gen_pin_pairs!(SCU410, 0x410, 22);
    gen_pin_pairs!(SCU410, 0x410, 23);
    gen_pin_pairs!(SCU410, 0x410, 24);
    gen_pin_pairs!(SCU410, 0x410, 25);
    gen_pin_pairs!(SCU410, 0x410, 26);
    gen_pin_pairs!(SCU410, 0x410, 27);
    gen_pin_pairs!(SCU410, 0x410, 28);
    gen_pin_pairs!(SCU410, 0x410, 29);
    gen_pin_pairs!(SCU410, 0x410, 30);
    gen_pin_pairs!(SCU410, 0x410, 31);

    gen_pin_pairs!(SCU414, 0x414, 0);
    gen_pin_pairs!(SCU414, 0x414, 1);
    gen_pin_pairs!(SCU414, 0x414, 2);
    gen_pin_pairs!(SCU414, 0x414, 3);
    gen_pin_pairs!(SCU414, 0x414, 4);
    gen_pin_pairs!(SCU414, 0x414, 5);
    gen_pin_pairs!(SCU414, 0x414, 6);
    gen_pin_pairs!(SCU414, 0x414, 7);
    gen_pin_pairs!(SCU414, 0x414, 8);
    gen_pin_pairs!(SCU414, 0x414, 9);
    gen_pin_pairs!(SCU414, 0x414, 10);
    gen_pin_pairs!(SCU414, 0x414, 11);
    gen_pin_pairs!(SCU414, 0x414, 12);
    gen_pin_pairs!(SCU414, 0x414, 13);
    gen_pin_pairs!(SCU414, 0x414, 14);
    gen_pin_pairs!(SCU414, 0x414, 15);
    gen_pin_pairs!(SCU414, 0x414, 16);
    gen_pin_pairs!(SCU414, 0x414, 17);
    gen_pin_pairs!(SCU414, 0x414, 18);
    gen_pin_pairs!(SCU414, 0x414, 19);
    gen_pin_pairs!(SCU414, 0x414, 20);
    gen_pin_pairs!(SCU414, 0x414, 21);
    gen_pin_pairs!(SCU414, 0x414, 22);
    gen_pin_pairs!(SCU414, 0x414, 23);
    gen_pin_pairs!(SCU414, 0x414, 24);
    gen_pin_pairs!(SCU414, 0x414, 25);
    gen_pin_pairs!(SCU414, 0x414, 26);
    gen_pin_pairs!(SCU414, 0x414, 27);
    gen_pin_pairs!(SCU414, 0x414, 28);
    gen_pin_pairs!(SCU414, 0x414, 29);
    gen_pin_pairs!(SCU414, 0x414, 30);
    gen_pin_pairs!(SCU414, 0x414, 31);

    gen_pin_pairs!(SCU418, 0x418, 0);
    gen_pin_pairs!(SCU418, 0x418, 1);
    gen_pin_pairs!(SCU418, 0x418, 2);
    gen_pin_pairs!(SCU418, 0x418, 3);
    gen_pin_pairs!(SCU418, 0x418, 4);
    gen_pin_pairs!(SCU418, 0x418, 5);
    gen_pin_pairs!(SCU418, 0x418, 6);
    gen_pin_pairs!(SCU418, 0x418, 7);
    gen_pin_pairs!(SCU418, 0x418, 8);
    gen_pin_pairs!(SCU418, 0x418, 9);
    gen_pin_pairs!(SCU418, 0x418, 10);
    gen_pin_pairs!(SCU418, 0x418, 11);
    gen_pin_pairs!(SCU418, 0x418, 12);
    gen_pin_pairs!(SCU418, 0x418, 13);
    gen_pin_pairs!(SCU418, 0x418, 14);
    gen_pin_pairs!(SCU418, 0x418, 15);
    gen_pin_pairs!(SCU418, 0x418, 16);
    gen_pin_pairs!(SCU418, 0x418, 17);
    gen_pin_pairs!(SCU418, 0x418, 18);
    gen_pin_pairs!(SCU418, 0x418, 19);
    gen_pin_pairs!(SCU418, 0x418, 20);
    gen_pin_pairs!(SCU418, 0x418, 21);
    gen_pin_pairs!(SCU418, 0x418, 22);
    gen_pin_pairs!(SCU418, 0x418, 23);
    gen_pin_pairs!(SCU418, 0x418, 24);
    gen_pin_pairs!(SCU418, 0x418, 25);
    gen_pin_pairs!(SCU418, 0x418, 26);
    gen_pin_pairs!(SCU418, 0x418, 27);
    gen_pin_pairs!(SCU418, 0x418, 28);
    gen_pin_pairs!(SCU418, 0x418, 29);
    gen_pin_pairs!(SCU418, 0x418, 30);
    gen_pin_pairs!(SCU418, 0x418, 31);

    gen_pin_pairs!(SCU41C, 0x41C, 0);
    gen_pin_pairs!(SCU41C, 0x41C, 1);
    gen_pin_pairs!(SCU41C, 0x41C, 2);
    gen_pin_pairs!(SCU41C, 0x41C, 3);
    gen_pin_pairs!(SCU41C, 0x41C, 4);
    gen_pin_pairs!(SCU41C, 0x41C, 5);
    gen_pin_pairs!(SCU41C, 0x41C, 6);
    gen_pin_pairs!(SCU41C, 0x41C, 7);
    gen_pin_pairs!(SCU41C, 0x41C, 8);
    gen_pin_pairs!(SCU41C, 0x41C, 9);
    gen_pin_pairs!(SCU41C, 0x41C, 10);
    gen_pin_pairs!(SCU41C, 0x41C, 11);
    gen_pin_pairs!(SCU41C, 0x41C, 12);
    gen_pin_pairs!(SCU41C, 0x41C, 13);
    gen_pin_pairs!(SCU41C, 0x41C, 14);
    gen_pin_pairs!(SCU41C, 0x41C, 15);
    gen_pin_pairs!(SCU41C, 0x41C, 16);
    gen_pin_pairs!(SCU41C, 0x41C, 17);
    gen_pin_pairs!(SCU41C, 0x41C, 18);
    gen_pin_pairs!(SCU41C, 0x41C, 19);
    gen_pin_pairs!(SCU41C, 0x41C, 20);
    gen_pin_pairs!(SCU41C, 0x41C, 21);
    gen_pin_pairs!(SCU41C, 0x41C, 22);
    gen_pin_pairs!(SCU41C, 0x41C, 23);
    gen_pin_pairs!(SCU41C, 0x41C, 24);
    gen_pin_pairs!(SCU41C, 0x41C, 25);
    gen_pin_pairs!(SCU41C, 0x41C, 26);
    gen_pin_pairs!(SCU41C, 0x41C, 27);
    gen_pin_pairs!(SCU41C, 0x41C, 28);
    gen_pin_pairs!(SCU41C, 0x41C, 29);
    gen_pin_pairs!(SCU41C, 0x41C, 30);
    gen_pin_pairs!(SCU41C, 0x41C, 31);

    gen_pin_pairs!(SCU430, 0x430, 0);
    gen_pin_pairs!(SCU430, 0x430, 1);
    gen_pin_pairs!(SCU430, 0x430, 2);
    gen_pin_pairs!(SCU430, 0x430, 3);
    gen_pin_pairs!(SCU430, 0x430, 4);
    gen_pin_pairs!(SCU430, 0x430, 5);
    gen_pin_pairs!(SCU430, 0x430, 6);
    gen_pin_pairs!(SCU430, 0x430, 7);
    gen_pin_pairs!(SCU430, 0x430, 8);
    gen_pin_pairs!(SCU430, 0x430, 9);
    gen_pin_pairs!(SCU430, 0x430, 10);
    gen_pin_pairs!(SCU430, 0x430, 11);
    gen_pin_pairs!(SCU430, 0x430, 12);
    gen_pin_pairs!(SCU430, 0x430, 13);
    gen_pin_pairs!(SCU430, 0x430, 14);
    gen_pin_pairs!(SCU430, 0x430, 15);
    gen_pin_pairs!(SCU430, 0x430, 16);
    gen_pin_pairs!(SCU430, 0x430, 17);
    gen_pin_pairs!(SCU430, 0x430, 18);
    gen_pin_pairs!(SCU430, 0x430, 19);
    gen_pin_pairs!(SCU430, 0x430, 20);
    gen_pin_pairs!(SCU430, 0x430, 21);
    gen_pin_pairs!(SCU430, 0x430, 22);
    gen_pin_pairs!(SCU430, 0x430, 23);
    gen_pin_pairs!(SCU430, 0x430, 24);
    gen_pin_pairs!(SCU430, 0x430, 25);
    gen_pin_pairs!(SCU430, 0x430, 26);
    gen_pin_pairs!(SCU430, 0x430, 27);
    gen_pin_pairs!(SCU430, 0x430, 28);
    gen_pin_pairs!(SCU430, 0x430, 29);
    gen_pin_pairs!(SCU430, 0x430, 30);
    gen_pin_pairs!(SCU430, 0x430, 31);

    gen_pin_pairs!(SCU434, 0x434, 0);
    gen_pin_pairs!(SCU434, 0x434, 1);
    gen_pin_pairs!(SCU434, 0x434, 2);
    gen_pin_pairs!(SCU434, 0x434, 3);
    gen_pin_pairs!(SCU434, 0x434, 4);
    gen_pin_pairs!(SCU434, 0x434, 5);
    gen_pin_pairs!(SCU434, 0x434, 6);
    gen_pin_pairs!(SCU434, 0x434, 7);
    gen_pin_pairs!(SCU434, 0x434, 8);
    gen_pin_pairs!(SCU434, 0x434, 9);
    gen_pin_pairs!(SCU434, 0x434, 10);
    gen_pin_pairs!(SCU434, 0x434, 11);
    gen_pin_pairs!(SCU434, 0x434, 12);
    gen_pin_pairs!(SCU434, 0x434, 13);
    gen_pin_pairs!(SCU434, 0x434, 14);
    gen_pin_pairs!(SCU434, 0x434, 15);
    gen_pin_pairs!(SCU434, 0x434, 16);
    gen_pin_pairs!(SCU434, 0x434, 17);
    gen_pin_pairs!(SCU434, 0x434, 18);
    gen_pin_pairs!(SCU434, 0x434, 19);
    gen_pin_pairs!(SCU434, 0x434, 20);
    gen_pin_pairs!(SCU434, 0x434, 21);
    gen_pin_pairs!(SCU434, 0x434, 22);
    gen_pin_pairs!(SCU434, 0x434, 23);
    gen_pin_pairs!(SCU434, 0x434, 24);
    gen_pin_pairs!(SCU434, 0x434, 25);
    gen_pin_pairs!(SCU434, 0x434, 26);
    gen_pin_pairs!(SCU434, 0x434, 27);
    gen_pin_pairs!(SCU434, 0x434, 28);
    gen_pin_pairs!(SCU434, 0x434, 29);
    gen_pin_pairs!(SCU434, 0x434, 30);
    gen_pin_pairs!(SCU434, 0x434, 31);

    gen_pin_pairs!(SCU4B0, 0x4B0, 0);
    gen_pin_pairs!(SCU4B0, 0x4B0, 1);
    gen_pin_pairs!(SCU4B0, 0x4B0, 2);
    gen_pin_pairs!(SCU4B0, 0x4B0, 3);
    gen_pin_pairs!(SCU4B0, 0x4B0, 4);
    gen_pin_pairs!(SCU4B0, 0x4B0, 5);
    gen_pin_pairs!(SCU4B0, 0x4B0, 6);
    gen_pin_pairs!(SCU4B0, 0x4B0, 7);
    gen_pin_pairs!(SCU4B0, 0x4B0, 8);
    gen_pin_pairs!(SCU4B0, 0x4B0, 9);
    gen_pin_pairs!(SCU4B0, 0x4B0, 10);
    gen_pin_pairs!(SCU4B0, 0x4B0, 11);
    gen_pin_pairs!(SCU4B0, 0x4B0, 12);
    gen_pin_pairs!(SCU4B0, 0x4B0, 13);
    gen_pin_pairs!(SCU4B0, 0x4B0, 14);
    gen_pin_pairs!(SCU4B0, 0x4B0, 15);
    gen_pin_pairs!(SCU4B0, 0x4B0, 16);
    gen_pin_pairs!(SCU4B0, 0x4B0, 17);
    gen_pin_pairs!(SCU4B0, 0x4B0, 18);
    gen_pin_pairs!(SCU4B0, 0x4B0, 19);
    gen_pin_pairs!(SCU4B0, 0x4B0, 20);
    gen_pin_pairs!(SCU4B0, 0x4B0, 21);
    gen_pin_pairs!(SCU4B0, 0x4B0, 22);
    gen_pin_pairs!(SCU4B0, 0x4B0, 23);
    gen_pin_pairs!(SCU4B0, 0x4B0, 24);
    gen_pin_pairs!(SCU4B0, 0x4B0, 25);
    gen_pin_pairs!(SCU4B0, 0x4B0, 26);
    gen_pin_pairs!(SCU4B0, 0x4B0, 27);
    gen_pin_pairs!(SCU4B0, 0x4B0, 28);
    gen_pin_pairs!(SCU4B0, 0x4B0, 29);
    gen_pin_pairs!(SCU4B0, 0x4B0, 30);
    gen_pin_pairs!(SCU4B0, 0x4B0, 31);

    gen_pin_pairs!(SCU4B4, 0x4B4, 0);
    gen_pin_pairs!(SCU4B4, 0x4B4, 1);
    gen_pin_pairs!(SCU4B4, 0x4B4, 2);
    gen_pin_pairs!(SCU4B4, 0x4B4, 3);
    gen_pin_pairs!(SCU4B4, 0x4B4, 4);
    gen_pin_pairs!(SCU4B4, 0x4B4, 5);
    gen_pin_pairs!(SCU4B4, 0x4B4, 6);
    gen_pin_pairs!(SCU4B4, 0x4B4, 7);
    gen_pin_pairs!(SCU4B4, 0x4B4, 8);
    gen_pin_pairs!(SCU4B4, 0x4B4, 9);
    gen_pin_pairs!(SCU4B4, 0x4B4, 10);
    gen_pin_pairs!(SCU4B4, 0x4B4, 11);
    gen_pin_pairs!(SCU4B4, 0x4B4, 12);
    gen_pin_pairs!(SCU4B4, 0x4B4, 13);
    gen_pin_pairs!(SCU4B4, 0x4B4, 14);
    gen_pin_pairs!(SCU4B4, 0x4B4, 15);
    gen_pin_pairs!(SCU4B4, 0x4B4, 16);
    gen_pin_pairs!(SCU4B4, 0x4B4, 17);
    gen_pin_pairs!(SCU4B4, 0x4B4, 18);
    gen_pin_pairs!(SCU4B4, 0x4B4, 19);
    gen_pin_pairs!(SCU4B4, 0x4B4, 20);
    gen_pin_pairs!(SCU4B4, 0x4B4, 21);
    gen_pin_pairs!(SCU4B4, 0x4B4, 22);
    gen_pin_pairs!(SCU4B4, 0x4B4, 23);
    gen_pin_pairs!(SCU4B4, 0x4B4, 24);
    gen_pin_pairs!(SCU4B4, 0x4B4, 25);
    gen_pin_pairs!(SCU4B4, 0x4B4, 26);
    gen_pin_pairs!(SCU4B4, 0x4B4, 27);
    gen_pin_pairs!(SCU4B4, 0x4B4, 28);
    gen_pin_pairs!(SCU4B4, 0x4B4, 29);
    gen_pin_pairs!(SCU4B4, 0x4B4, 30);
    gen_pin_pairs!(SCU4B4, 0x4B4, 31);

    gen_pin_pairs!(SCU4B8, 0x4B8, 0);
    gen_pin_pairs!(SCU4B8, 0x4B8, 1);
    gen_pin_pairs!(SCU4B8, 0x4B8, 2);
    gen_pin_pairs!(SCU4B8, 0x4B8, 3);
    gen_pin_pairs!(SCU4B8, 0x4B8, 4);
    gen_pin_pairs!(SCU4B8, 0x4B8, 5);
    gen_pin_pairs!(SCU4B8, 0x4B8, 6);
    gen_pin_pairs!(SCU4B8, 0x4B8, 7);
    gen_pin_pairs!(SCU4B8, 0x4B8, 8);
    gen_pin_pairs!(SCU4B8, 0x4B8, 9);
    gen_pin_pairs!(SCU4B8, 0x4B8, 10);
    gen_pin_pairs!(SCU4B8, 0x4B8, 11);
    gen_pin_pairs!(SCU4B8, 0x4B8, 12);
    gen_pin_pairs!(SCU4B8, 0x4B8, 13);
    gen_pin_pairs!(SCU4B8, 0x4B8, 14);
    gen_pin_pairs!(SCU4B8, 0x4B8, 15);
    gen_pin_pairs!(SCU4B8, 0x4B8, 16);
    gen_pin_pairs!(SCU4B8, 0x4B8, 17);
    gen_pin_pairs!(SCU4B8, 0x4B8, 18);
    gen_pin_pairs!(SCU4B8, 0x4B8, 19);
    gen_pin_pairs!(SCU4B8, 0x4B8, 20);
    gen_pin_pairs!(SCU4B8, 0x4B8, 21);
    gen_pin_pairs!(SCU4B8, 0x4B8, 22);
    gen_pin_pairs!(SCU4B8, 0x4B8, 23);
    gen_pin_pairs!(SCU4B8, 0x4B8, 24);
    gen_pin_pairs!(SCU4B8, 0x4B8, 25);
    gen_pin_pairs!(SCU4B8, 0x4B8, 26);
    gen_pin_pairs!(SCU4B8, 0x4B8, 27);
    gen_pin_pairs!(SCU4B8, 0x4B8, 28);
    gen_pin_pairs!(SCU4B8, 0x4B8, 29);
    gen_pin_pairs!(SCU4B8, 0x4B8, 30);
    gen_pin_pairs!(SCU4B8, 0x4B8, 31);

    gen_pin_pairs!(SCU4BC, 0x4BC, 0);
    gen_pin_pairs!(SCU4BC, 0x4BC, 1);
    gen_pin_pairs!(SCU4BC, 0x4BC, 2);
    gen_pin_pairs!(SCU4BC, 0x4BC, 3);
    gen_pin_pairs!(SCU4BC, 0x4BC, 4);
    gen_pin_pairs!(SCU4BC, 0x4BC, 5);
    gen_pin_pairs!(SCU4BC, 0x4BC, 6);
    gen_pin_pairs!(SCU4BC, 0x4BC, 7);
    gen_pin_pairs!(SCU4BC, 0x4BC, 8);
    gen_pin_pairs!(SCU4BC, 0x4BC, 9);
    gen_pin_pairs!(SCU4BC, 0x4BC, 10);
    gen_pin_pairs!(SCU4BC, 0x4BC, 11);
    gen_pin_pairs!(SCU4BC, 0x4BC, 12);
    gen_pin_pairs!(SCU4BC, 0x4BC, 13);
    gen_pin_pairs!(SCU4BC, 0x4BC, 14);
    gen_pin_pairs!(SCU4BC, 0x4BC, 15);
    gen_pin_pairs!(SCU4BC, 0x4BC, 16);
    gen_pin_pairs!(SCU4BC, 0x4BC, 17);
    gen_pin_pairs!(SCU4BC, 0x4BC, 18);
    gen_pin_pairs!(SCU4BC, 0x4BC, 19);
    gen_pin_pairs!(SCU4BC, 0x4BC, 20);
    gen_pin_pairs!(SCU4BC, 0x4BC, 21);
    gen_pin_pairs!(SCU4BC, 0x4BC, 22);
    gen_pin_pairs!(SCU4BC, 0x4BC, 23);
    gen_pin_pairs!(SCU4BC, 0x4BC, 24);
    gen_pin_pairs!(SCU4BC, 0x4BC, 25);
    gen_pin_pairs!(SCU4BC, 0x4BC, 26);
    gen_pin_pairs!(SCU4BC, 0x4BC, 27);
    gen_pin_pairs!(SCU4BC, 0x4BC, 28);
    gen_pin_pairs!(SCU4BC, 0x4BC, 29);
    gen_pin_pairs!(SCU4BC, 0x4BC, 30);
    gen_pin_pairs!(SCU4BC, 0x4BC, 31);

    gen_pin_pairs!(SCU690, 0x690, 0);
    gen_pin_pairs!(SCU690, 0x690, 1);
    gen_pin_pairs!(SCU690, 0x690, 2);
    gen_pin_pairs!(SCU690, 0x690, 3);
    gen_pin_pairs!(SCU690, 0x690, 4);
    gen_pin_pairs!(SCU690, 0x690, 5);
    gen_pin_pairs!(SCU690, 0x690, 6);
    gen_pin_pairs!(SCU690, 0x690, 7);
    gen_pin_pairs!(SCU690, 0x690, 8);
    gen_pin_pairs!(SCU690, 0x690, 9);
    gen_pin_pairs!(SCU690, 0x690, 10);
    gen_pin_pairs!(SCU690, 0x690, 11);
    gen_pin_pairs!(SCU690, 0x690, 12);
    gen_pin_pairs!(SCU690, 0x690, 13);
    gen_pin_pairs!(SCU690, 0x690, 14);
    gen_pin_pairs!(SCU690, 0x690, 15);
    gen_pin_pairs!(SCU690, 0x690, 16);
    gen_pin_pairs!(SCU690, 0x690, 17);
    gen_pin_pairs!(SCU690, 0x690, 18);
    gen_pin_pairs!(SCU690, 0x690, 19);
    gen_pin_pairs!(SCU690, 0x690, 20);
    gen_pin_pairs!(SCU690, 0x690, 21);
    gen_pin_pairs!(SCU690, 0x690, 22);
    gen_pin_pairs!(SCU690, 0x690, 23);
    gen_pin_pairs!(SCU690, 0x690, 24);
    gen_pin_pairs!(SCU690, 0x690, 25);
    gen_pin_pairs!(SCU690, 0x690, 26);
    gen_pin_pairs!(SCU690, 0x690, 27);
    gen_pin_pairs!(SCU690, 0x690, 28);
    gen_pin_pairs!(SCU690, 0x690, 29);
    gen_pin_pairs!(SCU690, 0x690, 30);
    gen_pin_pairs!(SCU690, 0x690, 31);

    gen_pin_pairs!(SCU694, 0x694, 0);
    gen_pin_pairs!(SCU694, 0x694, 1);
    gen_pin_pairs!(SCU694, 0x694, 2);
    gen_pin_pairs!(SCU694, 0x694, 3);
    gen_pin_pairs!(SCU694, 0x694, 4);
    gen_pin_pairs!(SCU694, 0x694, 5);
    gen_pin_pairs!(SCU694, 0x694, 6);
    gen_pin_pairs!(SCU694, 0x694, 7);
    gen_pin_pairs!(SCU694, 0x694, 8);
    gen_pin_pairs!(SCU694, 0x694, 9);
    gen_pin_pairs!(SCU694, 0x694, 10);
    gen_pin_pairs!(SCU694, 0x694, 11);
    gen_pin_pairs!(SCU694, 0x694, 12);
    gen_pin_pairs!(SCU694, 0x694, 13);
    gen_pin_pairs!(SCU694, 0x694, 14);
    gen_pin_pairs!(SCU694, 0x694, 15);
    gen_pin_pairs!(SCU694, 0x694, 16);
    gen_pin_pairs!(SCU694, 0x694, 17);
    gen_pin_pairs!(SCU694, 0x694, 18);
    gen_pin_pairs!(SCU694, 0x694, 19);
    gen_pin_pairs!(SCU694, 0x694, 20);
    gen_pin_pairs!(SCU694, 0x694, 21);
    gen_pin_pairs!(SCU694, 0x694, 22);
    gen_pin_pairs!(SCU694, 0x694, 23);
    gen_pin_pairs!(SCU694, 0x694, 24);
    gen_pin_pairs!(SCU694, 0x694, 25);
    gen_pin_pairs!(SCU694, 0x694, 26);
    gen_pin_pairs!(SCU694, 0x694, 27);
    gen_pin_pairs!(SCU694, 0x694, 28);
    gen_pin_pairs!(SCU694, 0x694, 29);
    gen_pin_pairs!(SCU694, 0x694, 30);
    gen_pin_pairs!(SCU694, 0x694, 31);

    gen_pin_pairs!(SCU69C, 0x69C, 0);
    gen_pin_pairs!(SCU69C, 0x69C, 1);
    gen_pin_pairs!(SCU69C, 0x69C, 2);
    gen_pin_pairs!(SCU69C, 0x69C, 3);
    gen_pin_pairs!(SCU69C, 0x69C, 4);
    gen_pin_pairs!(SCU69C, 0x69C, 5);
    gen_pin_pairs!(SCU69C, 0x69C, 6);
    gen_pin_pairs!(SCU69C, 0x69C, 7);
    gen_pin_pairs!(SCU69C, 0x69C, 8);
    gen_pin_pairs!(SCU69C, 0x69C, 9);
    gen_pin_pairs!(SCU69C, 0x69C, 10);
    gen_pin_pairs!(SCU69C, 0x69C, 11);
    gen_pin_pairs!(SCU69C, 0x69C, 12);
    gen_pin_pairs!(SCU69C, 0x69C, 13);
    gen_pin_pairs!(SCU69C, 0x69C, 14);
    gen_pin_pairs!(SCU69C, 0x69C, 15);
    gen_pin_pairs!(SCU69C, 0x69C, 16);
    gen_pin_pairs!(SCU69C, 0x69C, 17);
    gen_pin_pairs!(SCU69C, 0x69C, 18);
    gen_pin_pairs!(SCU69C, 0x69C, 19);
    gen_pin_pairs!(SCU69C, 0x69C, 20);
    gen_pin_pairs!(SCU69C, 0x69C, 21);
    gen_pin_pairs!(SCU69C, 0x69C, 22);
    gen_pin_pairs!(SCU69C, 0x69C, 23);
    gen_pin_pairs!(SCU69C, 0x69C, 24);
    gen_pin_pairs!(SCU69C, 0x69C, 25);
    gen_pin_pairs!(SCU69C, 0x69C, 26);
    gen_pin_pairs!(SCU69C, 0x69C, 27);
    gen_pin_pairs!(SCU69C, 0x69C, 28);
    gen_pin_pairs!(SCU69C, 0x69C, 29);
    gen_pin_pairs!(SCU69C, 0x69C, 30);
    gen_pin_pairs!(SCU69C, 0x69C, 31);

    gen_pin_pairs!(SCU6B0, 0x6B0, 0);
    gen_pin_pairs!(SCU6B0, 0x6B0, 1);
    gen_pin_pairs!(SCU6B0, 0x6B0, 2);
    gen_pin_pairs!(SCU6B0, 0x6B0, 3);
    gen_pin_pairs!(SCU6B0, 0x6B0, 4);
    gen_pin_pairs!(SCU6B0, 0x6B0, 5);
    gen_pin_pairs!(SCU6B0, 0x6B0, 6);
    gen_pin_pairs!(SCU6B0, 0x6B0, 7);
    gen_pin_pairs!(SCU6B0, 0x6B0, 8);
    gen_pin_pairs!(SCU6B0, 0x6B0, 9);
    gen_pin_pairs!(SCU6B0, 0x6B0, 10);
    gen_pin_pairs!(SCU6B0, 0x6B0, 11);
    gen_pin_pairs!(SCU6B0, 0x6B0, 12);
    gen_pin_pairs!(SCU6B0, 0x6B0, 13);
    gen_pin_pairs!(SCU6B0, 0x6B0, 14);
    gen_pin_pairs!(SCU6B0, 0x6B0, 15);
    gen_pin_pairs!(SCU6B0, 0x6B0, 16);
    gen_pin_pairs!(SCU6B0, 0x6B0, 17);
    gen_pin_pairs!(SCU6B0, 0x6B0, 18);
    gen_pin_pairs!(SCU6B0, 0x6B0, 19);
    gen_pin_pairs!(SCU6B0, 0x6B0, 20);
    gen_pin_pairs!(SCU6B0, 0x6B0, 21);
    gen_pin_pairs!(SCU6B0, 0x6B0, 22);
    gen_pin_pairs!(SCU6B0, 0x6B0, 23);
    gen_pin_pairs!(SCU6B0, 0x6B0, 24);
    gen_pin_pairs!(SCU6B0, 0x6B0, 25);
    gen_pin_pairs!(SCU6B0, 0x6B0, 26);
    gen_pin_pairs!(SCU6B0, 0x6B0, 27);
    gen_pin_pairs!(SCU6B0, 0x6B0, 28);
    gen_pin_pairs!(SCU6B0, 0x6B0, 29);
    gen_pin_pairs!(SCU6B0, 0x6B0, 30);
    gen_pin_pairs!(SCU6B0, 0x6B0, 31);
}

// Pin to be cleared or set
pub const PIN_SPI2CS0: PinctrlPin = PinctrlPin {
    offset: 0x41C,
    bit: 30,
    clear: false,
};
pub const PIN_SPI2CS1: PinctrlPin = PinctrlPin {
    offset: 0x41C,
    bit: 31,
    clear: false,
};

pub const PIN_SPI2CK: PinctrlPin = PinctrlPin {
    offset: 0x430,
    bit: 0,
    clear: false,
};
pub const PIN_SPI2DQ0: PinctrlPin = PinctrlPin {
    offset: 0x430,
    bit: 1,
    clear: false,
};
pub const PIN_SPI2DQ1: PinctrlPin = PinctrlPin {
    offset: 0x430,
    bit: 2,
    clear: false,
};
pub const PIN_SPI2DQ2: PinctrlPin = PinctrlPin {
    offset: 0x430,
    bit: 3,
    clear: false,
};
pub const PIN_SPI2DQ3: PinctrlPin = PinctrlPin {
    offset: 0x430,
    bit: 4,
    clear: false,
};
pub const PIN_FWSPIDQ2: PinctrlPin = PinctrlPin {
    offset: 0x430,
    bit: 10,
    clear: false,
};
pub const PIN_FWSPIDQ3: PinctrlPin = PinctrlPin {
    offset: 0x430,
    bit: 11,
    clear: false,
};
pub const PIN_SPI1DQ2: PinctrlPin = PinctrlPin {
    offset: 0x430,
    bit: 17,
    clear: false,
};
pub const PIN_SPI1DQ3: PinctrlPin = PinctrlPin {
    offset: 0x430,
    bit: 18,
    clear: false,
};

pub const PIN_QSPIM2_RST_IN: PinctrlPin = PinctrlPin {
    offset: 0x69c,
    bit: 9,
    clear: false,
};
pub const PIN_QSPIM4_RST_IN: PinctrlPin = PinctrlPin {
    offset: 0x69c,
    bit: 11,
    clear: false,
};

pub const PIN_SPIM0_CSIN_CTRL1: PinctrlPin = PinctrlPin {
    offset: 0x410,
    bit: 0,
    clear: true,
};
pub const PIN_SPIM0_CSIN_CTRL13: PinctrlPin = PinctrlPin {
    offset: 0x4B0,
    bit: 0,
    clear: true,
};
pub const PIN_SPIM0_CSIN_CTRL31: PinctrlPin = PinctrlPin {
    offset: 0x690,
    bit: 0,
    clear: false,
};
pub const PIN_SPIM0_CLKIN_CTRL1: PinctrlPin = PinctrlPin {
    offset: 0x410,
    bit: 1,
    clear: true,
};
pub const PIN_SPIM0_CLKIN_CTRL13: PinctrlPin = PinctrlPin {
    offset: 0x4B0,
    bit: 1,
    clear: true,
};
pub const PIN_SPIM0_CLKIN_CTRL31: PinctrlPin = PinctrlPin {
    offset: 0x690,
    bit: 1,
    clear: false,
};
pub const PIN_SPIM0_MOSIIN_CTRL1: PinctrlPin = PinctrlPin {
    offset: 0x410,
    bit: 2,
    clear: true,
};
pub const PIN_SPIM0_MOSIIN_CTRL13: PinctrlPin = PinctrlPin {
    offset: 0x4B0,
    bit: 2,
    clear: true,
};
pub const PIN_SPIM0_MOSIIN_CTRL31: PinctrlPin = PinctrlPin {
    offset: 0x690,
    bit: 2,
    clear: false,
};
pub const PIN_SPIM0_MISOIN_CTRL1: PinctrlPin = PinctrlPin {
    offset: 0x410,
    bit: 3,
    clear: true,
};
pub const PIN_SPIM0_MISOIN_CTRL13: PinctrlPin = PinctrlPin {
    offset: 0x4B0,
    bit: 3,
    clear: true,
};
pub const PIN_SPIM0_MISOIN_CTRL31: PinctrlPin = PinctrlPin {
    offset: 0x690,
    bit: 3,
    clear: false,
};
pub const PIN_SPIM0_IO2IN_CTRL1: PinctrlPin = PinctrlPin {
    offset: 0x410,
    bit: 4,
    clear: true,
};
pub const PIN_SPIM0_IO2IN_CTRL13: PinctrlPin = PinctrlPin {
    offset: 0x4B0,
    bit: 4,
    clear: true,
};
pub const PIN_SPIM0_IO2IN_CTRL31: PinctrlPin = PinctrlPin {
    offset: 0x690,
    bit: 4,
    clear: false,
};
pub const PIN_SPIM0_IO3IN_CTRL1: PinctrlPin = PinctrlPin {
    offset: 0x410,
    bit: 5,
    clear: true,
};
pub const PIN_SPIM0_IO3IN_CTRL13: PinctrlPin = PinctrlPin {
    offset: 0x4B0,
    bit: 5,
    clear: true,
};
pub const PIN_SPIM0_IO3IN_CTRL31: PinctrlPin = PinctrlPin {
    offset: 0x690,
    bit: 5,
    clear: false,
};
pub const PIN_SPIM0_CSNOUT_CTRL1: PinctrlPin = PinctrlPin {
    offset: 0x410,
    bit: 6,
    clear: true,
};
pub const PIN_SPIM0_CSNOUT_CTRL13: PinctrlPin = PinctrlPin {
    offset: 0x4B0,
    bit: 6,
    clear: true,
};
pub const PIN_SPIM0_CSNOUT_CTRL31: PinctrlPin = PinctrlPin {
    offset: 0x690,
    bit: 6,
    clear: false,
};
pub const PIN_SPIM0_CLKOUT_CTRL1: PinctrlPin = PinctrlPin {
    offset: 0x410,
    bit: 7,
    clear: true,
};
pub const PIN_SPIM0_CLKOUT_CTRL13: PinctrlPin = PinctrlPin {
    offset: 0x4B0,
    bit: 7,
    clear: true,
};
pub const PIN_SPIM0_CLKOUT_CTRL31: PinctrlPin = PinctrlPin {
    offset: 0x690,
    bit: 7,
    clear: false,
};
pub const PIN_SPIM0_MOSIOUT_CTRL1: PinctrlPin = PinctrlPin {
    offset: 0x410,
    bit: 8,
    clear: true,
};
pub const PIN_SPIM0_MOSIOUT_CTRL13: PinctrlPin = PinctrlPin {
    offset: 0x4B0,
    bit: 8,
    clear: true,
};
pub const PIN_SPIM0_MOSIOUT_CTRL31: PinctrlPin = PinctrlPin {
    offset: 0x690,
    bit: 8,
    clear: false,
};
pub const PIN_SPIM0_MISOOUT_CTRL1: PinctrlPin = PinctrlPin {
    offset: 0x410,
    bit: 9,
    clear: true,
};
pub const PIN_SPIM0_MISOOUT_CTRL13: PinctrlPin = PinctrlPin {
    offset: 0x4B0,
    bit: 9,
    clear: true,
};
pub const PIN_SPIM0_MISOOUT_CTRL31: PinctrlPin = PinctrlPin {
    offset: 0x690,
    bit: 9,
    clear: false,
};
pub const PIN_SPIM0_IO2OUT_CTRL1: PinctrlPin = PinctrlPin {
    offset: 0x410,
    bit: 10,
    clear: true,
};
pub const PIN_SPIM0_IO2OUT_CTRL13: PinctrlPin = PinctrlPin {
    offset: 0x4B0,
    bit: 10,
    clear: true,
};
pub const PIN_SPIM0_IO2OUT_CTRL31: PinctrlPin = PinctrlPin {
    offset: 0x690,
    bit: 10,
    clear: false,
};
pub const PIN_SPIM0_IO3OUT_CTRL1: PinctrlPin = PinctrlPin {
    offset: 0x410,
    bit: 11,
    clear: true,
};
pub const PIN_SPIM0_IO3OUT_CTRL13: PinctrlPin = PinctrlPin {
    offset: 0x4B0,
    bit: 11,
    clear: true,
};
pub const PIN_SPIM0_IO3OUT_CTRL31: PinctrlPin = PinctrlPin {
    offset: 0x690,
    bit: 11,
    clear: false,
};
pub const PIN_SPIM0_MUX_CTRL1: PinctrlPin = PinctrlPin {
    offset: 0x410,
    bit: 12,
    clear: true,
};

pub const PIN_SPIM0_MUX_CTRL13: PinctrlPin = PinctrlPin {
    offset: 0x4B0,
    bit: 12,
    clear: true,
};

pub const PIN_SPIM0_MUX_CTRL31: PinctrlPin = PinctrlPin {
    offset: 0x690,
    bit: 12,
    clear: false,
};
pub const PIN_SPIM0_RSTOUT_CTRL1: PinctrlPin = PinctrlPin {
    offset: 0x410,
    bit: 13,
    clear: true,
};

pub const PIN_SPIM0_RSTOUT_CTRL13: PinctrlPin = PinctrlPin {
    offset: 0x4B0,
    bit: 13,
    clear: true,
};

pub const PIN_SPIM0_RSTOUT_CTRL31: PinctrlPin = PinctrlPin {
    offset: 0x690,
    bit: 13,
    clear: false,
};
pub const PIN_SPIM0_RSTIN_CTRL2: PinctrlPin = PinctrlPin {
    offset: 0x414,
    bit: 24,
    clear: true,
};

pub const PIN_SPIM0_RSTIN_CTRL14: PinctrlPin = PinctrlPin {
    offset: 0x4B4,
    bit: 24,
    clear: true,
};

pub const PIN_SPIM0_RSTIN_CTRL32: PinctrlPin = PinctrlPin {
    offset: 0x690,
    bit: 24,
    clear: false,
};
pub const PIN_SPIM1_CSIN_CTRL1: PinctrlPin = PinctrlPin {
    offset: 0x410,
    bit: 14,
    clear: true,
};
pub const PIN_SPIM1_CSIN_CTRL13: PinctrlPin = PinctrlPin {
    offset: 0x4B0,
    bit: 14,
    clear: true,
};
pub const PIN_SPIM1_CSIN_CTRL31: PinctrlPin = PinctrlPin {
    offset: 0x690,
    bit: 14,
    clear: false,
};
pub const PIN_SPIM1_CLKIN_CTRL1: PinctrlPin = PinctrlPin {
    offset: 0x410,
    bit: 15,
    clear: true,
};
pub const PIN_SPIM1_CLKIN_CTRL13: PinctrlPin = PinctrlPin {
    offset: 0x4B0,
    bit: 15,
    clear: true,
};
pub const PIN_SPIM1_CLKIN_CTRL31: PinctrlPin = PinctrlPin {
    offset: 0x690,
    bit: 15,
    clear: false,
};
pub const PIN_SPIM1_MOSIIN_CTRL1: PinctrlPin = PinctrlPin {
    offset: 0x410,
    bit: 16,
    clear: true,
};
pub const PIN_SPIM1_MOSIIN_CTRL13: PinctrlPin = PinctrlPin {
    offset: 0x4B0,
    bit: 16,
    clear: true,
};
pub const PIN_SPIM1_MOSIIN_CTRL31: PinctrlPin = PinctrlPin {
    offset: 0x690,
    bit: 16,
    clear: false,
};
pub const PIN_SPIM1_MISOIN_CTRL1: PinctrlPin = PinctrlPin {
    offset: 0x410,
    bit: 17,
    clear: true,
};
pub const PIN_SPIM1_MISOIN_CTRL13: PinctrlPin = PinctrlPin {
    offset: 0x4B0,
    bit: 17,
    clear: true,
};
pub const PIN_SPIM1_MISOIN_CTRL31: PinctrlPin = PinctrlPin {
    offset: 0x690,
    bit: 17,
    clear: false,
};
pub const PIN_SPIM1_IO2IN_CTRL1: PinctrlPin = PinctrlPin {
    offset: 0x410,
    bit: 18,
    clear: true,
};
pub const PIN_SPIM1_IO2IN_CTRL13: PinctrlPin = PinctrlPin {
    offset: 0x4B0,
    bit: 18,
    clear: true,
};
pub const PIN_SPIM1_IO2IN_CTRL31: PinctrlPin = PinctrlPin {
    offset: 0x690,
    bit: 18,
    clear: false,
};
pub const PIN_SPIM1_IO3IN_CTRL1: PinctrlPin = PinctrlPin {
    offset: 0x410,
    bit: 19,
    clear: true,
};
pub const PIN_SPIM1_IO3IN_CTRL13: PinctrlPin = PinctrlPin {
    offset: 0x4B0,
    bit: 19,
    clear: true,
};
pub const PIN_SPIM1_IO3IN_CTRL31: PinctrlPin = PinctrlPin {
    offset: 0x690,
    bit: 19,
    clear: false,
};
pub const PIN_SPIM1_CSNOUT_CTRL1: PinctrlPin = PinctrlPin {
    offset: 0x410,
    bit: 20,
    clear: true,
};
pub const PIN_SPIM1_CSNOUT_CTRL13: PinctrlPin = PinctrlPin {
    offset: 0x4B0,
    bit: 20,
    clear: true,
};
pub const PIN_SPIM1_CSNOUT_CTRL31: PinctrlPin = PinctrlPin {
    offset: 0x690,
    bit: 20,
    clear: false,
};
pub const PIN_SPIM1_CLKOUT_CTRL1: PinctrlPin = PinctrlPin {
    offset: 0x410,
    bit: 21,
    clear: true,
};
pub const PIN_SPIM1_CLKOUT_CTRL13: PinctrlPin = PinctrlPin {
    offset: 0x4B0,
    bit: 21,
    clear: true,
};
pub const PIN_SPIM1_CLKOUT_CTRL31: PinctrlPin = PinctrlPin {
    offset: 0x690,
    bit: 21,
    clear: false,
};
pub const PIN_SPIM1_MOSIOUT_CTRL1: PinctrlPin = PinctrlPin {
    offset: 0x410,
    bit: 8,
    clear: true,
};
pub const PIN_SPIM1_MOSIOUT_CTRL13: PinctrlPin = PinctrlPin {
    offset: 0x4B0,
    bit: 8,
    clear: true,
};
pub const PIN_SPIM1_MOSIOUT_CTRL31: PinctrlPin = PinctrlPin {
    offset: 0x690,
    bit: 8,
    clear: false,
};
pub const PIN_SPIM1_MISOOUT_CTRL1: PinctrlPin = PinctrlPin {
    offset: 0x410,
    bit: 23,
    clear: true,
};
pub const PIN_SPIM1_MISOOUT_CTRL13: PinctrlPin = PinctrlPin {
    offset: 0x4B0,
    bit: 23,
    clear: true,
};
pub const PIN_SPIM1_MISOOUT_CTRL31: PinctrlPin = PinctrlPin {
    offset: 0x690,
    bit: 23,
    clear: false,
};
pub const PIN_SPIM1_IO2OUT_CTRL1: PinctrlPin = PinctrlPin {
    offset: 0x410,
    bit: 24,
    clear: true,
};
pub const PIN_SPIM1_IO2OUT_CTRL13: PinctrlPin = PinctrlPin {
    offset: 0x4B0,
    bit: 24,
    clear: true,
};
pub const PIN_SPIM1_IO2OUT_CTRL31: PinctrlPin = PinctrlPin {
    offset: 0x690,
    bit: 24,
    clear: false,
};
pub const PIN_SPIM1_IO3OUT_CTRL1: PinctrlPin = PinctrlPin {
    offset: 0x410,
    bit: 25,
    clear: true,
};
pub const PIN_SPIM1_IO3OUT_CTRL13: PinctrlPin = PinctrlPin {
    offset: 0x4B0,
    bit: 25,
    clear: true,
};
pub const PIN_SPIM1_IO3OUT_CTRL31: PinctrlPin = PinctrlPin {
    offset: 0x690,
    bit: 25,
    clear: false,
};
pub const PIN_SPIM1_MUX_CTRL1: PinctrlPin = PinctrlPin {
    offset: 0x410,
    bit: 26,
    clear: true,
};

pub const PIN_SPIM1_MUX_CTRL13: PinctrlPin = PinctrlPin {
    offset: 0x4B0,
    bit: 26,
    clear: true,
};

pub const PIN_SPIM1_MUX_CTRL31: PinctrlPin = PinctrlPin {
    offset: 0x690,
    bit: 26,
    clear: false,
};
pub const PIN_SPIM1_RSTOUT_CTRL1: PinctrlPin = PinctrlPin {
    offset: 0x410,
    bit: 27,
    clear: true,
};

pub const PIN_SPIM1_RSTOUT_CTRL13: PinctrlPin = PinctrlPin {
    offset: 0x4B0,
    bit: 27,
    clear: true,
};

pub const PIN_SPIM1_RSTOUT_CTRL31: PinctrlPin = PinctrlPin {
    offset: 0x690,
    bit: 27,
    clear: false,
};

pub const PIN_SPIM2_CSIN_CTRL1: PinctrlPin = PinctrlPin {
    offset: 0x410,
    bit: 28,
    clear: true,
};
pub const PIN_SPIM2_CSIN_CTRL13: PinctrlPin = PinctrlPin {
    offset: 0x4B0,
    bit: 28,
    clear: true,
};
pub const PIN_SPIM2_CSIN_CTRL31: PinctrlPin = PinctrlPin {
    offset: 0x690,
    bit: 28,
    clear: false,
};
pub const PIN_SPIM2_CLKIN_CTRL1: PinctrlPin = PinctrlPin {
    offset: 0x410,
    bit: 29,
    clear: true,
};
pub const PIN_SPIM2_CLKIN_CTRL13: PinctrlPin = PinctrlPin {
    offset: 0x4B0,
    bit: 29,
    clear: true,
};
pub const PIN_SPIM2_CLKIN_CTRL31: PinctrlPin = PinctrlPin {
    offset: 0x690,
    bit: 29,
    clear: false,
};
pub const PIN_SPIM2_MOSIIN_CTRL1: PinctrlPin = PinctrlPin {
    offset: 0x410,
    bit: 30,
    clear: true,
};
pub const PIN_SPIM2_MOSIIN_CTRL13: PinctrlPin = PinctrlPin {
    offset: 0x4B0,
    bit: 30,
    clear: true,
};
pub const PIN_SPIM2_MOSIIN_CTRL31: PinctrlPin = PinctrlPin {
    offset: 0x690,
    bit: 30,
    clear: false,
};
pub const PIN_SPIM2_MISOIN_CTRL1: PinctrlPin = PinctrlPin {
    offset: 0x410,
    bit: 31,
    clear: true,
};
pub const PIN_SPIM2_MISOIN_CTRL13: PinctrlPin = PinctrlPin {
    offset: 0x4B0,
    bit: 31,
    clear: true,
};
pub const PIN_SPIM2_MISOIN_CTRL31: PinctrlPin = PinctrlPin {
    offset: 0x690,
    bit: 31,
    clear: false,
};
pub const PIN_SPIM2_IO2IN_CTRL2: PinctrlPin = PinctrlPin {
    offset: 0x414,
    bit: 0,
    clear: true,
};
pub const PIN_SPIM2_IO2IN_CTRL14: PinctrlPin = PinctrlPin {
    offset: 0x4B4,
    bit: 0,
    clear: true,
};
pub const PIN_SPIM2_IO2IN_CTRL32: PinctrlPin = PinctrlPin {
    offset: 0x694,
    bit: 0,
    clear: false,
};
pub const PIN_SPIM2_IO3IN_CTRL2: PinctrlPin = PinctrlPin {
    offset: 0x414,
    bit: 1,
    clear: true,
};
pub const PIN_SPIM2_IO3IN_CTRL14: PinctrlPin = PinctrlPin {
    offset: 0x4B4,
    bit: 1,
    clear: true,
};
pub const PIN_SPIM2_IO3IN_CTRL32: PinctrlPin = PinctrlPin {
    offset: 0x694,
    bit: 1,
    clear: false,
};
pub const PIN_SPIM2_CSNOUT_CTRL2: PinctrlPin = PinctrlPin {
    offset: 0x414,
    bit: 2,
    clear: true,
};
pub const PIN_SPIM2_CSNOUT_CTRL14: PinctrlPin = PinctrlPin {
    offset: 0x4B4,
    bit: 2,
    clear: true,
};
pub const PIN_SPIM2_CSNOUT_CTRL32: PinctrlPin = PinctrlPin {
    offset: 0x694,
    bit: 2,
    clear: false,
};
pub const PIN_SPIM2_CLKOUT_CTRL2: PinctrlPin = PinctrlPin {
    offset: 0x414,
    bit: 3,
    clear: true,
};
pub const PIN_SPIM2_CLKOUT_CTRL14: PinctrlPin = PinctrlPin {
    offset: 0x4B4,
    bit: 3,
    clear: true,
};
pub const PIN_SPIM2_CLKOUT_CTRL32: PinctrlPin = PinctrlPin {
    offset: 0x694,
    bit: 3,
    clear: false,
};
pub const PIN_SPIM2_MOSIOUT_CTRL2: PinctrlPin = PinctrlPin {
    offset: 0x414,
    bit: 4,
    clear: true,
};
pub const PIN_SPIM2_MOSIOUT_CTRL14: PinctrlPin = PinctrlPin {
    offset: 0x4B4,
    bit: 4,
    clear: true,
};
pub const PIN_SPIM2_MOSIOUT_CTRL32: PinctrlPin = PinctrlPin {
    offset: 0x694,
    bit: 4,
    clear: false,
};
pub const PIN_SPIM2_MISOOUT_CTRL2: PinctrlPin = PinctrlPin {
    offset: 0x414,
    bit: 5,
    clear: true,
};
pub const PIN_SPIM2_MISOOUT_CTRL14: PinctrlPin = PinctrlPin {
    offset: 0x4B4,
    bit: 5,
    clear: true,
};
pub const PIN_SPIM2_MISOOUT_CTRL32: PinctrlPin = PinctrlPin {
    offset: 0x694,
    bit: 5,
    clear: false,
};
pub const PIN_SPIM2_IO2OUT_CTRL2: PinctrlPin = PinctrlPin {
    offset: 0x414,
    bit: 6,
    clear: true,
};
pub const PIN_SPIM2_IO2OUT_CTRL14: PinctrlPin = PinctrlPin {
    offset: 0x4B4,
    bit: 6,
    clear: true,
};
pub const PIN_SPIM2_IO2OUT_CTRL32: PinctrlPin = PinctrlPin {
    offset: 0x694,
    bit: 6,
    clear: false,
};
pub const PIN_SPIM2_IO3OUT_CTRL2: PinctrlPin = PinctrlPin {
    offset: 0x414,
    bit: 7,
    clear: true,
};
pub const PIN_SPIM2_IO3OUT_CTRL14: PinctrlPin = PinctrlPin {
    offset: 0x4B4,
    bit: 7,
    clear: true,
};
pub const PIN_SPIM2_IO3OUT_CTRL32: PinctrlPin = PinctrlPin {
    offset: 0x694,
    bit: 7,
    clear: false,
};
pub const PIN_SPIM2_MUX_CTRL2: PinctrlPin = PinctrlPin {
    offset: 0x414,
    bit: 8,
    clear: true,
};

pub const PIN_SPIM2_MUX_CTRL14: PinctrlPin = PinctrlPin {
    offset: 0x4B4,
    bit: 8,
    clear: true,
};

pub const PIN_SPIM2_MUX_CTRL32: PinctrlPin = PinctrlPin {
    offset: 0x694,
    bit: 8,
    clear: false,
};
pub const PIN_SPIM2_RSTOUT_CTRL2: PinctrlPin = PinctrlPin {
    offset: 0x414,
    bit: 9,
    clear: true,
};

pub const PIN_SPIM2_RSTOUT_CTRL14: PinctrlPin = PinctrlPin {
    offset: 0x4B4,
    bit: 9,
    clear: true,
};

pub const PIN_SPIM2_RSTOUT_CTRL32: PinctrlPin = PinctrlPin {
    offset: 0x694,
    bit: 9,
    clear: false,
};

pub const PIN_SPIM2_RSTIN_CTRL2: PinctrlPin = PinctrlPin {
    offset: 0x414,
    bit: 25,
    clear: true,
};

pub const PIN_SPIM2_RSTIN_CTRL14: PinctrlPin = PinctrlPin {
    offset: 0x4B4,
    bit: 25,
    clear: true,
};

pub const PIN_SPIM2_RSTIN_CTRL32: PinctrlPin = PinctrlPin {
    offset: 0x694,
    bit: 25,
    clear: false,
};

pub const PIN_SPIM3_CSIN_CTRL2: PinctrlPin = PinctrlPin {
    offset: 0x414,
    bit: 10,
    clear: true,
};
pub const PIN_SPIM3_CSIN_CTRL14: PinctrlPin = PinctrlPin {
    offset: 0x4B4,
    bit: 10,
    clear: true,
};
pub const PIN_SPIM3_CSIN_CTRL32: PinctrlPin = PinctrlPin {
    offset: 0x694,
    bit: 10,
    clear: false,
};
pub const PIN_SPIM3_CLKIN_CTRL2: PinctrlPin = PinctrlPin {
    offset: 0x414,
    bit: 11,
    clear: true,
};
pub const PIN_SPIM3_CLKIN_CTRL14: PinctrlPin = PinctrlPin {
    offset: 0x4B4,
    bit: 11,
    clear: true,
};
pub const PIN_SPIM3_CLKIN_CTRL32: PinctrlPin = PinctrlPin {
    offset: 0x694,
    bit: 11,
    clear: false,
};
pub const PIN_SPIM3_MOSIIN_CTRL2: PinctrlPin = PinctrlPin {
    offset: 0x414,
    bit: 12,
    clear: true,
};
pub const PIN_SPIM3_MOSIIN_CTRL14: PinctrlPin = PinctrlPin {
    offset: 0x4B4,
    bit: 12,
    clear: true,
};
pub const PIN_SPIM3_MOSIIN_CTRL32: PinctrlPin = PinctrlPin {
    offset: 0x694,
    bit: 12,
    clear: false,
};
pub const PIN_SPIM3_MISOIN_CTRL2: PinctrlPin = PinctrlPin {
    offset: 0x414,
    bit: 13,
    clear: true,
};
pub const PIN_SPIM3_MISOIN_CTRL14: PinctrlPin = PinctrlPin {
    offset: 0x4B4,
    bit: 13,
    clear: true,
};
pub const PIN_SPIM3_MISOIN_CTRL32: PinctrlPin = PinctrlPin {
    offset: 0x694,
    bit: 13,
    clear: false,
};
pub const PIN_SPIM3_IO2IN_CTRL2: PinctrlPin = PinctrlPin {
    offset: 0x414,
    bit: 14,
    clear: true,
};
pub const PIN_SPIM3_IO2IN_CTRL14: PinctrlPin = PinctrlPin {
    offset: 0x4B4,
    bit: 14,
    clear: true,
};
pub const PIN_SPIM3_IO2IN_CTRL32: PinctrlPin = PinctrlPin {
    offset: 0x694,
    bit: 14,
    clear: false,
};
pub const PIN_SPIM3_IO3IN_CTRL2: PinctrlPin = PinctrlPin {
    offset: 0x414,
    bit: 15,
    clear: true,
};
pub const PIN_SPIM3_IO3IN_CTRL14: PinctrlPin = PinctrlPin {
    offset: 0x4B4,
    bit: 15,
    clear: true,
};
pub const PIN_SPIM3_IO3IN_CTRL32: PinctrlPin = PinctrlPin {
    offset: 0x694,
    bit: 15,
    clear: false,
};
pub const PIN_SPIM3_CSNOUT_CTRL2: PinctrlPin = PinctrlPin {
    offset: 0x414,
    bit: 16,
    clear: true,
};
pub const PIN_SPIM3_CSNOUT_CTRL14: PinctrlPin = PinctrlPin {
    offset: 0x4B4,
    bit: 16,
    clear: true,
};
pub const PIN_SPIM3_CSNOUT_CTRL32: PinctrlPin = PinctrlPin {
    offset: 0x694,
    bit: 16,
    clear: false,
};
pub const PIN_SPIM3_CLKOUT_CTRL2: PinctrlPin = PinctrlPin {
    offset: 0x414,
    bit: 17,
    clear: true,
};
pub const PIN_SPIM3_CLKOUT_CTRL14: PinctrlPin = PinctrlPin {
    offset: 0x4B4,
    bit: 17,
    clear: true,
};
pub const PIN_SPIM3_CLKOUT_CTRL32: PinctrlPin = PinctrlPin {
    offset: 0x694,
    bit: 17,
    clear: false,
};
pub const PIN_SPIM3_MOSIOUT_CTRL2: PinctrlPin = PinctrlPin {
    offset: 0x414,
    bit: 18,
    clear: true,
};
pub const PIN_SPIM3_MOSIOUT_CTRL14: PinctrlPin = PinctrlPin {
    offset: 0x4B4,
    bit: 18,
    clear: true,
};
pub const PIN_SPIM3_MOSIOUT_CTRL32: PinctrlPin = PinctrlPin {
    offset: 0x694,
    bit: 18,
    clear: false,
};
pub const PIN_SPIM3_MISOOUT_CTRL2: PinctrlPin = PinctrlPin {
    offset: 0x414,
    bit: 19,
    clear: true,
};
pub const PIN_SPIM3_MISOOUT_CTRL14: PinctrlPin = PinctrlPin {
    offset: 0x4B4,
    bit: 19,
    clear: true,
};
pub const PIN_SPIM3_MISOOUT_CTRL32: PinctrlPin = PinctrlPin {
    offset: 0x694,
    bit: 19,
    clear: false,
};
pub const PIN_SPIM3_IO2OUT_CTRL2: PinctrlPin = PinctrlPin {
    offset: 0x414,
    bit: 20,
    clear: true,
};
pub const PIN_SPIM3_IO2OUT_CTRL14: PinctrlPin = PinctrlPin {
    offset: 0x4B4,
    bit: 20,
    clear: true,
};
pub const PIN_SPIM3_IO2OUT_CTRL32: PinctrlPin = PinctrlPin {
    offset: 0x694,
    bit: 20,
    clear: false,
};
pub const PIN_SPIM3_IO3OUT_CTRL2: PinctrlPin = PinctrlPin {
    offset: 0x414,
    bit: 21,
    clear: true,
};
pub const PIN_SPIM3_IO3OUT_CTRL14: PinctrlPin = PinctrlPin {
    offset: 0x4B4,
    bit: 21,
    clear: true,
};
pub const PIN_SPIM3_IO3OUT_CTRL32: PinctrlPin = PinctrlPin {
    offset: 0x694,
    bit: 21,
    clear: false,
};
pub const PIN_SPIM3_MUX_CTRL2: PinctrlPin = PinctrlPin {
    offset: 0x414,
    bit: 22,
    clear: true,
};

pub const PIN_SPIM3_MUX_CTRL14: PinctrlPin = PinctrlPin {
    offset: 0x4B4,
    bit: 22,
    clear: true,
};

pub const PIN_SPIM3_MUX_CTRL32: PinctrlPin = PinctrlPin {
    offset: 0x694,
    bit: 22,
    clear: false,
};
pub const PIN_SPIM3_RSTOUT_CTRL2: PinctrlPin = PinctrlPin {
    offset: 0x414,
    bit: 23,
    clear: true,
};

pub const PIN_SPIM3_RSTOUT_CTRL14: PinctrlPin = PinctrlPin {
    offset: 0x4B4,
    bit: 23,
    clear: true,
};

pub const PIN_SPIM3_RSTOUT_CTRL32: PinctrlPin = PinctrlPin {
    offset: 0x694,
    bit: 23,
    clear: false,
};

//Pin Group Aliases
pub const PINCTRL_FMC_QUAD: &[PinctrlPin] = &[PIN_FWSPIDQ2, PIN_FWSPIDQ3];
pub const PINCTRL_SPI1_QUAD: &[PinctrlPin] = &[PIN_SPI1DQ2, PIN_SPI1DQ3];
pub const PINCTRL_SPI2_DEFAULT: &[PinctrlPin] = &[
    PIN_SPI2CS0,
    PIN_SPI2CK,
    PIN_SPI2DQ0,
    PIN_SPI2DQ1,
    PIN_SPI2CS1,
];
pub const PINCTRL_SPI2_QUAD: &[PinctrlPin] = &[
    PIN_SPI2CS0,
    PIN_SPI2CK,
    PIN_SPI2DQ0,
    PIN_SPI2DQ1,
    PIN_SPI2CS1,
    PIN_SPI2DQ2,
    PIN_SPI2DQ3,
];
pub const PINCTRL_SPIM0_MUXSEL: &[PinctrlPin] = &[
    PIN_SPIM0_MUX_CTRL1,
    PIN_SPIM0_MUX_CTRL13,
    PIN_SPIM0_MUX_CTRL31,
];
pub const PINCTRL_SPIM1_MUXSEL: &[PinctrlPin] = &[
    PIN_SPIM1_MUX_CTRL1,
    PIN_SPIM1_MUX_CTRL13,
    PIN_SPIM1_MUX_CTRL31,
];
pub const PINCTRL_SPIM2_MUXSEL: &[PinctrlPin] = &[
    PIN_SPIM2_MUX_CTRL2,
    PIN_SPIM2_MUX_CTRL14,
    PIN_SPIM2_MUX_CTRL32,
];
pub const PINCTRL_SPIM3_MUXSEL: &[PinctrlPin] = &[
    PIN_SPIM3_MUX_CTRL2,
    PIN_SPIM3_MUX_CTRL14,
    PIN_SPIM3_MUX_CTRL32,
];

pub const PINCTRL_SPIM0_QUAD_DEFAULT: &[PinctrlPin] = &[
    PIN_SPIM0_CSIN_CTRL1,
    PIN_SPIM0_CSIN_CTRL13,
    PIN_SPIM0_CSIN_CTRL31,
    PIN_SPIM0_CLKIN_CTRL1,
    PIN_SPIM0_CLKIN_CTRL13,
    PIN_SPIM0_CLKIN_CTRL31,
    PIN_SPIM0_MOSIIN_CTRL1,
    PIN_SPIM0_MOSIIN_CTRL13,
    PIN_SPIM0_MOSIIN_CTRL31,
    PIN_SPIM0_MISOIN_CTRL1,
    PIN_SPIM0_MISOIN_CTRL13,
    PIN_SPIM0_MISOIN_CTRL31,
    PIN_SPIM0_IO2IN_CTRL1,
    PIN_SPIM0_IO2IN_CTRL13,
    PIN_SPIM0_IO2IN_CTRL31,
    PIN_SPIM0_IO3IN_CTRL1,
    PIN_SPIM0_IO3IN_CTRL13,
    PIN_SPIM0_IO3IN_CTRL31,
    PIN_SPIM0_CSNOUT_CTRL1,
    PIN_SPIM0_CSNOUT_CTRL13,
    PIN_SPIM0_CSNOUT_CTRL31,
    PIN_SPIM0_CLKOUT_CTRL1,
    PIN_SPIM0_CLKOUT_CTRL13,
    PIN_SPIM0_CLKOUT_CTRL31,
    PIN_SPIM0_MOSIOUT_CTRL1,
    PIN_SPIM0_MOSIOUT_CTRL13,
    PIN_SPIM0_MOSIOUT_CTRL31,
    PIN_SPIM0_MISOOUT_CTRL1,
    PIN_SPIM0_MISOOUT_CTRL13,
    PIN_SPIM0_MISOOUT_CTRL31,
    PIN_SPIM0_IO2OUT_CTRL1,
    PIN_SPIM0_IO2OUT_CTRL13,
    PIN_SPIM0_IO2OUT_CTRL31,
    PIN_SPIM0_IO3OUT_CTRL1,
    PIN_SPIM0_IO3OUT_CTRL13,
    PIN_SPIM0_IO3OUT_CTRL31,
    PIN_SPIM0_MUX_CTRL1,
    PIN_SPIM0_MUX_CTRL13,
    PIN_SPIM0_MUX_CTRL31,
    PIN_SPIM0_RSTOUT_CTRL1,
    PIN_SPIM0_RSTOUT_CTRL13,
    PIN_SPIM0_RSTOUT_CTRL31,
    PIN_SPIM0_RSTIN_CTRL2,
    PIN_SPIM0_RSTIN_CTRL14,
    PIN_SPIM0_RSTIN_CTRL32,
];
pub const PINCTRL_SPIM2_QUAD_DEFAULT: &[PinctrlPin] = &[
    PIN_SPIM2_CSIN_CTRL1,
    PIN_SPIM2_CSIN_CTRL13,
    PIN_SPIM2_CSIN_CTRL31,
    PIN_SPIM2_CLKIN_CTRL1,
    PIN_SPIM2_CLKIN_CTRL13,
    PIN_SPIM2_CLKIN_CTRL31,
    PIN_SPIM2_MOSIIN_CTRL1,
    PIN_SPIM2_MOSIIN_CTRL13,
    PIN_SPIM2_MOSIIN_CTRL31,
    PIN_SPIM2_MISOIN_CTRL1,
    PIN_SPIM2_MISOIN_CTRL13,
    PIN_SPIM2_MISOIN_CTRL31,
    PIN_SPIM2_IO2IN_CTRL2,
    PIN_SPIM2_IO2IN_CTRL14,
    PIN_SPIM2_IO2IN_CTRL32,
    PIN_SPIM2_IO3IN_CTRL2,
    PIN_SPIM2_IO3IN_CTRL14,
    PIN_SPIM2_IO3IN_CTRL32,
    PIN_SPIM2_CSNOUT_CTRL2,
    PIN_SPIM2_CSNOUT_CTRL14,
    PIN_SPIM2_CSNOUT_CTRL32,
    PIN_SPIM2_CLKOUT_CTRL2,
    PIN_SPIM2_CLKOUT_CTRL14,
    PIN_SPIM2_CLKOUT_CTRL32,
    PIN_SPIM2_MOSIOUT_CTRL2,
    PIN_SPIM2_MOSIOUT_CTRL14,
    PIN_SPIM2_MOSIOUT_CTRL32,
    PIN_SPIM2_MISOOUT_CTRL2,
    PIN_SPIM2_MISOOUT_CTRL14,
    PIN_SPIM2_MISOOUT_CTRL32,
    PIN_SPIM2_IO2OUT_CTRL2,
    PIN_SPIM2_IO2OUT_CTRL14,
    PIN_SPIM2_IO2OUT_CTRL32,
    PIN_SPIM2_IO3OUT_CTRL2,
    PIN_SPIM2_IO3OUT_CTRL14,
    PIN_SPIM2_IO3OUT_CTRL32,
    PIN_SPIM2_MUX_CTRL2,
    PIN_SPIM2_MUX_CTRL14,
    PIN_SPIM2_MUX_CTRL32,
    PIN_SPIM2_RSTOUT_CTRL2,
    PIN_SPIM2_RSTOUT_CTRL14,
    PIN_SPIM2_RSTOUT_CTRL32,
    PIN_SPIM2_RSTIN_CTRL2,
    PIN_SPIM2_RSTIN_CTRL14,
    PIN_SPIM2_RSTIN_CTRL32,
];

pub const PINCTRL_SPIM2_PINCTRL0: &[PinctrlPin] = &[
    PIN_SPIM2_CSIN_CTRL1,
    PIN_SPIM2_CSIN_CTRL13,
    PIN_SPIM2_CSIN_CTRL31,
    PIN_SPIM2_CLKIN_CTRL1,
    PIN_SPIM2_CLKIN_CTRL13,
    PIN_SPIM2_CLKIN_CTRL31,
    PIN_SPIM2_MOSIIN_CTRL1,
    PIN_SPIM2_MOSIIN_CTRL13,
    PIN_SPIM2_MOSIIN_CTRL31,
    PIN_SPIM2_MISOIN_CTRL1,
    PIN_SPIM2_MISOIN_CTRL13,
    PIN_SPIM2_MISOIN_CTRL31,
    PIN_SPIM2_IO2IN_CTRL2,
    PIN_SPIM2_IO2IN_CTRL14,
    PIN_SPIM2_IO2IN_CTRL32,
    PIN_SPIM2_IO3IN_CTRL2,
    PIN_SPIM2_IO3IN_CTRL14,
    PIN_SPIM2_IO3IN_CTRL32,
    PIN_SPIM2_CSNOUT_CTRL2,
    PIN_SPIM2_CSNOUT_CTRL14,
    PIN_SPIM2_CSNOUT_CTRL32,
    PIN_SPIM2_CLKOUT_CTRL2,
    PIN_SPIM2_CLKOUT_CTRL14,
    PIN_SPIM2_CLKOUT_CTRL32,
    PIN_SPIM2_MOSIOUT_CTRL2,
    PIN_SPIM2_MOSIOUT_CTRL14,
    PIN_SPIM2_MOSIOUT_CTRL32,
    PIN_SPIM2_MISOOUT_CTRL2,
    PIN_SPIM2_MISOOUT_CTRL14,
    PIN_SPIM2_MISOOUT_CTRL32,
    PIN_SPIM2_IO2OUT_CTRL2,
    PIN_SPIM2_IO2OUT_CTRL14,
    PIN_SPIM2_IO2OUT_CTRL32,
    PIN_SPIM2_IO3OUT_CTRL2,
    PIN_SPIM2_IO3OUT_CTRL14,
    PIN_SPIM2_IO3OUT_CTRL32,
    PIN_SPIM2_MUX_CTRL2,
    PIN_SPIM2_MUX_CTRL14,
    PIN_SPIM2_MUX_CTRL32,
];

pub const PINCTRL_SPIM3_PINCTRL0: &[PinctrlPin] = &[
    PIN_SPIM3_CSIN_CTRL2,
    PIN_SPIM3_CSIN_CTRL14,
    PIN_SPIM3_CSIN_CTRL32,
    PIN_SPIM3_CLKIN_CTRL2,
    PIN_SPIM3_CLKIN_CTRL14,
    PIN_SPIM3_CLKIN_CTRL32,
    PIN_SPIM3_MOSIIN_CTRL2,
    PIN_SPIM3_MOSIIN_CTRL14,
    PIN_SPIM3_MOSIIN_CTRL32,
    PIN_SPIM3_MISOIN_CTRL2,
    PIN_SPIM3_MISOIN_CTRL14,
    PIN_SPIM3_MISOIN_CTRL32,
    PIN_SPIM3_IO2IN_CTRL2,
    PIN_SPIM3_IO2IN_CTRL14,
    PIN_SPIM3_IO2IN_CTRL32,
    PIN_SPIM3_IO3IN_CTRL2,
    PIN_SPIM3_IO3IN_CTRL14,
    PIN_SPIM3_IO3IN_CTRL32,
    PIN_SPIM3_CSNOUT_CTRL2,
    PIN_SPIM3_CSNOUT_CTRL14,
    PIN_SPIM3_CSNOUT_CTRL32,
    PIN_SPIM3_CLKOUT_CTRL2,
    PIN_SPIM3_CLKOUT_CTRL14,
    PIN_SPIM3_CLKOUT_CTRL32,
    PIN_SPIM3_MOSIOUT_CTRL2,
    PIN_SPIM3_MOSIOUT_CTRL14,
    PIN_SPIM3_MOSIOUT_CTRL32,
    PIN_SPIM3_MISOOUT_CTRL2,
    PIN_SPIM3_MISOOUT_CTRL14,
    PIN_SPIM3_MISOOUT_CTRL32,
    PIN_SPIM3_IO2OUT_CTRL2,
    PIN_SPIM3_IO2OUT_CTRL14,
    PIN_SPIM3_IO2OUT_CTRL32,
    PIN_SPIM3_IO3OUT_CTRL2,
    PIN_SPIM3_IO3OUT_CTRL14,
    PIN_SPIM3_IO3OUT_CTRL32,
    PIN_SPIM3_MUX_CTRL2,
    PIN_SPIM3_MUX_CTRL14,
    PIN_SPIM3_MUX_CTRL32,
];

paste! {
    pub const PINCTRL_I2C0: &[PinctrlPin] = &[PIN_SCU414_28, PIN_SCU414_29];
    pub const PINCTRL_I2C1: &[PinctrlPin] = &[PIN_SCU414_30, PIN_SCU414_31];
    pub const PINCTRL_I2C2: &[PinctrlPin] = &[PIN_SCU418_0, PIN_SCU418_1];
    pub const PINCTRL_I2C3: &[PinctrlPin] = &[PIN_SCU418_2, PIN_SCU418_3];
    pub const PINCTRL_I2C4: &[PinctrlPin] = &[PIN_SCU418_4, PIN_SCU418_5];
    pub const PINCTRL_I2C5: &[PinctrlPin] = &[PIN_SCU418_6, PIN_SCU418_7];
    pub const PINCTRL_I2C6: &[PinctrlPin] = &[PIN_SCU418_8, PIN_SCU418_9];
    pub const PINCTRL_I2C7: &[PinctrlPin] = &[PIN_SCU418_10, PIN_SCU418_11];
    pub const PINCTRL_I2C8: &[PinctrlPin] = &[PIN_SCU418_12, PIN_SCU418_13];
    pub const PINCTRL_I2C9: &[PinctrlPin] = &[PIN_SCU418_14, PIN_SCU418_15];
    pub const PINCTRL_I2C10: &[PinctrlPin] =
        &[PIN_SCU4B8_16, PIN_SCU4B8_17, CLR_PIN_SCU418_16, CLR_PIN_SCU418_17];
    pub const PINCTRL_I2C11: &[PinctrlPin] =
        &[PIN_SCU4B8_18, PIN_SCU4B8_19, CLR_PIN_SCU418_18, CLR_PIN_SCU418_19];
    pub const PINCTRL_I2C12: &[PinctrlPin] =
        &[PIN_SCU4B8_20, PIN_SCU4B8_21, CLR_PIN_SCU418_20, CLR_PIN_SCU418_21];
    pub const PINCTRL_I2C13: &[PinctrlPin] =
        &[PIN_SCU4B8_22, PIN_SCU4B8_23, CLR_PIN_SCU418_22, CLR_PIN_SCU418_23];

    pub const PINCTRL_GPIOA0: &[PinctrlPin] = &[PIN_SCU410_0];
    pub const PINCTRL_GPIOA1: &[PinctrlPin] = &[PIN_SCU410_1];
    pub const PINCTRL_GPIOA2: &[PinctrlPin] = &[PIN_SCU410_2];
    pub const PINCTRL_GPIOA3: &[PinctrlPin] = &[PIN_SCU410_3];
    pub const PINCTRL_GPIOA4: &[PinctrlPin] = &[PIN_SCU410_4];
    pub const PINCTRL_GPIOA5: &[PinctrlPin] = &[PIN_SCU410_5];
    pub const PINCTRL_GPIOA6: &[PinctrlPin] = &[PIN_SCU410_6];
    pub const PINCTRL_GPIOA7: &[PinctrlPin] = &[PIN_SCU410_7];

    pub const PINCTRL_GPIOB0: &[PinctrlPin] = &[PIN_SCU410_8];
    pub const PINCTRL_GPIOB1: &[PinctrlPin] = &[PIN_SCU410_9];
    pub const PINCTRL_GPIOB2: &[PinctrlPin] = &[PIN_SCU410_10];
    pub const PINCTRL_GPIOB3: &[PinctrlPin] = &[PIN_SCU410_11];
    pub const PINCTRL_GPIOB4: &[PinctrlPin] = &[PIN_SCU410_12];
    pub const PINCTRL_GPIOB5: &[PinctrlPin] = &[PIN_SCU410_13];
    pub const PINCTRL_GPIOB6: &[PinctrlPin] = &[PIN_SCU410_14];
    pub const PINCTRL_GPIOB7: &[PinctrlPin] = &[PIN_SCU410_15];

    pub const PINCTRL_GPIOC0: &[PinctrlPin] = &[PIN_SCU410_16];
    pub const PINCTRL_GPIOC1: &[PinctrlPin] = &[PIN_SCU410_17];
    pub const PINCTRL_GPIOC2: &[PinctrlPin] = &[PIN_SCU410_18];
    pub const PINCTRL_GPIOC3: &[PinctrlPin] = &[PIN_SCU410_19];
    pub const PINCTRL_GPIOC4: &[PinctrlPin] = &[PIN_SCU410_20];
    pub const PINCTRL_GPIOC5: &[PinctrlPin] = &[PIN_SCU410_21];
    pub const PINCTRL_GPIOC6: &[PinctrlPin] = &[PIN_SCU410_22];
    pub const PINCTRL_GPIOC7: &[PinctrlPin] = &[PIN_SCU410_23];

    pub const PINCTRL_GPIOD0: &[PinctrlPin] = &[PIN_SCU410_24];
    pub const PINCTRL_GPIOD1: &[PinctrlPin] = &[PIN_SCU410_25];
    pub const PINCTRL_GPIOD2: &[PinctrlPin] = &[PIN_SCU410_26];
    pub const PINCTRL_GPIOD3: &[PinctrlPin] = &[PIN_SCU410_27];
    pub const PINCTRL_GPIOD4: &[PinctrlPin] = &[PIN_SCU410_28];
    pub const PINCTRL_GPIOD5: &[PinctrlPin] = &[PIN_SCU410_29];
    pub const PINCTRL_GPIOD6: &[PinctrlPin] = &[PIN_SCU410_30];
    pub const PINCTRL_GPIOD7: &[PinctrlPin] = &[PIN_SCU410_31];

    pub const PINCTRL_GPIOE0: &[PinctrlPin] = &[PIN_SCU414_0];
    pub const PINCTRL_GPIOE1: &[PinctrlPin] = &[PIN_SCU414_1];
    pub const PINCTRL_GPIOE2: &[PinctrlPin] = &[PIN_SCU414_2];
    pub const PINCTRL_GPIOE3: &[PinctrlPin] = &[PIN_SCU414_3];
    pub const PINCTRL_GPIOE4: &[PinctrlPin] = &[PIN_SCU414_4];
    pub const PINCTRL_GPIOE5: &[PinctrlPin] = &[PIN_SCU414_5];
    pub const PINCTRL_GPIOE6: &[PinctrlPin] = &[PIN_SCU414_6];
    pub const PINCTRL_GPIOE7: &[PinctrlPin] = &[PIN_SCU414_7];

    pub const PINCTRL_GPIOF0: &[PinctrlPin] = &[PIN_SCU414_8];
    pub const PINCTRL_GPIOF1: &[PinctrlPin] = &[PIN_SCU414_9];
    pub const PINCTRL_GPIOF2: &[PinctrlPin] = &[PIN_SCU414_10];
    pub const PINCTRL_GPIOF3: &[PinctrlPin] = &[PIN_SCU414_11];
    pub const PINCTRL_GPIOF4: &[PinctrlPin] = &[PIN_SCU414_12];
    pub const PINCTRL_GPIOF5: &[PinctrlPin] = &[PIN_SCU414_13];
    pub const PINCTRL_GPIOF6: &[PinctrlPin] = &[PIN_SCU414_14];
    pub const PINCTRL_GPIOF7: &[PinctrlPin] = &[PIN_SCU414_15];

    pub const PINCTRL_GPIOG0: &[PinctrlPin] = &[PIN_SCU414_16];
    pub const PINCTRL_GPIOG1: &[PinctrlPin] = &[PIN_SCU414_17];
    pub const PINCTRL_GPIOG2: &[PinctrlPin] = &[PIN_SCU414_18];
    pub const PINCTRL_GPIOG3: &[PinctrlPin] = &[PIN_SCU414_19];
    pub const PINCTRL_GPIOG4: &[PinctrlPin] = &[PIN_SCU414_20];
    pub const PINCTRL_GPIOG5: &[PinctrlPin] = &[PIN_SCU414_21];
    pub const PINCTRL_GPIOG6: &[PinctrlPin] = &[PIN_SCU414_22];
    pub const PINCTRL_GPIOG7: &[PinctrlPin] = &[PIN_SCU414_23];

    pub const PINCTRL_GPIOH0: &[PinctrlPin] = &[PIN_SCU414_24];
    pub const PINCTRL_GPIOH1: &[PinctrlPin] = &[PIN_SCU414_25];
    pub const PINCTRL_GPIOH2: &[PinctrlPin] = &[PIN_SCU414_26];
    pub const PINCTRL_GPIOH3: &[PinctrlPin] = &[PIN_SCU414_27];
    pub const PINCTRL_GPIOH4: &[PinctrlPin] = &[CLR_PIN_SCU414_28, CLR_PIN_SCU694_28];
    pub const PINCTRL_GPIOH5: &[PinctrlPin] = &[CLR_PIN_SCU414_29, CLR_PIN_SCU694_29];
    pub const PINCTRL_GPIOH6: &[PinctrlPin] = &[CLR_PIN_SCU414_30];
    pub const PINCTRL_GPIOH7: &[PinctrlPin] = &[CLR_PIN_SCU414_31];

    pub const PINCTRL_GPIOI0: &[PinctrlPin] = &[CLR_PIN_SCU418_0];
    pub const PINCTRL_GPIOI1: &[PinctrlPin] = &[CLR_PIN_SCU418_1];
    pub const PINCTRL_GPIOI2: &[PinctrlPin] = &[CLR_PIN_SCU418_2];
    pub const PINCTRL_GPIOI3: &[PinctrlPin] = &[CLR_PIN_SCU418_3];
    pub const PINCTRL_GPIOI4: &[PinctrlPin] = &[CLR_PIN_SCU418_4];
    pub const PINCTRL_GPIOI5: &[PinctrlPin] = &[CLR_PIN_SCU418_5];
    pub const PINCTRL_GPIOI6: &[PinctrlPin] = &[CLR_PIN_SCU418_6];
    pub const PINCTRL_GPIOI7: &[PinctrlPin] = &[CLR_PIN_SCU418_7];

    pub const PINCTRL_GPIOJ0: &[PinctrlPin] = &[CLR_PIN_SCU418_8, CLR_PIN_SCU4B8_8];
    pub const PINCTRL_GPIOJ1: &[PinctrlPin] = &[CLR_PIN_SCU418_9, CLR_PIN_SCU4B8_9];
    pub const PINCTRL_GPIOJ2: &[PinctrlPin] = &[CLR_PIN_SCU418_10, CLR_PIN_SCU4B8_10];
    pub const PINCTRL_GPIOJ3: &[PinctrlPin] = &[CLR_PIN_SCU418_11, CLR_PIN_SCU4B8_11];
    pub const PINCTRL_GPIOJ4: &[PinctrlPin] = &[CLR_PIN_SCU418_12, CLR_PIN_SCU4B8_12];
    pub const PINCTRL_GPIOJ5: &[PinctrlPin] = &[CLR_PIN_SCU418_13, CLR_PIN_SCU4B8_13];
    pub const PINCTRL_GPIOJ6: &[PinctrlPin] = &[CLR_PIN_SCU418_14, CLR_PIN_SCU4B8_14];
    pub const PINCTRL_GPIOJ7: &[PinctrlPin] = &[CLR_PIN_SCU418_15, CLR_PIN_SCU4B8_15];

    pub const PINCTRL_GPIOK0: &[PinctrlPin] = &[CLR_PIN_SCU418_16, CLR_PIN_SCU4B8_16];
    pub const PINCTRL_GPIOK1: &[PinctrlPin] = &[CLR_PIN_SCU418_17, CLR_PIN_SCU4B8_17];
    pub const PINCTRL_GPIOK2: &[PinctrlPin] = &[CLR_PIN_SCU418_18, CLR_PIN_SCU4B8_18];
    pub const PINCTRL_GPIOK3: &[PinctrlPin] = &[CLR_PIN_SCU418_19, CLR_PIN_SCU4B8_19];
    pub const PINCTRL_GPIOK4: &[PinctrlPin] = &[CLR_PIN_SCU418_20, CLR_PIN_SCU4B8_20];
    pub const PINCTRL_GPIOK5: &[PinctrlPin] = &[CLR_PIN_SCU418_21, CLR_PIN_SCU4B8_21];
    pub const PINCTRL_GPIOK6: &[PinctrlPin] = &[CLR_PIN_SCU418_22, CLR_PIN_SCU4B8_22];
    pub const PINCTRL_GPIOK7: &[PinctrlPin] = &[CLR_PIN_SCU418_23, CLR_PIN_SCU4B8_23];

    pub const PINCTRL_GPIOL4: &[PinctrlPin] = &[PIN_SCU418_28];
    pub const PINCTRL_GPIOL5: &[PinctrlPin] = &[PIN_SCU418_29];
    pub const PINCTRL_GPIOL6: &[PinctrlPin] = &[PIN_SCU418_30];
    pub const PINCTRL_GPIOL7: &[PinctrlPin] = &[PIN_SCU418_31];

    pub const PINCTRL_GPION0: &[PinctrlPin] = &[CLR_PIN_SCU41C_8];
    pub const PINCTRL_GPION1: &[PinctrlPin] = &[CLR_PIN_SCU41C_9, CLR_PIN_SCU69C_9];
    pub const PINCTRL_GPION2: &[PinctrlPin] = &[CLR_PIN_SCU41C_10];
    pub const PINCTRL_GPION3: &[PinctrlPin] = &[CLR_PIN_SCU41C_11, CLR_PIN_SCU69C_9];
    pub const PINCTRL_GPION4: &[PinctrlPin] = &[PIN_SCU41C_12];
    pub const PINCTRL_GPION5: &[PinctrlPin] = &[PIN_SCU41C_13];
    pub const PINCTRL_GPION6: &[PinctrlPin] = &[PIN_SCU41C_14];
    pub const PINCTRL_GPION7: &[PinctrlPin] = &[PIN_SCU41C_15];

    pub const PINCTRL_GPIOO0: &[PinctrlPin] = &[PIN_SCU41C_16];
    pub const PINCTRL_GPIOO1: &[PinctrlPin] = &[PIN_SCU41C_17];
    pub const PINCTRL_GPIOO2: &[PinctrlPin] = &[PIN_SCU41C_18];
    pub const PINCTRL_GPIOO3: &[PinctrlPin] = &[PIN_SCU41C_19];
    pub const PINCTRL_GPIOO4: &[PinctrlPin] = &[PIN_SCU41C_20];
    pub const PINCTRL_GPIOO5: &[PinctrlPin] = &[PIN_SCU41C_21];
    pub const PINCTRL_GPIOO6: &[PinctrlPin] = &[PIN_SCU41C_22];
    pub const PINCTRL_GPIOO7: &[PinctrlPin] = &[PIN_SCU41C_23];

    pub const PINCTRL_GPIOP0: &[PinctrlPin] = &[PIN_SCU41C_24];
    pub const PINCTRL_GPIOP1: &[PinctrlPin] = &[CLR_PIN_SCU41C_25, CLR_PIN_SCU4BC_25];
    pub const PINCTRL_GPIOP2: &[PinctrlPin] = &[CLR_PIN_SCU41C_26, CLR_PIN_SCU4BC_26];
    pub const PINCTRL_GPIOP3: &[PinctrlPin] = &[CLR_PIN_SCU41C_27, CLR_PIN_SCU4BC_27];
    pub const PINCTRL_GPIOP4: &[PinctrlPin] = &[CLR_PIN_SCU41C_28, CLR_PIN_SCU4BC_28];
    pub const PINCTRL_GPIOP5: &[PinctrlPin] = &[CLR_PIN_SCU41C_29, CLR_PIN_SCU4BC_29];
    pub const PINCTRL_GPIOP6: &[PinctrlPin] = &[CLR_PIN_SCU41C_30, CLR_PIN_SCU69C_30];
    pub const PINCTRL_GPIOP7: &[PinctrlPin] = &[CLR_PIN_SCU41C_31, CLR_PIN_SCU69C_31];

    pub const PINCTRL_GPIOQ0: &[PinctrlPin] = &[CLR_PIN_SCU430_0, CLR_PIN_SCU6B0_0];
    pub const PINCTRL_GPIOQ1: &[PinctrlPin] = &[CLR_PIN_SCU430_1, CLR_PIN_SCU6B0_1];
    pub const PINCTRL_GPIOQ2: &[PinctrlPin] = &[CLR_PIN_SCU430_2, CLR_PIN_SCU6B0_2];
    pub const PINCTRL_GPIOQ3: &[PinctrlPin] = &[CLR_PIN_SCU430_3, CLR_PIN_SCU6B0_3];
    pub const PINCTRL_GPIOQ4: &[PinctrlPin] = &[CLR_PIN_SCU430_4, CLR_PIN_SCU6B0_4];

    pub const PINCTRL_GPIOR2: &[PinctrlPin] = &[CLR_PIN_SCU430_10];
    pub const PINCTRL_GPIOR3: &[PinctrlPin] = &[CLR_PIN_SCU430_11];

    pub const PINCTRL_GPIOS2: &[PinctrlPin] = &[CLR_PIN_SCU430_17];
    pub const PINCTRL_GPIOS3: &[PinctrlPin] = &[CLR_PIN_SCU430_18];

    pub const PINCTRL_GPIOU0: &[PinctrlPin] = &[CLR_PIN_SCU434_0];
    pub const PINCTRL_GPIOU1: &[PinctrlPin] = &[CLR_PIN_SCU434_1];
    pub const PINCTRL_GPIOU2: &[PinctrlPin] = &[CLR_PIN_SCU434_2];
    pub const PINCTRL_GPIOU3: &[PinctrlPin] = &[CLR_PIN_SCU434_3];
    pub const PINCTRL_GPIOU4: &[PinctrlPin] = &[CLR_PIN_SCU434_4];
    pub const PINCTRL_GPIOU5: &[PinctrlPin] = &[CLR_PIN_SCU434_5];
    pub const PINCTRL_GPIOU6: &[PinctrlPin] = &[CLR_PIN_SCU434_6];
    pub const PINCTRL_GPIOU7: &[PinctrlPin] = &[CLR_PIN_SCU434_7];
}

#[macro_export]
macro_rules! modify_reg {
    ($reg:expr, $bit:expr, $clear:expr) => {{
        let reg = $reg;
        let bit = $bit;
        let clear = $clear;

        reg.modify(|r, w| unsafe {
            let current = r.bits();
            let new_val = if clear {
                current & !(1 << bit)
            } else {
                current | (1 << bit)
            };
            w.bits(new_val)
        });
    }};
}

impl Pinctrl {
    /// Write pinmux configuration to SCU register
    pub fn apply_pinctrl_group(pins: &[PinctrlPin]) {
        let scu = unsafe { &*ast1060_pac::Scu::ptr() };
        for pin in pins {
            match pin.offset {
                0x410 => modify_reg!(scu.scu410(), pin.bit, pin.clear),
                0x414 => modify_reg!(scu.scu414(), pin.bit, pin.clear),
                0x418 => modify_reg!(scu.scu418(), pin.bit, pin.clear),
                0x41C => modify_reg!(scu.scu41c(), pin.bit, pin.clear),
                0x430 => modify_reg!(scu.scu430(), pin.bit, pin.clear),
                0x434 => modify_reg!(scu.scu434(), pin.bit, pin.clear),
                0x4b0 => modify_reg!(scu.scu4b0(), pin.bit, pin.clear),
                0x4b4 => modify_reg!(scu.scu4b4(), pin.bit, pin.clear),
                0x4b8 => modify_reg!(scu.scu4b8(), pin.bit, pin.clear),
                0x4bc => modify_reg!(scu.scu4bc(), pin.bit, pin.clear),
                0x690 => modify_reg!(scu.scu690(), pin.bit, pin.clear),
                0x694 => modify_reg!(scu.scu694(), pin.bit, pin.clear),
                0x69c => modify_reg!(scu.scu69c(), pin.bit, pin.clear),
                0x6b0 => modify_reg!(scu.scu6b0(), pin.bit, pin.clear),
                _ => {}
            } //match
        } //for
    }
}
