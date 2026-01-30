use core::fmt::{self, Write};

use crate::{inter::without_inter, uart::UART1};

// https://os.phil-opp.com/vga-text-mode/ accessed 22/01/2026
#[macro_export]
macro_rules! print {
    ($($arg:tt)*) => ($crate::print::_print(format_args!($($arg)*)));
}

#[macro_export]
macro_rules! println {
    () => ($crate::print!("\n"));
    ($($arg:tt)*) => ($crate::print!("{}\n", format_args!($($arg)*)));
}

#[doc(hidden)]
pub fn _print(args: fmt::Arguments) {
    without_inter(|cs| {
        UART1.lock(cs).write_fmt(args).unwrap();
    });
}
