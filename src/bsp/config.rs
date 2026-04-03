//! Board-specific configuration constants

/// LED pin configuration
pub const LED_PIN: usize = 6; // PC6

/// UART2 pin configuration
pub const UART_TX_PIN: usize = 3; // PB3
pub const UART_RX_PIN: usize = 4; // PB4

/// UART baud rate
pub const UART_BAUDRATE: u32 = 921600;

/// External crystal frequency (HSE)
pub const HSE_FREQ_HZ: u32 = 8_000_000;

/// System clock frequency (SYSCLK)
pub const SYSCLK_FREQ_HZ: u32 = 170_000_000;