//! Shell module using embedded-cli
//!
//! Provides UART shell interface for debugging and control.

use core::convert::Infallible;

use embassy_stm32::usart::UartTx;
use embassy_stm32::mode::Async;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embedded_cli::cli::{CliBuilder, CliHandle};
use embedded_cli::codes;
use embedded_cli::Command;
use embedded_io::Write;
use static_cell::StaticCell;
use ufmt::uwrite;

/// LED subcommands
#[derive(Command)]
pub enum LedCommand {
    /// Turn LED on
    On,
    /// Turn LED off
    Off,
    /// Toggle LED
    Toggle,
}

/// System subcommands
#[derive(Command)]
pub enum SystemCommand {
    /// Show system info
    Info,
}

/// All shell commands
#[derive(Command)]
pub enum TopCommand<'a> {
    /// Say hello
    Hello {
        /// Name to greet
        name: Option<&'a str>,
    },
    /// Clear screen
    Clear,
    /// Show version
    Version,
    /// Echo text
    Echo {
        /// Text to echo
        text: Option<&'a str>,
    },
    /// Control LED
    Led {
        #[command(subcommand)]
        command: LedCommand,
    },
    /// System commands
    System {
        #[command(subcommand)]
        command: SystemCommand,
    },
}

/// Process command callback
fn process_command<W>(cli: &mut CliHandle<'_, W, W::Error>, cmd: TopCommand<'_>)
where
    W: Write,
{
    let writer = cli.writer();
    let _ = match cmd {
        TopCommand::Hello { name } => {
            uwrite!(writer, "Hello, {}!\r\n", name.unwrap_or("World"))
        }
        TopCommand::Clear => writer.write_str("\x1b[2J\x1b[H"),
        TopCommand::Version => writer.write_str("STM32G431 FOC v0.1.0\r\n"),
        TopCommand::Echo { text } => uwrite!(writer, "{}\r\n", text.unwrap_or("")),
        TopCommand::Led { command } => match command {
            LedCommand::On => writer.write_str("LED ON\r\n"),
            LedCommand::Off => writer.write_str("LED OFF\r\n"),
            LedCommand::Toggle => writer.write_str("LED TOGGLE\r\n"),
        },
        TopCommand::System { command } => match command {
            SystemCommand::Info => writer.write_str("MCU: STM32G431CB\r\nClock: 170MHz\r\nHSE: 8MHz\r\n"),
        },
    };
}

/// Static UART TX storage
static SHELL_TX: StaticCell<Mutex<CriticalSectionRawMutex, UartTx<'static, Async>>> = StaticCell::new();

/// Initialize shell TX
pub fn init_shell_tx(tx: UartTx<'static, Async>) {
    SHELL_TX.init(Mutex::new(tx));
}

/// Get shell TX reference
pub fn get_shell_tx() -> &'static Mutex<CriticalSectionRawMutex, UartTx<'static, Async>> {
    unsafe {
        &*(&SHELL_TX as *const StaticCell<Mutex<CriticalSectionRawMutex, UartTx<'static, Async>>>
           as *const Mutex<CriticalSectionRawMutex, UartTx<'static, Async>>)
    }
}

/// Writer wrapper for shell
pub struct ShellWriter {
    tx: &'static Mutex<CriticalSectionRawMutex, UartTx<'static, Async>>,
}

impl ShellWriter {
    pub fn new(tx: &'static Mutex<CriticalSectionRawMutex, UartTx<'static, Async>>) -> Self {
        Self { tx }
    }
}

impl embedded_io::ErrorType for ShellWriter {
    type Error = Infallible;
}

impl Write for ShellWriter {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        embassy_futures::block_on(async {
            let mut tx = self.tx.lock().await;
            let _ = tx.write(buf).await;
            Ok(buf.len())
        })
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        embassy_futures::block_on(async {
            let mut tx = self.tx.lock().await;
            let _ = tx.flush().await;
            Ok(())
        })
    }
}

impl ufmt::uWrite for ShellWriter {
    type Error = Infallible;
    fn write_str(&mut self, s: &str) -> Result<(), Self::Error> {
        self.write_all(s.as_bytes())
    }
}

/// Shell wrapper
pub struct Shell {
    cli: embedded_cli::cli::Cli<ShellWriter, Infallible, [u8; 64], [u8; 64]>,
}

impl Shell {
    pub fn new(writer: ShellWriter) -> Self {
        let cli = CliBuilder::default()
            .writer(writer)
            .prompt("G431> ")
            .command_buffer([0u8; 64])
            .history_buffer([0u8; 64])
            .build()
            .expect("CLI build");
        Self { cli }
    }

    pub fn process(&mut self, byte: u8) {
        let byte = if byte == 0x7F { codes::BACKSPACE } else { byte };
        let _ = self.cli.process_byte::<TopCommand, _>(
            byte,
            &mut TopCommand::processor(|cli, cmd| {
                process_command(cli, cmd);
                Ok(())
            }),
        );
    }

    pub fn print_welcome(&mut self) {
        let _ = self.cli.write(|writer| {
            writer.write_str("STM32G431 FOC v0.1.0\r\n")?;
            writer.write_str("Type 'help' for commands\r\n")
        });
    }
}