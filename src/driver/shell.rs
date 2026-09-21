//! Async line shell over USART2. No embedded-cli, no block_on.

use embassy_stm32::mode::Async;
use embassy_stm32::usart::{UartRx, UartTx};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;

use crate::app;
use crate::driver::led::Led;
use crate::driver::pwm::with_pwm;

const LINE_CAP: usize = 64;
const PROMPT: &[u8] = b"G431> ";

pub type LedHandle = Mutex<CriticalSectionRawMutex, Led>;

pub struct Shell {
    tx: UartTx<'static, Async>,
    line: [u8; LINE_CAP],
    len: usize,
}

impl Shell {
    pub fn new(tx: UartTx<'static, Async>) -> Self {
        Self {
            tx,
            line: [0; LINE_CAP],
            len: 0,
        }
    }

    pub async fn run(&mut self, rx: &mut UartRx<'static, Async>, led: &'static LedHandle) {
        let _ = self
            .write_all(b"STM32G431 FOC v0.1.0\r\nType 'help' for commands\r\n")
            .await;
        let _ = self.write_all(PROMPT).await;

        let mut byte = [0u8; 1];
        loop {
            match rx.read(&mut byte).await {
                Ok(()) => self.on_byte(byte[0], led).await,
                Err(e) => defmt::error!("UART RX error: {:?}", e),
            }
        }
    }

    async fn on_byte(&mut self, b: u8, led: &'static LedHandle) {
        match b {
            b'\r' | b'\n' => {
                let _ = self.write_all(b"\r\n").await;
                if self.len > 0 {
                    let mut tmp = [0u8; LINE_CAP];
                    tmp[..self.len].copy_from_slice(&self.line[..self.len]);
                    let n = self.len;
                    self.len = 0;
                    if let Ok(line) = core::str::from_utf8(&tmp[..n]) {
                        self.dispatch(line.trim(), led).await;
                    } else {
                        let _ = self.write_all(b"bad utf8\r\n").await;
                    }
                }
                let _ = self.write_all(PROMPT).await;
            }
            0x08 | 0x7F => {
                if self.len > 0 {
                    self.len -= 1;
                    let _ = self.write_all(b"\x08 \x08").await;
                }
            }
            b if b.is_ascii_graphic() || b == b' ' => {
                if self.len < LINE_CAP {
                    self.line[self.len] = b;
                    self.len += 1;
                    let _ = self.write_all(&[b]).await;
                }
            }
            _ => {}
        }
    }

    async fn dispatch(&mut self, line: &str, led: &'static LedHandle) {
        let mut toks = line.split_whitespace();
        let Some(cmd) = toks.next() else { return };
        match cmd {
            "help" | "?" => {
                let _ = self.write_all(
                    b"help | hello [name] | clear | version | echo [text]\r\n\
                      led on|off|toggle | system info | enc\r\n\
                      foc status|start|stop|align|id <mA>|iq <mA>|poles <n>|pwm <%>\r\n",
                )
                .await;
            }
            "hello" => {
                let name = toks.next().unwrap_or("World");
                let _ = self.write_all(b"Hello, ").await;
                let _ = self.write_all(name.as_bytes()).await;
                let _ = self.write_all(b"!\r\n").await;
            }
            "clear" => {
                let _ = self.write_all(b"\x1b[2J\x1b[H").await;
            }
            "version" => {
                let _ = self.write_all(b"STM32G431 FOC v0.1.0\r\n").await;
            }
            "echo" => {
                if let Some(rest) = line.get(4..).map(str::trim_start) {
                    if !rest.is_empty() {
                        let _ = self.write_all(rest.as_bytes()).await;
                    }
                }
                let _ = self.write_all(b"\r\n").await;
            }
            "led" => match toks.next() {
                Some("on") => {
                    led.lock().await.on();
                    let _ = self.write_all(b"LED ON\r\n").await;
                }
                Some("off") => {
                    led.lock().await.off();
                    let _ = self.write_all(b"LED OFF\r\n").await;
                }
                Some("toggle") => {
                    led.lock().await.toggle();
                    let _ = self.write_all(b"LED TOGGLE\r\n").await;
                }
                _ => {
                    let _ = self.write_all(b"usage: led on|off|toggle\r\n").await;
                }
            },
            "system" => match toks.next() {
                Some("info") => {
                    let _ = self
                        .write_all(b"MCU: STM32G431CB\r\nClock: 170MHz\r\nHSE: 8MHz\r\n")
                        .await;
                }
                _ => {
                    let _ = self.write_all(b"usage: system info\r\n").await;
                }
            },
            "enc" => {
                let ok = if app::enc_valid() { "ok" } else { "fail" };
                let _ = self.write_all(b"enc raw=").await;
                let _ = self.write_i32(app::enc_raw() as i32).await;
                let _ = self.write_all(b" mdeg=").await;
                let _ = self.write_i32(app::enc_mdeg()).await;
                let _ = self.write_all(b" w=").await;
                let _ = self.write_i32(app::enc_omega_mrad()).await;
                let _ = self.write_all(b" ").await;
                let _ = self.write_all(ok.as_bytes()).await;
                let _ = self.write_all(b"\r\n").await;
            }
            "foc" => self.cmd_foc(&mut toks).await,
            _ => {
                let _ = self.write_all(b"unknown: ").await;
                let _ = self.write_all(cmd.as_bytes()).await;
                let _ = self.write_all(b" (help)\r\n").await;
            }
        }
    }

    async fn cmd_foc(&mut self, toks: &mut core::str::SplitWhitespace<'_>) {
        match toks.next() {
            Some("status") => {
                let on = if app::enabled() { "on" } else { "off" };
                let _ = self.write_all(b"foc ").await;
                let _ = self.write_all(on.as_bytes()).await;
                let _ = self.write_all(b" id_ma=").await;
                let _ = self.write_i32(app::id_ma()).await;
                let _ = self.write_all(b" iq_ma=").await;
                let _ = self.write_i32(app::iq_ma()).await;
                let _ = self.write_all(b" poles=").await;
                let _ = self.write_i32(app::poles() as i32).await;
                let _ = self.write_all(b" pwm=").await;
                let _ = self.write_i32(app::pwm_pct() as i32).await;
                let _ = self.write_all(b"%\r\n").await;
            }
            Some("start") => {
                app::set_enable(true);
                let _ = with_pwm(|p| p.enable());
                let _ = self.write_all(b"PWM outputs ON\r\n").await;
            }
            Some("stop") => {
                app::set_enable(false);
                let _ = with_pwm(|p| p.disable());
                let _ = self.write_all(b"PWM outputs OFF\r\n").await;
            }
            Some("align") => {
                app::request_align();
                let _ = self.write_all(b"align requested\r\n").await;
            }
            Some("id") => match parse_i32(toks.next()) {
                Some(ma) => {
                    app::set_id_ma(ma);
                    let _ = self.write_all(b"id_ref=").await;
                    let _ = self.write_i32(app::id_ma()).await;
                    let _ = self.write_all(b" mA\r\n").await;
                }
                None => {
                    let _ = self.write_all(b"usage: foc id <mA>\r\n").await;
                }
            },
            Some("iq") => match parse_i32(toks.next()) {
                Some(ma) => {
                    app::set_iq_ma(ma);
                    let _ = self.write_all(b"iq_ref=").await;
                    let _ = self.write_i32(app::iq_ma()).await;
                    let _ = self.write_all(b" mA\r\n").await;
                }
                None => {
                    let _ = self.write_all(b"usage: foc iq <mA>\r\n").await;
                }
            },
            Some("poles") => match parse_u8(toks.next()) {
                Some(n) => {
                    app::set_poles(n);
                    let _ = self.write_all(b"poles=").await;
                    let _ = self.write_i32(app::poles() as i32).await;
                    let _ = self.write_all(b"\r\n").await;
                }
                None => {
                    let _ = self.write_all(b"usage: foc poles <n>\r\n").await;
                }
            },
            Some("pwm") => match parse_u8(toks.next()) {
                Some(pct) => {
                    app::set_pwm_pct(pct);
                    let _ = with_pwm(|p| p.set_duty_all(app::pwm_pct() as f32 / 100.0));
                    let _ = self.write_all(b"duty=").await;
                    let _ = self.write_i32(app::pwm_pct() as i32).await;
                    let _ = self.write_all(b"%\r\n").await;
                }
                None => {
                    let _ = self.write_all(b"usage: foc pwm <0-100>\r\n").await;
                }
            },
            _ => {
                let _ = self.write_all(b"usage: foc status|start|stop|align|id|iq|poles|pwm\r\n").await;
            }
        }
    }

    async fn write_all(&mut self, buf: &[u8]) -> Result<(), ()> {
        self.tx.write(buf).await.map_err(|_| ())
    }

    async fn write_i32(&mut self, v: i32) -> Result<(), ()> {
        let mut buf = [0u8; 12];
        let n = fmt_i32(v, &mut buf);
        self.write_all(&buf[..n]).await
    }
}

fn parse_i32(s: Option<&str>) -> Option<i32> {
    s.and_then(|t| t.parse().ok())
}

fn parse_u8(s: Option<&str>) -> Option<u8> {
    s.and_then(|t| t.parse().ok())
}

fn fmt_i32(v: i32, out: &mut [u8; 12]) -> usize {
    if v == 0 {
        out[0] = b'0';
        return 1;
    }
    let neg = v < 0;
    let mut n = if neg { (v as i64).unsigned_abs() } else { v as u64 };
    let mut tmp = [0u8; 11];
    let mut i = 0;
    while n > 0 {
        tmp[i] = b'0' + (n % 10) as u8;
        n /= 10;
        i += 1;
    }
    let mut o = 0;
    if neg {
        out[0] = b'-';
        o = 1;
    }
    while i > 0 {
        i -= 1;
        out[o] = tmp[i];
        o += 1;
    }
    o
}
