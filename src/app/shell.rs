//! USART2 line editor. Talks to `app::control` / telemetry, not to PWM.

use embassy_stm32::mode::Async;
use embassy_stm32::usart::{RingBufferedUartRx, UartTx};
use embassy_time::Timer;

use crate::app::{self, control};
use crate::driver::led::LedHandle;

const LINE_CAP: usize = 64;
const HIST_CAP: usize = 8;
const PROMPT: &[u8] = b"G431> ";

const ROOT_CMDS: &[&str] = &[
    "help", "?", "hello", "clear", "version", "echo", "led", "system", "enc", "adc", "foc",
];
const LED_SUB: &[&str] = &["on", "off", "toggle"];
const SYSTEM_SUB: &[&str] = &["info"];
const FOC_SUB: &[&str] = &[
    "status", "start", "stop", "align", "id", "iq", "poles", "pwm", "offset", "zero", "openloop", "rpm", "kp",
    "ki", "skp", "ski",
];

#[derive(Clone, Copy, PartialEq, Eq)]
enum Esc {
    None,
    Lead,
    Csi,
    Ss3,
}

pub struct Shell {
    tx: UartTx<'static, Async>,
    line: [u8; LINE_CAP],
    len: usize,
    hist: [[u8; LINE_CAP]; HIST_CAP],
    hist_len: [usize; HIST_CAP],
    hist_head: usize,
    hist_count: usize,
    hist_nav: Option<usize>,
    draft: [u8; LINE_CAP],
    draft_len: usize,
    esc: Esc,
    skip_lf: bool,
    painted: usize,
}

impl Shell {
    pub fn new(tx: UartTx<'static, Async>) -> Self {
        Self {
            tx,
            line: [0; LINE_CAP],
            len: 0,
            hist: [[0; LINE_CAP]; HIST_CAP],
            hist_len: [0; HIST_CAP],
            hist_head: 0,
            hist_count: 0,
            hist_nav: None,
            draft: [0; LINE_CAP],
            draft_len: 0,
            esc: Esc::None,
            skip_lf: false,
            painted: 0,
        }
    }

    pub async fn run(&mut self, rx: &mut RingBufferedUartRx<'static>, led: &'static LedHandle) {
        let _ = self
            .write_all(b"STM32G431 FOC v0.1.0\r\nType 'help'. Tab completes, up/down is history.\r\n")
            .await;
        let _ = self.write_all(PROMPT).await;

        let mut buf = [0u8; 64];
        loop {
            match rx.read(&mut buf).await {
                Ok(n) => {
                    for &b in &buf[..n] {
                        self.on_byte(b, led).await;
                    }
                }
                Err(e) => {
                    defmt::error!("UART RX error: {:?}", e);
                    Timer::after_millis(10).await;
                }
            }
        }
    }

    async fn on_byte(&mut self, b: u8, led: &'static LedHandle) {
        if drop_crlf_lf(&mut self.skip_lf, b) {
            return;
        }

        match self.esc {
            Esc::None => {}
            Esc::Lead => {
                self.esc = match b {
                    b'[' => Esc::Csi,
                    b'O' => Esc::Ss3,
                    _ => Esc::None,
                };
                if self.esc != Esc::None {
                    return;
                }
            }
            Esc::Csi => {
                if matches!(b, b'0'..=b'9' | b';' | b'?') {
                    return;
                }
                self.esc = Esc::None;
                match b {
                    b'A' => self.history_up().await,
                    b'B' => self.history_down().await,
                    _ => {}
                }
                return;
            }
            Esc::Ss3 => {
                self.esc = Esc::None;
                match b {
                    b'A' => self.history_up().await,
                    b'B' => self.history_down().await,
                    _ => {}
                }
                return;
            }
        }

        match b {
            0x1b => self.esc = Esc::Lead,
            0x03 => {
                self.hist_nav = None;
                self.len = 0;
                self.painted = 0;
                let _ = self.write_all(b"^C\r\n").await;
                let _ = self.write_all(PROMPT).await;
            }
            0x15 => {
                self.hist_nav = None;
                self.len = 0;
                self.redraw_line().await;
            }
            b'\t' => {
                self.hist_nav = None;
                self.complete().await;
            }
            b'\r' => {
                note_cr(&mut self.skip_lf);
                self.submit(led).await;
            }
            b'\n' => self.submit(led).await,
            0x08 | 0x7F => {
                self.hist_nav = None;
                if self.len > 0 {
                    self.len -= 1;
                    if self.painted > 0 {
                        self.painted -= 1;
                    }
                    let _ = self.write_all(b"\x08 \x08").await;
                }
            }
            b if b.is_ascii_graphic() || b == b' ' => {
                self.hist_nav = None;
                if self.len < LINE_CAP {
                    self.line[self.len] = b;
                    self.len += 1;
                    self.painted = self.len;
                    let _ = self.write_all(&[b]).await;
                }
            }
            _ => {}
        }
    }

    async fn submit(&mut self, led: &'static LedHandle) {
        let _ = self.write_all(b"\r\n").await;
        self.hist_nav = None;
        self.painted = 0;
        if self.len > 0 {
            let mut tmp = [0u8; LINE_CAP];
            tmp[..self.len].copy_from_slice(&self.line[..self.len]);
            let n = self.len;
            self.len = 0;
            if let Ok(line) = core::str::from_utf8(&tmp[..n]) {
                let line = line.trim();
                self.push_history(line);
                self.dispatch(line, led).await;
            } else {
                let _ = self.write_all(b"bad utf8\r\n").await;
            }
        }
        let _ = self.write_all(PROMPT).await;
    }

    fn push_history(&mut self, line: &str) {
        if line.is_empty() || line.len() > LINE_CAP {
            return;
        }
        if self.hist_count > 0 {
            let last = (self.hist_head + HIST_CAP - 1) % HIST_CAP;
            if self.hist_len[last] == line.len() && self.hist[last][..line.len()] == *line.as_bytes() {
                return;
            }
        }
        let i = self.hist_head;
        self.hist[i][..line.len()].copy_from_slice(line.as_bytes());
        self.hist_len[i] = line.len();
        self.hist_head = (self.hist_head + 1) % HIST_CAP;
        if self.hist_count < HIST_CAP {
            self.hist_count += 1;
        }
    }

    fn hist_index(&self, nav: usize) -> usize {
        (self.hist_head + HIST_CAP - 1 - nav) % HIST_CAP
    }

    fn load_hist(&mut self, nav: usize) {
        let i = self.hist_index(nav);
        let n = self.hist_len[i];
        self.line[..n].copy_from_slice(&self.hist[i][..n]);
        self.len = n;
    }

    async fn history_up(&mut self) {
        if self.hist_count == 0 {
            return;
        }
        match self.hist_nav {
            None => {
                self.draft[..self.len].copy_from_slice(&self.line[..self.len]);
                self.draft_len = self.len;
                self.hist_nav = Some(0);
                self.load_hist(0);
            }
            Some(n) if n + 1 < self.hist_count => {
                self.hist_nav = Some(n + 1);
                self.load_hist(n + 1);
            }
            Some(_) => return,
        }
        self.redraw_line().await;
    }

    async fn history_down(&mut self) {
        match self.hist_nav {
            None => {}
            Some(0) => {
                self.hist_nav = None;
                self.line[..self.draft_len].copy_from_slice(&self.draft[..self.draft_len]);
                self.len = self.draft_len;
                self.redraw_line().await;
            }
            Some(n) => {
                self.hist_nav = Some(n - 1);
                self.load_hist(n - 1);
                self.redraw_line().await;
            }
        }
    }

    async fn redraw_line(&mut self) {
        let _ = self.write_all(b"\r").await;
        let _ = self.write_all(PROMPT).await;
        let n = self.len;
        if n > 0 {
            let mut tmp = [0u8; LINE_CAP];
            tmp[..n].copy_from_slice(&self.line[..n]);
            let _ = self.write_all(&tmp[..n]).await;
        }
        if self.painted > n {
            let extra = self.painted - n;
            let spaces = [b' '; LINE_CAP];
            let _ = self.write_all(&spaces[..extra]).await;
            let backs = [0x08; LINE_CAP];
            let _ = self.write_all(&backs[..extra]).await;
        }
        let _ = self.write_all(b"\x1b[K").await;
        self.painted = n;
    }

    async fn complete(&mut self) {
        let Ok(line) = core::str::from_utf8(&self.line[..self.len]) else {
            return;
        };
        let words = completion_words(line);
        let Some((prefix, list)) = words else { return };
        let mut pfx_buf = [0u8; LINE_CAP];
        let pfx_n = prefix.len();
        pfx_buf[..pfx_n].copy_from_slice(prefix.as_bytes());

        let mut matches: [&str; 12] = [""; 12];
        let mut n = 0;
        for &w in list {
            if w.starts_with(prefix) && n < matches.len() {
                matches[n] = w;
                n += 1;
            }
        }
        if n == 0 {
            return;
        }
        let prefix = core::str::from_utf8(&pfx_buf[..pfx_n]).unwrap_or("");

        let lcp = longest_common_prefix(&matches[..n]);
        if lcp.len() > prefix.len() {
            if apply_completion(&mut self.line, &mut self.len, prefix, lcp) && n == 1 {
                maybe_space_after(&mut self.line, &mut self.len, lcp);
            }
            self.redraw_line().await;
            if n == 1 {
                return;
            }
        }

        if n > 1 {
            let _ = self.write_all(b"\r\n").await;
            for (i, word) in matches[..n].iter().enumerate() {
                if i > 0 {
                    let _ = self.write_all(b" ").await;
                }
                let _ = self.write_all(word.as_bytes()).await;
            }
            let _ = self.write_all(b"\r\n").await;
            self.redraw_line().await;
        } else if lcp.len() == prefix.len() {
            maybe_space_after(&mut self.line, &mut self.len, matches[0]);
            self.redraw_line().await;
        }
    }

    async fn dispatch(&mut self, line: &str, led: &'static LedHandle) {
        let mut toks = line.split_whitespace();
        let Some(cmd) = toks.next() else { return };
        match cmd {
            "help" | "?" => {
                let _ = self.write_all(
                    b"help | hello [name] | clear | version | echo [text]\r\n\
                      led on|off|toggle | system info | enc | adc\r\n\
                      foc status|start|stop|align|id <mA>|iq <mA>|poles <n>|pwm <%>\r\n\
                      foc kp|ki|skp|ski [x] | rpm <n> | openloop <vq_mV> <Hz>\r\n\
                      Tab: complete   Up/Down: history   Ctrl-C/U: abort/kill\r\n",
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
            "adc" => {
                let _ = self.write_all(b"iu=").await;
                let _ = self.write_i32(app::iu_ma() as i32).await;
                let _ = self.write_all(b" iv=").await;
                let _ = self.write_i32(app::iv_ma() as i32).await;
                let _ = self.write_all(b" iw=").await;
                let _ = self.write_i32(app::iw_ma() as i32).await;
                let _ = self.write_all(b" mA  vbus=").await;
                let _ = self.write_i32(app::vbus_mv() as i32).await;
                let _ = self.write_all(b" mV  t=").await;
                let _ = self.write_i32(app::temp_c10() as i32).await;
                let _ = self.write_all(b" (0.1C) raw ").await;
                let _ = self.write_i32(app::iu_raw() as i32).await;
                let _ = self.write_all(b" ").await;
                let _ = self.write_i32(app::iv_raw() as i32).await;
                let _ = self.write_all(b" ").await;
                let _ = self.write_i32(app::iw_raw() as i32).await;
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
                let s = control::snapshot();
                let _ = self.write_all(b"foc ").await;
                let _ = self.write_all(s.mode.as_str().as_bytes()).await;
                let _ = self.write_all(b" id_ma=").await;
                let _ = self.write_i32(s.id_ma).await;
                let _ = self.write_all(b" iq_ma=").await;
                let _ = self.write_i32(s.iq_ma).await;
                let _ = self.write_all(b" poles=").await;
                let _ = self.write_i32(s.poles as i32).await;
                let _ = self.write_all(b" pwm=").await;
                let _ = self.write_i32(s.pwm_pct as i32).await;
                let _ = self.write_all(b"% id=").await;
                let _ = self.write_i32(app::id_meas_ma() as i32).await;
                let _ = self.write_all(b" iq=").await;
                let _ = self.write_i32(app::iq_meas_ma() as i32).await;
                let _ = self.write_all(b" mA off=").await;
                let _ = self.write_i32(control::theta_e_off_mrad()).await;
                let _ = self.write_all(b" mrad rpm=").await;
                let _ = self.write_i32(app::rpm_meas()).await;
                let _ = self.write_all(b"/").await;
                let _ = self.write_i32(control::rpm_ref()).await;
                let _ = self.write_all(b"\r\n").await;
            }
            Some("start") => {
                if control::mode() == control::Mode::Fault {
                    let _ = self.write_all(b"blocked: fault (foc stop)\r\n").await;
                } else {
                    control::start();
                    let _ = self.write_all(b"current loop ON\r\n").await;
                }
            }
            Some("stop") => {
                control::stop();
                let _ = self.write_all(b"PWM outputs OFF\r\n").await;
            }
            Some("align") => {
                if control::mode() == control::Mode::Fault {
                    let _ = self.write_all(b"blocked: fault (foc stop)\r\n").await;
                } else {
                    control::request_align();
                    let _ = self.write_all(b"align requested\r\n").await;
                }
            }
            Some("id") => match parse_i32(toks.next()) {
                Some(ma) => {
                    control::set_id_ma(ma);
                    let _ = self.write_all(b"id_ref=").await;
                    let _ = self.write_i32(control::id_ma()).await;
                    let _ = self.write_all(b" mA\r\n").await;
                }
                None => {
                    let _ = self.write_all(b"usage: foc id <mA>\r\n").await;
                }
            },
            Some("iq") => match parse_i32(toks.next()) {
                Some(ma) => {
                    control::set_iq_ma(ma);
                    let _ = self.write_all(b"iq_ref=").await;
                    let _ = self.write_i32(control::iq_ma()).await;
                    let _ = self.write_all(b" mA\r\n").await;
                }
                None => {
                    let _ = self.write_all(b"usage: foc iq <mA>\r\n").await;
                }
            },
            Some("poles") => match parse_u8(toks.next()) {
                Some(n) => {
                    control::set_poles(n);
                    let _ = self.write_all(b"poles=").await;
                    let _ = self.write_i32(control::poles() as i32).await;
                    let _ = self.write_all(b"\r\n").await;
                }
                None => {
                    let _ = self.write_all(b"usage: foc poles <n>\r\n").await;
                }
            },
            Some("offset") => match toks.next() {
                None => {
                    let _ = self.write_all(b"theta_e_off=").await;
                    let _ = self.write_i32(control::theta_e_off_mrad()).await;
                    let _ = self.write_all(b" mrad\r\n").await;
                }
                Some(s) => match parse_i32(Some(s)) {
                    Some(mrad) => {
                        control::set_theta_e_off_mrad(mrad);
                        let _ = self.write_all(b"theta_e_off=").await;
                        let _ = self.write_i32(control::theta_e_off_mrad()).await;
                        let _ = self.write_all(b" mrad\r\n").await;
                    }
                    None => {
                        let _ = self.write_all(b"usage: foc offset [mrad]\r\n").await;
                    }
                },
            },
            Some("rpm") => match parse_i32(toks.next()) {
                Some(rpm) => {
                    if control::mode() == control::Mode::Fault {
                        let _ = self.write_all(b"blocked: fault (foc stop)\r\n").await;
                    } else {
                        control::start_speed(rpm);
                        let _ = self.write_all(b"speed rpm_ref=").await;
                        let _ = self.write_i32(control::rpm_ref()).await;
                        let _ = self.write_all(b"\r\n").await;
                    }
                }
                None => {
                    let _ = self.write_all(b"rpm=").await;
                    let _ = self.write_i32(app::rpm_meas()).await;
                    let _ = self.write_all(b"  ref=").await;
                    let _ = self.write_i32(control::rpm_ref()).await;
                    let _ = self.write_all(b"\r\n").await;
                }
            },
            Some("openloop") => match (parse_i32(toks.next()), parse_u8(toks.next())) {
                (Some(vq_mv), Some(hz)) => {
                    if control::mode() == control::Mode::Fault {
                        let _ = self.write_all(b"blocked: fault (foc stop)\r\n").await;
                    } else {
                        control::start_openloop(vq_mv, hz);
                        let _ = self.write_all(b"openloop vq=").await;
                        let _ = self.write_i32(control::ol_vq_mv()).await;
                        let _ = self.write_all(b" mV  ").await;
                        let _ = self.write_i32(control::ol_hz() as i32).await;
                        let _ = self.write_all(b" Hz\r\n").await;
                    }
                }
                _ => {
                    let _ = self.write_all(b"usage: foc openloop <vq_mV> <Hz>\r\n").await;
                }
            },
            Some("kp") => self.cmd_gain("kp", toks.next(), true, true).await,
            Some("ki") => self.cmd_gain("ki", toks.next(), true, false).await,
            Some("skp") => self.cmd_gain("skp", toks.next(), false, true).await,
            Some("ski") => self.cmd_gain("ski", toks.next(), false, false).await,
            Some("zero") => {
                control::capture_electrical_offset();
                let _ = self.write_all(b"theta_e_off=").await;
                let _ = self.write_i32(control::theta_e_off_mrad()).await;
                let _ = self.write_all(b" mrad\r\n").await;
            }
            Some("pwm") => match parse_u8(toks.next()) {
                Some(pct) => {
                    control::set_pwm_pct(pct);
                    let _ = self.write_all(b"duty=").await;
                    let _ = self.write_i32(control::pwm_pct() as i32).await;
                    let _ = self.write_all(b"%\r\n").await;
                }
                None => {
                    let _ = self.write_all(b"usage: foc pwm <0-100>\r\n").await;
                }
            },
            _ => {
                let _ = self.write_all(b"usage: foc status|start|stop|align|id|iq|poles|pwm|offset|zero|openloop|rpm|kp|ki|skp|ski\r\n").await;
            }
        }
    }

    async fn cmd_gain(&mut self, name: &str, arg: Option<&str>, current: bool, is_kp: bool) {
        if let Some(v) = parse_f32(arg) {
            if current {
                let (kp, ki) = if is_kp {
                    (v, control::current_ki())
                } else {
                    (control::current_kp(), v)
                };
                control::set_current_gains(kp, ki);
            } else {
                let (kp, ki) = if is_kp {
                    (v, control::speed_ki())
                } else {
                    (control::speed_kp(), v)
                };
                control::set_speed_gains(kp, ki);
            }
        } else if arg.is_some() {
            let _ = self.write_all(b"usage: foc kp|ki|skp|ski [value]\r\n").await;
            return;
        }
        let _ = self.write_all(name.as_bytes()).await;
        let _ = self.write_all(b"=").await;
        let val = match name {
            "kp" => control::current_kp(),
            "ki" => control::current_ki(),
            "skp" => control::speed_kp(),
            _ => control::speed_ki(),
        };
        let _ = self.write_f32(val).await;
        let _ = self.write_all(b"\r\n").await;
    }

    async fn write_all(&mut self, buf: &[u8]) -> Result<(), ()> {
        self.tx.write(buf).await.map_err(|_| ())
    }

    async fn write_i32(&mut self, v: i32) -> Result<(), ()> {
        let mut buf = [0u8; 12];
        let n = fmt_i32(v, &mut buf);
        self.write_all(&buf[..n]).await
    }

    async fn write_f32(&mut self, v: f32) -> Result<(), ()> {
        let neg = v < 0.0;
        let v = if neg { -v } else { v };
        let ip = v as i32;
        let frac = ((v - ip as f32) * 1000.0) as i32;
        if neg {
            self.write_all(b"-").await?;
        }
        self.write_i32(ip).await?;
        self.write_all(b".").await?;
        let mut d = [b'0'; 3];
        let mut f = frac.clamp(0, 999);
        d[2] = b'0' + (f % 10) as u8;
        f /= 10;
        d[1] = b'0' + (f % 10) as u8;
        f /= 10;
        d[0] = b'0' + (f % 10) as u8;
        self.write_all(&d).await
    }
}

fn completion_words(line: &str) -> Option<(&str, &'static [&'static str])> {
    let ends_space = line.ends_with(' ');
    let mut parts = line.split_whitespace();
    let first = parts.next();
    let second = parts.next();
    let extra = parts.next();

    if extra.is_some() {
        return None;
    }

    match (first, second, ends_space) {
        (None, _, _) => Some(("", ROOT_CMDS)),
        (Some(a), None, false) => Some((a, ROOT_CMDS)),
        (Some("foc"), None, true) => Some(("", FOC_SUB)),
        (Some("foc"), Some(b), false) => Some((b, FOC_SUB)),
        (Some("led"), None, true) => Some(("", LED_SUB)),
        (Some("led"), Some(b), false) => Some((b, LED_SUB)),
        (Some("system"), None, true) => Some(("", SYSTEM_SUB)),
        (Some("system"), Some(b), false) => Some((b, SYSTEM_SUB)),
        _ => None,
    }
}

fn longest_common_prefix<'a>(words: &[&'a str]) -> &'a str {
    let first = words[0];
    let mut end = first.len();
    for w in words.iter().skip(1) {
        end = first
            .as_bytes()
            .iter()
            .zip(w.as_bytes())
            .take_while(|(a, b)| a == b)
            .count()
            .min(end);
    }
    &first[..end]
}

fn apply_completion(line: &mut [u8; LINE_CAP], len: &mut usize, prefix: &str, filled: &str) -> bool {
    if filled.len() < prefix.len() {
        return false;
    }
    let add = &filled.as_bytes()[prefix.len()..];
    if *len + add.len() > LINE_CAP {
        return false;
    }
    line[*len..*len + add.len()].copy_from_slice(add);
    *len += add.len();
    true
}

fn maybe_space_after(line: &mut [u8; LINE_CAP], len: &mut usize, word: &str) {
    if matches!(word, "foc" | "led" | "system" | "hello" | "echo" | "id" | "iq" | "poles" | "pwm")
        && *len < LINE_CAP
        && (*len == 0 || line[*len - 1] != b' ')
    {
        line[*len] = b' ';
        *len += 1;
    }
}

fn drop_crlf_lf(skip_lf: &mut bool, b: u8) -> bool {
    if *skip_lf {
        *skip_lf = false;
        return b == b'\n';
    }
    false
}

fn note_cr(skip_lf: &mut bool) {
    *skip_lf = true;
}

fn parse_i32(s: Option<&str>) -> Option<i32> {
    s.and_then(|t| t.parse().ok())
}

fn parse_u8(s: Option<&str>) -> Option<u8> {
    s.and_then(|t| t.parse().ok())
}

fn parse_f32(s: Option<&str>) -> Option<f32> {
    let s = s?;
    let (neg, rest) = if let Some(r) = s.strip_prefix('-') {
        (true, r)
    } else {
        (false, s.strip_prefix('+').unwrap_or(s))
    };
    if rest.is_empty() {
        return None;
    }
    let mut it = rest.splitn(2, '.');
    let ip = it.next()?;
    let fp = it.next().unwrap_or("");
    if ip.is_empty() && fp.is_empty() {
        return None;
    }
    let mut v = if ip.is_empty() {
        0.0
    } else {
        ip.parse::<u32>().ok()? as f32
    };
    if !fp.is_empty() {
        let frac = fp.parse::<u32>().ok()? as f32;
        let mut den = 1.0f32;
        for _ in 0..fp.len() {
            den *= 10.0;
        }
        v += frac / den;
    }
    Some(if neg { -v } else { v })
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crlf_submits_once() {
        let mut skip = false;
        assert!(!drop_crlf_lf(&mut skip, b'\r'));
        note_cr(&mut skip);
        assert!(drop_crlf_lf(&mut skip, b'\n'));
        assert!(!drop_crlf_lf(&mut skip, b'a'));
    }

    #[test]
    fn lf_only_is_not_dropped() {
        let mut skip = false;
        assert!(!drop_crlf_lf(&mut skip, b'\n'));
    }

    #[test]
    fn complete_root_prefix() {
        let (p, list) = completion_words("fo").unwrap();
        assert_eq!(p, "fo");
        assert!(list.contains(&"foc"));
    }

    #[test]
    fn complete_foc_sub() {
        let (p, list) = completion_words("foc s").unwrap();
        assert_eq!(p, "s");
        assert!(list.contains(&"start"));
        assert!(list.contains(&"status"));
        assert!(list.contains(&"stop"));
    }

    #[test]
    fn complete_after_space() {
        let (p, list) = completion_words("led ").unwrap();
        assert_eq!(p, "");
        assert_eq!(list, LED_SUB);
    }

    #[test]
    fn complete_third_token_none() {
        assert!(completion_words("foc id 100").is_none());
    }

    #[test]
    fn apply_and_space() {
        let mut line = [0u8; LINE_CAP];
        line[..2].copy_from_slice(b"fo");
        let mut len = 2;
        assert!(apply_completion(&mut line, &mut len, "fo", "foc"));
        maybe_space_after(&mut line, &mut len, "foc");
        assert_eq!(&line[..len], b"foc ");
    }

    #[test]
    fn lcp_help_hello() {
        assert_eq!(longest_common_prefix(&["help", "hello"]), "hel");
    }

    #[test]
    fn fmt_ints() {
        let mut buf = [0u8; 12];
        let n = fmt_i32(0, &mut buf);
        assert_eq!(&buf[..n], b"0");
        let n = fmt_i32(-42, &mut buf);
        assert_eq!(&buf[..n], b"-42");
        let n = fmt_i32(i32::MIN, &mut buf);
        assert_eq!(&buf[..n], b"-2147483648");
    }
}
