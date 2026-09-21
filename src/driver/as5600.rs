//! AS5600 12-bit magnetic angle on I2C1 (PB8 SCL, PB7 SDA).

use embassy_stm32::i2c::I2c;
use embassy_stm32::mode::Async;
use embassy_stm32::time::Hertz;
use embassy_time::Duration;

use crate::bsp::config::AS5600_I2C_ADDR;

const REG_STATUS: u8 = 0x0B;
const REG_ANGLE: u8 = 0x0E;
const STATUS_MD: u8 = 1 << 5;
const STATUS_ML: u8 = 1 << 4;
const STATUS_MH: u8 = 1 << 3;
const TWO_PI: f32 = core::f32::consts::TAU;

#[derive(Clone, Copy, Default)]
pub struct As5600Status {
    pub magnet_ok: bool,
    pub too_weak: bool,
    pub too_strong: bool,
}

#[derive(Clone, Copy, Default)]
pub struct AngleSample {
    pub raw: u16,
    pub theta_m: f32,
    pub omega_m: f32,
    pub valid: bool,
    pub status: As5600Status,
}

pub struct As5600 {
    i2c: I2c<'static, Async, embassy_stm32::i2c::Master>,
    last_raw: Option<u16>,
    unwrap: i32,
    last_theta: f32,
}

impl As5600 {
    pub fn new(i2c: I2c<'static, Async, embassy_stm32::i2c::Master>) -> Self {
        Self {
            i2c,
            last_raw: None,
            unwrap: 0,
            last_theta: 0.0,
        }
    }

    pub async fn read(&mut self, dt: f32) -> AngleSample {
        let mut status_buf = [0u8; 1];
        let mut angle_buf = [0u8; 2];

        if self
            .i2c
            .write_read(AS5600_I2C_ADDR, &[REG_STATUS], &mut status_buf)
            .await
            .is_err()
        {
            return AngleSample {
                valid: false,
                ..Default::default()
            };
        }
        let st = status_buf[0];
        let status = As5600Status {
            magnet_ok: st & STATUS_MD != 0,
            too_weak: st & STATUS_ML != 0,
            too_strong: st & STATUS_MH != 0,
        };

        if self
            .i2c
            .write_read(AS5600_I2C_ADDR, &[REG_ANGLE], &mut angle_buf)
            .await
            .is_err()
        {
            return AngleSample {
                valid: false,
                status,
                ..Default::default()
            };
        }

        let raw = (((angle_buf[0] as u16) << 8) | angle_buf[1] as u16) & 0x0FFF;
        if let Some(prev) = self.last_raw {
            let mut d = raw as i32 - prev as i32;
            if d > 2048 {
                d -= 4096;
            } else if d < -2048 {
                d += 4096;
            }
            self.unwrap += d;
        } else {
            self.unwrap = raw as i32;
        }
        self.last_raw = Some(raw);

        let theta_m = (self.unwrap as f32) * (TWO_PI / 4096.0);
        let omega_m = if dt > 0.0 {
            (theta_m - self.last_theta) / dt
        } else {
            0.0
        };
        self.last_theta = theta_m;

        AngleSample {
            raw,
            theta_m,
            omega_m,
            valid: status.magnet_ok && !status.too_weak,
            status,
        }
    }
}

pub fn i2c_config() -> embassy_stm32::i2c::Config {
    let mut cfg = embassy_stm32::i2c::Config::default();
    cfg.frequency = Hertz::hz(crate::bsp::config::AS5600_I2C_HZ);
    cfg.timeout = Duration::from_millis(5);
    cfg
}
