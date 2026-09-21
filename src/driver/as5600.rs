//! AS5600 12-bit magnetic angle on I2C1 (PB8 SCL / PB7 SDA, J8 Z+ / B+).

use embassy_stm32::i2c::I2c;
use embassy_stm32::mode::Async;
use embassy_stm32::time::Hertz;
use embassy_time::Duration;

use crate::bsp::config::AS5600_I2C_ADDR;
use crate::foc::AngleTrack;

const REG_STATUS: u8 = 0x0B;
const REG_ANGLE: u8 = 0x0E;
const STATUS_MD: u8 = 1 << 5;
const STATUS_ML: u8 = 1 << 4;
const STATUS_MH: u8 = 1 << 3;

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
    track: AngleTrack,
    last_raw: u16,
}

impl As5600 {
    pub fn new(i2c: I2c<'static, Async, embassy_stm32::i2c::Master>) -> Self {
        Self {
            i2c,
            track: AngleTrack::new(),
            last_raw: 0,
        }
    }

    pub async fn read(&mut self, dt: f32) -> AngleSample {
        let mut status_buf = [0u8; 1];
        if self
            .i2c
            .write_read(AS5600_I2C_ADDR, &[REG_STATUS], &mut status_buf)
            .await
            .is_err()
        {
            return self.hold(As5600Status::default());
        }
        let st = status_buf[0];
        let status = As5600Status {
            magnet_ok: st & STATUS_MD != 0,
            too_weak: st & STATUS_ML != 0,
            too_strong: st & STATUS_MH != 0,
        };

        // ANGLE is a 16-bit special register: pointer at 0x0E, then 2 bytes (11:8, 7:0).
        // Do not burst from STATUS — after 0x0C the pointer stops incrementing.
        let mut angle_buf = [0u8; 2];
        if self
            .i2c
            .write_read(AS5600_I2C_ADDR, &[REG_ANGLE], &mut angle_buf)
            .await
            .is_err()
        {
            return self.hold(status);
        }
        if !status.magnet_ok || status.too_weak {
            return self.hold(status);
        }

        let raw = (u16::from(angle_buf[0]) << 8 | u16::from(angle_buf[1])) & 0x0FFF;
        self.last_raw = raw;
        let (theta_m, omega_m) = self.track.push(raw, dt);
        AngleSample {
            raw,
            theta_m,
            omega_m,
            valid: true,
            status,
        }
    }

    fn hold(&self, status: As5600Status) -> AngleSample {
        let (theta_m, omega_m) = self.track.last();
        AngleSample {
            raw: self.last_raw,
            theta_m,
            omega_m,
            valid: false,
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
