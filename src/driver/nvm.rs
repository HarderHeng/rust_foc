//! Last 2 KiB flash page: electrical offset and pole pairs (G431 page = 2 KiB).
//!
//! Erase stalls the CPU. Save only with PWM off (Idle). Never call from the current ISR.

use core::sync::atomic::{AtomicPtr, Ordering};

use embassy_stm32::flash::{Blocking, Flash};
use embassy_stm32::peripherals::FLASH;
use embassy_stm32::Peri;
use static_cell::StaticCell;

/// Offset from `0x0800_0000`. Last page of 128 KiB.
pub const PAGE_OFF: u32 = 0x1_F800;
pub const PAGE_END: u32 = 0x2_0000;
const ABS: u32 = 0x0800_0000 + PAGE_OFF;

const MAGIC: u32 = 0x3143_4F46; // "FOC1"
const VERSION: u16 = 1;
const SLOT: usize = 16;

#[derive(Clone, Copy, Debug)]
pub struct Record {
    pub theta_e_off_mrad: i32,
    pub poles: u8,
}

static SLOT_FLASH: StaticCell<Flash<'static, Blocking>> = StaticCell::new();
static FLASH_PTR: AtomicPtr<Flash<'static, Blocking>> = AtomicPtr::new(core::ptr::null_mut());

pub fn init(flash: Peri<'static, FLASH>) {
    let f = SLOT_FLASH.init(Flash::new_blocking(flash));
    FLASH_PTR.store(f as *mut _, Ordering::Release);
}

fn with_flash<R>(f: impl FnOnce(&mut Flash<'static, Blocking>) -> R) -> Option<R> {
    cortex_m::interrupt::free(|_| {
        let p = FLASH_PTR.load(Ordering::Acquire);
        if p.is_null() {
            None
        } else {
            Some(f(unsafe { &mut *p }))
        }
    })
}

fn checksum(body: &[u8; 12]) -> u32 {
    let mut c = 0xA5A5_5A5A;
    for chunk in body.chunks(4) {
        let mut w = [0u8; 4];
        w[..chunk.len()].copy_from_slice(chunk);
        c ^= u32::from_le_bytes(w);
        c = c.rotate_left(5).wrapping_add(0x9E37_79B9);
    }
    c
}

fn pack(r: Record) -> [u8; SLOT] {
    let mut body = [0u8; 12];
    body[0..4].copy_from_slice(&MAGIC.to_le_bytes());
    body[4..6].copy_from_slice(&VERSION.to_le_bytes());
    body[6] = r.poles.max(1);
    body[7] = 0;
    body[8..12].copy_from_slice(&r.theta_e_off_mrad.to_le_bytes());
    let mut out = [0u8; SLOT];
    out[..12].copy_from_slice(&body);
    out[12..16].copy_from_slice(&checksum(&body).to_le_bytes());
    out
}

fn unpack(raw: &[u8; SLOT]) -> Option<Record> {
    let mut body = [0u8; 12];
    body.copy_from_slice(&raw[..12]);
    let magic = u32::from_le_bytes(body[0..4].try_into().ok()?);
    let ver = u16::from_le_bytes(body[4..6].try_into().ok()?);
    let crc = u32::from_le_bytes(raw[12..16].try_into().ok()?);
    if magic != MAGIC || ver != VERSION || crc != checksum(&body) {
        return None;
    }
    Some(Record {
        poles: body[6].max(1),
        theta_e_off_mrad: i32::from_le_bytes(body[8..12].try_into().ok()?),
    })
}

/// Read the mapped page. No FLASH unlock.
pub fn load() -> Option<Record> {
    let mut raw = [0u8; SLOT];
    unsafe {
        core::ptr::copy_nonoverlapping(ABS as *const u8, raw.as_mut_ptr(), SLOT);
    }
    unpack(&raw)
}

pub fn save(r: Record) -> bool {
    let bytes = pack(r);
    let ok = with_flash(|f| {
        f.blocking_erase(PAGE_OFF, PAGE_END).is_ok() && f.blocking_write(PAGE_OFF, &bytes).is_ok()
    })
    .unwrap_or(false);
    if ok {
        defmt::info!("nvm saved off={} poles={}", r.theta_e_off_mrad, r.poles);
    } else {
        defmt::warn!("nvm save failed");
    }
    ok
}
