//! Version 3 calibration records in a single-page append journal.
//!
//! A torn append leaves preceding valid records readable. Reclaiming a full
//! page erases every record before writing the replacement, so power loss
//! during reclaim can lose calibration. This is not an atomic two-page store.

pub const SLOT_SIZE: usize = 16;
pub const SLOT_COUNT: usize = 128;
pub const PAGE_SIZE: usize = SLOT_SIZE * SLOT_COUNT;
const MAGIC: u32 = 0x3143_4F46;
// Version 1 may contain unqualified automatic alignment offsets. Version 2
// cannot distinguish qualified offsets from unverified `foc zero` captures.
// Both require recalibration; keep their occupied slots consumed in the journal.
const VERSION: u16 = 3;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Record {
    pub theta_e_off_mrad: i32,
    pub poles: u8,
}

impl Record {
    pub fn encode(self) -> Option<[u8; SLOT_SIZE]> {
        if self.poles == 0 || !(0..=6283).contains(&self.theta_e_off_mrad) {
            return None;
        }
        let mut raw = [0u8; SLOT_SIZE];
        raw[..4].copy_from_slice(&MAGIC.to_le_bytes());
        raw[4..6].copy_from_slice(&VERSION.to_le_bytes());
        raw[6] = self.poles;
        raw[8..12].copy_from_slice(&self.theta_e_off_mrad.to_le_bytes());
        let checksum = checksum(&raw[..12]);
        raw[12..].copy_from_slice(&checksum.to_le_bytes());
        Some(raw)
    }

    pub fn decode(raw: &[u8; SLOT_SIZE]) -> Option<Self> {
        if u32::from_le_bytes(raw[..4].try_into().ok()?) != MAGIC
            || u16::from_le_bytes(raw[4..6].try_into().ok()?) != VERSION
            || raw[7] != 0
            || u32::from_le_bytes(raw[12..].try_into().ok()?) != checksum(&raw[..12])
        {
            return None;
        }
        let record = Self {
            poles: raw[6],
            theta_e_off_mrad: i32::from_le_bytes(raw[8..12].try_into().ok()?),
        };
        record.encode()?;
        Some(record)
    }
}

fn checksum(body: &[u8]) -> u32 {
    let mut checksum = 0xA5A5_5A5A;
    for chunk in body.as_chunks::<4>().0 {
        let word = u32::from_le_bytes(*chunk);
        checksum ^= word;
        checksum = checksum.rotate_left(5).wrapping_add(0x9E37_79B9);
    }
    checksum
}

/// Implementations address exactly one 2 KiB page with 16-byte record slots.
pub trait Storage {
    fn read_slot(&mut self, index: usize) -> [u8; SLOT_SIZE];
    fn erase(&mut self) -> bool;
    fn write_slot(&mut self, index: usize, raw: &[u8; SLOT_SIZE]) -> bool;
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SaveOutcome {
    Unchanged,
    Appended,
    Reclaimed,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SaveError {
    InvalidRecord,
    Erase,
    Write,
    Verify,
}

fn scan(storage: &mut impl Storage) -> (Option<Record>, usize) {
    let mut latest = None;
    let mut next = 0;
    for index in 0..SLOT_COUNT {
        let raw = storage.read_slot(index);
        if raw != [0xFF; SLOT_SIZE] {
            // A torn/invalid slot is consumed; never program it a second time.
            next = index + 1;
        }
        if let Some(record) = Record::decode(&raw) {
            latest = Some(record);
        }
    }
    (latest, next)
}

pub fn load(storage: &mut impl Storage) -> Option<Record> {
    scan(storage).0
}

pub fn save(storage: &mut impl Storage, record: Record) -> Result<SaveOutcome, SaveError> {
    let raw = record.encode().ok_or(SaveError::InvalidRecord)?;
    let (latest, next) = scan(storage);
    if latest == Some(record) {
        return Ok(SaveOutcome::Unchanged);
    }
    let (index, outcome) = if next == SLOT_COUNT {
        if !storage.erase() {
            return Err(SaveError::Erase);
        }
        if (0..SLOT_COUNT).any(|index| storage.read_slot(index) != [0xFF; SLOT_SIZE]) {
            return Err(SaveError::Verify);
        }
        (0, SaveOutcome::Reclaimed)
    } else {
        (next, SaveOutcome::Appended)
    };
    if !storage.write_slot(index, &raw) {
        return Err(SaveError::Write);
    }
    if storage.read_slot(index) != raw {
        return Err(SaveError::Verify);
    }
    Ok(outcome)
}
