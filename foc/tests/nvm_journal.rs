use foc::nvm::{load, save, Record, SaveError, SaveOutcome, Storage, SLOT_COUNT, SLOT_SIZE};

struct Page {
    slots: [[u8; SLOT_SIZE]; SLOT_COUNT],
    erases: usize,
    writes: Vec<usize>,
    erase_ok: bool,
    erase_bytes: bool,
    write_ok: bool,
    write_bytes: usize,
}

impl Default for Page {
    fn default() -> Self {
        Self {
            slots: [[0xFF; SLOT_SIZE]; SLOT_COUNT],
            erases: 0,
            writes: Vec::new(),
            erase_ok: true,
            erase_bytes: true,
            write_ok: true,
            write_bytes: SLOT_SIZE,
        }
    }
}

impl Storage for Page {
    fn read_slot(&mut self, index: usize) -> [u8; SLOT_SIZE] {
        self.slots[index]
    }

    fn erase(&mut self) -> bool {
        self.erases += 1;
        if self.erase_ok && self.erase_bytes {
            self.slots.fill([0xFF; SLOT_SIZE]);
        }
        self.erase_ok
    }

    fn write_slot(&mut self, index: usize, raw: &[u8; SLOT_SIZE]) -> bool {
        assert_eq!(
            self.slots[index], [0xFF; SLOT_SIZE],
            "reprogrammed a used slot"
        );
        self.writes.push(index);
        self.slots[index][..self.write_bytes].copy_from_slice(&raw[..self.write_bytes]);
        self.write_ok
    }
}

fn record(offset: i32) -> Record {
    Record {
        theta_e_off_mrad: offset,
        poles: 7,
    }
}

// Independent copy of the existing on-flash v2 format, including its checksum.
fn legacy_bytes(version: u16, poles: u8, offset: i32, reserved: u8) -> [u8; SLOT_SIZE] {
    let mut raw = [0u8; SLOT_SIZE];
    raw[..4].copy_from_slice(b"FOC1");
    raw[4..6].copy_from_slice(&version.to_le_bytes());
    raw[6] = poles;
    raw[7] = reserved;
    raw[8..12].copy_from_slice(&offset.to_le_bytes());
    let mut sum = 0xA5A5_5A5A_u32;
    for offset in [0, 4, 8] {
        sum ^= u32::from_le_bytes(raw[offset..offset + 4].try_into().unwrap());
        sum = sum.rotate_left(5).wrapping_add(0x9E37_79B9);
    }
    raw[12..].copy_from_slice(&sum.to_le_bytes());
    raw
}

#[test]
fn accepts_existing_v2_first_slot_and_valid_boundary_values() {
    for poles in [1, 7, 255] {
        for offset in [0, 1234, 6283] {
            let expected = Record {
                poles,
                theta_e_off_mrad: offset,
            };
            let raw = legacy_bytes(2, poles, offset, 0);
            assert_eq!(expected.encode(), Some(raw));
            let mut page = Page::default();
            page.slots[0] = raw;
            assert_eq!(load(&mut page), Some(expected));
        }
    }
}

#[test]
fn rejects_invalid_fields_even_with_valid_checksum() {
    for raw in [
        legacy_bytes(1, 7, 100, 0),
        legacy_bytes(3, 7, 100, 0),
        legacy_bytes(2, 0, 100, 0),
        legacy_bytes(2, 7, -1, 0),
        legacy_bytes(2, 7, 6284, 0),
        legacy_bytes(2, 7, i32::MAX, 0),
        legacy_bytes(2, 7, 100, 1),
    ] {
        assert_eq!(Record::decode(&raw), None);
    }
    let mut page = Page::default();
    for invalid in [
        record(-1),
        record(6284),
        Record {
            poles: 0,
            ..record(0)
        },
    ] {
        assert_eq!(save(&mut page, invalid), Err(SaveError::InvalidRecord));
    }
    assert_eq!((page.erases, page.writes.len()), (0, 0));
}

#[test]
fn rejects_every_single_bit_corruption_and_blank_slots() {
    let raw = record(1234).encode().unwrap();
    for bit in 0..SLOT_SIZE * 8 {
        let mut corrupt = raw;
        corrupt[bit / 8] ^= 1 << (bit % 8);
        assert_eq!(Record::decode(&corrupt), None, "bit {bit}");
    }
    assert_eq!(Record::decode(&[0xFF; SLOT_SIZE]), None);
    assert_eq!(Record::decode(&[0; SLOT_SIZE]), None);
    assert_eq!(load(&mut Page::default()), None);
}

#[test]
fn successive_saves_append_and_identical_saves_do_no_flash_work() {
    let mut page = Page::default();
    for offset in 0..SLOT_COUNT as i32 {
        assert_eq!(save(&mut page, record(offset)), Ok(SaveOutcome::Appended));
        assert_eq!(save(&mut page, record(offset)), Ok(SaveOutcome::Unchanged));
        assert_eq!(load(&mut page), Some(record(offset)));
    }
    assert_eq!(page.erases, 0);
    assert_eq!(page.writes, (0..SLOT_COUNT).collect::<Vec<_>>());
    // Even a full page need not be erased to save the current record again.
    assert_eq!(save(&mut page, record(127)), Ok(SaveOutcome::Unchanged));
    assert_eq!((page.erases, page.writes.len()), (0, SLOT_COUNT));
}

#[test]
fn every_torn_append_preserves_prior_calibration_and_is_never_reprogrammed() {
    let next = record(3456).encode().unwrap();
    for written in 0..SLOT_SIZE {
        let mut page = Page::default();
        page.slots[0] = record(1234).encode().unwrap();
        page.slots[1][..written].copy_from_slice(&next[..written]);
        assert_eq!(
            load(&mut page),
            Some(record(1234)),
            "torn at byte {written}"
        );
        assert_eq!(save(&mut page, record(4567)), Ok(SaveOutcome::Appended));
        assert_eq!(page.writes, [if written == 0 { 1 } else { 2 }]);
        assert_eq!(load(&mut page), Some(record(4567)));
        assert_eq!(page.erases, 0);
    }
}

#[test]
fn appends_after_last_consumed_slot_including_corrupt_gaps() {
    let mut page = Page::default();
    page.slots[0] = record(100).encode().unwrap();
    page.slots[10] = [0; SLOT_SIZE];
    assert_eq!(save(&mut page, record(200)), Ok(SaveOutcome::Appended));
    assert_eq!(page.writes, [11]);
    assert_eq!(load(&mut page), Some(record(200)));
    page.slots[11][0] ^= 1;
    assert_eq!(load(&mut page), Some(record(100)));
}

#[test]
fn full_page_reclaims_once_and_reports_erase_and_write_failures() {
    let mut page = Page::default();
    for offset in 0..SLOT_COUNT as i32 {
        save(&mut page, record(offset)).unwrap();
    }
    page.erase_ok = false;
    assert_eq!(save(&mut page, record(500)), Err(SaveError::Erase));
    assert_eq!(load(&mut page), Some(record(127)));
    assert_eq!(page.writes.len(), SLOT_COUNT);
    page.erase_ok = true;
    assert_eq!(save(&mut page, record(500)), Ok(SaveOutcome::Reclaimed));
    assert_eq!(load(&mut page), Some(record(500)));
    assert_eq!(page.erases, 2); // One failed attempt and one successful reclaim.
    assert_eq!(page.writes.last(), Some(&0));
    assert!(page.slots[1..].iter().all(|raw| *raw == [0xFF; SLOT_SIZE]));
}

#[test]
fn write_error_and_false_success_are_detected_by_readback() {
    for write_ok in [false, true] {
        let mut page = Page::default();
        save(&mut page, record(100)).unwrap();
        page.write_bytes = 8;
        page.write_ok = write_ok;
        assert_eq!(
            save(&mut page, record(200)),
            Err(if write_ok {
                SaveError::Verify
            } else {
                SaveError::Write
            })
        );
        assert_eq!(load(&mut page), Some(record(100)));
        page.write_bytes = SLOT_SIZE;
        page.write_ok = true;
        assert_eq!(save(&mut page, record(300)), Ok(SaveOutcome::Appended));
        assert_eq!(page.writes, [0, 1, 2]);
        assert_eq!(load(&mut page), Some(record(300)));
    }
}

#[test]
fn power_loss_after_full_page_erase_can_lose_previous_calibration() {
    let mut page = Page::default();
    page.slots[0] = record(100).encode().unwrap();
    page.slots[SLOT_COUNT - 1] = [0; SLOT_SIZE];
    page.write_bytes = 8;
    page.write_ok = false;
    assert_eq!(save(&mut page, record(200)), Err(SaveError::Write));
    assert_eq!(load(&mut page), None, "single-page reclaim is not atomic");
    assert_eq!(page.erases, 1);
}

#[test]
fn false_erase_success_cannot_hide_new_record_behind_stale_slots() {
    let mut page = Page::default();
    page.slots[SLOT_COUNT - 1] = record(100).encode().unwrap();
    page.erase_bytes = false;
    assert_eq!(save(&mut page, record(200)), Err(SaveError::Verify));
    assert_eq!(load(&mut page), Some(record(100)));
    assert!(page.writes.is_empty());
}
