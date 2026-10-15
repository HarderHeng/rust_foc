"""Exercise ELF validation with a small independent ELF32 fixture."""
import struct
import subprocess
from pathlib import Path
import tempfile
import unittest

from artifact import command_output, inspect_elf, regions_from_script, source_inputs

REGIONS = {"FLASH": (0x08000000, 126 * 1024), "NVM": (0x0801F800, 2048),
           "RAM": (0x20000000, 32 * 1024)}
PHOFF = 52
SHOFF = 512


def elf_fixture():
    blob = bytearray(SHOFF + 7 * 40)
    blob[:16] = b"\x7fELF\x01\x01\x01" + bytes(9)
    struct.pack_into("<HHIIIIIHHHHHH", blob, 16,
                     2, 40, 1, 0x08000009, PHOFF, SHOFF, 0x05000000, 52, 32, 3, 40, 7, 6)
    # Vectors + .text, initialized RAM (.data), and zero/uninitialized RAM.
    for index, fields in enumerate([
        (1, 256, 0x08000000, 0x08000000, 16, 16, 5, 4),
        (1, 272, 0x20007FF4, 0x08000010, 4, 4, 6, 4),
        (1, 276, 0x20007FF8, 0x20007FF8, 0, 8, 6, 4),
    ]):
        struct.pack_into("<IIIIIIII", blob, PHOFF + index * 32, *fields)
    struct.pack_into("<II", blob, 256, 0x20007FF0, 0x08000009)
    blob[264:272] = b"abcdefgh"
    blob[272:276] = b"data"
    names = b"\0.vector_table\0.text\0.data\0.bss\0.uninit\0.shstrtab\0"
    blob[300:300 + len(names)] = names
    for index, fields in enumerate([
        (0, 0, 0, 0, 0, 0),
        (names.index(b".vector_table"), 1, 2, 0x08000000, 256, 8),
        (names.index(b".text"), 1, 6, 0x08000008, 264, 8),
        (names.index(b".data"), 1, 3, 0x20007FF4, 272, 4),
        (names.index(b".bss"), 8, 3, 0x20007FF8, 276, 4),
        (names.index(b".uninit"), 8, 3, 0x20007FFC, 276, 4),
        (names.index(b".shstrtab"), 3, 0, 0, 300, len(names)),
    ]):
        struct.pack_into("<IIIIIIIIII", blob, SHOFF + index * 40, *fields, 0, 0, 4, 0)
    return blob


class ArtifactTests(unittest.TestCase):
    def test_sizes_account_for_flash_copy_of_initialized_ram(self):
        result = inspect_elf(elf_fixture(), REGIONS)
        self.assertEqual(result["flash"]["bytes_used"], 20)
        self.assertEqual(result["ram"]["static_bytes"], 12)
        self.assertEqual(result["ram"]["stack_budget_bytes"], 32752)
        self.assertEqual(result["sections"][".data"]["bytes"], 4)

    def test_memory_script_regions_and_overlap(self):
        script = "FLASH : ORIGIN = 0x08000000, LENGTH = 126K\nNVM : ORIGIN = 0x0801F800, LENGTH = 2K\nRAM : ORIGIN = 0x20000000, LENGTH = 32K"
        self.assertEqual(regions_from_script(script), REGIONS)
        with self.assertRaisesRegex(ValueError, "overlaps"):
            regions_from_script(script.replace("126K", "128K"))
        with self.assertRaisesRegex(ValueError, "must define"):
            regions_from_script("")

    def test_rejects_nvm_programming_and_overlapping_loads(self):
        for physical, message in [(0x0801F800, "escapes"), (0x08000004, "overlapping")]:
            blob = elf_fixture()
            struct.pack_into("<I", blob, PHOFF + 32 + 12, physical)
            with self.assertRaisesRegex(ValueError, message):
                inspect_elf(blob, REGIONS)

    def test_rejects_allocated_section_outside_ram(self):
        blob = elf_fixture()
        struct.pack_into("<I", blob, SHOFF + 4 * 40 + 12, 0x20008000)
        with self.assertRaisesRegex(ValueError, "allocated section .bss escapes"):
            inspect_elf(blob, REGIONS)

    def test_rejects_static_ram_in_stack_region(self):
        blob = elf_fixture()
        struct.pack_into("<I", blob, 256, 0x20007FF8)
        with self.assertRaisesRegex(ValueError, "overlaps the flip-link stack"):
            inspect_elf(blob, REGIONS)

    def test_rejects_flash_load_at_wrong_physical_address(self):
        blob = elf_fixture()
        struct.pack_into("<I", blob, PHOFF + 12, 0x08001000)
        with self.assertRaisesRegex(ValueError, "FLASH LOAD physical address differs"):
            inspect_elf(blob, REGIONS)

    def test_rejects_sections_with_wrong_load_offset_or_no_load(self):
        for field_offset, value in [(16, 265), (12, 0x08000100)]:
            blob = elf_fixture()
            struct.pack_into("<I", blob, SHOFF + 2 * 40 + field_offset, value)
            with self.assertRaisesRegex(ValueError, "section .text has no matching LOAD mapping"):
                inspect_elf(blob, REGIONS)

    def test_rejects_bss_in_file_backed_memory(self):
        blob = elf_fixture()
        struct.pack_into("<I", blob, SHOFF + 4 * 40 + 12, 0x20007FF4)
        with self.assertRaisesRegex(ValueError, "section .bss has no matching LOAD mapping"):
            inspect_elf(blob, REGIONS)

    def test_rejects_wrong_entry_reset_and_stack(self):
        for offset, value, message in [
            (24, 0x0801F801, "entry point"),
            (260, 0x08000008, "reset vector"),
            (256, 0x20008004, "stack pointer"),
            (256, 0x20007FF1, "stack pointer"),
        ]:
            blob = elf_fixture()
            struct.pack_into("<I", blob, offset, value)
            with self.assertRaisesRegex(ValueError, message):
                inspect_elf(blob, REGIONS)

    def test_rejects_wrong_architecture_and_truncated_files(self):
        blob = elf_fixture()
        struct.pack_into("<H", blob, 18, 62)  # x86_64, not ARM
        with self.assertRaisesRegex(ValueError, "ARM executable"):
            inspect_elf(blob, REGIONS)
        for length in [0, 51, PHOFF + 31, SHOFF + 7 * 40 - 1]:
            with self.assertRaisesRegex(ValueError, "truncated"):
                inspect_elf(elf_fixture()[:length], REGIONS)

    def test_rejects_truncated_payload_and_invalid_section_names(self):
        blob = elf_fixture()
        struct.pack_into("<I", blob, PHOFF + 32 + 4, len(blob))
        with self.assertRaisesRegex(ValueError, "truncated ELF payload"):
            inspect_elf(blob, REGIONS)
        blob = elf_fixture()
        struct.pack_into("<I", blob, SHOFF + 2 * 40, 10000)
        with self.assertRaisesRegex(ValueError, "section name offset"):
            inspect_elf(blob, REGIONS)


class SourceInputTests(unittest.TestCase):
    def test_source_fingerprint_covers_dirty_tracked_and_untracked_inputs(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            subprocess.run(["git", "init", "--quiet", str(root)], check=True)
            (root / "src").mkdir()
            (root / "Cargo.toml").write_text("manifest\n")
            (root / "src/main.rs").write_text("tracked\n")
            (root / ".gitignore").write_text("/target\n/src/ignored.rs\n")
            subprocess.run(["git", "add", "Cargo.toml", "src/main.rs", ".gitignore"], cwd=root, check=True)
            first = source_inputs(root)
            self.assertEqual(source_inputs(root), first)
            (root / "src/main.rs").write_text("dirty\n")
            dirty = source_inputs(root)
            self.assertNotEqual(dirty["sha256"], first["sha256"])
            (root / "src/new.rs").write_text("new untracked input\n")
            new = source_inputs(root)
            self.assertIn("src/new.rs", new["files"])
            self.assertNotEqual(new["sha256"], dirty["sha256"])
            (root / "src/ignored.rs").write_text("ignored\n")
            (root / "target").mkdir()
            (root / "target/main").write_text("artifact\n")
            self.assertEqual(source_inputs(root), new)
            (root / "src/new.rs").rename(root / "src/renamed.rs")
            renamed = source_inputs(root)
            self.assertNotEqual(renamed["sha256"], new["sha256"])
            (root / "src/main.rs").unlink()
            deleted = source_inputs(root)
            self.assertIsNone(deleted["files"]["src/main.rs"])
            self.assertNotEqual(deleted["sha256"], renamed["sha256"])

    def test_command_output_preserves_status_prefix(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            subprocess.run(["git", "init", "--quiet", str(root)], check=True)
            (root / "file").write_text("tracked")
            subprocess.run(["git", "add", "file"], cwd=root, check=True)
            # Intent-to-add produces the leading-space status that strip loses.
            (root / "new").write_text("new")
            subprocess.run(["git", "add", "--intent-to-add", "new"], cwd=root, check=True)
            status = command_output(["git", "status", "--porcelain", "--", "new"], root)
            self.assertTrue(status.startswith(" A "), status)

if __name__ == "__main__":
    unittest.main()
