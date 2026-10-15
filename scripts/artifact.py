#!/usr/bin/env python3
"""Validate the linked STM32G431 ELF and write its software build manifest."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import struct
import subprocess
import sys


def require(condition, message):
    if not condition:
        raise ValueError(message)


def regions_from_script(script):
    regions = {}
    for name, address, length, unit in re.findall(
        r"\b(FLASH|RAM|NVM)\s*:\s*ORIGIN\s*=\s*(0x[0-9a-fA-F]+)\s*,\s*LENGTH\s*=\s*(\d+)\s*(K?)",
        script,
    ):
        regions[name] = (int(address, 16), int(length) * (1024 if unit else 1))
    require(set(regions) == {"FLASH", "RAM", "NVM"}, "memory.x must define FLASH, RAM and NVM")
    for name, (base, size) in regions.items():
        require(size > 0, f"{name} region is empty")
        for other, (other_base, other_size) in regions.items():
            if name < other:
                require(base + size <= other_base or other_base + other_size <= base,
                        f"{name} overlaps {other}")
    return regions


def inspect_elf(blob, regions):
    require(len(blob) >= 52, "truncated ELF header")
    require(blob[:7] == b"\x7fELF\x01\x01\x01", "expected ELF32 little-endian version 1")
    header = struct.unpack_from("<HHIIIIIHHHHHH", blob, 16)
    elf_type, machine, version, entry, phoff, shoff, _, ehsize, phsize, phcount, shsize, shcount, names_index = header
    require((elf_type, machine, version, ehsize) == (2, 40, 1, 52), "expected ARM executable ELF")
    require(phsize == 32 and shsize == 40 and phcount > 0 and shcount > 0, "invalid ELF table geometry")
    require(phoff + phsize * phcount <= len(blob) and shoff + shsize * shcount <= len(blob),
            "truncated ELF tables")
    require(names_index < shcount, "invalid section name table")
    flash_base, flash_size = regions["FLASH"]
    ram_base, ram_size = regions["RAM"]

    def fits(address, size, region):
        base, capacity = regions[region]
        return size >= 0 and base <= address and address + size <= base + capacity

    def file_bytes(offset, size):
        require(0 <= offset <= len(blob) and offset + size <= len(blob), "truncated ELF payload")
        return blob[offset:offset + size]

    flash_ranges = []
    executable_ranges = []
    loads = []
    image_bytes = 0
    for index in range(phcount):
        kind, offset, address, physical, file_size, memory_size, flags, _ = struct.unpack_from(
            "<IIIIIIII", blob, phoff + index * phsize)
        if kind != 1:
            continue
        require(file_size <= memory_size, "LOAD file size exceeds memory size")
        file_bytes(offset, file_size)
        if memory_size:
            require(fits(address, memory_size, "FLASH") or fits(address, memory_size, "RAM"),
                    "LOAD runtime address escapes firmware FLASH/RAM or overlaps NVM")
            if fits(address, memory_size, "FLASH"):
                require(physical == address, "FLASH LOAD physical address differs from runtime address")
        if file_size:
            require(fits(physical, file_size, "FLASH"), "LOAD image escapes firmware FLASH or overlaps NVM")
            flash_ranges.append((physical, physical + file_size))
            image_bytes += file_size
        if flags & 1 and file_size:
            require(fits(address, memory_size, "FLASH"), "executable LOAD outside FLASH")
            executable_ranges.append((address, address + file_size))
        loads.append((offset, address, file_size, memory_size, flags))
    require(flash_ranges, "ELF has no flash image")
    flash_ranges.sort()
    require(all(left[1] <= right[0] for left, right in zip(flash_ranges, flash_ranges[1:])),
            "overlapping flash LOAD segments")
    require(any(start <= entry & ~1 < end for start, end in executable_ranges), "entry point outside executable FLASH")

    raw_sections = [struct.unpack_from("<IIIIIIIIII", blob, shoff + index * shsize)
                    for index in range(shcount)]
    names = raw_sections[names_index]
    name_bytes = file_bytes(names[4], names[5])
    sections = {}
    ram_sections = []
    for raw in raw_sections:
        name_offset, kind, flags, address, offset, size, _, _, _, _ = raw
        require(name_offset < len(name_bytes), "invalid section name offset")
        end = name_bytes.find(b"\0", name_offset)
        require(end >= 0, "unterminated section name")
        name = name_bytes[name_offset:end].decode("ascii")
        if kind != 8:
            file_bytes(offset, size)
        if flags & 2 and size:
            require(fits(address, size, "FLASH") or fits(address, size, "RAM"),
                    f"allocated section {name} escapes FLASH/RAM or overlaps NVM")
            mapped = False
            for load_offset, load_address, load_file_size, load_memory_size, load_flags in loads:
                delta = address - load_address
                if delta < 0 or delta + size > load_memory_size:
                    continue
                if flags & 4 and not load_flags & 1 or flags & 1 and not load_flags & 2:
                    continue
                if kind == 8:
                    mapped = delta >= load_file_size
                else:
                    mapped = offset == load_offset + delta and delta + size <= load_file_size
                if mapped:
                    break
            require(mapped, f"allocated section {name} has no matching LOAD mapping")
            if fits(address, size, "RAM"):
                ram_sections.append((address, size))
        sections[name] = {"address": address, "bytes": size, "kind": kind, "flags": flags, "offset": offset}
    for name, region, kind in [(".text", "FLASH", 1), (".data", "RAM", 1), (".bss", "RAM", 8)]:
        require(name in sections, f"missing {name}")
        section = sections[name]
        require(section["kind"] == kind and section["flags"] & 2 and fits(section["address"], section["bytes"], region),
                f"invalid {name} placement/type")
    require(sections[".text"]["flags"] & 4, ".text is not executable")
    vector = sections.get(".vector_table")
    require(vector is not None and vector["address"] == flash_base and vector["bytes"] >= 8
            and vector["kind"] == 1 and vector["flags"] & 2,
            "missing vector table at FLASH origin")
    stack, reset = struct.unpack_from("<II", blob, vector["offset"])
    require(ram_base < stack <= ram_base + ram_size and stack % 8 == 0, "initial stack pointer outside aligned RAM")
    require(reset == entry and reset & 1, "reset vector must equal the Thumb ELF entry point")
    flash_used = max(end for _, end in flash_ranges) - flash_base
    require(all(address >= stack for address, size in ram_sections),
            "static RAM overlaps the flip-link stack region")
    return {
        "entry_point": entry,
        "sections": {name: {key: sections[name][key] for key in ("address", "bytes")}
                     for name in (".vector_table", ".text", ".rodata", ".data", ".bss", ".uninit")
                     if name in sections},
        "flash": {"bytes_used": flash_used, "bytes_in_image": image_bytes,
                  "capacity_bytes": flash_size, "free_bytes": flash_size - flash_used},
        "ram": {"static_bytes": sum(size for _, size in ram_sections),
                "capacity_bytes": ram_size, "stack_budget_bytes": stack - ram_base},
        "nvm": {"address": regions["NVM"][0], "reserved_bytes": regions["NVM"][1]},
    }


def command_output(command, root):
    return subprocess.check_output(command, cwd=root, text=True, timeout=10).rstrip("\n")


def source_inputs(root):
    """Hash tracked and nonignored new build/check inputs, including their paths."""
    listed = subprocess.check_output(["git", "ls-files", "--cached", "--others", "--exclude-standard", "-z"],
                                     cwd=root, timeout=10)
    explicit = {"Cargo.toml", "Cargo.lock", "build.rs", "memory.x", "rust-toolchain", "rust-toolchain.toml",
                ".cargo/config", ".cargo/config.toml", ".github/workflows/check.yml"}
    files = {}
    tree_hash = hashlib.sha256()
    for raw_name in sorted(set(listed.split(b"\0")) - {b""}):
        name = os.fsdecode(raw_name)
        parts = Path(name).parts
        if "target" in parts or not (name in explicit or parts[0] in {"src", "foc", "scripts"}):
            continue
        path = root / name
        digest = hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None
        files[name] = digest
        tree_hash.update(len(raw_name).to_bytes(8, "little"))
        tree_hash.update(raw_name)
        tree_hash.update(bytes.fromhex(digest) if digest else b"missing")
    return {"sha256": tree_hash.hexdigest(), "files": files}


def linker_metadata(root):
    # flip-link forwards --version to its native linker; it has no version flag.
    executable = shutil.which("flip-link")
    require(executable is not None, "flip-link is not on PATH")
    path = Path(executable).resolve()
    installed = command_output(["cargo", "install", "--list", "--root", str(path.parent.parent)], root)
    return {"path": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            "package": next((line.rstrip(":") for line in installed.splitlines()
                             if line.startswith("flip-link v")), None)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--elf", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    root = Path(__file__).resolve().parent.parent
    try:
        blob = args.elf.read_bytes()
        memory = (root / "memory.x").read_text()
        manifest = inspect_elf(blob, regions_from_script(memory))
        manifest.update({
            "schema_version": 1,
            "created_at_utc": datetime.now(timezone.utc).isoformat(),
            "target": "thumbv7em-none-eabihf",
            "profile": "release",
            "elf": {"file": args.elf.name, "bytes": len(blob), "sha256": hashlib.sha256(blob).hexdigest()},
            "git": {"commit": command_output(["git", "rev-parse", "HEAD"], root),
                    "status": command_output(["git", "status", "--porcelain"], root).splitlines()},
            "tools": {"rustc": command_output(["rustc", "--version"], root),
                      "cargo": command_output(["cargo", "--version"], root),
                      "flip-link": linker_metadata(root)},
            "inputs": {name: hashlib.sha256((root / name).read_bytes()).hexdigest()
                       for name in ("Cargo.lock", "memory.x", ".cargo/config.toml")},
            "source_inputs": source_inputs(root),
            "environment": {name: os.environ[name]
                            for name in ("DEFMT_LOG", "RUSTFLAGS", "CARGO_ENCODED_RUSTFLAGS") if name in os.environ},
        })
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(manifest, indent=2) + "\n")
    except (ValueError, OSError, UnicodeError, struct.error, subprocess.SubprocessError) as error:
        print(f"Artifact validation failed: {error}", file=sys.stderr)
        return 1
    print(f"ELF verified: FLASH {manifest['flash']['bytes_used']}/{manifest['flash']['capacity_bytes']} bytes; "
          f"static RAM {manifest['ram']['static_bytes']} bytes; stack budget {manifest['ram']['stack_budget_bytes']} bytes")
    print(f"Artifact manifest: {args.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
