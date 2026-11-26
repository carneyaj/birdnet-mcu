"""Check that explicit scratch buffers are not cleared by the startup table."""
import argparse
from pathlib import Path
import re
import struct
import subprocess
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("elf")
    args = parser.parse_args()
    listing = subprocess.check_output(["arm-none-eabi-objdump", "-h", args.elf], text=True)
    sections = {}
    for line in listing.splitlines():
        match = re.match(r"\s*\d+\s+(\S+)\s+([0-9a-f]+)\s+([0-9a-f]+)\s", line)
        if match:
            name, size, address = match.groups()
            sections[name] = (int(address, 16), int(size, 16))
    with tempfile.TemporaryDirectory() as directory:
        table = Path(directory) / "zero.bin"
        subprocess.run(["arm-none-eabi-objcopy", "--dump-section", f".zero.table={table}",
                        args.elf, str(Path(directory) / "copy.elf")], check=True)
        cleared = [(addr, words * 4) for addr, words in struct.iter_unpack("<II", table.read_bytes())]
    for name in (".scratch_dtcm", ".scratch_audio", ".frontend_cache", ".scratch_tensor"):
        address, size = sections.get(name, (0, 0))
        if not size:
            continue
        if any(count and address < start + count and start < address + size for start, count in cleared):
            raise SystemExit(f"Startup clears {name}")
        print(f"Verified {name}: {size} bytes excluded from startup clearing")
    # An explicitly selected, poison-qualified arena lives in its own section.
    # Ordinary .audio1 control state must still be zeroed on every wake.
    arena_address, arena_size = sections.get(".scratch_tensor", (0, 0))
    if arena_size and (arena_size != 1088 * 1024 or arena_address % 32):
        raise SystemExit("Unexpected scratch arena capacity/alignment")
    address, size = sections.get(".audio1", (0, 0))
    if size and not any(start == address and count >= size for start, count in cleared):
        raise SystemExit("Tensor arena unexpectedly removed from startup clearing")


if __name__ == "__main__":
    main()
