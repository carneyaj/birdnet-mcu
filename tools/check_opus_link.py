"""Reject an apparent successful link that discarded the detection encoder."""
import argparse
import struct
from pathlib import Path
import subprocess
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("elf")
    parser.add_argument("--image", type=Path)
    parser.add_argument("--payload", type=Path)
    args = parser.parse_args()
    output = subprocess.check_output(["arm-none-eabi-nm", "--defined-only", args.elf], text=True)
    names = {line.split()[-1] for line in output.splitlines() if line.split()}
    for name in ("opus_encode", "opus_encoder_init", "celt_encode_with_ec"):
        if name not in names:
            raise SystemExit(f"Opus encoder was not linked: missing {name}")
    print("Verified linked Opus encoder entry points")
    symbols = subprocess.check_output(["arm-none-eabi-nm", "-S", "--defined-only", args.elf], text=True)
    state = [line.split() for line in symbols.splitlines() if "g_encoder_state" in line]
    if len(state) != 1 or not (0x30000000 <= int(state[0][0], 16) and
                               int(state[0][0], 16) + int(state[0][1], 16) <= 0x30040000):
        raise SystemExit("Opus encoder state must reside wholly in DTCM")
    print("Verified Opus state resides in DTCM")
    if args.image:
        with tempfile.TemporaryDirectory() as directory:
            section = Path(directory) / "opus.bin"
            subprocess.run(["arm-none-eabi-objcopy", "--dump-section",
                            f".opus_text={section}", args.elf,
                            str(Path(directory) / "copy.elf")], check=True)
            data = section.read_bytes()
            if args.payload:
                payload = args.payload.read_bytes()
                if len(payload) != len(data) + 32 or payload[32:] != data:
                    raise SystemExit("Raw flash payload does not contain the exact linked Opus section")
                header = struct.unpack('<8I', payload[:32])
                fnv = 2166136261
                for b in data: fnv = ((fnv ^ b) * 16777619) & 0xffffffff
                if header != (0x424e4f50, 1, 0x340a0000, len(data), fnv, 0, 0, 0):
                    raise SystemExit("Raw codec header differs from linked code")
                if len(args.image.read_bytes()) >= 0x100000:
                    raise SystemExit("Boot image overlaps raw codec slot")
                print(f"Verified {len(data)} Opus SRAM section bytes in flash payload")
            else:
                if not data or args.image.read_bytes().find(data) < 0:
                    raise SystemExit("Flash image does not contain the complete Opus SRAM section")
                print(f"Verified {len(data)} Opus SRAM section bytes in flash image")


if __name__ == "__main__":
    main()
