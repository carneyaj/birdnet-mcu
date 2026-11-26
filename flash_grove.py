#!/usr/bin/env python3
"""Flash the paired BirdNET field image and model onto a Grove Vision AI V2."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent


def verify(bundle):
    manifest = json.loads((bundle / "deployment.json").read_text())
    boot = (bundle / "output.img").read_bytes()
    payload = (bundle / "model-with-labels.bin").read_bytes()
    model = manifest["flash_labels"]
    address = model["flash_offset"]
    if (not boot or len(boot) >= 0x200000
            or hashlib.sha256(boot).hexdigest() != manifest["image_sha256"]
            or manifest["id"].encode("ascii") + b"\0" not in boot):
        raise ValueError("Grove firmware image differs from its deployment manifest")
    if (address != 0x200000 or len(payload) != model["payload_bytes"]
            or address + ((len(payload) + 127) // 128) * 128 > model["flash_limit"]
            or model["flash_limit"] > 0x1000000
            or hashlib.sha256(payload).hexdigest() != model["payload_sha256"]):
        raise ValueError("Grove model and labels differ from their deployment manifest")
    return boot, payload, address


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--port", help="Grove's explicit serial device")
    action.add_argument("--list", action="store_true", help="Show available serial devices")
    action.add_argument("--verify", action="store_true", help="Check files without opening serial")
    parser.add_argument("--bundle", type=Path, default=ROOT / "prebuilt/grove")
    args = parser.parse_args()
    try:
        if args.list:
            from serial.tools.list_ports import comports
            ports = sorted(comports())
            for port in ports:
                print(f"{port.device}  {port.description}")
            if not ports:
                print("No serial ports found.")
            return 0
        boot, payload, address = verify(args.bundle)
        print(f"Verified Grove bundle ({len(boot):,} firmware bytes and {len(payload):,} model bytes).")
        if args.port:
            from tools.grove_xmodem import flash
            flash(args.port, boot, payload, address)
    except (ImportError, OSError, RuntimeError, ValueError, KeyError) as error:
        print(f"Grove flasher: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
