#!/usr/bin/env python3
"""Flash the merged BirdNET field image onto a XIAO ESP32S3 Sense."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parent


def verify(bundle):
    image = bundle / "xiao-esp32s3.bin"
    data = image.read_bytes()
    manifest = json.loads((bundle / "image.json").read_text())
    if (manifest["board"] != "esp32s3" or manifest["address"] != 0
            or not data or data[0] != 0xe9 or len(data) > 8 * 1024 * 1024
            or len(data) != manifest["bytes"]
            or hashlib.sha256(data).hexdigest() != manifest["sha256"]):
        raise ValueError("ESP32-S3 image differs from its bundle manifest")
    return image, len(data)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--port", help="XIAO's explicit serial device")
    action.add_argument("--list", action="store_true", help="Show available serial devices")
    action.add_argument("--verify", action="store_true", help="Check files without opening serial")
    parser.add_argument("--bundle", type=Path, default=ROOT / "prebuilt/esp32")
    parser.add_argument("--baud", type=int, default=460800, help="Use 115200 for a slower USB link")
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
        image, size = verify(args.bundle)
        print(f"Verified XIAO ESP32-S3 image ({size:,} bytes).", flush=True)
        if args.port:
            command = [sys.executable, "-m", "esptool", "--chip", "esp32s3", "--port", args.port,
                       "--baud", str(args.baud), "write_flash", "0x0", str(image)]
            return subprocess.run(command).returncode
    except (ImportError, OSError, ValueError, KeyError) as error:
        print(f"ESP32 flasher: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
