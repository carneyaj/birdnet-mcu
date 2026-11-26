"""Seal a merged, 8 MiB XIAO ESP32S3 image."""
import hashlib
import json
from pathlib import Path
import sys

image = Path(sys.argv[1])
data = image.read_bytes()
if len(data) > 8 * 1024 * 1024 or data[0] != 0xe9:
    raise ValueError("Invalid ESP32-S3 merged image")
manifest = {"board": "esp32s3", "address": 0, "bytes": len(data),
            "sha256": hashlib.sha256(data).hexdigest()}
(image.parent / "image.json").write_text(json.dumps(manifest, indent=2) + "\n")
