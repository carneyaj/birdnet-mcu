"""Deterministic, bounded label storage appended to a model flash payload."""
import hashlib
import struct

MAGIC = 0x314C4E42  # BNL1
MODEL_OFFSET = 0x200000
FLASH_END = 0x1000000


def fnv1a(data):
    value = 2166136261
    for byte in data:
        value = ((value ^ byte) * 16777619) & 0xffffffff
    return value


def encode(labels):
    if not labels or len(labels) > 65535 or len(set(labels)) != len(labels):
        raise ValueError("Expected 1..65535 unique labels")
    strings = bytearray()
    start = 16 + 4 * (len(labels) + 1)
    offsets = []
    for label in labels:
        if not label or any(c in label for c in "\0\r\n"):
            raise ValueError("Empty label or embedded NUL/newline")
        offsets.append(start + len(strings))
        strings.extend(label.encode("utf-8") + b"\0")
    offsets.append(start + len(strings))
    return struct.pack("<4I", MAGIC, 1, len(labels), offsets[-1]) + struct.pack(
        f"<{len(offsets)}I", *offsets) + strings


def package(model, labels, *, codec_on_demand=False):
    blob = encode(labels)
    if fnv1a(blob) == 0:
        raise ValueError("Zero label checksum is reserved for unverified retained state")
    offset = (len(model) + 4095) & ~4095
    payload = model + b"\xff" * (offset - len(model)) + blob
    end = 0x900000 if codec_on_demand else FLASH_END
    if MODEL_OFFSET + len(payload) > end:
        raise ValueError("Model + labels overlap the flash boundary/codec partition")
    info = dict(offset=offset, bytes=len(blob), fnv1a=fnv1a(blob),
                sha256=hashlib.sha256(blob).hexdigest(),
                payload_bytes=len(payload), payload_sha256=hashlib.sha256(payload).hexdigest(),
                flash_offset=MODEL_OFFSET, flash_limit=end)
    return payload, info
