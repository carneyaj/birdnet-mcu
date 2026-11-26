#!/usr/bin/env python3
"""Read-only BirdNET SD export over CH343 UART. Never sends bootloader commands."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import re
import struct
import sys
import time
import zlib

BEACON = b"\nBNFS2 READY\n"
BEACONS = {b"\nBNFS1 READY\n": 1, BEACON: 2}
KNOCK = b"BNFSOPEN"
CHUNK = 4096
LIST, NEXT, OPEN, READ, QUIT = range(1, 6)
OK, INVALID, IO_ERROR, END, BUSY = range(5)
LOGS = {"BIRDLOG.CSV", "ENVLOG.CSV", "AUDIOMAP.CSV", "BOOTSEQ.BIN", "HEALTH.CSV"}
SPECIES = r"(?:AMBIENT|[A-Za-z0-9_-]{1,48}-[0-9A-F]{8})"


class ProtocolError(RuntimeError):
    pass


def valid_directory(path: str) -> bool:
    if path in ("", "AUDIO") or re.fullmatch(rf"AUDIO/{SPECIES}", path):
        return True
    match = re.fullmatch(rf"(?:AUDIO/{SPECIES}/)?D([0-9]{{7}})", path)
    return bool(match and int(match[1]) <= 9999999 // 512)


def valid_file(path: str) -> bool:
    if path in LOGS or re.fullmatch(r"M[0-9]{7}\.TXT", path):
        return True
    if path.startswith("AUDIO/"):
        parts = path.split("/")
        if len(parts) != 4 or not re.fullmatch(SPECIES, parts[1]):
            return False
        path = "/".join(parts[2:])
    match = re.fullmatch(r"(?:D([0-9]{7})/)?A([0-9]{7})\.(?:BRC|WAV|OGG)", path)
    return bool(match and (match[1] is None or int(match[1]) == int(match[2]) // 512))


def request_packet(sequence: int, op: int, path: str = "", offset: int = 0, length: int = 0,
                   version: int = 2) -> bytes:
    name = path.encode("ascii")
    path_bytes = 24 if version == 1 else 96
    if len(name) > (21 if version == 1 else path_bytes - 1):
        raise ValueError("Remote path too long")
    body = struct.pack(f"<4sHBBII{path_bytes}s", f"BNQ{version}".encode(),
                       sequence, op, len(name), offset, length, name)
    return body + struct.pack("<I", zlib.crc32(body))


class Client:
    def __init__(self, serial_port):
        self.serial = serial_port
        self.sequence = 0
        self.last_reply = 0.0
        self.active = False
        self.started = 0.0
        self.version = 2

    def exact(self, count: int) -> bytes:
        result = bytearray()
        deadline = time.monotonic() + 4.0
        while len(result) < count and time.monotonic() < deadline:
            data = self.serial.read(count - len(result))
            result.extend(data)
        if len(result) != count:
            self.active = False
            raise ProtocolError("Serial timeout; partial download preserved. No further bytes sent.")
        return bytes(result)

    def response(self, sequence: int, prefix: bytes = b"") -> tuple[int, bytes]:
        deadline = time.monotonic() + 610
        while time.monotonic() < deadline:
            header = prefix + self.exact(12 - len(prefix))
            prefix = b""
            magic, received_id, status, reserved, size = struct.unpack("<4sHBBI", header)
            if magic != f"BNR{self.version}".encode() or received_id != sequence or reserved or size > CHUNK or status > BUSY:
                self.active = False
                raise ProtocolError(f"Unexpected protocol frame {header.hex()} "
                                    f"(expected id={sequence}); refusing to transmit again")
            data = self.exact(size)
            crc, = struct.unpack("<I", self.exact(4))
            if crc != zlib.crc32(header + data):
                self.active = False
                raise ProtocolError("Response checksum mismatch; reconnect to resume")
            self.last_reply = time.monotonic()
            if status != BUSY:
                return status, data
        self.active = False
        raise ProtocolError("Session deadline exceeded")

    def connect(self, wait: float = 30) -> str:
        # Listening is essential: any unsolicited byte could enter the ROM's
        # 100 ms update menu on a normal wake. Never probe ports with commands.
        self.serial.reset_input_buffer()
        seen = bytearray()
        deadline = time.monotonic() + wait
        while time.monotonic() < deadline:
            seen.extend(self.serial.read(1))
            version = next((v for beacon, v in BEACONS.items() if seen.endswith(beacon)), None)
            if version is not None:
                self.version = version
                response_magic = f"BNR{version}".encode()
                self.started = time.monotonic()
                self.serial.write(KNOCK)
                # Diagnostic builds may finish printing inference logs before
                # entering export. Only this setup phase permits resync; never
                # scan arbitrary audio payloads for an apparent frame boundary.
                prefix = bytearray()
                hello_deadline = time.monotonic() + 4
                while time.monotonic() < hello_deadline:
                    prefix.extend(self.serial.read(1))
                    if prefix.endswith(response_magic):
                        break
                    if len(prefix) > 4:
                        del prefix[:-4]
                else:
                    raise ProtocolError("No application HELLO after knock; reconnect without sending probes")
                status, data = self.response(0, response_magic)
                if status != OK:
                    raise ProtocolError("Firmware answered, but SD mount failed (card absent/backoff?)")
                self.active = True
                return data.decode("ascii")
            if len(seen) > len(BEACON):
                del seen[:-len(BEACON)]
        raise ProtocolError("No BNFS beacon. Install service-enabled firmware; do not reset into XMODEM.")

    def command(self, op: int, path: str = "", offset: int = 0, length: int = 0) -> tuple[int, bytes]:
        # A suspended laptop/process must not send into a later bootloader wake.
        # This bounds normal scheduling delays; it cannot prevent a physical
        # reset or OS suspension between this check and the USB write.
        if (not self.active or time.monotonic() - self.last_reply > 2 or
                time.monotonic() - self.started > 570):
            self.active = False
            raise ProtocolError("Session no longer fresh; reconnect before transmitting")
        self.sequence = self.sequence % 65535 + 1
        packet = request_packet(self.sequence, op, path, offset, length, self.version)
        try:
            for start in range(0, len(packet), 8):
                if time.monotonic() - self.last_reply > 2:
                    raise ProtocolError("Request stalled; reconnect")
                part = packet[start:start + 8]
                if self.serial.write(part) != len(part) or self.exact(1) != b"\x06":
                    raise ProtocolError("UART credit lost; reconnect")
            status, data = self.response(self.sequence)
            if status not in (OK, END):
                raise ProtocolError(f"Firmware rejected request {op}: status={status}")
            return status, data
        except Exception:
            self.active = False
            raise

    def listing(self, directory: str = "") -> list[dict]:
        if not valid_directory(directory):
            raise ValueError("Not an export directory")
        status, data = self.command(LIST, directory)
        entries = []
        path_bytes = 24 if self.version == 1 else 96
        entry_bytes = path_bytes + 8
        while status != END:
            if not data or len(data) % entry_bytes:
                self.active = False
                raise ProtocolError("Malformed directory response")
            for start in range(0, len(data), entry_bytes):
                name, size, is_dir = struct.unpack(f"<{path_bytes}sIB3x", data[start:start + entry_bytes])
                name = name.split(b"\0", 1)[0].decode("ascii")
                if is_dir not in (0, 1) or not (valid_directory(name) if is_dir else valid_file(name)):
                    self.active = False
                    raise ProtocolError("Unsafe remote path")
                if name.rpartition("/")[0] != directory:
                    self.active = False
                    raise ProtocolError("Entry outside requested directory")
                entries.append({"path": name, "size": size, "directory": bool(is_dir)})
            status, data = self.command(NEXT)
        return entries

    def walk(self, directory: str = ""):
        # Finish each paginated listing before descending: firmware keeps only
        # one directory handle. Layout depth is bounded by the path validator.
        entries = self.listing(directory)
        for entry in entries:
            if not entry["directory"]:
                yield entry
        for entry in entries:
            if entry["directory"]:
                yield from self.walk(entry["path"])

    def close(self):
        stats = None
        if (self.active and time.monotonic() - self.last_reply <= 2 and
                time.monotonic() - self.started <= 570):
            _, data = self.command(QUIT)
            if len(data) == 8:
                stats = struct.unpack("<II", data)
        self.active = False
        return stats


def crc_file(path: Path) -> int:
    crc = 0
    with path.open("rb") as stream:
        while data := stream.read(1024 * 1024):
            crc = zlib.crc32(data, crc)
    return crc


def safe_target(root: Path, remote: str) -> Path:
    if not valid_file(remote):
        raise ValueError("Not an export file")
    root = root.resolve()
    root.mkdir(parents=True, exist_ok=True)
    target = root / remote
    # Refuse symlinks, including part/metadata paths. Never follow a card-supplied
    # name or an existing symlink outside the explicitly chosen destination.
    parents = []
    parent = target.parent
    while parent != root:
        parents.append(parent)
        parent = parent.parent
    for path in (*parents, target, Path(str(target) + ".part"), Path(str(target) + ".part.json")):
        if path.is_symlink():
            raise ValueError(f"Refusing symlink: {path}")
    target.parent.mkdir(parents=True, exist_ok=True)
    return target


def download(client: Client, remote: str, destination: Path, overwrite: bool = False) -> Path:
    target = safe_target(destination, remote)
    # Hash local files BEFORE asking the board; no multi-second local work while
    # its five-second idle timer is running after OPEN. Big files may require a
    # fresh connection; command() fails closed if this took too long.
    existing = (target.stat().st_size, crc_file(target)) if target.exists() else None
    _, data = client.command(OPEN, remote)
    if len(data) != 8:
        raise ProtocolError("Malformed OPEN response")
    size, checksum = struct.unpack("<II", data)
    if existing == (size, checksum):
        return target
    if existing is not None and not overwrite:
        raise FileExistsError(f"Different local file exists: {target}; use --overwrite to replace it")
    part, meta = Path(str(target) + ".part"), Path(str(target) + ".part.json")
    identity = {"protocol": 1, "path": remote, "size": size, "crc32": checksum}
    if part.exists() or meta.exists():
        # Never truncate someone else's .part file. A stale tool-owned pair can
        # be restarted when the card's contents or a growing CSV have changed.
        if not (part.is_file() and meta.is_file()):
            raise FileExistsError(f"Unpaired partial files: {part}")
        old = json.loads(meta.read_text())
        if old.get("protocol") != 1 or old.get("path") != remote:
            raise FileExistsError(f"Unrecognized partial file: {part}")
        if old != identity or part.stat().st_size > size:
            part.unlink()
            meta.unlink()
    if not part.exists():
        meta.write_text(json.dumps(identity) + "\n")
        part.touch(exist_ok=False)
    offset = part.stat().st_size
    crc = crc_file(part)
    with part.open("ab") as stream:
        while offset < size:
            count = min(CHUNK, size - offset)
            _, block = client.command(READ, offset=offset, length=count)
            if len(block) != count:
                raise ProtocolError("Short READ response")
            stream.write(block)
            crc = zlib.crc32(block, crc)
            offset += len(block)
        stream.flush()
        os.fsync(stream.fileno())
    if crc != checksum:
        raise ProtocolError(f"Whole-file CRC mismatch: {part}; remove this .part/.part.json pair and retry")
    os.replace(part, target)
    meta.unlink()
    return target


def choose_port(port: str | None, watch: bool) -> str:
    from serial.tools import list_ports
    while True:
        if port and (not watch or os.path.exists(port)):
            return port  # Explicit tty aliases need not appear in list_ports().
        ports = list(list_ports.comports())
        matches = [p.device for p in ports if (p.device == port if port else p.vid == 0x1A86)]
        if len(matches) == 1:
            return matches[0]
        if len(matches) > 1:
            raise ValueError("Multiple WCH ports found; specify --port")
        if not watch:
            raise ValueError("Serial port absent; specify --port or use --watch to wait for attachment")
        time.sleep(0.5)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="Serial device; default: the single attached WCH/CH343 port")
    parser.add_argument("--watch", action="store_true", help="Wait in foreground for USB attachment; copy once then exit")
    parser.add_argument("--wait", type=float, default=30, help="Seconds to listen for the application beacon")
    parser.add_argument("--baud", type=int, default=921600)
    sub = parser.add_subparsers(dest="action", required=True)
    sub.add_parser("list", help="List exported files recursively")
    for action in ("get", "sync"):
        p = sub.add_parser(action)
        if action == "get":
            p.add_argument("remote")
        p.add_argument("--dest", type=Path, required=True)
        p.add_argument("--overwrite", action="store_true", help="Atomically replace changed local files, including growing CSVs")
    args = parser.parse_args()
    try:
        import serial
        port = choose_port(args.port, args.watch)
        # No RTS/DTR reset, bootloader magic, XMODEM, deletes or writes to the card.
        with serial.Serial(port, args.baud, timeout=0.02, write_timeout=1, exclusive=True) as ser:
            client = Client(ser)
            deployment = client.connect(args.wait)
            print(f"Connected {port}, deployment {deployment}. Classification paused during export.", file=sys.stderr)
            try:
                if args.action == "get":
                    print(download(client, args.remote, args.dest, args.overwrite))
                else:
                    for entry in client.walk():
                        if args.action == "list":
                            print(f"{entry['size']:10d}  {entry['path']}")
                        else:
                            print(download(client, entry["path"], args.dest, args.overwrite))
            finally:
                stats = client.close()
                if stats:
                    print(f"Export ended: {stats[0] / 1000:.1f}s, {stats[1]} capture windows discarded before close.", file=sys.stderr)
        return 0
    except ImportError:
        print("sd_download: install requirements-flash.txt in this Python environment", file=sys.stderr)
        return 1
    except (OSError, ValueError, ProtocolError) as error:
        print(f"sd_download: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
