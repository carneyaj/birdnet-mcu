#!/usr/bin/env python3
"""Flash WE2 firmware while aggressively entering the XMODEM boot menu."""

from __future__ import annotations

import io
import math
import sys
import time

import serial
import xmodem


PACKET_SIZE = 128
PROMPT = b"Send data using the xmodem protocol from your terminal"
REBOOT_PROMPT = b"Do you want to end file transmission and reboot system? (y)"


def open_serial(port: str, baudrate: int, timeout: float) -> serial.Serial:
    ser = serial.Serial()
    ser.port = port
    ser.timeout = timeout
    ser.write_timeout = 2
    ser.baudrate = baudrate
    ser.bytesize = serial.EIGHTBITS
    ser.stopbits = serial.STOPBITS_ONE
    ser.xonxoff = 0
    ser.rtscts = 0
    ser.parity = serial.PARITY_NONE
    if sys.platform != "win32":
        ser.exclusive = True
    ser.open()
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    return ser


def wait_for_boot_prompt(ser: serial.Serial, seconds: float) -> None:
    print("Press and release RESET on the Grove. Waiting for the boot menu.", flush=True)
    deadline = time.monotonic() + seconds
    seen = bytearray()
    tick = 0
    while time.monotonic() < deadline:
        ser.write(b"1\r")
        chunk = ser.read(256)
        if chunk:
            seen.extend(chunk)
            sys.stdout.buffer.write(chunk)
            sys.stdout.flush()
            if PROMPT in seen:
                print("\nBootloader XMODEM prompt detected.")
                return
        else:
            tick += 1
            if tick % 20 == 0:
                print(".", end="", flush=True)
        time.sleep(0.05)
    print()
    raise TimeoutError("WE2 bootloader XMODEM prompt was not detected")


def progress(total_packets: int, success_count: int, error_count: int) -> None:
    if not progress.total:
        return
    pct = min(total_packets / progress.total, 1.0)
    bar_total = 30
    filled = int(pct * bar_total)
    print(
        f"\r[{'#' * filled}{' ' * (bar_total - filled)}] "
        f"{pct:.2%} {total_packets}/{progress.total} error: {error_count}",
        end="",
        flush=True,
    )
    if pct >= 1.0:
        print()


progress.total = 0


def send_file(ser: serial.Serial, data: bytes, label: str, xmodem_timeout: float,
              tx_chunk: int = 0, tx_delay: float = 0.001) -> bool:
    received = bytearray()
    def getc(size: int, timeout: int = 1) -> bytes:
        prior_timeout = ser.timeout
        ser.timeout = xmodem_timeout
        try:
            data = ser.read(size)
            received.extend(data)
            del received[:-512]
            return data
        finally:
            ser.timeout = prior_timeout

    def putc(data: bytes, timeout: int = 1) -> int:
        if tx_chunk:
            sent = 0
            while sent < len(data):
                written = ser.write(data[sent:sent + tx_chunk])
                if not written:
                    return sent
                sent += written
                if not (getattr(ser, "_birdnet_packet_flush", False) or getattr(ser, "_birdnet_ack_drain", False)):
                    ser.flush()
                time.sleep(tx_delay)
            if getattr(ser, "_birdnet_packet_flush", False):
                ser.flush()
            return sent
        return ser.write(data)

    modem = xmodem.XMODEM(getc=getc, putc=putc, mode="xmodem")
    progress.total = math.ceil(len(data) / PACKET_SIZE)
    print(f"Sending {label} ({len(data):,} bytes)", flush=True)
    try:
        ok = modem.send(io.BytesIO(data), callback=progress)
    finally:
        # An ACK read can consume some of the next menu. Keep it for the next state.
        ser._birdnet_control_tail = bytes(received)
    print("xmodem_send bin file done!!" if ok else "xmodem_send bin file FAIL!!!!")
    return bool(ok)


def take_control_tail(ser):
    tail = bytearray(getattr(ser, "_birdnet_control_tail", b""))
    ser._birdnet_control_tail = b""
    return tail


def decline_reboot(ser: serial.Serial, seconds: float = 8.0, allow_ready: bool = False) -> None:
    deadline = time.monotonic() + seconds
    seen = take_control_tail(ser)
    last_receive = time.monotonic()
    answered = False
    if REBOOT_PROMPT in seen:
        ser.write(b"n\r")
        ser.flush()
        return
    while time.monotonic() < deadline:
        chunk = ser.read(256)
        if chunk:
            last_receive = time.monotonic()
            seen.extend(chunk)
            sys.stdout.buffer.write(chunk)
            sys.stdout.flush()
            if REBOOT_PROMPT in seen:
                if not answered:
                    ser.write(b"n\r")
                    ser.flush()
                return
            # Some bootloader paths immediately request the raw payload after
            # accepting its address preamble. Never send a stray 'n' into that
            # receiver. XMODEM repeats the C/NAK handshake for send_file().
            if allow_ready and chunk and all(c in (0x43, 0x15) for c in chunk):
                return
        # A complete, ACKed file has exactly one next-file decision here.
        # The board occasionally loses the short menu text. Answer once after
        # silence, as the SDK uploader does, but never stream repeated replies
        # into an observed raw-data handshake.
        if not answered and time.monotonic() - last_receive > 1.0:
            ser.write(b"n\r")
            ser.flush()
            answered = True
        time.sleep(0.01)
    raise TimeoutError("Bootloader next-file prompt or allowed payload handshake was not detected")



def flash(port: str, image: bytes, model: bytes, address: int) -> None:
    # The Himax bootloader receives separate 128-byte XMODEM files for the
    # application, the model address preamble and the model/label payload.
    timeout, tx_chunk, tx_delay = 30.0, 8, 0.001
    with open_serial(port, 921600, 0.05) as ser:
        ser._birdnet_ack_drain = True
        wait_for_boot_prompt(ser, 90.0)
        time.sleep(1.0)
        ser.reset_input_buffer()
        ser.write(b"1\r")
        if not send_file(ser, image, "Grove firmware", timeout, tx_chunk, tx_delay):
            raise RuntimeError("Firmware transfer failed")

        preamble = (b"\xc0\x5a" + address.to_bytes(4, "little") + bytes(4) + b"\x5a\xc0")
        decline_reboot(ser)
        if not send_file(ser, preamble.ljust(PACKET_SIZE, b"\xff"), "model address",
                         timeout, tx_chunk, tx_delay):
            raise RuntimeError("Model address transfer failed")
        decline_reboot(ser, seconds=60.0, allow_ready=True)
        if not send_file(ser, model, "int8 model and labels", timeout, tx_chunk, tx_delay):
            raise RuntimeError("Model transfer failed")

        print("Waiting for the final reboot prompt...", flush=True)
        deadline = time.monotonic() + 20
        seen = take_control_tail(ser)
        while time.monotonic() < deadline:
            if REBOOT_PROMPT in seen:
                ser.write(b"y\r")
                ser.flush()
                print("Transfer complete. The Grove is rebooting.")
                return
            chunk = ser.read(256)
            if chunk:
                seen.extend(chunk)
                sys.stdout.buffer.write(chunk)
                sys.stdout.flush()
        raise TimeoutError("Transfer finished without the expected reboot prompt")
