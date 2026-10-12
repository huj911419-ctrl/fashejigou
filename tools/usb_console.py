"""Read-only RoboMaster USB CDC terminal; serial IO has one owner.

Install optional PC dependency: python -m pip install pyserial
List ports: python tools/usb_console.py --list
Connect: python tools/usb_console.py --port COM7 --log logs/board.txt
No flashing, motor commands, automatic port selection or parameter writes.
"""

from __future__ import annotations

import argparse
from datetime import datetime
from pathlib import Path
import queue
import re
import sys
import threading
import time


def allowed_command(line: str) -> bool:
    if line in {"help", "status", "events", "ping", "version", "stream on", "stream off"}:
        return True
    if line in {"log 1", "log 2"}:
        return True
    log_period = re.fullmatch(r"log period ([0-9]{2,4})", line)
    if log_period is not None:
        return 20 <= int(log_period[1]) <= 1000
    match = re.fullmatch(r"period ([0-9]{3,5})", line)
    return match is not None and 200 <= int(match[1]) <= 10000


def input_worker(commands: queue.Queue[str]) -> None:
    try:
        for line in sys.stdin:
            try:
                commands.put_nowait(line.strip())
            except queue.Full:
                print("[PC] input queue full; command discarded", flush=True)
    except (OSError, ValueError):
        return


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--list", action="store_true", help="list serial ports, then exit")
    parser.add_argument("--port", help="explicit device, e.g. COM7 or /dev/ttyACM0")
    parser.add_argument("--log", type=Path, help="append timestamped text to this UTF-8 file")
    args = parser.parse_args()
    if not args.list and not args.port:
        parser.error("choose --list or specify --port; ports are never selected automatically")
    try:
        import serial
        from serial.tools import list_ports
    except ImportError:
        print("Missing optional dependency. Install with: python -m pip install pyserial", file=sys.stderr)
        return 2
    if args.list:
        for port in list_ports.comports():
            identity = f"{port.vid:04X}:{port.pid:04X}" if port.vid is not None and port.pid is not None else "unknown"
            print(f"{port.device} | {port.description} | VID:PID={identity} | serial={port.serial_number}")
        return 0

    log = None
    device = None
    commands: queue.Queue[str] = queue.Queue(maxsize=16)
    partial = bytearray()

    def emit(text: str) -> None:
        line = f"{datetime.now().astimezone().isoformat(timespec='milliseconds')} {text}"
        print(line, flush=True)
        if log is not None:
            log.write(line + "\n")
            log.flush()

    try:
        if args.log is not None:
            args.log.parent.mkdir(parents=True, exist_ok=True)
            log = args.log.open("a", encoding="utf-8")
        threading.Thread(target=input_worker, args=(commands,), daemon=True).start()
        emit("[PC] queries: help/status/events/ping/version/stream on/stream off/period 200..10000/log 1/log 2/log period 20..1000; exit quits locally")
        retry_at = 0.0
        while True:
            # Handle local exit even while disconnected. Never replay commands on reconnect.
            try:
                command = commands.get_nowait()
            except queue.Empty:
                command = None
            if command == "exit":
                break
            if command:
                if not allowed_command(command):
                    emit("[PC] rejected: unknown read-only command")
                elif device is None:
                    emit("[PC] disconnected: command discarded")
                else:
                    try:
                        payload = (command + "\n").encode("ascii")
                        if device.write(payload) != len(payload):
                            raise serial.SerialTimeoutException("partial command write")
                        emit(f"[PC -> board] {command}")
                    except (serial.SerialException, OSError) as error:
                        emit(f"[PC] write failed: {error}; reconnecting")
                        device.close()
                        device = None
                        partial.clear()
                        retry_at = time.monotonic() + 1.0
            if device is None:
                if time.monotonic() < retry_at:
                    time.sleep(0.02)
                    continue
                try:
                    device = serial.Serial(args.port, baudrate=115200, timeout=0.02, write_timeout=0.1)
                    partial.clear()
                    emit(f"[PC] opened {args.port}; waiting for board status (USB CDC, not UART baud timing)")
                    # Terminate any interrupted input from a previous terminal session.
                    device.write(b"\n")
                except (serial.SerialException, OSError) as error:
                    if device is not None:
                        device.close()
                    device = None
                    emit(f"[PC] open failed: {error}; retry in 1 s (check port with --list)")
                    retry_at = time.monotonic() + 1.0
                    continue
            try:
                chunk = device.read(2048)
                partial.extend(chunk)
                while b"\n" in partial:
                    line, _, remaining = partial.partition(b"\n")
                    partial = bytearray(remaining)
                    emit(line.rstrip(b"\r").decode("utf-8", errors="replace"))
                if len(partial) > 16384:
                    partial.clear()
                    emit("[PC] discarded oversized unterminated input; check firmware/protocol")
            except (serial.SerialException, OSError) as error:
                emit(f"[PC] disconnected: {error}; reconnecting")
                device.close()
                device = None
                partial.clear()
                retry_at = time.monotonic() + 1.0
    except KeyboardInterrupt:
        return 0
    except OSError as error:
        print(f"[PC] log/terminal error: {error}", file=sys.stderr)
        return 1
    finally:
        if device is not None:
            device.close()
        if log is not None:
            log.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
