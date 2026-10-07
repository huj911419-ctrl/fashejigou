"""Versioned friction-wheel parameter files and an explicit-port VOFA client.

Offline example/validate use only Python's standard library. Live export/import
require optional pyserial and the separately configured VOFA UART, not USB text
diagnostics. Import applies a complete profile to RAM while stopped; it never
sends RUN/STOP, retries COMMIT, or claims to save board Flash.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import re
import struct
import sys
import tempfile
import time
from typing import Callable, Protocol
import zlib


FORMAT = "robomaster.friction-profile"
SCHEMA = VERSION = UNITS_CODE = 1
FIELD_NAMES = ("ramp_rpm_s",) + tuple(
    f"f{wheel}_{field}"
    for wheel in range(1, 4)
    for field in ("kp", "ki", "kd", "current_limit")
)
FIELD_COUNT = len(FIELD_NAMES)
UNITS = {
    "code": UNITS_CODE,
    "ramp": "motor_rpm_per_second",
    "pid_error": "motor_degree_per_second",
    "pid_integral_time": "second",
    "pid_derivative_time": "second",
    "current_limit": "c620_raw",
}
LIMITS = ((1.0, 2000.0),) + ((0.0, 100.0), (0.0, 1000.0),
                           (0.0, 1.0), (1.0, 16384.0)) * 3
HEADER = struct.Struct("<4sHHHHI")
PAYLOAD = struct.Struct("<13f")
TRAILER = b"\x00\x00\x80\x7f"
FRAME_FLOATS = struct.Struct("<44f")
FRAME_SIZE = FRAME_FLOATS.size + len(TRAILER)
PROFILE_LEASE_SECONDS = 29.0  # Board expires at 30 s; retain a small host margin.

SNAPSHOT = 10
IMPORT_BEGIN = 11
IMPORT_FIELD = 12
IMPORT_COMMIT = 13
IMPORT_STATE = 108
STORAGE_AVAILABLE = 109
RESULT_NAMES = ("OK", "QUEUED", "SYNTAX", "RANGE", "UNKNOWN", "BUSY",
                "UNAVAILABLE", "RX_LOSS")


class ProfileError(ValueError):
    """Invalid profile, transport failure, or unconfirmed board operation."""


def integer(value: object, name: str, minimum: int, maximum: int) -> int:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ProfileError(f"{name}: expected integer {minimum}..{maximum}")
    if isinstance(value, float) and (not math.isfinite(value) or value != int(value)):
        raise ProfileError(f"{name}: expected integer {minimum}..{maximum}")
    if not minimum <= value <= maximum:
        raise ProfileError(f"{name}: expected integer {minimum}..{maximum}")
    return int(value)


def float32(value: object, name: str) -> float:
    """Round to the firmware's binary32; reject overflow and nonzero underflow."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ProfileError(f"{name}: expected finite float32 number")
    try:
        converted = float(value)
        rounded = struct.unpack("<f", struct.pack("<f", converted))[0]
    except (OverflowError, ValueError, struct.error) as error:
        raise ProfileError(f"{name}: outside float32 range") from error
    if not math.isfinite(rounded) or (converted != 0 and rounded == 0):
        raise ProfileError(f"{name}: non-finite or unrepresentable float32 number")
    return rounded


def validate_values(values: object) -> tuple[float, ...]:
    if not isinstance(values, (list, tuple)) or len(values) != FIELD_COUNT:
        raise ProfileError(f"values: expected exactly {FIELD_COUNT} fields")
    result = []
    for name, value, (low, high) in zip(FIELD_NAMES, values, LIMITS):
        number = float32(value, name)
        # Check the input too: rounding must not silently admit an out-of-range value.
        if not low <= value <= high or not low <= number <= high:
            raise ProfileError(f"{name}: expected {low:g}..{high:g}")
        result.append(number)
    return tuple(result)


@dataclass(frozen=True)
class Profile:
    """Immutable validated values; source is informational and is outside CRC."""

    values: tuple[float, ...]
    source: dict | None = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "values", validate_values(self.values))
        if self.source is not None:
            object.__setattr__(self, "source", validate_source(self.source))

    def binary(self) -> bytes:
        body = HEADER.pack(b"FRP1", SCHEMA, VERSION, UNITS_CODE, FIELD_COUNT, 0)
        body += PAYLOAD.pack(*self.values)
        return body + struct.pack("<I", zlib.crc32(body))

    @property
    def checksum(self) -> int:
        return struct.unpack("<I", self.binary()[-4:])[0]

    def document(self) -> dict:
        document = {
            "format": FORMAT, "schema": SCHEMA, "version": VERSION,
            "units": dict(UNITS), "values": dict(zip(FIELD_NAMES, self.values)),
            "crc32": f"{self.checksum:08x}",
        }
        if self.source is not None:
            document["source"] = self.source.copy()
        return document


def validate_source(source: object) -> dict:
    if not isinstance(source, dict) or set(source) != {
            "revision", "active_motor_mask", "current_caps"}:
        raise ProfileError("source: expected revision, active_motor_mask, current_caps")
    caps = source["current_caps"]
    if not isinstance(caps, list) or len(caps) != 3:
        raise ProfileError("source.current_caps: expected three numbers")
    checked_caps = [float32(value, "source.current_caps") for value in caps]
    if any(not 1 <= value <= 16384 for value in checked_caps):
        raise ProfileError("source.current_caps: expected 1..16384")
    return {
        "revision": integer(source["revision"], "source.revision", 0, 0xffffffff),
        "active_motor_mask": integer(source["active_motor_mask"], "source.active_motor_mask", 0, 7),
        "current_caps": checked_caps,
    }


def _unique_object(pairs: list[tuple[str, object]]) -> dict:
    result = {}
    for key, value in pairs:
        if key in result:
            raise ProfileError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def _reject_constant(value: str) -> None:
    raise ProfileError(f"non-finite JSON token: {value}")


def load_profile(path: Path) -> Profile:
    if path.stat().st_size > 16384:
        raise ProfileError("profile exceeds 16 KiB file limit")
    try:
        data = json.loads(path.read_text(encoding="utf-8"), object_pairs_hook=_unique_object,
                          parse_constant=_reject_constant)
    except (ValueError, UnicodeError, RecursionError) as error:
        raise ProfileError(f"invalid UTF-8 JSON profile: {error}") from error
    required = {"format", "schema", "version", "units", "values", "crc32"}
    if not isinstance(data, dict) or not required <= set(data) or set(data) - required - {"source"}:
        raise ProfileError("profile has missing or unknown top-level fields")
    if data["format"] != FORMAT:
        raise ProfileError("unsupported profile format")
    if integer(data["schema"], "schema", 1, 1) != SCHEMA:
        raise ProfileError("unsupported schema")
    if integer(data["version"], "version", 1, 1) != VERSION:
        raise ProfileError("unsupported version")
    if data["units"] != UNITS or isinstance(data["units"].get("code"), bool):
        raise ProfileError("units must exactly match the declared motor/PID units")
    if "source" in data and not isinstance(data["source"], dict):
        raise ProfileError("source must be an object when present")
    values = data["values"]
    if not isinstance(values, dict) or set(values) != set(FIELD_NAMES):
        raise ProfileError("values must contain exactly the 13 named profile fields")
    profile = Profile(tuple(values[name] for name in FIELD_NAMES), data.get("source"))
    crc = data["crc32"]
    if not isinstance(crc, str) or not re.fullmatch(r"[0-9a-fA-F]{8}", crc):
        raise ProfileError("crc32: expected eight hexadecimal digits")
    if int(crc, 16) != profile.checksum:
        raise ProfileError("CRC32 mismatch; changed/corrupt values must not be imported")
    return profile


def save_profile(path: Path, profile: Profile, force: bool = False) -> None:
    """Write in the same directory, flush, then atomically publish the whole file."""
    path = path.absolute()
    path.parent.mkdir(parents=True, exist_ok=True)
    reserved = False
    temporary = None
    try:
        # O_EXCL reserves the name before os.replace, including against a competing
        # export. Never silently overwrite an existing file without --force.
        if not force:
            descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
            os.close(descriptor)
            reserved = True
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", newline="\n",
                                         dir=path.parent, prefix=f".{path.name}.",
                                         suffix=".tmp", delete=False) as handle:
            temporary = Path(handle.name)
            json.dump(profile.document(), handle, indent=2, ensure_ascii=False, allow_nan=False)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
        temporary = None
        reserved = False
    except FileExistsError as error:
        raise ProfileError(f"{path} already exists; choose another name or use --force") from error
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
        if reserved:
            path.unlink(missing_ok=True)


def example_profile() -> Profile:
    """Conservative example only, never claimed to be mechanically tuned."""
    return Profile((300.0,) + (0.5, 0.1, 0.0, 1500.0) * 3)


class FrameDecoder:
    """Bounded JustFloat decoder; two adjacent frames establish byte alignment.

    JustFloat has no payload checksum. Framing/finite checks cannot detect every
    finite bit error; profile CRC and subsequent readback validate the parameters.
    """

    MAX_BUFFER = FRAME_SIZE * 8

    def __init__(self) -> None:
        self.buffer = bytearray()
        self.aligned = False

    @staticmethod
    def decode(raw: bytes) -> tuple[float, ...] | None:
        if len(raw) != FRAME_SIZE or raw[-4:] != TRAILER:
            return None
        values = FRAME_FLOATS.unpack(raw[:-4])
        if not all(math.isfinite(value) for value in values):
            return None
        for value, high in zip(values[:3], (65535, 7, 65535)):
            if value != int(value) or not 0 <= value <= high:
                return None
        return values

    def feed(self, chunk: bytes) -> list[tuple[float, ...]]:
        frames = []
        # Chunking keeps storage bounded even if an adapter hands us a large read.
        for offset in range(0, len(chunk), FRAME_SIZE * 2):
            self.buffer.extend(chunk[offset:offset + FRAME_SIZE * 2])
            while True:
                if not self.aligned:
                    found = False
                    for start in range(max(0, len(self.buffer) - 2 * FRAME_SIZE + 1)):
                        first = self.buffer[start:start + FRAME_SIZE]
                        second = self.buffer[start + FRAME_SIZE:start + 2 * FRAME_SIZE]
                        if self.decode(first) is not None and self.decode(second) is not None:
                            del self.buffer[:start]
                            self.aligned = found = True
                            break
                    if not found:
                        del self.buffer[:max(0, len(self.buffer) - (2 * FRAME_SIZE - 1))]
                        break
                if len(self.buffer) < FRAME_SIZE:
                    break
                frame = self.decode(self.buffer[:FRAME_SIZE])
                if frame is None:
                    self.aligned = False
                    del self.buffer[:1]
                    continue
                frames.append(frame)
                del self.buffer[:FRAME_SIZE]
        return frames


class ByteTransport(Protocol):
    """One synchronous owner. Reads and writes must themselves have timeouts."""

    def read(self, size: int) -> bytes: ...
    def write(self, data: bytes) -> int: ...
    def reset_input_buffer(self) -> None: ...


class ProfileClient:
    """Single in-flight request, monotonically advancing nonzero u16 sequence."""

    def __init__(self, transport: ByteTransport, timeout: float = 3.0,
                 clock: Callable[[], float] = time.monotonic,
                 pause: Callable[[float], None] = time.sleep) -> None:
        if not math.isfinite(timeout) or not 0.1 <= timeout <= 10.0:
            raise ProfileError("timeout must be 0.1..10 seconds")
        self.transport = transport
        self.timeout = timeout
        self.clock = clock
        self.pause = pause
        self.decoder = FrameDecoder()
        self.sequence = 0
        self.synchronized = False

    def _frames(self) -> list[tuple[float, ...]]:
        chunk = self.transport.read(512)
        if not chunk:
            self.pause(0.002)
        return self.decoder.feed(chunk)

    def synchronize(self) -> None:
        """Discard buffered stale data, then learn the last ACK before choosing seq."""
        self.transport.reset_input_buffer()
        self.decoder = FrameDecoder()
        deadline = self.clock() + self.timeout
        while self.clock() < deadline:
            frames = self._frames()
            if frames:
                self.sequence = int(frames[-1][0])
                self.synchronized = True
                return
        raise ProfileError("no valid 44-channel VOFA stream; check explicit port, baud and firmware")

    def request(self, verb: str, identity: int, arguments: tuple = (),
                expected: int = 0, deadline: float | None = None) -> tuple[int, float]:
        if not self.synchronized:
            self.synchronize()
        if verb not in {"get", "call"} or (verb == "call" and identity not in {10, 11, 12, 13}):
            raise ProfileError("client permits only profile operations and queries")
        self.sequence = self.sequence % 65535 + 1
        sequence = self.sequence
        text = f"{verb} {sequence} {identity}"
        if arguments:
            text += " " + " ".join(format(value, ".9g") for value in arguments)
        payload = (text + "\n").encode("ascii")
        end = self.clock() + self.timeout
        if deadline is not None:
            end = min(end, deadline)
        if self.clock() >= end:
            raise ProfileError("profile lease expired before sending the next request")
        if self.transport.write(payload) != len(payload):
            raise ProfileError("partial serial write; operation unconfirmed, no automatic retry")
        while self.clock() < end:
            for frame in self._frames():
                if int(frame[0]) != sequence or int(frame[2]) != identity:
                    continue
                result = int(frame[1])
                if result != expected:
                    detail = ""
                    if result == 5:
                        detail = ("; profile import requires stopped motors and no pending parameter update. "
                                  "Stop through your normal control interface first; this tool sends no STOP")
                    if result == 6:
                        detail = "; check profile/VOFA feature switches and board configuration"
                    raise ProfileError(f"{verb} {identity} rejected: {RESULT_NAMES[result]}{detail}")
                return sequence, frame[3]
        raise ProfileError(f"timeout waiting for ACK seq={sequence}, id={identity}; operation unconfirmed")

    def get(self, identity: int, deadline: float | None = None) -> float:
        return self.request("get", identity, deadline=deadline)[1]

    def snapshot(self) -> Profile:
        self.request("call", SNAPSHOT)
        deadline = self.clock() + PROFILE_LEASE_SECONDS
        metadata = [self.get(identity, deadline) for identity in range(100, 108)]
        for name, actual, expected in zip(("schema", "version", "units", "count"),
                                          metadata[:4], (SCHEMA, VERSION, UNITS_CODE, FIELD_COUNT)):
            if integer(actual, name, 0, 65535) != expected:
                raise ProfileError(f"firmware profile {name} unsupported: {actual:g}")
        crc = integer(metadata[4], "crc low", 0, 65535)
        crc |= integer(metadata[5], "crc high", 0, 65535) << 16
        revision = integer(metadata[6], "revision low", 0, 65535)
        revision |= integer(metadata[7], "revision high", 0, 65535) << 16
        mask = self.get(110, deadline)
        caps = [self.get(identity, deadline) for identity in range(111, 114)]
        values = tuple(self.get(identity, deadline) for identity in range(200, 213))
        profile = Profile(values, {"revision": revision, "active_motor_mask": mask,
                                   "current_caps": caps})
        if profile.checksum != crc:
            raise ProfileError("snapshot CRC mismatch; no profile file should be published")
        return profile

    def import_profile(self, profile: Profile) -> Profile:
        current = self.snapshot()
        for wheel, cap in enumerate(current.source["current_caps"]):
            if profile.values[4 + wheel * 4] > cap:
                raise ProfileError(f"F{wheel + 1} current_limit exceeds compiled board cap {cap:g}")
        crc = profile.checksum
        deadline = self.clock() + PROFILE_LEASE_SECONDS
        token, _ = self.request("call", IMPORT_BEGIN,
                                (SCHEMA, VERSION, UNITS_CODE, FIELD_COUNT, crc & 0xffff, crc >> 16),
                                deadline=deadline)
        for index, value in enumerate(profile.values):
            self.request("call", IMPORT_FIELD, (token, index, value), deadline=deadline)
        try:
            self.request("call", IMPORT_COMMIT, (token,), expected=1, deadline=deadline)
        except ProfileError as error:
            raise ProfileError(f"COMMIT was not confirmed queued: {error}. No automatic retry was sent") from error
        while self.clock() < deadline:
            try:
                state = integer(self.get(IMPORT_STATE, deadline), "import state", 0, 4)
            except ProfileError as error:
                raise ProfileError(f"COMMIT queued, but APPLIED was not confirmed: {error}") from error
            if state == 3:
                readback = self.snapshot()
                if readback.binary() != profile.binary():
                    raise ProfileError("board reported APPLIED but fresh profile readback differs")
                return readback
            if state in {0, 4}:
                raise ProfileError(f"board import did not apply (state={state}); inspect board diagnostics")
            self.pause(0.01)
        raise ProfileError("COMMIT queued but APPLIED was not confirmed; do not assume parameters changed")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    for name in ("example", "validate", "export", "import"):
        command = commands.add_parser(name)
        command.add_argument("path", type=Path, help="JSON profile path")
        if name in {"example", "export"}:
            command.add_argument("--force", action="store_true", help="replace an existing file")
        if name in {"export", "import"}:
            command.add_argument("--port", required=True, help="explicit VOFA UART port, e.g. COM7")
            command.add_argument("--baud", type=int, default=115200)
            command.add_argument("--timeout", type=float, default=3.0, help="per ACK timeout, 0.1..10 s")
    args = parser.parse_args(argv)
    try:
        if args.command == "example":
            save_profile(args.path, example_profile(), args.force)
            print(f"Example created: {args.path}; values are not mechanically tuned")
            return 0
        if args.command == "validate":
            profile = load_profile(args.path)
            print(f"Valid profile: 13 fields, CRC32={profile.checksum:08x}; board caps still require checking")
            return 0
        profile = load_profile(args.path) if args.command == "import" else None
        if args.command == "export" and args.path.exists() and not args.force:
            raise ProfileError("output already exists; choose another name or use --force")
        if not 1 <= args.baud <= 4000000 or not math.isfinite(args.timeout) or not 0.1 <= args.timeout <= 10:
            raise ProfileError("baud must be 1..4000000 and timeout 0.1..10 seconds")
        try:
            import serial
        except ImportError as error:
            raise ProfileError("live serial requires optional dependency: python -m pip install pyserial") from error
        with serial.Serial(args.port, baudrate=args.baud, timeout=0.02, write_timeout=0.2) as transport:
            client = ProfileClient(transport, args.timeout)
            if args.command == "export":
                exported = client.snapshot()
                save_profile(args.path, exported, args.force)
                print(f"Consistent RAM profile exported: {args.path}; CRC32={exported.checksum:08x}")
            else:
                applied = client.import_profile(profile)
                print(f"Applied to board RAM and read back: CRC32={applied.checksum:08x}. "
                      "Board Flash was not written; reboot restores compiled defaults")
        return 0
    except (ProfileError, OSError) as error:
        print(f"Profile operation failed: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
