"""Offline PC profile tests. No serial package, board, or physical port needed."""

from __future__ import annotations

import contextlib
import copy
import importlib.util
import io
import json
from pathlib import Path
import struct
import sys
import tempfile
import unittest
from unittest import mock
import zlib


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("friction_profiles", ROOT / "tools/friction_profiles.py")
profiles = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = profiles
SPEC.loader.exec_module(profiles)


def frame(sequence=0, result=0, identity=0, value=0.0):
    values = [0.0] * 44
    values[:4] = sequence, result, identity, value
    return profiles.FRAME_FLOATS.pack(*values) + profiles.TRAILER


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


class MockBoard:
    """Byte transport with a repeated latest ACK and complete profile protocol."""

    def __init__(self, clock):
        self.clock = clock
        self.profile = profiles.example_profile()
        self.frozen = self.profile
        self.caps = [1500.0] * 3
        self.revision = 7
        self.snapshot_revision = 7
        self.state = 0
        self.token = None
        self.expected_crc = None
        self.fields = {}
        self.commands = []
        self.last = frame(sequence=60000)
        self.pending = bytearray()
        self.reset_count = 0
        self.max_chunk = 37
        self.wrong_ack = False
        self.reject_commit = False
        self.drop_commit_ack = False
        self.never_apply = False
        self.fail_apply = False
        self.bad_readback = False
        self.bad_snapshot_crc = False
        self.partial_write = False
        self.silent = False

    def reset_input_buffer(self):
        self.reset_count += 1
        self.pending.clear()

    def read(self, size):
        self.clock.advance(0.001)
        if self.silent:
            return b""
        if not self.pending:
            self.pending.extend(self.last)
        count = min(size, self.max_chunk, len(self.pending))
        chunk = bytes(self.pending[:count])
        del self.pending[:count]
        return chunk

    def write(self, data):
        tokens = data.decode("ascii").strip().split()
        self.commands.append(tokens)
        if self.partial_write:
            return len(data) - 1
        verb, sequence, identity = tokens[0], int(tokens[1]), int(tokens[2])
        args = list(map(float, tokens[3:]))
        result, value = 0, 0.0
        if verb == "get":
            metadata = [1, 1, 1, 13, self.frozen.checksum & 0xffff,
                        self.frozen.checksum >> 16, self.snapshot_revision & 0xffff,
                        self.snapshot_revision >> 16]
            if 100 <= identity <= 107:
                value = metadata[identity - 100]
                if identity == 104 and self.bad_snapshot_crc:
                    value = int(value) ^ 1
            elif identity == 108:
                value = self.state
            elif identity == 109:
                value = 0
            elif identity == 110:
                value = 7
            elif 111 <= identity <= 113:
                value = self.caps[identity - 111]
            elif 200 <= identity <= 212:
                value = self.frozen.values[identity - 200]
            else:
                result = 4
        elif identity == profiles.SNAPSHOT:
            self.frozen = self.profile
            self.snapshot_revision = self.revision
        elif identity == profiles.IMPORT_BEGIN:
            self.token = sequence
            self.expected_crc = int(args[4]) | (int(args[5]) << 16)
            self.fields.clear()
            self.state = 1
        elif identity == profiles.IMPORT_FIELD:
            assert int(args[0]) == self.token
            self.fields[int(args[1])] = args[2]
        elif identity == profiles.IMPORT_COMMIT:
            assert int(args[0]) == self.token
            assert set(self.fields) == set(range(13))
            updated = profiles.Profile(tuple(self.fields[index] for index in range(13)))
            assert updated.checksum == self.expected_crc
            if self.reject_commit:
                result = 5
            else:
                result = 1
                self.state = 2 if self.never_apply else 4 if self.fail_apply else 3
                if self.state == 3:
                    self.profile = profiles.example_profile() if self.bad_readback else updated
                    self.revision += 1
            if self.drop_commit_ack:
                return len(data)
        else:
            raise AssertionError(f"unexpected operation: {tokens}")
        if self.wrong_ack:
            identity += 1
        self.last = frame(sequence, result, identity, value)
        return len(data)


class CodecTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix="friction-profile-test-")
        self.path = Path(self.directory.name) / "profile.json"
        self.addCleanup(self.directory.cleanup)

    def write_document(self, document):
        self.path.write_text(json.dumps(document), encoding="utf-8")

    def test_golden_binary_layout_and_crc(self):
        # Simple all-integer inputs make this vector independent of JSON formatting.
        values = (300.0,) + (1.0, 2.0, 0.0, 1500.0) * 3
        encoded = profiles.Profile(values).binary()
        self.assertEqual(len(encoded), 72)
        self.assertEqual(encoded[:16].hex(), "465250310100010001000d0000000000")
        self.assertEqual(encoded[16:20].hex(), "00009643")
        self.assertEqual(encoded[20:36].hex(), "0000803f00000040000000000080bb44")
        self.assertEqual(encoded[-4:], struct.pack("<I", zlib.crc32(encoded[:68])))
        self.assertEqual(encoded[-4:].hex(), "3221ac77")
        self.assertEqual(zlib.crc32(b"123456789"), 0xcbf43926)

    def test_json_roundtrip_and_optional_source(self):
        source = {"revision": 0xffffffff, "active_motor_mask": 2, "current_caps": [1500] * 3}
        profile = profiles.Profile(profiles.example_profile().values, source)
        profiles.save_profile(self.path, profile)
        restored = profiles.load_profile(self.path)
        self.assertEqual(restored.binary(), profile.binary())
        self.assertEqual(restored.source, source)
        self.assertEqual(restored.checksum, profiles.example_profile().checksum)

    def test_refuse_overwrite_and_atomic_replace(self):
        profiles.save_profile(self.path, profiles.example_profile())
        before = self.path.read_bytes()
        with self.assertRaisesRegex(profiles.ProfileError, "already exists"):
            profiles.save_profile(self.path, profiles.Profile((400,) + profiles.example_profile().values[1:]))
        self.assertEqual(self.path.read_bytes(), before)
        with mock.patch.object(profiles.os, "replace", side_effect=OSError("simulated disk failure")):
            with self.assertRaises(OSError):
                profiles.save_profile(self.path, profiles.example_profile(), force=True)
        self.assertEqual(self.path.read_bytes(), before)
        self.assertEqual(list(self.path.parent.glob("*.tmp")), [])
        profiles.save_profile(self.path, profiles.Profile((400,) + profiles.example_profile().values[1:]), force=True)
        self.assertEqual(profiles.load_profile(self.path).values[0], 400)

    def test_failed_new_export_leaves_no_output(self):
        with mock.patch.object(profiles.os, "replace", side_effect=OSError("simulated disk failure")):
            with self.assertRaises(OSError):
                profiles.save_profile(self.path, profiles.example_profile())
        self.assertFalse(self.path.exists())
        self.assertEqual(list(self.path.parent.iterdir()), [])

    def test_strict_schema_units_field_count_checksum(self):
        original = profiles.example_profile().document()
        mutations = (
            lambda d: d.update(schema=2),
            lambda d: d.update(version=0),
            lambda d: d.update(version=True),
            lambda d: d.update(units={**d["units"], "pid_error": "rpm"}),
            lambda d: d.update(units={**d["units"], "code": True}),
            lambda d: d.update(units="rpm"),
            lambda d: d.update(extra=1),
            lambda d: d.pop("format"),
            lambda d: d["values"].pop("f3_kd"),
            lambda d: d["values"].update(target_rpm=100),
            lambda d: d.update(crc32="00000000"),
            lambda d: d["values"].update(f1_kp=7),
            lambda d: d.update(source=None),
            lambda d: d.update(source={"revision": 7}),
        )
        for mutation in mutations:
            with self.subTest(mutation=mutation):
                document = copy.deepcopy(original)
                mutation(document)
                self.write_document(document)
                with self.assertRaises(profiles.ProfileError):
                    profiles.load_profile(self.path)

    def test_malformed_duplicate_nonfinite_and_oversized_json(self):
        for content in ('{', '{"schema":1,"schema":1}', '{"x":NaN}', '{"x":Infinity}',
                        ' ' * 16385, '{"schema":' + '9' * 5000 + '}', '[' * 1200 + ']' * 1200):
            with self.subTest(content=content[:30]):
                self.path.write_text(content, encoding="utf-8")
                with self.assertRaises(profiles.ProfileError):
                    profiles.load_profile(self.path)

    def test_float32_and_range_rejection(self):
        invalid = ((0, 0), (0, 2000.00001), (1, -1), (1, 100.000001), (2, 1001),
                   (3, 2), (4, 0), (4, 16385), (1, float("nan")), (1, float("inf")),
                   (1, 1e-60), (1, 1e100), (1, True), (1, "0.5"))
        for index, value in invalid:
            with self.subTest(index=index, value=value):
                values = list(profiles.example_profile().values)
                values[index] = value
                with self.assertRaises(profiles.ProfileError):
                    profiles.Profile(tuple(values))
        with self.assertRaises(profiles.ProfileError):
            profiles.Profile((1.0,) * 12)

    def test_source_values_are_informational_but_validated(self):
        source = {"revision": 1 << 10000, "active_motor_mask": 0, "current_caps": [1500] * 3}
        with self.assertRaises(profiles.ProfileError):
            profiles.Profile(profiles.example_profile().values, source)

    def test_cli_offline_does_not_need_serial(self):
        with mock.patch.dict(sys.modules, {"serial": None}), contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(profiles.main(["example", str(self.path)]), 0)
            self.assertEqual(profiles.main(["validate", str(self.path)]), 0)
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            profiles.main(["import", str(self.path)])


class DecoderTests(unittest.TestCase):
    def test_requires_two_frames_then_handles_split_reads(self):
        decoder = profiles.FrameDecoder()
        first = frame(1, 0, 100, 1)
        self.assertEqual(decoder.feed(first), [])
        frames = []
        for byte in frame(2, 0, 101, 1):
            frames.extend(decoder.feed(bytes([byte])))
        self.assertEqual([int(row[0]) for row in frames], [1, 2])
        self.assertEqual(len(decoder.feed(frame(3))), 1)

    def test_noise_truncation_resync_and_bounded_buffer(self):
        decoder = profiles.FrameDecoder()
        self.assertEqual(decoder.feed(b"noise" * 10000), [])
        self.assertLessEqual(len(decoder.buffer), decoder.MAX_BUFFER)
        rows = decoder.feed(frame(1)[:91] + b"broken" + frame(2) + frame(3))
        self.assertEqual([int(row[0]) for row in rows], [2, 3])
        rows = decoder.feed(frame(4)[:99] + frame(5) + frame(6))
        self.assertEqual([int(row[0]) for row in rows], [5, 6])

    def test_false_trailer_inside_finite_payload(self):
        # Trailer bytes crossing two float boundaries, not an aligned infinity.
        raw = bytearray(frame(12, 0, 100, 1))
        raw[65:69] = profiles.TRAILER
        self.assertIsNotNone(profiles.FrameDecoder.decode(raw))
        decoder = profiles.FrameDecoder()
        rows = decoder.feed(b"\x91\x42" + raw + frame(13))
        self.assertEqual([int(row[0]) for row in rows], [12, 13])

    def test_invalid_ack_and_nonfinite_channels(self):
        for raw in (frame(1.5), frame(1, 8), frame(1, 0, 65536), frame(1, 0, 1, float("nan"))):
            self.assertIsNone(profiles.FrameDecoder.decode(raw))
        decoder = profiles.FrameDecoder()
        rows = decoder.feed(frame(1, 9) + frame(2) + frame(3))
        self.assertEqual([int(row[0]) for row in rows], [2, 3])


class ClientTests(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.board = MockBoard(self.clock)
        self.client = profiles.ProfileClient(self.board, timeout=0.1,
                                              clock=self.clock, pause=self.clock.advance)

    def changed_profile(self):
        return profiles.Profile((410.0,) + (0.75, 0.15, 0.0, 1400.0) * 3)

    def test_snapshot_stale_drain_matching_and_metadata(self):
        self.board.pending.extend(frame(1, 0, 100, 999) * 2)
        result = self.client.snapshot()
        self.assertEqual(result.binary(), self.board.profile.binary())
        self.assertEqual(result.source["revision"], 7)
        self.assertEqual(self.board.reset_count, 1)
        self.assertEqual(self.board.commands[0][:3], ["call", "60001", "10"])

    def test_import_stages_all_then_commits_and_verifies(self):
        wanted = self.changed_profile()
        result = self.client.import_profile(wanted)
        self.assertEqual(result.binary(), wanted.binary())
        self.assertEqual(result.source["revision"], 8)
        calls = [command for command in self.board.commands if command[0] == "call"]
        operations = [int(command[2]) for command in calls]
        self.assertEqual(operations, [10, 11] + [12] * 13 + [13, 10])
        begin = next(command for command in calls if command[2] == "11")
        token = begin[1]
        self.assertTrue(all(command[3] == token for command in calls if command[2] in {"12", "13"}))
        sequences = [int(command[1]) for command in self.board.commands]
        self.assertEqual(len(sequences), len(set(sequences)))
        self.assertFalse(any(command[0] == "set" or command[2] in {"1", "2"} for command in self.board.commands))

    def test_compiled_current_cap_rejects_before_begin(self):
        self.board.caps[1] = 1300
        with self.assertRaisesRegex(profiles.ProfileError, "F2.*compiled"):
            self.client.import_profile(self.changed_profile())
        self.assertFalse(any(command[2] == "11" for command in self.board.commands))

    def test_busy_commit_reports_stopped_required_without_retry(self):
        self.board.reject_commit = True
        with self.assertRaisesRegex(profiles.ProfileError, "requires stopped motors"):
            self.client.import_profile(self.changed_profile())
        self.assertEqual(sum(command[2] == "13" for command in self.board.commands), 1)

    def test_missing_commit_ack_is_not_retried(self):
        self.board.drop_commit_ack = True
        with self.assertRaisesRegex(profiles.ProfileError, "No automatic retry"):
            self.client.import_profile(self.changed_profile())
        self.assertEqual(sum(command[2] == "13" for command in self.board.commands), 1)

    def test_queued_is_not_success_and_failed_state_rejected(self):
        self.board.never_apply = True
        with mock.patch.object(profiles, "PROFILE_LEASE_SECONDS", 1.0):
            with self.assertRaises(profiles.ProfileError):
                self.client.import_profile(self.changed_profile())
        self.assertEqual(self.board.state, 2)
        self.assertEqual(sum(command[2] == "13" for command in self.board.commands), 1)
        self.setUp()
        self.board.fail_apply = True
        with self.assertRaisesRegex(profiles.ProfileError, "state=4"):
            self.client.import_profile(self.changed_profile())

    def test_applied_requires_matching_fresh_readback(self):
        self.board.bad_readback = True
        with self.assertRaisesRegex(profiles.ProfileError, "readback differs"):
            self.client.import_profile(self.changed_profile())

    def test_bad_snapshot_crc_is_rejected(self):
        self.board.bad_snapshot_crc = True
        with self.assertRaisesRegex(profiles.ProfileError, "snapshot CRC mismatch"):
            self.client.snapshot()

    def test_ack_mismatch_times_out(self):
        self.board.wrong_ack = True
        with self.assertRaisesRegex(profiles.ProfileError, "timeout waiting for ACK"):
            self.client.snapshot()

    def test_no_stream_partial_write_and_sequence_wrap(self):
        self.board.silent = True
        with self.assertRaisesRegex(profiles.ProfileError, "no valid"):
            self.client.snapshot()
        self.setUp()
        self.board.partial_write = True
        with self.assertRaisesRegex(profiles.ProfileError, "partial serial write"):
            self.client.snapshot()
        self.setUp()
        self.board.last = frame(65535)
        self.client.snapshot()
        self.assertEqual(self.board.commands[0][1], "1")

    def test_run_stop_operations_not_available(self):
        for operation in (1, 2):
            with self.assertRaisesRegex(profiles.ProfileError, "only profile"):
                self.client.request("call", operation)
        self.assertEqual(self.board.commands, [])


if __name__ == "__main__":
    unittest.main(verbosity=2)
