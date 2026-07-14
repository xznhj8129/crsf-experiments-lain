import time
import unittest

from elrstest.crsf import FrameType, make_extended_frame, make_frame, make_rc_frame
from elrstest.link import HandsetSession, ParameterClient


class FakePort:
    def __init__(self) -> None:
        self.writes: list[bytes] = []
        self.to_read: list = []

    def write(self, data: bytes) -> None:
        self.writes.append(data)

    def read_frames(self) -> list:
        frames, self.to_read = self.to_read, []
        return frames


def parse_one(raw: bytes):
    from elrstest.crsf import FrameParser
    frames = FrameParser().feed(raw)
    assert len(frames) == 1
    return frames[0]


class HandsetSessionTests(unittest.TestCase):
    def test_first_poll_sends_rc_at_5hz_default(self) -> None:
        port = FakePort()
        session = HandsetSession(port)
        session.poll()
        self.assertEqual(len(port.writes), 1)
        self.assertEqual(port.writes[0], make_rc_frame(session.channels_us))
        self.assertAlmostEqual(session.interval, 0.2)

    def test_pending_frames_piggyback_on_rc_write(self) -> None:
        port = FakePort()
        session = HandsetSession(port)
        ping = make_extended_frame(FrameType.DEVICE_PING, 0x00, 0xEA)
        session.queue(ping)
        session.poll()
        self.assertEqual(port.writes[0], make_rc_frame(session.channels_us) + ping)

    def test_radio_id_sync_adjusts_interval(self) -> None:
        port = FakePort()
        session = HandsetSession(port)
        interval_ticks = 40000  # 4 ms = 250 Hz
        payload = bytes([0xEA, 0xEE, 0x10]) + interval_ticks.to_bytes(4, "big") + (0).to_bytes(4, "big")
        port.to_read = [parse_one(make_frame(FrameType.RADIO_ID, payload))]
        session.poll()
        self.assertTrue(session.synced)
        self.assertAlmostEqual(session.interval, 0.004)

    def test_own_echo_is_dropped(self) -> None:
        port = FakePort()
        session = HandsetSession(port)
        session.poll()
        echoed = parse_one(port.writes[0])
        port.to_read = [echoed]
        frames = session.poll()
        self.assertNotIn(echoed.raw, [frame.raw for frame in frames])

    def test_handset_origin_addresses(self) -> None:
        session = HandsetSession(FakePort())
        self.assertEqual(session.origin(0xEE), 0xEF)  # ELRS TX -> elrs lua address
        self.assertEqual(session.origin(0xEC), 0xEA)  # anything else -> radio


class ParameterClientTests(unittest.TestCase):
    def test_chunked_read_reassembles(self) -> None:
        class FakeTransport:
            def __init__(self) -> None:
                self.sent: list[bytes] = []
                self.chunks = [
                    make_extended_frame(FrameType.PARAMETER_SETTINGS_ENTRY, 0xC8, 0xEC,
                                        bytes([5, 1]) + b"AAAA"),
                    make_extended_frame(FrameType.PARAMETER_SETTINGS_ENTRY, 0xC8, 0xEC,
                                        bytes([5, 0]) + b"BBBB"),
                ]

            def origin(self, device_address: int) -> int:
                return 0xC8

            def queue(self, frame: bytes) -> None:
                self.sent.append(frame)

            def poll(self) -> list:
                if self.chunks:
                    return [parse_one(self.chunks.pop(0))]
                return []

        transport = FakeTransport()
        client = ParameterClient(transport, timeout_seconds=1.0)
        data = client._read_chunks(0xEC, 5, FrameType.PARAMETER_READ)
        self.assertEqual(data, b"AAAABBBB")
        self.assertEqual(len(transport.sent), 2)


if __name__ == "__main__":
    unittest.main()
