import unittest

from elrstest.crsf import (
    FrameParser,
    FrameType,
    crc8,
    make_battery_frame,
    make_extended_frame,
    make_frame,
    make_rc_frame,
    pack_channels_us,
    unpack_channels_us,
    validate_frame,
)


class CrsfTests(unittest.TestCase):
    def test_crsfproxy_rc_frame_is_reproduced_exactly(self) -> None:
        expected = bytes.fromhex(
            "c8 18 16 e0 03 1f f8 c0 c7 0a f0 81 0f 7c e0 03 1f f8 c0 07 3e f0 81 0f 7c 33"
        )
        channels = [1500] * 16
        channels[4] = 1000
        self.assertEqual(make_rc_frame(channels), expected)

    def test_channel_round_trip(self) -> None:
        channels = [1000, 1100, 1200, 1300, 1400, 1500, 1600, 1700, 1800, 2000, 1000, 1250, 1750, 1900, 1050, 1950]
        decoded = unpack_channels_us(pack_channels_us(channels))
        self.assertTrue(all(abs(actual - expected) <= 1 for actual, expected in zip(decoded, channels)))

    def test_parser_handles_chunks_and_visible_noise(self) -> None:
        first = make_battery_frame(251, 37, 0x012345, 73)
        second = make_extended_frame(FrameType.DEVICE_PING, 0x00, 0xEA)
        parser = FrameParser()
        self.assertEqual(parser.feed(b"noise" + first[:5]), [])
        frames = parser.feed(first[5:] + second)
        self.assertEqual([frame.raw for frame in frames], [first, second])
        self.assertEqual(parser.bytes_discarded, 5)
        self.assertEqual(parser.crc_errors, 0)

    def test_parser_rejects_bad_crc_and_recovers(self) -> None:
        damaged = bytearray(make_frame(FrameType.HEARTBEAT, b"\x01\x02"))
        damaged[-1] ^= 1
        valid = make_frame(FrameType.VARIO, b"\x00\x64")
        parser = FrameParser()
        frames = parser.feed(bytes(damaged) + valid)
        self.assertEqual([frame.raw for frame in frames], [valid])
        self.assertGreater(parser.crc_errors, 0)

    def test_crc_known_value(self) -> None:
        self.assertEqual(crc8(bytes.fromhex("16 e0 03 1f f8 c0 c7 0a f0 81 0f 7c e0 03 1f f8 c0 07 3e f0 81 0f 7c")), 0x33)
        self.assertTrue(validate_frame(make_frame(FrameType.HEARTBEAT, b"\x01\x02")))


if __name__ == "__main__":
    unittest.main()
