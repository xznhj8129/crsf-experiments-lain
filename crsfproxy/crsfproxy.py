#!/usr/bin/env python3
"""
Usage:
  python crsfproxy.py --device /dev/ttyUSB0 --baud 420000 --host 0.0.0.0 --port 60000 --loop_hz 250 --tx_rate 100 --telemetry_udp 192.168.4.2:40042

CRSF TX side bridge:
- Listens on a UDP port for 40-byte RC packets: <uint32 t_ms><16 x uint16 us><uint32 crc32>.
- Converts to CRSF RC_CHANNELS_PACKED and writes to the serial CRSF port at tx_rate.
- Parses inbound CRSF telemetry and can forward raw frames to a UDP target (e.g. MWP).

Telemetry output:
- Optional raw CRSF frames to --telemetry_udp (host:port)

Failsafe (proxy):
- If RC updates stop for < failsafe_time_ms, repeat last_valid_channels_us.
- If RC updates stop for >= failsafe_time_ms, send --failsafe_channels_us (defaults to throttle/arm low).
- Radio link failsafe (TX<->RX loss) is handled by the receiver, not this proxy.
- Failsafe channel values are configurable via --failsafe_channels_us.
"""

import argparse
import socket
import struct
import time
import zlib
from enum import IntEnum

import serial

CRSF_SYNC = 0xC8
CHANNEL_COUNT = 16
MIN_US = 900
MAX_US = 2100
ARM_LOW_US = 900
MID_US = 1500
UDP_PAYLOAD_LEN = 4 + CHANNEL_COUNT * 2
UDP_PACKET_LEN = UDP_PAYLOAD_LEN + 4
WRITE_TIMEOUT_S = 0.1
FAILSAFE_DEFAULT_US = [
    1500, 1500, 900, 1500, 900, 1500, 1500, 1500,
    1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500,
]

class PacketsTypes(IntEnum):
    GPS = 0x02
    VARIO = 0x07
    BATTERY_SENSOR = 0x08
    BARO_ALT = 0x09
    HEARTBEAT = 0x0B
    VIDEO_TRANSMITTER = 0x0F
    LINK_STATISTICS = 0x14
    RC_CHANNELS_PACKED = 0x16
    ATTITUDE = 0x1E
    FLIGHT_MODE = 0x21
    DEVICE_INFO = 0x29
    CONFIG_READ = 0x2C
    CONFIG_WRITE = 0x2D
    RADIO_ID = 0x3A

def crc8_dvb_s2(crc, a) -> int:
    crc ^= a
    for _ in range(8):
        if crc & 0x80:
            crc = (crc << 1) ^ 0xD5
        else:
            crc <<= 1
    return crc & 0xFF

def crc8_data(data) -> int:
    crc = 0
    for a in data:
        crc = crc8_dvb_s2(crc, a)
    return crc

def crsf_validate_frame(frame) -> bool:
    # frame = [0]sync [1]len [2]type ... [n]crc
    if len(frame) < 4:
        return False
    return crc8_data(frame[2:-1]) == frame[-1]

def signed_byte(b: int) -> int:
    return b - 256 if b >= 128 else b

def us_to_crsf(us: int) -> int:
    # 1000..2000 us -> 172..1811 (1639 steps)
    us = max(1000, min(2000, int(us)))
    return 172 + round((us - 1000) * 1639 / 1000)

def crsf_to_us(v: int) -> int:
    v = max(172, min(1811, int(v)))
    return 1000 + round((v - 172) * 1000 / 1639)

def packCrsfToBytes(channels) -> bytes:
    # channels are 16 ints in CRSF range (0..1811, typical 172..1811)
    if len(channels) != CHANNEL_COUNT:
        raise ValueError("CRSF must have 16 channels")
    result = bytearray()
    destShift = 0
    newVal = 0
    for ch in channels:
        newVal |= (ch << destShift) & 0xFF
        result.append(newVal)
        srcBitsLeft = 11 - 8 + destShift
        newVal = ch >> (11 - srcBitsLeft)
        if srcBitsLeft >= 8:
            result.append(newVal & 0xFF)
            newVal >>= 8
            srcBitsLeft -= 8
        destShift = srcBitsLeft
    return result  # 22 bytes

def channelsCrsfToChannelsPacket(channels_crsf) -> bytes:
    # channels_crsf: list of 16 values already in CRSF units
    payload = bytearray([PacketsTypes.RC_CHANNELS_PACKED])
    payload += packCrsfToBytes(channels_crsf)
    length = len(payload) + 1  # +1 for CRC
    frame = bytearray([CRSF_SYNC, length]) + payload
    frame.append(crc8_data(frame[2:]))
    return frame

def channelsUsToPacket(us_channels) -> bytes:
    # us_channels: list of 16 microsecond values
    if len(us_channels) != CHANNEL_COUNT:
        raise ValueError("Need 16 channels")
    crsf_ch = [us_to_crsf(v) for v in us_channels]
    return channelsCrsfToChannelsPacket(crsf_ch)

def unpack_rc_channels(payload: bytes) -> list:
    # payload is the 22 data bytes after type
    ch = []
    acc = 0
    bits = 0
    for b in payload[:22]:
        acc |= (b & 0xFF) << bits
        bits += 8
        if bits >= 11:
            ch.append(acc & 0x7FF)
            acc >>= 11
            bits -= 11
            if len(ch) == CHANNEL_COUNT:
                break
    if len(ch) != CHANNEL_COUNT:
        ch += [172] * (CHANNEL_COUNT - len(ch))
    return ch

def handleCrsfPacket(ptype: int, data: bytes, verbose=False):
    """
    Parse CRSF telemetry packet. Returns a dict, or None if unhandled.
    data is the whole frame, including sync and crc.
    """
    try:
        pkt_type = PacketsTypes(ptype)
    except ValueError:
        pkt_type = None

    now = time.time()
    out = {"t": now, "ptype": int(ptype)}

    if pkt_type == PacketsTypes.RADIO_ID:
        out["type"] = "RADIO_ID"
        out["raw"] = data.hex()
        return out

    if pkt_type == PacketsTypes.LINK_STATISTICS:
        out.update({
            "type": "LINK_STATISTICS",
            "rssi1": signed_byte(data[3]),
            "rssi2": signed_byte(data[4]),
            "lq": data[5],
            "snr": signed_byte(data[6]),
            "antenna": data[7],
            "mode": data[8],
            "power": data[9],
            "downlink_rssi": signed_byte(data[10]),
            "downlink_lq": data[11],
            "downlink_snr": signed_byte(data[12]),
        })
        return out

    if pkt_type == PacketsTypes.ATTITUDE:
        out.update({
            "type": "ATTITUDE",
            "pitch_rad": int.from_bytes(data[3:5], "big", signed=True) / 10000.0,
            "roll_rad": int.from_bytes(data[5:7], "big", signed=True) / 10000.0,
            "yaw_rad": int.from_bytes(data[7:9], "big", signed=True) / 10000.0,
        })
        return out

    if pkt_type == PacketsTypes.FLIGHT_MODE:
        raw_mode = bytes(data[3:-1]).decode("ascii", errors="ignore")
        clean = []
        for ch in raw_mode:
            if ch == "\x00":
                break
            if ch in ("*", " "):
                break
            if ch.isalpha() or ch.isdigit() or ch in ("!", "_"):
                clean.append(ch)
            else:
                break
        mode = "".join(clean)
        out.update({
            "type": "FLIGHT_MODE",
            "mode": mode,
            "raw_mode": raw_mode
        })
        return out

    if pkt_type == PacketsTypes.BATTERY_SENSOR:
        out.update({
            "type": "BATTERY_SENSOR",
            "vbat_v": int.from_bytes(data[3:5], "big", signed=True) / 10.0,
            "current_a": int.from_bytes(data[5:7], "big", signed=True) / 10.0,
            "mah": (data[7] << 16) | (data[8] << 8) | data[9],
            "pct": data[10],
        })
        return out

    if pkt_type == PacketsTypes.BARO_ALT:
        out.update({
            "type": "BARO_ALT",
            "alt_m": int.from_bytes(data[3:7], "big", signed=True) / 100.0,
        })
        return out

    if pkt_type == PacketsTypes.DEVICE_INFO:
        out.update({
            "type": "DEVICE_INFO",
            "raw": data.hex()
        })
        return out

    if pkt_type == PacketsTypes.GPS:
        out.update({
            "type": "GPS",
            "lat": int.from_bytes(data[3:7], "big", signed=True) / 1e7,
            "lon": int.from_bytes(data[7:11], "big", signed=True) / 1e7,
            "gspd_ms": int.from_bytes(data[11:13], "big", signed=True) / 36.0,
            "hdg_deg": int.from_bytes(data[13:15], "big", signed=True) / 100.0,
            "alt_m": int.from_bytes(data[15:17], "big", signed=True) - 1000,
            "sats": data[17],
        })
        return out

    if pkt_type == PacketsTypes.VARIO:
        out.update({
            "type": "VARIO",
            "vspd_ms": int.from_bytes(data[3:5], "big", signed=True) / 10.0,
        })
        return out

    if pkt_type == PacketsTypes.RC_CHANNELS_PACKED:
        payload = data[3:-1]  # type already at [2], CRC at [-1]
        ch_crsf = unpack_rc_channels(payload)
        out.update({
            "type": "RC_CHANNELS_PACKED",
            "ch_crsf": ch_crsf,
            "ch_us": [crsf_to_us(v) for v in ch_crsf],
        })
        return out

    out.update({"type": "UNKNOWN", "raw": data.hex()})
    if verbose:
        print(f"Telemetry UNKNOWN type=0x{ptype:02x} raw={data.hex()}")
    return out

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', default='0.0.0.0', required=False, help="UDP bind host for RC packets")
    parser.add_argument('--port', type=int, default=60000, required=False, help="UDP port for RC packets")
    parser.add_argument('--device', default='/dev/ttyUSB0', required=False, help="Serial device")
    parser.add_argument('--baud', type=int, default=115200, required=False, help="Serial device baudrate") #921600
    parser.add_argument('--tx_rate', type=float, default=100.0, help="RC frame rate Hz")
    parser.add_argument('--loop_hz', type=float, default=250.0, help="Max main loop rate Hz")
    parser.add_argument('--failsafe_time_ms', type=int, default=1000, help="Enter failsafe after this")
    parser.add_argument('--failsafe_channels_us', nargs=CHANNEL_COUNT, type=int, default=FAILSAFE_DEFAULT_US, help="Failsafe channels in microseconds (16 values, space-separated)")
    parser.add_argument('--telemetry_udp', help="Send raw CRSF telemetry frames to udp://host:port (e.g. MWP)")
    parser.add_argument('--debug', action='store_true', help="Verbose RC/telemetry logging")
    args = parser.parse_args()

    HOST = args.host
    PORT = args.port
    tx_rate = float(args.tx_rate)
    loop_hz = float(args.loop_hz)
    loop_period = 1.0 / loop_hz
    failsafe_time_ms = int(args.failsafe_time_ms)

    # RC channel state (microseconds). Initialize throttle and arm low.
    channels_us = [MID_US] * CHANNEL_COUNT
    channels_us[2] = MIN_US   # throttle
    channels_us[4] = ARM_LOW_US   # arm

    failsafe_us = list(args.failsafe_channels_us)

    print(f"Listening for UDP RC on {HOST}:{PORT}")

    # Open serial nonblocking-ish
    ser = serial.Serial(args.device, args.baud, timeout=0, write_timeout=WRITE_TIMEOUT_S)
    input_buf = bytearray()

    # UDP socket for RC input
    rc_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rc_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rc_sock.bind((HOST, PORT))
    rc_sock.setblocking(False)

    # Optional UDP socket for telemetry (raw CRSF frames)
    tele_sock = None
    tele_target = None
    if args.telemetry_udp is not None:
        if ":" not in args.telemetry_udp:
            raise ValueError("telemetry_udp must be host:port")
        host, port_s = args.telemetry_udp.rsplit(":", 1)
        tele_target = (host, int(port_s))
        tele_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        tele_sock.setblocking(False)
        print(f"Telemetry UDP target {tele_target}")
        if args.debug:
            print("Debug telemetry forwarding enabled")

    last_rc_sender = None
    last_rc_update_t = 0.0
    last_tx_t = 0.0
    last_valid_channels_us = None

    try:
        while True:
            loop_start = time.time()
            now = time.time()

            # Read RC updates from UDP (drain socket, keep latest)
            while True:
                try:
                    data, sender = rc_sock.recvfrom(128)
                except (BlockingIOError, InterruptedError):
                    break
                if len(data) != UDP_PACKET_LEN:
                    print(f"Bad RC packet length {len(data)} from {sender}")
                    continue
                payload = data[:UDP_PAYLOAD_LEN]
                crc_rx = struct.unpack_from("<I", data, UDP_PAYLOAD_LEN)[0]
                crc_calc = zlib.crc32(payload) & 0xFFFFFFFF
                if crc_calc != crc_rx:
                    print(f"CRC mismatch from {sender}: got {crc_rx:08x} expected {crc_calc:08x}")
                    continue
                _ts_ms = struct.unpack_from("<I", payload, 0)[0]
                ch = list(struct.unpack_from("<16H", payload, 4))
                if sender != last_rc_sender:
                    print(f"Receiving RC from {sender}")
                channels_us = ch
                last_valid_channels_us = ch
                last_rc_update_t = now
                last_rc_sender = sender

            # Read from serial
            try:
                waiting = ser.in_waiting
            except OSError:
                waiting = 0
            if waiting > 0:
                chunk = ser.read(waiting)
                if chunk.startswith(b"$X"):
                    # Some devices prepend 8 bytes of junk
                    chunk = chunk[8:]
                input_buf.extend(chunk)
                #if args.debug:
                #    print(f"Serial read {len(chunk)} bytes (buffer {len(input_buf)})")

            # Parse CRSF frames from buffer
            while len(input_buf) > 2:
                expected_len = input_buf[1] + 2  # length includes type..crc, plus sync+len here
                if expected_len > 64 or expected_len < 4:
                    # Malformed, flush
                    input_buf = bytearray()
                    break
                if len(input_buf) < expected_len:
                    break
                frame = bytes(input_buf[:expected_len])
                del input_buf[:expected_len]
                if not crsf_validate_frame(frame):
                    print("crc error:", frame.hex())
                    continue
                if tele_sock is not None:
                    try:
                        tele_sock.sendto(frame, tele_target)
                        if args.debug:
                            print(f"Sent telemetry frame len={len(frame)} type=0x{frame[2]:02x} to {tele_target}")
                    except BlockingIOError:
                        pass
                pkt = handleCrsfPacket(frame[2], frame, verbose=args.debug)
                if args.debug and pkt is not None:
                    ptype = pkt.get("type")
                    if ptype == "LINK_STATISTICS":
                        print(f"TEL LINK rssi1={pkt['rssi1']} rssi2={pkt['rssi2']} lq={pkt['lq']} snr={pkt['snr']} pwr={pkt['power']} down_rssi={pkt['downlink_rssi']} down_lq={pkt['downlink_lq']} down_snr={pkt['downlink_snr']}")
                    elif ptype == "ATTITUDE":
                        print(f"TEL ATTI p={pkt['pitch_rad']:.3f} r={pkt['roll_rad']:.3f} y={pkt['yaw_rad']:.3f}")
                    elif ptype == "FLIGHT_MODE":
                        print(f"TEL MODE {pkt['mode']} raw='{pkt.get('raw_mode','')}'")
                    elif ptype == "BATTERY_SENSOR":
                        print(f"TEL BATT {pkt['vbat_v']:.1f}V {pkt['current_a']:.1f}A {pkt['mah']}mAh {pkt['pct']}%")
                    elif ptype == "GPS":
                        print(f"TEL GPS lat={pkt['lat']:.6f} lon={pkt['lon']:.6f} alt={pkt['alt_m']}m gspd={pkt['gspd_ms']:.2f}m/s hdg={pkt['hdg_deg']:.2f} sats={pkt['sats']}")
                    elif ptype == "VARIO":
                        print(f"TEL VARIO vspd={pkt['vspd_ms']:.1f}m/s")
                    elif ptype == "BARO_ALT":
                        print(f"TEL BARO alt={pkt['alt_m']:.2f}m")
                    elif ptype == "RC_CHANNELS_PACKED":
                        print(f"TEL RC ch0-3={pkt['ch_us'][:4]} ch4-7={pkt['ch_us'][4:8]}")
                    #elif ptype == "RADIO_ID": # skip printing
                    #    print(f"TEL RADIO_ID {pkt['raw']}")
                    elif ptype == "DEVICE_INFO":
                        print(f"TEL DEVICE_INFO {pkt['raw']}")

            # Determine which channels to send
            if (now - last_tx_t) >= (1.0 / tx_rate):
                elapsed_ms = (now - last_rc_update_t) * 1000.0
                if last_valid_channels_us is None:
                    active = failsafe_us
                elif elapsed_ms >= failsafe_time_ms:
                    active = failsafe_us
                else:
                    active = last_valid_channels_us
                try:
                    frame = channelsUsToPacket(active)
                    ser.write(frame)
                except serial.SerialTimeoutException as e:
                    print(f"Serial write timeout port={args.device} baud={args.baud} frame_len={len(frame)} elapsed_ms={elapsed_ms:.1f} err={e}")
                last_tx_t = now

            loop_elapsed = time.time() - loop_start
            if loop_elapsed < loop_period:
                time.sleep(loop_period - loop_elapsed)

    except KeyboardInterrupt:
        print("Shutdown requested.")
    finally:
        try:
            rc_sock.close()
        except Exception:
            pass
        try:
            ser.close()
        except Exception:
            pass
        if tele_sock is not None:
            try:
                tele_sock.close()
            except Exception:
                pass

if __name__ == "__main__":
    main()
