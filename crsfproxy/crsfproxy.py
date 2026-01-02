#!/usr/bin/env python3
import argparse
import json
import socket
import time
from enum import IntEnum

import msgpack
import serial

"""
CRSF TX side bridge:
- Listens on a TCP port for JSON lines: {"id":"your_id","ch":[us...], "t":unix_ms}
  ch is 1..16 PWM in microseconds (988..2012 typical). Missing channels are left unchanged.
- Converts to CRSF RC_CHANNELS_PACKED and writes to the serial CRSF port at tx_rate.
- Parses inbound CRSF telemetry.

Telemetry output:
- TCP client: JSON lines
- Optional UDP broadcast: msgpack blobs to --udp_addr:--udp_port containing:
  {"id": my_id, "tele_packets":[{...}, ...], "t": unix_ms, "crc": 0}

Failsafe:
- If no RC update received for failsafe_time_ms, throttle and arm are forced low.
"""

CRSF_SYNC = 0xC8

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
    if len(channels) != 16:
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
    if len(us_channels) != 16:
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
            if len(ch) == 16:
                break
    if len(ch) != 16:
        ch += [172] * (16 - len(ch))
    return ch

def handleCrsfPacket(ptype: int, data: bytes):
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
        out.update({
            "type": "FLIGHT_MODE",
            "mode": bytes(data[3:-1]).decode("ascii", errors="ignore").rstrip("\x00")
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
    return out

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--id', default='tx1', required=False, help="My ID")
    parser.add_argument('--host', default='localhost', required=False, help="Socket bind host")
    parser.add_argument('--port', type=int, default=8099, required=False, help="Socket port")
    parser.add_argument('--device', default='/dev/ttyUSB0', required=False, help="Serial device")
    parser.add_argument('--baud', type=int, default=921600, required=False, help="Serial device baudrate") #921600
    parser.add_argument('--tx_rate', type=float, default=100.0, help="RC frame rate Hz")
    parser.add_argument('--loop_hz', type=float, default=250.0, help="Max main loop rate Hz")
    parser.add_argument('--failsafe_time_ms', type=int, default=1000, help="Enter failsafe after this")
    parser.add_argument('--telebuffer', type=int, default=32, help="Telemetry packets to batch per send")
    parser.add_argument('--udp_broadcast', action='store_true', help="Enable UDP broadcast for telemetry (msgpack)")
    parser.add_argument('--udp_addr', default='255.255.255.255', help="UDP broadcast address")
    parser.add_argument('--udp_port', type=int, default=8098, help="UDP broadcast port")
    args = parser.parse_args()

    my_id = args.id
    HOST = args.host
    PORT = args.port
    tx_rate = float(args.tx_rate)
    loop_hz = float(args.loop_hz)
    loop_period = 1.0 / loop_hz
    failsafe_time_ms = int(args.failsafe_time_ms)
    telebuffer = int(args.telebuffer)

    # RC channel state (microseconds). Initialize throttle and arm low.
    channels_us = [1500] * 16
    channels_us[3] = 900   # throttle
    channels_us[4] = 900   # arm

    failsafe_us = channels_us.copy() 
    failsafe_us[3] = 900 # ARE YOU SURE ABOUT THAT
    failsafe_us[4] = 900

    print(f"Listening for connections on {HOST}:{PORT}")

    # Open serial nonblocking-ish
    ser = serial.Serial(args.device, args.baud, timeout=0)
    input_buf = bytearray()

    # Server socket
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((HOST, PORT))
    srv.listen()
    srv.settimeout(0.1)

    # Optional UDP broadcast socket for telemetry (msgpack)
    udp_sock = None
    udp_target = None
    if args.udp_broadcast:
        udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        udp_sock.setblocking(False)
        udp_target = (args.udp_addr, args.udp_port)
        print(f"UDP telemetry broadcast enabled to {udp_target}")

    conn = None
    addr = None
    net_rx_buf = ""  # text buffer for newline-delimited JSON
    last_rc_update_t = 0.0
    last_tx_t = 0.0
    last_tele_send_t = 0.0
    tele_packets = []

    ACK = b"\x06"  # single byte ack

    try:
        while True:
            loop_start = time.time()
            now = time.time()

            # Accept new client if none connected
            if conn is None:
                try:
                    conn, addr = srv.accept()
                    conn.setblocking(False)
                    print(f"Connected by {addr}")
                    net_rx_buf = ""
                except socket.timeout:
                    pass
                except BlockingIOError:
                    pass

            # Read from net
            if conn is not None:
                try:
                    data = conn.recv(4096)
                    if data:
                        net_rx_buf += data.decode("utf-8", errors="ignore")
                        # process complete JSON lines
                        while '\n' in net_rx_buf:
                            line, net_rx_buf = net_rx_buf.split('\n', 1)
                            line = line.strip()
                            if not line:
                                continue
                            try:
                                cmd = json.loads(line)
                                # Expect {"id":"..","ch":[...],"t":...}
                                if "ch" in cmd:
                                    arr = cmd["ch"]
                                    if not isinstance(arr, list):
                                        raise ValueError("ch must be a list")
                                    # Update channels; allow short arrays to patch leading channels
                                    for i, v in enumerate(arr):
                                        if i >= 16:
                                            break
                                        if v is None:
                                            continue
                                        channels_us[i] = int(v)
                                    last_rc_update_t = now
                                # respond ack for each message
                                try:
                                    conn.sendall(ACK)
                                except BrokenPipeError:
                                    pass
                            except Exception as e:
                                print(f"Bad JSON from client: {e}")
                    else:
                        # client closed
                        print("Connection closed by client.")
                        conn.close()
                        conn = None
                        addr = None
                except (BlockingIOError, InterruptedError):
                    pass
                except ConnectionResetError:
                    print("Connection reset.")
                    try:
                        conn.close()
                    except Exception:
                        pass
                    conn = None
                    addr = None

            # Read from serial
            try:
                waiting = ser.in_waiting
            except OSError:
                waiting = 0
            if waiting and waiting > 0:
                chunk = ser.read(waiting)
                if chunk.startswith(b"$X"):
                    # Some devices prepend 8 bytes of junk
                    chunk = chunk[8:]
                input_buf.extend(chunk)

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
                pkt = handleCrsfPacket(frame[2], frame)
                if pkt is not None:
                    tele_packets.append(pkt)

            # Periodically send batched telemetry
            should_flush = tele_packets and (
                len(tele_packets) >= telebuffer or (now - last_tele_send_t) >= 0.1
            )
            if should_flush:
                out = {
                    "id": my_id,
                    "tele_packets": tele_packets[:telebuffer],
                    "t": int(now * 1000),
                    "crc": 0,
                }

                # TCP client: JSON lines
                if conn is not None:
                    try:
                        conn.sendall((json.dumps(out) + "\n").encode("utf-8"))
                    except (BrokenPipeError, ConnectionResetError):
                        print("Connection lost while sending telemetry.")
                        try:
                            conn.close()
                        except Exception:
                            pass
                        conn = None
                        addr = None

                # UDP broadcast: msgpack
                if udp_sock is not None:
                    try:
                        payload = msgpack.packb(out, use_bin_type=True)
                        udp_sock.sendto(payload, udp_target)
                    except Exception as e:
                        print(f"UDP send error: {e}")

                # drop sent items
                tele_packets = tele_packets[telebuffer:]
                last_tele_send_t = now

            # Determine which channels to send
            if (now - last_tx_t) >= (1.0 / tx_rate):
                elapsed_ms = (now - last_rc_update_t) * 1000.0
                if last_rc_update_t == 0.0 or elapsed_ms >= failsafe_time_ms:
                    active = failsafe_us
                else:
                    active = channels_us
                try:
                    ser.write(channelsUsToPacket(active))
                except Exception as e:
                    print(f"Serial write error: {e}")
                last_tx_t = now

            loop_elapsed = time.time() - loop_start
            if loop_elapsed < loop_period:
                time.sleep(loop_period - loop_elapsed)

    except KeyboardInterrupt:
        print("Shutdown requested.")
    finally:
        try:
            if conn is not None:
                conn.close()
        except Exception:
            pass
        try:
            srv.close()
        except Exception:
            pass
        try:
            ser.close()
        except Exception:
            pass
        if udp_sock is not None:
            try:
                udp_sock.close()
            except Exception:
                pass

if __name__ == "__main__":
    main()
