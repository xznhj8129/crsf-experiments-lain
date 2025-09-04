#!/usr/bin/env python3
import serial
import time
import argparse
from enum import IntEnum
from pathlib import Path
import csv
import json

CRSF_SYNC = 0xC8
connected = False

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
  crc = crc ^ a
  for ii in range(8):
    if crc & 0x80:
      crc = (crc << 1) ^ 0xD5
    else:
      crc = crc << 1
  return crc & 0xFF

def crc8_data(data) -> int:
    crc = 0
    for a in data:
        crc = crc8_dvb_s2(crc, a)
    return crc

def crsf_validate_frame(frame) -> bool:
    return crc8_data(frame[2:-1]) == frame[-1]

def signed_byte(b):
    return b - 256 if b >= 128 else b

def save_telemetry_csv(
    buffer,
    filename,
) -> None:
    fieldnames = ["time", "type", "data"]
    file_exists = Path(filename).is_file()
    with open(filename, mode="a", newline="", encoding="utf-8") as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

        if not file_exists:
            writer.writeheader()

        for packet in buffer:
            timestamp, pkt_type, data_dict = packet
            writer.writerow(
                {
                    "time": timestamp,
                    "type": pkt_type,
                    "data": json.dumps(data_dict, ensure_ascii=False),
                }
            )

def handleCrsfPacket(ptype, data):
    global connected
    pstr = PacketsTypes(ptype).name 
    if ptype == PacketsTypes.RADIO_ID and data[5] == 0x10:
        if not connected:
            print(f"OTX sync")
            connected = True
        else:
            pass
        return [time.time(),pstr,{}]

    elif ptype == PacketsTypes.LINK_STATISTICS:
        rssi1 = signed_byte(data[3])
        rssi2 = signed_byte(data[4])
        lq = data[5]
        snr = signed_byte(data[6])
        antenna = data[7]
        mode = data[8]
        power = data[9]
        # telemetry strength
        downlink_rssi = signed_byte(data[10])
        downlink_lq = data[11]
        downlink_snr = signed_byte(data[12])
        print(f"RSSI={rssi1}/{rssi2}dBm LQ={lq:03} mode={mode} ant={antenna} snr={snr} power={power} drssi={downlink_rssi} dlq={downlink_lq} dsnr={downlink_snr}")
        return [time.time(),pstr,{
            "rssi1": rssi1,
            "rssi2": rssi2,
            "lq": lq,
            "snr": snr,
            "antenna": antenna,
            "mode": mode,
            "power": power,
            "downrssi": downlink_rssi,
            "downlq": downlink_lq,
            "downsnr": downlink_snr,
            }]
    elif ptype == PacketsTypes.ATTITUDE:
        pitch = int.from_bytes(data[3:5], byteorder='big', signed=True) / 10000.0
        roll = int.from_bytes(data[5:7], byteorder='big', signed=True) / 10000.0
        yaw = int.from_bytes(data[7:9], byteorder='big', signed=True) / 10000.0
        print(f"Attitude: Pitch={pitch:0.2f} Roll={roll:0.2f} Yaw={yaw:0.2f} (rad)")
        return [time.time(),pstr,{
            "pitch": pitch,
            "roll": roll,
            "yaw": yaw,
            }]
    elif ptype == PacketsTypes.FLIGHT_MODE:
        packet = ''.join(map(chr, data[3:-2]))
        print(f"Flight Mode: {packet}")
        return [time.time(),pstr,{
            "mode": packet,
            }]
    elif ptype == PacketsTypes.BATTERY_SENSOR:
        vbat = int.from_bytes(data[3:5], byteorder='big', signed=True) / 10.0
        curr = int.from_bytes(data[5:7], byteorder='big', signed=True) / 10.0
        mah = data[7] << 16 | data[8] << 7 | data[9]
        pct = data[10]
        print(f"Battery: {vbat:0.2f}V {curr:0.1f}A {mah}mAh {pct}%")
        return [time.time(),pstr,{
            "vbat": vbat,
            "curr": curr,
            "mah": mah,
            "pct": pct,
            }]
    elif ptype == PacketsTypes.BARO_ALT:
        print(f"BaroAlt: ")
        return [time.time(),pstr,{}]

    elif ptype == PacketsTypes.DEVICE_INFO:
        packet = ' '.join(map(hex, data))
        print(f"Device Info: {packet}")
        return [time.time(),pstr,{
            "info": packet,
            }]

    elif data[2] == PacketsTypes.GPS:
        lat = int.from_bytes(data[3:7], byteorder='big', signed=True) / 1e7
        lon = int.from_bytes(data[7:11], byteorder='big', signed=True) / 1e7
        gspd = int.from_bytes(data[11:13], byteorder='big', signed=True) / 36.0
        hdg =  int.from_bytes(data[13:15], byteorder='big', signed=True) / 100.0
        alt = int.from_bytes(data[15:17], byteorder='big', signed=True) - 1000
        sats = data[17]
        print(f"GPS: Pos={lat} {lon} GSpd={gspd:0.1f}m/s Hdg={hdg:0.1f} Alt={alt}m Sats={sats}")
        return [time.time(),pstr,{
            "lat": lat,
            "lon": lon,
            "gspd": gspd,
            "hdg": hdg,
            "alt": alt,
            "sats": sats,
            }]

    elif ptype == PacketsTypes.VARIO:
        vspd = int.from_bytes(data[3:5], byteorder='big', signed=True) / 100.0
        print(f"VSpd: {vspd:0.1f}m/s")
        return [time.time(),pstr,{
            "vspd": vspd,
        }]
    elif ptype == PacketsTypes.RC_CHANNELS_PACKED:
        print(f"Channels: (data)")
        #pass
        return [time.time(),pstr,{}]
    else:
        packet = ' '.join(map(hex, data))
        print(f"Unknown 0x{ptype:02x}: {packet}")
        return [time.time(),"UNKNOWN",{
            "packet": packet,
        }]

parser = argparse.ArgumentParser()
parser.add_argument('-p', '--port', default='/dev/ttyUSB0', required=False)
parser.add_argument('-b', '--baud', default=115200, required=False)
args = parser.parse_args()
packetc = 20
packett = 5
buffer = []
lastpacket = time.time()
logfile = f"tele_log_{round(time.time())}.csv"

with serial.Serial(args.port, args.baud, timeout=2) as ser:
    input = bytearray()
    try:
        while True:
            if ser.in_waiting > 0:
                a = ser.read(ser.in_waiting)
                #print(a)
                input.extend(a)
            else:
                time.sleep(0.020)

            while len(input) > 2:
                # This simple parser works with malformed CRSF streams
                # it does not check the first byte for SYNC_BYTE, but
                # instead just looks for anything where the packet length
                # is 4-64 bytes, and the CRC validates
                expected_len = input[1] + 2
                if expected_len > 64 or expected_len < 4:
                    input = bytearray()
                elif len(input) >= expected_len:
                    single = input[:expected_len] # copy out this whole packet
                    input = input[expected_len:] # and remove it from the buffer

                    if not crsf_validate_frame(single): # single[-1] != crc:
                        packet = ' '.join(map(hex, single))
                        print(f"crc error: {packet}")
                        t = time.time()
                        pt = "ERROR"
                        dat = {"packet":packet}
                    else:
                        t, pt, dat = handleCrsfPacket(single[2], single)
                    buffer.append([t,pt,dat])
                    lastpacket = time.time()
                else:
                    break


                if len(buffer) >= packetc:
                    save_telemetry_csv(buffer, logfile)
                    buffer = []


            if (time.time()-lastpacket)>packett:
                save_telemetry_csv(buffer, logfile)
                buffer = []
    except:
        save_telemetry_csv(buffer, logfile)
