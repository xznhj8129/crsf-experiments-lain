#!/usr/bin/env python3
# udp_rc_sender_crc.py – example client with CRC32 & timestamp

import argparse, signal, socket, struct, time, zlib

ap = argparse.ArgumentParser()
ap.add_argument("--target", required=True, help="ESP32 IP")
ap.add_argument("--port", type=int, default=60000)
ap.add_argument("--rate", type=float, default=50)
args = ap.parse_args()

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
tgt = (args.target, args.port)
period = 1.0 / args.rate

ch = [1500,1500,1000,1500,1000] + [1500]*11

run = True
signal.signal(signal.SIGINT, lambda *_: globals().update(run=False))
signal.signal(signal.SIGTERM, lambda *_: globals().update(run=False))

start = time.time()
while run:
    # demo: throttle triangle 900-2100
    x = (time.time() - start) % 2.0
    ch[2] = int(900 + abs(x - 1.0)*1200)      # 900–2100

    # clamp
    ch = [max(900, min(2100, v)) for v in ch]

    payload = struct.pack('<I16H', int(time.time()*1000) & 0xFFFFFFFF, *ch)
    crc = zlib.crc32(payload) & 0xFFFFFFFF
    packet = payload + struct.pack('<I', crc)
    sock.sendto(packet, tgt)
    time.sleep(period)

sock.close()
