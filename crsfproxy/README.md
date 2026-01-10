# crsfproxy

Bridge between a UDP RC client (udp_crsf format) and a CRSF serial port. Sends RC_CHANNELS_PACKED to the radio and can forward raw telemetry frames over UDP (for MWP).

## Run

```bash

# In one terminal (telemetry streamed raw to MWP on :40042)
python crsfproxy.py --device /dev/ttyUSB0 --baud 420000 --host 0.0.0.0 --port 60000 --loop_hz 250 --tx_rate 100 --telemetry_udp 127.0.0.1:40042

# MWP example (listen for UDP telemetry)
mwp -d udp://:40042 -a

# RC source (joystick example)
python joystick_crsf.py --target 127.0.0.1 --port 60000 --rate 75
```

# Virtual pair for SITL or testing
socat -d -d pty,raw,echo=0 pty,raw,echo=0

## Message format

Client -> proxy (RC): 40-byte little-endian UDP payload  
`uint32 t_ms | 16 x uint16 channels in microseconds | uint32 crc32(payload)`  
CRC32 covers the first 36 bytes.

Telemetry (proxy -> client): raw CRSF frames sent unchanged to `--telemetry_udp` (host:port).

## Notes

- Loop limiter: `--loop_hz` caps main loop CPU. RC send rate is `--tx_rate`.
- Failsafe (proxy): if RC updates stop for < `--failsafe_time_ms`, repeat last valid channels. If RC updates stop for >= `--failsafe_time_ms`, send `--failsafe_channels_us` (16 values; default: `1500,1500,900,1500,900,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500`). Radio link failsafe (TX<->RX loss) is handled by the receiver, not this proxy. Default RC init sets throttle/arm low.
