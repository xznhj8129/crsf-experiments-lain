# crsfproxy

Bridge between a TCP RC client and a CRSF serial port. Sends RC_CHANNELS_PACKED to the radio and forwards telemetry back to the client (and optionally via UDP msgpack).

## Run

```bash

# In one terminal
python crsfproxy.py --device /dev/ttyUSB0 --baud 420000 --port 8099 --loop_hz 250 --tx_rate 100

# In another: send RC (gamepad example)
python gamepad_to_crsf.py --target 127.0.0.1 --id tx1 --rate 75
```

# Virtual pair for SITL or testing
socat -d -d pty,raw,echo=0 pty,raw,echo=0

## Message format

Client -> proxy: newline-delimited JSON  
`{"id":"tx1","ch":[us...], "t": unix_ms}`  
`ch` may be shorter than 16; values are microseconds. Missing channels stay unchanged.

Telemetry (proxy -> client): newline-delimited JSON objects with parsed CRSF packets. If `--udp_broadcast` is set, the same telemetry is emitted as msgpack blobs to `--udp_addr:--udp_port`.

## Notes

- Single TCP client per proxy instance; run multiple proxies on different ports/devices for parallel links.
- Loop limiter: `--loop_hz` caps main loop CPU. RC send rate is `--tx_rate`.
- Failsafe: if no RC arrives for `--failsafe_time_ms`, throttle (ch4) and arm (ch5) forced low. Default RC init sets throttle/arm low.
