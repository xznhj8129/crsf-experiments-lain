# crsf-experiments-lain

CRSF/ExpressLRS experiments.

- [`elrstest/`](elrstest/README.md) — headless ELRS bench test suite and
  build→define→flash→verify pipeline. Plug TX and RX into USB, get PASS/FAIL
  for CRSF, link, RC, telemetry, and Lua parameters; flash fork builds with
  per-unit defines without touching a button.
- `crsfproxy/` — CRSF proxy + joystick experiments
- `crsfrecorder/` — telemetry recording
- `crsf_to_msp/` — CRSF to MSP bridging
- `esp32-crsf-duplex/`, `udp_crsf/` — ESP32 bridging experiments
