# crsf-experiments-lain

CRSF/ExpressLRS experiments.

- [`elrstest`](https://github.com/xznhj8129/elrstest) — headless ELRS bench test suite and
  build→define→flash→verify pipeline. Plug TX and RX into USB, get PASS/FAIL
  for CRSF, link, RC, telemetry, and Lua parameters; flash fork builds with
  per-unit defines without touching a button.
- [`crsfproxy`](https://github.com/xznhj8129/crsfproxy) — UDP RC/telemetry proxy with remote
  ELRS Lua configuration and one-shot or curses clients
- `crsfrecorder/` — telemetry recording
- `crsf_to_msp/` — CRSF to MSP bridging
- `esp32-crsf-duplex/`, `udp_crsf/` — ESP32 bridging experiments
