# crsf-experiments-lain

CRSF/ExpressLRS experiments.

- [`crsfproxy`](https://github.com/xznhj8129/crsfproxy) - UDP RC/telemetry proxy with remote
  ELRS Lua configuration and one-shot or curses clients
- [`elrstest`](https://github.com/xznhj8129/elrstest) - headless ELRS bench test suite and
  build→define→flash→verify pipeline. Plug TX and RX into USB, get PASS/FAIL
  for CRSF, link, RC, telemetry, and Lua parameters; flash fork builds with
  per-unit defines without touching a button.
- [`esp32-crsf-duplex`](https://github.com/xznhj8129/esp32-crsf-duplex) - Conversion to-from Half-Duplex (S.Port CRSF pin) to Full Duplex (RX/TX) in software
- `crsfrecorder/` - telemetry recording
- `crsf_to_msp/` - CRSF to MSP bridging
- `udp_crsf/` - Early ESP32 bridging experiments, see CRSFproxy
