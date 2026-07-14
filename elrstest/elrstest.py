#!/usr/bin/env python3
"""Headless ExpressLRS/CRSF bench tester.

  python3 elrstest.py identify                 # which port is what
  python3 elrstest.py smoke                    # plug in, see what works
  python3 elrstest.py rf-sweep                 # sweep packet rates and RF bands
  python3 elrstest.py rx monitor               # live CRSF decode from the receiver
  python3 elrstest.py rx devices               # DEVICE_PING over the RX UART
  python3 elrstest.py rx params                # full Lua parameter dump
  python3 elrstest.py rx get "Tlm Power"
  python3 elrstest.py rx set "Tlm Power" 25
  python3 elrstest.py rx command "Enter Bind Mode" --confirm
  python3 elrstest.py tx probe                 # reset TX, read boot banner (un-wedges too)
  python3 elrstest.py tx monitor|devices|params|get|set|command ...
                                               # same as rx, over the handset port (needs wiring)
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

from elrstest.config import TestConfig, load_config
from elrstest.crsf import (
    CRSF_ADDRESS_RECEIVER,
    CRSF_ADDRESS_TRANSMITTER,
    FrameType,
    describe_frame,
)
from elrstest.flashprobe import flash_probe
from elrstest.link import (
    HandsetSession,
    ParameterClient,
    RxLink,
    SerialPort,
    parameter_text,
    sniff_crsf,
)
from elrstest.smoke import run_smoke
from elrstest.sweep import run_rf_sweep
from elrstest.teelog import tee_output

DEFAULT_CONFIG = Path(__file__).resolve().parent / "elrstest.ini"


def cmd_identify(config: TestConfig) -> int:
    from serial.tools import list_ports
    candidates = [port.device for port in list_ports.comports() if port.vid is not None]
    if not candidates:
        print("no USB serial ports found")
        return 2
    for device in sorted(candidates):
        total, counts = sniff_crsf(device, config.rx_baud, seconds=1.5)
        if total:
            names = {FrameType(t).name if t in FrameType._value2member_map_ else hex(t): n
                     for t, n in counts.items()}
            print(f"{device}: CRSF at {config.rx_baud} baud ({total} frames: {names}) -> receiver")
        else:
            print(f"{device}: silent at {config.rx_baud} baud -> TX flash port, an ELRS RX "
                  "idle after a lost link, unpowered, or wrong baud")
    print("\nNote: both CP2102s report identical serial numbers; if ports swap after replug, "
          "use /dev/serial/by-path/ names in elrstest.ini")
    return 0


def monitor(port: SerialPort, session: HandsetSession | None, seconds: float, show_hex: bool) -> None:
    started = time.monotonic()
    print(f"monitor device={port.device} baud={port.baud} (Ctrl+C to stop)", flush=True)
    link_count = 0
    sync_count = 0
    try:
        while seconds <= 0 or time.monotonic() - started < seconds:
            frames = session.poll() if session else port.read_frames()
            for frame in frames:
                if frame.type == FrameType.LINK_STATISTICS:
                    link_count += 1
                    if link_count % 10 != 1:
                        continue
                if frame.type == FrameType.RADIO_ID:
                    sync_count += 1
                    if sync_count % 100 != 1:
                        continue
                stamp = time.monotonic() - started
                extra = f" raw={frame.raw.hex()}" if show_hex else ""
                print(f"{stamp:8.3f} {describe_frame(frame)}{extra}", flush=True)
            time.sleep(0.001)
    except KeyboardInterrupt:
        pass
    print(f"summary bytes_read={port.bytes_read} crc_errors={port.parser.crc_errors} "
          f"bytes_discarded={port.parser.bytes_discarded}"
          + (f" rc_frames_sent={session.rc_frames_sent} sync_count={session.sync_count}"
             if session else ""), flush=True)


def run_endpoint_command(args: argparse.Namespace, config: TestConfig) -> int:
    if args.endpoint == "rx":
        device, baud = config.rx_port, config.rx_baud
        target_address = CRSF_ADDRESS_RECEIVER
    else:
        if not config.tx_handset_port:
            print("tx handset_port is not configured in elrstest.ini — the stock layout has "
                  "no CRSF on USB. See README 'TX module' for the hardware.json override, "
                  "or use 'tx probe'.")
            return 2
        device, baud = config.tx_handset_port, config.tx_handset_baud
        target_address = CRSF_ADDRESS_TRANSMITTER
    if args.device:
        device = args.device
    if args.baud:
        baud = args.baud

    with SerialPort(device, baud) as port:
        if args.endpoint == "rx":
            transport = RxLink(port)
        else:
            transport = HandsetSession(port)
            deadline = time.monotonic() + config.timeout_seconds
            while time.monotonic() < deadline and not transport.synced:
                transport.poll()
                time.sleep(0.001)
            if not transport.synced:
                print(f"warning: no RADIO_ID sync from TX module within {config.timeout_seconds}s",
                      flush=True)

        if args.action == "monitor":
            monitor(port, transport if args.endpoint == "tx" else None, args.seconds, args.hex)
            return 0

        client = ParameterClient(transport, config.parameter_timeout_seconds)
        if args.action == "devices":
            devices = client.discover()
            for address in sorted(devices):
                d = devices[address]
                print(f"address=0x{d.address:02X} name={d.name!r} serial={d.serial!r} "
                      f"hardware=0x{d.hardware_version:08X} software=0x{d.software_version:08X} "
                      f"parameters={d.parameter_count}")
            return 0 if devices else 2

        devices = client.discover()
        device_info = devices.get(target_address)
        if device_info is None:
            print(f"device 0x{target_address:02X} did not answer DEVICE_PING")
            return 2

        if args.action == "params":
            for parameter in client.read_all(device_info):
                print(parameter_text(parameter))
            return 0

        parameter = client.find(device_info, args.parameter)
        if args.action == "get":
            print(parameter_text(parameter))
            return 0
        if args.action == "set":
            result = client.write(target_address, parameter, args.value)
            print(f"old={result.old_value!r} verified={int(result.verified)}")
            print(parameter_text(result.parameter))
            return 0 if result.verified else 2
        if args.action == "command":
            result = client.command(target_address, parameter, args.confirm)
            print(parameter_text(result))
            return 0
    raise AssertionError(args.action)


def main() -> int:
    tee_output(__file__)
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("identify", help="sniff each USB serial port and say what it is")
    sub.add_parser("smoke", help="run every check the current wiring allows")
    sub.add_parser("rf-sweep", help="sweep every advertised packet rate and RF band")

    for endpoint in ("rx", "tx"):
        ep = sub.add_parser(endpoint, help=f"talk to the {endpoint.upper()} endpoint")
        actions = ep.add_subparsers(dest="action", required=True)
        if endpoint == "tx":
            actions.add_parser("probe", help="reset the module and read its boot banner")
        mon = actions.add_parser("monitor")
        mon.add_argument("--seconds", type=float, default=0)
        mon.add_argument("--hex", action="store_true")
        actions.add_parser("devices")
        actions.add_parser("params")
        get_p = actions.add_parser("get")
        get_p.add_argument("parameter")
        set_p = actions.add_parser("set")
        set_p.add_argument("parameter")
        set_p.add_argument("value")
        cmd_p = actions.add_parser("command")
        cmd_p.add_argument("parameter")
        cmd_p.add_argument("--confirm", action="store_true")
        for action_parser in actions.choices.values():
            action_parser.add_argument("--device")
            action_parser.add_argument("--baud", type=int)
        ep.set_defaults(endpoint=endpoint)

    args = parser.parse_args()
    config = load_config(args.config)

    if args.command == "identify":
        return cmd_identify(config)
    if args.command == "smoke":
        report = run_smoke(config)
        return 2 if report.failed else 0
    if args.command == "rf-sweep":
        report = run_rf_sweep(config)
        return 2 if report.failed else 0
    if args.command == "tx" and args.action == "probe":
        result = flash_probe(args.device or config.tx_flash_port)
        print(result.summary())
        if result.text:
            print("--- boot output ---")
            print(result.text)
        return 0 if result.app_booting else 2
    return run_endpoint_command(args, config)


if __name__ == "__main__":
    raise SystemExit(main())
