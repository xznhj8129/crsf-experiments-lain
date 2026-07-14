#!/usr/bin/env python3
"""Set live ExpressLRS TX parameters while supplying handset RC traffic.

  python3 config.py tx --link-mode Normal "RF Band=2.4GHz" \
    "Packet Rate=333Hz Full(-105dBm)" "Telem Ratio=1:2"

Link Mode defaults to Normal and is written first. Remaining assignments are
applied in command-line order, and every write is verified by reading it back
from the TX module.
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

from elrstest.config import load_config
from elrstest.crsf import CRSF_ADDRESS_TRANSMITTER
from elrstest.link import HandsetSession, ParameterClient, SerialPort, parameter_text


DEFAULT_CONFIG = Path(__file__).resolve().parent / "elrstest.ini"
TX_TRAFFIC_LOG = Path(__file__).resolve().parent / "tx.log"
POST_WRITE_RC_SECONDS = 1.0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--baud", type=int)
    parser.add_argument("--link-mode", choices=("Normal", "MAVLink"), default="Normal")
    parser.add_argument("port", help="tx for [tx].handset_port, or an explicit device path")
    parser.add_argument("settings", nargs="*", metavar="PARAMETER=VALUE")
    args = parser.parse_args()
    config = load_config(args.config)
    device = config.tx_handset_port if args.port == "tx" else args.port
    baud = args.baud if args.baud is not None else config.tx_handset_baud
    settings = [f"Link Mode={args.link_mode}"] + args.settings

    TX_TRAFFIC_LOG.write_text("", encoding="utf-8")
    with SerialPort(device, baud, TX_TRAFFIC_LOG) as port:
        handset = HandsetSession(port)
        deadline = time.monotonic() + config.timeout_seconds
        while time.monotonic() < deadline and not handset.synced:
            handset.poll()
            time.sleep(0.001)
        if not handset.synced:
            raise TimeoutError(
                f"device={device} baud={baud} "
                f"RADIO_ID timeout_seconds={config.timeout_seconds}")

        client = ParameterClient(handset, config.parameter_timeout_seconds)
        tx = client.discover()[CRSF_ADDRESS_TRANSMITTER]
        print(f"tx={tx.name!r} address=0x{tx.address:02X} device={device} baud={baud} "
              f"rc_rate_hz={1 / handset.interval:.1f} link_mode={args.link_mode!r} "
              f"settings={len(settings)}",
              flush=True)

        for assignment in settings:
            name, value = assignment.split("=", 1)
            parameter = client.find(tx, name)
            result = client.write(tx.address, parameter, value)
            handset.run(POST_WRITE_RC_SECONDS)
            print(f"parameter={parameter.name!r} requested={value!r} "
                  f"old={result.old_value!r} verified={int(result.verified)} "
                  f"rc_rate_hz={1 / handset.interval:.1f}", flush=True)
            print(parameter_text(result.parameter), flush=True)
            if not result.verified:
                return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
