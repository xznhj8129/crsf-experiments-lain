#!/usr/bin/env python3
"""End-to-end firmware pipeline for the ELRS bench: build -> define -> flash -> verify.

  python3 elrsflash.py build xr1          # pio run the unit's environment
  python3 elrsflash.py patch xr1          # bake target layout + phrase + rx-baud into the built image
  python3 elrsflash.py flash xr1          # upload over USB (RX: 'bl' trigger + in-app stub; TX: auto-reset)
  python3 elrsflash.py verify xr1         # ping the device, print name/version/hash
  python3 elrsflash.py pipeline all       # build+patch+flash+verify every unit, then run the smoke test

The binding phrase is read from the configured binding_defines file and patched
into both firmware images by binary_configurator.py.

Flashing never touches the littlefs partition (0x3d0000), so the Nomad's
hardware.json USB-handset override survives every firmware flash.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import time
from pathlib import Path

from elrstest.config import ForkConfig, UnitConfig, load_config, load_fork_config
from elrstest.crsf import FrameType, make_frame
from elrstest.link import HandsetSession, ParameterClient, RxLink, SerialPort
from elrstest.teelog import tee_output

DEFAULT_CONFIG = Path(__file__).resolve().parent / "elrstest.ini"
CRSF_ADDRESS_RECEIVER = 0xEC
CRSF_ADDRESS_TRANSMITTER = 0xEE


def build_dir(fork: ForkConfig, unit: UnitConfig) -> Path:
    return fork.dir / "src" / ".pio" / "build" / unit.env


def run_logged(cmd: list[str], cwd: Path, timeout: float = 900) -> None:
    """Run a command streaming its output through our stdout (and the tee log)."""
    display_cmd = [str(value) for value in cmd]
    if "--phrase" in display_cmd:
        display_cmd[display_cmd.index("--phrase") + 1] = "<redacted>"
    print(f"+ {' '.join(display_cmd)}", flush=True)
    process = subprocess.Popen(cmd, cwd=cwd, stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT, text=True, errors="replace")
    deadline = time.monotonic() + timeout
    for line in process.stdout:
        print(line, end="", flush=True)
        if time.monotonic() > deadline:
            process.kill()
            raise TimeoutError(f"command exceeded {timeout}s: {cmd[0]}")
    returncode = process.wait()
    if returncode != 0:
        raise RuntimeError(f"command failed with exit {returncode}: {cmd[0]}")


def build(fork: ForkConfig, unit: UnitConfig) -> None:
    pio = Path.home() / ".platformio" / "penv" / "bin" / "pio"
    run_logged([str(pio), "run", "-e", unit.env], cwd=fork.dir / "src")
    firmware = build_dir(fork, unit) / "firmware.bin"
    if not firmware.exists():
        raise FileNotFoundError(firmware)
    print(f"built {firmware}")


def patch(fork: ForkConfig, unit: UnitConfig, flash: bool = False) -> None:
    firmware = build_dir(fork, unit) / "firmware.bin"
    cmd = [sys.executable, "python/binary_configurator.py", str(firmware),
           "--target", unit.target, "--phrase-defines", str(fork.binding_defines)]
    if unit.role == "tx":
        cmd.append("--tx")
    else:
        cmd.extend(["--rx-baud", str(unit.rx_baud),
                    "--lock-on-first-connection" if unit.lock_on_first_connection
                    else "--no-lock-on-first-connection"])
    if unit.domain:
        cmd.extend(["--domain", unit.domain])
    if flash:
        cmd.extend(["--flash", "uart", "--port", unit.port, "--baud", str(unit.flash_baud)])
    run_logged(cmd, cwd=fork.dir / "src")


def trigger_rx_bootloader(unit: UnitConfig) -> None:
    """CRSF COMMAND 'bl' puts a running ELRS RX into its in-app serial-update stub.
    Harmless if the RX is already in the stub (bytes are ignored as SLIP noise)."""
    frame = make_frame(FrameType.COMMAND, b"bl", address=CRSF_ADDRESS_RECEIVER)
    with SerialPort(unit.port, unit.rx_baud) as port:
        port.write(frame)
        deadline = time.monotonic() + 1.5
        said = bytearray()
        while time.monotonic() < deadline:
            waiting = port.serial.in_waiting
            if waiting:
                said += port.serial.read(waiting)
            time.sleep(0.01)
    text = said.decode(errors="replace").strip()
    print(f"bl trigger sent; rx said: {text!r}" if text else
          "bl trigger sent; no banner (already in stub, or quiet)")
    time.sleep(0.3)


def flash_rx_stub(fork: ForkConfig, unit: UnitConfig, firmware: Path) -> None:
    """Flash the app image through the in-app esptool stub (needs the fork's
    patched esptool for its --passthrough flag; plain esptool expects the ROM)."""
    chip = "esp32c3" if "C3" in unit.env else "esp32"
    snippet = (
        "import sys; sys.path.insert(0, 'python/external/esptool'); sys.path.append('python');"
        "from external.esptool import esptool;"
        f"esptool.main(['--passthrough', '--chip', '{chip}', '--port', '{unit.port}',"
        f" '--baud', '{unit.rx_baud}', '--before', 'no_reset', '--after', 'hard_reset',"
        " 'write_flash', '-z', '--flash_mode', 'dio', '--flash_freq', '40m',"
        f" '--flash_size', 'detect', '0x10000', '{firmware}'])"
    )
    run_logged([sys.executable, "-c", snippet], cwd=fork.dir / "src")


def flash(fork: ForkConfig, unit: UnitConfig) -> None:
    if unit.role == "rx":
        patch(fork, unit)
        trigger_rx_bootloader(unit)
        flash_rx_stub(fork, unit, build_dir(fork, unit) / "firmware.bin")
    else:
        patch(fork, unit)
        image_dir = build_dir(fork, unit)
        pio_python = Path.home() / ".platformio" / "penv" / "bin" / "python"
        esptool = Path.home() / ".platformio" / "packages" / "tool-esptoolpy" / "esptool.py"
        run_logged([
            str(pio_python), str(esptool), "--chip", "esp32", "--port", unit.port,
            "--baud", str(unit.flash_baud), "--after", "hard_reset", "write_flash",
            "-z", "--flash_mode", "dio", "--flash_freq", "40m", "--flash_size", "detect",
            "0x1000", str(image_dir / "bootloader.bin"),
            "0x8000", str(image_dir / "partitions.bin"),
            "0xe000", str(image_dir / "boot_app0.bin"),
            "0x10000", str(image_dir / "firmware.bin"),
        ], cwd=fork.dir / "src")


def verify(config, fork: ForkConfig, unit: UnitConfig, settle_seconds: float = 4.0) -> bool:
    time.sleep(settle_seconds)  # let the freshly flashed firmware boot
    if unit.role == "rx":
        target = CRSF_ADDRESS_RECEIVER
        port = SerialPort(config.rx_port, config.rx_baud)
        make_transport = RxLink
    else:
        target = CRSF_ADDRESS_TRANSMITTER
        port = SerialPort(config.tx_handset_port, config.tx_handset_baud)
        make_transport = HandsetSession
    with port:
        transport = make_transport(port)
        client = ParameterClient(transport, config.parameter_timeout_seconds)
        deadline = time.monotonic() + 15
        device = None
        while time.monotonic() < deadline and device is None:
            device = client.discover(seconds=2.0).get(target)
        if device is None:
            print(f"verify {unit.name}: FAIL — no DEVICE_INFO from 0x{target:02X}")
            return False
        version_info = client.read(target, device.parameter_count)
        print(f"verify {unit.name}: {device.name!r} sw=0x{device.software_version:08X} "
              f"parameters={device.parameter_count} build={version_info.name!r} "
              f"hash={version_info.value!r}")
        return True


def main() -> int:
    tee_output(__file__)
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("command", choices=("build", "patch", "flash", "verify", "pipeline"))
    parser.add_argument("unit", help="unit name from [unit.*] sections, or 'all'")
    args = parser.parse_args()

    config = load_config(args.config)
    fork = load_fork_config(args.config)
    if args.unit == "all":
        units = list(fork.units.values())
    elif args.unit in fork.units:
        units = [fork.units[args.unit]]
    else:
        print(f"unknown unit {args.unit!r}; configured: {', '.join(fork.units)}")
        return 2

    failed = False
    for unit in units:
        if args.command == "build":
            build(fork, unit)
        elif args.command == "patch":
            patch(fork, unit)
        elif args.command == "flash":
            flash(fork, unit)
        elif args.command == "verify":
            failed |= not verify(config, fork, unit, settle_seconds=0)
        elif args.command == "pipeline":
            build(fork, unit)
            flash(fork, unit)
            failed |= not verify(config, fork, unit)

    if args.command == "pipeline" and not failed:
        print("\nrunning smoke test:", flush=True)
        from elrstest.smoke import run_smoke
        failed |= run_smoke(config).failed
    return 2 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
