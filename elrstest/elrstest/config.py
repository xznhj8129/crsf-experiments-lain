"""elrstest.ini loading."""

from __future__ import annotations

import configparser
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class TestConfig:
    rx_port: str
    rx_baud: int
    tx_flash_port: str
    tx_handset_port: str
    tx_handset_baud: int
    timeout_seconds: float
    parameter_timeout_seconds: float
    channel_tolerance_us: int


@dataclass(frozen=True)
class UnitConfig:
    name: str
    role: str  # "tx" or "rx"
    env: str  # platformio environment
    target: str  # targets.json path, e.g. radiomaster.tx_dual.nomad
    port: str  # flash/serial port (by-path preferred)
    flash_baud: int
    rx_baud: int  # runtime CRSF baud (rx role only, patched via --rx-baud)
    domain: str  # regulatory domain for 900M-capable units, e.g. fcc_915 ("" = firmware default)


@dataclass(frozen=True)
class ForkConfig:
    dir: Path
    binding_defines: Path
    units: dict[str, UnitConfig]


def load_config(path: Path) -> TestConfig:
    parser = configparser.ConfigParser()
    with path.open(encoding="utf-8") as config_file:
        parser.read_file(config_file)
    rx = parser["rx"]
    tx = parser["tx"]
    test = parser["test"]
    return TestConfig(
        rx_port=rx["port"],
        rx_baud=rx.getint("baud"),
        tx_flash_port=tx.get("flash_port", "").strip(),
        tx_handset_port=tx.get("handset_port", "").strip(),
        tx_handset_baud=tx.getint("handset_baud", 921600),
        timeout_seconds=test.getfloat("timeout_seconds", 20.0),
        parameter_timeout_seconds=test.getfloat("parameter_timeout_seconds", 3.0),
        channel_tolerance_us=test.getint("channel_tolerance_us", 8),
    )


def load_fork_config(path: Path) -> ForkConfig:
    parser = configparser.ConfigParser()
    with path.open(encoding="utf-8") as config_file:
        parser.read_file(config_file)
    if "fork" not in parser:
        raise KeyError(f"no [fork] section in {path} — add fork dir and [unit.*] sections")
    units = {}
    for section in parser.sections():
        if not section.startswith("unit."):
            continue
        name = section.split(".", 1)[1]
        unit = parser[section]
        units[name] = UnitConfig(
            name=name,
            role=unit["role"],
            env=unit["env"],
            target=unit["target"],
            port=unit["port"],
            flash_baud=unit.getint("flash_baud", 460800),
            rx_baud=unit.getint("rx_baud", 420000),
            domain=unit.get("domain", "").strip(),
        )
    return ForkConfig(
        dir=Path(parser["fork"]["dir"]),
        binding_defines=Path(parser["fork"]["binding_defines"]),
        units=units,
    )
