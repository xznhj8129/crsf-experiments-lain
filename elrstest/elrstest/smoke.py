"""Plug-in-and-see smoke test.

Runs every check the current wiring allows and reports PASS/FAIL/SKIP with
reasons, so "does my bench setup work?" is one command. Checks that need the
TX handset connection (RC passthrough, telemetry return, TX parameters) are
skipped with an explanation until a handset port is wired/configured.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field

from .config import TestConfig
from .crsf import (
    CRSF_ADDRESS_RECEIVER,
    CRSF_ADDRESS_TRANSMITTER,
    FrameType,
    unpack_channels_us,
)
from .flashprobe import flash_probe
from .link import HandsetSession, ParameterClient, RxLink, SerialPort


@dataclass
class Check:
    name: str
    outcome: str  # PASS / FAIL / SKIP
    detail: str = ""

    def line(self) -> str:
        return f"{self.outcome:4s} {self.name}: {self.detail}"


@dataclass
class SmokeReport:
    checks: list[Check] = field(default_factory=list)

    def add(self, name: str, outcome: str, detail: str = "") -> None:
        check = Check(name, outcome, detail)
        self.checks.append(check)
        print(check.line(), flush=True)

    @property
    def failed(self) -> bool:
        return any(check.outcome == "FAIL" for check in self.checks)


def run_smoke(config: TestConfig) -> SmokeReport:
    report = SmokeReport()

    # 1-3. Receiver answers the Lua parameter protocol on the wire; passive output
    # is informational only (an ELRS RX goes quiet after a lost link, so silence
    # with a working ping is normal).
    rx_device = None
    try:
        with SerialPort(config.rx_port, config.rx_baud) as port:
            client = ParameterClient(RxLink(port), config.parameter_timeout_seconds)
            devices = client.discover()
            rx_device = devices.get(CRSF_ADDRESS_RECEIVER)
            if rx_device:
                report.add("rx_device_info", "PASS",
                           f"{rx_device.name!r} sw=0x{rx_device.software_version:08X} "
                           f"parameters={rx_device.parameter_count}")
                parameters = client.read_all(rx_device)
                report.add("rx_parameters", "PASS",
                           f"read {len(parameters)}/{rx_device.parameter_count}: "
                           + ", ".join(p.name for p in parameters if not p.hidden))
            else:
                report.add("rx_device_info", "FAIL",
                           f"no DEVICE_INFO from 0xEC at {config.rx_baud} baud on "
                           f"{config.rx_port} — check wiring, power, and the receiver's "
                           f"baud option (saw {sorted(hex(a) for a in devices)})")
                report.add("rx_parameters", "SKIP", "rx did not answer DEVICE_PING")
            counts: dict[int, int] = {}
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline:
                for frame in port.read_frames():
                    counts[frame.type] = counts.get(frame.type, 0) + 1
                time.sleep(0.002)
            total = sum(counts.values())
            if total:
                detail = (f"{total} frames in 2s "
                          f"({counts.get(FrameType.LINK_STATISTICS, 0)} LINK_STATISTICS)")
            else:
                detail = ("quiet — normal for an ELRS RX after a lost link; "
                          "it beacons before first connection and streams when connected")
            report.add("rx_passive_output", "PASS" if (total or rx_device) else "FAIL", detail)
    except Exception as error:
        report.add("rx_device_info", "FAIL", f"{type(error).__name__}: {error}")
        report.add("rx_parameters", "SKIP", "rx port unavailable")
        report.add("rx_passive_output", "SKIP", "rx port unavailable")

    # 4. TX flash port: module powered and booting (also un-wedges a held-in-reset chip)
    if config.tx_flash_port:
        try:
            probe = flash_probe(config.tx_flash_port)
            report.add("tx_flash_probe", "PASS" if probe.app_booting else "FAIL", probe.summary())
        except Exception as error:
            report.add("tx_flash_probe", "FAIL", f"{type(error).__name__}: {error}")
    else:
        report.add("tx_flash_probe", "SKIP", "no tx flash_port configured")

    # 5. TX handset connection (requires wiring to the JR-bay CRSF pin, see README)
    handset = None
    if not config.tx_handset_port:
        report.add("tx_handset_sync", "SKIP",
                   "no tx handset_port configured — stock layout has no CRSF on USB; "
                   "see README 'TX module' for the hardware.json override or bay-pin wiring")
    else:
        try:
            handset_port = SerialPort(config.tx_handset_port, config.tx_handset_baud)
            handset_port.open()
            handset = HandsetSession(handset_port)
            deadline = time.monotonic() + config.timeout_seconds
            while time.monotonic() < deadline and not handset.synced:
                handset.poll()
                time.sleep(0.001)
            if handset.synced:
                report.add("tx_handset_sync", "PASS",
                           f"RADIO_ID sync, module wants {1 / handset.interval:.1f} Hz RC")
            else:
                report.add("tx_handset_sync", "FAIL",
                           f"no RADIO_ID sync within {config.timeout_seconds}s at "
                           f"{config.tx_handset_baud} baud")
                handset_port.close()
                handset = None
        except Exception as error:
            report.add("tx_handset_sync", "FAIL", f"{type(error).__name__}: {error}")
            handset = None

    if handset is None:
        for name in ("tx_device_info", "link_up", "rc_passthrough", "telemetry_return"):
            report.add(name, "SKIP", "needs tx handset connection")
        return report

    # 6. TX module identity over the handset link
    try:
        client = ParameterClient(handset, config.parameter_timeout_seconds)
        devices = client.discover()
        tx_device = devices.get(CRSF_ADDRESS_TRANSMITTER)
        if tx_device:
            report.add("tx_device_info", "PASS",
                       f"{tx_device.name!r} parameters={tx_device.parameter_count}")
        else:
            report.add("tx_device_info", "FAIL", "no DEVICE_INFO from 0xEE")
    except Exception as error:
        report.add("tx_device_info", "FAIL", f"{type(error).__name__}: {error}")

    # 7-9. Full RF link: RX sees link + our channels, TX sees injected telemetry
    try:
        with SerialPort(config.rx_port, config.rx_baud) as rx_port:
            rx = RxLink(rx_port)
            pattern = [1100, 1200, 1300, 1400, 1000] + [1500] * 11
            handset.channels_us = pattern
            started = time.monotonic()
            t_link = t_rc = t_telemetry = None
            rc_seen = None
            battery_frame = rx.inject_battery()
            next_battery = started + 0.2
            deadline = started + config.timeout_seconds
            while time.monotonic() < deadline and not (t_link and t_rc and t_telemetry):
                for frame in handset.poll():
                    if frame.type == FrameType.BATTERY_SENSOR and frame.raw == battery_frame \
                            and t_telemetry is None:
                        t_telemetry = time.monotonic() - started
                for frame in rx.poll():
                    if frame.type == FrameType.LINK_STATISTICS and frame.payload[2] > 0 \
                            and t_link is None:
                        t_link = time.monotonic() - started
                    if frame.type == FrameType.RC_CHANNELS_PACKED and t_rc is None:
                        channels = unpack_channels_us(frame.payload)
                        if all(abs(a - b) <= config.channel_tolerance_us
                               for a, b in zip(channels[:5], pattern[:5])):
                            t_rc = time.monotonic() - started
                            rc_seen = channels
                if time.monotonic() >= next_battery:
                    battery_frame = rx.inject_battery()
                    next_battery = time.monotonic() + 0.2
                time.sleep(0.001)
            window = f" within {config.timeout_seconds:.0f}s"
            report.add("link_up", "PASS" if t_link else "FAIL",
                       f"RX reports LQ > 0 after {t_link:.1f}s" if t_link else
                       f"RX never reported LQ > 0{window} — modules bound? TX transmitting?")
            report.add("rc_passthrough", "PASS" if t_rc else "FAIL",
                       f"RX outputs sent pattern after {t_rc:.1f}s {rc_seen[:5]}" if t_rc else
                       f"sent channel pattern never appeared on RX UART{window}")
            report.add("telemetry_return", "PASS" if t_telemetry else "FAIL",
                       f"injected battery frame back on TX side after {t_telemetry:.1f}s"
                       if t_telemetry else
                       f"injected battery frame never reached the TX side{window}")
    except Exception as error:
        for name in ("link_up", "rc_passthrough", "telemetry_return"):
            report.add(name, "FAIL", f"{type(error).__name__}: {error}")
    finally:
        handset.port.close()

    return report
