"""Plug-in-and-see smoke test.

Runs every check the current wiring allows and reports PASS/FAIL/SKIP with
reasons, so "does my bench setup work?" is one command. Checks that need the
TX handset connection (RC passthrough, telemetry return, TX parameters) are
skipped with an explanation until a handset port is wired/configured.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from pathlib import Path

from .config import TestConfig
from .crsf import (
    CRSF_ADDRESS_RECEIVER,
    CRSF_ADDRESS_TRANSMITTER,
    FrameType,
    make_extended_frame,
    unpack_channels_us,
)
from .flashprobe import flash_probe
from .link import (
    HandsetSession,
    ParameterClient,
    RxLink,
    SerialPort,
    expected_rx_rc_rate_hz,
)


RX_TRAFFIC_LOG = Path(__file__).resolve().parent.parent / "rx.log"
TX_TRAFFIC_LOG = Path(__file__).resolve().parent.parent / "tx.log"
RC_DROPOUT_INTERVALS = 3.0


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
    RX_TRAFFIC_LOG.write_text("", encoding="utf-8")
    TX_TRAFFIC_LOG.write_text("", encoding="utf-8")

    # 1-3. Receiver answers the Lua parameter protocol on the wire; passive output
    # is informational only (an ELRS RX goes quiet after a lost link, so silence
    # with a working ping is normal).
    rx_device = None
    try:
        with SerialPort(config.rx_port, config.rx_baud, RX_TRAFFIC_LOG) as port:
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
            handset_port = SerialPort(config.tx_handset_port, config.tx_handset_baud,
                                      TX_TRAFFIC_LOG)
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
    telemetry_ratio_name = None
    try:
        client = ParameterClient(handset, config.parameter_timeout_seconds)
        devices = client.discover()
        tx_device = devices.get(CRSF_ADDRESS_TRANSMITTER)
        if tx_device:
            telemetry_ratio = client.find(tx_device, "Telem Ratio")
            telemetry_ratio_name = telemetry_ratio.options[telemetry_ratio.value]
            report.add("tx_device_info", "PASS",
                       f"{tx_device.name!r} parameters={tx_device.parameter_count} "
                       f"telemetry_ratio={telemetry_ratio_name!r}")
        else:
            report.add("tx_device_info", "FAIL", "no DEVICE_INFO from 0xEE")
    except Exception as error:
        report.add("tx_device_info", "FAIL", f"{type(error).__name__}: {error}")

    # 7-9. Full RF link: RX sees link + our channels, TX sees injected telemetry
    try:
        with SerialPort(config.rx_port, config.rx_baud, RX_TRAFFIC_LOG) as rx_port:
            rx = RxLink(rx_port)
            pattern = [1100, 1200, 1300, 1400, 1000] + [1500] * 11
            handset.channels_us = pattern
            started = time.monotonic()
            t_link = t_rc = t_telemetry = None
            rc_seen = None
            lq = None
            mode = None
            rx_counts: dict[int, int] = {}
            tx_counts: dict[int, int] = {}
            battery_frame = rx.inject_battery()
            next_battery = started + 0.2
            deadline = started + config.timeout_seconds
            while time.monotonic() < deadline and not all(
                    value is not None for value in (t_link, t_rc, t_telemetry)):
                for frame in handset.poll():
                    tx_counts[frame.type] = tx_counts.get(frame.type, 0) + 1
                    if frame.type == FrameType.BATTERY_SENSOR and frame.raw == battery_frame \
                            and t_telemetry is None:
                        t_telemetry = time.monotonic() - started
                for frame in rx.poll():
                    rx_counts[frame.type] = rx_counts.get(frame.type, 0) + 1
                    if frame.type == FrameType.LINK_STATISTICS:
                        lq = frame.payload[2]
                        mode = frame.payload[5]
                        if lq > 0 and t_link is None:
                            t_link = time.monotonic() - started
                    if frame.type == FrameType.RC_CHANNELS_PACKED and t_rc is None:
                        channels = unpack_channels_us(frame.payload)
                        rc_seen = channels
                        if all(abs(a - b) <= config.channel_tolerance_us
                               for a, b in zip(channels[:5], pattern[:5])):
                            t_rc = time.monotonic() - started
                if time.monotonic() >= next_battery:
                    battery_frame = rx.inject_battery()
                    next_battery = time.monotonic() + 0.2
                time.sleep(0.001)
            measurement_seconds = 0.0
            radio_id_rate_hz = 1 / handset.interval
            expected_rc_rate_hz = expected_rx_rc_rate_hz(
                radio_id_rate_hz, telemetry_ratio_name)
            measured_rc_frames = 0
            measured_valid_rc_frames = 0
            observed_rc_rate_hz = 0.0
            delivery_percent = 0.0
            rc_dropout_events = 0
            estimated_missing_rc_frames = 0
            longest_rc_gap = 0.0
            link_dropout_events = 0
            tx_connected = None
            status_flags = None
            status_frames = 0
            measurement_passed = False
            if all(value is not None for value in (t_link, t_rc, t_telemetry)):
                dwell_started = time.monotonic()
                dwell_deadline = dwell_started + config.post_link_dwell_seconds
                radio_id_rate_hz = 1 / handset.interval
                expected_rc_rate_hz = expected_rx_rc_rate_hz(
                    radio_id_rate_hz, telemetry_ratio_name)
                expected_interval = 1 / expected_rc_rate_hz
                dropout_threshold = expected_interval * RC_DROPOUT_INTERVALS
                next_status_request = dwell_started
                last_rc_at = None
                link_was_valid = False
                while time.monotonic() < dwell_deadline:
                    now = time.monotonic()
                    if now >= next_status_request:
                        handset.queue(make_extended_frame(
                            FrameType.PARAMETER_WRITE,
                            CRSF_ADDRESS_TRANSMITTER,
                            handset.origin(CRSF_ADDRESS_TRANSMITTER),
                            bytes([0, 0]),
                        ))
                        next_status_request = now + 1.0
                    for frame in handset.poll():
                        tx_counts[frame.type] = tx_counts.get(frame.type, 0) + 1
                        if frame.type == FrameType.ELRS_STATUS \
                                and len(frame.extended_payload) >= 4:
                            status_frames += 1
                            status_flags = frame.extended_payload[3]
                            tx_connected = bool(status_flags & 1)
                    for frame in rx.poll():
                        frame_at = time.monotonic()
                        rx_counts[frame.type] = rx_counts.get(frame.type, 0) + 1
                        if frame.type == FrameType.LINK_STATISTICS:
                            lq = frame.payload[2]
                            mode = frame.payload[5]
                        elif frame.type == FrameType.RC_CHANNELS_PACKED:
                            measured_rc_frames += 1
                            channels = unpack_channels_us(frame.payload)
                            rc_seen = channels
                            valid_channels = all(
                                abs(actual - expected) <= config.channel_tolerance_us
                                for actual, expected in zip(channels[:5], pattern[:5]))
                            if valid_channels:
                                measured_valid_rc_frames += 1
                            if tx_connected and lq and last_rc_at is not None:
                                gap = frame_at - last_rc_at
                                longest_rc_gap = max(longest_rc_gap, gap)
                                if gap > dropout_threshold:
                                    rc_dropout_events += 1
                                    estimated_missing_rc_frames += max(
                                        1, round(gap / expected_interval) - 1)
                            if tx_connected and lq:
                                last_rc_at = frame_at
                    link_is_valid = bool(tx_connected and lq)
                    if link_was_valid and not link_is_valid:
                        link_dropout_events += 1
                    if not link_was_valid and link_is_valid:
                        last_rc_at = time.monotonic()
                    link_was_valid = link_is_valid
                    time.sleep(0.001)
                measurement_seconds = time.monotonic() - dwell_started
                if link_was_valid and last_rc_at is not None:
                    tail_gap = time.monotonic() - last_rc_at
                    longest_rc_gap = max(longest_rc_gap, tail_gap)
                    if tail_gap > dropout_threshold:
                        rc_dropout_events += 1
                        estimated_missing_rc_frames += max(
                            1, round(tail_gap / expected_interval) - 1)
                observed_rc_rate_hz = measured_rc_frames / measurement_seconds
                delivery_percent = measured_rc_frames / (
                    expected_rc_rate_hz * measurement_seconds) * 100
                measurement_passed = (
                    tx_connected is True
                    and bool(lq)
                    and measured_rc_frames > 0
                    and measured_valid_rc_frames == measured_rc_frames
                    and rc_dropout_events == 0
                    and link_dropout_events == 0
                )
            window = f" within {config.timeout_seconds:.0f}s"
            report.add("link_up", "PASS" if t_link is not None else "FAIL",
                       f"RX reports LQ > 0 after {t_link:.1f}s" if t_link is not None else
                       f"RX never reported LQ > 0{window} — modules bound? TX transmitting?")
            report.add("rc_passthrough", "PASS" if t_rc is not None and measurement_passed
                       else "FAIL",
                       f"RX outputs sent pattern after {t_rc:.1f}s {rc_seen[:5]} "
                       f"measurement_s={measurement_seconds:.3f} "
                       f"radio_id_rate_hz={radio_id_rate_hz:.1f} "
                       f"telemetry_ratio={telemetry_ratio_name!r} "
                       f"expected_rx_rc_hz={expected_rc_rate_hz:.1f} "
                       f"observed_rc_hz={observed_rc_rate_hz:.1f} "
                       f"delivery_percent={delivery_percent:.1f} "
                       f"measured_rc_frames={measured_rc_frames} "
                       f"measured_valid_rc_frames={measured_valid_rc_frames} "
                       f"rc_dropout_events={rc_dropout_events} "
                       f"estimated_missing_rc_frames={estimated_missing_rc_frames} "
                       f"longest_rc_gap_ms={longest_rc_gap * 1000:.3f} "
                       f"link_dropout_events={link_dropout_events} lq={lq} mode={mode} "
                       f"tx_connected={tx_connected} status_frames={status_frames} "
                       f"status_flags={status_flags}"
                       if t_rc is not None else
                       f"sent={pattern[:5]} last_rx={rc_seen[:5] if rc_seen else None} "
                       f"rx_types={dict(sorted(rx_counts.items()))}{window}")
            report.add("telemetry_return", "PASS" if t_telemetry is not None else "FAIL",
                       f"injected battery frame back on TX side after {t_telemetry:.1f}s"
                       if t_telemetry is not None else
                       f"battery frame never returned; tx_types={dict(sorted(tx_counts.items()))}{window}")
    except Exception as error:
        for name in ("link_up", "rc_passthrough", "telemetry_return"):
            report.add(name, "FAIL", f"{type(error).__name__}: {error}")
    finally:
        handset.port.close()

    return report
