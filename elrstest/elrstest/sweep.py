"""Sweep every advertised ELRS packet rate and RF band on connected hardware."""

from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path

from .config import TestConfig
from .crsf import (
    CRSF_ADDRESS_RECEIVER,
    CRSF_ADDRESS_TRANSMITTER,
    FrameType,
    Parameter,
    make_extended_frame,
    make_frame,
    unpack_channels_us,
)
from .link import (
    HandsetSession,
    ParameterClient,
    RxLink,
    SerialPort,
    expected_rx_rc_rate_hz,
)
from .smoke import SmokeReport


CHANNEL_PATTERN = [1100, 1200, 1300, 1400, 1000] + [1500] * 11
BAND_TRANSITION_RATE_PREFIXES = ("250Hz(", "150Hz(")
RC_DROPOUT_INTERVALS = 3.0
RX_TRAFFIC_LOG = Path(__file__).resolve().parent.parent / "rx.log"
TX_TRAFFIC_LOG = Path(__file__).resolve().parent.parent / "tx.log"


@dataclass(frozen=True)
class LinkResult:
    elapsed: float | None
    lq: int | None
    mode: int | None
    channels: list[int] | None
    link_frames: int
    rc_frames: int
    tx_connected: bool | None
    status_frames: int
    status_flags: int | None
    packets_bad: int | None
    packets_good: int | None
    measurement_seconds: float
    expected_rc_rate_hz: float
    measured_rc_frames: int
    measured_valid_rc_frames: int
    observed_rc_rate_hz: float
    delivery_percent: float
    rc_dropout_events: int
    estimated_missing_rc_frames: int
    longest_rc_gap_ms: float
    link_dropout_events: int
    measurement_passed: bool


def wait_for_valid_link(handset: HandsetSession, rx: RxLink, started: float,
                        config: TestConfig) -> LinkResult:
    lq = None
    mode = None
    channels = None
    link_frames = 0
    rc_frames = 0
    tx_connected = None
    status_frames = 0
    status_flags = None
    packets_bad = None
    packets_good = None
    valid_lq = False
    valid_rc = False
    valid_tx_connected = False
    deadline = started + config.reestablish_timeout_seconds
    next_status_request = started
    while time.monotonic() < deadline and not (valid_tx_connected and valid_lq and valid_rc):
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
            if frame.type == FrameType.ELRS_STATUS and len(frame.extended_payload) >= 4:
                data = frame.extended_payload
                status_frames += 1
                packets_bad = data[0]
                packets_good = int.from_bytes(data[1:3], "big")
                status_flags = data[3]
                tx_connected = bool(status_flags & 1)
                valid_tx_connected = tx_connected
        for frame in rx.poll():
            if frame.type == FrameType.LINK_STATISTICS:
                link_frames += 1
                lq = frame.payload[2]
                mode = frame.payload[5]
                valid_lq = lq > 0
            elif frame.type == FrameType.RC_CHANNELS_PACKED:
                rc_frames += 1
                channels = unpack_channels_us(frame.payload)
                valid_rc = all(abs(actual - expected) <= config.channel_tolerance_us
                               for actual, expected in zip(channels[:4], CHANNEL_PATTERN[:4]))
        time.sleep(0.001)
    elapsed = time.monotonic() - started
    measurement_seconds = 0.0
    radio_id_rate_hz = 1 / handset.interval
    expected_rc_rate_hz = expected_rx_rc_rate_hz(
        radio_id_rate_hz, config.telemetry_ratio)
    measured_rc_frames = 0
    measured_valid_rc_frames = 0
    observed_rc_rate_hz = 0.0
    delivery_percent = 0.0
    rc_dropout_events = 0
    estimated_missing_rc_frames = 0
    longest_rc_gap = 0.0
    link_dropout_events = 0
    measurement_passed = False
    if valid_tx_connected and valid_lq and valid_rc:
        dwell_started = time.monotonic()
        dwell_deadline = dwell_started + config.post_link_dwell_seconds
        expected_interval = 1 / expected_rc_rate_hz
        dropout_threshold = expected_interval * RC_DROPOUT_INTERVALS
        last_rc_at = dwell_started
        link_was_valid = True
        next_status_request = dwell_started
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
                if frame.type == FrameType.ELRS_STATUS and len(frame.extended_payload) >= 4:
                    data = frame.extended_payload
                    status_frames += 1
                    packets_bad = data[0]
                    packets_good = int.from_bytes(data[1:3], "big")
                    status_flags = data[3]
                    tx_connected = bool(status_flags & 1)
            for frame in rx.poll():
                frame_at = time.monotonic()
                if frame.type == FrameType.LINK_STATISTICS:
                    link_frames += 1
                    lq = frame.payload[2]
                    mode = frame.payload[5]
                    valid_lq = lq > 0
                elif frame.type == FrameType.RC_CHANNELS_PACKED:
                    rc_frames += 1
                    measured_rc_frames += 1
                    channels = unpack_channels_us(frame.payload)
                    valid_rc = all(abs(actual - expected) <= config.channel_tolerance_us
                                   for actual, expected in zip(
                                       channels[:4], CHANNEL_PATTERN[:4]))
                    if valid_rc:
                        measured_valid_rc_frames += 1
                    if tx_connected and valid_lq:
                        gap = frame_at - last_rc_at
                        longest_rc_gap = max(longest_rc_gap, gap)
                        if gap > dropout_threshold:
                            rc_dropout_events += 1
                            estimated_missing_rc_frames += max(
                                1, round(gap / expected_interval) - 1)
                        last_rc_at = frame_at
            link_is_valid = bool(tx_connected and valid_lq)
            if link_was_valid and not link_is_valid:
                link_dropout_events += 1
            if not link_was_valid and link_is_valid:
                last_rc_at = time.monotonic()
            link_was_valid = link_is_valid
            time.sleep(0.001)
        measurement_seconds = time.monotonic() - dwell_started
        if link_was_valid:
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
            measured_rc_frames > 0
            and measured_valid_rc_frames == measured_rc_frames
            and rc_dropout_events == 0
            and link_dropout_events == 0
        )
    return LinkResult(
        elapsed if valid_tx_connected and valid_lq and valid_rc else None,
        lq, mode, channels, link_frames, rc_frames, tx_connected, status_frames,
        status_flags, packets_bad, packets_good, measurement_seconds,
        expected_rc_rate_hz, measured_rc_frames, measured_valid_rc_frames,
        observed_rc_rate_hz, delivery_percent, rc_dropout_events,
        estimated_missing_rc_frames, longest_rc_gap * 1000, link_dropout_events,
        measurement_passed,
    )


def link_metrics_text(link: LinkResult) -> str:
    return (
        f"measurement_s={link.measurement_seconds:.3f} "
        f"expected_rx_rc_hz={link.expected_rc_rate_hz:.1f} "
        f"observed_rc_hz={link.observed_rc_rate_hz:.1f} "
        f"delivery_percent={link.delivery_percent:.1f} "
        f"measured_rc_frames={link.measured_rc_frames} "
        f"measured_valid_rc_frames={link.measured_valid_rc_frames} "
        f"rc_dropout_events={link.rc_dropout_events} "
        f"estimated_missing_rc_frames={link.estimated_missing_rc_frames} "
        f"longest_rc_gap_ms={link.longest_rc_gap_ms:.3f} "
        f"link_dropout_events={link.link_dropout_events} "
        f"measurement_passed={int(link.measurement_passed)}"
    )


def change_and_wait(report: SmokeReport, name: str, client: ParameterClient,
                    parameter: Parameter, value: int, option: str,
                    handset: HandsetSession, rx: RxLink,
                    config: TestConfig, band_id: int, rate_id: int) -> bool:
    before_band = client.read(CRSF_ADDRESS_TRANSMITTER, band_id)
    before_rate = client.read(CRSF_ADDRESS_TRANSMITTER, rate_id)
    print(
        f"change_start test={name!r} requested_index={value} requested_value={option!r} "
        f"tx_band={before_band.options[before_band.value]!r} "
        f"tx_rate={before_rate.options[before_rate.value]!r} "
        f"radio_id_rate_hz={1 / handset.interval:.1f}",
        flush=True,
    )
    started = time.monotonic()
    result = client.write(CRSF_ADDRESS_TRANSMITTER, parameter, str(value))
    rx.port.serial.reset_input_buffer()
    link = wait_for_valid_link(handset, rx, started, config)
    live_band = client.read(CRSF_ADDRESS_TRANSMITTER, band_id)
    live_rate = client.read(CRSF_ADDRESS_TRANSMITTER, rate_id)
    state = (
        f"tx_band={live_band.options[live_band.value]!r} "
        f"tx_rate={live_rate.options[live_rate.value]!r} "
        f"radio_id_rate_hz={1 / handset.interval:.1f} "
        f"tx_connected={link.tx_connected} status_frames={link.status_frames} "
        f"status_flags={link.status_flags} packets_bad={link.packets_bad} "
        f"packets_good={link.packets_good} lq={link.lq} mode={link.mode} "
        f"link_frames={link.link_frames} rc_frames={link.rc_frames} "
        f"channels={link.channels[:5] if link.channels else None} "
        f"rx_crc_errors={rx.port.parser.crc_errors} "
        f"rx_bytes_discarded={rx.port.parser.bytes_discarded} "
        f"{link_metrics_text(link)}"
    )
    passed = result.verified and link.elapsed is not None and link.measurement_passed
    report.add(
        name,
        "PASS" if passed else "FAIL",
        f"index={value} value={option!r} link+RC after {link.elapsed:.1f}s {state}"
        if passed else
        f"index={value} value={option!r} verified={int(result.verified)} "
        f"reestablish_s={link.elapsed} {state} "
        f"timeout={config.reestablish_timeout_seconds:.0f}s",
    )
    return passed


def change_band_and_wait(report: SmokeReport, name: str, client: ParameterClient,
                         band: Parameter, rate: Parameter, band_index: int,
                         band_name: str, handset: HandsetSession, rx: RxLink,
                         config: TestConfig) -> tuple[bool, Parameter, Parameter]:
    before_band = client.read(CRSF_ADDRESS_TRANSMITTER, band.id)
    before_rate = client.read(CRSF_ADDRESS_TRANSMITTER, rate.id)
    print(
        f"change_start test={name!r} requested_band_index={band_index} "
        f"requested_band={band_name!r} tx_band={before_band.options[before_band.value]!r} "
        f"tx_rate={before_rate.options[before_rate.value]!r} "
        f"radio_id_rate_hz={1 / handset.interval:.1f}",
        flush=True,
    )
    started = time.monotonic()
    band_result = client.write(CRSF_ADDRESS_TRANSMITTER, band, str(band_index))
    band = client.read(CRSF_ADDRESS_TRANSMITTER, band.id)
    rate = client.read(CRSF_ADDRESS_TRANSMITTER, rate.id)
    transition_rate = next(
        index
        for prefix in BAND_TRANSITION_RATE_PREFIXES
        for index, option in enumerate(rate.options)
        if option.startswith(prefix)
    )
    transition_rate_name = rate.options[transition_rate]
    rate_result = client.write(CRSF_ADDRESS_TRANSMITTER, rate, str(transition_rate))
    rx.port.serial.reset_input_buffer()
    link = wait_for_valid_link(handset, rx, started, config)
    band = client.read(CRSF_ADDRESS_TRANSMITTER, band.id)
    rate = client.read(CRSF_ADDRESS_TRANSMITTER, rate.id)
    state = (
        f"tx_band={band.options[band.value]!r} tx_rate={rate.options[rate.value]!r} "
        f"radio_id_rate_hz={1 / handset.interval:.1f} "
        f"tx_connected={link.tx_connected} status_frames={link.status_frames} "
        f"status_flags={link.status_flags} packets_bad={link.packets_bad} "
        f"packets_good={link.packets_good} lq={link.lq} mode={link.mode} "
        f"link_frames={link.link_frames} rc_frames={link.rc_frames} "
        f"channels={link.channels[:5] if link.channels else None} "
        f"rx_crc_errors={rx.port.parser.crc_errors} "
        f"rx_bytes_discarded={rx.port.parser.bytes_discarded} "
        f"{link_metrics_text(link)}"
    )
    passed = (band_result.verified and rate_result.verified
              and link.elapsed is not None and link.measurement_passed)
    report.add(
        name,
        "PASS" if passed else "FAIL",
        f"band_index={band_index} band={band_name!r} transition_rate_index="
        f"{transition_rate} transition_rate={transition_rate_name!r} "
        f"band_verified={int(band_result.verified)} "
        f"rate_verified={int(rate_result.verified)} "
        + (f"link+RC after {link.elapsed:.1f}s {state}" if passed else
           f"reestablish_s={link.elapsed} {state} "
           f"timeout={config.reestablish_timeout_seconds:.0f}s"),
    )
    return passed, band, rate


def run_rf_sweep(config: TestConfig) -> SmokeReport:
    report = SmokeReport()
    RX_TRAFFIC_LOG.write_text("", encoding="utf-8")
    TX_TRAFFIC_LOG.write_text("", encoding="utf-8")
    with SerialPort(config.tx_handset_port, config.tx_handset_baud, TX_TRAFFIC_LOG) as tx_port, \
            SerialPort(config.rx_port, config.rx_baud, RX_TRAFFIC_LOG) as rx_port:
        handset = HandsetSession(tx_port, CHANNEL_PATTERN)
        rx = RxLink(rx_port)

        started = time.monotonic()
        deadline = started + config.reestablish_timeout_seconds
        while time.monotonic() < deadline and not handset.synced:
            handset.poll()
            time.sleep(0.001)
        sync_elapsed = time.monotonic() - started
        report.add(
            "tx_handset_sync",
            "PASS" if handset.synced else "FAIL",
            f"RADIO_ID after {sync_elapsed:.1f}s rate={1 / handset.interval:.1f}Hz"
            if handset.synced else
            f"no RADIO_ID within {config.reestablish_timeout_seconds:.0f}s",
        )
        if not handset.synced:
            return report

        client = ParameterClient(handset, config.parameter_timeout_seconds)
        tx_device = None
        started = time.monotonic()
        deadline = started + config.reestablish_timeout_seconds
        while time.monotonic() < deadline and tx_device is None:
            tx_device = client.discover().get(CRSF_ADDRESS_TRANSMITTER)
        discovery_elapsed = time.monotonic() - started
        report.add(
            "tx_device_info",
            "PASS" if tx_device else "FAIL",
            f"{tx_device.name!r} after {discovery_elapsed:.1f}s parameters="
            f"{tx_device.parameter_count}" if tx_device else
            f"no DEVICE_INFO within {config.reestablish_timeout_seconds:.0f}s",
        )
        if tx_device is None:
            return report
        band = client.find(tx_device, "RF Band")
        initial_band_index = band.options.index(config.initial_rf_band)
        if band.value != initial_band_index:
            result = client.write(CRSF_ADDRESS_TRANSMITTER, band,
                                  str(initial_band_index))
            print(f"initial_band index={initial_band_index} value={config.initial_rf_band!r} "
                  f"verified={int(result.verified)}", flush=True)
            if not result.verified:
                report.add("initial_band", "FAIL",
                           f"index={initial_band_index} value={config.initial_rf_band!r} "
                           f"readback={result.parameter.value}")
                return report
        band = client.read(CRSF_ADDRESS_TRANSMITTER, band.id)
        rate = client.find(tx_device, "Packet Rate")
        initial_rate_index = rate.options.index(config.initial_packet_rate)
        if rate.value != initial_rate_index:
            result = client.write(CRSF_ADDRESS_TRANSMITTER, rate,
                                  str(initial_rate_index))
            print(f"initial_rate index={initial_rate_index} value={config.initial_packet_rate!r} "
                  f"verified={int(result.verified)}", flush=True)
            if not result.verified:
                report.add("initial_rate", "FAIL",
                           f"index={initial_rate_index} value={config.initial_packet_rate!r} "
                           f"readback={result.parameter.value}")
                return report

        band = client.read(CRSF_ADDRESS_TRANSMITTER, band.id)
        rate = client.read(CRSF_ADDRESS_TRANSMITTER, rate.id)

        telemetry_ratio = client.find(tx_device, "Telem Ratio")
        telemetry_ratio_index = telemetry_ratio.options.index(config.telemetry_ratio)
        if telemetry_ratio.value != telemetry_ratio_index:
            result = client.write(CRSF_ADDRESS_TRANSMITTER, telemetry_ratio,
                                  str(telemetry_ratio_index))
            telemetry_ratio = result.parameter
        ratio_verified = telemetry_ratio.value == telemetry_ratio_index
        report.add(
            "telemetry_ratio",
            "PASS" if ratio_verified else "FAIL",
            f"index={telemetry_ratio_index} value={config.telemetry_ratio!r} "
            f"readback={telemetry_ratio.value}",
        )
        if not ratio_verified:
            return report

        if config.manual_bind:
            rx.port.write(make_frame(FrameType.COMMAND, b"bd",
                                     address=CRSF_ADDRESS_RECEIVER))
            bind = client.find(tx_device, "Bind")
            bind_status = client.command(CRSF_ADDRESS_TRANSMITTER, bind, confirm=True)
            print(f"manual_bind=1 rx_command='bd' tx_command_id={bind.id} "
                  f"tx_status={bind_status.command_status}", flush=True)

        rx.port.serial.reset_input_buffer()

        started = time.monotonic()
        link = wait_for_valid_link(handset, rx, started, config)
        initial_link_passed = link.elapsed is not None and link.measurement_passed
        report.add(
            "initial_link",
            "PASS" if initial_link_passed else "FAIL",
            f"band={band.options[band.value]!r} rate={rate.options[rate.value]!r} "
            f"telemetry_ratio={config.telemetry_ratio!r} link+RC after {link.elapsed:.1f}s "
            f"radio_id_rate_hz={1 / handset.interval:.1f} "
            f"tx_connected={link.tx_connected} status_frames={link.status_frames} "
            f"status_flags={link.status_flags} packets_bad={link.packets_bad} "
            f"packets_good={link.packets_good} lq={link.lq} mode={link.mode} "
            f"link_frames={link.link_frames} rc_frames={link.rc_frames} "
            f"channels={link.channels[:5]} "
            f"rx_crc_errors={rx.port.parser.crc_errors} "
            f"rx_bytes_discarded={rx.port.parser.bytes_discarded} "
            f"{link_metrics_text(link)}"
            if link.elapsed is not None else
            f"band={band.options[band.value]!r} rate={rate.options[rate.value]!r} "
            f"telemetry_ratio={config.telemetry_ratio!r} "
            f"radio_id_rate_hz={1 / handset.interval:.1f} "
            f"tx_connected={link.tx_connected} status_frames={link.status_frames} "
            f"status_flags={link.status_flags} packets_bad={link.packets_bad} "
            f"packets_good={link.packets_good} lq={link.lq} mode={link.mode} "
            f"link_frames={link.link_frames} rc_frames={link.rc_frames} "
            f"channels={link.channels[:5] if link.channels else None} "
            f"rx_crc_errors={rx.port.parser.crc_errors} "
            f"rx_bytes_discarded={rx.port.parser.bytes_discarded} "
            f"{link_metrics_text(link)} "
            f"timeout={config.reestablish_timeout_seconds:.0f}s",
        )
        if not initial_link_passed:
            return report

        band = client.find(tx_device, "RF Band")
        rate = client.find(tx_device, "Packet Rate")
        original_band = band.options.index(config.initial_rf_band)
        original_rate = rate.options.index(config.initial_packet_rate)
        original_band_name = config.initial_rf_band
        original_rate_name = config.initial_packet_rate
        print(f"sweep device={tx_device.name!r} initial_band={original_band_name!r} "
              f"initial_rate={original_rate_name!r} telemetry_ratio="
              f"{config.telemetry_ratio!r} transition_timeout="
              f"{config.reestablish_timeout_seconds:.0f}s post_link_dwell="
              f"{config.post_link_dwell_seconds:.0f}s manual_bind="
              f"{int(config.manual_bind)}", flush=True)

        band_order = [original_band] + [index for index, option in enumerate(band.options)
                                        if option and index != original_band]
        for band_index in band_order:
            band_name = band.options[band_index]
            if band_index != original_band:
                transition_rate = next(
                    index
                    for prefix in BAND_TRANSITION_RATE_PREFIXES
                    for index, option in enumerate(rate.options)
                    if option.startswith(prefix)
                )
                if rate.value != transition_rate:
                    transition_rate_name = rate.options[transition_rate]
                    if not change_and_wait(
                            report,
                            f"transition_rate[{band.options[band.value]}][{transition_rate_name}]",
                            client, rate, transition_rate, transition_rate_name,
                            handset, rx, config, band.id, rate.id):
                        return report
                    rate = client.read(CRSF_ADDRESS_TRANSMITTER, rate.id)
                previous_band = band.value
                previous_band_name = band.options[previous_band]
                band_passed, band, rate = change_band_and_wait(
                    report, f"band[{band_name}]", client, band, rate, band_index,
                    band_name, handset, rx, config)
                if not band_passed:
                    rollback_passed, band, rate = change_band_and_wait(
                        report, f"rollback_band[{previous_band_name}]", client,
                        band, rate, previous_band, previous_band_name,
                        handset, rx, config)
                    if not rollback_passed:
                        return report
                    continue

            rate = client.read(CRSF_ADDRESS_TRANSMITTER, rate.id)
            current_rate = rate.value
            current_rate_name = rate.options[current_rate]
            print(f"band_index={band_index} band={band_name!r} "
                  f"current_rate_index={current_rate} current_rate={current_rate_name!r} "
                  f"rates={[(index, option) for index, option in enumerate(rate.options) if option]}",
                  flush=True)
            for rate_index, rate_name in enumerate(rate.options):
                if not rate_name or rate_index == current_rate:
                    continue
                previous_rate = current_rate
                previous_rate_name = rate.options[previous_rate]
                rate_passed = change_and_wait(
                    report, f"rate[{band_name}][{rate_name}]", client, rate,
                    rate_index, rate_name, handset, rx, config, band.id, rate.id)
                rate = client.read(CRSF_ADDRESS_TRANSMITTER, rate.id)
                if rate_passed:
                    current_rate = rate.value
                else:
                    rollback_passed = change_and_wait(
                        report, f"rollback_rate[{band_name}][{previous_rate_name}]",
                        client, rate, previous_rate, previous_rate_name,
                        handset, rx, config, band.id, rate.id)
                    rate = client.read(CRSF_ADDRESS_TRANSMITTER, rate.id)
                    current_rate = rate.value
                    if not rollback_passed:
                        return report

        band = client.read(CRSF_ADDRESS_TRANSMITTER, band.id)
        if band.value != original_band:
            change_and_wait(report, f"restore_band[{original_band_name}]", client, band,
                            original_band, original_band_name, handset, rx, config,
                            band.id, rate.id)
        rate = client.read(CRSF_ADDRESS_TRANSMITTER, rate.id)
        if rate.value != original_rate:
            change_and_wait(report, f"restore_rate[{original_rate_name}]", client, rate,
                            original_rate, original_rate_name, handset, rx, config,
                            band.id, rate.id)
    return report
