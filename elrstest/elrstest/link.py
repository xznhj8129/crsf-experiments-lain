"""Serial transports and CRSF clients.

Two verified ways to talk to ELRS hardware from a PC:

* RxLink: the receiver's CRSF UART (full duplex, fixed baud). The RX answers
  DEVICE_PING / PARAMETER_READ / PARAMETER_WRITE / COMMAND directly on the
  wire, but only when our origin address is the flight controller (0xC8) —
  replies to any other origin get routed over the RF link instead.

* HandsetSession: emulates a handset on the TX module's CRSF pin the way
  elrsbuddy does: RC frames at 5 Hz until the module sends RADIO_ID sync
  (0x3A subtype 0x10), then paced to the interval/shift the module dictates,
  with extended frames piggybacked into the same write after the RC frame.
  Handles the half-duplex echo of a one-wire JR-bay connection.
"""

from __future__ import annotations

import time
from collections import deque
from dataclasses import dataclass

import serial

from .crsf import (
    CRSF_ADDRESS_BROADCAST,
    CRSF_ADDRESS_ELRS_LUA,
    CRSF_ADDRESS_FLIGHT_CONTROLLER,
    CRSF_ADDRESS_RADIO_TRANSMITTER,
    CRSF_ADDRESS_TRANSMITTER,
    DeviceInfo,
    Frame,
    FrameParser,
    FrameType,
    Parameter,
    ParameterType,
    decode_parameter,
    encode_parameter_value,
    make_battery_frame,
    make_extended_frame,
    make_rc_frame,
    parse_device_info,
)

RADIO_ID_SYNC_SUBTYPE = 0x10
CRSF_TICKS_PER_SECOND = 10_000_000  # RADIO_ID durations are 0.1 us ticks


class SerialPort:
    """CRSF framing over one serial port.

    DTR and RTS are held deasserted: on modules whose USB-UART drives an
    ESP32 auto-reset circuit, an asserted RTS with deasserted DTR holds the
    chip in reset.
    """

    def __init__(self, device: str, baud: int) -> None:
        self.device = device
        self.baud = baud
        self.parser = FrameParser()
        self.bytes_read = 0
        self.bytes_written = 0
        self.serial = serial.Serial()
        self.serial.port = device
        self.serial.baudrate = baud
        self.serial.timeout = 0
        self.serial.write_timeout = 0.2
        self.serial.rts = False
        self.serial.dtr = False

    def open(self) -> None:
        self.serial.open()
        self.serial.reset_input_buffer()
        self.serial.reset_output_buffer()

    def close(self) -> None:
        self.serial.close()

    def read_frames(self) -> list[Frame]:
        waiting = self.serial.in_waiting
        if not waiting:
            return []
        data = self.serial.read(waiting)
        self.bytes_read += len(data)
        return self.parser.feed(data)

    def write(self, data: bytes) -> None:
        self.serial.write(data)
        self.bytes_written += len(data)

    def __enter__(self) -> SerialPort:
        self.open()
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()


def sniff_crsf(device: str, baud: int, seconds: float = 1.5) -> tuple[int, dict[int, int]]:
    """Passively count valid CRSF frames on a port. Returns (frames, {type: count})."""
    counts: dict[int, int] = {}
    total = 0
    with SerialPort(device, baud) as port:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            for frame in port.read_frames():
                counts[frame.type] = counts.get(frame.type, 0) + 1
                total += 1
            time.sleep(0.002)
    return total, counts


class RxLink:
    """Talk to an ELRS receiver on its CRSF UART, posing as the flight controller."""

    def __init__(self, port: SerialPort) -> None:
        self.port = port

    def origin(self, device_address: int) -> int:
        return CRSF_ADDRESS_FLIGHT_CONTROLLER

    def queue(self, frame: bytes) -> None:
        self.port.write(frame)

    def poll(self) -> list[Frame]:
        return self.port.read_frames()

    def inject_battery(self, voltage_decivolts: int = 251, current_deciamps: int = 37,
                       capacity_mah: int = 0x012345, remaining_percent: int = 73) -> bytes:
        frame = make_battery_frame(voltage_decivolts, current_deciamps, capacity_mah, remaining_percent)
        self.port.write(frame)
        return frame


class HandsetSession:
    """Emulates a CRSF handset for an ELRS TX module, elrsbuddy-style."""

    def __init__(self, port: SerialPort, channels_us: list[int] | None = None,
                 initial_interval_s: float = 0.05) -> None:
        self.port = port
        self.channels_us = channels_us or [1500, 1500, 1500, 1500, 1000] + [1500] * 11
        self.interval = initial_interval_s
        self.sync_count = 0
        self.rc_frames_sent = 0
        self.pending: list[bytes] = []
        self.next_write = time.monotonic()
        self.recent_sent: deque[tuple[float, bytes]] = deque(maxlen=64)

    @property
    def synced(self) -> bool:
        return self.sync_count > 0

    def origin(self, device_address: int) -> int:
        if device_address == CRSF_ADDRESS_TRANSMITTER:
            return CRSF_ADDRESS_ELRS_LUA
        return CRSF_ADDRESS_RADIO_TRANSMITTER

    def queue(self, frame: bytes) -> None:
        self.pending.append(frame)

    def _is_echo(self, frame: Frame) -> bool:
        now = time.monotonic()
        for stamp, raw in self.recent_sent:
            if raw == frame.raw and now - stamp < 0.5:
                self.recent_sent.remove((stamp, raw))
                return True
        return False

    def poll(self) -> list[Frame]:
        now = time.monotonic()
        if now >= self.next_write:
            rc = make_rc_frame(self.channels_us)
            burst = rc + b"".join(self.pending)
            sent_frames = [rc] + self.pending
            self.pending.clear()
            self.port.write(burst)
            self.rc_frames_sent += 1
            for raw in sent_frames:
                self.recent_sent.append((now, raw))
            self.next_write += self.interval
            if self.next_write <= now:
                self.next_write = now + self.interval

        frames = []
        for frame in self.port.read_frames():
            if frame.type == FrameType.RADIO_ID and len(frame.payload) >= 11 \
                    and frame.payload[2] == RADIO_ID_SYNC_SUBTYPE:
                interval = int.from_bytes(frame.payload[3:7], "big", signed=True)
                shift = int.from_bytes(frame.payload[7:11], "big", signed=True)
                if interval > 0:
                    self.interval = interval / CRSF_TICKS_PER_SECOND
                    self.next_write = time.monotonic() + self.interval + shift / CRSF_TICKS_PER_SECOND
                self.sync_count += 1
            if self._is_echo(frame):
                continue
            frames.append(frame)
        return frames

    def run(self, seconds: float) -> list[Frame]:
        frames: list[Frame] = []
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            frames.extend(self.poll())
            time.sleep(0.001)
        return frames


@dataclass(frozen=True)
class WriteResult:
    parameter: Parameter
    old_value: int | str | None
    verified: bool


def current_value_bytes(parameter: Parameter) -> bytes | None:
    """The parameter's current value encoded as it would be for a write."""
    if parameter.type == ParameterType.SELECTION:
        return bytes([parameter.value])
    if parameter.type.value <= ParameterType.INT64:
        size = 1 << (parameter.type.value // 2)
        return parameter.value.to_bytes(size, "big", signed=bool(parameter.type.value % 2))
    if parameter.type == ParameterType.FLOAT:
        return parameter.value.to_bytes(4, "big", signed=True)
    return None


class ParameterClient:
    """The Lua-script parameter protocol over any transport (RxLink or HandsetSession)."""

    def __init__(self, transport, timeout_seconds: float = 3.0) -> None:
        self.transport = transport
        self.timeout_seconds = timeout_seconds

    def _pump(self, deadline: float, want) -> Frame | None:
        while time.monotonic() < deadline:
            for frame in self.transport.poll():
                if want(frame):
                    return frame
            time.sleep(0.002)
        return None

    def discover(self, seconds: float = 2.0) -> dict[int, DeviceInfo]:
        ping = make_extended_frame(FrameType.DEVICE_PING, CRSF_ADDRESS_BROADCAST,
                                   self.transport.origin(CRSF_ADDRESS_BROADCAST))
        self.transport.queue(ping)
        devices: dict[int, DeviceInfo] = {}
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            for frame in self.transport.poll():
                if frame.type == FrameType.DEVICE_INFO:
                    device = parse_device_info(frame)
                    devices[device.address] = device
            time.sleep(0.002)
        return devices

    def _read_chunks(self, device_address: int, parameter_id: int,
                     query_type: int, query_value: int | None = None) -> bytes:
        origin = self.transport.origin(device_address)
        chunk = 0
        assembled = bytearray()
        while True:
            value = chunk if query_value is None else query_value
            self.transport.queue(make_extended_frame(query_type, device_address, origin,
                                                     bytes([parameter_id, value])))
            frame = self._pump(
                time.monotonic() + self.timeout_seconds,
                lambda f: f.type == FrameType.PARAMETER_SETTINGS_ENTRY
                and f.origin == device_address and f.extended_payload[0] == parameter_id,
            )
            if frame is None:
                raise TimeoutError(
                    f"device=0x{device_address:02X} parameter_id={parameter_id} "
                    f"chunk={chunk} timeout_seconds={self.timeout_seconds}")
            response = frame.extended_payload
            assembled.extend(response[2:])
            if response[1] == 0:
                return bytes(assembled)
            chunk += 1

    def read(self, device_address: int, parameter_id: int) -> Parameter:
        return decode_parameter(parameter_id,
                                self._read_chunks(device_address, parameter_id, FrameType.PARAMETER_READ))

    def read_all(self, device: DeviceInfo) -> list[Parameter]:
        return [self.read(device.address, parameter_id)
                for parameter_id in range(1, device.parameter_count + 1)]

    def find(self, device: DeviceInfo, name_or_id: str) -> Parameter:
        if name_or_id.isdecimal():
            return self.read(device.address, int(name_or_id))
        for parameter_id in range(1, device.parameter_count + 1):
            parameter = self.read(device.address, parameter_id)
            if parameter.name.casefold() == name_or_id.casefold():
                return parameter
        raise KeyError(f"parameter {name_or_id!r} not found on device 0x{device.address:02X}")

    def write(self, device_address: int, parameter: Parameter, value: str) -> WriteResult:
        encoded = encode_parameter_value(parameter, value)
        origin = self.transport.origin(device_address)
        self.transport.queue(make_extended_frame(FrameType.PARAMETER_WRITE, device_address, origin,
                                                 bytes([parameter.id]) + encoded))
        deadline = time.monotonic() + 0.4
        while time.monotonic() < deadline:
            self.transport.poll()
            time.sleep(0.002)
        updated = self.read(device_address, parameter.id)
        return WriteResult(updated, parameter.value, current_value_bytes(updated) == encoded)

    def command(self, device_address: int, parameter: Parameter, confirm: bool = False) -> Parameter:
        if parameter.type != ParameterType.COMMAND:
            raise ValueError(f"parameter={parameter.name!r} type={parameter.type.name} expected=COMMAND")
        origin = self.transport.origin(device_address)

        def send(step: int) -> None:
            self.transport.queue(make_extended_frame(FrameType.PARAMETER_WRITE, device_address, origin,
                                                     bytes([parameter.id, step])))

        send(1)  # start
        deadline = time.monotonic() + self.timeout_seconds
        while time.monotonic() < deadline:
            status = decode_parameter(
                parameter.id,
                self._read_chunks(device_address, parameter.id, FrameType.PARAMETER_WRITE, query_value=6))
            if status.command_status == 0:  # READY: command finished
                return status
            if status.command_status == 3:  # CONFIRMATION_NEEDED
                if not confirm:
                    raise RuntimeError(
                        f"parameter={parameter.name!r} needs confirmation: {status.command_info!r}")
                send(4)
            time.sleep((status.command_timeout or 10) / 100)
        raise TimeoutError(f"device=0x{device_address:02X} command={parameter.name!r} "
                           f"timeout_seconds={self.timeout_seconds}")


def parameter_text(parameter: Parameter) -> str:
    parts = [
        f"id={parameter.id}",
        f"parent={parameter.parent}",
        f"type={parameter.type.name}",
        f"hidden={int(parameter.hidden)}",
        f"name={parameter.name!r}",
    ]
    if parameter.type == ParameterType.SELECTION and isinstance(parameter.value, int) \
            and parameter.value < len(parameter.options):
        parts.append(f"value={parameter.options[parameter.value]!r}")
    elif parameter.value is not None:
        parts.append(f"value={parameter.value!r}")
    if parameter.minimum is not None:
        parts.extend((f"minimum={parameter.minimum}", f"maximum={parameter.maximum}",
                      f"default={parameter.default}"))
    if parameter.options:
        parts.append(f"options={';'.join(parameter.options)!r}")
    if parameter.unit:
        parts.append(f"unit={parameter.unit!r}")
    if parameter.type == ParameterType.COMMAND:
        parts.extend((f"command_status={parameter.command_status}",
                      f"command_timeout={parameter.command_timeout}",
                      f"command_info={parameter.command_info!r}"))
    return " ".join(parts)
