"""Liveness probe for an ESP32 module's flash/debug port (e.g. the Nomad's USB-C).

On the stock layout this port does NOT speak CRSF: ExpressLRS remaps the
UART0 peripheral onto the module-bay pin (GPIO4 on the Nomad) right after
boot, so USB only carries the ROM boot banner and early app logs — silence
here is normal, not a fault. What the probe gives us:

* proof the module is powered, flashed, and booting (banner + app log),
* recovery from the genuinely stuck states: a tool that exits with RTS
  asserted and DTR deasserted holds the chip in reset, and a chip left in
  the ROM bootloader never runs the app. The probe's reset pulse fixes both.
"""

from __future__ import annotations

import time
from dataclasses import dataclass

import serial

BOOT_BAUD = 115200


@dataclass(frozen=True)
class FlashProbeResult:
    device: str
    bytes_received: int
    rom_banner: bool
    boot_mode: str  # "flash", "download", or "unknown"
    reset_cause: str
    text: str

    @property
    def app_booting(self) -> bool:
        return self.rom_banner and self.boot_mode == "flash"

    def summary(self) -> str:
        if self.bytes_received == 0:
            return ("no boot output: module absent, unpowered, or reset lines not wired "
                    "(is this really the flash port?)")
        if self.boot_mode == "download":
            return "module is in the ROM bootloader (download mode) — flash it or pulse reset with IO0 high"
        if self.app_booting:
            return f"module alive: ROM banner seen ({self.reset_cause}), firmware booting"
        return "output received but no ESP ROM banner recognized"


def flash_probe(device: str, listen_seconds: float = 3.0) -> FlashProbeResult:
    """Pulse reset (RTS=EN, DTR=IO0 kept high) and capture boot output."""
    port = serial.Serial()
    port.port = device
    port.baudrate = BOOT_BAUD
    port.timeout = 0
    port.rts = False
    port.dtr = False
    port.open()
    try:
        port.reset_input_buffer()
        port.dtr = False  # IO0 high: boot from flash, not download mode
        port.rts = True   # EN low: reset
        time.sleep(0.1)
        port.rts = False  # release
        received = bytearray()
        deadline = time.monotonic() + listen_seconds
        while time.monotonic() < deadline:
            waiting = port.in_waiting
            if waiting:
                received += port.read(waiting)
            time.sleep(0.005)
    finally:
        port.close()

    text = received.decode("utf-8", errors="replace")
    rom_banner = "rst:0x" in text or text.startswith("ets ")
    if "DOWNLOAD" in text or "waiting for download" in text:
        boot_mode = "download"
    elif "FLASH_BOOT" in text:
        boot_mode = "flash"
    else:
        boot_mode = "unknown"
    reset_cause = "unknown"
    if "rst:0x" in text:
        start = text.index("rst:0x")
        reset_cause = text[start:text.find("\n", start)].strip().split(",")[0]
    return FlashProbeResult(device, len(received), rom_banner, boot_mode, reset_cause, text)
