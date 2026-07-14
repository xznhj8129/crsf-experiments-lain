# elrstest — headless ELRS bench testing

Plug the bench hardware into USB, then:

```bash
python3 elrstest.py identify   # which port is what
python3 elrstest.py smoke      # PASS/FAIL/SKIP for everything the wiring allows
```

Verified working against a RadioMaster Nomad and a RadioMaster XR1, both on
stock ExpressLRS 4.0.1 (XR1 at the standard 420000 CRSF baud), behind two
CP2102 adapters. With the Nomad's hardware.json override applied (below),
the full 9-check smoke passes over nothing but the two USB cables: RX CRSF
output, RX device info + parameters, TX boot probe, handset sync (333 Hz),
TX device info, RF link up, RC passthrough, and telemetry return.

### The receiver (full CRSF access over its UART)

The RX answers the complete Lua-configurator parameter protocol directly on
its CRSF UART — no radio, no RF link, no browser needed. The one trick: our
frames must use origin `0xC8` (flight controller) or the RX routes replies
over the (possibly dead) RF link instead of back down the wire.

```bash
python3 elrstest.py rx monitor                    # live decode: LINK_STATISTICS, RC out, telemetry
python3 elrstest.py rx devices                    # DEVICE_PING -> 'RM XR1', 13 parameters
python3 elrstest.py rx params                     # full parameter dump
python3 elrstest.py rx get "Tlm Power"
python3 elrstest.py rx set "Tlm Power" 25         # by option name or index; verified after write
python3 elrstest.py rx command "Enter Bind Mode" --confirm
```

### The TX module

**This bench's Nomad is already set up — nothing to do.** Its USB-C port is a
working full-duplex CRSF handset port; just plug in and use the `tx` commands.
The rest of this section explains why, and how to undo it.

**Why it needed setup.** On the stock layout the Nomad's USB-C port does not
speak CRSF. It is the ESP32's UART0 flash/debug port (internal CP2102 with
EN/IO0 auto-reset wiring). At boot, ExpressLRS remaps the UART0 peripheral
through the GPIO matrix onto the module-bay CRSF pin (`serial_rx/serial_tx: 4`),
so the USB pins go silent right after the boot banner — by design, not by
fault. Every tool that ever tried to speak CRSF at that port was doomed
regardless of baud rate. (The `/littlefs/options.json does not exist` boot
line is normal: littlefs holds only our hardware override, and the firmware
falls back to the config baked into the image for everything else.)

**What's applied.** A `/hardware.json` in the littlefs partition (0x3d0000)
with `serial_rx: 3, serial_tx: 1` — UART0 stays on the USB pins. ExpressLRS
loads this file *in preference to* the baked-in layout (`hardware_init()` in
`src/lib/OPTIONS/hardware.cpp`), so USB-C becomes the handset port. It is set
as `handset_port` in `elrstest.ini`, and `elrsflash.py` never writes that
partition, so **the override survives every firmware flash**. Confirm it any
time — this reads the file straight off the chip:

```bash
python3 - <<'PY'
import subprocess
subprocess.run(["/home/anon/miniconda3/bin/esptool.py", "--port",
  "/dev/serial/by-path/pci-0000:02:00.0-usb-0:8:1.0-port0", "--baud", "921600",
  "read_flash", "0x3d0000", "0x20000", "/tmp/lfs.bin"])
d = open("/tmp/lfs.bin","rb").read(); i = d.find(b'"serial_rx"')
print("override PRESENT" if b'"serial_rx": 3' in d else "override ABSENT (stock)")
PY
```

**To undo it** — required before this module goes back into a real radio's
module bay, because with `serial_tx: 1` it will not talk to a handset over the
bay pin:

```bash
# 1. delete the override, restoring exact stock behavior
/home/anon/miniconda3/bin/esptool.py \
  --port /dev/serial/by-path/pci-0000:02:00.0-usb-0:8:1.0-port0 \
  erase_region 0x3d0000 0x20000
# 2. blank handset_port in elrstest.ini so tx commands don't target dead USB
```

`tx probe` still works regardless of the override — it pulses reset and reads
the boot banner (proves power + boot, rescues a chip stuck in reset or the ROM
bootloader), and `smoke` runs it automatically.

**Doing this to a fresh module** (no override yet): either write a littlefs
image over USB with esptool (how this one was done — extract the module's own
layout, flip the two pins, `mklittlefs`, write to 0x3d0000), or use the stock
WiFi web API (`POST` the patched layout to `http://<module>/hardware.json`
after it joins WiFi / opens its AP), or clip a USB-UART onto the module-bay
CRSF pin through the JR/Lite adapter (one-wire half duplex; autobaud takes
921600 or 115200).

Once `handset_port` is set, `elrstest.py tx monitor|devices|params|get|set|command`
mirror the `rx` commands (handset origin addressing: `0xEF` for the TX
module, `0xEA` broadcast — same as elrsV3.lua), and `smoke` extends to the
full link: RC pattern through the air, telemetry injected at the RX coming
back on the TX side.

The handset session reproduces what elrsbuddy does: RC frames at 5 Hz until
the module sends `RADIO_ID` (0x3A/0x10) sync, then paced to the interval and
shift the module dictates, with extended frames appended to the same write
burst after the RC frame.

### Configuration

`elrstest.ini` (see `elrstest.ini.example`). Both CP2102s ship the same
serial number ("0001"), so `/dev/serial/by-id` collides — if a replug swaps
`ttyUSB0`/`ttyUSB1`, use the stable `/dev/serial/by-path/` names.

## elrsflash — build → define → flash → verify

`elrsflash.py` closes the loop: from fork source to flashed, configured,
verified hardware without touching a button.

```bash
python3 elrsflash.py pipeline all    # build + patch + flash + verify both units, then smoke
python3 elrsflash.py build xr1       # pio run the unit's env in the fork tree
python3 elrsflash.py flash nomad     # (re)patch defines and upload over USB
python3 elrsflash.py verify nomad    # ping the device, print build name + git hash
```

How the pieces fit ([fork] and [unit.*] sections in `elrstest.ini`):

- **Build**: `pio run -e <env>` in the configured firmware source tree.
- **Define**: the binding phrase comes from the configured `binding_defines`
  file and is patched into both images as the stock 6-byte UID (md5).
- The source tree's own `binary_configurator.py` patches the built image
  in place — target hardware layout, the same phrase (same md5 convention, so
  the UID matches the compiled key), per-unit options like `--rx-baud 420000`
  and `--domain fcc_915`.
- **Flash TX**: plain esptool via the Nomad's auto-reset lines. The littlefs
  partition is never written, so the USB-handset override survives.
- **Flash RX**: no reset lines needed — a CRSF `COMMAND 'bl'` frame drops the
  running firmware into its in-app esptool stub (`lib/SerialUpdate`), then the
  fork's bundled patched esptool (`--passthrough`) writes the app partition
  through it, and the stub reboots into the new firmware. Fully headless.
- **Verify**: DEVICE_PING over each endpoint, printing build name and git
  hash (e.g. `secure_4.0.1 FCC915, hash 3f3d4c`) so a stale flash is obvious.

Port names use `/dev/serial/by-path/` because both CP2102s report serial
"0001" and ttyUSB numbering swaps between replugs.

### Unit tests

```bash
python3 -m pytest tests/
```
