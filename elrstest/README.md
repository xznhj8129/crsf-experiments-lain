# elrstest — headless ELRS bench testing

## References:
https://github.com/tbs-fpv/tbs-crsf-spec/blob/main/crsf.md
https://github.com/Fourflies/elrsbuddy

## Description:

Plug the bench hardware into USB, then:

```bash
python3 elrstest.py identify   # which port is what
python3 elrstest.py smoke      # PASS/FAIL/SKIP for everything the wiring allows
python3 elrstest.py rf-sweep   # verify every advertised packet rate and RF band
```

Set one or more live TX parameters independently of a test with `config.py`:

```bash
python3 config.py tx --link-mode Normal "RF Band=2.4GHz" \
  "Packet Rate=333Hz Full(-105dBm)" "Telem Ratio=1:2"
```

`tx` selects `[tx].handset_port`; an explicit device path can be used in its
place. The script opens that TX handset port, supplies scheduled RC traffic,
discovers the TX's live parameter table, and applies assignments in command-line
order. It uses `[tx].handset_baud` unless `--baud` is supplied. Each value is
read back and verified before the next assignment, which is important because
changing `RF Band` replaces the live packet-rate table. The complete TX-side
CRSF exchange is written to `tx.log`.

Link mode defaults to `Normal` for ordinary CRSF transport; `--link-mode
MAVLink` selects MAVLink transport. `Link Mode` is always the first parameter
written and verified, before any band, rate, telemetry-ratio, or other
assignment. The script does not open or wait for the RX endpoint.

Verified working against a RadioMaster Nomad and a RadioMaster XR1, both on
stock ExpressLRS 4.0.1, behind two CP2102 adapters. The XR1 normally uses
420000 CRSF; this bench configures it for 460800 because the classic CP2102
is unreliable at 420000 (an inexact divisor) while 460800 and 115200 are
exact. 460800 is preferred: 115200 cannot carry K1000-rate RC output. The
XR1 currently runs `backups/xr1_stock401_460800_fcc915.bin` — stock 4.0.1
with baked-in UID, regulatory domain FCC915 (must match the TX or 915MHz
and X-Band never link), 460800 baud, and lock-on-first-connection disabled.
With the Nomad's hardware.json override applied (below),
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
mirror the `rx` commands (ELRS 4.0.1 TX-local origin `0xEF` and TX-module
destination `0xEE`, matching its cached stock Lua), and `smoke` extends to the
full link: RC pattern through the air, telemetry injected at the RX coming
back on the TX side.

The handset session sends RC frames at 20 Hz until the module sends `RADIO_ID`
(0x3A/0x10) sync, then follows the interval and shift the module dictates.
Like elrsbuddy, queued extended frames are appended after the 26-byte RC frame
in the next scheduled write. The latest timing shift applies after that write
and is then cleared.

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
  the UID matches the compiled key), per-unit options like the stock
  `--rx-baud 420000`, this bench's `--rx-baud 115200`, and `--domain fcc_915`.
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

## Reproducing the stock ELRS 4.0.1 recovery

This is the exact recovery path used on the Nomad/XR1 bench. It starts with a
stock XR1 accidentally restored to 420000 baud, where the classic CP2102
produces convincing but corrupt CRSF, and ends with the complete hardware
smoke test passing at 115200 baud.

### 1. Establish stable device names

Both CP2102s report the same serial number, so use the physical USB paths:

```bash
ls -l /dev/serial/by-path/
python3 elrstest.py identify
```

Put the RX adapter path in both `[rx].port` and `[unit.xr1].port`. Put the
Nomad USB-C path in `[tx].flash_port`, `[tx].handset_port`, and
`[unit.nomad].port`. For this bench the required RX values are:

```ini
[rx]
baud = 460800

[unit.xr1]
rx_baud = 460800
```

Do not use 420000 with the classic CP2102. On this adapter those
arbitrary rates are quantized badly enough to corrupt the stream.
(This recovery was originally performed at 115200, which also divides
exactly; the bench later moved to 460800 for K1000 headroom.)

### 2. Recognize the false high-baud result

At the wrong high baud, `rx monitor` appeared to receive thousands of these:

```text
TEL 0x3E destination=0xF0 origin=0x01 payload=0f7c33
```

That is not useful ELRS telemetry. Type `0x3E` is reserved by CRSF, and this
repeating frame was UART corruption that happened to pass CRC. Other strong
signatures were enormous `crc_errors`/`bytes_discarded` counts, sporadic RC
frames, changing decoded RF modes, and impossible positive RSSI values.

The reliable sanity check is a direct parameter query at 115200:

```bash
python3 elrstest.py rx devices
```

The expected result begins with:

```text
address=0xEC name='RM XR1' ... software=0x00040001 parameters=13
```

If the RX is still running at 420000, that query will be silent at 115200.
Use the RF-to-WiFi recovery below instead of trying to force the CP2102 to
carry 420000 baud.

### 3. Obtain the stock 4.0.1 image without building ELRS

ExpressLRS Configurator downloads the stock image directly. Select version
4.0.1 and target `RadioMaster XR1 Dual Band RX`; let Configurator download and
configure the image once. Its working files are under:

```bash
ELRS_CACHE="$HOME/.config/ExpressLRS Configurator/firmwares"
BINARY_TARGETS="$ELRS_CACHE/binary-targets"
ELRS_401=b5a5b5fdbed0d8420d0b4f5a3c55f380557def74
FLASHER="$ELRS_CACHE/cloud/ExpressLRS/$ELRS_401/firmware/flasher.pyz"
```

Before copying `firmware.bin`, make sure the most recently selected target is
really the XR1 and the image is 4.0.1:

```bash
strings "$BINARY_TARGETS/firmware.bin" | grep -E 'RadioMaster XR1|RM XR1|4\.0\.1'
```

**Do not assume the cached image carries any options.** The binary-targets
cache can be a bare target image, and each flasher patch pass **replaces the
options block wholesale** with only the flags given on that command line.
This bench lost a day to it: an image patched with only `--rx-baud` shipped
with no `uid` and no `domain`, so the RX fell back to the AU915 default while
the TX was FCC915 — every 2.4GHz mode passed and 915MHz/X-Band never linked
(binding still worked because the bound UID survives in NVS). Always pass the
full set — phrase, domain, rx-baud, lock flag — and verify the options JSON
in the image tail afterwards:

```bash
python3 - <<'PY'
import re
d = open("backups/xr1_stock401_460800_fcc915.bin","rb").read()
print(re.search(rb'\{"[^\x00]*?"rcvr-uart-baud"[^\x00]*?\}', d).group(0).decode())
PY
```

Make a workspace backup and patch with the complete option set:

```bash
mkdir -p backups
cp "$BINARY_TARGETS/firmware.bin" backups/xr1_stock401_115200.bin
IMAGE="$(pwd)/backups/xr1_stock401_115200.bin"

python3 "$FLASHER" \
  --dir "$BINARY_TARGETS" \
  --target radiomaster.rx_dual.xr1 \
  --rx-baud 115200 \
  "$IMAGE"
```

Use an absolute path for `IMAGE`. The Configurator flasher changes its working
directory internally; a relative image path works while patching but fails
when the WiFi uploader later tries to reopen it.

### 4. Keep the TX handset session alive until RF links

Cold link establishment is not immediate. Ten seconds was normal on the
recovered stock pair, and earlier cold starts took roughly 30 seconds. A
zero-LQ `LINK_STATISTICS` frame only proves the module is running; wait for
`lq > 0` before sending the receiver WiFi command.

The Nomad exposes `Enable Rx WiFi` as a TX parameter. This script sends the
normal command start/confirm sequence only after a real RF link is present,
then keeps producing handset RC long enough for the command to cross the air:

```bash
python3 - <<'PY'
import time
from pathlib import Path

from elrstest.config import load_config
from elrstest.crsf import (
    CRSF_ADDRESS_TRANSMITTER,
    FrameType,
    make_extended_frame,
)
from elrstest.link import HandsetSession, ParameterClient, SerialPort

config = load_config(Path("elrstest.ini"))
port = SerialPort(config.tx_handset_port, config.tx_handset_baud)
port.open()
handset = HandsetSession(port)

linked = False
while not linked:
    for frame in handset.poll():
        if frame.type == FrameType.LINK_STATISTICS and frame.payload[2] > 0:
            linked = True
    time.sleep(0.001)

client = ParameterClient(handset, 8)
tx = client.discover(8)[CRSF_ADDRESS_TRANSMITTER]
command = client.find(tx, "Enable Rx WiFi")
origin = handset.origin(tx.address)
print(f"rf_link={linked} command_id={command.id} command={command.name!r}")

for step in (1, 4):
    handset.queue(make_extended_frame(
        FrameType.PARAMETER_WRITE,
        tx.address,
        origin,
        bytes([command.id, step]),
    ))
    handset.run(1)

handset.run(12)
port.close()
PY
```

Step `1` starts the command and step `4` confirms it. The RX deliberately
leaves RF mode after accepting the command, so a generic command client may
time out while polling the final status even though WiFi started correctly.
The authoritative check is mDNS:

```bash
getent hosts elrs_rx.local
ping -c 1 elrs_rx.local
```

The receiver also advertises its live build options through mDNS. If
`avahi-browse` is installed, this shows the target, version, and current UART
define without touching the broken serial connection:

```bash
timeout 8s avahi-browse -art | grep -i elrs_rx
```

Before recovery it reported `RCVR_UART_BAUD=420000`.

### 5. Upload the prepared image over RX WiFi

Resolve the current address and upload the already-patched image:

```bash
RX_IP="$(getent ahostsv4 elrs_rx.local | awk 'NR == 1 {print $1}')"

python3 "$FLASHER" \
  --dir "$BINARY_TARGETS" \
  --target radiomaster.rx_dual.xr1 \
  --rx-baud 115200 \
  --flash wifi \
  --port "$RX_IP" \
  "$IMAGE"
```

Successful output includes `UPLOADING TO: http://<address>/update`, followed
by `UPLOAD SUCCESS`. Wait for the RX to reboot before opening its serial port.

### 6. Verify the direct serial path and the complete RF path

First prove that both directions of the RX UART work at 115200:

```bash
python3 elrstest.py rx devices
python3 elrstest.py rx params
```

Then run the real function test:

```bash
python3 elrstest.py smoke
```

The proven result is all nine checks passing:

```text
PASS rx_device_info
PASS rx_parameters
PASS rx_passive_output
PASS tx_flash_probe
PASS tx_handset_sync
PASS tx_device_info
PASS link_up
PASS rc_passthrough
PASS telemetry_return
```

The final three checks are not superficial protocol tests. They wait for the
RF link, send a distinctive five-channel pattern through the Nomad, verify
the exact pattern on the XR1 UART, inject a battery frame into the XR1, and
verify that exact frame returns through the Nomad. After all three paths are
valid, `smoke` also measures RC delivery for `post_link_dwell_seconds` and
prints the expected and observed rates, frame counts, delivery percentage,
missing-frame estimate, longest gap, and RC/link dropout counts.

### Sweep every packet rate and RF band

After `smoke` passes, run the longer hardware function test:

```bash
python3 elrstest.py rf-sweep
```

Every top-level script also mirrors its complete stdout/stderr into
`{scriptname}.log` (`elrstest.log`, `elrsflash.log`, `config.log`), truncated
per run — read results from there instead of scrollback.

Every `smoke` and `rf-sweep` run truncates `rx.log` and `tx.log`, then writes
the complete decoded CRSF traffic for that endpoint with monotonic timestamp,
direction, and raw frame hex. A frame repeating unchanged in one direction is
collapsed into a single `... repeated N more times ...` marker, emitted when
that direction's traffic changes (an unchanging RC stream no longer produces
megabytes of identical lines). The files are line-buffered and can be
followed while the test is running:

```bash
tail -F rx.log tx.log
```

Set the known starting/restoration mode, optional manual binding, and test
scope in `elrstest.ini`:

```ini
[rf_sweep]
initial_band = 2.4GHz
initial_rate = 333Hz Full(-105dBm)
telemetry_ratio = 1:2
manual_bind = false
reestablish_timeout_seconds = 30
post_link_dwell_seconds = 1
# scope: 'all', or comma-separated band labels / rate-label prefixes;
# test_rates = none sweeps band transitions only (~30 s on this bench)
test_bands = 2.4GHz,915MHz
test_rates = all
```

`test_bands` limits which advertised bands are entered (this bench excludes
X-Band: the XR1's single LR1121 cannot receive Dual-Band modes, so testing it
can only fail). `test_rates` limits the per-band rate sweep to matching
label prefixes (`250Hz,K1000`), or skips rate sweeping entirely with `none`
to exercise exactly the band-change path.

The test selects `initial_band` first, rereads that band's dynamic packet-rate
table, resolves `initial_rate` by its live label, and sets and verifies
`telemetry_ratio` before waiting for the initial link. It never carries a
numeric rate index between bands. With `manual_bind = true`, it commands the
directly connected XR1 into bind mode and invokes the Nomad's advertised
`Bind` command once before the initial link wait. This is optional recovery;
leave it `false` for the normal shared-binding-phrase setup.

It first requires a real link: the XR1 must report LQ above zero and output
the exact four analog control channels in the test pattern. AUX1 is printed
but is not part of the pass condition because its quantization changes with
the OTA packet mode. The test then reads the Nomad's live `RF Band`
and `Packet Rate` selections instead of assuming parameter IDs or a fixed
rate list. It tests every nonblank rate advertised for the current band,
switches through every other advertised band, verifies the link at the new
band's automatically selected rate, rereads that band's dynamic rate list,
and sweeps those rates too.

Before every band change, it selects and verifies the current band's
advertised 250 Hz mode, or 150 Hz when 250 Hz is unavailable. Immediately
after selecting the new band, it rereads the replaced rate table and selects
the new band's 250/150 Hz transition mode before starting the link timer.
This is necessary because stock ELRS 4.0.1 deliberately selects the fastest
supported rate whenever `RF Band` changes.

Each change has 30 seconds to produce both LQ above zero and the exact analog
RC pattern. Every line prints the measured reestablishment time. The original
band and rate are restored and verified at the end. After every successful
link check, the test holds that state for one second while continuing handset
RC traffic before reading or changing another parameter.

That post-link dwell is also an RC delivery measurement window. Each result
prints its duration, RADIO_ID air-slot rate, telemetry-adjusted expected RX RC
rate, observed RX RC rate, delivery percentage, total and pattern-valid RC
frame counts, longest RC gap, dropout events, estimated missing frames, and
link-state dropouts. For example, `1:2` telemetry uses every second RF slot,
so a 333 Hz RADIO_ID rate produces about 166.5 RX RC frames/s. An RC dropout
is a gap longer than three telemetry-adjusted RX RC intervals while TX
`ELRS_STATUS` still says connected and RX LQ remains above zero. A transition
fails if the window contains an invalid channel frame, an RC dropout, or a
link-state dropout.

The RX UART can be the limiting transport at the fastest OTA rates. A full
CRSF RC frame is 26 bytes; one frame per 1000 Hz RF packet would require
260 kbit/s before telemetry or UART framing overhead, so the bench's reliable
115200 RX setting cannot carry K1000 losslessly. The sweep does not hide this:
it records the failed mode and attempts the known-good rollback. Use a UART
adapter and RX baud comfortably above that data rate when K1000 itself is the
subject of the function test.

The RX firmware must also be configured with **Lock on first connection
disabled**. Stock 4.0.1 exposes this through the configurator as
`--no-lock-on-first-connection`. When the option is enabled, the RX sets
`LockRFmode` after its first link and deliberately stops disconnected RF-mode
scanning; the first band change then strands it on the old band until bind or
reflash recovery.

If a rate fails to reconnect, the test returns to the immediately preceding
known-good rate and verifies that rollback before testing anything else. A
failed band switch is likewise rolled back to the preceding known-good band
and rate. This prevents one lost link from producing meaningless cascading
failures in every later case.

## Reproducing a source build and define change

The stock recovery above does not need an ELRS build. Use this section when
testing modified ExpressLRS source or different build defines.

### Configure the source and define provider

Point `[fork]` at the source tree and at the `user_defines.txt` containing the
active binding phrase:

```ini
[fork]
dir = ../ExpressLRS
binding_defines = ../ExpressLRS/src/user_defines.txt
```

The phrase line must be active and have this exact form:

```text
-DMY_BINDING_PHRASE="your phrase"
```

Edit any other required ExpressLRS build defines in that source tree before
building. `elrsflash.py` intentionally reads only `MY_BINDING_PHRASE` from
the configured defines file for binary patching; it does not copy unrelated
flags from some other firmware tree.

The unit sections select the PlatformIO environment, unified target, domain,
flash port, and RX runtime baud. The relevant XR1 settings are:

```ini
[unit.xr1]
role = rx
env = Unified_ESP32C3_LR1121_RX_via_UART
target = radiomaster.rx_dual.xr1
rx_baud = 115200
domain = fcc_915
```

### Build, patch, flash, and verify explicitly

Run each phase separately while debugging:

```bash
python3 elrsflash.py build xr1
python3 elrsflash.py patch xr1
python3 elrsflash.py verify xr1

python3 elrsflash.py build nomad
python3 elrsflash.py patch nomad
python3 elrsflash.py verify nomad
```

`build` runs the configured PlatformIO environment. `patch` applies the
unified hardware target, binding phrase, regulatory domain, and `115200` RX
baud to the built image. The patched images are under the source tree at:

```text
src/.pio/build/Unified_ESP32C3_LR1121_RX_via_UART/firmware.bin
src/.pio/build/Unified_ESP32_LR1121_TX_via_UART/firmware.bin
```

Once the RX already speaks reliable 115200 CRSF, its headless serial flash
path is:

```bash
python3 elrsflash.py flash xr1
```

That command patches the image, sends the receiver-local CRSF `COMMAND 'bl'`,
then uses ELRS's patched esptool `--passthrough` support to write the app at
`0x10000`. It cannot rescue a stock 420000 RX through this classic CP2102;
use the RF-to-WiFi procedure above for that transition.

To upload the locally built image over RX WiFi instead, run the RF-link and
`Enable Rx WiFi` procedure in step 4, then point the same Configurator uploader
at the image already patched by `elrsflash.py`:

```bash
IMAGE="$(pwd)/../ExpressLRS/src/.pio/build/Unified_ESP32C3_LR1121_RX_via_UART/firmware.bin"
RX_IP="$(getent ahostsv4 elrs_rx.local | awk 'NR == 1 {print $1}')"

python3 "$FLASHER" \
  --dir "$BINARY_TARGETS" \
  --target radiomaster.rx_dual.xr1 \
  --rx-baud 115200 \
  --flash wifi \
  --port "$RX_IP" \
  "$IMAGE"
```

This is the complete define-to-radio path: edit `user_defines.txt`, build,
patch the unified target and options, command the linked RX into WiFi, upload
the patched build, let it reboot, then run `verify xr1` and `smoke`.

The Nomad has working ESP32 auto-reset lines, so its normal source-build flash
path is direct UART:

```bash
python3 elrsflash.py flash nomad
```

This writes the bootloader, partitions, boot app, and application but does not
write the LittleFS partition at `0x3d0000`. Therefore the USB CRSF pin override
survives ordinary firmware flashes.

After individual phases work, the combined path is:

```bash
python3 elrsflash.py pipeline all
```

The pipeline builds, patches, flashes, verifies both devices, and finally runs
the complete hardware smoke test. The RX must already be at the configured
115200 baud for the serial bootloader transition to work.

## Bench debugging map

| Symptom | Meaning and next check |
| --- | --- |
| Repeating `0x3E ... f0010f7c33`, huge CRC/discard counts | CP2102 corruption at an inexact baud (420000). Reflash the RX for 460800 (or 115200) over WiFi. |
| Every 2.4GHz mode passes but 915MHz and X-Band never link (`tx_connected=False`, no status message, RX silent) | Regulatory domain mismatch: 2.4GHz is domain-free, 900MHz FHSS tables differ per domain. Extract the options JSON from the flashed image tail — a missing `domain` key means the AU915 default. Repatch with `--domain fcc_915` on both ends. |
| 915MHz links but X-Band never does, on an XR1 | Hardware, not config: X-Band (Gemini Xrossband) modes are Dual-Band and stock `isSupportedRFRate()` rejects them on single-LR1121 receivers. The XR1 has one LR1121; only dual-radio RXs (DBR4 class) can receive X-Band. Expected sweep failure with this pair. |
| `tx_flash_probe` passes but `tx_handset_sync` fails | ESP32 boots, but runtime CRSF is not on USB pins. Check the Nomad LittleFS pin override and handset baud. |
| `tx_handset_sync` passes but LINK LQ stays zero | USB handset emulation works; wait through cold RF startup, then check phrase, domain, and model match. |
| LINK reaches LQ 100 but no RC frame appears at the RX adapter | Check RX baud and parser CRC/discard counters before blaming RF. The corrupt high-baud capture produced this exact illusion. |
| K1000 loses RC/LQ with the RX UART at 115200 | The UART is undersized: 26-byte RC frames at 1000 Hz alone are 260 kbit/s before UART framing and telemetry. Use a faster reliable adapter/baud for this mode. |
| Direct `rx devices` fails and injected telemetry never returns | Both operations need adapter TX to RX input. Check the 115200 setting and both UART wires. |
| RX monitor shows RC but the smoke pattern does not match | Inspect the smoke failure's `sent`, `last_rx`, and frame-type counts. These expose stale channels and parser corruption. |
| `Enable Rx WiFi` status polling times out | The RX may already have left RF mode. Check `getent hosts elrs_rx.local` before retrying. |
| TX shows RADIO_ID and LINK but no FC sensor frames | The RF link can be valid without craft telemetry. Verify the FC-to-RX CRSF direction separately; do not infer it from LINK alone. |

## Restoring the Nomad to its stock pin layout

The runtime pin swap is only a LittleFS `/hardware.json` override. It does not
replace the application firmware, bootloader, partition table, or factory
hardware definition. Removing that one partition restores the stock bay-pin
CRSF routing.

### Back up the current override

```bash
mkdir -p backups
TX_PORT="$(python3 -c 'import configparser; c=configparser.ConfigParser(); c.read("elrstest.ini"); print(c["tx"]["flash_port"])')"
ESPTOOL="$HOME/miniconda3/bin/esptool.py"

"$ESPTOOL" \
  --chip esp32 \
  --port "$TX_PORT" \
  --baud 921600 \
  read_flash 0x3d0000 0x20000 backups/nomad_littlefs.bin
```

### Erase only the LittleFS override

```bash
"$ESPTOOL" \
  --chip esp32 \
  --port "$TX_PORT" \
  --baud 921600 \
  erase_region 0x3d0000 0x20000
```

Power-cycle the Nomad after the erase. Then blank `handset_port` in
`elrstest.ini`, because stock firmware remaps runtime CRSF back to the module
bay pin and USB will no longer be a handset connection:

```ini
[tx]
handset_port =
```

The stock result is intentional:

- USB-C remains the ESP32 ROM flash/debug/reset connection.
- Runtime CRSF returns to the Nomad module-bay pin.
- `python3 elrstest.py tx probe` still works over USB.
- `tx monitor`, TX parameters, RC injection, and the full smoke test no longer
  work over USB unless the pin override is restored.

### Flash or recover it with normal ExpressLRS Configurator

In ExpressLRS Configurator select version 4.0.1, target `RadioMaster Nomad
X-Band`, regulatory domain `FCC 915`, method `UART`, and the Nomad USB-C port.
The internal CP2102 drives ESP32 EN/IO0, so Configurator can enter the ROM
bootloader and flash normally.

The LittleFS runtime override never prevented ROM flashing: ESP32 ROM always
uses UART0 pins 1/3. Erasing `0x3d0000` is required to restore stock runtime
bay-pin behavior, not to make the ROM bootloader electrically reachable.

If a failed flash leaves the module apparently dead, first run:

```bash
python3 elrstest.py tx probe
```

That deasserts the CP2102 reset controls, pulses a normal boot, and reports
whether the ESP32 is running the application or waiting in download mode.
Then retry the Configurator UART flash. A normal Configurator application
flash does not recreate the deleted LittleFS override.

To restore this bench's USB handset pin swap later, write the saved partition
back and power-cycle:

```bash
"$ESPTOOL" \
  --chip esp32 \
  --port "$TX_PORT" \
  --baud 921600 \
  write_flash 0x3d0000 backups/nomad_littlefs.bin
```

### Unit tests

```bash
python3 -m pytest tests/
```
