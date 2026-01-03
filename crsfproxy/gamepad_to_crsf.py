"""Usage:
python gamepad_to_crsf.py --target 192.168.4.1 --id tx1
python gamepad_to_crsf.py --target 192.168.4.1 --id tx1 --port 8099 --rate 75
"""

import argparse
import json
import socket
import time

import pygame

MIN_US = 900
MAX_US = 2100
MID_US = 1500
DEFAULT_PORT = 8099
DEFAULT_RATE_HZ = 50.0
AXIS_COUNT = 4
BUTTON_COUNT = 12


def axis_to_us(value: float) -> int:
    clamped = max(-1.0, min(1.0, float(value)))
    span = MAX_US - MIN_US
    return int(MIN_US + ((clamped + 1.0) * 0.5 * span))


def button_to_us(pressed: int) -> int:
    return MAX_US if pressed else MIN_US


def get_joystick_state(joystick):
    pygame.event.pump()
    axes = [round(joystick.get_axis(i), 3) for i in range(joystick.get_numaxes())]
    buttons = [joystick.get_button(i) for i in range(joystick.get_numbuttons())]
    while len(axes) < AXIS_COUNT:
        axes.append(0.0)
    while len(buttons) < BUTTON_COUNT:
        buttons.append(0)
    return axes[:AXIS_COUNT], buttons[:BUTTON_COUNT]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--target", required=True, help="crsfproxy TCP host")
    parser.add_argument("--id", required=True, help="Sender ID forwarded to proxy")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="crsfproxy TCP port")
    parser.add_argument("--rate", type=float, default=DEFAULT_RATE_HZ, help="Send rate in Hz")
    parser.add_argument("--joystick-index", type=int, default=0, help="Joystick index reported by pygame")
    args = parser.parse_args()

    period = 1.0 / args.rate

    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        raise RuntimeError("No joystick detected.")
    if args.joystick_index >= pygame.joystick.get_count():
        raise RuntimeError(f"Joystick index {args.joystick_index} not available.")

    joystick = pygame.joystick.Joystick(args.joystick_index)
    joystick.init()
    print(f"Joystick '{joystick.get_name()}' ready on index {args.joystick_index}.")

    if joystick.get_numaxes() < AXIS_COUNT:
        raise RuntimeError(f"Joystick must expose at least {AXIS_COUNT} axes.")

    conn = socket.create_connection((args.target, args.port), timeout=1.0)
    conn.setblocking(True)
    print(f"Connected to crsfproxy at {(args.target, args.port)}.")

    channels = [MID_US] * 16
    channels[2] = MIN_US
    channels[4] = MIN_US
    armed = False
    last_arm_btn = 0
    btn_latched = [False] * 4
    last_btns = [0] * 4

    try:
        while True:
            loop_start = time.time()
            axes, buttons = get_joystick_state(joystick)
            channels[0] = axis_to_us(axes[2])       # roll
            channels[1] = axis_to_us(axes[3])       # pitch
            channels[2] = axis_to_us(-axes[1])      # throttle (invert)
            channels[3] = axis_to_us(axes[0])       # yaw

            arm_btn = buttons[9]
            if arm_btn and not last_arm_btn:
                armed = not armed
            last_arm_btn = arm_btn
            channels[4] = MAX_US if armed else MIN_US

            for i in range(4):
                if buttons[i] and not last_btns[i]:
                    btn_latched[i] = not btn_latched[i]
                last_btns[i] = buttons[i]
                channels[5 + i] = MAX_US if btn_latched[i] else MIN_US

            payload = {
                "id": args.id,
                "ch": channels,
                "t": int(time.time() * 1000),
            }

            message = json.dumps(payload) + "\n"
            conn.sendall(message.encode("utf-8"))

            elapsed = time.time() - loop_start
            if elapsed < period:
                time.sleep(period - elapsed)
    finally:
        conn.close()
        pygame.quit()


if __name__ == "__main__":
    main()
