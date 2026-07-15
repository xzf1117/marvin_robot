#!/usr/bin/env python3
import argparse
import glob
import os
import termios
import time


COLOR_ADDR = {
    "green": 0x02,
    "red": 0x03,
    "yellow": 0x04,
    "blue": 0x09,
}

OP = {
    "off": 0x00,
    "on": 0x01,
    "blink": 0x02,
}


def configure(fd, baud):
    attrs = termios.tcgetattr(fd)
    baud_attr = getattr(termios, f"B{baud}", termios.B9600)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
    attrs[3] = 0
    attrs[4] = baud_attr
    attrs[5] = baud_attr
    termios.tcsetattr(fd, termios.TCSANOW, attrs)


def write_command(port, baud, address, op):
    checksum = (0xA0 + address + op) & 0xFF
    payload = bytes((0xA0, address, op, checksum))
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        configure(fd, baud)
        os.write(fd, payload)
    finally:
        os.close(fd)


def find_port():
    if os.path.exists("/dev/apex_warning_light"):
        return "/dev/apex_warning_light"
    for path in glob.glob("/dev/serial/by-id/*1a86*") + glob.glob("/dev/serial/by-id/*CH34*"):
        return path
    ports = sorted(glob.glob("/dev/ttyUSB*"))
    if ports:
        return ports[0]
    return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("state", choices=["off", "green", "red", "yellow", "blue"])
    parser.add_argument("--port")
    parser.add_argument("--baud", type=int, default=9600)
    parser.add_argument("--blink", action="store_true")
    args = parser.parse_args()
    port = args.port or find_port()
    if not port:
        raise SystemExit(
            "No warning light serial port found. Run: "
            "sudo /home/jjj/code/Apex_Deploy/scripts/bind_warning_light.sh"
        )

    if args.state == "off":
        write_command(port, args.baud, 0x00, OP["off"])
        return

    write_command(port, args.baud, 0x00, OP["off"])
    time.sleep(0.12)
    write_command(port, args.baud, COLOR_ADDR[args.state], OP["blink" if args.blink else "on"])


if __name__ == "__main__":
    main()
