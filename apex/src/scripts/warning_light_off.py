#!/usr/bin/env python3
import glob
import os
import termios


def find_port():
    if os.path.exists("/dev/apex_warning_light"):
        return "/dev/apex_warning_light"
    for path in glob.glob("/dev/serial/by-id/*1a86*") + glob.glob("/dev/serial/by-id/*CH34*"):
        return path
    ports = sorted(glob.glob("/dev/ttyUSB*"))
    if ports:
        return ports[0]
    return None


def configure(fd):
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
    attrs[3] = 0
    attrs[4] = termios.B9600
    attrs[5] = termios.B9600
    termios.tcsetattr(fd, termios.TCSANOW, attrs)


def main():
    port = find_port()
    if not port:
        return 0
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        configure(fd)
        os.write(fd, bytes((0xA0, 0x00, 0x00, 0xA0)))
    finally:
        os.close(fd)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
