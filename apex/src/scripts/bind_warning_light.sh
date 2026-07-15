#!/usr/bin/env bash
set -Eeuo pipefail

if [ "$(id -u)" -ne 0 ]; then
    echo "Please run with sudo: sudo $0" >&2
    exit 1
fi

found=0
modprobe ch341 2>/dev/null || true
for dev in /sys/bus/usb/devices/*; do
    [ -f "$dev/idVendor" ] || continue
    [ -f "$dev/idProduct" ] || continue

    vendor="$(cat "$dev/idVendor")"
    product="$(cat "$dev/idProduct")"
    if [ "$vendor:$product" != "1a86:7523" ]; then
        continue
    fi

    found=1
    dev_name="$(basename "$dev")"
    interface="${dev_name}:1.0"
    echo "Found CH340 warning light: $dev_name"

    if [ -L "/sys/bus/usb/devices/$interface/driver" ]; then
        driver="$(basename "$(readlink -f "/sys/bus/usb/devices/$interface/driver")")"
        if [ "$driver" != "ch341" ]; then
            echo "Unbinding $interface from $driver"
            echo "$interface" > "/sys/bus/usb/drivers/$driver/unbind"
        else
            echo "$interface is already bound to ch341"
        fi
    fi

    if [ -d /sys/bus/usb/drivers/ch341 ]; then
        if [ ! -L "/sys/bus/usb/devices/$interface/driver" ]; then
            echo "Binding $interface to ch341"
            echo "$interface" > /sys/bus/usb/drivers/ch341/bind || true
        fi
    else
        echo "ch341 driver directory not found; try: sudo modprobe ch341" >&2
    fi
done

if [ "$found" -eq 0 ]; then
    echo "No CH340 warning light found. Check USB connection." >&2
    exit 1
fi

sleep 0.5
echo "Serial devices:"
ls -l /dev/ttyUSB* 2>/dev/null || {
    echo "No /dev/ttyUSB* device was created." >&2
    exit 1
}
