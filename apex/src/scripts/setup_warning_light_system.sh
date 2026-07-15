#!/usr/bin/env bash
set -Eeuo pipefail

if [ "$(id -u)" -ne 0 ]; then
    echo "Please run with sudo: sudo $0" >&2
    exit 1
fi

rule_file="/etc/udev/rules.d/99-apex-warning-light-ch340.rules"
cat > "$rule_file" <<'EOF'
# KernelMind Apex USB warning light: CH340 serial converter.
# Keep ModemManager from probing the light after /dev/ttyUSB* appears.
SUBSYSTEM=="usb", ATTR{idVendor}=="1a86", ATTR{idProduct}=="7523", ENV{ID_MM_DEVICE_IGNORE}="1"
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", ENV{ID_MM_DEVICE_IGNORE}="1", SYMLINK+="apex_warning_light"
EOF

udevadm control --reload-rules
udevadm trigger

if pgrep -x brltty >/dev/null 2>&1; then
    echo "Stopping brltty because it commonly detaches CH340 serial devices."
    systemctl stop brltty.service brltty-udev.service 2>/dev/null || true
    pkill brltty 2>/dev/null || true
fi

systemctl disable brltty.service brltty-udev.service 2>/dev/null || true
systemctl mask brltty.service brltty-udev.service 2>/dev/null || true

# Override the distro brltty udev rules. Without this, udev can still launch
# brltty-udev.service even when brltty.service is disabled.
ln -sfn /dev/null /etc/udev/rules.d/85-brltty.rules

echo "Installed $rule_file"
echo "Disabled brltty udev probing via /etc/udev/rules.d/85-brltty.rules"
echo "Now unplug/replug the warning light, then run:"
echo "  sudo /home/jjj/code/Apex_Deploy/scripts/bind_warning_light.sh"
