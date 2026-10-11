#!/bin/sh
# Boot the BeagleBone Black with the TMC2209 device tree
set -eu

BOARD_DIR="$(dirname "$0")"

install -m 0644 -D "${BOARD_DIR}/extlinux.conf" "${BINARIES_DIR}/extlinux/extlinux.conf"
sed -i '/^fdtfile=/d' "${BINARIES_DIR}/uEnv.txt"
echo "fdtfile=am335x-boneblack-tmc2209.dtb" >> "${BINARIES_DIR}/uEnv.txt"
