#!/bin/sh
# Add a login prompt on the USB gadget serial port (see S30usbgadget)
set -eu

grep -q '^ttyGS0::' "${TARGET_DIR}/etc/inittab" || \
	sed -i '/GENERIC_SERIAL/a ttyGS0::respawn:/sbin/getty -L ttyGS0 115200 vt100 # USB_GADGET_SERIAL' \
		"${TARGET_DIR}/etc/inittab"
