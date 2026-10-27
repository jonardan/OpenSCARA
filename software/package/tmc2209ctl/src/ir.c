// SPDX-License-Identifier: MIT
/*
 * ir - button codes from an IR remote, see ir.h.
 *
 * The receiver is /sys/class/rc/rcN; its decoders are switched on through
 * rcN/protocols and its codes come as EV_MSC/MSC_SCAN events from the
 * input device rcN/inputM/eventK, i.e. /dev/input/eventK.  No keymap: the
 * scancodes are used as they are.
 *
 * The AM335x can take ~1 ms to wake from its deeper idle states after an
 * interrupt, as long as an NEC bit (560 us): while the receiver is open
 * the CPU is kept out of them through /dev/cpu_dma_latency, which holds
 * as long as the file stays open.  (The garbled frames seen on
 * 2026-10-27 were mostly the receiver's slow rise at 3.3 V, fixed with a
 * 10k pull-up on P8_16.)
 */
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <glob.h>
#include <limits.h>
#include <linux/input.h>
#include <poll.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "ir.h"
#include "tmc2209.h"

/* Every IR decoder the kernel may have; the ones it lacks are refused */
static const char *const ir_protocols[] = {
	"nec", "rc-5", "rc-5-sz", "rc-6", "jvc", "sony", "sanyo", "sharp",
	"mce_kbd", "xmp", "imon", "rc-mm",
};

/* Switch on the decoders of the receiver at rcdir; how many took */
static int ir_enable(const char *rcdir)
{
	char path[PATH_MAX + 16], val[32];
	size_t i;
	int fd, n = 0;

	snprintf(path, sizeof(path), "%s/protocols", rcdir);
	for (i = 0; i < sizeof(ir_protocols) / sizeof(ir_protocols[0]); i++) {
		fd = open(path, O_WRONLY);
		if (fd < 0) {
			err("%s: %s", path, strerror(errno));
			return 0;
		}
		snprintf(val, sizeof(val), "+%s", ir_protocols[i]);
		if (write(fd, val, strlen(val)) > 0)
			n++;
		close(fd);
	}
	return n;
}

/* Keep the CPU out of slow idle states until the program exits */
static void ir_no_idle(void)
{
	static int fd = -1;
	int32_t zero = 0;

	if (fd >= 0)
		return;
	fd = open("/dev/cpu_dma_latency", O_WRONLY | O_CLOEXEC);
	if (fd < 0 || write(fd, &zero, sizeof(zero)) != sizeof(zero))
		fprintf(stderr, "warning: /dev/cpu_dma_latency: %s; the remote "
			"may not be decoded\n", strerror(errno));
}

int ir_open(void)
{
	char ev[PATH_MAX], rcdir[PATH_MAX], dev[PATH_MAX + 16], *slash;
	glob_t g;
	int fd, i;

	if (glob("/sys/class/rc/rc*/input*/event*", 0, NULL, &g) ||
	    !g.gl_pathc) {
		err("no IR receiver in /sys/class/rc: is it in the device tree "
		    "(gpio-ir-receiver on P8_16) and the kernel (IR_GPIO_CIR)?");
		globfree(&g);
		return -1;
	}
	snprintf(ev, sizeof(ev), "%s", g.gl_pathv[0]);
	globfree(&g);
	/* rcN/inputM/eventK: rcN is two levels up */
	snprintf(rcdir, sizeof(rcdir), "%s", ev);
	for (i = 0; i < 2; i++)
		if ((slash = strrchr(rcdir, '/')))
			*slash = '\0';
	ir_no_idle();
	if (!ir_enable(rcdir)) {
		err("%s: no IR decoder could be switched on", rcdir);
		return -1;
	}
	snprintf(dev, sizeof(dev), "/dev/input/%s", strrchr(ev, '/') + 1);
	fd = open(dev, O_RDONLY | O_NONBLOCK | O_CLOEXEC);
	if (fd < 0)
		err("%s: %s", dev, strerror(errno));
	return fd;
}

int ir_read(int fd, int timeout_ms, uint32_t *code)
{
	struct pollfd p = { .fd = fd, .events = POLLIN };
	struct input_event e;
	ssize_t n;
	int r;

	for (;;) {
		/* Codes already read but not yet seen go first */
		while ((n = read(fd, &e, sizeof(e))) == sizeof(e))
			if (e.type == EV_MSC && e.code == MSC_SCAN) {
				*code = e.value;
				return 1;
			}
		if (n < 0 && errno != EAGAIN) {
			if (errno != EINTR)
				err("IR receiver: %s", strerror(errno));
			return -1;
		}
		if (stop_req)
			return -1;
		r = poll(&p, 1, timeout_ms);
		if (r < 0) {
			if (errno != EINTR)
				err("IR receiver: %s", strerror(errno));
			return -1;
		}
		if (r == 0)
			return 0;
	}
}

int ir_drain(int fd, int quiet_ms)
{
	uint32_t code;
	int r;

	while ((r = ir_read(fd, quiet_ms, &code)) > 0)
		;
	return r;
}
