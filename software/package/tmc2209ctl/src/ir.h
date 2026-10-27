/* SPDX-License-Identifier: MIT */
/*
 * ir - button codes from an IR remote, through the kernel's gpio-ir-recv
 * receiver (VS1838 on P8_16 in the device tree).  The kernel decodes the
 * pulses (NEC, RC-5, RC-6, Sony, ...) and reports each button press, and
 * every repeat while it is held (~110 ms apart), as one scancode.
 */
#ifndef IR_H
#define IR_H

#include <stdint.h>

/* Open the receiver with every decoder on; the input device in -1 on error */
int ir_open(void);

/*
 * Wait up to timeout_ms (-1: for ever) for a scancode: 1 with *code set,
 * 0 on timeout, -1 on error or a signal (Ctrl-C)
 */
int ir_read(int fd, int timeout_ms, uint32_t *code);

/* Throw away codes until none comes for quiet_ms; -1 on a signal */
int ir_drain(int fd, int quiet_ms);

#endif
