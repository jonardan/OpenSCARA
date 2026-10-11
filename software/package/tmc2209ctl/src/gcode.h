/* SPDX-License-Identifier: MIT */
/*
 * G-code: one line of text into what it asks for, without moving anything.
 * Pure parsing, no hardware: it builds and is tested on the PC (make test).
 *
 * Supported: G0 (rapid move), G1 (straight line at feed F, mm/min), G28
 * (home), G90 / G91 (absolute / relative), G21 (mm), M3 / M5 and the
 * aliases M106 / M107 (tool on / off), M2 / M30 (end of program).
 * Comments ';' to the end of the line and '( )'; line numbers N and
 * RepRap checksums '*nn' are skipped, '%' lines are empty.  Z is parsed
 * and reported (the arm has no Z), S is accepted on tool lines.
 */
#ifndef GCODE_H
#define GCODE_H

#include <stddef.h>

enum gc_dist { GC_DIST_KEEP = 0, GC_ABS, GC_REL };
enum gc_tool { GC_TOOL_KEEP = 0, GC_TOOL_ON, GC_TOOL_OFF };

/* Modal state carried from one line to the next: G0/G1 */
struct gc_state {
	int motion;		/* -1 none yet, 0 = G0, 1 = G1 */
};

/*
 * One parsed line.  When several things are on a line they are meant to
 * run in this order: dist, home, feed, move, tool, end.
 */
struct gc_line {
	enum gc_dist dist;	/* G90 / G91 on this line */
	int home;		/* G28 */
	int move;		/* X and/or Y given: move (motion says how) */
	int motion;		/* 0 = G0 rapid, 1 = G1 line */
	int has_x, has_y, has_z, has_f;
	double x, y, z;		/* mm, absolute or relative per dist */
	double f;		/* feed, mm/min */
	enum gc_tool tool;
	int end;		/* M2 / M30 */
};

void gc_init(struct gc_state *st);

/*
 * Parse one line (no newline needed) into *out, updating *st.  Returns 0,
 * or -1 with a message in err[errlen] (without the line number) and *st
 * untouched.
 */
int gc_parse(struct gc_state *st, const char *line, struct gc_line *out,
	     char *err, size_t errlen);

#endif
