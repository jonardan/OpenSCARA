/* SPDX-License-Identifier: MIT */
/*
 * Straight lines of the tip: the line split into short, equal segments,
 * with the joint angles at every point.  Pure math, no hardware: it builds
 * and is tested on the PC (make test).
 */
#ifndef LINE_H
#define LINE_H

#include "kinematics.h"

#define LINE_SEG_MM	1.0	/* longest segment */
#define LINE_JUMP_DEG	10.0	/* most a joint may turn over one segment */

/* Points (ends included) for a line of len mm in segments <= seg mm */
int line_npts(double len, double seg);

/*
 * Joint angles th[0..npts-1] of npts evenly spaced points on the line from
 * the tip at joint angles start[] to (x, y).  th[0] is start[]; the elbow
 * stays on the side it starts on.  On error, *bx / *by is the first point
 * that fails: KIN_TOO_FAR / KIN_TOO_CLOSE / KIN_LIMITS as kin_inverse(),
 * or KIN_JUMP when a joint would turn more than LINE_JUMP_DEG between two
 * points (the line passes too close to the shoulder axis).
 */
enum kin_result line_plan(const struct arm *a, const double start[2],
			  double x, double y, int npts, double (*th)[2],
			  double *bx, double *by);

#endif
