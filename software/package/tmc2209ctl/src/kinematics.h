/* SPDX-License-Identifier: MIT */
/*
 * Kinematics of the two-link SCARA arm: joint angles <-> tip X/Y.
 *
 * Pure math, no hardware: it builds and is tested on the PC (make test).
 * Angles in degrees: θ1 (shoulder) from the +X axis, θ2 (elbow) from the
 * line of link 1 (0 = straight), counter-clockwise positive seen from
 * above.  The shoulder axis is (0, 0); X/Y in mm.
 */
#ifndef KINEMATICS_H
#define KINEMATICS_H

struct arm {
	double l1;		/* shoulder axis to elbow axis, mm */
	double l2;		/* elbow axis to tool tip, mm */
	double min[2];		/* joint limits (deg), shoulder and elbow */
	double max[2];
	int floor_on;		/* the tip must stay at y >= floor_y */
	double floor_y;		/* mm: the table, seen from the arm */
};

/* Which way the elbow bends, for kin_inverse() */
enum elbow {
	ELBOW_ANY = 0,		/* whichever fits the limits (see kin_inverse) */
	ELBOW_POS = 1,		/* θ2 >= 0: link 2 turns counter-clockwise */
	ELBOW_NEG = -1,		/* θ2 <= 0: link 2 turns clockwise */
};

/* kin_inverse() results */
enum kin_result {
	KIN_OK = 0,
	KIN_TOO_FAR,		/* r > l1 + l2 */
	KIN_TOO_CLOSE,		/* r < |l1 - l2| */
	KIN_LIMITS,		/* reachable, but not inside the joint limits */
	KIN_JUMP,		/* line.c: a joint would jump between two points */
	KIN_BELOW,		/* y < floor_y: the tip would hit the table */
};

void kin_forward(const struct arm *a, const double th[2], double *x,
		 double *y);

/*
 * Joint angles th[] that put the tip at (x, y).  With ELBOW_ANY and both
 * solutions inside the limits, the one closer to near[] (the current
 * angles) is chosen, or ELBOW_POS if near is NULL.  th[] is only written
 * on KIN_OK.
 */
enum kin_result kin_inverse(const struct arm *a, double x, double y,
			    enum elbow elbow, const double *near, double th[2]);

/*
 * A joint move from[] to to[] turns both joints together (they start and
 * finish at the same time), so the tip follows the joint-space line, not
 * a straight one.  KIN_OK if every point of it, 1 degree apart, stays at
 * y >= floor_y; else KIN_BELOW with the first point below in *bx, *by.
 */
enum kin_result kin_joint_move(const struct arm *a, const double from[2],
			       const double to[2], double *bx, double *by);

const char *kin_strerror(enum kin_result r);

#endif
