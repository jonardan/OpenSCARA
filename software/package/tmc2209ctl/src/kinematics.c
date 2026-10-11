// SPDX-License-Identifier: MIT
/*
 * Kinematics of the two-link SCARA arm (see kinematics.h).
 *
 * Forward: the elbow is at l1 along θ1, the tip at l2 further along
 * θ1 + θ2.  Inverse, with r² = x² + y²:
 *
 *   cos θ2 = (r² - l1² - l2²) / (2 l1 l2)
 *   θ1     = atan2(y, x) - atan2(l2 sin θ2, l1 + l2 cos θ2)
 *
 * giving two solutions, θ2 = ±acos(...), mirror images about the line
 * from the shoulder to the tip.
 */
#include <math.h>
#include <stddef.h>

#include "kinematics.h"

#define DEG	(M_PI / 180.0)
/*
 * Rounding slack, so a point at exactly full reach or exactly on a joint
 * limit (FK of such a pose) still solves
 */
#define COS_EPS	1e-9
#define LIM_EPS	1e-6	/* deg */

void kin_forward(const struct arm *a, const double th[2], double *x, double *y)
{
	double t1 = th[0] * DEG, t12 = (th[0] + th[1]) * DEG;

	*x = a->l1 * cos(t1) + a->l2 * cos(t12);
	*y = a->l1 * sin(t1) + a->l2 * sin(t12);
}

/*
 * Bring angle t inside min..max by whole turns if it can be; returns 0 if
 * it fits.  atan2 gives -180..180, but a joint range may sit across ±180.
 */
static int fit(double *t, double min, double max)
{
	for (int k = -1; k <= 1; k++) {
		double v = *t + 360.0 * k;

		if (v >= min - LIM_EPS && v <= max + LIM_EPS) {
			*t = fmax(min, fmin(max, v));
			return 0;
		}
	}
	return -1;
}

/* One of the two solutions, sign = +1 or -1 for the elbow; 0 if inside the limits */
static int solve(const struct arm *a, double x, double y, double c2, int sign,
		 double th[2])
{
	double s2 = sign * sqrt(1.0 - c2 * c2);

	th[1] = atan2(s2, c2) / DEG;
	th[0] = (atan2(y, x) - atan2(a->l2 * s2, a->l1 + a->l2 * c2)) / DEG;
	if (th[0] <= -180.0)
		th[0] += 360.0;
	else if (th[0] > 180.0)
		th[0] -= 360.0;
	if (fit(&th[0], a->min[0], a->max[0]))
		return -1;
	return fit(&th[1], a->min[1], a->max[1]);
}

/* How far the joints must turn from near[] to th[] (deg, summed) */
static double dist(const double *near, const double th[2])
{
	return fabs(th[0] - near[0]) + fabs(th[1] - near[1]);
}

enum kin_result kin_inverse(const struct arm *a, double x, double y,
			    enum elbow elbow, const double *near, double th[2])
{
	double r2 = x * x + y * y, c2, pos[2], neg[2];
	int pos_ok, neg_ok;

	c2 = (r2 - a->l1 * a->l1 - a->l2 * a->l2) / (2.0 * a->l1 * a->l2);
	if (c2 > 1.0 + COS_EPS)
		return KIN_TOO_FAR;
	if (c2 < -1.0 - COS_EPS)
		return KIN_TOO_CLOSE;
	c2 = fmax(-1.0, fmin(1.0, c2));

	pos_ok = elbow != ELBOW_NEG && !solve(a, x, y, c2, 1, pos);
	neg_ok = elbow != ELBOW_POS && !solve(a, x, y, c2, -1, neg);
	if (pos_ok && neg_ok && near && dist(near, neg) < dist(near, pos))
		pos_ok = 0;
	if (pos_ok) {
		th[0] = pos[0];
		th[1] = pos[1];
	} else if (neg_ok) {
		th[0] = neg[0];
		th[1] = neg[1];
	} else {
		return KIN_LIMITS;
	}
	return KIN_OK;
}

const char *kin_strerror(enum kin_result r)
{
	switch (r) {
	case KIN_OK: return "ok";
	case KIN_TOO_FAR: return "out of reach (too far from the shoulder)";
	case KIN_TOO_CLOSE: return "out of reach (too close to the shoulder)";
	case KIN_LIMITS: return "reachable only outside the joint limits";
	}
	return "unknown error";
}
