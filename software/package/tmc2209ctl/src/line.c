// SPDX-License-Identifier: MIT
/*
 * Straight lines of the tip (see line.h).
 *
 * A joint move from A to B traces a curve; a line is drawn by going
 * through IK points about a millimetre apart, so between two of them the
 * joint move strays from the line by a negligible amount.
 */
#include <math.h>
#include <stddef.h>

#include "line.h"

int line_npts(double len, double seg)
{
	int n = (int)ceil(len / seg - 1e-9);

	return (n < 1 ? 1 : n) + 1;
}

enum kin_result line_plan(const struct arm *a, const double start[2],
			  double x, double y, int npts, double (*th)[2],
			  double *bx, double *by)
{
	enum elbow side = start[1] < 0 ? ELBOW_NEG : ELBOW_POS;
	double x0, y0;
	int i;

	kin_forward(a, start, &x0, &y0);
	th[0][0] = start[0];
	th[0][1] = start[1];
	for (i = 1; i < npts; i++) {
		double t = (double)i / (npts - 1);
		double px = x0 + (x - x0) * t, py = y0 + (y - y0) * t;
		enum kin_result r;

		if (i == npts - 1) {
			px = x;
			py = y;
		}
		r = kin_inverse(a, px, py, side, NULL, th[i]);
		if (r == KIN_OK &&
		    (fabs(th[i][0] - th[i - 1][0]) > LINE_JUMP_DEG ||
		     fabs(th[i][1] - th[i - 1][1]) > LINE_JUMP_DEG))
			r = KIN_JUMP;
		if (r != KIN_OK) {
			*bx = px;
			*by = py;
			return r;
		}
	}
	return KIN_OK;
}
