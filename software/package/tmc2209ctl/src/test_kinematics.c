// SPDX-License-Identifier: MIT
/* Host test for kinematics.c: make test */
#include <math.h>
#include <stdio.h>

#include "kinematics.h"

/* The arm as measured (tmc2209.conf) */
static const struct arm arm = {
	.l1 = 195.4, .l2 = 166.4,
	.min = { -109.25, -171 }, .max = { 111.75, 166 },
};

static int failures, checks;

#define CHECK(cond, ...) do {						\
	checks++;							\
	if (!(cond)) {							\
		failures++;						\
		printf("FAIL %s:%d: ", __FILE__, __LINE__);		\
		printf(__VA_ARGS__);					\
		putchar('\n');						\
	}								\
} while (0)

static int near_eq(double a, double b, double tol)
{
	return fabs(a - b) <= tol;
}

static void test_known_points(void)
{
	double th[2], x, y;

	/* Straight along +X */
	kin_forward(&arm, (double[]){ 0, 0 }, &x, &y);
	CHECK(near_eq(x, 361.8, 1e-9) && near_eq(y, 0, 1e-9),
	      "straight arm at (%f, %f)", x, y);
	CHECK(kin_inverse(&arm, 361.8, 0, ELBOW_ANY, NULL, th) == KIN_OK &&
	      near_eq(th[0], 0, 1e-3) && near_eq(th[1], 0, 1e-3),
	      "IK of full reach: %f %f", th[0], th[1]);

	/* Straight along +Y */
	kin_forward(&arm, (double[]){ 90, 0 }, &x, &y);
	CHECK(near_eq(x, 0, 1e-9) && near_eq(y, 361.8, 1e-9),
	      "straight arm up at (%f, %f)", x, y);

	/* Link 1 along +X, link 2 at a right angle: tip at (l1, l2) */
	kin_forward(&arm, (double[]){ 0, 90 }, &x, &y);
	CHECK(near_eq(x, 195.4, 1e-9) && near_eq(y, 166.4, 1e-9),
	      "elbow 90 at (%f, %f)", x, y);
	CHECK(kin_inverse(&arm, 195.4, 166.4, ELBOW_POS, NULL, th) == KIN_OK &&
	      near_eq(th[0], 0, 1e-9) && near_eq(th[1], 90, 1e-9),
	      "IK elbow 90: %f %f", th[0], th[1]);

	/* The mirror solution of the same point */
	CHECK(kin_inverse(&arm, 195.4, 166.4, ELBOW_NEG, NULL, th) == KIN_OK &&
	      th[1] < 0, "IK elbow negative: %f %f", th[0], th[1]);
	kin_forward(&arm, th, &x, &y);
	CHECK(near_eq(x, 195.4, 1e-9) && near_eq(y, 166.4, 1e-9),
	      "mirror solution reaches (%f, %f)", x, y);
}

static void test_unreachable(void)
{
	double th[2] = { 1234, 5678 };

	CHECK(kin_inverse(&arm, 362, 0, ELBOW_ANY, NULL, th) == KIN_TOO_FAR,
	      "362 mm should be too far");
	CHECK(kin_inverse(&arm, 0, -400, ELBOW_ANY, NULL, th) == KIN_TOO_FAR,
	      "400 mm should be too far");
	CHECK(kin_inverse(&arm, 10, 0, ELBOW_ANY, NULL, th) == KIN_TOO_CLOSE,
	      "10 mm should be too close");
	CHECK(kin_inverse(&arm, 0, 0, ELBOW_ANY, NULL, th) == KIN_TOO_CLOSE,
	      "the origin should be too close");
	/* Straight behind the shoulder (-X): θ1 = 180, outside -109.25..111.75 */
	CHECK(kin_inverse(&arm, -361.8, 0, ELBOW_ANY, NULL, th) == KIN_LIMITS,
	      "straight along -X should be outside the limits");
	/* 29 mm needs the elbow folded to 180, outside -171..166 */
	CHECK(kin_inverse(&arm, 29.0, 0, ELBOW_ANY, NULL, th) == KIN_LIMITS,
	      "fully folded should be outside the limits");
	CHECK(th[0] == 1234 && th[1] == 5678, "th[] written on failure");
}

/* IK(FK(θ)) = θ over the whole joint range */
static void test_round_trip(void)
{
	int n = 0;

	for (double t1 = arm.min[0]; t1 <= arm.max[0]; t1 += 2.5) {
		for (double t2 = arm.min[1]; t2 <= arm.max[1]; t2 += 2.5) {
			double th[2] = { t1, t2 }, back[2], x, y, bx, by;
			enum kin_result r;

			kin_forward(&arm, th, &x, &y);
			/* near = the start pose picks the same elbow side */
			r = kin_inverse(&arm, x, y, ELBOW_ANY, th, back);
			CHECK(r == KIN_OK, "IK(FK(%g, %g)) = %s", t1, t2,
			      kin_strerror(r));
			if (r != KIN_OK)
				continue;
			CHECK(near_eq(back[0], t1, 1e-6) &&
			      near_eq(back[1], t2, 1e-6),
			      "IK(FK(%g, %g)) = %.9f %.9f", t1, t2,
			      back[0], back[1]);
			kin_forward(&arm, back, &bx, &by);
			CHECK(near_eq(bx, x, 1e-6) && near_eq(by, y, 1e-6),
			      "FK(IK(%f, %f)) = %f %f", x, y, bx, by);

			/* Asking for the elbow side of θ2 gives the same */
			if (t2 != 0) {
				r = kin_inverse(&arm, x, y,
						t2 > 0 ? ELBOW_POS : ELBOW_NEG,
						NULL, back);
				CHECK(r == KIN_OK && near_eq(back[0], t1, 1e-6) &&
				      near_eq(back[1], t2, 1e-6),
				      "IK side of (%g, %g) = %.9f %.9f", t1,
				      t2, back[0], back[1]);
			}
			n++;
		}
	}
	CHECK(n > 5000, "only %d grid points", n);
}

/* ELBOW_ANY falls back to the side that fits the limits */
static void test_auto_side(void)
{
	double th[2], x, y;

	/*
	 * Shoulder near its min with the elbow bent clockwise: the mirror
	 * solution needs the shoulder further clockwise, past min (-162.7).
	 */
	kin_forward(&arm, (double[]){ -108, -60 }, &x, &y);
	CHECK(kin_inverse(&arm, x, y, ELBOW_POS, NULL, th) == KIN_LIMITS,
	      "positive elbow should be past the shoulder limit");
	CHECK(kin_inverse(&arm, x, y, ELBOW_ANY, NULL, th) == KIN_OK &&
	      near_eq(th[0], -108, 1e-6) && near_eq(th[1], -60, 1e-6),
	      "ELBOW_ANY picked %f %f", th[0], th[1]);
}

int main(void)
{
	test_known_points();
	test_unreachable();
	test_round_trip();
	test_auto_side();
	printf("kinematics: %d checks, %d failed\n", checks, failures);
	return failures ? 1 : 0;
}
