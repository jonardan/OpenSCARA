// SPDX-License-Identifier: MIT
/* Host test for line.c: make test */
#include <math.h>
#include <stdio.h>

#include "line.h"

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

#define MAXPTS	1000

/* Joint angles of (x, y) on the given elbow side */
static void ik(double x, double y, enum elbow side, double th[2])
{
	if (kin_inverse(&arm, x, y, side, NULL, th) != KIN_OK)
		printf("test setup: %f, %f unreachable\n", x, y);
}

/* Plan a line from (x0, y0) to (x, y) and check every point lies on it */
static void check_line(double x0, double y0, double x, double y,
		       enum elbow side)
{
	double start[2], th[MAXPTS][2], bx, by, len = hypot(x - x0, y - y0);
	int n = line_npts(len, LINE_SEG_MM), i;
	enum kin_result r;

	ik(x0, y0, side, start);
	r = line_plan(&arm, start, x, y, n, th, &bx, &by);
	CHECK(r == KIN_OK, "line %g,%g -> %g,%g: %s at %g,%g", x0, y0, x, y,
	      kin_strerror(r), bx, by);
	if (r != KIN_OK)
		return;
	CHECK(th[0][0] == start[0] && th[0][1] == start[1],
	      "point 0 is not the start pose");
	for (i = 0; i < n; i++) {
		double px, py, t = (double)i / (n - 1);

		kin_forward(&arm, th[i], &px, &py);
		CHECK(hypot(px - (x0 + (x - x0) * t),
			    py - (y0 + (y - y0) * t)) < 1e-6,
		      "point %d of %g,%g -> %g,%g is at %g,%g", i, x0, y0, x,
		      y, px, py);
		CHECK(side == ELBOW_POS ? th[i][1] >= 0 : th[i][1] <= 0,
		      "point %d: elbow changed side (%g)", i, th[i][1]);
	}
	CHECK(len / (n - 1) <= LINE_SEG_MM + 1e-9 && n >= 2,
	      "segments of %g mm", len / (n - 1));
}

static void test_npts(void)
{
	CHECK(line_npts(100, 1) == 101, "100 mm: %d", line_npts(100, 1));
	CHECK(line_npts(100.5, 1) == 102, "100.5 mm: %d", line_npts(100.5, 1));
	CHECK(line_npts(0.3, 1) == 2, "0.3 mm: %d", line_npts(0.3, 1));
	CHECK(line_npts(0, 1) == 2, "0 mm: %d", line_npts(0, 1));
}

/* The 100 mm square of the board readme, both elbow sides, and its diagonals */
static void test_square(void)
{
	static const double c[5][2] = {
		{ 150, -50 }, { 250, -50 }, { 250, 50 }, { 150, 50 },
		{ 150, -50 },
	};

	for (int i = 0; i < 4; i++) {
		check_line(c[i][0], c[i][1], c[i + 1][0], c[i + 1][1], ELBOW_POS);
		check_line(c[i][0], c[i][1], c[i + 1][0], c[i + 1][1], ELBOW_NEG);
	}
	check_line(150, -50, 250, 50, ELBOW_POS);
	check_line(250, -50, 150, 50, ELBOW_NEG);
	/* Long line across the front, and one toward the shoulder */
	check_line(200, -200, 200, 200, ELBOW_POS);
	check_line(330, 0, 60, 0, ELBOW_NEG);
}

static void test_errors(void)
{
	double start[2], th[MAXPTS][2], bx, by;
	enum kin_result r;
	int n;

	/* Ends out of reach: the first bad point is just past full reach */
	ik(300, 0, ELBOW_POS, start);
	n = line_npts(100, LINE_SEG_MM);
	r = line_plan(&arm, start, 400, 0, n, th, &bx, &by);
	CHECK(r == KIN_TOO_FAR && bx > 361.8 && bx < 363 && fabs(by) < 1e-9,
	      "too far: %s at %g,%g", kin_strerror(r), bx, by);

	/* Straight through the shoulder axis */
	ik(100, 0, ELBOW_POS, start);
	n = line_npts(200, LINE_SEG_MM);
	r = line_plan(&arm, start, -100, 0, n, th, &bx, &by);
	CHECK(r == KIN_TOO_CLOSE || r == KIN_JUMP || r == KIN_LIMITS,
	      "through the shoulder: %s at %g,%g", kin_strerror(r), bx, by);

	/* Behind the arm: the shoulder would pass its limit */
	ik(0, 300, ELBOW_POS, start);
	n = line_npts(hypot(300, 300), LINE_SEG_MM);
	r = line_plan(&arm, start, -300, 0, n, th, &bx, &by);
	CHECK(r == KIN_LIMITS, "behind: %s at %g,%g", kin_strerror(r), bx,
	      by);
}

int main(void)
{
	test_npts();
	test_square();
	test_errors();
	printf("line: %d checks, %d failed\n", checks, failures);
	return failures != 0;
}
