// SPDX-License-Identifier: MIT
/*
 * scara - arm-level commands for the two-joint SCARA arm.
 *
 * Joint 1 (shoulder) is motor 1, joint 2 (elbow) is motor 2.  Angles are
 * in degrees: the shoulder from the +X axis, the elbow from the line of
 * link 1 (0 = arm straight), counter-clockwise positive seen from above.
 * A positive 'tmc2209ctl turn' must turn the joint counter-clockwise; if
 * it doesn't, set invert_dir = 1 for that motor and flip its home_dir.
 *
 * The joint positions come from /run/scara.pos, written by homing and by
 * every move; arm moves refuse to run until both joints are homed.
 * Settings are read from /etc/tmc2209.conf (see tmc2209.h).
 */
#define _GNU_SOURCE
#include <ctype.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "tmc2209.h"

#define NJOINTS	2

static const char *const joint_name[NJOINTS] = { "shoulder", "elbow" };

static void usage(void)
{
	printf(
"Usage: scara [-c CONF] COMMAND [ARGS]\n"
"\n"
"  home [1|2|all]        home the joints (default all, elbow first)\n"
"  where                 print the joint angles\n"
"  joint A1 A2 [RPM]     move shoulder to A1 and elbow to A2 degrees\n"
"                        (absolute) together; needs homing, and refuses\n"
"                        angles outside min_angle..max_angle\n"
"  jog J DEG [RPM]       turn joint J (1 shoulder, 2 elbow) by DEG degrees\n"
"                        from where it is, ignoring the limits\n"
"\n"
"  Angles: shoulder from the +X axis, elbow from the line of link 1\n"
"  (0 = straight), counter-clockwise positive seen from above.  RPM is\n"
"  the speed of the motor that moves furthest (default %.0f).\n"
"  Ctrl-C decelerates a move to a stop.\n", motors[0].rpm);
}

/* Parse a number, rejecting trailing junk like "10x" */
static int parse_num(const char *s, double *v)
{
	char *end;

	*v = strtod(s, &end);
	if (end == s || *end || !isfinite(*v)) {
		err("'%s' is not a number", s);
		return -1;
	}
	return 0;
}

static int need_joints(void)
{
	if (nmotors < NJOINTS) {
		err("scara needs [motor1] and [motor2] in %s", CONF_PATH);
		return -1;
	}
	return 0;
}

static int cmd_where(void)
{
	double upd[NJOINTS];
	int j;

	for (j = 0; j < NJOINTS; j++) {
		upd[j] = usteps_per_deg(j);
		if (upd[j] <= 0)
			return -1;
	}
	pos_load();
	for (j = 0; j < NJOINTS; j++)
		printf("joint %d (%s): %8.2f deg%s\n", j + 1, joint_name[j],
		       pos[j] / upd[j],
		       homed[j] ? "" : "  (not homed: relative to power-on)");
	return 0;
}

/*
 * Move the joints by usteps[j] microsteps together; print where they end
 * up.  Ctrl-C stops early (decelerating), and the position stays counted.
 */
static int move_joints(int n, const int *mi, const double *usteps, double rpm)
{
	struct until u = { .fd = -1 };

	if (move_axes(n, mi, usteps, 0, rpm, &u))
		return -1;
	if (u.interrupted)
		printf("interrupted\n");
	return cmd_where();
}

static int cmd_joint(int argc, char **argv)
{
	static const int both[NJOINTS] = { 0, 1 };
	double target[NJOINTS], usteps[NJOINTS], upd, rpm = motors[0].rpm;
	int j;

	if (argc < 3 || argc > 4) {
		err("usage: joint A1 A2 [RPM]");
		return -1;
	}
	if (parse_num(argv[1], &target[0]) || parse_num(argv[2], &target[1]) ||
	    (argc > 3 && parse_num(argv[3], &rpm)))
		return -1;

	/* Check everything before moving anything */
	pos_load();
	for (j = 0; j < NJOINTS; j++) {
		const struct config *c = &motors[j];

		if (!homed[j]) {
			err("joint %d (%s) is not homed; run 'scara home'",
			    j + 1, joint_name[j]);
			return -1;
		}
		if (c->min_angle >= c->max_angle) {
			err("joint %d (%s): set min_angle < max_angle in "
			    "[motor%d] of %s (measure them with 'scara jog')",
			    j + 1, joint_name[j], j + 1, CONF_PATH);
			return -1;
		}
		if (target[j] < c->min_angle || target[j] > c->max_angle) {
			err("joint %d (%s): %.2f deg is outside its range "
			    "%.1f..%.1f", j + 1, joint_name[j], target[j],
			    c->min_angle, c->max_angle);
			return -1;
		}
	}
	for (j = 0; j < NJOINTS; j++) {
		upd = usteps_per_deg(j);
		if (upd <= 0)
			return -1;
		usteps[j] = lround(target[j] * upd) - pos[j];
	}
	return move_joints(NJOINTS, both, usteps, rpm);
}

static int cmd_jog(int argc, char **argv)
{
	double deg, usteps, upd, rpm = motors[0].rpm;
	int j;

	if (argc < 3 || argc > 4) {
		err("usage: jog J DEG [RPM]");
		return -1;
	}
	j = atoi(argv[1]) - 1;
	if (j < 0 || j >= NJOINTS) {
		err("joint must be 1 (shoulder) or 2 (elbow)");
		return -1;
	}
	if (parse_num(argv[2], &deg) || (argc > 3 && parse_num(argv[3], &rpm)))
		return -1;
	upd = usteps_per_deg(j);
	if (upd <= 0)
		return -1;
	usteps = lround(deg * upd);
	return move_joints(1, &j, &usteps, rpm);
}

/* home [1|2|all]: the elbow first, so the arm is folded before the shoulder swings */
static int cmd_home(int argc, char **argv)
{
	int mi[NJOINTS], n = 0;

	if (argc > 1 && strcmp(argv[1], "all") && strcmp(argv[1], "both")) {
		int j = atoi(argv[1]);

		if (j < 1 || j > NJOINTS) {
			err("usage: home [1|2|all]");
			return -1;
		}
		mi[n++] = j - 1;
	} else {
		mi[n++] = 1;
		mi[n++] = 0;
	}
	if (home_motors(n, mi))
		return -1;
	return cmd_where();
}

int main(int argc, char **argv)
{
	struct sigaction sa = { .sa_handler = on_sigint };
	const char *conf = getenv("TMC2209_CONF");
	const char *c;
	int i = 1;

	if (argc > 2 && !strcmp(argv[1], "-c")) {
		conf = argv[2];
		i = 3;
	}
	config_init(conf ? conf : CONF_PATH);
	if (i >= argc) {
		usage();
		return 1;
	}
	/* No SA_RESTART, like tmc2209ctl: Ctrl-C decelerates the move */
	sigaction(SIGINT, &sa, NULL);

	c = argv[i];
	argc -= i;
	argv += i;
	if (!strcmp(c, "help") || !strcmp(c, "-h") || !strcmp(c, "--help")) {
		usage();
		return 0;
	}
	if (need_joints())
		return 1;
	if (!strcmp(c, "where"))
		return cmd_where() ? 1 : 0;
	if (!strcmp(c, "joint"))
		return cmd_joint(argc, argv) ? 1 : 0;
	if (!strcmp(c, "jog"))
		return cmd_jog(argc, argv) ? 1 : 0;
	if (!strcmp(c, "home"))
		return cmd_home(argc, argv) ? 1 : 0;
	err("unknown command '%s' (try 'scara help')", c);
	return 1;
}
