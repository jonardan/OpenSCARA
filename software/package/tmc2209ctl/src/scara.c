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
#include <errno.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "gcode.h"
#include "kinematics.h"
#include "line.h"
#include "tmc2209.h"

#define NJOINTS	2

static const char *const joint_name[NJOINTS] = { "shoulder", "elbow" };

/* Set by 'gcode': moves don't print where they end up */
static int quiet;
/* A move was stopped by Ctrl-C */
static int stopped;
/* 'gcode -': answer each line with ok instead of printing it */
static int stream;

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
"  fk A1 A2              print the tip X/Y (mm) for joint angles A1 A2\n"
"  ik X Y [+|-]          print the joint angles for tip X/Y (mm); + or -\n"
"                        picks the elbow side (default: one inside the\n"
"                        limits, the closer to the current pose)\n"
"                        fk and ik only compute, they don't move\n"
"  goto X Y [+|-] [RPM]  move the tip to X/Y (mm): ik, then the joint\n"
"                        move; needs homing\n"
"  line X Y [F]          move the tip to X/Y (mm) in a straight line at F\n"
"                        mm/s (default line_speed, %.0f); needs homing,\n"
"                        and the elbow stays on the side it is on\n"
"  gcode [-n] FILE       run a G-code file: G0 X Y (goto), G1 X Y F\n"
"                        (line, F in mm/min), G28 (home), G90 / G91, G21;\n"
"                        M3/M5 (tool on/off: the tip LED lights only\n"
"                        while G1 draws with the tool on), M150 R U B\n"
"                        (its color, 0-255, white by default); Z ignored.\n"
"                        Every line is checked before anything moves;\n"
"                        -n only checks.\n"
"                        FILE - streams stdin: each line is checked, run\n"
"                        and answered ok (or error, and it stops)\n"
"  led [COLOR]           set the tip RGB LED, or print its color: off,\n"
"                        on (white), red, green, blue, yellow, cyan,\n"
"                        magenta, orange, purple, #rrggbb or R G B (0-255)\n"
"\n"
"  Angles: shoulder from the +X axis, elbow from the line of link 1\n"
"  (0 = straight), counter-clockwise positive seen from above.  X/Y in\n"
"  mm from the shoulder axis; angles 0 0 = arm straight along +X.  RPM is\n"
"  the speed of the motor that moves furthest (default %.0f).\n"
"  Ctrl-C decelerates a move to a stop.\n", motors[0].line_speed,
	motors[0].rpm);
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

/* The arm geometry and joint limits from the config file */
static int get_arm(struct arm *a)
{
	a->l1 = motors[0].link1_mm;
	a->l2 = motors[0].link2_mm;
	if (a->l1 <= 0 || a->l2 <= 0) {
		err("set link1_mm and link2_mm in %s", CONF_PATH);
		return -1;
	}
	for (int j = 0; j < NJOINTS; j++) {
		a->min[j] = motors[j].min_angle;
		a->max[j] = motors[j].max_angle;
	}
	a->table_on = !motors[0].max_y_off;
	a->table_y = motors[0].max_y_mm;
	return 0;
}

/* Current joint angles into th[]; 0 if both joints are homed */
static int get_angles(double th[NJOINTS])
{
	int j, ok = 1;

	pos_load();
	for (j = 0; j < NJOINTS; j++) {
		double upd = usteps_per_deg(j);

		if (upd <= 0)
			return -1;
		th[j] = pos[j] / upd;
		ok &= homed[j];
	}
	return ok ? 0 : 1;
}

static int cmd_where(void)
{
	double th[NJOINTS], x, y;
	struct arm a;
	int j, r = get_angles(th);

	if (r < 0)
		return -1;
	for (j = 0; j < NJOINTS; j++)
		printf("joint %d (%s): %8.2f deg%s\n", j + 1, joint_name[j],
		       th[j],
		       homed[j] ? "" : "  (not homed: relative to power-on)");
	if (r == 0 && motors[0].link1_mm > 0 && motors[0].link2_mm > 0) {
		get_arm(&a);
		kin_forward(&a, th, &x, &y);
		printf("tip: x %8.2f mm, y %8.2f mm\n", x, y);
	}
	return 0;
}

static int cmd_fk(int argc, char **argv)
{
	double th[NJOINTS], x, y;
	struct arm a;

	if (argc != 3) {
		err("usage: fk A1 A2");
		return -1;
	}
	if (parse_num(argv[1], &th[0]) || parse_num(argv[2], &th[1]) ||
	    get_arm(&a))
		return -1;
	kin_forward(&a, th, &x, &y);
	printf("tip: x %8.2f mm, y %8.2f mm\n", x, y);
	return 0;
}

static int cmd_ik(int argc, char **argv)
{
	double x, y, th[NJOINTS], now[NJOINTS];
	enum elbow side = ELBOW_ANY;
	enum kin_result r;
	struct arm a;

	if (argc < 3 || argc > 4) {
		err("usage: ik X Y [+|-]");
		return -1;
	}
	if (parse_num(argv[1], &x) || parse_num(argv[2], &y) || get_arm(&a))
		return -1;
	if (argc > 3) {
		if (!strcmp(argv[3], "+")) {
			side = ELBOW_POS;
		} else if (!strcmp(argv[3], "-")) {
			side = ELBOW_NEG;
		} else {
			err("elbow side must be + or -");
			return -1;
		}
	}
	/* Joint limits not measured: kin_inverse would reject everything */
	for (int j = 0; j < NJOINTS; j++) {
		if (a.min[j] >= a.max[j]) {
			err("joint %d (%s): set min_angle < max_angle in "
			    "[motor%d] of %s", j + 1, joint_name[j], j + 1,
			    CONF_PATH);
			return -1;
		}
	}
	/* Closer to the current pose, if it is known */
	pos_load();
	r = kin_inverse(&a, x, y, side,
			homed[0] && homed[1] && get_angles(now) == 0 ? now : NULL,
			th);
	if (r != KIN_OK) {
		err("%.2f, %.2f: %s", x, y, kin_strerror(r));
		return -1;
	}
	printf("joint 1 (%s): %8.2f deg\njoint 2 (%s): %8.2f deg\n",
	       joint_name[0], th[0], joint_name[1], th[1]);
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
	if (u.interrupted) {
		printf("interrupted\n");
		stopped = 1;
	}
	return quiet ? 0 : cmd_where();
}

/*
 * Move both joints together to absolute angles target[] (deg).  Checks
 * everything before moving anything: both joints homed, limits measured,
 * target inside them.
 */
static int move_to_angles(const double target[NJOINTS], double rpm)
{
	static const int both[NJOINTS] = { 0, 1 };
	double usteps[NJOINTS], upd;
	int j;

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

static int cmd_joint(int argc, char **argv)
{
	double target[NJOINTS], rpm = motors[0].rpm;

	if (argc < 3 || argc > 4) {
		err("usage: joint A1 A2 [RPM]");
		return -1;
	}
	if (parse_num(argv[1], &target[0]) || parse_num(argv[2], &target[1]) ||
	    (argc > 3 && parse_num(argv[3], &rpm)))
		return -1;
	return move_to_angles(target, rpm);
}

static int line_to(const struct arm *a, double x, double y, double speed);

/*
 * A straight line from the joint angles th[] to x, y, on paper (line.c):
 * KIN_OK with th[] set to where it ends, or the first bad point in *bx, *by.
 */
static enum kin_result plan_line(const struct arm *a, double x, double y,
				 double th[NJOINTS], double *bx, double *by)
{
	double x0, y0, (*pts)[NJOINTS];
	enum kin_result r;
	int npts;

	*bx = x;
	*by = y;
	kin_forward(a, th, &x0, &y0);
	npts = line_npts(hypot(x - x0, y - y0), LINE_SEG_MM);
	pts = malloc(npts * sizeof(*pts));
	if (!pts) {
		err("out of memory");
		exit(1);
	}
	r = line_plan(a, th, x, y, npts, pts, bx, by);
	if (r == KIN_OK)
		memcpy(th, pts[npts - 1], sizeof(pts[0]));
	free(pts);
	return r;
}

/* How a goto (G0) gets there, from plan_g0() */
struct g0_plan {
	int straight;		/* a straight line (line_to) */
	int via;		/* two joint moves: to via_th[], then to[] */
	double via_th[NJOINTS], to[NJOINTS];
};

/*
 * How a goto (G0) gets the tip from the joint angles from[] to x, y.  A
 * joint move turns both joints together, so the tip swings along a curve
 * that can swing past max_y_mm (into the table) even when both ends are
 * clear of it: out of the folded home pose it swings 100-350 mm towards
 * the table.  In order, to the IK solution nearest from[] and then (with
 * ELBOW_ANY) to the other elbow side: one joint move; the shoulder first,
 * then the elbow (out of home the folded tip then circles close to the
 * shoulder); the elbow first, then the shoulder.  Last, a straight line,
 * which can't cross the table since its edge is straight.  KIN_OK with *p
 * filled; else the first bad point of the direct move in *bx, *by.
 */
static enum kin_result plan_g0(const struct arm *a, double x, double y,
			       enum elbow side, const double from[NJOINTS],
			       struct g0_plan *p, double *bx, double *by)
{
	double sol[2][NJOINTS], v[NJOINTS], end[NJOINTS], tx, ty;
	enum kin_result r;
	int n = 1, i, k;

	memset(p, 0, sizeof(*p));
	*bx = x;
	*by = y;
	r = kin_inverse(a, x, y, side, from, sol[0]);
	if (r != KIN_OK)
		return r;
	if (side == ELBOW_ANY &&
	    kin_inverse(a, x, y, sol[0][1] < 0 ? ELBOW_POS : ELBOW_NEG, NULL,
			sol[1]) == KIN_OK)
		n = 2;
	for (i = 0; i < n; i++)
		if (kin_joint_move(a, from, sol[i], &tx, &ty) == KIN_OK) {
			memcpy(p->to, sol[i], sizeof(p->to));
			return KIN_OK;
		}
	for (k = 0; k < 2; k++) {
		for (i = 0; i < n; i++) {
			/*
			 * k = 0: shoulder first; k = 1: elbow first.  The
			 * joint that waits is brought inside its limits: home
			 * (elbow 171) is past max_angle (166).
			 */
			v[0] = k ? fmax(a->min[0], fmin(a->max[0], from[0])) :
				   sol[i][0];
			v[1] = k ? sol[i][1] :
				   fmax(a->min[1], fmin(a->max[1], from[1]));
			if (kin_joint_move(a, from, v, &tx, &ty) == KIN_OK &&
			    kin_joint_move(a, v, sol[i], &tx, &ty) == KIN_OK) {
				p->via = 1;
				memcpy(p->via_th, v, sizeof(v));
				memcpy(p->to, sol[i], sizeof(p->to));
				return KIN_OK;
			}
		}
	}
	memcpy(end, from, sizeof(end));
	if (plan_line(a, x, y, end, &tx, &ty) == KIN_OK) {
		p->straight = 1;
		memcpy(p->to, end, sizeof(p->to));
		return KIN_OK;
	}
	kin_joint_move(a, from, sol[0], bx, by);
	return KIN_TABLE;
}

/* G0 / goto as a straight line: as fast as the motors' rpm allows */
#define G0_LINE_SPEED	1000	/* mm/s, lowered by move_path */

/*
 * The tip to x, y: IK, then the same joint move as 'joint', or the safer
 * moves plan_g0 picks when that one would swing the tip into the table
 */
static int goto_xy(const struct arm *a, double x, double y, enum elbow side,
		   double rpm)
{
	double now[NJOINTS], bx, by;
	struct g0_plan p;
	enum kin_result r;

	if (get_angles(now)) {
		err("the arm is not homed; run 'scara home'");
		return -1;
	}
	r = plan_g0(a, x, y, side, now, &p, &bx, &by);
	if (r != KIN_OK) {
		if (r == KIN_TABLE)
			err("%.2f, %.2f: the move would take the tip to %.2f, "
			    "%.2f: %s", x, y, bx, by, kin_strerror(r));
		else
			err("%.2f, %.2f: %s", x, y, kin_strerror(r));
		return -1;
	}
	if (p.straight)
		return line_to(a, x, y, G0_LINE_SPEED);
	if (p.via && move_to_angles(p.via_th, rpm))
		return -1;
	return move_to_angles(p.to, rpm);
}

/* goto X Y [+|-] [RPM] */
static int cmd_goto(int argc, char **argv)
{
	double x, y, rpm = motors[0].rpm;
	enum elbow side = ELBOW_ANY;
	struct arm a;
	int i = 3;

	if (argc < 3 || argc > 5) {
		err("usage: goto X Y [+|-] [RPM]");
		return -1;
	}
	if (parse_num(argv[1], &x) || parse_num(argv[2], &y) || get_arm(&a))
		return -1;
	if (i < argc && (!strcmp(argv[i], "+") || !strcmp(argv[i], "-")))
		side = argv[i++][0] == '+' ? ELBOW_POS : ELBOW_NEG;
	if (i < argc && parse_num(argv[i++], &rpm))
		return -1;
	if (i < argc) {
		err("usage: goto X Y [+|-] [RPM]");
		return -1;
	}
	return goto_xy(&a, x, y, side, rpm);
}

/*
 * The tip goes through the points xy[0..n-1] in straight lines at speed
 * mm/s, as one move: each line is split into ~1 mm segments (line.c) and
 * all of them are streamed through one stepping loop, so the tip neither
 * curves nor stops at the joins; it accelerates only at the start and
 * stops only at the end.  Every point is checked before anything moves.
 */
static int path_to(const struct arm *a, int n, const double (*xy)[2],
		   double speed)
{
	static const int both[NJOINTS] = { 0, 1 };
	double x0, y0, now[NJOINTS], upd[NJOINTS], (*th)[NJOINTS] = NULL;
	double asked, bx, by, *dist = NULL;
	enum kin_result r;
	int i, j, k, np, off, npts, interrupted, ret = -1;
	long *pts = NULL;

	if (speed <= 0) {
		err("speed must be > 0 mm/s");
		return -1;
	}
	if (get_angles(now)) {
		err("the arm is not homed; run 'scara home'");
		return -1;
	}
	for (j = 0; j < NJOINTS; j++) {
		upd[j] = usteps_per_deg(j);
		if (upd[j] <= 0)
			return -1;
	}
	kin_forward(a, now, &x0, &y0);
	npts = 1;
	for (k = 0, bx = x0, by = y0; k < n; k++) {
		npts += line_npts(hypot(xy[k][0] - bx, xy[k][1] - by),
				  LINE_SEG_MM) - 1;
		bx = xy[k][0];
		by = xy[k][1];
	}
	th = malloc(npts * sizeof(*th));
	pts = malloc(npts * NJOINTS * sizeof(*pts));
	dist = malloc(npts * sizeof(*dist));
	if (!th || !pts || !dist) {
		err("out of memory");
		goto out;
	}
	memcpy(th[0], now, sizeof(th[0]));
	dist[0] = 0;
	for (k = 0, off = 0; k < n; k++) {
		double len = hypot(xy[k][0] - x0, xy[k][1] - y0);

		np = line_npts(len, LINE_SEG_MM);
		r = line_plan(a, th[off], xy[k][0], xy[k][1], np, th + off,
			      &bx, &by);
		if (r != KIN_OK) {
			err("line to %.2f, %.2f: at %.2f, %.2f: %s", xy[k][0],
			    xy[k][1], bx, by, kin_strerror(r));
			goto out;
		}
		for (i = 1; i < np; i++)
			dist[off + i] = dist[off] + len * i / (np - 1);
		off += np - 1;
		x0 = xy[k][0];
		y0 = xy[k][1];
	}
	/* Point 0 is exactly where the motors are */
	for (j = 0; j < NJOINTS; j++)
		pts[j] = pos[j];
	for (i = 1; i < npts; i++)
		for (j = 0; j < NJOINTS; j++)
			pts[i * NJOINTS + j] = lround(th[i][j] * upd[j]);

	asked = speed;
	if (move_path(NJOINTS, both, npts, pts, dist, &speed,
		      motors[0].line_accel, &interrupted))
		goto out;
	if (speed < asked && asked < G0_LINE_SPEED)
		printf("speed lowered to %.1f mm/s: a motor would go over its "
		       "rpm\n", speed);
	if (interrupted) {
		printf("interrupted\n");
		stopped = 1;
	}
	ret = quiet ? 0 : cmd_where();
out:
	free(th);
	free(pts);
	free(dist);
	return ret;
}

/* line X Y [F]: the tip goes to X/Y in a straight line at F mm/s */
static int line_to(const struct arm *a, double x, double y, double speed)
{
	const double xy[1][2] = { { x, y } };

	return path_to(a, 1, xy, speed);
}

/* line X Y [F] */
static int cmd_line(int argc, char **argv)
{
	double x, y, speed = motors[0].line_speed;
	struct arm a;

	if (argc < 3 || argc > 4) {
		err("usage: line X Y [F]");
		return -1;
	}
	if (parse_num(argv[1], &x) || parse_num(argv[2], &y) ||
	    (argc > 3 && parse_num(argv[3], &speed)) || get_arm(&a))
		return -1;
	return line_to(&a, x, y, speed);
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

/*
 * The tip RGB LED is three kernel LEDs from the device tree (gpio-leds red,
 * pwm-leds green and blue), not GPIO lines requested here: a requested
 * line goes back to an input when the program exits, which would switch
 * the LED off.  The kernel holds them instead.  Colors are 0-255 per
 * channel; a channel with max_brightness 1 (red, plain GPIO) is on for
 * anything above 0.  With led_common = anode the pins are inverted.
 */
static const char *const led_color_names[3] = { "red", "green", "blue" };

static const char *led_name(int i)
{
	return i == 0 ? motors[0].led_red : i == 1 ? motors[0].led_green :
	       motors[0].led_blue;
}

static const char *led_path(int i, const char *file)
{
	static char path[128];

	snprintf(path, sizeof(path), "/sys/class/leds/%s/%s", led_name(i),
		 file);
	return path;
}

static void led_error(int i)
{
	err("%s: %s (is led_%s = %s right in %s?)", led_path(i, "brightness"),
	    strerror(errno), led_color_names[i], led_name(i), CONF_PATH);
}

/* A number from a sysfs file, or -1 */
static int led_read(int i, const char *file)
{
	char buf[16];
	FILE *f = fopen(led_path(i, file), "r");
	int ok = f && fgets(buf, sizeof(buf), f);

	if (f)
		fclose(f);
	return ok ? atoi(buf) : -1;
}

/* max_brightness of channel i (1 for GPIO, 255 for PWM), or -1 */
static int led_max(int i)
{
	static int max[3];

	if (max[i] <= 0)
		max[i] = led_read(i, "max_brightness");
	return max[i];
}

static const int led_black[3];
static const int led_white[3] = { 255, 255, 255 };

/* Last color written: -1 not known yet; led_lit: it is not black */
static int led_shown[3] = { -1, -1, -1 };
static int led_lit;

/* Show a color; a failure is reported once and doesn't stop a drawing */
static int led_show(const int rgb[3])
{
	static int said;
	int i, max, raw, ret = 0;
	FILE *f;

	for (i = 0; i < 3; i++) {
		if (rgb[i] == led_shown[i])
			continue;
		max = led_max(i);
		/* Rounded up, so a dim value still lights a GPIO channel */
		raw = max > 0 ? (rgb[i] * max + 254) / 255 : 0;
		if (motors[0].led_anode)
			raw = max - raw;
		f = max > 0 ? fopen(led_path(i, "brightness"), "w") : NULL;
		if (!f || fprintf(f, "%d", raw) < 0 || fclose(f)) {
			if (!said++)
				led_error(i);
			led_shown[i] = -1;
			ret = -1;
			continue;
		}
		led_shown[i] = rgb[i];
	}
	led_lit = rgb[0] || rgb[1] || rgb[2];
	return ret;
}

static int led_off(void)
{
	return led_show(led_black);
}

/* Colors for "scara led NAME" */
static const struct {
	const char *name;
	int rgb[3];
} led_colors[] = {
	{ "white", { 255, 255, 255 } }, { "red", { 255, 0, 0 } },
	{ "green", { 0, 255, 0 } }, { "blue", { 0, 0, 255 } },
	{ "yellow", { 255, 255, 0 } }, { "cyan", { 0, 255, 255 } },
	{ "magenta", { 255, 0, 255 } }, { "orange", { 255, 64, 0 } },
	{ "purple", { 255, 0, 160 } }, { "on", { 255, 255, 255 } },
	{ "off", { 0, 0, 0 } },
};

/* "0".."255" into *v */
static int led_byte(const char *s, int *v)
{
	char *end;
	long n = strtol(s, &end, 10);

	if (!*s || *end || n < 0 || n > 255)
		return -1;
	*v = n;
	return 0;
}

/* A color: a name, #rrggbb, or R G B (argc 1 or 3) */
static int led_parse(int argc, char **argv, int rgb[3])
{
	unsigned int hex;
	char *end;
	size_t i;

	if (argc == 3)
		return led_byte(argv[0], &rgb[0]) ||
		       led_byte(argv[1], &rgb[1]) ||
		       led_byte(argv[2], &rgb[2]) ? -1 : 0;
	if (argc != 1)
		return -1;
	if (argv[0][0] == '#' && strlen(argv[0]) == 7) {
		hex = strtoul(argv[0] + 1, &end, 16);
		if (*end)
			return -1;
		rgb[0] = hex >> 16;
		rgb[1] = hex >> 8 & 255;
		rgb[2] = hex & 255;
		return 0;
	}
	for (i = 0; i < sizeof(led_colors) / sizeof(led_colors[0]); i++)
		if (!strcmp(argv[0], led_colors[i].name)) {
			memcpy(rgb, led_colors[i].rgb, sizeof(led_colors[i].rgb));
			return 0;
		}
	return -1;
}

static int cmd_led(int argc, char **argv)
{
	int rgb[3], raw, max, i;

	if (argc > 1) {
		if (led_parse(argc - 1, argv + 1, rgb)) {
			err("usage: led [off|on|white|red|green|blue|yellow|"
			    "cyan|magenta|orange|purple|#rrggbb|R G B]");
			return -1;
		}
		return led_show(rgb);
	}
	for (i = 0; i < 3; i++) {
		max = led_max(i);
		raw = max > 0 ? led_read(i, "brightness") : -1;
		if (raw < 0) {
			led_error(i);
			return -1;
		}
		if (motors[0].led_anode)
			raw = max - raw;
		rgb[i] = (raw * 255 + max / 2) / max;
	}
	if (rgb[0] || rgb[1] || rgb[2])
		printf("led: %d %d %d\n", rgb[0], rgb[1], rgb[2]);
	else
		printf("led: off\n");
	return 0;
}

/* A G-code program: the text of every line and what it parsed to */
struct prog {
	const char *name;
	char **text;
	struct gc_line *l;
	int n;
};

#define MIN_MOVE_MM	0.01	/* shorter G0/G1 moves are skipped */

static void prog_free(struct prog *p)
{
	for (int i = 0; i < p->n; i++)
		free(p->text[i]);
	free(p->text);
	free(p->l);
}

/* Read and parse the whole file; -1 at the first bad line */
static int prog_load(const char *name, struct prog *p)
{
	FILE *f = fopen(name, "r");
	char *buf = NULL, msg[128];
	struct gc_state st;
	size_t cap = 0;
	ssize_t len;
	int max = 0, ret = -1;

	memset(p, 0, sizeof(*p));
	p->name = name;
	if (!f) {
		err("%s: %s", name, strerror(errno));
		return -1;
	}
	gc_init(&st);
	while ((len = getline(&buf, &cap, f)) >= 0) {
		while (len > 0 && (buf[len - 1] == '\n' || buf[len - 1] == '\r'))
			buf[--len] = '\0';
		if (p->n == max) {
			char **t;
			struct gc_line *l;

			max = max ? 2 * max : 256;
			t = realloc(p->text, max * sizeof(*t));
			if (t)
				p->text = t;
			l = realloc(p->l, max * sizeof(*l));
			if (l)
				p->l = l;
			if (!t || !l) {
				err("out of memory");
				goto out;
			}
		}
		if (gc_parse(&st, buf, &p->l[p->n], msg, sizeof(msg))) {
			err("%s:%d: %s", p->name, p->n + 1, msg);
			goto out;
		}
		p->text[p->n] = strdup(buf);
		if (!p->text[p->n]) {
			err("out of memory");
			goto out;
		}
		p->n++;
	}
	ret = 0;
out:
	free(buf);
	fclose(f);
	return ret;
}

/*
 * Where a G0 (IK) or G1 (every point of the line) from joint angles th[]
 * to x, y would end, into th[], without moving; on error *bx, *by is the
 * point that fails.
 */
static enum kin_result plan_move(const struct arm *a, int motion, double x,
				 double y, double th[NJOINTS], double *bx,
				 double *by)
{
	struct g0_plan p;
	enum kin_result r;

	if (motion)
		return plan_line(a, x, y, th, bx, by);
	r = plan_g0(a, x, y, ELBOW_ANY, th, &p, bx, by);
	if (r == KIN_OK)
		memcpy(th, p.to, sizeof(p.to));
	return r;
}

/* Where a run of the program is, carried from one line to the next */
struct run {
	const struct arm *a;
	const char *name;
	int dry;
	double th[NJOINTS], x, y, speed;
	int rel, is_homed, said_z;
	int tool;		/* M3: G1 moves light the LED, until M5 */
	int rgb[3];		/* M150: the color they light it with */
};

static void run_init(struct run *r, const struct arm *a, const char *name,
		     int dry)
{
	memset(r, 0, sizeof(*r));
	r->a = a;
	r->name = name;
	r->dry = dry;
	r->speed = motors[0].line_speed;
	memcpy(r->rgb, led_white, sizeof(r->rgb));
	r->is_homed = get_angles(r->th) == 0;
	if (r->is_homed)
		kin_forward(a, r->th, &r->x, &r->y);
}

/*
 * Run line n, or with r->dry only follow it on paper: the move is planned
 * from where the previous one left the joints, so an unreachable point is
 * found before anything moves.  G0 is 'goto', G1 is 'line', G28 is 'home'.
 * The LED is on only during a G1 move after M3: off for G0, G28 and M5
 * (M3 itself waits for the next G1), in the color of the last M150 (white
 * before any); an M150 while it is lit recolors it at once.  Z is reported once, on the dry run.
 * Returns 1 after M2/M30.
 */
static int run_line(struct run *r, const struct gc_line *l, const char *text,
		    int n)
{
	static const int fold_first[NJOINTS] = { 1, 0 };
	double tx, ty, bx, by;
	enum kin_result k;
	int j;

	if (stop_req || stopped) {
		printf("stopped before line %d\n", n);
		return -1;
	}
	if (!r->dry && !stream && (l->dist || l->home || l->move ||
				   l->has_f || l->color || l->end))
		printf("%d: %s\n", n, text);
	if (r->dry && l->has_z && !r->said_z++)
		printf("%s:%d: Z ignored, the arm has no Z axis\n", r->name, n);
	if (l->dist)
		r->rel = l->dist == GC_REL;
	if (l->home) {
		if (!r->dry)
			led_off();
		if (r->dry) {
			for (j = 0; j < NJOINTS; j++)
				r->th[j] = motors[j].home_angle;
		} else if (home_motors(NJOINTS, fold_first) ||
			   get_angles(r->th)) {
			err("%s:%d: homing failed", r->name, n);
			return -1;
		}
		r->is_homed = 1;
		kin_forward(r->a, r->th, &r->x, &r->y);
	}
	if (l->has_f)
		r->speed = l->f / 60;	/* mm/min to mm/s */
	if (l->color) {
		memcpy(r->rgb, l->rgb, sizeof(r->rgb));
		if (!r->dry && led_lit)
			led_show(r->rgb);
	}
	if (l->move) {
		tx = l->has_x ? l->x + (r->rel ? r->x : 0) : r->x;
		ty = l->has_y ? l->y + (r->rel ? r->y : 0) : r->y;
		if (!r->is_homed) {
			err("%s:%d: the arm is not homed; run 'scara home' or "
			    "put G28 before the first move", r->name, n);
			return -1;
		}
		if (hypot(tx - r->x, ty - r->y) < MIN_MOVE_MM) {
			;
		} else if (r->dry) {
			k = plan_move(r->a, l->motion, tx, ty, r->th, &bx, &by);
			if (k != KIN_OK) {
				err("%s:%d: %s: at %.2f, %.2f: %s", r->name, n,
				    text, bx, by, kin_strerror(k));
				return -1;
			}
		} else {
			led_show(l->motion && r->tool ? r->rgb : led_black);
			if (l->motion ? line_to(r->a, tx, ty, r->speed) :
			    goto_xy(r->a, tx, ty, ELBOW_ANY, motors[0].rpm)) {
				err("%s:%d: %s", r->name, n, text);
				return -1;
			}
			if (get_angles(r->th))
				return -1;
		}
		r->x = tx;
		r->y = ty;
	}
	if (l->tool)
		r->tool = l->tool == GC_TOOL_ON;
	if (!r->dry && (!r->tool || l->end))
		led_off();
	return l->end ? 1 : 0;
}

/*
 * Look-ahead: from line i, a G1 move, the lines that run with it as one
 * path: G1 moves at the same feed, turning by at most corner_deg at each
 * join, with only lines that don't move (comments, G90/G91) in between.
 * Fills xy[*n] with the points and *r with the state after the last line
 * taken; returns the index of that line.
 */
static int chain(const struct prog *p, int i, struct run *r,
		 double (*xy)[2], int *n)
{
	double cosmax = cos(motors[0].corner_deg * M_PI / 180);
	double dx0 = 0, dy0 = 0;
	struct run c = *r;
	int k, last = i;

	*n = 0;
	for (k = i; k < p->n; k++) {
		const struct gc_line *l = &p->l[k];
		double sp = l->has_f ? l->f / 60 : c.speed;
		int rel = l->dist ? l->dist == GC_REL : c.rel;
		double tx, ty, dx, dy, len;

		if (k > i && (l->home || l->tool || l->color || l->end ||
			      sp != c.speed || (l->move && !l->motion)))
			break;
		if (l->color)	/* only on line i: M150 on the G1 line */
			memcpy(c.rgb, l->rgb, sizeof(c.rgb));
		if (l->move) {
			tx = l->has_x ? l->x + (rel ? c.x : 0) : c.x;
			ty = l->has_y ? l->y + (rel ? c.y : 0) : c.y;
			dx = tx - c.x;
			dy = ty - c.y;
			len = hypot(dx, dy);
			if (len >= MIN_MOVE_MM) {
				if (*n && (dx * dx0 + dy * dy0) /
				    (len * hypot(dx0, dy0)) < cosmax)
					break;
				xy[*n][0] = tx;
				xy[*n][1] = ty;
				(*n)++;
				dx0 = dx;
				dy0 = dy;
			}
			c.x = tx;
			c.y = ty;
		}
		c.rel = rel;
		c.speed = sp;
		last = k;
	}
	*r = c;
	return last;
}

/*
 * Run (or with dry only check) every line of the program.  When running,
 * G1 lines that chain() joins go as one path (path_to), without stopping
 * at the joins; the dry run checked the same points line by line.
 */
static int prog_run(const struct prog *p, const struct arm *a, int dry)
{
	double (*xy)[2] = NULL;
	struct run r, c;
	int i, k, n, last, ret = -1;

	if (!dry && !(xy = malloc(p->n * sizeof(*xy)))) {
		err("out of memory");
		return -1;
	}
	run_init(&r, a, p->name, dry);
	for (i = 0; i < p->n; i++) {
		const struct gc_line *l = &p->l[i];

		if (!dry && r.is_homed && l->move && l->motion && !l->home &&
		    !l->end && !stop_req && !stopped) {
			c = r;
			last = chain(p, i, &c, xy, &n);
			if (n > 1) {
				for (k = i; k <= last; k++)
					printf("%d: %s\n", k + 1, p->text[k]);
				led_show(c.tool ? c.rgb : led_black);
				if (path_to(a, n, (const double (*)[2])xy,
					    c.speed) || get_angles(c.th)) {
					err("%s:%d-%d: G1 path failed", p->name,
					    i + 1, last + 1);
					goto out;
				}
				r = c;
				i = last;
				continue;
			}
		}
		k = run_line(&r, l, p->text[i], i + 1);
		if (k < 0)
			goto out;
		if (k)
			break;
	}
	ret = 0;
out:
	free(xy);
	return ret;
}

/*
 * gcode [-n] -: run stdin one line at a time, for a sender that waits for
 * the answer to each line before sending the next.  Each line is checked
 * on paper from where the arm is, then run, then answered "ok" (or
 * "error", and nothing more is read).
 */
static int stream_run(const struct arm *a, int dry)
{
	struct run r, check;
	struct gc_state st;
	struct gc_line l;
	char *buf = NULL, msg[128];
	size_t cap = 0;
	ssize_t len;
	int n = 0, ret = 0, end = 0;

	gc_init(&st);
	run_init(&r, a, "stdin", 0);
	while (!end && (len = getline(&buf, &cap, stdin)) >= 0) {
		n++;
		while (len > 0 && (buf[len - 1] == '\n' || buf[len - 1] == '\r'))
			buf[--len] = '\0';
		if (gc_parse(&st, buf, &l, msg, sizeof(msg))) {
			err("stdin:%d: %s", n, msg);
			ret = -1;
			break;
		}
		check = r;
		check.dry = 1;
		ret = run_line(&check, &l, buf, n);
		r.said_z = check.said_z;
		if (ret >= 0 && !dry)
			ret = run_line(&r, &l, buf, n);
		if (dry)
			r = check;
		if (ret < 0)
			break;
		if (stop_req || stopped) {
			printf("stopped at line %d\n", n);
			ret = -1;
			break;
		}
		end = ret;
		ret = 0;
		printf("ok\n");
		fflush(stdout);
	}
	if (!end && !ret && (stop_req || stopped)) {
		/* Ctrl-C while waiting for a line: getline gave up (EINTR) */
		printf("stopped after line %d\n", n);
		ret = -1;
	}
	if (ret) {
		printf("error\n");
		fflush(stdout);
	}
	free(buf);
	return ret;
}

/* gcode [-n] FILE: check the whole file, then run it; "-": stream stdin */
static int cmd_gcode(int argc, char **argv)
{
	int dry = argc > 1 && !strcmp(argv[1], "-n"), ret = -1;
	struct prog p = { 0 };
	struct arm a;

	if (argc != 2 + dry) {
		err("usage: gcode [-n] FILE");
		return -1;
	}
	if (get_arm(&a))
		return -1;
	if (!strcmp(argv[1 + dry], "-")) {
		stream = quiet = 1;
		ret = stream_run(&a, dry);
		stream = quiet = 0;
		if (!dry)
			led_off();
		return ret ? ret : dry ? 0 : cmd_where();
	}
	if (prog_load(argv[1 + dry], &p))
		goto out;
	if (prog_run(&p, &a, 1))
		goto out;
	if (dry) {
		printf("%s: %d lines, every move reachable\n", p.name, p.n);
		ret = 0;
		goto out;
	}
	quiet = 1;
	ret = prog_run(&p, &a, 0);
	led_off();	/* off however the file ended */
	quiet = 0;
	if (!ret)
		ret = cmd_where();
out:
	prog_free(&p);
	return ret;
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
	/* kill or a closed console: stop the same way, so the LED goes off */
	sigaction(SIGTERM, &sa, NULL);
	sigaction(SIGHUP, &sa, NULL);

	c = argv[i];
	argc -= i;
	argv += i;
	if (!strcmp(c, "help") || !strcmp(c, "-h") || !strcmp(c, "--help")) {
		usage();
		return 0;
	}
	if (!strcmp(c, "led"))
		return cmd_led(argc, argv) ? 1 : 0;
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
	if (!strcmp(c, "fk"))
		return cmd_fk(argc, argv) ? 1 : 0;
	if (!strcmp(c, "ik"))
		return cmd_ik(argc, argv) ? 1 : 0;
	if (!strcmp(c, "goto"))
		return cmd_goto(argc, argv) ? 1 : 0;
	if (!strcmp(c, "line"))
		return cmd_line(argc, argv) ? 1 : 0;
	if (!strcmp(c, "gcode"))
		return cmd_gcode(argc, argv) ? 1 : 0;
	err("unknown command '%s' (try 'scara help')", c);
	return 1;
}
