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
"  gcode [-n] FILE       run a G-code file (- for stdin): G0 X Y (goto),\n"
"                        G1 X Y F (line, F in mm/min), G28 (home), G90 /\n"
"                        G91, G21; M3/M5 and Z are ignored for now.\n"
"                        Every line is checked before anything moves;\n"
"                        -n only checks\n"
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

/* The tip to x, y: IK, then the same joint move as 'joint' */
static int goto_xy(const struct arm *a, double x, double y, enum elbow side,
		   double rpm)
{
	double th[NJOINTS], now[NJOINTS];
	enum kin_result r;

	if (get_angles(now)) {
		err("the arm is not homed; run 'scara home'");
		return -1;
	}
	r = kin_inverse(a, x, y, side, now, th);
	if (r != KIN_OK) {
		err("%.2f, %.2f: %s", x, y, kin_strerror(r));
		return -1;
	}
	return move_to_angles(th, rpm);
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
 * line X Y [F]: the tip goes to X/Y in a straight line at F mm/s.  The
 * line is split into ~1 mm segments (line.c) streamed through one
 * stepping loop, so the tip neither curves nor stops at segment joins.
 * Every point is checked before anything moves.
 */
static int line_to(const struct arm *a, double x, double y, double speed)
{
	static const int both[NJOINTS] = { 0, 1 };
	double x0, y0, now[NJOINTS], upd[NJOINTS], (*th)[NJOINTS];
	double asked, bx, by;
	enum kin_result r;
	int i, j, npts, interrupted, ret = -1;
	long *pts;

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
	npts = line_npts(hypot(x - x0, y - y0), LINE_SEG_MM);
	th = malloc(npts * sizeof(*th));
	pts = malloc(npts * NJOINTS * sizeof(*pts));
	if (!th || !pts) {
		err("out of memory");
		goto out;
	}
	r = line_plan(a, now, x, y, npts, th, &bx, &by);
	if (r != KIN_OK) {
		err("line to %.2f, %.2f: at %.2f, %.2f: %s", x, y, bx, by,
		    kin_strerror(r));
		goto out;
	}
	/* Point 0 is exactly where the motors are */
	for (j = 0; j < NJOINTS; j++)
		pts[j] = pos[j];
	for (i = 1; i < npts; i++)
		for (j = 0; j < NJOINTS; j++)
			pts[i * NJOINTS + j] = lround(th[i][j] * upd[j]);

	asked = speed;
	if (move_path(NJOINTS, both, npts, pts,
		      hypot(x - x0, y - y0) / (npts - 1), &speed,
		      motors[0].line_accel, &interrupted))
		goto out;
	if (speed < asked)
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
	return ret;
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

/* Read and parse the whole file ("-": stdin); -1 at the first bad line */
static int prog_load(const char *name, struct prog *p)
{
	FILE *f = strcmp(name, "-") ? fopen(name, "r") : stdin;
	char *buf = NULL, msg[128];
	struct gc_state st;
	size_t cap = 0;
	ssize_t len;
	int max = 0, ret = -1;

	memset(p, 0, sizeof(*p));
	p->name = f == stdin ? "stdin" : name;
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
	if (f != stdin)
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
	double x0, y0, (*pts)[NJOINTS];
	enum kin_result r;
	int npts;

	*bx = x;
	*by = y;
	if (!motion)
		return kin_inverse(a, x, y, ELBOW_ANY, th, th);
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

/*
 * Run the program, or with dry only follow it on paper: every move is
 * planned from where the previous one left the joints, so the first
 * unreachable point is found before anything moves.  G0 is 'goto', G1 is
 * 'line', G28 is 'home'.  M3/M5 and Z are reported once, on the dry run.
 */
static int prog_run(const struct prog *p, const struct arm *a, int dry)
{
	static const int fold_first[NJOINTS] = { 1, 0 };
	double th[NJOINTS], x = 0, y = 0, tx, ty, bx, by;
	double speed = motors[0].line_speed;
	int i, j, rel = 0, is_homed, said_z = 0, said_tool = 0;
	enum kin_result r;

	is_homed = get_angles(th) == 0;
	if (is_homed)
		kin_forward(a, th, &x, &y);
	for (i = 0; i < p->n; i++) {
		const struct gc_line *l = &p->l[i];
		int n = i + 1;

		if (stop_req || stopped) {
			printf("stopped before line %d\n", n);
			return -1;
		}
		if (!dry && (l->dist || l->home || l->move || l->has_f ||
			     l->end))
			printf("%d: %s\n", n, p->text[i]);
		if (dry && l->has_z && !said_z++)
			printf("%s:%d: Z ignored, the arm has no Z axis\n",
			       p->name, n);
		if (dry && l->tool && !said_tool++)
			printf("%s:%d: M3/M5 ignored, the tip LED is not "
			       "wired yet\n", p->name, n);
		if (l->dist)
			rel = l->dist == GC_REL;
		if (l->home) {
			if (dry) {
				for (j = 0; j < NJOINTS; j++)
					th[j] = motors[j].home_angle;
			} else if (home_motors(NJOINTS, fold_first) ||
				   get_angles(th)) {
				err("%s:%d: homing failed", p->name, n);
				return -1;
			}
			is_homed = 1;
			kin_forward(a, th, &x, &y);
		}
		if (l->has_f)
			speed = l->f / 60;	/* mm/min to mm/s */
		if (l->move) {
			tx = l->has_x ? l->x + (rel ? x : 0) : x;
			ty = l->has_y ? l->y + (rel ? y : 0) : y;
			if (!is_homed) {
				err("%s:%d: the arm is not homed; run 'scara "
				    "home' or put G28 before the first move",
				    p->name, n);
				return -1;
			}
			if (hypot(tx - x, ty - y) < MIN_MOVE_MM) {
				;
			} else if (dry) {
				r = plan_move(a, l->motion, tx, ty, th, &bx,
					      &by);
				if (r != KIN_OK) {
					err("%s:%d: %s: at %.2f, %.2f: %s",
					    p->name, n, p->text[i], bx, by,
					    kin_strerror(r));
					return -1;
				}
			} else if (l->motion ? line_to(a, tx, ty, speed) :
				   goto_xy(a, tx, ty, ELBOW_ANY,
					   motors[0].rpm)) {
				err("%s:%d: %s", p->name, n, p->text[i]);
				return -1;
			}
			x = tx;
			y = ty;
		}
		if (l->end)
			break;
	}
	return 0;
}

/* gcode [-n] FILE: check the whole file, then run it */
static int cmd_gcode(int argc, char **argv)
{
	int dry = argc > 1 && !strcmp(argv[1], "-n"), ret = -1;
	struct prog p = { 0 };
	struct arm a;

	if (argc != 2 + dry) {
		err("usage: gcode [-n] FILE");
		return -1;
	}
	if (get_arm(&a) || prog_load(argv[1 + dry], &p))
		goto out;
	if (prog_run(&p, &a, 1))
		goto out;
	if (dry) {
		printf("%s: %d lines, every move reachable\n", p.name, p.n);
		ret = 0;
		goto out;
	}
	quiet = 1;
	if (prog_run(&p, &a, 0))
		goto out;
	quiet = 0;
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
