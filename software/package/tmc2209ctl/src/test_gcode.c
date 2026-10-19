// SPDX-License-Identifier: MIT
/* Host test for gcode.c: make test */
#include <stdio.h>
#include <string.h>

#include "gcode.h"

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

static struct gc_state st;
static struct gc_line l;
static char msg[128];

/* Parse s with the current modal state; 0 or -1 */
static int parse(const char *s)
{
	msg[0] = 0;
	return gc_parse(&st, s, &l, msg, sizeof(msg));
}

/* s parses and moves to x, y with motion G0/G1 */
static void check_move(const char *s, int motion, double x, double y)
{
	CHECK(parse(s) == 0, "'%s': %s", s, msg);
	CHECK(l.move && l.motion == motion && l.x == x && l.y == y,
	      "'%s': move %d motion %d x %g y %g", s, l.move, l.motion, l.x,
	      l.y);
}

/* s is refused with a message containing want */
static void check_error(const char *s, const char *want)
{
	struct gc_state before = st;

	CHECK(parse(s) == -1, "'%s' should fail", s);
	CHECK(strstr(msg, want), "'%s': message '%s', want '%s'", s, msg,
	      want);
	CHECK(st.motion == before.motion, "'%s' changed the state", s);
}

static void lines(void)
{
	gc_init(&st);
	CHECK(parse("") == 0 && !l.move && !l.home && !l.tool, "empty line");
	CHECK(parse("   ; just a comment") == 0 && !l.move, "comment");
	CHECK(parse("(only) (comments)") == 0 && !l.move, "( ) comments");
	CHECK(parse("%") == 0 && !l.move, "%%");
	check_error("X10 Y10", "without G0 or G1");

	check_move("G0 X10 Y-20.5", 0, 10, -20.5);
	check_move("X30 Y40", 0, 30, 40);	/* modal G0 */
	check_move("G01 X1 Y2 F600", 1, 1, 2);
	CHECK(l.has_f && l.f == 600, "F600: %g", l.f);
	check_move("X3", 1, 3, 0);		/* modal G1, Y kept */
	CHECK(l.has_x && !l.has_y && !l.has_f, "X only");
	check_move("g00x5y6", 0, 5, 6);		/* lowercase, packed */
	check_move("N20 G1 X.5 Y-.25 ; comment X99", 1, 0.5, -0.25);
	check_move("G1 (comment X99) X7 Y8*57", 1, 7, 8);
	check_move("G1 X+1. Y-0", 1, 1, 0);
	check_move("G1 X1 Y2 Z3", 1, 1, 2);
	CHECK(l.has_z && l.z == 3, "Z parsed");

	CHECK(parse("G1 F300") == 0 && !l.move && l.has_f && l.f == 300,
	      "feed only");
	CHECK(parse("G0 Z5") == 0 && !l.move && l.has_z, "Z only: no move");
	CHECK(st.motion == 0, "G0 Z5 sets G0");

	CHECK(parse("G28") == 0 && l.home && !l.move, "G28");
	CHECK(parse("G28 X0 Y0") == 0 && l.home && !l.move, "G28 X0 Y0");
	CHECK(parse("G90") == 0 && l.dist == GC_ABS, "G90");
	CHECK(parse("G91") == 0 && l.dist == GC_REL, "G91");
	CHECK(parse("G21 G90") == 0 && l.dist == GC_ABS, "G21 G90");
	CHECK(parse("G1 X1") == 0 && l.dist == GC_DIST_KEEP, "no G90/G91");
	CHECK(parse("M3") == 0 && l.tool == GC_TOOL_ON, "M3");
	CHECK(parse("M3 S1000") == 0 && l.tool == GC_TOOL_ON, "M3 S");
	CHECK(parse("M106") == 0 && l.tool == GC_TOOL_ON, "M106");
	CHECK(parse("M5") == 0 && l.tool == GC_TOOL_OFF, "M5");
	CHECK(parse("M107") == 0 && l.tool == GC_TOOL_OFF, "M107");
	CHECK(parse("M150 R255 U128 B0") == 0 && l.color && l.rgb[0] == 255 &&
	      l.rgb[1] == 128 && l.rgb[2] == 0 && !l.tool, "M150 R U B");
	CHECK(parse("M150 B200") == 0 && l.color && l.rgb[0] == 0 &&
	      l.rgb[1] == 0 && l.rgb[2] == 200, "M150: missing ones are 0");
	CHECK(parse("M150") == 0 && l.color && !l.rgb[0] && !l.rgb[1] &&
	      !l.rgb[2], "M150 alone: black");
	CHECK(parse("M150 R255 U255 B100 P51") == 0 && l.rgb[0] == 51 &&
	      l.rgb[1] == 51 && l.rgb[2] == 20, "M150 P scales: %d %d %d",
	      l.rgb[0], l.rgb[1], l.rgb[2]);
	CHECK(parse("M3") == 0 && !l.color, "M3: no color");
	CHECK(parse("M2") == 0 && l.end, "M2");
	CHECK(parse("M30") == 0 && l.end, "M30");

	check_error("G1 X1 X2", "X twice");
	check_error("G0 G1 X1", "G0 and G1");
	check_error("G90 G91", "G90 and G91");
	check_error("M3 M5", "on and off");
	check_error("G20", "inches");
	check_error("G2 X1 Y1 I5 J0", "unknown code G2");
	check_error("G1.5 X1", "unknown code G1.5");
	check_error("M4", "unknown code M4");
	check_error("M150 R256", "R must be 0-255");
	check_error("M150 U-1", "U must be 0-255");
	check_error("M150 R1 R2", "R twice");
	check_error("G1 X1 R5", "only go with M150");
	check_error("M3 B5", "only go with M150");
	check_error("T1", "unknown word T");
	check_error("G1 X1 E5", "unknown word E");
	check_error("G1 X", "X without a number");
	check_error("G1 Xinf", "X without a number");
	check_error("G1 X1 F0", "F must be > 0");
	check_error("G1 X1 (open", "'(' without ')'");
	check_error("G1 X1 # 2", "unexpected '#'");

	/* A refused line leaves the modal state alone */
	parse("G1 X0");
	check_error("G0 X1 X2", "X twice");
	CHECK(st.motion == 1, "still G1");
}

/* The sample file: every line parses; count what it does */
static void sample(void)
{
	FILE *f = fopen("square.gcode", "r");
	char buf[256];
	int n = 0, moves = 0, g1 = 0, homes = 0, z = 0, on = 0, off = 0;
	int rel = 0, end = 0;

	CHECK(f, "square.gcode missing");
	if (!f)
		return;
	gc_init(&st);
	while (fgets(buf, sizeof(buf), f)) {
		n++;
		buf[strcspn(buf, "\r\n")] = 0;
		CHECK(parse(buf) == 0, "square.gcode:%d: %s", n, msg);
		moves += l.move;
		g1 += l.move && l.motion == 1;
		homes += l.home;
		z += l.has_z;
		on += l.tool == GC_TOOL_ON;
		off += l.tool == GC_TOOL_OFF;
		rel += l.dist == GC_REL;
		end += l.end;
	}
	fclose(f);
	CHECK(moves == 6 && g1 == 4, "moves %d, G1 %d", moves, g1);
	CHECK(homes == 1 && z == 1 && on == 1 && off == 1 && rel == 1 &&
	      end == 1, "home %d z %d on %d off %d rel %d end %d", homes, z,
	      on, off, rel, end);
}

/* The RGB sample: every line parses, four colours */
static void rgb_sample(void)
{
	FILE *f = fopen("rgb.gcode", "r");
	char buf[256];
	int n = 0, colors = 0;

	CHECK(f, "rgb.gcode missing");
	if (!f)
		return;
	gc_init(&st);
	while (fgets(buf, sizeof(buf), f)) {
		n++;
		buf[strcspn(buf, "\r\n")] = 0;
		CHECK(parse(buf) == 0, "rgb.gcode:%d: %s", n, msg);
		colors += l.color;
	}
	fclose(f);
	CHECK(colors == 4, "colors %d", colors);
}

int main(void)
{
	lines();
	sample();
	rgb_sample();
	printf("test_gcode: %d checks, %d failed\n", checks, failures);
	return failures ? 1 : 0;
}
