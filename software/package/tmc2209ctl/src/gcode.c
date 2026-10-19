// SPDX-License-Identifier: MIT
/* G-code line parser; see gcode.h */
#include <ctype.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "gcode.h"

void gc_init(struct gc_state *st)
{
	st->motion = -1;
}

static int fail(char *err, size_t errlen, const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	vsnprintf(err, errlen, fmt, ap);
	va_end(ap);
	return -1;
}

/*
 * A G-code number: [+-]digits[.digits].  Not strtod(), which would also
 * take "1E5" (G1E5), hex and "inf".  Returns the end, or NULL.
 */
static const char *number(const char *p, double *v)
{
	double sign = 1, scale = 1;
	int digits = 0;

	*v = 0;
	if (*p == '+' || *p == '-')
		sign = *p++ == '-' ? -1 : 1;
	for (; isdigit((unsigned char)*p); p++, digits++)
		*v = *v * 10 + (*p - '0');
	if (*p == '.')
		for (p++; isdigit((unsigned char)*p); p++, digits++)
			*v += (*p - '0') * (scale /= 10);
	if (!digits)
		return NULL;
	*v *= sign;
	return p;
}

int gc_parse(struct gc_state *st, const char *line, struct gc_line *out,
	     char *err, size_t errlen)
{
	struct gc_line l;
	const char *p = line, *end;
	int motion = -1, seen_s = 0, has_rgbp[4] = { 0 }, k;
	double v, rgbp[4] = { 0, 0, 0, 255 };

	memset(&l, 0, sizeof(l));
	while (*p) {
		int c = toupper((unsigned char)*p);

		if (isspace((unsigned char)c) || c == '%') {
			p++;
			continue;
		}
		if (c == ';' || c == '*')	/* comment, checksum */
			break;
		if (c == '(') {
			end = strchr(p, ')');
			if (!end)
				return fail(err, errlen, "'(' without ')'");
			p = end + 1;
			continue;
		}
		if (!isalpha(c))
			return fail(err, errlen, "unexpected '%c'", *p);
		for (p++; *p == ' ' || *p == '\t'; p++)
			;
		end = number(p, &v);
		if (!end)
			return fail(err, errlen, "%c without a number", c);
		p = end;

		switch (c) {
		case 'N':		/* line number */
			break;
		case 'G':
			if (v == 0 || v == 1) {
				if (motion >= 0 && motion != (int)v)
					return fail(err, errlen,
						    "G0 and G1 on one line");
				motion = (int)v;
			} else if (v == 28) {
				l.home = 1;
			} else if (v == 90 || v == 91) {
				enum gc_dist d = v == 90 ? GC_ABS : GC_REL;

				if (l.dist && l.dist != d)
					return fail(err, errlen,
						    "G90 and G91 on one line");
				l.dist = d;
			} else if (v == 21) {
				;	/* mm: the only unit */
			} else if (v == 20) {
				return fail(err, errlen, "G20 (inches) is not "
					    "supported, only mm (G21)");
			} else {
				return fail(err, errlen, "unknown code G%g", v);
			}
			break;
		case 'M':
			if (v == 3 || v == 106 || v == 5 || v == 107) {
				enum gc_tool t = v == 3 || v == 106 ?
					GC_TOOL_ON : GC_TOOL_OFF;

				if (l.tool && l.tool != t)
					return fail(err, errlen, "tool on and "
						    "off on one line");
				l.tool = t;
			} else if (v == 150) {
				l.color = 1;
			} else if (v == 2 || v == 30) {
				l.end = 1;
			} else {
				return fail(err, errlen, "unknown code M%g", v);
			}
			break;
		case 'X':
		case 'Y':
		case 'Z':
		case 'F': {
			int *has = c == 'X' ? &l.has_x : c == 'Y' ? &l.has_y :
				   c == 'Z' ? &l.has_z : &l.has_f;
			double *val = c == 'X' ? &l.x : c == 'Y' ? &l.y :
				      c == 'Z' ? &l.z : &l.f;

			if (*has)
				return fail(err, errlen, "%c twice", c);
			*has = 1;
			*val = v;
			break;
		}
		case 'R':		/* M150 red, green (U), blue, brightness */
		case 'U':
		case 'B':
		case 'P': {
			k = c == 'R' ? 0 : c == 'U' ? 1 : c == 'B' ? 2 : 3;
			if (has_rgbp[k]++)
				return fail(err, errlen, "%c twice", c);
			if (v < 0 || v > 255)
				return fail(err, errlen, "%c must be 0-255", c);
			rgbp[k] = v;
			break;
		}
		case 'S':		/* tool power: not used yet */
			if (seen_s++)
				return fail(err, errlen, "S twice");
			break;
		default:
			return fail(err, errlen, "unknown word %c", c);
		}
	}

	if (!l.color && (has_rgbp[0] || has_rgbp[1] || has_rgbp[2] ||
			 has_rgbp[3]))
		return fail(err, errlen, "R, U, B and P only go with M150");
	for (k = 0; l.color && k < 3; k++)
		l.rgb[k] = (int)(rgbp[k] * rgbp[3] / 255 + 0.5);
	if (l.has_f && l.f <= 0)
		return fail(err, errlen, "F must be > 0");
	/* G28 X0 Y0 means "home X and Y": both are always homed */
	if ((l.has_x || l.has_y) && !l.home) {
		l.motion = motion >= 0 ? motion : st->motion;
		if (l.motion < 0)
			return fail(err, errlen, "X/Y without G0 or G1");
		l.move = 1;
	}
	if (motion >= 0)
		st->motion = motion;
	*out = l;
	return 0;
}
