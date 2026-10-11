// SPDX-License-Identifier: MIT
/*
 * tmc2209ctl - drive stepper motors through TMC2209 drivers from the terminal.
 *
 * Configuration (current, microsteps, chopper mode, enable/disable) and
 * velocity mode (VACTUAL) go over the TMC2209 single-wire UART (PDN_UART).
 * Several drivers can share that UART, told apart by their MS1/MS2
 * address straps.  Exact-distance moves use STEP/DIR pulses on two GPIO
 * lines per motor; moves of several motors run together and finish
 * together.  Each motor's endstop switch is read from one more GPIO line,
 * and "home" drives each joint onto its switch to find a known position.
 *
 * Defaults are read from /etc/tmc2209.conf ([motorN] sections hold the
 * per-motor settings) and can be overridden on the command line.  Run
 * "tmc2209ctl help" for usage.
 */
#define _GNU_SOURCE
#include <ctype.h>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/gpio.h>
#include <math.h>
#include <poll.h>
#include <sched.h>
#include <signal.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include "tmc2209.h"

/* Motors the commands act on (-m, or "motor" in the shell) */
static int sel[MAX_MOTORS] = { 0 };
static int nsel = 1;

static int cmd_run(double rpm)
{
	int32_t targets[MAX_MOTORS];
	int k;

	for (k = 0; k < nsel; k++) {
		int usteps;

		select_motor(sel[k]);
		if (ensure_ready() || get_microsteps(&usteps) || set_enabled(1))
			return -1;
		targets[k] = rpm_to_vactual(cfg.invert_dir ? -rpm : rpm, usteps);
		pos_lost(cur);
	}
	if (ramp_motors(nsel, sel, targets))
		return -1;
	for (k = 0; k < nsel && rpm != 0; k++) {
		select_motor(sel[k]);
		printf("%srunning at %.1f rpm (hardware step generator); "
			       "'tmc2209ctl stop' to stop\n", tag(), rpm);
	}
	return 0;
}

static int cmd_stop(void)
{
	int32_t targets[MAX_MOTORS] = { 0 };

	stop_req = 0;
	if (ramp_motors(nsel, sel, targets))
		return -1;
	printf("stopped (motor still energised; 'disable' to release it)\n");
	return 0;
}

/* ------------------------------------------------------------------ */
/* Endstops                                                            */
/* ------------------------------------------------------------------ */

static void print_endstop(int m, int value)
{
	const struct config *c = &motors[m];

	printf("motor %d: %-8s (%s = %d%s)\n", m + 1,
	       endstop_pressed(c, value) ? "pressed" : "released",
	       c->endstop_line, value, c->endstop_nc ? ", NC switch" : "");
}

/*
 * Print the endstop of every motor that has one; with 'watch', keep
 * printing each change until Ctrl-C.  A change is printed once the line
 * has been quiet for DEBOUNCE_MS, so switch bounce shows up as one event.
 */
#define DEBOUNCE_MS	10

static int cmd_endstop(int argc, char **argv)
{
	struct pollfd p[MAX_MOTORS];
	int mi[MAX_MOTORS], last[MAX_MOTORS];
	int watch = argc > 1 && !strcmp(argv[1], "watch");
	int k, n = 0, ret = -1;

	if (argc > 1 && !watch) {
		err("usage: endstop [watch]");
		return -1;
	}
	for (int m = 0; m < nmotors; m++) {
		if (!motors[m].endstop_line[0]) {
			printf("motor %d: no endstop_line in %s\n", m + 1,
			       CONF_PATH);
			continue;
		}
		p[n].fd = request_input(motors[m].endstop_line);
		p[n].events = POLLIN;
		if (p[n].fd < 0)
			goto out;
		last[n] = get_line(p[n].fd);
		if (last[n] < 0) {
			err("cannot read GPIO '%s': %s", motors[m].endstop_line,
			    strerror(errno));
			n++;
			goto out;
		}
		mi[n] = m;
		print_endstop(m, last[n]);
		n++;
	}
	if (!n || !watch) {
		ret = 0;
		goto out;
	}

	puts("watching the endstops, Ctrl-C to stop");
	fflush(stdout);
	stop_req = 0;
	while (!stop_req) {
		int timeout = -1;

		/* Wait for an edge, then until DEBOUNCE_MS pass without one */
		for (;;) {
			int r = poll(p, n, timeout);

			if (r < 0 && errno == EINTR)
				break;
			if (r < 0) {
				err("poll: %s", strerror(errno));
				goto out;
			}
			if (!r)
				break;
			for (k = 0; k < n; k++) {
				struct gpioevent_data ev;

				if ((p[k].revents & POLLIN) &&
				    read(p[k].fd, &ev, sizeof(ev)) < 0) {
					err("reading GPIO events: %s",
					    strerror(errno));
					goto out;
				}
			}
			timeout = DEBOUNCE_MS;
		}
		for (k = 0; k < n && !stop_req; k++) {
			int v = get_line(p[k].fd);

			if (v >= 0 && v != last[k]) {
				last[k] = v;
				print_endstop(mi[k], v);
				fflush(stdout);
			}
		}
	}
	stop_req = 0;
	ret = 0;
out:
	for (k = 0; k < n; k++)
		if (p[k].fd >= 0)
			close(p[k].fd);
	return ret;
}

/*
 * home [1|2|all]: home the given motors (default all), the highest number
 * first, so the elbow (motor 2) is folded onto its switch before the
 * shoulder swings.  Stops at the first failure.
 */
static int cmd_home(int argc, char **argv)
{
	int mi[MAX_MOTORS], n = 0, k;

	if (argc > 1 && strcmp(argv[1], "all") && strcmp(argv[1], "both")) {
		int m = atoi(argv[1]);

		if (m < 1 || m > nmotors) {
			err("usage: home [1..%d|all]", nmotors);
			return -1;
		}
		mi[n++] = m - 1;
	} else {
		for (k = nmotors - 1; k >= 0; k--)
			mi[n++] = k;
	}

	return home_motors(n, mi);
}

/* ------------------------------------------------------------------ */
/* Status / raw register access                                        */
/* ------------------------------------------------------------------ */

static const struct { const char *name; uint8_t reg; } regs[] = {
	{ "GCONF", REG_GCONF }, { "GSTAT", REG_GSTAT },
	{ "IFCNT", REG_IFCNT }, { "IOIN", REG_IOIN },
	{ "IHOLD_IRUN", REG_IHOLD_IRUN }, { "TPOWERDOWN", REG_TPOWERDOWN },
	{ "TSTEP", REG_TSTEP }, { "TPWMTHRS", REG_TPWMTHRS },
	{ "VACTUAL", REG_VACTUAL }, { "SG_RESULT", REG_SG_RESULT },
	{ "MSCNT", REG_MSCNT }, { "CHOPCONF", REG_CHOPCONF },
	{ "DRV_STATUS", REG_DRV_STATUS }, { "PWMCONF", REG_PWMCONF },
};

static int parse_reg(const char *s)
{
	char *end;
	long v;

	for (size_t i = 0; i < sizeof(regs) / sizeof(regs[0]); i++)
		if (!strcasecmp(s, regs[i].name))
			return regs[i].reg;
	v = strtol(s, &end, 0);
	if (*end || v < 0 || v > 0x7F)
		return -1;
	return (int)v;
}

static int cmd_status(void)
{
	uint32_t gconf, gstat, ifcnt, ioin, chop, drv, tstep, sg, mscnt;
	int vsense;

	if (tmc_read(REG_IOIN, &ioin) || tmc_read(REG_GCONF, &gconf) ||
	    tmc_read(REG_GSTAT, &gstat) || tmc_read(REG_IFCNT, &ifcnt) ||
	    tmc_read(REG_CHOPCONF, &chop) || tmc_read(REG_DRV_STATUS, &drv) ||
	    tmc_read(REG_TSTEP, &tstep) || tmc_read(REG_SG_RESULT, &sg) ||
	    tmc_read(REG_MSCNT, &mscnt))
		return -1;
	vsense = !!(chop & CHOP_VSENSE);

	if (nmotors > 1)
		printf("Motor         : %d  (STEP %.5s, DIR %.5s)\n", cur + 1,
		       cfg.step_line, cfg.dir_line);
	printf("Chip          : TMC22xx version 0x%02X %s, addr %d on %s @ %d\n",
	       ioin >> 24, (ioin >> 24) == 0x21 ? "(TMC2209)" : "(?)",
	       cfg.addr, cfg.dev, cfg.baud);
	printf("Pins          : ENN=%u MS1=%u MS2=%u STEP=%u DIR=%u SPREAD=%u\n",
	       ioin & 1, (ioin >> 2) & 1, (ioin >> 3) & 1, (ioin >> 7) & 1,
	       (ioin >> 9) & 1, (ioin >> 8) & 1);
	printf("Driver        : %s%s\n", (chop & CHOP_TOFF_MASK) ?
	       "ENABLED" : "disabled (motor free)",
	       (ioin & 1) ? "  [EN pin is HIGH - hardware disabled!]" : "");
	printf("Microsteps    : %d%s\n",
	       256 >> ((chop & CHOP_MRES_MASK) >> CHOP_MRES_SHIFT),
	       (gconf & GCONF_MSTEP_REG_SELECT) ? "" :
	       " (from MS1/MS2 pins - not configured yet, run 'init')");
	printf("Chopper       : %s, currently %s\n",
	       (gconf & GCONF_EN_SPREADCYCLE) ? "spreadCycle" : "stealthChop",
	       (drv >> 30) & 1 ? "stealthChop" : "spreadCycle");
	printf("Current       : actual %.0f mA RMS (CS_ACTUAL=%u)\n",
	       current_of((drv >> 16) & 0x1F, vsense), (drv >> 16) & 0x1F);
	printf("Standstill    : %s   TSTEP=%u  SG_RESULT=%u  MSCNT=%u\n",
	       (drv >> 31) ? "yes" : "no", tstep, sg, mscnt);
	printf("Temperature   : %s\n",
	       (drv >> 1) & 1 ? "OVERTEMP SHUTDOWN (>150C)" :
	       (drv & 1) ? "prewarning (>120C)" :
	       (drv >> 8) & 1 ? ">120C" : "ok");
	printf("Faults        :%s%s%s%s%s%s%s\n",
	       (drv >> 2) & 1 ? " short-to-GND-A" : "",
	       (drv >> 3) & 1 ? " short-to-GND-B" : "",
	       (drv >> 4) & 1 ? " short-VS-A" : "",
	       (drv >> 5) & 1 ? " short-VS-B" : "",
	       (drv >> 6) & 1 ? " open-load-A" : "",
	       (drv >> 7) & 1 ? " open-load-B" : "",
	       (drv & 0xFC) ? "" : " none");
	printf("GSTAT         : 0x%X%s%s%s\n", gstat,
	       gstat & 1 ? " reset" : "", gstat & 2 ? " drv_err" : "",
	       gstat & 4 ? " undervoltage(VM)" : "");
	printf("UART writes   : %u (IFCNT)\n", ifcnt & 0xFF);
	printf("VACTUAL       : %d (last set by this tool)\n", load_vactual());
	return 0;
}

/* ------------------------------------------------------------------ */
/* Command dispatch                                                    */
/* ------------------------------------------------------------------ */

static void usage(void)
{
	printf(
"Usage: tmc2209ctl [-m 1|2|all] [-d DEV] [-a ADDR] [-b BAUD] [-c CONF]\n"
"                  COMMAND [ARGS]\n"
"\n"
"  -m selects the motor(s) the command acts on (default 1).  With 'all',\n"
"  every command runs on each motor and moves run simultaneously.\n"
"\n"
"Setup:\n"
"  init                  apply /etc/tmc2209.conf to the driver (motor off)\n"
"  status                show driver state, temperature, faults\n"
"  motors                list the configured motors (address, pins)\n"
"  enable | disable      energise / release the motor coils\n"
"  current MA [HOLD%%]    set RMS run current (and hold %% of it)\n"
"  microsteps N          1,2,4,8,16,32,64,128,256\n"
"  mode stealth|spread   stealthChop (quiet) or spreadCycle (more torque)\n"
"\n"
"Motion:\n"
"  turn REVS [RPM]       exact move of REVS revolutions (negative = reverse)\n"
"  move USTEPS [RPM]     exact move of USTEPS microsteps  (STEP/DIR pins)\n"
"  sync R1 R2 [RPM]      motor 1 turns R1 revs and motor 2 R2 revs at the\n"
"                        same time; the longer move runs at RPM and both\n"
"                        finish together\n"
"  run RPM               spin continuously, hardware-timed (UART VACTUAL)\n"
"  stop                  ramp a 'run' down to zero\n"
"  Ctrl-C decelerates a move/turn/sync to a stop.\n"
"\n"
"Endstops (every motor, -m is ignored):\n"
"  endstop               print each switch: released or pressed\n"
"  endstop watch         print every change until Ctrl-C\n"
"  home [1|2|all]        drive each joint onto its switch (default all,\n"
"                        motor 2 first); needs home_dir in the config\n"
"\n"
"Other:\n"
"  read REG              read a register (name like CHOPCONF or number)\n"
"  write REG VALUE       write a register\n"
"  motor 1|2|all         (shell) choose the motor for the next commands\n"
"  shell                 interactive prompt accepting the commands above\n"
"\n"
"Defaults: %s @ %d baud, %d motor(s), %d mA, %d usteps, %.0f rpm, "
"accel %.1f rev/s^2\n",
	cfg.dev, cfg.baud, nmotors, cfg.current_ma, cfg.microsteps, cfg.rpm,
	cfg.accel);
}

static int need(int argc, int n, const char *what)
{
	if (argc < n) {
		err("usage: %s", what);
		return 0;
	}
	return 1;
}

/* Parse "1", "2", ... or "all"/"both" into sel[] */
static int parse_motor_sel(const char *s)
{
	int m;

	if (!strcmp(s, "all") || !strcmp(s, "both")) {
		for (m = 0; m < nmotors; m++)
			sel[m] = m;
		nsel = nmotors;
		return 0;
	}
	m = atoi(s);
	if (m < 1 || m > nmotors) {
		err("motor must be 1..%d or 'all' (motors come from the "
		    "[motorN] sections of %s)", nmotors, CONF_PATH);
		return -1;
	}
	sel[0] = m - 1;
	nsel = 1;
	return 0;
}

static void cmd_motors(void)
{
	for (int i = 0; i < nmotors; i++) {
		const struct config *c = &motors[i];

		printf("motor %d: addr %d (MS1=%s MS2=%s)  STEP '%s'  DIR '%s'  "
		       "ENDSTOP '%s'%s  %d mA  %d usteps%s\n", i + 1, c->addr,
		       c->addr & 1 ? "3.3V" : "GND", c->addr & 2 ? "3.3V" : "GND",
		       c->step_line, c->dir_line, c->endstop_line,
		       c->endstop_nc ? " (NC)" : "", c->current_ma, c->microsteps,
		       c->invert_dir ? "  inverted" : "");
	}
}

/* Commands that act on one motor at a time (the active cfg) */
static int run_one(int argc, char **argv)
{
	const char *c = argv[0];

	if (!strcmp(c, "init"))
		return apply_config();
	if (!strcmp(c, "status") || !strcmp(c, "info"))
		return cmd_status();
	if (!strcmp(c, "enable")) {
		if (ensure_ready() || set_enabled(1))
			return -1;
		printf("%smotor energised\n", tag());
		return 0;
	}
	if (!strcmp(c, "disable")) {
		if (load_vactual() && stop_one())
			return -1;
		if (set_enabled(0))
			return -1;
		pos_lost(cur);
		printf("%smotor released\n", tag());
		return 0;
	}
	if (!strcmp(c, "current")) {
		if (!need(argc, 2, "current MA [HOLD%]"))
			return -1;
		return set_current(atoi(argv[1]),
				   argc > 2 ? atoi(argv[2]) : cfg.hold_pct);
	}
	if (!strcmp(c, "microsteps")) {
		uint32_t chop;
		int m;

		if (!need(argc, 2, "microsteps N"))
			return -1;
		m = mres_of(atoi(argv[1]));
		if (m < 0) {
			err("microsteps must be a power of two 1..256");
			return -1;
		}
		if (load_vactual()) {
			err("%sstop the motor first", tag());
			return -1;
		}
		if (tmc_read(REG_CHOPCONF, &chop))
			return -1;
		chop = (chop & ~CHOP_MRES_MASK) | (uint32_t)m << CHOP_MRES_SHIFT;
		if (tmc_write(REG_CHOPCONF, chop))
			return -1;
		pos_lost(cur);
		printf("%smicrosteps = %d\n", tag(), 256 >> m);
		return 0;
	}
	if (!strcmp(c, "mode")) {
		uint32_t g;

		if (!need(argc, 2, "mode stealth|spread") ||
		    tmc_read(REG_GCONF, &g))
			return -1;
		if (!strncmp(argv[1], "stealth", 7))
			g &= ~GCONF_EN_SPREADCYCLE;
		else if (!strncmp(argv[1], "spread", 6))
			g |= GCONF_EN_SPREADCYCLE;
		else {
			err("mode must be stealth or spread");
			return -1;
		}
		if (tmc_write(REG_GCONF, g))
			return -1;
		printf("%smode = %s\n", tag(), (g & GCONF_EN_SPREADCYCLE) ?
		       "spreadCycle" : "stealthChop");
		return 0;
	}
	if (!strcmp(c, "read")) {
		uint32_t v;
		int r;

		if (!need(argc, 2, "read REG"))
			return -1;
		r = parse_reg(argv[1]);
		if (r < 0) {
			err("unknown register '%s'", argv[1]);
			return -1;
		}
		if (tmc_read(r, &v))
			return -1;
		printf("%s0x%02X = 0x%08X (%u)\n", tag(), r, v, v);
		return 0;
	}
	if (!strcmp(c, "write")) {
		int r;

		if (!need(argc, 3, "write REG VALUE"))
			return -1;
		r = parse_reg(argv[1]);
		if (r < 0) {
			err("unknown register '%s'", argv[1]);
			return -1;
		}
		return tmc_write(r, strtoul(argv[2], NULL, 0));
	}
	err("unknown command '%s' (try 'help')", c);
	return -2;
}

static int run_cmd(int argc, char **argv)
{
	const char *c = argv[0];
	double amount[MAX_MOTORS];
	int k, ret = 0;

	select_motor(sel[0]);
	if (!strcmp(c, "help") || !strcmp(c, "-h") || !strcmp(c, "--help")) {
		usage();
		return 0;
	}
	if (!strcmp(c, "motors")) {
		cmd_motors();
		return 0;
	}
	if (!strcmp(c, "motor")) {
		if (argc < 2) {
			cmd_motors();
			return 0;
		}
		return parse_motor_sel(argv[1]);
	}
	if (!strcmp(c, "turn") || !strcmp(c, "move")) {
		double rpm = argc > 2 ? atof(argv[2]) : cfg.rpm;

		if (!need(argc, 2, "turn REVS [RPM] | move USTEPS [RPM]"))
			return -1;
		for (k = 0; k < nsel; k++)
			amount[k] = atof(argv[1]);
		return move_axes(nsel, sel, amount, !strcmp(c, "turn"), rpm,
				 NULL);
	}
	if (!strcmp(c, "sync")) {
		static const int both[2] = { 0, 1 };
		double rpm = argc > 3 ? atof(argv[3]) : motors[0].rpm;

		if (nmotors < 2) {
			err("sync needs [motor1] and [motor2] in %s", CONF_PATH);
			return -1;
		}
		if (!need(argc, 3, "sync REVS1 REVS2 [RPM]"))
			return -1;
		amount[0] = atof(argv[1]);
		amount[1] = atof(argv[2]);
		return move_axes(2, both, amount, 1, rpm, NULL);
	}
	if (!strcmp(c, "run")) {
		if (!need(argc, 2, "run RPM"))
			return -1;
		return cmd_run(atof(argv[1]));
	}
	if (!strcmp(c, "stop"))
		return cmd_stop();
	if (!strcmp(c, "endstop"))
		return cmd_endstop(argc, argv);
	if (!strcmp(c, "home"))
		return cmd_home(argc, argv);

	/* Everything else: once per selected motor, carry on past failures */
	for (k = 0; k < nsel; k++) {
		int r;

		select_motor(sel[k]);
		if (k && !strcmp(c, "status"))
			putchar('\n');
		r = run_one(argc, argv);
		if (r == -2)
			return -1;
		if (r)
			ret = -1;
	}
	return ret;
}

static int shell(void)
{
	char line[256];

	puts("tmc2209ctl shell - 'help' for commands, 'motor 1|2|all' to "
	     "choose the motor, 'quit' to exit");
	for (;;) {
		char *argv[16];
		int argc = 0;

		if (nsel == 1)
			printf("tmc%d> ", sel[0] + 1);
		else
			fputs("tmc*> ", stdout);
		fflush(stdout);
		if (!fgets(line, sizeof(line), stdin)) {
			if (errno == EINTR && !feof(stdin)) {
				clearerr(stdin);
				stop_req = 0;
				putchar('\n');
				continue;
			}
			putchar('\n');
			return 0;
		}
		for (char *t = strtok(line, " \t\r\n"); t && argc < 16;
		     t = strtok(NULL, " \t\r\n"))
			argv[argc++] = t;
		if (!argc)
			continue;
		if (!strcmp(argv[0], "quit") || !strcmp(argv[0], "exit"))
			return 0;
		if (!strcmp(argv[0], "shell"))
			continue;
		run_cmd(argc, argv);
	}
}

int main(int argc, char **argv)
{
	struct sigaction sa = { .sa_handler = on_sigint };
	const char *conf = getenv("TMC2209_CONF");
	const char *dev = NULL, *msel = NULL;
	int i, baud = 0, addr = -1;

	/* -c must be handled before the others so the CLI overrides it */
	for (i = 1; i < argc - 1; i++)
		if (!strcmp(argv[i], "-c"))
			conf = argv[i + 1];
	if (!conf)
		conf = CONF_PATH;

	config_init(conf);

	for (i = 1; i < argc && argv[i][0] == '-' && argv[i][1] &&
	     !isdigit((unsigned char)argv[i][1]); i++) {
		if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help"))
			break;
		if (i + 1 >= argc) {
			usage();
			return 1;
		}
		if (!strcmp(argv[i], "-d"))
			dev = argv[++i];
		else if (!strcmp(argv[i], "-a"))
			addr = atoi(argv[++i]);
		else if (!strcmp(argv[i], "-b"))
			baud = atoi(argv[++i]);
		else if (!strcmp(argv[i], "-m"))
			msel = argv[++i];
		else if (!strcmp(argv[i], "-c"))
			i++;
		else {
			err("unknown option %s", argv[i]);
			return 1;
		}
	}
	for (int m = 0; m < nmotors; m++) {
		if (dev)
			snprintf(motors[m].dev, sizeof(motors[m].dev), "%s", dev);
		if (baud)
			motors[m].baud = baud;
	}
	if (msel && parse_motor_sel(msel))
		return 1;
	if (addr >= 0) {
		if (nsel != 1) {
			err("-a needs a single motor (-m 1, -m 2, ...)");
			return 1;
		}
		motors[sel[0]].addr = addr;
	}
	select_motor(sel[0]);
	if (i >= argc) {
		usage();
		return 1;
	}

	/* No SA_RESTART: Ctrl-C must interrupt fgets() in the shell */
	sigaction(SIGINT, &sa, NULL);

	if (!strcmp(argv[i], "shell"))
		return shell();
	return run_cmd(argc - i, argv + i) ? 1 : 0;
}
