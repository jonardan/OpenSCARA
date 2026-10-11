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

#define CONF_PATH	"/etc/tmc2209.conf"
#define STATE_PATH	"/run/tmc2209.%d.vactual"	/* per UART address */
#define POS_PATH	"/run/scara.pos"
#define MAX_MOTORS	4

/* TMC2209 registers */
#define REG_GCONF	0x00
#define REG_GSTAT	0x01
#define REG_IFCNT	0x02
#define REG_IOIN	0x06
#define REG_IHOLD_IRUN	0x10
#define REG_TPOWERDOWN	0x11
#define REG_TSTEP	0x12
#define REG_TPWMTHRS	0x13
#define REG_VACTUAL	0x22
#define REG_SG_RESULT	0x41
#define REG_MSCNT	0x6A
#define REG_CHOPCONF	0x6C
#define REG_DRV_STATUS	0x6F
#define REG_PWMCONF	0x70

#define GCONF_PDN_DISABLE	(1u << 6)
#define GCONF_MSTEP_REG_SELECT	(1u << 7)
#define GCONF_MULTISTEP_FILT	(1u << 8)
#define GCONF_EN_SPREADCYCLE	(1u << 2)
#define GCONF_SHAFT		(1u << 3)

#define CHOP_TOFF_MASK		0xFu
#define CHOP_VSENSE		(1u << 17)
#define CHOP_MRES_SHIFT		24
#define CHOP_MRES_MASK		(0xFu << CHOP_MRES_SHIFT)
#define CHOP_DEDGE		(1u << 29)
#define CHOP_TOFF_ON		3

#define TMC_FCLK	12000000.0	/* internal oscillator, Hz */
#define RSENSE		0.110		/* BTT TMC2209 sense resistor, ohm */
#define MAX_STEP_RATE	20000.0		/* GPIO stepping limit, usteps/s */

struct config {
	char dev[64];
	int addr;
	int baud;
	int current_ma;
	int hold_pct;
	int microsteps;
	int spreadcycle;
	int invert_dir;
	int steps_per_rev;
	double rpm;
	double accel;		/* rev/s^2 */
	char step_line[32];
	char dir_line[32];
	char endstop_line[32];	/* empty: no endstop */
	int endstop_nc;		/* switch wired COM/NC: line high when pressed */
	int home_dir;		/* +1/-1: motor direction toward the switch */
	double home_angle;	/* joint angle (deg) at the switch */
	double home_rpm;	/* homing speed (motor rpm) */
	int motor_teeth;	/* pulley teeth: joint turns motor_teeth/joint_teeth */
	int joint_teeth;	/*   of a revolution per motor revolution */
};

static struct config cfg = {
	.dev = "/dev/ttyS1",
	.addr = 0,
	.baud = 115200,
	.current_ma = 600,
	.hold_pct = 50,
	.microsteps = 16,
	.spreadcycle = 0,
	.invert_dir = 0,
	.steps_per_rev = 200,
	.rpm = 60,
	.accel = 2,
	.step_line = "P9_16 [ehrpwm1b]",
	.dir_line = "P9_18 [spi0_d1]",
	.home_rpm = 20,
	.motor_teeth = 1,
	.joint_teeth = 1,
};

/* cfg is the active motor: a copy of motors[cur] */
static struct config motors[MAX_MOTORS];
static int nmotors = 1;
static int cur;

/* Motors the commands act on (-m, or "motor" in the shell) */
static int sel[MAX_MOTORS] = { 0 };
static int nsel = 1;

static int uart_fd = -1;
static char uart_dev[64];
static volatile sig_atomic_t stop_req;

static void select_motor(int i)
{
	cur = i;
	cfg = motors[i];
}

/* "motor 2: " prefix for messages, empty with a single motor */
static const char *tag(void)
{
	static char buf[24];

	if (nmotors < 2)
		return "";
	snprintf(buf, sizeof(buf), "motor %d: ", cur + 1);
	return buf;
}

static void on_sigint(int sig)
{
	(void)sig;
	stop_req = 1;
}

static void err(const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	fputs("error: ", stderr);
	vfprintf(stderr, fmt, ap);
	fputc('\n', stderr);
	va_end(ap);
}

/* ------------------------------------------------------------------ */
/* Configuration file                                                  */
/* ------------------------------------------------------------------ */

static char *trim(char *s)
{
	char *e;

	while (isspace((unsigned char)*s))
		s++;
	e = s + strlen(s);
	while (e > s && isspace((unsigned char)e[-1]))
		*--e = '\0';
	return s;
}

static int set_option(const char *key, const char *val)
{
	if (!strcmp(key, "device"))
		snprintf(cfg.dev, sizeof(cfg.dev), "%s", val);
	else if (!strcmp(key, "address"))
		cfg.addr = atoi(val);
	else if (!strcmp(key, "baud"))
		cfg.baud = atoi(val);
	else if (!strcmp(key, "current_ma"))
		cfg.current_ma = atoi(val);
	else if (!strcmp(key, "hold_percent"))
		cfg.hold_pct = atoi(val);
	else if (!strcmp(key, "microsteps"))
		cfg.microsteps = atoi(val);
	else if (!strcmp(key, "mode"))
		cfg.spreadcycle = !strcmp(val, "spreadcycle");
	else if (!strcmp(key, "invert_dir"))
		cfg.invert_dir = atoi(val);
	else if (!strcmp(key, "steps_per_rev"))
		cfg.steps_per_rev = atoi(val);
	else if (!strcmp(key, "rpm"))
		cfg.rpm = atof(val);
	else if (!strcmp(key, "accel"))
		cfg.accel = atof(val);
	else if (!strcmp(key, "step_line"))
		snprintf(cfg.step_line, sizeof(cfg.step_line), "%s", val);
	else if (!strcmp(key, "dir_line"))
		snprintf(cfg.dir_line, sizeof(cfg.dir_line), "%s", val);
	else if (!strcmp(key, "endstop_line"))
		snprintf(cfg.endstop_line, sizeof(cfg.endstop_line), "%s", val);
	else if (!strcmp(key, "endstop_nc"))
		cfg.endstop_nc = atoi(val);
	else if (!strcmp(key, "home_dir"))
		cfg.home_dir = atoi(val);
	else if (!strcmp(key, "home_angle"))
		cfg.home_angle = atof(val);
	else if (!strcmp(key, "home_rpm"))
		cfg.home_rpm = atof(val);
	else if (!strcmp(key, "motor_teeth"))
		cfg.motor_teeth = atoi(val);
	else if (!strcmp(key, "joint_teeth"))
		cfg.joint_teeth = atoi(val);
	else
		return -1;
	return 0;
}

/*
 * Apply the keys of one section of the file to cfg: section 0 is the part
 * before any [motorN] header (shared defaults), N is [motorN].  Returns the
 * highest motor number found in the file.
 */
static int load_config(const char *path, int section)
{
	char line[256];
	int n = 0, in = 0, max = 0;
	FILE *f = fopen(path, "r");

	if (!f)
		return 0;
	while (fgets(line, sizeof(line), f)) {
		char *s = trim(line), *eq;
		int m;

		n++;
		if (!*s || *s == '#')
			continue;
		if (*s == '[') {
			if (sscanf(s, "[motor%d]", &m) == 1 && m >= 1 &&
			    m <= MAX_MOTORS) {
				in = m;
				if (m > max)
					max = m;
			} else {
				if (!section)
					fprintf(stderr, "%s:%d: unknown section %s "
						"(use [motor1]..[motor%d])\n",
						path, n, s, MAX_MOTORS);
				in = -1;
			}
			continue;
		}
		if (in != section)
			continue;
		eq = strchr(s, '=');
		if (!eq) {
			fprintf(stderr, "%s:%d: ignoring malformed line\n", path, n);
			continue;
		}
		*eq = '\0';
		if (set_option(trim(s), trim(eq + 1)))
			fprintf(stderr, "%s:%d: unknown key '%s'\n", path, n, trim(s));
	}
	fclose(f);
	return max;
}

/* ------------------------------------------------------------------ */
/* TMC2209 UART                                                        */
/* ------------------------------------------------------------------ */

static uint8_t tmc_crc(const uint8_t *d, int len)
{
	uint8_t crc = 0;

	for (int i = 0; i < len; i++) {
		uint8_t b = d[i];

		for (int j = 0; j < 8; j++) {
			if ((crc >> 7) ^ (b & 1))
				crc = (crc << 1) ^ 0x07;
			else
				crc <<= 1;
			b >>= 1;
		}
	}
	return crc;
}

static speed_t baud_const(int baud)
{
	switch (baud) {
	case 9600: return B9600;
	case 19200: return B19200;
	case 38400: return B38400;
	case 57600: return B57600;
	case 115200: return B115200;
	case 230400: return B230400;
	case 460800: return B460800;
	default: return 0;
	}
}

static int uart_open(void)
{
	struct termios t;
	speed_t sp;

	if (uart_fd >= 0 && !strcmp(uart_dev, cfg.dev))
		return 0;
	if (uart_fd >= 0)
		close(uart_fd);
	uart_fd = -1;
	sp = baud_const(cfg.baud);
	if (!sp) {
		err("unsupported baud rate %d", cfg.baud);
		return -1;
	}
	uart_fd = open(cfg.dev, O_RDWR | O_NOCTTY);
	if (uart_fd < 0) {
		err("cannot open %s: %s", cfg.dev, strerror(errno));
		return -1;
	}
	if (tcgetattr(uart_fd, &t)) {
		err("%s is not a serial port", cfg.dev);
		close(uart_fd);
		uart_fd = -1;
		return -1;
	}
	cfmakeraw(&t);
	t.c_cflag |= CLOCAL | CREAD;
	t.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);
	t.c_cc[VMIN] = 0;
	t.c_cc[VTIME] = 0;
	cfsetispeed(&t, sp);
	cfsetospeed(&t, sp);
	tcsetattr(uart_fd, TCSANOW, &t);
	tcflush(uart_fd, TCIOFLUSH);
	snprintf(uart_dev, sizeof(uart_dev), "%s", cfg.dev);
	return 0;
}

static int read_exact(uint8_t *buf, int n, int timeout_ms)
{
	int got = 0;

	while (got < n) {
		struct pollfd p = { .fd = uart_fd, .events = POLLIN };
		int r = poll(&p, 1, timeout_ms);

		if (r <= 0)
			return got;
		r = read(uart_fd, buf + got, n - got);
		if (r <= 0)
			return got;
		got += r;
	}
	return got;
}

/*
 * The TMC2209 UART is a single wire: our TX (through 1k) and RX are both
 * tied to PDN_UART, so everything we send comes back as an echo first.
 * The bus is shared by all drivers (and maybe several tmc2209ctl
 * processes), so each request/reply pair holds an exclusive lock.
 */
static int uart_xfer_locked(const uint8_t *tx, int txlen, uint8_t *rx,
			    int rxlen)
{
	uint8_t echo[8];

	tcflush(uart_fd, TCIFLUSH);
	if (write(uart_fd, tx, txlen) != txlen)
		return -1;
	tcdrain(uart_fd);
	if (read_exact(echo, txlen, 50) != txlen || memcmp(echo, tx, txlen)) {
		err("no UART echo on %s: the RX wire (P9_26) must connect to "
		    "PDN_UART, and TX (P9_24) to PDN_UART through 1k", cfg.dev);
		return -2;
	}
	if (rxlen && read_exact(rx, rxlen, 50) != rxlen)
		return -3;
	return 0;
}

static int uart_xfer(const uint8_t *tx, int txlen, uint8_t *rx, int rxlen)
{
	int r;

	flock(uart_fd, LOCK_EX);
	r = uart_xfer_locked(tx, txlen, rx, rxlen);
	flock(uart_fd, LOCK_UN);
	return r;
}

static int tmc_read(uint8_t reg, uint32_t *val)
{
	uint8_t tx[4] = { 0x05, (uint8_t)cfg.addr, reg, 0 };
	uint8_t rx[8];

	if (uart_open())
		return -1;
	tx[3] = tmc_crc(tx, 3);
	for (int attempt = 0; attempt < 3; attempt++) {
		int r = uart_xfer(tx, 4, rx, 8);

		if (r == -2 || r == -1)
			return -1;
		if (r == 0 && rx[0] == 0x05 && rx[1] == 0xFF && rx[2] == reg &&
		    rx[7] == tmc_crc(rx, 7)) {
			*val = (uint32_t)rx[3] << 24 | (uint32_t)rx[4] << 16 |
			       (uint32_t)rx[5] << 8 | rx[6];
			return 0;
		}
		usleep(2000);
	}
	err("%sno reply from TMC2209 (addr %d) reading reg 0x%02X. Check: 12V "
	    "on VM (the chip needs motor power to talk), 3.3V on VIO, common "
	    "GND, MS1 to %s and MS2 to %s for address %d", tag(), cfg.addr, reg,
	    cfg.addr & 1 ? "3.3V" : "GND", cfg.addr & 2 ? "3.3V" : "GND",
	    cfg.addr);
	return -1;
}

static int tmc_write(uint8_t reg, uint32_t val)
{
	uint8_t tx[8] = {
		0x05, (uint8_t)cfg.addr, (uint8_t)(reg | 0x80),
		val >> 24, val >> 16, val >> 8, val, 0
	};

	if (uart_open())
		return -1;
	tx[7] = tmc_crc(tx, 7);
	if (uart_xfer(tx, 8, NULL, 0))
		return -1;
	/* Give the chip a moment before the next datagram */
	usleep(500);
	return 0;
}

/* ------------------------------------------------------------------ */
/* Position file                                                       */
/* ------------------------------------------------------------------ */

/*
 * POS_PATH keeps each motor's position between commands: microsteps
 * counted by STEP/DIR moves, positive = the direction of "turn 1", and
 * whether it is homed (the count is then relative to the joint at 0 deg).
 * One line per motor: "motorN POSITION HOMED".  It lives in /run, so a
 * reboot clears it; so does anything that loses steps (disable, 'run',
 * a driver reset).
 */
static long pos[MAX_MOTORS];
static int homed[MAX_MOTORS];

static void pos_load(void)
{
	char line[128];
	FILE *f = fopen(POS_PATH, "r");

	memset(pos, 0, sizeof(pos));
	memset(homed, 0, sizeof(homed));
	if (!f)
		return;
	while (fgets(line, sizeof(line), f)) {
		long p;
		int m, h;

		if (sscanf(line, "motor%d %ld %d", &m, &p, &h) == 3 &&
		    m >= 1 && m <= MAX_MOTORS) {
			pos[m - 1] = p;
			homed[m - 1] = !!h;
		}
	}
	fclose(f);
}

static void pos_save(void)
{
	FILE *f = fopen(POS_PATH ".tmp", "w");

	if (!f)
		return;
	fprintf(f, "# motor position (usteps) homed - written by tmc2209ctl\n");
	for (int m = 0; m < nmotors; m++)
		fprintf(f, "motor%d %ld %d\n", m + 1, pos[m], homed[m]);
	fclose(f);
	rename(POS_PATH ".tmp", POS_PATH);
}

static void pos_add(int m, long usteps)
{
	pos_load();
	pos[m] += usteps;
	pos_save();
}

/* The motor may have moved without us counting: position unknown */
static void pos_lost(int m)
{
	pos_load();
	if (homed[m]) {
		homed[m] = 0;
		pos_save();
	}
}

/* ------------------------------------------------------------------ */
/* Driver helpers                                                      */
/* ------------------------------------------------------------------ */

static int mres_of(int microsteps)
{
	for (int m = 0; m <= 8; m++)
		if ((256 >> m) == microsteps)
			return m;
	return -1;
}

/* Current scale (CS 0..31) and vsense for an RMS current, as Klipper does */
static void calc_current(int ma, int *cs, int *vsense)
{
	double i = ma / 1000.0;
	double c = 32.0 * M_SQRT2 * i * (RSENSE + 0.02) / 0.325 - 1.0;

	*vsense = 0;
	if (c < 16.0) {
		*vsense = 1;
		c = 32.0 * M_SQRT2 * i * (RSENSE + 0.02) / 0.180 - 1.0;
	}
	*cs = (int)lround(c);
	if (*cs < 0)
		*cs = 0;
	if (*cs > 31)
		*cs = 31;
}

static double current_of(int cs, int vsense)
{
	double vfs = vsense ? 0.180 : 0.325;

	return (cs + 1) / 32.0 * vfs / (RSENSE + 0.02) / M_SQRT2 * 1000.0;
}

static const char *state_path(void)
{
	static char path[64];

	snprintf(path, sizeof(path), STATE_PATH, cfg.addr);
	return path;
}

static int save_vactual(int32_t v)
{
	FILE *f = fopen(state_path(), "w");

	if (!f)
		return -1;
	fprintf(f, "%d\n", v);
	fclose(f);
	return 0;
}

static int32_t load_vactual(void)
{
	int v = 0;
	FILE *f = fopen(state_path(), "r");

	if (f) {
		if (fscanf(f, "%d", &v) != 1)
			v = 0;
		fclose(f);
	}
	return v;
}

static int set_current(int ma, int hold_pct)
{
	uint32_t chop;
	int cs, vsense, ihold;

	if (ma < 50 || ma > 2000) {
		err("current must be 50..2000 mA RMS");
		return -1;
	}
	if (hold_pct < 0 || hold_pct > 100) {
		err("hold percent must be 0..100");
		return -1;
	}
	calc_current(ma, &cs, &vsense);
	ihold = (cs * hold_pct + 50) / 100;
	if (tmc_read(REG_CHOPCONF, &chop))
		return -1;
	chop = vsense ? chop | CHOP_VSENSE : chop & ~CHOP_VSENSE;
	if (tmc_write(REG_CHOPCONF, chop) ||
	    tmc_write(REG_IHOLD_IRUN, 8u << 16 | (uint32_t)cs << 8 | ihold))
		return -1;
	printf("%srun current %.0f mA RMS (CS=%d, vsense=%d), hold %.0f mA\n",
	       tag(), current_of(cs, vsense), cs, vsense,
	       current_of(ihold, vsense));
	return 0;
}

/* Full configuration from cfg.  Leaves the motor de-energised. */
static int apply_config(void)
{
	uint32_t ioin, ifcnt0, ifcnt1, chop;
	int mres = mres_of(cfg.microsteps);
	int cs, vsense, ihold;
	const int writes = 7;

	if (mres < 0) {
		err("microsteps must be 1,2,4,...,256 (got %d)", cfg.microsteps);
		return -1;
	}
	if (tmc_read(REG_IOIN, &ioin))
		return -1;
	if ((ioin >> 24) != 0x21)
		fprintf(stderr, "warning: IOIN version 0x%02X, expected 0x21 "
			"(TMC2209)\n", ioin >> 24);
	if (tmc_read(REG_IFCNT, &ifcnt0))
		return -1;

	calc_current(cfg.current_ma, &cs, &vsense);
	ihold = (cs * cfg.hold_pct + 50) / 100;
	chop = 0u			/* toff=0: driver off */
	     | 5u << 4			/* hstrt */
	     | 0u << 7			/* hend */
	     | 2u << 15			/* tbl */
	     | (vsense ? CHOP_VSENSE : 0)
	     | (uint32_t)mres << CHOP_MRES_SHIFT
	     | 1u << 28;		/* intpol: interpolate to 256 usteps */

	if (tmc_write(REG_GSTAT, 0x7) ||
	    tmc_write(REG_GCONF, GCONF_PDN_DISABLE | GCONF_MSTEP_REG_SELECT |
		      GCONF_MULTISTEP_FILT |
		      (cfg.spreadcycle ? GCONF_EN_SPREADCYCLE : 0)) ||
	    tmc_write(REG_CHOPCONF, chop) ||
	    tmc_write(REG_IHOLD_IRUN, 8u << 16 | (uint32_t)cs << 8 | ihold) ||
	    tmc_write(REG_TPOWERDOWN, 20) ||
	    tmc_write(REG_TPWMTHRS, 0) ||
	    tmc_write(REG_VACTUAL, 0))
		return -1;
	save_vactual(0);

	if (tmc_read(REG_IFCNT, &ifcnt1))
		return -1;
	if (((ifcnt1 - ifcnt0) & 0xFF) != writes) {
		err("%sTMC2209 accepted %u of %d writes; check wiring/noise",
		    tag(), (ifcnt1 - ifcnt0) & 0xFF, writes);
		return -1;
	}
	pos_lost(cur);
	printf("%sTMC2209 (addr %d) configured: %d usteps, %s, run %.0f mA, hold %.0f mA "
	       "(motor off)\n", tag(), cfg.addr, cfg.microsteps,
	       cfg.spreadcycle ? "spreadCycle" : "stealthChop",
	       current_of(cs, vsense), current_of(ihold, vsense));
	return 0;
}

/* Re-apply the configuration if the driver was power-cycled (12V off/on) */
static int ensure_ready(void)
{
	uint32_t gstat;

	if (tmc_read(REG_GSTAT, &gstat))
		return -1;
	if (gstat & 1) {
		printf("%sdriver was reset (power cycle?), applying %s\n",
		       tag(), CONF_PATH);
		return apply_config();
	}
	if (gstat & 2)
		fprintf(stderr, "%swarning: driver error flag set (overtemp "
			"or short); see 'status'\n", tag());
	return 0;
}

static int set_enabled(int on)
{
	uint32_t chop;

	if (tmc_read(REG_CHOPCONF, &chop))
		return -1;
	chop &= ~CHOP_TOFF_MASK;
	if (on)
		chop |= CHOP_TOFF_ON;
	return tmc_write(REG_CHOPCONF, chop);
}

static int get_microsteps(int *usteps)
{
	uint32_t chop;

	if (tmc_read(REG_CHOPCONF, &chop))
		return -1;
	*usteps = 256 >> ((chop & CHOP_MRES_MASK) >> CHOP_MRES_SHIFT);
	return 0;
}

static double usteps_per_rev(int usteps)
{
	return (double)cfg.steps_per_rev * usteps;
}

/* ------------------------------------------------------------------ */
/* Velocity mode (VACTUAL): the chip generates the steps itself        */
/* ------------------------------------------------------------------ */

/* VACTUAL unit is usteps per (2^24 / fCLK) seconds */
static int32_t rpm_to_vactual(double rpm, int usteps)
{
	double hz = rpm / 60.0 * usteps_per_rev(usteps);
	double v = hz * 16777216.0 / TMC_FCLK;

	if (v > 8388607)
		v = 8388607;
	if (v < -8388607)
		v = -8388607;
	return (int32_t)lround(v);
}

/*
 * Ramp the selected motors (indexes mi[0..count-1]) to their VACTUAL
 * targets together, one velocity step per motor every 20 ms.
 */
static int ramp_motors(int count, const int *mi, const int32_t *targets)
{
	int32_t cur_v[MAX_MOTORS], target[MAX_MOTORS], step[MAX_MOTORS];
	int k, busy = 1, ret = 0;

	for (k = 0; k < count; k++) {
		int usteps;
		double dv;

		select_motor(mi[k]);
		if (get_microsteps(&usteps))
			return -1;
		cur_v[k] = load_vactual();
		target[k] = targets[k];
		/* velocity change per 20 ms tick */
		dv = cfg.accel * usteps_per_rev(usteps) * 16777216.0 /
		     TMC_FCLK * 0.02;
		step[k] = dv < 1 ? 1 : (int32_t)dv;
	}
	while (busy && !ret) {
		busy = 0;
		if (stop_req) {
			/* Ctrl-C while ramping: slow everything down */
			for (k = 0; k < count; k++)
				target[k] = 0;
			stop_req = 0;
		}
		for (k = 0; k < count; k++) {
			int32_t c = cur_v[k], t = target[k];

			if (c == t)
				continue;
			busy = 1;
			if (c < t)
				c = c + step[k] > t ? t : c + step[k];
			else
				c = c - step[k] < t ? t : c - step[k];
			select_motor(mi[k]);
			if (tmc_write(REG_VACTUAL, (uint32_t)c & 0xFFFFFF)) {
				ret = -1;
				break;
			}
			cur_v[k] = c;
		}
		if (busy && !ret)
			usleep(20000);
	}
	for (k = 0; k < count; k++) {
		select_motor(mi[k]);
		save_vactual(cur_v[k]);
	}
	return ret;
}

static int stop_one(void)
{
	int32_t zero = 0;
	int me = cur;

	return ramp_motors(1, &me, &zero);
}

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
/* STEP/DIR through GPIO character devices                             */
/* ------------------------------------------------------------------ */

static int find_line(const char *name, char *chip, size_t len, int *offset)
{
	DIR *d = opendir("/dev");
	struct dirent *e;
	int found = -1;

	if (!d)
		return -1;
	while (found < 0 && (e = readdir(d))) {
		struct gpiochip_info ci;
		char path[300];
		int fd;

		if (strncmp(e->d_name, "gpiochip", 8))
			continue;
		snprintf(path, sizeof(path), "/dev/%s", e->d_name);
		fd = open(path, O_RDWR);
		if (fd < 0)
			continue;
		if (!ioctl(fd, GPIO_GET_CHIPINFO_IOCTL, &ci)) {
			for (unsigned int i = 0; i < ci.lines; i++) {
				struct gpioline_info li = { .line_offset = i };

				if (!ioctl(fd, GPIO_GET_LINEINFO_IOCTL, &li) &&
				    !strcmp(li.name, name)) {
					snprintf(chip, len, "%s", path);
					*offset = i;
					found = 0;
					break;
				}
			}
		}
		close(fd);
	}
	closedir(d);
	return found;
}

/* Open the gpiochip holding line 'name'; its offset goes to *offset */
static int open_chip_of(const char *name, int *offset)
{
	char chip[300];

	if (find_line(name, chip, sizeof(chip), offset)) {
		err("GPIO line '%s' not found (is the tmc2209 device tree "
		    "loaded?)", name);
		return -1;
	}
	return open(chip, O_RDWR);
}

static int request_line(const char *name, int value)
{
	struct gpiohandle_request req;
	int offset, fd, r;

	fd = open_chip_of(name, &offset);
	if (fd < 0)
		return -1;
	memset(&req, 0, sizeof(req));
	req.lineoffsets[0] = offset;
	req.lines = 1;
	req.flags = GPIOHANDLE_REQUEST_OUTPUT;
	req.default_values[0] = value;
	snprintf(req.consumer_label, sizeof(req.consumer_label), "tmc2209ctl");
	r = ioctl(fd, GPIO_GET_LINEHANDLE_IOCTL, &req);
	close(fd);
	if (r) {
		err("cannot request GPIO '%s': %s", name, strerror(errno));
		return -1;
	}
	return req.fd;
}

static inline void set_line(int fd, int v)
{
	struct gpiohandle_data d = { .values = { v } };

	ioctl(fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &d);
}

/*
 * Input line that also reports both edges (read the events from the fd);
 * the value can be read from it like from a line handle.
 */
static int request_input(const char *name)
{
	struct gpioevent_request req;
	int offset, fd, r;

	fd = open_chip_of(name, &offset);
	if (fd < 0)
		return -1;
	memset(&req, 0, sizeof(req));
	req.lineoffset = offset;
	req.handleflags = GPIOHANDLE_REQUEST_INPUT;
	req.eventflags = GPIOEVENT_REQUEST_BOTH_EDGES;
	snprintf(req.consumer_label, sizeof(req.consumer_label), "tmc2209ctl");
	r = ioctl(fd, GPIO_GET_LINEEVENT_IOCTL, &req);
	close(fd);
	if (r) {
		err("cannot request GPIO '%s': %s", name, strerror(errno));
		return -1;
	}
	return req.fd;
}

static int get_line(int fd)
{
	struct gpiohandle_data d;

	if (ioctl(fd, GPIOHANDLE_GET_LINE_VALUES_IOCTL, &d))
		return -1;
	return d.values[0];
}

static void ts_add(struct timespec *t, double sec)
{
	long ns = (long)(sec * 1e9);

	t->tv_nsec += ns;
	while (t->tv_nsec >= 1000000000L) {
		t->tv_nsec -= 1000000000L;
		t->tv_sec++;
	}
}

/* Pull-up input: a COM/NO switch pulls the line low when pressed */
static int endstop_pressed(const struct config *c, int value)
{
	return !!value == !!c->endstop_nc;
}

/*
 * Optional stop condition for move_axes(), and what happened.  With fd >= 0
 * (an endstop line, single motor) the move stops as soon as the switch
 * reads 'pressed' (1) or released (0) on ENDSTOP_READS checks in a row.
 */
#define ENDSTOP_READS	3

struct until {
	int fd;
	int pressed;
	int hit;		/* out: stopped on the switch */
	int interrupted;	/* out: Ctrl-C */
	long done;		/* out: usteps moved, signed */
};

struct axis {
	int motor;
	int sign;
	long n, done, err;
	int step_fd, dir_fd;
	double upr;		/* usteps per revolution */
	double accel;		/* usteps/s^2 */
};

/*
 * Exact move of several motors at once over STEP/DIR.  amount[k] is in
 * revolutions (revs != 0) or microsteps for motor mi[k].  The motor with
 * the longest move runs at rpm with the acceleration ramp; the others are
 * spread over it (Bresenham), so all of them start and finish together.
 * With u, the move can also stop on an endstop (see struct until).
 */
static int move_axes(int count, const int *mi, const double *amount, int revs,
		     double rpm, struct until *u)
{
	struct sched_param sp = { .sched_priority = 50 };
	struct axis ax[MAX_MOTORS], *m = NULL;
	struct timespec next;
	double vmax, a, v0;
	int k, ret = -1, reads = 0;
	long n;

	if (rpm <= 0) {
		err("speed must be > 0 rpm");
		return -1;
	}
	memset(ax, 0, sizeof(ax));
	for (k = 0; k < count; k++)
		ax[k].step_fd = ax[k].dir_fd = -1;
	if (u)
		u->hit = u->interrupted = u->done = 0;

	for (k = 0; k < count; k++) {
		struct axis *x = &ax[k];
		uint32_t chop;
		long steps;

		select_motor(mi[k]);
		x->motor = mi[k];
		if (ensure_ready())
			goto out;
		/* VACTUAL must be 0 or STEP input is ignored */
		if (load_vactual() && stop_one())
			goto out;
		if (tmc_read(REG_CHOPCONF, &chop))
			goto out;
		x->upr = usteps_per_rev(256 >>
				((chop & CHOP_MRES_MASK) >> CHOP_MRES_SHIFT));
		x->accel = cfg.accel * x->upr;
		steps = lround(revs ? amount[k] * x->upr : amount[k]);
		x->n = labs(steps);
		x->sign = steps < 0 ? -1 : 1;
		if (!x->n)
			continue;
		chop &= ~(CHOP_DEDGE | CHOP_TOFF_MASK);
		chop |= CHOP_TOFF_ON;
		if (tmc_write(REG_CHOPCONF, chop))
			goto out;
		x->dir_fd = request_line(cfg.dir_line,
					 (steps > 0) ^ !!cfg.invert_dir);
		if (x->dir_fd < 0)
			goto out;
		x->step_fd = request_line(cfg.step_line, 0);
		if (x->step_fd < 0)
			goto out;
		if (!m || x->n > m->n)
			m = x;
	}
	if (!m) {
		ret = 0;
		goto out;
	}

	vmax = rpm / 60.0 * m->upr;
	if (vmax > MAX_STEP_RATE) {
		fprintf(stderr, "warning: %.0f usteps/s is above the GPIO limit, "
			"capping at %.0f (%.0f rpm). Use 'run' for higher "
			"speeds.\n", vmax, MAX_STEP_RATE,
			MAX_STEP_RATE * 60.0 / m->upr);
		vmax = MAX_STEP_RATE;
	}
	a = m->accel;
	v0 = fmin(vmax, 0.05 * m->upr);
	for (k = 0; k < count; k++)
		ax[k].err = m->n / 2;

	mlockall(MCL_CURRENT | MCL_FUTURE);
	if (sched_setscheduler(0, SCHED_FIFO, &sp))
		fprintf(stderr, "warning: no realtime priority, motion may "
			"stutter\n");

	stop_req = 0;
	n = m->n;
	clock_gettime(CLOCK_MONOTONIC, &next);
	ts_add(&next, 20e-6);	/* DIR setup time */
	while (m->done < n) {
		double va = sqrt(v0 * v0 + 2 * a * m->done);
		double vd = sqrt(v0 * v0 + 2 * a * (n - m->done));
		double v = fmin(vmax, fmin(va, vd));
		int fire[MAX_MOTORS];
		struct timespec now;

		if (stop_req) {
			/* Ctrl-C: decelerate to a stop instead of halting */
			long dec = (long)((v * v - v0 * v0) / (2 * a));

			if (m->done + dec < n)
				n = m->done + dec;
			stop_req = 0;
			if (u)
				u->interrupted = 1;
		}
		if (u && u->fd >= 0) {
			int val = get_line(u->fd);

			if (val >= 0 && endstop_pressed(&motors[m->motor], val) ==
			    u->pressed)
				reads++;
			else
				reads = 0;
			if (reads >= ENDSTOP_READS) {
				u->hit = 1;
				break;
			}
		}
		for (k = 0; k < count; k++) {
			struct axis *x = &ax[k];

			fire[k] = 0;
			if (!x->n || x->done >= x->n)
				continue;
			x->err += x->n;
			if (x->err >= m->n) {
				x->err -= m->n;
				fire[k] = 1;
			}
		}
		clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
		for (k = 0; k < count; k++)
			if (fire[k])
				set_line(ax[k].step_fd, 1);
		for (k = 0; k < count; k++)
			if (fire[k]) {
				set_line(ax[k].step_fd, 0);
				ax[k].done++;
			}
		ts_add(&next, 1.0 / v);
		/* If we fell far behind, don't burst to catch up */
		clock_gettime(CLOCK_MONOTONIC, &now);
		if (now.tv_sec > next.tv_sec + 1 ||
		    (now.tv_sec - next.tv_sec) * 1e9 +
		    (now.tv_nsec - next.tv_nsec) > 2e9 / v)
			next = now;
	}

	sp.sched_priority = 0;
	sched_setscheduler(0, SCHED_OTHER, &sp);
	ret = 0;
out:
	for (k = 0; k < count; k++) {
		struct axis *x = &ax[k];

		if (x->step_fd >= 0)
			close(x->step_fd);
		if (x->dir_fd >= 0)
			close(x->dir_fd);
		if (ret || !x->n)
			continue;
		pos_add(x->motor, x->sign * x->done);
		if (u) {
			u->done = x->sign * x->done;
			continue;
		}
		select_motor(x->motor);
		printf("%smoved %ld usteps (%.3f rev)%s\n", tag(),
		       x->sign * x->done, x->sign * x->done / x->upr,
		       x->done < x->n ? " - interrupted" : "");
	}
	return ret;
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

/* ------------------------------------------------------------------ */
/* Homing                                                              */
/* ------------------------------------------------------------------ */

#define HOME_CLEAR_DEG		90	/* max travel to get off a pressed switch */
#define HOME_MAX_DEG		360	/* max travel to find the switch */
#define HOME_BACKOFF_DEG	5	/* back off before the slow approach */
#define HOME_SLOW_DIV		5	/* slow approach = home_rpm / this */

/*
 * Turn motor m's joint by deg (positive = toward the switch) at rpm,
 * stopping early if u->fd says so.  Fails on Ctrl-C.
 */
static int home_move(int m, double deg, double rpm, struct until *u)
{
	const struct config *c = &motors[m];
	double revs = deg / 360.0 * c->joint_teeth / c->motor_teeth *
		      c->home_dir;

	if (stop_req || move_axes(1, &m, &revs, 1, rpm, u))
		return -1;
	if (u->interrupted) {
		err("motor %d: homing interrupted", m + 1);
		return -1;
	}
	return 0;
}

static int home_one(int m)
{
	const struct config *c = &motors[m];
	struct until u = { .fd = -1 };
	double slow = c->home_rpm / HOME_SLOW_DIV;
	int fd, usteps, v, ret = -1;

	fd = request_input(c->endstop_line);
	if (fd < 0)
		return -1;
	u.fd = fd;
	pos_lost(m);
	printf("motor %d: homing (home_dir %+d, %.0f rpm)\n", m + 1,
	       c->home_dir, c->home_rpm);
	fflush(stdout);

	v = get_line(fd);
	if (v >= 0 && endstop_pressed(c, v)) {
		printf("motor %d: switch already pressed, moving off it\n", m + 1);
		u.pressed = 0;
		if (home_move(m, -HOME_CLEAR_DEG, c->home_rpm, &u))
			goto out;
		if (!u.hit) {
			err("motor %d: switch still pressed after %d deg away "
			    "from it; check its wiring and endstop_nc", m + 1,
			    HOME_CLEAR_DEG);
			goto out;
		}
	}

	/* Fast approach */
	u.pressed = 1;
	if (home_move(m, HOME_MAX_DEG, c->home_rpm, &u))
		goto out;
	if (!u.hit) {
		err("motor %d: switch not reached after %d deg of joint travel; "
		    "is it unplugged or broken, or home_dir wrong?", m + 1,
		    HOME_MAX_DEG);
		goto out;
	}

	/* Back off, then slow approach for a repeatable click */
	u.fd = -1;
	if (home_move(m, -HOME_BACKOFF_DEG, c->home_rpm, &u))
		goto out;
	v = get_line(fd);
	if (v < 0 || endstop_pressed(c, v)) {
		err("motor %d: switch still pressed after backing off %d deg",
		    m + 1, HOME_BACKOFF_DEG);
		goto out;
	}
	u.fd = fd;
	if (home_move(m, 2 * HOME_BACKOFF_DEG, slow, &u))
		goto out;
	if (!u.hit) {
		err("motor %d: switch not reached again on the slow approach",
		    m + 1);
		goto out;
	}

	/* The joint is now at home_angle */
	select_motor(m);
	if (get_microsteps(&usteps))
		goto out;
	pos_load();
	pos[m] = lround(c->home_angle * usteps_per_rev(usteps) *
			c->joint_teeth / c->motor_teeth / 360.0);
	homed[m] = 1;
	pos_save();
	printf("motor %d: homed, joint at %.1f deg\n", m + 1, c->home_angle);
	ret = 0;
out:
	close(fd);
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

	/* Check every motor's settings before moving any of them */
	for (k = 0; k < n; k++) {
		const struct config *c = &motors[mi[k]];

		if (!c->endstop_line[0]) {
			err("motor %d: no endstop_line in %s", mi[k] + 1,
			    CONF_PATH);
			return -1;
		}
		if (c->home_dir != 1 && c->home_dir != -1) {
			err("motor %d: set home_dir = 1 or -1 in [motor%d] of %s "
			    "(the direction of 'turn' that goes toward the "
			    "switch)", mi[k] + 1, mi[k] + 1, CONF_PATH);
			return -1;
		}
		if (c->motor_teeth <= 0 || c->joint_teeth <= 0 ||
		    c->home_rpm <= 0) {
			err("motor %d: motor_teeth, joint_teeth and home_rpm "
			    "must be > 0", mi[k] + 1);
			return -1;
		}
	}
	stop_req = 0;
	for (k = 0; k < n; k++)
		if (home_one(mi[k]))
			return -1;
	return 0;
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
	struct config base;
	int i, baud = 0, addr = -1;

	/* -c must be handled before the others so the CLI overrides it */
	for (i = 1; i < argc - 1; i++)
		if (!strcmp(argv[i], "-c"))
			conf = argv[i + 1];
	if (!conf)
		conf = CONF_PATH;

	/* Shared keys first, then each [motorN] section on top of them */
	nmotors = load_config(conf, 0);
	base = cfg;
	if (!nmotors)
		nmotors = 1;
	for (i = 0; i < nmotors; i++) {
		cfg = base;
		load_config(conf, i + 1);
		motors[i] = cfg;
	}

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
