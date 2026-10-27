// SPDX-License-Identifier: MIT
/*
 * TMC2209 driver code shared by tmc2209ctl and scara: the configuration
 * file, the single-wire UART, velocity mode, STEP/DIR moves over GPIO,
 * endstops, homing and the position file.  See tmc2209.h.
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

struct config cfg = {
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
	.line_speed = 20,
	.line_accel = 100,
	.corner_deg = 15,
	.led_red = "tip-red",
	.led_green = "tip-green",
	.led_blue = "tip-blue",
};

/* cfg is the active motor: a copy of motors[cur] */
struct config motors[MAX_MOTORS];
int nmotors = 1;
int cur;

static int uart_fd = -1;
static char uart_dev[64];
volatile sig_atomic_t stop_req;

void select_motor(int i)
{
	cur = i;
	cfg = motors[i];
}

/* "motor 2: " prefix for messages, empty with a single motor */
const char *tag(void)
{
	static char buf[24];

	if (nmotors < 2)
		return "";
	snprintf(buf, sizeof(buf), "motor %d: ", cur + 1);
	return buf;
}

void on_sigint(int sig)
{
	(void)sig;
	stop_req = 1;
}

void err(const char *fmt, ...)
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
	else if (!strcmp(key, "min_angle"))
		cfg.min_angle = atof(val);
	else if (!strcmp(key, "max_angle"))
		cfg.max_angle = atof(val);
	else if (!strcmp(key, "link1_mm"))
		cfg.link1_mm = atof(val);
	else if (!strcmp(key, "link2_mm"))
		cfg.link2_mm = atof(val);
	else if (!strcmp(key, "line_speed"))
		cfg.line_speed = atof(val);
	else if (!strcmp(key, "line_accel"))
		cfg.line_accel = atof(val);
	else if (!strcmp(key, "corner_deg"))
		cfg.corner_deg = atof(val);
	else if (!strcmp(key, "max_y_mm")) {
		cfg.max_y_off = !strcmp(val, "off");
		cfg.max_y_mm = cfg.max_y_off ? 0 : atof(val);
	} else if (!strcmp(key, "led_red") || !strcmp(key, "led"))
		snprintf(cfg.led_red, sizeof(cfg.led_red), "%s", val);
	else if (!strcmp(key, "led_green"))
		snprintf(cfg.led_green, sizeof(cfg.led_green), "%s", val);
	else if (!strcmp(key, "led_blue"))
		snprintf(cfg.led_blue, sizeof(cfg.led_blue), "%s", val);
	else if (!strcmp(key, "led_common")) {
		if (strcmp(val, "cathode") && strcmp(val, "anode"))
			return -1;
		cfg.led_anode = !strcmp(val, "anode");
	} else {
		return -1;
	}
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

/* Shared keys of the file first, then each [motorN] section on top of them */
void config_init(const char *path)
{
	struct config base;

	nmotors = load_config(path, 0);
	base = cfg;
	if (!nmotors)
		nmotors = 1;
	for (int i = 0; i < nmotors; i++) {
		cfg = base;
		load_config(path, i + 1);
		motors[i] = cfg;
	}
	select_motor(0);
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

int tmc_read(uint8_t reg, uint32_t *val)
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

int tmc_write(uint8_t reg, uint32_t val)
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
long pos[MAX_MOTORS];
int homed[MAX_MOTORS];

void pos_load(void)
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
	fprintf(f, "# motor position (usteps) homed\n");
	for (int m = 0; m < nmotors; m++)
		fprintf(f, "motor%d %ld %d\n", m + 1, pos[m], homed[m]);
	fclose(f);
	rename(POS_PATH ".tmp", POS_PATH);
}

void pos_add(int m, long usteps)
{
	pos_load();
	pos[m] += usteps;
	pos_save();
}

/* The motor may have moved without us counting: position unknown */
void pos_lost(int m)
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

int mres_of(int microsteps)
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

double current_of(int cs, int vsense)
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

int32_t load_vactual(void)
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

int set_current(int ma, int hold_pct)
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
int apply_config(void)
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
int ensure_ready(void)
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

int set_enabled(int on)
{
	uint32_t chop;

	if (tmc_read(REG_CHOPCONF, &chop))
		return -1;
	chop &= ~CHOP_TOFF_MASK;
	if (on)
		chop |= CHOP_TOFF_ON;
	return tmc_write(REG_CHOPCONF, chop);
}

int get_microsteps(int *usteps)
{
	uint32_t chop;

	if (tmc_read(REG_CHOPCONF, &chop))
		return -1;
	*usteps = 256 >> ((chop & CHOP_MRES_MASK) >> CHOP_MRES_SHIFT);
	return 0;
}

double usteps_per_rev(int usteps)
{
	return (double)cfg.steps_per_rev * usteps;
}

/*
 * Microsteps per joint degree for motor m, with the microstepping read
 * from the driver: steps_per_rev x microsteps x joint_teeth / motor_teeth
 * / 360.  Selects motor m.  Returns 0 on error.
 */
double usteps_per_deg(int m)
{
	int usteps;

	select_motor(m);
	if (cfg.motor_teeth <= 0 || cfg.joint_teeth <= 0) {
		err("motor %d: motor_teeth and joint_teeth must be > 0", m + 1);
		return 0;
	}
	if (get_microsteps(&usteps))
		return 0;
	return usteps_per_rev(usteps) * cfg.joint_teeth / cfg.motor_teeth /
	       360.0;
}

/* ------------------------------------------------------------------ */
/* Velocity mode (VACTUAL): the chip generates the steps itself        */
/* ------------------------------------------------------------------ */

/* VACTUAL unit is usteps per (2^24 / fCLK) seconds */
int32_t rpm_to_vactual(double rpm, int usteps)
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
int ramp_motors(int count, const int *mi, const int32_t *targets)
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

int stop_one(void)
{
	int32_t zero = 0;
	int me = cur;

	return ramp_motors(1, &me, &zero);
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
int request_input(const char *name)
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

int get_line(int fd)
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
int endstop_pressed(const struct config *c, int value)
{
	return !!value == !!c->endstop_nc;
}

struct axis {
	int motor;
	int sign;
	long n, done, err;
	int step_fd, dir_fd;
	int invert_dir;
	uint32_t chop;
	double upr;		/* usteps per revolution */
	double accel;		/* usteps/s^2 */
};

/*
 * Get motor m ready for STEP/DIR: enabled, VACTUAL 0 (or STEP is
 * ignored), and its microstepping into x->upr.  Selects motor m.
 */
static int axis_prepare(struct axis *x, int m)
{
	select_motor(m);
	x->motor = m;
	x->invert_dir = cfg.invert_dir;
	if (ensure_ready())
		return -1;
	if (load_vactual() && stop_one())
		return -1;
	if (tmc_read(REG_CHOPCONF, &x->chop))
		return -1;
	x->upr = usteps_per_rev(256 >>
			((x->chop & CHOP_MRES_MASK) >> CHOP_MRES_SHIFT));
	x->accel = cfg.accel * x->upr;
	return 0;
}

/* Step on rising edges only, and take the STEP and DIR lines (DIR forward or not) */
static int axis_lines(struct axis *x, int forward)
{
	uint32_t chop = x->chop;

	select_motor(x->motor);
	chop &= ~(CHOP_DEDGE | CHOP_TOFF_MASK);
	chop |= CHOP_TOFF_ON;
	if (tmc_write(REG_CHOPCONF, chop))
		return -1;
	x->dir_fd = request_line(cfg.dir_line, !!forward ^ !!x->invert_dir);
	if (x->dir_fd < 0)
		return -1;
	x->step_fd = request_line(cfg.step_line, 0);
	return x->step_fd < 0 ? -1 : 0;
}

static void axis_close(struct axis *x)
{
	if (x->step_fd >= 0)
		close(x->step_fd);
	if (x->dir_fd >= 0)
		close(x->dir_fd);
	x->step_fd = x->dir_fd = -1;
}

/* Real-time priority and locked memory for a stepping loop, and back */
static void rt_enter(void)
{
	struct sched_param sp = { .sched_priority = 50 };

	mlockall(MCL_CURRENT | MCL_FUTURE);
	if (sched_setscheduler(0, SCHED_FIFO, &sp))
		fprintf(stderr, "warning: no realtime priority, motion may "
			"stutter\n");
}

static void rt_leave(void)
{
	struct sched_param sp = { .sched_priority = 0 };

	sched_setscheduler(0, SCHED_OTHER, &sp);
}

/* Sleep until t, then pulse STEP on the axes with fire[k] set */
static void fire_at(struct timespec *t, int count, struct axis *ax,
		    const int *fire)
{
	int k;

	clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, t, NULL);
	for (k = 0; k < count; k++)
		if (fire[k])
			set_line(ax[k].step_fd, 1);
	for (k = 0; k < count; k++)
		if (fire[k])
			set_line(ax[k].step_fd, 0);
}

/* Next pulse dt after t; if we fell far behind, don't burst to catch up */
static void next_after(struct timespec *t, double dt)
{
	struct timespec now;

	ts_add(t, dt);
	clock_gettime(CLOCK_MONOTONIC, &now);
	if (now.tv_sec > t->tv_sec + 1 ||
	    (now.tv_sec - t->tv_sec) * 1e9 + (now.tv_nsec - t->tv_nsec) >
	    2e9 * dt)
		*t = now;
}

/*
 * Exact move of several motors at once over STEP/DIR.  amount[k] is in
 * revolutions (revs != 0) or microsteps for motor mi[k].  The motor with
 * the longest move runs at rpm with the acceleration ramp; the others are
 * spread over it (Bresenham), so all of them start and finish together.
 * With u, the move can also stop on an endstop (see struct until).
 */
int move_axes(int count, const int *mi, const double *amount, int revs,
		     double rpm, struct until *u)
{
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
		long steps;

		if (axis_prepare(x, mi[k]))
			goto out;
		steps = lround(revs ? amount[k] * x->upr : amount[k]);
		x->n = labs(steps);
		x->sign = steps < 0 ? -1 : 1;
		if (!x->n)
			continue;
		if (axis_lines(x, steps > 0))
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

	rt_enter();
	stop_req = 0;
	n = m->n;
	clock_gettime(CLOCK_MONOTONIC, &next);
	ts_add(&next, 20e-6);	/* DIR setup time */
	while (m->done < n) {
		double va = sqrt(v0 * v0 + 2 * a * m->done);
		double vd = sqrt(v0 * v0 + 2 * a * (n - m->done));
		double v = fmin(vmax, fmin(va, vd));
		int fire[MAX_MOTORS];

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
		fire_at(&next, count, ax, fire);
		for (k = 0; k < count; k++)
			ax[k].done += fire[k];
		next_after(&next, 1.0 / v);
	}

	rt_leave();
	ret = 0;
out:
	for (k = 0; k < count; k++) {
		struct axis *x = &ax[k];

		axis_close(x);
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

/*
 * Lowest speed (path units/s) a path may need so no motor goes over
 * MAX_STEP_RATE or its own rpm on any segment.
 */
static double path_cap(int count, const struct axis *ax, int npts,
		       const long *pts, const double *dist)
{
	double cap = INFINITY;
	int i, k;

	for (k = 0; k < count; k++) {
		double rate = fmin(MAX_STEP_RATE,
				   motors[ax[k].motor].rpm / 60.0 * ax[k].upr);

		for (i = 1; i < npts; i++) {
			long d = labs(pts[i * count + k] -
				      pts[(i - 1) * count + k]);

			if (d)
				cap = fmin(cap, rate * (dist[i] - dist[i - 1]) / d);
		}
	}
	return cap;
}

int (*path_stop)(void);

int move_path(int count, const int *mi, int npts, const long *pts,
	      const double *dist, double *speed, double accel,
	      int *interrupted)
{
	struct axis ax[MAX_MOTORS];
	struct timespec next;
	double v0, end;
	int i, k, dir[MAX_MOTORS], soft = 0, ret = -1;

	*interrupted = 0;
	if (*speed <= 0 || accel <= 0 || npts < 2) {
		err("path speed and acceleration must be > 0");
		return -1;
	}
	memset(ax, 0, sizeof(ax));
	for (k = 0; k < count; k++)
		ax[k].step_fd = ax[k].dir_fd = -1;
	for (k = 0; k < count; k++) {
		if (axis_prepare(&ax[k], mi[k]))
			goto out;
		/* DIR of the first segment that moves this motor */
		dir[k] = 1;
		for (i = 1; i < npts; i++) {
			long d = pts[i * count + k] - pts[(i - 1) * count + k];

			if (d) {
				dir[k] = d > 0 ? 1 : -1;
				break;
			}
		}
		if (axis_lines(&ax[k], dir[k] > 0))
			goto out;
	}
	*speed = fmin(*speed, path_cap(count, ax, npts, pts, dist));

	end = dist[npts - 1];
	v0 = fmin(*speed, sqrt(2 * accel * PATH_V0_DIST));

	rt_enter();
	stop_req = 0;
	clock_gettime(CLOCK_MONOTONIC, &next);
	ts_add(&next, 20e-6);	/* DIR setup time */
	for (i = 1; i < npts; i++) {
		long d[MAX_MOTORS], n = 0, j;
		double s0 = dist[i - 1], seg = dist[i] - s0;

		if (s0 >= end)
			break;
		if (!soft && path_stop && path_stop())
			soft = 1;
		for (k = 0; k < count; k++) {
			d[k] = pts[i * count + k] - pts[(i - 1) * count + k];
			ax[k].n = labs(d[k]);
			if (ax[k].n > n)
				n = ax[k].n;
			if (d[k] && (d[k] > 0 ? 1 : -1) != dir[k]) {
				dir[k] = -dir[k];
				set_line(ax[k].dir_fd,
					 (dir[k] > 0) ^ !!ax[k].invert_dir);
				ts_add(&next, 20e-6);
			}
		}
		if (!n) {
			/* Neither motor moves on this segment: just wait */
			double v = fmin(*speed, sqrt(v0 * v0 + 2 * accel * s0));

			next_after(&next, seg / v);
			continue;
		}
		for (k = 0; k < count; k++)
			ax[k].err = n / 2;
		for (j = 0; j < n; j++) {
			double s = s0 + (j + 0.5) * seg / n;
			double v = fmin(*speed,
				   fmin(sqrt(v0 * v0 + 2 * accel * s),
					sqrt(v0 * v0 + 2 * accel *
					     fmax(0, end - s))));
			int fire[MAX_MOTORS];

			if (stop_req || soft == 1) {
				/* Ctrl-C or path_stop: decelerate to a stop */
				end = fmin(end, s + (v * v - v0 * v0) /
					   (2 * accel));
				if (stop_req)
					*interrupted = 1;
				stop_req = 0;
				soft = 2;
			}
			if (s >= end)
				break;
			for (k = 0; k < count; k++) {
				struct axis *x = &ax[k];

				fire[k] = 0;
				x->err += x->n;
				if (x->err >= n) {
					x->err -= n;
					fire[k] = 1;
				}
			}
			fire_at(&next, count, ax, fire);
			for (k = 0; k < count; k++)
				ax[k].done += fire[k] * (d[k] < 0 ? -1 : 1);
			next_after(&next, seg / n / v);
		}
	}
	rt_leave();
	ret = 0;
out:
	for (k = 0; k < count; k++) {
		axis_close(&ax[k]);
		if (!ret)
			pos_add(ax[k].motor, ax[k].done);
	}
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
	double upd;
	int fd, v, ret = -1;

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
	upd = usteps_per_deg(m);
	if (upd <= 0)
		goto out;
	pos_load();
	pos[m] = lround(c->home_angle * upd);
	homed[m] = 1;
	pos_save();
	printf("motor %d: homed, joint at %.1f deg\n", m + 1, c->home_angle);
	ret = 0;
out:
	close(fd);
	return ret;
}

/*
 * Home motors mi[0..n-1] in that order.  Every motor's settings are
 * checked before any of them moves; stops at the first failure.
 */
int home_motors(int n, const int *mi)
{
	int k;

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
