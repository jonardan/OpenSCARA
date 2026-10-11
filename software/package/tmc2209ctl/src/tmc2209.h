/* SPDX-License-Identifier: MIT */
/*
 * TMC2209 driver code shared by tmc2209ctl and scara.
 *
 * The configuration file gives one struct config per motor (motors[]);
 * cfg is a copy of the active one, motors[cur], which the driver calls act
 * on (select_motor() changes it).  Functions return 0 on success and -1
 * after printing an error.
 */
#ifndef TMC2209_H
#define TMC2209_H

#include <signal.h>
#include <stdint.h>


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
	double min_angle;	/* safe joint range (deg); both 0 = not */
	double max_angle;	/*   measured yet: scara joint refuses */
	double link1_mm;	/* shoulder axis to elbow axis (shared key) */
	double link2_mm;	/* elbow axis to tool tip (shared key) */
	double line_speed;	/* default tip speed for lines, mm/s (shared) */
	double line_accel;	/* tip acceleration for lines, mm/s^2 (shared) */
};

extern struct config cfg, motors[MAX_MOTORS];
extern int nmotors, cur;
extern volatile sig_atomic_t stop_req;	/* set by on_sigint (Ctrl-C) */

/* Position file (POS_PATH): usteps per motor, and whether it is homed */
extern long pos[MAX_MOTORS];
extern int homed[MAX_MOTORS];

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

void config_init(const char *path);
void select_motor(int i);
const char *tag(void);
void on_sigint(int sig);
void err(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

int tmc_read(uint8_t reg, uint32_t *val);
int tmc_write(uint8_t reg, uint32_t val);

void pos_load(void);
void pos_add(int m, long usteps);
void pos_lost(int m);

int mres_of(int microsteps);
double current_of(int cs, int vsense);
int32_t load_vactual(void);
int set_current(int ma, int hold_pct);
int apply_config(void);
int ensure_ready(void);
int set_enabled(int on);
int get_microsteps(int *usteps);
double usteps_per_rev(int usteps);
double usteps_per_deg(int m);

int32_t rpm_to_vactual(double rpm, int usteps);
int ramp_motors(int count, const int *mi, const int32_t *targets);
int stop_one(void);

int request_input(const char *name);
int get_line(int fd);
int endstop_pressed(const struct config *c, int value);
int move_axes(int count, const int *mi, const double *amount, int revs,
	      double rpm, struct until *u);

/*
 * Stream a path through one stepping loop.  pts[i * count + k] is the
 * absolute position (usteps) of motor mi[k] at point i; pts[0..] must be
 * where the motors are now.  Points are seg apart along the path (any
 * length unit, e.g. mm); the path runs at *speed units/s, accelerating at
 * accel units/s^2 only at its start and end.  *speed is lowered first if a
 * segment would need a motor faster than its rpm (or MAX_STEP_RATE).
 * Ctrl-C decelerates along the path and sets *interrupted.
 */
#define PATH_V0_DIST	0.1	/* start/stop speed: reached after this distance */

int move_path(int count, const int *mi, int npts, const long *pts,
	      double seg, double *speed, double accel, int *interrupted);

int home_motors(int n, const int *mi);

#endif
