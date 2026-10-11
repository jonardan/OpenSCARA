CFLAGS ?= -O2
CFLAGS += -Wall -Wextra

all: tmc2209ctl scara

tmc2209ctl: tmc2209ctl.c tmc2209.c tmc2209.h
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ tmc2209ctl.c tmc2209.c -lm

scara: scara.c tmc2209.c tmc2209.h kinematics.c kinematics.h line.c line.h
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ scara.c tmc2209.c kinematics.c line.c -lm

# Pure math, run on the PC: make test
test_kinematics: test_kinematics.c kinematics.c kinematics.h
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ test_kinematics.c kinematics.c -lm

test_line: test_line.c line.c line.h kinematics.c kinematics.h
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ test_line.c line.c kinematics.c -lm

test: test_kinematics test_line
	./test_kinematics
	./test_line

clean:
	rm -f tmc2209ctl scara test_kinematics test_line

.PHONY: all clean test
