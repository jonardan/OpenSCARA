CFLAGS ?= -O2
CFLAGS += -Wall -Wextra

all: tmc2209ctl scara

tmc2209ctl: tmc2209ctl.c tmc2209.c tmc2209.h
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ tmc2209ctl.c tmc2209.c -lm

scara: scara.c tmc2209.c tmc2209.h
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ scara.c tmc2209.c -lm

clean:
	rm -f tmc2209ctl scara

.PHONY: all clean
