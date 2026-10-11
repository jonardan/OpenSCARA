CFLAGS ?= -O2
CFLAGS += -Wall -Wextra

tmc2209ctl: tmc2209ctl.c
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $< -lm

clean:
	rm -f tmc2209ctl
