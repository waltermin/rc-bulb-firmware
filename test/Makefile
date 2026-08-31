# Host build of the pure, SDK-free unit tests.
#   make        # build and run every test
CC ?= cc
CFLAGS ?= -I../main -Wall -Wextra -O2

.PHONY: all
all: test_protocol test_pwm_schedule

# The 0x01 LightUpdate is compile-time gated by PROTO_ENABLE_LEGACY_LIGHT_UPDATE.
# Exercise both settings: legacy on (legacy + new packet tests) and legacy off
# (0x01 rejected + new packet tests).
test_protocol: test_protocol.c ../main/protocol.c ../main/protocol.h
	$(CC) $(CFLAGS) -DPROTO_ENABLE_LEGACY_LIGHT_UPDATE=1 test_protocol.c ../main/protocol.c -o test_protocol_legacy
	./test_protocol_legacy
	$(CC) $(CFLAGS) -DPROTO_ENABLE_LEGACY_LIGHT_UPDATE=0 test_protocol.c ../main/protocol.c -o test_protocol_nolegacy
	./test_protocol_nolegacy

# The PWM schedule compiler is pure integer/float math (no SDK).
test_pwm_schedule: test_pwm_schedule.c ../main/pwm_schedule.c ../main/pwm_schedule.h ../main/config.h
	$(CC) $(CFLAGS) test_pwm_schedule.c ../main/pwm_schedule.c -lm -o test_pwm_schedule_bin
	./test_pwm_schedule_bin

.PHONY: clean
clean:
	rm -f test_protocol test_protocol_legacy test_protocol_nolegacy \
	      test_pwm_schedule test_pwm_schedule_bin *.exe *.obj
