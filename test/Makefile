# Host build of the pure protocol parser test.
#   make        # build and run (both legacy-on and legacy-off configurations)
CC ?= cc
CFLAGS ?= -I../main -Wall -Wextra -O2

# The 0x01 LightUpdate is compile-time gated by PROTO_ENABLE_LEGACY_LIGHT_UPDATE.
# Exercise both settings: legacy on (legacy + new packet tests) and legacy off
# (0x01 rejected + new packet tests).
test_protocol: test_protocol.c ../main/protocol.c ../main/protocol.h
	$(CC) $(CFLAGS) -DPROTO_ENABLE_LEGACY_LIGHT_UPDATE=1 test_protocol.c ../main/protocol.c -o test_protocol_legacy
	./test_protocol_legacy
	$(CC) $(CFLAGS) -DPROTO_ENABLE_LEGACY_LIGHT_UPDATE=0 test_protocol.c ../main/protocol.c -o test_protocol_nolegacy
	./test_protocol_nolegacy

.PHONY: clean
clean:
	rm -f test_protocol test_protocol_legacy test_protocol_nolegacy *.exe *.obj
