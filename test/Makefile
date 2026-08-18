# Host build of the pure protocol parser test.
#   make        # build and run
CC ?= cc
CFLAGS ?= -I../main -Wall -Wextra -O2

test_protocol: test_protocol.c ../main/protocol.c ../main/protocol.h
	$(CC) $(CFLAGS) test_protocol.c ../main/protocol.c -o test_protocol
	./test_protocol

.PHONY: clean
clean:
	rm -f test_protocol test_protocol.exe *.obj
