// airtest.h — over-the-air relay/LED test pattern for a finished switch (see airtest.c).

#ifndef AIRTEST_H
#define AIRTEST_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

bool     airtest_start(uint8_t id, uint8_t channel);  // false if already running
bool     airtest_running(void);
void     airtest_stop(void);
uint32_t airtest_duration_ms(void);

// Hold a relay (or LED) bitfield on the air until airtest_hold_off().
void     airtest_hold(bool led, const uint8_t bits[32], uint8_t channel);
void     airtest_hold_off(void);
// Hold any packet body (tag + payload) on the air, in one of 8 slots.
bool     airtest_hold_raw(int slot, const uint8_t *body, size_t len, uint8_t channel);
enum { HOLD_CLEAR_ALL, HOLD_CLEAR_RELAY, HOLD_CLEAR_LED, HOLD_CLEAR_RAW };
void     airtest_hold_clear(int what);
bool     airtest_holding(void);

#endif  // AIRTEST_H
