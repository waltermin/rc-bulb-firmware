// selftest.h — on-device self-test of the Lemtone path (LEMTONE_SELFTEST builds).
//
// Runs once after boot in its own task and reports on the console (74880 baud):
//   1. the Lemtone reference vectors solved on this chip (correctness + timing),
//   2. 0x07 frames injected through the real RX path (parse, duplicate suppression,
//      keep-alive, fallback) into the controller,
//   3. a visible colour sequence and lumen fade through the same path.
// Needs no base station. The bulb carries on normally afterwards.

#ifndef BULB_SELFTEST_H
#define BULB_SELFTEST_H

#ifdef __cplusplus
extern "C" {
#endif

// Start the self-test task. No-op unless built with LEMTONE_SELFTEST=1. Call after
// controller_start() and sniffer_start().
void selftest_start(void);

#ifdef __cplusplus
}
#endif

#endif  // BULB_SELFTEST_H
