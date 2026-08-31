// pwm_schedule.h — pure, host-testable PWM schedule compiler.
//
// This is the mathematical core of the custom software PWM engine, with NO SDK
// dependency (only <stdint.h> and config.h macros), so it builds and unit-tests
// on the host exactly like protocol.c.
//
// The engine gives each channel its own duty, phase, and period. Every period is
// a power-of-two number of 200 ns ticks, so all channels are harmonics of one
// another and stack into a single, perfectly repeatable schedule. That schedule
// compiles to one linear edge table the ISR walks:
//
//   - Each block (one channel's square wave) is 2^n ticks long and repeats
//     L / 2^n times inside the schedule, where L = 2^Nmax is the longest block.
//   - An "edge" is a tick at which one or more channels change level. The table
//     stores, per edge, the tick offset and the ABSOLUTE on/off state of every
//     channel (one bit each) — not a delta. The ISR translates that state byte to
//     GPIO set/clear writes via a lookup table built from the channel->GPIO map.
//   - There is always an entry at offset 0 carrying the t=0 state; the table
//     wraps (repeats) at L.
//
// Only pwm_output.c (the SDK/ISR side) knows which GPIO each channel bit maps to;
// this module is deliberately GPIO-agnostic and works purely in channel indices.

#ifndef BULB_PWM_SCHEDULE_H
#define BULB_PWM_SCHEDULE_H

#include <stdbool.h>
#include <stdint.h>

#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

// A channel's state byte packs one bit per channel index (bit i == channel i is
// HIGH). That caps us at 8 channels; the real hardware has PWM_MAX_CHANNELS (5).
_Static_assert(PWM_MAX_CHANNELS <= 8, "PWM state byte holds at most 8 channels");
// Edge offsets pack into the high 24 bits of a uint32_t, and the FRC1 timer load
// register is 23-bit; keeping 2^Nmax <= 2^22 leaves both with headroom.
_Static_assert(PWM_PERIOD_LOG2_MAX <= 22, "period exponent too large for 24-bit offset / 23-bit timer");
_Static_assert(PWM_PERIOD_LOG2_MIN <= PWM_PERIOD_LOG2_MAX, "PWM period-log2 range inverted");

// One engine input, keyed to a physical channel index (its position in the
// engine's channel table). The channel's GPIO is NOT here — see the header note.
typedef struct {
    float   duty;         // [0,1]; exactly 0.0f means "off" (dropped from the schedule)
    float   phase;        // [0,1) fraction of THIS channel's own period
    uint8_t period_log2;  // n; block length = 2^n ticks
} pwm_chan_req_t;

// A compiled edge: high 24 bits = tick offset from schedule start (t=0); low 8
// bits = absolute per-channel on/off state at/after this edge (bit i == channel i
// high). Reproducing the full state every edge makes GPIO writes idempotent and
// glitch-free.
typedef uint32_t pwm_edge_t;
#define PWM_EDGE_MAKE(offset, state) (((uint32_t)(offset) << 8) | ((uint32_t)(state) & 0xFFu))
#define PWM_EDGE_OFFSET(e)           ((uint32_t)(e) >> 8)
#define PWM_EDGE_STATE(e)            ((uint8_t)((e) & 0xFFu))

// Worst-case edge count: one channel at Nmax (longest block, 2 edges) with the
// other (C-1) at Nmin (each repeating 2^(Nmax-Nmin) times, 2 edges per repeat),
// plus the t=0 entry and a little headroom. Buffers auto-size from the config
// range, so widening PWM_PERIOD_LOG2_MIN..MAX visibly grows DRAM use here.
#define PWM_SCHED_SPREAD (PWM_PERIOD_LOG2_MAX - PWM_PERIOD_LOG2_MIN)
#define PWM_MAX_EDGES \
    (2 * (PWM_MAX_CHANNELS - 1) * (1u << PWM_SCHED_SPREAD) + PWM_MAX_CHANNELS + 2)

// A compiled, repeatable schedule: a sorted edge table plus its wrap length.
typedef struct {
    pwm_edge_t edges[PWM_MAX_EDGES];  // edges[0].offset == 0, holds the t=0 state
    uint16_t   count;                 // number of active entries (>= 1)
    uint32_t   length_ticks;          // L = 2^Nmax; the wrap point
} pwm_schedule_t;

// Compile per-channel {duty, phase, period_log2} requests into a schedule.
//   reqs   : one entry per physical channel, indexed 0..nreqs-1 (== the state bit)
//   nreqs  : number of channels (<= PWM_MAX_CHANNELS)
//   out    : filled in on success
// Returns true on success. Returns false only if the (bounded) edge count would
// exceed PWM_MAX_EDGES — unreachable for any n within the configured range, so a
// false is a safety net (the caller keeps the previous schedule). When every
// channel has duty 0.0f, produces a single all-low t=0 entry.
bool pwm_compile(const pwm_chan_req_t *reqs, int nreqs, pwm_schedule_t *out);

#ifdef __cplusplus
}
#endif

#endif  // BULB_PWM_SCHEDULE_H
