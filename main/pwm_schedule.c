// pwm_schedule.c — pure PWM schedule compiler (see pwm_schedule.h).
//
// Turns per-channel {duty, phase, period_log2} into one sorted edge table. No SDK
// dependency: integer/float math only, so it unit-tests on the host.

#include "pwm_schedule.h"

#include <math.h>
#include <stdlib.h>

// One raw level-change before sorting/merging: at `offset` ticks, channel `chan`
// takes level `level` (1 = high, 0 = low).
typedef struct {
    uint32_t offset;
    uint8_t  chan;
    uint8_t  level;
} toggle_t;

static int toggle_cmp(const void *a, const void *b) {
    uint32_t oa = ((const toggle_t *)a)->offset;
    uint32_t ob = ((const toggle_t *)b)->offset;
    return (oa > ob) - (oa < ob);
}

static inline float clamp01f(float x) {
    if (x < 0.0f) return 0.0f;
    if (x > 1.0f) return 1.0f;
    return x;
}

bool pwm_compile(const pwm_chan_req_t *reqs, int nreqs, pwm_schedule_t *out) {
    // 1. Find the longest active block (Nmax) among channels with duty > 0.
    uint8_t nmax = 0;
    bool any = false;
    for (int i = 0; i < nreqs; i++) {
        if (reqs[i].duty > 0.0f) {
            any = true;
            if (reqs[i].period_log2 > nmax) nmax = reqs[i].period_log2;
        }
    }
    if (!any) {
        // Everything off: a single t=0 entry with all channels low.
        out->edges[0] = PWM_EDGE_MAKE(0, 0);
        out->count = 1;
        out->length_ticks = 1u << PWM_DEFAULT_PERIOD_LOG2;
        return true;
    }
    const uint32_t L = 1u << nmax;

    // 2. Build the t=0 state and the raw toggle list, channel by channel.
    uint8_t  initial_state = 0;
    toggle_t toggles[PWM_MAX_EDGES];
    int      nt = 0;

    for (int i = 0; i < nreqs; i++) {
        const float d = clamp01f(reqs[i].duty);
        if (d <= 0.0f) continue;  // excluded: state bit stays 0 (driven low)

        const uint32_t block = 1u << reqs[i].period_log2;
        uint32_t on_ticks = (uint32_t)lroundf(d * (float)block);

        if (on_ticks >= block) {          // full on: constant high, no edges
            initial_state |= (uint8_t)(1u << i);
            continue;
        }
        if (on_ticks == 0) {              // rounds to off: stays low, no edges
            continue;
        }

        const uint32_t on_time  = (uint32_t)lroundf(reqs[i].phase * (float)block) % block;
        const uint32_t fall_pos = (on_time + on_ticks) % block;  // in [0, block), != on_time

        // High at t=0 iff (0 - on_time) mod block < on_ticks — i.e. offset 0 lies
        // inside the wrapped [on_time, on_time+on_ticks) high interval.
        const uint32_t phase_at_zero = (block - on_time) % block;
        if (phase_at_zero < on_ticks) {
            initial_state |= (uint8_t)(1u << i);
        }

        // Replicate the block across the schedule. Each repeat contributes a rise
        // (->high) at on_time and a fall (->low) at fall_pos; a toggle landing on
        // offset 0 is already captured by initial_state, so skip it.
        const uint32_t reps = L >> reqs[i].period_log2;
        for (uint32_t k = 0; k < reps; k++) {
            const uint32_t base = k * block;
            uint32_t rise = base + on_time;
            uint32_t fall = base + fall_pos;
            if (rise != 0) {
                if (nt >= (int)PWM_MAX_EDGES) return false;
                toggles[nt++] = (toggle_t){rise, (uint8_t)i, 1};
            }
            if (fall != 0) {
                if (nt >= (int)PWM_MAX_EDGES) return false;
                toggles[nt++] = (toggle_t){fall, (uint8_t)i, 0};
            }
        }
    }

    // 3. Sort toggles by offset; toggles at the same offset merge into one entry.
    qsort(toggles, (size_t)nt, sizeof(toggles[0]), toggle_cmp);

    // 4. Assemble the edge table. entry[0] is always the t=0 state; later entries
    //    carry the absolute state after each offset's toggles are applied. Skip an
    //    offset whose net effect leaves the state unchanged.
    out->edges[0] = PWM_EDGE_MAKE(0, initial_state);
    uint16_t count = 1;
    uint8_t  state = initial_state;
    uint8_t  last_emitted = initial_state;

    int j = 0;
    while (j < nt) {
        const uint32_t off = toggles[j].offset;
        // Apply every toggle at this offset.
        while (j < nt && toggles[j].offset == off) {
            const uint8_t bit = (uint8_t)(1u << toggles[j].chan);
            state = toggles[j].level ? (uint8_t)(state | bit)
                                     : (uint8_t)(state & ~bit);
            j++;
        }
        if (state != last_emitted) {
            if (count >= (uint16_t)PWM_MAX_EDGES) return false;
            out->edges[count++] = PWM_EDGE_MAKE(off, state);
            last_emitted = state;
        }
    }

    out->count = count;
    out->length_ticks = L;
    return true;
}
