// test_pwm_schedule.c — host unit tests for the pure PWM schedule compiler.
//
// Builds against ../main/pwm_schedule.c only (no SDK), like test_protocol.c.
//   cd test && make        # runs this plus the protocol tests

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "pwm_schedule.h"

static int g_fail = 0;

#define CHECK(cond)                                                        \
    do {                                                                   \
        if (!(cond)) {                                                     \
            printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);       \
            g_fail++;                                                      \
        }                                                                  \
    } while (0)

// Total ticks each channel spends HIGH over one full schedule cycle [0, L).
static void high_counts(const pwm_schedule_t *s, uint32_t counts[8]) {
    for (int c = 0; c < 8; c++) counts[c] = 0;
    for (int i = 0; i < s->count; i++) {
        uint32_t start = PWM_EDGE_OFFSET(s->edges[i]);
        uint32_t end = (i + 1 < s->count) ? PWM_EDGE_OFFSET(s->edges[i + 1])
                                          : s->length_ticks;
        uint8_t st = PWM_EDGE_STATE(s->edges[i]);
        for (int c = 0; c < 8; c++) {
            if ((st >> c) & 1) counts[c] += (end - start);
        }
    }
}

// Level of one channel at an arbitrary tick (last entry whose offset <= t).
static int level_at(const pwm_schedule_t *s, uint32_t t, int chan) {
    uint8_t st = PWM_EDGE_STATE(s->edges[0]);
    for (int i = 1; i < s->count; i++) {
        if (PWM_EDGE_OFFSET(s->edges[i]) <= t) st = PWM_EDGE_STATE(s->edges[i]);
        else break;
    }
    return (st >> chan) & 1;
}

// Entries must be strictly increasing in offset, entry[0] at 0, within bounds.
static void check_wellformed(const pwm_schedule_t *s) {
    CHECK(s->count >= 1);
    CHECK(s->count <= PWM_MAX_EDGES);
    CHECK(PWM_EDGE_OFFSET(s->edges[0]) == 0);
    for (int i = 1; i < s->count; i++) {
        CHECK(PWM_EDGE_OFFSET(s->edges[i]) > PWM_EDGE_OFFSET(s->edges[i - 1]));
        CHECK(PWM_EDGE_OFFSET(s->edges[i]) < s->length_ticks);
    }
}

static void test_single_50pct_zero_phase(void) {
    printf("single channel, 50%%, phase 0, n=10\n");
    pwm_chan_req_t reqs[1] = {{.duty = 0.5f, .phase = 0.0f, .period_log2 = 10}};
    pwm_schedule_t s;
    CHECK(pwm_compile(reqs, 1, &s));
    check_wellformed(&s);
    CHECK(s.length_ticks == 1024);
    CHECK(s.count == 2);                       // rise folded into t=0, one fall
    uint32_t hc[8];
    high_counts(&s, hc);
    CHECK(hc[0] == 512);
    CHECK(level_at(&s, 0, 0) == 1);            // on at t=0 (phase 0)
    CHECK(level_at(&s, 511, 0) == 1);
    CHECK(level_at(&s, 512, 0) == 0);
    CHECK(level_at(&s, 1023, 0) == 0);
}

static void test_two_harmonics(void) {
    printf("two channels, n=10 and n=12, both 50%%\n");
    pwm_chan_req_t reqs[2] = {
        {.duty = 0.5f, .phase = 0.0f, .period_log2 = 10},
        {.duty = 0.5f, .phase = 0.0f, .period_log2 = 12},
    };
    pwm_schedule_t s;
    CHECK(pwm_compile(reqs, 2, &s));
    check_wellformed(&s);
    CHECK(s.length_ticks == 4096);
    uint32_t hc[8];
    high_counts(&s, hc);
    CHECK(hc[0] == 512 * 4);                    // 4 repeats of the 1024-tick block
    CHECK(hc[1] == 2048);                       // one 4096-tick block, 50%
}

static void test_phase_wrap(void) {
    printf("phase 0.75 wraps across t=0\n");
    pwm_chan_req_t reqs[1] = {{.duty = 0.5f, .phase = 0.75f, .period_log2 = 10}};
    pwm_schedule_t s;
    CHECK(pwm_compile(reqs, 1, &s));
    check_wellformed(&s);
    uint32_t hc[8];
    high_counts(&s, hc);
    CHECK(hc[0] == 512);
    CHECK(level_at(&s, 0, 0) == 1);            // wrap region covers t=0
    CHECK(level_at(&s, 255, 0) == 1);
    CHECK(level_at(&s, 256, 0) == 0);          // fall_pos = (768+512)%1024 = 256
    CHECK(level_at(&s, 767, 0) == 0);
    CHECK(level_at(&s, 768, 0) == 1);          // rise at on_time = 768
}

static void test_full_on(void) {
    printf("duty 1.0 -> static high, no toggles\n");
    pwm_chan_req_t reqs[1] = {{.duty = 1.0f, .phase = 0.0f, .period_log2 = 11}};
    pwm_schedule_t s;
    CHECK(pwm_compile(reqs, 1, &s));
    check_wellformed(&s);
    CHECK(s.count == 1);
    CHECK(PWM_EDGE_STATE(s.edges[0]) == 0x1);
    uint32_t hc[8];
    high_counts(&s, hc);
    CHECK(hc[0] == s.length_ticks);            // high the entire cycle
}

static void test_dim_rounds_to_off(void) {
    printf("tiny duty rounds to 0 ticks -> off\n");
    // 0.0001 * 1024 = 0.1 tick -> rounds to 0.
    pwm_chan_req_t reqs[1] = {{.duty = 0.0001f, .phase = 0.0f, .period_log2 = 10}};
    pwm_schedule_t s;
    CHECK(pwm_compile(reqs, 1, &s));
    CHECK(s.count == 1);
    CHECK(PWM_EDGE_STATE(s.edges[0]) == 0);    // stays low
}

static void test_min_sharp_pulse(void) {
    printf("one-tick pulse survives (dimmest representable pulse)\n");
    // duty just over 1/1024 rounds to a single 1 us tick high.
    pwm_chan_req_t reqs[1] = {{.duty = 1.0f / 1024.0f, .phase = 0.0f, .period_log2 = 10}};
    pwm_schedule_t s;
    CHECK(pwm_compile(reqs, 1, &s));
    check_wellformed(&s);
    uint32_t hc[8];
    high_counts(&s, hc);
    CHECK(hc[0] == 1);                          // exactly one tick high
}

static void test_all_off(void) {
    printf("all channels off -> single all-low entry\n");
    pwm_chan_req_t reqs[3] = {
        {.duty = 0.0f, .phase = 0.0f, .period_log2 = 10},
        {.duty = 0.0f, .phase = 0.0f, .period_log2 = 12},
        {.duty = 0.0f, .phase = 0.0f, .period_log2 = 11},
    };
    pwm_schedule_t s;
    CHECK(pwm_compile(reqs, 3, &s));
    CHECK(s.count == 1);
    CHECK(PWM_EDGE_STATE(s.edges[0]) == 0);
    CHECK(s.length_ticks == (1u << PWM_DEFAULT_PERIOD_LOG2));
}

static void test_worst_case_size(void) {
    printf("worst-case spread stays within PWM_MAX_EDGES\n");
    // One channel at Nmax, the rest at Nmin — the table blow-up the bound covers.
    pwm_chan_req_t reqs[PWM_MAX_CHANNELS];
    for (int i = 0; i < PWM_MAX_CHANNELS; i++) {
        reqs[i].duty = 0.5f;
        reqs[i].phase = (float)i / (float)PWM_MAX_CHANNELS;  // vary phases
        reqs[i].period_log2 = (i == 0) ? PWM_PERIOD_LOG2_MAX : PWM_PERIOD_LOG2_MIN;
    }
    pwm_schedule_t s;
    CHECK(pwm_compile(reqs, PWM_MAX_CHANNELS, &s));
    check_wellformed(&s);
    CHECK(s.length_ticks == (1u << PWM_PERIOD_LOG2_MAX));
    printf("  (worst-case entries: %u / %u)\n", s.count, (unsigned)PWM_MAX_EDGES);
}

static void test_reconstruction(void) {
    printf("reconstruction: high-tick count == round(duty*block)*reps\n");
    const uint8_t ns[]      = {10, 11, 12, 10, 12};
    const float   duties[]  = {0.10f, 0.33f, 0.80f, 0.5f, 0.017f};
    const float   phases[]  = {0.0f, 0.25f, 0.5f, 0.9f, 0.123f};
    pwm_chan_req_t reqs[5];
    for (int i = 0; i < 5; i++) {
        reqs[i].duty = duties[i];
        reqs[i].phase = phases[i];
        reqs[i].period_log2 = ns[i];
    }
    pwm_schedule_t s;
    CHECK(pwm_compile(reqs, 5, &s));
    check_wellformed(&s);
    uint32_t hc[8];
    high_counts(&s, hc);
    for (int i = 0; i < 5; i++) {
        uint32_t block = 1u << ns[i];
        uint32_t on = (uint32_t)(duties[i] * block + 0.5f);
        uint32_t reps = s.length_ticks / block;
        CHECK(hc[i] == on * reps);
    }
}

int main(void) {
    test_single_50pct_zero_phase();
    test_two_harmonics();
    test_phase_wrap();
    test_full_on();
    test_dim_rounds_to_off();
    test_min_sharp_pulse();
    test_all_off();
    test_worst_case_size();
    test_reconstruction();

    if (g_fail) {
        printf("\n%d check(s) FAILED\n", g_fail);
        return 1;
    }
    printf("\nall pwm_schedule tests passed\n");
    return 0;
}
