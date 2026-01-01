#include "timer.h"

#include <stdio.h>
#include <string.h>
#include <time.h>
#include "emu.h"
#include "interrupt.h"
#include "schedule.h"
#include "mem.h"
#include "sp804.h"

/* 90010000, 900C0000, 900D0000 */
timer_state timer;
#define ADDR_TO_TP(addr) (&timer.pairs[((addr) >> 16) % 5])

uint32_t timer_read(uint32_t addr) {
    struct timerpair *tp = ADDR_TO_TP(addr);
    cycle_count_delta = 0; // Avoid slowdown by fast-forwarding through polling loops
    switch (addr & 0x003F) {
        case 0x00: return tp->timers[0].value;
        case 0x04: return tp->timers[0].divider;
        case 0x08: return tp->timers[0].control;
        case 0x0C: return tp->timers[1].value;
        case 0x10: return tp->timers[1].divider;
        case 0x14: return tp->timers[1].control;
        case 0x18: case 0x1C: case 0x20: case 0x24: case 0x28: case 0x2C:
            return tp->completion_value[((addr & 0x3F) - 0x18) >> 2];
    }
    return bad_read_word(addr);
}
void timer_write(uint32_t addr, uint32_t value) {
    struct timerpair *tp = ADDR_TO_TP(addr);
    switch (addr & 0x003F) {
        case 0x00: tp->timers[0].start_value = tp->timers[0].value = value; return;
        case 0x04: tp->timers[0].divider = value; return;
        case 0x08: tp->timers[0].control = value & 0x1F; return;
        case 0x0C: tp->timers[1].start_value = tp->timers[1].value = value; return;
        case 0x10: tp->timers[1].divider = value; return;
        case 0x14: tp->timers[1].control = value & 0x1F; return;
        case 0x18: case 0x1C: case 0x20: case 0x24: case 0x28: case 0x2C:
            tp->completion_value[((addr & 0x3F) - 0x18) >> 2] = value; return;
        case 0x30: return;
    }
    bad_write_word(addr, value);
}
void timer_int_check(struct timerpair *tp) {
    int_set(INT_TIMER0 + (tp - timer.pairs), tp->int_status & tp->int_mask);
}
void timer_advance(struct timerpair *tp, int ticks) {
    struct timer *t;
    for (t = &tp->timers[0]; t != &tp->timers[2]; t++) {
        int newticks;
        if (t->control & 0x10)
            continue;
        for (newticks = t->ticks + ticks; newticks > t->divider; newticks -= (t->divider + 1)) {
            int compl = t->control & 7;
            t->ticks = 0;

            if (compl == 0 && t->value == 0)
                /* nothing */;
            else if (compl != 0 && compl != 7 && t->value == tp->completion_value[compl - 1])
                t->value = t->start_value;
            else
                t->value += (t->control & 8) ? +1 : -1;

            if (t == &tp->timers[0]) {
                for (compl = 0; compl < 6; compl++) {
                    if (t->value == tp->completion_value[compl]) {
                        tp->int_status |= 1 << compl;
                        timer_int_check(tp);
                    }
                }
            }
        }
        t->ticks = newticks;
    }
}
static void timer_event(int index) {
    // TODO: should use seperate schedule item for each timer,
    //       only fired on significant events
    event_repeat(index, 1);
    timer_advance(&timer.pairs[0], 703);
    timer_advance(&timer.pairs[1], 1);
    timer_advance(&timer.pairs[2], 1);
}
void timer_reset() {
    memset(timer.pairs, 0, sizeof timer.pairs);
    int i;
    for (i = 0; i < 3; i++) {
        timer.pairs[i].timers[0].control = 0x10;
        timer.pairs[i].timers[1].control = 0x10;
    }
    sched.items[SCHED_TIMERS].clock = CLOCK_32K;
    sched.items[SCHED_TIMERS].proc = timer_event;
}



// CX2 timers - implemented using SP804 dual-timer modules

/* SP804 timer modules for CX II:
 *   Timer 0 (0x90010000): Fast timer with configurable clock
 *   Timer 1 (0x900C0000): Standard timer at 12MHz
 *   Timer 2 (0x900D0000): Standard timer at 32768Hz
 */
static sp804_module sp804_timers[3];
static bool sp804_initialized = false;

/* Forward declarations for scheduler event handlers */
static void timer_cx_event_32k(int index);
static void timer_cx_event_12m(int index);

/*
 * Address to module index mapping.
 * 0x90010000 -> 0, 0x900C0000 -> 1, 0x900D0000 -> 2
 */
static inline int timer_cx_addr_to_index(uint32_t addr) {
    return ((addr >> 16) % 5);
}

/*
 * Initialize the SP804 timer modules (called once).
 */
static void timer_cx_init_modules(void) {
    /* Timer 0: Fast timer with CPU/4 default, supports clock switching */
    sp804_init(&sp804_timers[0], SP804_CLK_CPU_DIV4, true, INT_TIMER0);
    
    /* Timer 1: Standard 12MHz timer */
    sp804_init(&sp804_timers[1], SP804_CLK_12MHZ, false, INT_TIMER1);
    
    /* Timer 2: Standard 32768Hz timer */
    sp804_init(&sp804_timers[2], SP804_CLK_32K, false, INT_TIMER2);
    
    sp804_initialized = true;
}

uint32_t timer_cx_read(uint32_t addr) {
    int idx = timer_cx_addr_to_index(addr);
    return sp804_read(&sp804_timers[idx], addr & 0xFFFF);
}

void timer_cx_write(uint32_t addr, uint32_t value) {
    int idx = timer_cx_addr_to_index(addr);
    sp804_write(&sp804_timers[idx], addr & 0xFFFF, value);
}

/*
 * Scheduler event handler for 32kHz-based timers.
 */
static void timer_cx_event_32k(int index) {
    event_repeat(index, 1);
    
    /* Timer 2 always runs at 32kHz */
    sp804_tick(&sp804_timers[2], 1);
    
    /* Timer 0 (fast timer) may run at 32kHz if configured */
    if (sp804_get_clock_source(&sp804_timers[0]) == SP804_CLK_32K) {
        sp804_tick(&sp804_timers[0], 1);
    }
}

/*
 * Scheduler event handler for 12MHz-based timers.
 */
static void timer_cx_event_12m(int index) {
    /* Tick once per scheduler event at 12MHz */
    event_repeat(index, 1);
    
    /* Timer 1 runs at 12MHz */
    sp804_tick(&sp804_timers[1], 1);
    
    /* Timer 0 (fast timer) may run at 12MHz if configured */
    if (sp804_get_clock_source(&sp804_timers[0]) == SP804_CLK_12MHZ) {
        sp804_tick(&sp804_timers[0], 1);
    }
}

/*
 * High-frequency tick for fast timer in CPU/4 mode.
 * Called from CPU execution loop.
 */
void timer_cx_fast_tick(uint32_t ticks) {
    if (!sp804_initialized) return;
    
    /* Only tick if fast timer is in CPU/4 mode */
    if (sp804_get_clock_source(&sp804_timers[0]) == SP804_CLK_CPU_DIV4) {
        sp804_tick(&sp804_timers[0], ticks);
    }
}

void timer_cx_reset(void) {
    if (!sp804_initialized) {
        timer_cx_init_modules();
    }
    
    for (int i = 0; i < 3; i++) {
        sp804_reset(&sp804_timers[i]);
    }
    
    /* Timer 2 and fast timer (in 32kHz mode) use SCHED_TIMERS at 32kHz */
    sched.items[SCHED_TIMERS].clock = CLOCK_32K;
    sched.items[SCHED_TIMERS].proc = timer_cx_event_32k;
    
    /* Timer 1 and fast timer (in 12MHz mode) use SCHED_TIMER_12M at 12MHz */
    sched.items[SCHED_TIMER_12M].clock = CLOCK_12M;
    sched.items[SCHED_TIMER_12M].proc = timer_cx_event_12m;
}

bool timer_suspend(emu_snapshot *snapshot)
{
    /* Save SP804 timer state */
    for (int i = 0; i < 3; i++) {
        if (!sp804_suspend(&sp804_timers[i], snapshot)) {
            return false;
        }
    }
    /* Also save classic timer state for non-CX models */
    return snapshot_write(snapshot, &timer, sizeof(timer));
}

bool timer_resume(const emu_snapshot *snapshot)
{
    /* Restore SP804 timer state */
    for (int i = 0; i < 3; i++) {
        if (!sp804_resume(&sp804_timers[i], snapshot)) {
            return false;
        }
    }
    /* Also restore classic timer state for non-CX models */
    return snapshot_read(snapshot, &timer, sizeof(timer));
}
