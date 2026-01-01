// new timer stuff

#ifndef _H_TIMER
#define _H_TIMER

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

struct timer {
    uint16_t ticks;
    uint16_t start_value;     /* Write value of +00 */
    uint16_t value;           /* Read value of +00  */
    uint16_t divider;         /* Value of +04 */
    uint16_t control;         /* Value of +08 */
};

struct timerpair {
    struct timer timers[2];
    uint16_t completion_value[6];
    uint8_t int_mask;
    uint8_t int_status;
};

typedef struct timer_state {
    struct timerpair pairs[3];
} timer_state;

uint32_t timer_read(uint32_t addr);
void timer_write(uint32_t addr, uint32_t value);
void timer_reset(void);

// CX2 timers (implemented using SP804 dual-timer modules)
uint32_t timer_cx_read(uint32_t addr);
void timer_cx_write(uint32_t addr, uint32_t value);
void timer_cx_reset(void);
void timer_cx_fast_tick(uint32_t ticks);

typedef struct emu_snapshot emu_snapshot;

bool timer_suspend(emu_snapshot *snapshot);
bool timer_resume(const emu_snapshot *snapshot);

#ifdef __cplusplus
}
#endif

#endif