/*
 * ARM SP804 Dual-Timer Module Implementation
 * 
 * This implements the ARM PrimeCell SP804 dual-timer with extensions
 * for the TI-Nspire CX/CX II fast timer variant.
 * 
 * Standard SP804 register map (per timer, offset by 0x20 for timer 2):
 *   +0x00: Load       - Timer load value
 *   +0x04: Value      - Current timer value (read-only)
 *   +0x08: Control    - Timer control register
 *   +0x0C: IntClr     - Interrupt clear (write-only)
 *   +0x10: RIS        - Raw interrupt status (read-only)
 *   +0x14: MIS        - Masked interrupt status (read-only)
 *   +0x18: BGLoad     - Background load value
 * 
 * Fast timer extension:
 *   +0x80: ClockCtrl  - Clock source control (fast timer only)
 *         Bit 0: Use 12MHz clock
 *         Bit 1: Use 32768Hz clock (overrides bit 0)
 *         0: Use CPU/4 clock (fast mode)
 */

#ifndef _H_SP804
#define _H_SP804

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* SP804 Control Register bits */
#define SP804_CTRL_ONESHOT      (1 << 0)  /* One-shot mode */
#define SP804_CTRL_32BIT        (1 << 1)  /* 32-bit counter (vs 16-bit) */
#define SP804_CTRL_PRESCALE_MASK (3 << 2) /* Prescaler: 00=/1, 01=/16, 10=/256 */
#define SP804_CTRL_PRESCALE_1   (0 << 2)
#define SP804_CTRL_PRESCALE_16  (1 << 2)
#define SP804_CTRL_PRESCALE_256 (2 << 2)
#define SP804_CTRL_IE           (1 << 5)  /* Interrupt enable */
#define SP804_CTRL_PERIODIC     (1 << 6)  /* Periodic mode (vs free-running) */
#define SP804_CTRL_ENABLE       (1 << 7)  /* Timer enable */

/* Fast timer clock control bits */
#define SP804_FAST_CLK_12MHZ    (1 << 0)  /* Use 12MHz clock */
#define SP804_FAST_CLK_32K      (1 << 1)  /* Use 32768Hz clock (priority) */

/* Clock source enumeration for the timer */
typedef enum sp804_clock_source {
    SP804_CLK_CPU_DIV4,   /* CPU clock / 4 (fast timer default) */
    SP804_CLK_12MHZ,      /* 12 MHz */
    SP804_CLK_32K,        /* 32768 Hz */
} sp804_clock_source;

/*
 * Single timer state within an SP804 module.
 * Each SP804 module contains two of these.
 */
typedef struct sp804_timer {
    /* Registers */
    uint32_t load;        /* Load value register */
    uint32_t value;       /* Current counter value */
    uint32_t bgload;      /* Background load value */
    uint8_t control;      /* Control register */
    
    /* Internal state */
    uint16_t prescale_count; /* Prescaler counter */
    bool int_status;      /* Raw interrupt status */
    bool pending_load;    /* Load pending from write to Load register */
} sp804_timer;

/*
 * SP804 dual-timer module.
 * 
 * Template parameters (set at init time):
 *   - base_clock: The base clock source for this module
 *   - is_fast_timer: Whether this module supports the fast timer extension
 *   - int_num: Interrupt number to raise when timer expires
 */
typedef struct sp804_module {
    sp804_timer timers[2];
    
    /* Configuration (set at init, immutable) */
    sp804_clock_source base_clock;
    bool is_fast_timer;
    uint8_t int_num;      /* Interrupt number for this module */
    
    /* Fast timer extension state */
    uint8_t fast_clock_ctrl;  /* +0x80 register value */
    
    /* Cached effective clock for scheduling */
    sp804_clock_source effective_clock;
} sp804_module;

/*
 * Initialize an SP804 module with the given configuration.
 * 
 * @param module      Pointer to the module to initialize
 * @param base_clock  Default clock source for this module
 * @param is_fast     Whether this module supports fast timer extension
 * @param int_num     Interrupt number to signal on timer expiry
 */
void sp804_init(sp804_module *module, sp804_clock_source base_clock, 
                bool is_fast, uint8_t int_num);

/*
 * Reset an SP804 module to power-on state.
 * 
 * @param module  Pointer to the module to reset
 */
void sp804_reset(sp804_module *module);

/*
 * Read from an SP804 module register.
 * 
 * @param module  Pointer to the module
 * @param offset  Register offset (0x00-0xFF)
 * @return        Register value
 */
uint32_t sp804_read(sp804_module *module, uint32_t offset);

/*
 * Write to an SP804 module register.
 * 
 * @param module  Pointer to the module
 * @param offset  Register offset (0x00-0xFF)
 * @param value   Value to write
 */
void sp804_write(sp804_module *module, uint32_t offset, uint32_t value);

/*
 * Advance the timer state by the given number of ticks.
 * Call this from the scheduler at the appropriate clock rate.
 * 
 * @param module  Pointer to the module
 * @param ticks   Number of clock ticks to advance
 */
void sp804_tick(sp804_module *module, uint32_t ticks);

/*
 * Get the effective clock source for this module.
 * For fast timers, this depends on the fast_clock_ctrl register.
 * 
 * @param module  Pointer to the module
 * @return        The effective clock source
 */
sp804_clock_source sp804_get_clock_source(const sp804_module *module);

/*
 * Get the clock rate in Hz for the given clock source.
 * 
 * @param source  Clock source enumeration
 * @return        Clock frequency in Hz
 */
uint32_t sp804_get_clock_rate(sp804_clock_source source);

/*
 * Suspend/resume support for snapshots.
 */
typedef struct emu_snapshot emu_snapshot;
bool sp804_suspend(const sp804_module *module, emu_snapshot *snapshot);
bool sp804_resume(sp804_module *module, const emu_snapshot *snapshot);

#ifdef __cplusplus
}
#endif

#endif /* _H_SP804 */
