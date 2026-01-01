/*
 * ARM SP804 Dual-Timer Module Implementation
 * 
 * See sp804.h for documentation.
 */

#include "sp804.h"
#include "emu.h"
#include "interrupt.h"
#include "schedule.h"

#include <string.h>

/* PrimeCell ID registers (standard ARM peripheral identification) */
static const uint8_t sp804_primecell_id[] = {
    0x04, 0x18, 0x14, 0x00,  /* PeriphID0-3 */
    0x0D, 0xF0, 0x05, 0xB1   /* PrimeCellID0-3 */
};

/*
 * Update interrupt state for the module.
 * Checks both timers and raises/lowers the interrupt line accordingly.
 */
static void sp804_update_interrupt(sp804_module *module) {
    bool irq = false;
    
    for (int i = 0; i < 2; i++) {
        sp804_timer *t = &module->timers[i];
        /* Interrupt fires if raw status is set AND interrupt enable is set */
        if (t->int_status && (t->control & SP804_CTRL_IE)) {
            irq = true;
            break;
        }
    }
    
    int_set(module->int_num, irq);
}

/*
 * Get prescaler divisor from control register.
 */
static uint32_t sp804_get_prescale_divisor(uint8_t control) {
    switch (control & SP804_CTRL_PRESCALE_MASK) {
        case SP804_CTRL_PRESCALE_1:   return 1;
        case SP804_CTRL_PRESCALE_16:  return 16;
        case SP804_CTRL_PRESCALE_256: return 256;
        default:                      return 1; /* Undefined, treat as /1 */
    }
}

/*
 * Get the counter mask based on timer size (16 or 32 bit).
 */
static uint32_t sp804_get_counter_mask(uint8_t control) {
    return (control & SP804_CTRL_32BIT) ? 0xFFFFFFFF : 0xFFFF;
}

/*
 * Decrement a single timer by one tick (after prescaling).
 * Handles wrapping, periodic reload, and interrupt generation.
 */
static void sp804_decrement_timer(sp804_module *module, sp804_timer *t) {
    uint32_t mask = sp804_get_counter_mask(t->control);
    uint32_t value = t->value & mask;
    
    if (value == 0) {
        /* Timer has wrapped/expired */
        t->int_status = true;
        sp804_update_interrupt(module);
        
        if (t->control & SP804_CTRL_ONESHOT) {
            /* One-shot mode: stop the timer */
            t->control &= ~SP804_CTRL_ENABLE;
        } else if (t->control & SP804_CTRL_PERIODIC) {
            /* Periodic mode: reload from load register */
            t->value = t->load & mask;
        } else {
            /* Free-running mode: wrap to max value */
            t->value = mask;
        }
    } else {
        t->value = (value - 1) & mask;
    }
}

/*
 * Advance a single timer by the given number of input clock ticks.
 */
static void sp804_advance_timer(sp804_module *module, sp804_timer *t, uint32_t ticks) {
    if (!(t->control & SP804_CTRL_ENABLE)) {
        return;  /* Timer disabled */
    }
    
    /* Handle pending load from write to Load register */
    if (t->pending_load) {
        t->pending_load = false;
        t->value = t->load & sp804_get_counter_mask(t->control);
    }
    
    uint32_t prescale_div = sp804_get_prescale_divisor(t->control);
    
    /* Process each input tick through the prescaler */
    for (uint32_t i = 0; i < ticks; i++) {
        t->prescale_count++;
        if (t->prescale_count >= prescale_div) {
            t->prescale_count = 0;
            sp804_decrement_timer(module, t);
        }
    }
}

void sp804_init(sp804_module *module, sp804_clock_source base_clock, 
                bool is_fast, uint8_t int_num) {
    memset(module, 0, sizeof(*module));
    module->base_clock = base_clock;
    module->is_fast_timer = is_fast;
    module->int_num = int_num;
    module->effective_clock = base_clock;
}

void sp804_reset(sp804_module *module) {
    /* Reset both timers to power-on state */
    for (int i = 0; i < 2; i++) {
        sp804_timer *t = &module->timers[i];
        t->load = 0;
        t->value = 0xFFFFFFFF;
        t->bgload = 0;
        t->control = 0x20;  /* Free-running, interrupt enabled, but timer disabled */
        t->prescale_count = 0;
        t->int_status = false;
        t->pending_load = false;
    }
    
    /* Reset fast timer clock control */
    module->fast_clock_ctrl = 0;
    module->effective_clock = module->base_clock;
    
    /* Clear any pending interrupt */
    int_set(module->int_num, false);
}

sp804_clock_source sp804_get_clock_source(const sp804_module *module) {
    if (!module->is_fast_timer) {
        return module->base_clock;
    }
    
    /* Fast timer: check clock control register */
    if (module->fast_clock_ctrl & SP804_FAST_CLK_32K) {
        return SP804_CLK_32K;  /* Bit 1 takes priority */
    } else if (module->fast_clock_ctrl & SP804_FAST_CLK_12MHZ) {
        return SP804_CLK_12MHZ;
    } else {
        return SP804_CLK_CPU_DIV4;  /* Fast mode: CPU/4 */
    }
}

uint32_t sp804_get_clock_rate(sp804_clock_source source) {
    switch (source) {
        case SP804_CLK_CPU_DIV4:
            /* Return CPU clock / 4. Use scheduler's clock rate. */
            return sched.clock_rates[CLOCK_CPU] / 4;
        case SP804_CLK_12MHZ:
            return 12000000;
        case SP804_CLK_32K:
            return 32768;
        default:
            return 32768;
    }
}

uint32_t sp804_read(sp804_module *module, uint32_t offset) {
    /* Avoid slowdown in polling loops */
    cycle_count_delta += 1000;
    
    /* Timer selection: 0x00-0x1F = timer 0, 0x20-0x3F = timer 1 */
    int timer_idx = (offset >> 5) & 1;
    sp804_timer *t = &module->timers[timer_idx];
    
    switch (offset & 0xFFF) {
        case 0x00: case 0x20:  /* Load */
            return t->load;
            
        case 0x04: case 0x24:  /* Value (current counter) */
            return t->value & sp804_get_counter_mask(t->control);
            
        case 0x08: case 0x28:  /* Control */
            return t->control;
            
        case 0x0C: case 0x2C:  /* IntClr - write only, reads as 0 */
            return 0;
            
        case 0x10: case 0x30:  /* RIS - Raw Interrupt Status */
            return t->int_status ? 1 : 0;
            
        case 0x14: case 0x34:  /* MIS - Masked Interrupt Status */
            return (t->int_status && (t->control & SP804_CTRL_IE)) ? 1 : 0;
            
        case 0x18: case 0x38:  /* BGLoad */
            // return t->bgload;
            // arm states bgload is same as load, but hardware returns zero always
            // mimic that behavior
            return 0; 
            
        case 0x1C: case 0x3C:  /* Reserved */
            return 0;
            
        /* Fast timer clock control register */
        case 0x80:
            if (module->is_fast_timer) {
                return module->fast_clock_ctrl;
            }
            return 0;
            
        /* PrimeCell ID registers */
        case 0xFE0: return sp804_primecell_id[0];
        case 0xFE4: return sp804_primecell_id[1];
        case 0xFE8: return sp804_primecell_id[2];
        case 0xFEC: return sp804_primecell_id[3];
        case 0xFF0: return sp804_primecell_id[4];
        case 0xFF4: return sp804_primecell_id[5];
        case 0xFF8: return sp804_primecell_id[6];
        case 0xFFC: return sp804_primecell_id[7];
    }
    
    return 0;  /* Unknown register */
}

void sp804_write(sp804_module *module, uint32_t offset, uint32_t value) {
    /* Timer selection: 0x00-0x1F = timer 0, 0x20-0x3F = timer 1 */
    int timer_idx = (offset >> 5) & 1;
    sp804_timer *t = &module->timers[timer_idx];
    
    switch (offset & 0xFFF) {
        case 0x00: case 0x20:  /* Load */
            t->load = value;
            t->pending_load = true;  /* Will reload on next tick */
            return;
            
        case 0x04: case 0x24:  /* Value - writes ignored (read-only) */
            return;
            
        case 0x08: case 0x28:  /* Control */
            t->control = value & 0xEF;  /* Mask valid bits */
            sp804_update_interrupt(module);
            return;
            
        case 0x0C: case 0x2C:  /* IntClr - any write clears interrupt */
            t->int_status = false;
            sp804_update_interrupt(module);
            return;
            
        case 0x10: case 0x30:  /* RIS - read only */
        case 0x14: case 0x34:  /* MIS - read only */
            return;
            
        case 0x18: case 0x38:  /* BGLoad */
            t->bgload = value;
            t->load = value;  /* Also updates load register, but doesn't trigger reload */
            return;
            
        /* Fast timer clock control register */
        case 0x80:
            if (module->is_fast_timer) {
                module->fast_clock_ctrl = value & 0x03;  /* Only bits 0-1 matter */
                module->effective_clock = sp804_get_clock_source(module);
            }
            return;
    }
    
    /* Unknown register - ignore write */
}

void sp804_tick(sp804_module *module, uint32_t ticks) {
    sp804_advance_timer(module, &module->timers[0], ticks);
    sp804_advance_timer(module, &module->timers[1], ticks);
}

bool sp804_suspend(const sp804_module *module, emu_snapshot *snapshot) {
    /* Save timer state for each timer */
    for (int i = 0; i < 2; i++) {
        const sp804_timer *t = &module->timers[i];
        if (!snapshot_write(snapshot, &t->load, sizeof(t->load)) ||
            !snapshot_write(snapshot, &t->value, sizeof(t->value)) ||
            !snapshot_write(snapshot, &t->bgload, sizeof(t->bgload)) ||
            !snapshot_write(snapshot, &t->control, sizeof(t->control)) ||
            !snapshot_write(snapshot, &t->prescale_count, sizeof(t->prescale_count)) ||
            !snapshot_write(snapshot, &t->int_status, sizeof(t->int_status)) ||
            !snapshot_write(snapshot, &t->pending_load, sizeof(t->pending_load))) {
            return false;
        }
    }
    
    /* Save fast timer state */
    if (!snapshot_write(snapshot, &module->fast_clock_ctrl, sizeof(module->fast_clock_ctrl))) {
        return false;
    }
    
    return true;
}

bool sp804_resume(sp804_module *module, const emu_snapshot *snapshot) {
    /* Restore timer state for each timer */
    for (int i = 0; i < 2; i++) {
        sp804_timer *t = &module->timers[i];
        if (!snapshot_read(snapshot, &t->load, sizeof(t->load)) ||
            !snapshot_read(snapshot, &t->value, sizeof(t->value)) ||
            !snapshot_read(snapshot, &t->bgload, sizeof(t->bgload)) ||
            !snapshot_read(snapshot, &t->control, sizeof(t->control)) ||
            !snapshot_read(snapshot, &t->prescale_count, sizeof(t->prescale_count)) ||
            !snapshot_read(snapshot, &t->int_status, sizeof(t->int_status)) ||
            !snapshot_read(snapshot, &t->pending_load, sizeof(t->pending_load))) {
            return false;
        }
    }
    
    /* Restore fast timer state */
    if (!snapshot_read(snapshot, &module->fast_clock_ctrl, sizeof(module->fast_clock_ctrl))) {
        return false;
    }
    
    /* Recalculate effective clock */
    module->effective_clock = sp804_get_clock_source(module);
    
    return true;
}
