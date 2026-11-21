/**
 * @file    sm5800_hal.h
 * @brief   Hardware Abstraction Layer for SM-5800 AXI4-Lite PS driver.
 * @req     LLR-SM5800-HAL-001
 * @design  SDD-SM5800-2.2
 */

#ifndef SM5800_HAL_H
#define SM5800_HAL_H

#include <stdbool.h>
#include <stdint.h>

/* Assertion hook: compiles to ((void)0) in flight/certification builds */
#if defined(SM5800_ENABLE_ASSERT)
/**
 * @brief   Handle internal contract violation in debug builds.
 * @req     LLR-SM5800-HAL-006
 * @design  SDD-SM5800-2.2.6
 * @note    Does not return in debug builds. Not compiled in release builds.
 */
void sm5800_hal_assert_fail(void);

#define SM5800_ASSERT(expr)             \
    do                                  \
    {                                   \
        if (!(expr))                    \
        {                               \
            sm5800_hal_assert_fail();   \
        }                               \
        else                            \
        {                               \
            /* Contract satisfied */    \
        }                               \
    }                                   \
    while (false)
#else
#define SM5800_ASSERT(expr) ((void)0)
#endif /* SM5800_ENABLE_ASSERT */

/**
 * @brief       Read a 32-bit memory-mapped AXI4-Lite register.
 * @req         LLR-SM5800-HAL-001
 * @design      SDD-SM5800-2.2.1
 * @param[in]   base_addr   Base physical/virtual address, 4-byte aligned.
 * @param[in]   offset_b    Register byte offset, 4-byte aligned.
 * @return      32-bit register word read from hardware.
 * @note        ISR-safe. Reentrant. Single volatile access. WCET: O(1).
 */
uint32_t sm5800_hal_read_reg(uintptr_t base_addr,
                             uint32_t offset_b);

/**
 * @brief       Write a 32-bit memory-mapped AXI4-Lite register.
 * @req         LLR-SM5800-HAL-002
 * @design      SDD-SM5800-2.2.2
 * @param[in]   base_addr   Base physical/virtual address, 4-byte aligned.
 * @param[in]   offset_b    Register byte offset, 4-byte aligned.
 * @param[in]   reg_word    32-bit word to write to hardware.
 * @note        ISR-safe. Reentrant. Single volatile access. WCET: O(1).
 */
void sm5800_hal_write_reg(uintptr_t base_addr,
                          uint32_t offset_b,
                          uint32_t reg_word);

/**
 * @brief       Execute a Data Memory Barrier (DMB) instruction.
 * @req         LLR-SM5800-HAL-003
 * @design      SDD-SM5800-2.2.3
 * @note        ISR-safe. Reentrant. Ensures ordering across ISR/main context.
 */
void sm5800_hal_dmb(void);

/**
 * @brief       Execute a Data Synchronization Barrier (DSB) instruction.
 * @req         LLR-SM5800-HAL-004
 * @design      SDD-SM5800-2.2.4
 * @note        ISR-safe. Reentrant. Ensures MMIO writes complete before return.
 */
void sm5800_hal_dsb(void);

/**
 * @brief       Enter an IRQ-masked critical section on the local PS core.
 * @req         LLR-SM5800-HAL-005
 * @design      SDD-SM5800-2.2.5
 * @return      Previous processor interrupt mask state to pass to exit_crit.
 * @note        ISR-safe. Reentrant (nestable via saved mask). WCET: O(1).
 */
uint32_t sm5800_hal_enter_crit(void);

/**
 * @brief       Restore the processor interrupt mask saved by enter_crit.
 * @req         LLR-SM5800-HAL-005
 * @design      SDD-SM5800-2.2.5
 * @param[in]   saved_mask  State word returned by sm5800_hal_enter_crit().
 * @note        ISR-safe. Reentrant. WCET: O(1).
 */
void sm5800_hal_exit_crit(uint32_t saved_mask);

#endif /* SM5800_HAL_H */
