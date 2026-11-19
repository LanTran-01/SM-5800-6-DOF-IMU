/**
 * @file    sm5800_hal.c
 * @brief   ARM Cortex-A9 (Zynq-7000 PS) bare-metal HAL for SM-5800 IP core.
 * @req     LLR-SM5800-HAL-001
 * @design  SDD-SM5800-2.2
 */

#include "sm5800_hal.h"

#include <stdint.h>

/* 32-bit word alignment mask for AXI4-Lite MMIO registers */
#define HAL_WORD_ALIGN_MASK     (0x03U)

/* ARM Cortex-A9 CPSR IRQ disable bit (I-bit, bit 7) */
#define HAL_CPSR_IRQ_MASK       (0x00000080U)


uint32_t sm5800_hal_read_reg(uintptr_t base_addr,
                             uint32_t offset_b)
{
    uintptr_t reg_addr = base_addr + (uintptr_t)offset_b;
    uint32_t reg_word = 0U;

    SM5800_ASSERT(0U != base_addr);
    SM5800_ASSERT(0U == (reg_addr & (uintptr_t)HAL_WORD_ALIGN_MASK));

    /* MISRA deviation: M 11.4 - MMIO requires integer-to-pointer cast -
     * base_addr validated at init; offset_b is 4-byte aligned constant. */
    uint32_t const volatile * p_reg = (uint32_t const volatile *)reg_addr;
    reg_word = *p_reg;

    return reg_word;
}


void sm5800_hal_write_reg(uintptr_t base_addr,
                          uint32_t offset_b,
                          uint32_t reg_word)
{
    uintptr_t reg_addr = base_addr + (uintptr_t)offset_b;

    SM5800_ASSERT(0U != base_addr);
    SM5800_ASSERT(0U == (reg_addr & (uintptr_t)HAL_WORD_ALIGN_MASK));

    /* MISRA deviation: M 11.4 - MMIO requires integer-to-pointer cast -
     * base_addr validated at init; offset_b is 4-byte aligned constant. */
    uint32_t volatile * p_reg = (uint32_t volatile *)reg_addr;
    *p_reg = reg_word;
}


void sm5800_hal_dmb(void)
{
    /* MISRA deviation: M Dir 4.3, M 1.2 - ARM architectural DMB barrier -
     * isolated in HAL translation unit for cross-context memory ordering. */
    __asm__ volatile ("dmb" : : : "memory");
}


void sm5800_hal_dsb(void)
{
    /* MISRA deviation: M Dir 4.3, M 1.2 - ARM architectural DSB barrier -
     * isolated in HAL translation unit to drain AXI4-Lite posted writes. */
    __asm__ volatile ("dsb" : : : "memory");
}


uint32_t sm5800_hal_enter_crit(void)
{
    uint32_t cpsr_word = 0U;
    uint32_t result = 0U;

    /* MISRA deviation: M Dir 4.3, M 1.2 - ARM CPSR IRQ mask instruction -
     * isolated in HAL for deterministic short critical sections on PS. */
    __asm__ volatile ("mrs %0, cpsr\n\t"
                      "cpsid i"
                      : "=r" (cpsr_word)
                      :
                      : "memory");

    result = cpsr_word & HAL_CPSR_IRQ_MASK;

    return result;
}


void sm5800_hal_exit_crit(uint32_t saved_mask)
{
    if (0U == (saved_mask & HAL_CPSR_IRQ_MASK))
    {
        /* MISRA deviation: M Dir 4.3, M 1.2 - ARM CPSR IRQ unmask -
         * restores IRQ enable only if IRQs were enabled prior to entry. */
        __asm__ volatile ("cpsie i" : : : "memory");
    }
    else
    {
        /* IRQs were already masked before entry; keep masked for nesting */
    }
}
