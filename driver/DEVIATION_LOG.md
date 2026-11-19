# MISRA Compliance:2020 Deviation Log — SM-5800 PS Driver

This document records all approved deviations against the unified **MISRA C:2012 + BARR-C:2018 + DO-178A/B/C** C Coding Rules for the SM-5800 AXI4-Lite PS driver (`driver/`).

## Deviation Summary

| ID | File & Function | Rule | Category | Justification | Safety Mitigation |
|---|---|---|---|---|---|
| `DEV-001` | `sm5800_hal.c` (`sm5800_hal_read_reg`, `sm5800_hal_write_reg`) | MISRA C:2012 Rule 11.4 | Advisory | Accessing memory-mapped AXI4-Lite peripheral registers on ARM PS requires converting a `uintptr_t` physical/virtual address into a `volatile uint32_t *` pointer. | Cast is confined strictly to the single HAL module (`sm5800_hal.c`). `base_addr` is validated as non-zero and 4-byte aligned during `sm5800_init()` (PDI check), `offset_b` is always a 4-byte aligned constant from `sm5800_regs.h`, and `SM5800_ASSERT()` checks invariants in debug builds. |
| `DEV-002` | `sm5800_hal.c` (`sm5800_hal_dmb`, `sm5800_hal_dsb`, `sm5800_hal_enter_crit`, `sm5800_hal_exit_crit`) | MISRA C:2012 Dir 4.3, Rule 1.2 | Required / Advisory | Standard ISO C99 provides no portable instructions for ARM architectural memory barriers (`DMB`, `DSB`) or local IRQ masking (`CPSR` `cpsid i` / `cpsie i`) needed to guarantee AXI posted-write completion and ISR/task critical sections. | All assembly instructions are isolated exclusively in the target-specific HAL translation unit (`sm5800_hal.c`), keeping `sm5800_drv.c` 100% pure ISO C99 without inline assembly or `#ifdef`. Note: for ARM Cortex-A53 (AArch64 / Zynq UltraScale+), swap `sm5800_hal.c` at the build-system level with an AArch64 HAL using `dmb sy`, `dsb sy`, `mrs %0, daif`, `msr daifset, #2`, and `msr daifclr, #2`. |

## Recommended Build Command (Section 12 Profile)

```bash
arm-none-eabi-gcc -std=c99 -pedantic-errors -O1 -fno-inline \
    -Wall -Wextra -Werror -Wconversion -Wsign-conversion -Wshadow \
    -Wstrict-prototypes -Wmissing-prototypes -Wundef -Wcast-qual \
    -Wcast-align=strict -Wvla -Wstack-usage=256 -fstack-usage -fno-common \
    -Idriver \
    driver/sm5800_hal.c \
    driver/sm5800_drv.c \
    driver/sm5800_ps_example.c \
    -o sm5800_ps_app.elf
```

> **Note for `-pedantic-errors` on `sm5800_hal.c`:** `__asm__` is permitted by GCC even under `-std=c99 -pedantic-errors` (unlike bare `asm`), so the exact build command above compiles cleanly with zero warning suppression pragmas.
