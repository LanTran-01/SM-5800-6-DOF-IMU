/**
 * @file    mock_sm5800_hal.h
 * @brief   Deterministic Mock HAL for host unit testing of sm5800_drv.c.
 */

#ifndef MOCK_SM5800_HAL_H
#define MOCK_SM5800_HAL_H

#include <stdbool.h>
#include <stdint.h>

#define MOCK_REG_COUNT          (32U)

void mock_hal_reset(void);
void mock_hal_set_reg(uint32_t offset_b, uint32_t reg_word);
uint32_t mock_hal_get_reg(uint32_t offset_b);
void mock_hal_set_rst_busy_polls(uint32_t busy_count);
uint32_t mock_hal_get_dsb_count(void);
uint32_t mock_hal_get_dmb_count(void);
uint32_t mock_hal_get_lock_set_count(void);
uint32_t mock_hal_get_lock_clr_count(void);
bool mock_hal_was_snap_locked_on_read(void);

#endif /* MOCK_SM5800_HAL_H */
