/**
 * @file    mock_sm5800_hal.c
 * @brief   Mock HAL simulating SM-5800 AXI4-Lite registers for host unit tests.
 */

#include "mock_sm5800_hal.h"

#include <stdbool.h>
#include <stdint.h>

#include "sm5800_hal.h"
#include "sm5800_regs.h"

static uint32_t g_regs[MOCK_REG_COUNT] = { 0U };
static uint32_t g_rst_busy_polls = 0U;
static uint32_t g_dsb_count = 0U;
static uint32_t g_dmb_count = 0U;
static uint32_t g_lock_set_count = 0U;
static uint32_t g_lock_clr_count = 0U;
static bool g_is_snap_locked = false;
static bool g_all_reads_locked = true;


void mock_hal_reset(void)
{
    uint32_t i = 0U;

    for (i = 0U; i < MOCK_REG_COUNT; i++)
    {
        g_regs[i] = 0U;
    }

    g_regs[SM5800_REG_CORE_ID >> 2U] = SM5800_CORE_ID_EXPECTED;
    g_regs[SM5800_REG_CTRL >> 2U] = SM5800_CTRL_CLK_EN_MASK;
    g_rst_busy_polls = 0U;
    g_dsb_count = 0U;
    g_dmb_count = 0U;
    g_lock_set_count = 0U;
    g_lock_clr_count = 0U;
    g_is_snap_locked = false;
    g_all_reads_locked = true;
}


void mock_hal_set_reg(uint32_t offset_b, uint32_t reg_word)
{
    uint32_t idx = offset_b >> 2U;

    if (idx < MOCK_REG_COUNT)
    {
        g_regs[idx] = reg_word;
    }
    else
    {
        /* Ignore out-of-range test offset */
    }
}


uint32_t mock_hal_get_reg(uint32_t offset_b)
{
    uint32_t idx = offset_b >> 2U;
    uint32_t reg_word = 0U;

    if (idx < MOCK_REG_COUNT)
    {
        reg_word = g_regs[idx];
    }
    else
    {
        reg_word = 0U;
    }

    return reg_word;
}


void mock_hal_set_rst_busy_polls(uint32_t busy_count)
{
    g_rst_busy_polls = busy_count;
}


uint32_t mock_hal_get_dsb_count(void)
{
    return g_dsb_count;
}


uint32_t mock_hal_get_dmb_count(void)
{
    return g_dmb_count;
}


uint32_t mock_hal_get_lock_set_count(void)
{
    return g_lock_set_count;
}


uint32_t mock_hal_get_lock_clr_count(void)
{
    return g_lock_clr_count;
}


bool mock_hal_was_snap_locked_on_read(void)
{
    return g_all_reads_locked;
}


uint32_t sm5800_hal_read_reg(uintptr_t base_addr, uint32_t offset_b)
{
    uint32_t idx = offset_b >> 2U;
    uint32_t reg_word = 0U;

    (void)base_addr;

    if ((offset_b >= SM5800_REG_MC_TIMESTAMP)
        && (offset_b <= SM5800_REG_DELTA_VEL_Z))
    {
        if (!g_is_snap_locked)
        {
            g_all_reads_locked = false;
        }
        else
        {
            /* Expected coherent read while SNAP_LOCK is active */
        }
    }
    else
    {
        /* Non-telemetry register read */
    }

    if (SM5800_REG_STATUS == offset_b)
    {
        if (0U < g_rst_busy_polls)
        {
            g_regs[idx] = g_regs[idx] | SM5800_STS_RST_BUSY_MASK;
            g_rst_busy_polls--;
        }
        else
        {
            g_regs[idx] = g_regs[idx] & (~SM5800_STS_RST_BUSY_MASK);
        }
    }
    else
    {
        /* Normal register read */
    }

    if (idx < MOCK_REG_COUNT)
    {
        reg_word = g_regs[idx];
    }
    else
    {
        reg_word = 0U;
    }

    return reg_word;
}


void sm5800_hal_write_reg(uintptr_t base_addr,
                          uint32_t offset_b,
                          uint32_t reg_word)
{
    uint32_t idx = offset_b >> 2U;

    (void)base_addr;

    if (idx < MOCK_REG_COUNT)
    {
        if (SM5800_REG_INT_STATUS == offset_b)
        {
            /* Hardware W1C behavior */
            g_regs[idx] = g_regs[idx] & (~reg_word);
        }
        else if (SM5800_REG_CTRL == offset_b)
        {
            uint32_t w1p_mask = SM5800_CTRL_RST_PULSE_MASK
                                | SM5800_CTRL_CLR_STATS_MASK
                                | SM5800_CTRL_SOFT_RST_MASK;
            uint32_t sts_idx = SM5800_REG_STATUS >> 2U;

            if (0U != (reg_word & SM5800_CTRL_SNAP_LOCK_MASK))
            {
                g_is_snap_locked = true;
                g_lock_set_count++;
            }
            else if (g_is_snap_locked)
            {
                g_is_snap_locked = false;
                g_lock_clr_count++;
            }
            else
            {
                /* Lock unchanged */
            }

            if (0U != (reg_word & SM5800_CTRL_CMD_BIT_MASK))
            {
                g_regs[sts_idx] = g_regs[sts_idx] | SM5800_STS_BIT_ACT_MASK;
            }
            else
            {
                g_regs[sts_idx] = g_regs[sts_idx] & (~SM5800_STS_BIT_ACT_MASK);
            }

            if (0U != (reg_word & SM5800_CTRL_CLR_STATS_MASK))
            {
                g_regs[SM5800_REG_MSG1_RX_CNT >> 2U] = 0U;
                g_regs[SM5800_REG_MSG2_RX_CNT >> 2U] = 0U;
                g_regs[SM5800_REG_CRC_ERR_CNT >> 2U] = 0U;
                g_regs[SM5800_REG_FRAME_ERR_CNT >> 2U] = 0U;
                g_regs[SM5800_REG_SEQ_ERR_CNT >> 2U] = 0U;
            }
            else
            {
                /* Stats retained */
            }

            g_regs[idx] = reg_word & (~w1p_mask);
        }
        else
        {
            g_regs[idx] = reg_word;
        }
    }
    else
    {
        /* Ignore out-of-range write */
    }
}


void sm5800_hal_dmb(void)
{
    g_dmb_count++;
}


void sm5800_hal_dsb(void)
{
    g_dsb_count++;
}


uint32_t sm5800_hal_enter_crit(void)
{
    return 0U;
}


void sm5800_hal_exit_crit(uint32_t saved_mask)
{
    (void)saved_mask;
}
