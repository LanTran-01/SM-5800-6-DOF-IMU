/**
 * @file    sm5800_drv.c
 * @brief   PS driver for SM-5800 6-DOF IMU (100 Hz Msg ID 02 IRQ + Reset/BIT).
 * @req     LLR-SM5800-DRV-001
 * @design  SDD-SM5800-3.0
 */

#include "sm5800_drv.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "sm5800_hal.h"
#include "sm5800_regs.h"

/* Byte-alignment and PDI serialization constants */
#define ALIGN_WORD_MASK         (0x03U)
#define CFG_SERIAL_BYTES        (19U)
#define BITS_PER_BYTE           (8U)
#define BYTE_MASK               (0xFFU)

/* IEEE 802.3 reflected CRC-32 polynomial and preset */
#define CRC32_POLY_REFLECTED    (0xEDB88320U)
#define CRC32_INIT_MASK         (0xFFFFFFFFU)
#define CRC32_FINAL_XOR         (0xFFFFFFFFU)
#define CRC32_LSB_MASK          (0x00000001U)

/* Two's complement decode constants (16-bit, 8-bit, and 32-bit fields) */
#define RAW_S16_MASK            (0x0000FFFFU)
#define RAW_S16_SIGN_BIT        (0x00008000U)
#define RAW_S16_MODULUS         (0x00010000U)
#define RAW_S8_SIGN_BIT         (0x00000080U)
#define RAW_S8_MODULUS          (0x00000100U)
#define RAW_S32_SIGN_BIT        (0x80000000U)
#define RAW_S32_MODULUS         (INT64_C(4294967296))

/* SM-5800 Sec. 5.2.1 & 5.2.2 physical scaling constants (SI units):
 * - Angular Rate : 2^-20 * 600 rad/s            = 5.722045898e-4 rad/s/LSB
 * - Linear Accel : 2^-14 * 600 * 0.3048 m/s^2   = 0.011162109375 m/s^2/LSB
 * - Delta Angle  : 2^-33 rad                    = 1.1641532182693481e-10 rad
 * - Delta Vel    : 2^-27 * 0.3048 m/s           = 2.2709369659423828e-9 m/s
 * - Rad to Deg   : 180 / pi                     = 57.2957795 deg/rad */
#define RATE_LSB_TO_RAD_S       (5.722045898e-4f)
#define ACCEL_LSB_TO_M_S2       (0.011162109375f)
#define DANG_LSB_TO_RAD_F64     (1.1641532182693481e-10)
#define DVEL_LSB_TO_M_S_F64     (2.2709369659423828e-9)
#define RAD_TO_DEG_SCALE        (57.2957795f)

/* Bit shift for packing 5 SW2 faults above 4 SW1 fault bits */
#define SW2_FAULT_OUT_SHIFT     (4U)

/* IRQ sources armed at init: 100 Hz sample, BIT fault, sensor silent */
#define DRV_INT_ENABLE_MASK     (SM5800_INT_MSG2_RDY_MASK  \
                                 | SM5800_INT_IMU_FAULT_MASK \
                                 | SM5800_INT_TIMEOUT_MASK)

/* Fault/diagnostic IRQ bits latched by ISR into isr_fault_flags */
#define DRV_INT_FAULT_MASK      (SM5800_INT_IMU_FAULT_MASK \
                                 | SM5800_INT_RX_ERR_MASK  \
                                 | SM5800_INT_SEQ_GAP_MASK \
                                 | SM5800_INT_TIMEOUT_MASK)

/* Byte shift constants for deterministic LE PDI serialization */
#define SHIFT_BYTE_1            (8U)
#define SHIFT_BYTE_2            (16U)
#define SHIFT_BYTE_3            (24U)
#define SHIFT_BYTE_4            (32U)
#define SHIFT_BYTE_5            (40U)
#define SHIFT_BYTE_6            (48U)
#define SHIFT_BYTE_7            (56U)


/* Private helper prototypes */
static uint32_t compute_crc32_bytes(uint8_t const * p_bytes,
                                    size_t byte_count);
static void pack_u32_le(uint32_t word_in,
                        uint8_t * p_out,
                        size_t offset_b);
static void pack_u64_le(uint64_t dword_in,
                        uint8_t * p_out,
                        size_t offset_b);
static sm5800_status_t validate_config(sm5800_config_t const * p_cfg);
static uint32_t build_ctrl_word(sm5800_config_t const * p_cfg);
static int32_t decode_s16_lsb(uint32_t reg_word);
static int32_t decode_s32_lsb(uint32_t reg_word);
static int16_t decode_s8_temp_degc(uint32_t status_w1);
static uint16_t decode_bit_faults(uint32_t status_w1,
                                  uint32_t status_w2);
static void read_locked_msg2_raw(uintptr_t base_addr,
                                 uint32_t ctrl_shadow,
                                 sm5800_msg2_raw_t * p_raw);
static void convert_raw_to_eng(sm5800_msg2_raw_t const * p_raw,
                               sm5800_msg2_eng_t * p_eng);
static void populate_diag_flags(uint32_t status_word,
                                sm5800_diag_t * p_diag);


/*============================================================================
 * Public Functions
 *==========================================================================*/

sm5800_status_t sm5800_calc_cfg_crc32(sm5800_config_t const * p_cfg,
                                      uint32_t * p_crc32)
{
    sm5800_status_t result = SM5800_OK;

    if ((NULL == p_cfg) || (NULL == p_crc32))
    {
        result = SM5800_ERR_PARAM;
    }
    else
    {
        uint8_t serial_octets[CFG_SERIAL_BYTES] = { 0U };
        uint64_t addr_u64 = (uint64_t)p_cfg->base_addr;

        pack_u32_le(p_cfg->schema_version, serial_octets, 0U);
        pack_u64_le(addr_u64, serial_octets, 4U);
        pack_u32_le(p_cfg->timeout_us, serial_octets, 12U);

        if (p_cfg->is_falling_edge)
        {
            serial_octets[16U] = 1U;
        }
        else
        {
            serial_octets[16U] = 0U;
        }

        if (p_cfg->is_raw_crc_mode)
        {
            serial_octets[17U] = 1U;
        }
        else
        {
            serial_octets[17U] = 0U;
        }

        if (p_cfg->is_msg2_only)
        {
            serial_octets[18U] = 1U;
        }
        else
        {
            serial_octets[18U] = 0U;
        }

        *p_crc32 = compute_crc32_bytes(serial_octets, CFG_SERIAL_BYTES);
    }

    return result;
}


sm5800_status_t sm5800_init(sm5800_drv_t * p_drv,
                            sm5800_config_t const * p_cfg)
{
    sm5800_status_t result = SM5800_OK;

    if ((NULL == p_drv) || (NULL == p_cfg))
    {
        result = SM5800_ERR_PARAM;
    }
    else
    {
        result = validate_config(p_cfg);
    }

    if (SM5800_OK == result)
    {
        uint32_t core_id = sm5800_hal_read_reg(p_cfg->base_addr,
                                               SM5800_REG_CORE_ID);

        if (SM5800_CORE_ID_EXPECTED != core_id)
        {
            uint32_t saved_mask = sm5800_hal_enter_crit();

            p_drv->state = SM5800_STATE_FAULT;
            p_drv->is_msg2_pending = false;
            sm5800_hal_exit_crit(saved_mask);
            result = SM5800_ERR_HW_ID;
        }
        else
        {
            uint32_t ctrl_word = build_ctrl_word(p_cfg);
            uint32_t soft_rst_word = ctrl_word | SM5800_CTRL_SOFT_RST_MASK;
            uint32_t saved_mask = sm5800_hal_enter_crit();

            /* Mask IRQs and mark UNINIT first so re-init is race-free */
            p_drv->state = SM5800_STATE_UNINIT;
            sm5800_hal_write_reg(p_cfg->base_addr, SM5800_REG_INT_ENABLE, 0U);
            sm5800_hal_dsb();

            p_drv->base_addr = p_cfg->base_addr;
            p_drv->ctrl_shadow = ctrl_word;
            p_drv->is_msg2_pending = false;
            p_drv->isr_msg2_count = 0U;
            p_drv->isr_fault_flags = 0U;

            /* Reset parser, apply 100 Hz config, clear pending IRQ flags */
            sm5800_hal_write_reg(p_drv->base_addr,
                                 SM5800_REG_CTRL,
                                 soft_rst_word);
            sm5800_hal_write_reg(p_drv->base_addr,
                                 SM5800_REG_TIMEOUT_CFG,
                                 p_cfg->timeout_us);
            sm5800_hal_write_reg(p_drv->base_addr,
                                 SM5800_REG_INT_STATUS,
                                 SM5800_INT_ALL_MASK);
            sm5800_hal_write_reg(p_drv->base_addr, SM5800_REG_CTRL, ctrl_word);
            sm5800_hal_dsb();

            p_drv->state = SM5800_STATE_READY;
            sm5800_hal_dmb();

            /* Arm 100 Hz Msg 02 IRQ plus IMU fault and link-timeout IRQs */
            sm5800_hal_write_reg(p_drv->base_addr,
                                 SM5800_REG_INT_ENABLE,
                                 DRV_INT_ENABLE_MASK);
            sm5800_hal_dsb();
            sm5800_hal_exit_crit(saved_mask);
        }
    }
    else
    {
        /* Parameter or PDI validation failed; preserve error status */
    }

    return result;
}


sm5800_status_t sm5800_deinit(sm5800_drv_t * p_drv)
{
    sm5800_status_t result = SM5800_OK;

    if (NULL == p_drv)
    {
        result = SM5800_ERR_PARAM;
    }
    else if (SM5800_STATE_READY != p_drv->state)
    {
        result = SM5800_ERR_STATE;
    }
    else
    {
        uint32_t saved_mask = sm5800_hal_enter_crit();

        p_drv->state = SM5800_STATE_UNINIT;
        p_drv->is_msg2_pending = false;
        sm5800_hal_write_reg(p_drv->base_addr, SM5800_REG_INT_ENABLE, 0U);
        sm5800_hal_write_reg(p_drv->base_addr,
                             SM5800_REG_INT_STATUS,
                             SM5800_INT_ALL_MASK);
        sm5800_hal_write_reg(p_drv->base_addr, SM5800_REG_CTRL, 0U);
        sm5800_hal_dsb();

        p_drv->ctrl_shadow = 0U;
        sm5800_hal_exit_crit(saved_mask);
    }

    return result;
}


void sm5800_irq_isr(sm5800_drv_t * p_drv)
{
    if ((NULL != p_drv) && (SM5800_STATE_READY == p_drv->state))
    {
        uint32_t int_flags = sm5800_hal_read_reg(p_drv->base_addr,
                                                 SM5800_REG_INT_STATUS);

        if (0U != int_flags)
        {
            uint32_t aux_bits = int_flags & DRV_INT_FAULT_MASK;

            /* Acknowledge hardware W1C flags and drain AXI posted write */
            sm5800_hal_write_reg(p_drv->base_addr,
                                 SM5800_REG_INT_STATUS,
                                 int_flags);
            sm5800_hal_dsb();

            if (0U != (int_flags & SM5800_INT_MSG2_RDY_MASK))
            {
                uint32_t next_count = p_drv->isr_msg2_count + 1U;
                p_drv->isr_msg2_count = next_count;
                p_drv->is_msg2_pending = true;
            }
            else
            {
                /* Non-Msg2 event; do not signal 100 Hz Msg 02 pending */
            }

            if (0U != aux_bits)
            {
                uint32_t prev_faults = p_drv->isr_fault_flags;
                p_drv->isr_fault_flags = prev_faults | aux_bits;
            }
            else
            {
                /* No auxiliary status bits set */
            }

            sm5800_hal_dmb();
        }
        else
        {
            /* Spurious interrupt with zero active flags: no action */
        }
    }
    else
    {
        /* Guard against NULL or uninitialized handle in ISR context */
    }
}


sm5800_status_t sm5800_take_irq_faults(sm5800_drv_t * p_drv,
                                       uint32_t * p_faults)
{
    sm5800_status_t result = SM5800_OK;

    if ((NULL == p_drv) || (NULL == p_faults))
    {
        result = SM5800_ERR_PARAM;
    }
    else if (SM5800_STATE_READY != p_drv->state)
    {
        result = SM5800_ERR_STATE;
    }
    else
    {
        uint32_t saved_mask = sm5800_hal_enter_crit();

        *p_faults = p_drv->isr_fault_flags;
        p_drv->isr_fault_flags = 0U;
        sm5800_hal_exit_crit(saved_mask);
    }

    return result;
}


bool sm5800_is_msg2_pending(sm5800_drv_t const * p_drv)
{
    bool is_pending = false;

    if ((NULL != p_drv) && (SM5800_STATE_READY == p_drv->state))
    {
        sm5800_hal_dmb();
        is_pending = p_drv->is_msg2_pending;
    }
    else
    {
        is_pending = false;
    }

    return is_pending;
}


sm5800_status_t sm5800_read_msg2_raw(sm5800_drv_t * p_drv,
                                     sm5800_msg2_raw_t * p_raw)
{
    sm5800_status_t result = SM5800_OK;

    if ((NULL == p_drv) || (NULL == p_raw))
    {
        result = SM5800_ERR_PARAM;
    }
    else if (SM5800_STATE_READY != p_drv->state)
    {
        result = SM5800_ERR_STATE;
    }
    else
    {
        uint32_t saved_mask = sm5800_hal_enter_crit();
        bool is_pending = p_drv->is_msg2_pending;

        if (!is_pending)
        {
            sm5800_hal_exit_crit(saved_mask);
            result = SM5800_ERR_NO_SAMPLE;
        }
        else
        {
            p_drv->is_msg2_pending = false;
            read_locked_msg2_raw(p_drv->base_addr,
                                 p_drv->ctrl_shadow,
                                 p_raw);
            sm5800_hal_exit_crit(saved_mask);
            sm5800_hal_dmb();
        }
    }

    return result;
}


sm5800_status_t sm5800_raw_to_eng_msg2(sm5800_msg2_raw_t const * p_raw,
                                       sm5800_msg2_eng_t * p_eng)
{
    sm5800_status_t result = SM5800_OK;

    if ((NULL == p_raw) || (NULL == p_eng))
    {
        result = SM5800_ERR_PARAM;
    }
    else
    {
        convert_raw_to_eng(p_raw, p_eng);
    }

    return result;
}


sm5800_status_t sm5800_read_msg2_eng(sm5800_drv_t * p_drv,
                                     sm5800_msg2_eng_t * p_eng)
{
    sm5800_status_t result = SM5800_OK;

    if ((NULL == p_drv) || (NULL == p_eng))
    {
        result = SM5800_ERR_PARAM;
    }
    else
    {
        sm5800_msg2_raw_t raw_snap =
        {
            .rate_x_lsb      = 0,
            .rate_y_lsb      = 0,
            .rate_z_lsb      = 0,
            .accel_x_lsb     = 0,
            .accel_y_lsb     = 0,
            .accel_z_lsb     = 0,
            .delta_ang_x_lsb = 0,
            .delta_ang_y_lsb = 0,
            .delta_ang_z_lsb = 0,
            .delta_vel_x_lsb = 0,
            .delta_vel_y_lsb = 0,
            .delta_vel_z_lsb = 0,
            .status_word1    = 0U,
            .status_word2    = 0U,
            .timestamp_us    = 0U
        };

        result = sm5800_read_msg2_raw(p_drv, &raw_snap);

        if (SM5800_OK == result)
        {
            convert_raw_to_eng(&raw_snap, p_eng);
        }
        else
        {
            /* Propagate SM5800_ERR_STATE or SM5800_ERR_NO_SAMPLE */
        }
    }

    return result;
}


sm5800_status_t sm5800_trigger_reset(sm5800_drv_t * p_drv,
                                     uint32_t poll_limit)
{
    sm5800_status_t result = SM5800_OK;

    if ((NULL == p_drv) || (poll_limit > SM5800_RESET_POLL_MAX))
    {
        result = SM5800_ERR_PARAM;
    }
    else if (SM5800_STATE_READY != p_drv->state)
    {
        result = SM5800_ERR_STATE;
    }
    else
    {
        uint32_t saved_mask = sm5800_hal_enter_crit();
        uint32_t pulse_word = p_drv->ctrl_shadow | SM5800_CTRL_RST_PULSE_MASK;

        p_drv->is_msg2_pending = false;
        sm5800_hal_write_reg(p_drv->base_addr, SM5800_REG_CTRL, pulse_word);
        sm5800_hal_dsb();
        sm5800_hal_exit_crit(saved_mask);

        if (0U < poll_limit)
        {
            uint32_t remaining = poll_limit;
            uint32_t status_word = sm5800_hal_read_reg(p_drv->base_addr,
                                                       SM5800_REG_STATUS);

            while ((0U != (status_word & SM5800_STS_RST_BUSY_MASK))
                   && (0U < remaining))
            {
                remaining--;
                status_word = sm5800_hal_read_reg(p_drv->base_addr,
                                                  SM5800_REG_STATUS);
            }

            if (0U != (status_word & SM5800_STS_RST_BUSY_MASK))
            {
                result = SM5800_ERR_TIMEOUT;
            }
            else
            {
                result = SM5800_OK;
            }
        }
        else
        {
            /* Non-blocking fire-and-return requested (poll_limit == 0) */
        }
    }

    return result;
}


sm5800_status_t sm5800_hold_reset(sm5800_drv_t * p_drv)
{
    sm5800_status_t result = SM5800_OK;

    if (NULL == p_drv)
    {
        result = SM5800_ERR_PARAM;
    }
    else if (SM5800_STATE_READY != p_drv->state)
    {
        result = SM5800_ERR_STATE;
    }
    else
    {
        uint32_t saved_mask = sm5800_hal_enter_crit();

        p_drv->is_msg2_pending = false;
        p_drv->ctrl_shadow = p_drv->ctrl_shadow | SM5800_CTRL_RST_HOLD_MASK;
        sm5800_hal_write_reg(p_drv->base_addr,
                             SM5800_REG_CTRL,
                             p_drv->ctrl_shadow);
        sm5800_hal_dsb();
        sm5800_hal_exit_crit(saved_mask);
    }

    return result;
}


sm5800_status_t sm5800_release_reset(sm5800_drv_t * p_drv)
{
    sm5800_status_t result = SM5800_OK;

    if (NULL == p_drv)
    {
        result = SM5800_ERR_PARAM;
    }
    else if (SM5800_STATE_READY != p_drv->state)
    {
        result = SM5800_ERR_STATE;
    }
    else
    {
        uint32_t saved_mask = sm5800_hal_enter_crit();

        p_drv->ctrl_shadow = p_drv->ctrl_shadow & (~SM5800_CTRL_RST_HOLD_MASK);
        sm5800_hal_write_reg(p_drv->base_addr,
                             SM5800_REG_CTRL,
                             p_drv->ctrl_shadow);
        sm5800_hal_dsb();
        sm5800_hal_exit_crit(saved_mask);
    }

    return result;
}


sm5800_status_t sm5800_start_cmd_bit(sm5800_drv_t * p_drv)
{
    sm5800_status_t result = SM5800_OK;

    if (NULL == p_drv)
    {
        result = SM5800_ERR_PARAM;
    }
    else if (SM5800_STATE_READY != p_drv->state)
    {
        result = SM5800_ERR_STATE;
    }
    else
    {
        uint32_t saved_mask = sm5800_hal_enter_crit();

        p_drv->is_msg2_pending = false;
        p_drv->ctrl_shadow = p_drv->ctrl_shadow | SM5800_CTRL_CMD_BIT_MASK;
        sm5800_hal_write_reg(p_drv->base_addr,
                             SM5800_REG_CTRL,
                             p_drv->ctrl_shadow);
        sm5800_hal_dsb();
        sm5800_hal_exit_crit(saved_mask);
    }

    return result;
}


sm5800_status_t sm5800_stop_cmd_bit(sm5800_drv_t * p_drv)
{
    sm5800_status_t result = SM5800_OK;

    if (NULL == p_drv)
    {
        result = SM5800_ERR_PARAM;
    }
    else if (SM5800_STATE_READY != p_drv->state)
    {
        result = SM5800_ERR_STATE;
    }
    else
    {
        uint32_t saved_mask = sm5800_hal_enter_crit();

        p_drv->ctrl_shadow = p_drv->ctrl_shadow & (~SM5800_CTRL_CMD_BIT_MASK);
        sm5800_hal_write_reg(p_drv->base_addr,
                             SM5800_REG_CTRL,
                             p_drv->ctrl_shadow);
        sm5800_hal_dsb();
        sm5800_hal_exit_crit(saved_mask);
    }

    return result;
}


bool sm5800_is_reset_busy(sm5800_drv_t const * p_drv)
{
    bool is_busy = false;

    if ((NULL != p_drv) && (SM5800_STATE_READY == p_drv->state))
    {
        uint32_t status_word = sm5800_hal_read_reg(p_drv->base_addr,
                                                   SM5800_REG_STATUS);
        if (0U != (status_word & SM5800_STS_RST_BUSY_MASK))
        {
            is_busy = true;
        }
        else
        {
            is_busy = false;
        }
    }
    else
    {
        is_busy = false;
    }

    return is_busy;
}


bool sm5800_is_cmd_bit_active(sm5800_drv_t const * p_drv)
{
    bool is_active = false;

    if ((NULL != p_drv) && (SM5800_STATE_READY == p_drv->state))
    {
        uint32_t status_word = sm5800_hal_read_reg(p_drv->base_addr,
                                                   SM5800_REG_STATUS);
        if (0U != (status_word & SM5800_STS_BIT_ACT_MASK))
        {
            is_active = true;
        }
        else
        {
            is_active = false;
        }
    }
    else
    {
        is_active = false;
    }

    return is_active;
}


sm5800_status_t sm5800_read_diag(sm5800_drv_t const * p_drv,
                                 sm5800_diag_t * p_diag)
{
    sm5800_status_t result = SM5800_OK;

    if ((NULL == p_drv) || (NULL == p_diag))
    {
        result = SM5800_ERR_PARAM;
    }
    else if (SM5800_STATE_READY != p_drv->state)
    {
        result = SM5800_ERR_STATE;
    }
    else
    {
        uintptr_t base = p_drv->base_addr;
        uint32_t status_word = sm5800_hal_read_reg(base, SM5800_REG_STATUS);
        uint32_t ferr_word = sm5800_hal_read_reg(base,
                                                 SM5800_REG_FRAME_ERR_CNT);
        uint32_t dbg_word = sm5800_hal_read_reg(base, SM5800_REG_DEBUG_CRC);
        uint32_t abort_shifted = (ferr_word & SM5800_FERR_ABORT_MASK)
                                 >> SM5800_FERR_ABORT_SHIFT;

        p_diag->msg1_rx_count = sm5800_hal_read_reg(base,
                                                    SM5800_REG_MSG1_RX_CNT);
        p_diag->msg2_rx_count = sm5800_hal_read_reg(base,
                                                    SM5800_REG_MSG2_RX_CNT);
        p_diag->crc_err_count = sm5800_hal_read_reg(base,
                                                    SM5800_REG_CRC_ERR_CNT);
        p_diag->seq_err_count = sm5800_hal_read_reg(base,
                                                    SM5800_REG_SEQ_ERR_CNT);
        sm5800_hal_dmb();
        p_diag->isr_msg2_count = p_drv->isr_msg2_count;

        p_diag->frame_err_count = (uint16_t)(ferr_word & SM5800_FERR_FMT_MASK);
        p_diag->abort_err_count = (uint16_t)abort_shifted;
        p_diag->last_crc_residue = (uint16_t)(dbg_word
                                              & SM5800_DBG_RESIDUE_MASK);
        populate_diag_flags(status_word, p_diag);
    }

    return result;
}


sm5800_status_t sm5800_clear_diag(sm5800_drv_t * p_drv)
{
    sm5800_status_t result = SM5800_OK;

    if (NULL == p_drv)
    {
        result = SM5800_ERR_PARAM;
    }
    else if (SM5800_STATE_READY != p_drv->state)
    {
        result = SM5800_ERR_STATE;
    }
    else
    {
        uint32_t saved_mask = sm5800_hal_enter_crit();
        uint32_t clear_word = p_drv->ctrl_shadow | SM5800_CTRL_CLR_STATS_MASK;

        p_drv->isr_msg2_count = 0U;
        p_drv->isr_fault_flags = 0U;
        sm5800_hal_write_reg(p_drv->base_addr, SM5800_REG_CTRL, clear_word);
        sm5800_hal_dsb();
        sm5800_hal_exit_crit(saved_mask);
    }

    return result;
}


float sm5800_rad_s_to_deg_s(float rate_rad_s)
{
    float rate_deg_s = rate_rad_s * RAD_TO_DEG_SCALE;

    return rate_deg_s;
}


/*============================================================================
 * Private Functions
 *==========================================================================*/

/**
 * @brief       Compute IEEE 802.3 CRC-32 over a caller-provided byte array.
 * @req         LLR-SM5800-DRV-030
 * @design      SDD-SM5800-3.4.1
 * @param[in]   p_bytes     Input byte array, guaranteed non-NULL by caller.
 * @param[in]   byte_count  Number of bytes, guaranteed > 0 by caller.
 * @return      Finalized 32-bit CRC-32 checksum.
 * @note        ISR-safe. Reentrant. Bounded by byte_count * 8 iterations.
 */
static uint32_t compute_crc32_bytes(uint8_t const * p_bytes,
                                    size_t byte_count)
{
    uint32_t crc = CRC32_INIT_MASK;
    size_t i = 0U;

    SM5800_ASSERT(NULL != p_bytes);
    SM5800_ASSERT(0U < byte_count);

    for (i = 0U; i < byte_count; i++)
    {
        uint32_t bit_idx = 0U;

        crc = crc ^ (uint32_t)p_bytes[i];

        for (bit_idx = 0U; bit_idx < BITS_PER_BYTE; bit_idx++)
        {
            if (0U != (crc & CRC32_LSB_MASK))
            {
                crc = (crc >> 1U) ^ CRC32_POLY_REFLECTED;
            }
            else
            {
                crc = crc >> 1U;
            }
        }
    }

    crc = crc ^ CRC32_FINAL_XOR;

    return crc;
}


/**
 * @brief       Serialize a 32-bit unsigned word little-endian into p_out.
 * @req         LLR-SM5800-DRV-031
 * @design      SDD-SM5800-3.4.2
 * @param[in]   word_in   32-bit word to serialize.
 * @param[out]  p_out     Destination byte array with >= offset_b + 4 capacity.
 * @param[in]   offset_b  Starting byte index in p_out.
 * @note        ISR-safe. Reentrant. Avoids unaligned struct casts (M 11.3).
 */
static void pack_u32_le(uint32_t word_in,
                        uint8_t * p_out,
                        size_t offset_b)
{
    SM5800_ASSERT(NULL != p_out);

    p_out[offset_b]      = (uint8_t)(word_in & BYTE_MASK);
    p_out[offset_b + 1U] = (uint8_t)((word_in >> SHIFT_BYTE_1) & BYTE_MASK);
    p_out[offset_b + 2U] = (uint8_t)((word_in >> SHIFT_BYTE_2) & BYTE_MASK);
    p_out[offset_b + 3U] = (uint8_t)((word_in >> SHIFT_BYTE_3) & BYTE_MASK);
}


/**
 * @brief       Serialize a 64-bit unsigned word little-endian into p_out.
 * @req         LLR-SM5800-DRV-032
 * @design      SDD-SM5800-3.4.3
 * @param[in]   dword_in  64-bit word to serialize.
 * @param[out]  p_out     Destination byte array with >= offset_b + 8 capacity.
 * @param[in]   offset_b  Starting byte index in p_out.
 * @note        ISR-safe. Reentrant. Portable across 32-bit and 64-bit targets.
 */
static void pack_u64_le(uint64_t dword_in,
                        uint8_t * p_out,
                        size_t offset_b)
{
    SM5800_ASSERT(NULL != p_out);

    p_out[offset_b]      = (uint8_t)(dword_in & (uint64_t)BYTE_MASK);
    p_out[offset_b + 1U] = (uint8_t)((dword_in >> SHIFT_BYTE_1)
                                     & (uint64_t)BYTE_MASK);
    p_out[offset_b + 2U] = (uint8_t)((dword_in >> SHIFT_BYTE_2)
                                     & (uint64_t)BYTE_MASK);
    p_out[offset_b + 3U] = (uint8_t)((dword_in >> SHIFT_BYTE_3)
                                     & (uint64_t)BYTE_MASK);
    p_out[offset_b + 4U] = (uint8_t)((dword_in >> SHIFT_BYTE_4)
                                     & (uint64_t)BYTE_MASK);
    p_out[offset_b + 5U] = (uint8_t)((dword_in >> SHIFT_BYTE_5)
                                     & (uint64_t)BYTE_MASK);
    p_out[offset_b + 6U] = (uint8_t)((dword_in >> SHIFT_BYTE_6)
                                     & (uint64_t)BYTE_MASK);
    p_out[offset_b + 7U] = (uint8_t)((dword_in >> SHIFT_BYTE_7)
                                     & (uint64_t)BYTE_MASK);
}


/**
 * @brief       Validate PDI schema version, field bounds, and CRC-32.
 * @req         LLR-SM5800-DRV-033
 * @design      SDD-SM5800-3.4.4
 * @param[in]   p_cfg  Configuration PDI block, guaranteed non-NULL by caller.
 * @return      SM5800_OK, SM5800_ERR_PDI_SCHEMA, SM5800_ERR_PARAM,
 *              SM5800_ERR_PDI_CRC.
 * @note        Reentrant. Enforces DO-178C PDI integrity before HW access.
 */
static sm5800_status_t validate_config(sm5800_config_t const * p_cfg)
{
    sm5800_status_t result = SM5800_OK;
    uintptr_t align_bits = 0U;

    SM5800_ASSERT(NULL != p_cfg);

    align_bits = p_cfg->base_addr & (uintptr_t)ALIGN_WORD_MASK;

    if (SM5800_CFG_SCHEMA_VER != p_cfg->schema_version)
    {
        result = SM5800_ERR_PDI_SCHEMA;
    }
    else if ((0U == p_cfg->base_addr) || (0U != align_bits))
    {
        result = SM5800_ERR_PARAM;
    }
    else if (p_cfg->timeout_us < SM5800_TIMEOUT_MIN_US)
    {
        result = SM5800_ERR_PARAM;
    }
    else if (p_cfg->timeout_us > SM5800_TIMEOUT_MAX_US)
    {
        result = SM5800_ERR_PARAM;
    }
    else
    {
        uint32_t expected_crc = 0U;
        result = sm5800_calc_cfg_crc32(p_cfg, &expected_crc);

        if ((SM5800_OK == result) && (expected_crc != p_cfg->crc32))
        {
            result = SM5800_ERR_PDI_CRC;
        }
        else
        {
            /* PDI schema, bounds, and CRC-32 verified */
        }
    }

    return result;
}


/**
 * @brief       Construct the FPGA CTRL register image from validated config.
 * @req         LLR-SM5800-DRV-034
 * @design      SDD-SM5800-3.4.5
 * @param[in]   p_cfg  Validated PDI configuration, guaranteed non-NULL.
 * @return      32-bit CTRL register bitmask.
 * @note        ISR-safe. Reentrant. Always sets SDLC_CLK_EN = 1.
 */
static uint32_t build_ctrl_word(sm5800_config_t const * p_cfg)
{
    uint32_t ctrl_word = SM5800_CTRL_CLK_EN_MASK;

    SM5800_ASSERT(NULL != p_cfg);

    if (p_cfg->is_falling_edge)
    {
        ctrl_word = ctrl_word | SM5800_CTRL_FALL_EDGE_MASK;
    }
    else
    {
        /* Default rising-edge SDLC sampling */
    }

    if (p_cfg->is_raw_crc_mode)
    {
        ctrl_word = ctrl_word | SM5800_CTRL_CRC_RAW_MASK;
    }
    else
    {
        /* Default Z8530 SDLC inverted FCS (0xF0B8 residue) */
    }

    if (p_cfg->is_msg2_only)
    {
        ctrl_word = ctrl_word | SM5800_CTRL_MSG2_ONLY_MASK;
    }
    else
    {
        /* Both Msg 01 and Msg 02 commit registers */
    }

    return ctrl_word;
}


/**
 * @brief       Decode a 16-bit two's-complement LSB register into int32_t.
 * @req         LLR-SM5800-DRV-035
 * @design      SDD-SM5800-3.4.6
 * @param[in]   reg_word  Raw 32-bit AXI register containing 16-bit signed LSB.
 * @return      Signed count in range -32768..+32767.
 * @note        ISR-safe. Portable C99 without signed bitwise ops or UB casts.
 */
static int32_t decode_s16_lsb(uint32_t reg_word)
{
    uint32_t low_bits = reg_word & RAW_S16_MASK;
    int32_t result = 0;

    if (low_bits >= RAW_S16_SIGN_BIT)
    {
        uint32_t mag_u32 = RAW_S16_MODULUS - low_bits;
        result = -((int32_t)mag_u32);
    }
    else
    {
        result = (int32_t)low_bits;
    }

    return result;
}


/**
 * @brief       Decode a 32-bit two's-complement LSB register into int32_t.
 * @req         LLR-SM5800-DRV-036
 * @design      SDD-SM5800-3.4.7
 * @param[in]   reg_word  Raw 32-bit AXI register containing 32-bit signed LSB.
 * @return      Signed count in range INT32_MIN..INT32_MAX.
 * @note        ISR-safe. Uses int64_t subtraction to handle INT32_MIN cleanly.
 */
static int32_t decode_s32_lsb(uint32_t reg_word)
{
    int32_t result = 0;

    if (reg_word >= RAW_S32_SIGN_BIT)
    {
        int64_t signed_count = (int64_t)reg_word - RAW_S32_MODULUS;
        result = (int32_t)signed_count;
    }
    else
    {
        result = (int32_t)reg_word;
    }

    return result;
}


/**
 * @brief       Extract accelerometer temperature (1 degC/LSB) from Status W1.
 * @req         LLR-SM5800-DRV-037
 * @design      SDD-SM5800-3.4.8
 * @param[in]   status_w1  Raw SM-5800 Status Word 1 register word.
 * @return      Accelerometer temperature in degrees Celsius, -128..+127.
 * @note        ISR-safe. Source: SM-5800 Sec. 5.2.1 Status Word 1 bits [15:8].
 */
static int16_t decode_s8_temp_degc(uint32_t status_w1)
{
    uint32_t temp_bits = (status_w1 & SM5800_SW1_TEMP_MASK)
                         >> SM5800_SW1_TEMP_SHIFT;
    int16_t result = 0;

    if (temp_bits >= RAW_S8_SIGN_BIT)
    {
        uint32_t mag_u32 = RAW_S8_MODULUS - temp_bits;
        result = (int16_t)(-((int32_t)mag_u32));
    }
    else
    {
        result = (int16_t)temp_bits;
    }

    return result;
}


/**
 * @brief       Combine SM-5800 Status Word 1 and 2 BIT fault flags into mask.
 * @req         LLR-SM5800-DRV-038
 * @design      SDD-SM5800-3.4.9
 * @param[in]   status_w1  Raw Status Word 1 register word.
 * @param[in]   status_w2  Raw Status Word 2 register word.
 * @return      9-bit fault bitmask matching SM5800_FLT_* constants.
 * @note        ISR-safe. Reentrant.
 */
static uint16_t decode_bit_faults(uint32_t status_w1,
                                  uint32_t status_w2)
{
    uint32_t sw1_faults = (status_w1 & SM5800_SW1_FAULTS_MASK)
                          >> SM5800_SW1_FAULTS_SHIFT;
    uint32_t sw2_faults = (status_w2 & SM5800_SW2_FAULTS_MASK)
                          >> SM5800_SW2_FAULTS_SHIFT;
    uint32_t combined = sw1_faults | (sw2_faults << SW2_FAULT_OUT_SHIFT);
    uint16_t result = (uint16_t)combined;

    return result;
}


/**
 * @brief       Lock FPGA shadow snapshot and read all 100 Hz Msg 02 raw regs.
 * @req         LLR-SM5800-DRV-039
 * @design      SDD-SM5800-3.4.10
 * @param[in]   base_addr    AXI4-Lite base address, validated by caller.
 * @param[in]   ctrl_shadow  Cached unlocked CTRL register word.
 * @param[out]  p_raw        Destination raw LSB struct, non-NULL by caller.
 * @note        Not ISR-safe. Releases SNAP_LOCK immediately after MMIO reads.
 */
static void read_locked_msg2_raw(uintptr_t base_addr,
                                 uint32_t ctrl_shadow,
                                 sm5800_msg2_raw_t * p_raw)
{
    uint32_t lock_word = ctrl_shadow | SM5800_CTRL_SNAP_LOCK_MASK;
    uint32_t wx_word = 0U;
    uint32_t wy_word = 0U;
    uint32_t wz_word = 0U;
    uint32_t ax_word = 0U;
    uint32_t ay_word = 0U;
    uint32_t az_word = 0U;
    uint32_t dax_word = 0U;
    uint32_t day_word = 0U;
    uint32_t daz_word = 0U;
    uint32_t dvx_word = 0U;
    uint32_t dvy_word = 0U;
    uint32_t dvz_word = 0U;
    uint32_t sw1_word = 0U;
    uint32_t sw2_word = 0U;
    uint32_t sp_ts_us = 0U;

    SM5800_ASSERT(NULL != p_raw);

    /* Freeze coherent FPGA shadow registers before reading Msg 02 fields */
    sm5800_hal_write_reg(base_addr, SM5800_REG_CTRL, lock_word);
    sm5800_hal_dsb();

    wx_word = sm5800_hal_read_reg(base_addr, SM5800_REG_ANG_RATE_X);
    wy_word = sm5800_hal_read_reg(base_addr, SM5800_REG_ANG_RATE_Y);
    wz_word = sm5800_hal_read_reg(base_addr, SM5800_REG_ANG_RATE_Z);
    ax_word = sm5800_hal_read_reg(base_addr, SM5800_REG_LIN_ACC_X);
    ay_word = sm5800_hal_read_reg(base_addr, SM5800_REG_LIN_ACC_Y);
    az_word = sm5800_hal_read_reg(base_addr, SM5800_REG_LIN_ACC_Z);
    dax_word = sm5800_hal_read_reg(base_addr, SM5800_REG_DELTA_ANG_X);
    day_word = sm5800_hal_read_reg(base_addr, SM5800_REG_DELTA_ANG_Y);
    daz_word = sm5800_hal_read_reg(base_addr, SM5800_REG_DELTA_ANG_Z);
    dvx_word = sm5800_hal_read_reg(base_addr, SM5800_REG_DELTA_VEL_X);
    dvy_word = sm5800_hal_read_reg(base_addr, SM5800_REG_DELTA_VEL_Y);
    dvz_word = sm5800_hal_read_reg(base_addr, SM5800_REG_DELTA_VEL_Z);
    sw1_word = sm5800_hal_read_reg(base_addr, SM5800_REG_STATUS_WORD1);
    sw2_word = sm5800_hal_read_reg(base_addr, SM5800_REG_STATUS_WORD2);
    sp_ts_us = sm5800_hal_read_reg(base_addr, SM5800_REG_SP_TIMESTAMP);

    /* Release hardware shadow lock immediately after capturing raw words */
    sm5800_hal_write_reg(base_addr, SM5800_REG_CTRL, ctrl_shadow);
    sm5800_hal_dsb();

    p_raw->rate_x_lsb = decode_s16_lsb(wx_word);
    p_raw->rate_y_lsb = decode_s16_lsb(wy_word);
    p_raw->rate_z_lsb = decode_s16_lsb(wz_word);
    p_raw->accel_x_lsb = decode_s16_lsb(ax_word);
    p_raw->accel_y_lsb = decode_s16_lsb(ay_word);
    p_raw->accel_z_lsb = decode_s16_lsb(az_word);
    p_raw->delta_ang_x_lsb = decode_s32_lsb(dax_word);
    p_raw->delta_ang_y_lsb = decode_s32_lsb(day_word);
    p_raw->delta_ang_z_lsb = decode_s32_lsb(daz_word);
    p_raw->delta_vel_x_lsb = decode_s32_lsb(dvx_word);
    p_raw->delta_vel_y_lsb = decode_s32_lsb(dvy_word);
    p_raw->delta_vel_z_lsb = decode_s32_lsb(dvz_word);
    p_raw->status_word1 = (uint16_t)(sw1_word & RAW_S16_MASK);
    p_raw->status_word2 = (uint16_t)(sw2_word & RAW_S16_MASK);
    p_raw->timestamp_us = sp_ts_us;
}


/**
 * @brief       Convert raw Msg 02 LSB counts and status words to SI units.
 * @req         LLR-SM5800-DRV-040
 * @design      SDD-SM5800-3.4.11
 * @param[in]   p_raw  Raw Message ID 02 snapshot, guaranteed non-NULL.
 * @param[out]  p_eng  Human-readable SI output struct, guaranteed non-NULL.
 * @note        Not ISR-safe (uses float). Scales per SM-5800 Sec. 5.2.1/5.2.2.
 */
static void convert_raw_to_eng(sm5800_msg2_raw_t const * p_raw,
                               sm5800_msg2_eng_t * p_eng)
{
    uint32_t sw1_u32 = 0U;
    uint32_t sw2_u32 = 0U;

    SM5800_ASSERT(NULL != p_raw);
    SM5800_ASSERT(NULL != p_eng);

    sw1_u32 = (uint32_t)p_raw->status_word1;
    sw2_u32 = (uint32_t)p_raw->status_word2;

    /* Scale raw counts into SI units (rad/s, m/s^2, rad, m/s); 32-bit
     * Spatial deltas use 64-bit double intermediate math to preserve all
     * 31 magnitude bits before rounding once to float */
    p_eng->rate_x_rad_s = (float)p_raw->rate_x_lsb * RATE_LSB_TO_RAD_S;
    p_eng->rate_y_rad_s = (float)p_raw->rate_y_lsb * RATE_LSB_TO_RAD_S;
    p_eng->rate_z_rad_s = (float)p_raw->rate_z_lsb * RATE_LSB_TO_RAD_S;
    p_eng->accel_x_m_s2 = (float)p_raw->accel_x_lsb * ACCEL_LSB_TO_M_S2;
    p_eng->accel_y_m_s2 = (float)p_raw->accel_y_lsb * ACCEL_LSB_TO_M_S2;
    p_eng->accel_z_m_s2 = (float)p_raw->accel_z_lsb * ACCEL_LSB_TO_M_S2;
    p_eng->delta_ang_x_rad = (float)((double)p_raw->delta_ang_x_lsb
                                     * DANG_LSB_TO_RAD_F64);
    p_eng->delta_ang_y_rad = (float)((double)p_raw->delta_ang_y_lsb
                                     * DANG_LSB_TO_RAD_F64);
    p_eng->delta_ang_z_rad = (float)((double)p_raw->delta_ang_z_lsb
                                     * DANG_LSB_TO_RAD_F64);
    p_eng->delta_vel_x_m_s = (float)((double)p_raw->delta_vel_x_lsb
                                     * DVEL_LSB_TO_M_S_F64);
    p_eng->delta_vel_y_m_s = (float)((double)p_raw->delta_vel_y_lsb
                                     * DVEL_LSB_TO_M_S_F64);
    p_eng->delta_vel_z_m_s = (float)((double)p_raw->delta_vel_z_lsb
                                     * DVEL_LSB_TO_M_S_F64);

    p_eng->accel_temp_degc = decode_s8_temp_degc(sw1_u32);
    p_eng->bit_fault_mask = decode_bit_faults(sw1_u32, sw2_u32);
    p_eng->seq_counter = (uint8_t)(sw1_u32 & SM5800_SW1_SEQ_MASK);
    p_eng->sw_version = (uint8_t)(sw2_u32 & SM5800_SW2_VER_MASK);
    p_eng->timestamp_us = p_raw->timestamp_us;

    if (0U != (sw1_u32 & SM5800_SW1_MOD_FAIL_MASK))
    {
        p_eng->has_module_failed = true;
    }
    else
    {
        p_eng->has_module_failed = false;
    }
}


/**
 * @brief       Decode STATUS register bitfields into diagnostics struct.
 * @req         LLR-SM5800-DRV-041
 * @design      SDD-SM5800-3.4.12
 * @param[in]   status_word  Raw 32-bit STATUS register word.
 * @param[out]  p_diag       Diagnostics output struct, non-NULL by caller.
 * @note        ISR-safe. Reentrant. Avoids boolean-assigned compound ops.
 */
static void populate_diag_flags(uint32_t status_word,
                                sm5800_diag_t * p_diag)
{
    uint32_t id_bits = (status_word & SM5800_STS_LAST_ID_MASK)
                       >> SM5800_STS_LAST_ID_SHIFT;
    uint32_t err_bits = (status_word & SM5800_STS_LAST_ERR_MASK)
                        >> SM5800_STS_LAST_ERR_SHIFT;

    SM5800_ASSERT(NULL != p_diag);

    p_diag->last_msg_id = (uint8_t)id_bits;
    p_diag->last_err_mask = (uint8_t)err_bits;

    if (0U != (status_word & SM5800_STS_FLAGS_OK_MASK))
    {
        p_diag->is_flags_alive = true;
    }
    else
    {
        p_diag->is_flags_alive = false;
    }

    if (0U != (status_word & SM5800_STS_STREAM_OK_MASK))
    {
        p_diag->is_stream_active = true;
    }
    else
    {
        p_diag->is_stream_active = false;
    }

    if (0U != (status_word & SM5800_STS_IMU_FAULT_MASK))
    {
        p_diag->has_imu_fault = true;
    }
    else
    {
        p_diag->has_imu_fault = false;
    }

    if (0U != (status_word & SM5800_STS_MOD_FAIL_MASK))
    {
        p_diag->has_module_failed = true;
    }
    else
    {
        p_diag->has_module_failed = false;
    }
}
