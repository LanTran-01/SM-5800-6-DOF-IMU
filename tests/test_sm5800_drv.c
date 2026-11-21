/**
 * @file    test_sm5800_drv.c
 * @brief   Host C99 unit test suite covering TC-DRV-01 through TC-DRV-16.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "mock_sm5800_hal.h"
#include "sm5800_drv.h"
#include "sm5800_regs.h"

#define TEST_BASE_ADDR          ((uintptr_t)0x43C00000U)

static uint32_t g_pass_count = 0U;
static uint32_t g_fail_count = 0U;


static void check_cond(bool is_ok, char const * p_label)
{
    if (is_ok)
    {
        g_pass_count++;
    }
    else
    {
        g_fail_count++;
        (void)printf("[FAIL] %s\n", p_label);
    }
}


static bool is_near_f(float actual, float target, float eps)
{
    float diff = actual - target;
    bool is_ok = false;

    if (diff < 0.0f)
    {
        diff = -diff;
    }
    else
    {
        /* diff already non-negative */
    }

    if (diff <= eps)
    {
        is_ok = true;
    }
    else
    {
        is_ok = false;
    }

    return is_ok;
}


static sm5800_config_t make_valid_cfg(void)
{
    sm5800_config_t cfg =
    {
        .schema_version  = SM5800_CFG_SCHEMA_VER,
        .base_addr       = TEST_BASE_ADDR,
        .timeout_us      = SM5800_TIMEOUT_DFLT_US,
        .is_falling_edge = false,
        .is_raw_crc_mode = false,
        .is_msg2_only    = true,
        .crc32           = 0U
    };
    uint32_t crc = 0U;

    (void)sm5800_calc_cfg_crc32(&cfg, &crc);
    cfg.crc32 = crc;

    return cfg;
}


static void test_pdi_and_init(void)
{
    sm5800_drv_t drv = { .state = SM5800_STATE_UNINIT };
    sm5800_config_t cfg = make_valid_cfg();
    uint32_t crc = 0U;
    sm5800_status_t st = SM5800_OK;

    /* TC-DRV-01: NULL parameter checks on CRC */
    st = sm5800_calc_cfg_crc32(NULL, &crc);
    check_cond(SM5800_ERR_PARAM == st, "TC-DRV-01: calc_crc NULL cfg");
    st = sm5800_calc_cfg_crc32(&cfg, NULL);
    check_cond(SM5800_ERR_PARAM == st, "TC-DRV-01: calc_crc NULL out");

    /* TC-DRV-02: PDI validation branches */
    mock_hal_reset();
    cfg.schema_version = 0x12345678U;
    check_cond(SM5800_ERR_PDI_SCHEMA == sm5800_init(&drv, &cfg),
               "TC-DRV-02: bad schema");

    cfg = make_valid_cfg();
    cfg.base_addr = (uintptr_t)0x43C00002U;
    (void)sm5800_calc_cfg_crc32(&cfg, &cfg.crc32);
    check_cond(SM5800_ERR_PARAM == sm5800_init(&drv, &cfg),
               "TC-DRV-02: unaligned base_addr");

    cfg = make_valid_cfg();
    cfg.timeout_us = 1000U;
    (void)sm5800_calc_cfg_crc32(&cfg, &cfg.crc32);
    check_cond(SM5800_ERR_PARAM == sm5800_init(&drv, &cfg),
               "TC-DRV-02: timeout below min");

    cfg = make_valid_cfg();
    cfg.timeout_us = 2000000U;
    (void)sm5800_calc_cfg_crc32(&cfg, &cfg.crc32);
    check_cond(SM5800_ERR_PARAM == sm5800_init(&drv, &cfg),
               "TC-DRV-02: timeout above max");

    cfg = make_valid_cfg();
    cfg.crc32 = cfg.crc32 ^ 0x00000001U;
    check_cond(SM5800_ERR_PDI_CRC == sm5800_init(&drv, &cfg),
               "TC-DRV-02: bad PDI CRC-32");

    /* TC-DRV-03: Wrong CORE_ID vs valid init */
    cfg = make_valid_cfg();
    mock_hal_set_reg(SM5800_REG_CORE_ID, 0xDEADBEEFU);
    check_cond(SM5800_ERR_HW_ID == sm5800_init(&drv, &cfg),
               "TC-DRV-03: bad CORE_ID");
    check_cond(SM5800_STATE_FAULT == drv.state,
               "TC-DRV-03: state transitions to FAULT");

    mock_hal_reset();
    check_cond(SM5800_OK == sm5800_init(&drv, &cfg),
               "TC-DRV-03: nominal 100Hz init");
    check_cond((SM5800_INT_MSG2_RDY_MASK | SM5800_INT_IMU_FAULT_MASK
                | SM5800_INT_TIMEOUT_MASK)
               == mock_hal_get_reg(SM5800_REG_INT_ENABLE),
               "TC-DRV-03: INT_ENABLE == MSG2 | IMU_FAULT | TIMEOUT");
    check_cond(0U != (mock_hal_get_reg(SM5800_REG_CTRL)
                      & SM5800_CTRL_MSG2_ONLY_MASK),
               "TC-DRV-03: CTRL.MSG2_ONLY_MODE set");

    /* TC-DRV-04: SEU bit-flip in state & clean deinit */
    drv.state = (sm5800_state_t)((uint32_t)SM5800_STATE_READY ^ 0x01U);
    check_cond(SM5800_ERR_STATE == sm5800_deinit(&drv),
               "TC-DRV-04: SEU bit-flip rejected");
    drv.state = SM5800_STATE_READY;
    check_cond(SM5800_OK == sm5800_deinit(&drv),
               "TC-DRV-04: nominal deinit");
    check_cond(SM5800_STATE_UNINIT == drv.state,
               "TC-DRV-04: state == UNINIT after deinit");
}


static void test_isr_read_and_convert(void)
{
    sm5800_drv_t drv = { .state = SM5800_STATE_UNINIT };
    sm5800_config_t cfg = make_valid_cfg();
    sm5800_msg2_raw_t raw = { .timestamp_us = 0U };
    sm5800_msg2_eng_t eng = { .timestamp_us = 0U };

    mock_hal_reset();
    (void)sm5800_init(&drv, &cfg);

    /* TC-DRV-05 & TC-DRV-06: ISR safety, W1C, and pending flag */
    sm5800_irq_isr(NULL);
    check_cond(!sm5800_is_msg2_pending(&drv),
               "TC-DRV-05: not pending before IRQ");
    check_cond(SM5800_ERR_NO_SAMPLE == sm5800_read_msg2_raw(&drv, &raw),
               "TC-DRV-07: ERR_NO_SAMPLE when idle");

    /* Populate Msg 02 boundary negative values: -32768, -1, INT32_MIN */
    mock_hal_set_reg(SM5800_REG_ANG_RATE_X, 0xFFFF8000U);
    mock_hal_set_reg(SM5800_REG_ANG_RATE_Y, 0xFFFFFFFFU);
    mock_hal_set_reg(SM5800_REG_ANG_RATE_Z, 0x000006D4U);
    mock_hal_set_reg(SM5800_REG_LIN_ACC_X, 0x0000036FU);
    mock_hal_set_reg(SM5800_REG_LIN_ACC_Y, 0xFFFFFC91U);
    mock_hal_set_reg(SM5800_REG_LIN_ACC_Z, 0x0000036FU);
    mock_hal_set_reg(SM5800_REG_DELTA_ANG_X, 0x80000000U);
    mock_hal_set_reg(SM5800_REG_DELTA_ANG_Y, 0x0083126FU);
    mock_hal_set_reg(SM5800_REG_DELTA_ANG_Z, 0xFFFFFFFFU);
    mock_hal_set_reg(SM5800_REG_DELTA_VEL_X, 0x0041F372U);
    mock_hal_set_reg(SM5800_REG_DELTA_VEL_Y, 0x00000000U);
    /* 1 g x 10 ms = 0.0980665 m/s -> 43183277 LSB (2^-27 ft/s per LSB) */
    mock_hal_set_reg(SM5800_REG_DELTA_VEL_Z, 0x0292ECADU);
    mock_hal_set_reg(SM5800_REG_STATUS_WORD1, 0xF6F5U);
    mock_hal_set_reg(SM5800_REG_STATUS_WORD2, 0xF807U);
    mock_hal_set_reg(SM5800_REG_SP_TIMESTAMP, 123456U);

    /* Fire 100 Hz Msg 02 interrupt */
    mock_hal_set_reg(SM5800_REG_INT_STATUS, SM5800_INT_MSG2_RDY_MASK);
    sm5800_irq_isr(&drv);
    check_cond(0U == mock_hal_get_reg(SM5800_REG_INT_STATUS),
               "TC-DRV-05: ISR cleared INT_STATUS W1C");
    check_cond(sm5800_is_msg2_pending(&drv),
               "TC-DRV-05: pending == true after Msg 02 IRQ");

    /* TC-DRV-07: Atomic raw read & two's complement boundaries */
    check_cond(SM5800_OK == sm5800_read_msg2_raw(&drv, &raw),
               "TC-DRV-07: read_msg2_raw OK");
    check_cond(mock_hal_was_snap_locked_on_read(),
               "TC-DRV-07: all telemetry reads held SNAP_LOCK");
    check_cond(raw.rate_x_lsb == -32768, "TC-DRV-07: -32768 s16 decode");
    check_cond(raw.rate_y_lsb == -1, "TC-DRV-07: -1 s16 decode");
    check_cond(raw.delta_ang_x_lsb == (-2147483647 - 1),
               "TC-DRV-07: INT32_MIN s32 decode");

    /* TC-DRV-08: Direct raw-to-engineering conversion check */
    check_cond(SM5800_OK == sm5800_raw_to_eng_msg2(&raw, &eng),
               "TC-DRV-08: raw_to_eng_msg2 OK");
    check_cond(is_near_f(eng.rate_z_rad_s, 1.0002f, 0.002f),
               "TC-DRV-08: 1748 LSB -> 1.0002 rad/s");
    check_cond(is_near_f(eng.accel_z_m_s2, 9.8115f, 0.01f),
               "TC-DRV-08: 879 LSB -> ~9.81 m/s^2 (1g)");
    check_cond(is_near_f(eng.delta_ang_y_rad, 0.0010f, 0.00001f),
               "TC-DRV-08: 8589935 LSB -> 0.001 rad");
    check_cond(is_near_f(eng.delta_vel_z_m_s, 0.0980665f, 0.00001f),
               "TC-DRV-08: 43183277 LSB -> 0.09807 m/s");
    check_cond(eng.accel_temp_degc == -10,
               "TC-DRV-08: 0xF6 -> -10 degC");
    check_cond(eng.bit_fault_mask == 0x01FFU,
               "TC-DRV-08: all 9 BIT fault bits decoded");
    check_cond(eng.has_module_failed,
               "TC-DRV-08: has_module_failed decoded");
    check_cond(eng.seq_counter == 5U, "TC-DRV-08: seq_counter == 5");
    check_cond(eng.sw_version == 7U, "TC-DRV-08: sw_version == 7");

    /* TC-DRV-09: One-call sm5800_read_msg2_eng */
    mock_hal_set_reg(SM5800_REG_INT_STATUS, SM5800_INT_MSG2_RDY_MASK);
    sm5800_irq_isr(&drv);
    check_cond(SM5800_OK == sm5800_read_msg2_eng(&drv, &eng),
               "TC-DRV-09: one-call read_msg2_eng OK");
    check_cond(SM5800_ERR_NO_SAMPLE == sm5800_read_msg2_eng(&drv, &eng),
               "TC-DRV-09: second read returns ERR_NO_SAMPLE");

    /* TC-DRV-15: rad/s to deg/s helper */
    check_cond(is_near_f(sm5800_rad_s_to_deg_s(1.0f), 57.2958f, 0.01f),
               "TC-DRV-15: 1 rad/s -> 57.2958 deg/s");
}


static void test_reset_bit_and_diag(void)
{
    sm5800_drv_t drv = { .state = SM5800_STATE_UNINIT };
    sm5800_config_t cfg = make_valid_cfg();
    sm5800_diag_t diag = { .msg1_rx_count = 0U };

    mock_hal_reset();
    (void)sm5800_init(&drv, &cfg);

    /* TC-DRV-10: Timed RESET_L pulse & bounded poll */
    mock_hal_set_rst_busy_polls(3U);
    check_cond(SM5800_OK == sm5800_trigger_reset(&drv, 10U),
               "TC-DRV-10: trigger_reset completes within poll limit");
    mock_hal_set_rst_busy_polls(20U);
    check_cond(SM5800_ERR_TIMEOUT == sm5800_trigger_reset(&drv, 5U),
               "TC-DRV-10: trigger_reset reports timeout");
    check_cond(SM5800_ERR_PARAM
               == sm5800_trigger_reset(&drv, SM5800_RESET_POLL_MAX + 1U),
               "TC-DRV-10: poll_limit > MAX rejected");

    /* TC-DRV-11: Manual RESET_L hold & release */
    check_cond(SM5800_OK == sm5800_hold_reset(&drv),
               "TC-DRV-11: hold_reset OK");
    check_cond(0U != (mock_hal_get_reg(SM5800_REG_CTRL)
                      & SM5800_CTRL_RST_HOLD_MASK),
               "TC-DRV-11: CTRL.RST_HOLD set");
    check_cond(SM5800_OK == sm5800_release_reset(&drv),
               "TC-DRV-11: release_reset OK");
    check_cond(0U == (mock_hal_get_reg(SM5800_REG_CTRL)
                      & SM5800_CTRL_RST_HOLD_MASK),
               "TC-DRV-11: CTRL.RST_HOLD cleared");

    /* TC-DRV-12 & TC-DRV-13: Commanded BIT & status queries */
    check_cond(SM5800_OK == sm5800_start_cmd_bit(&drv),
               "TC-DRV-12: start_cmd_bit OK");
    check_cond(sm5800_is_cmd_bit_active(&drv),
               "TC-DRV-13: is_cmd_bit_active == true");
    check_cond(SM5800_OK == sm5800_stop_cmd_bit(&drv),
               "TC-DRV-12: stop_cmd_bit OK");
    check_cond(!sm5800_is_cmd_bit_active(&drv),
               "TC-DRV-13: is_cmd_bit_active == false");

    /* TC-DRV-14: Diagnostics read & clear */
    mock_hal_set_reg(SM5800_REG_MSG1_RX_CNT, 50U);
    mock_hal_set_reg(SM5800_REG_MSG2_RX_CNT, 10U);
    mock_hal_set_reg(SM5800_REG_CRC_ERR_CNT, 2U);
    mock_hal_set_reg(SM5800_REG_FRAME_ERR_CNT, 0x00030004U);
    mock_hal_set_reg(SM5800_REG_DEBUG_CRC, 0x0A2CF0B8U);
    mock_hal_set_reg(SM5800_REG_STATUS, 0x02000203U);
    check_cond(SM5800_OK == sm5800_read_diag(&drv, &diag),
               "TC-DRV-14: read_diag OK");
    check_cond(diag.msg2_rx_count == 10U, "TC-DRV-14: msg2_rx_count == 10");
    check_cond(diag.abort_err_count == 3U, "TC-DRV-14: abort_err_count == 3");
    check_cond(diag.frame_err_count == 4U, "TC-DRV-14: frame_err_count == 4");
    check_cond(diag.last_crc_residue == 0xF0B8U, "TC-DRV-14: residue 0xF0B8");
    check_cond(diag.is_flags_alive && diag.is_stream_active,
               "TC-DRV-14: link flags alive & stream active");

    check_cond(SM5800_OK == sm5800_clear_diag(&drv),
               "TC-DRV-14: clear_diag OK");
    check_cond(0U == mock_hal_get_reg(SM5800_REG_MSG2_RX_CNT),
               "TC-DRV-14: FPGA counters cleared");
}


static void test_fault_irq(void)
{
    sm5800_drv_t drv = { .state = SM5800_STATE_UNINIT };
    sm5800_config_t cfg = make_valid_cfg();
    uint32_t faults = 0xFFFFFFFFU;

    /* TC-DRV-17: fault IRQs reach the PS even when no Msg 02 arrives */
    mock_hal_reset();
    check_cond(SM5800_ERR_STATE == sm5800_take_irq_faults(&drv, &faults),
               "TC-DRV-17: take_irq_faults before init -> ERR_STATE");
    (void)sm5800_init(&drv, &cfg);
    check_cond(SM5800_ERR_PARAM == sm5800_take_irq_faults(NULL, &faults),
               "TC-DRV-17: NULL handle -> ERR_PARAM");
    check_cond(SM5800_ERR_PARAM == sm5800_take_irq_faults(&drv, NULL),
               "TC-DRV-17: NULL output -> ERR_PARAM");

    check_cond(SM5800_OK == sm5800_take_irq_faults(&drv, &faults),
               "TC-DRV-17: take_irq_faults OK when idle");
    check_cond(0U == faults, "TC-DRV-17: no faults after init");

    /* Sensor goes silent: FPGA raises only LINK_TIMEOUT (no MSG2_READY) */
    mock_hal_set_reg(SM5800_REG_INT_STATUS, SM5800_INT_TIMEOUT_MASK);
    sm5800_irq_isr(&drv);
    check_cond(0U == mock_hal_get_reg(SM5800_REG_INT_STATUS),
               "TC-DRV-17: ISR acknowledged LINK_TIMEOUT (W1C)");
    check_cond(!sm5800_is_msg2_pending(&drv),
               "TC-DRV-17: LINK_TIMEOUT does not fake a Msg 02 sample");

    /* Second event before the application polls: flags accumulate */
    mock_hal_set_reg(SM5800_REG_INT_STATUS, SM5800_INT_IMU_FAULT_MASK);
    sm5800_irq_isr(&drv);

    check_cond(SM5800_OK == sm5800_take_irq_faults(&drv, &faults),
               "TC-DRV-17: take_irq_faults OK");
    check_cond((SM5800_INT_TIMEOUT_MASK | SM5800_INT_IMU_FAULT_MASK)
               == faults,
               "TC-DRV-17: TIMEOUT and IMU_FAULT both reported");

    check_cond(SM5800_OK == sm5800_take_irq_faults(&drv, &faults),
               "TC-DRV-17: second take_irq_faults OK");
    check_cond(0U == faults, "TC-DRV-17: faults cleared after take");
}


int main(void)
{
    int exit_code = 0;

    test_pdi_and_init();
    test_isr_read_and_convert();
    test_reset_bit_and_diag();
    test_fault_irq();

    (void)printf("==================================================\n");
    (void)printf("SM-5800 Driver Unit Tests: %u passed, %u failed\n",
                 g_pass_count,
                 g_fail_count);
    (void)printf("==================================================\n");

    if (0U != g_fail_count)
    {
        exit_code = 1;
    }
    else
    {
        exit_code = 0;
    }

    return exit_code;
}
