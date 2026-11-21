/**
 * @file    sm5800_ps_example.c
 * @brief   PS example: 100 Hz Msg 02 IRQ readout, Reset/BIT recovery & verify.
 * @req     LLR-SM5800-APP-001
 * @design  SDD-SM5800-4.0
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "sm5800_drv.h"
#include "sm5800_regs.h"

/* Example AXI4-Lite peripheral base address from Vivado Address Editor */
#define IMU_AXI_BASE_ADDR       ((uintptr_t)0x43C00000U)

/* Bounded iteration limits for example acquisition and BIT dwell */
#define APP_MAX_POLL_CYCLES     (1000000U)
#define APP_TARGET_SAMPLES      (100U)
#define APP_CMD_BIT_DWELL_TICKS (200000U)

/* Bench self-check vector: 1 rad/s gyro, ~1 g accel, 1 mrad, 0.0981 m/s */
#define TEST_RATE_X_LSB         (1748)
#define TEST_ACCEL_Z_LSB        (879)
#define TEST_DANG_X_LSB         (8589935)
#define TEST_DVEL_Z_LSB         (43183277)
#define TEST_SW1_NOMINAL_25C    (0x1902U)
#define TEST_SW2_VER_3          (0x0003U)
#define TEST_RATE_1RAD_MIN      (0.995f)
#define TEST_RATE_1RAD_MAX      (1.005f)
#define TEST_ACCEL_1G_MIN_M_S2  (9.75f)
#define TEST_ACCEL_1G_MAX_M_S2  (9.86f)
#define TEST_DANG_1MRAD_MIN     (0.00099f)
#define TEST_DANG_1MRAD_MAX     (0.00101f)
#define TEST_DVEL_1G_MIN_M_S    (0.0975f)
#define TEST_DVEL_1G_MAX_M_S    (0.0986f)
#define TEST_TEMP_EXPECTED_C    (25)
#define TEST_VER_EXPECTED       (3U)
#define MAX_VALID_RATE_DEG_S    (2000.0f)

/* Consecutive RESET_L recovery attempts allowed for a silent sensor */
#define APP_MAX_LINK_RECOVERY   (3U)

/**
 * @brief   Module-private SM-5800 driver instance handle.
 * @req     LLR-SM5800-APP-001
 * @design  SDD-SM5800-4.0
 */
static sm5800_drv_t g_imu_drv =
{
    .state           = SM5800_STATE_UNINIT,
    .base_addr       = 0U,
    .ctrl_shadow     = 0U,
    .is_msg2_pending = false,
    .isr_msg2_count  = 0U,
    .isr_fault_flags = 0U
};

/**
 * @brief   Latest human-readable 100 Hz Message ID 02 telemetry snapshot.
 * @req     LLR-SM5800-APP-005
 * @design  SDD-SM5800-4.4
 */
static sm5800_msg2_eng_t g_last_eng =
{
    .rate_x_rad_s      = 0.0f,
    .rate_y_rad_s      = 0.0f,
    .rate_z_rad_s      = 0.0f,
    .accel_x_m_s2      = 0.0f,
    .accel_y_m_s2      = 0.0f,
    .accel_z_m_s2      = 0.0f,
    .delta_ang_x_rad   = 0.0f,
    .delta_ang_y_rad   = 0.0f,
    .delta_ang_z_rad   = 0.0f,
    .delta_vel_x_m_s   = 0.0f,
    .delta_vel_y_m_s   = 0.0f,
    .delta_vel_z_m_s   = 0.0f,
    .accel_temp_degc   = 0,
    .bit_fault_mask    = 0U,
    .seq_counter       = 0U,
    .sw_version        = 0U,
    .has_module_failed = false,
    .timestamp_us      = 0U
};

/**
 * @brief   Latest X-axis angular rate in deg/s for bench display.
 * @req     LLR-SM5800-APP-005
 * @design  SDD-SM5800-4.4
 */
static float g_last_rate_x_deg_s = 0.0f;

/**
 * @brief   Consecutive link-timeout recoveries since the last good sample.
 * @req     LLR-SM5800-APP-008
 * @design  SDD-SM5800-4.7
 */
static uint32_t g_link_recovery_count = 0U;


/* Private prototypes */
static sm5800_status_t init_imu_100hz(void);
static sm5800_status_t verify_raw_to_eng_math(void);
static sm5800_status_t handle_msg2_and_faults(
    sm5800_msg2_eng_t const * p_eng);
static sm5800_status_t run_commanded_bit_cycle(void);
static sm5800_status_t handle_link_faults(void);

/**
 * @brief       PS GIC Interrupt Service Routine bound to PL irq_o (100 Hz).
 * @req         LLR-SM5800-APP-002
 * @design      SDD-SM5800-4.1
 * @note        ISR-safe. Register this handler with XScuGic for FPGA irq_o.
 *              Delegates directly to sm5800_irq_isr(); no error swallowed.
 */
void sm5800_ps_gic_isr(void);


void sm5800_ps_gic_isr(void)
{
    sm5800_irq_isr(&g_imu_drv);
}


/**
 * @brief       Bare-metal PS entry point for 100 Hz Msg 02 acquisition loop.
 * @req         LLR-SM5800-APP-003
 * @design      SDD-SM5800-4.2
 * @return      0 on normal completion, 1 if self-check, init, or read fails.
 * @note        Not ISR-safe. Bounded by APP_MAX_POLL_CYCLES iterations.
 */
int main(void)
{
    int exit_code = 0;
    sm5800_status_t status = verify_raw_to_eng_math();

    if (SM5800_OK == status)
    {
        status = init_imu_100hz();
    }
    else
    {
        /* Self-check failed; skip hardware initialization */
    }

    if (SM5800_OK != status)
    {
        exit_code = 1;
    }
    else
    {
        uint32_t sample_count = 0U;
        uint32_t poll_cycles = 0U;
        bool is_running = true;

        /* Note: Connect sm5800_ps_gic_isr to the GIC IRQ ID of FPGA irq_o
         * via vendor BSP (e.g. XScuGic_Connect) before entering this loop. */
        while ((sample_count < APP_TARGET_SAMPLES)
               && (poll_cycles < APP_MAX_POLL_CYCLES)
               && is_running)
        {
            poll_cycles++;
            status = handle_link_faults();

            if (SM5800_OK != status)
            {
                /* Sensor stayed silent after bounded recovery attempts */
                exit_code = 1;
                is_running = false;
            }
            else if (sm5800_is_msg2_pending(&g_imu_drv))
            {
                sm5800_msg2_eng_t eng_sample = g_last_eng;

                /* Single-call atomic read + conversion to verifiable SI */
                status = sm5800_read_msg2_eng(&g_imu_drv, &eng_sample);

                if (SM5800_OK == status)
                {
                    status = handle_msg2_and_faults(&eng_sample);
                }
                else
                {
                    /* Read failed; exit acquisition loop cleanly */
                }

                if (SM5800_OK == status)
                {
                    sample_count++;
                    g_link_recovery_count = 0U;
                }
                else
                {
                    exit_code = 1;
                    is_running = false;
                }
            }
            else
            {
                /* Wait for next 100 Hz Message ID 02 interrupt from FPGA */
            }
        }

        if (sample_count < APP_TARGET_SAMPLES)
        {
            exit_code = 1;
        }
        else
        {
            /* Target sample count acquired within poll budget */
        }

        status = sm5800_deinit(&g_imu_drv);
        if (SM5800_OK != status)
        {
            exit_code = 1;
        }
        else
        {
            /* Clean shutdown complete */
        }
    }

    return exit_code;
}


/**
 * @brief       Build CRC-protected PDI config and init 100 Hz Msg 02 mode.
 * @req         LLR-SM5800-APP-004
 * @design      SDD-SM5800-4.3
 * @return      SM5800_OK on success, or sm5800_status_t error code.
 * @note        Not ISR-safe. Configures FPGA strictly for 100 Hz Msg ID 02.
 */
static sm5800_status_t init_imu_100hz(void)
{
    sm5800_status_t result = SM5800_OK;
    uint32_t computed_crc = 0U;
    sm5800_config_t cfg =
    {
        .schema_version  = SM5800_CFG_SCHEMA_VER,
        .base_addr       = IMU_AXI_BASE_ADDR,
        .timeout_us      = SM5800_TIMEOUT_DFLT_US,
        .is_falling_edge = false,
        .is_raw_crc_mode = false,
        .is_msg2_only    = true,
        .crc32           = 0U
    };

    result = sm5800_calc_cfg_crc32(&cfg, &computed_crc);

    if (SM5800_OK == result)
    {
        cfg.crc32 = computed_crc;
        result = sm5800_init(&g_imu_drv, &cfg);
    }
    else
    {
        /* CRC calculation failed; propagate status */
    }

    return result;
}


/**
 * @brief       Verify one-call raw-to-engineering conversion with 1 g vector.
 * @req         LLR-SM5800-APP-006
 * @design      SDD-SM5800-4.5
 * @return      SM5800_OK if converted 1 g accel is within tolerance, else err.
 * @note        Not ISR-safe (uses float). Uses explicit bounds, no float ==.
 */
static sm5800_status_t verify_raw_to_eng_math(void)
{
    sm5800_status_t result = SM5800_OK;
    sm5800_msg2_raw_t test_raw =
    {
        .rate_x_lsb      = TEST_RATE_X_LSB,
        .rate_y_lsb      = 0,
        .rate_z_lsb      = 0,
        .accel_x_lsb     = 0,
        .accel_y_lsb     = 0,
        .accel_z_lsb     = TEST_ACCEL_Z_LSB,
        .delta_ang_x_lsb = TEST_DANG_X_LSB,
        .delta_ang_y_lsb = 0,
        .delta_ang_z_lsb = 0,
        .delta_vel_x_lsb = 0,
        .delta_vel_y_lsb = 0,
        .delta_vel_z_lsb = TEST_DVEL_Z_LSB,
        .status_word1    = TEST_SW1_NOMINAL_25C,
        .status_word2    = TEST_SW2_VER_3,
        .timestamp_us    = 0U
    };
    sm5800_msg2_eng_t test_eng =
    {
        .rate_x_rad_s      = 0.0f,
        .rate_y_rad_s      = 0.0f,
        .rate_z_rad_s      = 0.0f,
        .accel_x_m_s2      = 0.0f,
        .accel_y_m_s2      = 0.0f,
        .accel_z_m_s2      = 0.0f,
        .delta_ang_x_rad   = 0.0f,
        .delta_ang_y_rad   = 0.0f,
        .delta_ang_z_rad   = 0.0f,
        .delta_vel_x_m_s   = 0.0f,
        .delta_vel_y_m_s   = 0.0f,
        .delta_vel_z_m_s   = 0.0f,
        .accel_temp_degc   = 0,
        .bit_fault_mask    = 0U,
        .seq_counter       = 0U,
        .sw_version        = 0U,
        .has_module_failed = false,
        .timestamp_us      = 0U
    };

    result = sm5800_raw_to_eng_msg2(&test_raw, &test_eng);

    if (SM5800_OK == result)
    {
        if ((test_eng.rate_x_rad_s < TEST_RATE_1RAD_MIN)
            || (test_eng.rate_x_rad_s > TEST_RATE_1RAD_MAX))
        {
            result = SM5800_ERR_PARAM;
        }
        else if ((test_eng.accel_z_m_s2 < TEST_ACCEL_1G_MIN_M_S2)
                 || (test_eng.accel_z_m_s2 > TEST_ACCEL_1G_MAX_M_S2))
        {
            result = SM5800_ERR_PARAM;
        }
        else if ((test_eng.delta_ang_x_rad < TEST_DANG_1MRAD_MIN)
                 || (test_eng.delta_ang_x_rad > TEST_DANG_1MRAD_MAX))
        {
            result = SM5800_ERR_PARAM;
        }
        else if ((test_eng.delta_vel_z_m_s < TEST_DVEL_1G_MIN_M_S)
                 || (test_eng.delta_vel_z_m_s > TEST_DVEL_1G_MAX_M_S))
        {
            result = SM5800_ERR_PARAM;
        }
        else if ((TEST_TEMP_EXPECTED_C != test_eng.accel_temp_degc)
                 || (0U != test_eng.bit_fault_mask)
                 || (TEST_VER_EXPECTED != test_eng.sw_version)
                 || test_eng.has_module_failed)
        {
            result = SM5800_ERR_PARAM;
        }
        else
        {
            result = SM5800_OK;
        }
    }
    else
    {
        /* Conversion failed; propagate error */
    }

    return result;
}


/**
 * @brief       Store Msg 02 telemetry and trigger PS->FPGA Reset/BIT on fault.
 * @req         LLR-SM5800-APP-005
 * @design      SDD-SM5800-4.4
 * @param[in]   p_eng  Converted 100 Hz Msg 02 sample, guaranteed non-NULL.
 * @return      SM5800_OK or error status from Reset/BIT recovery command.
 * @note        Not ISR-safe. Commands RESET_L on latched Module Failed, or
 *              Commanded BIT (BIT_L) on active Background BIT fault flags.
 */
static sm5800_status_t handle_msg2_and_faults(
    sm5800_msg2_eng_t const * p_eng)
{
    sm5800_status_t result = SM5800_OK;

    g_last_eng = *p_eng;
    g_last_rate_x_deg_s = sm5800_rad_s_to_deg_s(p_eng->rate_x_rad_s);

    if ((g_last_rate_x_deg_s < -MAX_VALID_RATE_DEG_S)
        || (g_last_rate_x_deg_s > MAX_VALID_RATE_DEG_S))
    {
        result = SM5800_ERR_PARAM;
    }
    else if (p_eng->has_module_failed)
    {
        /* Severe latched fault: command FPGA to pulse SM-5800 RESET_L > 1ms */
        result = sm5800_trigger_reset(&g_imu_drv, SM5800_RESET_POLL_MAX);
    }
    else if (0U != p_eng->bit_fault_mask)
    {
        /* Active BIT fault: command FPGA to run SM-5800 Commanded BIT */
        result = run_commanded_bit_cycle();
    }
    else
    {
        /* Nominal 100 Hz sample ready for navigation / flight control */
    }

    return result;
}


/**
 * @brief       Execute a bounded PS->FPGA->IMU Commanded BIT cycle (BIT_L).
 * @req         LLR-SM5800-APP-007
 * @design      SDD-SM5800-4.6
 * @return      SM5800_OK or driver error status.
 * @note        Not ISR-safe. Replace dwell loop with PS hardware timer (>=200ms
 *              per SM-5800 Sec. 6.3.2) in target RTOS/bare-metal system.
 */
static sm5800_status_t run_commanded_bit_cycle(void)
{
    sm5800_status_t result = sm5800_start_cmd_bit(&g_imu_drv);

    if (SM5800_OK == result)
    {
        uint32_t dwell_tick = 0U;
        bool bit_held_ok = true;

        /* Bounded placeholder dwell; use PS TTC/GTimer for >= 200 ms hold */
        for (dwell_tick = 0U; dwell_tick < APP_CMD_BIT_DWELL_TICKS;
             dwell_tick++)
        {
            bool is_active = sm5800_is_cmd_bit_active(&g_imu_drv);
            if (!is_active)
            {
                bit_held_ok = false;
                break;
            }
            else
            {
                /* Continue holding BIT_L low during Commanded BIT window */
            }
        }

        result = sm5800_stop_cmd_bit(&g_imu_drv);
        if ((SM5800_OK == result) && (!bit_held_ok))
        {
            result = SM5800_ERR_STATE;
        }
        else
        {
            /* BIT_L held for full dwell window and released cleanly */
        }
    }
    else
    {
        /* Failed to assert BIT_L; propagate error */
    }

    return result;
}


/**
 * @brief       React to fault IRQs latched by the ISR (sensor went silent).
 * @req         LLR-SM5800-APP-008
 * @design      SDD-SM5800-4.7
 * @return      SM5800_OK, error from sm5800_trigger_reset(), or
 *              SM5800_ERR_TIMEOUT once APP_MAX_LINK_RECOVERY is exhausted.
 * @note        Not ISR-safe. On LINK_TIMEOUT pulses RESET_L (> 1 ms) so the
 *              SM-5800 reboots; the FPGA watchdog re-arms and reports again
 *              if the sensor still sends no valid data.
 */
static sm5800_status_t handle_link_faults(void)
{
    uint32_t faults = 0U;
    sm5800_status_t result = sm5800_take_irq_faults(&g_imu_drv, &faults);

    if ((SM5800_OK == result)
        && (0U != (faults & SM5800_INT_TIMEOUT_MASK)))
    {
        if (g_link_recovery_count < APP_MAX_LINK_RECOVERY)
        {
            g_link_recovery_count++;
            result = sm5800_trigger_reset(&g_imu_drv,
                                          SM5800_RESET_POLL_MAX);
        }
        else
        {
            result = SM5800_ERR_TIMEOUT;
        }
    }
    else
    {
        /* No link timeout; BIT faults are handled per Msg 02 sample */
    }

    return result;
}
