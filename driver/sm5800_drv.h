/**
 * @file    sm5800_drv.h
 * @brief   PS driver API for SM-5800 6-DOF IMU (100 Hz Msg ID 02 IRQ mode).
 * @req     LLR-SM5800-DRV-001
 * @design  SDD-SM5800-3.0
 */

#ifndef SM5800_DRV_H
#define SM5800_DRV_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Parameter Data Item (PDI) schema version for sm5800_config_t */
#define SM5800_CFG_SCHEMA_VER          (0x00010100U)

/* Watchdog timeout bounds in microseconds (100 Hz nominal = 10000 us) */
#define SM5800_TIMEOUT_MIN_US          (12000U)
#define SM5800_TIMEOUT_MAX_US          (1000000U)
#define SM5800_TIMEOUT_DFLT_US         (30000U)

/* Bounded poll limit for waiting on timed RESET_L completion (> 1 ms spec) */
#define SM5800_RESET_POLL_MAX          (500000U)

/* Decoded BIT fault masks for sm5800_msg2_eng_t.bit_fault_mask */
#define SM5800_FLT_MOD_FAIL_MASK       (0x0001U)
#define SM5800_FLT_GYRO_C_MASK         (0x0002U)
#define SM5800_FLT_GYRO_B_MASK         (0x0004U)
#define SM5800_FLT_GYRO_A_MASK         (0x0008U)
#define SM5800_FLT_GYRO_TST_MASK       (0x0010U)
#define SM5800_FLT_ACCL_TST_MASK       (0x0020U)
#define SM5800_FLT_OTHR_TST_MASK       (0x0040U)
#define SM5800_FLT_MEM_TST_MASK        (0x0080U)
#define SM5800_FLT_PROC_TST_MASK       (0x0100U)

/**
 * @brief   Driver status codes returned by fallible SM-5800 API calls.
 * @req     LLR-SM5800-DRV-002
 * @design  SDD-SM5800-3.1
 */
typedef enum
{
    SM5800_OK             = 0,
    SM5800_ERR_PARAM      = 1,
    SM5800_ERR_STATE      = 2,
    SM5800_ERR_PDI_SCHEMA = 3,
    SM5800_ERR_PDI_CRC    = 4,
    SM5800_ERR_HW_ID      = 5,
    SM5800_ERR_NO_SAMPLE  = 6,
    SM5800_ERR_TIMEOUT    = 7
} sm5800_status_t;

/**
 * @brief   SEU-hardened driver lifecycle state (Hamming distance >= 15).
 * @req     LLR-SM5800-DRV-003
 * @design  SDD-SM5800-3.1
 */
typedef enum
{
    SM5800_STATE_UNINIT = 0x25C3691E,
    SM5800_STATE_READY  = 0x5A3C16E1,
    SM5800_STATE_FAULT  = 0x3C5A916E
} sm5800_state_t;

/**
 * @brief   PDI configuration block for SM-5800 100 Hz PS driver init.
 * @req     LLR-SM5800-DRV-004
 * @design  SDD-SM5800-3.2
 */
typedef struct
{
    uint32_t  schema_version;  /* Must equal SM5800_CFG_SCHEMA_VER */
    uintptr_t base_addr;       /* 4-byte aligned AXI4-Lite base address */
    uint32_t  timeout_us;      /* Watchdog timeout, us, 12000..1000000 */
    bool      is_falling_edge; /* false: rising edge, true: falling edge */
    bool      is_raw_crc_mode; /* false: 0xF0B8 SDLC, true: 0x0000 raw */
    bool      is_msg2_only;    /* true: commit/IRQ strictly on 100Hz Msg 02 */
    uint32_t  crc32;           /* IEEE 802.3 CRC-32 over preceding fields */
} sm5800_config_t;

/**
 * @brief   Raw LSB counts captured from FPGA for 100 Hz Message ID 02 (56 B).
 * @req     LLR-SM5800-DRV-005
 * @design  SDD-SM5800-3.2
 */
typedef struct
{
    int32_t  rate_x_lsb;      /* Signed 16-bit LSB, 2^-20 * 600 rad/s */
    int32_t  rate_y_lsb;      /* Signed 16-bit LSB, -32768..+32767 */
    int32_t  rate_z_lsb;      /* Signed 16-bit LSB, -32768..+32767 */
    int32_t  accel_x_lsb;     /* Signed 16-bit LSB, 2^-14 * 600 ft/s^2 */
    int32_t  accel_y_lsb;     /* Signed 16-bit LSB, -32768..+32767 */
    int32_t  accel_z_lsb;     /* Signed 16-bit LSB, -32768..+32767 */
    int32_t  delta_ang_x_lsb; /* Signed 32-bit LSB, 2^-33 rad over 0.01 s */
    int32_t  delta_ang_y_lsb; /* Signed 32-bit LSB, INT32_MIN..INT32_MAX */
    int32_t  delta_ang_z_lsb; /* Signed 32-bit LSB, INT32_MIN..INT32_MAX */
    int32_t  delta_vel_x_lsb; /* Signed 32-bit LSB, 2^-27 ft/s over 0.01 s */
    int32_t  delta_vel_y_lsb; /* Signed 32-bit LSB, INT32_MIN..INT32_MAX */
    int32_t  delta_vel_z_lsb; /* Signed 32-bit LSB, INT32_MIN..INT32_MAX */
    uint16_t status_word1;    /* Raw SM-5800 Status Word 1 (Sec. 5.2.1) */
    uint16_t status_word2;    /* Raw SM-5800 Status Word 2 (Sec. 5.2.1) */
    uint32_t timestamp_us;    /* Hardware capture timestamp, microseconds */
} sm5800_msg2_raw_t;

/**
 * @brief   Human-readable SI engineering telemetry for 100 Hz Msg 02 (60 B).
 * @req     LLR-SM5800-DRV-006
 * @design  SDD-SM5800-3.2
 */
typedef struct
{
    float    rate_x_rad_s;      /* Angular rate X, rad/s, +/-18.75 rad/s */
    float    rate_y_rad_s;      /* Angular rate Y, rad/s, +/-18.75 rad/s */
    float    rate_z_rad_s;      /* Angular rate Z, rad/s, +/-18.75 rad/s */
    float    accel_x_m_s2;      /* Linear accel X, m/s^2 (1g ~ 9.807 m/s^2) */
    float    accel_y_m_s2;      /* Linear accel Y, m/s^2, +/-365.8 m/s^2 */
    float    accel_z_m_s2;      /* Linear accel Z, m/s^2, +/-365.8 m/s^2 */
    float    delta_ang_x_rad;   /* Delta angle X over 0.01 s, radians */
    float    delta_ang_y_rad;   /* Delta angle Y over 0.01 s, radians */
    float    delta_ang_z_rad;   /* Delta angle Z over 0.01 s, radians */
    float    delta_vel_x_m_s;   /* Delta velocity X over 0.01 s, m/s */
    float    delta_vel_y_m_s;   /* Delta velocity Y over 0.01 s, m/s */
    float    delta_vel_z_m_s;   /* Delta velocity Z over 0.01 s, m/s */
    int16_t  accel_temp_degc;   /* Accel temperature, deg C, -128..+127 */
    uint16_t bit_fault_mask;    /* SM5800_FLT_* decoded BIT fault flags */
    uint8_t  seq_counter;       /* 4-bit rolling counter, 0..15 */
    uint8_t  sw_version;        /* SM-5800 firmware version, 0..255 */
    bool     has_module_failed; /* Latched Status Word 1 bit 4 */
    uint32_t timestamp_us;      /* Hardware capture time, microseconds */
} sm5800_msg2_eng_t;

/**
 * @brief   Link health and error counter diagnostics snapshot.
 * @req     LLR-SM5800-DRV-007
 * @design  SDD-SM5800-3.2
 */
typedef struct
{
    uint32_t msg1_rx_count;     /* Valid Message ID 01 frames received */
    uint32_t msg2_rx_count;     /* Valid Message ID 02 frames received */
    uint32_t crc_err_count;     /* Frames rejected for CRC mismatch */
    uint32_t seq_err_count;     /* 4-bit sequence counter gap events */
    uint32_t isr_msg2_count;    /* 100 Hz Message ID 02 interrupts serviced */
    uint16_t frame_err_count;   /* Format/Addr/Align/Length error count */
    uint16_t abort_err_count;   /* Mid-frame SDLC abort count */
    uint16_t last_crc_residue;  /* CRC residue of last closed frame */
    uint8_t  last_msg_id;       /* Last valid Msg ID (0x01 or 0x02) */
    uint8_t  last_err_mask;     /* Last rejected frame error bitmask */
    bool     is_flags_alive;    /* true: receiving SDLC 0x7E flags */
    bool     is_stream_active;  /* true: valid Msg 02 within watchdog */
    bool     has_imu_fault;     /* true: active BIT fault in SW1/SW2 */
    bool     has_module_failed; /* true: latched Module Failed bit set */
} sm5800_diag_t;

/**
 * @brief   Caller-owned SM-5800 driver instance handle (zero shared globals).
 * @req     LLR-SM5800-DRV-008
 * @design  SDD-SM5800-3.2
 */
typedef struct
{
    sm5800_state_t    state;           /* SEU-hardened lifecycle state */
    uintptr_t         base_addr;       /* AXI4-Lite peripheral base address */
    uint32_t          ctrl_shadow;     /* Cached CTRL register write image */
    volatile bool     is_msg2_pending; /* Set by ISR on 100 Hz Msg ID 02 IRQ */
    volatile uint32_t isr_msg2_count;  /* Total 100 Hz Msg 02 IRQs handled */
    volatile uint32_t isr_fault_flags; /* Latched auxiliary IRQ status bits */
} sm5800_drv_t;

/**
 * @brief       Compute deterministic IEEE 802.3 CRC-32 for a PDI config block.
 * @req         LLR-SM5800-DRV-010
 * @design      SDD-SM5800-3.3.1
 * @param[in]   p_cfg    Configuration block to checksum, must not be NULL.
 * @param[out]  p_crc32  Computed 32-bit CRC output, must not be NULL.
 * @return      SM5800_OK, SM5800_ERR_PARAM.
 * @note        ISR-safe. Reentrant. Non-blocking. WCET: O(1) (19 bytes).
 */
sm5800_status_t sm5800_calc_cfg_crc32(sm5800_config_t const * p_cfg,
                                      uint32_t * p_crc32);

/**
 * @brief       Validate PDI config, verify CORE_ID, and arm 100 Hz Msg 02 IRQ.
 * @req         LLR-SM5800-DRV-011
 * @design      SDD-SM5800-3.3.2
 * @param[out]  p_drv   Driver handle to initialize, must not be NULL.
 * @param[in]   p_cfg   Validated PDI configuration, must not be NULL.
 * @return      SM5800_OK, SM5800_ERR_PARAM, SM5800_ERR_PDI_SCHEMA,
 *              SM5800_ERR_PDI_CRC, SM5800_ERR_HW_ID.
 * @note        Not ISR-safe. Reentrant across distinct handles. Non-blocking.
 */
sm5800_status_t sm5800_init(sm5800_drv_t * p_drv,
                            sm5800_config_t const * p_cfg);

/**
 * @brief       Disable SM-5800 FPGA interrupts and clock, reset driver state.
 * @req         LLR-SM5800-DRV-012
 * @design      SDD-SM5800-3.3.3
 * @param[out]  p_drv   Initialized driver handle, must not be NULL.
 * @return      SM5800_OK, SM5800_ERR_PARAM, SM5800_ERR_STATE.
 * @note        Not ISR-safe. Reentrant across distinct handles. Non-blocking.
 */
sm5800_status_t sm5800_deinit(sm5800_drv_t * p_drv);

/**
 * @brief       PS Interrupt Service Routine connected to FPGA irq_o (100 Hz).
 * @req         LLR-SM5800-DRV-013
 * @design      SDD-SM5800-3.3.4
 * @param[out]  p_drv   Driver handle registered with GIC, must not be NULL.
 * @note        ISR-safe. Deterministic O(1), no loops, no floating point.
 *              Invalid p_drv or uninitialized state is ignored safely so the
 *              ISR never traps or corrupts memory on spurious interrupts.
 */
void sm5800_irq_isr(sm5800_drv_t * p_drv);

/**
 * @brief       Check whether a new 100 Hz Msg ID 02 sample was flagged by ISR.
 * @req         LLR-SM5800-DRV-014
 * @design      SDD-SM5800-3.3.5
 * @param[in]   p_drv   Driver handle, may be NULL (returns false safely).
 * @return      true if 100 Hz Msg ID 02 sample is pending; false otherwise.
 * @note        ISR-safe. Reentrant. Non-blocking. WCET: O(1).
 */
bool sm5800_is_msg2_pending(sm5800_drv_t const * p_drv);

/**
 * @brief       Atomically lock and read raw 100 Hz Message ID 02 registers.
 * @req         LLR-SM5800-DRV-015
 * @design      SDD-SM5800-3.3.6
 * @param[out]  p_drv  Initialized driver handle, must not be NULL.
 * @param[out]  p_raw  Raw register LSB output struct, must not be NULL.
 * @return      SM5800_OK, SM5800_ERR_PARAM, SM5800_ERR_STATE,
 *              SM5800_ERR_NO_SAMPLE.
 * @note        Not ISR-safe. Uses hardware CTRL.SNAP_LOCK for tear-free read.
 */
sm5800_status_t sm5800_read_msg2_raw(sm5800_drv_t * p_drv,
                                     sm5800_msg2_raw_t * p_raw);

/**
 * @brief       Convert raw 100 Hz Msg 02 counts into verifiable SI eng units.
 * @req         LLR-SM5800-DRV-016
 * @design      SDD-SM5800-3.3.7
 * @param[in]   p_raw  Raw Message ID 02 register snapshot, must not be NULL.
 * @param[out]  p_eng  Human-readable SI output struct, must not be NULL.
 * @return      SM5800_OK, SM5800_ERR_PARAM.
 * @note        Not ISR-safe (uses float). Reentrant. Pure conversion for user
 *              readout and bench verification (1 g ~ 9.807 m/s^2).
 */
sm5800_status_t sm5800_raw_to_eng_msg2(sm5800_msg2_raw_t const * p_raw,
                                       sm5800_msg2_eng_t * p_eng);

/**
 * @brief       One-call atomic read and SI conversion of 100 Hz Msg ID 02.
 * @req         LLR-SM5800-DRV-017
 * @design      SDD-SM5800-3.3.8
 * @param[out]  p_drv  Initialized driver handle, must not be NULL.
 * @param[out]  p_eng  Human-readable SI output struct, must not be NULL.
 * @return      SM5800_OK, SM5800_ERR_PARAM, SM5800_ERR_STATE,
 *              SM5800_ERR_NO_SAMPLE.
 * @note        Not ISR-safe (uses float). Locks FPGA snapshot, reads all 14
 *              Msg 02 fields, unlocks, and converts directly to SI units.
 */
sm5800_status_t sm5800_read_msg2_eng(sm5800_drv_t * p_drv,
                                     sm5800_msg2_eng_t * p_eng);

/**
 * @brief       Command FPGA to assert timed RESET_L low pulse (> 1 ms).
 * @req         LLR-SM5800-DRV-018
 * @design      SDD-SM5800-3.3.9
 * @param[out]  p_drv       Initialized driver handle, must not be NULL.
 * @param[in]   poll_limit  Max status polls, 0 = fire-and-return, max
 *                          SM5800_RESET_POLL_MAX.
 * @return      SM5800_OK, SM5800_ERR_PARAM, SM5800_ERR_STATE,
 *              SM5800_ERR_TIMEOUT.
 * @note        Not ISR-safe. Bounded by poll_limit iterations.
 */
sm5800_status_t sm5800_trigger_reset(sm5800_drv_t * p_drv,
                                     uint32_t poll_limit);

/**
 * @brief       Command FPGA to manually hold SM-5800 RESET_L low (Sec. 4.4).
 * @req         LLR-SM5800-DRV-019
 * @design      SDD-SM5800-3.3.10
 * @param[out]  p_drv   Initialized driver handle, must not be NULL.
 * @return      SM5800_OK, SM5800_ERR_PARAM, SM5800_ERR_STATE.
 * @note        Not ISR-safe. Non-blocking. Hold > 1 ms before release.
 */
sm5800_status_t sm5800_hold_reset(sm5800_drv_t * p_drv);

/**
 * @brief       Command FPGA to release SM-5800 RESET_L back to open-drain HiZ.
 * @req         LLR-SM5800-DRV-020
 * @design      SDD-SM5800-3.3.11
 * @param[out]  p_drv   Initialized driver handle, must not be NULL.
 * @return      SM5800_OK, SM5800_ERR_PARAM, SM5800_ERR_STATE.
 * @note        Not ISR-safe. Non-blocking. IMU activates in <= 0.8 s.
 */
sm5800_status_t sm5800_release_reset(sm5800_drv_t * p_drv);

/**
 * @brief       Command FPGA to assert BIT_L low for Commanded BIT (Sec. 6.3).
 * @req         LLR-SM5800-DRV-021
 * @design      SDD-SM5800-3.3.12
 * @param[out]  p_drv   Initialized driver handle, must not be NULL.
 * @return      SM5800_OK, SM5800_ERR_PARAM, SM5800_ERR_STATE.
 * @note        Not ISR-safe. Non-blocking. Hold >= 200 ms before stopping.
 */
sm5800_status_t sm5800_start_cmd_bit(sm5800_drv_t * p_drv);

/**
 * @brief       Command FPGA to release BIT_L high and resume 100 Hz Msg 02.
 * @req         LLR-SM5800-DRV-022
 * @design      SDD-SM5800-3.3.13
 * @param[out]  p_drv   Initialized driver handle, must not be NULL.
 * @return      SM5800_OK, SM5800_ERR_PARAM, SM5800_ERR_STATE.
 * @note        Not ISR-safe. Non-blocking. Normal output resumes <= 100 ms.
 */
sm5800_status_t sm5800_stop_cmd_bit(sm5800_drv_t * p_drv);

/**
 * @brief       Query whether FPGA timed RESET_L pulse is currently active.
 * @req         LLR-SM5800-DRV-023
 * @design      SDD-SM5800-3.3.14
 * @param[in]   p_drv   Driver handle, may be NULL (returns false safely).
 * @return      true if STATUS.RST_BUSY is asserted; false otherwise.
 * @note        ISR-safe. Reentrant. Non-blocking. WCET: O(1).
 */
bool sm5800_is_reset_busy(sm5800_drv_t const * p_drv);

/**
 * @brief       Query whether FPGA is currently asserting BIT_L low.
 * @req         LLR-SM5800-DRV-024
 * @design      SDD-SM5800-3.3.15
 * @param[in]   p_drv   Driver handle, may be NULL (returns false safely).
 * @return      true if STATUS.BIT_ACTIVE is asserted; false otherwise.
 * @note        ISR-safe. Reentrant. Non-blocking. WCET: O(1).
 */
bool sm5800_is_cmd_bit_active(sm5800_drv_t const * p_drv);

/**
 * @brief       Read hardware link health, error counters, and debug residue.
 * @req         LLR-SM5800-DRV-025
 * @design      SDD-SM5800-3.3.16
 * @param[in]   p_drv   Initialized driver handle, must not be NULL.
 * @param[out]  p_diag  Diagnostics output structure, must not be NULL.
 * @return      SM5800_OK, SM5800_ERR_PARAM, SM5800_ERR_STATE.
 * @note        Not ISR-safe. Reentrant across distinct handles. Non-blocking.
 */
sm5800_status_t sm5800_read_diag(sm5800_drv_t const * p_drv,
                                 sm5800_diag_t * p_diag);

/**
 * @brief       Clear FPGA packet/error counters and latched ISR fault flags.
 * @req         LLR-SM5800-DRV-026
 * @design      SDD-SM5800-3.3.17
 * @param[out]  p_drv   Initialized driver handle, must not be NULL.
 * @return      SM5800_OK, SM5800_ERR_PARAM, SM5800_ERR_STATE.
 * @note        Not ISR-safe. Non-blocking.
 */
sm5800_status_t sm5800_clear_diag(sm5800_drv_t * p_drv);

/**
 * @brief       Convert angular rate from rad/s to deg/s for bench checking.
 * @req         LLR-SM5800-DRV-027
 * @design      SDD-SM5800-3.3.18
 * @param[in]   rate_rad_s  Angular rate in rad/s, +/-18.75 rad/s.
 * @return      Angular rate in deg/s, +/-1074.3 deg/s.
 * @note        Not ISR-safe (uses float). Pure math; cannot fail.
 */
float sm5800_rad_s_to_deg_s(float rate_rad_s);

#endif /* SM5800_DRV_H */
