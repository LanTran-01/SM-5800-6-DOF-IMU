/**
 * @file    sm5800_regs.h
 * @brief   SM-5800 AXI4-Lite IP core register offsets, masks, and shifts.
 * @req     LLR-SM5800-REG-001
 * @design  SDD-SM5800-2.1
 */

#ifndef SM5800_REGS_H
#define SM5800_REGS_H

/* Expected CORE_ID register value: IP 0x5800, version 1.0.0 */
#define SM5800_CORE_ID_EXPECTED        (0x58000100U)

/* AXI4-Lite byte-addressed register offsets (sm5800_axi4_lite_if.v) */
#define SM5800_REG_CORE_ID             (0x00U)
#define SM5800_REG_CTRL                (0x04U)
#define SM5800_REG_STATUS              (0x08U)
#define SM5800_REG_INT_STATUS          (0x0CU)
#define SM5800_REG_INT_ENABLE          (0x10U)
#define SM5800_REG_TIMEOUT_CFG         (0x14U)
#define SM5800_REG_MC_TIMESTAMP        (0x18U)
#define SM5800_REG_SP_TIMESTAMP        (0x1CU)

/* Motion Control registers (Sec. 5.2.1, included in Msg ID 01 and Msg ID 02) */
#define SM5800_REG_ANG_RATE_X          (0x20U)
#define SM5800_REG_ANG_RATE_Y          (0x24U)
#define SM5800_REG_ANG_RATE_Z          (0x28U)
#define SM5800_REG_LIN_ACC_X           (0x2CU)
#define SM5800_REG_LIN_ACC_Y           (0x30U)
#define SM5800_REG_LIN_ACC_Z           (0x34U)
#define SM5800_REG_STATUS_WORD1        (0x38U)
#define SM5800_REG_STATUS_WORD2        (0x3CU)

/* Spatial Data registers (Sec. 5.2.2, 100 Hz Message ID 02) */
#define SM5800_REG_DELTA_ANG_X         (0x40U)
#define SM5800_REG_DELTA_ANG_Y         (0x44U)
#define SM5800_REG_DELTA_ANG_Z         (0x48U)
#define SM5800_REG_DELTA_VEL_X         (0x4CU)
#define SM5800_REG_DELTA_VEL_Y         (0x50U)
#define SM5800_REG_DELTA_VEL_Z         (0x54U)

/* Diagnostics and counters */
#define SM5800_REG_MSG1_RX_CNT         (0x58U)
#define SM5800_REG_MSG2_RX_CNT         (0x5CU)
#define SM5800_REG_CRC_ERR_CNT         (0x60U)
#define SM5800_REG_FRAME_ERR_CNT       (0x64U)
#define SM5800_REG_SEQ_ERR_CNT         (0x68U)
#define SM5800_REG_DEBUG_CRC           (0x6CU)

/* CTRL register (0x04) bit masks */
#define SM5800_CTRL_CLK_EN_MASK        (0x00000001U)
#define SM5800_CTRL_SNAP_LOCK_MASK     (0x00000002U)
#define SM5800_CTRL_CMD_BIT_MASK       (0x00000004U)
#define SM5800_CTRL_RST_PULSE_MASK     (0x00000008U)
#define SM5800_CTRL_RST_HOLD_MASK      (0x00000010U)
#define SM5800_CTRL_FALL_EDGE_MASK     (0x00000020U)
#define SM5800_CTRL_CRC_RAW_MASK       (0x00000040U)
#define SM5800_CTRL_CRC_BYP_MASK       (0x00000080U)
#define SM5800_CTRL_CLR_STATS_MASK     (0x00000100U)
#define SM5800_CTRL_SOFT_RST_MASK      (0x00000200U)
#define SM5800_CTRL_MSG1_ONLY_MASK     (0x00000400U)
#define SM5800_CTRL_MSG2_ONLY_MASK     (0x00000800U)

/* STATUS register (0x08) bit masks and shifts */
#define SM5800_STS_FLAGS_OK_MASK       (0x00000001U)
#define SM5800_STS_STREAM_OK_MASK      (0x00000002U)
#define SM5800_STS_IMU_FAULT_MASK      (0x00000004U)
#define SM5800_STS_MOD_FAIL_MASK       (0x00000008U)
#define SM5800_STS_LOCKED_MASK         (0x00000010U)
#define SM5800_STS_RST_BUSY_MASK       (0x00000020U)
#define SM5800_STS_BIT_ACT_MASK        (0x00000040U)
#define SM5800_STS_SDLC_SYNC_MASK      (0x00000080U)
#define SM5800_STS_LAST_ID_MASK        (0x0000FF00U)
#define SM5800_STS_LAST_ID_SHIFT       (8U)
#define SM5800_STS_LAST_SEQ_MASK       (0x000F0000U)
#define SM5800_STS_LAST_SEQ_SHIFT      (16U)
#define SM5800_STS_LAST_ERR_MASK       (0x00F00000U)
#define SM5800_STS_LAST_ERR_SHIFT      (20U)
#define SM5800_STS_MSG1_ONLY_MASK      (0x01000000U)
#define SM5800_STS_MSG2_ONLY_MASK      (0x02000000U)

/* INT_STATUS (0x0C) and INT_ENABLE (0x10) bit masks */
#define SM5800_INT_MSG1_RDY_MASK       (0x00000001U)
#define SM5800_INT_MSG2_RDY_MASK       (0x00000002U)
#define SM5800_INT_IMU_FAULT_MASK      (0x00000004U)
#define SM5800_INT_RX_ERR_MASK         (0x00000008U)
#define SM5800_INT_SEQ_GAP_MASK        (0x00000010U)
#define SM5800_INT_TIMEOUT_MASK        (0x00000020U)
#define SM5800_INT_MC_ANY_MASK         (0x00000040U)
#define SM5800_INT_ALL_MASK            (0x0000007FU)

/* SM-5800 Status Word 1 (Sec. 5.2.1) masks and shifts */
#define SM5800_SW1_SEQ_MASK            (0x000FU)
#define SM5800_SW1_MOD_FAIL_MASK       (0x0010U)
#define SM5800_SW1_GYRO_C_MASK         (0x0020U)
#define SM5800_SW1_GYRO_B_MASK         (0x0040U)
#define SM5800_SW1_GYRO_A_MASK         (0x0080U)
#define SM5800_SW1_FAULTS_MASK         (0x00F0U)
#define SM5800_SW1_FAULTS_SHIFT        (4U)
#define SM5800_SW1_TEMP_MASK           (0xFF00U)
#define SM5800_SW1_TEMP_SHIFT          (8U)

/* SM-5800 Status Word 2 (Sec. 5.2.1) masks and shifts */
#define SM5800_SW2_VER_MASK            (0x00FFU)
#define SM5800_SW2_FAULTS_MASK         (0xF800U)
#define SM5800_SW2_FAULTS_SHIFT        (11U)

/* FRAME_ERR_CNT (0x64) packing masks and shifts */
#define SM5800_FERR_FMT_MASK           (0x0000FFFFU)
#define SM5800_FERR_ABORT_MASK         (0xFFFF0000U)
#define SM5800_FERR_ABORT_SHIFT        (16U)

/* DEBUG_CRC (0x6C) packing masks and shifts */
#define SM5800_DBG_RESIDUE_MASK        (0x0000FFFFU)
#define SM5800_DBG_LEN_MASK            (0x003F0000U)
#define SM5800_DBG_LEN_SHIFT           (16U)
#define SM5800_DBG_ADDR_MASK           (0xFF000000U)
#define SM5800_DBG_ADDR_SHIFT          (24U)

#endif /* SM5800_REGS_H */
