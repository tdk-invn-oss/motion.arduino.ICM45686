/*
 *
 * Copyright (c) [2024] by InvenSense, Inc.
 * 
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

#ifndef __INV_IMU_EDMP_GAF_MEMMAP_H__
#define __INV_IMU_EDMP_GAF_MEMMAP_H__

#ifdef __cplusplus
extern "C" {
#endif

/* gaf_state_init
 *
 * Current state of GAF initialisation, set to 0 if host requires to perform it, edmp will set it back to 1 when done
 */
#define EDMP_GAF_STATE_INIT                                     0x187a
#define EDMP_GAF_STATE_INIT_SIZE                                1

/* gaf_state_config
 *
 * GAF algorithm parameters
 */
#define EDMP_GAF_STATE_CONFIG                                   0x1620
#define EDMP_GAF_STATE_CONFIG_SIZE                              68

/* gaf_gain_mounting_matrix_a
 *
 * A 3x3 matrix [GMA] expressed in s32q30 that will represent the combined matrix of:
 * - The device orientation [M], expressed as -1/0/1 to remap axes
 * - The sensor gain [G], expressed as coefficents on the diagonal elements
 * - The sensor cross-axis sensitivy [CS], expressed as coefficent on the non-diagonal elements
 * It is computed as: [GMA] = [M]*[G]*[CS]
 * Default:
 * [1<<30   0     0  ]
 * [  0   1<<30   0  ]
 * [  0     0   1<<30]
 */
#define EDMP_GAF_GAIN_MOUNTING_MATRIX_A                         0x19b8
#define EDMP_GAF_GAIN_MOUNTING_MATRIX_A_SIZE                    36

/* gaf_gain_mounting_matrix_g
 *
 * A 3x3 matrix [GMG] expressed in s32q30 that will represent the combined matrix of:
 * - The device orientation [M], expressed as -1/0/1 to remap axes
 * - The sensor gain [G], expressed as coefficents on the diagonal elements
 * - The sensor cross-axis sensitivy [CS], expressed as coefficent on the non-diagonal elements
 * It is computed as: [GMG] = [M]*[G]*[CS]
 * Default:
 * [1<<30   0     0  ]
 * [  0   1<<30   0  ]
 * [  0     0   1<<30]
 */
#define EDMP_GAF_GAIN_MOUNTING_MATRIX_G                         0x19dc
#define EDMP_GAF_GAIN_MOUNTING_MATRIX_G_SIZE                    36

/* gaf_state_gyro_bias
 *
 * Gyroscope bias to start with (one value per axis).
 * Unit: dps in s32q16
 * Default: 0;0;0
 */
#define EDMP_GAF_STATE_GYRO_BIAS                                0x17ac
#define EDMP_GAF_STATE_GYRO_BIAS_SIZE                           12

/* gaf_state_acc_bias
 *
 * Accelerometer bias to start with (one value per axis).
 * Unit: g in s32q16
 * Default: 0;0;0
 */
#define EDMP_GAF_STATE_ACC_BIAS                                 0x187c
#define EDMP_GAF_STATE_ACC_BIAS_SIZE                            12

#ifdef __cplusplus
}
#endif

#endif // __INV_IMU_EDMP_GAF_MEMMAP_H__
