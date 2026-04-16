/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2021 InvenSense Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively "Software") is subject
 * to InvenSense and its licensors' intellectual property rights under U.S. and international copyright
 * and other intellectual property rights laws.
 *
 * InvenSense and its licensors retain all intellectual property and proprietary rights in and to the Software
 * and any use, reproduction, disclosure or distribution of the Software without an express license agreement
 * from InvenSense is strictly prohibited.
 *
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, THE SOFTWARE IS
 * PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, IN NO EVENT SHALL
 * INVENSENSE BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, OR ANY
 * DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
 *
 * ________________________________________________________________________________________________________
 */

#ifndef __INV_IMU_EDMP_RAM_GAF_DEFS_H__
#define __INV_IMU_EDMP_RAM_GAF_DEFS_H__

#ifdef __cplusplus
extern "C" {
#endif

#define RAM_GAF_PRGM_BASE	0x500
#define RAM_GAF_DATA_BASE	0x1620
#define RAM_GAF_DATA_SIZE	0x3E0
#define RAM_GAF_STACK_END	0x500
#define RAM_GAF_FIFO_FRAME_ID_QUAT_START	0x1
#define RAM_GAF_FIFO_FRAME_ID_QUAT_END	0x2
#define RAM_GAF_FIFO_FRAME_ID_BIAS_START	0x3
#define RAM_GAF_FIFO_FRAME_ID_BIAS_END	0x4


#ifdef __cplusplus
}
#endif

#endif // __INV_IMU_EDMP_RAM_GAF_DEFS_H__
