/** \file algorithm.h ******************************************************
 *
 * Project: MAXREFDES117#
 * Filename: algorithm.h
 * Description: This module is the heart rate/SpO2 calculation algorithm header file
 *
 * Revision History:
 *\n 1-18-2016 Rev 01.00 SK Initial release.
 *\n
 *
 * --------------------------------------------------------------------
 *
 * This code follows the following naming conventions:
 *
 *\n char              ch_pmod_value
 *\n char (array)      s_pmod_s_string[16]
 *\n float             f_pmod_value
 *\n int32_t           n_pmod_value
 *\n int32_t (array)   an_pmod_value[16]
 *\n int16_t           w_pmod_value
 *\n int16_t (array)   aw_pmod_value[16]
 *\n uint16_t          uw_pmod_value
 *\n uint16_t (array)  auw_pmod_value[16]
 *\n uint8_t           uch_pmod_value
 *\n uint8_t (array)   auch_pmod_buffer[16]
 *\n uint32_t          un_pmod_value
 *\n int32_t *         pn_pmod_value
 *
 * ------------------------------------------------------------------------- */
/*******************************************************************************
 * Copyright (C) 2015 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *******************************************************************************
 */
#pragma once

#include <stdint.h>

#define true 1
#define false 0
#define FS 25    //sampling frequency
#define BUFFER_SIZE  (FS* 4)
#define MA4_SIZE  4 // DONOT CHANGE
#define BUFFER_SIZE_MA4 BUFFER_SIZE-MA4_SIZE
#define min(x,y) ((x) < (y) ? (x) : (y))

/**
 * @brief        Calculate the heart rate and SpO2 level
 * @par          Details
 *               By detecting  peaks of PPG cycle and corresponding AC/DC of red/infra-red signal, the an_ratio for the SPO2 is computed.
 *               Since this algorithm is aiming for Arm M0/M3. formaula for SPO2 did not achieve the accuracy due to register overflow.
 *               Thus, accurate SPO2 is precalculated and save longo uch_spo2_table[] per each an_ratio.
 *
 * @param[in]    *pun_ir_buffer           - IR sensor data buffer
 * @param[in]    n_ir_buffer_length      - IR sensor data buffer length
 * @param[in]    *pun_red_buffer          - Red sensor data buffer
 * @param[out]    *pn_spo2                - Calculated SpO2 value
 * @param[out]    *pch_spo2_valid         - 1 if the calculated SpO2 value is valid
 * @param[out]    *pn_heart_rate          - Calculated heart rate value
 * @param[out]    *pch_hr_valid           - 1 if the calculated heart rate value is valid
 *
 * @retval       None
 */
void maxim_heart_rate_and_oxygen_saturation(
    uint32_t *pun_ir_buffer,
    int32_t n_ir_buffer_length,
    uint32_t *pun_red_buffer,
    float *pn_spo2,
    int8_t *pch_spo2_valid,
    int32_t *pn_heart_rate,
    int8_t *pch_hr_valid);

/**
 * @brief        Find peaks
 * @par          Details
 *               Find at most MAX_NUM peaks above MIN_HEIGHT separated by at least MIN_DISTANCE
 *
 * @retval       None
 */
void maxim_find_peaks(
    int32_t *pn_locs,
    int32_t *n_npks,
    int32_t *pn_x,
    int32_t n_size,
    int32_t n_min_height,
    int32_t n_min_distance,
    int32_t n_max_num);

/**
 * @brief        Find peaks above n_min_height
 * @par          Details
 *               Find all peaks above MIN_HEIGHT
 *
 * @retval       None
 */
void maxim_peaks_above_min_height(
    int32_t *pn_locs,
    int32_t *n_npks,
    int32_t *pn_x,
    int32_t n_size,
    int32_t n_min_height);

/**
 * @brief        Remove peaks
 * @par          Details
 *               Remove peaks separated by less than MIN_DISTANCE
 *
 * @retval       None
 */
void maxim_remove_close_peaks(
    int32_t *pn_locs,
    int32_t *pn_npks,
    int32_t *pn_x,
    int32_t n_min_distance);

/**
 * @brief        Sort array
 * @par          Details
 *               Sort array in ascending order (insertion sort algorithm)
 *
 * @retval       None
 */
void maxim_sort_ascend(
    int32_t *pn_x,
    int32_t n_size);

/**
 * @brief        Sort indices
 * @par          Details
 *               Sort indices according to descending order (insertion sort algorithm)
 *
 * @retval       None
 */
void maxim_sort_indices_descend(
    int32_t *pn_x,
    int32_t *pn_indx,
    int32_t n_size);
