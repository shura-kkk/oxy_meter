/** \file algorithm.cpp ******************************************************
 *
 * Project: MAXREFDES117#
 * Filename: algorithm.cpp
 * Description: This module calculates the heart rate/SpO2 level
 *
 *
 * --------------------------------------------------------------------
 *
 * This code follows the following naming conventions:
 *
 * char              ch_pmod_value
 * char (array)      s_pmod_s_string[16]
 * float             f_pmod_value
 * int32_t           n_pmod_value
 * int32_t (array)   an_pmod_value[16]
 * int16_t           w_pmod_value
 * int16_t (array)   aw_pmod_value[16]
 * uint16_t          uw_pmod_value
 * uint16_t (array)  auw_pmod_value[16]
 * uint8_t           uch_pmod_value
 * uint8_t (array)   auch_pmod_buffer[16]
 * uint32_t          un_pmod_value
 * int32_t *         pn_pmod_value
 *
 * ------------------------------------------------------------------------- */
/*******************************************************************************
 * Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
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

#include <stdint.h>
#include <stdio.h>
#include "algorithm.h"

//uch_spo2_table is approximated as  -45.060*ratioAverage* ratioAverage + 30.354 *ratioAverage + 94.845 ;
// const static float uch_spo2_table[184]=
// {   94.845,95.144034,95.434056,95.715066,95.987064,96.25005,96.504024,96.748986,96.984936,97.211874,97.4298,97.638714,97.838616,98.029506,
//     98.211384,98.38425,98.548104,98.702946,98.848776,98.985594,99.1134,99.232194,99.341976,99.442746,99.534504,99.61725,99.690984,99.755706,
//     99.811416,99.858114,99.8958,99.924474,99.944136,99.954786,99.956424,99.94905,99.932664,99.907266,99.872856,99.829434,99.777,99.715554,
//     99.645096,99.565626,99.477144,99.37965,99.273144,99.157626,99.033096,98.899554,98.757,98.605434,98.444856,98.275266,98.096664,97.90905,
//     97.712424,97.506786,97.292136,97.068474,96.8358,96.594114,96.343416,96.083706,95.814984,95.53725,95.250504,94.954746,94.649976,94.336194,
//     94.0134,93.681594,93.340776,92.990946,92.632104,92.26425,91.887384,91.501506,91.106616,90.702714,90.2898,89.867874,89.436936,88.996986,
//     88.548024,88.09005,87.623064,87.147066,86.662056,86.168034,85.665,85.152954,84.631896,84.101826,83.562744,83.01465,82.457544,81.891426,
//     81.316296,80.732154,80.139,79.536834,78.925656,78.305466,77.676264,77.03805,76.390824,75.734586,75.069336,74.395074,73.7118,73.019514,
//     72.318216,71.607906,70.888584,70.16025,69.422904,68.676546,67.921176,67.156794,66.3834,65.600994,64.809576,64.009146,63.199704,62.38125,
//     61.553784,60.717306,59.871816,59.017314,58.1538,57.281274,56.399736,55.509186,54.609624,53.70105,52.783464,51.856866,50.921256,49.976634,
//     49.023,48.060354,47.088696,46.108026,45.118344,44.11965,43.111944,42.095226,41.069496,40.034754,38.991,37.938234,36.876456,35.805666,
//     34.725864,33.63705,32.539224,31.432386,30.316536,29.191674,28.0578,26.914914,25.763016,24.602106,23.432184,22.25325,21.065304,19.868346,
//     18.662376,17.447394,16.2234,14.990394,13.748376,12.497346,11.237304,9.96825,8.690184,7.403106,6.107016,4.801914,3.4878,2.164674,0.832536,
//     0.0
// };

void maxim_sort_ascend_float(
    float *pn_x,
    int32_t n_size);

void maxim_heart_rate_and_oxygen_saturation(
    uint32_t *pun_ir_buffer,
    int32_t n_ir_buffer_length,
    uint32_t *pun_red_buffer,
    float *pn_spo2,
    int8_t *pch_spo2_valid,
    int32_t *pn_heart_rate,
    int8_t *pch_hr_valid)
{
    uint32_t un_ir_mean;
    int32_t k, n_i_ratio_count;
    int32_t i, n_exact_ir_valley_locs_count, n_middle_idx;
    int32_t n_th1, n_npks;
    int32_t an_ir_valley_locs[15] ;
    int32_t n_peak_interval_sum;
    // static int32_t n_last_peak_interval=FS; // Initialize it to 25, which corresponds to heart rate of 60 bps, RF

    int32_t n_y_ac = 0, n_x_ac = 0;
    //  int32_t n_spo2_calc;
    int32_t n_y_dc_max = 0, n_x_dc_max = 0;
    int32_t n_y_dc_max_idx = 0, n_x_dc_max_idx = 0;
    float an_ratio[5], n_ratio_average;
    int32_t n_nume, n_denom ;
    int32_t an_x[ BUFFER_SIZE]; //ir
    int32_t an_y[ BUFFER_SIZE]; //red

    // calculates DC mean and subtracts DC from ir
    un_ir_mean =0;
    for (k=0 ; k<n_ir_buffer_length ; k++ ) un_ir_mean += pun_ir_buffer[k] ;
    un_ir_mean =un_ir_mean/n_ir_buffer_length ;

    // remove DC and invert signal so that we can use peak detector as valley detector
    for (k=0 ; k<n_ir_buffer_length ; k++ )
        an_x[k] = un_ir_mean - pun_ir_buffer[k] ;

    // 4 pt Moving Average
    for(k=0; k< BUFFER_SIZE_MA4; k++) {
        an_x[k]=( an_x[k]+an_x[k+1]+ an_x[k+2]+ an_x[k+3])/(int)4;
    }
    // calculate threshold
    n_th1=0;
    for ( k=0 ; k<BUFFER_SIZE_MA4 ; k++) {
        n_th1 +=  an_x[k];
    }
    n_th1= n_th1/ (BUFFER_SIZE_MA4);
    if( n_th1<30) n_th1=30; // min allowed
    if( n_th1>60) n_th1=60; // max allowed

    for ( k=0 ; k<15; k++) an_ir_valley_locs[k]=0;
    // since we flipped signal, we use peak detector as valley detector
    maxim_find_peaks( an_ir_valley_locs, &n_npks, an_x, BUFFER_SIZE_MA4, n_th1, 4, 15 );//peak_height, peak_distance, max_num_peaks
    n_peak_interval_sum =0;
    if (n_npks>=2) {
        for (k=1; k<n_npks; k++) n_peak_interval_sum += (an_ir_valley_locs[k] - an_ir_valley_locs[k -1] ) ;
        n_peak_interval_sum =n_peak_interval_sum/(n_npks-1);
        *pn_heart_rate =(int32_t)( (FS*60)/ n_peak_interval_sum );
        *pch_hr_valid  = 1;
    }
    else  {
        *pn_heart_rate = -999; // unable to calculate because # of peaks are too small
        *pch_hr_valid  = 0;
    }

    //  load raw value again for SPO2 calculation : RED(=y) and IR(=X)
    for (k=0 ; k<n_ir_buffer_length ; k++ )  {
        an_x[k] =  pun_ir_buffer[k] ;
        an_y[k] =  pun_red_buffer[k] ;
    }

    // find precise min near an_ir_valley_locs
    n_exact_ir_valley_locs_count = n_npks;

    //using exact_ir_valley_locs , find ir-red DC and ir-red AC for SPO2 calibration an_ratio
    //finding AC/DC maximum of raw

    n_ratio_average =0;
    n_i_ratio_count = 0;
    for(k=0; k< 5; k++) an_ratio[k]=0;
    for (k=0; k< n_exact_ir_valley_locs_count; k++) {
        if (an_ir_valley_locs[k] > BUFFER_SIZE ) {
            *pn_spo2 =  -999 ; // do not use SPO2 since valley loc is out of range
            *pch_spo2_valid  = 0;
            return;
        }
    }
    // find max between two valley locations
    // and use an_ratio betwen AC compoent of Ir & Red and DC compoent of Ir & Red for SPO2
    for (k=0; k< n_exact_ir_valley_locs_count-1; k++) {
        n_y_dc_max= -16777216 ;
        n_x_dc_max= -16777216;
        if (an_ir_valley_locs[k+1]-an_ir_valley_locs[k] >3) {
            for (i=an_ir_valley_locs[k]; i< an_ir_valley_locs[k+1]; i++) {
                if (an_x[i]> n_x_dc_max) {
                    n_x_dc_max =an_x[i];
                    n_x_dc_max_idx=i;
                }
                if (an_y[i]> n_y_dc_max) {
                    n_y_dc_max =an_y[i];
                    n_y_dc_max_idx=i;
                }
            }
            n_y_ac= (an_y[an_ir_valley_locs[k+1]] - an_y[an_ir_valley_locs[k] ] )*(n_y_dc_max_idx -an_ir_valley_locs[k]); //red
            n_y_ac=  an_y[an_ir_valley_locs[k]] + n_y_ac/ (an_ir_valley_locs[k+1] - an_ir_valley_locs[k])  ;
            n_y_ac=  an_y[n_y_dc_max_idx] - n_y_ac;    // subracting linear DC compoenents from raw
            n_x_ac= (an_x[an_ir_valley_locs[k+1]] - an_x[an_ir_valley_locs[k] ] )*(n_x_dc_max_idx -an_ir_valley_locs[k]); // ir
            n_x_ac=  an_x[an_ir_valley_locs[k]] + n_x_ac/ (an_ir_valley_locs[k+1] - an_ir_valley_locs[k]);
            n_x_ac=  an_x[n_y_dc_max_idx] - n_x_ac;      // subracting linear DC compoenents from raw
            n_nume=( n_y_ac *n_x_dc_max) ; //prepare X100 to preserve floating value
            n_denom= ( n_x_ac *n_y_dc_max);
            if (n_denom>0  && n_i_ratio_count <5 &&  n_nume != 0)
            {
                // an_ratio[n_i_ratio_count]= (n_nume * 100)/n_denom ; //formular is ( n_y_ac *n_x_dc_max) / ( n_x_ac *n_y_dc_max) ;
                an_ratio[n_i_ratio_count]= ((float)n_y_ac / (float)n_x_ac) * ((float)n_x_dc_max / (float)n_y_dc_max); //formular is ( n_y_ac *n_x_dc_max) / ( n_x_ac *n_y_dc_max) ;
                n_i_ratio_count++;
            }
        }
    }
    // choose median value since PPG signal may varies from beat to beat
    maxim_sort_ascend_float(an_ratio, n_i_ratio_count);
    n_middle_idx= n_i_ratio_count/2;

    if (n_middle_idx >1)
        n_ratio_average =( an_ratio[n_middle_idx-1] +an_ratio[n_middle_idx])/2; // use median
    else
        n_ratio_average = an_ratio[n_middle_idx ];

    *pn_spo2 = -45.060*n_ratio_average* n_ratio_average/10000 + 30.354 *n_ratio_average/100 + 94.845;
    if (*pn_spo2 >= 0 && *pn_spo2 <= 100) {
        *pch_spo2_valid = 1;
    } else {
        *pch_spo2_valid = 0;
    }
}

void maxim_find_peaks(
    int32_t *pn_locs,
    int32_t *n_npks,
    int32_t  *pn_x,
    int32_t n_size,
    int32_t n_min_height,
    int32_t n_min_distance,
    int32_t n_max_num )
{
    maxim_peaks_above_min_height( pn_locs, n_npks, pn_x, n_size, n_min_height );
    maxim_remove_close_peaks( pn_locs, n_npks, pn_x, n_min_distance );
    *n_npks = min( *n_npks, n_max_num );
}

void maxim_peaks_above_min_height(
    int32_t *pn_locs,
    int32_t *n_npks,
    int32_t *pn_x,
    int32_t n_size,
    int32_t n_min_height)
{
    int32_t i = 1, n_width;
    *n_npks = 0;

    while (i < n_size-1) {
        if (pn_x[i] > n_min_height && pn_x[i] > pn_x[i-1]) {     // find left edge of potential peaks
            n_width = 1;
            while (i+n_width < n_size && pn_x[i] == pn_x[i+n_width])  // find flat peaks
                n_width++;
            if (pn_x[i] > pn_x[i+n_width] && (*n_npks) < 15 ) {      // find right edge of peaks
                pn_locs[(*n_npks)++] = i;
                // for flat peaks, peak location is left edge
                i += n_width+1;
            } else
                i += n_width;
        }
        else
            i++;
    }
}

void maxim_remove_close_peaks(
    int32_t *pn_locs,
    int32_t *pn_npks,
    int32_t *pn_x,
    int32_t n_min_distance)
{

    int32_t i, j, n_old_npks, n_dist;

    /* Order peaks from large to small */
    maxim_sort_indices_descend( pn_x, pn_locs, *pn_npks );

    for ( i = -1; i < *pn_npks; i++ ) {
        n_old_npks = *pn_npks;
        *pn_npks = i+1;
        for ( j = i+1; j < n_old_npks; j++ ) {
            n_dist =  pn_locs[j] - ( i == -1 ? -1 : pn_locs[i] ); // lag-zero peak of autocorr is at index -1
            if ( n_dist > n_min_distance || n_dist < -n_min_distance )
                pn_locs[(*pn_npks)++] = pn_locs[j];
        }
    }

    // Resort indices int32_to ascending order
    maxim_sort_ascend( pn_locs, *pn_npks );
}

void maxim_sort_ascend(
    int32_t *pn_x,
    int32_t n_size)
{
    int32_t i, j, n_temp;
    for (i = 1; i < n_size; i++) {
        n_temp = pn_x[i];
        for (j = i; j > 0 && n_temp < pn_x[j-1]; j--)
            pn_x[j] = pn_x[j-1];
        pn_x[j] = n_temp;
    }
}

void maxim_sort_ascend_float(
    float *pn_x,
    int32_t n_size)
{
    int32_t i, j, n_temp;
    for (i = 1; i < n_size; i++) {
        n_temp = pn_x[i];
        for (j = i; j > 0 && n_temp < pn_x[j-1]; j--)
            pn_x[j] = pn_x[j-1];
        pn_x[j] = n_temp;
    }
}

void maxim_sort_indices_descend(
    int32_t *pn_x,
    int32_t *pn_indx,
    int32_t n_size)
{
    int32_t i, j, n_temp;
    for (i = 1; i < n_size; i++) {
        n_temp = pn_indx[i];
        for (j = i; j > 0 && pn_x[n_temp] > pn_x[pn_indx[j-1]]; j--)
            pn_indx[j] = pn_indx[j-1];
        pn_indx[j] = n_temp;
    }
}


