/* ----------------------------------------------------------------------
 * Project:      CMSIS CV Library
 * Title:        arm_sobel_x.c
 * Description:  Sobel filter on x axis filter CMSIS-CV
 *
 *
 * Target Processor: Cortex-M and Cortex-A cores
 * -------------------------------------------------------------------- */
/*
 * Copyright (C) 2014 ARM Limited or its affiliates. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "arm_linear_filter_common.h"
#include "cv/linear_filters.h"
#include "dsp/basic_math_functions.h"
#include <stdio.h>

// The kernel applied by this filter is [-1,-2,-1]
//                                      [ 0, 0, 0]
//                                      [ 1, 2, 1]

// Apply a [1,2,1] kernel in the horizontal direction on the input image
#define HORIZONTAL_COMPUTE_SCALAR(data_0, data_1, data_2) (data_0 + (data_1 * 2) + data_2)
// Apply a [-1,0,1] kernel in the vertical direction on the input image
#define VERTICAL_COMPUTE_SCALAR(data_0, data_1, data_2) (-(data_0) + (data_2))

#if defined(ARM_MATH_MVEI) && !defined(ARM_MATH_AUTOVECTORIZE)

// Apply a [1,0,-1] kernel in the vertical direction on the input image
#define VERTICAL_COMPUTE_VECTOR(vect_1, vec2, vect_3, vect_res)                                                        \
    (void)vec2;                                                                                                        \
    int16x8x2_t vect_3x2;                                                                                              \
    vect_res.val[0] = vmovlbq(vect_1);                                                                                 \
    vect_3x2.val[0] = vmovlbq(vect_3);                                                                                 \
    vect_res.val[0] = vsubq(vect_3x2.val[0], vect_res.val[0]);                                                         \
    vect_res.val[1] = vmovltq(vect_1);                                                                                 \
    vect_3x2.val[1] = vmovltq(vect_3);                                                                                 \
    vect_res.val[1] = vsubq(vect_3x2.val[1], vect_res.val[1]);

// Apply a [1,2,1] kernel in the horizontal direction on the input image using vectors
#define HORIZONTAL_COMPUTE_VECTOR(vect_1, vect_2, vect_3, vect_out)                                                    \
    vect_1 = vaddq(vect_1, vect_3);                                                                                    \
    vect_2 = vshlq_n(vect_2, 1);                                                                                       \
    vect_out = vaddq(vect_2, vect_1);

#endif

/**
  @ingroup linearFilter
 */

/**
 * @brief      Return the scratch size for sobels functions
 *
 * @param[in]     width         The width of the image
 * @return		  Scratch size in bytes
 */
uint16_t arm_cv_get_scratch_size_sobel(int width)
{
    return (width * sizeof(q15_t));
}

#define INPUT INPUT_8
#define OUTPUT OUTPUT_16
#include "arm_linear_filter_common.c"
void arm_sobel_x(const arm_cv_image_gray8_t *imageIn, arm_cv_image_q15_t *imageOut, q15_t *scratch,
                 const int8_t borderType)
{
    LINEAR_GENERIC(imageIn, imageOut, scratch, borderType)
}
