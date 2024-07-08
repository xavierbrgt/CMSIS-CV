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

// q2_13_t;

#define HORIZONTAL_COMPUTE_SCALAR_SOBEL(data_0, data_1, data_2) (data_0 + (data_1 * 2) + data_2)
#define VERTICAL_COMPUTE_SCALAR_SOBEL(data_0, data_1, data_2) (-(data_0 << 5) + (data_2 << 5))

#if defined(ARM_MATH_MVEI) && !defined(ARM_MATH_AUTOVECTORIZE)

#define VERTICAL_COMPUTE_VECTOR_SOBEL(vect_1, vect_2, vect_3, vect_res)                                                \
    int16x8x2_t vect_3x2;                                                                                              \
    (void)vect_2;                                                                                                      \
    vect_res.val[0] = vshllbq(vect_1, 5);                                                                              \
    vect_3x2.val[0] = vshllbq(vect_3, 5);                                                                              \
    vect_res.val[0] = vsubq(vect_3x2.val[0], vect_res.val[0]);                                                         \
    vect_res.val[1] = vshlltq(vect_1, 5);                                                                              \
    vect_3x2.val[1] = vshlltq(vect_3, 5);                                                                              \
    vect_res.val[1] = vsubq(vect_3x2.val[1], vect_res.val[1]);

#define HORIZONTAL_COMPUTE_VECTOR_SOBEL(vect_1, vect_2, vect_3, vect_out)                                              \
    vect_1 = vaddq(vect_1, vect_3);                                                                                    \
    vect_2 = vshlq_n(vect_2, 1);                                                                                       \
    vect_out = vaddq(vect_2, vect_1);

#endif
/**
 * @brief      Return the scratch size for sobel x function
 *
 * @param[in]     width         The width of the image
 * @return		  Scratch size in bytes
 */
uint16_t arm_cv_get_scratch_size_sobel_x(int width)
{
    return(width*sizeof(q15_t));
}

/**     
 * @brief          Sobel filter computing the gradient on the x axis
 *
 * @param[in]      ImageIn     The input image
 * @param[out]     ImageOut    The output image
 * @param[in,out]  scratch     Buffer
 * @param[in]      borderType  Type of border to use, supported are Replicate Wrap and Reflect
 * 
 * @par Temporary buffer sizing:
 * 
 * Will use a temporary buffer to store intermediate values of gradient and magnitude.
 *
 * Size of temporary buffer is given by
 * arm_cv_get_scratch_size_sobel_x(int width)
 */
#if defined(ARM_MATH_MVEI) && !defined(ARM_MATH_AUTOVECTORIZE)

void arm_sobel_x(const arm_cv_image_gray8_t *imageIn, arm_cv_image_q15_t *imageOut, q15_t *scratch, int8_t borderType)
{
    int width = imageOut->width;
    int height = imageOut->height;
    uint8_t *dataIn = imageIn->pData;
    q15_t *dataOut = imageOut->pData;
    int offset[3];

    BORDER_OFFSET(offset, LEFT_TOP, height, borderType);
    for (int y = 0; y < width - 15; y += 16)
    {
        uint8x16_t vec1 = vld1q(&dataIn[y + offset[0] * width]);
        uint8x16_t vec2 = vld1q(&dataIn[y + offset[1] * width]);
        uint8x16_t vec3 = vld1q(&dataIn[y + offset[2] * width]);
        // int16x8x2_t vect_res = VerticalCompute(vec1,vec2,vec3);
        int16x8x2_t vect_res;
        VERTICAL_COMPUTE_VECTOR_SOBEL(vec1, vec2, vec3, vect_res)
        vst2q(&scratch[y], vect_res);
    }
    for (int y = width - (width % 16); y < width; y++)
    {
        scratch[y] = VERTICAL_COMPUTE_SCALAR_SOBEL(dataIn[y + offset[0] * width], dataIn[y + offset[1] * width],
                                                   dataIn[y + offset[2] * width]);
    }
    BORDER_OFFSET(offset, LEFT_TOP, width, borderType);
    dataOut[0] = HORIZONTAL_COMPUTE_SCALAR_SOBEL(scratch[offset[0]], scratch[offset[1]], scratch[offset[2]]);
    BORDER_OFFSET(offset, MIDDLE, width, borderType);
    for (int y = 1; y < width - 8; y += 8)
    {
        int16x8_t vec1 = vld1q(&scratch[y + offset[0]]);
        int16x8_t vec2 = vld1q(&scratch[y + offset[1]]);
        int16x8_t vec3 = vld1q(&scratch[y + offset[2]]);
        // int8x16_t vect_out = HorizonCompute(vec1,vec2,vec3);
        int16x8_t vect_out;
        HORIZONTAL_COMPUTE_VECTOR_SOBEL(vec1, vec2, vec3, vect_out)
        vst1q(&dataOut[y], vect_out);
    }
    for (int y = width - ((width - 1) % 8); y < width - 1; y++)
    {
        dataOut[y] =
            HORIZONTAL_COMPUTE_SCALAR_SOBEL(scratch[y + offset[0]], scratch[y + offset[1]], scratch[y + offset[2]]);
    }
    BORDER_OFFSET(offset, RIGHT_BOT, width, borderType);
    dataOut[width - 1] = HORIZONTAL_COMPUTE_SCALAR_SOBEL(scratch[width - 1 + offset[0]], scratch[width - 1 + offset[1]],
                                                         scratch[width - 1 + offset[2]]);

    for (int x = 1; x < height - 1; x++)
    {
        BORDER_OFFSET(offset, MIDDLE, height, borderType);
        for (int y = 0; y < width - 15; y += 16)
        {
            uint8x16_t vec1 = vld1q(&dataIn[x * width + y + offset[0] * width]);
            uint8x16_t vec2 = vld1q(&dataIn[x * width + y + offset[1] * width]);
            uint8x16_t vec3 = vld1q(&dataIn[x * width + y + offset[2] * width]);
            int16x8x2_t vect_res;
            VERTICAL_COMPUTE_VECTOR_SOBEL(vec1, vec2, vec3, vect_res);
            vst2q(&scratch[y], vect_res);
        }
        for (int y = width - (width % 16); y < width; y++)
        {
            scratch[y] = VERTICAL_COMPUTE_SCALAR_SOBEL(dataIn[x * width + y + offset[0] * width],
                                                       dataIn[x * width + y + offset[1] * width],
                                                       dataIn[x * width + y + offset[2] * width]);
        }
        BORDER_OFFSET(offset, LEFT_TOP, width, borderType);
        dataOut[x * width] =
            HORIZONTAL_COMPUTE_SCALAR_SOBEL(scratch[offset[0]], scratch[offset[1]], scratch[offset[2]]);
        BORDER_OFFSET(offset, MIDDLE, width, borderType);
        for (int y = 1; y < width - 8; y += 8)
        {
            int16x8_t vec1 = vld1q(&scratch[y + offset[0]]);
            int16x8_t vec2 = vld1q(&scratch[y + offset[1]]);
            int16x8_t vec3 = vld1q(&scratch[y + offset[2]]);
            // int8x16_t vect_out = HorizonCompute(vec1,vec2,vec3);
            int16x8_t vect_out;
            HORIZONTAL_COMPUTE_VECTOR_SOBEL(vec1, vec2, vec3, vect_out)
            vst1q(&dataOut[x * width + y], vect_out);
        }
        for (int y = width - ((width - 1) % 8); y < width - 1; y++)
        {
            dataOut[x * width + y] =
                HORIZONTAL_COMPUTE_SCALAR_SOBEL(scratch[y + offset[0]], scratch[y + offset[1]], scratch[y + offset[2]]);
        }
        BORDER_OFFSET(offset, RIGHT_BOT, width, borderType);
        dataOut[x * width + width - 1] = HORIZONTAL_COMPUTE_SCALAR_SOBEL(
            scratch[width - 1 + offset[0]], scratch[width - 1 + offset[1]], scratch[width - 1 + offset[2]]);
    }

    int x = height - 1;
    BORDER_OFFSET(offset, RIGHT_BOT, height, borderType);
    for (int y = 0; y < width - 15; y += 16)
    {
        uint8x16_t vec1 = vld1q(&dataIn[x * width + y + offset[0] * width]);
        uint8x16_t vec2 = vld1q(&dataIn[x * width + y + offset[1] * width]);
        uint8x16_t vec3 = vld1q(&dataIn[x * width + y + offset[2] * width]);
        // int16x8x2_t vect_res = VerticalCompute(vec1,vec2,vec3);
        int16x8x2_t vect_res;
        VERTICAL_COMPUTE_VECTOR_SOBEL(vec1, vec2, vec3, vect_res);
        vst2q(&scratch[y], vect_res);
    }
    for (int y = width - (width % 16); y < width; y++)
    {
        scratch[y] = VERTICAL_COMPUTE_SCALAR_SOBEL(dataIn[x * width + y + offset[0] * width],
                                                   dataIn[x * width + y + offset[1] * width],
                                                   dataIn[x * width + y + offset[2] * width]);
    }
    BORDER_OFFSET(offset, LEFT_TOP, width, borderType);
    dataOut[x * width] = HORIZONTAL_COMPUTE_SCALAR_SOBEL(scratch[offset[0]], scratch[offset[1]], scratch[offset[2]]);
    BORDER_OFFSET(offset, MIDDLE, width, borderType);
    for (int y = 1; y < width - 8; y += 8)
    {
        int16x8_t vec1 = vld1q(&scratch[y + offset[0]]);
        int16x8_t vec2 = vld1q(&scratch[y + offset[1]]);
        int16x8_t vec3 = vld1q(&scratch[y + offset[2]]);
        // int8x16_t vect_out = HorizonCompute(vec1,vec2,vec3);
        int16x8_t vect_out;
        HORIZONTAL_COMPUTE_VECTOR_SOBEL(vec1, vec2, vec3, vect_out)
        vst1q(&dataOut[x * width + y], vect_out);
    }
    for (int y = width - ((width - 1) % 8); y < width - 1; y++)
    {
        dataOut[x * width + y] =
            HORIZONTAL_COMPUTE_SCALAR_SOBEL(scratch[y + offset[0]], scratch[y + offset[1]], scratch[y + offset[2]]);
    }
    BORDER_OFFSET(offset, RIGHT_BOT, width, borderType);
    dataOut[x * width + width - 1] = HORIZONTAL_COMPUTE_SCALAR_SOBEL(
        scratch[width - 1 + offset[0]], scratch[width - 1 + offset[1]], scratch[width - 1 + offset[2]]);
}

#else
void arm_sobel_x(const arm_cv_image_gray8_t *imageIn, arm_cv_image_q15_t *imageOut, q15_t *scratch, int8_t borderType)
{
    int width = imageOut->width;
    int height = imageOut->height;
    uint8_t *dataIn = imageIn->pData;
    q15_t *dataOut = imageOut->pData;
    int offset[3];

    BORDER_OFFSET(offset, LEFT_TOP, height, borderType);
    for (int y = 0; y < width; y++)
    {
        scratch[y] = VERTICAL_COMPUTE_SCALAR_SOBEL(dataIn[y + offset[0] * width], dataIn[y + offset[1] * width],
                                                   dataIn[y + offset[2] * width]);
    }
    BORDER_OFFSET(offset, LEFT_TOP, width, borderType);
    dataOut[0] = HORIZONTAL_COMPUTE_SCALAR_SOBEL(scratch[offset[0]], scratch[offset[1]], scratch[offset[2]]);
    BORDER_OFFSET(offset, MIDDLE, width, borderType);
    for (int y = 1; y < width - 1; y++)
    {
        dataOut[y] =
            HORIZONTAL_COMPUTE_SCALAR_SOBEL(scratch[y + offset[0]], scratch[y + offset[1]], scratch[y + offset[2]]);
    }
    BORDER_OFFSET(offset, RIGHT_BOT, width, borderType);
    dataOut[width - 1] = HORIZONTAL_COMPUTE_SCALAR_SOBEL(scratch[width - 1 + offset[0]], scratch[width - 1 + offset[1]],
                                                         scratch[width - 1 + offset[2]]);
    for (int x = 1; x < height - 1; x++)
    {
        BORDER_OFFSET(offset, MIDDLE, height, borderType);
        for (int y = 0; y < width; y++)
        {
            scratch[y] = VERTICAL_COMPUTE_SCALAR_SOBEL(dataIn[x * width + y + offset[0] * width],
                                                       dataIn[x * width + y + offset[1] * width],
                                                       dataIn[x * width + y + offset[2] * width]);
        }
        BORDER_OFFSET(offset, LEFT_TOP, width, borderType);
        dataOut[x * width] =
            HORIZONTAL_COMPUTE_SCALAR_SOBEL(scratch[offset[0]], scratch[offset[1]], scratch[offset[2]]);
        BORDER_OFFSET(offset, MIDDLE, width, borderType);
        for (int y = 1; y < width - 1; y++)
        {
            dataOut[x * width + y] =
                HORIZONTAL_COMPUTE_SCALAR_SOBEL(scratch[y + offset[0]], scratch[y + offset[1]], scratch[y + offset[2]]);
        }
        BORDER_OFFSET(offset, RIGHT_BOT, width, borderType);
        dataOut[x * width + width - 1] = HORIZONTAL_COMPUTE_SCALAR_SOBEL(
            scratch[width - 1 + offset[0]], scratch[width - 1 + offset[1]], scratch[width - 1 + offset[2]]);
    }
    int x = height - 1;
    BORDER_OFFSET(offset, RIGHT_BOT, height, borderType);
    for (int y = 0; y < width; y++)
    {
        scratch[y] = VERTICAL_COMPUTE_SCALAR_SOBEL(dataIn[x * width + y + offset[0] * width],
                                                   dataIn[x * width + y + offset[1] * width],
                                                   dataIn[x * width + y + offset[2] * width]);
    }
    BORDER_OFFSET(offset, LEFT_TOP, width, borderType);
    dataOut[x * width] = HORIZONTAL_COMPUTE_SCALAR_SOBEL(scratch[offset[0]], scratch[offset[1]], scratch[offset[2]]);
    BORDER_OFFSET(offset, MIDDLE, width, borderType);
    for (int y = 1; y < width - 1; y++)
    {
        dataOut[x * width + y] =
            HORIZONTAL_COMPUTE_SCALAR_SOBEL(scratch[y + offset[0]], scratch[y + offset[1]], scratch[y + offset[2]]);
    }
    BORDER_OFFSET(offset, RIGHT_BOT, width, borderType);
    dataOut[x * width + width - 1] = HORIZONTAL_COMPUTE_SCALAR_SOBEL(
        scratch[width - 1 + offset[0]], scratch[width - 1 + offset[1]], scratch[width - 1 + offset[2]]);
}
#endif
