/* ----------------------------------------------------------------------
 * Project:      CMSIS CV Library
 * Title:        arm_generic_filter.c
 * Description:  Generic template for linear filter, this one doing a Gaussian
 * filter
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

// Convert the q15 value from the buffer back to an uint8 value
#define Q15_TO_U8(a) ((a) >> 10)
// Apply a kernel [1,2,1] to the input data
#define COMPUTE_SCALAR_GAUSSIAN_3(data_0, data_1, data_2) (data_0 * 0x08 + (data_1 * 0x10) + data_2 * 0x08)

// Do the processing of a line of the input image,
// borderLocation allow to treat the upper and lowest line of the image using the same macro
#define LINE_PROCESSING_SCALAR_GAUSSIAN_3(borderLocation, width, scratch, dataIn, offset, borderType, line, height)    \
    BORDER_OFFSET(offset, borderLocation, height, borderType);                                                         \
    for (int y = 0; y < width; y++)                                                                                    \
    {                                                                                                                  \
        scratch[y] = COMPUTE_SCALAR_GAUSSIAN_3(dataIn[line * width + y + offset[0] * width],                           \
                                               dataIn[line * width + y + offset[1] * width],                           \
                                               dataIn[line * width + y + offset[2] * width]);                          \
    }                                                                                                                  \
    BORDER_OFFSET(offset, LEFT_TOP, width, borderType);                                                                \
    dataOut[line * width] =                                                                                            \
        Q15_TO_U8(COMPUTE_SCALAR_GAUSSIAN_3(scratch[offset[0]], scratch[offset[1]], scratch[offset[2]]));              \
    BORDER_OFFSET(offset, MIDDLE, width, borderType);                                                                  \
    for (int y = 1; y < width - 1; y++)                                                                                \
    {                                                                                                                  \
        dataOut[line * width + y] = Q15_TO_U8(                                                                         \
            COMPUTE_SCALAR_GAUSSIAN_3(scratch[y + offset[0]], scratch[y + offset[1]], scratch[y + offset[2]]));        \
    }                                                                                                                  \
    BORDER_OFFSET(offset, RIGHT_BOT, width, borderType);                                                               \
    dataOut[line * width + width - 1] = Q15_TO_U8(COMPUTE_SCALAR_GAUSSIAN_3(                                           \
        scratch[width - 1 + offset[0]], scratch[width - 1 + offset[1]], scratch[width - 1 + offset[2]]));

#if defined(ARM_MATH_MVEI) && !defined(ARM_MATH_AUTOVECTORIZE)

// vec 1,2 and 3 are vector containing lines of the input image
// Apply a kernel [1,2,1] to the input value and convert to q15
#define VERTICAL_COMPUTE_VECTOR_GAUSSIAN(vect_1, vect_2, vect_3, vect_res)                                             \
    int16x8x2_t vect_1x2;                                                                                              \
    int16x8x2_t vect_3x2;                                                                                              \
    vect_1x2.val[0] = vshllbq(vect_1, 3);                                                                              \
    vect_3x2.val[0] = vshllbq(vect_3, 3);                                                                              \
    vect_1x2.val[0] = vaddq(vect_1x2.val[0], vect_3x2.val[0]);                                                         \
    vect_1x2.val[1] = vshlltq(vect_1, 3);                                                                              \
    vect_3x2.val[1] = vshlltq(vect_3, 3);                                                                              \
    vect_1x2.val[1] = vaddq(vect_1x2.val[1], vect_3x2.val[1]);                                                         \
    vect_res.val[0] = vshllbq(vect_2, 4);                                                                              \
    vect_res.val[0] = vaddq(vect_res.val[0], vect_1x2.val[0]);                                                         \
    vect_res.val[1] = vshlltq(vect_2, 4);                                                                              \
    vect_res.val[1] = vaddq(vect_res.val[1], vect_1x2.val[1]);

// vec 1,2 and 3 are vector containing columns of the input image
// Apply a kernel [1,2,1] to the input values and convert the result back to uint8 format
#define HORIZONTAL_COMPUTE_VECTOR_GAUSSIAN(vect_1, vect_2, vect_3, vect_out)                                           \
    vect_out = vdupq_n_s16(0);                                                                                         \
    vect_2.val[0] = vshlq_n_s16(vect_2.val[0], 1);                                                                     \
    vect_2.val[0] = vaddq_s16(vect_2.val[0], vect_1.val[0]);                                                           \
    vect_2.val[1] = vshlq_n_s16(vect_2.val[1], 1);                                                                     \
    vect_2.val[0] = vaddq_s16(vect_2.val[0], vect_3.val[0]);                                                           \
    vect_2.val[0] = vshrq(vect_2.val[0], 7);                                                                           \
    vect_2.val[1] = vaddq_s16(vect_2.val[1], vect_1.val[1]);                                                           \
    vect_2.val[1] = vaddq_s16(vect_2.val[1], vect_3.val[1]);                                                           \
    vect_2.val[1] = vshrq(vect_2.val[1], 7);                                                                           \
    vect_out = vmovntq(vect_out, vect_2.val[1]);                                                                       \
    vect_out = vmovnbq(vect_out, vect_2.val[0]);

// Process one line of the input image using vectors and tails
// borderLocation allow to treat the upper and lowest line of the image using the same macro
#define LINE_PROCESSING_VECTOR_GAUSSIAN_3(borderLocation, width, scratch, dataIn, offset, borderType, line, height)    \
    BORDER_OFFSET(offset, borderLocation, height, borderType);                                                         \
    for (int y = 0; y < width - 15; y += 16)                                                                           \
    {                                                                                                                  \
        uint8x16_t vec1 = vld1q(&dataIn[line * width + y + offset[0] * width]);                                        \
        uint8x16_t vec2 = vld1q(&dataIn[line * width + y + offset[1] * width]);                                        \
        uint8x16_t vec3 = vld1q(&dataIn[line * width + y + offset[2] * width]);                                        \
        int16x8x2_t vect_res;                                                                                          \
        VERTICAL_COMPUTE_VECTOR_GAUSSIAN(vec1, vec2, vec3, vect_res);                                                  \
        vst2q(&scratch[y], vect_res);                                                                                  \
    }                                                                                                                  \
    for (int y = width - (width % 16); y < width; y++)                                                                 \
    {                                                                                                                  \
        scratch[y] = COMPUTE_SCALAR_GAUSSIAN_3(dataIn[line * width + y + offset[0] * width],                           \
                                               dataIn[line * width + y + offset[1] * width],                           \
                                               dataIn[line * width + y + offset[2] * width]);                          \
    }                                                                                                                  \
    BORDER_OFFSET(offset, LEFT_TOP, width, borderType);                                                                \
    dataOut[line * width] =                                                                                            \
        Q15_TO_U8(COMPUTE_SCALAR_GAUSSIAN_3(scratch[offset[0]], scratch[offset[1]], scratch[offset[2]]));              \
    BORDER_OFFSET(offset, MIDDLE, width, borderType);                                                                  \
    for (int y = 1; y < width - 16; y += 16)                                                                           \
    {                                                                                                                  \
        int16x8x2_t vec1 = vld2q(&scratch[y + offset[0]]);                                                             \
        int16x8x2_t vec2 = vld2q(&scratch[y + offset[1]]);                                                             \
        int16x8x2_t vec3 = vld2q(&scratch[y + offset[2]]);                                                             \
        int8x16_t vect_out;                                                                                            \
        HORIZONTAL_COMPUTE_VECTOR_GAUSSIAN(vec1, vec2, vec3, vect_out)                                                 \
        vst1q((int8_t *)&dataOut[line * width + y], vect_out);                                                         \
    }                                                                                                                  \
    for (int y = width - ((width - 1) % 16); y < width - 1; y++)                                                       \
    {                                                                                                                  \
        dataOut[line * width + y] = Q15_TO_U8(                                                                         \
            COMPUTE_SCALAR_GAUSSIAN_3(scratch[y + offset[0]], scratch[y + offset[1]], scratch[y + offset[2]]));        \
    }                                                                                                                  \
    BORDER_OFFSET(offset, RIGHT_BOT, width, borderType);                                                               \
    dataOut[line * width + width - 1] = Q15_TO_U8(COMPUTE_SCALAR_GAUSSIAN_3(                                           \
        scratch[width - 1 + offset[0]], scratch[width - 1 + offset[1]], scratch[width - 1 + offset[2]]));

#endif

/**
  @ingroup linearFilter
 */

/**
 * @brief      Return the scratch size for generic gaussian function
 *
 * @param[in]     width        The width of the image in pixels
 * @return		  Scratch size in bytes
 */
uint16_t arm_cv_get_scratch_size_gaussian_generic(int width)
{
    return (width * sizeof(q15_t));
}

/**
  @ingroup linearFilter
 */

/**
 * @brief          Generic 2D linear filter for grayscale data computing in q15, doing a gaussian
 *
 * @param[in]      imageIn     The input image
 * @param[out]     imageOut    The output image
 * @param[in,out]  scratch     Temporary buffer
 * @param[in]      borderType  Type of border to use, supported are Replicate Wrap and Reflect
 *
 * @par Temporary buffer sizing:
 *
 * Will use a temporary buffer to store intermediate values of gradient and magnitude.
 *
 * Size of temporary buffer is given by
 * arm_cv_get_scratch_size_gaussian_generic(int width)
 */
#if defined(ARM_MATH_MVEI) && !defined(ARM_MATH_AUTOVECTORIZE)
void arm_gaussian_generic_3x3_fixp(const arm_cv_image_gray8_t *imageIn, arm_cv_image_gray8_t *imageOut, q15_t *scratch,
                                   int8_t borderType)
{
    int width = imageOut->width;
    int height = imageOut->height;
    uint8_t *dataIn = imageIn->pData;
    uint8_t *dataOut = imageOut->pData;
    int offset[3];

    LINE_PROCESSING_VECTOR_GAUSSIAN_3(LEFT_TOP, width, scratch, dataIn, offset, borderType, 0, height)
    for (int x = 1; x < height - 1; x++)
    {
        LINE_PROCESSING_VECTOR_GAUSSIAN_3(MIDDLE, width, scratch, dataIn, offset, borderType, x, height)
    }
    int x = height - 1;
    LINE_PROCESSING_VECTOR_GAUSSIAN_3(RIGHT_BOT, width, scratch, dataIn, offset, borderType, x, height)
}

#else
void arm_gaussian_generic_3x3_fixp(const arm_cv_image_gray8_t *imageIn, arm_cv_image_gray8_t *imageOut, q15_t *scratch,
                                   int8_t borderType)
{
    int width = imageOut->width;
    int height = imageOut->height;
    uint8_t *dataIn = imageIn->pData;
    uint8_t *dataOut = imageOut->pData;
    int offset[3];

    LINE_PROCESSING_SCALAR_GAUSSIAN_3(LEFT_TOP, width, scratch, dataIn, offset, borderType, 0, height)
    for (int x = 1; x < height - 1; x++)
    {
        LINE_PROCESSING_SCALAR_GAUSSIAN_3(MIDDLE, width, scratch, dataIn, offset, borderType, x, height)
    }
    int x = height - 1;
    LINE_PROCESSING_SCALAR_GAUSSIAN_3(RIGHT_BOT, width, scratch, dataIn, offset, borderType, x, height)
}
#endif
