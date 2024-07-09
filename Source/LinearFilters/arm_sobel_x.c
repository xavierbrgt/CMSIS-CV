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

//Convert from uint8 to q2 13
#define U8_TO_Q2_13(a) ((a) << 5)

//Apply a [1,2,1] kernel in the horizontal direction on the input image
#define HORIZONTAL_COMPUTE_SCALAR_SOBEL_X(data_0, data_1, data_2) (data_0 + (data_1 * 2) + data_2)
//Apply a [-1,0,1] kernel in the vertical direction on the input image, plus a conversion to q2_13
#define VERTICAL_COMPUTE_SCALAR_SOBEL_X(data_0, data_1, data_2) (-U8_TO_Q2_13(data_0) + U8_TO_Q2_13(data_2))

//Process a line on the input image
//borderLocation allow to modulate the macro to treat the three possible cases
#define LINE_PROCESSING_SCALAR_SOBEL_X_3(borderLocation, width, scratch, dataIn, offset, borderType, line, height)     \
    BORDER_OFFSET(offset, borderLocation, height, borderType);                                                         \
    for (int y = 0; y < width; y++)                                                                                    \
    {                                                                                                                  \
        scratch[y] = VERTICAL_COMPUTE_SCALAR_SOBEL_X(dataIn[line * width + y + offset[0] * width],                     \
                                                     dataIn[line * width + y + offset[1] * width],                     \
                                                     dataIn[line * width + y + offset[2] * width]);                    \
    }                                                                                                                  \
    BORDER_OFFSET(offset, LEFT_TOP, width, borderType);                                                                \
    dataOut[line * width] =                                                                                            \
        HORIZONTAL_COMPUTE_SCALAR_SOBEL_X(scratch[offset[0]], scratch[offset[1]], scratch[offset[2]]);                 \
    BORDER_OFFSET(offset, MIDDLE, width, borderType);                                                                  \
    for (int y = 1; y < width - 1; y++)                                                                                \
    {                                                                                                                  \
        dataOut[line * width + y] =                                                                                    \
            HORIZONTAL_COMPUTE_SCALAR_SOBEL_X(scratch[y + offset[0]], scratch[y + offset[1]], scratch[y + offset[2]]); \
    }                                                                                                                  \
    BORDER_OFFSET(offset, RIGHT_BOT, width, borderType);                                                               \
    dataOut[line * width + width - 1] = HORIZONTAL_COMPUTE_SCALAR_SOBEL_X(                                             \
        scratch[width - 1 + offset[0]], scratch[width - 1 + offset[1]], scratch[width - 1 + offset[2]]);

#if defined(ARM_MATH_MVEI) && !defined(ARM_MATH_AUTOVECTORIZE)

//Apply a [1,0,-1] kernel in the vertical direction on the input image, plus a conversion to q2_13 using vectors
#define VERTICAL_COMPUTE_VECTOR_SOBEL_X(vect_1, vect_3, vect_res)                                                      \
    int16x8x2_t vect_3x2;                                                                                              \
    vect_res.val[0] = vshllbq(vect_1, 5);                                                                              \
    vect_3x2.val[0] = vshllbq(vect_3, 5);                                                                              \
    vect_res.val[0] = vsubq(vect_3x2.val[0], vect_res.val[0]);                                                         \
    vect_res.val[1] = vshlltq(vect_1, 5);                                                                              \
    vect_3x2.val[1] = vshlltq(vect_3, 5);                                                                              \
    vect_res.val[1] = vsubq(vect_3x2.val[1], vect_res.val[1]);

//Apply a [1,2,1] kernel in the horizontal direction on the input image using vectors
#define HORIZONTAL_COMPUTE_VECTOR_SOBEL_X(vect_1, vect_2, vect_3, vect_out)                                            \
    vect_1 = vaddq(vect_1, vect_3);                                                                                    \
    vect_2 = vshlq_n(vect_2, 1);                                                                                       \
    vect_out = vaddq(vect_2, vect_1);


//Process a line on the input image using vectors
//borderLocation allow to modulate the macro to treat the three possible cases
#define LINE_PROCESSING_VECTOR_SOBEL_X_3(borderLocation, width, scratch, dataIn, offset, borderType, line, height)     \
    BORDER_OFFSET(offset, borderLocation, height, borderType);                                                         \
    for (int y = 0; y < width - 15; y += 16)                                                                           \
    {                                                                                                                  \
        uint8x16_t vec1 = vld1q(&dataIn[line * width + y + offset[0] * width]);                                        \
        uint8x16_t vec3 = vld1q(&dataIn[line * width + y + offset[2] * width]);                                        \
        int16x8x2_t vect_res;                                                                                          \
        VERTICAL_COMPUTE_VECTOR_SOBEL_X(vec1, vec3, vect_res);                                                         \
        vst2q(&scratch[y], vect_res);                                                                                  \
    }                                                                                                                  \
    for (int y = width - (width % 16); y < width; y++)                                                                 \
    {                                                                                                                  \
        scratch[y] = VERTICAL_COMPUTE_SCALAR_SOBEL_X(dataIn[line * width + y + offset[0] * width],                     \
                                                     dataIn[line * width + y + offset[1] * width],                     \
                                                     dataIn[line * width + y + offset[2] * width]);                    \
    }                                                                                                                  \
    BORDER_OFFSET(offset, LEFT_TOP, width, borderType);                                                                \
    dataOut[line * width] =                                                                                            \
        HORIZONTAL_COMPUTE_SCALAR_SOBEL_X(scratch[offset[0]], scratch[offset[1]], scratch[offset[2]]);                 \
    BORDER_OFFSET(offset, MIDDLE, width, borderType);                                                                  \
    for (int y = 1; y < width - 8; y += 8)                                                                             \
    {                                                                                                                  \
        int16x8_t vec1 = vld1q(&scratch[y + offset[0]]);                                                               \
        int16x8_t vec2 = vld1q(&scratch[y + offset[1]]);                                                               \
        int16x8_t vec3 = vld1q(&scratch[y + offset[2]]);                                                               \
        int16x8_t vect_out;                                                                                            \
        HORIZONTAL_COMPUTE_VECTOR_SOBEL_X(vec1, vec2, vec3, vect_out)                                                  \
        vst1q(&dataOut[line * width + y], vect_out);                                                                   \
    }                                                                                                                  \
    for (int y = width - ((width - 1) % 8); y < width - 1; y++)                                                        \
    {                                                                                                                  \
        dataOut[line * width + y] =                                                                                    \
            HORIZONTAL_COMPUTE_SCALAR_SOBEL_X(scratch[y + offset[0]], scratch[y + offset[1]], scratch[y + offset[2]]); \
    }                                                                                                                  \
    BORDER_OFFSET(offset, RIGHT_BOT, width, borderType);                                                               \
    dataOut[line * width + width - 1] = HORIZONTAL_COMPUTE_SCALAR_SOBEL_X(                                             \
        scratch[width - 1 + offset[0]], scratch[width - 1 + offset[1]], scratch[width - 1 + offset[2]]);

#endif
/**
  @ingroup linearFilter
 */

/**
 * @brief      Return the scratch size for sobel x function
 *
 * @param[in]     width         The width of the image
 * @return		  Scratch size in bytes
 */
uint16_t arm_cv_get_scratch_size_sobel_x(int width)
{
    return (width * sizeof(q15_t));
}

/**
  @ingroup linearFilter
 */

/**
 * @brief          Sobel filter computing the gradient on the x axis
 *
 * @param[in]      imageIn     The input image
 * @param[out]     imageOut    The output image
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

    LINE_PROCESSING_VECTOR_SOBEL_X_3(LEFT_TOP, width, scratch, dataIn, offset, borderType, 0, height)
    for (int x = 1; x < height - 1; x++)
    {
        LINE_PROCESSING_VECTOR_SOBEL_X_3(MIDDLE, width, scratch, dataIn, offset, borderType, x, height)
    }

    int x = height - 1;
    LINE_PROCESSING_VECTOR_SOBEL_X_3(RIGHT_BOT, width, scratch, dataIn, offset, borderType, x, height)
}

#else
void arm_sobel_x(const arm_cv_image_gray8_t *imageIn, arm_cv_image_q15_t *imageOut, q15_t *scratch, int8_t borderType)
{
    int width = imageOut->width;
    int height = imageOut->height;
    uint8_t *dataIn = imageIn->pData;
    q15_t *dataOut = imageOut->pData;
    int offset[3];

    LINE_PROCESSING_SCALAR_SOBEL_X_3(LEFT_TOP, width, scratch, dataIn, offset, borderType, 0, height)
    for (int x = 1; x < height - 1; x++)
    {
        LINE_PROCESSING_SCALAR_SOBEL_X_3(MIDDLE, width, scratch, dataIn, offset, borderType, x, height)
    }
    int x = height - 1;
    LINE_PROCESSING_SCALAR_SOBEL_X_3(RIGHT_BOT, width, scratch, dataIn, offset, borderType, x, height)
}
#endif
