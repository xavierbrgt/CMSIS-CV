/* ----------------------------------------------------------------------
 * Project:      CMSIS CV Library
 * Title:        arm_linear_filter_generator_5.c
 * Description:  common code used by different linear filter functions
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

#ifndef ARM_CV_LINEAR_FILTER_COMMON_C
#define ARM_CV_LINEAR_FILTER_COMMON_C
#include "arm_linear_filter_common.h"

#if ARM_CV_LINEAR_OUTPUT_TYPE == ARM_CV_LINEAR_OUTPUT_UINT_8
#define OUTPUT_TYPE uint8_t
#else
#define OUTPUT_TYPE q15_t
#endif

#if defined(ARM_MATH_MVEI) && !defined(ARM_MATH_AUTOVECTORIZE)

#define LOOP_INPUT_TO_BUFFER(width, scratch, dataIn, line, offset)                                                     \
    for (int y = 0; y < width - 15; y += 16)                                                                           \
    {                                                                                                                  \
        uint8x16_t vec1 = vld1q(&dataIn[line * width + y + offset[0] * width]);                                        \
        uint8x16_t vec2 = vld1q(&dataIn[line * width + y + offset[1] * width]);                                        \
        uint8x16_t vec3 = vld1q(&dataIn[line * width + y + offset[2] * width]);                                        \
        int16x8x2_t vect_res;                                                                                          \
        VERTICAL_COMPUTE_VECTOR(vec1, vec2, vec3, vect_res);                                                           \
        vst2q(&scratch[y], vect_res);                                                                                  \
    }                                                                                                                  \
    for (int y = width - (width % 16); y < width; y++)                                                                 \
    {                                                                                                                  \
        scratch[y] = VERTICAL_COMPUTE_SCALAR(dataIn[line * width + y + offset[0] * width],                             \
                                             dataIn[line * width + y + offset[1] * width],                             \
                                             dataIn[line * width + y + offset[2] * width]);                            \
    }

#if (ARM_CV_LINEAR_OUTPUT_TYPE == ARM_CV_LINEAR_OUTPUT_UINT_8)
// Do the processing of a line of the input image,
// borderLocation allow to treat the upper and lowest line of the image using the same macro
#define LOOP_BUFFER_TO_OUTPUT(width, scratch, dataOut, line, offset)                                                   \
    for (int y = 1; y < width - 16; y += 16)                                                                           \
    {                                                                                                                  \
        int16x8x2_t vec1 = vld2q(&scratch[y + offset[0]]);                                                             \
        int16x8x2_t vec2 = vld2q(&scratch[y + offset[1]]);                                                             \
        int16x8x2_t vec3 = vld2q(&scratch[y + offset[2]]);                                                             \
        int8x16_t vect_out;                                                                                            \
        HORIZONTAL_COMPUTE_VECTOR(vec1, vec2, vec3, vect_out)                                                          \
        vst1q((int8_t *)&dataOut[line * width + y], vect_out);                                                         \
    }                                                                                                                  \
    for (int y = width - ((width - 1) % 16); y < width - 1; y++)                                                       \
    {                                                                                                                  \
        dataOut[line * width + y] =                                                                                    \
            HORIZONTAL_COMPUTE_SCALAR(scratch[y + offset[0]], scratch[y + offset[1]], scratch[y + offset[2]]);         \
    }

//output q15
#else

#define LOOP_BUFFER_TO_OUTPUT(width, scratch, dataOut, line, offset)                                                   \
    for (int y = 1; y < width - 8; y += 8)                                                                             \
    {                                                                                                                  \
        int16x8_t vec1 = vld1q(&scratch[y + offset[0]]);                                                               \
        int16x8_t vec2 = vld1q(&scratch[y + offset[1]]);                                                               \
        int16x8_t vec3 = vld1q(&scratch[y + offset[2]]);                                                               \
        int16x8_t vect_out;                                                                                            \
        HORIZONTAL_COMPUTE_VECTOR(vec1, vec2, vec3, vect_out)                                                          \
        vst1q(&dataOut[line * width + y], vect_out);                                                                   \
    }                                                                                                                  \
    for (int y = width - ((width - 1) % 8); y < width - 1; y++)                                                        \
    {                                                                                                                  \
        dataOut[line * width + y] =                                                                                    \
            HORIZONTAL_COMPUTE_SCALAR(scratch[y + offset[0]], scratch[y + offset[1]], scratch[y + offset[2]]);         \
    }

#endif

//does not depend on the type of input output so common for the variation
#define LINE_PROCESSING(borderLocation, width, scratch, dataIn, offset, borderType, line, height)                      \
    BORDER_OFFSET(offset, borderLocation, height, borderType)                                                          \
    LOOP_INPUT_TO_BUFFER(width, scratch, dataIn, line, offset)                                                         \
    BORDER_OFFSET(offset, ARM_CV_LEFT_TOP, width, borderType)                                                          \
    dataOut[line * width] = HORIZONTAL_COMPUTE_SCALAR(scratch[offset[0]], scratch[offset[1]], scratch[offset[2]]);     \
    BORDER_OFFSET(offset, ARM_CV_MIDDLE, width, borderType)                                                            \
    LOOP_BUFFER_TO_OUTPUT(width, scratch, dataOut, line, offset)                                                       \
    BORDER_OFFSET(offset, ARM_CV_RIGHT_BOT, width, borderType)                                                         \
    dataOut[line * width + width - 1] = HORIZONTAL_COMPUTE_SCALAR(                                                     \
        scratch[width - 1 + offset[0]], scratch[width - 1 + offset[1]], scratch[width - 1 + offset[2]]);

//scalar case
#else

#define LINE_PROCESSING(borderLocation, width, scratch, dataIn, offset, borderType, line, height)                      \
    BORDER_OFFSET(offset, borderLocation, height, borderType);                                                         \
    for (int y = 0; y < width; y++)                                                                                    \
    {                                                                                                                  \
        scratch[y] = VERTICAL_COMPUTE_SCALAR(dataIn[line * width + y + offset[0] * width],                             \
                                             dataIn[line * width + y + offset[1] * width],                             \
                                             dataIn[line * width + y + offset[2] * width]);                            \
    }                                                                                                                  \
    BORDER_OFFSET(offset, ARM_CV_LEFT_TOP, width, borderType);                                                         \
    dataOut[line * width] = HORIZONTAL_COMPUTE_SCALAR(scratch[offset[0]], scratch[offset[1]], scratch[offset[2]]);     \
    BORDER_OFFSET(offset, ARM_CV_MIDDLE, width, borderType);                                                           \
    for (int y = 1; y < width - 1; y++)                                                                                \
    {                                                                                                                  \
        dataOut[line * width + y] =                                                                                    \
            HORIZONTAL_COMPUTE_SCALAR(scratch[y + offset[0]], scratch[y + offset[1]], scratch[y + offset[2]]);         \
    }                                                                                                                  \
    BORDER_OFFSET(offset, ARM_CV_RIGHT_BOT, width, borderType);                                                        \
    dataOut[line * width + width - 1] = HORIZONTAL_COMPUTE_SCALAR(                                                     \
        scratch[width - 1 + offset[0]], scratch[width - 1 + offset[1]], scratch[width - 1 + offset[2]]);

#endif



//definition of line processing linear function
//two cases one for vector and one for scalar
//output type vary depending on the define of the function file
#if defined(ARM_MATH_MVEI) && !defined(ARM_MATH_AUTOVECTORIZE)

__STATIC_INLINE void line_processing_linear(int8_t borderLocation, int16_t width, q15_t *scratch, uint8_t *dataIn,
                                            OUTPUT_TYPE *dataOut, int *offset, const uint8_t borderType, uint8_t line,
                                            uint8_t height)
{
    BORDER_OFFSET(offset, borderLocation, height, borderType);
    LOOP_INPUT_TO_BUFFER(width, scratch, dataIn, line, offset);
    BORDER_OFFSET(offset, ARM_CV_LEFT_TOP, width, borderType);
    dataOut[line * width] = HORIZONTAL_COMPUTE_SCALAR(scratch[offset[0]], scratch[offset[1]], scratch[offset[2]]);
    BORDER_OFFSET(offset, ARM_CV_MIDDLE, width, borderType);
    LOOP_BUFFER_TO_OUTPUT(width, scratch, dataOut, line, offset);
    BORDER_OFFSET(offset, ARM_CV_RIGHT_BOT, width, borderType);
    dataOut[line * width + width - 1] = HORIZONTAL_COMPUTE_SCALAR(
        scratch[width - 1 + offset[0]], scratch[width - 1 + offset[1]], scratch[width - 1 + offset[2]]);
}

#else

__STATIC_INLINE void line_processing_linear(int8_t borderLocation, int16_t width, q15_t *scratch, uint8_t *dataIn,
                                            OUTPUT_TYPE *dataOut, int *offset, const uint8_t borderType, uint8_t line,
                                            uint8_t height)
{
    BORDER_OFFSET(offset, borderLocation, height, borderType);
    for (int y = 0; y < width; y++)
    {
        scratch[y] = VERTICAL_COMPUTE_SCALAR(dataIn[line * width + y + offset[0] * width],
                                             dataIn[line * width + y + offset[1] * width],
                                             dataIn[line * width + y + offset[2] * width]);
    }
    BORDER_OFFSET(offset, ARM_CV_LEFT_TOP, width, borderType);
    dataOut[line * width] = HORIZONTAL_COMPUTE_SCALAR(scratch[offset[0]], scratch[offset[1]], scratch[offset[2]]);
    BORDER_OFFSET(offset, ARM_CV_MIDDLE, width, borderType);
    for (int y = 1; y < width - 1; y++)
    {
        dataOut[line * width + y] =
            HORIZONTAL_COMPUTE_SCALAR(scratch[y + offset[0]], scratch[y + offset[1]], scratch[y + offset[2]]);
    }
    BORDER_OFFSET(offset, ARM_CV_RIGHT_BOT, width, borderType);
    dataOut[line * width + width - 1] = HORIZONTAL_COMPUTE_SCALAR(
        scratch[width - 1 + offset[0]], scratch[width - 1 + offset[1]], scratch[width - 1 + offset[2]]);
}
#endif

#define LINEAR_GENERIC(imageIn, imageOut, scratch, borderType) \
    int width = imageOut->width;                                                                                       \
    int height = imageOut->height;                                                                                     \
    uint8_t *dataIn = imageIn->pData;                                                                                  \
    OUTPUT_TYPE*dataOut = imageOut->pData;                                                                             \
    int offset[3];                                                                                                     \
    LINE_PROCESSING(ARM_CV_LEFT_TOP, width, scratch, dataIn, offset, borderType, 0, height)                            \
    for (int x = 1; x < height - 1; x++)                                                                               \
    {                                                                                                                  \
        LINE_PROCESSING(ARM_CV_MIDDLE, width, scratch, dataIn, offset, borderType, x, height)                          \
    }                                                                                                                  \
    int x = height - 1;                                                                                                \
    LINE_PROCESSING(ARM_CV_RIGHT_BOT, width, scratch, dataIn, offset, borderType, x, height)

//define file
#endif