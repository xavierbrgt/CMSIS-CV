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

#ifndef ARM_CV_LINEAR_FILTER_COMMON_SCRATCH_C
#define ARM_CV_LINEAR_FILTER_COMMON_SCRATCH_C
#include "arm_linear_filter_common.h"
#include <stdio.h>

#if ARM_CV_LINEAR_OUTPUT_TYPE == ARM_CV_LINEAR_OUTPUT_UINT_8
#define OUTPUT_TYPE uint8_t
#else
#define OUTPUT_TYPE q15_t
#endif

#if defined(KERNEL_5)
#define KERNEL_SIZE 5
#else
#if defined(KERNEL_7)
#define KERNEL_SIZE 7
#else
#define KERNEL_SIZE 3
#endif
#endif

#if defined(BUFFER_15)
#define BUFFER_TYPE q15_t
#define BUFFER_TYPE_DOUBLE_VECTOR int16x8x2_t

#else
#define BUFFER_TYPE q31_t
#define BUFFER_TYPE_DOUBLE_VECTOR int32x4x4_t

#endif

#if defined(KERNEL_5)

#define HORIZONTAL_ATTRIBUTION(indexOut, indexIn, offset, dataOut, dataIn)                                             \
    dataOut[indexOut] = HORIZONTAL_COMPUTE_SCALAR(dataIn[indexIn + offset[0]], dataIn[indexIn + offset[1]],            \
                                                  dataIn[indexIn + offset[2]], dataIn[indexIn + offset[3]],            \
                                                  dataIn[indexIn + offset[4]]);

#define VERTICAL_ATTRIBUTION(indexOut, indexIn, offset, dataOut, dataIn)                                               \
    dataOut[indexOut] = VERTICAL_COMPUTE_SCALAR(                                                                       \
        dataIn[indexIn + offset[0] * width], dataIn[indexIn + offset[1] * width], dataIn[indexIn + offset[2] * width], \
        dataIn[indexIn + offset[3] * width], dataIn[indexIn + offset[4] * width]);

#define LOOP_INPUT_TO_BUFFER(width, scratch, dataIn, line, offset)                                                     \
    for (int y = 0; y < width - 15; y += 16)                                                                           \
    {                                                                                                                  \
        uint8x16_t vec1 = vld1q(&dataIn[line * width + y + offset[0] * width]);                                        \
        uint8x16_t vec2 = vld1q(&dataIn[line * width + y + offset[1] * width]);                                        \
        uint8x16_t vec3 = vld1q(&dataIn[line * width + y + offset[2] * width]);                                        \
        uint8x16_t vec4 = vld1q(&dataIn[line * width + y + offset[3] * width]);                                        \
        uint8x16_t vec5 = vld1q(&dataIn[line * width + y + offset[4] * width]);                                        \
        BUFFER_TYPE_DOUBLE_VECTOR vect_res;                                                                            \
        VERTICAL_COMPUTE_VECTOR(vec1, vec2, vec3, vec4, vec5, vect_res);                                               \
        vst2q(&scratch[y], vect_res);                                                                                  \
    }                                                                                                                  \
    for (int y = width - (width % 16); y < width; y++)                                                                 \
    {                                                                                                                  \
        VERTICAL_ATTRIBUTION(y, line *width + y, offset, scratch, dataIn)                                              \
    }

#if (ARM_CV_LINEAR_OUTPUT_TYPE == ARM_CV_LINEAR_OUTPUT_UINT_8)

// Do the processing of a line of the input image,
// borderLocation allow to treat the upper and lowest line of the image using the same macro
#define LOOP_BUFFER_TO_OUTPUT(width, scratch, dataOut, line, offset)                                                   \
    for (int y = 2; y < width - 16; y += 16)                                                                           \
    {                                                                                                                  \
        BUFFER_TYPE_DOUBLE_VECTOR vec1 = vld2q(&scratch[y + offset[0]]);                                               \
        BUFFER_TYPE_DOUBLE_VECTOR vec2 = vld2q(&scratch[y + offset[1]]);                                               \
        BUFFER_TYPE_DOUBLE_VECTOR vec3 = vld2q(&scratch[y + offset[2]]);                                               \
        BUFFER_TYPE_DOUBLE_VECTOR vec4 = vld2q(&scratch[y + offset[3]]);                                               \
        BUFFER_TYPE_DOUBLE_VECTOR vec5 = vld2q(&scratch[y + offset[4]]);                                               \
        int8x16_t vect_out;                                                                                            \
        HORIZONTAL_COMPUTE_VECTOR(vec1, vec2, vec3, vec4, vec5, vect_out)                                              \
        vst1q((int8_t *)&dataOut[line * width + y], vect_out);                                                         \
    }                                                                                                                  \
    for (int y = width - ((width - 2) % 16); y < width - 2; y++)                                                       \
    {                                                                                                                  \
        HORIZONTAL_ATTRIBUTION(line *width + y, y, offset, dataOut, scratch)                                           \
    }

// output q15
#else

#define LOOP_BUFFER_TO_OUTPUT(width, scratch, dataOut, line, offset)                                                   \
    for (int y = 2; y < width - 8; y += 8)                                                                             \
    {                                                                                                                  \
        uint16x8_t vec1 = vld1q(&scratch[y + offset[0]]);                                                              \
        uint16x8_t vec2 = vld1q(&scratch[y + offset[1]]);                                                              \
        uint16x8_t vec3 = vld1q(&scratch[y + offset[2]]);                                                              \
        uint16x8_t vec4 = vld1q(&scratch[y + offset[3]]);                                                              \
        uint16x8_t vec5 = vld1q(&scratch[y + offset[4]]);                                                              \
        int16x8_t vect_out;                                                                                            \
        HORIZONTAL_COMPUTE_VECTOR(vec1, vec2, vec3, vec4, vec5, vect_out)                                              \
        vst1q(&dataOut[line * width + y], vect_out);                                                                   \
    }                                                                                                                  \
    for (int y = width - ((width - 1) % 8); y < width - 2; y++)                                                        \
    {                                                                                                                  \
        HORIZONTAL_ATTRIBUTION(line *width, y, offset, dataOut, scratch)                                               \
    }

#endif

#else

#if defined(KERNEL_7)

#define HORIZONTAL_ATTRIBUTION(indexOut, indexIn, offset, dataOut, dataIn)                                             \
    dataOut[indexOut] = HORIZONTAL_COMPUTE_SCALAR(dataIn[indexIn + offset[0]], dataIn[indexIn + offset[1]],            \
                                                  dataIn[indexIn + offset[2]], dataIn[indexIn + offset[3]],            \
                                                  dataIn[indexIn + offset[4]], dataIn[indexIn + offset[5]],            \
                                                  dataIn[indexIn + offset[6]]);

#define VERTICAL_ATTRIBUTION(indexOut, indexIn, offset, dataOut, dataIn)                                               \
    dataOut[indexOut] = VERTICAL_COMPUTE_SCALAR(                                                                       \
        dataIn[indexIn + offset[0] * width], dataIn[indexIn + offset[1] * width], dataIn[indexIn + offset[2] * width], \
        dataIn[indexIn + offset[3] * width], dataIn[indexIn + offset[4] * width], dataIn[indexIn + offset[5] * width], \
        dataIn[indexIn + offset[6] * width]);

#if defined(BUFFER_15)

#define LOOP_INPUT_TO_BUFFER(width, scratch, dataIn, line, offset)                                                     \
    for (int y = 0; y < width - 15; y += 16)                                                                           \
    {                                                                                                                  \
        uint8x16_t vec1 = vld1q(&dataIn[line * width + y + offset[0] * width]);                                        \
        uint8x16_t vec2 = vld1q(&dataIn[line * width + y + offset[1] * width]);                                        \
        uint8x16_t vec3 = vld1q(&dataIn[line * width + y + offset[2] * width]);                                        \
        uint8x16_t vec4 = vld1q(&dataIn[line * width + y + offset[3] * width]);                                        \
        uint8x16_t vec5 = vld1q(&dataIn[line * width + y + offset[4] * width]);                                        \
        uint8x16_t vec6 = vld1q(&dataIn[line * width + y + offset[5] * width]);                                        \
        uint8x16_t vec7 = vld1q(&dataIn[line * width + y + offset[6] * width]);                                        \
        BUFFER_TYPE_DOUBLE_VECTOR vect_res;                                                                            \
        VERTICAL_COMPUTE_VECTOR(vec1, vec2, vec3, vec4, vec5, vec6, vec7, vect_res);                                   \
        vst2q(&scratch[y], vect_res);                                                                                  \
    }                                                                                                                  \
    for (int y = width - (width % 16); y < width; y++)                                                                 \
    {                                                                                                                  \
        VERTICAL_ATTRIBUTION(y, line *width + y, offset, scratch, dataIn)                                              \
    }

#else

#define LOOP_INPUT_TO_BUFFER(width, scratch, dataIn, line, offset)                                                     \
    for (int y = 0; y < width - 15; y += 16)                                                                           \
    {                                                                                                                  \
        uint8x16_t vec1 = vld1q(&dataIn[line * width + y + offset[0] * width]);                                        \
        uint8x16_t vec2 = vld1q(&dataIn[line * width + y + offset[1] * width]);                                        \
        uint8x16_t vec3 = vld1q(&dataIn[line * width + y + offset[2] * width]);                                        \
        uint8x16_t vec4 = vld1q(&dataIn[line * width + y + offset[3] * width]);                                        \
        uint8x16_t vec5 = vld1q(&dataIn[line * width + y + offset[4] * width]);                                        \
        uint8x16_t vec6 = vld1q(&dataIn[line * width + y + offset[5] * width]);                                        \
        uint8x16_t vec7 = vld1q(&dataIn[line * width + y + offset[6] * width]);                                        \
        BUFFER_TYPE_DOUBLE_VECTOR vect_res;                                                                            \
        VERTICAL_COMPUTE_VECTOR(vec1, vec2, vec3, vec4, vec5, vec6, vec7, vect_res);                                   \
        vst4q(&scratch[y], vect_res);                                                                                  \
    }                                                                                                                  \
    for (int y = width - (width % 16); y < width; y++)                                                                 \
    {                                                                                                                  \
        VERTICAL_ATTRIBUTION(y, line *width + y, offset, scratch, dataIn)                                              \
    }

#endif
#if (ARM_CV_LINEAR_OUTPUT_TYPE == ARM_CV_LINEAR_OUTPUT_UINT_8)

#if defined(BUFFER_15)
// Do the processing of a line of the input image,
// borderLocation allow to treat the upper and lowest line of the image using the same macro
#define LOOP_BUFFER_TO_OUTPUT(width, scratch, dataOut, line, offset)                                                   \
    for (int y = 3; y < width - 16; y += 16)                                                                           \
    {                                                                                                                  \
        int16x8x2_t vec1 = vld2q(&scratch[y + offset[0]]);                                                             \
        int16x8x2_t vec2 = vld2q(&scratch[y + offset[1]]);                                                             \
        int16x8x2_t vec3 = vld2q(&scratch[y + offset[2]]);                                                             \
        int16x8x2_t vec4 = vld2q(&scratch[y + offset[3]]);                                                             \
        int16x8x2_t vec5 = vld2q(&scratch[y + offset[4]]);                                                             \
        int16x8x2_t vec6 = vld2q(&scratch[y + offset[5]]);                                                             \
        int16x8x2_t vec7 = vld2q(&scratch[y + offset[6]]);                                                             \
        int8x16_t vect_out;                                                                                            \
        HORIZONTAL_COMPUTE_VECTOR(vec1, vec2, vec3, vec4, vec5, vec6, vec7, vect_out)                                  \
        vst1q((int8_t *)&dataOut[line * width + y], vect_out);                                                         \
    }                                                                                                                  \
    for (int y = width - ((width - 3) % 16); y < width - 3; y++)                                                       \
    {                                                                                                                  \
        HORIZONTAL_ATTRIBUTION(line *width + y, y, offset, dataOut, scratch)                                           \
    }
#else
#define LOOP_BUFFER_TO_OUTPUT(width, scratch, dataOut, line, offset)                                                   \
    for (int y = 3; y < width - 16; y += 16)                                                                           \
    {                                                                                                                  \
        int32x4x4_t vec1 = vld4q(&scratch[y + offset[0]]);                                                             \
        int32x4x4_t vec2 = vld4q(&scratch[y + offset[1]]);                                                             \
        int32x4x4_t vec3 = vld4q(&scratch[y + offset[2]]);                                                             \
        int32x4x4_t vec4 = vld4q(&scratch[y + offset[3]]);                                                             \
        int32x4x4_t vec5 = vld4q(&scratch[y + offset[4]]);                                                             \
        int32x4x4_t vec6 = vld4q(&scratch[y + offset[5]]);                                                             \
        int32x4x4_t vec7 = vld4q(&scratch[y + offset[6]]);                                                             \
        int8x16_t vect_out;                                                                                            \
        HORIZONTAL_COMPUTE_VECTOR(vec1, vec2, vec3, vec4, vec5, vec6, vec7, vect_out)                                  \
        vst1q((int8_t *)&dataOut[line * width + y], vect_out);                                                         \
    }                                                                                                                  \
    for (int y = width - ((width - 3) % 16); y < width - 3; y++)                                                       \
    {                                                                                                                  \
        HORIZONTAL_ATTRIBUTION(line *width + y, y, offset, dataOut, scratch)                                           \
    }
#endif
// output q15
#else

#define LOOP_BUFFER_TO_OUTPUT(width, scratch, dataOut, line, offset)                                                   \
    for (int y = 3; y < width - 8; y += 8)                                                                             \
    {                                                                                                                  \
        q15x8_t vec1 = vld1q(&scratch[y + offset[0]]);                                                                 \
        q15x8_t vec2 = vld1q(&scratch[y + offset[1]]);                                                                 \
        q15x8_t vec3 = vld1q(&scratch[y + offset[2]]);                                                                 \
        q15x8_t vec4 = vld1q(&scratch[y + offset[3]]);                                                                 \
        q15x8_t vec5 = vld1q(&scratch[y + offset[4]]);                                                                 \
        q15x8_t vec6 = vld1q(&scratch[y + offset[5]]);                                                                 \
        q15x8_t vec7 = vld1q(&scratch[y + offset[6]]);                                                                 \
        q15x8_t vect_out;                                                                                              \
        HORIZONTAL_COMPUTE_VECTOR(vec1, vec2, vec3, vec4, vec5, vec6, vec7, vect_out)                                  \
        vst1q(&dataOut[line * width + y], vect_out);                                                                   \
    }                                                                                                                  \
    for (int y = width - ((width - 3) % 8); y < width - 3; y++)                                                        \
    {                                                                                                                  \
        HORIZONTAL_ATTRIBUTION(line *width, y, offset, dataOut, scratch)                                               \
    }

#endif
#else
#define HORIZONTAL_ATTRIBUTION(indexOut, indexIn, offset, dataOut, dataIn)                                             \
    dataOut[indexOut] = HORIZONTAL_COMPUTE_SCALAR(dataIn[indexIn + offset[0]], dataIn[indexIn + offset[1]],            \
                                                  dataIn[indexIn + offset[2]]);

#define VERTICAL_ATTRIBUTION(indexOut, indexIn, offset, dataOut, dataIn)                                               \
    dataOut[indexOut] =                                                                                                \
        VERTICAL_COMPUTE_SCALAR(dataIn[indexIn + offset[0] * width], dataIn[indexIn + offset[1] * width],              \
                                dataIn[indexIn + offset[2] * width]);

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

// output q15
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
#endif

#endif

#if defined(ARM_MATH_MVEI) && !defined(ARM_MATH_AUTOVECTORIZE)

__STATIC_INLINE void line_processing_linear(int8_t borderLocation, int16_t width, BUFFER_TYPE *scratch, uint8_t *dataIn,
                                            OUTPUT_TYPE *dataOut, int *offset, const uint8_t borderType, uint8_t line,
                                            uint8_t height)
{
    BORDER_OFFSET(offset, borderLocation, height, borderType);
    LOOP_INPUT_TO_BUFFER(width, scratch, dataIn, line, offset);
    for (int y = 0; y < KERNEL_SIZE >> 1; y++)
    {
        BORDER_OFFSET(offset, y, width, borderType);
        HORIZONTAL_ATTRIBUTION(line * width + y, y, offset, dataOut, scratch);
    }
    BORDER_OFFSET(offset, KERNEL_SIZE >> 1, width, borderType);
    for (int y = 0; y < KERNEL_SIZE >> 1; y++)
    {
        BORDER_OFFSET(offset, y, width, borderType);
        HORIZONTAL_ATTRIBUTION(line * width + y, y, offset, dataOut, scratch);
    }
    BORDER_OFFSET(offset, KERNEL_SIZE >> 1, width, borderType);
    LOOP_BUFFER_TO_OUTPUT(width, scratch, dataOut, line, offset);
    for (int y = (KERNEL_SIZE >> 1) + 1; y < KERNEL_SIZE; y++)
    {
        BORDER_OFFSET(offset, y, width, borderType);
        HORIZONTAL_ATTRIBUTION(line * width + width - KERNEL_SIZE + y, width - KERNEL_SIZE + y, offset, dataOut,
                               scratch);
    }
}
#else

__STATIC_INLINE void line_processing_linear(int8_t borderLocation, int16_t width, BUFFER_TYPE *scratch, uint8_t *dataIn,
                                            OUTPUT_TYPE *dataOut, int *offset, const uint8_t borderType, uint8_t line,
                                            uint8_t height)
{
    BORDER_OFFSET(offset, borderLocation, height, borderType);
    for (int y = 0; y < width; y++)
    {
        VERTICAL_ATTRIBUTION(y, line * width + y, offset, scratch, dataIn);
    }
    for (int y = 0; y < KERNEL_SIZE >> 1; y++)
    {
        BORDER_OFFSET(offset, y, width, borderType);
        HORIZONTAL_ATTRIBUTION(line * width + y, y, offset, dataOut, scratch);
    }
    BORDER_OFFSET(offset, KERNEL_SIZE >> 1, width, borderType);
    for (int y = KERNEL_SIZE >> 1; y < width - (KERNEL_SIZE >> 1); y++)
    {
        HORIZONTAL_ATTRIBUTION(line * width + y, y, offset, dataOut, scratch);
    }
    for (int y = (KERNEL_SIZE >> 1) + 1; y < KERNEL_SIZE; y++)
    {
        BORDER_OFFSET(offset, y, width, borderType);
        HORIZONTAL_ATTRIBUTION(line * width + width - KERNEL_SIZE + y, width - KERNEL_SIZE + y, offset, dataOut,
                               scratch);
    }
}

#endif

#define LINEAR_GENERIC(imageIn, imageOut, scratch, borderType)                                                         \
    int width = imageOut->width;                                                                                       \
    int height = imageOut->height;                                                                                     \
    uint8_t *dataIn = imageIn->pData;                                                                                  \
    OUTPUT_TYPE *dataOut = imageOut->pData;                                                                            \
    int offset[KERNEL_SIZE];                                                                                           \
    for (int y = 0; y < KERNEL_SIZE >> 1; y++)                                                                         \
    {                                                                                                                  \
        line_processing_linear(y, width, scratch, dataIn, dataOut, offset, borderType, y, height);                     \
    }                                                                                                                  \
    for (int x = KERNEL_SIZE >> 1; x < height - (KERNEL_SIZE >> 1); x++)                                               \
    {                                                                                                                  \
        line_processing_linear((KERNEL_SIZE >> 1), width, scratch, dataIn, dataOut, offset, borderType, x, height);    \
    }                                                                                                                  \
    for (int y = (KERNEL_SIZE >> 1) + 1; y < KERNEL_SIZE; y++)                                                         \
    {                                                                                                                  \
        int x = height - KERNEL_SIZE + y;                                                                              \
        line_processing_linear(y, width, scratch, dataIn, dataOut, offset, borderType, x, height);                     \
    }
#endif
