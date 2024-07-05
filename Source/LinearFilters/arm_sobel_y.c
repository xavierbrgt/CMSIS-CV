/* ----------------------------------------------------------------------
 * Project:      CMSIS CV Library
 * Title:        gaussian.c
 * Description:  Gaussian filter CMSIS-CV
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

#include "cv/linear_filters.h"
#include "dsp/basic_math_functions.h"
#include "arm_linear_filter_common.h"


#define HORIZONTAL_COMPUTE_SCALAR_SOBEL_Y(data_0, data_1, data_2) (-(data_0) + (data_2))
#define VERTICAL_COMPUTE_SCALAR_SOBEL_Y(data_0, data_1, data_2) ((data_0 )+ (data_1<<1) + (data_2))<<5

#if defined(ARM_MATH_MVEI) && !defined(ARM_MATH_AUTOVECTORIZE)

#define VERTICAL_COMPUTE_VECTOR_SOBEL_Y(vect_1, vect_2, vect_3, vect_res) int16x8x2_t vect_1x2;\
    int16x8x2_t vect_3x2;\
    vect_1x2.val[0] = vshllbq(vect_1,5);\
    vect_3x2.val[0] = vshllbq(vect_3,5);\
    vect_1x2.val[0] = vaddq(vect_1x2.val[0], vect_3x2.val[0]);\
    vect_1x2.val[1] = vshlltq(vect_1,5);\
    vect_3x2.val[1] = vshlltq(vect_3,5);\
    vect_1x2.val[1] = vaddq(vect_1x2.val[1], vect_3x2.val[1]);\
    vect_res.val[0] = vshllbq(vect_2,6);\
    vect_res.val[0] = vaddq(vect_res.val[0], vect_1x2.val[0]);\
    vect_res.val[1] = vshlltq(vect_2,6);\
    vect_res.val[1] = vaddq(vect_res.val[1], vect_1x2.val[1]);

#define HORIZONTAL_COMPUTE_VECTOR_SOBEL_Y(vect_1, vect_2, vect_3, vect_out) vect_out = vsubq(vect_3, vect_1);\
    (void)vect_2;
#endif

#if defined(ARM_MATH_MVEI) && !defined(ARM_MATH_AUTOVECTORIZE)



void arm_sobel_y(const arm_cv_image_gray8_t* imageIn, 
                                   arm_cv_image_q15_t* imageOut,
                                   q15_t* scratch,
                                   int8_t borderType)
{
    int w = imageOut->width;
    int h = imageOut->height;
    uint8_t* dataIn = imageIn->pData;
    q15_t* dataOut = imageOut->pData;
    int offset[3];
    BORDER_OFFSET(offset, left_top, h, borderType);
    for(int y =0; y<w-15; y+=16)
    {
        uint8x16_t vec1 = vld1q(&dataIn[y+offset[0]*w]);
        uint8x16_t vec2 = vld1q(&dataIn[y+offset[1]*w]);
        uint8x16_t vec3 = vld1q(&dataIn[y+offset[2]*w]);
        //int16x8x2_t vect_res = VerticalCompute(vec1,vec2,vec3);
        int16x8x2_t vect_res;
        VERTICAL_COMPUTE_VECTOR_SOBEL_Y(vec1,vec2,vec3, vect_res)
        vst2q(&scratch[y], vect_res);
    }
    for(int y = w-(w %16); y < w; y ++)
    {
        scratch[y] = VERTICAL_COMPUTE_SCALAR_SOBEL_Y(dataIn[y+offset[0]*w],dataIn[y+offset[1]*w],dataIn[y+offset[2]*w]);
    }
    BORDER_OFFSET(offset, left_top, w, borderType);
    dataOut[0] = HORIZONTAL_COMPUTE_SCALAR_SOBEL_Y(scratch[offset[0]],scratch[offset[1]],scratch[offset[2]]);
    BORDER_OFFSET(offset, middle, w, borderType);
    for(int y =1; y<w-8; y+=8)
    {
        int16x8_t vec1 = vld1q(&scratch[y+offset[0]]);
        int16x8_t vec2 = vld1q(&scratch[y+offset[1]]);
        int16x8_t vec3 = vld1q(&scratch[y+offset[2]]);
        //int8x16_t vect_out = HorizonCompute(vec1,vec2,vec3);
        int16x8_t vect_out;
        HORIZONTAL_COMPUTE_VECTOR_SOBEL_Y(vec1,vec2,vec3, vect_out)
        vst1q(&dataOut[y], vect_out);
    }
    for(int y =w-((w-1)%8); y<w-1; y++)
    {
        dataOut[y] = HORIZONTAL_COMPUTE_SCALAR_SOBEL_Y(scratch[y+offset[0]],scratch[y+offset[1]],scratch[y+offset[2]]);
    }     
    BORDER_OFFSET(offset, right_bot, w, borderType);
    dataOut[w-1] = HORIZONTAL_COMPUTE_SCALAR_SOBEL_Y(scratch[w-1+offset[0]],scratch[w-1+offset[1]],scratch[w-1+offset[2]]);

    for(int x = 1; x<h-1; x++)
    {
        BORDER_OFFSET(offset, middle, h, borderType);
        for(int y =0; y<w-15; y+=16)
        {
            uint8x16_t vec1 = vld1q(&dataIn[x*w + y+offset[0]*w]);
            uint8x16_t vec2 = vld1q(&dataIn[x*w + y+offset[1]*w]);
            uint8x16_t vec3 = vld1q(&dataIn[x*w + y+offset[2]*w]);
            int16x8x2_t vect_res;
            VERTICAL_COMPUTE_VECTOR_SOBEL_Y(vec1,vec2,vec3,vect_res);
            vst2q(&scratch[y], vect_res);
        }
        for(int y = w-(w %16); y < w; y ++)
        {
            scratch[y] = VERTICAL_COMPUTE_SCALAR_SOBEL_Y(dataIn[x*w + y+offset[0]*w],dataIn[x*w + y+offset[1]*w],dataIn[x*w + y+offset[2]*w]);
        }
        BORDER_OFFSET(offset, left_top, w, borderType);
        dataOut[x*w] = HORIZONTAL_COMPUTE_SCALAR_SOBEL_Y(scratch[offset[0]],scratch[offset[1]],scratch[offset[2]]);
        BORDER_OFFSET(offset, middle, w, borderType);
        for(int y =1; y<w-8; y+=8)
        {
            int16x8_t vec1 = vld1q(&scratch[y+offset[0]]);
            int16x8_t vec2 = vld1q(&scratch[y+offset[1]]);
            int16x8_t vec3 = vld1q(&scratch[y+offset[2]]);
            //int8x16_t vect_out = HorizonCompute(vec1,vec2,vec3);
            int16x8_t vect_out;
            HORIZONTAL_COMPUTE_VECTOR_SOBEL_Y(vec1,vec2,vec3, vect_out)
            vst1q(&dataOut[x*w + y], vect_out);
        }
        for(int y =w-((w-1)%8); y<w-1; y++)
        {
            dataOut[x*w + y] = HORIZONTAL_COMPUTE_SCALAR_SOBEL_Y(scratch[y+offset[0]],scratch[y+offset[1]],scratch[y+offset[2]]);
        }
        BORDER_OFFSET(offset, right_bot, w, borderType);
        dataOut[x*w+w-1] = HORIZONTAL_COMPUTE_SCALAR_SOBEL_Y(scratch[w-1+offset[0]],scratch[w-1+offset[1]],scratch[w-1+offset[2]]);
    }

    int x = h-1;
    BORDER_OFFSET(offset, right_bot, h, borderType);
    for(int y =0; y<w-15; y+=16)
    {
        uint8x16_t vec1 = vld1q(&dataIn[x*w + y+offset[0]*w]);
        uint8x16_t vec2 = vld1q(&dataIn[x*w + y+offset[1]*w]);
        uint8x16_t vec3 = vld1q(&dataIn[x*w + y+offset[2]*w]);
        //int16x8x2_t vect_res = VerticalCompute(vec1,vec2,vec3);
        int16x8x2_t vect_res;
        VERTICAL_COMPUTE_VECTOR_SOBEL_Y(vec1,vec2,vec3,vect_res);
        vst2q(&scratch[y], vect_res);
    }
    for(int y = w-(w %16); y < w; y ++)
    {
        scratch[y] = VERTICAL_COMPUTE_SCALAR_SOBEL_Y(dataIn[x*w + y+offset[0]*w],dataIn[x*w + y+offset[1]*w],dataIn[x*w + y+offset[2]*w]);
    }
    BORDER_OFFSET(offset, left_top, w, borderType);
    dataOut[x*w] = HORIZONTAL_COMPUTE_SCALAR_SOBEL_Y(scratch[offset[0]],scratch[offset[1]],scratch[offset[2]]);
    BORDER_OFFSET(offset, middle, w, borderType);
    for(int y =1; y<w-8; y+=8)
    {
        int16x8_t vec1 = vld1q(&scratch[y+offset[0]]);
        int16x8_t vec2 = vld1q(&scratch[y+offset[1]]);
        int16x8_t vec3 = vld1q(&scratch[y+offset[2]]);
        //int8x16_t vect_out = HorizonCompute(vec1,vec2,vec3);
        int16x8_t vect_out;
        HORIZONTAL_COMPUTE_VECTOR_SOBEL_Y(vec1,vec2,vec3, vect_out)
        vst1q(&dataOut[x*w + y], vect_out);
    }
    for(int y =w-((w-1)%8); y<w-1; y++)
    {
        dataOut[x*w + y] = HORIZONTAL_COMPUTE_SCALAR_SOBEL_Y(scratch[y+offset[0]],scratch[y+offset[1]],scratch[y+offset[2]]);
    }
    BORDER_OFFSET(offset, right_bot, w, borderType);
    dataOut[x*w+w-1] = HORIZONTAL_COMPUTE_SCALAR_SOBEL_Y(scratch[w-1+offset[0]],scratch[w-1+offset[1]],scratch[w-1+offset[2]]);
}

#else
void arm_sobel_y(const arm_cv_image_gray8_t* imageIn, 
                                   arm_cv_image_q15_t* imageOut,
                                   q15_t* scratch,
                                   int8_t borderType)
{
    int w = imageOut->width;
    int h = imageOut->height;
    uint8_t* dataIn = imageIn->pData;
    q15_t* dataOut = imageOut->pData;
    /*      top part        */
    int offset[3];
    BORDER_OFFSET(offset, left_top, h, borderType);
    for(int y =0; y<w; y++)
    {
        scratch[y] = VERTICAL_COMPUTE_SCALAR_SOBEL_Y(dataIn[y+offset[0]*w],dataIn[y+offset[1]*w],dataIn[y+offset[2]*w]);
    }
    BORDER_OFFSET(offset, left_top, w, borderType);
    dataOut[0] = HORIZONTAL_COMPUTE_SCALAR_SOBEL_Y(scratch[offset[0]],scratch[offset[1]],scratch[offset[2]]);
    BORDER_OFFSET(offset, middle, w, borderType);
    for(int y =1; y<w-1; y++)
    {
        dataOut[y] = HORIZONTAL_COMPUTE_SCALAR_SOBEL_Y(scratch[y+offset[0]],scratch[y+offset[1]],scratch[y+offset[2]]);
    }
    BORDER_OFFSET(offset, right_bot, w, borderType);
    dataOut[w-1] = HORIZONTAL_COMPUTE_SCALAR_SOBEL_Y(scratch[w-1+offset[0]],scratch[w-1+offset[1]],scratch[w-1+offset[2]]);
    /*      middle part      */
    for(int x = 1; x<h-1; x++)
    {
        BORDER_OFFSET(offset, middle, h, borderType);
        for(int y =0; y<w; y++)
        {
            scratch[y] = VERTICAL_COMPUTE_SCALAR_SOBEL_Y(dataIn[x*w +y+offset[0]*w],dataIn[x*w + y+offset[1]*w],dataIn[x*w + y+offset[2]*w]);
        }
        BORDER_OFFSET(offset, left_top, w, borderType);
        dataOut[x*w] = HORIZONTAL_COMPUTE_SCALAR_SOBEL_Y(scratch[offset[0]],scratch[offset[1]],scratch[offset[2]]);
        BORDER_OFFSET(offset, middle, w, borderType);
        for(int y =1; y<w-1; y++)
        {
            dataOut[x*w+y] = HORIZONTAL_COMPUTE_SCALAR_SOBEL_Y(scratch[y+offset[0]],scratch[y+offset[1]],scratch[y+offset[2]]);
        }
        BORDER_OFFSET(offset, right_bot, w, borderType);
        dataOut[x*w+w-1] = HORIZONTAL_COMPUTE_SCALAR_SOBEL_Y(scratch[w-1+offset[0]],scratch[w-1+offset[1]],scratch[w-1+offset[2]]);
    }
    /*      bottom part     */
    int x = h-1;
    BORDER_OFFSET(offset, right_bot, h, borderType);
    for(int y =0; y<w; y++)
    {
        scratch[y] = VERTICAL_COMPUTE_SCALAR_SOBEL_Y(dataIn[x*w +y+offset[0]*w],dataIn[x*w + y+offset[1]*w],dataIn[x*w + y+offset[2]*w]);
    }
    BORDER_OFFSET(offset, left_top, w, borderType);
    dataOut[x*w] = HORIZONTAL_COMPUTE_SCALAR_SOBEL_Y(scratch[offset[0]],scratch[offset[1]],scratch[offset[2]]);
    BORDER_OFFSET(offset, middle, w, borderType);
    for(int y =1; y<w-1; y++)
    {
        dataOut[x*w+y] = HORIZONTAL_COMPUTE_SCALAR_SOBEL_Y(scratch[y+offset[0]],scratch[y+offset[1]],scratch[y+offset[2]]);
    }
    BORDER_OFFSET(offset, right_bot, w, borderType);
    dataOut[x*w+w-1] = HORIZONTAL_COMPUTE_SCALAR_SOBEL_Y(scratch[w-1+offset[0]],scratch[w-1+offset[1]],scratch[w-1+offset[2]]);
}
#endif