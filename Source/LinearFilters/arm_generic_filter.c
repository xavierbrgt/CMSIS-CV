/* ----------------------------------------------------------------------
 * Project:      CMSIS CV Library
 * Title:        arm_generic_filter.c
 * Description:  Generic template for linear filter, this one doing a Gaussian filter
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

#define VERTICAL_COMPUTE_SCALAR_GAUSSIAN(data_0, data_1, data_2) (data_0*0x08 + (data_1*0x10) + data_2*0x08)
#define HORIZONTAL_COMPUTE_SCALAR_GAUSSIAN(data_0, data_1, data_2) ((data_0*0x08 + (data_1*0x10) + data_2*0x08)>>10)

#if defined(ARM_MATH_MVEI) && !defined(ARM_MATH_AUTOVECTORIZE)

/*__STATIC_FORCEINLINE int16x8x2_t VerticalCompute(uint8x16_t vect_1, uint8x16_t vect_2, uint8x16_t vect_3);
__STATIC_FORCEINLINE int8x16_t HorizonCompute(int16x8x2_t vect_1, int16x8x2_t vect_2, int16x8x2_t vect_3);*/
#define VERTICAL_COMPUTE_VECTOR_GAUSSIAN_1(vect_1, vect_2, vect_3) int16x8x2_t vect_2x2;\
    int16x8x2_t vect_1x2;\
    int16x8x2_t vect_3x2;\
    vect_1x2.val[0] = vshllbq(vect_1,3);\
    vect_3x2.val[0] = vshllbq(vect_3,3);\
    vect_1x2.val[0] = vaddq(vect_1x2.val[0], vect_3x2.val[0]);\
    vect_1x2.val[1] = vshlltq(vect_1,3);\
    vect_3x2.val[1] = vshlltq(vect_3,3);\
    vect_1x2.val[1] = vaddq(vect_1x2.val[1], vect_3x2.val[1]);\
    vect_2x2.val[0] = vshllbq(vect_2,4);\
    vect_2x2.val[0] = vaddq(vect_2x2.val[0], vect_1x2.val[0]);\
    vect_2x2.val[1] = vshlltq(vect_2,4);\
    vect_2x2.val[1] = vaddq(vect_2x2.val[1], vect_1x2.val[1]);\
    int16x8x2_t vect_res = vect_2x2;

#define VERTICAL_COMPUTE_VECTOR_GAUSSIAN_2(vect_1, vect_2, vect_3, vect_res) int16x8x2_t vect_1x2;\
    int16x8x2_t vect_3x2;\
    vect_1x2.val[0] = vshllbq(vect_1,3);\
    vect_3x2.val[0] = vshllbq(vect_3,3);\
    vect_1x2.val[0] = vaddq(vect_1x2.val[0], vect_3x2.val[0]);\
    vect_1x2.val[1] = vshlltq(vect_1,3);\
    vect_3x2.val[1] = vshlltq(vect_3,3);\
    vect_1x2.val[1] = vaddq(vect_1x2.val[1], vect_3x2.val[1]);\
    vect_res.val[0] = vshllbq(vect_2,4);\
    vect_res.val[0] = vaddq(vect_res.val[0], vect_1x2.val[0]);\
    vect_res.val[1] = vshlltq(vect_2,4);\
    vect_res.val[1] = vaddq(vect_res.val[1], vect_1x2.val[1]);

#define HORIZONTAL_COMPUTE_VECTOR_GAUSSIAN(vect_1, vect_2, vect_3, vect_out) vect_out = vdupq_n_s16(0);\
    vect_2.val[0] = vshlq_n_s16(vect_2.val[0], 1);\
    vect_2.val[0] = vaddq_s16(vect_2.val[0], vect_1.val[0]);\
    vect_2.val[1] = vshlq_n_s16(vect_2.val[1], 1);\
    vect_2.val[0] = vaddq_s16(vect_2.val[0], vect_3.val[0]);\
    vect_2.val[0] = vshrq(vect_2.val[0], 7);\
    vect_2.val[1] = vaddq_s16(vect_2.val[1], vect_1.val[1]);\
    vect_2.val[1] = vaddq_s16(vect_2.val[1], vect_3.val[1]);\
    vect_2.val[1] = vshrq(vect_2.val[1], 7);\
    vect_out = vmovntq(vect_out, vect_2.val[1]);\
    vect_out = vmovnbq(vect_out, vect_2.val[0]);
#endif


#if defined(ARM_MATH_MVEI) && !defined(ARM_MATH_AUTOVECTORIZE)

/*__STATIC_FORCEINLINE int16x8x2_t VerticalCompute(uint8x16_t vect_1, uint8x16_t vect_2, uint8x16_t vect_3)
{
    int16x8x2_t vect_2x2;
    int16x8x2_t vect_1x2;
    int16x8x2_t vect_3x2;
    vect_1x2.val[0] = vshllbq(vect_1,3);
    vect_3x2.val[0] = vshllbq(vect_3,3);
    vect_1x2.val[0] = vaddq(vect_1x2.val[0], vect_3x2.val[0]);
    vect_1x2.val[1] = vshlltq(vect_1,3);
    vect_3x2.val[1] = vshlltq(vect_3,3);
    vect_1x2.val[1] = vaddq(vect_1x2.val[1], vect_3x2.val[1]);
    vect_2x2.val[0] = vshllbq(vect_2,4);
    vect_2x2.val[0] = vaddq(vect_2x2.val[0], vect_1x2.val[0]);
    vect_2x2.val[1] = vshlltq(vect_2,4);
    vect_2x2.val[1] = vaddq(vect_2x2.val[1], vect_1x2.val[1]);
    return(vect_2x2);
}

__STATIC_FORCEINLINE int8x16_t HorizonCompute(int16x8x2_t vect_1, int16x8x2_t vect_2, int16x8x2_t vect_3)
{
    int8x16_t vec_out;
    vec_out = vdupq_n_s16(0);
    vect_2.val[0] = vshlq_n_s16(vect_2.val[0], 1);
    vect_2.val[0] = vaddq_s16(vect_2.val[0], vect_1.val[0]);
    vect_2.val[1] = vshlq_n_s16(vect_2.val[1], 1);
    vect_2.val[0] = vaddq_s16(vect_2.val[0], vect_3.val[0]);
    vect_2.val[0] = vshrq(vect_2.val[0], 7);
    vect_2.val[1] = vaddq_s16(vect_2.val[1], vect_1.val[1]);
    vect_2.val[1] = vaddq_s16(vect_2.val[1], vect_3.val[1]);
    vect_2.val[1] = vshrq(vect_2.val[1], 7);
    vec_out = vmovntq(vec_out, vect_2.val[1]);
    vec_out = vmovnbq(vec_out, vect_2.val[0]);
    return(vec_out);
}*/
#else
/*__STATIC_FORCEINLINE int HorizonCompute_sc(int data_0, int data_1, int data_2)
{
    int out = (data_0*0x08 + (data_1*0x10) + data_2*0x08)>>10;
    return(out);
}
__STATIC_FORCEINLINE int VerticalCompute_sc(int data_0, int data_1, int data_2)
{
    int out = data_0*0x08 + (data_1*0x10) + data_2*0x08;
    return(out);
}*/
#endif

#if defined(ARM_MATH_MVEI) && !defined(ARM_MATH_AUTOVECTORIZE)

void arm_linear_filter_generic(const arm_cv_image_gray8_t* imageIn, 
                                   arm_cv_image_gray8_t* imageOut,
                                   q15_t* scratch,
                                   int8_t borderType)
{
    //Vertical treatment
    int width = imageOut->width;
    int height = imageOut->height;
    uint8_t* dataIn = imageIn->pData;
    uint8_t* dataOut = imageOut->pData;
    /*      top part        */
    int offset[3];
    BORDER_OFFSET(offset, left_top, height, borderType);
    for(int y =0; y<width-15; y+=16)
    {
        uint8x16_t vec1 = vld1q(&dataIn[y+offset[0]*width]);
        uint8x16_t vec2 = vld1q(&dataIn[y+offset[1]*width]);
        uint8x16_t vec3 = vld1q(&dataIn[y+offset[2]*width]);
        //int16x8x2_t vect_res = VerticalCompute(vec1,vec2,vec3);
        int16x8x2_t vect_res;
        VERTICAL_COMPUTE_VECTOR_GAUSSIAN_2(vec1,vec2,vec3, vect_res)
        vst2q(&scratch[y], vect_res);
    }
    for(int y = width-(width %16); y < width; y ++)
    {
        scratch[y] = VERTICAL_COMPUTE_SCALAR_GAUSSIAN(dataIn[y+offset[0]*width],dataIn[y+offset[1]*width],dataIn[y+offset[2]*width]);
    }
    BORDER_OFFSET(offset, left_top, width, borderType);
    dataOut[0] = HORIZONTAL_COMPUTE_SCALAR_GAUSSIAN(scratch[offset[0]],scratch[offset[1]],scratch[offset[2]]);
    BORDER_OFFSET(offset, middle, width, borderType);
    for(int y =1; y<width-16; y+=16)
    {
        int16x8x2_t vec1 = vld2q(&scratch[y+offset[0]]);
        int16x8x2_t vec2 = vld2q(&scratch[y+offset[1]]);
        int16x8x2_t vec3 = vld2q(&scratch[y+offset[2]]);
        //int8x16_t vect_out = HorizonCompute(vec1,vec2,vec3);
        int8x16_t vect_out;
        HORIZONTAL_COMPUTE_VECTOR_GAUSSIAN(vec1,vec2,vec3, vect_out)
        vst1q((int8_t*)&dataOut[y], vect_out);
    }
    for(int y =width-((width-1)%16); y<width-1; y++)
    {
        dataOut[y] = HORIZONTAL_COMPUTE_SCALAR_GAUSSIAN(scratch[y+offset[0]],scratch[y+offset[1]],scratch[y+offset[2]]);
    }       
    BORDER_OFFSET(offset, right_bot, width, borderType);
    dataOut[width-1] = HORIZONTAL_COMPUTE_SCALAR_GAUSSIAN(scratch[width-1+offset[0]],scratch[width-1+offset[1]],scratch[width-1+offset[2]]);
    /*      middle part      */
    for(int x = 1; x<height-1; x++)
    {
        BORDER_OFFSET(offset, middle, height, borderType);
        for(int y =0; y<width-15; y+=16)
        {
            uint8x16_t vec1 = vld1q(&dataIn[x*width + y+offset[0]*width]);
            uint8x16_t vec2 = vld1q(&dataIn[x*width + y+offset[1]*width]);
            uint8x16_t vec3 = vld1q(&dataIn[x*width + y+offset[2]*width]);
            int16x8x2_t vect_res;
            VERTICAL_COMPUTE_VECTOR_GAUSSIAN_2(vec1,vec2,vec3,vect_res);
            vst2q(&scratch[y], vect_res);
        }
        for(int y = width-(width %16); y < width; y ++)
        {
            scratch[y] = VERTICAL_COMPUTE_SCALAR_GAUSSIAN(dataIn[x*width + y+offset[0]*width],dataIn[x*width + y+offset[1]*width],dataIn[x*width + y+offset[2]*width]);
        }
        BORDER_OFFSET(offset, left_top, width, borderType);
        dataOut[x*width] = HORIZONTAL_COMPUTE_SCALAR_GAUSSIAN(scratch[offset[0]],scratch[offset[1]],scratch[offset[2]]);
        BORDER_OFFSET(offset, middle, width, borderType);
        for(int y =1; y<width-16; y+=16)
        {
            int16x8x2_t vec1 = vld2q(&scratch[y+offset[0]]);
            int16x8x2_t vec2 = vld2q(&scratch[y+offset[1]]);
            int16x8x2_t vec3 = vld2q(&scratch[y+offset[2]]);
            //int8x16_t vect_out = HorizonCompute(vec1,vec2,vec3);
            int8x16_t vect_out;
            HORIZONTAL_COMPUTE_VECTOR_GAUSSIAN(vec1,vec2,vec3, vect_out)
            vst1q((int8_t*)&dataOut[x*width + y], vect_out);
        }
        for(int y =width-((width-1)%16); y<width-1; y++)
        {
            dataOut[x*width + y] = HORIZONTAL_COMPUTE_SCALAR_GAUSSIAN(scratch[y+offset[0]],scratch[y+offset[1]],scratch[y+offset[2]]);
        }
        BORDER_OFFSET(offset, right_bot, width, borderType);
        dataOut[x*width+width-1] = HORIZONTAL_COMPUTE_SCALAR_GAUSSIAN(scratch[width-1+offset[0]],scratch[width-1+offset[1]],scratch[width-1+offset[2]]);
    }
    /*      bottom part     */
    int x = height-1;
    BORDER_OFFSET(offset, right_bot, height, borderType);
    for(int y =0; y<width-15; y+=16)
    {
        uint8x16_t vec1 = vld1q(&dataIn[x*width + y+offset[0]*width]);
        uint8x16_t vec2 = vld1q(&dataIn[x*width + y+offset[1]*width]);
        uint8x16_t vec3 = vld1q(&dataIn[x*width + y+offset[2]*width]);
        //int16x8x2_t vect_res = VerticalCompute(vec1,vec2,vec3);
        int16x8x2_t vect_res;
        VERTICAL_COMPUTE_VECTOR_GAUSSIAN_2(vec1,vec2,vec3,vect_res);
        vst2q(&scratch[y], vect_res);
    }
    for(int y = width-(width %16); y < width; y ++)
    {
        scratch[y] = VERTICAL_COMPUTE_SCALAR_GAUSSIAN(dataIn[x*width + y+offset[0]*width],dataIn[x*width + y+offset[1]*width],dataIn[x*width + y+offset[2]*width]);
    }
    BORDER_OFFSET(offset, left_top, width, borderType);
    dataOut[x*width] = HORIZONTAL_COMPUTE_SCALAR_GAUSSIAN(scratch[offset[0]],scratch[offset[1]],scratch[offset[2]]);
    BORDER_OFFSET(offset, middle, width, borderType);
    for(int y =1; y<width-16; y+=16)
    {
        int16x8x2_t vec1 = vld2q(&scratch[y+offset[0]]);
        int16x8x2_t vec2 = vld2q(&scratch[y+offset[1]]);
        int16x8x2_t vec3 = vld2q(&scratch[y+offset[2]]);
        //int8x16_t vect_out = HorizonCompute(vec1,vec2,vec3);
        int8x16_t vect_out;
        HORIZONTAL_COMPUTE_VECTOR_GAUSSIAN(vec1,vec2,vec3, vect_out)
        vst1q((int8_t*)&dataOut[x*width + y], vect_out);
    }
    for(int y =width-((width-1)%16); y<width-1; y++)
    {
        dataOut[x*width + y] = HORIZONTAL_COMPUTE_SCALAR_GAUSSIAN(scratch[y+offset[0]],scratch[y+offset[1]],scratch[y+offset[2]]);
    }
    BORDER_OFFSET(offset, right_bot, width, borderType);
    dataOut[x*width+width-1] = HORIZONTAL_COMPUTE_SCALAR_GAUSSIAN(scratch[width-1+offset[0]],scratch[width-1+offset[1]],scratch[width-1+offset[2]]);
}

#else
void arm_linear_filter_generic(const arm_cv_image_gray8_t* imageIn, 
                                   arm_cv_image_gray8_t* imageOut,
                                   q15_t* scratch,
                                   int8_t borderType)
{
    //Vertical treatment
    int width = imageOut->width;
    int height = imageOut->height;
    uint8_t* dataIn = imageIn->pData;
    uint8_t* dataOut = imageOut->pData;
    /*      top part        */
    int offset[3];
    BORDER_OFFSET(offset, left_top, height, borderType);
    for(int y =0; y<width; y++)
    {
        scratch[y] = VERTICAL_COMPUTE_SCALAR_GAUSSIAN(dataIn[y+offset[0]*width],dataIn[y+offset[1]*width],dataIn[y+offset[2]*width]);
    }
    BORDER_OFFSET(offset, left_top, width, borderType);
    dataOut[0] = HORIZONTAL_COMPUTE_SCALAR_GAUSSIAN(scratch[offset[0]],scratch[offset[1]],scratch[offset[2]]);
    BORDER_OFFSET(offset, middle, width, borderType);
    for(int y =1; y<width-1; y++)
    {
        dataOut[y] = HORIZONTAL_COMPUTE_SCALAR_GAUSSIAN(scratch[y+offset[0]],scratch[y+offset[1]],scratch[y+offset[2]]);
    }
    BORDER_OFFSET(offset, right_bot, width, borderType);
    dataOut[width-1] = HORIZONTAL_COMPUTE_SCALAR_GAUSSIAN(scratch[width-1+offset[0]],scratch[width-1+offset[1]],scratch[width-1+offset[2]]);
    /*      middle part      */
    for(int x = 1; x<height-1; x++)
    {
        BORDER_OFFSET(offset, middle, height, borderType);
        for(int y =0; y<width; y++)
        {
            scratch[y] = VERTICAL_COMPUTE_SCALAR_GAUSSIAN(dataIn[x*width +y+offset[0]*width],dataIn[x*width + y+offset[1]*width],dataIn[x*width + y+offset[2]*width]);//3 5 not good, loop on size of offset
        }
        //two more loop for kernel size 5
        BORDER_OFFSET(offset, left_top, width, borderType);
        dataOut[x*width] = HORIZONTAL_COMPUTE_SCALAR_GAUSSIAN(scratch[offset[0]],scratch[offset[1]],scratch[offset[2]]);
        BORDER_OFFSET(offset, middle, width, borderType);
        for(int y =1; y<width-1; y++)
        {
            dataOut[x*width+y] = HORIZONTAL_COMPUTE_SCALAR_GAUSSIAN(scratch[y+offset[0]],scratch[y+offset[1]],scratch[y+offset[2]]);
        }
        BORDER_OFFSET(offset, right_bot, width, borderType);
        dataOut[x*width+width-1] = HORIZONTAL_COMPUTE_SCALAR_GAUSSIAN(scratch[width-1+offset[0]],scratch[width-1+offset[1]],scratch[width-1+offset[2]]);
    }

    /*      bottom part     */
    int x = height-1;
    BORDER_OFFSET(offset, right_bot, height, borderType);
    for(int y =0; y<width; y++)
    {
        scratch[y] = VERTICAL_COMPUTE_SCALAR_GAUSSIAN(dataIn[x*width +y+offset[0]*width],dataIn[x*width + y+offset[1]*width],dataIn[x*width + y+offset[2]*width]);//3 5 not good, loop on size of offset
    }
    BORDER_OFFSET(offset, left_top, width, borderType);
    dataOut[x*width] = HORIZONTAL_COMPUTE_SCALAR_GAUSSIAN(scratch[offset[0]],scratch[offset[1]],scratch[offset[2]]);
    BORDER_OFFSET(offset, middle, width, borderType);
    for(int y =1; y<width-1; y++)
    {
        dataOut[x*width+y] = HORIZONTAL_COMPUTE_SCALAR_GAUSSIAN(scratch[y+offset[0]],scratch[y+offset[1]],scratch[y+offset[2]]);
    }
    BORDER_OFFSET(offset, right_bot, width, borderType);
    dataOut[x*width+width-1] = HORIZONTAL_COMPUTE_SCALAR_GAUSSIAN(scratch[width-1+offset[0]],scratch[width-1+offset[1]],scratch[width-1+offset[2]]);
}
#endif
