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

int16x8x2_t VerticalCompute(uint8x16_t vect_1, uint8x16_t vect_2, uint8x16_t vect_3);
int8x16_t HorizonCompute(int16x8x2_t vect_1, int16x8x2_t vect_2, int16x8x2_t vect_3);
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

__STATIC_FORCEINLINE int HorizonCompute_sc(int data_0, int data_1, int data_2)
{
    int out = (data_0*0x08 + (data_1*0x10) + data_2*0x08)>>10;
    return(out);
}
__STATIC_FORCEINLINE int VerticalCompute_sc(int data_0, int data_1, int data_2)
{
    int out = data_0*0x08 + (data_1*0x10) + data_2*0x08;
    return(out);
}
#if defined(ARM_MATH_MVEI) && !defined(ARM_MATH_AUTOVECTORIZE)

__STATIC_FORCEINLINE int16x8x2_t VerticalCompute(uint8x16_t vect_1, uint8x16_t vect_2, uint8x16_t vect_3)
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
}
#endif

#if defined(ARM_MATH_MVEI) && !defined(ARM_MATH_AUTOVECTORIZE)

void arm_linear_filter_generic(const arm_cv_image_gray8_t* ImageIn, 
                                   arm_cv_image_gray8_t* ImageOut,
                                   q15_t* Buffer,
                                   int8_t bordertype)
{
    //Vertical treatment
    int w = ImageOut->width;
    int h = ImageOut->height;
    /*      top part        */
    int offset[3];//3 5 no good // putting the offset in constant could make sens for size 3, not scalable thought
    BORDER_OFFSET(offset, left_top, h, bordertype);
    for(int y =0; y<w-15; y+=16)
    {
        uint8x16_t vec1 = vld1q(&ImageIn->pData[y+offset[0]*w]);
        uint8x16_t vec2 = vld1q(&ImageIn->pData[y+offset[1]*w]);
        uint8x16_t vec3 = vld1q(&ImageIn->pData[y+offset[2]*w]);
        //int16x8x2_t vect_res = VerticalCompute(vec1,vec2,vec3);
        int16x8x2_t vect_res;
        VERTICAL_COMPUTE_VECTOR_GAUSSIAN_2(vec1,vec2,vec3, vect_res)
        vst2q(&Buffer[y], vect_res);
    }
    for(int y = w-(w %16); y < w; y ++)
    {
        Buffer[y] = VERTICAL_COMPUTE_SCALAR_GAUSSIAN(ImageIn->pData[y+offset[0]*w],ImageIn->pData[y+offset[1]*w],ImageIn->pData[y+offset[2]*w]);
    }
    // more loop for kernel size 5
    BORDER_OFFSET(offset, left_top, w, bordertype);
    ImageOut->pData[0] = HORIZONTAL_COMPUTE_SCALAR_GAUSSIAN(Buffer[offset[0]],Buffer[offset[1]],Buffer[offset[2]]);
    BORDER_OFFSET(offset, middle, w, bordertype);
    for(int y =1; y<w-16; y+=16)
    {
        int16x8x2_t vec1 = vld2q(&Buffer[y+offset[0]]);
        int16x8x2_t vec2 = vld2q(&Buffer[y+offset[1]]);
        int16x8x2_t vec3 = vld2q(&Buffer[y+offset[2]]);
        //int8x16_t vect_out = HorizonCompute(vec1,vec2,vec3);
        int8x16_t vect_out;
        HORIZONTAL_COMPUTE_VECTOR_GAUSSIAN(vec1,vec2,vec3, vect_out)
        vst1q((int8_t*)&ImageOut->pData[y], vect_out);
    }
    for(int y =w-((w-1)%16); y<w-1; y++)
    {
        ImageOut->pData[y] = HORIZONTAL_COMPUTE_SCALAR_GAUSSIAN(Buffer[y+offset[0]],Buffer[y+offset[1]],Buffer[y+offset[2]]);
    }       
    BORDER_OFFSET(offset, right_bot, w, bordertype);
    ImageOut->pData[w-1] = HORIZONTAL_COMPUTE_SCALAR_GAUSSIAN(Buffer[w-1+offset[0]],Buffer[w-1+offset[1]],Buffer[w-1+offset[2]]);
    /*      middle part      */
    for(int x = 1; x<h-1; x++)
    {
        BORDER_OFFSET(offset, middle, h, bordertype);
        for(int y =0; y<w-15; y+=16)
        {
            uint8x16_t vec1 = vld1q(&ImageIn->pData[x*w + y+offset[0]*w]);
            uint8x16_t vec2 = vld1q(&ImageIn->pData[x*w + y+offset[1]*w]);
            uint8x16_t vec3 = vld1q(&ImageIn->pData[x*w + y+offset[2]*w]);
            int16x8x2_t vect_res;
            VERTICAL_COMPUTE_VECTOR_GAUSSIAN_2(vec1,vec2,vec3,vect_res);
            vst2q(&Buffer[y], vect_res);
        }
        for(int y = w-(w %16); y < w; y ++)
        {
            Buffer[y] = VERTICAL_COMPUTE_SCALAR_GAUSSIAN(ImageIn->pData[x*w + y+offset[0]*w],ImageIn->pData[x*w + y+offset[1]*w],ImageIn->pData[x*w + y+offset[2]*w]);
        }
        // more loop for kernel size 5
        BORDER_OFFSET(offset, left_top, w, bordertype);
        ImageOut->pData[x*w] = HORIZONTAL_COMPUTE_SCALAR_GAUSSIAN(Buffer[offset[0]],Buffer[offset[1]],Buffer[offset[2]]);
        BORDER_OFFSET(offset, middle, w, bordertype);
        for(int y =1; y<w-16; y+=16)
        {
            int16x8x2_t vec1 = vld2q(&Buffer[y+offset[0]]);
            int16x8x2_t vec2 = vld2q(&Buffer[y+offset[1]]);
            int16x8x2_t vec3 = vld2q(&Buffer[y+offset[2]]);
            //int8x16_t vect_out = HorizonCompute(vec1,vec2,vec3);
            int8x16_t vect_out;
            HORIZONTAL_COMPUTE_VECTOR_GAUSSIAN(vec1,vec2,vec3, vect_out)
            vst1q((int8_t*)&ImageOut->pData[x*w + y], vect_out);
        }
        for(int y =w-((w-1)%16); y<w-1; y++)
        {
            ImageOut->pData[x*w + y] = HORIZONTAL_COMPUTE_SCALAR_GAUSSIAN(Buffer[y+offset[0]],Buffer[y+offset[1]],Buffer[y+offset[2]]);
        }
        BORDER_OFFSET(offset, right_bot, w, bordertype);
        ImageOut->pData[x*w+w-1] = HORIZONTAL_COMPUTE_SCALAR_GAUSSIAN(Buffer[w-1+offset[0]],Buffer[w-1+offset[1]],Buffer[w-1+offset[2]]);
    }
    /*      bottom part     */
    int x = h-1;
    BORDER_OFFSET(offset, right_bot, h, bordertype);
    for(int y =0; y<w-15; y+=16)
    {
        uint8x16_t vec1 = vld1q(&ImageIn->pData[x*w + y+offset[0]*w]);
        uint8x16_t vec2 = vld1q(&ImageIn->pData[x*w + y+offset[1]*w]);
        uint8x16_t vec3 = vld1q(&ImageIn->pData[x*w + y+offset[2]*w]);
        //int16x8x2_t vect_res = VerticalCompute(vec1,vec2,vec3);
        int16x8x2_t vect_res;
        VERTICAL_COMPUTE_VECTOR_GAUSSIAN_2(vec1,vec2,vec3,vect_res);
        vst2q(&Buffer[y], vect_res);
    }
    for(int y = w-(w %16); y < w; y ++)
    {
        Buffer[y] = VERTICAL_COMPUTE_SCALAR_GAUSSIAN(ImageIn->pData[x*w + y+offset[0]*w],ImageIn->pData[x*w + y+offset[1]*w],ImageIn->pData[x*w + y+offset[2]*w]);
    }
    // more loop for kernel size 5
    BORDER_OFFSET(offset, left_top, w, bordertype);
    ImageOut->pData[x*w] = HORIZONTAL_COMPUTE_SCALAR_GAUSSIAN(Buffer[offset[0]],Buffer[offset[1]],Buffer[offset[2]]);
    BORDER_OFFSET(offset, middle, w, bordertype);
    for(int y =1; y<w-16; y+=16)
    {
        int16x8x2_t vec1 = vld2q(&Buffer[y+offset[0]]);
        int16x8x2_t vec2 = vld2q(&Buffer[y+offset[1]]);
        int16x8x2_t vec3 = vld2q(&Buffer[y+offset[2]]);
        //int8x16_t vect_out = HorizonCompute(vec1,vec2,vec3);
        int8x16_t vect_out;
        HORIZONTAL_COMPUTE_VECTOR_GAUSSIAN(vec1,vec2,vec3, vect_out)
        vst1q((int8_t*)&ImageOut->pData[x*w + y], vect_out);
    }
    for(int y =w-((w-1)%16); y<w-1; y++)
    {
        ImageOut->pData[x*w + y] = HORIZONTAL_COMPUTE_SCALAR_GAUSSIAN(Buffer[y+offset[0]],Buffer[y+offset[1]],Buffer[y+offset[2]]);
    }
    BORDER_OFFSET(offset, right_bot, w, bordertype);
    ImageOut->pData[x*w+w-1] = HORIZONTAL_COMPUTE_SCALAR_GAUSSIAN(Buffer[w-1+offset[0]],Buffer[w-1+offset[1]],Buffer[w-1+offset[2]]);
}

#else
void arm_linear_filter_generic(const arm_cv_image_gray8_t* ImageIn, 
                                   arm_cv_image_gray8_t* ImageOut,
                                   q15_t* Buffer,
                                   int8_t bordertype)
{
    //Vertical treatment
    int w = ImageOut->width;
    int h = ImageOut->height;
    /*      top part        */
    int offset[3];//3 5 no good
    BORDER_OFFSET(offset, left_top, h, bordertype);
    for(int y =0; y<w; y++)
    {
        Buffer[y] = VERTICAL_COMPUTE_SCALAR_GAUSSIAN(ImageIn->pData[y+offset[0]*w],ImageIn->pData[y+offset[1]*w],ImageIn->pData[y+offset[2]*w]);//3 5 not good, loop on size of offset
    }
    //two more loop for kernel size 5
    BORDER_OFFSET(offset, left_top, w, bordertype);
    ImageOut->pData[0] = HORIZONTAL_COMPUTE_SCALAR_GAUSSIAN(Buffer[offset[0]],Buffer[offset[1]],Buffer[offset[2]]);
    BORDER_OFFSET(offset, middle, w, bordertype);
    for(int y =1; y<w-1; y++)
    {
        ImageOut->pData[y] = HORIZONTAL_COMPUTE_SCALAR_GAUSSIAN(Buffer[y+offset[0]],Buffer[y+offset[1]],Buffer[y+offset[2]]);//3 5 not good, loop on size of offset
    }
    BORDER_OFFSET(offset, right_bot, w, bordertype);
    ImageOut->pData[w-1] = HORIZONTAL_COMPUTE_SCALAR_GAUSSIAN(Buffer[w-1+offset[0]],Buffer[w-1+offset[1]],Buffer[w-1+offset[2]]);
    /*      middle part      */
    for(int x = 1; x<h-1; x++)
    {
        BORDER_OFFSET(offset, middle, h, bordertype);
        for(int y =0; y<w; y++)
        {
            Buffer[y] = VERTICAL_COMPUTE_SCALAR_GAUSSIAN(ImageIn->pData[x*w +y+offset[0]*w],ImageIn->pData[x*w + y+offset[1]*w],ImageIn->pData[x*w + y+offset[2]*w]);//3 5 not good, loop on size of offset
        }
        //two more loop for kernel size 5
        BORDER_OFFSET(offset, left_top, w, bordertype);
        ImageOut->pData[x*w] = HORIZONTAL_COMPUTE_SCALAR_GAUSSIAN(Buffer[offset[0]],Buffer[offset[1]],Buffer[offset[2]]);
        BORDER_OFFSET(offset, middle, w, bordertype);
        for(int y =1; y<w-1; y++)
        {
            ImageOut->pData[x*w+y] = HORIZONTAL_COMPUTE_SCALAR_GAUSSIAN(Buffer[y+offset[0]],Buffer[y+offset[1]],Buffer[y+offset[2]]);//3 5 not good, loop on size of offset
        }
        BORDER_OFFSET(offset, right_bot, w, bordertype);
        ImageOut->pData[x*w+w-1] = HORIZONTAL_COMPUTE_SCALAR_GAUSSIAN(Buffer[w-1+offset[0]],Buffer[w-1+offset[1]],Buffer[w-1+offset[2]]);
    }

    /*      bottom part     */
    int x = h-1;
    BORDER_OFFSET(offset, right_bot, h, bordertype);
    for(int y =0; y<w; y++)
    {
        Buffer[y] = VERTICAL_COMPUTE_SCALAR_GAUSSIAN(ImageIn->pData[x*w +y+offset[0]*w],ImageIn->pData[x*w + y+offset[1]*w],ImageIn->pData[x*w + y+offset[2]*w]);//3 5 not good, loop on size of offset
    }
    //two more loop for kernel size 5
    BORDER_OFFSET(offset, left_top, w, bordertype);
    ImageOut->pData[x*w] = HORIZONTAL_COMPUTE_SCALAR_GAUSSIAN(Buffer[offset[0]],Buffer[offset[1]],Buffer[offset[2]]);
    BORDER_OFFSET(offset, middle, w, bordertype);
    for(int y =1; y<w-1; y++)
    {
        ImageOut->pData[x*w+y] = HORIZONTAL_COMPUTE_SCALAR_GAUSSIAN(Buffer[y+offset[0]],Buffer[y+offset[1]],Buffer[y+offset[2]]);//3 5 not good, loop on size of offset
    }
    BORDER_OFFSET(offset, right_bot, w, bordertype);
    ImageOut->pData[x*w+w-1] = HORIZONTAL_COMPUTE_SCALAR_GAUSSIAN(Buffer[w-1+offset[0]],Buffer[w-1+offset[1]],Buffer[w-1+offset[2]]);
}
#endif
