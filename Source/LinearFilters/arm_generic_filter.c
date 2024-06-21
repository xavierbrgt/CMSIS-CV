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
#include <stdio.h>
#include "arm_linear_filter_common.h"

__STATIC_FORCEINLINE int VerticalCompute_sc(int data_0, int data_1, int data_2)
{
    int out = data_0*0x08 + (data_1*0x10) + data_2*0x08;
    return(out);
}


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
    border_offset(&offset[0], left_top, h, bordertype);
    for(int y =0; y<w-15; y+=16)
    {
        uint8x16_t vec1 = vld1q(&ImageIn->pData[y+offset[0]*w]);
        uint8x16_t vec2 = vld1q(&ImageIn->pData[y+offset[1]*w]);
        uint8x16_t vec3 = vld1q(&ImageIn->pData[y+offset[2]*w]);
        int16x8x2_t vect_res = VerticalCompute(vec1,vec2,vec3);
        vst2q(&Buffer[y], vect_res);
    }
    for(int y = w-(w %16); y < w; y ++)
    {
        Buffer[y] = VerticalCompute_sc(ImageIn->pData[y+offset[0]*w],ImageIn->pData[y+offset[1]*w],ImageIn->pData[y+offset[2]*w]);
    }
    for(int y = 0; y < w; y ++)
    {
        Buffer[y] = VerticalCompute_sc(ImageIn->pData[y+offset[0]*w],ImageIn->pData[y+offset[1]*w],ImageIn->pData[y+offset[2]*w]);
    }
    // more loop for kernel size 5
    border_offset(&offset[0], left_top, w, bordertype);
    ImageOut->pData[0] = HorizonCompute_sc(Buffer[offset[0]],Buffer[offset[1]],Buffer[offset[2]]);
    border_offset(&offset[0], middle, w, bordertype);
    for(int y =1; y<w-16; y+=16)
    {
        int16x8x2_t vec1 = vld2q(&Buffer[y+offset[0]]);
        int16x8x2_t vec2 = vld2q(&Buffer[y+offset[1]]);
        int16x8x2_t vec3 = vld2q(&Buffer[y+offset[2]]);
        int8x16_t vect_out = HorizonCompute(vec1,vec2,vec3);
        vst1q((int8_t*)&ImageOut->pData[y], vect_out);
    }
    for(int y =w-((w-1)%16); y<w-1; y++)
    {
        ImageOut->pData[y] = HorizonCompute_sc(Buffer[y+offset[0]],Buffer[y+offset[1]],Buffer[y+offset[2]]);
    }
    for(int y =1; y<w-1; y++)
    {
        ImageOut->pData[y] = HorizonCompute_sc(Buffer[y+offset[0]],Buffer[y+offset[1]],Buffer[y+offset[2]]);
    }        
    border_offset(&offset[0], right_bot, w, bordertype);
    ImageOut->pData[w-1] = HorizonCompute_sc(Buffer[w-1+offset[0]],Buffer[w-1+offset[1]],Buffer[w-1+offset[2]]);
    /*      middle part      */
    for(int x = 1; x<h-1; x++)
    {
        border_offset(&offset[0], middle, h, bordertype);
        for(int y =0; y<w-15; y+=16)
        {
            uint8x16_t vec1 = vld1q(&ImageIn->pData[x*w + y+offset[0]*w]);
            uint8x16_t vec2 = vld1q(&ImageIn->pData[x*w + y+offset[1]*w]);
            uint8x16_t vec3 = vld1q(&ImageIn->pData[x*w + y+offset[2]*w]);
            int16x8x2_t vect_res = VerticalCompute(vec1,vec2,vec3);
            vst2q(&Buffer[y], vect_res);
        }
        for(int y = w-(w %16); y < w; y ++)
        {
            Buffer[y] = VerticalCompute_sc(ImageIn->pData[x*w + y+offset[0]*w],ImageIn->pData[x*w + y+offset[1]*w],ImageIn->pData[x*w + y+offset[2]*w]);
        }
        // more loop for kernel size 5
        border_offset(&offset[0], left_top, w, bordertype);
        ImageOut->pData[x*w] = HorizonCompute_sc(Buffer[offset[0]],Buffer[offset[1]],Buffer[offset[2]]);
        border_offset(&offset[0], middle, w, bordertype);
        for(int y =1; y<w-16; y+=16)
        {
            int16x8x2_t vec1 = vld2q(&Buffer[y+offset[0]]);
            int16x8x2_t vec2 = vld2q(&Buffer[y+offset[1]]);
            int16x8x2_t vec3 = vld2q(&Buffer[y+offset[2]]);
            int8x16_t vect_out = HorizonCompute(vec1,vec2,vec3);
            vst1q((int8_t*)&ImageOut->pData[x*w + y], vect_out);
        }
        for(int y =w-((w-1)%16); y<w-1; y++)
        {
            ImageOut->pData[x*w + y] = HorizonCompute_sc(Buffer[y+offset[0]],Buffer[y+offset[1]],Buffer[y+offset[2]]);
        }
        border_offset(&offset[0], right_bot, w, bordertype);
        ImageOut->pData[x*w+w-1] = HorizonCompute_sc(Buffer[w-1+offset[0]],Buffer[w-1+offset[1]],Buffer[w-1+offset[2]]);
    }
    /*      bottom part     */
    int x = h-1;
    border_offset(&offset[0], right_bot, h, bordertype);
    for(int y =0; y<w-15; y+=16)
    {
        uint8x16_t vec1 = vld1q(&ImageIn->pData[x*w + y+offset[0]*w]);
        uint8x16_t vec2 = vld1q(&ImageIn->pData[x*w + y+offset[1]*w]);
        uint8x16_t vec3 = vld1q(&ImageIn->pData[x*w + y+offset[2]*w]);
        int16x8x2_t vect_res = VerticalCompute(vec1,vec2,vec3);
        vst2q(&Buffer[y], vect_res);
    }
    for(int y = w-(w %16); y < w; y ++)
    {
        Buffer[y] = VerticalCompute_sc(ImageIn->pData[x*w + y+offset[0]*w],ImageIn->pData[x*w + y+offset[1]*w],ImageIn->pData[x*w + y+offset[2]*w]);
    }
    // more loop for kernel size 5
    border_offset(&offset[0], left_top, w, bordertype);
    ImageOut->pData[x*w] = HorizonCompute_sc(Buffer[offset[0]],Buffer[offset[1]],Buffer[offset[2]]);
    border_offset(&offset[0], middle, w, bordertype);
    for(int y =1; y<w-16; y+=16)
    {
        int16x8x2_t vec1 = vld2q(&Buffer[y+offset[0]]);
        int16x8x2_t vec2 = vld2q(&Buffer[y+offset[1]]);
        int16x8x2_t vec3 = vld2q(&Buffer[y+offset[2]]);
        int8x16_t vect_out = HorizonCompute(vec1,vec2,vec3);
        vst1q((int8_t*)&ImageOut->pData[x*w + y], vect_out);
    }
    for(int y =w-((w-1)%16); y<w-1; y++)
    {
        ImageOut->pData[x*w + y] = HorizonCompute_sc(Buffer[y+offset[0]],Buffer[y+offset[1]],Buffer[y+offset[2]]);
    }
    border_offset(&offset[0], right_bot, w, bordertype);
    ImageOut->pData[x*w+w-1] = HorizonCompute_sc(Buffer[w-1+offset[0]],Buffer[w-1+offset[1]],Buffer[w-1+offset[2]]);
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
    border_offset(&offset[0], left_top, h, bordertype);
    for(int y =0; y<w; y++)
    {
        Buffer[y] = VerticalCompute_sc(ImageIn->pData[y+offset[0]*w],ImageIn->pData[y+offset[1]*w],ImageIn->pData[y+offset[2]*w]);//3 5 not good, loop on size of offset
    }
    //two more loop for kernel size 5
    border_offset(&offset[0], left_top, w, bordertype);
    ImageOut->pData[0] = HorizonCompute_sc(Buffer[offset[0]],Buffer[offset[1]],Buffer[offset[2]]);
    border_offset(&offset[0], middle, w, bordertype);
    for(int y =1; y<w-1; y++)
    {
        ImageOut->pData[y] = HorizonCompute_sc(Buffer[y+offset[0]],Buffer[y+offset[1]],Buffer[y+offset[2]]);//3 5 not good, loop on size of offset
    }
    border_offset(&offset[0], right_bot, w, bordertype);
    ImageOut->pData[w-1] = HorizonCompute_sc(Buffer[w-1+offset[0]],Buffer[w-1+offset[1]],Buffer[w-1+offset[2]]);
    /*      middle part      */
    for(int x = 1; x<h-1; x++)
    {
        border_offset(&offset[0], middle, h, bordertype);
        for(int y =0; y<w; y++)
        {
            Buffer[y] = VerticalCompute_sc(ImageIn->pData[x*w +y+offset[0]*w],ImageIn->pData[x*w + y+offset[1]*w],ImageIn->pData[x*w + y+offset[2]*w]);//3 5 not good, loop on size of offset
        }
        //two more loop for kernel size 5
        border_offset(&offset[0], left_top, w, bordertype);
        ImageOut->pData[x*w] = HorizonCompute_sc(Buffer[offset[0]],Buffer[offset[1]],Buffer[offset[2]]);
        border_offset(&offset[0], middle, w, bordertype);
        for(int y =1; y<w-1; y++)
        {
            ImageOut->pData[x*w+y] = HorizonCompute_sc(Buffer[y+offset[0]],Buffer[y+offset[1]],Buffer[y+offset[2]]);//3 5 not good, loop on size of offset
        }
        border_offset(&offset[0], right_bot, w, bordertype);
        ImageOut->pData[x*w+w-1] = HorizonCompute_sc(Buffer[w-1+offset[0]],Buffer[w-1+offset[1]],Buffer[w-1+offset[2]]);
    }

    /*      bottom part     */
    int x = h-1;
    border_offset(&offset[0], right_bot, h, bordertype);
    for(int y =0; y<w; y++)
    {
        Buffer[y] = VerticalCompute_sc(ImageIn->pData[x*w +y+offset[0]*w],ImageIn->pData[x*w + y+offset[1]*w],ImageIn->pData[x*w + y+offset[2]*w]);//3 5 not good, loop on size of offset
    }
    //two more loop for kernel size 5
    border_offset(&offset[0], left_top, w, bordertype);
    ImageOut->pData[x*w] = HorizonCompute_sc(Buffer[offset[0]],Buffer[offset[1]],Buffer[offset[2]]);
    border_offset(&offset[0], middle, w, bordertype);
    for(int y =1; y<w-1; y++)
    {
        ImageOut->pData[x*w+y] = HorizonCompute_sc(Buffer[y+offset[0]],Buffer[y+offset[1]],Buffer[y+offset[2]]);//3 5 not good, loop on size of offset
    }
    border_offset(&offset[0], right_bot, w, bordertype);
    ImageOut->pData[x*w+w-1] = HorizonCompute_sc(Buffer[w-1+offset[0]],Buffer[w-1+offset[1]],Buffer[w-1+offset[2]]);
}
#endif
