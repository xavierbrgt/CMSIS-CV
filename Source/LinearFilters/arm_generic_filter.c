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

#if defined(ARM_MATH_MVEI) && !defined(ARM_MATH_AUTOVECTORIZE)
int VerticalCompute_sc(int data_0, int data_1, int data_2)
{
    int out = data_0*0x08 + (data_1*0x10) + data_2*0x08;
    return(out);
}
int16x8x2_t VerticalCompute(uint8x16_t vect_1, uint8x16_t vect_2, uint8x16_t vect_3)
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
int8x16_t HorizonCompute(int16x8x2_t vect_1, int16x8x2_t vect_2, int16x8x2_t vect_3)
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
void top_border(int indice, int* offset_list, int width)
{
    (void)indice;//use in case of 5 sized kernel
    offset_list[0] = 0;
    offset_list[1] = 0;
    offset_list[2] = width;
}
void bot_border(int indice, int* offset_list, int width)
{
    (void)indice;//use in case of 5 sized kernel
    offset_list[0] = -width;
    offset_list[1] = 0;
    offset_list[2] = 0;
}
void left_border(int indice, int* offset_list)
{
    (void)indice;//use in case of 5 sized kernel
    offset_list[0] = 0;
    offset_list[1] = 0;
    offset_list[2] = 1;
}
void right_border(int indice, int* offset_list)
{
    (void)indice;//use in case of 5 sized kernel
    offset_list[0] = -1;
    offset_list[1] = 0;
    offset_list[2] = 0;
}
void middle_h(int* offset_list)
{
    offset_list[0] = -1;
    offset_list[1] = 0;
    offset_list[2] = 1;
}
void middle_v(int* offset_list, int width)
{
    offset_list[0] = -width;
    offset_list[1] = 0;
    offset_list[2] = width;
}
int HorizonCompute_sc(int data_0, int data_1, int data_2)
{
    int out = data_0*0x08 + (data_1*0x10) + data_2*0x08;
    return(out);
}

void arm_linear_filter_generic(const arm_cv_image_gray8_t* ImageIn, 
                                   arm_cv_image_gray8_t* ImageOut,
                                   q15_t* Buffer
                                   /*bordertype*/)
{
    //Vertical treatment
    int w = ImageOut->width;
    int offset_h[3];
    int offset_v[3];
    middle_h(&offset_h[0]);
    middle_v(&offset_v[0], w);

    /*      top part        */
    int offset[3];//3 5 no good // putting the offset in constant could make sens for size 3, not scalable thought
    top_border(0, &offset[0], w);
    for(int y =0; y<w-15; y+=16)
    {
        //kernel 5 different
        uint8x16_t vec1 = vld1q(&ImageIn->pData[y+offset[0]]);
        uint8x16_t vec2 = vld1q(&ImageIn->pData[y+offset[1]]);
        uint8x16_t vec3 = vld1q(&ImageIn->pData[y+offset[2]]);
        int16x8x2_t vect_res = VerticalCompute(vec1,vec2,vec3);
        vst2q(&Buffer[y], vect_res);
    }
    for(int y = w-(w %16); y < w; y ++)
    {
        Buffer[y] = VerticalCompute_sc(ImageIn->pData[y+offset[0]],ImageIn->pData[y+offset[1]],ImageIn->pData[y+offset[2]]);
    }

    //two more loop for kernel size 5
    left_border(0, &offset[0]);
    ImageOut->pData[0] = HorizonCompute_sc(Buffer[0+offset[0]],Buffer[0+offset[1]],Buffer[0+offset[2]])>>10;
    for(int y =1; y<w-16; y+=16)
    {
        int16x8x2_t vec1 = vld2q(&Buffer[y+offset_h[0]]);
        int16x8x2_t vec2 = vld2q(&Buffer[y+offset_h[1]]);
        int16x8x2_t vec3 = vld2q(&Buffer[y+offset_h[2]]);
        int8x16_t vect_out = HorizonCompute(vec1,vec2,vec3);
        vst1q((int8_t*)&ImageOut->pData[y], vect_out);
    }
    for(int y =w-1-((w-1)%16); y<w-1; y++)
    {
        ImageOut->pData[y] = HorizonCompute_sc(Buffer[y+offset_h[0]],Buffer[y+offset_h[1]],Buffer[y+offset_h[2]])>>10;
    }
    right_border(0, &offset[0]);
    ImageOut->pData[w-1] = HorizonCompute_sc(Buffer[w-1 + offset[0]],Buffer[w-1 + offset[1]],Buffer[w-1 +offset[2]])>>10;
    
    /*      middle part      */
    for(int x = 1; x<ImageOut->height-1; x++)
    {
        for(int y =0; y<w-15; y+=16)
        {
            uint8x16_t vec1 = vld1q(&ImageIn->pData[x*w + y+offset_v[0]]);
            uint8x16_t vec2 = vld1q(&ImageIn->pData[x*w + y+offset_v[1]]);
            uint8x16_t vec3 = vld1q(&ImageIn->pData[x*w + y+offset_v[2]]);
            int16x8x2_t vect_res = VerticalCompute(vec1,vec2,vec3);
            vst2q(&Buffer[y], vect_res);
        }
        for(int y = w-(w %16); y < w; y ++)
        {
            Buffer[y] = VerticalCompute_sc(ImageIn->pData[x*w + y+offset_v[0]],ImageIn->pData[x*w + y+offset_v[1]],ImageIn->pData[x*w + y+offset_v[2]]);
        }

        // more loop for kernel size 5
        left_border(0, &offset[0]);
        ImageOut->pData[x*w] = HorizonCompute_sc(Buffer[offset[0]],Buffer[offset[1]],Buffer[offset[2]])>>10;
        for(int y =1; y<w-16; y+=16)
        {
            int16x8x2_t vec1 = vld2q(&Buffer[y+offset_h[0]]);
            int16x8x2_t vec2 = vld2q(&Buffer[y+offset_h[1]]);
            int16x8x2_t vec3 = vld2q(&Buffer[y+offset_h[2]]);
            int8x16_t vect_out = HorizonCompute(vec1,vec2,vec3);
            vst1q((int8_t*)&ImageOut->pData[x*w + y], vect_out);
        }
        for(int y =w-1-((w-1)%16); y<w-1; y++)
        {
            ImageOut->pData[x*w + y] = HorizonCompute_sc(Buffer[y+offset_h[0]],Buffer[y+offset_h[1]],Buffer[y+offset_h[2]])>>10;
        }
        right_border(0, &offset[0]);
        ImageOut->pData[x*w+w-1] = HorizonCompute_sc(Buffer[w-1+offset[0]],Buffer[w-1+offset[1]],Buffer[w-1+offset[2]])>>10;
    }
    /*      bottom part     */
    int x = ImageOut->height-1;
    bot_border(0, &offset[0], w);
    for(int y =0; y<w-15; y+=16)
    {
        uint8x16_t vec1 = vld1q(&ImageIn->pData[x*w + y+offset[0]]);
        uint8x16_t vec2 = vld1q(&ImageIn->pData[x*w + y+offset[1]]);
        uint8x16_t vec3 = vld1q(&ImageIn->pData[x*w + y+offset[2]]);
        int16x8x2_t vect_res = VerticalCompute(vec1,vec2,vec3);
        vst2q(&Buffer[y], vect_res);
    }
    for(int y = w-(w %16); y < w; y ++)
    {
        Buffer[y] = VerticalCompute_sc(ImageIn->pData[x*w + y+offset[0]],ImageIn->pData[x*w + y+offset[1]],ImageIn->pData[x*w + y+offset[2]]);
    }
    //two more loop for kernel size 5
    left_border(0, &offset[0]);
    ImageOut->pData[x*w] = HorizonCompute_sc(Buffer[0+offset[0]],Buffer[0+offset[1]],Buffer[0+offset[2]])>>10;
    for(int y =1; y<w-16; y+=16)
    {
        int16x8x2_t vec1 = vld2q(&Buffer[y+offset[0]]);
        int16x8x2_t vec2 = vld2q(&Buffer[y+offset[1]]);
        int16x8x2_t vec3 = vld2q(&Buffer[y+offset[2]]);
        int8x16_t vect_out = HorizonCompute(vec1,vec2,vec3);
        vst1q((int8_t*)&ImageOut->pData[x*w + y], vect_out);
    }
    for(int y =w-1-((w-1)%16); y<w-1; y++)
    {
        ImageOut->pData[x*w + y] = HorizonCompute_sc(Buffer[y+offset[0]],Buffer[y+offset[1]],Buffer[y+offset[2]])>>10;
    }
    right_border(0, &offset[0]);
    ImageOut->pData[x*w +  w -1] = HorizonCompute_sc(Buffer[w-1+offset[0]],Buffer[w-1+offset[1]],Buffer[w-1+offset[2]])>>10;
}
#else
int VerticalCompute(int data_0, int data_1, int data_2)
{
    int out = data_0*0x08 + data_1*0x10 + data_2*0x08;
    return(out);
}
//gaussian sym so same as vertical
int HorizonCompute(int data_0, int data_1, int data_2)
{
    int out = (data_0*0x08 + data_1*0x10 + data_2*0x08)>>10;
    return(out);
}
void top_border(int indice, int* offset_list, int width)
{
    (void)indice;//use in case of 5 sized kernel
    offset_list[0] = 0;
    offset_list[1] = 0;
    offset_list[2] = width;
}
void bot_border(int indice, int* offset_list, int width)
{
    (void)indice;//use in case of 5 sized kernel
    offset_list[0] = -width;
    offset_list[1] = 0;
    offset_list[2] = 0;
}
void left_border(int indice, int* offset_list)
{
    (void)indice;//use in case of 5 sized kernel
    offset_list[0] = 0;
    offset_list[1] = 0;
    offset_list[2] = 1;
}
void right_border(int indice, int* offset_list)
{
    (void)indice;//use in case of 5 sized kernel
    offset_list[0] = -1;
    offset_list[1] = 0;
    offset_list[2] = 0;
}
void middle_h(int* offset_list)
{
    offset_list[0] = -1;
    offset_list[1] = 0;
    offset_list[2] = 1;
}
void middle_v(int* offset_list, int width)
{
    offset_list[0] = -width;
    offset_list[1] = 0;
    offset_list[2] = width;
}

void arm_linear_filter_generic(const arm_cv_image_gray8_t* ImageIn, 
                                   arm_cv_image_gray8_t* ImageOut,
                                   q15_t* Buffer)
{
    
    //Vertical treatment
    int w = ImageOut->width;
    int offset_h[3];
    int offset_v[3];
    middle_h(&offset_h[0]);
    middle_v(&offset_v[0], w);

    /*      top part        */
    int offset[3];//3 5 no good
    top_border(0, &offset[0], w);
    for(int y =0; y<w; y++)
    {
        Buffer[y] = VerticalCompute(ImageIn->pData[y+offset[0]],ImageIn->pData[y+offset[1]],ImageIn->pData[y+offset[2]]);//3 5 not good, loop on size of offset
    }
    //two more loop for kernel size 5
    left_border(0, &offset[0]);
    ImageOut->pData[0] = HorizonCompute(Buffer[0+offset[0]],Buffer[0+offset[1]],Buffer[0+offset[2]]);
    for(int y =1/*margin_size*/; y<w-1/*margin_size*/; y++)
    {
        ImageOut->pData[y] = HorizonCompute(Buffer[y+offset_h[0]],Buffer[y+offset_h[1]],Buffer[y+offset_h[2]]);//3 5 not good, loop on size of offset
    }
    right_border(0, &offset[0]);
    ImageOut->pData[w-1] = HorizonCompute(Buffer[w-1 + offset[0]],Buffer[w-1 + offset[1]],Buffer[w-1 +offset[2]]);
    
    /*      middle part      */
    for(int x = 1; x<ImageOut->height-1; x++)
    {
        for(int y =0; y<w; y++)
        {
            Buffer[y] = VerticalCompute(ImageIn->pData[x*w +y+offset_v[0]],ImageIn->pData[x*w + y+offset_v[1]],ImageIn->pData[x*w + y+offset_v[2]]);//3 5 not good, loop on size of offset
        }
        //two more loop for kernel size 5
        left_border(0, &offset[0]);
        ImageOut->pData[x*w] = HorizonCompute(Buffer[offset[0]],Buffer[offset[1]],Buffer[offset[2]]);
        for(int y =1; y<w-1; y++)
        {
            ImageOut->pData[x*w+y] = HorizonCompute(Buffer[y+offset_h[0]],Buffer[y+offset_h[1]],Buffer[y+offset_h[2]]);//3 5 not good, loop on size of offset
        }
        right_border(0, &offset[0]);
        ImageOut->pData[x*w+w-1] = HorizonCompute(Buffer[w-1+offset[0]],Buffer[w-1+offset[1]],Buffer[w-1+offset[2]]);
    }

    /*      bottom part     */
    int x = ImageOut->height-1;
    bot_border(0, &offset[0], w);
    for(int y =0; y<w; y++)
    {
        Buffer[y] = VerticalCompute(ImageIn->pData[x*w + y+offset[0]],ImageIn->pData[x*w + y+offset[1]],ImageIn->pData[x*w + y+offset[2]]);//3 5 not good, loop on size of offset
    }
    //two more loop for kernel size 5
    left_border(0, &offset[0]);
    ImageOut->pData[x*w] = HorizonCompute(Buffer[0+offset[0]],Buffer[0+offset[1]],Buffer[0+offset[2]]);
    for(int y =1; y<w-1; y++)
    {
        ImageOut->pData[x*w+y] = HorizonCompute(Buffer[y+offset_h[0]],Buffer[y+offset_h[1]],Buffer[y+offset_h[2]]);//3 5 not good, loop on size of offset
    }
    right_border(0, &offset[0]);
    ImageOut->pData[x*w +  w -1] = HorizonCompute(Buffer[w-1+offset[0]],Buffer[w-1+offset[1]],Buffer[w-1+offset[2]]);
}
#endif
