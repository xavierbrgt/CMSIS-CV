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
#include <stdio.h>

//q2_13_t;

#define HORIZONTAL_COMPUTE_SCALAR_SOBEL(data_0, data_1, data_2) (data_0 + (data_1*2) + data_2)
#define VERTICAL_COMPUTE_SCALAR_SOBEL(data_0, data_1, data_2) (-(data_0<<5) + (data_2<<5))

#if defined(ARM_MATH_MVEI) && !defined(ARM_MATH_AUTOVECTORIZE)

#define VERTICAL_COMPUTE_VECTOR_SOBEL(vect_1, vect_2, vect_3, vect_res) int16x8x2_t vect_3x2;\
    (void)vect_2;\
    vect_res.val[0] = vshllbq(vect_1,5);\
    vect_3x2.val[0] = vshllbq(vect_3,5);\
    vect_res.val[0] = vsubq(vect_3x2.val[0], vect_res.val[0]);\
    vect_res.val[1] = vshlltq(vect_1,5);\
    vect_3x2.val[1] = vshlltq(vect_3,5);\
    vect_res.val[1] = vsubq(vect_3x2.val[1], vect_res.val[1]);

#define HORIZONTAL_COMPUTE_VECTOR_SOBEL(vect_1, vect_2, vect_3, vect_out)     vect_1 = vaddq(vect_1, vect_3);\
    vect_2 = vshlq_n(vect_2,1);\
    vect_out = vaddq(vect_2, vect_1);

#endif
#if defined(ARM_MATH_MVEI) && !defined(ARM_MATH_AUTOVECTORIZE)



void arm_sobel_x(const arm_cv_image_gray8_t* ImageIn, 
                                   arm_cv_image_q15_t* ImageOut,
                                   q15_t* Buffer,
                                   int8_t bordertype)
{
    //Vertical treatment
    int w = ImageOut->width;
    int h = ImageOut->height;

    int offset[3];//3 5 no good // putting the offset in constant could make sens for size 3, not scalable thought
    BORDER_OFFSET(offset, left_top, h, bordertype);
    for(int y =0; y<w-15; y+=16)
    {
        uint8x16_t vec1 = vld1q(&ImageIn->pData[y+offset[0]*w]);
        uint8x16_t vec2 = vld1q(&ImageIn->pData[y+offset[1]*w]);
        uint8x16_t vec3 = vld1q(&ImageIn->pData[y+offset[2]*w]);
        //int16x8x2_t vect_res = VerticalCompute(vec1,vec2,vec3);
        int16x8x2_t vect_res;
        VERTICAL_COMPUTE_VECTOR_SOBEL(vec1,vec2,vec3, vect_res)
        vst2q(&Buffer[y], vect_res);
    }
    for(int y = w-(w %16); y < w; y ++)
    {
        Buffer[y] = VERTICAL_COMPUTE_SCALAR_SOBEL(ImageIn->pData[y+offset[0]*w],ImageIn->pData[y+offset[1]*w],ImageIn->pData[y+offset[2]*w]);
    }
    // more loop for kernel size 5
    BORDER_OFFSET(offset, left_top, w, bordertype);
    ImageOut->pData[0] = HORIZONTAL_COMPUTE_SCALAR_SOBEL(Buffer[offset[0]],Buffer[offset[1]],Buffer[offset[2]]);
    BORDER_OFFSET(offset, middle, w, bordertype);
    for(int y =1; y<w-8; y+=8)
    {
        int16x8_t vec1 = vld1q(&Buffer[y+offset[0]]);
        int16x8_t vec2 = vld1q(&Buffer[y+offset[1]]);
        int16x8_t vec3 = vld1q(&Buffer[y+offset[2]]);
        //int8x16_t vect_out = HorizonCompute(vec1,vec2,vec3);
        int16x8_t vect_out;
        HORIZONTAL_COMPUTE_VECTOR_SOBEL(vec1,vec2,vec3, vect_out)
        vst1q(&ImageOut->pData[y], vect_out);
    }
    for(int y =w-((w-1)%8); y<w-1; y++)
    {
        ImageOut->pData[y] = HORIZONTAL_COMPUTE_SCALAR_SOBEL(Buffer[y+offset[0]],Buffer[y+offset[1]],Buffer[y+offset[2]]);
    }     
    BORDER_OFFSET(offset, right_bot, w, bordertype);
    ImageOut->pData[w-1] = HORIZONTAL_COMPUTE_SCALAR_SOBEL(Buffer[w-1+offset[0]],Buffer[w-1+offset[1]],Buffer[w-1+offset[2]]);

    for(int x = 1; x<h-1; x++)
    {
        BORDER_OFFSET(offset, middle, h, bordertype);
        for(int y =0; y<w-15; y+=16)
        {
            uint8x16_t vec1 = vld1q(&ImageIn->pData[x*w + y+offset[0]*w]);
            uint8x16_t vec2 = vld1q(&ImageIn->pData[x*w + y+offset[1]*w]);
            uint8x16_t vec3 = vld1q(&ImageIn->pData[x*w + y+offset[2]*w]);
            int16x8x2_t vect_res;
            VERTICAL_COMPUTE_VECTOR_SOBEL(vec1,vec2,vec3,vect_res);
            vst2q(&Buffer[y], vect_res);
        }
        for(int y = w-(w %16); y < w; y ++)
        {
            Buffer[y] = VERTICAL_COMPUTE_SCALAR_SOBEL(ImageIn->pData[x*w + y+offset[0]*w],ImageIn->pData[x*w + y+offset[1]*w],ImageIn->pData[x*w + y+offset[2]*w]);
        }
        // more loop for kernel size 5
        BORDER_OFFSET(offset, left_top, w, bordertype);
        ImageOut->pData[x*w] = HORIZONTAL_COMPUTE_SCALAR_SOBEL(Buffer[offset[0]],Buffer[offset[1]],Buffer[offset[2]]);
        BORDER_OFFSET(offset, middle, w, bordertype);
        for(int y =1; y<w-8; y+=8)
        {
            int16x8_t vec1 = vld1q(&Buffer[y+offset[0]]);
            int16x8_t vec2 = vld1q(&Buffer[y+offset[1]]);
            int16x8_t vec3 = vld1q(&Buffer[y+offset[2]]);
            //int8x16_t vect_out = HorizonCompute(vec1,vec2,vec3);
            int16x8_t vect_out;
            HORIZONTAL_COMPUTE_VECTOR_SOBEL(vec1,vec2,vec3, vect_out)
            vst1q(&ImageOut->pData[x*w + y], vect_out);
        }
        for(int y =w-((w-1)%8); y<w-1; y++)
        {
            ImageOut->pData[x*w + y] = HORIZONTAL_COMPUTE_SCALAR_SOBEL(Buffer[y+offset[0]],Buffer[y+offset[1]],Buffer[y+offset[2]]);
        }
        BORDER_OFFSET(offset, right_bot, w, bordertype);
        ImageOut->pData[x*w+w-1] = HORIZONTAL_COMPUTE_SCALAR_SOBEL(Buffer[w-1+offset[0]],Buffer[w-1+offset[1]],Buffer[w-1+offset[2]]);
    }

    int x = h-1;
    BORDER_OFFSET(offset, right_bot, h, bordertype);
    for(int y =0; y<w-15; y+=16)
    {
        uint8x16_t vec1 = vld1q(&ImageIn->pData[x*w + y+offset[0]*w]);
        uint8x16_t vec2 = vld1q(&ImageIn->pData[x*w + y+offset[1]*w]);
        uint8x16_t vec3 = vld1q(&ImageIn->pData[x*w + y+offset[2]*w]);
        //int16x8x2_t vect_res = VerticalCompute(vec1,vec2,vec3);
        int16x8x2_t vect_res;
        VERTICAL_COMPUTE_VECTOR_SOBEL(vec1,vec2,vec3,vect_res);
        vst2q(&Buffer[y], vect_res);
    }
    for(int y = w-(w %16); y < w; y ++)
    {
        Buffer[y] = VERTICAL_COMPUTE_SCALAR_SOBEL(ImageIn->pData[x*w + y+offset[0]*w],ImageIn->pData[x*w + y+offset[1]*w],ImageIn->pData[x*w + y+offset[2]*w]);
    }
    // more loop for kernel size 5
    BORDER_OFFSET(offset, left_top, w, bordertype);
    ImageOut->pData[x*w] = HORIZONTAL_COMPUTE_SCALAR_SOBEL(Buffer[offset[0]],Buffer[offset[1]],Buffer[offset[2]]);
    BORDER_OFFSET(offset, middle, w, bordertype);
    for(int y =1; y<w-8; y+=8)
    {
        int16x8_t vec1 = vld1q(&Buffer[y+offset[0]]);
        int16x8_t vec2 = vld1q(&Buffer[y+offset[1]]);
        int16x8_t vec3 = vld1q(&Buffer[y+offset[2]]);
        //int8x16_t vect_out = HorizonCompute(vec1,vec2,vec3);
        int16x8_t vect_out;
        HORIZONTAL_COMPUTE_VECTOR_SOBEL(vec1,vec2,vec3, vect_out)
        vst1q(&ImageOut->pData[x*w + y], vect_out);
    }
    for(int y =w-((w-1)%8); y<w-1; y++)
    {
        ImageOut->pData[x*w + y] = HORIZONTAL_COMPUTE_SCALAR_SOBEL(Buffer[y+offset[0]],Buffer[y+offset[1]],Buffer[y+offset[2]]);
    }
    BORDER_OFFSET(offset, right_bot, w, bordertype);
    ImageOut->pData[x*w+w-1] = HORIZONTAL_COMPUTE_SCALAR_SOBEL(Buffer[w-1+offset[0]],Buffer[w-1+offset[1]],Buffer[w-1+offset[2]]);
}

#else
void arm_sobel_x(const arm_cv_image_gray8_t* ImageIn, 
                                   arm_cv_image_q15_t* ImageOut,
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
        Buffer[y] = VERTICAL_COMPUTE_SCALAR_SOBEL(ImageIn->pData[y+offset[0]*w],ImageIn->pData[y+offset[1]*w],ImageIn->pData[y+offset[2]*w]);//3 5 not good, loop on size of offset
    }
    //two more loop for kernel size 5
    BORDER_OFFSET(offset, left_top, w, bordertype);
    ImageOut->pData[0] = HORIZONTAL_COMPUTE_SCALAR_SOBEL(Buffer[offset[0]],Buffer[offset[1]],Buffer[offset[2]]);
    BORDER_OFFSET(offset, middle, w, bordertype);
    for(int y =1; y<w-1; y++)
    {
        ImageOut->pData[y] = HORIZONTAL_COMPUTE_SCALAR_SOBEL(Buffer[y+offset[0]],Buffer[y+offset[1]],Buffer[y+offset[2]]);//3 5 not good, loop on size of offset
    }
    BORDER_OFFSET(offset, right_bot, w, bordertype);
    ImageOut->pData[w-1] = HORIZONTAL_COMPUTE_SCALAR_SOBEL(Buffer[w-1+offset[0]],Buffer[w-1+offset[1]],Buffer[w-1+offset[2]]);
    /*      middle part      */
    for(int x = 1; x<h-1; x++)
    {
        BORDER_OFFSET(offset, middle, h, bordertype);
        for(int y =0; y<w; y++)
        {
            Buffer[y] = VERTICAL_COMPUTE_SCALAR_SOBEL(ImageIn->pData[x*w +y+offset[0]*w],ImageIn->pData[x*w + y+offset[1]*w],ImageIn->pData[x*w + y+offset[2]*w]);//3 5 not good, loop on size of offset
            if(y==45&&x==48)
            {
                printf("vertical %d %d %d\n%d\n", ImageIn->pData[x*w +y+offset[0]*w],ImageIn->pData[x*w + y+offset[1]*w],ImageIn->pData[x*w + y+offset[2]*w], Buffer[y]);
            }
        }
        //two more loop for kernel size 5
        BORDER_OFFSET(offset, left_top, w, bordertype);
        ImageOut->pData[x*w] = HORIZONTAL_COMPUTE_SCALAR_SOBEL(Buffer[offset[0]],Buffer[offset[1]],Buffer[offset[2]]);
        BORDER_OFFSET(offset, middle, w, bordertype);
        for(int y =1; y<w-1; y++)
        {
            ImageOut->pData[x*w+y] = HORIZONTAL_COMPUTE_SCALAR_SOBEL(Buffer[y+offset[0]],Buffer[y+offset[1]],Buffer[y+offset[2]]);//3 5 not good, loop on size of offset
            if(x==48&&y==45)
            {
                printf("Hor %d %d %d\n%d\n", Buffer[y+offset[0]],Buffer[y+offset[1]],Buffer[y+offset[2]], ImageOut->pData[x*w+y]);
            }
        }
        BORDER_OFFSET(offset, right_bot, w, bordertype);
        ImageOut->pData[x*w+w-1] = HORIZONTAL_COMPUTE_SCALAR_SOBEL(Buffer[w-1+offset[0]],Buffer[w-1+offset[1]],Buffer[w-1+offset[2]]);
    }

    /*      bottom part     */
    int x = h-1;
    BORDER_OFFSET(offset, right_bot, h, bordertype);
    for(int y =0; y<w; y++)
    {
        Buffer[y] = VERTICAL_COMPUTE_SCALAR_SOBEL(ImageIn->pData[x*w +y+offset[0]*w],ImageIn->pData[x*w + y+offset[1]*w],ImageIn->pData[x*w + y+offset[2]*w]);//3 5 not good, loop on size of offset
    }
    //two more loop for kernel size 5
    BORDER_OFFSET(offset, left_top, w, bordertype);
    ImageOut->pData[x*w] = HORIZONTAL_COMPUTE_SCALAR_SOBEL(Buffer[offset[0]],Buffer[offset[1]],Buffer[offset[2]]);
    BORDER_OFFSET(offset, middle, w, bordertype);
    for(int y =1; y<w-1; y++)
    {
        ImageOut->pData[x*w+y] = HORIZONTAL_COMPUTE_SCALAR_SOBEL(Buffer[y+offset[0]],Buffer[y+offset[1]],Buffer[y+offset[2]]);//3 5 not good, loop on size of offset
    }
    BORDER_OFFSET(offset, right_bot, w, bordertype);
    ImageOut->pData[x*w+w-1] = HORIZONTAL_COMPUTE_SCALAR_SOBEL(Buffer[w-1+offset[0]],Buffer[w-1+offset[1]],Buffer[w-1+offset[2]]);
    x= 48;
    int y=45;
    printf("OUTFINAL %d %d, %d", x, y, ImageOut->pData[x*w+y]);
}
#endif
