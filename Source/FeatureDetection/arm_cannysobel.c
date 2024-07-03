/* ----------------------------------------------------------------------
 * Project:      CMSIS CV Library
 * Title:        cannysobel.c
 * Description:  Last steps of the canny edge detector (after the gaussian filter)
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
#include "cv/feature_detection.h"
#include "dsp/basic_math_functions.h"
#include "dsp/fast_math_functions.h"
#include <stdio.h>

#define Q15_ONE 0x7FFF
#define Q8_ONE 0xFF

#define NUM_LINE_BUFFER 3
#define DEG_TO_RAD_Q2_13(angle) (q15_t)roundf(angle* PI/180.0f * powf(2.0f, 13))

#define DECISION(x0,x1,x2,x3,x4,x5,x6,x7,y0, threshold, w, datain, dataout, idx, mag, col, row) if( mag <= datain[((col-x0)%NUM_LINE_BUFFER)*w+row-y0] ||\
																	    mag <= datain[((col-x1)%NUM_LINE_BUFFER)*w+row+y0])\
					{\
						dataout[idx] = 0;\
						continue;\
					}\
					else\
					{\
						if(mag < threshold)\
						{\
							if(datain[((col-(x2))%NUM_LINE_BUFFER) * (w)+row-1]>=(int)(threshold) ||\
							   datain[((col-(x3))%NUM_LINE_BUFFER) * (w)+row+(y0-1)]>=(int)(threshold) ||\
							   datain[((col-(x4))%NUM_LINE_BUFFER) * (w)+row+1]>=(int)(threshold) ||\
							   datain[((col-(x5))%NUM_LINE_BUFFER) * (w)+row-1]>=(int)(threshold) ||\
							   datain[((col-(x6))%NUM_LINE_BUFFER) * (w)+row-(y0-1)]>=(int)(threshold) ||\
							   datain[((col-(x7))%NUM_LINE_BUFFER) * (w)+row+1]>=(int)(threshold))  \
							{\
								dataout[idx] = Q8_ONE;\
								continue;\
							}\
							else\
							{\
								dataout[idx] = 0;\
								continue;\
							}\
						}\
						else\
						{\
							dataout[idx] = Q8_ONE;\
							continue;\
						}\
					}\
					continue;

#define DECISION_LAST(x0,x2,x3,x4,x5,x6,x7,y0, threshold, w, datain, dataout, idx, mag, col, row) if( mag <= datain[((col-x0)%NUM_LINE_BUFFER)*w+row-y0])\
					{\
						dataout[idx] = 0;\
						continue;\
					}\
					else\
					{\
						if(mag < threshold)\
						{\
							if(datain[((col-(x2))%NUM_LINE_BUFFER) * (w)+row-1]>=(int)(threshold) ||\
							   datain[((col-(x3))%NUM_LINE_BUFFER) * (w)+row]>=(int)(threshold) ||\
							   datain[((col-(x4))%NUM_LINE_BUFFER) * (w)+row+1]>=(int)(threshold) ||\
							   datain[((col-(x5))%NUM_LINE_BUFFER) * (w)+row-1]>=(int)(threshold) ||\
							   datain[((col-(x6))%NUM_LINE_BUFFER) * (w)+row]>=(int)(threshold) ||\
							   datain[((col-(x7))%NUM_LINE_BUFFER) * (w)+row+1]>=(int)(threshold))  \
							{\
								dataout[idx] = Q8_ONE;\
								continue;\
							}\
							else\
							{\
								dataout[idx] = 0;\
								continue;\
							}\
						}\
						else\
						{\
							dataout[idx] = Q8_ONE;\
							continue;\
						}\
					}\
					continue;
#define U8_TO_Q2_13(a) ((a)<<5)
#define Q5_10_TO_Q15(a) ((a)<<5)
#define VERTICAL_CASE(threshold, w, datain, dataout, idx, mag, col, row) DECISION(2,2,3,3,3,1,1,1,1, threshold, w, datain, dataout, idx, mag, col, row)
#define DIAGONAL_45_CASE(threshold, w, datain, dataout, idx, mag, col, row) DECISION(1,3,3,3,2,2,1,1,1, threshold, w, datain, dataout, idx, mag, col, row)
#define HORIZONTAL_CASE(threshold, w, datain, dataout, idx, mag, col, row) DECISION(3,1,3,3,2,2,1,1,0, threshold, w, datain, dataout, idx, mag, col, row)
#define DIAGONAL_135_CASE(threshold, w, datain, dataout, idx, mag, col, row) DECISION(3,1,3,3,2,2,1,1,1, threshold, w, datain, dataout, idx, mag, col, row)

#define VERTICAL_CASE_BOT_BORDER(threshold, w, datain, dataout, idx, mag, col, row) DECISION(2,2,1,1,1,1,1,1,1, threshold, w, datain, dataout, idx, mag, col, row)
#define DIAGONAL_45_CASE_BOT_BORDER(threshold, w, datain, dataout, idx, mag, col, row) DECISION_LAST(1,1,1,2,2,1,1,1, threshold, w, datain, dataout, idx, mag, col, row)
#define HORIZONTAL_CASE_BOT_BORDER(threshold, w, datain, dataout, idx, mag, col, row) DECISION(1,1,1,1,2,2,1,1,0, threshold, w, datain, dataout, idx, mag, col, row)
#define DIAGONAL_135_CASE_BOT_BORDER(threshold, w, datain, dataout, idx, mag, col, row) DECISION_LAST(1,1,1,2,2,1,1,-1, threshold, w, datain, dataout, idx, mag, col, row)

#define THRESHOLDING_HYSTERESIS(angle, high_threshold, w, data_mag, data_out, idx, mag, x, y) if((angle ) < (DEG_TO_RAD_Q2_13(22)))\
				{\
					VERTICAL_CASE(high_threshold, w, data_mag, data_out, idx, mag, x, y)\
				}\
				else if((angle ) < (DEG_TO_RAD_Q2_13(67)))\
				{\
					DIAGONAL_45_CASE(high_threshold, w, data_mag, data_out, idx, mag, x, y)\
				}\
				else if((angle ) < (DEG_TO_RAD_Q2_13(112)))\
				{\
					HORIZONTAL_CASE(high_threshold, w, data_mag, data_out, idx, mag, x, y)\
				}\
				else if((angle ) < (DEG_TO_RAD_Q2_13(160)))\
				{\
					DIAGONAL_135_CASE(high_threshold, w, data_mag, data_out, idx, mag, x, y)\
				}\
				else\
				{\
					VERTICAL_CASE(high_threshold, w, data_mag, data_out, idx, mag, x, y)\
				}

#define THRESHOLDING_HYSTERESIS_BOTTOM_BORDER(angle, high_threshold, w, data_mag, data_out, idx, mag, x, y)if((angle ) < (DEG_TO_RAD_Q2_13(22)))\
			{\
				VERTICAL_CASE_BOT_BORDER( high_threshold, w, data_mag, data_out, idx, mag, x, y)\
			}\
			else if((angle ) < (DEG_TO_RAD_Q2_13(67)))\
			{\
				DIAGONAL_45_CASE_BOT_BORDER( high_threshold, w, data_mag, data_out, idx, mag, x, y)\
			}\
			else if((angle ) < (DEG_TO_RAD_Q2_13(112)))\
			{\
				HORIZONTAL_CASE_BOT_BORDER( high_threshold, w, data_mag, data_out, idx, mag, x, y)\
			}\
			else if((angle ) < (DEG_TO_RAD_Q2_13(160)))\
			{\
				DIAGONAL_135_CASE_BOT_BORDER( high_threshold, w, data_mag, data_out, idx, mag, x, y)\
			}\
			else\
			{\
				VERTICAL_CASE_BOT_BORDER( high_threshold, w, data_mag, data_out, idx, mag, x, y)\
			}

uint16_t arm_cv_get_scratch_size_canny_sobel(int width)
{
	return(15*width*sizeof(q15_t));
}

#if defined(ARM_MATH_MVEI) && !defined(ARM_MATH_AUTOVECTORIZE)
static void arm_cv_gradient_intermediate(int idx, int idxout, int w, uint8_t* data_in, arm_cv_gradient_q15_t* data_grad2)
{
	q15x8x2_t vect_2x2;
	q15x8x2_t vect_1x2;
	q15x8x2_t vect_3x2;
	q15x8x4_t vect_1x4;
	q15x8x2_t vect_lowx2;
	q15x8x2_t vect_highx2;
	uint8x16_t vect_1 = vld1q(&data_in[idx-1]);
	vect_1x2.val[0] = vshllbq(vect_1,5);
	uint8x16_t vect_3 = vld1q(&data_in[idx+1]);
	vect_3x2.val[0] = vshllbq(vect_3,5);
	vect_1x2.val[0] = vaddq(vect_1x2.val[0], vect_3x2.val[0]);
	vect_1x2.val[1] = vshlltq(vect_1,5);
	uint8x16_t vect_2 = vld1q(&data_in[idx]);
	vect_3x2.val[1] = vshlltq(vect_3,5);
	vect_1x2.val[1] = vaddq(vect_1x2.val[1], vect_3x2.val[1]);
	vect_2x2.val[0] = vshllbq(vect_2,6);
	vect_1x2.val[0] = vaddq(vect_2x2.val[0], vect_1x2.val[0]);
	vect_2x2.val[1] = vshlltq(vect_2,6);
	vect_1x2.val[1] = vaddq(vect_2x2.val[1], vect_1x2.val[1]);
	uint8x16_t vecth = vld1q(&data_in[idx-w]);
	vect_highx2.val[0] = vshllbq(vecth,5);
	vect_highx2.val[1] = vshlltq(vecth,5);
	uint8x16_t vectl = vld1q(&data_in[idx+w]);
	vect_lowx2.val[0] = vshllbq(vectl,5);
	vect_lowx2.val[1] = vshlltq(vectl,5);
	vect_2x2.val[0] = vaddq(vect_lowx2.val[0], vect_2x2.val[0]);
	vect_2x2.val[0] = vaddq(vect_highx2.val[0], vect_2x2.val[0]);
	vect_2x2.val[1] = vaddq(vect_lowx2.val[1], vect_2x2.val[1]);
	vect_2x2.val[1] = vaddq(vect_highx2.val[1], vect_2x2.val[1]);
	vect_1x4.val[0] = vect_1x2.val[0];
	vect_1x4.val[1] = vect_2x2.val[0];
	vect_1x4.val[2] = vect_1x2.val[1];
	vect_1x4.val[3] = vect_2x2.val[1];
	vst4q((&data_grad2[idxout].x), vect_1x4);
}

static void arm_cv_gradient_magnitude(int x, int y, int w, int idxout, arm_cv_gradient_q15_t* data_grad1, arm_cv_gradient_q15_t* data_grad2, q15_t* data_mag)
{
	for(int p = 0; p<2; p++)
	{
		y += p*8;
		idxout += p*8;
		q15x8x2_t vect_buff_di = vld2q_s16(&data_grad2[idxout].x);

		q15x8x2_t vec_x_y_1 = vld2q_s16(&data_grad2[((x-2)%NUM_LINE_BUFFER)*w + y].x);

		q15x8x2_t vec_y_1 = vld2q_s16(&data_grad2[((x-1)%NUM_LINE_BUFFER)*w + y-1].x);

		q15x8x2_t vect_grad_1;
		q15x8_t vectgradx = vsubq_s16(vec_x_y_1.val[0], vect_buff_di.val[0]);
		q15x8x2_t vec_y_2 = vld2q_s16(&data_grad2[((x-1)%NUM_LINE_BUFFER)*w + y+1].x);
		int numVect = 8;
		int16x8_t vect_res_1;
		q31x4_t vect_gradx_1 = vmullbq_int_s16(vectgradx, vectgradx);
		q15x8_t vectgrady = vsubq_s16(vec_y_1.val[1], vec_y_2.val[1]);
		q31x4_t vect_grady_1 = vmullbq_int_s16(vectgrady, vectgrady);

		vect_gradx_1 = vaddq_s32(vect_gradx_1, vect_grady_1);
		q31x4_t vect_gradx_2 = vmulltq_int_s16(vectgradx, vectgradx);
		vect_gradx_1 = vshrq(vect_gradx_1, 1);
		q31x4_t vect_grady_2 = vmulltq_int_s16(vectgrady, vectgrady);

		vect_gradx_2 = vaddq_s32(vect_gradx_2, vect_grady_2);
		vect_gradx_2 = vshrq(vect_gradx_2, 1);
		
		vect_grad_1.val[0] = vectgradx;
		vect_grad_1.val[1] = vectgrady;
		vst2q(&data_grad1[(x-1)%NUM_LINE_BUFFER*w+y].x, vect_grad_1);
		for(int j =0; j < numVect; j+=2)
		{
			if(vectgradx[j]==0&&vectgrady[j]==0)
			{
				vect_res_1[j] =  0;
				continue;
			}
			q15_t out;
			q31_t root ;
			arm_sqrt_q31(vect_gradx_1[j>>1], &root);
			out = root>>15;
			vect_res_1[j] = out;
		}
		for(int j =1; j < numVect; j+=2)
		{
			if(vectgradx[j]==0&&vectgrady[j]==0)
			{
				vect_res_1[j] =  0;
				continue;
			}
			q15_t out;
			q31_t root ;
			arm_sqrt_q31(vect_gradx_2[j>>1], &root);
			out = root>>15;
			vect_res_1[j] = out;
		}
		vst1q_s16((int16_t*)&data_mag[((x-1)%NUM_LINE_BUFFER)*w + y], vect_res_1);
		y -= p*8;
	}
}

static void arm_cv_gradient_magnitude_tail(int x, int x3, int w, arm_cv_gradient_q15_t* data_grad2,const arm_cv_image_gray8_t* imageIn, q15_t* data_mag, uint8_t* data_out, arm_cv_gradient_q15_t* data_grad1)
{
	uint8_t* data_in = imageIn->pData;
	for(int y= ((w-1)&0xFFE0); y < w; y++)
	{
		if((y==0||y == w-1)&&x!=0&& x != imageIn->height-1)
		{
			data_grad2[x3*w +y].y = Q5_10_TO_Q15(data_in[(x-1)*w+y] + (data_in[x*w+y]<<1) + data_in[(x+1)*w+y]);
			data_mag[((((x-1)%NUM_LINE_BUFFER*w)) + y)] =0;
			data_out[x*w+y] = 0;
			continue;
		}
		if( x==0 || y==0||y == w-1)
		{
			data_mag[((((x-1)%NUM_LINE_BUFFER*w)) + y)] =0;
			data_out[x*w+y] = 0;
			continue;
		}
		data_grad2[x3*w +y].y = Q5_10_TO_Q15(data_in[(x-1)*w+y] + (data_in[x*w+y]<<1) + data_in[(x+1)*w+y]);
		data_grad2[x3*w +y].x = Q5_10_TO_Q15(data_in[x*w+(y-1)] + (data_in[x*w+(y)]<<1) + data_in[x*w+(y+1)]);
		if(x==1)
		{
			continue;
		}
		q63_t gradx = data_grad2[((x-2)%NUM_LINE_BUFFER)*w +y].x - data_grad2[(x3)*w +y].x;
		q63_t grady = data_grad2[(x-1)%NUM_LINE_BUFFER*w +(y-1)].y - data_grad2[(x-1)%NUM_LINE_BUFFER*w +(y+1)].y;
		data_grad1[(x-1)%NUM_LINE_BUFFER*w+y].x = gradx;
		data_grad1[(x-1)%NUM_LINE_BUFFER*w+y].y = grady;
		if(gradx==0&&grady==0)
		{
			data_mag[((((x-1)%NUM_LINE_BUFFER*w)) + y)] = 0;
			data_out[(x-1)*w+y]= 0 ;
			continue;
		}
		q15_t vect[2] = { (q15_t)gradx, (q15_t)grady};
		q31_t in[2] = {((q31_t)vect[0]), ((q31_t)vect[1])};
		q31_t out2[2];
		q31_t out3;
		q31_t root;
		out2[0]=(in[0]*in[0]);
		out2[1]=(in[1]*in[1]);
		out3 = (out2[0]+out2[1])>>1;
		arm_sqrt_q31(out3, &root);
		data_mag[((x-1)%NUM_LINE_BUFFER*w) + y] = root>>15;
	}
}
#else
static void arm_cv_compute_buffer_line_canny_sobel(const arm_cv_image_gray8_t* imageIn, 
                                     arm_cv_image_gray8_t* imageOut, 
                                     arm_cv_gradient_q15_t* grad1, 
                                     arm_cv_gradient_q15_t* grad2,
									 q15_t* mag, 
									 int lineIdx)
{
	int w = imageIn->width;
	int xm = lineIdx%NUM_LINE_BUFFER;
	int idx;
	uint8_t* data_in = imageIn->pData;
	uint8_t* data_out = imageOut->pData;
	grad2[xm*w].y = (data_in[(lineIdx-1)*w] + (data_in[lineIdx*w]<<1) + data_in[(lineIdx+1)*w])<<5;
	data_out[(lineIdx-1)*w] = 0;
	for(int y = 1; y < w-1; y ++)
	{
		idx = (lineIdx-1)*w+y;

		grad2[xm*w +y].y = Q5_10_TO_Q15(data_in[(lineIdx-1)*w+y] + (data_in[lineIdx*w+y]<<1) + data_in[(lineIdx+1)*w+y]);
		grad2[xm*w +y].x = Q5_10_TO_Q15(data_in[lineIdx*w+(y-1)] + (data_in[lineIdx*w+(y)]<<1) + data_in[lineIdx*w+(y+1)]);

		q15_t gradx = grad2[((lineIdx-2)%NUM_LINE_BUFFER)*w +y].x - grad2[(xm)*w +y].x;
		q15_t grady = grad2[((lineIdx-1)%NUM_LINE_BUFFER)*w +(y-1)].y - grad2[((lineIdx-1)%NUM_LINE_BUFFER)*w +(y+1)].y;
		if(gradx==0&&grady==0)
		{
			data_out[idx] = 0;
			int idxp = (lineIdx-1)%NUM_LINE_BUFFER * w +y;
			mag[idxp] = 0;
			grad1[idxp].y = grady;
			grad1[idxp].x = gradx;
			continue;
		}
		//Computation of the magnitude
		q15_t vect[2] = { (q15_t)gradx, (q15_t)grady};
		q15_t out;

		q31_t in[2] = {((q31_t)vect[0]), ((q31_t)vect[1])};
		q31_t out2[2];
		q31_t out3;
		q31_t root;
		//multiplication of two q15 give a q31 in output
		out2[0]=(in[0]*in[0]);
		out2[1]=(in[1]*in[1]);
		//addition of two q1.30 give a q2.29 shift by one a q1.30
		out3 = (out2[0]+out2[1])>>1;
		//root q31 give in output a q31 shit by 15, back to a q15 because of the previous shift by one 
		arm_sqrt_q31(out3, &root);
		out = root>>15;
		grad1[(lineIdx-1)%NUM_LINE_BUFFER * w + y].y = grady;
		grad1[(lineIdx-1)%NUM_LINE_BUFFER * w + y].x = gradx;
		mag[(lineIdx-1)%NUM_LINE_BUFFER * w + y] = out;
	}
	grad2[xm*w+w-1].y = Q5_10_TO_Q15(data_in[(lineIdx-1)*w+w-1] + (data_in[lineIdx*w+w-1]<<1) + data_in[(lineIdx+1)*w+w-1]);
	data_out[(lineIdx-1)*w+w-1] = 0;
}
#endif
//function performing canny edge on an image where a gaussian filter has been applied
//this function uses three buffers, one for storing intermediate values for computing the gradient, one for storing the gradient and one for storing the magnitude conputed with the gradient
//exept the buffer for the magnitude, the buffer have two component
//the purpose of this function is to avoid repetition of compute by storing the intermediate part of the compute and by fusing the end of canny edge and sobel
//to avoid repetition of condition for the end of the canny edge
/**
  @ingroup featureDetection
 */

/**
 * @brief      Canny edge with sobel integrated
 *
 * @param[in]     imageIn         The input image
 * @param[out]    imageOut        The output image
 * @param[in,out] scratch   Temporary buffer 
 * @param[in]     lowThreshold   The low threshold
 * @param[in]     highThreshold  The high threshold
 * 
 * @par  Temporary buffer sizing:
 * 
 * Size of temporary buffers, given by uint16_t arm_cv_get_scratch_size_canny_sobel(int width):
 *   - scratch\n
 *     15*w*sizeof(q15_t) where w is the input image width
 */
#if defined(ARM_MATH_MVEI) && !defined(ARM_MATH_AUTOVECTORIZE)

void arm_canny_edge_sobel_fixp(const arm_cv_image_gray8_t* imageIn, 
                                     arm_cv_image_gray8_t* imageOut, 
									 q15_t* scratch,
                                     uint8_t lowThreshold,
                                     uint8_t highThreshold)
{
	q15x8_t vect_mag;
    int x = 0;
	int w = imageIn->width;
	q31_t low_threshold = U8_TO_Q2_13(lowThreshold);
	q31_t high_threshold = U8_TO_Q2_13(highThreshold);
	arm_cv_gradient_q15_t* data_grad2 = (arm_cv_gradient_q15_t*)&scratch[NUM_LINE_BUFFER*w];
	q15_t* data_mag = scratch;
	arm_cv_gradient_q15_t* data_grad1 = (arm_cv_gradient_q15_t*)&scratch[3*NUM_LINE_BUFFER*w];
	uint8_t* data_in = imageIn->pData;
	uint8_t* data_out = imageOut->pData;
	for(int t = 0; t<w*3; t++)
	{
		data_grad1[t].x=0;
		data_mag[t]=0;
		data_grad2[t].x=0;
	}
	//First line of buffer
	data_out[x*w] = 0;
	for(int y = 1; y<((imageIn -> width)>>4)+1; y++)
	{
		int idx = ((y-1)<<4) + 1;

		q15x8x2_t vect_2x2;
		q15x8x2_t vect_1x2;
		q15x8x2_t vect_3x2;
		q15x8x4_t vect_1x4;

		uint8x16_t vect_1 = vld1q(&data_in[idx-1]);
		vect_1x2.val[0] = vshllbq(vect_1,5);
		
		uint8x16_t vect_3 = vld1q(&data_in[idx+1]);
		vect_3x2.val[0] = vshllbq(vect_3,5);
		vect_1x2.val[0] = vaddq(vect_1x2.val[0], vect_3x2.val[0]);
		vect_1x2.val[1] = vshlltq(vect_1,5);
		
		uint8x16_t vect_2 = vld1q(&data_in[idx]);
		vect_3x2.val[1] = vshlltq(vect_3,5);
		vect_1x2.val[1] = vaddq(vect_1x2.val[1], vect_3x2.val[1]);
		vect_2x2.val[0] = vshllbq(vect_2,6);
		vect_2x2.val[0] = vaddq(vect_2x2.val[0], vect_1x2.val[0]);
		vect_2x2.val[1] = vshlltq(vect_2,6);
		vect_2x2.val[1] = vaddq(vect_2x2.val[1], vect_1x2.val[1]);

		q15x8_t vect_void = vdupq_n_s16(0);
		vect_1x4.val[0] = vect_2x2.val[0];
		vect_1x4.val[1] = vect_void;
		vect_1x4.val[2] = vect_2x2.val[1];
		vect_1x4.val[3] = vect_void;
		vst4q((&data_grad2[idx].x), vect_1x4);
		vst1q(&data_out[idx], (uint8x16_t)vect_void);
		vst1q(&data_out[idx+16], (uint8x16_t)vect_void);
	}
	//tail
	int numtail = (w-1)-((w-1)&(0xffe0));
	if(numtail>0)
	{
		for(int j=0; j<numtail+1; j++)
		{
			int y = ((imageIn -> width)-numtail-1);
			data_grad2[y+j].x = Q5_10_TO_Q15(data_in[(y-1)+j] + (data_in[(y)+j]<<1) + data_in[(y+1)+j]);
			data_grad2[y+j].y = 0;
			data_out[y+j]=0;
		}
	}
	data_out[x*w +w-1] = 0;
	//Second line for buffer
	x =1;
	data_grad2[x*w].x =0;
	data_out[x*w] = 0;
	data_grad2[x*w].y = (data_in[(x-1)*w] + (data_in[x*w]<<1) + data_in[(x+1)*w])<<5;
	for(int y =1; y<((w)>>4)+1; y++)
	{
		int idx = w +((y-1)<<4)+1;
		arm_cv_gradient_intermediate(idx, idx, w, data_in, data_grad2);
	}
	//Tail
	if(numtail>0)
	{
		for(int j=0; j<numtail+1; j++)
		{
			int x=1;
			int y = ((w)-numtail-1);
			data_grad2[x*w +y+j].y = Q5_10_TO_Q15(data_in[(x-1)*w+y+j] + (data_in[x*w+y+j]<<1) + data_in[(x+1)*w+y+j]);
			data_grad2[x*w +y+j].x = Q5_10_TO_Q15(data_in[x*w+(y-1)+j] + (data_in[x*w+(y)+j]<<1) + data_in[x*w+(y+1)+j]);
		}
	}
	data_grad2[w+w-1].x =0;
	data_grad2[w+w-1].y = Q5_10_TO_Q15(data_in[(x-1)*w+w-1] + (data_in[x*w+w-1]<<1) + data_in[(x+1)*w+w-1]);
	for(int x =2; x< 3; x++)
	{
		int x3 = x%NUM_LINE_BUFFER;
		data_grad2[x3*w].y = Q5_10_TO_Q15(data_in[(x-1)*w] + (data_in[x*w]<<1) + data_in[(x+1)*w]);
		data_out[(x-2)*w] =0;
		for(int y =1; y< w-15;y+=16)
		{
			int idx = x*w + y;
			int idx3 = x3*w + y;
			arm_cv_gradient_intermediate(idx, idx3, w, data_in, data_grad2);
			arm_cv_gradient_magnitude(x, y, w, idx3, data_grad1, data_grad2, data_mag);
		}
		//Tail
		arm_cv_gradient_magnitude_tail(x, x3, w, data_grad2, imageIn, data_mag, data_out, data_grad1);
	}  
	//Core loop
	for(int x = 3; x< imageIn->height; x++)
	{
		int x3 = x%NUM_LINE_BUFFER;
		data_grad2[x3*w].y = Q5_10_TO_Q15(data_in[(x-1)*w] + (data_in[x*w]<<1) + data_in[(x+1)*w]);
		data_out[(x-2)*w] = 0;
		for(int y =1; y< w-16;y+=16)
		{
			int idx = x*w + y;
			int idx3 = x3*w + y;
			arm_cv_gradient_intermediate(idx, idx3, w, data_in, data_grad2);
			arm_cv_gradient_magnitude(x, y, w, idx3, data_grad1, data_grad2, data_mag);
		}
		arm_cv_gradient_magnitude_tail(x, x3, w, data_grad2, imageIn, data_mag, data_out, data_grad1);
        for(int y= 1; y < ((w-2)&0xFFE0); y+=8)
        {
			int idx = (x-2)*w +y;
            vect_mag = vld1q(&data_mag[((x-2)%NUM_LINE_BUFFER) * (w)+y]);
			uint8x16_t vect_out;
            int16x8x2_t vect_grad = vld2q_s16(&data_grad1[((x-2)%NUM_LINE_BUFFER)*w+y].x);
			for(int j =0; j< 8; j+=1)
            {
				int mag = vect_mag[j];
				if(mag < low_threshold)
				{
					vect_out[j] = 0;
					continue;
				}
				else
				{
					q15_t angle =0;
					arm_atan2_q15(vect_grad.val[0][j], vect_grad.val[1][j], &angle);

					arm_abs_q15( &angle, &angle, 1);
					THRESHOLDING_HYSTERESIS(angle, high_threshold, w, data_mag, vect_out, j, mag, x, y+j)
				}
            }
			vst1q((uint8_t*)&data_out[idx], vect_out);
        }
		//tail
		for(int y= ((w-1)&0xFFE0); y < w; y++)
        {
			int idx = (x-2)*w +y;
            int mag = data_mag[((x-2)%NUM_LINE_BUFFER) * (w)+y];
			if(mag != 0)
			{
				if(mag < low_threshold)
				{
					data_out[idx] = 0;
					continue;
				}
				else
				{
					q15_t angle;
					arm_cv_gradient_q15_t grad = data_grad1[((x-2)%NUM_LINE_BUFFER)*w+y];
					arm_atan2_q15(grad.x, grad.y, &angle);
					arm_abs_q15( &angle, &angle, 1);
					THRESHOLDING_HYSTERESIS(angle, high_threshold, w, data_mag, data_out, idx, mag, x, y)
				}
			}
			else
			{
				data_out[idx] = 0;
			}
		}
	}
	//Last lines
	for(int x = imageIn->height; x <imageIn->height+1 ;x++)
	{
		for(int y= 0; y < ((w-2)&0xFFE0); y+=8)
		{
			data_out[(x-1)*w] = 0;
			int idx = (x-2)*w +y;
			vect_mag = vld1q(&data_mag[((x-2)%NUM_LINE_BUFFER) * (w)+y]);
			uint8x16_t vect_out;
			if(y+16<w-1)
			{
				uint8x16_t vect_void = vdupq_n_u8(0);
				vst1q(&data_out[(x-1)*w+y], vect_void);
			}
			int16x8x2_t vect_grad = vld2q_s16(&data_grad1[((x-2)%NUM_LINE_BUFFER)*w+y].x);
			for(int j =0; j< 8; j++)
			{
				int mag = vect_mag[j];
				if(mag != 0)
				{
					if(mag < low_threshold)
					{
						vect_out[j] = 0;
						
						continue;
					}
					else
					{
						q15_t angle;
						arm_atan2_q15(vect_grad.val[0][j],  vect_grad.val[1][j], &angle);
						arm_abs_q15( &angle, &angle, 1);
						THRESHOLDING_HYSTERESIS_BOTTOM_BORDER(angle, high_threshold, w, data_mag, vect_out, j, mag, x, y+j)
					}
				}
				else
				{
					vect_out[j] = 0;
				}
			}
			vst1q((uint8_t*)&data_out[idx], vect_out);
        }
		//tail
		for(int y= ((w-1)&0xFFE0); y < w; y++)
		{
			int idx = (x-2)*w +y;
			int mag = data_mag[((x-2)%NUM_LINE_BUFFER) * (w)+y];
			data_out[idx+w]=0;
			q15_t angle;
			arm_cv_gradient_q15_t grad = data_grad1[((x-2)%NUM_LINE_BUFFER)*w+y];
			arm_atan2_q15(grad.x, grad.y, &angle);
			arm_abs_q15( &angle, &angle, 1);
			if(mag < low_threshold)
			{
				data_out[idx] = 0;
				continue;
			}
			else
			{
				q15_t angle;
				arm_atan2_q15(data_grad1[((x-2)%NUM_LINE_BUFFER)*w+y].x, data_grad1[((x-2)%NUM_LINE_BUFFER)*w+y].y, &angle);
				if(data_grad1[((x-2)%NUM_LINE_BUFFER)*w+y].x ==0&& data_grad1[((x-2)%NUM_LINE_BUFFER)*w+y].y==0)
				{
					angle = 0;
				}
				arm_abs_q15( &angle, &angle, 1);
				THRESHOLDING_HYSTERESIS_BOTTOM_BORDER(angle, high_threshold, w, data_mag, data_out, idx, mag, x, y)
			}
		}
	}
}   
#else

void arm_canny_edge_sobel_fixp(const arm_cv_image_gray8_t* imageIn, 
                                     arm_cv_image_gray8_t* imageOut, 
									 q15_t* scratch,
                                     uint8_t lowThreshold,
                                     uint8_t highThreshold)
{
	int16_t w = imageIn->width;
	q31_t low_threshold = U8_TO_Q2_13(lowThreshold);
	q31_t high_threshold = U8_TO_Q2_13(highThreshold);

	arm_cv_gradient_q15_t* data_grad2 = (arm_cv_gradient_q15_t*)&scratch[NUM_LINE_BUFFER*w];
	q15_t* data_mag = scratch;
	arm_cv_gradient_q15_t* data_grad1 = (arm_cv_gradient_q15_t*)&scratch[3*NUM_LINE_BUFFER*w];
	uint8_t* data_in = imageIn->pData;
	uint8_t* data_out = imageOut->pData;
	int x = 0;
	for(int t = 0; t<w*NUM_LINE_BUFFER; t++)
	{
		data_grad1[t].x=0;
		data_mag[t]=0;
		data_grad2[t].x=0;
	}
	data_out[x*w] = 0;
	for(int y = 1; y< w- 1; y++)
	{
		//Apply [1,2,1] kernel for computation of a part of the sobel gradient
		data_grad2[x*w +y].x = Q5_10_TO_Q15(data_in[x*w+(y-1)] + (data_in[x*w+(y)]<<1) + data_in[x*w+(y+1)]);
		data_out[x*w +y] = 0;
	}
	data_out[x*w +w-1] = 0;
	x = 1;
	data_grad2[x*w].y = (data_in[(x-1)*w] + (data_in[x*w]<<1) + data_in[(x+1)*w]);
	data_out[x*w] = 0;
	for(int y = 1; y< w- 1; y++)
	{
		data_grad2[x*w +y].y = Q5_10_TO_Q15(data_in[(x-1)*w+y] + (data_in[x*w+y]<<1) + data_in[(x+1)*w+y]);
		data_grad2[x*w +y].x = Q5_10_TO_Q15(data_in[x*w+(y-1)] + (data_in[x*w+(y)]<<1) + data_in[x*w+(y+1)]);
	}
	data_grad2[x*w+w-1].y = Q5_10_TO_Q15(data_in[(x-1)*w+w-1] + (data_in[x*w+w-1]<<1) + data_in[(x+1)*w+w-1]);
	data_out[x*w +w-1] = 0;
	x=2;
	//first line
	arm_cv_compute_buffer_line_canny_sobel(imageIn, 
                                     imageOut, 
                                     data_grad1, 
                                     data_grad2,
									 data_mag, 
									 x);
	//core loop
	for( int x = 3; x < imageIn->height; x++)
    {
        arm_cv_compute_buffer_line_canny_sobel(imageIn, 
                                     imageOut, 
                                     data_grad1, 
                                     data_grad2,
									 data_mag, 
									 x);

		for( int y =1; y < w-1; y++)
		{
			int idx = (x-2)*w +y;
			int mag = data_mag[((x-2)%NUM_LINE_BUFFER) * (w)+y];
			if(mag < low_threshold)
			{
				data_out[idx] = 0;
				continue;
			}
			else
			{
				q15_t angle;
				arm_atan2_q15(data_grad1[((x-2)%NUM_LINE_BUFFER)*w+y].x, data_grad1[((x-2)%NUM_LINE_BUFFER)*w+y].y, &angle);
				arm_abs_q15( &angle, &angle, 1);
				THRESHOLDING_HYSTERESIS(angle, high_threshold, w, data_mag, data_out, idx, mag, x, y)
			}
		}
	}
	//last line
	x = imageIn->height;
	data_out[x*w] = 0;
	for( int y =1; y < w-1; y++)
	{
		int idx = (x-2)*w +y;
		int mag = data_mag[((x-2)%NUM_LINE_BUFFER) * (w)+y];
		data_out[idx+w] = 0;
		if(mag < low_threshold)
		{
			data_out[idx] = 0;
			continue;
		}
		else
		{
			q15_t angle;
			arm_atan2_q15(data_grad1[((x-2)%NUM_LINE_BUFFER)*w+y].x, data_grad1[((x-2)%NUM_LINE_BUFFER)*w+y].y, &angle);
			if(data_grad1[((x-2)%NUM_LINE_BUFFER)*w+y].x ==0&& data_grad1[((x-2)%NUM_LINE_BUFFER)*w+y].y==0)
			{
				angle = 0;
			}
			arm_abs_q15( &angle, &angle, 1);
			THRESHOLDING_HYSTERESIS_BOTTOM_BORDER(angle, high_threshold, w, data_mag, data_out, idx, mag, x, y)
		}
	}
	data_out[x*w +w-1] = 0;
}
#endif