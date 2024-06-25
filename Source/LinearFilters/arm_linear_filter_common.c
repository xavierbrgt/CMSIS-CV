

#include "arm_linear_filter_common.h"

void border_offset_replicate(int* offset_list, int border)
{
    switch (border)
    {
    case middle:
        offset_list[0] = -1;
        offset_list[1] = 0;
        offset_list[2] = 1;
        break;
    case right_bot:
        offset_list[0] = -1;
        offset_list[1] = 0;
        offset_list[2] = 0;
        break;
    default:
        offset_list[0] = 0;
        offset_list[1] = 0;
        offset_list[2] = 1;
        break;
    }
}

void border_offset_wrap(int* offset_list, int border, int dim)
{
    switch (border)
    {

    case middle:
        offset_list[0] = -1;
        offset_list[1] = 0;
        offset_list[2] = 1;
        break;
    case right_bot:
        offset_list[0] = -1;
        offset_list[1] = 0;
        offset_list[2] = -dim+1;
        break;
    default:
        offset_list[0] = dim-1;
        offset_list[1] = 0;
        offset_list[2] = 1;
    break;
    }
}

void border_offset_reflect(int* offset_list, int border)
{
    switch (border)
    {
    case middle:
        offset_list[0] = -1;
        offset_list[1] = 0;
        offset_list[2] = 1;
        break;
    case right_bot:
        offset_list[0] = -1;
        offset_list[1] = 0;
        offset_list[2] = -1;
        break;
    default:
        offset_list[0] = 1;
        offset_list[1] = 0;
        offset_list[2] = 1;
    break;
    }
}
void border_offset(int* offset_list, int border, int dim, int border_type)
{
    switch (border_type)
    {
    case Border_Replicate:
        (void)dim;
        BORDER_OFFSET_REPLICATE(offset_list, border);
        break;
    case Border_Wrap:
        BORDER_OFFSET_WRAP(offset_list, border, dim);
        break;
    case Border_Reflect:
        (void)dim;
        BORDER_OFFSET_REFLECT(offset_list, border);
        break;
    default:
        break;
    }
}
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