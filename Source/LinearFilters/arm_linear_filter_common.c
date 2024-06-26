/* ----------------------------------------------------------------------
 * Project:      CMSIS CV Library
 * Title:        arm_linear_filter_common.c
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
