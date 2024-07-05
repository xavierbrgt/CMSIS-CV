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

void border_offset_replicate(int* offsetList, int border)
{
    switch (border)
    {
    case MIDDLE:
        offsetList[0] = -1;
        offsetList[1] = 0;
        offsetList[2] = 1;
        break;
    case RIGHT_BOT:
        offsetList[0] = -1;
        offsetList[1] = 0;
        offsetList[2] = 0;
        break;
    default:
        offsetList[0] = 0;
        offsetList[1] = 0;
        offsetList[2] = 1;
        break;
    }
}

void border_offset_wrap(int* offsetList, int border, int dim)
{
    switch (border)
    {

    case MIDDLE:
        offsetList[0] = -1;
        offsetList[1] = 0;
        offsetList[2] = 1;
        break;
    case RIGHT_BOT:
        offsetList[0] = -1;
        offsetList[1] = 0;
        offsetList[2] = -dim+1;
        break;
    default:
        offsetList[0] = dim-1;
        offsetList[1] = 0;
        offsetList[2] = 1;
    break;
    }
}

void border_offset_reflect(int* offsetList, int border)
{
    switch (border)
    {
    case MIDDLE:
        offsetList[0] = -1;
        offsetList[1] = 0;
        offsetList[2] = 1;
        break;
    case RIGHT_BOT:
        offsetList[0] = -1;
        offsetList[1] = 0;
        offsetList[2] = -1;
        break;
    default:
        offsetList[0] = 1;
        offsetList[1] = 0;
        offsetList[2] = 1;
    break;
    }
}
void border_offset(int* offsetList, int border, int dim, int border_type)
{
    switch (border_type)
    {
    case BORDER_REPLICATE:
        (void)dim;
        BORDER_OFFSET_REPLICATE(offsetList, border);
        break;
    case BORDER_WRAP:
        BORDER_OFFSET_WRAP(offsetList, border, dim);
        break;
    case BORDER_REFLECT:
        (void)dim;
        BORDER_OFFSET_REFLECT(offsetList, border);
        break;
    default:
        break;
    }
}
