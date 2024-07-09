/* ----------------------------------------------------------------------
 * Project:      CMSIS CV Library
 * Title:        arm_image_resize_common.h
 * Description:  Common declarations for CMSIS-CV image resize functions
 *
 * Target Processor: Cortex-M and Cortex-A cores
 * -------------------------------------------------------------------- */
/*
 * Copyright (C) 2024 Himax Technologies, Inc. or its affiliates. All rights reserved.
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
#ifndef ARM_CV_LINEAR_FILTER_COMMON_H
#define ARM_CV_LINEAR_FILTER_COMMON_H

#include "cv\linear_filters.h"
#ifdef __cplusplus
extern "C"
{
#endif

#define MIDDLE 0
#define RIGHT_BOT 1
#define LEFT_TOP 2

void border_offset_replicate(int *offset_list, int border);
void border_offset(int *offset_list, int border, int dim, int border_type);
void border_offset_wrap(int *offset_list, int border, int dim);
void border_offset_reflect(int *offset_list, int border);

//Give the offset for the border case Replicate
#define BORDER_OFFSET_REPLICATE(list, position)                                                                        \
    switch (position)                                                                                                  \
    {                                                                                                                  \
    case MIDDLE:                                                                                                       \
        list[0] = -1;                                                                                                  \
        list[1] = 0;                                                                                                   \
        list[2] = 1;                                                                                                   \
        break;                                                                                                         \
    case RIGHT_BOT:                                                                                                    \
        list[0] = -1;                                                                                                  \
        list[1] = 0;                                                                                                   \
        list[2] = 0;                                                                                                   \
        break;                                                                                                         \
    default:                                                                                                           \
        list[0] = 0;                                                                                                   \
        list[1] = 0;                                                                                                   \
        list[2] = 1;                                                                                                   \
        break;                                                                                                         \
    }

//Give the offset for the border case Wrap
#define BORDER_OFFSET_WRAP(list, position, dim)                                                                        \
    switch (position)                                                                                                  \
    {                                                                                                                  \
    case MIDDLE:                                                                                                       \
        list[0] = -1;                                                                                                  \
        list[1] = 0;                                                                                                   \
        list[2] = 1;                                                                                                   \
        break;                                                                                                         \
    case RIGHT_BOT:                                                                                                    \
        list[0] = -1;                                                                                                  \
        list[1] = 0;                                                                                                   \
        list[2] = -dim + 1;                                                                                            \
        break;                                                                                                         \
    default:                                                                                                           \
        list[0] = dim - 1;                                                                                             \
        list[1] = 0;                                                                                                   \
        list[2] = 1;                                                                                                   \
        break;                                                                                                         \
    }

//Give the offset for the border case Reflect
#define BORDER_OFFSET_REFLECT(list, position)                                                                          \
    switch (position)                                                                                                  \
    {                                                                                                                  \
    case MIDDLE:                                                                                                       \
        list[0] = -1;                                                                                                  \
        list[1] = 0;                                                                                                   \
        list[2] = 1;                                                                                                   \
        break;                                                                                                         \
    case RIGHT_BOT:                                                                                                    \
        list[0] = -1;                                                                                                  \
        list[1] = 0;                                                                                                   \
        list[2] = -1;                                                                                                  \
        break;                                                                                                         \
    default:                                                                                                           \
        list[0] = 1;                                                                                                   \
        list[1] = 0;                                                                                                   \
        list[2] = 1;                                                                                                   \
        break;                                                                                                         \
    }

//This macro give the offset depending on the border type specified
#define BORDER_OFFSET(list, position, dim, border_type)                                                                \
    switch (border_type)                                                                                               \
    {                                                                                                                  \
    case BORDER_REPLICATE:                                                                                             \
        (void)dim;                                                                                                     \
        BORDER_OFFSET_REPLICATE(list, position)                                                                        \
        break;                                                                                                         \
    case BORDER_WRAP:                                                                                                  \
        BORDER_OFFSET_WRAP(list, position, dim)                                                                        \
        break;                                                                                                         \
    case BORDER_REFLECT:                                                                                               \
        (void)dim;                                                                                                     \
        BORDER_OFFSET_REFLECT(list, position)                                                                          \
        break;                                                                                                         \
    default:                                                                                                           \
        break;                                                                                                         \
    }

#ifdef __cplusplus
}
#endif

#endif