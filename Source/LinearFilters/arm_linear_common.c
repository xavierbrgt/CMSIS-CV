
#include "arm_linear_filter_common.h"
#include "cv/linear_filters.h"
#include "dsp/basic_math_functions.h"


/**
  @ingroup linearFilter
 */

/**
 * @brief      Return the scratch size for generic gaussian function
 *
 * @param[in]     width        The width of the image in pixels
 * @return		  Scratch size in bytes
 */
uint16_t arm_cv_get_scratch_size_generic(int width)
{
    return (width * sizeof(q15_t));
}

/**
  @ingroup linearFilter
 */

/**     
 * @brief          Sobel filter computing the gradient on the vertical axis
 *
 * @param[in]      imageIn     The input image
 * @param[out]     imageOut    The output image
 * @param[in,out]  scratch     Buffer
 * @param[in]      borderType  Type of border to use, supported are Replicate Wrap and Reflect
 * 
 * @par Temporary buffer sizing:
 * 
 * Will use a temporary buffer to store intermediate values of gradient and magnitude.
 *
 * Size of temporary buffer is given by
 * arm_cv_get_scratch_size_sobel_x(int width)
 */
uint16_t arm_cv_get_scratch_size_sobel(int width)
{
    return (width * sizeof(q15_t));
}