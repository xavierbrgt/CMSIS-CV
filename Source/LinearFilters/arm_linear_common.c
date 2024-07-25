
#include "arm_linear_filter_common.h"
#include "cv/linear_filters.h"
#include "dsp/basic_math_functions.h"


/**
  @ingroup linearFilter
 */

/**
 * @brief      Return the scratch size for generic linear filter function
 *
 * @param[in]     width        The width of the image in pixels
 * @return		  Scratch size in bytes
 */
uint16_t arm_get_scratch_size_generic(int width)
{
    return (width * sizeof(q15_t));
}

/**
  @ingroup linearFilter
 */

/**     
 * @brief      Return the scratch size for Sobbel functions, same as the generic linear filters
 *
 * @param[in]     width        The width of the image in pixels
 * @return		  Scratch size in bytes
 */
uint16_t arm_get_scratch_size_sobel(int width)
{
    return (width * sizeof(q15_t));
}