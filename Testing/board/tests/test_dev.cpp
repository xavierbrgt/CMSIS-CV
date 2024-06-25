#include "common.h"
#include "load.hpp"
#include "test_config.h"
#include <vector>
#include <stdlib.h>
#include <stdio.h>
extern "C" {
    #include "cv/feature_detection.h"
    #include "cv/linear_filters.h"
}

#if defined(TESTDEV)

void test_dev(const unsigned char* inputs,
                 unsigned char* &outputs,
                 uint32_t &total_bytes,
                 long &cycles)
{
    long start,end;
    uint32_t width,height;
    int bufid = TENSOR_START;
    
    //int8_t border_type = Border_Reflect;
    int8_t border_type = Border_Replicate;
    //int8_t border_type = Border_Wrap;
    get_img_dims(inputs,bufid,&width,&height);
    std::vector<BufferDescription> desc = {BufferDescription(Shape(height,width)
                                                            ,kIMG_NUMPY_TYPE_SINT16)
                                          };

    outputs = create_write_buffer(desc,total_bytes);
    q15_t* Buffer_tmp = (q15_t*)malloc(width*sizeof(q15_t));
    const uint8_t *src = Buffer<uint8_t>::read(inputs,bufid);
    int16_t *dst = Buffer<int16_t>::write(outputs,0);

    const arm_cv_image_gray8_t input={(uint16_t)width,(uint16_t)height,(uint8_t*)src};
    //arm_cv_image_q15_t output={(uint16_t)width,(uint16_t)height,(int16_t*)dst};
    arm_cv_image_q15_t output;
    output.width=width;
    output.height=height;
    output.pData=dst;
    
    // The test to run is executed with some timing code.
    start = time_in_cycles();
    //arm_linear_filter_generic(&input,&output, Buffer_tmp, border_type);
    arm_sobel_y(&input,&output, Buffer_tmp, border_type);
    end = time_in_cycles();
    cycles = end - start;
    free(Buffer_tmp);
}

void run_test(const unsigned char* inputs,
              const uint32_t testid,
              const uint32_t funcid,
              unsigned char* &wbuf,
              uint32_t &total_bytes,
              long &cycles)
{

    wbuf = nullptr;
    (void)testid;
    (void)funcid;
    test_dev(inputs,wbuf,total_bytes,cycles);
}

#endif