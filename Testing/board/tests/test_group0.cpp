#include "common.h"
#include "load.hpp"
#include "test_config.h"
#include <vector>

extern "C" {
    #include "cv/linear_filters.h"
    //#include "arm_linear_filter_common.h"

}

#if defined(TESTGROUP0)

void test_gauss(const unsigned char* inputs,
                 unsigned char* &outputs,
                 uint32_t &total_bytes,
                 uint32_t test_id,
                 long &cycles)
{
    long start,end;
    uint32_t width,height;
    int bufid = TENSOR_START + test_id;

    get_img_dims(inputs,bufid,&width,&height);
    std::vector<BufferDescription> desc = {BufferDescription(Shape(height,width)
                                                            ,kIMG_GRAY8_TYPE)
                                          };

    outputs = create_write_buffer(desc,total_bytes);

    const uint8_t *src = Buffer<uint8_t>::read(inputs,bufid);
    uint8_t *dst = Buffer<uint8_t>::write(outputs,0);

    const arm_cv_image_gray8_t input={(uint16_t)width,(uint16_t)height,(uint8_t*)src};
    arm_cv_image_gray8_t output={(uint16_t)width,(uint16_t)height,(uint8_t*)dst};
    
    // The test to run is executed with some timing code.
    start = time_in_cycles();
    arm_gaussian_filter_3x3_fixp(&input,&output);
    end = time_in_cycles();
    cycles = end - start;
}
void test_gauss2(const unsigned char* inputs,
                 unsigned char* &outputs,
                 uint32_t &total_bytes,
                 uint32_t test_id,
                 long &cycles,
                 int8_t mode)
{
    long start,end;
    uint32_t width,height;
    int bufid = TENSOR_START + test_id;
    int border_type;
    switch (mode)
    {
    case 2:
        border_type = Border_Wrap;
        break;
    case 1:
        border_type = Border_Reflect;/*Wrap;*/
        break;
    case 0:
        border_type = Border_Replicate;/*Wrap;*/
        break;
    default:
        border_type = Border_Reflect;
        break;
    }
    //int8_t border_type = Border_Reflect;
    //int8_t border_type = Border_Replicate;
    //int8_t border_type = Border_Wrap;
    get_img_dims(inputs,bufid,&width,&height);
    std::vector<BufferDescription> desc = {BufferDescription(Shape(height,width)
                                                            ,kIMG_GRAY8_TYPE)
                                          };

    outputs = create_write_buffer(desc,total_bytes);
    q15_t* Buffer_tmp = (q15_t*)malloc(width*sizeof(q15_t));
    const uint8_t *src = Buffer<uint8_t>::read(inputs,bufid);
    uint8_t *dst = Buffer<uint8_t>::write(outputs,0);

    const arm_cv_image_gray8_t input={(uint16_t)width,(uint16_t)height,(uint8_t*)src};
    arm_cv_image_gray8_t output={(uint16_t)width,(uint16_t)height,(uint8_t*)dst};
    
    // The test to run is executed with some timing code.
    start = time_in_cycles();
    arm_linear_filter_generic(&input,&output, Buffer_tmp, border_type);
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
    switch(funcid)
    {
        case 0:
            test_gauss2(inputs,wbuf,total_bytes,testid,cycles, 0);
            break;
        case 1:
            test_gauss2(inputs,wbuf,total_bytes,testid,cycles, 1);
            break;
        case 2:
            test_gauss2(inputs,wbuf,total_bytes,testid,cycles, 2);
            //test_gauss(inputs,wbuf,total_bytes,testid,cycles);
            break;
    }

}

#endif