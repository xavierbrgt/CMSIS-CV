#include "common.h"
#include "load.hpp"
#include "test_config.h"
#include <vector>
#include <stdlib.h>

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

    get_img_dims(inputs,bufid,&width,&height);
    std::vector<BufferDescription> desc = {BufferDescription(Shape(height,width)
                                                            ,kIMG_GRAY8_TYPE)
                                          };

    outputs = create_write_buffer(desc,total_bytes);

    const uint8_t *src = Buffer<uint8_t>::read(inputs,bufid);
    uint8_t *dst = Buffer<uint8_t>::write(outputs,0);

    const arm_cv_image_gray8_t input={(uint16_t)width,(uint16_t)height,(uint8_t*)src};
    arm_cv_image_gray8_t output={(uint16_t)width,(uint16_t)height,(uint8_t*)dst};

    arm_cv_gradient_q15_t* Buffer_tmp_grad1 = (arm_cv_gradient_q15_t*)malloc(3*width*sizeof(arm_cv_gradient_q15_t));
    q15_t* Buffer_tmp_mag = (q15_t*)malloc(3*width*sizeof(q15_t));
    arm_cv_gradient_q15_t* Buffer_tmp_grad2 = (arm_cv_gradient_q15_t*)malloc(3*width*sizeof(arm_cv_gradient_q15_t));
    int height3 = 3;
    arm_cv_image_gradient_q15_t Img_tmp_grad1 = {(uint16_t)width, (uint16_t)height3, (arm_cv_gradient_q15_t*)Buffer_tmp_grad1};
    arm_cv_image_q15_t Img_tmp_mag = {(uint16_t)width, (uint16_t)height3, (q15_t*)Buffer_tmp_mag};
    arm_cv_image_gradient_q15_t Img_tmp_grad2 = {(uint16_t)width, (uint16_t)height3, (arm_cv_gradient_q15_t*)Buffer_tmp_grad2};
    
    //uint8_t *buff_int =(uint8_t*)malloc(height*width*sizeof(uint8_t));
    //arm_cv_image_gray8_t intermediate={(uint16_t)width,(uint16_t)height,(uint8_t*)buff_int};
    // The test to run is executed with some timing code.
    start = time_in_cycles();
    printf("still running1");
    //arm_gaussian_filter_3x3_fixp(&input,&output);
    arm_canny_edge_sobel_fixp(&input,&output, &Img_tmp_grad1, &Img_tmp_mag, &Img_tmp_grad2, 78,33);
    
    printf("still running5\n");
    end = time_in_cycles();
    cycles = end - start;

    free(Buffer_tmp_grad1);
    free(Buffer_tmp_mag);
    free(Buffer_tmp_grad2);
    printf("still running6\n");
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
    printf("still running7\n");
}

#endif