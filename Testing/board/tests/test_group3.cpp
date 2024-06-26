#include "common.h"
#include "load.hpp"
#include "test_config.h"
#include <vector>

extern "C" {
    #include "cv/feature_detection.h"
}

#if defined(TESTGROUP3)


void test_sobel(const unsigned char* inputs,
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

    arm_cv_gradient_q15_t* Buffer_tmp_grad1 = (arm_cv_gradient_q15_t*)malloc(width*sizeof(arm_cv_gradient_q15_t));
    q15_t* Buffer_tmp_mag = (q15_t*)malloc(width*sizeof(q15_t));
    arm_cv_gradient_q15_t* Buffer_tmp_grad2 = (arm_cv_gradient_q15_t*)malloc(width*sizeof(arm_cv_gradient_q15_t));
    int height3 = 3;
    arm_cv_image_gradient_q15_t Img_tmp_grad1 = {(uint16_t)width, (uint16_t)height3, (arm_cv_gradient_q15_t*)Buffer_tmp_grad1};
    arm_cv_image_q15_t Img_tmp_mag = {(uint16_t)width, (uint16_t)height3, (q15_t*)Buffer_tmp_mag};
    arm_cv_image_gradient_q15_t Img_tmp_grad2 = {(uint16_t)width, (uint16_t)height3, (arm_cv_gradient_q15_t*)Buffer_tmp_grad2};
    

    // The test to run is executed with some timing code.
    start = time_in_cycles();
    arm_canny_edge_sobel_fixp(&input,&output, &Img_tmp_grad1, &Img_tmp_mag, &Img_tmp_grad2, 78,33);
    end = time_in_cycles();
    cycles = end - start;

    free(Buffer_tmp_grad1);
    free(Buffer_tmp_mag);
    free(Buffer_tmp_grad2);
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
            test_sobel(inputs,wbuf,total_bytes,testid,cycles);
            break;
    }

}

#endif