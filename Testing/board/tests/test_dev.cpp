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


/*void test_dev(const unsigned char* inputs,
                 unsigned char* &outputs,
                 uint32_t &total_bytes,
                 long &cycles)
{
    printf("start test dev\n");
    long start,end;
    uint32_t width,height;
    int bufid = TENSOR_START + 0;

    // BGR_8U3C has dimension [3,H,W]
    get_img_dims(inputs,bufid,&height,&width);


    printf("no issue \n");
    std::vector<BufferDescription> desc = {BufferDescription(Shape(height,width)
                                                            ,kIMG_GRAY8_TYPE)
                                          };

    outputs = create_write_buffer(desc,total_bytes);

    printf("no issue at this point\n");
    uint8_t *dst = Buffer<uint8_t>::write(outputs,0);
    printf("issue \n");
    const uint8_t *src = Buffer<uint8_t>::read(inputs,bufid);    
    printf("no issue \n");
    arm_cv_image_gradient_q15_t Img_tmp_grad1;
    arm_cv_image_q15_t Img_tmp_mag;
    arm_cv_image_gradient_q15_t Img_tmp_grad2;
    printf("no issue \n");
    arm_cv_gradient_q15_t *tmp_grad1 = (arm_cv_gradient_q15_t*)malloc(3*width*sizeof(arm_cv_gradient_q15_t));
    printf("no issue \n");
    int16_t *tmp_mag = (int16_t*)malloc(3*width*sizeof(q15_t));
    printf("no issue \n");
    arm_cv_gradient_q15_t *tmp_grad2 = (arm_cv_gradient_q15_t*)malloc(3*width*sizeof(arm_cv_gradient_q15_t));
    printf("no issue \n");
    Img_tmp_mag.width=width;
    Img_tmp_mag.height=3;
    Img_tmp_mag.pData=tmp_mag;
    printf("no issue \n");
    Img_tmp_grad1.width=width;
    Img_tmp_grad1.height=3;
    Img_tmp_grad1.pData=tmp_grad1;
    printf("no issue \n");
    Img_tmp_grad2.width=width;
    Img_tmp_grad2.height=3;
    Img_tmp_grad2.pData=tmp_grad2;
    
    const arm_cv_image_gray8_t input={(uint16_t)width,
                                       (uint16_t)height,
                                       (uint8_t*)src};
    arm_cv_image_gray8_t output;
    output.width=width;
    output.height=height;
    output.pData=dst;
    printf("no issue \n");
    int low_threshold = 78;
    int high_threshold = 33;
    
    // The test to run is executed with some timing code.
    start = time_in_cycles();
    arm_canny_edge_sobel_fixp( &input, &output, &Img_tmp_grad1, &Img_tmp_mag, &Img_tmp_grad2, low_threshold, high_threshold);
    end = time_in_cycles();
    cycles = end - start;
    printf("no issue \n");

    free(tmp_grad1);
    free(tmp_grad2);
    free(tmp_mag);
}*/

void test_dev(const unsigned char* inputs,
                 unsigned char* &outputs,
                 uint32_t &total_bytes,
                 long &cycles)
{
    long start,end;
    uint32_t width,height;
    int bufid = TENSOR_START;
    
    //int8_t border_type = Border_Reflect;
    //int8_t border_type = Border_Replicate;
    int8_t border_type = Border_Wrap;
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
    (void)testid;
    (void)funcid;
    test_dev(inputs,wbuf,total_bytes,cycles);
}

#endif