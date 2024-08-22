/*************************************************************
 * Math Engine 
 * All function must be device !
 * 
 *************************************************************/
#ifndef MATH_HPP
#define MATH_HPP

#include <cstdint>

#if defined(ENABLE_CUDA_ARCH) && defined(ENABLE_GLM)
    #error "ENABLE_CUDA_ARCH and ENABLE_GLM "
#elif !defined(ENABLE_CUDA_ARCH) && !defined(ENABLE_GLM)
    #error "ENABLE_CUDA_ARCH or ENABLE_GLM. Required."
#elif defined(ENABLE_CUDA_ARCH)
    #include <cuda_runtime.h>
    #include <cublas_v2.h>
    #include <device_launch_parameters.h>
#elif defined(ENABLE_GLM)
    #include <glm/glm.hpp>
    #include <glm/gtc/constants.hpp>
#endif
#ifdef ENABLE_SSE && !defined(ENABLE_CUDA_ARCH)
  #ifdef _MSC_VER
    #if defined(_M_IX86_FP) && _M_IX86_FP >= 1
      #include <intrin.h>
      #include <xmmintrin.h>
    #endif
  #endif
#endif

#ifdef ENABLE_CUDA_ARCH
__device__ void vec_diff(const double* input,double* output, int32_t n){
  for(int i=1; i <n; i++){
    output[i] = input[i] - input[i-1];
  }
};

__device__ void cum_sum(const double* input, double* output, int32_t n){
  double temp = 0;
  for(int i = 0; i < n; i++ ){
    temp += input[i];
    output[i] = temp;
  }
};
#endif

#ifdef ENABLE_GLM
void vec_diff(const glm::dvec2* input, glm::dvec2* output, int32_t n) {
    for (int i = 1; i < n; i++) {
        output[i] = input[i] - input[i - 1];  
    }
}

void cum_sum(const glm::dvec2* input, glm::dvec2* output, int32_t n) {
    glm::dvec2 temp(0.0, 0.0);   
    for (int i = 0; i < n; i++) {
        temp += input[i];   
        output[i] = temp;   
    }
}
#endif
#endif