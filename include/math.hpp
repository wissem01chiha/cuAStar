#ifndef MATH_HPP
#define MATH_HPP

#include <cstdint>

#if defined(ENABLE_CUDA_ARCH) && defined(ENABLE_GLM)
    #warning "ENABLE_CUDA_ARCH and ENABLE_GLM "
#elif !defined(ENABLE_CUDA_ARCH) && !defined(ENABLE_GLM)
    #error "ENABLE_CUDA_ARCH or ENABLE_GLM. Required."
#elif defined(ENABLE_CUDA_ARCH)
    #include <cuda_runtime.h>
    #include <cublas_v2.h>
    #include <device_launch_parameters.h>
#elif defined(ENABLE_GLM)
    #include <glm/glm.hpp>
    #include <glm/gtc/constants.hpp>
    #include <glm/gtc/matrix_inverse.hpp> 
    #include <vector_double2.hpp>
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

namespace linAlg{
  __device__ void inv3x3mat(const double *A, double *A_inv){
      double det = A[0] * (A[4] * A[8] - A[5] * A[7]) -
                 A[1] * (A[3] * A[8] - A[5] * A[6]) +
                 A[2] * (A[3] * A[7] - A[4] * A[6]);
      if (det == 0.0) {
        return;
    }
    double inv_det = 1.0 / det;
    A_inv[0] =  inv_det * (A[4] * A[8] - A[5] * A[7]);
    A_inv[1] = -inv_det * (A[1] * A[8] - A[2] * A[7]);
    A_inv[2] =  inv_det * (A[1] * A[5] - A[2] * A[4]);

    A_inv[3] = -inv_det * (A[3] * A[8] - A[5] * A[6]);
    A_inv[4] =  inv_det * (A[0] * A[8] - A[2] * A[6]);
    A_inv[5] = -inv_det * (A[0] * A[5] - A[2] * A[3]);

    A_inv[6] =  inv_det * (A[3] * A[7] - A[4] * A[6]);
    A_inv[7] = -inv_det * (A[0] * A[7] - A[1] * A[6]);
    A_inv[8] =  inv_det * (A[0] * A[4] - A[1] * A[3]);
}

__device__ void mat3x3MulVec(const double* A, const double* v, double* result){
    result[0] = A[0] * v[0] + A[1] * v[1] + A[2] * v[2];
    result[1] = A[3] * v[0] + A[4] * v[1] + A[5] * v[2];
    result[2] = A[6] * v[0] + A[7] * v[1] + A[8] * v[2];
};
}; // namespace linAlg

__device__ void computeUpdateMinMax(double* o_,double step,int32_t* min_o,int32_t* max_o,int32_t n){
   int tid = threadIdx.x + blockIdx.x * blockDim.x;
   if (tid < n) {
    int32_t map_x = (int32_t)round(o_[tid] / step);
    atomicMin(min_o, map_x);
    atomicMax(max_o, map_x);
   };
};

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

__device__ void interp_refer(const double* params, double x, double* result){
  *result =  params[0] * x * x + params[1] * x + params[2];
};

__device__ void quadratic_interpolation(double* result_array,double* x, 
                                      double* y){
  double A[9];
  double Y[3];
  A[0] = x[0] * x[0]; A[1] = x[0]; A[2] = 1.0;
  A[3] = x[1] * x[1]; A[4] = x[1]; A[5] = 1.0;
  A[6] = x[2] * x[2]; A[7] = x[2]; A[8] = 1.0;

  Y[0] = y[0];
  Y[1] = y[1];
  Y[2] = y[2];
  double A_inv[9];
  linAlg::inv3x3mat(A,A_inv);
  linAlg::mat3x3MulVec(A_inv,Y,result_array);
};

__global__ void calc_third_derivative(double *t, double *results, double a3, 
                                    double a4, double a5, int n){
    int idx = threadIdx.x + blockIdx.x * blockDim.x;
    if (idx < n) {
        results[idx] = 6 * a3 + 24 * a4 * t[idx] + 60 * a5 * pow(t[idx], 2);
    }
};

#endif

#ifdef defined(ENABLE_GLM)
void vec_diff(const glm::dvec2* input, glm::dvec2* output, int32_t n) {
    for (int i = 1; i < n; i++) {
        output[i] = input[i] - input[i - 1];  
    }
};

void cum_sum(const glm::dvec2* input, glm::dvec2* output, int32_t n){
    glm::dvec2 temp(0.0, 0.0);   
    for (int i = 0; i < n; i++) {
        temp += input[i];   
        output[i] = temp;   
    }
};

void interp_refer(const double* params, double x, double* result){
  *result =  params[0] * x * x + params[1] * x + params[2];
};

void quadratic_interpolation(const double x[3], const double y[3], double* result) {
    glm::mat3 A(
        glm::pow(x[0], 2), x[0], 1,
        glm::pow(x[1], 2), x[1], 1,
        glm::pow(x[2], 2), x[2], 1
    );

    glm::vec3 Y(y[0], y[1], y[2]);

    glm::vec3 solution = glm::inverse(A) * Y;
    result[0] = solution[0];
    result[1] = solution[1];
    result[2] = solution[2];
};
#endif
#endif