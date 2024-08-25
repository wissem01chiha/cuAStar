#ifndef CUBIC_SPLINE_HPP
#define CUBIC_SPLINE_HPP

#include <cstdint>
#ifdef _WIN32
  #include <windows.h>
#endif
#if defined(ENABLE_CUDA_ARCH) && defined(ENABLE_GLM)
    #error "ENABLE_CUDA_ARCH and ENABLE_GLM "
#elif !defined(ENABLE_CUDA_ARCH) && !defined(ENABLE_GLM)
    #warning "ENABLE_CUDA_ARCH or ENABLE_GLM. Required."
#elif defined(ENABLE_CUDA_ARCH)
    #include <cuda_runtime.h>
    #include <cublas_v2.h>
    #include <device_launch_parameters.h>
    #include <device_functions.h> 
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
#ifdef _DEBUG_
  #include "../extern/loguru/loguru.hpp"
  #include "../extern/loguru/loguru.cpp"
#endif

#include "utils.hpp"
#include "math.hpp"
#include "common.hpp"

using Vec_f=std::vector<float>;
using Poi_f=std::array<float, 2>;
using Vec_Poi=std::vector<Poi_f>;
 // d_i * (x-x_i)^3 + c_i * (x-x_i)^2 + b_i * (x-x_i) + a_i
class Spline{
public:
  double* x;
  double* y;
  int32_t nx;
  double* h;
  double* a;
  double* b;
  double* c;
  double* d;

#if defined(ENABLE_CUDA_ARCH) 
  __device__ Spline(int32_t n );
  __global__ Spline(double* x_, double* y_, int32_t nx);
  __device__ double calc(double t);
  __device__ double calc_d(double t);
  __device__ double calc_dd(double t);
  __global__ void computeMatrixA(double* d_A, const double* h, int32_t nx);
  __global__ void computeMatrixB(double* d_B, const double* h, int32_t nx);
  __device__ int bisect(double t, int start, int end);
  #elif defined(ENABLE_GLM)
    Spline(int32_t n );
    double calc(double t);
    double calc_dd(double t);
    void computeMatrixA(double* d_A, const double* h, int32_t nx);
    void computeMatrixB(double* d_B, const double* h, int32_t nx);
    int bisect(double t, int start, int end);
  #endif
protected:
#if defined(ENABLE_CUDA_ARCH)
  __device__ ~Spline();
#endif
};

class Spline2d{
public:
  Spline sx;
  Spline sy;
  Vec_f  s;
#if defined(ENABLE_CUDA_ARCH) 
  __device__ Spline2d(Vec_f x, Vec_f y);
  __device__ Poi_f  computePostion(double s_t);
  __global__ double computeCurvature(double s_t);
  __global__ double computeYaw(double s_t);
#elif defined(ENABLE_GLM)

#endif
private:
  Vec_f calc_s(Vec_f x, Vec_f y);
protected:
#if defined(ENABLE_CUDA_ARCH)
  __device__ ~Spline2d();
#endif
};

class Spline3d{
public:
  Spline sx;
  Spline sy;
  Vec_f  s;
#if defined(ENABLE_CUDA_ARCH) 
  __device__ Spline3d(Vec_f x, Vec_f y);
  __device__ Poi_f  computePostion(double s_t);
  __global__ double computeCurvature(double s_t);
  __global__ double computeYaw(double s_t);
#elif defined(ENABLE_GLM)

#endif
private:
  Vec_f calc_s(Vec_f x, Vec_f y);
protected:
#if defined(ENABLE_CUDA_ARCH)
  __device__ ~Spline3d();
#endif
};

#endif