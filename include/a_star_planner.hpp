/************************************************************
 *  
 ************************************************************/
#ifndef A_STAR_PLANNER_HPP
#define A_STAR_PLANNER_HPP

#define ENABLE_CUDA_ARCH 1
#include <cstdint>
#ifdef _WIN32
  #include <windows.h>
#endif
#if defined(ENABLE_CUDA_ARCH) && defined(ENABLE_GLM)
    #error "ENABLE_CUDA_ARCH and ENABLE_GLM "
#elif !defined(ENABLE_CUDA_ARCH) && !defined(ENABLE_GLM)
    #error "Either ENABLE_CUDA_ARCH or ENABLE_GLM. Required."
#elif defined(ENABLE_CUDA_ARCH)
    #include <cuda_runtime.h>
    #include <cublas_v2.h>
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
#include "utils.hpp"
#include "struct.hpp"
#include "motion_model.hpp"

class AstarPlanner
{
public:
#ifdef ENABLE_CUDA_ARCH
    __device__ AstarPlanner();
    __device__ void computeFinalPath(internal::Node * goal, float step, float* rx, float* ry, int* path_size);
    __device__ void computeObstacleMap(const int32_t* ox, const int32_t* oy, int32_t num_obstacles,
                                    const int32_t min_ox, const int32_t max_ox,
                                    const int32_t min_oy, const int32_t max_oy,
                                    float step, float vr, int32_t* obmap);
    __device__ void  verifyNode(internal::Node* node, const int32_t* obmap, int32_t min_ox,int32_t max_ox,
                                    int32_t min_oy, 
                                    int32_t max_oy,
                                    int32_t xwidth,
                                    bool* state);
    __device__ void computeHeuristic(internal::Node* n1, internal::Node* n2, float w, double* hfun);
    __device__ void computeMotionModel();
    __global__ void execute(double sx, double sy, double gx,  double gy, float* ox_, 
                           float* oy_, float* step, float* rr);
#else
    AstarPlanner();
    void computeFinalPath(internal::Node * goal, float step, float* rx, float* ry, int* path_size);
    void computeObstacleMap(const int32_t* ox, const int32_t* oy, int32_t num_obstacles,
                                    const int32_t min_ox, const int32_t max_ox,
                                    const int32_t min_oy, const int32_t max_oy,
                                    float step, float vr, int32_t* obmap);
    void  verifyNode(internal::Node* node, const int32_t* obmap, int32_t min_ox,int32_t max_ox,
                                    int32_t min_oy, 
                                    int32_t max_oy,
                                    int32_t xwidth,
                                    bool* state);
    void computeHeuristic(internal::Node* n1, internal::Node* n2, float w, double* hfun);
    void computeMotionModel();
    void execute(double sx, double sy, double gx,  double gy, float* ox_, 
                           float* oy_, float* step, float* rr);
#endif
protected:
#ifndef ENABLE_CUDA_ARCH
    ~AstarPlanner();
#endif
};
#endif