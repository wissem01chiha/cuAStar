/**
 * 
 * 
 */
#ifndef A_STAR_PLANNER_HPP
#define A_STAR_PLANNER_HPP

#include <cstdint>
#include <cfloat>
#ifdef _WIN32
  #include <windows.h>
#endif
#if defined(__CUDACC__)
    #define HOST_FUN __host__
    #define DEVICE_FUN __device__
    #define HOST_DEVICE_FUN __host__ __device__
    #define GLOBAL_FUN __global__
    #include <cuda_runtime.h>
    #include <math_constants.h>
#else
    #define HOST_FUN 
    #define DEVICE_FUN 
    #define HOST_DEVICE_FUN 
    #include <vector>
    #include <cmath>
    #include <string>
    #include <corecrt_math_defines.h>
#endif
#ifdef ENABLE_SSE && !defined(__CUDACC__)
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
#include "common.hpp"
#include "math.hpp"
#include "planner.hpp"

class AstarPlanner
{
public:
    /** @brief  Default constructor */
    HOST_DEVICE_FUN  AstarPlanner();

    /** @brief compute and retun the optimal trajectory */
    HOST_DEVICE_FUN void computeTrajectory(double sx, double sy, double gx, double gy,
                                  double* ox_, double* oy_, int n,
                                  double step, double rr,
                                  int32_t* visit_map, double* path_cost,
                                  internal::Node2d* motion_model,internal::Node2d* traj_nodes);
private:
    /** @brief */
    GLOBAL_FUN void collison();

    /** @brief */
    GLOBAL_FUN void computeHeuristic(internal::Node2d* n1, internal::Node2d* n2,
                                  double w,double* hfun);
    /** @brief  */
    GLOBAL_FUN void  verifyNode(internal::Node2d* node,const int32_t* obmap, 
                              int32_t min_ox,int32_t max_ox, int32_t min_oy, 
                              int32_t max_oy, int32_t xwidth, bool* state);
    /** @brief */
    GLOBAL_FUN void computeObstacleMap(const int32_t* ox, const int32_t* oy, 
                                      int32_t num_obstacles,
                                      const int32_t min_ox, const int32_t max_ox,
                                      const int32_t min_oy, const int32_t max_oy,
                                      double step, double vr, int32_t* obmap);
    /** @brief compute the final path */
    GLOBAL_FUN void computeFinalPath(internal::Node2d * goal, double step,
                                  double* rx, double* ry, int* path_size);

    /** @brief */
    GLOBAL_FUN void computeMotionModel(internal::Node2d* motion_model);
protected:
  HOST_DEVICE_FUN ~AstarPlanner();
};
#endif