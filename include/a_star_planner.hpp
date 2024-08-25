#ifndef A_STAR_PLANNER_HPP
#define A_STAR_PLANNER_HPP

#include <cstdint>
#include <cfloat>
#ifdef _WIN32
  #include <windows.h>
#endif
#if defined(ENABLE_CUDA_ARCH) && defined(ENABLE_GLM)
    #warning "ENABLE_CUDA_ARCH and ENABLE_GLM "
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
#ifdef _DEBUG_
  #include "../extern/loguru/loguru.hpp"
  #include "../extern/loguru/loguru.cpp"
#endif
#include "utils.hpp"
#include "common.hpp"
#include "math.hpp"
#include "planner.hpp"

/**
 * @brief 
 * all public member function should be host function 
 */
class AstarPlanner
{
public:
#ifdef ENABLE_CUDA_ARCH

    /** @brief  Default constructor */
    __host__  AstarPlanner();

    /** @brief */
    __host__ void computeTrajectory(double sx, double sy, double gx, double gy,
                                  double* ox_, double* oy_, int n,
                                  double step, double rr,
                                  int32_t* visit_map, double* path_cost,
                                  internal::Node2d* motion_model,internal::Node2d* traj_nodes);
#elif defined(ENABLE_GLM)

    /** @brief Default constructor */
    AstarPlanner();
    
    /** @brief compute the planning trajectory */
    void computeTrajectory(double sx, double sy, double gx,  double gy, double* ox_, 
                           double* oy_, double* step, double* rr);
#endif
private:
#ifdef ENABLE_CUDA_ARCH

  /** @brief */
  __global__ void collison();

  /** @brief */
  __global__ void computeHeuristic(internal::Node2d* n1, internal::Node2d* n2,
                                double w,double* hfun);
  /** @brief  */
  __global__ void  verifyNode(internal::Node2d* node,const int32_t* obmap, 
                            int32_t min_ox,int32_t max_ox, int32_t min_oy, 
                            int32_t max_oy, int32_t xwidth, bool* state);
  /** @brief */
  __global__ void computeObstacleMap(const int32_t* ox, const int32_t* oy, 
                                    int32_t num_obstacles,
                                    const int32_t min_ox, const int32_t max_ox,
                                    const int32_t min_oy, const int32_t max_oy,
                                    double step, double vr, int32_t* obmap);
  /** @brief compute the final path */
  __global__ void computeFinalPath(internal::Node2d * goal, double step,
                                double* rx, double* ry, int* path_size);

  /** @brief */
  __global__ void computeMotionModel(internal::Node2d* motion_model);

#elif defined(ENABLE_GLM)

    /** @brief */
    void computeFinalPath(internal::Node2d * goal, double step, double* rx, double* ry, int* path_size);

    /** @brief  */
    void computeObstacleMap(const int32_t* ox, const int32_t* oy, int32_t num_obstacles,
                                    const int32_t min_ox, const int32_t max_ox,
                                    const int32_t min_oy, const int32_t max_oy,
                                    double step, double vr, int32_t* obmap);
    void  verifyNode(internal::Node2d* node, const int32_t* obmap, int32_t min_ox,int32_t max_ox,
                                    int32_t min_oy, 
                                    int32_t max_oy,
                                    int32_t xwidth,
                                    bool* state);
    void computeHeuristic(internal::Node2d* n1, internal::Node2d* n2, double w, double* hfun);
    void computeMotionModel(internal::Node2d* motion_model);
#endif
protected:
#ifdef ENABLE_CUDA_ARCH
  ~AstarPlanner();
#elif defined(ENABLE_GLM)
  ~AstarPlanner();
#endif
};
#endif