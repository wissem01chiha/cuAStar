#ifndef COMMON_HPP
#define COMMON_HPP

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
#endif
#ifdef ENABLE_SSE && !defined(ENABLE_CUDA_ARCH)
  #ifdef _MSC_VER
    #if defined(_M_IX86_FP) && _M_IX86_FP >= 1
      #include <intrin.h>
      #include <xmmintrin.h>
    #endif
  #endif
#endif



namespace internal{
/**
 * @brief Base class for 2d nodes reprsentation
 * @param p_node : pointer to the parent node
 * @param sum_cost : cumulative cost from the start node to the current node.
 * @note the node postion is parmtrised in grid steps integer number 
 */
class Node2d{
  public:
    int32_t x;
    int32_t y;
    double  sum_cost;
    Node2d* p_node;

  __host__ __device__ Node2d(int32_t x_, int32_t y_,
                          double sum_cost_=0, 
                          Node2d* p_node_=nullptr){
 x = x_; y = y_; 
 sum_cost = sum_cost_; 
 p_node = p_node_;
}
};

class Node3d{
  public:
    int32_t x;
    int32_t y;
    int32_t z;
    double  sum_cost;
    Node3d* p_node;

  __host__ __device__  Node3d(int32_t x_, int32_t y_,
                            int32_t z,
                            double sum_cost_=0, 
                            Node3d* p_node_=nullptr){
    x= x_; y = y_;
    sum_cost = sum_cost_;
    p_node = p_node_;
  }

  __host__ __device__ double distanceTo(const Node3d& other){

  };


};

class State2d {
  double x;
  double y;
  double yaw;
  double v;
  State2d* p_state;
  __host__ __device__ State2d(double x_,double y_,
                          double yaw_ ,double v_){
    x = x_;
    y = y_;
    yaw = yaw_;
    v = v_;
  };
};

class State3d {
  double x;
  double y;
  double z;
  double yaw;
  double pitch;
  double roll;
  State3d* p_state;
  __host__ __device__ State3d(double x_,double y_,double z_,
                            double yaw_,double pitch_, 
                            double roll_){
    x = x_;
    y = y_;
    yaw   = yaw_;
    pitch = pitch_;
    roll  = roll_;
  };
};

struct TrajectoryState{
  float x;
  float y;
  float yaw;
  __host__ __device__ TrajectoryState(float x_, float y_, 
                                    float yaw_){
    x = x_;
    y = y_;
    yaw = yaw_;
  };
};
}; // namespace internal
#endif