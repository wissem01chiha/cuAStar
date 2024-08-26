#ifndef COMMON_HPP
#define COMMON_HPP

#include <cstdint>

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

    HOST_DEVICE_FUN Node2d(int32_t x_, int32_t y_,
                              double sum_cost_=0, 
                              Node2d* p_node_=nullptr){
    x = x_; y = y_; 
    sum_cost = sum_cost_; 
    p_node = p_node_;
    }
    HOST_DEVICE_FUN bool isEqual(const Node2d& other,double eps=1e-6){
    return fabs(other.x - x) < eps &&
           fabs(other.y - y) < eps;
    }

};

class Node3d{
  public:
    int32_t x;
    int32_t y;
    int32_t z;
    double  sum_cost;
    Node3d* p_node;

  HOST_DEVICE_FUN  Node3d(int32_t x_, int32_t y_,
                            int32_t z,
                            double sum_cost_=0, 
                            Node3d* p_node_=nullptr){
    x= x_; y = y_;
    sum_cost = sum_cost_;
    p_node = p_node_;
  }

  HOST_DEVICE_FUN double distanceTo(const Node3d& other) const {
        return sqrt(double((other.x - x) * (other.x - x) +
                           (other.y - y) * (other.y - y) +
                           (other.z - z) * (other.z - z)));
  }

  HOST_DEVICE_FUN bool isEqual(const Node3d& other,double eps=1e-6){
    return fabs(other.x - x) < eps &&
           fabs(other.y - y) < eps &&
           fabs(other.z - z) < eps;
  }
};

class State3d {
  double x;
  double y;
  double z;
  double yaw;
  double pitch;
  double roll;
  State3d* p_state;

  HOST_DEVICE_FUN State3d(double x_,double y_,double z_,
                        double yaw_,double pitch_, 
                        double roll_){
    x = x_;
    y = y_;
    yaw   = yaw_;
    pitch = pitch_;
    roll  = roll_;
    };
};

class State2d {
  double x;
  double y;
  double yaw;
  State2d* p_state;

  HOST_DEVICE_FUN State2d(double x_,double y_,double yaw_,
                        State2d* p_state_=nullptr){
    x = x_;
    y = y_;
    yaw   = yaw_;
    p_state = p_state_;
    };
};

class TrajectoryState{
  float x;
  float y;
  float yaw;
 
  HOST_DEVICE_FUN TrajectoryState(float x_, float y_, 
                                      float yaw_){
    x = x_;
    y = y_;
    yaw = yaw_;
  };
};
}; // namespace internal
#endif