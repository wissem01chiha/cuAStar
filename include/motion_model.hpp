#ifndef MOTION_MODEL_HPP
#define MOTION_MODEL_HPP

#include <cstdint>
#ifdef _WIN32
  #include <windows.h>
#endif
#if defined(__CUDACC__)
    #define HOST_FUN __host__
    #define DEVICE_FUN __device__
    #define HOST_DEVICE_FUN __host__ __device__
    #define GLOBAL_FUN __global__
    #include <cuda_runtime.h>
    #include <cublas_v2.h>
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

#include "utils.hpp"
#include "common.hpp"
#include "math.hpp"

namespace internal {

 /** @brief */
 HOST_DEVICE_FUN void compute2dMotionModel(const int32_t* step, 
                                            Node2d* model){
    model[0] = Node2d(*step, 0, *step);
    model[1] = Node2d(0, *step, *step);
    model[2] = Node2d(-*step, 0, *step);
    model[3] = Node2d(0, -*step, *step);
    model[4] = Node2d(-*step, -*step,*step *sqrt(2));
    model[5] = Node2d(-*step, *step, *step * sqrt(2));
    model[6] = Node2d(*step, -*step, *step * sqrt(2));
    model[7] = Node2d(*step, *step, *step * sqrt(2));
 };

  /** @brief  */
  HOST_DEVICE_FUN void compute3dMotionModel(const int32_t* step, 
                                              Node3d* model){

      model[0] = Node3d(1 * (*step), 0 * (*step), 0 * (*step), 1);
      model[1] = Node3d(-1 * (*step), 0 * (*step), 0 * (*step), 1);
      model[2] = Node3d(0 * (*step), 1 * (*step), 0 * (*step), 1);
      model[3] = Node3d(0 * (*step), 0 * (*step), -1 * (*step), 1);
      model[4] = Node3d(1 * (*step), 1 * (*step), 0 * (*step), sqrt(2));
      model[5] = Node3d(1 * (*step), -1 * (*step), 0 * (*step), sqrt(2));
      model[6] = Node3d(-1 * (*step), 1 * (*step), 0 * (*step), sqrt(2));
      model[7] = Node3d(-1 * (*step), -1 * (*step), 0 * (*step), sqrt(2));
      model[8] = Node3d(1 * (*step), 0 * (*step), 1 * (*step), sqrt(2));
      model[9] = Node3d(1 * (*step), 0 * (*step), -1 * (*step), sqrt(2));
      model[10] = Node3d(-1 * (*step), 0 * (*step), 1 * (*step), sqrt(2));
      model[11] = Node3d(-1 * (*step), 0 * (*step), -1 * (*step), sqrt(2));
      model[12] = Node3d(0 * (*step), 1 * (*step), 1 * (*step), sqrt(2));
      model[13] = Node3d(0 * (*step), 1 * (*step), -1 * (*step), sqrt(2));
      model[14] = Node3d(0 * (*step), -1 * (*step), 1 * (*step), sqrt(2));
      model[15] = Node3d(0 * (*step), -1 * (*step), -1 * (*step), sqrt(2));
      model[16] = Node3d(1 * (*step), 1 * (*step), 1 * (*step), sqrt(3));
      model[17] = Node3d(1 * (*step), 1 * (*step), -1 * (*step), sqrt(3));
      model[18] = Node3d(1 * (*step), -1 * (*step), 1 * (*step), sqrt(3));  
      model[19] = Node3d(1 * (*step), -1 * (*step), -1 * (*step), sqrt(3));  
      model[20] = Node3d(-1 * (*step), 1 * (*step), 1 * (*step), sqrt(3));
      model[21] = Node3d(-1 * (*step), 1 * (*step), -1 * (*step), sqrt(3));
      model[22] = Node3d(-1 * (*step), -1 * (*step), 1 * (*step), sqrt(3));
      model[23] = Node3d(-1 * (*step), -1 * (*step), -1 * (*step), sqrt(3)); 
  };









struct Parameter{
  float distance;
  std::array<float, 3> steering_sequence{{0,0,0}};
  Parameter(float distance_, std::array<float, 3> steering_sequence_){
    distance = distance_;
    steering_sequence = steering_sequence_;
  };
};


using Traj = std::vector<internal ::TrajectoryState>; // this is for 2d 
using StateList = std::vector<internal ::TrajectoryState>;
using ParameterList = std::vector<Parameter>;

 
class MotionModel{
public:
  const float base_l;
  const float ds;
  State2d     state;

  HOST_DEVICE_FUN MotionModel(float base_l_, float ds_, State2d state_):
    base_l(base_l_), ds(ds_), state(state_){};
  __device__ void update(float v_, float delta, float dt);
  __device__ State2d update(internal::State2d state_, float delta, float dt);
  __device__ Traj generate_trajectory(Parameter);
  __device__ internal ::TrajectoryState  generate_last_state(Parameter);
  
protected:
  HOST_DEVICE_FUN ~MotionModel();
}; // class MotionModel
}; //namespace internal
#endif