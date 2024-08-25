/**
 * @file motion_model.cu
 */
#define ENABLE_CUDA_ARCH 1 
#include "../include/motion_model.hpp"

namespace internal{

__device__ void MotionModel::update(float v_, float delta, float dt){
  state.v = v_;
  state.x = state.x + state.v * std::cos(state.yaw) * dt;
  state.y = state.y + state.v * std::sin(state.yaw) * dt;
  state.yaw = state.yaw + state.v / base_l * std::tan(delta) * dt;
  state.yaw = YAW_P2P(state.yaw);
};

__device__ State2d MotionModel::update(State2d state_, float delta, float dt){
  state_.x = state_.x + state_.v * std::cos(state_.yaw) * dt;
  state_.y = state_.y + state_.v * std::sin(state_.yaw) * dt;
  state_.yaw = state_.yaw + state_.v / base_l * std::tan(delta) * dt;
  state_.yaw = YAW_P2P(state_.yaw);
  return state_;
};

Traj MotionModel::generate_trajectory(Parameter p){
  float n =  p.distance / ds;
  float horizon = p.distance / state.v;

  // boost::math::cubic_b_spline<float> spline(
  //   p.steering_sequence.data(), p.steering_sequence.size(),
  //   0, horizon/p.steering_sequence.size());
  std::vector<float> spline = quadratic_interpolation(
    {{0, horizon/2, horizon}},
    p.steering_sequence);

  Traj output;
  internal::State2d state_ = state;

  for(float i=0.0; i<horizon; i+=horizon/n){
    float kp = interp_refer(spline, i);
    state_ = update(state_, kp, horizon/n);
    TrajectoryState xyyaw{state_.x, state_.y, state_.yaw};
    output.push_back(xyyaw);
  }
  return output;
}

TrajectoryState MotionModel::generate_last_state(Parameter p){
  float n = p.distance / ds;
  float horizon = p.distance / state.v;

  // boost::math::cubic_b_spline<float> spline(
  //   p.steering_sequence.data(), p.steering_sequence.size(),
  //   0, horizon/p.steering_sequence.size());
  std::vector<float> spline = quadratic_interpolation(
    {{0, horizon/2, horizon}},
    p.steering_sequence);

  internal::State2d state_ = state;
  for(float i=0.0; i<horizon; i+=horizon/n){
      float kp = interp_refer(spline, i);
      state_ = update(state_, kp, horizon/n);
  }
  return TrajectoryState{state_.x, state_.y, state_.yaw};
}

}; //namespace internal