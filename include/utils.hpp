/**
 * utilites functions and routines 
 * 
 */
#include <cstdint>
#include <cuda_runtime.h>
#include <cublas_v2.h>

#include "common.hpp"

namespace internal{

  __device__ const Node motionModel2d[] = {
    Node(1, 0, 1),
    Node(0, 1, 1),
    Node(-1, 0, 1),
    Node(0, -1, 1),
    Node(-1, -1, sqrt(2)),
    Node(-1, 1, sqrt(2)),
    Node(1, -1, sqrt(2)),
    Node(1, 1, sqrt(2))
};

  __device__ const Node3d motionModel3d[] = {

    Node3d(1, 0, 0, 1), Node3d(-1, 0, 0, 1),
    Node3d(0, 1, 0, 1), Node3d(0, -1, 0, 1),
    Node3d(0, 0, 1, 1), Node3d(0, 0, -1, 1),
    
    Node3d(1, 1, 0, sqrt(2)), Node3d(1, -1, 0, sqrt(2)),
    Node3d(-1, 1, 0, sqrt(2)), Node3d(-1, -1, 0, sqrt(2)),
    Node3d(1, 0, 1, sqrt(2)), Node3d(1, 0, -1, sqrt(2)),
    Node3d(-1, 0, 1, sqrt(2)), Node3d(-1, 0, -1, sqrt(2)),
    Node3d(0, 1, 1, sqrt(2)), Node3d(0, 1, -1, sqrt(2)),
    Node3d(0, -1, 1, sqrt(2)), Node3d(0, -1, -1, sqrt(2)),

    Node3d(1, 1, 1, sqrt(3)), Node3d(1, 1, -1, sqrt(3)),
    Node3d(1, -1, 1, sqrt(3)), Node3d(1, -1, -1, sqrt(3)),
    Node3d(-1, 1, 1, sqrt(3)), Node3d(-1, 1, -1, sqrt(3)),
    Node3d(-1, -1, 1, sqrt(3)), Node3d(-1, -1, -1, sqrt(3))
};

  __device__ int bisect(double t, int start, int end){
    int mid = (start+end)/2;
    if (t==x[mid] || end-start<=1){
      return mid;
    }else if (t>x[mid]){
      return bisect(t, mid, end);
    }else{
      return bisect(t, start, mid);
    }
  }
  };