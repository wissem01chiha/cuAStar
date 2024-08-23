/**
 * @file cubic_spline_planner.cu
 */

#include<iostream>
#include<sstream>
#include<string>
#include<vector>
#include<array>
#include<cmath>


#include "../include/cubic_spline_trajectory.hpp"

__device__ Spline::Spline(int32_t n ): nx(n){
  x = new double[n];
  y = new double[n];
  h = new double[n];
  a = new double[n];
  b = new double[n];
  c = new double[n];
  d = new double[n];
};

__device__ Spline::~Spline() {
        delete[] x;
        delete[] y;
        delete[] h;
        delete[] a;
        delete[] b;
        delete[] c;
        delete[] d;
};

  __global__ Spline::Spline(double* x_, double* y_, int32_t nx){
    x = x_; 
    y = y_;
   
    h(vec_diff(x_));
    a(y_)
    Eigen::MatrixXf A = calc_A();
    Eigen::VectorXf B = calc_B();
    Eigen::VectorXf c_eigen = A.colPivHouseholderQr().solve(B);
    double * c_pointer = c_eigen.data();
    c.assign(c_pointer, c_pointer+c_eigen.rows());

    for(int i=0; i<nx-1; i++){
      d.push_back((c[i+1]-c[i])/(3.0*h[i]));
      b.push_back((a[i+1] - a[i])/h[i] - h[i] * (c[i+1] + 2*c[i])/3.0);
    }
  };

    double Spline::calc(double t){
    if(t<x.front() || t>x.back()){
      throw std::invalid_argument( "received value out of the pre-defined range" );
    }
    int seg_id = bisect(t, 0, nx);
    double dx = t - x[seg_id];
    return a[seg_id] + b[seg_id] * dx + c[seg_id] * dx * dx + d[seg_id] * dx * dx * dx;
  };

  double Spline::calc_d(double t){
    if(t<x.front() || t>x.back()){
      throw std::invalid_argument( "received value out of the pre-defined range" );
    }
    int seg_id = bisect(t, 0, nx-1);
    double dx = t - x[seg_id];
    return b[seg_id]  + 2 * c[seg_id] * dx + 3 * d[seg_id] * dx * dx;
  }

  double Spline::calc_dd(double t){
    if(t<x.front() || t>x.back()){
      throw std::invalid_argument( "received value out of the pre-defined range" );
    }
    int seg_id = bisect(t, 0, nx);
    double dx = t - x[seg_id];
    return 2 * c[seg_id] + 6 * d[seg_id] * dx;
  }

  __global__ void Spline::computeMatrixA(double* d_A, const double* h, int32_t nx)
  {
    int idx = threadIdx.x;
    for (int i = idx; i < nx * nx; i += blockDim.x) {
        d_A[i] = 0.0;
    }
    if (idx == 0) {
        d_A[0] = 1.0;
        d_A[(nx - 1) * nx + (nx - 1)] = 1.0;
    }
    for (int i = idx; i < nx - 1; i += blockDim.x) {
        if (i != nx - 2) {
            d_A[(i + 1) * nx + (i + 1)] = 2.0 * (h[i] + h[i + 1]);
        }
        d_A[(i + 1) * nx + i] = h[i];
        d_A[i * nx + (i + 1)] = h[i];
    }
}

  __global__ void Spline::computeMatrixB(double* d_B, const double* h, int32_t nx)
  {
    int idx = threadIdx.x;
    for(int i =idx; i< nx*nx; i+=blockDim.x){
      d_B[i] = 0.0;
    }
    for (int i = idx; i < nx - 1; i += blockDim.x){
      if (i != nx - 2) {
        d_B[( i + 1 ) * nx + ( i + 1 )] = 3.0*(a[i+2]-a[i+1])/h[i+1]-3.0*(a[i+1]-a[i])/h[i];
      }
      d_B[(i + 1) * nx + i] = h[i];
      d_B[i * nx + (i + 1)] = h[i];
    }
  };

  int Spline::bisect(double t, int start, int end){
    int mid = (start+end)/2;
    if (t==x[mid] || end-start<=1){
      return mid;
    }else if (t>x[mid]){
      return bisect(t, mid, end);
    }else{
      return bisect(t, start, mid);
    }
  }

  Spline2d::Spline2d(Vec_f x, Vec_f y){
    s = calc_s(x, y);
    sx = Spline(s, x);
    sy = Spline(s, y);
  };

  Poi_f Spline2d::calc_postion(double s_t){
    double x = sx.calc(s_t);
    double y = sy.calc(s_t);
    return {{x, y}};
  };
  double calc_curvature(double s_t){
    double dx = sx.calc_d(s_t);
    double ddx = sx.calc_dd(s_t);
    double dy = sy.calc_d(s_t);
    double ddy = sy.calc_dd(s_t);
    return (ddy * dx - ddx * dy)/(dx * dx + dy * dy);
  };

  double Spline2d::calc_yaw(double s_t){
    double dx = sx.calc_d(s_t);
    double dy = sy.calc_d(s_t);
    return std::atan2(dy, dx);
  };

  Vec_f calc_s(Vec_f x, Vec_f y){
    Vec_f ds;
    Vec_f out_s{0};
    Vec_f dx = vec_diff(x);
    Vec_f dy = vec_diff(y);

    for(unsigned int i=0; i<dx.size(); i++){
      ds.push_back(std::sqrt(dx[i]*dx[i] + dy[i]*dy[i]));
    }

    Vec_f cum_ds = cum_sum(ds);
    out_s.insert(out_s.end(), cum_ds.begin(), cum_ds.end());
    return out_s;
  };
 