/**
 * @file test_math.cu 
 */
#define ENABLE_CUDA_ARCH 1
#include "../include/math.hpp"
#include <iostream>

__global__ void ker_vec_diff(const double* d_input,double* d_output,int32_t n){
    vec_diff(d_input, d_output, n);
    __syncthreads();
};

__global__ void ker_quadratic_interpolation(double* result_array,double* x, 
                                      double* y){
    quadratic_interpolation(result_array,x,y);
    __syncthreads();
};

__global__ void ker_cum_sum(const double* input, double* output, int32_t n){
    cum_sum(input, output,n);
    __syncthreads();
};

__global__ void interp_refer(const double* params, double x, double* result){
  *result =  params[0] * x * x + params[1] * x + params[2];
};


int main() {
    const int32_t n = 4;
    double h_input[n] = {1.0, 2.0, 3.0, 4.0};
    double h_output[n] = {2};

    double* d_input;
    cudaMalloc(&d_input, n * sizeof(double));

    double* d_output;
    cudaMalloc(&d_output, n * sizeof(double));

    cudaMemcpy(d_input, h_input, n * sizeof(double), cudaMemcpyHostToDevice);

    int threadsPerBlock = 3;
    int blocksPerGrid = (n + threadsPerBlock - 1) / threadsPerBlock;

    ker_vec_diff<<<blocksPerGrid, threadsPerBlock>>>(d_input, d_output, n);
    cudaDeviceSynchronize();

    cudaMemcpy(h_output, d_output, n * sizeof(double), cudaMemcpyDeviceToHost);

    std::cout << "Input values: ";
    for (int i = 0; i < n; i++) {
        std::cout << h_input[i] << " ";
    }
    std::cout << std::endl;

    std::cout << "Output values: ";
    for (int i = 0; i < n; i++) {
        std::cout << h_output[i] << " ";
    }
    std::cout << std::endl;

    double expected_output[n] = {0.0, 1.0, 1.0, 1.0};
    for (int i = 1; i < n; i++) {
        if (h_output[i] == expected_output[i]) {
            std::cout << "Value at index " << i << " is correct: " << h_output[i] << std::endl;
        } else {
            std::cout << "Value at index " << i << " is incorrect: " << h_output[i] << std::endl;
        }
    }
    cudaFree(d_input);
    cudaFree(d_output);
    return 0;
};
