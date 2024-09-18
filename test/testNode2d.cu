#define STBI_MSC_SECURE_CRT
#define STB_IMAGE_WRITE_IMPLEMENTATION
#define STB_IMAGE_IMPLEMENTATION
#define CUASTAR_DEBUG
#define CUASTAR_IMPLEMENTATION
#include "cuAstar/cuAStar.hpp"


__global__ void testNode2dKernel(Node2d<float>* d_nodes, bool* results) {
    int tid = blockIdx.x * blockDim.x + threadIdx.x;

    if (tid == 0) {
     
        Node2d<float> node1(1.0, 2.0);
        Node2d<float> node2(1.0f, 2.0f);
        Node2d<float> node3(2.0f, 3.0f);

     
        results[0] = node1.isEqual(node2); 
        results[1] = !node1.isEqual(node3); 

  
        float dist = node1.distanceTo(node3);
        results[2] = fabs(dist - sqrt(2.0f)) < 1e-6; 
    }
}

int main() {

    bool* d_results;
    bool h_results[3] = {false, false, false};
    cudaMalloc((void**)&d_results, 3 * sizeof(bool));

    testNode2dKernel<<<1, 1>>>(nullptr, d_results);

    cudaMemcpy(h_results, d_results, 3 * sizeof(bool), cudaMemcpyDeviceToHost);

    std::cout << "isEqual Test 1: " << (h_results[0] ? "Passed" : "Failed") << std::endl;
    std::cout << "isEqual Test 2: " << (h_results[1] ? "Passed" : "Failed") << std::endl;
    std::cout << "distanceTo Test: " << (h_results[2] ? "Passed" : "Failed") << std::endl;


    cudaFree(d_results);

    return 0;
}