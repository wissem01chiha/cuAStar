
// nvcc -std=c++17 -o testNode2d test/testNode2d.cu
#include "../cpp/cuAStar.hpp"

// Kernel function to test Node2d methods on GPU
__global__ void testNode2dKernel(Node2d<float>* d_nodes, bool* results) {
    int tid = blockIdx.x * blockDim.x + threadIdx.x;

    if (tid == 0) {
        // Create some Node2d objects
        Node2d<float> node1(1.0f, 2.0f);
        Node2d<float> node2(1.0f, 2.0f);
        Node2d<float> node3(2.0f, 3.0f);

        // Test isEqual
        results[0] = node1.isEqual(node2); // Should be true
        results[1] = !node1.isEqual(node3); // Should be true

        // Test distanceTo
        float dist = node1.distanceTo(node3);
        results[2] = fabs(dist - sqrt(2.0f)) < 1e-6; // Should be true
    }
}

int main() {
    // Allocate device memory for test results
    bool* d_results;
    bool h_results[3] = {false, false, false};
    cudaMalloc((void**)&d_results, 3 * sizeof(bool));

    // Launch the kernel
    testNode2dKernel<<<1, 1>>>(nullptr, d_results);

    // Copy results back to host
    cudaMemcpy(h_results, d_results, 3 * sizeof(bool), cudaMemcpyDeviceToHost);

    // Print test results
    std::cout << "isEqual Test 1: " << (h_results[0] ? "Passed" : "Failed") << std::endl;
    std::cout << "isEqual Test 2: " << (h_results[1] ? "Passed" : "Failed") << std::endl;
    std::cout << "distanceTo Test: " << (h_results[2] ? "Passed" : "Failed") << std::endl;

    // Clean up
    cudaFree(d_results);

    return 0;
}