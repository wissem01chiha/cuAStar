#define _DEBUG_
// nvcc -std=c++17 -o build/testSortNodes test/testSortNodes.cu
#include <cuda_runtime.h>
#include "../cpp/cuAStar.hpp"

template <typename T>
void printNodes(Node3d<T>* nodes, int N) {
    for (int i = 0; i < N; ++i) {
        std::cout << "Node " << i << ": ( " << nodes[i].x << ", " << nodes[i].y << ", " << nodes[i].z << ")\n";
    }
}

// Check for CUDA errors
#define CUDA_CHECK(call) { \
    cudaError_t err = call; \
    if (err != cudaSuccess) { \
        std::cerr << "CUDA error in file '" << __FILE__ << "' in line " << __LINE__ << ": " << cudaGetErrorString(err) << std::endl; \
        exit(EXIT_FAILURE); \
    } \
}

int main() {
    const int N = 2024;  // Number of nodes
    const int axis = 1;  // Axis to sort by (1: x, 2: y, 3: z)

    // Allocate and initialize nodes on host
    Node3d<float> h_nodes[N];
    for (int i = 0; i < N; ++i) {
        h_nodes[i] = Node3d<float>(rand() / (float)RAND_MAX, rand() / (float)RAND_MAX, rand() / (float)RAND_MAX);
    }

    // Allocate device memory
    Node3d<float> *d_nodesArray, *d_nodesArraySorted;
    CUDA_CHECK(cudaMalloc((void**)&d_nodesArray, N * sizeof(Node3d<float>)));
    CUDA_CHECK(cudaMalloc((void**)&d_nodesArraySorted, N * sizeof(Node3d<float>)));

    // Copy data to device
    CUDA_CHECK(cudaMemcpy(d_nodesArray, h_nodes, N * sizeof(Node3d<float>), cudaMemcpyHostToDevice));

    // Create CUDA events for timing
    cudaEvent_t start, stop;
    CUDA_CHECK(cudaEventCreate(&start));
    CUDA_CHECK(cudaEventCreate(&stop));

    // Launch kernel to sort nodes by the specified axis
    int blocks = (N + threadsPerBlock - 1) / threadsPerBlock;

    // Start recording the event
    CUDA_CHECK(cudaEventRecord(start, 0));

    enumerationSortNodes<Node3d<float>, float><<<blocks, threadsPerBlock>>>(d_nodesArray, N, axis, d_nodesArraySorted);

    // Check for kernel launch errors
    CUDA_CHECK(cudaPeekAtLastError());

    // Stop recording the event
    CUDA_CHECK(cudaEventRecord(stop, 0));

    // Wait for the event to complete
    CUDA_CHECK(cudaEventSynchronize(stop));

    // Calculate the elapsed time
    float milliseconds = 0;
    CUDA_CHECK(cudaEventElapsedTime(&milliseconds, start, stop));

  

    // Copy sorted data back to host
    CUDA_CHECK(cudaMemcpy(h_nodes, d_nodesArraySorted, N * sizeof(Node3d<float>), cudaMemcpyDeviceToHost));

    // Print sorted nodes
    std::cout << "Sorted Nodes:\n";
    printNodes(h_nodes, N);

    // Free device memory
    CUDA_CHECK(cudaFree(d_nodesArray));
    CUDA_CHECK(cudaFree(d_nodesArraySorted));
    // Print the execution time in milliseconds
    std::cout << "Kernel execution time: " << milliseconds << " ms" << std::endl;
    // Destroy CUDA events
    CUDA_CHECK(cudaEventDestroy(start));
    CUDA_CHECK(cudaEventDestroy(stop));

    return 0;
}
