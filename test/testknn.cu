
// nvcc -std=c++17 -o build/testknn test/testknn.cu
#include "../include/cuAStar.hpp"
#include <iostream>
#include <vector>
#include <cassert>
#include <cuda_runtime.h>

int main() {
    using T = float;
    using NodeType = Node3d<T>;

    // Number of nodes and the number of nearest neighbors to find
    int N = 1024;
    int k = 7;
    int range = 20;

    // Allocate host memory for nodes
    std::vector<NodeType> h_nodesX(N);
    std::vector<NodeType> h_nodesY(N);
    std::vector<NodeType> h_nodesZ(N);
    std::vector<NodeType> h_kNodes(k);  // To store the k nearest nodes

    // Initialize nodes with some random values or specific values for testing
    for (int i = 0; i < N; ++i) {
        h_nodesX[i] = NodeType(static_cast<T>(rand()) / RAND_MAX,
                               static_cast<T>(rand()) / RAND_MAX,
                               static_cast<T>(rand()) / RAND_MAX);

        h_nodesY[i] = NodeType(static_cast<T>(rand()) / RAND_MAX,
                               static_cast<T>(rand()) / RAND_MAX,
                               static_cast<T>(rand()) / RAND_MAX);

        h_nodesZ[i] = NodeType(static_cast<T>(rand()) / RAND_MAX,
                               static_cast<T>(rand()) / RAND_MAX,
                               static_cast<T>(rand()) / RAND_MAX);
    }

    // Define a target node
    NodeType targetNode(static_cast<T>(0.5), static_cast<T>(0.5), static_cast<T>(0.5));

    // Allocate device memory
    NodeType *d_nodesX, *d_nodesY, *d_nodesZ, *d_kNodes;
    cudaMalloc((void**)&d_nodesX, N * sizeof(NodeType));
    cudaMalloc((void**)&d_nodesY, N * sizeof(NodeType));
    cudaMalloc((void**)&d_nodesZ, N * sizeof(NodeType));
    cudaMalloc((void**)&d_kNodes, k * sizeof(NodeType));

    // Copy data to device
    cudaMemcpy(d_nodesX, h_nodesX.data(), N * sizeof(NodeType), cudaMemcpyHostToDevice);
    cudaMemcpy(d_nodesY, h_nodesY.data(), N * sizeof(NodeType), cudaMemcpyHostToDevice);
    cudaMemcpy(d_nodesZ, h_nodesZ.data(), N * sizeof(NodeType), cudaMemcpyHostToDevice);

    // CUDA events for timing
    cudaEvent_t start, stop;
    cudaEventCreate(&start);
    cudaEventCreate(&stop);

    // Start recording
    cudaEventRecord(start);

    // Launch the kernel (1 block with N threads, assuming N <= 1024)
    computeChunKnnNodes<NodeType, T><<<1, N>>>(d_nodesX, d_nodesY, d_nodesZ, targetNode, N, k, range, d_kNodes);

    // Stop recording
    cudaEventRecord(stop);
    cudaEventSynchronize(stop);

    // Calculate elapsed time
    float milliseconds = 0;
    cudaEventElapsedTime(&milliseconds, start, stop);

    // Copy the result back to host
    cudaMemcpy(h_kNodes.data(), d_kNodes, k * sizeof(NodeType), cudaMemcpyDeviceToHost);

    // Validate the results or print them out for visual inspection
    std::cout << "The k nearest nodes are:\n";
    for (int i = 0; i < k; ++i) {
        std::cout << "Node " << i + 1 << ": (" << h_kNodes[i].x << ", " << h_kNodes[i].y << ", " << h_kNodes[i].z << ")\n";
    }

    // Print the total number of nodes and kernel execution time
    std::cout << "Total number of nodes: " << N << "\n";
    std::cout << "Kernel execution time: " << milliseconds << " ms\n";

    // Free device memory
    cudaFree(d_nodesX);
    cudaFree(d_nodesY);
    cudaFree(d_nodesZ);
    cudaFree(d_kNodes);

    // Destroy CUDA events
    cudaEventDestroy(start);
    cudaEventDestroy(stop);

    return 0;
}
