// nvcc -std=c++17 -o build/testcheckNodeExsist test/testcheckNodeExsist.cu
#include "../include/cuAStar.hpp"
#include <iostream>

int main() {
    using T = float;
    using NodeType = Node3d<T>;

    const size_t numNodes = 5;

    // Create an array of nodes on the host
    NodeType h_nodesArray[numNodes] = {
        NodeType(1.0f, 2.0f, 3.0f),
        NodeType(4.0f, 5.0f, 6.0f),
        NodeType(7.0f, 8.0f, 9.0f),
        NodeType(10.0f, 11.0f, 12.0f),
        NodeType(13.0f, 14.0f, 15.0f)
    };

    // The node to check
    NodeType h_nodeToCheck(4.0f, 5.0f, 6.0f);

    // Allocate memory on the device
    NodeType* d_nodesArray;
    NodeType* d_nodeToCheck;
    bool* d_status;
    bool h_status = false;

    cudaMalloc(&d_nodesArray, numNodes * sizeof(NodeType));
    cudaMalloc(&d_nodeToCheck, sizeof(NodeType));
    cudaMalloc(&d_status, sizeof(bool));

    // Copy data to device
    cudaMemcpy(d_nodesArray, h_nodesArray, numNodes * sizeof(NodeType), cudaMemcpyHostToDevice);
    cudaMemcpy(d_nodeToCheck, &h_nodeToCheck, sizeof(NodeType), cudaMemcpyHostToDevice);
    cudaMemcpy(d_status, &h_status, sizeof(bool), cudaMemcpyHostToDevice);

    // Launch the kernel with 1 block and numNodes threads
    checkNodeExsist<NodeType, T><<<1, numNodes>>>(d_nodesArray, d_nodeToCheck, d_status, numNodes);

    // Copy the result back to host
    cudaMemcpy(&h_status, d_status, sizeof(bool), cudaMemcpyDeviceToHost);

    // Print the result
    std::cout << "Node exists in the array: " << (h_status ? "True" : "False") << std::endl;

    // Free device memory
    cudaFree(d_nodesArray);
    cudaFree(d_nodeToCheck);
    cudaFree(d_status);

    return 0;
}