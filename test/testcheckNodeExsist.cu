// nvcc -std=c++17 -o build/testcheckNodeExsist test/testcheckNodeExsist.cu
#include "../include/cuAStar.hpp"
#include <iostream>

const size_t numNodes = 2043; // Increased number of nodes for testing

int main() {
    using T = float;
    using NodeType = Node3d<T>;

    // Create an array of nodes on the host with more nodes
    NodeType* h_nodesArray = new NodeType[numNodes];
    for (size_t i = 0; i < numNodes; ++i) {
        h_nodesArray[i] = NodeType(i * 1.0f, i * 2.0f, i * 3.0f);
    }

    // The node to check
    NodeType h_nodeToCheck(1.0f, 2.0f, 3.0f); // Modify as needed

    // Allocate memory on the device
    NodeType* d_nodesArray;
    NodeType* d_nodeToCheck;
    int* d_status;
    int h_status = 0; // Using 0 for false and 1 for true

    cudaMalloc(&d_nodesArray, numNodes * sizeof(NodeType));
    cudaMalloc(&d_nodeToCheck, sizeof(NodeType));
    cudaMalloc(&d_status, sizeof(int));

    // Copy data to device
    cudaMemcpy(d_nodesArray, h_nodesArray, numNodes * sizeof(NodeType), cudaMemcpyHostToDevice);
    cudaMemcpy(d_nodeToCheck, &h_nodeToCheck, sizeof(NodeType), cudaMemcpyHostToDevice);
    cudaMemcpy(d_status, &h_status, sizeof(int), cudaMemcpyHostToDevice);

    // Define block and grid sizes
    //int threadsPerBlock = 256; // Adjust as needed
    int blocksPerGrid = (numNodes + threadsPerBlock - 1) / threadsPerBlock;

    // Launch the kernel
    checkNodeExsist<NodeType, T><<<blocksPerGrid, threadsPerBlock>>>(d_nodesArray, numNodes, d_nodeToCheck, d_status );

    // Copy the result back to host
    cudaMemcpy(&h_status, d_status, sizeof(int), cudaMemcpyDeviceToHost);

    // Print the result
    std::cout << "Node exists in the array: " << (h_status ? "True" : "False") << std::endl;

    // Free device memory
    cudaFree(d_nodesArray);
    cudaFree(d_nodeToCheck);
    cudaFree(d_status);
    delete[] h_nodesArray; // Free host memory

    return 0;
}
