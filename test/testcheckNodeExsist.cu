#include "../include/cuAStar.hpp"
#include <iostream>

const size_t numNodes = 2043; 

int main() {
    using T = float;
    using NodeType = Node3d<T>;

    NodeType* h_nodesArray = new NodeType[numNodes];
    for (size_t i = 0; i < numNodes; ++i) {
        h_nodesArray[i] = NodeType(i * 1.0f, i * 2.0f, i * 3.0f);
    }

    NodeType h_nodeToCheck(1.0f, 2.0f, 3.0f); 

    NodeType* d_nodesArray;
    NodeType* d_nodeToCheck;
    int* d_status;
    int h_status = 0; 

    cudaMalloc(&d_nodesArray, numNodes * sizeof(NodeType));
    cudaMalloc(&d_nodeToCheck, sizeof(NodeType));
    cudaMalloc(&d_status, sizeof(int));

    cudaMemcpy(d_nodesArray, h_nodesArray, numNodes * sizeof(NodeType), cudaMemcpyHostToDevice);
    cudaMemcpy(d_nodeToCheck, &h_nodeToCheck, sizeof(NodeType), cudaMemcpyHostToDevice);
    cudaMemcpy(d_status, &h_status, sizeof(int), cudaMemcpyHostToDevice);

    int blocksPerGrid = (numNodes + threadsPerBlock - 1) / threadsPerBlock;

    checkNodeExsist<NodeType, T><<<blocksPerGrid, threadsPerBlock>>>(d_nodesArray, numNodes, d_nodeToCheck, d_status );


    cudaMemcpy(&h_status, d_status, sizeof(int), cudaMemcpyDeviceToHost);

    std::cout << "Node exists in the array: " << (h_status ? "True" : "False") << std::endl;

    cudaFree(d_nodesArray);
    cudaFree(d_nodeToCheck);
    cudaFree(d_status);
    delete[] h_nodesArray; 

    return 0;
}
