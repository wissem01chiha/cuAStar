
// nvcc -std=c++17 -o build/testsuccesor test/testsuccesor.cu
#define CUASTAR_DEBUG
#include "../include/cuAStar.hpp"
#include <iostream>


 

// Kernel and computeNodeTrajectoryCost function as previously defined...

int main() {
    using T = float; // Set the data type for Node3d (float in this case)

    // Define the number of k-nearest neighbors
    const int k = 8;

    // Initialize host data
    std::vector<Node3d<T>> h_knnNodesArray(k);
    for (int i = 0; i < k; ++i) {
        h_knnNodesArray[i] = Node3d<T>(static_cast<T>(i + 1), static_cast<T>(i + 2), static_cast<T>(i + 3));
    }

    Node3d<T> h_bestNode;
    Node3d<T> h_endNode = Node3d<T>(static_cast<T>(5.5), static_cast<T>(2.5), static_cast<T>(3));

    // Allocate device memory
    Node3d<T>* d_knnNodesArray;
    Node3d<T>* d_bestNode;
    Node3d<T>* d_endNode;

    cudaMalloc(&d_knnNodesArray, k * sizeof(Node3d<T>));
    cudaMalloc(&d_bestNode, sizeof(Node3d<T>));
    cudaMalloc(&d_endNode, sizeof(Node3d<T>)); // Allocate memory for d_endNode

    // Copy data to device
    cudaMemcpy(d_knnNodesArray, h_knnNodesArray.data(), k * sizeof(Node3d<T>), cudaMemcpyHostToDevice);
    cudaMemcpy(d_endNode, &h_endNode, sizeof(Node3d<T>), cudaMemcpyHostToDevice); // Corrected cudaMemcpy

    // Launch kernel (using one block with k threads and shared memory)
    int threadsPerBlock = k; // Set the number of threads per block
    computeSucessorNode<Node3d<float>, float><<<1, threadsPerBlock>>>(d_knnNodesArray, k, d_endNode, d_bestNode);

    // Copy result back to host
    cudaMemcpy(&h_bestNode, d_bestNode, sizeof(Node3d<T>), cudaMemcpyDeviceToHost);

    // Print all nodes
    std::cout << "All nodes:" << std::endl;
    for (const auto& node : h_knnNodesArray) {
        node.printNodeInfo();
    }
    std::cout << "end nodes:" << std::endl;
    h_endNode.printNodeInfo();

    // Print the best node
    std::cout << "The best node is: (" 
              << h_bestNode.x << ", " 
              << h_bestNode.y << ", " 
              << h_bestNode.z << ")" << std::endl;

    // Free device memory
    cudaFree(d_knnNodesArray);
    cudaFree(d_bestNode);
    cudaFree(d_endNode); // Free the allocated memory for d_endNode

    return 0;
}