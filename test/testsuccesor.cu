#define CUASTAR_DEBUG
#include "../include/cuAStar.hpp"
#include <iostream>

int main() {
    using T = float; 

    const int k = 128;

    std::vector<Node3d<T>> h_knnNodesArray(k);
    for (int i = 0; i < k; ++i) {
        h_knnNodesArray[i] = Node3d<T>(static_cast<T>(i + 1), static_cast<T>(i + 2), static_cast<T>(i + 3));
    }

    Node3d<T> h_bestNode;
    Node3d<T> h_endNode = Node3d<T>(static_cast<T>(100), static_cast<T>(101), static_cast<T>(103));

    Node3d<T>* d_knnNodesArray;
    Node3d<T>* d_bestNode;
    Node3d<T>* d_endNode;

    cudaMalloc(&d_knnNodesArray, k * sizeof(Node3d<T>));
    cudaMalloc(&d_bestNode, sizeof(Node3d<T>));
    cudaMalloc(&d_endNode, sizeof(Node3d<T>)); 

    cudaMemcpy(d_knnNodesArray, h_knnNodesArray.data(), k * sizeof(Node3d<T>), cudaMemcpyHostToDevice);
    cudaMemcpy(d_endNode, &h_endNode, sizeof(Node3d<T>), cudaMemcpyHostToDevice); 

    int threadsPerBlock = k; 
    computeOptimalNode<Node3d<float>, float><<<1, threadsPerBlock>>>(d_knnNodesArray, k, d_endNode, d_bestNode);

    cudaMemcpy(&h_bestNode, d_bestNode, sizeof(Node3d<T>), cudaMemcpyDeviceToHost);

    std::cout << "All nodes:" << std::endl;
    for (const auto& node : h_knnNodesArray) {
        node.printNodeInfo();
    }
    std::cout << "end nodes:" << std::endl;
    h_endNode.printNodeInfo();

    std::cout << "The best node is: (" 
              << h_bestNode.x << ", " 
              << h_bestNode.y << ", " 
              << h_bestNode.z << ")" << std::endl;


    cudaFree(d_knnNodesArray);
    cudaFree(d_bestNode);
    cudaFree(d_endNode); 

    return 0;
}