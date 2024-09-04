#include "../include/cuAStar.hpp"
#include <iostream>
#include <cstdlib> 
#include <cuda_runtime.h>


int main() {
    srand(1000);
    const int N = 6540;  // Number of nodes
    const int dim = 3;   // Dimensionality of the KD-tree (3 for 3D)
    const int threadsPerBlock = 1024;

    // Allocate and initialize host memory for nodes
    Node3d<float>* h_chunkArray = new Node3d<float>[N];
    Node3d<float>* h_treeArray = new Node3d<float>[N];
    for (int i = 0; i < N; ++i) {
        h_chunkArray[i] = Node3d<float>(rand(), rand(), rand() );
    }

    // Allocate device memory
    Node3d<float>* d_chunkArray;
    Node3d<float>* d_treeArray;
    cudaMalloc((void**)&d_chunkArray, N * sizeof(Node3d<float>));
    cudaMalloc((void**)&d_treeArray, N * sizeof(Node3d<float>));

    // Copy data to device
    cudaMemcpy(d_chunkArray, h_chunkArray, N * sizeof(Node3d<float>), cudaMemcpyHostToDevice);

    // Launch the buildChunKDTree kernel
    int blocks = (N + threadsPerBlock - 1) / threadsPerBlock;
    buildChunKDTree<Node3d<float>, float><<<1, threadsPerBlock>>>(d_chunkArray, N, dim, d_treeArray);

    // Copy the resulting tree back to host
    cudaMemcpy(h_treeArray, d_treeArray, N * sizeof(Node3d<float>), cudaMemcpyDeviceToHost);

    // Print out the parent-child relationships for the first few nodes
    std::cout << "Simple verification test: parent-child relationships\n";
    for (int i = 0; i < 20 && i < N; ++i) {
        std::cout << "Node " << i << ":\n";
        std::cout << "  Parent node: (" << h_treeArray[i].x << ", " << h_treeArray[i].y << ", " << h_treeArray[i].z << ")\n";
        if (h_treeArray[i].c_nodes[0] != nullptr) {
            std::cout << "  Left child: (" << h_treeArray[i].c_nodes[0]->x << ", " 
                      << h_treeArray[i].c_nodes[0]->y << ", " << h_treeArray[i].c_nodes[0]->z << ")\n";
          

        } else {
            std::cout << "  Left child: nullptr\n";
        }
        if (h_treeArray[i].c_nodes[1] != nullptr) {
            std::cout << "  Right child: (" << h_treeArray[i].c_nodes[1]->x << ", " 
                      << h_treeArray[i].c_nodes[1]->y << ", " << h_treeArray[i].c_nodes[1]->z << ")\n";
        } else {
            std::cout << "  Right child: nullptr\n";
        }
    }

    // Free device memory
    cudaFree(d_chunkArray);
    cudaFree(d_treeArray);
    
    // Free host memory
    delete[] h_chunkArray;
    delete[] h_treeArray;

    return 0;
}