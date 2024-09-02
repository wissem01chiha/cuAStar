// nvcc -rdc=true -std=c++17 -o build/testDKtree test/testDKtree.cu
// ebale dynamic parallism -rdc=true , the device compute cacpbilty should be of 3.5 >

#define _DEBUG_
#define USE_CUASTAR_TYPEDEF
#include "../cpp/cuAStar.hpp"
#include <iostream>

template <typename T>
void printNodeInfo(const Node3d<T>* node, int index) {
    if (node != nullptr) {
        std::cout << "Node " << index << ": ( "
                  << node->x << ", " << node->y << ", " << node->z << ")\n";
    } else {
        std::cout << "Node " << index << ": nullptr\n";
    }
}


template <typename T>
void printParentChildRelations(Node3d<T>* nodes, int N) {
    std::cout << "Parent-Child Relationships:\n";
    
    for (int i = 0; i < std::min(6, N); ++i) { // Print first 6 nodes or less if fewer nodes exist
        std::cout << "Parent Node " << i << ":\n";
        printNodeInfo(&nodes[i], i);
        
        std::cout << "  Child Node 0: ";
        printNodeInfo(nodes[i].c_nodes[0], i); // Print child node info or nullptr

        std::cout << "  Child Node 1: ";
        printNodeInfo(nodes[i].c_nodes[1], i); // Print child node info or nullptr
    }
}


int main() {
    const int N = 1023;  // Number of nodes
    const int dim = 3;   // Dimension (x, y, z)
    
    // Allocate and initialize nodes on host
    Node3d<float> h_nodes[N];
    for (int i = 0; i < N; ++i) {
        h_nodes[i] = Node3d<float>(rand() / (float)RAND_MAX, rand() / (float)RAND_MAX, rand() / (float)RAND_MAX);
    }

    // Allocate device memory
    Node3d<float> *d_chunkArray, *d_treeArray;
    cudaMalloc((void**)&d_chunkArray, N * sizeof(Node3d<float>));
    cudaMalloc((void**)&d_treeArray, N * sizeof(Node3d<float>));

    // Copy data to device
    cudaMemcpy(d_chunkArray, h_nodes, N * sizeof(Node3d<float>), cudaMemcpyHostToDevice);
    
    // Launch kernel to build KD-tree iteratively
    const int threadsPerBlock = 256; // Adjust based on GPU capability
    const int blocks = (N + threadsPerBlock - 1) / threadsPerBlock;
    
    // Measure kernel execution time
    cudaEvent_t start, stop;
    cudaEventCreate(&start);
    cudaEventCreate(&stop);
    cudaEventRecord(start);

    buildChunKDTree<Node3d<float>, float><<<1, threadsPerBlock>>>(d_chunkArray, N, dim, d_treeArray);

    cudaEventRecord(stop);
    cudaEventSynchronize(stop);
    
    float elapsedTime;
    cudaEventElapsedTime(&elapsedTime, start, stop);
    std::cout << "Kernel execution time: " << elapsedTime << " ms\n";
    
    // Copy tree data back to host for verification
    Node3d<float> h_treeArray[N];
    cudaMemcpy(h_treeArray, d_treeArray, N * sizeof(Node3d<float>), cudaMemcpyDeviceToHost);
    
    // Print parent-child relationships for the first few nodes
    printParentChildRelations<float>(h_treeArray, N);

    // Free device memory
    cudaFree(d_chunkArray);
    cudaFree(d_treeArray);

    // Clean up CUDA events
    cudaEventDestroy(start);
    cudaEventDestroy(stop);

    return 0;
}