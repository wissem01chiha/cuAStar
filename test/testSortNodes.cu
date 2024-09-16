#define CUASTAR_DEBUG
#include <cuda_runtime.h>
#include "../include/cuAStar.hpp"

template <typename T>
void printNodes(Node3d<T>* nodes, int N) {
    for (int i = 0; i < N; ++i) {
        std::cout << "Node " << i << ": ( " << nodes[i].x << ", " << nodes[i].y << ", " << nodes[i].z << ")\n";
    }
}

#define CUDA_CHECK(call) { \
    cudaError_t err = call; \
    if (err != cudaSuccess) { \
        std::cerr << "CUDA error in file '" << __FILE__ << "' in line " << __LINE__ << ": " << cudaGetErrorString(err) << std::endl; \
        exit(EXIT_FAILURE); \
    } \
}

int main() {
    const int N = 2024;  
    const int axis = 1;  


    Node3d<float> h_nodes[N];
    for (int i = 0; i < N; ++i) {
        h_nodes[i] = Node3d<float>(rand() / (float)RAND_MAX, rand() / (float)RAND_MAX, rand() / (float)RAND_MAX);
    }

    Node3d<float> *d_nodesArray, *d_nodesArraySorted;
    CUDA_CHECK(cudaMalloc((void**)&d_nodesArray, N * sizeof(Node3d<float>)));
    CUDA_CHECK(cudaMalloc((void**)&d_nodesArraySorted, N * sizeof(Node3d<float>)));

    CUDA_CHECK(cudaMemcpy(d_nodesArray, h_nodes, N * sizeof(Node3d<float>), cudaMemcpyHostToDevice));

    cudaEvent_t start, stop;
    CUDA_CHECK(cudaEventCreate(&start));
    CUDA_CHECK(cudaEventCreate(&stop));

    int blocks = (N + threadsPerBlock - 1) / threadsPerBlock;


    CUDA_CHECK(cudaEventRecord(start, 0));

    enumerationSortNodes<Node3d<float>, float><<<blocks, threadsPerBlock>>>(d_nodesArray, N, axis, d_nodesArraySorted);


    CUDA_CHECK(cudaPeekAtLastError());

    CUDA_CHECK(cudaEventRecord(stop, 0));

    CUDA_CHECK(cudaEventSynchronize(stop));

    float milliseconds = 0;
    CUDA_CHECK(cudaEventElapsedTime(&milliseconds, start, stop));


    CUDA_CHECK(cudaMemcpy(h_nodes, d_nodesArraySorted, N * sizeof(Node3d<float>), cudaMemcpyDeviceToHost));

    std::cout << "Sorted Nodes:\n";
    printNodes(h_nodes, N);

    CUDA_CHECK(cudaFree(d_nodesArray));
    CUDA_CHECK(cudaFree(d_nodesArraySorted));

    std::cout << "Kernel execution time: " << milliseconds << " ms" << std::endl;

    CUDA_CHECK(cudaEventDestroy(start));
    CUDA_CHECK(cudaEventDestroy(stop));

    return 0;
}
