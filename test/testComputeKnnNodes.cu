#define STBI_MSC_SECURE_CRT
#define STB_IMAGE_WRITE_IMPLEMENTATION
#define STB_IMAGE_IMPLEMENTATION
#define CUASTAR_DEBUG
#define CUASTAR_IMPLEMENTATION
#include "cuAstar/cuAStar.hpp"

int main() {
    using T = float;
    using NodeType = Node3d<T>;

    int N = 1025;
    int k = 10;
    int range = 20;


    std::vector<NodeType> h_nodesX(N);
    std::vector<NodeType> h_nodesY(N);
    std::vector<NodeType> h_nodesZ(N);
    std::vector<NodeType> h_kNodes(k);  


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

    NodeType targetNode(0.5, 0.5, 0.5);

    NodeType *d_nodesX, *d_nodesY, *d_nodesZ, *d_kNodes;
    cudaMalloc((void**)&d_nodesX, N * sizeof(NodeType));
    cudaMalloc((void**)&d_nodesY, N * sizeof(NodeType));
    cudaMalloc((void**)&d_nodesZ, N * sizeof(NodeType));
    cudaMalloc((void**)&d_kNodes, k * sizeof(NodeType));


    cudaMemcpy(d_nodesX, h_nodesX.data(), N * sizeof(NodeType), cudaMemcpyHostToDevice);
    cudaMemcpy(d_nodesY, h_nodesY.data(), N * sizeof(NodeType), cudaMemcpyHostToDevice);
    cudaMemcpy(d_nodesZ, h_nodesZ.data(), N * sizeof(NodeType), cudaMemcpyHostToDevice);

    cudaEvent_t start, stop;
    cudaEventCreate(&start);
    cudaEventCreate(&stop);

    cudaEventRecord(start);
    int blocksNum = (N + threadsPerBlock - 1) / threadsPerBlock;

    computeKnnNodes<NodeType, T><<<blocksNum , N>>>(d_nodesX, d_nodesY, d_nodesZ, targetNode, N, k, range, d_kNodes);

   
    cudaEventRecord(stop);
    cudaEventSynchronize(stop);

    float milliseconds = 0;
    cudaEventElapsedTime(&milliseconds, start, stop);


    cudaMemcpy(h_kNodes.data(), d_kNodes, k * sizeof(NodeType), cudaMemcpyDeviceToHost);

    std::cout << "The k nearest nodes are:\n";
    for (int i = 0; i < k; ++i) {
        std::cout << "Node " << i + 1 << ": (" << h_kNodes[i].x << ", " << h_kNodes[i].y << ", " << h_kNodes[i].z << ")\n";
    }


    std::cout << "Total number of nodes: " << N << "\n";
    std::cout << "Kernel execution time: " << milliseconds << " ms\n";

    cudaFree(d_nodesX);
    cudaFree(d_nodesY);
    cudaFree(d_nodesZ);
    cudaFree(d_kNodes);

    cudaEventDestroy(start);
    cudaEventDestroy(stop);

    return 0;
}
