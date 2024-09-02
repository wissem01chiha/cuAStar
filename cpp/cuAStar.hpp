/**
 * 
 * dsign tip: public member class are hhost functions to allocate memory and 
 * other private are kernels 
 * Astar
 *   
 * cuAstar Librray
 * https://towardsdatascience.com/understanding-a-path-algorithms-and-implementation-with-python-4d8458d6ccc7
 * 
 * If there are no blocked cells/obstacles then we can just find the exact 
 * value of h without any pre-computation using the distance formula/Euclidean Distance
 * all memory errors checks foe cuda are in debug mode !
 * $ g++ -std=c++0x main.cpp -pthread
 * nvcc -o random_gen_example random_gen_example.cu -lcurand
 * divide the point cloud data into small region called bubles a N pt cloud is divided into 
 * N/k where k is the window size , for each subarea , inti a substarting node
 * explore (like running the normal A star on that subgroups nodes) 
 * with one setp the surouding and at each step , synchonize all subregions 
 * https://pantelis.github.io/artificial-intelligence/aiml-common/lectures/planning/search/a-star/index.html
 * 
 * for the multithreding and concurrency support computations support the c++ 11 standard is required 
 * see https://en.cppreference.com/w/cpp/thread
 */
#ifndef CUASTAR_HPP
#define CUASTAR_HPP

#define STB_IMAGE_WRITE_IMPLEMENTATION 

#if __cplusplus > 201703L
    #warning "C++ 17 Required, potential errors!"
#endif
#ifdef _MSC_VER
    #if _MSC_VER < 1910
        #warning "MSVC 2019 or Later is Officially Supported"
    #endif
#endif
#ifdef __GNUC__
	#pragma GCC system_header
#endif
#include <cstdint>
#include <cfloat>
#ifdef _WIN32
  #include <windows.h>
#endif

#include <array>
#include <chrono>
#include <ctime>
#include <string>
#include <stdexcept>
#include <algorithm> 
#include <filesystem>  

#if defined(__CUDACC__)
    #define HOST_FUN __host__
    #define DEVICE_FUN __device__
    #define HOST_DEVICE_FUN __host__ __device__
    #include <cuda_runtime.h>
    #include <curand_kernel.h>
    #include <math_constants.h> 
    #include <device_launch_parameters.h>
    #include <stack>
    #ifdef _DEBUG_
        #include <cstdio>
        #include <cstdlib>
    #endif
#else
    #define HOST_FUN 
    #define DEVICE_FUN 
    #define HOST_DEVICE_FUN
    #include <vector>
    #ifdef _OPENMP
        #include <omp.h>
    #endif
    #include <cmath>
    #ifdef _MSC_VER
        #include <corecrt_math_defines.h>
    #endif
    #ifdef ENABLE_THREADS && __cplusplus >= 201103L
        #include <thread>
    #endif
    #ifdef ENABLE_SSE && !defined(ENABLE_THREADS)
    #ifdef _MSC_VER
        #if defined(_M_IX86_FP) && _M_IX86_FP >= 1
            #include <intrin.h>
            #include <xmmintrin.h>
        #endif
    #else
        #include <xmmintrin.h>
    #endif
#endif
#endif

#ifdef _DEBUG_
    #if defined(__CUDACC__)
    #define CUDA_CHECK_ERROR(err) \
        if (err != cudaSuccess) { \
            LOG_F(ERROR, "CUDA error: %s", cudaGetErrorString(err)); \
            exit(EXIT_FAILURE); \
        }
    #endif
    #include <iostream>
    #include "../extern/loguru/loguru.hpp"
    #include "../extern/loguru/loguru.cpp"
#else
    #define CUDA_CHECK_ERROR(err)
#endif
#ifdef ENABLE_VTK
  #include <vtkActor.h>
  #include <vtkNamedColors.h>
  #include <vtkNew.h>
  #include <vtkPLYReader.h>
  #include <vtkSphereSource.h>
  #include <vtkPolyData.h>
  #include <vtkPolyDataMapper.h>
  #include <vtkProperty.h>
  #include <vtkRenderWindow.h>
  #include <vtkRenderWindowInteractor.h>
  #include <vtkRenderer.h>
  #include <vtkUnsignedCharArray.h>
  #include <vtkPointData.h>
  #include <vtkAppendPolyData.h>
  #include <vtkSmartPointer.h>
#endif

#include "../extern/rapidcsv/rapidcsv.h"
#include "../extern/happly/happly.h"
#include "../extern/stb/stb_image.h"
#include "../extern/stb/stb_image_write.h"

/** @brief Generates a random float between 0 and 1 with precision up to 1e-6. */
template <typename T>
__global__ void curandx(unsigned int seed, T* d_val) {

    int idx = threadIdx.x + blockIdx.x * blockDim.x;
    curandState state;
    curand_init(seed, idx, 0, &state);
    d_val[idx] = curand_uniform(&state);
};

/** 
 * @brief   Base class for 2D nodes representation
 * @tparam  T Numeric type for the sum_cost (e.g., T, float)
 * @param x : x-coordinate in the grid
 * @param y : y-coordinate in the grid
 * @param z = 0 : this coordinate is used only to ensure generic function call 
 *          that do not throw an error in using 3d nodes specific operations 
 *          to 2d nodes based template methods.
 * @param sum_cost : cumulative cost from the start node to the current node
 * @param p_node : pointer to the parent node
 * @param c_nodes array of the 8 nearest neighbors considered as childs nodes 
 */
template <typename T>
class Node2d {
public:
    T x;           
    T y;
    T z = static_cast<T>(0);         
    T sum_cost;   
    Node2d* p_node; 
    Node2d* c_nodes[2];

    uint8_t r = 125;
    uint8_t g = 100;
    uint8_t b = 120;      

    /** @brief default constructor for the Node2d class */
    HOST_DEVICE_FUN Node2d(){
        x= T(0);
        y= T(0);
        sum_cost = T(0);
        p_node = nullptr;
        c_nodes[0] = nullptr;
        c_nodes[1] = nullptr;
    }

    /** @brief Constructor for the Node2d class */
    HOST_DEVICE_FUN Node2d(T x_, T y_, T sum_cost_ = 0,Node2d* p_node_ = nullptr){
        x= x_;
        y= y_;
        sum_cost = sum_cost_;
        p_node = p_node_;
        c_nodes[0] = nullptr;
        c_nodes[1] = nullptr;
    }

    /** @brief Overlaoding Constructor for the general puropse template 
     * calls */
    HOST_DEVICE_FUN Node2d(T x_, T y_, T z_){
        x= x_;
        y= y_;
        sum_cost = T(0);
        p_node = nullptr;
        c_nodes[0] = nullptr;
        c_nodes[1] = nullptr;
    }

    /** @brief Checks if two nodes are equal based on their positions */
    HOST_DEVICE_FUN bool isEqual(const Node2d& other, T eps=static_cast<T>(1e-6)) const 
    {
        return fabs(other.x-x)<eps && fabs(other.y-y) < eps;
    }

    /** @brief Computes the Euclidean distance to another node */
    HOST_DEVICE_FUN T distanceTo(const Node2d& other) const {
        return sqrt(static_cast<T>((other.x - x) * (other.x - x) +
                                   (other.y - y) * (other.y - y)));
    }

    #ifdef _DEBUG_
        /** @brief Prints the information of this node */
        HOST_FUN void printNodeInfo() const {
            std::cout << "Node: (x: " << x << ", y: " << y 
                    << ", sum_cost: " << sum_cost << ", color: (" << static_cast<int>(r) 
                    << ", " << static_cast<int>(g) << ", " << static_cast<int>(b) << "))\n";
        }
    #endif
};

/** 
 * @brief Base class for 3D nodes representation
 * @tparam T Numeric type for the sum_cost (e.g., T, float)
 * @param x : x-coordinate in the grid
 * @param y : y-coordinate in the grid
 * @param z : z-coordinate in the grid
 * @param sum_cost : cumulative cost from the start node to the current node
 * @param p_node : pointer to the parent node
 * @param c_nodes array of the 8 nearest neighbors considered as childs nodes 
 */
template <typename T>
class Node3d {
public:
    T x;         
    T y;         
    T z;           
    T sum_cost;   
    Node3d* p_node;
    Node3d* c_nodes[2];  

    uint8_t r = 125;
    uint8_t g = 100;
    uint8_t b = 120;   

    /** @brief Default constructor for the Node3d class */
    HOST_DEVICE_FUN Node3d(){
        x = T(0);
        y = T(0);
        z = T(0);
        sum_cost = T(0);
        p_node = nullptr;
        c_nodes[0] = nullptr;
        c_nodes[1] = nullptr;
    };
    /** @brief init a random node, position bounds is in [0,1] */
    HOST_FUN Node3d(unsigned int  seed){

        T* d_x;
        T* d_y;
        T* d_z;
        cudaMalloc((void**)&d_x, sizeof(T));
        cudaMalloc((void**)&d_y, sizeof(T));
        cudaMalloc((void**)&d_z, sizeof(T));

        curandx<T><<<2, 1>>>(seed, d_x);
        curandx<T><<<2, 1>>>(seed, d_y);
        curandx<T><<<2, 1>>>(seed, d_z);
        cudaMemcpy(&x, d_x, sizeof(T), cudaMemcpyDeviceToHost);
        cudaMemcpy(&y, d_y, sizeof(T), cudaMemcpyDeviceToHost);
        cudaMemcpy(&z, d_z, sizeof(T), cudaMemcpyDeviceToHost);

        c_nodes[0] = nullptr;
        c_nodes[1] = nullptr;
        sum_cost = T(0);
        p_node = nullptr;
    };

    /** @brief Constructor for the Node3d class */
    HOST_DEVICE_FUN Node3d(T x_,T y_,T z_,T sum_cost_=0, Node3d* p_node_=nullptr){
        x = x_;
        y = y_;
        z = z_;
        sum_cost = sum_cost_;
        p_node = p_node_;
    };

    /** @brief Computes the Euclidean distance to another node */
    HOST_DEVICE_FUN T distanceTo(const Node3d& other) const {
        return sqrt(static_cast<T>((other.x - x) * (other.x - x) +
                                   (other.y - y) * (other.y - y) +
                                   (other.z - z) * (other.z - z)));
    }

    /** @brief Checks if two nodes are equal based on their positions */
    HOST_DEVICE_FUN bool isEqual(const Node3d& other,T eps=static_cast<T>(1e-6))const {
        return fabs(other.x - x) < eps &&
               fabs(other.y - y) < eps &&
               fabs(other.z - z) < eps;
    }

    #ifdef _DEBUG_
        /** @brief Prints the information of this node */
        HOST_FUN void printNodeInfo() const {
            std::cout << "Node: (x: " << x << ", y: " << y << ", z: " << z
                    << ", sum_cost: " << sum_cost << ", color: (" << static_cast<int>(r) 
                    << ", " << static_cast<int>(g) << ", " << static_cast<int>(b) << "))\n";
        }
    #endif
};

#ifdef USE_CUASTAR_TYPEDEF
    typedef Node2d<double> Node2dDouble;  
    typedef Node2d<float>  Node2dFloat;   
    typedef Node3d<double> Node3dDouble; 
    typedef Node3d<float>  Node3dFloat;  
#endif

typedef struct {
    unsigned char r;
    unsigned char g;
    unsigned char b;
} Color;

typedef struct {
    int x;
    int y;
    int radius;
} Circle;

/** @brief This is the default threads number, but you can choose up to 1024 */ 
const int threadsPerBlock = 256; 

#ifdef _DEBUG_
    /** @brief Debug function to print given 3d nodes array to user shell*/
    template <typename T>
    HOST_FUN void print3dNodes(Node3d<T>* nodes, int N) {
        for (int i = 0; i < N; ++i) {
            std::cout << "Node " << i << ": ( " << nodes[i].x << ", " 
                        << nodes[i].y << ", " << nodes[i].z << ")\n";
        }
    };

    /** @brief Debug function to print given 2d nodes array to user shell*/
    template <typename T>
    HOST_FUN void print2dNodes(Node2d<T>* nodes, int N) {
        for (int i = 0; i < N; ++i) {
            std::cout << "Node " << i << ": ( " << nodes[i].x << ", " 
                                                << nodes[i].y << ")\n";
        }
    };

    /** 
     * @brief Debug recursive function to print a nodes tree to user shell,
     * @note This function is not recommended to invoke with large trees data,
     * structures
     */
    template<typename NodeType>
    HOST_FUN void printTree(NodeType* node, int depth = 0) {
        if (!node) return;
        for (int i = 0; i < depth; ++i) std::cout << "  ";
        std::cout << "Node(" << node->x << ", " << node->y << ", " << node->z << ")\n";
        printTree(node->c_nodes[0], depth + 1); 
        printTree(node->c_nodes[1], depth + 1);  
    };    

#endif

/**
 * @brief CUDA kernel for sorting a numerical array using the enumeration sort algorithm.
 * Original implementation by: Meng Hongyu, Guo Fangjin, UCAS, China
 * https://arxiv.org/pdf/1505.07605
 * Modified by: Wissem Chiha, 01-09-2024, using templates.
 */
template <typename T>
__global__ void enumerationSort(T * a, int N,  T * b){

    int cnt = 0; 
    int tid = threadIdx.x; 
    int ttid = blockIdx.x * threadsPerBlock + tid; 
    T val = a[ttid]; 
    __shared__ T cache[threadsPerBlock]; 
    for ( int i = tid; i < N; i += threadsPerBlock ){ 
    cache[tid] = a[i]; 
    __syncthreads();   
    for ( int j = 0; j < threadsPerBlock; ++j ) 
    if ( val > cache[j] )             
    cnt++;            
    __syncthreads(); 
    } 
    b[cnt] = val;
};

/** 
 * @brief CUDA kernel that sorts an array of nodes based on a specified axis.
 * This kernel sorts the nodes in ascending order according to their value 
 * along the given axis (1: x, 2: y, 3: z).
 * @param d_nodesArray          Device pointer to the array of nodes to be sorted.
 * @param ax                    Axis to sort by (1: x, 2: y, 3: z).
 * @param N                     Number of nodes in the array.
 * @param d_nodesArraySorted    Device pointer to the array where sorted nodes will be stored.
 * @note the call of this kernel with ax = 3 and Node2d class has no effect, and it do
 * not throw any error
 */
template <typename NodeType, typename T>
__global__ void enumerationSortNodes(NodeType* d_nodesArray,int N,int ax,
                            NodeType *d_nodesArraySorted){

    int cnt = 0;
    int tid = threadIdx.x;
    int ttid = blockIdx.x * threadsPerBlock + tid; 
    NodeType val = d_nodesArray[ttid];
    __shared__ NodeType cache[threadsPerBlock];
    for ( int i = tid; i < N; i += threadsPerBlock ){ 
        cache[tid] = d_nodesArray[i]; 
        __syncthreads();   
        for ( int j = 0; j < threadsPerBlock; ++j ){ 
            if ( ax == 1 ){
                if ( val.x > cache[j].x )             
                cnt++;
            }else if ( ax == 2){
                if ( val.y > cache[j].y )             
                cnt++;
            }else if ( ax == 3 ){
                if ( val.z > cache[j].z )             
                cnt++;
            }
        }            
        __syncthreads(); 
    } 
    atomicExch(&d_nodesArraySorted[cnt], val);
}

/** @brief Update nearest neighbors list if the new node is closer  */ 
template <typename NodeType, typename T>
DEVICE_FUN void updateNearest(NodeType* nearestNodes, T* distances, 
                const NodeType& node, const NodeType& otherNode, int k) {

    T dist = node.distanceTO(otherNode);
    for (int i = 0; i < k; ++i) {
        if (dist < distances[i]) {
        
            for (int j = k - 1; j > i; --j) {
                distances[j] = distances[j - 1];
                nearestNodes[j] = nearestNodes[j - 1];
            }
            distances[i] = dist;
            nearestNodes[i] = otherNode;
            break;
        }
    }
}


/**
 * @brief Computes the K-nereast neighoods of a given node in a device 
 * array structure , by sorting x, y, z attributes, without building the K-D tree
 * this kernel should excute on 1 block, 
 * @param range: control the exploration range of the KNN, fixed , adjustee based 
 * on the point cloud distribution density  , low for high dense data and high for low 
 * dense distrubution
 * @param k should be < 256 
 */
template <typename NodeType, typename T>
__global__ void computeKnnNodes(NodeType* d_sortedX, 
                                NodeType* d_sortedY, 
                                NodeType* d_sortedZ,
                                NodeType targetNode, 
                                int N, int k, int range, 
                                NodeType* d_kNodes) {
    int idx = threadIdx.x;

    if (idx < N) {
        __shared__ NodeType nearestNodes[256];
        __shared__ T distances[256];

        T maxDist = 1e30f;   
        if (idx == 0) {
            for (int i = 0; i < k; ++i) {
                distances[i] = maxDist;
                nearestNodes[i] = targetNode;  
            }
        }
        __syncthreads();

        for (int i = max(0, idx - range); i < min(N, idx + range); ++i) {
            updateNearest(nearestNodes, distances, targetNode, d_sortedX[i], k);
        }
        for (int i = max(0, idx - range); i < min(N, idx + range); ++i) {
            updateNearest(nearestNodes, distances, targetNode, d_sortedY[i], k);
        }
        for (int i = max(0, idx - range); i < min(N, idx + range); ++i) {
            updateNearest(nearestNodes, distances, targetNode, d_sortedZ[i], k);
        }
        if (idx == 0) {  
            for (int i = 0; i < k; ++i) {
                d_kNodes[i] = nearestNodes[i];
            }
        }
    }
}

/**  
 * @brief check if a given node exsits in device nodes array or not
 * @note this method will be decpatred in next versions 
*/
template <typename NodeType, typename T>
__global__ void checkNodeExsist(NodeType* d_nodesArray, const NodeType* nodeToCheck,
                            bool* status,size_t numNodes){

    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < numNodes){
        if (d_nodesArray[idx].isEqual(*nodeToCheck)) {
            *status = true;
    }}
};

/** 
 * @brief heuristic function which computes the estimated cost to the goal 
 * node, starting from the current given node, it simply the euclidian distances
 * or the manthetn distance could be used 
 */
template <typename NodeType, typename T>
DEVICE_FUN void computeHeuristicDistance(NodeType* Node, T* hfun){
    *hfun = Node->distanceTo(endNode);
};

/** 
 * @brief get the cost of the a path computed the path is given as 
 * an array of variables  each 
 * one of NodeType  f(n) = g(n) + h(n) 
*/
template <typename NodeType, typename T>
DEVICE_FUN void computeTrajectoryCost(const NodeType* node, T* p_cost_){
    T h_fun;
    computeHeuristicDistance(*node, &h_fun); 
    *p_cost_ = node->sum_cost + h_fun;
};

/** 
 * @brief computes the best node sucessor based on path metric :f(n) = g(n) + h(n) 
 * from the k neiregst neighbbodes nodes, each thread will compute the trajecory cost 
 * and then get the node of the mimium cost, 
*/
template <typename NodeType, typename T>
__global__ void computeSucessorNode(const NodeType* d_knnNodesArray, int k,
                                    const NodeType* endNode,
                                    NodeType* d_bestNode){

    int idx  = threadIdx.x;
    extern __shared__ T sharedCost[];
    extern __shared__ int sharedIndex[];

    if (idx < k) {
        sharedIndex[idx] = idx;
    }

    if ( idx < k  ){
        T cost;
        TT* idx_cost = &cost;
        computeTrajectoryCost<NodeType, T>(d_knnNodesArray[idx],idx_cost);
       sharedCost[idx] = cost;
    }
    __syncthreads();

    int* d_minIndex;
    for (unsigned int s  = blockDim.x/2; s > 0; s >>= 1 ){

        if (idx < s ){
            if (sharedCost[idx] > sharedCost[idx + s]) {
                sharedCost[idx] = sharedCost[idx + s];
                sharedIndex[idx] = sharedIndex[idx + s];
            }
        }
    }
    if (idx == 0) {
        *d_minIndex = sharedIndex[0];
    }
    *d_bestNode =   d_knnNodesArray[d_minIndex];
};


/** 
 * @brief soothing the distcreate computed trajectory, using either spline 
 * basis or bezier curves , or just normale interpolations.
 * @param N number of points or control points in the trajectory 
 * @param k number of the evaulation point of the new trajectory  
 */ 
template <typename NodeType, typename T>
__global__ void smoothTrajectory(const NodeType* d_trajNodesArray,int N,int k,
                            NodeType* d_trajNodesArraySmooth){


};

/**
 * @class AstarPlanner
 * 
 */
template <typename NodeType, typename T>
class AstarPlanner
{
public:

    /** 
     * @brief  default constructor, allocate the fixed size momory on 
     * gpu for the member variables  
     */
    HOST_FUN  AstarPlanner();

    /** 
     * @brief init the host and device memory with random nodes, if the number of
     * ndodes given os diffrent from class alraedy  vars old, ovveride them 
     */
    HOST_FUN void initRandomNodesArray(size_t numNodes_, unsigned int seed);

    /** 
     * @brief fill the host and device memory with the nodes from the a point
     * cloud data file .PLY  
     */
    HOST_FUN void initNodesArray(const std::string& plyFilePath);

    /** 
     * @brief set the start and the goal nodes either in 3d or 2d, check if the 
     * nodes exsits in the h_nodesArray first 
     */
    HOST_FUN void setStartAndGoalNodes(NodeType* startNode_, NodeType* goalNode_);

    /**
     * @brief compute the optimal trajectory, ie array of nodeType values, 
     * this is the final function used to retive the optimal traj  
     */
    HOST_FUN  void computeTrajectory();

    /** @brief svaes a trajectory to pointcloud file .PLY using happly librray */
    HOST_FUN void saveTrajectory2csv(const std::string outputFilePath);

    /** 
     * @brief read a point cloud data file (now only PLy supported) and fill the gridMap
     * object memebr of the class with nodes consruted from the point cloud 
     */
   HOST_FUN void saveTrajectory2png(const std::string& outputFilePath);

 
private:
    size_t    numNodes;
    NodeType * d_nodesArray; 
    NodeType * h_nodesArray;

    NodeType*   startNode;
    NodeType*   endNode;

    NodeType*   h_pathArray;
    NodeType*   d_pathArray;

    /** @brief Check wahat ever we get into the gail node or not   */
    template <typename NodeType, typename T>
    HOST_DEVICE_FUN bool isGoalReached(NodeType* n, const T eps);

protected:
    /** 
     * @brief free all memory on deice or on host , free all 
     * class device or host varaible , qqsoit 
     */
    HOST_FUN ~AstarPlanner();
};

#ifdef USE_CUASTAR_TYPEDEF
    typedef AstarPlanner<Node2d<float>,  float>  AstarPlanner2dFloat;
    typedef AstarPlanner<Node2d<double>, double> AstarPlanner2dDouble;
    typedef AstarPlanner<Node3d<float>,  float>  AstarPlanner3dFloat;
    typedef AstarPlanner<Node3d<double>, double> AstarPlanner3dDouble;
#endif

#ifdef CUASTAR_IMPLEMENTATION 

    template <typename NodeType, typename T>
    HOST_FUN AstarPlanner<NodeType, T>::AstarPlanner(){

        cudaError_t err = cudaMalloc((void**)&d_nodesArray, numNodes*sizeof(NodeType));
        cudaMalloc((void**)&d_pathArray, numNodes * sizeof(NodeType));
        h_nodesArray = new NodeType[numNodes];  
        CUDA_CHECK_ERROR(err);
    }

    template <typename NodeType, typename T>
    HOST_FUN void AstarPlanner<NodeType, T>::initRandomNodesArray(size_t numNodes_,unsigned int seed){

        numNodes = numNodes_;
        NodeType* h_nodesArray = new NodeType[numNodes];

        for (size_t i = 0; i < numNodes; ++i) {
            h_nodesArray[i] = NodeType(seed + i); 
        }
        NodeType* d_nodesArray_;
        cudaError_t err = cudaMalloc((void**)&d_nodesArray_, numNodes * sizeof(NodeType));
        CUDA_CHECK_ERROR(err);
        cudaMemcpy(d_nodesArray_, h_nodesArray,numNodes*sizeof(NodeType),cudaMemcpyHostToDevice);
        this->d_nodesArray = d_nodesArray_;
    }

    template <typename NodeType, typename T>
    HOST_FUN void AstarPlanner<NodeType, T>::initNodesArray(const std::string& plyFilePath) {

        numNodes = static_cast<size_t>(getPlyPointNum(plyFilePath));
        NodeType* h_nodesArray = new NodeType[numNodes];
        NodeType* d_nodesArray_ = nullptr;
        cudaError_t err = cudaMalloc((void**)&d_nodesArray_, numNodes * sizeof(NodeType));
        #ifdef _DEBUG_
            if (err != cudaSuccess) {
                LOG_F(ERROR, "Failed to allocate device memory: %s", cudaGetErrorString(err));
                delete[] h_nodesArray;   
                exit(EXIT_FAILURE);
            }
        #endif
        for (size_t i = 0; i < numNodes; ++i) {
            try {
                NodeType n = getPlyNode(plyFilePath, i);
                h_nodesArray[i] = n;
            } catch (const std::exception& e) {
                #ifdef _DEBUG_
                    LOG_F(ERROR, "Exception while getting PLY node at index %zu: %s", i, e.what());
                #endif
                delete[] h_nodesArray;
                cudaError_t err = cudaFree(d_nodesArray_);
            }
        }
        err = cudaMemcpy(d_nodesArray_,h_nodesArray,numNodes*sizeof(NodeType),cudaMemcpyHostToDevice);
        #ifdef _DEBUG_
            if (err != cudaSuccess) {
                LOG_F(ERROR, "Failed to copy data from host to device: %s", cudaGetErrorString(err));
                delete[] h_nodesArray; 
                cudaFree(d_nodesArray_);
                exit(EXIT_FAILURE);
            }
        #endif
        this->d_nodesArray = d_nodesArray_;
    }

    template <typename NodeType, typename T>
    HOST_FUN AstarPlanner<NodeType, T>::~AstarPlanner(){

        delete[] h_nodesArray;
        h_nodesArray = nullptr;
        cudaFree(d_nodesArray);
    };

    template <typename NodeType, typename T>
    HOST_FUN void AstarPlanner<NodeType, T>::setStartAndGoalNodes(NodeType* startNode_,
                                                                 NodeType* goalNode_){
        bool* d_startNodeExists;
        bool* d_goalNodeExists;
        cudaMalloc((void**)&d_startNodeExists, sizeof(bool));
        cudaMalloc((void**)&d_goalNodeExists , sizeof(bool));

        cudaMemset(d_startNodeExists,0, sizeof(bool));
        cudaMemset(d_goalNodeExists, 0, sizeof(bool));

        
        int blocksPerGrid = (numNodes+threadsPerBlock-1)/threadsPerBlock;
        checkNodeExsist<<<blocksPerGrid, threadsPerBlock>>>(startNode_, d_startNodeExists);
        checkNodeExsist<<<blocksPerGrid, threadsPerBlock>>>(goalNode_, d_goalNodeExists);

        bool h_startNodeExists = false;
        bool h_goalNodeExists = false;

        cudaMemcpy(&h_startNodeExists,d_startNodeExists,sizeof(bool),cudaMemcpyDeviceToHost);
        cudaMemcpy(&h_goalNodeExists,d_goalNodeExists,sizeof(bool),cudaMemcpyDeviceToHost);

       if (h_startNodeExists && h_goalNodeExists) {
        startNode = startNode_;
        endNode = goalNode_;
        } else {
            #ifdef _DEBUG_
                if (!h_startNodeExists) {
                    LOG_F(ERROR, "Start node does not exist");
                }
                if (!h_goalNodeExists) {
                    LOG_F(ERROR, "Goal node does not exist");
                }
            #endif
        }
        cudaFree(d_startNodeExists);
        cudaFree(d_goalNodeExists);
    };

    template<typename NodeType, typename T>
    HOST_FUN  void AstarPlanner<NodeType, T>::computeTrajectory(){
        
        h_pathArray[0] = startNode;
        cudaMemcpy(&h_pathArray,d_pathArray,sizeof(NodeType),cudaMemcpyHostToDevice);

    }
    
    template <typename NodeType, typename T>
    HOST_FUN void AstarPlanner<NodeType, T>::saveTrajectory2png(const std::string& outputFilePath){

       namespace fs = std::filesystem;
       try {
            fs::path outputPath(outputFilePath);
            fs::path directory = outputPath.parent_path();
            if (!directory.empty() && !fs::exists(directory)){
                fs::create_directories(directory);
            }
            if (fs::exists(outputPath)) {
                fs::remove(outputPath);
            }
            array2PointCloudImg(h_pathArray,numNodes,outputFilePath,width,height,radiusRatio);
        } catch (const fs::filesystem_error& e) {
            std::cerr << "Filesystem error: " << e.what() << '\n';
        } catch (const std::exception& e) {
            std::cerr << "Error: " << e.what() << '\n';
        }
    };

    template <typename NodeType, typename T>
    HOST_DEVICE_FUN bool  AstarPlanner<NodeType, T>::isGoalReached(NodeType* n, const T eps){
        if (*n == endNode) {
            return true;
        }
        return static_cast<T>(n->distanceTo(endNode)) < eps;
    }

#endif // CUASTAR_IMPLEMENTATION


/** @brief Check if a given .ply file path exists or not */
HOST_FUN bool isPlyValid(const std::string plyFilePath){

    try {
      happly::PLYData plyIn(plyFilePath);
      return true;
      } 
      catch (const std::runtime_error& e) {
          #ifdef _DEBUG_
            LOG_F(ERROR, "Error opening PLY file: %s", e.what());
          #endif
          return false;
      };
}

/** @brief Get a Node3d object from a .ply file using index idx */
template <typename NodeType, typename T>
HOST_FUN NodeType getPlyNode(const std::string& plyFilePath, const size_t idx) {
    
    if (!isPlyValid(plyFilePath)) {
        #ifdef _DEBUG_
            LOG_F(ERROR, "Invalid PLY file: '%s'", plyFilePath.c_str());
        #endif
        throw std::invalid_argument("Invalid PLY file.");
    }
    happly::PLYData plyIn(plyFilePath);
    std::vector<std::array<double, 3>> vertices = plyIn.getVertexPositions();
    if (idx < vertices.size()) {
        
        const auto& vertex = vertices[idx];
        return NodeType(static_cast<T>(vertex[0]), static_cast<T>(vertex[1]), 
            static_cast<int32_t>(vertex[2]));

    }else {
        #ifdef _DEBUG_
            LOG_F(ERROR, "Index out of range in PLY file '%s': requested idx = %zu, max idx = %zu",
              plyFilePath.c_str(), idx, vertices.size() - 1);
        #endif
    }
}

/** @brief Return the number of points stored in a .ply file */
HOST_FUN size_t getPlyPointNum(const std::string plyFilePath){

  if (isPlyValid(plyFilePath)){
    happly::PLYData plyIn(plyFilePath);
    std::vector<std::array<double, 3>> vertices = plyIn.getVertexPositions();
    return static_cast<size_t>(vertices.size());
  }else{
    return static_cast<size_t>(0);
  }
};

/** @brief Draw a 2D sphere (plain circle) given radius, center, and color. */
void drawFilledCircle(unsigned char* image, int width, int height, const Circle* circle, 
                    const Color* color) {
    int cx = circle->x;
    int cy = circle->y;
    int radius = circle->radius;

    for (int y = (cy - radius > 0 ? cy - radius : 0); y <= (cy + radius < height ? cy + 
    radius : height - 1); ++y) {
        for (int x = (cx - radius > 0 ? cx - radius : 0); x <= (cx + radius < width ? cx 
        + radius : width - 1); ++x) {
            int dx = x - cx;
            int dy = y - cy;
            if (dx * dx + dy * dy <= radius * radius) {
                int index = (y * width + x) * 3;
                image[index] = color->r;
                image[index + 1] = color->g;
                image[index + 2] = color->b;
            }
        }
    }
}

/** @brief Draw a sample of point cloud nodes as 2D colored circles */
HOST_FUN void drawFilledCircles(unsigned char* image, int width, int height, 
                       const int* centersX, const int* centersY, 
                       const unsigned char* colorsR, const unsigned char* 
                       colorsG, const unsigned char* colorsB, 
                       int numCircles, double radiusRatio) {
    
    int radius = static_cast<int>(radiusRatio * std::min(width, height));

    for (int i = 0; i < numCircles; ++i) {
        Circle circle = {centersX[i], centersY[i], radius};
        Color color = {colorsR[i], colorsG[i], colorsB[i]};
        drawFilledCircle(image, width, height, &circle, &color);
    }
}

/**  @brief Draw and save a given 2D points cloud of nodes as an image. */
HOST_FUN void savePointCloudImage(const char* filePath, int width, int height, 
                         const int* centersX, const int* centersY, 
                         const unsigned char* colorsR, const unsigned char* colorsG, 
                         const unsigned char* colorsB, 
                         int numCircles, double radiusRatio) {

    unsigned char* image = new unsigned char[width * height * 3];
    std::memset(image, 255, width * height * 3);  
    drawFilledCircles(image, width, height, centersX, centersY, colorsR, 
    colorsG, colorsB, numCircles, radiusRatio);

    stbi_write_png(filePath, width, height, 3, image, width * 3);

    delete[] image;
}

/**  
 * @brief Draw and save a given 2D point cloud of nodes as an image, 
 * the data is given as a 1d host array of NodeType template objects.
 */
template <typename NodeType>
HOST_FUN void array2PointCloudImg(const NodeType* h_arrayNodes, size_t numNodes,
                         const char* pngFilePath, int width, int height, 
                         double radiusRatio = 0.01) {
    
    int* centersX = new int[numNodes];
    int* centersY = new int[numNodes];
    unsigned char* colorsR = new unsigned char[numNodes];
    unsigned char* colorsG = new unsigned char[numNodes];
    unsigned char* colorsB = new unsigned char[numNodes];
    
    for (size_t i = 0; i < numNodes; ++i) {
        centersX[i] = static_cast<int>(h_arrayNodes[i].x);
        centersY[i] = static_cast<int>(h_arrayNodes[i].y);
        colorsR[i] = static_cast<unsigned char>(h_arrayNodes[i].r);
        colorsG[i] = static_cast<unsigned char>(h_arrayNodes[i].g);
        colorsB[i] = static_cast<unsigned char>(h_arrayNodes[i].b);
    }
    
    savePointCloudImage(pngFilePath, width, height, centersX, 
    centersY, colorsR, colorsG, colorsB, static_cast<int>(numNodes), 
    radiusRatio);
    
    delete[] centersX;
    delete[] centersY;
    delete[] colorsR;
    delete[] colorsG;
    delete[] colorsB;
}

#ifdef ENABLE_VTK

    /**
     * @brief Display a node as a sphere with a specified radius and color.
     * @param node Pointer to the Node3d object to visualize.
     * @param radius Radius of the sphere (default is 0.01).
     * @param color Array of size 3 containing RGB values for the color.
     * @note this function has no cv2 implementation.
     */
    void vizNode3d(const Node3d* node, double radius = 0.01, double color[3]) {
    
        vtkNew<vtkNamedColors> colors;
        vtkNew<vtkSphereSource> nodeSource;
        nodeSource->SetCenter(node->x, node->y, node->z);
        nodeSource->SetRadius(radius);
        nodeSource->SetThetaResolution(20);
        nodeSource->SetPhiResolution(20);
        nodeSource->Update();

        vtkNew<vtkPolyDataMapper> nodeSourceMapper;
        nodeSourceMapper->SetInputConnection(nodeSource->GetOutputPort());
        nodeSourceMapper->ScalarVisibilityOn();  

        vtkNew<vtkActor> nodeSourceActor;
        nodeSourceActor->SetMapper(nodeSourceMapper);
        nodeSourceActor->GetProperty()->SetColor(color[0] ,color[1],color[2]);  
        nodeSourceActor->GetProperty()->SetOpacity(1.0);

        vtkNew<vtkRenderer> renderer;
        vtkNew<vtkRenderWindow> renderWindow;
        renderWindow->AddRenderer(renderer);
        vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;
        renderWindowInteractor->SetRenderWindow(renderWindow);
        renderer->AddActor(nodeSourceActor);
        renderer->SetBackground(colors->GetColor3d("White").GetData());
        renderWindow->Render();
        renderWindowInteractor->Start();
    };

    /**
     * @brief Visualizes the entire 3D point cloud from a specified .ply file.
     * This function renders the point cloud contained in the given .ply file, 
     * preserving the original RGBA colors.
     * @note This function does not have an implementation using OpenCV (cv2).
     * It relies on external visualization libraries, and the point cloud data
     * must include RGBA color information.
     * @param inputFilePath The path to the input .ply file containing the 3D point 
     * cloud data.
     * @param pointsRadius The radius for all points in the visualization. 
     * @note Use a smaller radius for large point clouds to 
     * ensure clarity and performance. The default value is 0.01.
     */
    void vizWorld3d(const std::string inputFilePath,const double pointsRadius=0.01){

        vtkNew<vtkNamedColors> colors;
        vtkNew<vtkPLYReader> reader;
        reader->SetFileName(inputFilePath.c_str());
        reader->Update();                           

        vtkPolyData* polyData = reader->GetOutput();
        vtkPointData* pointData = polyData->GetPointData();

        vtkUnsignedCharArray* colorsArray=
        vtkUnsignedCharArray::SafeDownCast(pointData->GetArray("RGBA"));
        if (!colorsArray){
            std::cerr << 
            "Error:The PLY file does not contain an array named 'RGBA'!"<<std::endl;
        }
        vtkIdType numPoints = polyData->GetNumberOfPoints();
        std::cout << "Number of points: " << numPoints << std::endl;

        vtkNew<vtkAppendPolyData> appendFilter;

        for (vtkIdType i = 0; i < numPoints; ++i){
        double p[3];
        polyData->GetPoint(i, p);
        unsigned char* color = colorsArray->GetPointer(4 * i);
        unsigned char rgba[4] = {
            color[0],  
            color[1],  
            color[2],  
            color[3]   
        };

        vtkNew<vtkSphereSource> sphereSource;
        sphereSource->SetCenter(p);
        sphereSource->SetRadius(pointsRadius); 
        sphereSource->SetThetaResolution(8);
        sphereSource->SetPhiResolution(8);
        sphereSource->Update();

        vtkIdType numSpherePoints = sphereSource->GetOutput()->GetNumberOfPoints();

        vtkNew<vtkUnsignedCharArray> colorArray;
        colorArray->SetNumberOfComponents(4); 
        colorArray->SetName("Colors");
        for (vtkIdType j = 0; j < numSpherePoints; ++j){
            colorArray->InsertNextTypedTuple(rgba);
        }
        sphereSource->GetOutput()->GetPointData()->SetScalars(colorArray);
        appendFilter->AddInputConnection(sphereSource->GetOutputPort());
        };

        vtkNew<vtkRenderer> renderer;
        vtkNew<vtkRenderWindow> renderWindow;
        renderWindow->AddRenderer(renderer);
        vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;
        renderWindowInteractor->SetRenderWindow(renderWindow);
    
        vtkNew<vtkPolyDataMapper> mapper;
        mapper->SetInputConnection(appendFilter->GetOutputPort());
        mapper->ScalarVisibilityOn();  
        mapper->SetScalarModeToUsePointData(); 
        mapper->SelectColorArray("Colors");

        vtkNew<vtkActor> pointCloudActor;
        pointCloudActor->SetMapper(mapper);

        renderer->AddActor(pointCloudActor);
        renderer->SetBackground(colors->GetColor3d("White").GetData());
        renderWindow->Render();
        renderWindowInteractor->Start();
    };

    /**
     * @brief display the 2d trajectory data using a 3d trajectory file 
     * a 3d trajectory file is a non rgba point cloud file stored in .ply 
     * format  
     * @note this function do not have cv2 implemantion
     */
    void viz3dTrajectory(const std::string inputFilePath){
     

    };
#endif // ENABLE_VTK
#endif // CUASTAR_HPP