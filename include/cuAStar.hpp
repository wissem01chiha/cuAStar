/**
 * @file cuAstar.hpp
 * @version 0.1.0
 * @author wissem chiha
 * @date 03-09-2024
 * 
 * @link https://towardsdatascience.com/understanding-a-path-algorithms-and-implementation-with-python-4d8458d6ccc7
 * 
 * @mainpage
 * If there are no blocked cells/obstacles then we can just find the exact 
 * value of h without any pre-computation using the distance formula/Euclidean Distance
 * all memory errors checks foe cuda are in debug mode !
 * $ g++ -std=c++0x main.cpp -pthread
 * nvcc -o random_gen_example random_gen_example.cu -lcurand
 * divide the point cloud data into small region called bubles a N pt cloud is divided into 
 * N/k where k is the window size , for each subarea , inti a substarting node
 * explore (like running the normal A star on that subgroups nodes) 
 * with one setp the surouding and at each step , synchonize all subregions 
 * 
 * https://pantelis.github.io/artificial-intelligence/aiml-common/lectures/planning/search/a-star/index.html
 * 
 * @note for the multithreding and concurrency support computations support 
 * the c++ 11 standard is required More Info :  https://en.cppreference.com/w/cpp/thread
 * https://github.com/arvkr/ransac_cuda_eecse4750
 * @note : 3D visulisation and rending is enabled only throut VTK,wich should be installed ,
 * we use VTK version 9.3.1, for more information refre to VTK installation guide   
 */
#ifndef CUASTAR_HPP
#define CUASTAR_HPP

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
    #ifdef CUASTAR_DEBUG
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

#define STB_IMAGE_WRITE_IMPLEMENTATION 

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

#ifdef CUASTAR_DEBUG
    #if defined(__CUDACC__)
    #define CUDA_CHECK_ERROR(err) \
        if (err != cudaSuccess) { \
            LOG_F(ERROR, "CUDA error: %s", cudaGetErrorString(err)); \
            exit(EXIT_FAILURE); \
        }
    #endif
    #define LOG_MESSAGE(level, format, ...) \
        LOG_F(level, format, ##__VA_ARGS__)
    #include <iostream>
    #include "../extern/loguru/loguru.hpp"
    #include "../extern/loguru/loguru.cpp"
#else
    #define CUDA_CHECK_ERROR(err)
    #define LOG_MESSAGE(level, format, ...) \
        do {} while (0)
#endif
#ifdef CUASTAR_USE_VTK
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
 * @brief  implemntation of atomic min for float type as cuda do not support it
 * original implentation by : https://forums.developer.nvidia.com/t/atomicmin-with-float/22033 
 */
__device__ float fatomicMin(float *addr, float value) {
    float old = *addr, assumed;
    if (old <= value) return old;
    do {
        assumed = old;
        old = atomicCAS((unsigned int*)addr, __float_as_int(assumed), 
        __float_as_int(value < assumed ? value : assumed));
    } while (old != assumed);
    return old;
}

__device__ float fatomicMax(float *addr, float value) {
    float old = *addr, assumed;
    if (old >= value) return old;
    do {
        assumed = old;
        old = atomicCAS((unsigned int*)addr, __float_as_int(assumed), 
        __float_as_int(value > assumed ? value : assumed));
    } while (old != assumed);
    return old;
}

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
 */
template <typename T>
class Node2d {
public:
    T x;           
    T y;
    T z = static_cast<T>(0);         
    T sum_cost;   
    Node2d* p_node; 

    int r = 125;
    int g = 100;
    int b = 120;      

    /** @brief default constructor for the Node2d class */
    HOST_DEVICE_FUN Node2d(){
        x= T(0);
        y= T(0);
        sum_cost = T(0);
        p_node = nullptr;
    }

    /** @brief Constructor for the Node2d class */
    HOST_DEVICE_FUN Node2d(T x_, T y_, T sum_cost_ ,Node2d* p_node_ ){
        x= x_;
        y= y_;
        sum_cost = sum_cost_;
        p_node = p_node_;
    }

    /** 
     * @brief Overlaoding Constructor for the general puropse template 
     * calls 
     */
    HOST_DEVICE_FUN Node2d(T x_, T y_, T z_){
        x= x_;
        y= y_;
        sum_cost = T(0);
        p_node = nullptr;
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

    #ifdef CUASTAR_DEBUG
        /** @brief Prints the information of this node from a host member */
        HOST_FUN void printNodeInfo() const {
            std::cout << "Node: (x: " << x << ", y: " << y 
                    << ", sum_cost: " << sum_cost << ", color: (" << static_cast<int>(r) 
                    << ", " << static_cast<int>(g) << ", " << static_cast<int>(b) << "))\n";
        }

        /** @brief Prints the node information from device code  */
        DEVICE_FUN void printDeviceNodeInfo() const {
            printf("Node: (x: %.6f, y: %.6f, sum_cost: %.6f, color: (%d, %d, %d))\n",
               x, y, sum_cost, static_cast<int>(r), static_cast<int>(g), static_cast<int>(b));
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
 */
template <typename T>
class Node3d {
public:
    T x;         
    T y;         
    T z;           
    T sum_cost;   
    Node3d* p_node;

    int r = 125;
    int g = 100;
    int b = 120;   

    /** @brief Default constructor for the Node3d class */
    HOST_DEVICE_FUN Node3d(){
        x = T(0);
        y = T(0);
        z = T(0);
        sum_cost = T(0);
        p_node = nullptr;
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

    #ifdef CUASTAR_DEBUG
        /** @brief Prints the information of this node */
        HOST_FUN void printNodeInfo() const {
            std::cout << "Node: (x: " << x << ", y: " << y << ", z: " << z
                    << ", sum_cost: " << sum_cost << ", color: (" << static_cast<int>(r) 
                    << ", " << static_cast<int>(g) << ", " << static_cast<int>(b) << "))\n";
        }

        /** @brief Prints the node information from device code  */
        DEVICE_FUN void printDeviceNodeInfo() const {
            printf("Node: (x: %.6f, y: %.6f, z: %.6f, sum_cost: %.6f, color: (%d, %d, %d))\n",
               x, y, z, sum_cost, static_cast<int>(r), static_cast<int>(g), static_cast<int>(b));
        }
        
    #endif
};

#ifdef CUASTAR_USE_TYPEDEF
    typedef Node2d<double> Node2dDouble;  
    typedef Node2d<float>  Node2dFloat;   
    typedef Node3d<double> Node3dDouble; 
    typedef Node3d<float>  Node3dFloat;  
#endif
/**
 * @brief rgb color base struture 
 */
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
const int threadsPerBlock = 1024; 

#ifdef CUASTAR_DEBUG
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
     * structures, this function will be decaprted
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
 * @brief Check if a given point cloud data file .ply file path 
 * exists or not
 * @throw a user error to cmd in debug mode, else none. 
 */
HOST_FUN bool isPlyValid(const std::string plyFilePath){

    try {
      happly::PLYData plyIn(plyFilePath);
      return true;
      } 
      catch (const std::runtime_error& e) {
          LOG_MESSAGE(ERROR, "Error opening PLY file: %s", e.what());
          return false;
      };
}

/** 
 * @brief extract a Node object from a .ply file defined by index idx, 
 * in the file, reprsent the line number of point coordiantes, it is computioanlly slow , 
 * not use in loops or other , just for single node extrcation,
 * this method will be decapred in other versions
 */
template <typename NodeType, typename T>
HOST_FUN NodeType readPlyNode(const std::string& plyFilePath, const size_t idx) {
    
    if (!isPlyValid(plyFilePath)) {
        LOG_MESSAGE(ERROR,"Invalid PLY file: '%s'", plyFilePath.c_str());
    }
    happly::PLYData plyIn(plyFilePath);
    std::vector<std::array<double, 3>> vertices = plyIn.getVertexPositions();
    if (idx < vertices.size()) {
        const auto& vertex = vertices[idx];
        return NodeType(static_cast<T>(vertex[0]), static_cast<T>(vertex[1]), 
            static_cast<int32_t>(vertex[2]));
    }else {
        LOG_MESSAGE(ERROR,"Index out of range in PLY file '%s': requested idx = %zu, max idx = %zu", 
         plyFilePath.c_str(), idx, vertices.size() - 1);
    }
}

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
 * @brief compute the 2d bound box dimension for 2d plotting scaling issues
 * for N nodes elments the minimal block grid number is :
 *  (N + thredsPerBlock - 1) / thredsPerBlock; 
 * interblock synhonization , usen shared memory
 * Thread 0 of each block updates the global min/max values using atomic operations
 * note we cannot template the function with double , because cuda cannot use atomic 
 * operation for doubles , just for cuda computation cabolit >6.0 
 * see: https://forums.developer.nvidia.com/t/atomic-functions-for-double-precision/9963
 * https://forums.developer.nvidia.com/t/why-does-atomicadd-not-work-with-doubles-as-input/56429
 * atomic operations  is only supported by devices of compute capability 5.0 and higher.
 * @todo Use of Local Reductions per Block
 */
template <typename NodeType>
__global__ void computePointCloudBoundBox(const NodeType* h_arrayNodes, int N,  
                                        float* minX, float* maxX, float* minY, float* maxY){

    __shared__ float localMinX, localMaxX, localMinY, localMaxY;

    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    
    if( threadIdx.x == 0){ // for each block intilize eeach thred has it owen copy
        localMinX = FLT_MAX;
        localMaxX = -FLT_MAX;
        localMinY = FLT_MAX;
        localMaxY = -FLT_MAX;
    }
    __syncthreads();

    if (idx < N){

        NodeType node = h_arrayNodes[idx];

        fatomicMax(&localMaxX, node.x);
        fatomicMin(&localMinX, node.x);
        fatomicMax(&localMaxY, node.y);
        fatomicMin(&localMinY, node.y);
    }
    __syncthreads();
    
    if (threadIdx.x == 0) {
        fatomicMin(minX, localMinX);
        fatomicMax(maxX, localMaxX);
        fatomicMin(minY, localMinY);
        fatomicMax(maxY, localMaxY);
    }
}

/**  
 * @brief Draw and save a given 2D point cloud of nodes as an image, 
 * the data is given as a 1d host array of NodeType template objects.
 * @param scaleFactor : zoom image coffienct
 * @param radiusRatio :  
 */
template <typename NodeType>
HOST_FUN void array2PointCloudImg(const NodeType* h_arrayNodes, int numNodes,
                        const char* pngFilePath,double scaleFactor=100,
                        double radiusRatio = 0.01) {
    
    float h_minX = FLT_MAX, h_maxX = -FLT_MAX, h_minY = FLT_MAX, h_maxY = -FLT_MAX;

    float *d_minX, *d_maxX, *d_minY, *d_maxY;
    cudaMalloc(&d_minX, sizeof(float));
    cudaMalloc(&d_maxX, sizeof(float));
    cudaMalloc(&d_minY, sizeof(float));
    cudaMalloc(&d_maxY, sizeof(float));

    cudaMemcpy(d_minX, &h_minX, sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_maxX, &h_maxX, sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_minY, &h_minY, sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_maxY, &h_maxY, sizeof(float), cudaMemcpyHostToDevice);

    NodeType* d_arrayNodes;
    cudaMalloc(&d_arrayNodes, numNodes * sizeof(NodeType));
    cudaMemcpy(d_arrayNodes, h_arrayNodes, numNodes * sizeof(NodeType), cudaMemcpyHostToDevice);

    int blocksNum =  (numNodes + threadsPerBlock - 1) / threadsPerBlock;
    computePointCloudBoundBox<NodeType><<<blocksNum, threadsPerBlock>>>(
        d_arrayNodes, numNodes, d_minX, d_maxX, d_minY, d_maxY);

    cudaMemcpy(&h_minX, d_minX, sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(&h_maxX, d_maxX, sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(&h_minY, d_minY, sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(&h_maxY, d_maxY, sizeof(float), cudaMemcpyDeviceToHost);

    cudaFree(d_minX);
    cudaFree(d_maxX);
    cudaFree(d_minY);
    cudaFree(d_maxY);
    cudaFree(d_arrayNodes);

    int* centersX = new int[numNodes];
    int* centersY = new int[numNodes];
    unsigned char* colorsR = new unsigned char[numNodes];
    unsigned char* colorsG = new unsigned char[numNodes];
    unsigned char* colorsB = new unsigned char[numNodes];
    
    double r = (h_maxY - h_minY)/(h_maxX - h_minX);
    int width = static_cast<int>(scaleFactor * (h_maxX - h_minX));
    int height = static_cast<int>( r * width);

    for (int i = 0; i < numNodes; ++i) {
        centersX[i] = static_cast<int>((h_arrayNodes[i].x - h_minX)/(h_maxX - h_minX) * width );
        centersY[i] = static_cast<int>((h_arrayNodes[i].y - h_minY)/(h_maxY - h_minY) * height );
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

/** @brief Return the number of points stored in a .ply file */
HOST_FUN size_t getPlyPointNum(const std::string plyFilePath){

  if (isPlyValid(plyFilePath)){
    happly::PLYData plyIn(plyFilePath);
    std::vector<std::array<double, 3>> vertices = plyIn.getVertexPositions();
    return static_cast<size_t>(vertices.size());
  }else{
    return static_cast<size_t>(0);
  }
}

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

    for (int i = 0; i < k; ++i) {
        if (nearestNodes[i].isEqual(otherNode,static_cast<T>(1e-8))) {
            return;  
        }
    }
    T dist = node.distanceTo(otherNode);
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
 * array structure , by sorting x, y, z attributes, without using the K-D tree
 * this kernel should excute on 1 block, so the maxium nodes number sorted arrays 
 * is 1024 
 * @param range: control the exploration range of the KNN, fixed , adjustee based 
 * on the point cloud distribution density , low for high dense data and high for low 
 * dense distrubution
 * @param k should be < threadsPerBlock
 */
template <typename NodeType, typename T>
__global__ void computeKnnNodes(NodeType* d_sortedX, NodeType* d_sortedY, NodeType* d_sortedZ,
                                NodeType targetNode, int N, int k, int range, 
                                NodeType* d_kNodes) {
    int idx = threadIdx.x;

    if (idx < N) {
       extern __shared__ NodeType nearestNodes[];
       extern __shared__ T distances[];

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
 * @brief Checks if a given node exists in the device nodes array.
 * @note This method will be deprecated in future versions.
 * @note we use atomic exchange to ensure thred safty writtin value to the 
 * we reprsent by : 0 FALSE and any other int the TRUE value, 
 * perform chek if only nly perform check if the index is within 
 * bounds and status is still false (0)
 */
template <typename NodeType, typename T>
__global__ void checkNodeExsist(const NodeType* d_nodesArray, int numNodes,
                                 const NodeType* nodeToCheck, int* status) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    
    if (idx < numNodes) {
        if (atomicExch(status, 0) == 0) {
            if (d_nodesArray[idx].isEqual(*nodeToCheck)) {
                atomicExch(status, 1);  
            }
       }
    }
}

/** 
 * @brief heuristic function which computes the estimated cost to the goal 
 * node, starting from the current given node, it simply the euclidian distances
 * or the manthetn distance could be used 
 */
template <typename NodeType, typename T>
DEVICE_FUN void computeHeuristicDistance(const NodeType& node, const NodeType& targetNode,
                                    T* hfun){
    *hfun = node.distanceTo(targetNode);
};

/**
 * @brief Computes the trajectory cost metric starting from a given node.
 * The objective function is: f(n) = g(n) + h(n),
 * where g(n) is the path cost from the start node to the current node,
 * and h(n) is the heuristic function value.
 *
 * @param node The current node representing the position in the trajectory.
 */
template <typename NodeType, typename T>
DEVICE_FUN void computeNodeTrajectoryCost(const NodeType& node, const NodeType& targetNode,
                                        T* p_cost_){
    T h_fun;
    computeHeuristicDistance(node, targetNode, &h_fun); 
    *p_cost_ = node.sum_cost + h_fun;
};

/** 
 * @brief Computes the best successor node based on path metric f(n) = g(n) + h(n) 
 *        from the k nearest neighbor nodes. Each thread will compute the trajectory cost 
 *        and then determine the node with the minimum cost.
 *        This function is designed to execute within a single block.
 * 
 * @param d_knnNodesArray Array of k-nearest neighbor nodes of the current node to be checked.
 *                        The maximum size is 1024.
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
        T* idx_cost = &cost;
        computeNodeTrajectoryCost<NodeType,T>(d_knnNodesArray[idx],*endNode, idx_cost);
       sharedCost[idx] = cost;
    }
    __syncthreads();

    if (idx == 0) {
        T minCost = sharedCost[0];
        int minIndex = 0;

        for (int i = 1; i < k; ++i) {
            if (sharedCost[i] < minCost) {
                minCost = sharedCost[i];
                minIndex = i;
            }
        }

    *d_bestNode = d_knnNodesArray[minIndex];
    }
};

/**
 * @brief Given a trajectory path represented by a device array of nodes,
 * it computes the path cost: the Euclidean distance between the start and 
 * destination nodes. The start node is the first node in the array, and the 
 * destination node is the last node.
 * @note Boundary checks are not performed; N is assumed to be within the 
 * array size range.
 */
template <typename NodeType, typename T>
DEVICE_FUN void computeTrajectoryCost(const NodeType* d_tarjNodesArray, int N, T* cost_){
    T cost = static_cast<T>(0);
    for (int i = 0; i < N - 1; i++) {
        cost += d_tarjNodesArray[i].distanceTo(d_tarjNodesArray[i + 1]);
    }
    *cost_ = cost; 
}

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
 * @brief Extracts nodes without a parent from an array of N nodes.
 * Iterates over the input array, where each node has a p_node attribute (pointer to its parent).
 * Constructs a new array containing only nodes where p_node == nullptr (i.e., no parent).
 */
template <typename NodeType, typename T>
__global__ void getRootNodes(const NodeType* d_nodesArray, int numNodes,
                                NodeType* d_rootsArray){

    int idx = blockIdx.x * blockDim.x + threadIdx.x;
     if (idx < numNodes){
        if (d_nodesArray[idx].p_node == nullptr) {
            d_rootsArray[idx] = d_nodesArray[idx];
        }
     }
};

/**
 * @brief Given an array of nodes, checks if there are any nodes without a parent, 
 * referred to as orphan nodes. Each thread in each block checks its assigned node.
 * @param N The number of nodes.
 * @param status A boolean flag to indicate if an orphan node is found.
 */
template <typename NodeType, typename T>
__global__ void hasOrphanNodes(const NodeType* d_nodesArray, int N, bool* status) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < N) {
        if (d_nodesArray[idx].p_node == nullptr) {
            *status = true;
        }
    }
}


/**
 * @class AstarPlanner base class 
 * divide the datset into chunks of max 1024 node, maxium , compute the best node 
 * into, each chunk on a single grid block, each block , writes the best node 
 * (wich is the closed to target), to the global device memory,each chunk has a root node 
 * and a sucessor the sucessor parent points to the root,and the cost function , the chunk 
 * sum_cost of root is 0 and for the child is the distance btewwen the both , 
 * start with the chunk wich have the mimum cost function, (p_sum + heursitic) 
 * wich is assumed to be the final chunk that have the goal node, named the endChunk,
 * now we sav the nodes into a new_array include the start and gaolnode
 * and the chunks 2-nodes , we llok fro the trajectory that pass from all thes points ,
 * with minium path cost, so, we allocte this array on the device global meomry, and 
 * for a single block each thread compute, 
 * 1 < chunk_size < 1024, each thred within the same only block will compute a possible 
 * combinition, each thred will loop all the nodes in the array, starting from the 
 * endnode, if this has a parent it go throug else , it computes the nereast 
 * neighbood node , and if this also
 * 
 * after computing the chunks and save each chunk nodes (2*NodeType), and the matric:
 * cost to got from chunk root to child chunk + the heuristic % to-go node
 * the number of chunks is : N/threadsPerBlock
 * afther constructing the nodes grid 
 
 */
template <typename NodeType, typename T>
class AstarPlanner
{
public:
    /** 
     * @brief  default constructor, allocate momory for class attributes on 
     * host and device, this function, sets the node number to thredsPerBlock
     * var and allocate memeory   
     */
    HOST_FUN  AstarPlanner();

    /**
     * @brief construct from a user defined point clouds array 
     * init number nodes allocate and fill member attributes , copy them to device
     * and init memebr attributes device vars 
     */
    HOST_FUN AstarPlanner(NodeType& h_nodesArray_,  int numNodes_);

    /** 
     * @brief fill the host and device memory with the nodes from the a point
     * cloud data file .ply
     * @note this method fill only host attributes class mmeber, no device 
     * initlisation when need create a device var and copy from it  
     */
    HOST_FUN AstarPlanner(const std::string& plyFilePath);

    /** 
     * @brief init the host and device memory with random nodes, if the number of
     * ndodes given os diffrent from class alraedy  vars old, ovveride them
     * @param seed : random seed initilization  
     */
    HOST_FUN AstarPlanner(int numNodes_, unsigned int seed);

    /** 
     * @brief set the start  nodes either in 3d or 2d, check if the 
     * node exsits in the h_nodesArray first, if the start are
     * set, it defaulted to the first and last node in the nodes array. 
     * @param startNode_ a construted nodeType for the given node
     the given node should exsit in the point cloud, 
     */
    HOST_FUN void setInitialNode(NodeType* startNode_);

    /**
    @brief set the end node , it should exsit in the point cloud and 
    it should be diffrent from 
     */
    HOST_FUN void setTargetNode(NodeType* endNode_);

    /**
     * @brief  computes he chunks open set array for all chunks in the
     point cloud array , divide into chunks,  compute the knns , reoder 
     by fmin values and append to the h_chunksOpenSetArray , the rest of nodes 
     are divided inti chunks and and reorded, 
     the default size of chunks is the bloc  size    
     */
    HOST_FUN  void computeChunkOpenSet(const int chunkSize_ = threadsPerBlock );

    /**
     * @brief given the the map tree construted, satart from the end node and,
     * given that each node has unique parent, it saves the nodes and copy up to the 
     * start node
     */
    HOST_FUN void computeTrajectory();

    /** 
     * @brief saves a trajectory to pointcloud file .ply using happly library 
     */
    HOST_FUN void writeTrajectory2csv(const std::string outputFilePath);

    /** 
     * @brief read a point cloud data file (now only PLy supported) and fill the gridMap
     * object memebr of the class with nodes consruted from the point cloud
     * @param outputFilePath image fil path to save to 
     */
   HOST_FUN void saveTrajectory2png(const std::string& outputFilePath);

   /**
    * @brief saves the point cloud data array into a 2d, view along z-axis, 
    * to a png file, the setting for this function are taken to default, this function 
    * require the host array or nodes filled, the color used for each node is the default 
    * in r , g ,b arrtibutes in each node to update them use updateNodesColor()
    */
   HOST_FUN void visualize2dPointCloud(const std::string imageFilePath);

   /**
    * @brief this method take an array reprsent a map and debug it in 2d, nodes 
    * are not displayed a unicolor lines between child-parent is ploted
    */
   HOST_FUN void visualize2dTreeMap();

   #ifdef CUASTAR_USE_VTK
    /**
    * @brief given an array of 3d points cloud in node3d type, wich each node
    * has parent-child, viuslize the 3d map using lines, in VTK, displaying
    * points as sphres is not supported, the array is  h_mapArray 
    */
    HOST_FUN void visualize3dTreeMap();

    /**
     * @brief render the computed trajectory i point cloud, with the path in 
     * continus lines in specifc color 
     */
    HOST_FUN void visualize3dTrajectory();

    /**
     * @brief visulize the 3d grid map in only lines with the same color
     * and the trajectory in lines with the same color, diffrent with 
     * the color of the map. 
     */
    HOST_FUN void visualize3dTrajTreeMap();

   #endif

    /** 
     * @brief free all memory on deice or on host memory allocation
     * in the constructor or over functions 
     */
    HOST_FUN ~AstarPlanner();

private:
    int        numNodes;
    int        chunkSize; ///< the chunks size controls the 
    
    NodeType * h_nodesArray;

    NodeType*  h_chunksOpenSetArray; ///< each chunk has it open set wich store nodes by the f from
                                    /// min to max , thes nodes from the chunk knn array, use the computeNodesucessor
                                    ///< to order them, wich compute the node with min f(n) with an array  

    NodeType* h_openSetArray; ///< given the local opensets, of chunks, it gets the roots of each chunk, 
                             ///< form a new arra and compute the gloabl openset, n pallel way 

    NodeType* h_closedSetArray; ///< any node not in openset is in closed set,

    NodeType*   startNode;
    NodeType*   endNode;
   // there is N/chunkSize * size(chunkOpenSet=k)[c1,c2, c3, c4,...] we exclude the start and the to-go to node !
    //                                              |       |
     //                                           a1,a2,..   a1, a2,...
     // N/chunkSize * k , to compute the trajectory path,start with c1 nodes , for a1,...ak, the root node of the chunks 
     // will be the node with minium f(n) , , so take each a1 (for NumChunks ai) and 
     // the fisrt node in trajectory is t1=  min_f_(a1_i i in chunks,)=a_s the second is t2 = min_f_n(a_2i i in chunks all),
     // and at ech time verify if we reach the goal or not 
    NodeType*   h_pathArray; ///< for storing the final computed traj on host , 
    NodeType*   d_pathArray; ///< ..... on device;


    /** 
     * @brief Check wahat ever we get into the goal node or not,
     * using a threshold value esplion for proximity checking.
     */
    HOST_DEVICE_FUN bool isGoalReached(const NodeType* n, const T eps);

    HOST_FUN void computeOpenSetArray();
};

#ifdef CUASTAR_USE_TYPEDEF
    typedef AstarPlanner<Node2d<float>,  float>  AstarPlanner2dFloat;
    typedef AstarPlanner<Node2d<double>, double> AstarPlanner2dDouble;
    typedef AstarPlanner<Node3d<float>,  float>  AstarPlanner3dFloat;
    typedef AstarPlanner<Node3d<double>, double> AstarPlanner3dDouble;
#endif

#ifdef CUASTAR_IMPLEMENTATION 

    template <typename NodeType, typename T>
    HOST_FUN AstarPlanner<NodeType, T>::AstarPlanner(){

        numNodes = threadsPerBlock; 
        cudaError_t err = cudaMalloc((void**)&d_nodesArray, numNodes*sizeof(NodeType));
        cudaMalloc((void**)&d_pathArray, numNodes * sizeof(NodeType));
        h_nodesArray = new NodeType[numNodes];  
        CUDA_CHECK_ERROR(err);
    }

    template <typename NodeType, typename T>
    HOST_FUN AstarPlanner<NodeType, T>::AstarPlanner(NodeType& h_nodesArray_, int numNodes_){

        numNodes = numNodes_;
        h_nodesArray = new NodeType[numNodes];
        memcpy(h_nodesArray, h_nodesArray_, numNodes * sizeof(NodeType));
        cudaError_t err1 = cudaMalloc((void**)&d_nodesArray, numNodes * sizeof(NodeType));
        cudaError_t err2 = cudaMalloc((void**)&d_pathArray, numNodes * sizeof(NodeType));
        CUDA_CHECK_ERROR(err1);
        CUDA_CHECK_ERROR(err2);
        cudaMemcpy(d_nodesArray, h_nodesArray, numNodes*sizeof(NodeType), cudaMemcpyHostToDevice);
    }

    template <typename NodeType, typename T>
    HOST_FUN AstarPlanner<NodeType, T>::AstarPlanner(int numNodes_,unsigned int seed){

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
    };

    template <typename NodeType, typename T>
    HOST_FUN AstarPlanner<NodeType, T>::AstarPlanner(const std::string& plyFilePath) {

        numNodes = static_cast<int>(getPlyPointNum(plyFilePath));
        h_nodesArray = new NodeType[numNodes];

        if (!isPlyValid(plyFilePath)) {
            LOG_MESSAGE(ERROR,"Invalid PLY file: '%s'", plyFilePath.c_str());
        }
        happly::PLYData plyIn(plyFilePath);
        std::vector<std::array<double, 3>> vertices = plyIn.getVertexPositions();
        std::vector<std::array<unsigned char, 3>> colors = plyIn.getVertexColors();
        
        for (int i = 0; i < numNodes; ++i) {
            try {
                const auto& vertex = vertices[i];
                const auto& color = colors[i];
                NodeType n(static_cast<T>(vertex[0]), static_cast<T>(vertex[1]), 
                        static_cast<T>(vertex[2]));
                n.r = static_cast<int>(color[0]);
                n.g = static_cast<int>(color[1]);
                n.b = static_cast<int>(color[2]);

                h_nodesArray[i] = n;
            } catch (const std::exception& e) {
                LOG_MESSAGE(ERROR,"An Error Occured: '%s", e);
            }
        }
    }

    template <typename NodeType, typename T>
    HOST_FUN void AstarPlanner<NodeType, T>::setInitialNode(NodeType* startNode_) {
        cudaError_t err;

        // Allocate and initialize device memory for status
        int* d_startNodeExists;
        err = cudaMalloc((void**)&d_startNodeExists, sizeof(int));
        if (err != cudaSuccess) {
            std::cerr << "Error allocating memory for d_startNodeExists: " << cudaGetErrorString(err) << std::endl;
            return;
        }
        
        err = cudaMemset(d_startNodeExists, 0, sizeof(int)); // Initialize status to 0 (false)
        if (err != cudaSuccess) {
            std::cerr << "Error setting memory for d_startNodeExists: " << cudaGetErrorString(err) << std::endl;
            cudaFree(d_startNodeExists);
            return;
        }

        // Allocate memory for the nodes array on the device
        NodeType* d_nodesArray;
        err = cudaMalloc((void**)&d_nodesArray, numNodes * sizeof(NodeType));
        if (err != cudaSuccess) {
            std::cerr << "Error allocating memory for d_nodesArray: " << cudaGetErrorString(err) << std::endl;
            cudaFree(d_startNodeExists);
            return;
        }

        // Copy host nodes array to device memory
        err = cudaMemcpy(d_nodesArray, h_nodesArray, numNodes * sizeof(NodeType), cudaMemcpyHostToDevice);
        if (err != cudaSuccess) {
            std::cerr << "Error copying memory to d_nodesArray: " << cudaGetErrorString(err) << std::endl;
            cudaFree(d_nodesArray);
            cudaFree(d_startNodeExists);
            return;
        }

        // Calculate grid and block size
        int blocksPerGrid = (numNodes + threadsPerBlock - 1) / threadsPerBlock;

        // Allocate memory for the start node on the device
        NodeType* d_startNode;
        err = cudaMalloc((void**)&d_startNode, sizeof(NodeType));
        if (err != cudaSuccess) {
            std::cerr << "Error allocating memory for d_startNode: " << cudaGetErrorString(err) << std::endl;
            cudaFree(d_nodesArray);
            cudaFree(d_startNodeExists);
            return;
        }

        // Copy start node to device memory
        err = cudaMemcpy(d_startNode, startNode_, sizeof(NodeType), cudaMemcpyHostToDevice);
        if (err != cudaSuccess) {
            std::cerr << "Error copying memory to d_startNode: " << cudaGetErrorString(err) << std::endl;
            cudaFree(d_nodesArray);
            cudaFree(d_startNodeExists);
            cudaFree(d_startNode);
            return;
        }

        // Launch the kernel
        checkNodeExsist<NodeType, T><<<blocksPerGrid, threadsPerBlock>>>(d_nodesArray, numNodes, d_startNode, d_startNodeExists);
        err = cudaGetLastError();
        if (err != cudaSuccess) {
            std::cerr << "Error launching kernel: " << cudaGetErrorString(err) << std::endl;
            cudaFree(d_nodesArray);
            cudaFree(d_startNodeExists);
            cudaFree(d_startNode);
            return;
        }

        // Synchronize the device
        err = cudaDeviceSynchronize();
        if (err != cudaSuccess) {
            std::cerr << "Error synchronizing device: " << cudaGetErrorString(err) << std::endl;
        }

        // Copy the result from device to host
        int h_startNodeExists = 0;
        err = cudaMemcpy(&h_startNodeExists, d_startNodeExists, sizeof(int), cudaMemcpyDeviceToHost);
        if (err != cudaSuccess) {
            std::cerr << "Error copying memory from d_startNodeExists: " << cudaGetErrorString(err) << std::endl;
            cudaFree(d_nodesArray);
            cudaFree(d_startNodeExists);
            cudaFree(d_startNode);
            return;
        }

        // Print the result
        std::cout << "Node existence status: " << (h_startNodeExists ? "True" : "False") << std::endl;

        // Clean up
        cudaFree(d_startNodeExists);
        cudaFree(d_nodesArray);
        cudaFree(d_startNode);
    }

    template <typename NodeType, typename T>
    HOST_FUN void AstarPlanner<NodeType, T>::setTargetNode(NodeType* endNode_) {
     
        if (endNode_->isEqual(startNode)) {
            throw std::runtime_error("Target node should not match start node");
            return;
        }
        int* d_endNodeExists;
        cudaMalloc((void**)&d_endNodeExists, sizeof(int));
        cudaMemset(d_endNodeExists, 0, sizeof(int)); 

        NodeType* d_nodesArray;
        cudaMalloc((void**)&d_nodesArray, numNodes * sizeof(NodeType));
        cudaMemcpy(d_nodesArray, h_nodesArray, numNodes * sizeof(NodeType), cudaMemcpyHostToDevice);

        int blocksPerGrid = (numNodes + threadsPerBlock - 1) / threadsPerBlock;

        NodeType* d_endNode;
        cudaMalloc((void**)&d_endNode, sizeof(NodeType));
        cudaMemcpy(d_endNode, endNode_, sizeof(NodeType), cudaMemcpyHostToDevice);
        
        checkNodeExsist<NodeType, T><<<blocksPerGrid, threadsPerBlock>>>(d_nodesArray, numNodes,
                                                                 d_endNode, d_endNodeExists);

        int h_endNodeExists = 0;
        cudaMemcpy(&h_endNodeExists, d_endNodeExists, sizeof(int), cudaMemcpyDeviceToHost);
        cudaFree(d_startNodeExists);
        cudaFree(d_nodesArray);
        cudaFree(d_startNode);
    }

    template<typename NodeType, typename T>
    HOST_FUN  void AstarPlanner<NodeType, T>::computeChunkOpenSet(const int chunkSize_ = threadsPerBlock){

        int chunksNum = numNodes / threadsPerBlock ;
        int blocksNum =  (numNodes + threadsPerBlock - 1) / threadsPerBlock;

        NodeType* d_chunksOpenSetArray;
        cuMalloc(&d_chunksOpenSetArray,numNodes* sizeof(NodeType));

        for (int i = 0; i < chunksNum; i++){

            // get the chunk elemnts 
            h_chunkNodes = 

            enumerationSortNodes<NodeType,T><<<blocksNum,threadsPerBlock>>>(d_chunkNodesArray,threadsPerBlock,1,d_chunkSortedX);
            enumerationSortNodes<NodeType,T><<<blocksNum,threadsPerBlock>>>(d_chunkNodesArray,threadsPerBlock,2,d_chunkSortedY);
            enumerationSortNodes<NodeType,T><<<blocksNum,threadsPerBlock>>>(d_chunkNodesArray,threadsPerBlock,3,d_chunkSortedZ); 

            computeKnnNodes<NodeType,T><<<blocksNum,threadsPerBlock>>>(d_chunkSortedX, d_chunkSortedY, d_chunkSortedZ,
                                d_endNode, numNodes, int k, int range, 
                                NodeType* d_kNodes);

        }


        cudaMemcpy(&d_chunksOpenSetArray,h_chunksOpenSetArray,numNodes*sizeof(NodeType),cudaMemcpyDeviceToHost);
 
    };

    template<typename NodeType, typename T>
    HOST_FUN void AstarPlanner<NodeType, T>::computeTrajectory(){

        h_pathArray = new NodeType[numNodes];
        cuMalloc(&d_pathArray, sizeof(NodeType));
    }


    template<typename NodeType, typename T>
    HOST_FUN void AstarPlanner<NodeType, T>::writeTrajectory2csv(const std::string outputFilePath){

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
            array2PointCloudImg<NodeType>(h_nodesArray, numNodes, outputFilePath.c_str(),800, 0.001);
        } catch (const fs::filesystem_error& e) {
            std::cerr << "Filesystem error: " << e.what() << '\n';
        } catch (const std::exception& e) {
            std::cerr << "Error: " << e.what() << '\n';
        }
    };

    template <typename NodeType, typename T>
    HOST_DEVICE_FUN bool  AstarPlanner<NodeType,T>::isGoalReached(const NodeType* n, const T eps){
        if (*n == endNode) {
            return true;
        }
        return static_cast<T>(n->distanceTo(endNode)) < eps;
    };

    template <typename NodeType, typename T>
    HOST_FUN void AstarPlanner<NodeType,T>::visualize2dPointCloud(const std::string imageFilePath){

            namespace fs = std::filesystem;
            fs::path outputpath(imageFilePath);
            std::cout << "number of nodes " << numNodes << std::endl;
            if(fs::exists(outputpath)){
                fs::remove(outputpath);  
            }
            array2PointCloudImg<NodeType>(h_nodesArray, numNodes, imageFilePath.c_str(),800, 0.001);
    }

    template <typename NodeType, typename T>
    HOST_FUN AstarPlanner<NodeType, T>::~AstarPlanner(){

        delete[] h_nodesArray;
        numNodes  = 0 ; 
        h_nodesArray = nullptr;
    };


    #ifdef CUASTAR_USE_VTK

        template <typename NodeType, typename T>
        HOST_FUN void AstarPlanner<NodeType, T>::visualize3dTreeMap(){

        }

        template <typename NodeType, typename T>
        HOST_FUN void AstarPlanner<NodeType, T>::visualize3dTrajectory(){

        }

        template <typename NodeType, typename T>
        HOST_FUN void AstarPlanner<NodeType, T>::visualize3dTrajTreeMap(){

        }

   #endif
#endif // CUASTAR_IMPLEMENTATION


#ifdef CUASTAR_USE_VTK

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
#endif // CUASTAR_USE_VTK
#endif // CUASTAR_HPP