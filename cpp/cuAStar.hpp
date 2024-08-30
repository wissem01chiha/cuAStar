/**
 * @file a_start_planner.hpp
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
#define CUASTAR_IMPLEMENTATION 1
#define __CUDACC__
#if __cplusplus > 201703L
    #warning "C++ 17 Required, potential errors!"
#endif
#ifdef _MSC_VER
    #if _MSC_VER < 1910
        #warning "MSVC 2019 or later is officially supported"
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
#if defined(__CUDACC__)
    #define HOST_FUN __host__
    #define DEVICE_FUN __device__
    #define HOST_DEVICE_FUN __host__ __device__
    #define GLOBAL_FUN __global__
    #include <cuda_runtime.h>
    #include <curand_kernel.h>
    #include <math_constants.h>
#else
    #define HOST_FUN 
    #define DEVICE_FUN 
    #define HOST_DEVICE_FUN
    #define GLOBAL_FUN 
    #include <vector>
    #ifdef _OPENMP
        #include <omp.h>
    #endif
    #include <cmath>
    #include <string>
    #include <chrono>
    #include <type_traits>
    #include <stdexcept>
    #include <filesystem>
    #include <iostream>
    #include <array>
    #include <algorithm> 
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
  #include <iostream>
  #define STB_IMAGE_WRITE_IMPLEMENTATION 1
  #include "../extern/loguru/loguru.hpp"
  #include "../extern/loguru/loguru.cpp"
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

/** 
 * @brief   Base class for 2D nodes representation
 * @tparam  T Numeric type for the sum_cost (e.g., T, float)
 * @param x : x-coordinate in the grid
 * @param y : y-coordinate in the grid
 * @param sum_cost : cumulative cost from the start node to the current node
 * @param p_node : pointer to the parent node
 */
template <typename T>
class Node2d {
public:
    T x;           
    T y;         
    T sum_cost;   
    Node2d* p_node; 

    uint8_t r = 125;
    uint8_t g = 100;
    uint8_t b = 120;      

    /** @brief default constructor for the Node2d class */
    HOST_DEVICE_FUN Node2d(){
        x= T(0);
        y= T(0);
        sum_cost = T(0);
        p_node = nullptr;
    }

    /** @brief Constructor for the Node2d class */
    HOST_DEVICE_FUN Node2d(T x_, T y_, T sum_cost_ = 0,Node2d* p_node_ = nullptr){
        x= x_;
        y= y_;
        sum_cost = sum_cost_;
        p_node = p_node_;
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
    };
    /** @brief constract a random node, postion bounds is in [0,1] */
    HOST_FUN Node3d(unsigned int  seed){

        T* d_x;
        T* d_y;
        T* d_z;
        cudaMalloc((void**)&d_x, sizeof(T));
        cudaMalloc((void**)&d_y, sizeof(T));
        cudaMalloc((void**)&d_z, sizeof(T));

        rand<<<1,1>>>(seed, d_x);
        rand<<<1,1>>>(seed, d_y);
        rand<<<1,1>>>(seed, d_z);
        cudaMemcpy(&x, d_x, sizeof(T), cudaMemcpyDeviceToHost);
        cudaMemcpy(&y, d_y, sizeof(T), cudaMemcpyDeviceToHost);
        cudaMemcpy(&z, d_z, sizeof(T), cudaMemcpyDeviceToHost);
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
};
typedef Node2d<double> Node2dDouble;  
typedef Node2d<float>  Node2dFloat;   
typedef Node3d<double> Node3dDouble; 
typedef Node3d<float>  Node3dFloat;  


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
    HOST_FUN saveTrajectory2csv(const std::string outputFilePath);

    /** 
     * @brief read a point cloud data file (now only PLy supported) and fill the gridMap
     * object memebr of the class with nodes consruted from the point cloud 
     */
   HOST_FUN void saveTrajectory2png(const std::string outputFilePath);

private:

    size_t    numNodes;
    NodeType* d_nodesArray; 
    NodeType* h_nodesArray;

    NodeType*   startNode;
    NodeType*   endNode;

    NodeType*   h_pathArray;
    NodeType*   d_pathArray;

    int threadsPerBlock = 256;

    /** @return true if the node exsits in device nodes array*/
    GLOBAL_FUN void checkNodeExsist(const NodeType* nodeToCheck, bool* status);

    /** 
     * @brief heuristic function which computes the estimated cost to the goal 
     * node, starting from the current given node, it simply the euclidian distances
     * the manthetn distance could be used 
     */
    DEVICE_FUN void computeHeuristic(NodeType* Node, T* hfun);

    /** 
     * @brief get the cost of the a path computed the path is given as 
     * an array of variables  each 
     * one of NodeType  f(n) = g(n) + h(n) 
     */
    GLOBAL_FUN void computeTrajectoryCost(const NodeType* node,T* p_cost_);

    /** 
     * @return true if this node is the goal node, return true if we resach 
     * the exact node or true if we are in a very close node, 
     * within a fixed thresh hold 
     */
    HOST_DEVICE_FUN bool isGoalReached(NodeType* n, const T eps=static_cast<T>(1e-6));

    /**
     * @brief given a current node  at a gievn step it computes the 
     * potenial sucessors and if it, ther is no predified motion model!,
     * so each thread will process a sucessor and a globl synchronization will
     * filter the best sucessor (they compute the f(n) valeur for each sucessor )
     * hwo get this sucessors in uncontrained motion model, we assume that
     * the points in the file are not initially sorted, the secah is in 
     * d_nodesArray, at each time update the node p_node and sum_cost 
    */
    GLOBAL_FUN void computeSucessor(NodeType* node);


    /** 
     * @brief smooth trajecory after computetion loop the nodes computed [N1, N2, N3]
     * if there is a jerk in node N2 it will be elimated of rescaled by recomputed 
     * a window appreoch could be used !
     */
    GLOBAL_FUN void smoothTrajectory();

    /** @brief interpolate the given nodes to get a continus curve trajectory  */


protected:

    /** 
     * @brief free all memory on deice or on host , free all 
     * class device or host varaible , qqsoit 
     */
    HOST_FUN ~AstarPlanner();
};

typedef AstarPlanner<Node2dFloat,  float>  AstarPlanner2dFloat;
typedef AstarPlanner<Node2dDouble, double> AstarPlanner2dDouble;
typedef AstarPlanner<Node3dFloat,  float>  AstarPlanner3dFloat;
typedef AstarPlanner<Node3dDouble, double> AstarPlanner3dDouble;

#ifdef CUASTAR_IMPLEMENTATION 

    template <typename NodeType, typename T>
    HOST_FUN AstarPlanner<NodeType, T>::AstarPlanner(){

        cudaError_t err = cudaMalloc((void**)&d_nodesArray, numNodes*sizeof(NodeType));
        cudaMalloc((void**)&d_pathArray, numNodes * sizeof(NodeType));
        h_nodesArray = new NodeType[numNodes];  
        #ifdef _DEBUG_
            if (err != cudaSuccess) {
                LOG_F(ERROR, "Failed to allocate device memory");
                exit(EXIT_FAILURE);
            }
        #endif
    };

    template <typename NodeType, typename T>
    HOST_FUN void AstarPlanner<NodeType, T>::initRandomNodesArray(size_t numNodes_,unsigned int seed){

        numNodes = numNodes_;
        NodeType* h_nodesArray = new NodeType[numNodes];

        for (size_t i = 0; i < numNodes; ++i) {
            h_nodesArray[i] = NodeType(seed + i); 
        }

        NodeType* d_nodesArray_;
        cudaError_t err = cudaMalloc((void**)&d_nodesArray_, numNodes * sizeof(NodeType));
        #ifdef _DEBUG_
            if (err != cudaSuccess) {
                LOG_F(ERROR, "Failed to allocate device memory");
                exit(EXIT_FAILURE);
            }
        #endif
        cudaMemcpy(d_nodesArray_, h_nodesArray,numNodes*sizeof(NodeType),cudaMemcpyHostToDevice);
        this->d_nodesArray = d_nodesArray_;
    };

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
                NodeType n = getPlyNode<NodeType>(plyFilePath, i);
                h_nodesArray[i] = n;
            } catch (const std::exception& e) {
                #ifdef _DEBUG_
                LOG_F(ERROR, "Exception while getting PLY node at index %zu: %s", i, e.what());
                #endif
                delete[] h_nodesArray;
                cudaFree(d_nodesArray_);
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
    GLOBAL_FUN void AstarPlanner<NodeType, T>::checkNodeExsist(const NodeType* nodeToCheck, 
                                                            bool* status){
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if (idx < numNodes){
            if (d_nodesArray[idx] == *nodeToCheck) {
            *status = true;
        }}
    }

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
    DEVICE_FUN void  AstarPlanner<NodeType, T>::computeHeuristic(NodeType* Node, T* hfun){
        T* hfun = Node.distanceTo(endNode);
    }

    template <typename NodeType, typename T>
    HOST_DEVICE_FUN bool AstarPlanner<NodeType, T>::isGoalReached(NodeType* n,
                                         const T eps=static_cast<T>(1e-6)) const {
    
        if (*n == endNode) {
            return true;
        }
        return static_cast<T>(n->distanceTo(endNode)) < eps;
    }
    
    template <typename NodeType, typename T>
    GLOBAL_FUN void AstarPlanner<NodeType, T>::smoothTrajectory(){

    }

    template <typename NodeType, typename T>
    GLOBAL_FUN void AstarPlanner<NodeType, T>::computeTrajectoryCost(const NodeType* node, T* p_cost_){
        T h_fun;
        computeHeuristic(*node, &h_fun); 
        *p_cost_ = node->sum_cost + h_fun;
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
    }

#endif // CUASTAR_IMPLEMENTATION


/** @brief  Generate a single random float between 0 and 1 */
GLOBAL_FUN void rand(unsigned int seed, float* randomValue) {

    curandState state;
    curand_init(seed, 0, 0, &state);
    *randomValue = curand_uniform(&state);
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
        if constexpr (std::is_same<NodeType, Node2d>::value) {
            return NodeType(static_cast<T>(vertex[0]), static_cast<T>(vertex[1]));
        } else if constexpr (std::is_same<NodeType, Node3d>::value) {
            return NodeType(static_cast<T>(vertex[0]), static_cast<T>(vertex[1]), 
            static_cast<int32_t>(vertex[2]));
        } else {
            #ifdef _DEBUG_
                LOG_F(ERROR, "Unsupported NodeType in PLY file: '%s'", plyFilePath.c_str());
            #endif
            throw std::invalid_argument("Unsupported NodeType.");
        }
    }else {
        #ifdef _DEBUG_
            LOG_F(ERROR, "Index out of range in PLY file '%s': requested idx = %zu, max idx = %zu",
              plyFilePath.c_str(), idx, vertices.size() - 1);
        #endif
        throw std::out_of_range("Index out of range in PLY file vertices.");
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

/** @brief Check if a given .ply file path exists or not */
HOST_FUN bool isPlyValid(const std::string plyFilePath){

    try {
      happly::PLYData plyIn(plyFilePath);
      return true;
      } 
      catch (const std::runtime_error& e) {
          #ifdef _DEBUG_
          LOG_F(ERROR, "Error opening PLY file: %s", e.what());
            #ifdef _WIN32
              MessageBox(NULL, TEXT("Error opening PLY file."), TEXT("File Error"),       
              MB_ICONERROR | MB_OK);
            #endif
          #endif
          return false;
      };
};

/**  @brief Draw and save a given 2D point cloud of nodes as an image, the data
 * is given as a 1d host array of NodeType template objects. */
template <typename NodeType>
HOST_FUN void array2PointCloudImg(const NodeType* h_arrayNodes, size_t numNodes,
                                  const std::string& pngFilePath, int width, int height, 
                                  double radiusRatio = 0.01) {

    std::vector<std::pair<int, int>> centers;
    std::vector<std::array<uint8_t, 3>> colors;
    for (size_t i = 0; i < numNodes; ++i) {
        int x = static_cast<int>(h_arrayNodes[i].x);
        int y = static_cast<int>(h_arrayNodes[i].y);
        centers.emplace_back(x, y);
        std::array<uint8_t, 3> color = {static_cast<uint8_t>(h_arrayNodes[i].r),
                                        static_cast<uint8_t>(h_arrayNodes[i].g),
                                        static_cast<uint8_t>(h_arrayNodes[i].b)};
        colors.push_back(color);
    }
    savePointCloudImage(pngFilePath, width, height, centers, colors, radiusRatio);
}

/**  @brief Draw and save a given 2D points cloud of nodes as an image. */
HOST_FUN void savePointCloudImage(const std::string& filePath, int width, int height, 
                      const std::vector<std::pair<int, int>>& centers,
                      const std::vector<std::array<uint8_t, 3>>& colors, 
                      double radiusRatio = 0.01) {

    std::vector<uint8_t> image(width * height * 3, 255);   
    drawFilledCircles(image.data(), width, height, centers, colors, radiusRatio);
    stbi_write_png(filePath.c_str(), width, height, 3, image.data(), width * 3);
}

/** @brief Draw a sample of point cloud nodes as 2D colored circles */
HOST_FUN void drawFilledCircles(uint8_t* image, int width, int height, 
                       const std::vector<std::pair<int, int>>& centers, 
                       const std::vector<std::array<uint8_t, 3>>& colors, 
                       double radiusRatio = 0.01) {

    int radius = static_cast<int>(radiusRatio * std::min(width, height));
    
    for (size_t i = 0; i < centers.size(); ++i) {
        Circle circle = {{centers[i].first, centers[i].second}, radius};
        Color color = {colors[i][0], colors[i][1], colors[i][2]};
        drawFilledCircle(image, width, height, circle, color);
    }
}

/** @brief Draw a 2D sphere (plain circle) given radius, center, and color. */
HOST_FUN void drawFilledCircle(uint8_t* image, int width, int height, const Circle& circle, 
            const Color& color){

    int minX = std::max(0, circle.center.x - circle.radius);
    int maxX = std::min(width - 1, circle.center.x + circle.radius);
    int minY = std::max(0, circle.center.y - circle.radius);
    int maxY = std::min(height - 1, circle.center.y + circle.radius);

    int radiusSquared = circle.radius * circle.radius;

    for (int y = minY; y <= maxY; ++y) {
        for (int x = minX; x <= maxX; ++x) {
            int dx = x - circle.center.x;
            int dy = y - circle.center.y;
            int distanceSquared = dx * dx + dy * dy;
            if (distanceSquared <= radiusSquared) {
                int offset = (y * width + x) * 3;
                image[offset] = color.r;
                image[offset + 1] = color.g;
                image[offset + 2] = color.b;
            }
        }
    }
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