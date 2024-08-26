/**
 * @file utils.hpp
 * group all Input -ouput , data form sofwtare, using third party libs,
 * for point cloud data refre :https://paulbourke.net/dataformats/ply/
 * cuRobotics support only .ply files, thirdparty lib used is happly
 * https://github.com/nmwsharp/happly
 * @note all of the IO functions are host functions 
 * ! no parallization using cuda for PLY data handling , only mathematics 
 * Author : Wissem Chiha
 */
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <cstdint>
#include <fstream>
#include <stdexcept>
#include "../extern/happly/happly.h"
#include "../extern/stb/stb_image.h"
#include "../extern/stb/stb_image_write.h"

#ifdef _WIN32
  #include <windows.h>
#endif
#if defined(__CUDACC__)
    #define HOST_FUN __host__
    #define DEVICE_FUN __device__
    #define HOST_DEVICE_FUN __host__ __device__
    #include <cuda_runtime.h>
    #include <math_constants.h>
#else
    #define HOST_FUN 
    #define DEVICE_FUN 
    #define HOST_DEVICE_FUN 
    #include <vector>
    #include <cmath>
    #include <string>
    #include <corecrt_math_defines.h>
    #include <algorithm>
#endif
#ifdef _DEBUG_
  #include "../extern/loguru/loguru.hpp"
  #include "../extern/loguru/loguru.cpp"
#endif
#ifdef __GNUC__
	#pragma GCC system_header
#endif
#include "common.hpp"

HOST_DEVICE_FUN double wrap2pi(double angle){
  return fmod(fmod(angle+CUDART_PI,2*CUDART_PI)-2*CUDART_PI,
  2*CUDART_PI)+CUDART_PI;
  };

namespace IO {

/** @brief Check if a given .PLY file path exists or not */
  HOST_FUN bool isValid(const std::string PLYfilePath){
    try {
      happly::PLYData plyIn(PLYfilePath);
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

/** @brief Return the number of points stored in a .PLY file */
HOST_FUN int getPLYNumPts(const std::string PLYfilePath){
  if (isValid(PLYfilePath)){
    happly::PLYData plyIn(PLYfilePath);
    std::vector<std::array<double, 3>> vertices = plyIn.getVertexPositions();
    return vertices.size();
  }
};

/** @brief Get a Node3d object from a PLY file using index idx */
HOST_FUN internal::Node3d getPLYNode3d(const std::string& PLYfilePath,
                                    const int32_t idx) {
    if (isValid(PLYfilePath)) {
        happly::PLYData plyIn(PLYfilePath);
        std::vector<std::array<double, 3>> vertices = plyIn.getVertexPositions();
        if (idx >= 0 && idx < static_cast<int32_t>(vertices.size())) {
            const auto& vertex = vertices[idx];
            return internal::Node3d(vertex[0], vertex[1], vertex[2]);
        } else {
            #ifdef _DEBUG_
            LOG_F(ERROR, "Index out of range in PLY file '%s': requested idx = %d, max idx = %d",
                  PLYfilePath.c_str(), idx, static_cast<int32_t>(vertices.size()) - 1);
            #endif
            throw std::out_of_range("Index out of range in PLY file vertices.");
        }
  }
};

/** @brief Check if a given Node3d exists in a .PLY file */
HOST_FUN bool exsistNode3dPLY (const std::string& PLYfilePath, internal::Node3d node){
  if (isValid(PLYfilePath)) {
    happly::PLYData plyIn(PLYfilePath);
    std::vector<std::array<double, 3>> vertices = plyIn.getVertexPositions();
    for (const auto& vertex : vertices) {
      internal::Node3d vertexNode(vertex[0], vertex[1], vertex[2]);
      if (node.isEqual(vertexNode)) {
            return true;
      }
    }
  }
  return false;
}

/** @brief Write a trajectory described by an array of Node3d[] to a PLY file
 * If the file exists, output a warning using logcpp in _DEBUG_ mode and 
 * replace the file with the new one */
HOST_FUN void writeTraj2PLY(const internal::Node3d* nodes,const int32_t numNodes,
                 const std::string& PLYfilePath)
{
    std::ifstream file(PLYfilePath);
    if (file.good()) {
        #ifdef _DEBUG_
          LOG_WARN("File already exists: " + PLYfilePath + ". It will be replaced.");
        #endif
        file.close(); 
    }
    happly::PLYData plyOut(PLYfilePath);

    std::vector<std::array<double, 3>> vertexPositions;
    vertexPositions.reserve(numNodes); 
    for (size_t i = 0; i < numNodes; ++i) {
        vertexPositions.emplace_back(nodes[i].x, nodes[i].y, nodes[i].z);
    }
    plyOut.addVertexPositions(vertexPositions);
    plyOut.write(PLYfilePath, happly::DataFormat::ASCII);
    #ifdef _DEBUG_
      LOG_INFO("Successfully wrote trajectory to PLY file: " + PLYfilePath);
    #endif
}

/** @brief Write a trajectory described by an array of nodes to a CSV file
 * @note This function does not rely on the rapidcsv third-party library */
HOST_FUN void writeTraj2CSV(const internal::Node3d* nodes, int32_t numNodes, 
                const std::string& csvFilePath){

    std::ofstream outFile(csvFilePath);
    if (!outFile.is_open()) {
        std::cerr << "Error opening file: " << csvFilePath << std::endl;
        return;
    }
    outFile << "x,y,z" << std::endl;
    for (int32_t i = 0; i < numNodes; ++i) {
        outFile << nodes[i].x << ','
                << nodes[i].y << ','
                << nodes[i].z << std::endl;
    }
    outFile.close();
    #ifdef _DEBUG_
      LOG_INFO("Successfully wrote trajectory to CSV file:" + csvFilePath);
    #endif
}

/** @brief Construct an array of Node3d objects from a PLY file using happly lib
 * @return An array of elements of type Node3d 
 * This function should be called from host or device 
 * @todo Write a similar function for CSV extraction */
HOST_FUN void getPLYNode3dMap(const std::string PLYfilePath, internal::Node3d* nodes){

  if (isValid(PLYfilePath)){
    happly::PLYData plyIn(PLYfilePath);
    std::vector<std::array<double,3>> vertexPositions = plyIn.getVertexPositions();
    
  for (size_t i = 0; i < vertexPositions.size(); ++i) {
      nodes[i].x = vertexPositions[i][0];
      nodes[i].y = vertexPositions[i][1];
      nodes[i].z = vertexPositions[i][2];
  }
}}

/** @brief Construct an array of Node2d objects from a PLY file using happly lib 
 * The file is assumed to contain 3D points, ignoring the z component */
HOST_FUN void getPLYNode2dMap(const std::string PLYfilePath, internal::Node2d* nodes){
  if (isValid(PLYfilePath)){
    happly::PLYData plyIn(PLYfilePath);
    std::vector<std::array<double,3>> vertexPositions = plyIn.getVertexPositions();
    
  for (size_t i = 0; i < vertexPositions.size(); ++i) {
      nodes[i].x = vertexPositions[i][0];
      nodes[i].y = vertexPositions[i][1];
  }}
};

/** @brief Write an array of 2D point cloud nodes in image format 
 * Width, height: dimensions of the image 
 * @note All nodes are assumed to have the same default colors! Override this 
 * for nodes extracted from RGBA PLY files */
HOST_FUN void writeNode2d2Img(const internal::Node2d* nodes,const int32_t numNodes, 
                  const std::string& filePath, int width, int height, 
                  double radiusRatio = 0.01){
  try
  {
    std::vector<std::pair<int, int>>    centers ;
    std::vector<std::array<uint8_t, 3>> colors ;
    centers.reserve(numNodes);
    colors.reserve(numNodes);
    for (size_t i = 0; i < numNodes; i++)
    {
      centers.push_back({nodes[i].x,nodes[i].y});
      colors.push_back({0,0,0});
    }
    Img::save2dNodesImage(filePath, width, height, centers, colors, radiusRatio);
  }
  catch(const std::runtime_error& e)
  {
    #ifdef _DEBUG_
      LOG_F(ERROR, "Error Writing Nodes to file: %s", e.what());
    #else
      std::cerr << e.what() << '\n';
    #endif
  }
}

/** @brief Write an array of 3D point cloud nodes in image format 
 * Width, height: dimensions of the image 
 * @note All nodes are assumed to have the same default colors! Override this 
 * for nodes extracted from RGBA PLY files 
 * This view is the projection of the point cloud along the z axis 
 * @todo Change the writing view using glm projection matrices 
 * to map from 3D (x, y, z) to a given perspective 2D (x, y), e.g., 
 * change the projection axis 
 * and view direction */
HOST_FUN void writeNode3d2Img(const internal::Node3d* nodes,const int32_t numNodes, 
                  const std::string& filePath, int width, int height, 
                  double radiusRatio = 0.01){
  try
  {
    std::vector<std::pair<int, int>>    centers ;
    std::vector<std::array<uint8_t, 3>> colors ;
    centers.reserve(numNodes);
    colors.reserve(numNodes);
    for (size_t i = 0; i < numNodes; i++)
    {
      centers.push_back({nodes[i].x,nodes[i].y});
      colors.push_back({0,0,0});
    }
    Img::save2dNodesImage(filePath, width, height, centers, colors, radiusRatio);
  }
  catch(const std::runtime_error& e)
  {
    #ifdef _DEBUG_
      LOG_F(ERROR, "Error Writing Nodes to file: %s", e.what());
    #else
      std::cerr << e.what() << '\n';
    #endif
  }
}

#ifdef _DEBUG_
  /** @brief Only in debug mode, print all information of a .PLY file using happly lib */
  HOST_FUN void verbosePLY(const std::string& PLYfilePath) {
      happly::PLYData plyIn(PLYfilePath);
      std::string format = plyIn.getFileFormat() == happly::DataFormat::ASCII ? "ASCII" : "Binary";
      std::cout << "PLY File: " << PLYfilePath << "\nFormat: " << format << std::endl;
      std::vector<std::string> comments = plyIn.getComments();
      if (!comments.empty()) {
          std::cout << "Comments:" << std::endl;
          for (const std::string& comment : comments) {
              std::cout << "  - " << comment << std::endl;
          }
      } else { std::cout << "No comments found in the PLY file." << std::endl;}
      std::vector<std::string> elementNames = plyIn.getElementNames();
      std::cout << "Elements found in the PLY file:" << std::endl;
      for (const std::string& elementName : elementNames) {
          size_t count = plyIn.getElementCount(elementName);
          std::cout << "  - " << elementName << " (Count: " << count << ")" << std::endl;
      }
  }
#endif
}; // namespace IO

namespace Img{

/** @brief Draw a 2D sphere (plain circle) given radius, center, and color */
HOST_FUN void draw2dNode(uint8_t* image, int width, int height, int centerX, 
                    int centerY, int radius, uint8_t r, uint8_t g, uint8_t b){

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int dx = x - centerX;
            int dy = y - centerY;
            int distanceSquared = dx * dx + dy * dy;
            if (distanceSquared <= radius * radius) {
                int offset = (y * width + x) * 3;
                image[offset] = r;
                image[offset + 1] = g;
                image[offset + 2] = b;
            }
        }
    }
}

/** @brief Draw a sample of point cloud nodes as 2D colored circles */
HOST_FUN void draw2dNodes(uint8_t* image, int width, int height, 
                      const std::vector<std::pair<int, int>>& centers, 
                      const std::vector<std::array<uint8_t, 3>>& colors, 
                      double radiusRatio = 0.01) {

    int radius = static_cast<int>(radiusRatio * std::min(width, height)); 
    for (size_t i = 0; i < centers.size(); ++i) {
        int centerX = centers[i].first;
        int centerY = centers[i].second;
        uint8_t r = colors[i][0];
        uint8_t g = colors[i][1];
        uint8_t b = colors[i][2];
        draw2dNode(image, width, height, centerX, centerY, radius, r, g, b);
    }
}

/** @brief Draw and save a given 2D point cloud of nodes */
HOST_FUN void save2dNodesImage(const std::string& filePath, int width, int height, 
                    const std::vector<std::pair<int, int>>& centers,
                    const std::vector<std::array<uint8_t, 3>>& colors, 
                    double radiusRatio = 0.01) {

    std::vector<uint8_t> image(width * height * 3, 255); 
    draw2dNodes(image.data(), width, height, centers, colors, radiusRatio);
    stbi_write_png(filePath.c_str(), width, height, 3, image.data(), width * 3);
};
}; //namespace Img