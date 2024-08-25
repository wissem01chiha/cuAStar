/**
 * @file utils.hpp
 * group all Input -ouput , data form sofwtare, using third party libs,
 * for point cloud data refre :https://paulbourke.net/dataformats/ply/
 * cuRobotics support only .ply files, thirdparty lib used is happly
 * https://github.com/nmwsharp/happly
 */
#include <cstdint>
#include "../extern/happly/happly.h"
#include "../extern/stb/stb_image.h"
#include "../extern/stb/stb_image_write.h"

#if defined(ENABLE_CUDA_ARCH) && defined(ENABLE_GLM)
    #warning "ENABLE_CUDA_ARCH and ENABLE_GLM "
#elif !defined(ENABLE_CUDA_ARCH) && !defined(ENABLE_GLM)
    #error "ENABLE_CUDA_ARCH or ENABLE_GLM. Required."
#elif defined(ENABLE_CUDA_ARCH)
    #include <cuda_runtime.h>
    #include <math_constants.h>
#elif defined(ENABLE_GLM)
    #include <glm/glm.hpp>
    #include <glm/gtc/constants.hpp>
    #include <glm/gtc/matrix_inverse.hpp> 
    #include <vector_double2.hpp>
    #include <corecrt_math_defines.h>
#endif
#ifdef _DEBUG_
  #include "../extern/loguru/loguru.hpp"
  #include "../extern/loguru/loguru.cpp"
#endif
#ifdef __GNUC__
	#pragma GCC system_header
#endif
#include "common.hpp"

#if defined(ENABLE_CUDA_ARCH)
  __device__ double wrap2pi(double angle){
    return fmod(fmod(angle+CUDART_PI,2*CUDART_PI)-2*CUDART_PI,
    2*CUDART_PI)+CUDART_PI;
  };
#elif defined(ENABLE_GLM)
  double wrap2pi(double angle) {
      return std::fmod(std::fmod(angle +M_PI,2*M_PI)-2*M_PI,2*M_PI)+M_PI;
  };
#endif

namespace IO {

int32_t getPLYNumPts(){

};

/** @brief get a Node3d object from a PLY file refrenced by index idx */
internal::Node3d getPLYNode3d(const std::string PLYfilePath, const int32_t idx){

  happly::PLYData plyIn(PLYfilePath);
  std::vector<std::array<double, 3>> getVertexPositions(std::string vertexElementName = "vertex");
};

/** @brief write a Node3d object to a PLY file  */
void writeNode3d2PLY(const internal::Node3d& node,const std::string PLYfilePath)
{
  happly::PLYData plyIn(PLYfilePath);
  std::vector<std::array<unsigned char, 3>> getVertexColors(std::string vertexElementName = "vertex");
};

/** @brief construct an array of Node3d Objects form a PLY file */
void getPLYNode3dMap(const std::string PLYfilePath){

};

/** @brief same of 2 d Nodes objects */
void getPLYNode2dMap(){

};

/** @brief write an array of 2d nodes in image format */
void writeNode2d2Img(const internal::Node2d nodes_array[]){
  #ifdef _DEBUG_

  #endif
}

/** @brief write an array of 2d nodes in image format */
void writeNode3d2Img(const internal::Node2d nodes_array[]){
  #ifdef _DEBUG_

  #endif
}

#ifdef _DEBUG_
  /** @brief only in debug mode  */
  void vorbosePLY(const std::string PLYfilePath){
      std::vector<std::string> PLYData::comments ;
 
  };
#endif
}; // namespace IO