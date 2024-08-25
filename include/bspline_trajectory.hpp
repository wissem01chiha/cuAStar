/**
 * @file 
 */
#include <cstdint>

#if defined(ENABLE_CUDA_ARCH) && defined(ENABLE_GLM)
    #warning "ENABLE_CUDA_ARCH and ENABLE_GLM "
#elif !defined(ENABLE_CUDA_ARCH) && !defined(ENABLE_GLM)
    #error "ENABLE_CUDA_ARCH or ENABLE_GLM. Required."
#elif defined(ENABLE_CUDA_ARCH)
    #include <cuda_runtime.h>
    #include <cublas_v2.h>
    #include <device_launch_parameters.h>
#elif defined(ENABLE_GLM)
    #include <glm/glm.hpp>
    #include <glm/gtc/constants.hpp>
#endif
#ifdef ENABLE_SSE && !defined(ENABLE_CUDA_ARCH)
  #ifdef _MSC_VER
    #if defined(_M_IX86_FP) && _M_IX86_FP >= 1
      #include <intrin.h>
      #include <xmmintrin.h>
    #endif
  #endif
#endif


#include "utils.hpp"
#include "common.hpp"
#include "math.hpp"

class BsplineTrajectory
{
private:
   
public:
  BsplineTrajectory(/* args */);
  ~BsplineTrajectory();
};

 

