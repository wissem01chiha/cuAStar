/*************************************************************************
	> File Name: a_star.cpp
  SSE/AVX
  NEON: NEON is a SIMD instruction set designed for ARM processors. 
  It is specifically tailored for ARM architectures, which are commonly used in mobile devices, 
  embedded systems, and some edge devices.
  SSE/AVX: Intel CPUs use different SIMD instruction sets,
   primarily SSE (Streaming SIMD Extensions)
  and AVX (Advanced Vector Extensions). 
  These are designed for x86/x86-64 architectures, which are typical
   in desktops, laptops, and servers
 ************************************************************************/

#ifdef _WIN32
  #include <windows.h>
#endif

#ifdef __CUDA_ARCH__
  #include <cuda_runtime.h>
  #include <cublas_v2.h>
#else
  #warning "CUDA Compilation Required !"
#endif

#ifdef USE_SSE
  #ifdef _MSC_VER
    #if defined(_M_IX86_FP) && _M_IX86_FP >= 1
      #include <intrin.h>
      #include <xmmintrin.h>
    #endif
  #endif
#endif

#include <cstdint>
#include "utils.cuh"

