<h1 align="center" style="font-family: Arial, sans-serif;">cuAstar</h1>

 <p align="center">
   <img src ="https://img.shields.io/badge/cuda-12.5-brightgreen?style=flat-square" alt= "cuda version">
    <img src="https://img.shields.io/github/last-commit/wissem01chiha/cuAstar?style=flat-square&logo=github&logoColor=white"
         alt="GitHub last commit">
      <img src="https://img.shields.io/github/stars/wissem01chiha/cuAstar?style=flat-square&logo=github&logoColor=white"   alt="GitHub Stars">
    <a href="https://github.com/wissem01chiha/cuAstar/issues">
    <img src="https://img.shields.io/github/issues/wissem01chiha/cuAstar?style=flat-square&logo=github&logoColor=white"
         alt="GitHub issues">  
    <a href="https://wissem01chiha.github.io/cuAStar/">
    <img src="https://img.shields.io/github/deployments/wissem01chiha/cuAstar/github-pages?style=flat-square&label=deployment&link=https%3A%2F%2Fwissem01chiha.github.io%2FcuAStar%2F"
         alt="deployments">
    <a href="https://twitter.com/intent/tweet?text=Try this Counter-Strike 2 autoexec:&url=https%3A%2F%2Fgithub.com%2FArmynC%2FArminC-AutoExec">
    <img src="https://img.shields.io/github/license/wissem01chiha/cuAstar?style=flat-square&logo=twitter"
       alt="License">
</p>
<h5 align="center">Fast, modern, fully templated, single-file, header-only, parallel implementation of A* trajectory planner on NVIDIA GPUs in point cloud data.</h5>
<p align="center">
  <a href="#installation">Installation</a> •
  <a href="#updating">Updating</a> •
  <a href="#dataset">Dataset</a> •
  <a href="#benchmark">Benchmark</a> •
  <a href="#features">Features</a> •
  <a href="#dependencies">Dependencies</a>•  
  <a href="#wiki">Wiki</a> •
  <a href="#examples">Examples</a> •
  <a href="#to-do">Todo</a> •
  <a href="#contributing">Contributing</a> •
  <a href="#license">License</a>
</p>
      
---
**cuAstar** is a template cuda-based implementation, proposed by me, for generating fast, optimal trajectories within scanned point cloud data. It supports handling and rendering of both 2D and 3D points.

> [!NOTE]  
> This is an experimental version. Trajectory computation time and optimality may not be the best, and optimizations are required. Check the [To-Do](#to-do) section for more information.

## Dependencies

Building the library requires an NVIDIA-capable device with the CUDA Toolkit installed, as well as CMake.

- [CMake](https://cmake.org/download/)
- [NVIDIA GPU Computation Toolkit](https://docs.nvidia.com/cuda/cuda-installation-guide-microsoft-windows/index.html)

The library is designed to maintain a header-only structure, so lightweight header-only libraries are used for data file processing and logging:

- [rapidcsv](https://github.com/d99kris/rapidcsv) for reading and writing CSV trajectory files.
- [happly](https://github.com/nmwsharp/happly) for parsing point cloud `.ply` files.
- [loguru](https://github.com/emilk/loguru) for enhanced logging in debug mode.
- [stb_image](https://github.com/nothings/stb) for reading and writing image formats.

> [!WARNING]   
> The minimum C++ standard required is C++17. Ensure that your compiler supports the standard features.

## Installation

For building platform-specific binaries and test executables:

```shell
git clone https://github.com/wissem01chiha/cuAStar
```

Build the code in Release configuration with default CMake options and flags:

```shell
   mkdir build 
   cd build 
   cmake -G "Visual Studio 17 2022" ..
   cmake --build . --config Release 
```

By default, tests are built when compiling the code:

```shell
   cmake -G "Visual Studio 17 2022"  -DBUILD_TESTS=ON ..
```
Build with examples support:

```shell
   cmake -G "Visual Studio 17 2022"  -DBUILD_EXEMPLES=ON ..
``` 
For developing or integrating the project into your codebase, just download the [cuAstar](include/cuAstar) folder, ensure required libraries are installed, and include the librray with
```cpp
   #define CUASTAR_IMPLEMENTATION 
   #include <cuAstar/cuAstar.hpp>
   ...
```
cuAstar includes internal debugging features which are non-dependent on compiler-specific debugging options. These features can be used with the following macro:

```cpp
   #define CUASTAR_DEBUG 
```

> [!NOTE] 
> When the debug macro is not set, cuAstar uses a default shell logging format, so there is no need to link against the [loguru](https://github.com/emilk/loguru) library.



> [!IMPORTANT]  
> 3D Renaering using VTK is not offically supported, exepriemtal tests are using VTK 9.3.1, ensure building to Release version, enable cuAstar building with VTK support  
> cmake -G "Visual Studio 17 2022" -DUSE_VTK=ON ..
 


> There is no CPU version of the code. work in prgress.
        



<details>
  <summary>Click to expand</summary>
  
see this issue :  [CUDA compile problems on Windows, Cmake error: No CUDA toolset found](https://stackoverflow.com/questions/56636714/cuda-compile-problems-on-windows-cmake-error-no-cuda-toolset-found)
> [!IMPORTANT] 
</details>




## Dataset
we use as a test dataset used for this project is [OpenTrench3D](https://www.kaggle.com/datasets/hestogpony/opentrench3d).

## Environment Setup
To set up your environment, follow these steps:

1. Open the Visual Studio Developer Command Prompt:
"C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\Common7\Tools\VsDevCmd.bat"

2. Generate Visual Studio project files using CMake:

3. Build the project in Release configuration:


## Examples
exempl scripts intend to be placed in AN [examples](exemple/) folder,
build exempleare intred to  be placed in 
```cpp
#include 
```
## Documentation
the daetailed documenattaion of class and function in [cuAStar](https://wissem01chiha.github.io/cuAStar/)

![VTK_trench](build/VTK_trench.png)
![Trajectory 2D](build/traj2d.png)
![Clear Trajectory 2D](build/traj2d_clear.png)

## Benchmark
Performance benchmarks:

- Node setup time: 74 ms
- Chunk open set computation time: 13562 ms
- Trajectory computation time: 47 ms
- 2D Trajectory visualization time: 1970 ms
- Total execution time: 21189 ms

### TO-DO 
- intergate a tes cases 
### Reference

### Cite this Work


## Dataset
The dataset used for this project is [OpenTrench3D](https://www.kaggle.com/datasets/hestogpony/opentrench3d).

## Environment Setup
To set up your environment, follow these steps:

1. Open the Visual Studio Developer Command Prompt:
"C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\Common7\Tools\VsDevCmd.bat"

2. Generate Visual Studio project files using CMake:

3. Build the project in Release configuration:

## Building Tests
Currently, building the test suite requires a batch script which is not yet available.
### 
## Examples
build exempleare intred to  be placed in ![examples](exemple/) folder, 

## Documentation
the daetailed documenattaion of class and function in [cuAStar](https://wissem01chiha.github.io/cuAStar/)

![VTK_trench](build/VTK_trench.png)
![Trajectory 2D](build/traj2d.png)
![Clear Trajectory 2D](build/traj2d_clear.png)

## Benchmark
Performance benchmarks:

- Node setup time: 74 ms
- Chunk open set computation time: 13562 ms
- Trajectory computation time: 47 ms
- 2D Trajectory visualization time: 1970 ms
- Total execution time: 21189 ms

### TO-DO 
- intergate a tes cases 