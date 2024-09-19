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

> There is no CPU version of **cuAstar** this is a work in progress.
  
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


**VTK Integration**

3D Point cloud enviroment and trajectory debuging data uses [Visualization Toolkit](https://vtk.org/), while is not offically tested or supported, by cuAstar, for any bugs encountered during installation, please make sure to open an issue at: [Issues](https://github.com/wissem01chiha/cuAStar),

```shell
   cmake -G "Visual Studio 17 2022" -DUSE_VTK=ON  ..
``` 
For building and installing the Visualization Toolkit, refer to the official documentation: [install VTK](https://docs.vtk.org/en/latest/build_instructions/build.html)
 

## Dataset
We used a random sample of the [OpenTrench3D](https://www.kaggle.com/datasets/hestogpony/opentrench3d) dataset as a test dataset for this project. The original data files (containing more than 1 million points) were downsampled using [MeshLab](https://www.meshlab.net/) software to a maximum of 100,000 points.

other data samples could be found at [Point Cloud Datasets](https://github.com/antao97/PointCloudDatasets)


## Examples
exempl scripts intend to be placed in AN [examples](exemple/) folder,
build exempleare intred to  be placed in

initlize the planner with point cloud datset file 
```cpp
   #define CUASTAR_IMPLEMENTATION
   #include "cuAstar/cuAStar.hpp"

   AstarPlanner<Node3d<T>,T> planner("point_cloud_file.ply");

```
init the pallner with random 2d or 3d point cloud 
```cpp
   #define CUASTAR_IMPLEMENTATION
   #include "cuAstar/cuAStar.hpp"

   int pointNum = 1000;
   unsigned int seed = 50;

   AstarPlanner<Node3d<T>,T> planner(pointNum, seed);

```
inilize the planner with a given 


<p align="center">
  <img src="docs/source/trench_overivew.png" alt="VTK_trench" width="250" height="220"/>
  <img src="docs/source/trench2d.png" alt="VTK_trench" width="250" height="220"/>
</p>


```cpp
   #define CUASTAR_IMPLEMENTATION
   #include "cuAstar/cuAStar.hpp"

   int pointNum = 1000;
   unsigned int seed = 50;

   AstarPlanner<Node3d<T>,T> planner(pointNum, seed);

```

<p align="center">
  <img src="docs/source/simple_chunks.png" alt="simple_chunks" width="250" height="220"/>
   <img src="docs/source/chunks_with_start_en.png" alt="chunks_with_start_end" width="250" height="220"/>
</p>


## Documentation
The daetailed documenattaion of palnner class members and cuda functions in [cuAStar](https://wissem01chiha.github.io/cuAStar/)

**Building Documentation**

documenation was building using the interpolabilty between doxygen and sphnix tools, 
you nedd to install doxygen , sphnix and breathe

```shell
   doxygen Doxyfile 
   sphinx-build -b html .  docs/build   
```

## Benchmark

Performance benchmarks for two different node sets are given below:

| Execution Task                 | 93,000 nodes | 24,000 nodes |
|--------------------------------|--------------|--------------|
| Node setup                     | 74           | 45           |
| Chunk open set computation     | 13562        | 8000         |
| Trajectory computation time    | 47           | 25           |
| 2D Trajectory visualization    | 1970         | 1300         |
| Total execution time           | 21189        | 9370         |

> [!IMPORTANT]   
> The minimum C++ standard required is C++17. Ensure that your compiler supports the standard features.


### TO-DO 
- intergate a tes cases 
 

### TO-DO 
- intergate a tes cases 

## Contribuation 

### Reference

### Cite this Work