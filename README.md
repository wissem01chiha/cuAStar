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
**cuAstar** is a template CUDA-based implementation, proposed by me, for generating fast, optimal trajectories within scanned point cloud data. It supports handling and rendering of both 2D and 3D points.

> [!NOTE]  
> This is an experimental version. Trajectory computation time and optimality may not be the best, and optimizations are required. Check the [To-Do](#to-do) section for more information.

## Dependencies

   the lib is desoned to maintain heder only property, so we used some lighwaeith header only libs for data file proseening and logging 
   - [rapidcsv]() 
   - [happly]()  for parsing point cloud .ply files 
   - [loguru]() for enhanced logging in degug mode
   - [stb_image]() for writing , reading images format 
   - 
> [!NOTE]  
> The crosshair is designed for a 1920x1080 resolution; in other case, the experience may vary.  

<details>
  <summary>Click to expand</summary>
  
  Here is the content hidden inside the collapsible section.
  
</details>


## Installation

> [!IMPORTANT]  
> The binds system has changed. Instead of doing the name of the key, there are scancodes assigned per key.
        
> [!NOTE]  
> The crosshair is designed for a 1920x1080 resolution; in other case, the experience may vary.

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
## Building documenttation 

## Examples
build exempleare intred to  be placed in ![examples](exemple/) folder, 
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