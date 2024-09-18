<h1 align="center">cuAstar</h1>
<div align="center">
Fast, modern, fully templated, single-file, header-only, parallel implementation of A* trajectory planner on NVIDIA GPUs in point cloud data.
</div>
<div align="center">
  <!-- Stability -->
  <a href="https://nodejs.org/api/documentation.html#documentation_stability_index">
    <img src="https://img.shields.io/badge/stability-experimental-orange.svg?style=flat-square"
      alt="API stability" />
  </a>
  <!-- NPM version -->
  <a href="https://npmjs.org/package/choo">
    <img src="https://img.shields.io/npm/v/choo.svg?style=flat-square"
      alt="NPM version" />
  </a>
  <!-- Build Status -->
  <a href="https://travis-ci.org/choojs/choo">
    <img src="https://img.shields.io/travis/choojs/choo/master.svg?style=flat-square"
      alt="Build Status" />
  </a>
  <!-- Test Coverage -->
  <a href="https://codecov.io/github/choojs/choo">
    <img src="https://img.shields.io/codecov/c/github/choojs/choo/master.svg?style=flat-square"
      alt="Test Coverage" />
  </a>
  <!-- Standard -->
  <a href="https://standardjs.com">
    <img src="https://img.shields.io/badge/code%20style-standard-brightgreen.svg?style=flat-square"
      alt="Standard" />
  </a>
</div>


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