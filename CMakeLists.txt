cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(CUASTAR VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
set(CMAKE_VERBOSE_MAKEFILE ON)

option(BUILD_TESTS "Building CXX or CUDA Tests" ON)
option(BUILD_EXEMPLES "Building CXX or CUDA Exemples " OFF)
option(BUILD_DOCS "Build documentation or not" OFF)
option(USE_CUDA "Building with CUDA Support " ON)
option(USE_VTK "Building with VTK Support " OFF)

option(USE_CPM   "Whether to use CPM."  OFF)
option(USE_CONAN "Whether to use Conan." OFF)
option(USE_VCPKG "Whether to use VCPKG." OFF)

if(USE_CPM)
    file(
        DOWNLOAD
        https://github.com/cpm-cmake/CPM.cmake/releases/download/v0.38.3/CPM.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/cmake/CPM.cmake
        EXPECTED_HASH SHA256=cc155ce02e7945e7b8967ddfaff0b050e958a723ef7aad3766d368940cb15494
    )
    include(${CMAKE_CURRENT_BINARY_DIR}/cmake/CPM.cmake)
    CPMAddPackage(...)
endif()

if(NOT(USE_CPM) AND NOT(USE_CONAN) AND NOT(USE_VCPKG) )
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -I${CMAKE_CURRENT_SOURCE_DIR}/extern")
endif()

if(USE_CUDA)
    find_package(CUDAToolkit REQUIRED)
    target_link_libraries(${PROJECT_NAME} CUDA::cudart)
else()

endif()

if(USE_VTK)
    set(VTK_INCLUDE_DIR "C:/Program Files/VTK-9.3.1/src")
    set(VTK_DIR "C:/Program Files/VTK-9.3.1/build")
    set(VTK_LIB_DIR "C:/Program Files/VTK-9.3.1/build/lib")
    link_directories("C:/Program Files/VTK-9.3.1")
    include_directories(${VTK_INCLUDE_DIR})

    find_package(VTK REQUIRED)
    find_package(VTK COMPONENTS 
    CommonColor
    CommonCore
    FiltersSources
    IOImage
    InteractionStyle
    RenderingContextOpenGL2
    RenderingCore
    RenderingFreeType
    RenderingGL2PSOpenGL2
    RenderingOpenGL2
    RenderingUI
    IOPLY
    CommonDataModel
    )

    if (NOT VTK_FOUND)
        message(FATAL_ERROR "VTK build folder not found.")
    endif()

    set(CMAKE_NINJA_FORCE_RESPONSE_FILE "ON" CACHE BOOL "Force Ninja to use response files.")
    vtk_module_autoinit(
        TARGETS  
        MODULES ${VTK_LIBRARIES}
    )
endif()

if(BUILD_TESTS)
    add_subdirectory(test)
endif()

if(BUILD_DOCS)
    find_package(Doxygen REQUIRED)
endif()