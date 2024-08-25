cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(cuRobotics VERSION 0.1.0)

set(Torch FALSE)
set(enable_test False)

set(SRCDIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

set(BINDIR ${CMAKE_CURRENT_BINARY_DIR}/build)

set(CMAKE_CXX_STANDARD 14)

# Compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -I${CMAKE_CURRENT_SOURCE_DIR}/extern/mujoco/include")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -I${CMAKE_CURRENT_SOURCE_DIR}/extern/glfw/include")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -I${CMAKE_CURRENT_SOURCE_DIR}/include")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -I${CMAKE_CURRENT_SOURCE_DIR}/extern/glm")

# Linker flags
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/extern/mujoco/bin ${CMAKE_CURRENT_SOURCE_DIR}/extern/glfw/lib)
#set(LDFLAGS "-lmujoco -lglfw3")

# List all .cpp and .c source files
file(GLOB SOURCES ${SRCDIR}/*.cpp ${SRCDIR}/*.c)


# Generate executables
foreach(source_file ${SOURCES})
    get_filename_component(executable_name ${source_file} NAME_WE)
    set(executable_path ${BINDIR}/${executable_name})
    add_executable(${executable_name} ${source_file})
    #target_link_libraries(${executable_name} ${LDFLAGS})
    set_target_properties(${executable_name} PROPERTIES
                          RUNTIME_OUTPUT_DIRECTORY ${BINDIR})
endforeach(source_file ${SOURCES})

# Custom target for log message
add_custom_target(log
    COMMAND echo "building package mjUr5e..."
)



    if (MSVC)
        file(GLOB TORCH_DLLS "${TORCH_INSTALL_PREFIX}/lib/*.dll")
        add_custom_command(TARGET example-app
                       POST_BUILD
                       COMMAND ${CMAKE_COMMAND} -E copy_if_different
                       ${TORCH_DLLS}
                       $<TARGET_FILE_DIR:example-app>)
    endif (MSVC)