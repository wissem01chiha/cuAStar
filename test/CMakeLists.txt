if(USE_CUDA)
    file(GLOB TEST_SOURCES "*.cu")
else()
    file(GLOB TEST_SOURCES ".cpp")
endif()

add_executable(Tests ${TEST_SOURCES})
