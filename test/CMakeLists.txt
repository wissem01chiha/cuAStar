if(USE_CUDA)
    set(TEST_SOURCES 
        testAstarPlanner.cu
        testCheckNodeExsist.cu
        testComputeChunkOpenSet.cu
        testComputeKnnNodes.cu
        testComputeOptimalNode.cu
        testComputeTrajectory.cu
        testEnumerationSortNodes.cu
        testGetPlyPointNum.cu
        testIsPlyValid.cu
        testNode2d.cu
        testNode3d.cu
        testSetInitialNode.cu
    ) 
else()
    set(TEST_SOURCES) 
endif()

foreach(test_file ${TEST_SOURCES})

    get_filename_component(test_name ${test_file} NAME_WE)
    add_executable(${test_name} ${test_file})
    target_include_directories(${test_name} PRIVATE
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/extern
    )
    if(USE_CUDA)
        target_link_libraries(${test_name} PRIVATE LOGURU CUDA::cudart)
    endif()
    
endforeach()


