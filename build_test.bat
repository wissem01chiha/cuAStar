@echo on
nvcc -std=c++17 -o build/testAstarPlanner test/testAstarPlanner.cu
nvcc -std=c++17 -o build/testNode3d test/testNode3d.cu
nvcc -std=c++17 -o build/teststartend test/testStartEndNodes.cu
nvcc -std=c++17 --diag-suppress=20054 -o build/testTrajectory test/testTrajectory.cu
nvcc -std=c++17 -o build/testSortNodes test/testSortNodes.cu
nvcc -std=c++17 -o build/testply test/testply.cu
nvcc -std=c++17 -o build/testknn test/testknn.cu
nvcc -std=c++17 -o testNode2d test/testNode2d.cu
nvcc -std=c++17 --diag-suppress=20054 -o build/testsuccesor test/testsuccesor.cu
nvcc -std=c++17 -o build/testAstarPlanner test/testAstarPlanner.cu
nvcc -std=c++17 -o build/testcheckNodeExsist test/testcheckNodeExsist.cu
nvcc -std=c++17 -o build/testChunkOpenSet test/testChunkOpenSet.cu

