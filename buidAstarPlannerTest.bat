@echo on
nvcc -std=c++17 -o build/testAstarPlanner test/testAstarPlanner.cu
cd build
testAstarPlanner.exe
