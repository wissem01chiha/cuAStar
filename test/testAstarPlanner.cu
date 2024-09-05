
// nvcc -std=c++17 -o build/testAstarPlanner test/testAstarPlanner.cu
#define CUASTAR_IMPLEMENTATION 
#define CUASTAR_DEBUG
#include "../include/cuAStar.hpp"

int main(){

    AstarPlanner<Node3d<double>,double> a();


    return 0; 
}