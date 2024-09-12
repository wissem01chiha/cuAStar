// // nvcc -std=c++17 -o build/testChunkOpenSet test/testChunkOpenSet.cu
#define CUASTAR_IMPLEMENTATION 
#define CUASTAR_DEBUG

#include "../include/cuAStar.hpp"

int main(){
    using T = float;
    using NodeType = Node3d<T>;
    AstarPlanner<Node3d<T>,T> planner("Area_1_Site_93340.ply");
    planner.computeChunkOpenSet();

    return 0;
}