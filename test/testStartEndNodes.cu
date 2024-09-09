// nvcc -std=c++17 -o build/teststartend test/testStartEndNodes.cu
#define CUASTAR_IMPLEMENTATION 
#define CUASTAR_DEBUG
#include "../include/cuAStar.hpp"


int main(){

    using T = float;
    using NodeType = Node3d<T>;

    AstarPlanner<Node3d<T>,T> planner("Area_1_Site_1_24463.ply");

    NodeType start(81.188, 47.832, 1.505);

    planner.setStartNode(&start);
    return 0;
}