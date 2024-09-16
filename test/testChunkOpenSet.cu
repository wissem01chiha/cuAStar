#define CUASTAR_IMPLEMENTATION 
#define CUASTAR_DEBUG

#include "../include/cuAStar.hpp"

int main(){
    using T = float;
    using NodeType = Node3d<T>;
    //Area_1_Site_1_24463
    AstarPlanner<Node3d<T>,T> planner("Area_1_Site_93340.ply");
    NodeType start(82.029, 48.097, 1.174);
    NodeType end(84.009, 50.173, 2.073);
    planner.setTargetNode(&end);
    planner.setInitialNode(&start);
    planner.computeChunkOpenSet();

    return 0;
}