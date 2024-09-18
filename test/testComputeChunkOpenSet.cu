#define STBI_MSC_SECURE_CRT
#define STB_IMAGE_WRITE_IMPLEMENTATION
#define STB_IMAGE_IMPLEMENTATION
#define CUASTAR_DEBUG
#define CUASTAR_IMPLEMENTATION
#include "cuAstar/cuAStar.hpp"

int main(){
    using T = float;
    using NodeType = Node3d<T>;

    AstarPlanner<Node3d<T>,T> planner("Area_1_Site_93340.ply");
    NodeType start(82.029, 48.097, 1.174);
    NodeType end(84.009, 50.173, 2.073);
    planner.setTargetNode(&end);
    planner.setInitialNode(&start);
    planner.computeChunkOpenSet();

    return 0;
}