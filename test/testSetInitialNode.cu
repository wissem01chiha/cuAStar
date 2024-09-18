#define STBI_MSC_SECURE_CRT
#define STB_IMAGE_WRITE_IMPLEMENTATION
#define STB_IMAGE_IMPLEMENTATION
#define CUASTAR_DEBUG
#define CUASTAR_IMPLEMENTATION
#include "cuAstar/cuAStar.hpp"

int main(){

    using T = float;
    using NodeType = Node3d<T>;

    AstarPlanner<Node3d<T>,T> planner("Area_1_Site_1_24463.ply");

    NodeType start(81.188, 47.832, 1.505);

    planner.setInitialNode(&start);
    return 0;
}