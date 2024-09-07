
// nvcc -std=c++17 -o build/testAstarPlanner test/testAstarPlanner.cu
// nvcc -std=c++17 --coverage -o build/testAstarPlanner test/testAstarPlanner.cu
#define CUASTAR_IMPLEMENTATION 
#define CUASTAR_DEBUG
#include "../include/cuAStar.hpp"

int main(){
    using T = float;
    using NodeType = Node3d<T>;
    const int numNodes = 5;

    // Create an array of nodes on the host
    NodeType h_nodesArray[numNodes] = {
        NodeType(250.0, 250.0, 250.0),
        NodeType(150.5f, 5.0f, 6.0f),
        NodeType(50.0f, 3.789f, 1.5f),
        NodeType(10.0f, 11.0f, 12.0f),
        NodeType(13.0f, 14.0f, 15.0f)
    };

    // array2PointCloudImg<Node2d<T>>(h_nodesArray, numNodes,
               //          "testfig1.png", 750 , 500, 0.01);

    AstarPlanner<Node3d<T>,T> planner("Area_1_Site_93340.ply");
    planner.visualize2dPointCloud("testfig2.png");

    AstarPlanner<Node3d<T>,T> planner2(10,500);
     
    return 0; 
}