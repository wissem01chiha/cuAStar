// nvcc -std=c++17 -o build/testply test/testply.cu
#define CUASTAR_DEBUG
#include "../include/cuAStar.hpp"

int  main(){
    bool x = isPlyValid("Area_1_Site_1_24463.ply");
    size_t n =  getPlyPointNum("Area_1_Site_1_24463.ply");
    std::cout << "the file valididty is " << x << std::endl;
    std::cout << "the number of nodes is " << n;
    return 0;
}