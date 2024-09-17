#define CUASTAR_DEBUG
#include "../include/cuAStar.hpp"

int  main(){
    size_t n =  getPlyPointNum("Area_1_Site_1_24463.ply");
    std::cout << "the number of nodes is " << n;
    return 0;
}