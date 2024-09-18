#define STBI_MSC_SECURE_CRT
#define STB_IMAGE_WRITE_IMPLEMENTATION
#define STB_IMAGE_IMPLEMENTATION
#define CUASTAR_DEBUG
#define CUASTAR_IMPLEMENTATION
#include "cuAstar/cuAStar.hpp"

int  main(){
    size_t n =  getPlyPointNum("Area_1_Site_1_24463.ply");
    std::cout << "the number of nodes is " << n;
    return 0;
}