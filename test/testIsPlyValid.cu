#define STBI_MSC_SECURE_CRT
#define STB_IMAGE_WRITE_IMPLEMENTATION
#define STB_IMAGE_IMPLEMENTATION
#define CUASTAR_DEBUG
#define CUASTAR_IMPLEMENTATION
#include "cuAstar/cuAStar.hpp"

int  main(){
    bool x = isPlyValid("Area_1_Site_1_24463.ply");
    std::cout << "the file valididty is " << x << std::endl;
    return 0;
}