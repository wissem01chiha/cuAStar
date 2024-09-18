#define STBI_MSC_SECURE_CRT
#define STB_IMAGE_WRITE_IMPLEMENTATION
#define STB_IMAGE_IMPLEMENTATION
#define CUASTAR_DEBUG
#define CUASTAR_IMPLEMENTATION
#include "cuAstar/cuAStar.hpp"

template <typename T>
void testNode3d() {

    Node3d<T> node1;
    std::cout << "Node 1: x=" << node1.x << ", y=" << node1.y << ", z=" << node1.z << std::endl;

    unsigned int seed = 10;
    Node3d<T> node2(seed);
    std::cout << "Node 2: x=" << node2.x << ", y=" << node2.y << ", z=" << node2.z << std::endl;

    T x = static_cast<T>(1.5);
    T y = static_cast<T>(2.5);
    T z = static_cast<T>(3.5);
    Node3d<T> node3(x, y, z);
    std::cout << "Node 3: x=" << node3.x << ", y=" << node3.y << ", z=" << node3.z << std::endl;

    T distance = node1.distanceTo(node3);
    std::cout << "Distance between Node 1 and Node 3: " << distance << std::endl;

    bool equal = node1.isEqual(node3);
    std::cout << "Node 1 and Node 3 are equal (based on position): " << std::boolalpha << equal << std::endl;
}

int main() {
    std::cout << "Testing Node3d<float>:" << std::endl;
    testNode3d<float>();

    std::cout << "\nTesting Node3d<double>:" << std::endl;
    testNode3d<double>();

    return 0;
}