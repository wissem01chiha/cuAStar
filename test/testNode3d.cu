// nvcc -std=c++17 -o build/testNode3d test/testNode3d.cu
#include "../cpp/cuAStar.hpp"
#include <iostream>

template <typename T>
void testNode3d() {
    // Test default constructor
    Node3d<T> node1;
    std::cout << "Node 1: x=" << node1.x << ", y=" << node1.y << ", z=" << node1.z << std::endl;

    // Test random constructor (seed = 10)
    unsigned int seed = 10;
    Node3d<T> node2(seed);
    std::cout << "Node 2: x=" << node2.x << ", y=" << node2.y << ", z=" << node2.z << std::endl;

    // Test parameterized constructor
    T x = static_cast<T>(1.5);
    T y = static_cast<T>(2.5);
    T z = static_cast<T>(3.5);
    Node3d<T> node3(x, y, z);
    std::cout << "Node 3: x=" << node3.x << ", y=" << node3.y << ", z=" << node3.z << std::endl;

    // Test distanceTo method
    T distance = node1.distanceTo(node3);
    std::cout << "Distance between Node 1 and Node 3: " << distance << std::endl;

    // Test isEqual method
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