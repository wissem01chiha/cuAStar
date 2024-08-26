#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "../extern/stb/stb_image.h"
#include "../extern/stb/stb_image_write.h"
#include <cmath>
#include <vector>
#include <cstdint>
#include <string>
#include <array>

/** @brief Draw a 2d sphere (plain circle), given radius, center and color */
void drawSphere(uint8_t* image, int width, int height, int centerX, 
                    int centerY, int radius, uint8_t r, uint8_t g, uint8_t b){

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int dx = x - centerX;
            int dy = y - centerY;
            int distanceSquared = dx * dx + dy * dy;
            if (distanceSquared <= radius * radius) {
                int offset = (y * width + x) * 3;
                image[offset] = r;
                image[offset + 1] = g;
                image[offset + 2] = b;
            }
        }
    }
}




// Function to draw multiple spheres onto an image buffer
void drawSpheresOnImage(uint8_t* image, int width, int height, const std::vector<std::pair<int, int>>& centers, const std::vector<std::array<uint8_t, 3>>& colors, double radiusRatio = 0.05) {
    int radius = static_cast<int>(radiusRatio * std::min(width, height));  // Calculate the radius based on the image size

    for (size_t i = 0; i < centers.size(); ++i) {
        int centerX = centers[i].first;
        int centerY = centers[i].second;
        uint8_t r = colors[i][0];
        uint8_t g = colors[i][1];
        uint8_t b = colors[i][2];

        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                int dx = x - centerX;
                int dy = y - centerY;
                int distanceSquared = dx * dx + dy * dy;

                if (distanceSquared <= radius * radius) {
                    int offset = (y * width + x) * 3;
                    image[offset] = r;
                    image[offset + 1] = g;
                    image[offset + 2] = b;
                }
            }
        }
    }
}

// Function to create an image and save it using stb_image_write
void saveSpheresImage(const std::string& filePath, int width, int height, const std::vector<std::pair<int, int>>& centers, const std::vector<std::array<uint8_t, 3>>& colors) {
    // Create a buffer for the image (3 channels: R, G, B)
    std::vector<uint8_t> image(width * height * 3, 255);  // Initialize with white background

    // Draw the spheres on the image
    drawSpheresOnImage(image.data(), width, height, centers, colors);

    // Save the image as a PNG file
    stbi_write_png(filePath.c_str(), width, height, 3, image.data(), width * 3);
}

int main() {
    int width = 512;  // Image width
    int height = 512; // Image height

    // Define the centers of the spheres
    std::vector<std::pair<int, int>> centers = {
        {128, 128},
        {384, 128},
        {256, 384},
        {50,50}
    };

    // Define the colors of the spheres
    std::vector<std::array<uint8_t, 3>> colors = {
        std::array<uint8_t, 3>{255, 0, 0},   // Red
        std::array<uint8_t, 3>{0, 255, 0},   // Green
        std::array<uint8_t, 3>{0, 0, 255},   // Blue
        std::array<uint8_t, 3>{12, 3, 124}    // Blue
    };

    saveSpheresImage("spheres.png", width, height, centers, colors);

    return 0;
}
