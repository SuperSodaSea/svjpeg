#include "svjpeg.hpp"

int main() {
    std::FILE* file = std::fopen("test.jpg", "wb");
    std::uint8_t data[256 * 256 * 3];
    std::uint8_t* p = data;
    for(int y = 0; y < 256; ++y)
        for(int x = 0; x < 256; ++x)
            *p++ = x, *p++ = y, *p++ = 128;
    svjpeg(file, 256, 256, data);
    std::fclose(file);
}
