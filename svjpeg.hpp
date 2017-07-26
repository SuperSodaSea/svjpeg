/*
 *
 * MIT License
 *
 * Copyright (c) 2017 SuperSodaSea
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#ifndef SVJPEG_HPP
#define SVJPEG_HPP

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <algorithm>
#include <initializer_list>
#include <tuple>

namespace Impl {

inline std::tuple<double, double, double> rgb2ycbcr(const std::uint8_t* rgb) {
    double r = rgb[0], g = rgb[1], b = rgb[2];
    return std::tuple<double, double, double>(0.299 * r + 0.587 * g + 0.114 * b - 128,
        - 0.1687 * r - 0.3313 * g + 0.5 * b, 0.5 * r - 0.4187 * g - 0.0813 * b);
}
inline void dct(const double* input, std::int16_t* output) {
    constexpr double PI = 3.1415926;
    auto f = [](int x) { return x ? 1 : 1 / std::sqrt(2); };
    for(int v = 0; v < 8; ++v) for(int u = 0; u < 8; ++u) {
        double sum = 0;
        for(int y = 0; y < 8; ++y) for(int x = 0; x < 8; ++x)
            sum += input[y * 8 + x] * std::cos((2 * x + 1) * u * PI * 0.0625) * std::cos((2 * y + 1) * v * PI * 0.0625);
        output[v * 8 + u] = std::round(sum * 0.25 * f(u) * f(v) / 4);
    }
}

}

inline void svjpeg(std::FILE* file, std::uint32_t width, std::uint32_t height, std::uint8_t* data) {
    static const std::uint8_t ZZ[64] = {
        0, 1, 8, 16, 9, 2, 3, 10, 17, 24, 32, 25, 18, 11, 4, 5,
        12, 19, 26, 33, 40, 48, 41, 34, 27, 20, 13, 6, 7, 14, 21, 28,
        35, 42, 49, 56, 57, 50, 43, 36, 29, 22, 15, 23, 30, 37, 44, 51,
        58, 59, 52, 45, 38, 31, 39, 46, 53, 60, 61, 54, 47, 55, 62, 63,
    };
    
    auto encode = [](std::int16_t x) -> std::pair<int, int> {
        if(x) { int s = floor(log2(abs(x))) + 1; return {s, x > 0 ? x : x + (1 << s) - 1}; } else return {0, 0};
    };
    auto write = [file](std::initializer_list<std::uint8_t> d) { std::fwrite(d.begin(), 1, d.size(), file); };
    auto write16 = [write](std::uint16_t d) { write({std::uint8_t(d >> 8), std::uint8_t(d)}); };
    auto writeMarker = [write](std::uint8_t code) { write({0xFF, code}); };
    std::uint8_t buf = 0, off = 0;
    auto writeBit = [write, &buf, &off](bool d) {
        buf |= d << (7 - off); if(++off == 8) { if(buf == 0xFF) write({0xFF, 0x00}); else write({buf}); buf = off = 0; }
    };
    auto writeBits = [writeBit](std::uint16_t d, int s) { for(int i = 0; i < s; ++i) writeBit(d & (1 << (s - i - 1))); };
    auto writeHuffman = [writeBits](std::uint8_t d) { if(d == 0xFF) writeBits(0x1FE, 9); else writeBits(d, 8); };
    
    writeMarker(0xD8); // SOI
    for(std::uint8_t i = 0; i < 2; ++i) { writeMarker(0xDB), write({0x00, 0x43, i}); for(int j = 0; j < 64; ++j) write({0x04}); } // DQT
    writeMarker(0xC0); // SOF0
        write({0x00, 0x11, 0x08}), write16(height), write16(width),
        write({0x03, 0x01, 0x11, 0x00, 0x02, 0x11, 0x01, 0x03, 0x11, 0x01});
    writeMarker(0xC4); write({0x00, 0x1F, 0x00}); // DHT
        for(int i = 0; i < 16; ++i) write({std::uint8_t(i == 7 ? 12 : 0)});
        for(int i = 0; i < 12; ++i) write({std::uint8_t(i)});
    writeMarker(0xC4); write({0x01, 0x13, 0x10}); // DHT
        for(int i = 0; i < 16; ++i) write({std::uint8_t(i == 7 ? 255 : i == 8 ? 1 : 0)});
        for(int i = 0; i < 256; ++i) write({std::uint8_t(i)});
    writeMarker(0xDA); write({0x00, 0x0C, 0x03, 0x01, 0x00, 0x02, 0x00, 0x03, 0x00, 0x00, 0x3F, 0x00}); // SOS
    double ycbcr[3][64]; std::int16_t lastDC[3] = {}, dct1[64], dct2[64];
    for(std::uint32_t ch = (height + 7) / 8, cy = 0; cy < ch; ++cy) for(std::uint32_t cw = (width + 7) / 8, cx = 0; cx < cw; ++cx) {
        for(int i = 0; i < 64; ++i) { // RGB -> YCbCr
            std::uint32_t x = std::min(cx * 8 + i % 8, width - 1), y = std::min(cy * 8 + i / 8, height - 1); 
            std::tie(ycbcr[0][i], ycbcr[1][i], ycbcr[2][i]) = Impl::rgb2ycbcr(data + (y * width + x) * 3);
        }
        for(int i = 0; i < 3; ++i) {
            Impl::dct(ycbcr[i], dct1); // DCT & Quantization
            for(int i = 0; i < 64; ++i) dct2[i] = dct1[ZZ[i]];
            std::int16_t delta = dct2[0] - lastDC[i]; lastDC[i] = dct2[0]; // DC
            auto dc = encode(delta); writeHuffman(dc.first); writeBits(dc.second, dc.first);
            int index = 1; // AC
            do {
                int zero = 0;
                while(!dct2[index] && ++index < 64) ++zero; if(index == 64) { writeHuffman(0); break; }
                while(zero >= 16) writeHuffman(0xF0), zero -= 16;
                auto ac = encode(dct2[index]); writeHuffman(ac.first | (zero << 4)); writeBits(ac.second, ac.first);
            } while(++index < 64);
        }
    }
    if(off != 0) write({buf});
    writeMarker(0xD9); // EOI
}

#endif
