// ColorLUT.cpp
#include "ColorLUT.h"

// 静态成员定义
const std::array<uint8_t, 65536> ColorLUT::table = []{
    std::array<uint8_t, 65536> lut{};
    for(int i=0; i<65536; ++i) lut[i] = i >> 8;
    return lut;
}();

// 公共接口实现
uint8_t ColorLUT::convert(uint16_t v) {
    return table[v];  // 类内部可以访问私有成员
}

// 可选：通过方法访问（更安全）
const std::array<uint8_t, 65536>& ColorLUT::getTable() {
    return table;
}