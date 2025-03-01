// ColorLUT.h - 简化颜色转换表实现
#pragma once
#include <cstdint>
#include <array>  // 必须包含array头文件

struct ColorLUT {
    // 公共访问接口
    static uint8_t convert(uint16_t v);
    
private:
    // 私有静态成员声明
    static const std::array<uint8_t, 65536> table;
    
    // 新增私有初始化方法（可选）
    static const std::array<uint8_t, 65536>& getTable();
};