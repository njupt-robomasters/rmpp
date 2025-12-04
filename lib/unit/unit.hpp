#pragma once

#include <cstddef>

namespace unit {
    constexpr size_t MAX_STR_SIZE = 16;

    class Unit {
    public:
        char m_str[MAX_STR_SIZE]{}; // 单位文本描述
        float m_scale = 1.0f;       // 单位缩放比例

        // 无参构造
        constexpr Unit() = default;

        // 拷贝构造
        constexpr Unit(const Unit& other) {
            Unit::strncpy(m_str, other.m_str, MAX_STR_SIZE);
            m_scale = other.m_scale;
        }

        // 用str和scale构造
        constexpr Unit(const char* str, const float scale) {
            Unit::strncpy(m_str, str, MAX_STR_SIZE);
            m_scale = scale;
        }

        // 赋值运算，优先用自己的单位，自己无单位时用对方的单位
        constexpr Unit& operator=(const Unit& other) {
            if (Unit::strlen(m_str) == 0) {
                Unit::strncpy(m_str, other.m_str, MAX_STR_SIZE);
                m_scale = other.m_scale;
            }
            return *this;
        }

        // 加法运算，优先用左操作数的单位，左操作数无单位时用右操作数的单位
        constexpr Unit operator+(const Unit& other) const {
            Unit ret;
            if (Unit::strlen(m_str) != 0) {
                Unit::strncpy(ret.m_str, m_str, MAX_STR_SIZE);
                ret.m_scale = m_scale;
            } else {
                Unit::strncpy(ret.m_str, other.m_str, MAX_STR_SIZE);
                ret.m_scale = other.m_scale;
            }
            return ret;
        }

        // +=运算，优先用自己的单位，自己无单位时用对方的单位
        constexpr Unit& operator+=(const Unit& other) {
            if (Unit::strlen(m_str) == 0) {
                Unit::strncpy(m_str, other.m_str, MAX_STR_SIZE);
                m_scale = other.m_scale;
            }
            return *this;
        }

        // 减法运算，优先用左操作数的单位，左操作数无单位时用右操作数的单位
        constexpr Unit operator-(const Unit& other) const {
            Unit ret;
            if (Unit::strlen(m_str) != 0) {
                Unit::strncpy(ret.m_str, m_str, MAX_STR_SIZE);
                ret.m_scale = m_scale;
            } else {
                Unit::strncpy(ret.m_str, other.m_str, MAX_STR_SIZE);
                ret.m_scale = other.m_scale;
            }
            return ret;
        }

        // -=运算，优先用自己的单位，自己无单位时用对方的单位
        constexpr Unit& operator-=(const Unit& other) {
            if (Unit::strlen(m_str) == 0) {
                Unit::strncpy(m_str, other.m_str, MAX_STR_SIZE);
                m_scale = other.m_scale;
            }
            return *this;
        }

        // 乘法运算
        constexpr Unit operator*(const Unit& other) const {
            Unit ret;
            combineStr(ret.m_str, m_str, "*", other.m_str);
            ret.m_scale = m_scale * other.m_scale;
            return ret;
        }

        // *=运算
        constexpr Unit& operator*=(const Unit& other) {
            combineStr(m_str, m_str, "*", other.m_str);
            m_scale *= other.m_scale;
            return *this;
        }

        // 除法运算
        constexpr Unit operator/(const Unit& other) const {
            Unit ret;
            combineStr(ret.m_str, m_str, "/", other.m_str);
            ret.m_scale = m_scale / other.m_scale;
            return ret;
        }

        // /=运算
        constexpr Unit& operator/=(const Unit& other) {
            combineStr(m_str, m_str, "/", other.m_str);
            m_scale /= other.m_scale;
            return *this;
        }

        // 后置实现 Unit * float，因为此时UnitFloat还未定义
        constexpr auto operator*(float value) const;

    private:
        static constexpr size_t strlen(const char* str) {
            size_t len = 0;
            while (str[len] != '\0') {
                len++;
            }
            return len;
        }

        static constexpr char* strncat(char* dest, const char* src, const size_t size) {
            // 找到 dest 字符串的末尾
            const size_t dest_start = strlen(dest);
            size_t i = 0;

            // 直到遇到 src 的终止符，或达到 dest 的最大长度
            for (; src[i] != '\0' && dest_start + i < size - 1; i++) {
                dest[dest_start + i] = src[i];
            }

            // 添加终止符
            dest[dest_start + i] = '\0';

            return dest;
        }

        static constexpr char* strncpy(char* dest, const char* src, const size_t size) {
            std::size_t i = 0;

            // 复制源字符串的字符，直到遇到空字符或达到 dest 的最大长度
            for (; src[i] != '\0' && i < size - 1; ++i) {
                dest[i] = src[i];
            }

            // 用空字符填充剩余位置
            for (; i < size; i++) {
                dest[i] = '\0';
            }

            return dest;
        }

        static constexpr void combineStr(char* dest, const char* str1, const char* symbol, const char* str2) {
            const bool str1_empty = strlen(str1) == 0;
            const bool str2_empty = strlen(str2) == 0;

            // 左右值均无单位，返回空字符串
            if (str1_empty && str2_empty) {
                Unit::strncpy(dest, "", MAX_STR_SIZE);
                return;
            }

            // 左值无单位
            if (str1_empty) {
                if (symbol[0] == '/') {
                    Unit::strncpy(dest, "1/", MAX_STR_SIZE);
                    Unit::strncat(dest, str2, MAX_STR_SIZE);
                    return;
                } else {
                    Unit::strncpy(dest, str2, MAX_STR_SIZE);
                    return;
                }
            }

            // 右值无单位，用左值的单位
            if (str2_empty) {
                Unit::strncpy(dest, str1, MAX_STR_SIZE);
                return;
            }

            // 左右值都有单位
            Unit::strncpy(dest, str1, MAX_STR_SIZE);
            Unit::strncat(dest, symbol, MAX_STR_SIZE);
            Unit::strncat(dest, str2, MAX_STR_SIZE);
        }
    };

    inline constexpr Unit default_unit = Unit();
}
