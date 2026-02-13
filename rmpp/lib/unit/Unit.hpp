#pragma once

#include <cstddef>

namespace unit {
    class Unit {
    public:
        static constexpr size_t MAX_STR_SIZE = 16;
        static constexpr bool ENABLE_STR = false;

        char str[MAX_STR_SIZE]{}; // 单位文本描述
        float scale = 1.0f;       // 单位缩放比例
        bool has_unit = false;    // 是否有单位

        // 无参构造
        constexpr Unit() = default;

        // 用str和scale构造
        constexpr Unit(const char* str, const float scale) {
            strncpy(this->str, str, MAX_STR_SIZE);
            this->scale = scale;
            this->has_unit = strlen(str) != 0;
        }

        // 拷贝构造
        constexpr Unit(const Unit& other) {
            strncpy(this->str, other.str, MAX_STR_SIZE);
            this->scale = other.scale;
            this->has_unit = other.has_unit;
        }

        // 赋值运算，优先用自己的单位，自己无单位时用对方的单位
        constexpr Unit& operator=(const Unit& other) {
            if (!has_unit) { // 自己无单位
                strncpy(this->str, other.str, MAX_STR_SIZE);
                this->scale = other.scale;
                this->has_unit = other.has_unit;
            }
            return *this;
        }

        // 加法运算，优先用自己的单位，自己无单位时用对方的单位
        constexpr Unit operator+(const Unit& other) const {
            Unit ret;
            if (!has_unit) { // 自己无单位
                strncpy(ret.str, other.str, MAX_STR_SIZE);
                ret.scale = other.scale;
                ret.has_unit = other.has_unit;
            } else {
                strncpy(ret.str, this->str, MAX_STR_SIZE);
                ret.scale = this->scale;
                ret.has_unit = this->has_unit;
            }
            return ret;
        }

        // +=运算，优先用自己的单位，自己无单位时用对方的单位
        constexpr Unit& operator+=(const Unit& other) {
            if (!has_unit) { // 自己无单位
                strncpy(this->str, other.str, MAX_STR_SIZE);
                this->scale = other.scale;
                this->has_unit = other.has_unit;
            }
            return *this;
        }

        // 减法运算，优先用自己的单位，自己无单位时用对方的单位
        constexpr Unit operator-(const Unit& other) const {
            Unit ret;
            if (!has_unit) { // 自己无单位
                strncpy(ret.str, other.str, MAX_STR_SIZE);
                ret.scale = other.scale;
                ret.has_unit = other.has_unit;
            } else {
                strncpy(ret.str, this->str, MAX_STR_SIZE);
                ret.scale = this->scale;
                ret.has_unit = this->has_unit;
            }
            return ret;
        }

        // -=运算，优先用自己的单位，自己无单位时用对方的单位
        constexpr Unit& operator-=(const Unit& other) {
            if (!has_unit) { // 自己无单位
                strncpy(this->str, other.str, MAX_STR_SIZE);
                this->scale = other.scale;
                this->has_unit = other.has_unit;
            }
            return *this;
        }

        // 乘法运算
        constexpr Unit operator*(const Unit& other) const {
            Unit ret;
            combineStr(ret.str, this->str, "*", other.str);
            ret.scale = this->scale * other.scale;
            ret.has_unit = true;
            return ret;
        }

        // *=运算
        constexpr Unit& operator*=(const Unit& other) {
            combineStr(this->str, this->str, "*", other.str);
            this->scale *= other.scale;
            this->has_unit = true;
            return *this;
        }

        // 除法运算
        constexpr Unit operator/(const Unit& other) const {
            Unit ret;
            combineStr(ret.str, this->str, "/", other.str);
            ret.scale = scale / other.scale;
            ret.has_unit = true;
            return ret;
        }

        // /=运算
        constexpr Unit& operator/=(const Unit& other) {
            combineStr(this->str, this->str, "/", other.str);
            this->scale /= other.scale;
            this->has_unit = true;
            return *this;
        }

        // 后置实现 Unit * float，因为此时返回值UnitFloat还未定义
        constexpr auto operator*(float value) const;

    private:
        static constexpr size_t strlen(const char* str) {
            size_t len = 0;

            while (str[len] != '\0') {
                len++;
            }

            return len;
        }

        static constexpr void strncat(char* dest, const char* src, const size_t size) {
            if constexpr (ENABLE_STR) {
                // 找到 dest 字符串的末尾
                const size_t dest_start = strlen(dest);
                size_t i = 0;

                // 直到遇到 src 的终止符，或达到 dest 的最大长度
                for (; src[i] != '\0' && dest_start + i < size - 1; i++) {
                    dest[dest_start + i] = src[i];
                }

                // 添加终止符
                dest[dest_start + i] = '\0';
            }
        }

        static constexpr void strncpy(char* dest, const char* src, const size_t size) {
            if constexpr (ENABLE_STR) {
                std::size_t i = 0;

                // 复制源字符串的字符，直到遇到空字符或达到 dest 的最大长度
                for (; src[i] != '\0' && i < size - 1; i++) {
                    dest[i] = src[i];
                }

                // 添加终止符
                dest[i] = '\0';

                // 用空字符填充剩余位置
                // for (; i < size; i++) {
                //     dest[i] = '\0';
                // }
            }
        }

        static constexpr void combineStr(char* dest, const char* str1, const char* symbol, const char* str2) {
            if constexpr (ENABLE_STR) {
                const bool str1_empty = strlen(str1) == 0;
                const bool str2_empty = strlen(str2) == 0;

                // 左右值均无单位，返回空字符串
                if (str1_empty && str2_empty) {
                    strncpy(dest, "", MAX_STR_SIZE);
                    return;
                }

                // 左值无单位
                if (str1_empty) {
                    if (symbol[0] == '/') {
                        strncpy(dest, "1/", MAX_STR_SIZE);
                        strncat(dest, str2, MAX_STR_SIZE);
                        return;
                    } else {
                        strncpy(dest, str2, MAX_STR_SIZE);
                        return;
                    }
                }

                // 右值无单位，用左值的单位
                if (str2_empty) {
                    strncpy(dest, str1, MAX_STR_SIZE);
                    return;
                }

                // 左右值都有单位，拼接单位
                strncpy(dest, str1, MAX_STR_SIZE);
                strncat(dest, symbol, MAX_STR_SIZE);
                strncat(dest, str2, MAX_STR_SIZE);
            }
        }
    };

    inline constexpr Unit default_unit = Unit();
}