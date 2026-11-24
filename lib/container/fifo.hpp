#pragma once

#include <algorithm>

// 定长环形队列：[] 按队首起的逻辑下标访问，0 为队首；不加锁，跨中断使用由调用方保护
template <typename T, size_t N>
class Fifo {
  public:
    size_t size = 0; // 元素个数；外部只读

  private:
    T data[N]{};
    size_t head = 0; // 队首下标

  public:
    // 追加到队尾；满时弹出最旧元素
    void push(const T& value) {
        if (size == N)
            pop(1);
        data[(head + size) % N] = value;
        size++;
    }

    // 按序追加，返回被挤出的元素总数；单批超过容量时仍只保留最新 N 个
    size_t push(const T* values, const size_t n) {
        const size_t dropped = n - std::min(n, N - size);
        for (size_t i = 0; i < n; i++)
            push(values[i]);
        return dropped;
    }

    // 复制队首至多 n 个，不取出，返回实际个数
    size_t peek(T* values, const size_t n) const {
        const size_t count = std::min(n, size);
        for (size_t i = 0; i < count; i++)
            values[i] = data[(head + i) % N];
        return count;
    }

    // 复制并取出队首至多 n 个，返回实际个数
    size_t pop(T* values, const size_t n) {
        const size_t count = peek(values, n);
        pop(count);
        return count;
    }

    // 丢弃队首至多 n 个
    void pop(size_t n) {
        n = std::min(n, size);
        head = (head + n) % N;
        size -= n;
    }

    void clear() {
        head = 0;
        size = 0;
    }

    // i 须小于 size
    T& operator[](const size_t i) {
        return data[(head + i) % N];
    }

    const T& operator[](const size_t i) const {
        return data[(head + i) % N];
    }
};
