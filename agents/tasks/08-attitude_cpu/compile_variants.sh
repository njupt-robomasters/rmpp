#!/bin/sh
# 单独编译 quaternion_ekf.cpp 的多种选项，统计 solve() 代码量与调用的外部函数数量
# 用法：sh agents/tasks/08-attitude_cpu/compile_variants.sh（仓库根目录）
set -e
OUT=agents/tasks/08-attitude_cpu/out
SRC=lib/attitude/quaternion_ekf.cpp
BASE="-DEIGEN_DONT_VECTORIZE -DEIGEN_MAX_ALIGN_BYTES=0 -Ilib -Ilib/third_party/eigen-3.4.0 -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -fdata-sections -ffunction-sections -fno-rtti -fno-exceptions -fno-threadsafe-statics -std=gnu++23 -g"
for v in "Og:-Og" "Og_NDEBUG:-Og -DNDEBUG" "O1:-O1" "O2:-O2"; do
    name=${v%%:*}; flags=${v#*:}
    arm-none-eabi-g++ $BASE $flags -c $SRC -o $OUT/$name.o
    arm-none-eabi-objdump -d -C $OUT/$name.o > $OUT/$name.s
    text=$(arm-none-eabi-size $OUT/$name.o | awk 'NR==2{print $1}')
    funcs=$(grep -c '^[0-9a-f]* <' $OUT/$name.s)
    bl=$(grep -c -E '\sbl\s' $OUT/$name.s)
    echo "$name: text=$text B, functions=$funcs, bl=$bl"
done
