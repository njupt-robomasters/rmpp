#!/bin/sh
# 烧录 out/<变体>/rmpp.elf，运行 18 s 后读取统计变量（需 C 板连接 CMSIS-DAP）
# 用法：sh agents/tasks/08-attitude_cpu/measure.sh og og_nodebug ...（仓库根目录）
T=agents/tasks/08-attitude_cpu
for name in "$@"; do
    elf=$T/out/$name/rmpp.elf
    addr() { arm-none-eabi-nm -C "$elf" | awk -v s="$1" '$3==s{print "0x"$1}'; }
    openocd -f boards/c_board/openocd.cfg -c "init; reset halt; program $elf verify; reset run; sleep 18000; \
        echo [mdw $(addr imu_cycles) 3]; echo [mdw $(addr attitude_cycles) 3]; echo [mdw $(addr app_monitor) 4]; echo [mdw $(addr SystemCoreClock)]; shutdown" \
        > $T/out/$name.measure.log 2>&1
    echo "== $name"
    grep -E "^0x" $T/out/$name.measure.log | python -c "
import sys,struct
rows=[l.split(':')[1].split() for l in sys.stdin]
f=lambda h:struct.unpack('<f',struct.pack('<I',int(h,16)))[0]
clk=int(rows[3][0],16)
for n,r in (('imu',rows[0]),('attitude',rows[1])):
    print(f'{n}: last={int(r[0],16)} max={int(r[1],16)} avg={f(r[2]):.0f} cycles = {f(r[2])/clk*1e6:.1f} us')
m=[f(x) for x in rows[2]]
print(f'monitor: freq={m[0]:.0f} Hz cpu={m[2]*100:.1f}% cpu_lpf={m[3]*100:.1f}%  clk={clk}')
"
done
