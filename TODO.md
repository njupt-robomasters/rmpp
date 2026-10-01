# bsp
- cubemx的串口dma

# 电机库
- dm motor
- dji motor

# imu

# mavlink
移除mavlink
换成minipc，一个auto_aim，一个nav
整理裁判系统协议markdown文档
使用c++重构ui发送

# referee
ui移入referee
referee接收两个串口，独立解析器
referee报文解析增加注册回调
