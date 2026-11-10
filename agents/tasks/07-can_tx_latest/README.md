# CAN 发送：同 ID 只留最新（已撤销）

## 结论

- 曾实现并在两板上板验证“同 ID 只留最新一帧、满时撤销最早写入”，用户最终撤销，代码恢复为写入硬件、满时丢弃
- 撤销原因：CAN ACK 由任一正常接收的节点回复，某个电机离线不会让帧积压；改为开启自动重发、按写入顺序发送（C 板 Transmit Fifo Priority = Enable，mc02 TX FIFO 模式）
- 长期结论已写入 `docs/bsp.md` FDCAN 一节

## 实测得到的硬件知识（2026-11-10，CAN1 与 CAN2 短接，关闭自动重发时测得）

- 撤销未在发送的待发帧：mc02 约 70 ns，C 板约 0.19 µs 生效；正在发送的帧要等本次尝试结束
- mc02 FIFO 模式撤销中间元素不释放空位（TFFL 不变），读指针越过后才回收；Queue 模式撤销后立即可用，且 TFFL、TFGI 恒为 0
- CubeMX 要求 FDCAN Tx Buffers Nbr + Tx Fifo Queue Elmts Nbr = 32
- 初始化模式（C 板 `MCR.INRQ`、mc02 `CCCR.INIT`）下写入的帧保持待发，撤销立即生效，可用于暂停发送做测试

## 测试程序

`app/`（mc02）、`app_c/`（C 板）测试的是已撤销的实现，仅作暂停发送、寄存器读取等测试写法参考，与当前代码行为不一致
