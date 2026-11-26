# 08 蜂鸣器

状态：正式实现在 `lib/melody`；本主题保留音色实验与样机

## 文档

- `music.md`：`Song`/`Voice`/`Part` 结构、`play` 与 `play_poly` 两种模式、MIDI 转换规则

## 记录

- `buzzer/`：Python 音色模拟脚本与 mc02 中间固件样机。结论是方波旋律加方波伴奏、PWM-DAC 复音；钢琴波表在该蜂鸣器上响度不足，已放弃

## 待办

mc02 `examples-melody` 上板；C 板复音算力未实测，整车只用普通模式
