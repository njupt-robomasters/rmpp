# 蜂鸣器曲目

曲目是 `Note` 数组，每首一个 `.hpp`，放在 `lib/misc/music/`，由 `tools/midi_to_music.py` 从 MIDI 生成。

```cpp
#include "misc/melody.hpp"
#include "misc/music/c5d5g5.hpp"

Melody melody; // 使用板载蜂鸣器，周期调用 melody.update()
melody.play(C5D5G5);
```

## 格式

```cpp
struct Note {
    uint8_t midi; // MIDI 音符号，69 为 A4 = 440 Hz；0 为休止
    uint8_t len;  // 时长，单位 10 ms，最大 255 即 2550 ms
};
```

`Melody` 只在 midi 变化时操作 BSP `Buzzer`：发声调用 `set_tone(hz)`（板卡实现为 50% 占空比），休止调用 `mute()`。相邻同音条目连续发声，需要断开时在数据中写休止。

## 转换

```sh
python tools/midi_to_music.py
```

依赖系统 Python 已安装的 `mido`，试听使用 `winsound`，仅支持 Windows。脚本依次询问：

| 输入 | 默认 | 说明 |
| --- | --- | --- |
| MIDI 路径 | 无 | 可拖入文件，自动去掉引号 |
| 休止最长时间 | 1000 ms | 限制 MIDI 中的长空白 |
| 音轨 | 全部合并 | 输入序号选择，`p 序号` 试听单轨，`p` 试听合并结果；单音轨同样可试听 |
| 输出目录 | `lib/misc/music` | 相对项目根目录 |
| 曲目名 | MIDI 文件名的 snake_case | 文件名为 `曲目名.hpp`，数组名大写 |

试听播放的是转换后的结果，与板上行为一致。

## 规则

1. 合并所选音轨，按 tempo 变化把 tick 换算为时间；打击乐通道 10 不转换
2. 同一 tick 先处理松开再处理按下
3. 唯一活跃音符：空闲时按下即占用；占用期间更高的音按下立即抢占，不高于当前音的按下忽略，同时按下的和弦因此取最高音；活跃音符松开后记为休止，不回退到仍按住的音，直到下一次按下
4. 每个切换时刻四舍五入到 10 ms 网格后再求时长，误差不累积；时长为 0 的段丢弃，相邻同音段合并
5. 同音重击之间的休止被对齐为 0 时，从前一个音借 10 ms 作为休止，保证能听出重击
6. 休止超过设定上限时截短，去掉结尾休止
7. 超过 2550 ms 的音拆成多个同音条目，总时长不变
