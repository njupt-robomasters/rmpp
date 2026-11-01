"""把 MIDI 转为蜂鸣器曲目 .hpp，交互输入路径和参数；转换规则见 docs/music.md"""

from array import array
from collections import Counter
from pathlib import Path
import re
import tempfile
import wave
import winsound

import mido

ROOT = Path(__file__).resolve().parents[1]
UNIT = 10  # 时长单位 ms，与 Note::len 一致
MAX_LEN = 255  # Note::len 上限
DRUM = 9  # 打击乐通道（channel 10）不转换
RATE = 22050  # 试听采样率


def ask(prompt, default):
    return input(f"{prompt} [{default}]: ").strip().strip('"') or default


def load_events(midi):
    """合并所有音轨并换算为秒，返回 [(sec, kind, track, channel, note)]，kind 1 松开、2 按下"""
    events = []
    for track, messages in enumerate(midi.tracks):
        tick = 0
        for msg in messages:
            tick += msg.time
            if msg.type == "set_tempo":
                kind = 0
            elif msg.type == "note_off" or (msg.type == "note_on" and msg.velocity == 0):
                kind = 1
            elif msg.type == "note_on":
                kind = 2
            else:
                continue
            events.append((tick, kind, track, msg))

    # 同一 tick 依次处理 tempo、松开、按下；稳定排序保持文件记录顺序
    events.sort(key=lambda e: (e[0], e[1]))

    result = []
    tempo, last_tick, sec = 500000, 0, 0.0
    for tick, kind, track, msg in events:
        sec += mido.tick2second(tick - last_tick, midi.ticks_per_beat, tempo)
        last_tick = tick
        if kind == 0:
            tempo = msg.tempo
        elif msg.channel != DRUM:
            result.append((sec, kind, track, msg.channel, msg.note))
    return result


def convert(events, tracks, max_rest):
    """单音抢占并对齐到网格，返回 [(midi, len)]"""
    # 唯一活跃音符：空闲时按下即占用，占用期间更高的音抢占，其余按下忽略；活跃音符松开后记为休止，不回退到仍按住的音
    active = None
    changes = []
    end = 0.0
    for sec, kind, track, channel, note in events:
        if track not in tracks:
            continue
        end = sec
        key = (track, channel, note)
        if kind == 1 and active == key:
            active = None
            changes.append((sec, 0))
        elif kind == 2 and (active is None or note > active[2]):
            active = key
            changes.append((sec, note))
    changes.append((end, None))

    # 各切换时刻四舍五入到网格后再求时长，误差不累积
    grid = [(int(sec * 1000 / UNIT + 0.5), midi) for sec, midi in changes]
    segs = []
    for (t, midi), (t_next, midi_next) in zip(grid, grid[1:]):
        length = t_next - t

        # 同音连击之间的休止被对齐为 0 时，从前一个音借 1 个单位，保证能听出重击
        if midi == 0 and length == 0 and segs and segs[-1][0] == midi_next and segs[-1][1] > 1:
            segs[-1][1] -= 1
            length = 1

        if length == 0:
            continue
        if segs and segs[-1][0] == midi:
            segs[-1][1] += length
        else:
            segs.append([midi, length])

    # 限制休止时长，去掉结尾休止
    for seg in segs:
        if seg[0] == 0:
            seg[1] = min(seg[1], max_rest)
    while segs and segs[-1][0] == 0:
        segs.pop()

    # 超过上限的拆成多个同音条目，播放器会连续发声
    notes = []
    for midi, length in segs:
        while length > 0:
            notes.append((midi, min(length, MAX_LEN)))
            length -= MAX_LEN
    return notes


def preview(notes):
    """合成方波试听，与播放器相同：同音连续不断开，休止静音"""
    samples = array("h")
    phase = 0.0
    for midi, length in notes:
        count = RATE * length * UNIT // 1000
        if midi == 0:
            samples.extend([0] * count)
            continue
        step = 440 * 2 ** ((midi - 69) / 12) / RATE
        for _ in range(count):
            samples.append(6000 if phase < 0.5 else -6000)
            phase = (phase + step) % 1

    path = Path(tempfile.gettempdir()) / "midi_to_music_preview.wav"
    with wave.open(str(path), "wb") as file:
        file.setnchannels(1)
        file.setsampwidth(2)
        file.setframerate(RATE)
        file.writeframes(samples.tobytes())
    winsound.PlaySound(str(path), winsound.SND_FILENAME | winsound.SND_ASYNC)
    input("试听中，回车停止")
    winsound.PlaySound(None, 0)
    path.unlink()


def main():
    while not (path := Path(input("MIDI 路径: ").strip().strip('"'))).is_file():
        print("文件不存在")
    midi = mido.MidiFile(path)
    max_rest = max(1, round(int(ask("休止最长时间 ms", "1000")) / UNIT))

    events = load_events(midi)
    counts = Counter(e[2] for e in events if e[1] == 2)
    if not counts:
        print("没有可转换的音符")
        return

    # 选择音轨，可先试听转换结果；省略序号表示全部合并
    for track in sorted(counts):
        print(f"  {track}: {midi.tracks[track].name or '-'}，{counts[track]} 个音符")
    print("输入音轨序号选择，回车全部合并；加 p 试听，如 p 0、p")
    while True:
        cmd = input("> ").strip().lower()
        listen = cmd.startswith("p")
        arg = cmd[1:].strip() if listen else cmd
        if arg in ("", "a"):
            tracks = set(counts)
        elif arg.isdigit() and int(arg) in counts:
            tracks = {int(arg)}
        else:
            print("无效输入")
            continue
        if not listen:
            break
        preview(convert(events, tracks, max_rest))

    notes = convert(events, tracks, max_rest)
    output = ROOT / ask("输出目录", "lib/misc/music")
    name = re.sub(r"[^0-9a-zA-Z]+", "_", path.stem).strip("_").lower() or "music"
    if name[0].isdigit():
        name = "music_" + name
    name = ask("曲目名", name)
    file = output / f"{name}.hpp"
    if file.exists() and input(f"{file} 已存在，覆盖？[y/N]: ").strip().lower() != "y":
        return

    body = "".join(f"    {{{midi}, {length}}},\n" for midi, length in notes)
    output.mkdir(parents=True, exist_ok=True)
    file.write_text(
        f'#pragma once\n\n#include "misc/melody.hpp"\n\ninline constexpr Note {name.upper()}[] = {{\n{body}}};\n',
        encoding="utf-8",
        newline="\n",
    )
    print(f"已写入 {file}：{len(notes)} 条，{sum(n[1] for n in notes) * UNIT / 1000:.2f} s")


if __name__ == "__main__":
    main()
