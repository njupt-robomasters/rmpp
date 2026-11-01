"""从 MIDI 生成多声部曲目头文件 app/test/buzzer_poly/songs.hpp，供各曲目工程共用

按通道分为旋律组（右手）与伴奏组（左手），槽位按需新增，不丢音；每槽输出一个 VoiceNote 数组（非休止条目即一次起音，时长单位 10 ms）
槽 0 为主线：旋律组取最高音、伴奏组取最低音，更极端的音抢占，与 tools/midi_to_music.py 的单槽规则相同
其余槽为内声部：组内其余按下的音及被主线抢占的音放入编号最小的空槽，无空槽时新增槽位
伴奏低于 CHORD_MIN 的音按八度升高，蜂鸣器发不出过低的音
"""

import sys
from pathlib import Path

import mido

sys.path.insert(0, str(Path(__file__).resolve().parent))
from simulate import ROOT, load_notes  # noqa: E402

OUT = ROOT / "app/test/buzzer_poly/songs.hpp"
UNIT = 0.01  # 时长单位 s
CHORD_MIN = 48  # 伴奏最低音 C3

# (MIDI，常量名，旋律通道，伴奏通道)；通道布局因文件而异，按音轨名确认
SONGS = [
    ("reference/midi/你_从第15小节.mid", "NI", {0}, {1}),
    ("reference/midi/always_with_me.mid", "ALWAYS_WITH_ME", {0, 1}, {2, 3}),
    # 主旋律（钢琴）通道 0、伴奏（吉他）通道 1；不取主旋和音（弦乐）通道 2 与伴奏和音通道 4（与通道 1 完全重复）
    ("reference/midi/平凡之路.mid", "PING_FAN_ZHI_LU", {0}, {1}),
]


def allocate(notes, prefer_high):
    """把 [(start, end, note, velocity)] 分到槽位，返回各槽 [(start, end, note)]"""
    sign = -1 if prefer_high else 1
    # 同一时刻先松开再按下，按下时极端音在前，和弦的最高（最低）音占主线
    events = sorted(
        [(e, 0, 0, i) for i, (_, e, _, _) in enumerate(notes)] + [(s, 1, sign * n, i) for i, (s, _, n, _) in enumerate(notes)]
    )
    slots, active = [[]], [None]  # active 各槽为 (note 序号, 段起点, 音高)

    def close(k, t):
        _, since, pitch = active[k]
        if t > since:
            slots[k].append((since, t, pitch))
        active[k] = None

    def put_inner(item):
        free = [k for k in range(1, len(active)) if active[k] is None]
        if not free:
            free = [len(active)]
            active.append(None)
            slots.append([])
        active[free[0]] = item

    for t, is_on, _, i in events:
        if not is_on:
            for k in range(len(active)):
                if active[k] and active[k][0] == i:
                    close(k, t)
            continue

        # 主线：空闲或更极端时占用，被抢占的音转入内声部继续发声（重新起音）
        pitch = notes[i][2]
        if active[0] is None or sign * pitch < sign * active[0][2]:
            if active[0]:
                old = active[0][0]
                close(0, t)
                put_inner((old, t, notes[old][2]))
            active[0] = (i, t, pitch)
        else:
            put_inner((i, t, pitch))
    return slots


def to_entries(segs):
    """[(start, end, note)] 转为 [(midi, len)]，0 为休止；切换时刻对齐到 UNIT 网格，对齐后时长为 0 的音丢弃

    同音重击之间没有休止时，从前一个音借 1 个 UNIT 作休止，保证能听出重击（与 tools/midi_to_music.py 规则相同）
    """
    entries, pos = [], 0
    for start, end, note in segs:
        a, b = round(start / UNIT), round(end / UNIT)
        if b <= a:
            continue
        if a > pos:
            entries.append((0, a - pos))
        elif entries and entries[-1][0] == note and entries[-1][1] > 1:
            entries[-1] = (note, entries[-1][1] - 1)
            entries.append((0, 1))
        entries.append((note, b - a))
        pos = b
    return entries


def raise_low(notes):
    """低于 CHORD_MIN 的音按八度升高"""
    result = []
    for s, e, n, v in notes:
        while n < CHORD_MIN:
            n += 12
        result.append((s, e, n, v))
    return result


def array(name, entries):
    assert max(n for n, _ in entries) < 128 and max(n for _, n in entries) < 65536
    rows = "".join(f"    {{{note}, {length}}},\n" for note, length in entries)
    return f"inline constexpr VoiceNote {name}[] = {{\n{rows}}};\n"


def group(name, notes, segs):
    """输出一组槽位数组与槽位表，并打印覆盖率（按音符时长）"""
    names = [f"{name}_{k}" for k in range(len(segs))]
    total = sum(e - s for s, e, _, _ in notes)
    played = sum(e - s for slot in segs for s, e, _ in slot)
    print(f"  {name}: {len(segs)} 槽，覆盖 {played / total * 100:.0f}%")
    body = "".join(array(n, to_entries(s) or [(0, 1)]) + "\n" for n, s in zip(names, segs))
    return body + f"inline constexpr std::span<const VoiceNote> {name}[] = {{{', '.join(names)}}};\n"


def main():
    body, max_slots = [], 0
    for path, name, melody_channels, chord_channels in SONGS:
        midi = mido.MidiFile(ROOT / path)
        melody = load_notes(midi, melody_channels)
        chords = raise_low(load_notes(midi, chord_channels))
        melody_slots, chord_slots = allocate(melody, True), allocate(chords, False)
        max_slots = max(max_slots, len(melody_slots), len(chord_slots))
        print(name)
        body.append(
            f"// {Path(path).name}\n"
            + group(f"{name}_MELODY", melody, melody_slots)
            + "\n"
            + group(f"{name}_CHORDS", chords, chord_slots)
            + f"\ninline constexpr Song {name} = {{{name}_MELODY, {name}_CHORDS}};\n"
        )

    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(
        "// 由 agents/tasks/02-buzzer_timbre/gen_poly.py 生成，勿手改\n\n"
        "#pragma once\n\n"
        "#include <cstddef>\n#include <cstdint>\n#include <span>\n\n"
        f"inline constexpr size_t MAX_SLOTS = {max_slots}; // 各曲各组槽位数上限，槽 0 为主线\n\n"
        "// 单声部条目：每个非休止条目是一次起音\n"
        "struct VoiceNote {\n"
        "    uint8_t midi; // MIDI 音符号，69 为 A4 = 440 Hz；0 为休止\n"
        "    uint16_t len; // 时长，单位 10 ms\n"
        "};\n\n"
        "// 多声部曲目：旋律组与伴奏组，各槽为一个单声部\n"
        "struct Song {\n"
        "    std::span<const std::span<const VoiceNote>> melody;\n"
        "    std::span<const std::span<const VoiceNote>> chords;\n"
        "};\n\n" + "\n".join(body),
        encoding="utf-8",
        newline="\n",
    )
    print(f"已写入 {OUT}")


if __name__ == "__main__":
    main()
