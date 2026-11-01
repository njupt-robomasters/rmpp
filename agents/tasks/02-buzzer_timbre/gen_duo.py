"""从 MIDI 生成双声部曲目头文件 app/test/buzzer_duo/songs.hpp

旋律：旋律通道内最高音抢占；伴奏：伴奏通道内最低音抢占并升八度；规则与 simulate.py 的 7 系列相同
每个非休止条目是一次起音，时长单位 10 ms，切换时刻对齐到 10 ms 网格
"""

import sys
from pathlib import Path

import mido

sys.path.insert(0, str(Path(__file__).resolve().parent))
from simulate import ROOT, load_notes, mono_segments  # noqa: E402

OUT = ROOT / "app/test/buzzer_duo/songs.hpp"
UNIT = 0.01  # 时长单位 s
BASS_SHIFT = 12  # 伴奏移调，半音

# (MIDI，常量名，旋律通道，伴奏通道)；通道布局因文件而异，按音轨名确认
SONGS = [
    ("reference/midi/你_从第15小节.mid", "NI", {0}, {1}),
    ("reference/midi/always_with_me.mid", "ALWAYS_WITH_ME", {0, 1}, {2, 3}),
]


def to_entries(segs):
    """[(start, end, note)] 转为 [(midi, len)]，0 为休止；对齐后时长为 0 的音丢弃"""
    entries, pos = [], 0
    for start, end, note in segs:
        a, b = round(start / UNIT), round(end / UNIT)
        if b <= a:
            continue
        if a > pos:
            entries.append((0, a - pos))
        entries.append((note, b - a))
        pos = b
    return entries


def array(name, entries):
    assert entries and max(n for n, _ in entries) < 128 and max(n for _, n in entries) < 65536
    rows = "".join(f"    {{{note}, {length}}},\n" for note, length in entries)
    return f"inline constexpr VoiceNote {name}[] = {{\n{rows}}};\n"


def main():
    body = []
    for path, name, melody_channels, bass_channels in SONGS:
        midi = mido.MidiFile(ROOT / path)
        melody = to_entries(mono_segments(load_notes(midi, melody_channels), True, 0))
        bass = to_entries(mono_segments(load_notes(midi, bass_channels), False, BASS_SHIFT))
        body.append(
            f"// {Path(path).name}\n"
            + array(f"{name}_MELODY", melody)
            + "\n"
            + array(f"{name}_BASS", bass)
            + f"\ninline constexpr Song {name} = {{{name}_MELODY, {name}_BASS}};\n"
        )
        seconds = max(sum(n for _, n in melody), sum(n for _, n in bass)) * UNIT
        print(f"{name}: 旋律 {len(melody)} 条，伴奏 {len(bass)} 条，{seconds:.1f} s")

    OUT.write_text(
        "// 由 agents/tasks/02-buzzer_timbre/gen_duo.py 生成，勿手改\n\n"
        "#pragma once\n\n"
        "#include <cstdint>\n#include <span>\n\n"
        "// 单声部条目：每个非休止条目是一次起音\n"
        "struct VoiceNote {\n"
        "    uint8_t midi; // MIDI 音符号，69 为 A4 = 440 Hz；0 为休止\n"
        "    uint16_t len; // 时长，单位 10 ms\n"
        "};\n\n"
        "// 双声部曲目\n"
        "struct Song {\n"
        "    std::span<const VoiceNote> melody;\n"
        "    std::span<const VoiceNote> bass;\n"
        "};\n\n" + "\n".join(body),
        encoding="utf-8",
        newline="\n",
    )
    print(f"已写入 {OUT}")


if __name__ == "__main__":
    main()
