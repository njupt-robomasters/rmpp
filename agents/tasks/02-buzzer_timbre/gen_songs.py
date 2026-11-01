"""用 tools/midi_to_song.py 的规则批量生成测试曲目，音轨分组已人工确认；并把旧单声部曲目包装为 Song"""

import re
import sys
from pathlib import Path

import mido

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tools"))
import midi_to_song as m2s  # noqa: E402

OUT = ROOT / "lib/melody/music"

# (MIDI, 曲目名, 旋律音轨, 伴奏音轨)；其余音轨不用
SONGS = [
    ("always_with_me.mid", "always_with_me", {2, 3}, {5, 6}),
    ("你_从第15小节.mid", "you", {1}, {2}),
    ("平凡之路.mid", "the_ordinary_road", {2}, {3}),  # 音轨 1 主旋和音、音轨 4 伴奏和音为重复配器
]

for file, name, melody_tracks, chord_tracks in SONGS:
    midi = mido.MidiFile(ROOT / "reference/midi" / file)
    tracks = m2s.load_tracks(midi)
    names = {i: m2s.track_name(midi.tracks[i]) for i in tracks}
    suggested, _ = m2s.suggest(tracks, names)
    roles = {i: "m" if i in melody_tracks else "c" if i in chord_tracks else "x" for i in tracks}
    melody, chords = m2s.build(tracks, roles)
    assert len(melody) <= m2s.MAX_VOICES and len(chords) <= m2s.MAX_VOICES
    (OUT / f"{name}.hpp").write_text(m2s.header(name, file, melody, chords), encoding="utf-8", newline="\n")
    same = "一致" if suggested == roles else f"不同：建议 {suggested}"
    print(f"{name}: 旋律 {len(melody)} 声部，伴奏 {len(chords)} 声部；工具建议分组{same}")

# 旧单声部曲目数据不变，包装为只有旋律主声部的 Song
for name in ("c5d5g5", "g5g5g5"):
    path = OUT / f"{name}.hpp"
    text = path.read_text(encoding="utf-8")
    if "Song" in text:
        continue
    notes = [(int(a), int(b)) for a, b in re.findall(r"\{(\d+), (\d+)\}", text)]
    path.write_text(m2s.header(name, "原单声部曲目", [notes], []), encoding="utf-8", newline="\n")
    print(f"{name}: 包装 {len(notes)} 条")
