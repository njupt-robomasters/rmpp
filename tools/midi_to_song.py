"""把 MIDI 转为蜂鸣器曲目 .hpp（Song：旋律组 + 伴奏组），交互选择音轨并试听；规则见 agents/docs/music.md

用法：python tools/midi_to_song.py [MIDI 路径]
依赖：mido；试听用 winsound，仅 Windows
"""

from array import array
from pathlib import Path
import math
import re
import sys
import tempfile
import wave
import winsound

import mido

ROOT = Path(__file__).resolve().parents[1]
UNIT = 0.01  # Note::len 单位 s
MAX_LEN = 255  # Note::len 上限
MAX_VOICES = 8  # 每组声部上限，与 lib/melody/melody.hpp 一致
CHORD_MIN = 48  # 伴奏低于 C3 的音按八度升高，蜂鸣器发不出过低的音
DRUM = 9  # 打击乐通道（channel 10）不转换
RATE = 16000  # 试听采样率
PREVIEW_SECONDS = 20  # 试听时长 s

# 音轨名关键字，用于建议分组
MELODY_HINTS = ("rh", "right", "melody", "主旋律", "右手", "pianor")
CHORD_HINTS = ("lh", "left", "bass", "伴奏", "左手", "pianol")


def ask(prompt, default):
    return input(f"{prompt} [{default}]: ").strip().strip('"') or default


def track_name(track):
    """音轨名；MIDI 常用 GBK 存中文，mido 按 latin-1 读取，这里还原"""
    try:
        return track.name.encode("latin-1").decode("gbk")
    except (UnicodeEncodeError, UnicodeDecodeError):
        return track.name


def load_tracks(midi):
    """各音轨的音符 [(start, end, note)]，单位 s，按 tempo 换算；只含有音符的音轨"""
    tempos = sorted((tick, msg.tempo) for track in midi.tracks for tick, msg in timed(track) if msg.type == "set_tempo")

    def seconds(tick):
        sec, last_tick, tempo = 0.0, 0, 500000
        for t, value in tempos:
            if t >= tick:
                break
            sec += mido.tick2second(t - last_tick, midi.ticks_per_beat, tempo)
            last_tick, tempo = t, value
        return sec + mido.tick2second(tick - last_tick, midi.ticks_per_beat, tempo)

    result = {}
    for index, track in enumerate(midi.tracks):
        held, notes = {}, []
        for tick, msg in timed(track):
            if msg.type not in ("note_on", "note_off") or msg.channel == DRUM:
                continue
            key = (msg.channel, msg.note)
            if key in held:
                notes.append((seconds(held.pop(key)), seconds(tick), msg.note))
            if msg.type == "note_on" and msg.velocity > 0:
                held[key] = tick
        if notes:
            result[index] = sorted(notes)
    return result


def timed(track):
    """(绝对 tick, msg)；同一 tick 内松开排在按下之前"""
    tick, items = 0, []
    for msg in track:
        tick += msg.time
        is_on = msg.type == "note_on" and msg.velocity > 0
        items.append((tick, is_on, msg))
    items.sort(key=lambda item: (item[0], item[1]))
    return [(tick, msg) for tick, _, msg in items]


def max_polyphony(notes):
    events = sorted([(s, 1) for s, _, _ in notes] + [(e, -1) for _, e, _ in notes])
    count = peak = 0
    for _, d in events:
        count += d
        peak = max(peak, count)
    return peak


def suggest(tracks, names):
    """按音轨名与内容建议分组：'m' 旋律、'c' 伴奏、'x' 不用，并给出原因"""
    roles, reasons = {}, {}
    for index in tracks:
        name = names[index].lower()
        if any(h in name for h in MELODY_HINTS):
            roles[index] = "m"
        elif any(h in name for h in CHORD_HINTS):
            roles[index] = "c"

    # 与前面音轨完全相同、或起音全部被包含的音轨是重复配器，叠加会变响并互相干扰
    onsets = {i: {(round(s, 2), n) for s, _, n in tracks[i]} for i in tracks}
    for i in tracks:
        for j in tracks:
            if j != i and onsets[i] <= onsets[j] and (len(onsets[i]) < len(onsets[j]) or j < i):
                roles[i] = "x"
                reasons[i] = f"与音轨 {j} 重复"
                break

    # 无名音轨按音高归组
    for index in tracks:
        if index not in roles:
            pitches = sorted(n for _, _, n in tracks[index])
            roles[index] = "m" if pitches[len(pitches) // 2] >= 60 else "c"
            reasons[index] = "按音高推测"
    return roles, reasons


def allocate(notes, prefer_high):
    """分配声部：声部 0 为主声部，更高（伴奏为更低）的音抢占，松开即休止，与普通模式一致；
    其余音及被抢占的音进入编号最小的空声部，无空声部时新增。返回各声部 [(start, end, note)]"""
    sign = -1 if prefer_high else 1
    events = sorted([(e, 0, 0, i) for i, (_, e, _) in enumerate(notes)] + [(s, 1, sign * n, i) for i, (s, _, n) in enumerate(notes)])
    voices, active = [[]], [None]  # active 为 (音符序号, 段起点, 音高)

    def close(k, t):
        _, since, pitch = active[k]
        if t > since:
            voices[k].append((since, t, pitch))
        active[k] = None

    def put_inner(item):
        free = [k for k in range(1, len(active)) if active[k] is None]
        if not free:
            free = [len(active)]
            active.append(None)
            voices.append([])
        active[free[0]] = item

    for t, is_on, _, i in events:
        if not is_on:
            for k in range(len(active)):
                if active[k] and active[k][0] == i:
                    close(k, t)
            continue
        pitch = notes[i][2]
        if active[0] is None or sign * pitch < sign * active[0][2]:
            if active[0]:
                old = active[0][0]
                close(0, t)
                put_inner((old, t, notes[old][2]))
            active[0] = (i, t, pitch)
        else:
            put_inner((i, t, pitch))
    return voices


def to_entries(segs, origin):
    """[(start, end, note)] 转为 [(midi, len)]：切换时刻对齐到 10 ms 网格，误差不累积；
    同音重击之间无休止时从前一个音借 10 ms 作休止；超过 2550 ms 的条目拆分"""
    merged, pos = [], 0
    for start, end, note in segs:
        a, b = round((start - origin) / UNIT), round((end - origin) / UNIT)
        if b <= a:
            continue
        if a > pos:
            merged.append([0, a - pos])
        elif merged and merged[-1][0] == note and merged[-1][1] > 1:
            merged[-1][1] -= 1
            merged.append([0, 1])
        merged.append([note, b - a])
        pos = b

    entries = []
    for note, length in merged:
        while length > 0:
            entries.append((note, min(length, MAX_LEN)))
            length -= MAX_LEN
    return entries


def build(tracks, roles):
    """按分组生成 (旋律声部列表, 伴奏声部列表)，每个声部为 [(midi, len)]；全曲首音对齐到 0"""
    melody = [n for i in tracks if roles[i] == "m" for n in tracks[i]]
    chords = []
    for i in tracks:
        if roles[i] == "c":
            for s, e, n in tracks[i]:
                while n < CHORD_MIN:
                    n += 12
                chords.append((s, e, n))
    origin = min(s for s, _, _ in melody + chords)
    melody_voices = [to_entries(v, origin) for v in allocate(melody, True)] if melody else []
    chord_voices = [to_entries(v, origin) for v in allocate(chords, False)] if chords else []
    return melody_voices, chord_voices


def render(voice_groups, start, seconds):
    """试听合成：voice_groups 为 [(声部列表, 份额)]；方波加指数衰减，近似板上复音模式"""
    count = int(seconds * RATE)
    out = [0.0] * count
    first = int(start / UNIT)
    for voices, share in voice_groups:
        weight = share / math.sqrt(max(len(voices), 1))
        for voice in voices:
            pos, current, attack = 0, 0, 0
            for note, length in voice:
                if note != current:
                    attack = pos
                    current = note
                a, b = max(pos - first, 0), min(pos + length - first, int(seconds / UNIT))
                if note and b > a:
                    hz = 440 * 2 ** ((note - 69) / 12)
                    for k in range(int(a * UNIT * RATE), min(int(b * UNIT * RATE), count)):
                        t = k / RATE + start
                        age = t - attack * UNIT
                        amp = 0.3 + 0.7 * math.exp(-age / 0.5)
                        out[k] += weight * amp * (1 if (hz * t) % 1 < 0.5 else -1)
                pos += length
    peak = max(1.0, max(abs(x) for x in out) if out else 1.0)
    return array("h", (int(8000 * x / peak) for x in out))


def play(samples):
    path = Path(tempfile.gettempdir()) / "midi_to_song_preview.wav"
    with wave.open(str(path), "wb") as file:
        file.setnchannels(1)
        file.setsampwidth(2)
        file.setframerate(RATE)
        file.writeframes(samples.tobytes())
    winsound.PlaySound(str(path), winsound.SND_FILENAME | winsound.SND_ASYNC)
    input("试听中，回车停止")
    winsound.PlaySound(None, 0)
    path.unlink()


def show(tracks, names, roles, reasons):
    labels = {"m": "旋律", "c": "伴奏", "x": "不用"}
    print("\n  序号  分组  音符  音域     最多同时  音轨名")
    for i, notes in tracks.items():
        pitches = [n for _, _, n in notes]
        note = f"  （建议：{reasons[i]}）" if i in reasons else ""
        print(f"  {i:>4}  {labels[roles[i]]}  {len(notes):>4}  {min(pitches):>3}-{max(pitches):<3}  {max_polyphony(notes):>8}  {names[i] or '-'}{note}")


HELP = """
按音轨分组：旋律组主声部取最高音（普通模式只播它），伴奏组主声部取最低音，其余同时发声的音进入内声部
  m 序号...   设为旋律        c 序号...   设为伴奏        x 序号...   不用
  p 序号      试听单个音轨    p          试听复音模式    pm         试听普通模式（旋律主声部）
  t 秒        设置试听起点    回车       确认分组，进入输出
试听为电脑近似合成，音量与板上不同，只用于确认音轨内容
"""


def choose(tracks, names):
    roles, reasons = suggest(tracks, names)
    start = 0.0
    print(HELP)
    while True:
        show(tracks, names, roles, reasons)
        print(f"  试听起点 {start:.0f} s，时长 {PREVIEW_SECONDS} s")
        words = input("> ").strip().lower().split()
        if not words:
            if "m" not in roles.values():
                print("至少需要一个旋律音轨")
                continue
            return roles

        cmd, args = words[0], words[1:]
        if cmd in ("m", "c", "x") and args and all(a.isdigit() and int(a) in tracks for a in args):
            for a in args:
                roles[int(a)] = cmd
                reasons.pop(int(a), None)
        elif cmd == "t" and len(args) == 1 and args[0].replace(".", "", 1).isdigit():
            start = float(args[0])
        elif cmd == "p" and len(args) == 1 and args[0].isdigit() and int(args[0]) in tracks:
            voices, _ = build({0: tracks[int(args[0])]}, {0: "m"})
            play(render([(voices, 1.0)], start, PREVIEW_SECONDS))
        elif cmd == "p" and not args:
            melody, chords = build(tracks, roles)
            share = 0.7 if chords else 1.0
            play(render([(melody, share), (chords, 1 - share)], start, PREVIEW_SECONDS))
        elif cmd == "pm" and not args:
            melody, _ = build(tracks, roles)
            play(render([(melody[:1], 1.0)], start, PREVIEW_SECONDS))
        else:
            print("无效输入" + HELP)


def header(name, source, melody, chords):
    const = name.upper()

    def part(group, voices):
        names = [f"{const}_{group}_{k}" for k in range(len(voices))]
        body = ""
        for array_name, voice in zip(names, voices):
            rows = "".join(f"    {{{n}, {length}}},\n" for n, length in voice)
            body += f"inline constexpr Note {array_name}[] = {{\n{rows}}};\n\n"
        return body + f"inline constexpr Voice {const}_{group}[] = {{{', '.join(names)}}};\n\n"

    text = f"// 由 tools/midi_to_song.py 从 {source} 生成\n\n#pragma once\n\n#include \"melody/melody.hpp\"\n\n"
    text += part("MELODY", melody)
    if chords:
        text += part("CHORDS", chords)
        text += f"inline constexpr Song {const} = {{.melody = {const}_MELODY, .chords = {const}_CHORDS}};\n"
    else:
        text += f"inline constexpr Song {const} = {{.melody = {const}_MELODY}};\n"
    return text


def main():
    if len(sys.argv) > 1:
        path = Path(sys.argv[1].strip('"'))
    else:
        path = Path(input("MIDI 路径（可拖入文件）: ").strip().strip('"'))
    while not path.is_file():
        path = Path(input("文件不存在，重新输入: ").strip().strip('"'))

    midi = mido.MidiFile(path)
    tracks = load_tracks(midi)
    if not tracks:
        print("没有可转换的音符")
        return
    names = {i: track_name(midi.tracks[i]) for i in tracks}
    print(f"{path.name}：{len(tracks)} 个有音符的音轨，约 {midi.length:.0f} s")

    roles = choose(tracks, names)
    melody, chords = build(tracks, roles)
    for group, voices in (("旋律", melody), ("伴奏", chords)):
        if len(voices) > MAX_VOICES:
            print(f"{group}组需要 {len(voices)} 个声部，超过上限 {MAX_VOICES}，请把部分音轨设为不用")
            return
    print(f"旋律 {len(melody)} 个声部，伴奏 {len(chords)} 个声部")

    output = ROOT / ask("输出目录", "lib/melody/music")
    default = re.sub(r"[^0-9a-zA-Z]+", "_", path.stem).strip("_").lower() or "song"
    name = ask("曲目名（snake_case，常量名为其大写）", default)
    while not re.fullmatch(r"[a-z][a-z0-9_]*", name):
        name = input("只能包含小写字母、数字和下划线，且以字母开头: ").strip()
    file = output / f"{name}.hpp"
    if file.exists() and input(f"{file} 已存在，覆盖？[y/N]: ").strip().lower() != "y":
        return

    output.mkdir(parents=True, exist_ok=True)
    file.write_text(header(name, path.name, melody, chords), encoding="utf-8", newline="\n")
    seconds = max(sum(n for _, n in v) for v in melody + chords) * UNIT
    print(f"已写入 {file}：{seconds:.1f} s；播放 melody.play_poly({name.upper()}) 或 melody.play({name.upper()})")


if __name__ == "__main__":
    main()
