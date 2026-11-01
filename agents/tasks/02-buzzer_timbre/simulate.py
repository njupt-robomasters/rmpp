"""用 MIDI 模拟蜂鸣器各音色方案并输出 wav，方案与模型说明见同目录 README.md"""

import argparse
import sys
import wave
from pathlib import Path

import mido
import numpy as np
from scipy import signal

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tools"))
from midi_to_music import UNIT, convert, load_events  # noqa: E402  与板上播放器相同的单音转换

FS = 96000  # 内部渲染采样率
OUT_FS = 48000  # 输出 wav 采样率
DAC_FS = 32000  # 方案 3 改写 CCR 的采样率，FS 的整数分之一
CCR_MAX = 1000  # 方案 3 PWM 级数：TIM4 时钟 84 MHz / 载波 84 kHz
ENV_STEP = 0.001  # 方案 1、2 占空比更新周期 s
MAX_REST = 100  # 单音转换休止上限，单位 UNIT


def freq(note):
    return 440 * 2 ** ((note - 69) / 12)


def load_notes(midi, channels=None):
    """非打击乐音符，返回按开始时间排序的 [(start, end, note, velocity)]，单位 s；全曲首音对齐到 0，channels 为空取全部通道"""
    events, sec = [], 0.0
    for msg in midi:  # 迭代时 msg.time 已按 tempo 换算为秒
        sec += msg.time
        if msg.type not in ("note_on", "note_off") or msg.channel == 9:
            continue
        is_on = msg.type == "note_on" and msg.velocity > 0
        events.append((sec, is_on, msg.channel, msg.note, msg.velocity))

    # 同一时刻先松开再按下
    events.sort(key=lambda e: (e[0], e[1]))
    notes, held = [], {}
    for sec, is_on, channel, note, velocity in events:
        key = (channel, note)
        if key in held:
            start, vel = held.pop(key)
            notes.append((start, sec, note, vel, channel))
        if is_on:
            held[key] = (sec, velocity)

    notes.sort()
    t0 = notes[0][0]
    return [(s - t0, e - t0, n, v) for s, e, n, v, c in notes if channels is None or c in channels]


def square(f, t, duty):
    """引脚电平 0/1"""
    return ((f * t) % 1.0 < duty).astype(np.float32)


def env_duty(age, tau):
    """幅度按 exp(-age/tau) 衰减对应的占空比；基波幅度 ∝ sin(π·duty)，每 ENV_STEP 更新一次"""
    age = np.floor(age / ENV_STEP) * ENV_STEP
    return np.arcsin(np.exp(-age / tau)) / np.pi


def render_mono(midi, tau, length):
    """现有播放器：单音抢占、方波；tau 为 inf 时即当前固定 50% 占空比"""
    events = load_events(midi)
    tracks = {e[2] for e in events if e[1] == 2}

    # 合并被拆分的长音，板上同音相邻条目连续发声，包络不重启
    segs = []
    for note, n in convert(events, tracks, MAX_REST):
        if segs and segs[-1][0] == note:
            segs[-1][1] += n
        else:
            segs.append([note, n])

    out = np.zeros(length, np.float32)
    pos = 0
    for note, n in segs:
        count = min(n * UNIT * FS // 1000, length - pos)
        if note:
            t = np.arange(count) / FS
            out[pos : pos + count] = square(freq(note), t, env_duty(t, tau))
        pos += count
    return out


def soft_amp(age, f, tau_a4, floor):
    """改进包络：衰减时间随音高变化（低音慢），衰减到 floor 后保持；每 ENV_STEP 更新一次"""
    age = np.floor(age / ENV_STEP) * ENV_STEP
    tau = np.clip(tau_a4 * (440 / f) ** 0.5, 0.3, 2.0)
    return floor + (1 - floor) * np.exp(-age / tau)


def mono_segments(notes, prefer_high, shift):
    """单声部抢占：更高（prefer_high）或更低的音抢占，当前音松开即休止，返回 [(start, end, note + shift)]"""
    events = sorted([(s, 1, i) for i, (s, *_) in enumerate(notes)] + [(e, 0, i) for i, (_, e, *_) in enumerate(notes)])
    segs, active, since = [], None, 0.0
    for sec, is_on, i in events:
        note = notes[i][2]
        if not is_on:
            if active == i:
                segs.append((since, sec, note + shift))
                active = None
            continue
        cur = None if active is None else notes[active][2]
        if cur is None or (note > cur if prefer_high else note < cur):
            if active is not None:
                segs.append((since, sec, cur + shift))
            active, since = i, sec
    return [s for s in segs if s[1] > s[0]]


def render_direct(segs, tau_a4, floor, length):
    """直驱方波，用占空比实现 soft_amp 包络"""
    out = np.zeros(length, np.float32)
    for start, end, note in segs:
        a, b = round(start * FS), min(round(end * FS), length)
        t = np.arange(a, b) / FS
        f = freq(note)
        out[a:b] = square(f, t, np.arcsin(soft_amp(t - start, f, tau_a4, floor)) / np.pi)
    return out


def render_accent(melody, bass, accent, tau_a4, floor, length):
    """直驱单音：旋律为主，低音每次起音插入 accent 秒后回到旋律（旋律包络照常推进）"""
    out = render_direct(melody, tau_a4, floor, length)
    for start, end, note in bass:
        a, b = round(start * FS), min(round(min(end, start + accent) * FS), length)
        t = np.arange(a, b) / FS
        out[a:b] = square(freq(note), t, 0.5)
    return out


def render_dac_square(voices, length, release=0.0):
    """PWM-DAC 方波混音：voices 为 [(segs, weight, tau_a4, floor)]，各声部 ±weight·soft_amp，DAC_FS 量化为 CCR_MAX 级后零阶保持

    幅度包络不改变方波形状，音色在衰减中保持不变；release 为松开后线性淡出时间 s，不超过到下一音的间隔
    """
    n = length * DAC_FS // FS
    mix = np.zeros(n)
    for segs, weight, tau_a4, floor in voices:
        for k, (start, end, note) in enumerate(segs):
            next_start = segs[k + 1][0] if k + 1 < len(segs) else end + release
            tail = max(0.0, min(release, next_start - end))
            a, b = round(start * DAC_FS), min(round((end + tail) * DAC_FS), n)
            t = np.arange(a, b) / DAC_FS
            f = freq(note)
            wave_ = np.where((f * t) % 1.0 < 0.5, 1.0, -1.0)
            amp = soft_amp(t - start, f, tau_a4, floor)
            if tail > 0:
                amp *= np.clip(1 - (t - end) / tail, 0, 1)
            mix[a:b] += weight * amp * wave_

    ccr = np.round((0.5 + 0.5 * np.clip(mix, -1, 1)) * CCR_MAX)
    pin = np.repeat(ccr / CCR_MAX, FS // DAC_FS).astype(np.float32)
    return np.pad(pin, (0, length - len(pin)), constant_values=0.5)


# 用户 FPGA 电子琴项目（github.com/xzqiaochu/piano）对钢琴音 FFT 取平均得到的 1–15 次谐波幅度比
PIANO = [1, 0.340, 0.102, 0.085, 0.070, 0.065, 0.028, 0.085, 0.011, 0.030, 0.010, 0.014, 0.012, 0.013, 0.004]


def schroeder_phases(amps):
    """Schroeder 相位：幅度不变，压低波形峰值因数"""
    p = np.asarray(amps, float) ** 2
    p /= p.sum()
    return [-2 * np.pi * sum((h - l) * p[l - 1] for l in range(1, h)) for h in range(1, len(amps) + 1)]


def table_wave(amps, phases=None):
    """谐波合成波形，峰值归一化到 1，与满幅方波同峰值；超过 20 kHz 的谐波丢弃，等效每音高一张限带波表"""
    phases = phases or [0.0] * len(amps)
    x = np.arange(4096) / 4096
    peak = np.max(np.abs(sum(a * np.sin(2 * np.pi * h * x + ph) for h, (a, ph) in enumerate(zip(amps, phases), 1))))

    def wave(f, t, decay):
        y = np.zeros_like(t)
        for h, (a, ph) in enumerate(zip(amps, phases), 1):
            if h * f > 20000:
                break
            y += a * np.sin(2 * np.pi * h * f * t + ph)
        return y / peak

    return wave


def shaped_wave(amps, fc, p):
    """按蜂鸣器调整频谱：第 h 次谐波乘 min(1, h·f/fc)^p，削弱落在蜂鸣器有效频段以下的基波与低次谐波

    高音基波已在 fc 以上，频谱不变；每个音按自身频谱重新归一化峰值，等效板上按音区分表
    """
    x = np.arange(1024) / 1024

    def wave(f, t, decay):
        hs = [h for h in range(1, len(amps) + 1) if h * f <= 20000]
        a = [amps[h - 1] * min(1.0, h * f / fc) ** p for h in hs]
        peak = np.max(np.abs(sum(ah * np.sin(2 * np.pi * h * x) for h, ah in zip(hs, a))))
        return sum(ah * np.sin(2 * np.pi * h * f * t) for h, ah in zip(hs, a)) / peak

    return wave


def crossfade_wave(bright, dark):
    """按包络衰减项 decay 从亮表过渡到暗表，模拟钢琴高次谐波衰减更快"""
    return lambda f, t, decay: decay * bright(f, t, decay) + (1 - decay) * dark(f, t, decay)


def square_wave(f, t, decay):
    return np.where((f * t) % 1.0 < 0.5, 1.0, -1.0)


def agc_gain(level, ratio, max_gain, attack, release):
    """与 app/test/common/agc.hpp 相同的自动增益，为加快模拟每 1 ms 更新一次（板上 100 µs）"""
    block = FS // 1000
    lv = level[::block]
    target = np.minimum(np.maximum(lv, 1e-6) ** (1 / ratio - 1), max_gain)
    down = 1 - np.exp(-block / FS / attack) if attack > 0 else 1.0
    up = 1 - np.exp(-block / FS / release)
    gains, gain = np.empty_like(target), 1.0
    for i, x in enumerate(target):
        gain += (down if x < gain else up) * (x - gain)
        gains[i] = gain
    return np.repeat(gains, block)[: len(level)]


def render_synth(voices, length, release, agc, drive=0.0):
    """PWM-DAC 多声部：voices 为 [(segs, weight, tau_a4, floor, wave)]，wave(f, t, decay) 返回峰值 1 的波形

    包络与 app/test/common/voice.hpp 相同；agc 为 (ratio, max_gain, attack, release) 或 None；按 3250 级 CCR 量化
    """
    mix, level = np.zeros(length), np.zeros(length)
    for segs, weight, tau_a4, floor, wave in voices:
        for k, (start, end, note) in enumerate(segs):
            next_start = segs[k + 1][0] if k + 1 < len(segs) else end + release
            tail = max(0.0, min(release, next_start - end))
            a, b = round(start * FS), min(round((end + tail) * FS), length)
            t = np.arange(a, b) / FS
            f = freq(note)
            decay = np.exp(-(t - start) / np.clip(tau_a4 * (440 / f) ** 0.5, 0.3, 2.0))
            amp = weight * (floor + (1 - floor) * decay)
            if tail > 0:
                amp *= np.clip(1 - (t - end) / tail, 0, 1)
            mix[a:b] += amp * wave(f, t, decay)
            level[a:b] += amp

    if agc:
        mix *= agc_gain(level, *agc)

    # 软限幅：tanh(drive·x)/tanh(drive)，峰值不变、抬高平均幅度，代价是增加奇次谐波
    if drive > 0:
        mix = np.tanh(drive * mix) / np.tanh(drive)
    print(f"  限幅样本 {np.mean(np.abs(mix) > 1) * 100:.3f}%")
    return (np.round((0.5 + 0.5 * np.clip(mix, -1, 1)) * 3250) / 3250).astype(np.float32)


def render_arpeggio(notes, slot, voices, tau, length):
    """每 slot 秒在当前最高的 voices 个音之间轮换，各音按自身起音时刻做占空比包络"""
    out = np.zeros(length, np.float32)
    active, i = [], 0
    for k in range(int(length / FS / slot)):
        t0 = k * slot
        while i < len(notes) and notes[i][0] <= t0:
            active.append(notes[i])
            i += 1
        active = [n for n in active if n[1] > t0]
        if not active:
            continue

        # 同音高只保留最近一次按下，取最高的 voices 个
        latest = {}
        for n in active:
            if n[2] not in latest or n[0] > latest[n[2]][0]:
                latest[n[2]] = n
        chord = [latest[p] for p in sorted(latest)[-voices:]]
        start, _, note, _ = chord[k % len(chord)]

        a, b = round(t0 * FS), round((t0 + slot) * FS)
        t = np.arange(a, b) / FS
        out[a:b] = square(freq(note), t, env_duty(t - start, tau))
    return out


def piano_tone(f, velocity, hold, duration):
    """加法合成钢琴音：快速起音，各次谐波指数衰减且越高越快，略带非谐性，松键后制音"""
    t = np.arange(int(duration * DAC_FS)) / DAC_FS
    tau1 = np.clip(0.8 * (440 / f) ** 0.5, 0.3, 3.0)  # 基波余音，低音更长
    y = np.zeros_like(t)
    for h in range(1, 9):
        fh = h * f * np.sqrt(1 + 1e-4 * h * h)
        if fh > 10000:
            break
        y += np.sin(2 * np.pi * fh * t) * np.exp(-t * (1 + 0.6 * (h - 1)) / tau1) / h

    env = np.minimum(t / 0.003, 1.0)  # 3 ms 起音
    env *= np.exp(-np.maximum(t - hold, 0) / 0.06)  # 制音
    fade = min(len(t), int(0.005 * DAC_FS))  # 结尾 5 ms 淡出，声部被抢占时不出爆音
    env[len(t) - fade :] *= np.linspace(1, 0, fade)
    return y * env * (velocity / 127) ** 1.5 / 2


def render_dac(notes, voices, gain, length):
    """PWM-DAC：软件混合最多 voices 个声部，DAC_FS 量化为 CCR_MAX 级后零阶保持"""
    # 声部分配：同音重击替换旧声部，满时抢占最早开始的声部
    stop = [min(e + 0.4, s + 6.9 * np.clip(0.8 * (440 / freq(n)) ** 0.5, 0.3, 3.0)) for s, e, n, _ in notes]
    playing = []
    for i, (start, _, note, _) in enumerate(notes):
        playing = [j for j in playing if stop[j] > start]
        for j in [j for j in playing if notes[j][2] == note]:
            stop[j] = start
            playing.remove(j)
        if len(playing) >= voices:
            j = min(playing, key=lambda j: notes[j][0])
            stop[j] = start
            playing.remove(j)
        playing.append(i)

    mix = np.zeros(length * DAC_FS // FS)
    for (start, end, note, velocity), end_stop in zip(notes, stop):
        a = round(start * DAC_FS)
        tone = piano_tone(freq(note), velocity, end - start, end_stop - start)[: len(mix) - a]
        mix[a : a + len(tone)] += tone

    x = mix * gain
    print(f"  方案 3 限幅样本 {np.mean(np.abs(x) > 1) * 100:.3f}%")
    ccr = np.round((0.5 + 0.5 * np.clip(x, -1, 1)) * CCR_MAX)
    pin = np.repeat(ccr / CCR_MAX, FS // DAC_FS).astype(np.float32)
    return np.pad(pin, (0, length - len(pin)), constant_values=0.5)


def buzzer_sos(f0, q, peak_db):
    """蜂鸣器粗略频响：300 Hz 高通 + f0 处谐振峰 + 10 kHz 低通"""
    amp = 10 ** (peak_db / 40)
    w0 = 2 * np.pi * f0 / FS
    alpha = np.sin(w0) / (2 * q)
    b = [1 + alpha * amp, -2 * np.cos(w0), 1 - alpha * amp]
    a = [1 + alpha / amp, -2 * np.cos(w0), 1 - alpha / amp]
    return np.vstack(
        [
            signal.butter(2, 300, "highpass", fs=FS, output="sos"),
            signal.tf2sos(b, a),
            signal.butter(2, 10000, "lowpass", fs=FS, output="sos"),
        ]
    )


def write_wav(path, samples):
    with wave.open(str(path), "wb") as file:
        file.setnchannels(1)
        file.setsampwidth(2)
        file.setframerate(OUT_FS)
        file.writeframes((np.clip(samples, -1, 1) * 32767).astype(np.int16).tobytes())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("midi", nargs="?", default=str(ROOT / "reference/midi/always_with_me.mid"))
    parser.add_argument("--out", default=str(Path(__file__).resolve().parent / "out"))
    parser.add_argument("--start", type=float, default=0.0, help="输出起点 s")
    parser.add_argument("--seconds", type=float, default=0.0, help="输出时长 s，0 为到结尾")
    parser.add_argument("--f0", type=float, default=4000.0, help="蜂鸣器谐振频率 Hz")
    parser.add_argument("--q", type=float, default=3.0, help="谐振峰 Q")
    parser.add_argument("--peak-db", type=float, default=18.0, help="谐振峰增益 dB")
    parser.add_argument("--tau", type=float, default=0.35, help="方案 1 幅度衰减时间常数 s")
    parser.add_argument("--arp-tau", type=float, default=0.5, help="方案 2 幅度衰减时间常数 s")
    parser.add_argument("--slot", type=float, default=0.015, help="方案 2 琶音切换周期 s")
    parser.add_argument("--arp-voices", type=int, default=3, help="方案 2 轮换音数")
    parser.add_argument("--voices", type=int, default=4, help="方案 3 声部数")
    parser.add_argument("--mix-gain", type=float, default=0.35, help="方案 3 混音增益")
    parser.add_argument("--soft-tau", type=float, default=0.7, help="改进包络 A4 处衰减时间常数 s，按 sqrt(440/f) 随音高缩放")
    parser.add_argument("--floor", type=float, default=0.35, help="改进包络余音底，相对峰值")
    parser.add_argument("--bass-tau", type=float, default=0.4, help="7b-7d 低音 A4 处衰减时间常数 s")
    parser.add_argument("--bass-floor", type=float, default=0.12, help="7b-7d 低音余音底")
    parser.add_argument("--release", type=float, default=0.03, help="7b-7d 松开淡出时间 s")
    parser.add_argument("--accent", type=float, default=0.025, help="低音插入时长 s")
    parser.add_argument("--bass-shift", type=int, default=12, help="低音声部移调，半音")
    parser.add_argument("--only", default="", help="只渲染名称包含任一关键字的方案，逗号分隔；基线总是渲染")
    parser.add_argument("--agc-ratio", type=float, default=4.0, help="9 系列自动增益压缩比")
    parser.add_argument("--raw", action="store_true", help="另存不经蜂鸣器模型的理想输出")
    args = parser.parse_args()

    midi = mido.MidiFile(args.midi)
    notes = load_notes(midi)
    length = int((max(n[1] for n in notes) + 2) * FS)
    print(f"{Path(args.midi).name}：{len(notes)} 个音符，{length / FS:.1f} s")

    # 旋律取右手（通道 0、1）最高音，低音取左手（通道 2、3）最低音并移调
    def melody(shift):
        return mono_segments(load_notes(midi, {0, 1}), True, shift)

    def bass(shift=0):
        return mono_segments(load_notes(midi, {2, 3}), False, args.bass_shift + shift)

    # 钢琴波表：零相位、Schroeder 相位、亮暗双表（暗表第 h 次谐波再乘 0.6^(h-1)）
    piano = table_wave(PIANO)
    piano_s = table_wave(PIANO, schroeder_phases(PIANO))
    dark = [a * 0.6**h for h, a in enumerate(PIANO)]
    piano_fade = crossfade_wave(piano_s, table_wave(dark, schroeder_phases(dark)))
    agc = (args.agc_ratio, 8.0, 0.005, 0.15)

    piano_fade_zero = crossfade_wave(piano, table_wave(dark))

    def shaped_fade(fc, p):
        return crossfade_wave(shaped_wave(PIANO, fc, p), shaped_wave(dark, fc, p))

    def synth(melody_wave, bass_wave, melody_weight=0.5, bass_weight=0.5, ratio=None, drive=0.0):
        return render_synth(
            [
                (melody(0), melody_weight, args.soft_tau, args.floor, melody_wave),
                (bass(), bass_weight, args.bass_tau, args.bass_floor, bass_wave),
            ],
            length,
            args.release,
            (ratio, *agc[1:]) if ratio else agc,
            drive,
        )

    renders = {
        "0_baseline": lambda: render_mono(midi, np.inf, length),
        "1_envelope": lambda: render_mono(midi, args.tau, length),
        "2_arpeggio": lambda: render_arpeggio(notes, args.slot, args.arp_voices, args.arp_tau, length),
        "3_pwm_dac": lambda: render_dac(notes, args.voices, args.mix_gain, length),
        "4_soft": lambda: render_direct(melody(0), args.soft_tau, args.floor, length),
        "5_soft_up12": lambda: render_direct(melody(12), args.soft_tau, args.floor, length),
        "6_dac_mono": lambda: render_dac_square([(melody(0), 1.0, args.soft_tau, args.floor)], length),
        "7_dac_duo": lambda: render_dac_square(
            [(melody(0), 0.5, args.soft_tau, args.floor), (bass(), 0.5, args.soft_tau, args.floor)], length
        ),
        # 低音衰减更快、余音底更低，旋律休止时伴奏已经收敛
        "7b_bass_decay": lambda: render_dac_square(
            [(melody(0), 0.5, args.soft_tau, args.floor), (bass(), 0.5, args.bass_tau, args.bass_floor)],
            length,
            args.release,
        ),
        # 在 7b 基础上旋律 0.6、伴奏 0.4
        "7c_melody_lead": lambda: render_dac_square(
            [(melody(0), 0.6, args.soft_tau, args.floor), (bass(), 0.4, args.bass_tau, args.bass_floor)],
            length,
            args.release,
        ),
        # 在 7b 基础上整体升八度
        "7d_up12": lambda: render_dac_square(
            [(melody(12), 0.5, args.soft_tau, args.floor), (bass(12), 0.5, args.bass_tau, args.bass_floor)],
            length,
            args.release,
        ),
        "8_accent": lambda: render_accent(melody(0), bass(), args.accent, args.soft_tau, args.floor, length),
        # 9 系列：与板上 soft 系列相同的自动增益，只换波形
        "9a_square": lambda: synth(square_wave, square_wave),
        "9b_piano": lambda: synth(piano, piano),
        "9c_piano_schroeder": lambda: synth(piano_s, piano_s),
        "9d_piano_fade": lambda: synth(piano_fade, piano_fade),
        "9e_square_piano": lambda: synth(square_wave, piano_s, 0.6, 0.4),
        # 9f–9i：零相位亮暗双表，不同提响度手段
        "9f_fade_zero": lambda: synth(piano_fade_zero, piano_fade_zero),
        "9g_fade_agc8": lambda: synth(piano_fade_zero, piano_fade_zero, ratio=8.0),
        "9h_fade_drive": lambda: synth(piano_fade_zero, piano_fade_zero, drive=2.5),
        "9i_fade_lead": lambda: synth(piano_fade_zero, piano_fade_zero, 0.65, 0.35, ratio=6.0),
        # 9j–9l：9f 的频谱按蜂鸣器调整，利用缺失基波效应，削弱程度逐步加大
        "9j_shape_1500": lambda: synth(*[shaped_fade(1500, 1.0)] * 2),
        "9k_shape_2000": lambda: synth(*[shaped_fade(2000, 1.5)] * 2),
        "9l_shape_3000": lambda: synth(*[shaped_fade(3000, 2.0)] * 2),
    }
    if args.only:
        keys = args.only.split(",")
        renders = {k: v for k, v in renders.items() if k == "0_baseline" or any(s in k for s in keys)}
    sos = buzzer_sos(args.f0, args.q, args.peak_db)
    dc = signal.butter(1, 20, "highpass", fs=FS, output="sos")
    a = int(args.start * OUT_FS)
    b = a + int(args.seconds * OUT_FS) if args.seconds > 0 else None

    outputs = {}
    for name, render in renders.items():
        print(f"渲染 {name}")
        pin = render()
        outputs[name] = signal.resample_poly(signal.sosfilt(sos, pin), OUT_FS, FS)[a:b]
        if args.raw:
            outputs[name + "_raw"] = signal.resample_poly(signal.sosfilt(dc, pin), OUT_FS, FS)[a:b]

    # 同类输出共用增益，保留方案间的真实响度差
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    for suffix in ("", "_raw"):
        group = {k: v for k, v in outputs.items() if k.endswith("_raw") == bool(suffix)}
        if not group:
            continue
        scale = 0.9 / max(np.max(np.abs(v)) for v in group.values())
        ref = np.sqrt(np.mean(group["0_baseline" + suffix] ** 2))
        for name, samples in group.items():
            write_wav(out / f"{name}.wav", samples * scale)
            rms = 20 * np.log10(np.sqrt(np.mean(samples**2)) / ref)
            print(f"  {name}.wav  RMS 相对基线 {rms:+.1f} dB")
    print(f"输出目录 {out}")


if __name__ == "__main__":
    main()
