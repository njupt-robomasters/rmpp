"""热量降频参数仿真：链路延迟下剩余热量闭环的最低值和收敛时间。

目标热量 target = (10·ref − c)·delay + reserve，低于目标按 gain 降频；
热量包每 100 ms 更新，控制器看到的是 delay 之前的剩余热量。

用法：python agents/03-control/shooter/heat_sim.py
"""

SHOT = 10.0  # 17mm 单发热量，heat
LIMIT = 240.0  # 热量上限，heat
DELAY = 0.3  # 链路延迟，s
RESERVE = 10.0  # 保留热量，heat
DT = 0.001  # 仿真步长，s
SAMPLE = 0.1  # 热量包周期，s


def run(cooling, ref, gain, t_end=60.0):
    heat = LIMIT
    delay_steps = round(DELAY / DT)
    sample_steps = round(SAMPLE / DT)
    history = [LIMIT] * delay_steps
    measured = LIMIT
    min_heat = heat
    steady = None
    freq = ref
    for i in range(int(t_end / DT)):
        if i % sample_steps == 0:
            measured = history[-delay_steps]
        target = (SHOT * ref - cooling) * DELAY + RESERVE
        freq = min(ref, max(0.0, cooling / SHOT + gain * (measured - target)))
        heat += (cooling - SHOT * freq) * DT
        history.append(heat)
        min_heat = min(min_heat, heat)
        if steady is None and abs(freq - cooling / SHOT) < 0.05:
            steady = i * DT
    return min_heat, steady, freq


def main():
    print(f"{'c':>3} {'gain':>5} {'min H':>7} {'settle':>7} {'steady f':>9}")
    for cooling in (40, 20, 10):
        for gain in (0.15, 0.2, 0.25, 0.3, 0.4):
            min_heat, steady, freq = run(cooling, 10.0, gain)
            print(f"{cooling:>3} {gain:>5} {min_heat:>7.1f} {steady:>7.2f} {freq:>9.2f}")


if __name__ == "__main__":
    main()
