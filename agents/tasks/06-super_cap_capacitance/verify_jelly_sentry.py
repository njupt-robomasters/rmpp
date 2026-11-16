"""读取实际 C++ 模式分支，按遥控器、比赛阶段与裁判连接状态核对策略表。"""

from itertools import product
from pathlib import Path
import re
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[3]
source = (ROOT / "app/jelly_sentry/chassis.cpp").read_text(encoding="utf-8")
block = source.split("const bool is_cap_off =", 1)[1].split("super_cap.set_mode(cap_mode);", 1)[0]
off_expression = block.split(";", 1)[0]
branches = re.findall(r"(?:if|else if) \((.*?)\) \{\s*cap_mode = SuperCap::(\w+);", block, re.S)
assert len(branches) == 5
assert "SuperCap::Mode cap_mode = SuperCap::OFF;" in block


def evaluate(expression, context):
    expression = expression.replace("::", ".").replace("&&", " and ").replace("||", " or ")
    expression = re.sub(r"!(?!=)", "not ", expression).strip()
    return eval(f"({expression})", {"__builtins__": {}}, context)


count = 0
for connected, stage, robot_id, fs, vt in product(
    (False, True), ("PREPARATION", "REFEREE_SELF_CHECK", "COUNTDOWN_5SEC", "GAMING", "END"),
    (7, 107, 3), (None, "UP", "MID", "DOWN", "ERR"), (None, "C", "N", "S", "ERR"),
):
    context = {
        "fsi6x": SimpleNamespace(is_connect=fs is not None, swc=fs),
        "vt13": SimpleNamespace(is_connect=vt is not None, mode=vt),
        "referee": SimpleNamespace(is_connected=connected),
        "FSi6X": SimpleNamespace(UP="UP", MID="MID", DOWN="DOWN"),
        "VT13": SimpleNamespace(C="C", N="N", S="S"),
        "Referee": SimpleNamespace(REFEREE_SELF_CHECK="REFEREE_SELF_CHECK", COUNTDOWN_5SEC="COUNTDOWN_5SEC"),
        "progress": stage,
        "sentry_in_game": lambda: stage == "GAMING" and robot_id in (7, 107),
    }
    context["is_cap_off"] = evaluate(off_expression, context)
    actual = "OFF"
    for condition, mode in branches:
        if evaluate(condition, context):
            actual = mode
            break

    # 按已确认的优先级生成独立期望表；ERR 与断联均无人工模式请求
    manual_requests = {"UP": "OFF", "C": "OFF", "MID": "CHARGE", "N": "CHARGE", "DOWN": "BOOST", "S": "BOOST"}
    requests = {manual_requests.get(fs), manual_requests.get(vt)}
    phase_mode = {"REFEREE_SELF_CHECK": "CHARGE", "COUNTDOWN_5SEC": "CHARGE"}.get(stage)
    if stage == "GAMING" and robot_id in (7, 107):
        phase_mode = "BOOST"
    expected = "OFF"
    if connected and "OFF" not in requests:
        expected = phase_mode or ("BOOST" if "BOOST" in requests else "CHARGE" if "CHARGE" in requests else "OFF")
    assert actual == expected, (connected, stage, robot_id, fs, vt, expected, actual)
    count += 1

print(f"PASS: {count} mode cases (actual C++ conditions; host expression evaluation)")
