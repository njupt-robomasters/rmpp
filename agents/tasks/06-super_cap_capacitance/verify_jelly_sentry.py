"""读取实际 C++ 超电模式分支，按 VT13 挡位、比赛阶段、机器人 ID 与裁判连接状态核对策略表。"""

from itertools import product
from pathlib import Path
import re
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[3]
source = (ROOT / "app/jelly_sentry/chassis.cpp").read_text(encoding="utf-8")
block = source.split("const auto progress = referee.game_status.game_progress;", 1)[1].split("super_cap.set_referee_limit", 1)[0]
block = re.sub(r"//.*", "", block)
branches = re.findall(r"(?:if|else if) \((.*?)\)\s*super_cap\.set_mode\(SuperCap::(\w+)\);", block, re.S)
default = re.search(r"else\s*super_cap\.set_mode\(SuperCap::(\w+)\);\s*$", block.strip(), re.S).group(1)
assert len(branches) == 3 and default == "OFF"


def evaluate(expression, context):
    expression = expression.replace("::", ".").replace("&&", " and ").replace("||", " or ")
    expression = re.sub(r"!(?!=)", "not ", expression).strip()
    return eval(f"({expression})", {"__builtins__": {}}, context)


count = 0
for connected, stage, robot_id, vt in product(
    (False, True), ("PREPARATION", "REFEREE_SELF_CHECK", "COUNTDOWN_5SEC", "GAMING", "END"),
    (7, 107, 3), (None, "C", "N", "S"),
):
    context = {
        "vt13": SimpleNamespace(is_enable=vt in ("N", "S"), rc=SimpleNamespace(mode=vt or "ERR")),
        "referee": SimpleNamespace(is_connect=connected),
        "VT13": SimpleNamespace(C="C", N="N", S="S"),
        "Referee": SimpleNamespace(REFEREE_SELF_CHECK="REFEREE_SELF_CHECK", COUNTDOWN_5SEC="COUNTDOWN_5SEC"),
        "progress": stage,
        "sentry_in_game": lambda: stage == "GAMING" and robot_id in (7, 107),
    }
    actual = default
    for condition, mode in branches:
        if evaluate(condition, context):
            actual = mode
            break

    # 独立期望表：裁判离线或 C 挡 OFF；N/S 挡或哨兵比赛中 BOOST；自检、倒计时 CHARGE；其余 OFF
    if not connected or vt == "C":
        expected = "OFF"
    elif vt in ("N", "S") or (stage == "GAMING" and robot_id in (7, 107)):
        expected = "BOOST"
    elif stage in ("REFEREE_SELF_CHECK", "COUNTDOWN_5SEC"):
        expected = "CHARGE"
    else:
        expected = "OFF"
    assert actual == expected, (connected, stage, robot_id, vt, expected, actual)
    count += 1

print(f"PASS: {count} mode cases (actual C++ conditions; host expression evaluation)")
