"""对照保存提交，检查实机调参与发送开关在接口迁移中保留。"""

from pathlib import Path
import json
import re
import subprocess

ROOT = Path(__file__).resolve().parents[3]
BASELINE = "4d59574"


def baseline(file):
    return subprocess.check_output(["git", "show", f"{BASELINE}:{file}"], cwd=ROOT).decode("utf-8")


def compact(text):
    return re.sub(r"\s+", "", text)


def active_sends(text):
    text = re.sub(r"//[^\n]*|/\*[\s\S]*?\*/", "", text)
    return re.findall(r"\b\w+\s*(?:\.|->)\s*send\([^;]*;", text)


checks = []
for relative in ("app/jelly_sentry/gimbal.hpp", "app/jelly_sentry/shooter.hpp", "app/jelly_sentry/chassis.hpp"):
    old = baseline(relative)
    new = (ROOT / relative).read_text(encoding="utf-8")
    for name, body in re.findall(r"inline Pid::Config (\w+)\s*=\s*\{([^}]+)\}", old):
        current = re.search(rf"inline Pid::Config {name}\s*=\s*\{{([^}}]+)\}}", new)
        assert current and compact(body) == compact(current[1]), name
        checks.append(name)
    for name, value in re.findall(r"inline volatile uint32_t (\w+)\s*=\s*(\d+)", old):
        assert re.search(rf"{name}\s*=\s*{value}\s*;", new), name
        checks.append(name)
    if relative.endswith("gimbal.hpp"):
        for value in (".yaw = -90 * deg", ".roll = 180 * deg", ".offset = -61.3f * deg", ".ff = -1.2f", ".pitch_max = 25 * deg"):
            assert compact(value) in compact(new), value
        checks.append("IMU 安装、pitch2 零点与限位")

for relative in ["app/jelly_sentry/app.cpp", *[p.relative_to(ROOT).as_posix() for p in sorted((ROOT / "app/jelly_sentry_debug").glob("*/app.cpp"))]]:
    old = active_sends(baseline(relative))
    new = active_sends((ROOT / relative).read_text(encoding="utf-8"))
    assert [compact(s) for s in old] == [compact(s) for s in new], relative
    checks.append(f"发送开关：{relative}")

presets = json.loads((ROOT / "CMakePresets.json").read_text())["configurePresets"]
names = [p["name"] for p in presets]
assert len(names) == len(set(names))
assert "leg_down" in names and "leg_up" in names and "leg" not in names
assert "jelly_sentry" in names and sum(n.startswith("jelly_sentry_debug-") for n in names) == 10
checks.append("构建预设合并")
ioc = (ROOT / "boards/mc02/rmpp.ioc").read_text()
fdcan = (ROOT / "boards/mc02/Core/Src/fdcan.c").read_text()
assert "FDCAN2.AutoRetransmission=ENABLE" in ioc
assert "FDCAN2.TxFifoQueueMode=FDCAN_TX_FIFO_OPERATION" in ioc
assert "hfdcan2.Init.AutoRetransmission = ENABLE;" in fdcan
assert "hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;" in fdcan
checks.append("CAN2 自动重发与 FIFO 配置一致")
print(f"通过 {len(checks)} 项基线与配置检查")
