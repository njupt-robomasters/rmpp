"""在主机运行轮腿上板自瞄收发与控制回归。"""

from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[3]
OUT = ROOT / "agents/tmp/minipc_protocol/leg_up"
SOURCES = [
    "agents/05-comm/minipc_protocol/leg_up_aim_test.cpp",
    "app/leg/leg_up/chassis.cpp",
    "app/leg/leg_up/gimbal.cpp",
    "app/leg/leg_up/minipc.cpp",
    "app/leg/leg_up/referee.cpp",
    "app/leg/leg_up/shooter.cpp",
    "lib/attitude/attitude.cpp",
    "lib/attitude/quaternion_ekf.cpp",
    "lib/bsp/imu.cpp",
    "lib/minipc/aim.cpp",
    "lib/motor/gm6020.cpp",
    "lib/motor/m3508.cpp",
    "lib/motor/motor.cpp",
    "lib/motor/vesc.cpp",
    "lib/pid/pid.cpp",
    "lib/power/super_cap.cpp",
    "lib/rc/vt13.cpp",
    "lib/referee/crc.cpp",
    "lib/referee/referee.cpp",
    "lib/referee/referee_sender.cpp",
    "lib/referee/robot_ui.cpp",
    "lib/robot/gimbal_classic.cpp",
    "lib/robot/shooter_17mm.cpp",
]


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    executable = OUT / "leg_up_aim_test.exe"
    subprocess.run([
        "g++", "-std=c++23", "-O1", "-Wall", "-Wextra", "-mno-ms-bitfields",
        "-Wno-packed-bitfield-compat", "-DEIGEN_DONT_VECTORIZE", "-DEIGEN_MAX_ALIGN_BYTES=0",
        "-Ilib", "-isystem", "lib/third_party/eigen-3.4.0",
        *SOURCES, "-o", str(executable),
    ], cwd=ROOT, check=True)
    subprocess.run([str(executable)], cwd=ROOT, check=True)


if __name__ == "__main__":
    main()
