"""按两个赛季的协议明细表分别检查 ARM 数据布局。"""

from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "agents/tmp/13-referee_relay/packet_layout"

SIZES = {
    "GameStatus": (11, 11), "GameResult": (1, 1), "GameRobotHp": (16, 24),
    "EventData": (4, 8), "RefereeWarning": (3, 3), "DartInfo": (3, 3),
    "RobotStatus": (13, 17), "PowerHeatData": (14, 14), "RobotPos": (12, 12),
    "Buff": (8, 7), "HurtData": (1, 7), "ShootData": (7, 7),
    "ProjectileAllowance": (8, 8), "RfidStatus": (5, 6), "DartClientCmd": (6, 6),
    "GroundRobotPosition": (40, 40), "RadarMarkData": (2, 2), "SentryInfo": (6, 14),
    "RadarInfo": (1, 8), "MapCommand": (12, 12), "Figure": (15, 15), "SentryCommand": (4, 4),
}


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    for index, header in enumerate(("packet", "packet2027")):
        checks = [f'#include "referee/{header}.hpp"', '#include <cstddef>']
        checks.extend(f'static_assert(sizeof({name}) == {sizes[index]});' for name, sizes in SIZES.items())
        offsets = [("RobotStatus", "chassis_power_limit", 10), ("SentryInfo", "sentry_info_2", 4)]
        if header == "packet":
            offsets.extend([("GameRobotHp", "reserved", 8), ("GameRobotHp", "ally_base_HP", 14)])
        if header == "packet2027":
            offsets.extend([("RobotStatus", "bullet_speed_limit", 12), ("GameRobotHp", "outpost_R_HP", 12), ("GameRobotHp", "outpost_B_HP", 16),
                            ("GameRobotHp", "ally_control_degree", 20), ("GameRobotHp", "enemy_control_degree", 22),
                            ("EventData", "event_data_2", 4), ("HurtData", "hit_sequence", 1),
                            ("HurtData", "hit_position_x", 3), ("HurtData", "hit_position_y", 5),
                            ("GroundRobotPosition", "reserved_1", 8), ("RfidStatus", "rfid_status_2", 4),
                            ("RadarInfo", "radar_info_2", 1), ("RadarInfo", "fortify_center_x", 2),
                            ("RadarInfo", "fortify_center_y", 4), ("RadarInfo", "fortify_radius", 6)])
        checks.extend(f'static_assert(offsetof({name}, {field}) == {offset});' for name, field, offset in offsets)
        subprocess.run(["arm-none-eabi-g++", "-std=c++20", "-mcpu=cortex-m7", "-mthumb", "-I", str(ROOT / "lib"),
                        "-x", "c++", "-c", "-", "-o", str(OUT / f"{header}.o")],
                       input="\n".join(checks), text=True, check=True)
        print(f"{header}.hpp: OK")


if __name__ == "__main__":
    main()
