"""OpenOCD TCL 只读运行态采样，不连接 GDB，不暂停或复位 MCU。

地址来自 build/dummy/rmpp.elf；更换固件后需重新核对。
"""

import argparse
import csv
import hashlib
import json
from pathlib import Path
import socket
import struct
import time


class OpenOcd:
    def __init__(self):
        self.sock = socket.create_connection(("127.0.0.1", 6666), timeout=5)
        self.pending = b""

    def command(self, command):
        self.sock.sendall(command.encode() + b"\x1a")
        while b"\x1a" not in self.pending:
            self.pending += self.sock.recv(65536)
        response, self.pending = self.pending.split(b"\x1a", 1)
        return response.decode().strip()

    def read(self, address, count):
        response = self.command(f"read_memory 0x{address:x} 32 {count}")
        words = [int(word, 0) for word in response.split()]
        if len(words) != count:
            raise RuntimeError(response)
        return struct.pack(f"<{count}I", *words)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=180)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    out = Path(__file__).parent / "out" / "follow_pid"
    out.mkdir(parents=True, exist_ok=True)
    ocd = OpenOcd()
    print("target:", ocd.command("targets"), flush=True)

    # 直接解析 ELF32 程序头，逐段比对运行中 Flash 与 ELF。
    elf = Path("build/dummy/rmpp.elf").read_bytes()
    phoff = struct.unpack_from("<I", elf, 28)[0]
    phentsize, phnum = struct.unpack_from("<HH", elf, 42)
    verified = []
    for index in range(phnum):
        kind, offset, _, physical, size, _, _, _ = struct.unpack_from(
            "<8I", elf, phoff + index * phentsize
        )
        if kind != 1 or not size or not 0x08000000 <= physical < 0x08100000:
            continue
        observed = bytearray()
        for position in range(0, size, 1024):
            length = min(1024, size - position)
            observed.extend(ocd.read(physical + position, (length + 3) // 4)[:length])
        if observed != elf[offset:offset + size]:
            raise RuntimeError(f"Flash 与 ELF 不符: 0x{physical:x}")
        verified.append({"address": hex(physical), "size": size,
                         "sha256": hashlib.sha256(observed).hexdigest()})
    config = struct.unpack("<8f", ocd.read(0x20000058, 8))
    metadata = {"flash_verified": verified, "follow_config": dict(zip(
        ["kp", "ki", "kd", "ff", "max_i", "max_out", "err_fc", "d_fc"], config)),
        "system_clock": struct.unpack("<I", ocd.read(0x20000004, 1))[0],
        "started": time.strftime("%Y-%m-%d %H:%M:%S")}
    print(json.dumps(metadata, ensure_ascii=False), flush=True)
    if args.check:
        print("pid:", struct.unpack("<14I", ocd.read(0x200029A0, 14)), flush=True)
        return
    stamp = time.strftime("%Y%m%d_%H%M%S")
    (out / f"{stamp}.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    columns = ["host_s", "read_ms", "out", "err", "last_err", "p", "i", "d", "dt",
               "pid_cycles", "yaw_speed", "yaw_position", "yaw_raw_speed", "yaw_raw_position",
               "yaw_callback_dt", "yaw_callback_cycles", "enable", "gimbal_yaw", "mode"]
    start = time.perf_counter()
    count = 0
    next_report = 0
    with (out / f"{stamp}.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(columns)
        while time.perf_counter() - start < args.seconds:
            before = time.perf_counter()
            # 每次单独读取完整 PID；其时间戳可用于剔除重复或跨周期样本。
            pid = ocd.read(0x200029A0, 14)
            yaw = ocd.read(0x200026A0, 14)
            state = ocd.read(0x20002930, 4)
            after = time.perf_counter()
            pf = struct.unpack("<14f", pid)
            yf = struct.unpack("<14f", yaw)
            writer.writerow([before - start, (after - before) * 1000,
                             pf[0], pf[2], pf[3], pf[4], pf[5], pf[6], pf[8],
                             struct.unpack_from("<Q", pid, 48)[0],
                             yf[0], yf[1], yf[5], yf[6], yf[8],
                             struct.unpack_from("<Q", yaw, 48)[0],
                             state[0], struct.unpack_from("<f", state, 8)[0],
                             struct.unpack_from("<I", state, 12)[0]])
            count += 1
            if after - start >= next_report:
                stream.flush()
                print(f"{after - start:.1f}s samples={count} err={pf[2]:.6f} "
                      f"d={pf[6]:.3f} dt={pf[8] * 1000:.3f}ms enable={state[0]}", flush=True)
                next_report += 10
    print("capture complete", count, ocd.command("targets"), flush=True)


if __name__ == "__main__":
    main()
