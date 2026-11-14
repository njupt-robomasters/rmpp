"""通过 OpenOCD telnet 采样 chassis 关键变量，只读，不复位不写入

用法：python sample.py [采样率Hz，默认50]
先启动 openocd，再运行本脚本；Ctrl-C 停止
"""
import socket, time, struct, sys, datetime

HOST, PORT = "localhost", 4444

# chassis 基址 0x20002638，字段偏移由 DWARF 提取
VARS = {
    "lqr_use_integral": (0x20002668, "u8"),   # bool
    "d_x_ref":          (0x200026C0, "f32"),
    "pitch":            (0x200026D0, "f32"),
    "d_pitch":          (0x200026D4, "f32"),
    "roll":             (0x200026D8, "f32"),
    "d_yaw_gyro":       (0x200026E0, "f32"),
    "x":                (0x200026E8, "f32"),
    "d_x":              (0x200026EC, "f32"),
    "x_int":            (0x20002884, "f32"),
}


def read_mem(sock, addr, fmt):
    size = 4 if fmt != "u8" else 1
    cmd = f"mdw {addr} 1\n" if size == 4 else f"mdb {addr} 1\n"
    sock.sendall(cmd.encode())
    # 读到提示符
    data = b""
    while b">" not in data:
        data += sock.recv(4096)
    # 解析输出：第一行是 "0xADDR: 0xVALUE"
    for line in data.decode(errors="replace").split("\n"):
        if "0x" in line and ":" in line:
            parts = line.split(":")
            if len(parts) >= 2:
                val_str = parts[-1].strip().split()[0]
                raw = int(val_str, 16)
                if fmt == "f32":
                    return struct.unpack("<f", struct.pack("<I", raw))[0]
                return raw
    return None


def main():
    rate = float(sys.argv[1]) if len(sys.argv) > 1 else 50.0
    interval = 1.0 / rate

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((HOST, PORT))
    sock.settimeout(3.0)
    # 读欢迎信息
    try:
        sock.recv(4096)
    except socket.timeout:
        pass

    print(f"采样率 {rate:.1f} Hz，按 Ctrl-C 停止")
    print(f"{'time':>8}  {'x':>9}  {'d_x':>9}  {'x_int':>9}  {'d_x_ref':>8}  {'pitch':>8}  {'d_pitch':>8}  {'lqr_i':>5}")

    outfile = open("sample.csv", "w", encoding="utf-8")
    outfile.write("t,x,d_x,x_int,d_x_ref,pitch,d_pitch,roll,d_yaw_gyro,lqr_use_integral\n")

    t0 = time.time()
    try:
        while True:
            t = time.time() - t0
            vals = {}
            for name, (addr, fmt) in VARS.items():
                vals[name] = read_mem(sock, addr, fmt)
            x = vals["x"] or 0
            dx = vals["d_x"] or 0
            xi = vals["x_int"] or 0
            dxref = vals["d_x_ref"] or 0
            pt = vals["pitch"] or 0
            dpt = vals["d_pitch"] or 0
            li = vals["lqr_use_integral"] or 0
            ts = f"{t:8.3f}"
            print(f"\r{ts}  {x:9.4f}  {dx:9.4f}  {xi:9.4f}  {dxref:8.4f}  {pt:8.4f}  {dpt:8.4f}  {li:5.0f}", end="", flush=True)
            outfile.write(f"{t:.4f},{x:.6f},{dx:.6f},{xi:.6f},{dxref:.6f},{pt:.6f},{dpt:.6f},{vals['roll'] or 0:.6f},{vals['d_yaw_gyro'] or 0:.6f},{li}\n")
            outfile.flush()
            elapsed = time.time() - t0 - t
            sleep = interval - elapsed
            if sleep > 0:
                time.sleep(sleep)
    except KeyboardInterrupt:
        print("\n采样结束")
    finally:
        outfile.close()
        sock.close()


if __name__ == "__main__":
    main()
