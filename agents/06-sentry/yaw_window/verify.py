"""验证大小yaw窗口分配，并局部编译整车与云台调试入口；复用工具并清除Zig缓存。"""
import json
import os
from pathlib import Path
import re
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[3]
TASK = Path(__file__).resolve().parent
OUT = ROOT / 'agents/tmp/yaw_window'


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    env = {**os.environ, 'ZIG_GLOBAL_CACHE_DIR': str(OUT / 'zig_cache'),
           'ZIG_LOCAL_CACHE_DIR': str(OUT / 'zig_local_cache')}
    exe = OUT / 'yaw_test.exe'
    command = [str(ROOT / 'agents/.tools/ziglang/zig.exe'), 'c++', '-std=c++23', '-O1',
               '-Wno-unknown-pragmas', '-Wno-deprecated-declarations',
               '-DEIGEN_DONT_VECTORIZE', '-DEIGEN_MAX_ALIGN_BYTES=0',
               '-I' + str(ROOT / 'lib'), '-I' + str(ROOT / 'lib/third_party/eigen-3.4.0'),
               '-I' + str(ROOT / 'app/jelly_sentry'), str(TASK / 'yaw_test.cpp'),
               *[str(ROOT / unit) for unit in ['lib/robot/gimbal_fold.cpp', 'lib/motor/motor.cpp', 'lib/pid/pid.cpp']],
               '-o', str(exe)]
    subprocess.run(command, cwd=ROOT, env=env, check=True)
    result = subprocess.run([str(exe)], cwd=ROOT, capture_output=True, text=True)
    (OUT / 'host.log').write_text(result.stdout + result.stderr, encoding='utf-8')
    print(result.stdout + result.stderr, flush=True)
    result.check_returncode()
    rows = json.loads((ROOT / 'build/jelly_sentry/compile_commands.json').read_text(encoding='utf-8'))
    units = [row for row in rows if '/app/jelly_sentry/' in row['file'].replace('\\', '/')]
    template = next(row for row in units if row['file'].endswith('/app.cpp'))
    for suffix in ['gimbal', 'gimbal_motor']:
        units.append({**template, 'file': (ROOT / f'app/jelly_sentry_debug/{suffix}/app.cpp').as_posix()})
    summary = {}
    for row in units:
        name = Path(row['file']).relative_to(ROOT).as_posix()
        output = OUT / (name.replace('/', '_') + '.obj')
        command = re.sub(r' -o .*? -c .*$', lambda _: f' -o "{output.as_posix()}" -c "{row["file"]}"', row['command'])
        result = subprocess.run(command, cwd=row['directory'], text=True, capture_output=True)
        log = result.stdout + result.stderr
        (OUT / (output.stem + '.log')).write_text(log, encoding='utf-8')
        if log: print(log, flush=True)
        result.check_returncode()
        summary[name] = {'exit': result.returncode, 'warnings': log.count('warning:')}
        print('ARM compiled:', name, flush=True)
    (OUT / 'arm_summary.json').write_text(json.dumps(summary, indent=2) + '\n', encoding='utf-8')
    for name in ['zig_cache', 'zig_local_cache']:
        path = (OUT / name).resolve()
        assert path.is_relative_to(OUT.resolve())
        shutil.rmtree(path, ignore_errors=True)


if __name__ == '__main__':
    main()
