"""检查波形边界和结束条件；构建独立调参固件，复用现有工具。"""
import os
from pathlib import Path
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[3]
TASK = Path(__file__).resolve().parent
OUT = Path(__file__).resolve().parents[2] / 'out/25-yaw_pid'
BUNDLES = Path(os.environ['LOCALAPPDATA']) / 'stm32cube/bundles'


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    env = {**os.environ, 'ZIG_GLOBAL_CACHE_DIR': str(OUT / 'zig_cache'),
           'ZIG_LOCAL_CACHE_DIR': str(OUT / 'zig_local_cache')}
    exe = OUT / 'profile_test.exe'
    try:
        subprocess.run([str(ROOT / 'agents/.tools/ziglang/zig.exe'), 'c++', '-std=c++23', '-O1',
                        '-I' + str(ROOT / 'lib'), str(TASK / 'profile_test.cpp'), '-o', str(exe)],
                       cwd=ROOT, env=env, check=True)
        subprocess.run([str(exe)], cwd=ROOT, check=True)
        print('Profile: step/sine bounds, invalid input and automatic stop passed', flush=True)
        axis_exe = OUT / 'axis_test.exe'
        subprocess.run([str(ROOT / 'agents/.tools/ziglang/zig.exe'), 'c++', '-std=c++23', '-O1',
                        '-Wno-unknown-pragmas', '-Wno-deprecated-declarations',
                        '-DEIGEN_DONT_VECTORIZE', '-DEIGEN_MAX_ALIGN_BYTES=0',
                        '-I' + str(ROOT / 'lib'), '-I' + str(ROOT / 'lib/third_party/eigen-3.4.0'),
                        str(TASK / 'axis_test.cpp'),
                        str(ROOT / 'lib/motor/motor.cpp'), str(ROOT / 'lib/pid/pid.cpp'),
                        '-o', str(axis_exe)], cwd=ROOT, env=env, check=True)
        subprocess.run([str(axis_exe)], cwd=ROOT, check=True)
        print('Axes: independent hold, disturbance recovery, limits and disable passed', flush=True)
    finally:
        for name in ('zig_cache', 'zig_local_cache'):
            path = (OUT / name).resolve()
            assert path.is_relative_to(OUT.resolve())
            shutil.rmtree(path, ignore_errors=True)

    cmake = next((BUNDLES / 'cmake').glob('*/bin/cmake.exe'))
    ninja = next((BUNDLES / 'ninja').glob('*/bin/ninja.exe'))
    gcc = next((BUNDLES / 'gnu-tools-for-stm32').glob('*/bin/arm-none-eabi-gcc.exe'))
    env = {**os.environ, 'PATH': str(gcc.parent) + os.pathsep + os.environ['PATH']}
    build = OUT / 'build'
    commands = [
        [str(cmake), '-S', str(ROOT), '-B', str(build), '-G', 'Ninja',
         '-DCMAKE_MAKE_PROGRAM=' + str(ninja), '-DCMAKE_BUILD_TYPE=Debug',
         '-DRMPP_APP=jelly_sentry_debug/yaw_pid'],
        [str(cmake), '--build', str(build), '--parallel', '8'],
    ]
    for i, command in enumerate(commands):
        result = subprocess.run(command, cwd=ROOT, env=env, text=True, capture_output=True)
        (OUT / f'build_{i}.log').write_text(result.stdout + result.stderr, encoding='utf-8')
        print(result.stdout[-4000:] + result.stderr[-4000:], flush=True)
        result.check_returncode()


if __name__ == '__main__':
    main()
