# Float and Angle Migration

## Scope and Representation

Application code now uses `float` in SI units, with Celsius retained for
temperature and shots/radian for feeder coefficients. `math::Angle` contains
one private float in radians, normalizes all numeric assignments to `[-pi, pi)`,
supports implicit conversion in both directions and the four compound assignments.
Ordinary arithmetic returns float. Multi-turn IMU yaw and ordinary angle errors
remain float. No legacy include path or runtime unit metadata is retained.

The normalization uses bounded binary remainder steps, works in constexpr
expressions on both tested compilers, and propagates NaN/infinity. Helpers retain
the original upper-bound-first clamp order and map endpoint behavior.

MAVLink remains enabled with unchanged generated messages, IDs and CRCs. Flash
still uses `yaw_offset`, now storing radians directly. Historical degree values
are not migrated; recalibration is required as agreed.

## Unit Audit

| Data path | Input / original scaling | Internal value | Output boundary |
| --- | --- | --- | --- |
| DWT | uint32 tick subtraction / core clock | seconds; frequency 1/dt | HAL delay ticks; timeout and reset semantics unchanged |
| CPU usage | running time / callback interval | dimensionless ratio | no implicit percent storage |
| PID | old kp/ki/kd/ff, limits and fc evaluated in standard units | float errors, seconds, SI gains, Hz | same filter, anti-windup, clamps and reset |
| M2006/M3508/GM6020 | encoder /8192 *rev; signed rpm /reduction; current /command range *max current | rad, rad/s, A, N*m | current ratio *command range, same truncation/inversion |
| RM6623 | encoder /8192 *rev; current feedback ratio | rad, A, N*m; absent speed stays zero | current ratio *max command |
| MF9025 | encoder /65535 *rev; speed *deg_s; current ratio | rad, rad/s, A | existing little-endian current packet |
| CustomMotor | angle /100 *deg; speed *deg_s; current /100 | rad, rad/s, A | A *100 -> int16 |
| DM4310/DM6006 | MIT packed angle, velocity, torque ranges | rad, rad/s, N*m; current = torque/Kt | existing range clamps, 12/16-bit encoding and enable packets |
| CyberDog | MIT packed angle, velocity and current ranges | rad, rad/s, A; torque = current*Kt | same current encoding despite former Nm tag |
| VESC | eRPM /pole pairs *rpm; current /10; position /50 *deg; temperatures /10 | rad/s, A, rad, Celsius | speed /rpm *pole pairs; existing brake packets |
| VESC duty | raw /1000 *pct | raw *0.00001, deliberately preserved | existing value retained pending protocol review |
| Chassis | wheel speed *radius; torque /radius | m/s and N; rotation in rad | inverse kinematics returns rad/s and N*m |
| Power limiting | torque*speed + 3*current^2*resistance | W; buffer J; gains dimensionless | same quadratic root and limit order |
| Gimbal | encoder/IMU angles; speed integration | rad, rad/s | limited modes use ordinary error; circular modes use Angle error |
| Feeders | configured shots/revolution divided by rev | shots/rad; bullet speed m/s; firing rate shots/s | rub speed = bullet speed/radius; feed speed = rate/coefficient |
| IMU | EKF Yaw/Pitch/Roll/YawTotalAngle in degrees | rad; total yaw unwrapped; gyro rad/s; accel m/s^2 | quaternion and frame functions consume radians |
| IMU heater | target 45 Celsius, kp=100*pct/C, ki=1*pct/C | target 45; kp=1; ki=0.01; integral limit 0.4 | duty ratio capped at 1, no Kelvin offset |
| MAVLink vision | yaw/pitch degrees; wr rpm; distance m | Angle radians, rad/s, m | existing fields/control branches |
| MAVLink Insta360 | a0/a1/a2 degrees, NaN means no detection | float radians; confidence unchanged | Sentry consumes directly without another *deg |
| MAVLink navigation | position m, speed m/s, yaw degrees | m, m/s, rad | target position unchanged; former swapped identity tags removed |
| MAVLink telemetry | IMU radians, referee seconds and m/s | SI members | radians /deg, remaining seconds truncated to uint16 |
| UI | angles, rotation rate, ratios, bullet speed/rate, current | rad, rad/s, ratio, m/s or shots/s, A | /deg, /rpm, /pct before integer conversion |
| Hero UI | formerly bullet speed *10 upstream, range 200 | speed m/s, range 20 | *10 only for the displayed number; progress ratio unchanged |
| Flash | `yaw_offset` float | normalized rad | raw radians both ways, same key in both gimbals |

Every former default-constructed quantity is zero initialized after conversion,
including omitted PID/config fields, motor feedback, control targets and locals.
Numeric input arguments are passed by value. Non-unit scale factors and reciprocal
factors remain in place; no PID tuning was performed.

The complete before/after configuration values are in
[`configuration-audit.csv`](../build/unit-migration/host/configuration-audit.csv).
For example, hero yaw-speed kp remains 1.4323945 A/(rad/s), pitch-speed kp remains
2.86478901 N*m/(rad/s), pitch ki remains 57.2957802, and the hero feeder coefficient
remains 0.95492959 shots/rad.

## Verification

Baseline revision: `f72e9041bed01fb8e6ded80665dc3a7b63795b9e`, with the user's
pre-existing gripper deletions excluded from firmware builds. The starting staged
deletions and `.idea/editor.xml` changes were preserved. The baseline archive and
extracted source are under `build/unit-migration`.

- MinGW GCC 15.2.0, explicit C++17: math edge/constexpr tests and 200,000 random
  float bit patterns checked against fmod. Includes neighboring endpoints,
  max-float inputs, NaN/Inf, conversions, inverse scales and helper NaN behavior.
- Arm GCC 15.3.1: the same math test translation unit compiles, including constexpr
  assertions. Firmware language standard/toolchain settings were not upgraded.
- Actual old/new PID, DWT, all ten motor drivers, three chassis implementations,
  both feeders, gimbals, IMU adapter, UI and MAVLink are compiled into separate
  host programs. Hardware time registers, CAN/CDC/UART, sensor/EKF inputs, heater
  output and Flash storage are stubbed; control algorithms are not duplicated.
- 7,004 differential observations across hero, infantry, sentry and engineer
  chassis configuration: zero failures. Float tolerance is
  `1e-6 + 1e-5 * abs(baseline)`. CAN bytes, MAVLink packets including CRCs, and UI
  integer fields are compared exactly. Both sides producing the known power-limit
  NaN is recorded as agreement, not evidence that the old formula is valid.
- 2,077 additional UI degree truncation and VESC eRPM quantization boundary cases:
  zero integer differences, no tolerance relaxation.
- Supplemental ARM syntax compilation: 12 of 13 debug examples and engineer
  chassis pass; the same two pre-existing failures occur before and after migration.
- Production-source residual scan: no UnitFloat, old Angle templates, toFloat,
  unit namespace/includes or runtime unit metadata. `git diff --check` passes.

All firmware builds below ran through CLion's integrated terminal using its
bundled CMake/Ninja. Each explicitly configured one vehicle, checked
`compile_commands.json`, and rebuilt with `--clean-first`. Logs, ELF, map,
app-input record and memory report are archived per vehicle/configuration in
[`build/unit-migration/final`](../build/unit-migration/final).

| Vehicle | Configuration | Result | RAM bytes | Flash bytes |
| --- | --- | --- | ---: | ---: |
| hero | Debug | PASS | 33,064 | 108,992 |
| hero | Release | PASS | 32,992 | 117,088 |
| infantry | Debug | PASS | 32,416 | 107,644 |
| infantry | Release | PASS | 32,344 | 114,068 |
| sentry | Debug | PASS | 32,880 | 109,968 |
| sentry | Release | PASS | 32,808 | 120,448 |

CCMRAM use is zero for all builds; available RAM is 128 KiB and Flash is 1 MiB.
The pre-migration Debug RAM/Flash baselines were hero 57,328/153,380,
infantry 54,016/150,652, sentry 56,552/158,832 bytes. Hero Release baseline was
57,264/142,696 bytes. Default selection and `build/Debug`, `build/Release` artifacts
are restored to hero.

## Retained Findings and Limits

- `app/engineer/misc.hpp` constructs Mavlink without the required config argument.
- `app/debug/gimbal_motor.cpp` references an undefined `motor` object. Neither
  compile failure was introduced or fixed by this migration.
- `Chassis_DualSteer::powerControlAfterMotorUpdate` divides by `2*a`; zero commanded
  currents (including low-buffer shutdown) can yield NaN. The differential fixture
  reproduces it in both versions. It remains unchanged.
- VESC duty retains `/1000 *pct`, which should be checked against the actual VESC
  firmware protocol before any separate correction.
- The former CyberDog Nm/current tag, dimensionless current ratios labeled A,
  Robot angle/acceleration/firing-rate tags and MAVLink position/velocity tags were
  inconsistent. Identity factors meant the existing numeric values could be kept.
- Motor reduction accumulates encoder deltas but its final stored angle is still
  normalized; it is not an unbounded multi-turn measurement. The comment is clarified.
- 17 mm and 42 mm shooter UI comparisons use different inequality directions;
  their exact-threshold and NaN color behavior remains different.
- Release warnings from third-party `QuaternionEKF.c` (type punning / aliasing)
  and `ui_interface.c` (array bounds) also occur in the baseline Release build.
  New plain-float CPU usage locals produce an unused-variable warning; the
  calculation is retained. No warning was suppressed to obtain the results.

Software verification does not establish robot behavior under real sensor noise,
interrupt timing or mechanical load. No firmware was flashed, no robot was started,
and no PID parameters were tuned.
