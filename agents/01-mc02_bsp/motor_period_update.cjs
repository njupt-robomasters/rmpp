const fs=require('fs');const root='C:/Users/joushua/Desktop/RM/rmpp/';
function replace(file,before,after){const p=root+file;const s=fs.readFileSync(p,'utf8').replace(/\r\n/g,'\n');if(!s.includes(before))throw Error('Target changed: '+file);fs.writeFileSync(p,s.replace(before,after));}
for(const file of ['app/jelly_sentry/gimbal.hpp','app/jelly_sentry/shooter.hpp'])replace(file,'#pragma once\n','#pragma once\n\n#include <cstdint>\n');
replace('app/jelly_sentry/gimbal.hpp','inline RMDL7025 yaw2({','// 运行时可写发送周期，单位 ms；主循环约 1 ms，7025 建议取 2~99\ninline volatile uint32_t yaw2_send_period_ms = 2;\ninline RMDL7025 yaw2({');
replace('app/jelly_sentry/gimbal.hpp','inline DM4310 pitch1({','// 运行时可写发送周期，单位 ms，建议取 1~99，避免超过反馈超时\ninline volatile uint32_t pitch1_send_period_ms = 4;\ninline DM4310 pitch1({');
replace('app/jelly_sentry/gimbal.hpp','inline DM4310 pitch2({','// 运行时可写发送周期，单位 ms，建议取 1~99，避免超过反馈超时\ninline volatile uint32_t pitch2_send_period_ms = 1;\ninline DM4310 pitch2({');
replace('app/jelly_sentry/shooter.hpp','inline EL05 shoot({','// 运行时可写发送周期，单位 ms，建议取 1~99，避免超过反馈超时\ninline volatile uint32_t shoot_send_period_ms = 2;\ninline EL05 shoot({');
replace('app/jelly_sentry/app.cpp','static uint8_t chassis_divider = 0, pitch1_divider = 0, yaw2_divider = 0, shoot_divider = 0;','static uint8_t chassis_divider = 0;\nstatic uint32_t pitch1_divider = 0, pitch2_divider = 0, yaw2_divider = 0, shoot_divider = 0;');
replace('app/jelly_sentry/app.cpp','// CAN2：pitch1 仅负责机构升降，每 4 ms 发送（250 Hz）；pitch2 保持 1 kHz','// CAN2：pitch1 / pitch2 发送周期由 gimbal.hpp 的可写变量决定');
replace('app/jelly_sentry/app.cpp','if (++pitch1_divider >= 4)','if (++pitch1_divider >= pitch1_send_period_ms)');
replace('app/jelly_sentry/app.cpp','    pitch2.send();','    if (++pitch2_divider >= pitch2_send_period_ms) {\n        pitch2_divider = 0;\n        pitch2.send();\n    }');
replace('app/jelly_sentry/app.cpp','// CAN2：小 yaw 7025 应答上限 500 Hz，每 2 ms 发一帧，转矩与读角度交替，各 250 Hz','// CAN2：小 yaw 7025 转矩与读角度交替，各占总发送频率的一半');
replace('app/jelly_sentry/app.cpp','if (++yaw2_divider >= 2)','if (++yaw2_divider >= yaw2_send_period_ms)');
replace('app/jelly_sentry/app.cpp','// CAN2：拨弹 EL05，每 2 ms 发送（500 Hz）','// CAN2：EL05 发送周期由 shooter.hpp 的可写变量决定');
replace('app/jelly_sentry/app.cpp','if (++shoot_divider >= 2)','if (++shoot_divider >= shoot_send_period_ms)');
for(const file of ['app/jelly_sentry_debug/shooter/app.cpp','app/jelly_sentry_debug/shooter_shoot/app.cpp']) {
replace(file,'uint8_t shoot_divider = 0;','uint32_t shoot_divider = 0;');
replace(file,'// 控制计算保持 1 kHz，EL05 每 2 ms 发送（500 Hz）','// 控制计算保持 1 kHz，发送周期与整车 shoot_send_period_ms 共用');
replace(file,'if (++shoot_divider >= 2)','if (++shoot_divider >= shoot_send_period_ms)');
}
replace('app/jelly_sentry_debug/gimbal/app.cpp','    Timer loop_timer;','    Timer loop_timer;\n    uint32_t pitch2_divider = 0;');
replace('app/jelly_sentry_debug/gimbal/app.cpp','        pitch2.send();','        if (++pitch2_divider >= pitch2_send_period_ms) {\n            pitch2_divider = 0;\n            pitch2.send();\n        }');
replace('app/jelly_sentry_debug/gimbal_motor/app.cpp','// - 测试 yaw2 时把循环周期改为 2 ms（7025 应答上限 500 Hz）','// - send_period_ms 绑定对应电机的发送周期；测试 yaw2 时绑定 yaw2_send_period_ms，控制计算仍为 1 kHz');
replace('app/jelly_sentry_debug/gimbal_motor/app.cpp','static DM4310& motor = pitch2;','static DM4310& motor = pitch2;\nstatic volatile uint32_t& send_period_ms = pitch2_send_period_ms;');
replace('app/jelly_sentry_debug/gimbal_motor/app.cpp','    Timer loop_timer;','    Timer loop_timer;\n    uint32_t send_divider = 0;');
replace('app/jelly_sentry_debug/gimbal_motor/app.cpp','        motor.send();','        if (++send_divider >= send_period_ms) {\n            send_divider = 0;\n            motor.send();\n        }');
replace('agents/docs/application.md','- 整车和发射调试入口共用shoot实例，控制计算保持1ms，EL05每2ms调用send（500Hz）；失能停止帧、使能帧和周期控制帧均请求状态应答','- 整车和发射调试入口共用shoot实例，控制计算保持1ms，EL05按shoot_send_period_ms发送（默认2ms、500Hz）；失能停止帧、使能帧和周期控制帧均请求状态应答');
replace('agents/docs/application.md','- 整车控制计算保持1kHz；pitch1仅负责阶段性机构升降，每4个控制周期发送一次（250Hz），pitch2每周期发送（1000Hz）','- 整车控制计算保持1kHz；pitch1、pitch2和yaw2的发送周期可写变量放在gimbal.hpp各电机实例旁，EL05的shoot_send_period_ms放在shooter.hpp的shoot实例旁');
replace('agents/docs/application.md','- 小yaw 7025每2周期发送一次（500Hz），0xA1电流控制与0x94读角度交替，各250Hz','- 默认周期：pitch1_send_period_ms=4（250Hz）、pitch2_send_period_ms=1（1000Hz）、yaw2_send_period_ms=2（500Hz，A1/94各250Hz）、shoot_send_period_ms=2（500Hz）');
replace('agents/docs/application.md','- EL05每2周期发送一次（500Hz），发射调试shooter和shooter_shoot使用相同发送频率','- 各变量为inline volatile uint32_t，运行时通过调试器写入，下一个发送判断使用新值，不清零已有计数。计数器保留在cpp，不作为调频入口；周期0等效每主循环发送，超过100ms可能触发主控反馈离线，7025建议周期至少2ms');
fs.appendFileSync(root+'agents/docs/application.md','- gimbal单轴调试共用pitch2周期；gimbal_motor通过send_period_ms引用当前测试电机的周期变量；shooter与shooter_shoot共用shoot周期。变量仅影响CAN发送，反馈按电机应答而定，不改变PID计算周期\n');
fs.appendFileSync(root+'agents/01-mc02_bsp/README.md','\n### 运行时发送周期（2026-11-14）\n\n- 按用户要求在hpp各电机实例旁加入inline volatile uint32_t周期变量：pitch1_send_period_ms=4、pitch2_send_period_ms=1、yaw2_send_period_ms=2、shoot_send_period_ms=2\n- 整车、pitch2单轴/通用单电机调试、两个EL05发射调试入口读取对应周期；计数器为uint32_t，控制计算周期保持1ms\n- 调试器可写，当前周期计数保留，下一次判断使用新值；0等效1ms，7025建议不小于2ms，周期应小于主控100ms反馈超时\n- 尚未烧录或通过探针改写变量，局部编译与检查结果见续记\n');
console.log('Writable motor send periods added beside motor instances.');
