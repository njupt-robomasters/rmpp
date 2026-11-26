const fs=require('fs');
const root='C:/Users/joushua/Desktop/RM/rmpp/';
function replace(file, before, after) {
 const p=root+file;
 const s=fs.readFileSync(p,'utf8').replace(/\r\n/g,'\n');
 if(s.includes(after)) return;
 if(!s.includes(before)) throw new Error('Target changed: '+file);
 fs.writeFileSync(p,s.replace(before,after));
}
replace('app/jelly_sentry/app.cpp',
 'static uint8_t chassis_divider = 0, yaw2_divider = 0;',
 'static uint8_t chassis_divider = 0, pitch1_divider = 0, yaw2_divider = 0, shoot_divider = 0;');
replace('app/jelly_sentry/app.cpp',
 '    // CAN2：折叠关节 pitch1、俯仰 pitch2\n    pitch1.send();\n    pitch2.send();',
 '    // CAN2：pitch1 仅负责机构升降，每 4 ms 发送（250 Hz）；pitch2 保持 1 kHz\n    if (++pitch1_divider >= 4) {\n        pitch1_divider = 0;\n        pitch1.send();\n    }\n    pitch2.send();');
replace('app/jelly_sentry/app.cpp',
 '    // CAN2：拨弹 EL05\n    shoot.send();',
 '    // CAN2：拨弹 EL05，每 2 ms 发送（500 Hz）\n    if (++shoot_divider >= 2) {\n        shoot_divider = 0;\n        shoot.send();\n    }');
for(const file of ['app/jelly_sentry_debug/shooter/app.cpp','app/jelly_sentry_debug/shooter_shoot/app.cpp']) {
 replace(file,'    Timer loop_timer;','    Timer loop_timer;\n    uint8_t shoot_divider = 0;');
 replace(file,'        shoot.send();','        // 控制计算保持 1 kHz，EL05 每 2 ms 发送（500 Hz）\n        if (++shoot_divider >= 2) {\n            shoot_divider = 0;\n            shoot.send();\n        }');
}
replace('agents/02-conventions/application.md',
 '- 整车和发射调试入口共用shoot实例，每1ms调用send；失能停止帧、使能帧和周期控制帧均请求状态应答',
 '- 整车和发射调试入口共用shoot实例，控制计算保持1ms，EL05每2ms调用send（500Hz）；失能停止帧、使能帧和周期控制帧均请求状态应答');
fs.appendFileSync(root+'agents/02-conventions/application.md','\n## jelly_sentry CAN2 发送调度\n\n- 整车控制计算保持1kHz；pitch1仅负责阶段性机构升降，每4个控制周期发送一次（250Hz），pitch2每周期发送（1000Hz）\n- 小yaw 7025每2周期发送一次（500Hz），0xA1电流控制与0x94读角度交替，各250Hz\n- EL05每2周期发送一次（500Hz），发射调试shooter和shooter_shoot使用相同发送频率\n- 失能、使能和清错帧替换当次发送，不额外增加发送频率。正常每请求一帧应答一帧时，主控发送2250帧/秒、反馈预计2250帧/秒，总计4500帧/秒；1Mbps标准8字节帧理论占用约50%~61%，不含错误重传与额外主动上报\n- 频率是按1ms主循环计算的目标值，真实周期、反馈频率与7025断联是否改善仍需上板测量\n');
fs.appendFileSync(root+'agents/01-board/mc02_bsp/README.md','\n### CAN2 降载（2026-11-14）\n\n- 按用户指定：pitch1发送250Hz、EL05发送500Hz；pitch2保持1000Hz、小yaw保持总500Hz（A1/94各250Hz），主控控制计算周期不变\n- 整车app.cpp分频发送；shooter与shooter_shoot调试入口同步EL05 500Hz。gimbal调试当前只发送pitch2，shooter_rub不发送EL05，未改动\n- 正常一问一答时总线预计4500帧/秒，相比原7000帧/秒减少约35.7%；理论占用从78%~95%降至50%~61%。未抓包实测，尚不能认定已修复7025断联\n- 脚本can2_rate_update.cjs保存本轮修改；局部编译与最终检查结果见下方续记\n');
replace('agents/README.md','EL05 MIT 接入与待上板验证项','EL05 MIT 接入、CAN2 降载与待上板验证项');
console.log('CAN2 rates updated: pitch1 250 Hz, EL05 500 Hz.');
