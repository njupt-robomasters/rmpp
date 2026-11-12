import sys
p = sys.argv[1]
s = open(p, encoding='utf-8', newline='').read()
nl = '\r\n' if '\r\n' in s else '\n'
start = s.index('    // 五连杆示意（站立姿态）')
end = s.index('    // 单腿状态与输出')

def leg(title, front_back, motors, rel):
    return [
        f'    // {title}',
        '    //',
        f'    //   {front_back}',
        '    //',
        f'    //                  O ------> +x       O：{motors}',
        '    //                 / \                 angle_bod：O 处 ∠BOD',
        '    //    AB (phi1)   /   \   ED (phi4)',
        '    //               /     \               angle_obc：B 处 ∠OBC',
        '    //              B       D',
        '    //               \     /',
        '    //    BC (phi2)   \   /   DC (phi3)',
        '    //                 \ /',
        '    //                  C                 轮轴',
        '    //                  |',
        f'    //                  v +y               rel_theta：{rel}',
        '    //',
    ]

lines = [
    '    // 五连杆示意（站立姿态）：关节电机 A、E 同轴装于 O，A 驱动 AB、E 驱动 ED；C 为轮轴',
    '    // 图中 x 向右、y 向下，杆方位角从 +x 向 +y（顺时针）为正；电机零位时 AB 指向 -x（phi1 = π）、ED 指向 +x（phi4 = 0）',
    '    // 虚拟腿为 OC：L = |OC|，phi0 为 OC 方位角，正下方为 π/2',
    '    //',
] + leg('左腿：在车左侧面向关节电机输出轴看，车头在左', '车头 <--                         --> 车尾',
        '关节电机 A（left_back）、E（left_front）', 'C 偏向车尾（右）为正') \
  + leg('右腿：在车右侧面向关节电机输出轴看，车头在右；与左腿镜像，同一套方位角定义', '车尾 <--                         --> 车头',
        '关节电机 A（right_front）、E（right_back）', 'C 偏向车尾（左）为正')

for l in lines:
    assert not l.rstrip().endswith('\'), l
s = s[:start] + nl.join(lines) + nl + s[end:]
open(p, 'w', encoding='utf-8', newline='').write(s)
