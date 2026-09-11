# -*- coding: utf-8 -*-
"""
问题1：交会定位法多边形定位区域直径计算与覆盖性分析 —— 算法实现
对应文档：问题1-交会定位区域直径与覆盖性.md（§3 半平面交、§4 直径、§5 覆盖性）

用法：
    python problem1_algorithm.py              # 运行内置三个算例的自测
    from problem1_algorithm import locate_region
"""

import math

EPS = 1e-9            # 浮点容差（§3.3【补充】）
DISK_R = 1800.0       # 目标区域半径（米）
DISK_N = 96           # 圆域内接正多边形边数


def sector_to_halfplanes(x, y, theta_deg, delta_deg=1.0):
    """§3.1：将检测点 (x,y) 处示向度 theta_deg 的 ±delta 误差扇区转化为两个半平面
    a*X + b*Y + c <= 0，返回 [(a1,b1,c1), (a2,b2,c2)]。
    """
    halfplanes = []
    for sign in (-1.0, +1.0):          # -1：下边界 θ-δ；+1：上边界 θ+δ
        ang = math.radians(theta_deg + sign * delta_deg)
        if sign < 0:
            a, b = math.sin(ang), -math.cos(ang)
        else:
            a, b = -math.sin(ang), math.cos(ang)
        c = -(a * x + b * y)
        halfplanes.append((a, b, c))
    return halfplanes


def _regular_polygon(cx, cy, r, n):
    """圆心 (cx,cy)、半径 r 的内接正 n 边形，顶点按逆时针排列。"""
    return [(cx + r * math.cos(2 * math.pi * k / n),
             cy + r * math.sin(2 * math.pi * k / n)) for k in range(n)]


def _clip_polygon(poly, a, b, c):
    """用半平面 a*X + b*Y + c <= 0 切割凸多边形（含边界，容差 EPS）。"""

    def inside(p):
        return a * p[0] + b * p[1] + c <= EPS

    def intersect(p, q):
        vp = a * p[0] + b * p[1] + c
        vq = a * q[0] + b * q[1] + c
        t = vp / (vp - vq)
        return (p[0] + t * (q[0] - p[0]), p[1] + t * (q[1] - p[1]))

    if not poly:
        return []
    out = []
    prev, prev_in = poly[-1], inside(poly[-1])
    for cur in poly:
        cur_in = inside(cur)
        if cur_in:
            if not prev_in:
                out.append(intersect(prev, cur))
            out.append(cur)
        elif prev_in:
            out.append(intersect(prev, cur))
        prev, prev_in = cur, cur_in
    return out


def halfplane_intersection(halfplanes):
    """§3.3：增量切割法求半平面交。
    初始多边形取目标圆域的内接正 DISK_N 边形（修正：不用矩形包围盒）：
    题目背景保证干扰源位于圆域内，且真实干扰源必在所有误差扇区内，
    故圆域裁剪不丢失真实定位区域，并保证结果非空、有界。
    """
    verts = _regular_polygon(0.0, 0.0, DISK_R, DISK_N)
    for (a, b, c) in halfplanes:
        verts = _clip_polygon(verts, a, b, c)
        if len(verts) < 3:
            break
    return verts


def polygon_diameter(verts):
    """§4.2：凸多边形直径在顶点对之间取得，枚举所有顶点对。返回 (D, (A, B))。"""
    best_d2, pair = -1.0, None
    for i in range(len(verts)):
        for j in range(i + 1, len(verts)):
            d2 = (verts[i][0] - verts[j][0]) ** 2 + (verts[i][1] - verts[j][1]) ** 2
            if d2 > best_d2:
                best_d2, pair = d2, (verts[i], verts[j])
    return math.sqrt(best_d2), pair


def check_coverage(verts, A, B):
    """§5.2：以 AB 为直径作圆（圆心为 AB 中点、半径 D/2），
    检查定位区域所有顶点是否在圆内（含边界，容差 EPS）。
    返回 (是否覆盖, 顶点到圆心最大距离, 圆半径)。
    """
    ox, oy = (A[0] + B[0]) / 2.0, (A[1] + B[1]) / 2.0
    r = math.hypot(A[0] - B[0], A[1] - B[1]) / 2.0
    max_d = max(math.hypot(v[0] - ox, v[1] - oy) for v in verts)
    return max_d <= r + EPS, max_d, r


def locate_region(measurements, delta_deg=1.0):
    """总流程。measurements = [(x, y, theta_deg), ...]
    返回 (顶点列表, 直径 D, 直径端点对 (A, B), 覆盖性判定 bool)。
    """
    halfplanes = []
    for (x, y, th) in measurements:
        halfplanes.extend(sector_to_halfplanes(x, y, th, delta_deg))
    verts = halfplane_intersection(halfplanes)
    if len(verts) < 3:
        raise ValueError("定位区域为空或退化（顶点数 < 3）")
    D, (A, B) = polygon_diameter(verts)
    covered, _, _ = check_coverage(verts, A, B)
    return verts, D, (A, B), covered


def _show(name, verts, D, pair, covered):
    print(f"【{name}】定位区域顶点（{len(verts)} 个）：")
    for v in verts:
        print(f"    ({v[0]:10.4f}, {v[1]:10.4f})")
    (A, B) = pair
    print(f"  直径 D = {D:.4f} 米，端点 ({A[0]:.4f}, {A[1]:.4f}) 与 ({B[0]:.4f}, {B[1]:.4f})")
    print(f"  以直径为直径的圆能否覆盖定位区域：{'能' if covered else '不能'}\n")


def main():
    # §6.1 算例1：两检测点交会定位
    meas1 = [(0, 0, 60), (100, 0, 120)]
    verts1, D1, pair1, cov1 = locate_region(meas1)
    _show("算例1", verts1, D1, pair1, cov1)
    assert abs(D1 - 6.9884) < 1e-3, D1
    assert cov1 is True

    # §6.2 算例2：三检测点环绕定位
    meas2 = [(0, 0, 45), (1000, 0, 135), (500, 1000, 270)]
    verts2, D2, pair2, cov2 = locate_region(meas2)
    _show("算例2", verts2, D2, pair2, cov2)
    assert abs(D2 - 34.9208) < 1e-3, D2
    assert cov2 is True

    # §6.3 算例3：等边三角形反例（不能覆盖）
    # 注：文档中检测点2、3的坐标四舍五入到 2 位小数（674.65, -755.93 等）；
    # 此处按构造精确计算：检测点位于三角形边的延长线上、距顶点 B/C 恰 1000 米，
    # 扇区边界方向角恰为 49.1066° / 130.8934°，从而交会区域恰为边长 40 米的
    # 等边三角形（B、C 精确为 (±20, 0)，D = 40.0000）。
    ang49 = math.radians(49.1066)
    s2 = (20 + 1000 * math.cos(ang49), -1000 * math.sin(ang49))    # ≈ (674.65, -755.93)
    s3 = (-20 - 1000 * math.cos(ang49), -1000 * math.sin(ang49))   # ≈ (-674.65, -755.93)
    meas3 = [(-1000, 0, 1), (s2[0], s2[1], 131.8934), (s3[0], s3[1], 48.1066)]
    verts3, D3, pair3, cov3 = locate_region(meas3)
    _show("算例3", verts3, D3, pair3, cov3)
    assert abs(D3 - 40.0) < 1e-3, D3
    assert cov3 is False

    print("三个算例全部通过，数值与文档一致。")


if __name__ == "__main__":
    main()
