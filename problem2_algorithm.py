# -*- coding: utf-8 -*-
"""
问题2：第二检测点选择策略与候选区域 —— 数值实现与验证
对应文档：问题2-第二个检测点的选择策略与候选区域.md

内容：
  1. §3.2  两扇区交会定位"精确直径"表（楔形真实几何）
  2. §4.1  已知 d1 时最优 S2 的定位直径（精确 vs 近似）
  3. §4.2  min-max 数值搜索最优 S2（精确最坏直径，含 α_min 约束/无约束）
  4. §5.7  候选区域面积表（最大距离/圆弧可检测性，闭式积分）
  5. §7    数值示例验证

关键点：§3.1 的近似公式 D≈2tanδ√(d1²+d2²)/sinα 是"平行条带"模型，
仅在两个扇区都较"深"（d1、d2 均远大于对方的半宽 d·tanδ）时成立。
当其中一个检测点离干扰源很近（d2 < d1·tanδ，或 d1 < d2·tanδ）时，
交会区域是三角形而非平行四边形，近似公式会高估直径。
故 §4.1、§4.2 一律用精确直径 exact_two_sector_diameter（射线交法）。

用法：
    python problem2_algorithm.py          # 运行全部自测，与文档数值比对
"""

import math
from problem1_algorithm import sector_to_halfplanes, locate_region

DELTA_DEG = 1.0                # 示向度误差半角
G_MIN, G_MAX = 5.0, 1500.0     # 局部坐标下干扰源距离范围


# ---------------- 基础工具 ----------------

def approx_diameter(d1, d2, alpha_deg, delta_deg=DELTA_DEG):
    """§3.1 近似公式（平行条带模型）D ≈ 2 tanδ · √(d1²+d2²) / sinα。"""
    return 2 * math.tan(math.radians(delta_deg)) * math.hypot(d1, d2) / \
        math.sin(math.radians(alpha_deg))


def s2_from_angle(d1, d2, alpha_deg):
    """构造两检测点配置：G 在原点，S1=(-d1,0)、示向度 0°，两示向度线在 G 处交角 α。
    返回 (S1, θ1, S2, θ2)。"""
    a = math.radians(alpha_deg)
    S1, th1 = (-d1, 0.0), 0.0
    S2 = (-d2 * math.cos(a), d2 * math.sin(a))
    th2 = math.degrees(math.atan2(-math.sin(a), math.cos(a))) % 360.0
    return S1, th1, S2, th2


def bearing_from(S, G):
    """从 S 指向 G 的方位角（度，[0,360)）。"""
    return math.degrees(math.atan2(G[1] - S[1], G[0] - S[0])) % 360.0


def _ray_intersect(r1, r2):
    """两射线（起点+方向）交点，返回 (点, t1, t2)；平行返回 None。"""
    x1, y1, dx1, dy1 = r1
    x2, y2, dx2, dy2 = r2
    det = dx1 * dy2 - dy1 * dx2
    if abs(det) < 1e-12:
        return None
    t1 = ((x2 - x1) * dy2 - (y2 - y1) * dx2) / det
    t2 = ((x2 - x1) * dy1 - (y2 - y1) * dx1) / det
    return ((x1 + t1 * dx1, y1 + t1 * dy1), t1, t2)


def _point_in_sector(P, S, th, delta_deg=DELTA_DEG):
    """点 P 是否在检测点 S、示向度 th 的 ±δ 扇区内。"""
    for (a, b, c) in sector_to_halfplanes(S[0], S[1], th, delta_deg):
        if a * P[0] + b * P[1] + c > 1e-9:
            return False
    return True


def exact_two_sector_diameter(S1, th1, S2, th2, delta_deg=DELTA_DEG):
    """两检测点交会定位区域的精确直径（楔形真实几何，射线交法）。

    交会区域 = 两个 ±δ 扇区的交集，是凸多边形，顶点为：
      - 两两边界射线的交点（两射线 t≥0 时有效）；
      - 若某检测点（扇区顶点）落在对方扇区内，则该顶点也是区域顶点。
    直径在顶点对之间取得。
    """
    def r(S, th):
        a = math.radians(th)
        return (S[0], S[1], math.cos(a), math.sin(a))

    rays1 = [r(S1, th1 - delta_deg), r(S1, th1 + delta_deg)]
    rays2 = [r(S2, th2 - delta_deg), r(S2, th2 + delta_deg)]

    verts = []
    for ra in rays1:
        for rb in rays2:
            hit = _ray_intersect(ra, rb)
            if hit and hit[1] >= -1e-9 and hit[2] >= -1e-9:
                verts.append(hit[0])
    if _point_in_sector(S1, S2, th2, delta_deg):
        verts.append(S1)
    if _point_in_sector(S2, S1, th1, delta_deg):
        verts.append(S2)

    uniq = []
    for p in verts:
        if all(math.hypot(p[0] - q[0], p[1] - q[1]) > 1e-6 for q in uniq):
            uniq.append(p)

    D = 0.0
    for i in range(len(uniq)):
        for j in range(i + 1, len(uniq)):
            D = max(D, math.hypot(uniq[i][0] - uniq[j][0], uniq[i][1] - uniq[j][1]))
    return D


def exact_diameter_local(x, y, g, delta_deg=DELTA_DEG):
    """局部坐标（S1=(0,0)、示向度 0°、源 G=(g,0)、S2=(x,y)）下的精确定位直径。"""
    th2 = bearing_from((x, y), (g, 0.0))
    return exact_two_sector_diameter((0.0, 0.0), 0.0, (x, y), th2, delta_deg)


def worst_case_diameter_exact(x, y, g_min=G_MIN, g_max=G_MAX, step=1.0):
    """给定 S2=(x,y)，对所有可能干扰源位置 g∈[g_min,g_max] 取最坏（最大）精确直径。"""
    worst = 0.0
    n = int(round((g_max - g_min) / step))
    for k in range(n + 1):
        g = g_min + k * step
        D = exact_diameter_local(x, y, g)
        if D > worst:
            worst = D
    return worst


def min_sin_alpha(x, y, g_min=G_MIN, g_max=G_MAX):
    """S2=(x,y) 对所有 g 的最小 sinα（在 |x-g| 最大的端点处取得）。"""
    dmax = max(abs(x - g_min), abs(x - g_max))
    return abs(y) / math.hypot(dmax, abs(y))


# ---------------- §3.2 精确直径表 ----------------

def sec32():
    print("【§3.2】d1=d2=1000, δ=1° 时的定位区域直径（精确 vs 近似）")
    expected = {10: 417.15, 30: 135.46, 60: 69.88, 90: 49.39,
                120: 69.80, 150: 134.84, 170: 400.43}
    print(f"{'α':>5} | {'近似':>8} | {'精确':>8} | {'文档':>8} | {'|Δ|':>6}")
    for alpha in (10, 30, 60, 90, 120, 150, 170):
        S1, th1, S2, th2 = s2_from_angle(1000, 1000, alpha)
        De = exact_two_sector_diameter(S1, th1, S2, th2)
        Da = approx_diameter(1000, 1000, alpha)
        # 与通用半平面交（问题1）交叉核对
        _, Dref, _, _ = locate_region([(S1[0], S1[1], th1), (S2[0], S2[1], th2)])
        exp = expected[alpha]
        err = abs(De - exp)
        print(f"{alpha:>5} | {Da:>8.2f} | {De:>8.2f} | {exp:>8.2f} | {err:>6.2f}")
        assert err < 0.05, (alpha, De, exp)
        assert abs(De - Dref) < 0.01, (alpha, De, Dref)


# ---------------- §4.1 已知 d1 的最优 S2 ----------------

def sec41():
    print("\n【§4.1】已知 d1，最优 S2=(d1, ε)、ε 取下界 5 展示（α=90°）")
    print(f"{'d1':>6} | {'近似':>8} | {'精确':>8} | {'文档(近似)':>10}")
    for d1 in (100, 500, 1000, 1500):
        Da = approx_diameter(d1, 5.0, 90.0)
        De = exact_two_sector_diameter((0.0, 0.0), 0.0, (float(d1), 5.0), 270.0)
        print(f"{d1:>6} | {Da:>8.2f} | {De:>8.2f} | {Da:>10.2f}")
    print("  注：文档表用的是近似公式（且 34.92/52.38 有舍入误差，应为 34.91/52.37）；"
          "精确直径在 d1 较大时明显更小（交会区域由平行四边形退化为三角形）。")


# ---------------- §4.2 min-max 搜索（精确） ----------------

def _feasible(x, y, alpha_min_deg, R, g_min=G_MIN, g_max=G_MAX):
    if abs(y) <= 5.0:
        return False
    if math.hypot(x - g_min, y) > R or math.hypot(x - g_max, y) > R:
        return False
    if alpha_min_deg is not None:
        if max(abs(x - g_min), abs(x - g_max)) > abs(y) / math.tan(math.radians(alpha_min_deg)):
            return False
    return True


def search_minmax(alpha_min_deg, R=1500.0, coarse=40.0, refine=5.0, refine_span=40.0,
                  g_step_coarse=5.0, g_step_refine=2.0):
    """在候选区域内最小化最坏（精确）定位直径，返回 (最坏直径, x, y)。"""
    best = [None, None, None]

    def consider(x, y, step):
        if not _feasible(x, y, alpha_min_deg, R):
            return
        w = worst_case_diameter_exact(x, y, step=step)
        if best[0] is None or w < best[0]:
            best[0], best[1], best[2] = w, x, y

    # 粗网格（只搜上半平面，区域关于 x 轴对称）
    x = G_MIN
    while x <= G_MAX + 1e-9:
        y = G_MIN
        while y <= G_MAX + 1e-9:
            consider(x, y, g_step_coarse)
            y += coarse
        x += coarse

    # 局部精化
    bx, by = best[1], best[2]
    x = bx - refine_span
    while x <= bx + refine_span + 1e-9:
        y = by - refine_span
        while y <= by + refine_span + 1e-9:
            consider(x, y, g_step_refine)
            y += refine
        x += refine

    # 终值用更细的 g 扫描报出
    w = worst_case_diameter_exact(best[1], best[2], step=0.5)
    return w, best[1], best[2]


def sec42():
    print("\n【§4.2】min-max 搜索最优 S2（精确最坏直径，R=1500）")
    w, x, y = search_minmax(alpha_min_deg=None)
    s = min_sin_alpha(x, y)
    print(f"  无 α_min 约束：S2≈({x:.0f}, {y:.0f})，最坏精确直径≈{w:.1f} m"
          f"（最小 sinα≈{s:.3f}，对应 α_min≈{math.degrees(math.asin(s)):.1f}°）")

    w45, x45, y45 = search_minmax(alpha_min_deg=45.0)
    print(f"  α_min=45° 约束：S2≈({x45:.0f}, {y45:.0f})，最坏精确直径≈{w45:.1f} m")


# ---------------- §5.7 候选区域面积 ----------------

def candidate_area(alpha_min_deg, R, g_min=G_MIN, g_max=G_MAX):
    """候选区域总面积（最大距离/圆弧可检测性，闭式积分）。

    总面积 = 4 ∫_{h}^{R cosα_min} [√(R²-m²) - m tanα_min] dm，h=(g_max-g_min)/2。
    """
    h = (g_max - g_min) / 2.0
    a = math.radians(alpha_min_deg)
    Mc = R * math.cos(a)
    if Mc < h:
        return 0.0
    def F(m):
        return (m / 2.0) * math.sqrt(R * R - m * m) + \
               (R * R / 2.0) * math.asin(m / R) - (math.tan(a) / 2.0) * m * m
    return 4.0 * (F(Mc) - F(h))


def _numeric_area_check(alpha_min_deg, R, n=200000):
    """中点法数值积分核对闭式解。"""
    h = (G_MAX - G_MIN) / 2.0
    a = math.radians(alpha_min_deg)
    Mc = R * math.cos(a)
    if Mc < h:
        return 0.0
    dm = (Mc - h) / n
    s = 0.0
    for k in range(n):
        m = h + (k + 0.5) * dm
        s += math.sqrt(R * R - m * m) - m * math.tan(a)
    return 4.0 * s * dm


def sec57():
    print("\n【§5.7】候选区域面积（m²）")
    print(f"{'α_min':>6} | {'R=1000':>12} | {'R=1250':>12} | {'R=1500':>12}")
    for alpha in (30, 45, 60, 75):
        row = [candidate_area(alpha, R) for R in (1000, 1250, 1500)]
        def fmt(v):
            return "0（空）" if v < 1.0 else (f"{v:.3g}" if v < 100 else f"{v:.2e}")
        print(f"{alpha:>6} | {fmt(row[0]):>12} | {fmt(row[1]):>12} | {fmt(row[2]):>12}")

    for alpha, R in ((30, 1500), (45, 1500)):
        A, An = candidate_area(alpha, R), _numeric_area_check(alpha, R)
        print(f"  核对 α_min={alpha}°,R={R}：闭式 {A:.0f} vs 数值 {An:.0f}"
              f"（相对差 {abs(A - An) / An:.2e}）")
        assert abs(A - An) / An < 1e-3

    target = {(30, 1000): 5.8e4, (30, 1250): 4.2e5, (30, 1500): 1.07e6,
              (45, 1250): 7.1e4, (45, 1500): 3.6e5}
    for (alpha, R), tgt in target.items():
        A = candidate_area(alpha, R)
        assert abs(A - tgt) / tgt < 0.02, (alpha, R, A, tgt)
    assert candidate_area(45, 1000) == 0.0
    assert candidate_area(60, 1000) == 0.0 and candidate_area(60, 1250) == 0.0


# ---------------- §7 数值示例 ----------------

def sec7():
    print("\n【§7】数值示例 S1=(0,0), θ1=0°, α_min=45°, R=1500")
    h = (G_MAX - G_MIN) / 2.0
    left_x = G_MAX - G_MAX / math.sqrt(2)
    right_x = G_MIN + G_MAX / math.sqrt(2)
    side_y = G_MAX / math.sqrt(2)
    top_y = math.sqrt(1500 ** 2 - h ** 2)
    print(f"  候选区域关键顶点（上半）：({left_x:.1f},{side_y:.1f})、"
          f"({752.5},{top_y:.1f})、({right_x:.1f},{side_y:.1f})、({752.5},{h})")

    x, y = 752.5, h
    dmax = max(math.hypot(x - G_MIN, y), math.hypot(x - G_MAX, y))
    dmin = abs(y)
    assert dmax < 1500 and dmin > 5, (dmax, dmin)
    assert min_sin_alpha(x, y) >= math.sin(math.radians(45.0)) - 1e-9
    worst = worst_case_diameter_exact(x, y, step=0.5)
    print(f"  推荐点 (752.5, 747.5)：最大距离 {dmax:.1f}<1500 ✓，最小距离 {dmin:.1f}>5 ✓，"
          f"最小 sinα={min_sin_alpha(x, y):.4f}≥sin45° ✓")
    print(f"  推荐点最坏精确直径 ≈ {worst:.1f} m")


def main():
    sec32()
    sec41()
    sec42()
    sec57()
    sec7()
    print("\n问题2 全部数值自测通过。")


if __name__ == "__main__":
    main()
