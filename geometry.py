# -*- coding: utf-8 -*-
"""
问题3：几何工具（复用问题1算法）
- 示向度误差扇区 → 半平面
- 半平面交：1800 米圆域内接正 96 边形初始化（与问题1修正一致，不用矩形包围盒）
- 定位区域直径：凸多边形顶点对枚举

说明：本模块是 problem1_algorithm.py（问题1已验证实现）的薄封装，
     接口与 problem3_robot_model1.py 配套；需与 problem3_robot_model1.py、problem1_algorithm.py 同目录。
"""
from problem1_algorithm import (
    sector_to_halfplanes as _sector_to_halfplanes,
    halfplane_intersection as _halfplane_intersection,
    polygon_diameter as _polygon_diameter,
)

DELTA = 1.0   # 示向度误差 ±1°


def sector_to_halfplanes(x, y, theta_deg, delta_deg=DELTA):
    """(x,y) 处示向度 theta_deg 的 ±delta_deg 扇区 → 两个半平面"""
    return _sector_to_halfplanes(x, y, theta_deg, delta_deg)


def intersect_halfplanes(halfplanes):
    """半平面交（增量切割法，1800米圆域内接正96边形初始化），返回凸多边形顶点列表"""
    return _halfplane_intersection(halfplanes)


def polygon_diameter_bruteforce(verts):
    """凸多边形定位区域直径（顶点对枚举），返回 (直径, 端点A, 端点B)"""
    D, (A, B) = _polygon_diameter(verts)
    return (D, A, B)
