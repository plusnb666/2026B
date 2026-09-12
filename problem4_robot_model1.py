# -*- coding: utf-8 -*-
"""
问题4：全向与定向混合干扰源的检测与清除策略。

设计（依据第四问建模文档，并在审阅后修正）：
- 阶段1 全局粗扫：13 个检测点（中心 + 内圈6点×600m + 外圈6点×1200m 错开30°），
  每个点扫描全部未清除频道，记录"有信号/无信号"模式与示向度；
  布局保证对全向源（R≥1000）全覆盖；对定向源任意 180° 扇形
  与附近检测点方位集合几乎必然相交。
- 阶段2 类型识别：接收半径内同时出现"有信号+无信号"的频道判定为定向，
  并用滑动 180° 窗口估计定向方向；否则按全向处理。
- 阶段3 补漏定位：仅 1 次观测的频道做横向探针第二测点（沿示向度前进 600m +
  垂直偏移 300m；该点相对源的方位与已知信号点几乎一致，天然落在定向覆盖区内）。
- 阶段4 定位清除：两示向度交点定位（反向射线/平行退化时取中点，全场内限幅）；
  清除失败后原地重新测量 → 横向探针重定位（最多3轮）→ ±15m 网格兜底；
  原地测量落在定向盲区时，改用定向方向 θ_d 引导探针；全部失败如实标记，
  不谎报。
- 清除只与距离有关（≤20m），与覆盖角度无关，流程与问题3相同。

运行：模拟器问题4演练测试接口就绪后
    python robot_model4.py
日志：robot_model4_logs/robot_model4_log{N}.txt（编号，不覆盖）。
"""
import json
import math
import os
import time
import urllib.request

# ==============================
# 配置
# ==============================
BASE_URL = "http://127.0.0.1:2026"
ROBOT_ID = "202610061109"
ARENA_RADIUS = 1800.0
R_MIN = 1000.0        # 有效接收半径下限（类型判据用）
SPEED = 5.0
DETECT_TIME = 5.0
SWITCH_TIME = 1.0
CLEAR_RADIUS = 20.0
NEAR_DIST = 5.0
MAX_FIX_ROUNDS = 3
PROBE_AHEAD = 600.0   # 横向探针：沿示向度前进距离
PROBE_SIDE = 300.0    # 横向探针：垂直偏移
DIR_PROBE_AHEAD = 400.0  # 定向引导探针：沿 θ_d 前进距离
DIR_PROBE_SIDE = 300.0   # 定向引导探针：垂直偏移
CHANNELS = tuple(range(1, 21))

# ==============================
# 状态常量
# ==============================
UNKNOWN = 0       # 尚未发现该频道有信号
LOCATING = 1      # 已有1个示向度
LOCATED = 2       # 已定位（至少2个示向度或near）
CLEARED = 3       # 已清除
CLEAR_FAILED = 4  # 已定位但清除失败（如实标记，不谎报）


# ==============================
# 几何工具
# ==============================
def locate_from_two_points(p1, theta1, p2, theta2):
    """根据两个检测点位置和示向度计算交点坐标。
    两射线平行、或交点落在某条射线的反方向上（源在两测点之间）时，
    返回两测点连线中点作为估计。"""
    x1, y1 = p1
    x2, y2 = p2
    a1 = math.radians(theta1)
    a2 = math.radians(theta2)
    v1x, v1y = math.cos(a1), math.sin(a1)
    v2x, v2y = math.cos(a2), math.sin(a2)

    # 解方程: p1 + t*v1 = p2 + s*v2
    dx = x2 - x1
    dy = y2 - y1
    det = -v1x * v2y + v1y * v2x
    if abs(det) < 1e-9:
        # 平行，返回中点
        return ((x1 + x2) / 2, (y1 + y2) / 2)

    t = (dx * (-v2y) - dy * (-v2x)) / det
    s = (v1x * dy - v1y * dx) / det
    if t < 0 or s < 0:
        # 交点在某条射线的反方向上（源在两测点之间），返回中点
        return ((x1 + x2) / 2, (y1 + y2) / 2)
    return (x1 + v1x * t, y1 + v1y * t)


def clamp_to_arena(pos, limit=1790.0):
    """把位置限幅到场内（距原点≤limit）"""
    x, y = pos
    r = math.hypot(x, y)
    if r <= limit:
        return pos
    scale = limit / r
    return (x * scale, y * scale)


class ChannelInfo:
    def __init__(self, ch):
        self.ch = ch
        self.state = UNKNOWN
        self.bearings = []       # [(测点位置, 示向度)]，仅 direction 观测
        self.patterns = []       # [(测点位置, has_signal)]，全部检测记录
        self.estimated_pos = None
        self.source_type = "unknown"   # "omni" / "directional" / "unknown"
        self.direction_deg = None      # 定向方向 θ_d（判定向后估计）


# ==============================
# HTTP 客户端
# ==============================
class RobotClient:
    def __init__(self, robot_id=ROBOT_ID, base_url=BASE_URL):
        self.robot_id = robot_id
        self.base_url = base_url
        self.current_pos = (0.0, 0.0)
        self.current_channel = 1
        self.virtual_time = 0.0
        self.measure_count = 0
        self.clear_count = 0
        self.req_counter = 0

    def _post(self, path, payload, retries=3):
        data = json.dumps(payload).encode('utf-8')
        for attempt in range(retries):
            req = urllib.request.Request(
                self.base_url + path,
                data=data,
                headers={'Content-Type': 'application/json'},
                method='POST'
            )
            try:
                with urllib.request.urlopen(req, timeout=5) as resp:
                    return json.loads(resp.read().decode('utf-8'))
            except Exception as e:
                if attempt == retries - 1:
                    print(f'[HTTP] {path} 失败: {e}')
                    return None
                time.sleep(0.5 * (attempt + 1))
        return None

    def _base_payload(self):
        self.req_counter += 1
        return {
            'arena_id': 'default',
            'robot_id': self.robot_id,
            'request_id': f'req-{self.req_counter}'
        }

    def enter(self):
        # 倒计时未结束时接口会拒绝或直接重置连接，轮询直至成功
        while True:
            resp = self._post('/enter', self._base_payload())
            if resp and resp.get('accepted'):
                self.virtual_time = resp['virtual_time_s']
                self.current_pos = (0.0, 0.0)
                return resp
            time.sleep(1.0)

    def measure(self, position, channel):
        payload = self._base_payload()
        payload['position'] = {'x': position[0], 'y': position[1]}
        payload['channel'] = channel
        resp = self._post('/measure', payload)
        if resp and resp.get('accepted'):
            self.virtual_time = resp['virtual_time_s']
            self.current_pos = position
            self.current_channel = channel
            self.measure_count += 1
        return resp

    def clear(self, position, channel):
        payload = self._base_payload()
        payload['position'] = {'x': position[0], 'y': position[1]}
        payload['channel'] = channel
        resp = self._post('/clear', payload)
        if resp and resp.get('accepted'):
            self.virtual_time = resp['virtual_time_s']
            self.current_pos = position
            self.clear_count += 1
        return resp

    def exit(self):
        return self._post('/exit', self._base_payload())


# ==============================
# 主策略
# ==============================
class RobotStrategy4:
    def __init__(self, client, log_to_file=True):
        self.client = client
        self.log_to_file = log_to_file
        self.infos = [ChannelInfo(ch) for ch in CHANNELS]
        self.log = []
        self.real_start = 0.0
        self.real_end = 0.0

    # ---------- 日志 ----------
    def log_action(self, action, detail=""):
        entry = f"[t={self.client.virtual_time:.1f}s] {action} {detail}"
        self.log.append(entry)
        print(entry)

    # ---------- 检测点布局 ----------
    def make_detection_points(self):
        """中心点 + 内圈6点（600m，间隔60°）+ 外圈6点（1200m，错开30°），共13点"""
        pts = [(0.0, 0.0)]
        for i in range(6):
            ang = math.radians(i * 60)
            pts.append((600.0 * math.cos(ang), 600.0 * math.sin(ang)))
        for i in range(6):
            ang = math.radians(i * 60 + 30)
            pts.append((1200.0 * math.cos(ang), 1200.0 * math.sin(ang)))
        return pts

    # ---------- 类型识别与定向方向估计 ----------
    def _approx_source_pos(self, info):
        """类型识别用的近似源位置：优先定位估计，否则用最近信号点的示向度射线"""
        if info.estimated_pos is not None:
            return info.estimated_pos
        # 用第一个有示向度的信号测点，沿示向度前进 500m
        for pos, theta in info.bearings:
            rad = math.radians(theta)
            return (pos[0] + 500.0 * math.cos(rad),
                    pos[1] + 500.0 * math.sin(rad))
        # 无示向度（near 定位）：用信号测点本身
        for pos, has in info.patterns:
            if has:
                return pos
        return None

    def _classify_and_estimate_direction(self, info):
        """按文档 §2.1 判据判断类型；判定向后用滑动 180° 窗口估计 θ_d。
        判据：接收半径内同时存在有信号与无信号点 → 定向（全向不可能）。"""
        sig = [pos for pos, has in info.patterns if has]
        nosig = [pos for pos, has in info.patterns if not has]
        if not sig or not nosig:
            info.source_type = "omni" if sig else "unknown"
            return
        g = self._approx_source_pos(info)
        if g is None:
            return
        # 无信号点中，是否存在距近似源位置 < R_MIN-50 的（接收半径内却无信号）
        close_nosig = [pos for pos in nosig
                       if math.hypot(pos[0] - g[0], pos[1] - g[1]) < R_MIN - 50.0]
        if not close_nosig:
            # 无信号点可能都是超距，不能作为定向证据，按全向处理
            info.source_type = "omni"
            return
        info.source_type = "directional"
        info.direction_deg = self._estimate_direction(g, sig, close_nosig)

    def _estimate_direction(self, g, sig_pos, nosig_pos):
        """滑动 180° 窗口：找包含所有信号方位、排除所有无信号方位的窗口，
        窗口中心即 θ_d；无精确解时取包含信号最多、无信号最少的窗口。"""
        def az(pos):
            return math.degrees(math.atan2(pos[1] - g[1], pos[0] - g[0])) % 360.0
        phi_sig = sorted(az(p) for p in sig_pos)
        phi_nosig = sorted(az(p) for p in nosig_pos)

        def inside(phi, alpha):
            return (phi - alpha) % 360.0 < 180.0

        best = None
        for alpha in range(0, 360, 2):
            if all(inside(p, alpha) for p in phi_sig) and \
               not any(inside(p, alpha) for p in phi_nosig):
                return (alpha + 90.0) % 360.0
            score = sum(1 for p in phi_sig if inside(p, alpha)) - \
                sum(1 for p in phi_nosig if inside(p, alpha))
            if best is None or score > best[1]:
                best = ((alpha + 90.0) % 360.0, score)
        return best[0] if best is not None else 0.0

    # ---------- 阶段1：扫描 ----------
    def scan_channel(self, pos, ch):
        info = self.infos[ch - 1]
        res = self.client.measure(pos, ch)
        if not res or not res.get('accepted'):
            return
        result = res.get('measure_result')
        if result == 'direction':
            theta = res['svd_deg']
            info.bearings.append((pos, theta))
            info.patterns.append((pos, True))
            self.log_action(f"  频道{ch}:",
                            f"示向度={theta:.2f}° (已观测{len(info.bearings)}次) ")
            if info.state == UNKNOWN:
                info.state = LOCATING
            elif info.state == LOCATING:
                # 取最近两个示向度定位
                p1, t1 = info.bearings[-2]
                p2, t2 = info.bearings[-1]
                info.estimated_pos = clamp_to_arena(
                    locate_from_two_points(p1, t1, p2, t2))
                info.state = LOCATED
                self.log_action(f"  频道{ch}:",
                                f"已定位，交点估计位置=({info.estimated_pos[0]:.0f},{info.estimated_pos[1]:.0f}) ")
        elif result == 'near':
            info.bearings.append((pos, None))
            info.patterns.append((pos, True))
            info.estimated_pos = pos
            info.state = LOCATED
            self.log_action(f"  频道{ch}:", "距离过近(<5m)，定位到当前位置 ")
        else:
            # no_signal：不参与定位，但记录模式用于类型判断
            info.patterns.append((pos, False))

    def scan_at_point(self, pos):
        """在当前检测点扫描所有未清除频道，记录信号模式"""
        chs = [info.ch for info in self.infos if info.state in (UNKNOWN, LOCATING)]
        if not chs:
            return
        chs.sort()
        self.log_action("SCAN", f"移动到({pos[0]:.0f},{pos[1]:.0f})，扫描 {len(chs)} 个频道")
        for ch in chs:
            self.scan_channel(pos, ch)

    # ---------- 阶段2：类型识别 ----------
    def classify_all(self):
        located_or_locating = [info for info in self.infos
                               if info.state in (LOCATING, LOCATED)]
        self.log_action("开始类型识别阶段",
                        f"待判频道: {sorted(i.ch for i in located_or_locating)}")
        for info in located_or_locating:
            self._classify_and_estimate_direction(info)
            if info.source_type == "directional":
                self.log_action(f"  频道{info.ch}:",
                                f"判定为定向，定向方向估计={info.direction_deg:.1f}° ")

    # ---------- 阶段3：补漏定位 ----------
    def second_measure(self, info):
        """对只有1个示向度的频道做第二测点（横向探针），完成交点定位。
        探针沿示向度前进，相对源的方位与已知信号点几乎一致，
        天然落在定向覆盖区内。返回 True 表示已定位。"""
        ch = info.ch
        pos1, theta1 = info.bearings[0]
        rad1 = math.radians(theta1)
        mid = (pos1[0] + PROBE_AHEAD * math.cos(rad1),
               pos1[1] + PROBE_AHEAD * math.sin(rad1))
        s2 = clamp_to_arena(
            (mid[0] - PROBE_SIDE * math.sin(rad1),
             mid[1] + PROBE_SIDE * math.cos(rad1)))
        self.log_action(f"二次定位频道{ch}:",
                        f"移动到({s2[0]:.0f},{s2[1]:.0f})并检测 ")
        res = self.client.measure(s2, ch)
        if res and res.get('accepted') and res.get('measure_result') == 'direction':
            theta2 = res['svd_deg']
            info.bearings.append((s2, theta2))
            info.patterns.append((s2, True))
            self.log_action(f"  频道{ch}:",
                            f"示向度={theta2:.2f}° (已观测{len(info.bearings)}次) ")
            p1, t1 = info.bearings[-2]
            p2, t2 = info.bearings[-1]
            info.estimated_pos = clamp_to_arena(
                locate_from_two_points(p1, t1, p2, t2))
            info.state = LOCATED
            self.log_action(f"  频道{ch}:",
                            f"已定位，交点估计位置=({info.estimated_pos[0]:.0f},{info.estimated_pos[1]:.0f}) ")
            return True
        if res and res.get('accepted') and res.get('measure_result') == 'near':
            info.bearings.append((s2, None))
            info.patterns.append((s2, True))
            info.estimated_pos = s2
            info.state = LOCATED
            self.log_action(f"  频道{ch}:", "距离过近(<5m)，定位到当前位置 ")
            return True
        info.patterns.append((s2, False))

        # no_signal：换对称偏移点再测一次
        s3 = clamp_to_arena(
            (mid[0] + PROBE_SIDE * math.sin(rad1),
             mid[1] - PROBE_SIDE * math.cos(rad1)))
        self.log_action(f"  频道{ch}:",
                        f"第二测点无信号，换对称点({s3[0]:.0f},{s3[1]:.0f})再测 ")
        res = self.client.measure(s3, ch)
        if res and res.get('accepted') and res.get('measure_result') == 'direction':
            theta2 = res['svd_deg']
            info.bearings.append((s3, theta2))
            info.patterns.append((s3, True))
            self.log_action(f"  频道{ch}:",
                            f"示向度={theta2:.2f}° (已观测{len(info.bearings)}次) ")
            p1, t1 = info.bearings[-2]
            p2, t2 = info.bearings[-1]
            info.estimated_pos = clamp_to_arena(
                locate_from_two_points(p1, t1, p2, t2))
            info.state = LOCATED
            self.log_action(f"  频道{ch}:",
                            f"已定位，交点估计位置=({info.estimated_pos[0]:.0f},{info.estimated_pos[1]:.0f}) ")
            return True
        if res and res.get('accepted') and res.get('measure_result') == 'near':
            info.bearings.append((s3, None))
            info.patterns.append((s3, True))
            info.estimated_pos = s3
            info.state = LOCATED
            self.log_action(f"  频道{ch}:", "距离过近(<5m)，定位到当前位置 ")
            return True
        info.patterns.append((s3, False))

        # 仍无信号：若已判定向，沿 θ_d 方向在覆盖区内再选一个探针点
        self._classify_and_estimate_direction(info)
        if info.source_type == "directional" and info.direction_deg is not None:
            rad_d = math.radians(info.direction_deg)
            g = self._approx_source_pos(info)
            s4 = clamp_to_arena(
                (g[0] + DIR_PROBE_AHEAD * math.cos(rad_d)
                 - DIR_PROBE_SIDE * math.sin(rad_d),
                 g[1] + DIR_PROBE_AHEAD * math.sin(rad_d)
                 + DIR_PROBE_SIDE * math.cos(rad_d)))
            self.log_action(f"  频道{ch}:",
                            f"定向引导探针移动到({s4[0]:.0f},{s4[1]:.0f})并检测 ")
            res = self.client.measure(s4, ch)
            if res and res.get('accepted') and res.get('measure_result') == 'direction':
                theta2 = res['svd_deg']
                info.bearings.append((s4, theta2))
                info.patterns.append((s4, True))
                self.log_action(f"  频道{ch}:",
                                f"示向度={theta2:.2f}° (已观测{len(info.bearings)}次) ")
                p1, t1 = info.bearings[-2]
                p2, t2 = info.bearings[-1]
                info.estimated_pos = clamp_to_arena(
                    locate_from_two_points(p1, t1, p2, t2))
                info.state = LOCATED
                self.log_action(f"  频道{ch}:",
                                f"已定位，交点估计位置=({info.estimated_pos[0]:.0f},{info.estimated_pos[1]:.0f}) ")
                return True
            if res and res.get('accepted') and res.get('measure_result') == 'near':
                info.bearings.append((s4, None))
                info.patterns.append((s4, True))
                info.estimated_pos = s4
                info.state = LOCATED
                self.log_action(f"  频道{ch}:", "距离过近(<5m)，定位到当前位置 ")
                return True
            info.patterns.append((s4, False))
        self.log_action(f"  频道{ch}:", "补漏定位失败 ")
        return False

    # ---------- 阶段4：定位清除 ----------
    def _cleared_count(self):
        return sum(1 for info in self.infos if info.state == CLEARED)

    def clear_one(self, info):
        """清除单个干扰源：清除失败后原地重新测量 → 横向探针重定位（最多3轮）；
        原地测量落在定向盲区时，用 θ_d 引导探针；最后 ±15m 网格兜底；
        全部失败如实标记，不谎报。"""
        ch = info.ch
        pos = info.estimated_pos
        self._classify_and_estimate_direction(info)

        self.log_action(f"清除频道{ch}",
                        f"移动到估计位置({pos[0]:.0f},{pos[1]:.0f})并清除 ")

        for _ in range(MAX_FIX_ROUNDS):
            res = self.client.clear(pos, ch)
            if not res or not res.get('accepted'):
                return False
            if res.get('clear_result') == 'success':
                info.state = CLEARED
                self.log_action(f"  清除频道{ch}",
                                f"成功！(累计{self._cleared_count()}个) ")
                return True

            self.log_action(f"  清除频道{ch}", "未发现目标(20m内无干扰源) ")

            # 失败：原地重新测量
            res = self.client.measure(pos, ch)
            if res and res.get('accepted') and res.get('measure_result') == 'near':
                self.log_action(f"  清除频道{ch}", "距离过近，原地重试清除 ")
                continue
            if res and res.get('accepted') and res.get('measure_result') == 'direction':
                theta = res['svd_deg']
                info.bearings.append((pos, theta))
                info.patterns.append((pos, True))
                self.log_action(f"  修正频道{ch}:",
                                f"原地示向度={theta:.2f}° (已观测{len(info.bearings)}次) ")
                # 横向探针：沿示向度前进600m，垂直偏移300m 做第二测点
                rad = math.radians(theta)
                probe = clamp_to_arena(
                    (pos[0] + PROBE_AHEAD * math.cos(rad)
                     - PROBE_SIDE * math.sin(rad),
                     pos[1] + PROBE_AHEAD * math.sin(rad)
                     + PROBE_SIDE * math.cos(rad)))
                self.log_action(f"  修正频道{ch}:",
                                f"横向探针移动到({probe[0]:.0f},{probe[1]:.0f})并检测 ")
                res = self.client.measure(probe, ch)
                if res and res.get('accepted') and res.get('measure_result') == 'near':
                    info.bearings.append((probe, None))
                    info.patterns.append((probe, True))
                    pos = probe
                    continue
                if res and res.get('accepted') and res.get('measure_result') == 'direction':
                    theta2 = res['svd_deg']
                    info.bearings.append((probe, theta2))
                    info.patterns.append((probe, True))
                    # 用原地示向度与探针示向度做交点，重新定位
                    new_pos = clamp_to_arena(
                        locate_from_two_points(pos, theta, probe, theta2))
                    pos = new_pos
                    info.estimated_pos = new_pos
                    self.log_action(f"  修正频道{ch}:",
                                    f"新估计位置=({new_pos[0]:.0f},{new_pos[1]:.0f})，再次尝试清除 ")
                    continue
                info.patterns.append((probe, False))
                # 探针也无信号：当前位置可能在定向盲区，用 θ_d 引导
                self._classify_and_estimate_direction(info)
                if info.source_type == "directional" and info.direction_deg is not None:
                    rad_d = math.radians(info.direction_deg)
                    probe2 = clamp_to_arena(
                        (pos[0] + DIR_PROBE_AHEAD * math.cos(rad_d)
                         - DIR_PROBE_SIDE * math.sin(rad_d),
                         pos[1] + DIR_PROBE_AHEAD * math.sin(rad_d)
                         + DIR_PROBE_SIDE * math.cos(rad_d)))
                    self.log_action(f"  修正频道{ch}:",
                                    f"定向引导探针移动到({probe2[0]:.0f},{probe2[1]:.0f})并检测 ")
                    res = self.client.measure(probe2, ch)
                    if res and res.get('accepted') and res.get('measure_result') == 'near':
                        info.bearings.append((probe2, None))
                        info.patterns.append((probe2, True))
                        pos = probe2
                        continue
                    if res and res.get('accepted') and res.get('measure_result') == 'direction':
                        theta2 = res['svd_deg']
                        info.bearings.append((probe2, theta2))
                        info.patterns.append((probe2, True))
                        new_pos = clamp_to_arena(
                            locate_from_two_points(pos, theta, probe2, theta2))
                        pos = new_pos
                        info.estimated_pos = new_pos
                        self.log_action(f"  修正频道{ch}:",
                                        f"新估计位置=({new_pos[0]:.0f},{new_pos[1]:.0f})，再次尝试清除 ")
                        continue
                    info.patterns.append((probe2, False))
                break
            # 原地测量无信号：定向盲区引导或跳出
            if res and res.get('accepted'):
                info.patterns.append((pos, False))
            self._classify_and_estimate_direction(info)
            if info.source_type == "directional" and info.direction_deg is not None:
                rad_d = math.radians(info.direction_deg)
                probe = clamp_to_arena(
                    (pos[0] + DIR_PROBE_AHEAD * math.cos(rad_d)
                     - DIR_PROBE_SIDE * math.sin(rad_d),
                     pos[1] + DIR_PROBE_AHEAD * math.sin(rad_d)
                     + DIR_PROBE_SIDE * math.cos(rad_d)))
                self.log_action(f"  修正频道{ch}:",
                                f"原地无信号，定向引导探针移动到({probe[0]:.0f},{probe[1]:.0f})并检测 ")
                res = self.client.measure(probe, ch)
                if res and res.get('accepted') and res.get('measure_result') == 'near':
                    info.bearings.append((probe, None))
                    info.patterns.append((probe, True))
                    pos = probe
                    continue
                if res and res.get('accepted') and res.get('measure_result') == 'direction':
                    theta2 = res['svd_deg']
                    info.bearings.append((probe, theta2))
                    info.patterns.append((probe, True))
                    # 与最近一个有效示向度做交点
                    valid = [(p, t) for p, t in info.bearings
                             if t is not None and p != probe]
                    if valid:
                        p0, t0 = valid[-1]
                        new_pos = clamp_to_arena(
                            locate_from_two_points(p0, t0, probe, theta2))
                        pos = new_pos
                        info.estimated_pos = new_pos
                        self.log_action(f"  修正频道{ch}:",
                                        f"新估计位置=({new_pos[0]:.0f},{new_pos[1]:.0f})，再次尝试清除 ")
                        continue
                if res and res.get('accepted'):
                    info.patterns.append((probe, False))
            break

        # 最后手段：网格搜索（偏移15m，覆盖半径20m）
        offsets = [(15, 0), (-15, 0), (0, 15), (0, -15),
                   (15, 15), (-15, 15), (15, -15), (-15, -15)]
        for dx, dy in offsets:
            new_pos = clamp_to_arena((pos[0] + dx, pos[1] + dy))
            res = self.client.clear(new_pos, ch)
            if res.get('clear_result') == 'success':
                info.state = CLEARED
                self.log_action(f"  清除频道{ch}",
                                f"网格搜索偏移({dx},{dy})清除成功！(累计{self._cleared_count()}个) ")
                return True
            self.log_action(f"  清除频道{ch}",
                            f"网格搜索偏移({dx},{dy})未发现目标 ")

        # 如实标记清除失败，不谎报
        info.state = CLEAR_FAILED
        self.log_action(f"  清除频道{ch}",
                        "重定位与网格搜索均失败，如实标记未清除 ")
        return False

    def clear_all_located(self):
        located = [info for info in self.infos if info.state == LOCATED]
        self.log_action("开始清除阶段",
                        f"已定位频道: {sorted(i.ch for i in located)}")
        current = self.client.current_pos
        remaining = set(located)
        while remaining:
            # 最近邻选择
            next_info = min(remaining,
                            key=lambda info: math.hypot(
                                info.estimated_pos[0] - current[0],
                                info.estimated_pos[1] - current[1]))
            self.clear_one(next_info)
            remaining.remove(next_info)
            current = self.client.current_pos

    # ---------- 主流程 ----------
    def run(self):
        start_real = time.time()
        self.real_start = start_real
        resp = self.client.enter()
        self.log_action("ENTER",
                        f"现实时限={resp.get('remaining_real_duration_s', 1200)}s")

        try:
            # 阶段1：全局粗扫（13点）
            for pt in self.make_detection_points():
                self.scan_at_point(pt)

            # 阶段2：类型识别
            self.classify_all()

            # 阶段3：补漏定位（仅1次观测的频道）
            locating = [info for info in self.infos if info.state == LOCATING]
            self.log_action("开始补漏定位阶段",
                            f"仅1次观测的频道: {sorted(i.ch for i in locating)}")
            for info in locating:
                self.second_measure(info)
            # 补漏后重判类型
            self.classify_all()

            # 阶段4：定位清除
            self.clear_all_located()
        finally:
            self.client.exit()
            self.real_end = time.time()
            self.log_action("EXIT", f"清除频道数={self._cleared_count()}")
        self._print_summary()

    # ---------- 总结与日志 ----------
    def _print_summary(self):
        cleared = sorted(info.ch for info in self.infos if info.state == CLEARED)
        locating = sorted(info.ch for info in self.infos if info.state == LOCATING)
        failed = sorted(info.ch for info in self.infos if info.state == CLEAR_FAILED)
        unknown = sorted(info.ch for info in self.infos if info.state == UNKNOWN)
        directional = sorted(info.ch for info in self.infos
                             if info.source_type == "directional")
        omni = sorted(info.ch for info in self.infos if info.source_type == "omni")
        print("\n" + "=" * 60)
        print("测试总结（问题4：全向与定向混合干扰源策略）")
        print("=" * 60)
        print(f"总虚拟时间: {self.client.virtual_time:.1f} 秒")
        print(f"程序运行时间（现实）: {self.real_end - self.real_start:.1f} 秒")
        print(f"已清除频道: {cleared}")
        print(f"已清除数量: {len(cleared)}")
        print(f"未检测到信号频道: {unknown}")
        print(f"已发现未定位频道: {locating}")
        print(f"清除失败频道（如实未清除）: {failed}")
        print(f"判定为定向的频道: {directional}")
        print(f"判定为全向的频道: {omni}")
        for ch in directional:
            info = self.infos[ch - 1]
            if info.direction_deg is not None:
                print(f"  频道{ch} 定向方向估计: {info.direction_deg:.1f}°")
        if cleared:
            avg = self.client.virtual_time / len(cleared)
            print(f"平均定位清除时间（虚拟总时间/清除数）: {avg:.1f} 秒")
        print("观测统计:")
        for info in self.infos:
            if info.bearings:
                status = "已清除" if info.state == CLEARED else (
                    "清除失败" if info.state == CLEAR_FAILED else "未清除")
                print(f"  频道{info.ch}: {len(info.bearings)}次观测, {status}, "
                      f"类型={info.source_type}")
        print("=" * 60)

        if self.log_to_file:
            # 每次运行新建编号日志（robot_model4_logs/ 文件夹内），不覆盖历史
            base = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "robot_model4_logs")
            os.makedirs(base, exist_ok=True)
            n = 1
            while os.path.exists(os.path.join(base, f"robot_model4_log{n}.txt")):
                n += 1
            log_path = os.path.join(base, f"robot_model4_log{n}.txt")
            with open(log_path, "w", encoding="utf-8") as f:
                f.write("\n".join(self.log))
            print(f"日志已保存: {log_path}")


# ==============================
# 主入口
# ==============================
def main():
    print(f"参赛队号: {ROBOT_ID}")
    print(f"模拟器地址: {BASE_URL}")
    print("等待模拟器接口就绪（倒计时结束后自动开始，无需按键）...")

    client = RobotClient(robot_id=ROBOT_ID)
    strategy = RobotStrategy4(client)
    strategy.run()


if __name__ == '__main__':
    main()
