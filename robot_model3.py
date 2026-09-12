# -*- coding: utf-8 -*-
"""
问题3 第三套模型实现：基于信息收益的主动搜索与确定性补漏
配套文档：问题3-第三套模型-基于信息收益的主动搜索与确定性补漏.md
依赖：geometry.py、problem1_algorithm.py、robot.py（复用 RobotClient 与几何辅助）
运行：Python robot_model3.py <参赛队号>

与第一套（robot.py）的区别：
- 搜索阶段不用固定航点顺序，而是维护每个频道的候选区域网格 Ω_c，
  按"单位时间空间覆盖收益"价值函数滚动选择下一检测点（文档 §5）；
- 观测更新：no_signal 排除 1000m 圆盘、direction 交楔形、
  near 直接清除、清除失败排除 20m 圆盘（文档 §4，含原稿缺失的负信息利用）；
- 定位/清除/补漏阶段与第一套共用已验证机制（问题1半平面交、600/300
  二次检测点、40m 直径闸门+失败修正、13 航点确定性覆盖，文档 §6/§7）。
"""
import sys
import time
from math import cos, radians, sin

import numpy as np

from robot import (BASE_URL, ROBOT_ID, RobotClient,
                   REAL_EXIT_RESERVE, TIME_RESERVE, REAL_ACTION_MIN,
                   VIRTUAL_TIME_LIMIT, CLEAR_DIAM_GATE, MAX_CLEAR_RETRY,
                   SECOND_STATION_DIST, SECOND_STATION_OFFSET,
                   dist, point_on_ray, perpendicular_offset,
                   generate_search_waypoints, locate_from_observations)

# ============================================================
# 候选区域网格（文档 §3.3：10m 网格离散化 1800m 圆域）
# ============================================================
GRID_STEP = 10.0                  # 网格边长（米）
GRID_HALF = 1800.0                # 圆域半径（米）
_CELL_HALF_DIAG = GRID_STEP * 0.7071   # 格点半对角线 ≈7.07m，作安全裕量

# 排除半径的保守收缩：只排除"整格都在圆盘内"的格点，保证真实源永不被误排除
EXCLUDE_NO_SIGNAL_R = 1000.0 - _CELL_HALF_DIAG   # no_signal 排除半径（≈992.9m）
EXCLUDE_CLEAR_FAIL_R = 20.0 - _CELL_HALF_DIAG    # 清除失败排除半径（≈12.9m）
WEDGE_DELTA = 2.0                 # 网格楔形放宽到 ±2°（格点中心离散误差裕量）

_N = int(2 * GRID_HALF / GRID_STEP) + 1          # 361
_AXIS = np.linspace(-GRID_HALF, GRID_HALF, _N)
_GX, _GY = np.meshgrid(_AXIS, _AXIS)
_DISK = (_GX ** 2 + _GY ** 2) <= GRID_HALF ** 2  # 圆域内格点（初始候选区域）


def _circle_mask(sx, sy, r):
    """以 (sx,sy) 为圆心、r 为半径的圆盘格点掩码"""
    return (_GX - sx) ** 2 + (_GY - sy) ** 2 <= r ** 2


def _wedge_mask(sx, sy, theta_deg, delta_deg):
    """以 (sx,sy) 为顶点、示向度 theta±delta 的扇形格点掩码（含角度回绕）"""
    ang = np.degrees(np.arctan2(_GY - sy, _GX - sx)) % 360.0
    lo, hi = (theta_deg - delta_deg) % 360.0, (theta_deg + delta_deg) % 360.0
    if lo <= hi:
        return (ang >= lo) & (ang <= hi)
    return (ang >= lo) | (ang <= hi)


# ============================================================
# 候选检测点（文档 §5.4：极坐标采样 4 环 × 12 方向 + 原点，共 49 个）
# ============================================================
_CANDIDATES = [(0.0, 0.0)]
for _r in (400, 800, 1200, 1600):
    for _k in range(12):
        _a = radians(30.0 * _k)
        _CANDIDATES.append((_r * cos(_a), _r * sin(_a)))
_CANDIDATE_MASKS = [(sx, sy, _circle_mask(sx, sy, EXCLUDE_NO_SIGNAL_R))
                    for sx, sy in _CANDIDATES]

SCORE_EPS = 0.02    # Score 低于该值视为收益耗尽，转入确定性补漏（文档参数表，可调）


# ============================================================
# 频道候选区域模型（文档 §三/§四）
# ============================================================
class ChannelModel:
    """单个频道的候选区域 Ω_c 与状态"""
    def __init__(self):
        self.grid = _DISK.copy()   # True=干扰源可能出现位置
        self.observations = []     # [(pos, theta_deg), ...]
        self.status = "searching"  # searching / localized / cleared / empty
        self.estimate = None       # (ex, ey, diameter)
        self.clear_attempts = 0
        self.obs_count_at_attempt = 0

    @property
    def area(self):
        return int(self.grid.sum())

    def apply_direction(self, sx, sy, theta):
        """direction：候选区域与楔形相交（文档 §4.1，网格放宽到 ±2°）"""
        self.grid &= _wedge_mask(sx, sy, theta, WEDGE_DELTA)
        self.observations.append(((sx, sy), theta))
        self.status = "localized"

    def apply_no_signal(self, sx, sy):
        """no_signal：排除 1000m 圆盘（文档 §4.2，保守收缩保证安全）"""
        self.grid &= ~_circle_mask(sx, sy, EXCLUDE_NO_SIGNAL_R)

    def apply_clear_fail(self, px, py):
        """清除失败：排除 20m 圆盘（文档 §4.4 负信息利用）"""
        self.grid &= ~_circle_mask(px, py, EXCLUDE_CLEAR_FAIL_R)

    def locate(self):
        """交会定位（文档 §6.1：问题1 半平面交精确几何，不用网格）"""
        if len(self.observations) < 2:
            return None
        result = locate_from_observations(self.observations)
        if result is None:
            return None
        self.estimate = result[:3]
        return self.estimate


# ============================================================
# 主策略
# ============================================================
class RobotStrategy3:
    def __init__(self, client, log_to_file=True):
        self.client = client
        self.log_to_file = log_to_file   # False：离线测试等不落盘日志
        self.pos = (0.0, 0.0)
        self.channel = 1
        self.virtual_time = 0.0
        self.real_deadline_ts = 0.0
        self.real_start = 0.0
        self.real_end = 0.0

        self.cleared_channels = set()
        self.channels = {c: ChannelModel() for c in range(1, 21)}
        self.visited_waypoints = set()
        self.log = []

    def log_action(self, action, detail=""):
        entry = f"[t={self.virtual_time:.1f}s] {action} {detail}"
        self.log.append(entry)
        print(entry)

    def remaining_real(self):
        return self.real_deadline_ts - time.monotonic()

    def run(self, entered_resp=None):
        resp = entered_resp if entered_resp is not None else self.client.enter()
        if not resp or resp.get("accepted") is not True:
            print("进入失败！")
            return
        self.real_deadline_ts = time.monotonic() + float(
            resp.get("remaining_real_duration_s", 1200))
        self.real_start = time.monotonic()
        self.virtual_time = resp.get("virtual_time_s", 0)
        self.log_action("ENTER", f"现实时限={resp.get('remaining_real_duration_s')}s")

        try:
            self._main_loop()
        except Exception as e:
            self.log_action("ERROR", str(e))
            import traceback
            traceback.print_exc()
        finally:
            self.real_end = time.monotonic()
            self.client.exit()
            self.log_action("EXIT", f"清除频道数={len(self.cleared_channels)}")
            self._print_summary()

    def _main_loop(self):
        iteration = 0
        while (self.remaining_real() > REAL_EXIT_RESERVE
               and self.virtual_time < VIRTUAL_TIME_LIMIT):
            iteration += 1
            self.log_action(f"--- 迭代 {iteration} ---",
                            f"位置=({self.pos[0]:.0f},{self.pos[1]:.0f}), "
                            f"已清除={len(self.cleared_channels)}, "
                            f"现实剩余={self.remaining_real():.0f}s")

            # 步骤1：优先处理已定位频道（定位-清除闭环，文档 §6）
            if self._process_localized():
                continue

            # 步骤2：主动搜索——按价值函数选择下一检测点（文档 §5）
            best = self._best_candidate()
            if best is not None:
                (sx, sy), chs, score = best
                self._scan_channels(sx, sy, chs,
                                    f"移动到({sx:.0f},{sy:.0f})，"
                                    f"信息收益优先扫描 {len(chs)} 个频道")
            else:
                # 步骤3：确定性补漏（文档 §7）
                if self.remaining_real() < TIME_RESERVE:
                    self.log_action("现实时间不足，停止搜索")
                    break
                if not self._coverage_step():
                    self.log_action("所有航点已访问，停止")
                    break

            # 已清除 16 个（总数上限），其余频道必无源，提前结束
            if len(self.cleared_channels) >= 16:
                self.log_action("已清除16个（最大可能），其余频道判定为无源")
                for c, m in self.channels.items():
                    if m.status != "cleared":
                        m.status = "empty"
                break

    # ---------------- 定位与清除（文档 §6） ----------------
    def _process_localized(self):
        """处理已定位频道。返回 True 表示本迭代已执行动作。"""
        if self.remaining_real() < REAL_EXIT_RESERVE + REAL_ACTION_MIN:
            return False
        located, single = [], []
        for c, m in self.channels.items():
            if m.status != "localized":
                continue
            # 放弃后若在航点扫描中获得新观测，重新给予清除机会（文档 §6.4）
            if m.clear_attempts >= MAX_CLEAR_RETRY:
                if len(m.observations) > m.obs_count_at_attempt:
                    m.clear_attempts = 0
                else:
                    continue
            if m.estimate is not None:
                located.append((dist(self.pos, m.estimate[:2]), c))
            elif m.observations:
                (ox, oy), th = m.observations[-1]
                est = point_on_ray((ox, oy), th, 800)
                single.append((dist(self.pos, est), c))

        if located:
            located.sort()
            return self._approach_and_clear(located[0][1])
        if single and self.remaining_real() > TIME_RESERVE:
            single.sort()
            return self._second_station(single[0][1])
        return False

    def _approach_and_clear(self, ch):
        """已定位频道：闸门判断 → 横向补测（必要时）→ /clear（文档 §6.3）"""
        m = self.channels[ch]
        ex, ey, diam = m.estimate
        self.log_action(f"逼近频道{ch}: 估计位置=({ex:.0f},{ey:.0f}), "
                        f"定位直径={diam:.1f}m")

        if diam > CLEAR_DIAM_GATE:
            # 定位区域过大：在最近观测站示向度方向横向取点补测。
            # 沿示向线取点与既有观测共线、交角≈0°无信息量；
            # 横向取点保证交角大且到源≤957m必能收到信号（文档 §6.2）。
            (ox, oy), oth = min(m.observations,
                                key=lambda o: dist(o[0], (ex, ey)))
            mid = point_on_ray((ox, oy), oth, SECOND_STATION_DIST)
            probe = perpendicular_offset(mid, oth, SECOND_STATION_OFFSET)
            self.log_action(f"  定位直径{diam:.1f}m>{CLEAR_DIAM_GATE:.0f}m，"
                            f"横向补测点=({probe[0]:.0f},{probe[1]:.0f})")
            resp = self.client.measure(probe[0], probe[1], ch)
            if resp and resp.get("accepted") is True:
                self.virtual_time = resp.get("virtual_time_s", self.virtual_time)
                self.pos = probe
                self.channel = ch
                result = resp.get("measure_result")
                if result == "direction":
                    m.apply_direction(probe[0], probe[1], resp.get("svd_deg"))
                    m.locate()
                    if m.estimate is not None:
                        ex, ey, diam = m.estimate
                elif result == "near":
                    return self._move_and_clear(ch, self.pos)
                elif result == "no_signal":
                    m.apply_no_signal(probe[0], probe[1])
        return self._move_and_clear(ch, (ex, ey))

    def _second_station(self, ch):
        """单次观测频道：沿示向度 600m + 垂直偏移 300m 二次检测（文档 §6.2）"""
        m = self.channels[ch]
        pos1, theta1 = m.observations[-1]
        mid = point_on_ray(pos1, theta1, SECOND_STATION_DIST)
        s2 = perpendicular_offset(mid, theta1, SECOND_STATION_OFFSET)
        self.log_action(f"二次定位频道{ch}: 移动到({s2[0]:.0f},{s2[1]:.0f})并检测")

        resp = self.client.measure(s2[0], s2[1], ch)
        if resp and resp.get("accepted") is True:
            self.virtual_time = resp.get("virtual_time_s", self.virtual_time)
            self.pos = (s2[0], s2[1])
            self.channel = ch
            result = resp.get("measure_result")
            if result == "direction":
                theta2 = resp.get("svd_deg")
                m.apply_direction(s2[0], s2[1], theta2)
                self.log_action(f"  二次示向度={theta2:.2f}°")
                if m.locate() is not None:
                    ex, ey, _ = m.estimate
                    return self._move_and_clear(ch, (ex, ey))
                if len(m.observations) >= 4:
                    self.log_action(f"  多次定位失败，放弃频道{ch}")
                    m.observations = []
                    m.status = "searching"
            elif result == "near":
                self.log_action("  距离过近，直接清除")
                return self._move_and_clear(ch, self.pos)
            else:
                # no_signal：设计保证二次检测必能收到信号（≤957m<1000m），
                # 出现说明该源已被清除或数据异常：放弃该频道防止死循环
                self.log_action(f"  二次检测无信号，放弃频道{ch}")
                m.observations = []
                m.status = "searching"
        return True

    def _move_and_clear(self, ch, target):
        """移动到目标位置并尝试清除（/clear 一次指令完成移动+清除，文档 §6.3）"""
        m = self.channels[ch]
        m.clear_attempts += 1
        m.obs_count_at_attempt = len(m.observations)
        resp = self.client.clear(target[0], target[1], ch)
        if not resp or resp.get("accepted") is not True:
            self.log_action(f"  移动清除频道{ch} 失败（请求未接受）")
            return False

        self.virtual_time = resp.get("virtual_time_s", self.virtual_time)
        self.pos = (target[0], target[1])
        result = resp.get("clear_result")
        if result == "success":
            self.cleared_channels.add(ch)
            m.status = "cleared"
            m.grid.fill(False)
            self.log_action(f"  清除频道{ch} 成功！(累计{len(self.cleared_channels)}个)")
            return True

        self.log_action(f"  清除频道{ch} 未发现目标(20m内无干扰源)")
        m.apply_clear_fail(target[0], target[1])   # §4.4 负信息：排除 20m 圆盘
        if m.clear_attempts < MAX_CLEAR_RETRY:
            # 原地补测修正位置后立即再次清除（文档 §6.4）
            return self._refine_position(ch)
        return False

    def _do_clear(self, ch):
        """当前位置直接清除（near 场景：距干扰源≤5m<20m，必成功，文档 §4.3）"""
        m = self.channels[ch]
        m.clear_attempts += 1
        m.obs_count_at_attempt = len(m.observations)
        resp = self.client.clear(self.pos[0], self.pos[1], ch)
        if not resp or resp.get("accepted") is not True:
            self.log_action(f"  清除频道{ch} 失败（请求未接受）")
            return False
        self.virtual_time = resp.get("virtual_time_s", self.virtual_time)
        if resp.get("clear_result") == "success":
            self.cleared_channels.add(ch)
            m.status = "cleared"
            m.grid.fill(False)
            self.log_action(f"  清除频道{ch} 成功！(累计{len(self.cleared_channels)}个)")
            return True
        self.log_action(f"  清除频道{ch} 未发现目标(20m内无干扰源)")
        m.apply_clear_fail(self.pos[0], self.pos[1])
        if m.clear_attempts < MAX_CLEAR_RETRY:
            return self._refine_position(ch)
        return False

    def _refine_position(self, ch):
        """清除失败修正：原地补测 → 重新定位 → 立即再次清除（文档 §6.4）"""
        m = self.channels[ch]
        self.log_action(f"  修正频道{ch} 位置...")
        resp = self.client.measure(self.pos[0], self.pos[1], ch)
        if resp and resp.get("accepted") is True:
            self.virtual_time = resp.get("virtual_time_s", self.virtual_time)
            self.channel = ch
            result = resp.get("measure_result")
            if result == "direction":
                m.apply_direction(self.pos[0], self.pos[1], resp.get("svd_deg"))
                if m.locate() is not None:
                    ex, ey, _ = m.estimate
                    self.log_action(f"  新估计位置=({ex:.0f},{ey:.0f})，再次尝试清除")
                    return self._move_and_clear(ch, (ex, ey))
            elif result == "near":
                return self._move_and_clear(ch, self.pos)
        return False

    # ---------------- 主动搜索（文档 §5） ----------------
    def _best_candidate(self):
        """计算 49 个候选点（+当前位置）的 Score，返回 (位置, 频道集合, Score)"""
        searchable = [c for c, m in self.channels.items()
                      if m.status == "searching" and m.area > 0]
        if not searchable:
            return None

        cands = list(_CANDIDATE_MASKS)
        # 当前位置也算候选：移动代价为 0，实现"就地扫描"
        cands.append((self.pos[0], self.pos[1],
                      _circle_mask(self.pos[0], self.pos[1], EXCLUDE_NO_SIGNAL_R)))

        best = None
        best_score = 0.0
        for sx, sy, mask in cands:
            picked, gain = [], 0.0
            for c in searchable:
                m = self.channels[c]
                inter = int(np.count_nonzero(m.grid & mask))
                if inter > 0:
                    picked.append(c)
                    gain += inter / m.area
            if not picked:
                continue
            t_switch = len(picked) - (1 if self.channel in picked else 0)
            cost = dist(self.pos, (sx, sy)) / 5.0 + 5.0 * len(picked) + t_switch
            score = gain / cost
            if score > best_score:
                best_score = score
                best = ((sx, sy), picked, score)

        if best is None or best_score <= SCORE_EPS:
            return None
        return best

    def _scan_channels(self, sx, sy, chs, label):
        """移动到 (sx,sy) 并依次检测 chs（首个 /measure 移动，其余原地）"""
        self.log_action("SCAN", label)
        for ch in chs:
            if self.remaining_real() < REAL_EXIT_RESERVE + REAL_ACTION_MIN:
                self.log_action("现实时间不足，停止扫描")
                break
            self._measure_one(sx, sy, ch)

    def _measure_one(self, sx, sy, ch):
        """在 (sx,sy) 检测频道 ch 并按文档 §4 更新候选区域与状态"""
        resp = self.client.measure(sx, sy, ch)
        if not resp or resp.get("accepted") is not True:
            self.log_action(f"  频道{ch} 检测失败")
            return
        self.virtual_time = resp.get("virtual_time_s", self.virtual_time)
        self.pos = (sx, sy)
        self.channel = ch
        m = self.channels[ch]
        result = resp.get("measure_result")
        if result == "direction":
            theta = resp.get("svd_deg")
            m.apply_direction(sx, sy, theta)
            self.log_action(f"  频道{ch}: 示向度={theta:.2f}° "
                            f"(已观测{len(m.observations)}次)")
        elif result == "near":
            self.log_action(f"  频道{ch}: 距离过近(<5m)，直接清除")
            self._do_clear(ch)
        elif result == "no_signal":
            m.apply_no_signal(sx, sy)

    # ---------------- 确定性补漏（文档 §7） ----------------
    def _coverage_step(self):
        """按最近优先访问下一个未访问航点并全频扫描未结论频道。返回是否还有航点。"""
        waypoints = generate_search_waypoints()
        unvisited = [w for w in waypoints if w not in self.visited_waypoints]
        if not unvisited:
            self._finalize_empty()
            return False
        if self.remaining_real() < TIME_RESERVE:
            self.log_action("现实时间不足，停止补漏")
            return False

        w = min(unvisited, key=lambda p: dist(self.pos, p))
        self.visited_waypoints.add(w)
        chs = [c for c, m in self.channels.items()
               if m.status not in ("cleared", "empty")]
        self.log_action("COVERAGE", f"航点=({w[0]:.0f},{w[1]:.0f})，"
                                    f"全频扫描 {len(chs)} 个未结论频道")
        for ch in chs:
            if self.remaining_real() < REAL_EXIT_RESERVE + REAL_ACTION_MIN:
                self.log_action("现实时间不足，停止扫描")
                break
            self._measure_one(w[0], w[1], ch)
        return True

    def _finalize_empty(self):
        """全部航点访问完毕：无观测且候选区域已空的频道判定为不存在（文档 §7.2）"""
        for c, m in self.channels.items():
            if m.status in ("cleared", "empty"):
                continue
            if not m.observations and m.area == 0:
                m.status = "empty"
                self.log_action(f"  频道{c}: 候选区域已全部排除，判定无源(empty)")

    def _print_summary(self):
        print("\n" + "=" * 60)
        print("测试总结（第三套：信息收益主动搜索）")
        print("=" * 60)
        print(f"总虚拟时间: {self.virtual_time:.1f} 秒")
        print(f"程序运行时间（现实）: {self.real_end - self.real_start:.1f} 秒")
        print(f"已清除频道: {sorted(self.cleared_channels)}")
        print(f"已清除数量: {len(self.cleared_channels)}")
        empty_n = sum(1 for m in self.channels.values() if m.status == "empty")
        unresolved = [c for c, m in self.channels.items()
                      if m.status not in ("cleared", "empty")]
        print(f"判定无源频道: {empty_n} 个")
        print(f"未解决频道: {unresolved}")
        if self.cleared_channels:
            avg = self.virtual_time / len(self.cleared_channels)
            print(f"平均定位清除时间（虚拟总时间/清除数）: {avg:.1f} 秒")
        print("观测统计:")
        for c in sorted(self.channels.keys()):
            m = self.channels[c]
            if m.observations:
                status = "已清除" if m.status == "cleared" else "未清除"
                print(f"  频道{c}: {len(m.observations)}次观测, {status}")
        print("=" * 60)

        if self.log_to_file:
            # 每次运行新建编号日志（robot_model3_log1.txt…），不覆盖历史
            import os
            base = os.path.dirname(os.path.abspath(__file__))
            n = 1
            while os.path.exists(os.path.join(base, f"robot_model3_log{n}.txt")):
                n += 1
            log_path = os.path.join(base, f"robot_model3_log{n}.txt")
            with open(log_path, "w", encoding="utf-8") as f:
                f.write("\n".join(self.log))
            print(f"日志已保存: {log_path}")


# ============================================================
# 主入口
# ============================================================
def main():
    global ROBOT_ID
    if len(sys.argv) > 1:
        ROBOT_ID = sys.argv[1]

    print(f"参赛队号: {ROBOT_ID}")
    print(f"模拟器地址: {BASE_URL}")
    print("等待模拟器接口就绪（倒计时结束后自动开始，无需按键）...")

    client = RobotClient(robot_id=ROBOT_ID)
    strategy = RobotStrategy3(client)

    attempts = 0
    while True:
        resp = client.enter()
        if resp and resp.get("accepted") is True:
            print("接口已就绪，开始执行任务。")
            break
        attempts += 1
        if attempts % 10 == 1:
            print(f"接口未就绪，继续等待...（第 {attempts} 次尝试）")
        time.sleep(1.0)

    strategy.run(entered_resp=resp)


if __name__ == "__main__":
    main()
