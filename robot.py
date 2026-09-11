# -*- coding: utf-8 -*-
"""
问题3：机器狗自动搜索定位清除系统（全向干扰源场景）
配套文档：问题3-全向干扰源自动定位与清除策略.md
依赖：geometry.py（内部复用 problem1_algorithm.py）
运行：Python robot.py <参赛队号>
"""
import json
import os
import sys
import time
from urllib.request import Request, urlopen
from urllib.error import URLError, HTTPError
from math import cos, sin, radians, sqrt

from geometry import (sector_to_halfplanes, intersect_halfplanes,
                      polygon_diameter_bruteforce)

# ============================================================
# 配置
# ============================================================
BASE_URL = "http://127.0.0.1:2026"
ROBOT_ID = "202610061109"  # 运行前修改为参赛队号（也可用命令行参数传入）

ARENA_RADIUS = 1800   # 目标区域半径（米）

# 时间模型（双时钟，与文档"时间管理"一致）：
# 现实时钟：/enter 返回的 remaining_real_duration_s（≤1200秒）是现实时间，
#           由 time.monotonic() 追踪，决定何时收尾退出；
# 虚拟时钟：virtual_time_s 由模拟器返回，上限 360000 秒（100小时），
#           基本不构成约束，是评价指标（平均定位清除时间）的来源。
REAL_TIME_LIMIT_DEFAULT = 1200   # /enter 未返回时限时的兜底（现实秒）
REAL_EXIT_RESERVE = 30           # 现实剩余不足 30 秒：停止新操作，收尾退出
TIME_RESERVE = 60                # 现实剩余不足 60 秒：不再开启新的二次定位目标
REAL_ACTION_MIN = 15             # 单次扫描动作前要求的最小现实剩余（秒）
VIRTUAL_TIME_LIMIT = 360000      # 虚拟时间上限（模拟器硬限制，安全兜底）

# 策略参数
MIN_OBSERVATIONS = 2             # 定位所需最少观测数
MAX_CLEAR_RETRY = 3              # 单频道最大清除重试次数
CLEAR_DIAM_GATE = 40.0           # 定位直径超过该值（米）时先补测再清除
SECOND_STATION_DIST = 600        # 二次检测点：沿示向度前进距离（米）
SECOND_STATION_OFFSET = 300      # 二次检测点：垂直偏移距离（米）


# ============================================================
# HTTP 客户端
# ============================================================
class RobotClient:
    def __init__(self, base_url=BASE_URL, robot_id=ROBOT_ID, timeout=10):
        self.base_url = base_url
        self.robot_id = robot_id
        self.timeout = timeout
        self.req_counter = 0

    def _post(self, path, payload, retries=3):
        """POST 请求。网络失败重试时复用同一 payload（同一 request_id），
        符合接口"幂等重试"要求。"""
        url = self.base_url + path
        data = json.dumps(payload).encode("utf-8")
        for attempt in range(retries):
            try:
                req = Request(url, data=data,
                              headers={"Content-Type": "application/json"},
                              method="POST")
                with urlopen(req, timeout=self.timeout) as resp:
                    return json.loads(resp.read().decode("utf-8"))
            except (URLError, HTTPError, ConnectionError, OSError) as e:
                if attempt < retries - 1:
                    time.sleep(0.5)
                    continue
                print(f"  [HTTP Error] {path}: {e}")
                return None
        return None

    def _base(self, prefix):
        self.req_counter += 1
        return {
            "arena_id": "default",
            "robot_id": self.robot_id,
            "request_id": f"{prefix}-{self.req_counter}"
        }

    def enter(self):
        return self._post("/enter", self._base("enter"))

    def exit(self):
        return self._post("/exit", self._base("exit"))

    def measure(self, x, y, channel):
        payload = self._base("measure")
        payload["position"] = {"x": x, "y": y}
        payload["channel"] = channel
        return self._post("/measure", payload)

    def clear(self, x, y, channel):
        payload = self._base("clear")
        payload["position"] = {"x": x, "y": y}
        payload["channel"] = channel
        return self._post("/clear", payload)


# ============================================================
# 几何工具
# ============================================================
def dist(p1, p2):
    return sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def locate_from_observations(observations):
    """交会定位。observations: [(pos, theta_deg), ...]，其中 pos=(x, y)
    返回 (估计x, 估计y, 定位区域直径, 顶点列表) 或 None。
    初始裁剪区域为 1800 米圆域内接正 96 边形（与问题1修正一致，不用矩形包围盒）。
    """
    if len(observations) < MIN_OBSERVATIONS:
        return None

    hps = []
    for pos, theta in observations:
        hps.extend(sector_to_halfplanes(pos[0], pos[1], theta))

    poly = intersect_halfplanes(hps)
    if not poly or len(poly) < 3:
        return None

    cx_p = sum(p[0] for p in poly) / len(poly)
    cy_p = sum(p[1] for p in poly) / len(poly)

    diameter, _, _ = polygon_diameter_bruteforce(poly)
    return (cx_p, cy_p, diameter, poly)


def point_on_ray(origin, angle_deg, distance):
    """从原点出发，沿角度方向移动 distance 后的点"""
    rad = radians(angle_deg)
    return (origin[0] + distance * cos(rad), origin[1] + distance * sin(rad))


def perpendicular_offset(origin, angle_deg, offset):
    """从原点出发，沿角度方向的垂直方向偏移 offset（左侧为正）"""
    rad = radians(angle_deg)
    return (origin[0] - offset * sin(rad), origin[1] + offset * cos(rad))


# ============================================================
# 搜索路径生成
# ============================================================
def generate_search_waypoints():
    """搜索航点：原点 + 6 方向 × (900米, 1500米)，共 13 个。
    数值验证：圆域内任一点到最近航点距离 ≤ 901.9 米 < 最小有效接收半径 1000 米，
    保证全区域覆盖（见配套文档）。"""
    waypoints = [(0, 0)]
    for angle in range(0, 360, 60):
        for r in [900, 1500]:
            pt = point_on_ray((0, 0), angle, r)
            if dist(pt, (0, 0)) <= ARENA_RADIUS + 100:
                waypoints.append(pt)
    return waypoints


def nearest_undiscovered_position(current_pos, visited_positions):
    """下一个未访问的搜索航点（距当前位置最近优先）"""
    waypoints = generate_search_waypoints()
    best = None
    best_dist = float('inf')
    for wp in waypoints:
        visited = any(dist(wp, vp) < 200 for vp in visited_positions)
        if visited:
            continue
        d = dist(current_pos, wp)
        if d < best_dist:
            best_dist = d
            best = wp
    return best


# ============================================================
# 主策略
# ============================================================
class RobotStrategy:
    def __init__(self, client):
        self.client = client
        self.pos = (0.0, 0.0)
        self.channel = 1
        self.virtual_time = 0.0        # 虚拟时间（模拟器返回）
        self.real_deadline_ts = 0.0    # 现实截止时刻（time.monotonic 基准）
        self.real_start = 0.0          # /enter 成功时刻（现实），用于统计程序运行时间
        self.real_end = 0.0

        self.cleared_channels = set()
        self.observations = {}          # {channel: [(pos, theta), ...]}
        self.estimated_positions = {}   # {channel: (x, y, diameter)}
        self.clear_attempts = {}        # {channel: count}
        self.obs_count_at_attempt = {}  # {channel: 上次尝试清除时的观测数}
        self.visited_positions = [(0, 0)]
        self.log = []
        self.full_scan_next = True      # 下一轮迭代做全量扫描（原点/搜索航点）

    def log_action(self, action, detail=""):
        entry = f"[t={self.virtual_time:.1f}s] {action} {detail}"
        self.log.append(entry)
        print(entry)

    def remaining_real(self):
        """现实剩余时间（秒）"""
        return self.real_deadline_ts - time.monotonic()

    def run(self, entered_resp=None):
        """主流程。entered_resp：main() 轮询得到的 /enter 成功响应；None 时自行调用。"""
        resp = entered_resp if entered_resp is not None else self.client.enter()
        if not resp or resp.get("accepted") is not True:
            print("进入失败！")
            return
        self.real_deadline_ts = time.monotonic() + float(
            resp.get("remaining_real_duration_s", REAL_TIME_LIMIT_DEFAULT))
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

            # 步骤1：扫描（原点/搜索航点全量扫描，其余位置只扫观测不足2次的频道）
            self._scan_current_position(full=self.full_scan_next)
            self.full_scan_next = False

            # 步骤2：更新所有频道的定位估计
            self._update_estimations()

            # 步骤3：选择目标并执行清除
            target = self._select_target()
            if target is not None:
                self._approach_and_clear(target)
            else:
                # 没有可处理的目标，移动到下一个搜索航点
                if self.remaining_real() < TIME_RESERVE:
                    self.log_action("现实时间不足，停止搜索")
                    break
                next_pos = nearest_undiscovered_position(self.pos, self.visited_positions)
                if next_pos is None:
                    self.log_action("所有搜索位置已访问，停止")
                    break
                self._move_to(next_pos)
                self.visited_positions.append(next_pos)
                self.full_scan_next = True    # 航点做全量扫描

            # 已清除 16 个（总数上限），提前结束
            if len(self.cleared_channels) >= 16:
                self.log_action("已清除16个（最大可能），结束")
                break

    def _scan_current_position(self, full=False):
        """批量扫描。full=True（原点/搜索航点）：所有未清除频道；
        full=False（其余位置）：仅观测次数不足 MIN_OBSERVATIONS 次的频道。
        """
        if full:
            channels_to_scan = [ch for ch in range(1, 21) if ch not in self.cleared_channels]
        else:
            channels_to_scan = [ch for ch in range(1, 21)
                                if ch not in self.cleared_channels
                                and len(self.observations.get(ch, [])) < MIN_OBSERVATIONS]
        if not channels_to_scan:
            return

        self.log_action("SCAN", f"扫描 {len(channels_to_scan)} 个频道"
                                f"({'全量' if full else '补漏'})")

        for ch in channels_to_scan:
            # 现实时间检查
            if self.remaining_real() < REAL_EXIT_RESERVE + REAL_ACTION_MIN:
                self.log_action("现实时间不足，停止扫描")
                break

            # 本位置已对该频道测过方向（_move_to 移动时可能顺带测过），跳过
            obs = self.observations.get(ch, [])
            if obs and dist(obs[-1][0], self.pos) < 1:
                continue

            resp = self.client.measure(self.pos[0], self.pos[1], ch)
            if not resp or resp.get("accepted") is not True:
                self.log_action(f"  频道{ch} 检测失败")
                continue

            self.virtual_time = resp.get("virtual_time_s", self.virtual_time)
            self.channel = ch

            result = resp.get("measure_result")
            if result == "direction":
                theta = resp.get("svd_deg")
                self.observations.setdefault(ch, []).append((self.pos, theta))
                self.log_action(f"  频道{ch}: 示向度={theta:.2f}° "
                                f"(已观测{len(self.observations[ch])}次)")
            elif result == "near":
                self.log_action(f"  频道{ch}: 距离过近(<5m)，直接清除")
                self._do_clear(ch)
            elif result == "no_signal":
                pass

    def _update_estimations(self):
        """根据观测更新所有频道的定位估计"""
        for ch, obs in self.observations.items():
            if ch in self.cleared_channels:
                continue
            if len(obs) >= MIN_OBSERVATIONS:
                result = locate_from_observations(obs)
                if result is not None:
                    ex, ey, diameter, _ = result
                    self.estimated_positions[ch] = (ex, ey, diameter)

    def _select_target(self):
        """选择下一个要处理的目标频道。
        优先级：1. 已有定位估计且距离当前位置最近；2. 只有单次观测的频道。
        """
        # 候选1：已有定位估计的频道
        candidates = []
        for ch, (ex, ey, diam) in self.estimated_positions.items():
            if ch in self.cleared_channels:
                continue
            if self.clear_attempts.get(ch, 0) >= MAX_CLEAR_RETRY:
                # 放弃后又积累新观测（如搜索航点补测），重新给予清除机会
                if len(self.observations.get(ch, [])) > self.obs_count_at_attempt.get(ch, 0):
                    self.clear_attempts[ch] = 0
                else:
                    continue
            d = dist(self.pos, (ex, ey))
            candidates.append((d, ch))

        if candidates:
            candidates.sort()
            d, ch = candidates[0]
            self.log_action(f"选择目标: 频道{ch} (已定位, 距离={d:.0f}m)")
            return ch

        # 候选2：只有1个观测的频道，移动到第二个检测点
        single_obs = []
        for ch, obs in self.observations.items():
            if ch in self.cleared_channels:
                continue
            if len(obs) == 1:
                pos, theta = obs[0]
                # 估计干扰源在示向度方向上约800米处（用于排序）
                est_pos = point_on_ray(pos, theta, 800)
                d = dist(self.pos, est_pos)
                single_obs.append((d, ch))

        if single_obs and self.remaining_real() > TIME_RESERVE:
            single_obs.sort()
            d, ch = single_obs[0]
            self.log_action(f"选择目标: 频道{ch} (单次观测, 需二次定位, 估计距离={d:.0f}m)")
            return ch

        return None

    def _approach_and_clear(self, channel):
        """逼近目标频道并清除。
        已定位：直径≤闸门直接 /clear；直径过大先到估计位置补测再清除。
        单次观测：移动到第二个检测点获得第二次示向度后定位清除。
        """
        if channel in self.estimated_positions:
            ex, ey, diam = self.estimated_positions[channel]
            self.log_action(f"逼近频道{channel}: 估计位置=({ex:.0f},{ey:.0f}), "
                            f"定位直径={diam:.1f}m")

            if diam > CLEAR_DIAM_GATE:
                # 定位区域太大：在最近观测站的示向度方向横向取点补测。
                # 注意不能沿示向度方向取点——那样与既有观测共线、交角≈0°，
                # 新观测无信息量（离线测试种子5频道13的失败即源于此）；
                # 横向取点保证交角大，且该点到干扰源≤957米，必能收到信号
                # （与二次检测点同一设计，见文档3.3）。
                (ox, oy), oth = min(self.observations[channel],
                                    key=lambda o: dist(o[0], (ex, ey)))
                mid_point = point_on_ray((ox, oy), oth, SECOND_STATION_DIST)
                probe = perpendicular_offset(mid_point, oth, SECOND_STATION_OFFSET)
                self.log_action(f"  定位直径{diam:.1f}m>{CLEAR_DIAM_GATE:.0f}m，"
                                f"横向补测点=({probe[0]:.0f},{probe[1]:.0f})")
                resp = self.client.measure(probe[0], probe[1], channel)
                if resp and resp.get("accepted") is True:
                    self.virtual_time = resp.get("virtual_time_s", self.virtual_time)
                    self.pos = probe
                    self.channel = channel
                    result = resp.get("measure_result")
                    if result == "direction":
                        theta = resp.get("svd_deg")
                        self.observations.setdefault(channel, []).append((self.pos, theta))
                        self._update_estimations()
                        if channel in self.estimated_positions:
                            ex, ey, diam = self.estimated_positions[channel]
                    elif result == "near":
                        return self._move_and_clear(channel, self.pos)
                # 补测后（无论成功与否）继续尝试清除
            return self._move_and_clear(channel, (ex, ey))
        else:
            # 只有单次观测，进行第二次检测
            obs = self.observations[channel]
            if len(obs) < 1:
                return False
            pos1, theta1 = obs[0]

            # 第二个检测点：沿示向度方向前进600米，再垂直偏移300米
            # （数值验证：该点到干扰源距离≤957米<最小接收半径，二次检测必能收到信号）
            mid_point = point_on_ray(pos1, theta1, SECOND_STATION_DIST)
            s2_pos = perpendicular_offset(mid_point, theta1, SECOND_STATION_OFFSET)

            self.log_action(f"二次定位频道{channel}: "
                            f"移动到({s2_pos[0]:.0f},{s2_pos[1]:.0f})并检测")

            resp = self.client.measure(s2_pos[0], s2_pos[1], channel)
            if resp and resp.get("accepted") is True:
                self.virtual_time = resp.get("virtual_time_s", self.virtual_time)
                self.pos = (s2_pos[0], s2_pos[1])
                self.channel = channel
                result = resp.get("measure_result")
                if result == "direction":
                    theta2 = resp.get("svd_deg")
                    self.observations[channel].append((self.pos, theta2))
                    self.log_action(f"  二次示向度={theta2:.2f}°")
                    self._update_estimations()
                    if channel in self.estimated_positions:
                        ex, ey, diam = self.estimated_positions[channel]
                        return self._move_and_clear(channel, (ex, ey))
                elif result == "near":
                    self.log_action("  距离过近，直接清除")
                    return self._move_and_clear(channel, self.pos)
                else:
                    # no_signal：设计保证二次检测必能收到信号，出现说明该源已
                    # 被清除（如兜底移动意外清除）或数据异常：放弃该频道防止死循环
                    self.log_action(f"  二次检测无信号，放弃频道{channel}")
                    self.observations[channel] = []
            return False

    def _move_and_clear(self, channel, target):
        """移动到目标位置并尝试清除（用 /clear 指令实现移动+清除）"""
        self.clear_attempts[channel] = self.clear_attempts.get(channel, 0) + 1
        self.obs_count_at_attempt[channel] = len(self.observations.get(channel, []))
        resp = self.client.clear(target[0], target[1], channel)
        if not resp or resp.get("accepted") is not True:
            self.log_action(f"  移动清除频道{channel} 失败（请求未接受）")
            return False

        self.virtual_time = resp.get("virtual_time_s", self.virtual_time)
        self.pos = (target[0], target[1])
        result = resp.get("clear_result")
        if result == "success":
            self.cleared_channels.add(channel)
            self.log_action(f"  清除频道{channel} 成功！(累计{len(self.cleared_channels)}个)")
            return True
        else:
            self.log_action(f"  清除频道{channel} 未发现目标(20m内无干扰源)")
            if self.clear_attempts[channel] < MAX_CLEAR_RETRY:
                # 原地补测修正位置后再次清除（修正：原实现只移动到新估计不尝试清除）
                return self._refine_position(channel)
            return False

    def _do_clear(self, channel):
        """在当前位置执行清除（near 场景，距干扰源≤5米，必成功）"""
        self.clear_attempts[channel] = self.clear_attempts.get(channel, 0) + 1
        self.obs_count_at_attempt[channel] = len(self.observations.get(channel, []))
        resp = self.client.clear(self.pos[0], self.pos[1], channel)
        if not resp or resp.get("accepted") is not True:
            self.log_action(f"  清除频道{channel} 失败（请求未接受）")
            return False

        self.virtual_time = resp.get("virtual_time_s", self.virtual_time)
        result = resp.get("clear_result")
        if result == "success":
            self.cleared_channels.add(channel)
            self.log_action(f"  清除频道{channel} 成功！(累计{len(self.cleared_channels)}个)")
            return True
        else:
            self.log_action(f"  清除频道{channel} 未发现目标(20m内无干扰源)")
            if self.clear_attempts[channel] < MAX_CLEAR_RETRY:
                self._refine_position(channel)
            return False

    def _refine_position(self, channel):
        """清除失败后，在当前位置补测修正位置，并移动到新估计位置再次清除。
        返回最终是否清除成功。"""
        self.log_action(f"  修正频道{channel} 位置...")
        resp = self.client.measure(self.pos[0], self.pos[1], channel)
        if resp and resp.get("accepted") is True:
            self.virtual_time = resp.get("virtual_time_s", self.virtual_time)
            self.channel = channel
            if resp.get("measure_result") == "direction":
                theta = resp.get("svd_deg")
                self.observations.setdefault(channel, []).append((self.pos, theta))
                self._update_estimations()
                if channel in self.estimated_positions:
                    ex, ey, _ = self.estimated_positions[channel]
                    self.log_action(f"  新估计位置=({ex:.0f},{ey:.0f})，再次尝试清除")
                    return self._move_and_clear(channel, (ex, ey))
        return False

    def _move_to(self, target):
        """移动到目标位置：用 /measure 指令移动并顺带检测当前频道。
        measure 失败时用 /clear 兜底移动（clear 也会移动机器狗）。
        修正：兜底 clear 若意外清除成功，记录进已清除集合，防止统计失真。
        """
        if dist(self.pos, target) < 1:
            return
        resp = self.client.measure(target[0], target[1], self.channel)
        if resp and resp.get("accepted") is True:
            self.virtual_time = resp.get("virtual_time_s", self.virtual_time)
            self.pos = (target[0], target[1])
            result = resp.get("measure_result")
            if result == "direction":
                theta = resp.get("svd_deg")
                self.observations.setdefault(self.channel, []).append((self.pos, theta))
            elif result == "near":
                pass  # 过近，后续扫描时处理
        else:
            resp2 = self.client.clear(target[0], target[1], self.channel)
            if resp2 and resp2.get("accepted") is True:
                self.virtual_time = resp2.get("virtual_time_s", self.virtual_time)
                self.pos = (target[0], target[1])
                if resp2.get("clear_result") == "success":
                    self.cleared_channels.add(self.channel)
                    self.log_action(f"  兜底移动中意外清除频道{self.channel}，已记录")

    def _print_summary(self):
        print("\n" + "=" * 60)
        print("测试总结")
        print("=" * 60)
        print(f"总虚拟时间: {self.virtual_time:.1f} 秒")
        print(f"程序运行时间（现实）: {self.real_end - self.real_start:.1f} 秒")
        print(f"已清除频道: {sorted(self.cleared_channels)}")
        print(f"已清除数量: {len(self.cleared_channels)}")
        if self.cleared_channels:
            avg = self.virtual_time / len(self.cleared_channels)
            print(f"平均定位清除时间（虚拟总时间/清除数）: {avg:.1f} 秒")
        print(f"观测统计:")
        for ch in sorted(self.observations.keys()):
            status = "已清除" if ch in self.cleared_channels else "未清除"
            print(f"  频道{ch}: {len(self.observations[ch])}次观测, {status}")
        print("=" * 60)

        log_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "robot_log.txt")
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
    strategy = RobotStrategy(client)

    # 修正：去掉手工按 Enter——轮询 /enter 直到接口就绪。
    # 接口未就绪时 accepted=false，不消耗测试资源。
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
