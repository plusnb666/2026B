import math
import time
import json
import urllib.request

# ==============================
# 定位算法
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


# ==============================
# 状态常量
# ==============================
UNKNOWN = 0      # 尚未发现该频道有信号
LOCATING = 1     # 已有1个示向度
LOCATED = 2      # 已定位（至少2个示向度或near）
CLEARED = 3      # 已清除
CLEAR_FAILED = 4  # 已定位但清除失败（如实标记，不谎报）


class ChannelInfo:
    def __init__(self, ch):
        self.ch = ch
        self.state = UNKNOWN
        self.bearings = []  # 每个元素为 (检测点位置, 示向度)
        self.estimated_pos = None


# ==============================
# 策略主类
# ==============================
class Strategy:
    def __init__(self, sim, log_to_file=True):
        self.sim = sim
        self.log_to_file = log_to_file   # False：离线测试等不落盘日志
        self.infos = [ChannelInfo(ch) for ch in range(1, 21)]
        self.log = []
        self.real_start = 0.0
        self.real_end = 0.0

    def log_action(self, action, detail=""):
        entry = f"[t={self.sim.virtual_time:.1f}s] {action} {detail}"
        self.log.append(entry)
        print(entry)

    def make_detection_points(self):
        """中心点 + 外围8个点，半径1200m"""
        pts = [(0.0, 0.0)]
        R = 1200.0
        for i in range(8):
            ang = math.radians(i * 45)
            pts.append((R * math.cos(ang), R * math.sin(ang)))
        return pts

    @staticmethod
    def clamp_to_arena(pos, limit=1790.0):
        """把位置限幅到场内（距原点≤limit）"""
        x, y = pos
        r = math.hypot(x, y)
        if r <= limit:
            return pos
        scale = limit / r
        return (x * scale, y * scale)

    def second_measure(self, info):
        """对只有1个示向度的频道做第二测点（横向探针），完成交点定位。
        返回 True 表示已定位（LOCATED），False 表示定位失败（保持 LOCATING）。"""
        ch = info.ch
        pos1, theta1 = info.bearings[0]
        rad1 = math.radians(theta1)
        # 沿示向度前进600m，再垂直偏移300m 作为第二测点
        mid = (pos1[0] + 600 * math.cos(rad1),
               pos1[1] + 600 * math.sin(rad1))
        s2 = self.clamp_to_arena(
            (mid[0] - 300 * math.sin(rad1), mid[1] + 300 * math.cos(rad1)))
        self.log_action(f"二次定位频道{ch}:",
                        f"移动到({s2[0]:.0f},{s2[1]:.0f})并检测 ")
        res = self.sim.measure(s2, ch)
        if not res or not res.get('accepted'):
            return False
        if res['measure_result'] == 'direction':
            theta2 = res['svd_deg']
            info.bearings.append((s2, theta2))
            self.log_action(f"  频道{ch}:",
                            f"示向度={theta2:.2f}° (已观测{len(info.bearings)}次) ")
            p1, t1 = info.bearings[-2]
            p2, t2 = info.bearings[-1]
            info.estimated_pos = self.clamp_to_arena(
                locate_from_two_points(p1, t1, p2, t2))
            info.state = LOCATED
            self.log_action(f"  频道{ch}:",
                            f"已定位，交点估计位置=({info.estimated_pos[0]:.0f},{info.estimated_pos[1]:.0f}) ")
            return True
        if res['measure_result'] == 'near':
            info.estimated_pos = s2
            info.state = LOCATED
            self.log_action(f"  频道{ch}:", "距离过近(<5m)，定位到当前位置 ")
            return True
        # no_signal：换对称偏移点再测一次
        s3 = self.clamp_to_arena(
            (mid[0] + 300 * math.sin(rad1), mid[1] - 300 * math.cos(rad1)))
        self.log_action(f"  频道{ch}:",
                        f"第二测点无信号，换对称点({s3[0]:.0f},{s3[1]:.0f})再测 ")
        res = self.sim.measure(s3, ch)
        if not res or not res.get('accepted'):
            return False
        if res['measure_result'] == 'direction':
            theta2 = res['svd_deg']
            info.bearings.append((s3, theta2))
            self.log_action(f"  频道{ch}:",
                            f"示向度={theta2:.2f}° (已观测{len(info.bearings)}次) ")
            p1, t1 = info.bearings[-2]
            p2, t2 = info.bearings[-1]
            info.estimated_pos = self.clamp_to_arena(
                locate_from_two_points(p1, t1, p2, t2))
            info.state = LOCATED
            self.log_action(f"  频道{ch}:",
                            f"已定位，交点估计位置=({info.estimated_pos[0]:.0f},{info.estimated_pos[1]:.0f}) ")
            return True
        if res['measure_result'] == 'near':
            info.estimated_pos = s3
            info.state = LOCATED
            self.log_action(f"  频道{ch}:", "距离过近(<5m)，定位到当前位置 ")
            return True
        self.log_action(f"  频道{ch}:", "第二测点仍无信号，定位失败 ")
        return False

    def scan_channel(self, pos, ch):
        info = self.infos[ch - 1]
        res = self.sim.measure(pos, ch)
        if not res or not res.get('accepted'):
            return

        if res['measure_result'] == 'direction':
            theta = res['svd_deg']
            info.bearings.append((pos, theta))
            self.log_action(f"  频道{ch}:", f"示向度={theta:.2f}° (已观测{len(info.bearings)}次) ")
            if info.state == UNKNOWN:
                info.state = LOCATING
            elif info.state == LOCATING:
                # 取最近两个示向度定位
                p1, t1 = info.bearings[-2]
                p2, t2 = info.bearings[-1]
                info.estimated_pos = self.clamp_to_arena(
                    locate_from_two_points(p1, t1, p2, t2))
                info.state = LOCATED
                self.log_action(f"  频道{ch}:",
                                f"已定位，交点估计位置=({info.estimated_pos[0]:.0f},{info.estimated_pos[1]:.0f}) ")
        elif res['measure_result'] == 'near':
            # 距离很近，当前位置即可作为估计位置
            info.estimated_pos = pos
            info.state = LOCATED
            self.log_action(f"  频道{ch}:", "距离过近(<5m)，定位到当前位置 ")
        # no_signal 保持不变

    def scan_at_point(self, pos):
        """在当前检测点扫描所有未定位的频道"""
        chs = [info.ch for info in self.infos if info.state in (UNKNOWN, LOCATING)]
        if not chs:
            return
        chs.sort()
        self.log_action("SCAN", f"移动到({pos[0]:.0f},{pos[1]:.0f})，扫描 {len(chs)} 个频道")
        for ch in chs:
            self.scan_channel(pos, ch)

    def _cleared_count(self):
        return sum(1 for info in self.infos if info.state == CLEARED)

    def clear_one(self, info):
        """清除单个干扰源：清除失败后横向探针重定位（最多3轮），
        最后网格搜索兜底；全部失败则如实标记，不谎报"""
        ch = info.ch
        pos = info.estimated_pos

        self.log_action(f"清除频道{ch}",
                        f"移动到估计位置({pos[0]:.0f},{pos[1]:.0f})并清除 ")

        for _ in range(3):
            res = self.sim.clear(pos, ch)
            if not res or not res.get('accepted'):
                return False
            if res['clear_result'] == 'success':
                info.state = CLEARED
                self.log_action(f"  清除频道{ch}",
                                f"成功！(累计{self._cleared_count()}个) ")
                return True

            self.log_action(f"  清除频道{ch}", "未发现目标(20m内无干扰源) ")

            # 失败：原地重新测量
            res = self.sim.measure(pos, ch)
            if not res or not res.get('accepted'):
                break
            if res['measure_result'] == 'near':
                self.log_action(f"  清除频道{ch}", "距离过近，原地重试清除 ")
                continue
            if res['measure_result'] != 'direction':
                break
            theta = res['svd_deg']
            self.log_action(f"  修正频道{ch}:",
                            f"原地示向度={theta:.2f}° (已观测{len(info.bearings) + 1}次) ")
            # 横向探针：沿示向度前进600m，垂直偏移300m 做第二测点
            rad = math.radians(theta)
            probe = self.clamp_to_arena(
                (pos[0] + 600 * math.cos(rad) - 300 * math.sin(rad),
                 pos[1] + 600 * math.sin(rad) + 300 * math.cos(rad)))
            self.log_action(f"  修正频道{ch}:",
                            f"横向探针移动到({probe[0]:.0f},{probe[1]:.0f})并检测 ")
            res = self.sim.measure(probe, ch)
            if not res or not res.get('accepted'):
                break
            if res['measure_result'] == 'near':
                pos = probe
                continue
            if res['measure_result'] != 'direction':
                break
            theta2 = res['svd_deg']
            # 用原地示向度与探针示向度做交点，重新定位
            new_pos = self.clamp_to_arena(
                locate_from_two_points(pos, theta, probe, theta2))
            pos = new_pos
            info.estimated_pos = new_pos
            self.log_action(f"  修正频道{ch}:",
                            f"新估计位置=({new_pos[0]:.0f},{new_pos[1]:.0f})，再次尝试清除 ")

        # 最后手段：网格搜索（偏移15m，覆盖半径20m）
        offsets = [(15, 0), (-15, 0), (0, 15), (0, -15),
                   (15, 15), (-15, 15), (15, -15), (-15, -15)]
        for dx, dy in offsets:
            new_pos = self.clamp_to_arena((pos[0] + dx, pos[1] + dy))
            res = self.sim.clear(new_pos, ch)
            if res['clear_result'] == 'success':
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
        current = self.sim.current_pos
        remaining = set(located)
        while remaining:
            # 最近邻选择
            next_info = min(remaining,
                            key=lambda info: math.hypot(info.estimated_pos[0] - current[0],
                                                        info.estimated_pos[1] - current[1]))
            self.clear_one(next_info)
            remaining.remove(next_info)
            current = self.sim.current_pos

    def run(self):
        start_real = time.time()
        self.real_start = start_real
        resp = self.sim.enter()
        self.log_action("ENTER",
                        f"现实时限={resp.get('remaining_real_duration_s', 1200)}s")

        try:
            # 1. 扫描所有检测点
            for pt in self.make_detection_points():
                self.scan_at_point(pt)

            # 1.5 补漏：对只有1个示向度的频道做第二测点定位
            locating = [info for info in self.infos if info.state == LOCATING]
            self.log_action("开始补漏定位阶段",
                            f"仅1次观测的频道: {sorted(i.ch for i in locating)}")
            for info in locating:
                self.second_measure(info)

            # 2. 清除所有已定位干扰源
            self.clear_all_located()
        finally:
            # 3. 退出
            self.sim.exit()
            self.real_end = time.time()
            self.log_action("EXIT", f"清除频道数={self._cleared_count()}")
        elapsed_real = time.time() - start_real
        self._print_summary()

        cleared = sum(1 for info in self.infos if info.state == CLEARED)
        virtual_time = self.sim.virtual_time
        measure_cnt = getattr(self.sim, 'measure_count', 0)
        clear_cnt = getattr(self.sim, 'clear_count', 0)
        return {
            'cleared': cleared,
            'total_sources': getattr(self.sim, 'n_sources', '?'),
            'virtual_time': virtual_time,
            'real_time': elapsed_real,
            'measure_cnt': measure_cnt,
            'clear_cnt': clear_cnt,
        }

    def _print_summary(self):
        cleared = sorted(info.ch for info in self.infos if info.state == CLEARED)
        locating = sorted(info.ch for info in self.infos if info.state == LOCATING)
        failed = sorted(info.ch for info in self.infos if info.state == CLEAR_FAILED)
        unknown = sorted(info.ch for info in self.infos if info.state == UNKNOWN)
        print("\n" + "=" * 60)
        print("测试总结（robot(5)：9点扫描+两示向度定位策略）")
        print("=" * 60)
        print(f"总虚拟时间: {self.sim.virtual_time:.1f} 秒")
        print(f"程序运行时间（现实）: {self.real_end - self.real_start:.1f} 秒")
        print(f"已清除频道: {cleared}")
        print(f"已清除数量: {len(cleared)}")
        print(f"未检测到信号频道: {unknown}")
        print(f"已发现未定位频道: {locating}")
        print(f"清除失败频道（如实未清除）: {failed}")
        if cleared:
            avg = self.sim.virtual_time / len(cleared)
            print(f"平均定位清除时间（虚拟总时间/清除数）: {avg:.1f} 秒")
        print("观测统计:")
        for info in self.infos:
            if info.bearings:
                status = "已清除" if info.state == CLEARED else (
                    "清除失败" if info.state == CLEAR_FAILED else "未清除")
                print(f"  频道{info.ch}: {len(info.bearings)}次观测, {status}")
        print("=" * 60)

        if self.log_to_file:
            # 每次运行新建编号日志（robot5_logs/ 文件夹内），不覆盖历史
            import os
            base = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "robot5_logs")
            os.makedirs(base, exist_ok=True)
            n = 1
            while os.path.exists(os.path.join(base, f"robot5_log{n}.txt")):
                n += 1
            log_path = os.path.join(base, f"robot5_log{n}.txt")
            with open(log_path, "w", encoding="utf-8") as f:
                f.write("\n".join(self.log))
            print(f"日志已保存: {log_path}")


# ==============================
# 真实模拟器客户端（正式测试用）
# ==============================
ROBOT_ID = "202610061109"
BASE_URL = "http://127.0.0.1:2026"


class RealSimulator:
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
        payload = self._base_payload()
        return self._post('/exit', payload)


def real_test():
    print("=== 真实模拟器测试 ===")
    sim = RealSimulator(ROBOT_ID, BASE_URL)
    strat = Strategy(sim)
    stats = strat.run()
    print(f"清除结果: {stats['cleared']} | 虚拟时间: {stats['virtual_time']:.2f}s | "
          f"现实时间: {stats['real_time']*1000:.1f}ms")


if __name__ == '__main__':
    real_test()
