# -*- coding: utf-8 -*-
"""问题3第二套模型：圆心-环点分层搜索与逐源清除。

启动模拟器的问题3演练测试并等待接口就绪后运行：
    python robot_model2.py <参赛队号>
"""
from __future__ import annotations

import json
import math
import sys
import time
from dataclasses import dataclass
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

from problem1_algorithm import locate_region

BASE_URL = "http://127.0.0.1:2026"
ARENA_RADIUS = 1800.0
RING_RADIUS = 1200.0
CHANNELS = tuple(range(1, 21))
SPEED = 5.0
TIME_RESERVE = 120.0
MAX_REFINE_ROUNDS = 3


@dataclass
class Observation:
    position: tuple[float, float]
    angle: float


@dataclass
class Estimate:
    channel: int
    position: tuple[float, float]
    diameter: float


def distance(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def inside_arena(position: tuple[float, float], margin: float = 10.0) -> tuple[float, float]:
    radius = distance(position, (0.0, 0.0))
    limit = ARENA_RADIUS - margin
    if radius <= limit:
        return position
    scale = limit / radius
    return position[0] * scale, position[1] * scale


def centroid(vertices: list[tuple[float, float]]) -> tuple[float, float]:
    return (
        sum(point[0] for point in vertices) / len(vertices),
        sum(point[1] for point in vertices) / len(vertices),
    )


class RobotClient:
    def __init__(self, robot_id: str, base_url: str = BASE_URL, timeout: float = 8.0):
        self.robot_id = robot_id
        self.base_url = base_url.rstrip("/")
        self.timeout = timeout
        self.request_number = 0

    def _base(self, prefix: str) -> dict:
        self.request_number += 1
        return {
            "arena_id": "default",
            "robot_id": self.robot_id,
            "request_id": f"{prefix}-{self.request_number}",
        }

    def _post(self, path: str, payload: dict, retries: int = 3) -> dict | None:
        # 重试时复用相同 payload 和 request_id，避免动作重复执行。
        data = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
        for attempt in range(retries):
            request = Request(
                self.base_url + path,
                data=data,
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            try:
                with urlopen(request, timeout=self.timeout) as response:
                    return json.loads(response.read().decode("utf-8"))
            except HTTPError as error:
                body = error.read().decode("utf-8", errors="replace")
                try:
                    return json.loads(body)
                except json.JSONDecodeError:
                    print(f"HTTP {error.code}: {body}")
                    return None
            except (URLError, TimeoutError, OSError) as error:
                if attempt + 1 == retries:
                    print(f"连接失败 {path}: {error}")
                    return None
                time.sleep(0.3 * (attempt + 1))
        return None

    def enter(self) -> dict | None:
        return self._post("/enter", self._base("enter"))

    def exit(self) -> dict | None:
        return self._post("/exit", self._base("exit"))

    def measure(self, position: tuple[float, float], channel: int) -> dict | None:
        payload = self._base("measure")
        payload["position"] = {"x": position[0], "y": position[1]}
        payload["channel"] = channel
        return self._post("/measure", payload)

    def clear(self, position: tuple[float, float], channel: int) -> dict | None:
        payload = self._base("clear")
        payload["position"] = {"x": position[0], "y": position[1]}
        payload["channel"] = channel
        return self._post("/clear", payload)


class Model2Strategy:
    def __init__(self, client: RobotClient):
        self.client = client
        self.position = (0.0, 0.0)
        self.current_channel = 1
        self.virtual_time = 0.0
        self.real_start = 0.0
        self.real_limit = 0.0
        self.observations: dict[int, list[Observation]] = {}
        self.candidates: set[int] = set()
        self.estimates: dict[int, Estimate] = {}
        self.cleared: set[int] = set()
        self.log: list[str] = []

    def say(self, message: str) -> None:
        line = f"[virtual={self.virtual_time:.2f}s] {message}"
        print(line)
        self.log.append(line)

    def remaining_real_time(self) -> float:
        return self.real_limit - (time.monotonic() - self.real_start)

    def accept(self, response: dict | None) -> bool:
        if response is None or response.get("accepted") is not True:
            if response is not None:
                self.say(f"请求未接受：{response}")
            return False
        self.virtual_time = float(response.get("virtual_time_s", self.virtual_time))
        return True

    def save_measurement(self, position: tuple[float, float], channel: int, response: dict) -> str:
        self.position = position
        self.current_channel = channel
        result = response.get("measure_result", "")
        if result == "direction" and isinstance(response.get("svd_deg"), (int, float)):
            self.observations.setdefault(channel, []).append(
                Observation(position, float(response["svd_deg"]))
            )
            self.candidates.add(channel)
        return result

    def scan_point(self, position: tuple[float, float], channels: tuple[int, ...]) -> None:
        for channel in channels:
            if self.remaining_real_time() <= TIME_RESERVE:
                return
            response = self.client.measure(position, channel)
            if not self.accept(response):
                continue
            result = self.save_measurement(position, channel, response)
            if result == "direction":
                self.say(f"频道 {channel} 检测到信号，观测数={len(self.observations[channel])}")
            elif result == "near":
                self.say(f"频道 {channel} 距离过近，直接清除")
                self.try_clear(channel, position)

    def initial_scan(self) -> None:
        anchors = [
            (0.0, 0.0),
            (RING_RADIUS, 0.0),
            (-RING_RADIUS / 2.0, math.sqrt(3.0) * RING_RADIUS / 2.0),
            (-RING_RADIUS / 2.0, -math.sqrt(3.0) * RING_RADIUS / 2.0),
        ]
        for index, point in enumerate(anchors, 1):
            self.say(f"扫描第 {index}/{len(anchors)} 个检测点 {point}")
            self.scan_point(point, CHANNELS)
        self.say(f"初始扫描完成，候选频道：{sorted(self.candidates)}")

    def add_second_observation(self, channel: int) -> None:
        observations = self.observations.get(channel, [])
        if len(observations) >= 2 or not observations:
            return
        first = observations[0]
        angle = math.radians(first.angle)
        point = (
            first.position[0] + 600.0 * math.cos(angle) - 300.0 * math.sin(angle),
            first.position[1] + 600.0 * math.sin(angle) + 300.0 * math.cos(angle),
        )
        point = inside_arena(point)
        response = self.client.measure(point, channel)
        if self.accept(response):
            result = self.save_measurement(point, channel, response)
            self.say(f"频道 {channel} 第二检测点结果：{result}")

    def build_estimate(self, channel: int) -> Estimate | None:
        observations = self.observations.get(channel, [])
        if len(observations) < 2:
            return None
        measurements = [
            (item.position[0], item.position[1], item.angle)
            for item in observations
        ]
        try:
            vertices, _, _, _ = locate_region(measurements)
        except (ValueError, ZeroDivisionError):
            return None
        if len(vertices) < 3:
            return None
        center = inside_arena(centroid(vertices))
        diameter = 2.0 * max(distance(center, vertex) for vertex in vertices)
        return Estimate(channel, center, diameter)

    def update_estimates(self) -> None:
        for channel in sorted(self.candidates - self.cleared):
            self.add_second_observation(channel)
            estimate = self.build_estimate(channel)
            if estimate is not None:
                self.estimates[channel] = estimate
                self.say(
                    f"频道 {channel} 定位到 ({estimate.position[0]:.1f}, "
                    f"{estimate.position[1]:.1f})，区域直径约 {estimate.diameter:.1f} 米"
                )

    def try_clear(self, channel: int, position: tuple[float, float]) -> bool:
        response = self.client.clear(position, channel)
        if not self.accept(response):
            return False
        self.position = position
        if response.get("clear_result") == "success":
            self.cleared.add(channel)
            self.say(f"频道 {channel} 清除成功，累计 {len(self.cleared)} 个")
            return True
        self.say(f"频道 {channel} 清除落空")
        return False

    def refine_and_clear(self, estimate: Estimate) -> bool:
        if self.try_clear(estimate.channel, estimate.position):
            return True
        for round_number in range(MAX_REFINE_ROUNDS):
            radius = 40.0 / (round_number + 1)
            for degree in (0.0, 90.0, 180.0, 270.0):
                if self.remaining_real_time() <= TIME_RESERVE:
                    return False
                angle = math.radians(degree)
                probe = inside_arena(
                    (self.position[0] + radius * math.cos(angle),
                     self.position[1] + radius * math.sin(angle))
                )
                response = self.client.measure(probe, estimate.channel)
                if not self.accept(response):
                    continue
                result = self.save_measurement(probe, estimate.channel, response)
                if result == "near" and self.try_clear(estimate.channel, probe):
                    return True
                if result == "direction":
                    new_estimate = self.build_estimate(estimate.channel)
                    if new_estimate is not None:
                        self.estimates[estimate.channel] = new_estimate
                        if self.try_clear(estimate.channel, new_estimate.position):
                            return True
            self.say(f"频道 {estimate.channel} 完成第 {round_number + 1} 轮局部精修")
        return False

    def run(self) -> None:
        # 倒计时未结束时接口会拒绝或直接重置连接，轮询直至成功
        while True:
            response = self.client.enter()
            if self.accept(response):
                break
            time.sleep(1.0)
        self.real_start = time.monotonic()
        self.real_limit = float(response.get("remaining_real_duration_s", 1200.0))
        self.say(f"进入成功，现实剩余时间 {self.real_limit:.1f} 秒")
        try:
            self.initial_scan()
            self.update_estimates()
            targets = list(self.estimates.values())
            targets.sort(key=lambda item: (item.diameter, distance(self.position, item.position)))
            for target in targets:
                if target.channel in self.cleared:
                    continue
                if self.remaining_real_time() <= TIME_RESERVE:
                    break
                self.say(f"开始巡游清除频道 {target.channel}")
                self.refine_and_clear(target)
        finally:
            response = self.client.exit()
            self.say("主动退出测试" if response and response.get("accepted") else "测试已结束或退出失败")
            self.write_log()
            print(f"成功清除：{len(self.cleared)} 个，频道：{sorted(self.cleared)}")
            print(f"虚拟定位清除时间：{self.virtual_time:.2f} 秒")

    def write_log(self) -> None:
        # 每次运行新建编号日志（robot_model2_logs/ 文件夹内），不覆盖历史
        import os
        base = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "robot_model2_logs")
        os.makedirs(base, exist_ok=True)
        n = 1
        while os.path.exists(os.path.join(base, f"robot_model2_log{n}.txt")):
            n += 1
        log_path = os.path.join(base, f"robot_model2_log{n}.txt")
        with open(log_path, "w", encoding="utf-8") as file:
            file.write("\n".join(self.log))
        print(f"日志已保存: {log_path}")


def main() -> None:
    robot_id = sys.argv[1] if len(sys.argv) > 1 else input("请输入参赛队号：").strip()
    if not robot_id:
        raise SystemExit("参赛队号不能为空")
    print(f"模拟器地址：{BASE_URL}")
    print("等待模拟器接口就绪（倒计时结束后自动开始，无需按键）...")
    Model2Strategy(RobotClient(robot_id)).run()


if __name__ == "__main__":
    main()
