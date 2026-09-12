# -*- coding: utf-8 -*-
"""离线假模拟器（共享）：按附件2 规则模拟全向干扰源场景，
供 problem3_robot_model1.py 与 problem3_robot_model3.py 的离线端到端测试共用。
默认端口 20262（避开真实模拟器 2026）。
"""
import contextlib
import io
import json
import math
import random
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

DEFAULT_PORT = 20262

sources = []   # (x, y, channel, receive_radius)
state = {
    "pos": (0.0, 0.0),
    "channel": 1,
    "vt": 0.0,
    "cleared": set(),
    "entered": False,
}
_err_cache = {}
_rng = random.Random(42)


def make_sources(n, seed):
    r = random.Random(seed)
    srcs = []
    ch = 1
    while len(srcs) < n:
        rr = math.sqrt(r.random()) * 1750
        a = r.random() * 2 * math.pi
        x, y = rr * math.cos(a), rr * math.sin(a)
        R = r.uniform(1000, 1500)
        srcs.append((x, y, ch, R))
        ch += 1
    # 故意放一个离原点很近的源（考验 near 直接清除路径）
    srcs[0] = (4.0, 3.0, 1, r.uniform(1000, 1500))
    return srcs


def _error(x, y, ch):
    """同一地点同一频道的检测误差固定（附件2：同地点电磁环境固定）"""
    key = (x, y, ch)
    if key not in _err_cache:
        _err_cache[key] = _rng.uniform(-1.0, 1.0)
    return _err_cache[key]


def _source_of(ch):
    for s in sources:
        if s[2] == ch:
            return s
    return None


class Handler(BaseHTTPRequestHandler):
    def do_POST(self):
        n = int(self.headers.get("Content-Length", 0))
        body = json.loads(self.rfile.read(n).decode("utf-8"))
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        resp = self._dispatch(body)
        self.wfile.write(json.dumps(resp).encode("utf-8"))

    def log_message(self, *a):
        pass

    def _dispatch(self, p):
        path = self.path
        pos = p.get("position")
        ch = p.get("channel")
        tx = float(pos["x"]) if pos else state["pos"][0]
        ty = float(pos["y"]) if pos else state["pos"][1]
        move = math.hypot(tx - state["pos"][0], ty - state["pos"][1]) / 5.0

        if path == "/enter":
            state["entered"] = True
            return {"accepted": True, "virtual_time_s": 0.0,
                    "remaining_real_duration_s": 1200}
        if path == "/exit":
            state["entered"] = False
            return {"accepted": True, "virtual_time_s": state["vt"]}

        if not state["entered"]:
            return {"accepted": False, "virtual_time_s": 0.0}

        if path == "/measure":
            switch = 1.0 if (ch != state["channel"]) else 0.0
            state["vt"] += move + 5.0 + switch
            state["pos"] = (tx, ty)
            state["channel"] = ch
            s = _source_of(ch)
            result, svd = "no_signal", None
            if s is not None and ch not in state["cleared"]:
                d = math.hypot(tx - s[0], ty - s[1])
                if d <= s[3]:
                    if d <= 5.0:
                        result = "near"
                    else:
                        true_ang = math.degrees(math.atan2(s[1] - ty, s[0] - tx)) % 360
                        svd = (true_ang + _error(tx, ty, ch)) % 360
                        result = "direction"
            resp = {"accepted": True, "virtual_time_s": state["vt"],
                    "measure_result": result}
            if svd is not None:
                resp["svd_deg"] = svd
            return resp

        if path == "/clear":
            s = _source_of(ch)
            ok = (s is not None and ch not in state["cleared"]
                  and math.hypot(tx - s[0], ty - s[1]) <= 20.0)
            state["vt"] += move + (5.0 if ok else 3.0)
            state["pos"] = (tx, ty)
            if ok:
                state["cleared"].add(ch)
                return {"accepted": True, "virtual_time_s": state["vt"],
                        "clear_result": "success"}
            return {"accepted": True, "virtual_time_s": state["vt"],
                    "clear_result": "no_target_in_range"}

        return {"accepted": False, "virtual_time_s": 0.0}


def reset(n, seed):
    """重置场景并启动假模拟器服务，返回 server 对象"""
    global sources, state, _err_cache, _rng
    sources = make_sources(n, seed)
    state = {"pos": (0.0, 0.0), "channel": 1, "vt": 0.0,
             "cleared": set(), "entered": False}
    _err_cache = {}
    _rng = random.Random(1000 + seed)

    server = ThreadingHTTPServer(("127.0.0.1", DEFAULT_PORT), Handler)
    t = threading.Thread(target=server.serve_forever, daemon=True)
    t.start()
    return server


def run_case(client_cls, make_strategy, n, seed, port=DEFAULT_PORT):
    """跑一个随机场景。client_cls(base_url=..., robot_id=...) 构造客户端；
    make_strategy(client) 构造策略对象。返回 (清除数, 总数, 虚拟时间, 输出日志文本)。"""
    global sources, state, _err_cache, _rng
    sources = make_sources(n, seed)
    state = {"pos": (0.0, 0.0), "channel": 1, "vt": 0.0,
             "cleared": set(), "entered": False}
    _err_cache = {}
    _rng = random.Random(1000 + seed)

    server = ThreadingHTTPServer(("127.0.0.1", port), Handler)
    t = threading.Thread(target=server.serve_forever, daemon=True)
    t.start()
    try:
        client = client_cls(base_url=f"http://127.0.0.1:{port}", robot_id="TEST")
        strategy = make_strategy(client)
        resp = client.enter()
        assert resp and resp["accepted"] is True
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            strategy.run(entered_resp=resp)
        return len(state["cleared"]), n, state["vt"], buf.getvalue()
    finally:
        server.shutdown()
