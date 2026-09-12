# -*- coding: utf-8 -*-
"""问题3 第二套（problem3_robot_model2.py，用户实现）离线端到端测试：
本地假模拟器（fake_sim.py），10 个随机场景（与其他两套同种子，便于对比）。"""
import problem3_robot_model2
import fake_sim


class Adapter:
    """Model2Strategy.run 不接受 entered_resp，适配 fake_sim.run_case 接口"""
    def __init__(self, client):
        self.s = problem3_robot_model2.Model2Strategy(client)

    def run(self, entered_resp=None):
        self.s.run()


def run_case(n, seed):
    return fake_sim.run_case(problem3_robot_model2.RobotClient, Adapter, n, seed)


def main():
    cases = [(10, 1), (12, 2), (12, 3), (16, 4), (16, 5), (12, 6),
             (16, 7), (14, 8), (11, 9), (15, 10)]
    print(f"{'场景':<14} {'清除':>6} {'虚拟时间':>10} {'结果':>6}")
    all_pass = True
    for n, seed in cases:
        ok, total, vt, logbuf = run_case(n, seed)
        passed = ok == total
        all_pass &= passed
        print(f"n={n},seed={seed:<4} {ok:>4}/{total} {vt:>10.1f}s "
              f"{'PASS' if passed else 'FAIL'}")
        if not passed:
            print(logbuf[-800:])
    print("=" * 60)
    if all_pass:
        print("全部场景 PASS")
    else:
        raise SystemExit("FAIL: 存在未全部清除的场景")


if __name__ == "__main__":
    main()
