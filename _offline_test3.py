# -*- coding: utf-8 -*-
"""问题3 第三套（robot_model3.py）离线端到端测试：
本地假模拟器（fake_sim.py），10 个随机场景（与第一套同种子，便于对比）。"""
import robot_model3
import fake_sim


def run_case(n, seed):
    return fake_sim.run_case(
        robot_model3.RobotClient,
        lambda client: robot_model3.RobotStrategy3(client, log_to_file=False),
        n, seed)


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
            print(logbuf)
    print("=" * 60)
    if all_pass:
        print("全部场景 PASS：信息收益主动搜索策略可在未知源数量下完整清除")
    else:
        raise SystemExit("FAIL: 存在未全部清除的场景")


if __name__ == "__main__":
    main()
