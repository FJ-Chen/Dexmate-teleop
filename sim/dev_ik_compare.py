#!/usr/bin/env python3
"""Pink 与 cuRobo 两个 IK 后端的离线对比台。不依赖 Isaac。

    cd <仓库根目录> && .venv-isaac/bin/python sim/dev_ik_compare.py
    .venv-isaac/bin/python sim/dev_ik_compare.py --frames 800   # 截短真实素材

必须用 .venv-isaac(cuRobo 装在那里);.venv 没有 CUDA torch,起不来。

三份素材,两个后端同样喂,逐项打表。数字如实:cuRobo 更差也照实报。

  1. synthetic  可达轨迹:双手绕 HOME 画圆(半径 8cm)并同时摆动掌向
     (±25°,在腕量程内)。量的是正常跟随的精度与平滑。
  2. overrange  超量程(沿用 dev_relax_latch 的场景):右腕命令绕 EE 红色轴
     拧 150°(超出 j6 ±80° 量程)保持 6 秒,然后回到可达姿态 3 秒。量的是
     顶限位期间的行为(贴限位占比)与**需求回量程后能不能回来**(这正是
     用户抱怨的反面,两个后端都必须能回来,回不来直接判失败)。
  3. playlist   真实素材:logs/regress/base1_playlist_all13.csv 的 cmd 关节角
     (13 段拼接的签收回归素材)用 FK 反造 6D 腕目标流。不重写映射,拿到的
     就是真素材的目标分布(含贴近限位的姿态)。

指标(与回归台/dev_limit_lock 同一把尺):
  - 腕位置误差 mean/max [mm]、朝向误差 mean/max [deg](对解出的 q 做 FK,
    与命令目标比;不是求解器自报);
  - 贴限位帧占比 [%](14 关节任一距 URDF 限位 1° 以内,dev_limit_lock 同款
    算法)与最长连续 [s];
  - 逐帧关节跳变:>5° 帧占比 [%] 与 p99 [deg](回归台 jitter 同款);
  - 单步 solve() 耗时 p50/p95 [ms]。

每个会打印的数字先过「尺子自检」:拿构造出来的已知答案(贴限位序列、
含 10° 跳变的序列、恒定目标)验金属尺本身,尺子不对直接退出 —— 这个仓库
为「度量本身就错」付过学费(verify-the-metric-first)。

Pink 侧配置 = 当前控制台签收默认(ori_cost 0.5、substeps 3、
relax_at_limit 0.01、relax_margin 3.0);cuRobo 侧 = CuroboVegaIK 默认。
"""
from __future__ import annotations

import argparse
import csv
import pathlib
import sys
import time

import numpy as np
import pinocchio as pin

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from pink_vega_ik import EE_FRAME, PinkVegaIK  # noqa: E402

ROOT = pathlib.Path(__file__).resolve().parents[1]
CSV_REAL = ROOT / "logs/regress/base1_playlist_all13.csv"
HZ = 50.0
NEAR_DEG = 1.0      # 贴限位判据,与 dev_limit_lock 一致
JUMP_DEG = 5.0      # 跳变判据,与回归台一致


# ---------------------------------------------------------------- 尺子 ----
def pinned_stats(q: np.ndarray, lo: np.ndarray, hi: np.ndarray) -> dict:
    near = np.radians(NEAR_DEG)
    pinned = (q <= lo + near) | (q >= hi - near)
    any_p = pinned.any(axis=1)
    best = cur = 0
    for v in any_p:
        cur = cur + 1 if v else 0
        best = max(best, cur)
    return {"pct": float(100 * any_p.mean()), "longest_s": best / HZ,
            "joint_frames": int(pinned.sum())}


def jump_stats(q: np.ndarray) -> dict:
    d = np.degrees(np.abs(np.diff(q, axis=0)))
    worst = d.max(axis=1) if len(d) else np.zeros(1)
    return {"pct5": float(100 * (worst > JUMP_DEG).mean()),
            "p99": float(np.percentile(worst, 99))}


def ang_deg(Ra, Rb) -> float:
    return float(np.degrees(np.linalg.norm(pin.log3(Ra.T @ Rb))))


def ruler_selfcheck(lo: np.ndarray, hi: np.ndarray) -> int:
    """先拿已知答案验尺子。任何一条不符,退出码非零。"""
    bad = 0
    # 贴限位:一列恒在下限上的序列必须报 100%,离限位远的必须报 0%
    q = np.tile((lo + hi) / 2, (100, 1))
    r0 = pinned_stats(q, lo, hi)
    q2 = q.copy()
    q2[:, 5] = lo[5]                    # L_arm_j6 压在下限
    r1 = pinned_stats(q2, lo, hi)
    ok = r0["pct"] == 0.0 and r1["pct"] == 100.0 and r1["longest_s"] == 2.0
    print(f"[尺] 贴限位:量程中央 {r0['pct']:.0f}%,压死下限 {r1['pct']:.0f}%"
          f"(最长 {r1['longest_s']:.1f}s)  {'✅' if ok else '❌ 尺子坏了'}")
    bad += 0 if ok else 1
    # 跳变:静止序列 0%;每 25 帧插一个 10° 台阶(4% 的帧)后,占比要
    # 抓到约 4%,p99 也必须被顶到台阶量级(2% < 4%,p99 落在台阶上)。
    j0 = jump_stats(q)
    q3 = q.copy()
    for k in range(25, 100, 25):
        q3[k:, 3] += np.radians(10.0) * (1 if (k // 25) % 2 else -1)
    j1 = jump_stats(q3)
    ok = j0["pct5"] == 0.0 and 2.0 < j1["pct5"] < 6.0 and j1["p99"] > 5.0
    print(f"[尺] 跳变:静止 {j0['pct5']:.1f}%,4% 帧含 10° 台阶时报 "
          f"{j1['pct5']:.1f}% / p99 {j1['p99']:.1f}°"
          f"  {'✅' if ok else '❌ 尺子坏了'}")
    bad += 0 if ok else 1
    return bad


# ------------------------------------------------------------- 素材 ----
def fk_pose(model, data, q, frame):
    pin.framesForwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    M = data.oMf[model.getFrameId(frame)]
    return M.translation.copy(), M.rotation.copy()


def quat_wxyz(R) -> list:
    q = pin.Quaternion(R)
    return [q.w, q.x, q.y, q.z]


def material_synthetic(model, data, q_home) -> list:
    """双手画圆 + 掌向摆动(±25°,腕量程内),10 秒 500 帧。"""
    home = {h: fk_pose(model, data, q_home, EE_FRAME[h]) for h in EE_FRAME}
    frames = []
    for i in range(500):
        t = i / HZ
        ph = 2 * np.pi * t / 10.0
        tgt = {}
        for h in EE_FRAME:
            p0, R0 = home[h]
            p = p0 + np.array([0.04 * np.sin(ph),
                               0.08 * (np.cos(ph) - 1.0),
                               0.08 * np.sin(ph)])
            R = R0 @ pin.exp3(np.array([0.0, np.radians(25.0) * np.sin(ph),
                                        0.0]))
            tgt[h] = (p, quat_wxyz(R))
        frames.append(tgt)
    return frames


def material_overrange(model, data, q_home) -> list:
    """右腕绕 EE 红色轴拧 150°(超 j6 量程)保持 6 秒,回可达 3 秒。
    左手钉在 HOME。场景与 dev_relax_latch 第 4 条相同。"""
    home = {h: fk_pose(model, data, q_home, EE_FRAME[h]) for h in EE_FRAME}
    p0, R0 = home["right"]
    axis = R0[:, 0]
    R_bad = pin.exp3(np.radians(150.0) * axis) @ R0
    frames = []
    for i in range(300):
        frames.append({"right": (p0, quat_wxyz(R_bad)),
                       "left": (home["left"][0], quat_wxyz(home["left"][1]))})
    for i in range(150):
        frames.append({"right": (p0, quat_wxyz(R0)),
                       "left": (home["left"][0], quat_wxyz(home["left"][1]))})
    return frames


def material_from_joint_csv(model, data, pin_names, csv_path,
                            limit_frames=None) -> list:
    """把关节 CSV(回归台格式,cmd_<关节名> 列,50Hz)FK 成 6D 腕目标流。

    可复用:换一份素材 = 换 csv_path 重跑(S3 就是这么用)。返回与合成
    素材同构的帧列表 [{hand: (pos, quat_wxyz)}, ...]。"""
    with open(csv_path) as fh:
        rows = list(csv.DictReader(fh))
    if limit_frames:
        rows = rows[:limit_frames]
    cols = {n: f"cmd_{n}" for n in pin_names}
    frames = []
    for r in rows:
        q = np.array([float(r[cols[n]]) for n in pin_names])
        tgt = {}
        for h in EE_FRAME:
            p, R = fk_pose(model, data, q, EE_FRAME[h])
            tgt[h] = (p.copy(), quat_wxyz(R))
        frames.append(tgt)
    return frames


# ------------------------------------------------------------- 跑一遍 ----
def run_backend(ik, frames, model, data) -> dict:
    """喂同一份目标流,返回全部指标。ik 需要 set_target/solve/reset_home。"""
    lo = model.lowerPositionLimit
    hi = model.upperPositionLimit
    ik.reset_home()
    qs, times = [], []
    perr, oerr = [], []
    for tgt in frames:
        for h, (p, quat) in tgt.items():
            ik.set_target(h, p, quat)
        t0 = time.perf_counter()
        q = ik.solve()
        times.append(time.perf_counter() - t0)
        qs.append(q.copy())
        for h, (p, quat) in tgt.items():
            pe, Re = fk_pose(model, data, q, EE_FRAME[h])
            perr.append(np.linalg.norm(pe - p) * 1000)
            R_tgt = _rot_from_wxyz(quat)
            oerr.append(ang_deg(Re, R_tgt))
    qs = np.array(qs)
    times = np.array(times) * 1000
    perr, oerr = np.array(perr), np.array(oerr)
    return {
        "q": qs,
        "位置误差mm": (float(perr.mean()), float(perr.max())),
        "朝向误差deg": (float(oerr.mean()), float(oerr.max())),
        "贴限位": pinned_stats(qs, lo, hi),
        "跳变": jump_stats(qs),
        "耗时ms": (float(np.percentile(times, 50)),
                 float(np.percentile(times, 95))),
    }


def _rot_from_wxyz(quat) -> np.ndarray:
    w, x, y, z = quat
    return pin.Quaternion(w, x, y, z).toRotationMatrix()


def print_table(name: str, res: dict):
    print(f"\n== {name} ==")
    hdr = (f"{'后端':8s} {'位置误差mm(均/最大)':>22s} {'朝向误差deg(均/最大)':>22s} "
           f"{'贴限位%':>8s} {'最长s':>6s} {'跳变>5°%':>9s} {'跳变p99°':>9s} "
           f"{'耗时ms(p50/p95)':>16s}")
    print(hdr)
    for backend, r in res.items():
        print(f"{backend:8s} "
              f"{r['位置误差mm'][0]:10.1f}/{r['位置误差mm'][1]:8.1f}   "
              f"{r['朝向误差deg'][0]:10.1f}/{r['朝向误差deg'][1]:8.1f}   "
              f"{r['贴限位']['pct']:8.2f} {r['贴限位']['longest_s']:6.1f} "
              f"{r['跳变']['pct5']:9.2f} {r['跳变']['p99']:9.2f} "
              f"{r['耗时ms'][0]:8.2f}/{r['耗时ms'][1]:6.2f}")


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--frames", type=int, default=0,
                    help="截短真实素材到前 N 帧(0 = 全部 6889 帧)")
    ap.add_argument("--skip-real", action="store_true",
                    help="跳过真实素材(快速冒烟)")
    args = ap.parse_args()

    bad = 0

    # 账本模型(与两个后端共用同一 URDF 与关节序)
    ref = PinkVegaIK(dt=1.0 / HZ)
    model, data = ref.model, ref.data
    pin_names = ref.pin_names
    q_home = ref.q_home.copy()
    del ref

    bad += ruler_selfcheck(model.lowerPositionLimit.copy(),
                           model.upperPositionLimit.copy())
    if bad:
        print("尺子不过,后面的数字没有意义,退出。")
        return 1

    # 素材 3 的入口自检:CSV 第一行就是 HOME,FK 出来的腕位姿必须与
    # HOME FK 一致(<1mm),否则是列名/关节序读错了。
    if not args.skip_real:
        if not CSV_REAL.exists():
            print(f"❌ 缺素材 {CSV_REAL}")
            return 1
        with open(CSV_REAL) as fh:
            row0 = next(csv.DictReader(fh))
        q0 = np.array([float(row0[f"cmd_{n}"]) for n in pin_names])
        p_csv, _ = fk_pose(model, data, q0, EE_FRAME["right"])
        p_home, _ = fk_pose(model, data, q_home, EE_FRAME["right"])
        d0 = np.linalg.norm(p_csv - p_home) * 1000
        ok = d0 < 1.0
        print(f"[尺] 真实素材首帧 FK 距 HOME {d0:.2f}mm"
              f"  {'✅' if ok else '❌ CSV 解析错了'}")
        bad += 0 if ok else 1
        if bad:
            return 1

    # 两个后端。Pink 用控制台签收默认;cuRobo 用自身默认。
    print("\n构建后端 ...")
    t0 = time.time()
    pink = PinkVegaIK(dt=1.0 / HZ, orientation_cost=0.5, substeps=3,
                      relax_at_limit=0.01, relax_margin_deg=3.0)
    t_pink = time.time() - t0
    t0 = time.time()
    from curobo_vega_ik import CuroboVegaIK
    curobo = CuroboVegaIK(dt=1.0 / HZ)
    t_cur = time.time() - t0
    print(f"构建耗时:pink {t_pink:.1f}s  curobo {t_cur:.1f}s(含预热首解;"
          "冷 warp 缓存首次约 17s)")
    backends = {"pink": pink, "curobo": curobo}

    # ---- 稳态耗时基准 ---------------------------------------------------
    # 预热首解已吃在构建里,这里量的是 50Hz 控制环真正面对的稳态单步耗时
    # (cuRobo 侧含 ZMQ 往返,即部署路径)。这是 S6 架构选型的硬门槛数:
    # 环给 IK 的预算约 5ms。数字如实打,不达标不藏。
    print("\n== 稳态单步耗时(目标=HOME 位姿保持,300 步) ==")
    for n, ik in backends.items():
        ik.reset_home()
        home_tgt = {h: fk_pose(model, data, q_home, EE_FRAME[h])
                    for h in EE_FRAME}
        ts = []
        for _ in range(300):
            for h, (p, R) in home_tgt.items():
                ik.set_target(h, p, quat_wxyz(R))
            t0 = time.perf_counter()
            ik.solve()
            ts.append(time.perf_counter() - t0)
        ts = np.array(ts) * 1000
        p50, p95 = np.percentile(ts, 50), np.percentile(ts, 95)
        # 自检:稳态 p95 必须进 50Hz 的控制周期(20ms),否则这个后端在
        # 该机器上根本追不上环,后面的精度对比失去意义。
        ok = p95 < 20.0
        print(f"  {n:8s} p50 {p50:6.2f}ms  p95 {p95:6.2f}ms  "
              f"p99 {np.percentile(ts, 99):6.2f}ms  max {ts.max():6.2f}ms"
              f"  {'✅ < 20ms 周期' if ok else '❌ 追不上 50Hz 环'}"
              + ("" if p50 < 5.0 else "  ⚠ 超 5ms 预算"))
        bad += 0 if ok else 1

    # ---- 素材 1:可达合成轨迹 -------------------------------------------
    frames = material_synthetic(model, data, q_home)
    res1 = {n: run_backend(ik, frames, model, data)
            for n, ik in backends.items()}
    print_table("素材 1:可达合成轨迹(圆弧 + 掌向摆动,10 秒)", res1)
    for n, r in res1.items():
        ok = r["位置误差mm"][0] < 5.0
        print(f"  [自检] {n} 可达素材位置误差均值 {r['位置误差mm'][0]:.1f}mm < 5mm"
              f"  {'✅' if ok else '❌ 连可达目标都跟不住'}")
        bad += 0 if ok else 1

    # ---- 素材 2:超量程 + 恢复 ------------------------------------------
    frames = material_overrange(model, data, q_home)
    res2 = {}
    for n, ik in backends.items():
        res2[n] = run_backend(ik, frames, model, data)
        # 恢复判据:需求回到可达 3 秒后,右腕朝向误差 < 5°、全臂距限位 > 1°
        q_end = res2[n]["q"][-1]
        _, R_end = fk_pose(model, data, q_end, EE_FRAME["right"])
        _, R_home = fk_pose(model, data, q_home, EE_FRAME["right"])
        e_end = ang_deg(R_end, R_home)
        slack_end = float(np.degrees(np.minimum(
            q_end - model.lowerPositionLimit,
            model.upperPositionLimit - q_end).min()))
        res2[n]["恢复"] = (e_end, slack_end)
    print_table("素材 2:超量程拧腕 6 秒 + 回可达 3 秒(dev_relax_latch 场景)", res2)
    for n, r in res2.items():
        e_end, slack_end = r["恢复"]
        ok = e_end < 5.0 and slack_end > 1.0
        print(f"  [自检] {n} 需求回量程 3 秒后:朝向误差 {e_end:.1f}°"
              f"  全臂距限位最近 {slack_end:.1f}°"
              f"  {'✅ 能回来' if ok else '❌ 回不来 —— 顶限位锁死'}")
        bad += 0 if ok else 1

    # ---- 素材 3:真实素材 -----------------------------------------------
    if not args.skip_real:
        frames = material_from_joint_csv(model, data, pin_names, CSV_REAL,
                                         args.frames or None)
        res3 = {n: run_backend(ik, frames, model, data)
                for n, ik in backends.items()}
        print_table(f"素材 3:真实素材 playlist_all13({len(frames)} 帧,"
                    "FK 反造目标)", res3)
        # 自检:素材本身由 pink 生成(cmd 已带防护),pink 重解不应偏出
        # 太多;这里只挡「完全失控」(均值 < 30mm),对比数字照表如实读。
        for n, r in res3.items():
            ok = r["位置误差mm"][0] < 30.0
            print(f"  [自检] {n} 真实素材位置误差均值 {r['位置误差mm'][0]:.1f}mm"
                  f" < 30mm  {'✅' if ok else '❌ 失控'}")
            bad += 0 if ok else 1

    print(f"\n{'✅ 全部通过' if bad == 0 else f'❌ {bad} 项失败'}")
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
