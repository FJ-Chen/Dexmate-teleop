#!/usr/bin/env python3
"""相机画面进主页面这条链路的免硬件测试:mock 相机 → --pub-view → 镜像渲染。

    .venv/bin/python sim/dev_cam_view.py

三条断言(端口全用 16xxx,与生产隔离):
  1. 镜像收到画面帧,状态行变 ✅ 并带点数;
  2. 场景里出现了 /camera_view 点云,且点坐标已换到 viser 系(z 上,数值在
     偏置位置附近,不是原始毫米值);
  3. 相机进程停掉 2 秒后,状态行要报断流(不能永远显示 ✅)。
"""
from __future__ import annotations

import pathlib
import subprocess
import sys
import time

import numpy as np
import viser

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "scripts"))
from viser_isaac_mirror import Mirror  # noqa: E402

ROOT = pathlib.Path(__file__).resolve().parents[1]
VIEW = "tcp://127.0.0.1:16591"


def main() -> int:
    proc = subprocess.Popen(
        [str(ROOT / ".venv/bin/python"), "-u", "scripts/kinect_pointcloud.py",
         "--source", "mock", "--record-session", "tcp://127.0.0.1:16584",
         "--pub-view", "tcp://*:16591"],
        cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    server = viser.ViserServer(port=8197, verbose=False)
    mirror = Mirror(server, sub="tcp://127.0.0.1:16583",
                    control="tcp://127.0.0.1:16585",
                    hand_right="", hand_left="", pico="",
                    robot_state="", cam_view=VIEW)
    bad = 0
    try:
        t0 = time.time()
        while time.time() - t0 < 15 and mirror.n_cam < 3:
            mirror.tick()
            time.sleep(0.02)
        ok = mirror.n_cam >= 3 and "✅" in mirror.g_cam.value
        print(f"[1] 镜像收到画面 {mirror.n_cam} 帧,状态行「{mirror.g_cam.value}」"
              f"  {'✅' if ok else '❌ 没收到'}")
        bad += 0 if ok else 1

        pc = mirror._cam_pc
        if pc is None:
            print("[2] ❌ 场景里没有点云")
            bad += 1
        else:
            pts = np.asarray(pc.points)
            near = np.linalg.norm(pts.mean(axis=0) - mirror._CAM_OFFSET) < 3.0
            not_mm = np.abs(pts).max() < 50.0
            ok = len(pts) > 100 and near and not_mm
            print(f"[2] 点云 {len(pts)} 点,质心距偏置位 "
                  f"{np.linalg.norm(pts.mean(axis=0) - mirror._CAM_OFFSET):.2f}m,"
                  f"坐标量级 {np.abs(pts).max():.1f}"
                  f"  {'✅ 已换系' if ok else '❌ 坐标不对(毫米当米?没换系?)'}")
            bad += 0 if ok else 1
    finally:
        proc.terminate()
        proc.wait(timeout=10)

    t0 = time.time()
    while time.time() - t0 < 4:
        mirror.tick()
        time.sleep(0.05)
    ok = "断流" in mirror.g_cam.value
    print(f"[3] 相机进程停止后状态行「{mirror.g_cam.value}」"
          f"  {'✅ 报了断流' if ok else '❌ 还在装 ✅'}")
    bad += 0 if ok else 1

    print("✅ 全部通过" if bad == 0 else f"❌ {bad} 项失败")
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
