# 头部相机内参 / 位姿链 / RGB-D → 点云

> 2026-08-12 整理,供之后回看。
> 目标场景:**打印 ArUco 贴在螺母上 → 由头部相机估出螺母位姿 → 换算到机器人 base 系**。
> 本文只记录**代码里实际存在的东西**和**已经核对过的数字**;推测的部分都标了「⚠ 推测」。

---

## 0. V2AP-demo 在哪

`V2AP-demo` **不在本仓库里**,是另一组人的参考实现,只读。

| 仓库 | 角色 |
|---|---|
| `<仓库根目录>`(本仓库) | 我们的遥操作代码。Kinect 点云那条线在这里。 |
| `<V2AP-demo 根目录>` | 参考实现。相机 / 位姿 / 抓取那条线全在这里。本机上位于本仓库的**同级目录**(`ViTac-Assembly/` 的上一层)。 |

下文凡是 `demo/…`、`camera/…`、`dexmate/…` 开头的路径,都是 **V2AP-demo 里的**;
`scripts/…`、`sim/…` 开头的是**本仓库的**。

---

## 1. 我要找的那个脚本:`demo/phase2/capture_session.py`

它一次性采下「机器人状态 + 相机位姿 + 内参 + RGB-D」,打包成一个 session 目录。
**唯独没有物体位姿** —— V2AP 的物体位姿是另一台 GPU 机器(Titan)上跑 FoundationPose 出来的。

| 需要的东西 | 在哪 | 怎么来的 |
|---|---|---|
| base ← camera 外参 | `demo/phase2/extrinsics.py:29` `compute_T_base_cam()` | 用采集瞬间的关节角,在 `vega_1.urdf` 上跑 pinocchio 正运动学。`p_base = T_base_cam @ p_cam`。**没有手眼标定**,纯 URDF FK。 |
| 相机内参 | `demo/phase2/intrinsics.py:64` + `calib/head_zed_left_intrinsics.json` | 优先读标定文件;否则从 `get_camera_info()` 里递归找 fx/fy/cx/cy;再否则用标称值兜底。 |
| RGB-D | `demo/phase2/robot_capture.py:97` 或 `camera/zed_stream_receiver.py:48` | 两条链路,见 §3。 |
| **物体(螺母)位姿** | ❌ **不在这里** | FoundationPose 出 `T_cam_mesh`,再合成 `T_base_mesh = T_base_cam @ T_cam_mesh`(`demo/phase2/README.md:471-473`,消费点在 `retarget.py:35`)。 |

**换成 ArUco 要改的就是最后一行**:把你 solvePnP 出来的 `T_cam_obj` 顶替 `T_cam_mesh`,合成公式和坐标系约定原样不动。

> **V2AP 里没有任何 ArUco 位姿估计代码。** 全仓库唯一的 ArUco 相关文件是
> `camera/calibrate_charuco.py`,那是**标内参**用的 —— 它调了 `detectMarkers`(:177),
> 但从没调过 `estimatePoseSingleMarkers` / `solvePnP`。这一步要自己写。

---

## 2. 内参(内参只有这一份)

全仓库范围搜过一遍:**存下来的相机内参只有这两个文件**,内容一致(已逐位核对)。

- `demo/phase2/calib/head_zed_left_intrinsics.json`
- `demo/phase2/calib/K.npy`(float64 (3,3),给 FoundationPose / UCB 脚本用的副本)

`camera/` 目录本身**一个内参都不存** —— 它只负责**产出**(`calibrate_charuco.py` 写到上面那两个文件)和**传输**(几个 receiver 全是纯搬运,没有 K)。

```python
import numpy as np
# frame: zed_left_camera   resolution: 640 x 360   (ZED X Mini,左目)
K = np.array([[190.53290846068992,   0.0,               317.3777289060325 ],
              [  0.0,              172.37719692126257, 184.40564021189294],
              [  0.0,                0.0,                1.0             ]])
dist = np.array([0.0042244258666400605, -0.001071605136834047,
                 -0.002977924409797224, -0.0003299271513460427,
                  0.00034711048799465437])   # plumb_bob

ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, K, dist,
                              flags=cv2.SOLVEPNP_IPPE_SQUARE)   # 螺母上的 ArUco
```

标定来源(文件里自带的记录):`opencv_aruco_charuco`,17 视图,重投影 RMS **0.2247 px**,
calib.io 14×9 板 / 20 mm 方格 / 15 mm marker / `DICT_5X5_100`,
检测在 1920×1080、再由 `calibrate_charuco.py:257` `_scale_intrinsics_to_size` 等比缩到 640×360,
生成时间 2026-06-02。

### ⚠ 用之前必须先解决的两件事

**(1) 焦距在 V2AP 内部自相矛盾,差 1.8 倍。**

| 来源 | fx | 等效 HFOV @640 |
|---|---|---|
| 标定文件(上面这份) | 190.53 | 118.5° |
| 标称兜底 `constants.py:40` | 350.0 | 84.9° |

`solvePnP` 的平移量**与 fx 成正比**:fx 错 1.8 倍,0.5 m 处的螺母会被算成 0.9 m。
另外 `fx/fy = 1.1053`,方像素传感器在等比缩放下不该出现这个比值。
(⚠ 推测:和 ZED X Mini 原生 1920×1200 被压成 1920×1080 的竖向压缩量 0.9 吻合,但仓库里没有任何一句话证实这点。)
畸变系数接近 0 是**正常的** —— ZED SDK 默认输出已校正图像。

**动作**:打印标签贴在**卷尺量准的距离**上,跑一次 solvePnP,看返回距离对不对。
这是整条链上唯一真值来自物理世界的检查。设备出厂标定在本仓库里**没有任何副本**。

**(2) `zed_left_camera` 和 `zed_depth_frame` 不是同一个点。**

`dexmate/dexmate-urdf/robots/humanoid/vega_1/vega_1.urdf`,两个 link 都挂在 `head_l3` 上:

| link | origin xyz | rpy | 行号 |
|---|---|---|---|
| `zed_depth_frame` | `0.025  0.023 0.0489` | `-1.57079 0 -1.57079` | :331 |
| `zed_left_camera` | `0.0365 0.023 0.0489` | `-1.57079 0 -1.57079` | :339 |

差 **11.5 mm**(沿 head_l3 的 x)。代码里用的是 `zed_left_camera`(`constants.py:24`),
但 URDF 紧挨着的注释写的是:左/右相机帧**是给仿真渲染用的**,
「**the depth frame is used for the real robot transformation**」(:336)。

**结论**:纯靠 RGB 图做 ArUco → 用 `zed_left_camera` 是对的;一旦要融合深度,记住这 11.5 mm。

---

## 3. 相机本体与两条取流链路

**硬件**:ZED X Mini 双目,装在头部 `head_l3` 上(`camera/head_camera_receiver.py:21` 明写型号)。

| 链路 | 代码 | 细节 |
|---|---|---|
| **`zed_stream`(V2AP 默认)** | `camera/zed_stream_receiver.py:48` | TCP 自定义协议 `ZS01`,默认 `192.168.50.22:30000`(dexmate-nano)。左目 JPEG + 深度 **float32 米**(LZ4 压缩),统一缩放到 **640×360**。服务端起 `zed_streamer` 时**不能加 `--no-depth`**。 |
| **zenoh / dexsensor** | `robot.sensors.head_camera.get_obs(obs_keys=["left_rgb","depth"])` | 见 `demo/phase2/robot_capture.py:97`。外加 `get_camera_info(force_refresh=True)`(`robot_capture.py:165`)。 |

**深度单位**:这两条链路全程 **float32 米**。只有写 FoundationPose 场景目录时才转成 uint16 毫米(`constants.py:22`)。
**本仓库的 Kinect 那条线相反,原生是 int16 毫米** —— 跨仓库拷代码时这是最容易静默错 1000 倍的地方。

> ⚠ 关于 `get_camera_info()`:`zed_camera.py:379` 的 docstring 散文部分说包含
> 「calibration parameters」,但它**列举的返回键里没有 K**
> (只有 type / camera_id / status / model / serial_number / firmware_version /
> actual / configured / streams / statistics)。而 `intrinsics.py:21` 是**递归地**
> 在那个 dict 里到处找 fx/fy/cx/cy,找不到就用 350 兜底并打 warning。
> 这个防御性写法 + 他们干脆自己手标了一遍 charuco,说明**这个服务多半没可靠地返回过内参**。

---

## 4. RGB-D → 点云:仓库里现有的三种做法

### A. 从深度 + 内参自己重建(V2AP,唯一真正的「方法」)

`demo/phase2/table_height.py:14` `estimate_table_height_m_from_depth()`,核心就 5 行(:37-41):

```python
fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
x_cam = (us - cx) * zs / fx          # us, vs = 像素坐标;zs = 该像素深度(米)
y_cam = (vs - cy) * zs / fy
p_cam = np.stack([x_cam, y_cam, zs, np.ones_like(zs)], axis=1)   # (N,4) 齐次
p_base = (T_base_cam @ p_cam.T).T[:, :3]                          # 直接进 base 系
```

有效性筛选在 :30 —— `np.isfinite(patch) & (patch > 0.05) & (patch < 3.0)`。
**深度无效值是 NaN/inf/0**(`constants.py:30`),不是 k4a 那种纯 0。

**⚠ 注意 V2AP 从来没有建过整帧彩色点云**。这段只在一个 ROI(下方中间 55%~100% 行、25%~75% 列)
上跑,用途是量桌面高度;`validate_input.py:171` 拿它做采集后自检(base 系 z 中位数应 ≈ 桌高 ±5 cm)。
要整帧点云,把 ROI 换成全图、再把 `rgb.reshape(-1,3)` 按同一个 `valid` 掩码取色即可 —— 逻辑不用改。

反向投影(3D→像素)在 `demo/phase2/visualize_grasp.py:58` `_project_cam()`。

### B. 直接收 ZED 自己算好的点云(不用重建)

`camera/nvjpeg_zed_stream_client.py`:同一个 `ZS01` 流里除了 RGB / 深度,还有一路 **`TYPE_PC = 3`**(:31)。

- 载荷是 **(N, 4) float32**,即 ZED `MEASURE::XYZRGBA` / `sl::float4` 的原始布局(:55-56)。
- 前三个 float 是 xyz(米),**第 4 个 float 里按位打包了 RGBA**;
  解包函数 `xyzrgba_w_to_rgb_open3d()`(:70)照抄 ZED GLViewer 的 shader:
  `c = floatBitsToUint(w); R = c & 0xFF; G = (c>>8) & 0xFF; B = (c>>16) & 0xFF`。
- 用法见 :285,直接喂 Open3D(`--viz-pc`)。

**两个坑**:① 服务端起 `zed_streamer` 时不能加 `--no-pc`;
② `capture_session.py` 用的那个 `ZedStreamRgbdReceiver`(`zed_stream_receiver.py`)
**只解 `TYPE_LEFT` 和 `TYPE_DEPTH`,完全忽略 `TYPE_PC`** —— 想要这路点云得用
`nvjpeg_zed_stream_client.py` 那个 client,或者自己给 receiver 补一个分支。

### C. 本仓库的 Kinect 线:`scripts/kinect_pointcloud.py`

和 ZED 无关,是我们自己从零写的 Azure Kinect(k4a)那条线。可直接参考的部分:

| 位置 | 内容 |
|---|---|
| `:200` `cloud_from_arrays()` | (H,W,3) int16 毫米 + BGRA → 米制彩色点云;丢弃 `z == 0`(k4a 的无效值),BGR→RGB 翻转在 :217 |
| `:215` | **毫米 → 米的唯一转换点**,`/1000.0` |
| `:471-476` `_synth_depth()` | 和 §4-A 一模一样的针孔反投影,用来造「答案已知」的自检深度图 |
| `:291` `capture_clouds()` | 真机路径:重建由 **SDK 用设备出厂标定**完成(`capture.depth_point_cloud` / `transformed_depth_point_cloud`),脚本自己不算 —— `--align depth\|color` 决定点云对齐到哪一侧分辨率(:327/:330) |
| `:221` / `:254` / `:282` | 体素降采样(纯 numpy)/ 二进制 PLY 写出 / npz 写出 |
| `:396` `live_view()` | viser 浏览器实时显示(端口 8087),`--source mock` 无硬件也能验可视化 |
| `:549` `record_session()` | 按录制开关落盘,默认降到 1 cm 体素 / 10 Hz(整帧不降采样是 166 MB/s) |

自检:`--self-check`(无需相机,已 PASSED,并做过突变测试);冒烟脚本 `sim/dev_rec_cloud.py`。

### 三者对比

| | 坐标系 | 深度单位 | 谁做重建 | 有颜色 |
|---|---|---|---|---|
| A. V2AP 手算 | 可直接出 **base 系**(乘了 `T_base_cam`) | float32 米 | 自己(5 行) | 要自己按掩码取 |
| B. ZED 原生 | 深度相机光学系 | 米 | ZED SDK | 有(打包在 w 里) |
| C. Kinect | 深度相机光学系 | int16 毫米 | k4a SDK | 有 |

**共同的坑**:B 和 C 出来的点云都在**相机光学系(x 右 / y 下 / z 前)**,不是机器人系。
要进机器人系必须再乘外参。「点云出来了」≠「点云在机器人系里对」。
A 是唯一一个已经把 `T_base_cam` 乘进去的。

---

## 5. 要接 ArUco 螺母位姿,还差什么

1. **标记检测 + 位姿**:`cv2.aruco.detectMarkers` → `cv2.solvePnP`(`SOLVEPNP_IPPE_SQUARE`),
   用 §2 的 K/dist。输出 `rvec/tvec` → 组成 `T_cam_obj`(4×4)。V2AP 里这段代码不存在。
2. **合成到 base**:`T_base_obj = T_base_cam @ T_cam_obj`,`T_base_cam` 来自
   `compute_T_base_cam(joint_pos_dict)`(`extrinsics.py:29`)。
3. **先验内参**:§2 的 ⚠(1),卷尺核对 fx,否则距离整体缩放。
4. **贴纸物理量**:`solvePnP` 的 `obj_pts` 必须用**印出来后实测的**边长(打印机缩放很常见),
   不是设计值。

---

## 6. 相关旧记录

- `docs/workflow_fj_snapshot_20260810.md:702` —— 最早记下 V2AP `camera/` 里有什么。
- `docs/workflow_fj_snapshot_20260810.md:1430` —— 点云坐标系 / 外参那两条结构性提醒。
- `docs/PROJECT_HANDOFF.md:216` —— V2AP / T-Rex 作为只读参考实现的定位。
