# 方案 B:用 sim asset + FoundationPose 拿物体 6D 位姿(V2AP 现成管线)

> 2026-08-12 整理。从 V2AP-demo 里挖出的完整管线 + 针对「我们有 CAD/sim asset」的精简版。
> 配套阅读:[camera_intrinsics_and_pointcloud.md](camera_intrinsics_and_pointcloud.md)(内参、外参、点云)。
> 路径约定同前:`demo/…`、`camera/…` 是 **V2AP-demo** 里的;`scripts/…`、`docs/…` 是本仓库的。

---

## 0. 一句话结论

V2AP 的管线是 **7 步(T1–T7)**,其中 **T3(SAM3D 重建 mesh)和 T4(用深度估尺度)存在的唯一理由是他们没有物体模型**。

**我们有 sim asset,所以这两步整个删掉。** 管线从 7 步缩到 3 步:

```
V2AP:  T1 验证 → T2 分割 → T3 SAM3D 重建 → T4 估尺度 → T5 FoundationPose → T6 抓取候选 → T7 状态
我们:  T1 验证 → T2 分割 →      ✂ 删       →     ✂ 删    → T5 FoundationPose(喂 CAD) → 完
```

这不只是省事,是**精度上的净赚**:

| 被删的步骤 | 它引入的误差 |
|---|---|
| T3 SAM3D | 单视角重建,**物体背面是模型猜出来的**。整条链最大的误差源。 |
| T4 尺度估计 | `scale_factor = Z_real / Z_mesh`,深度中位数比值,带 `[0.3, 3.0]` 的 clamp guard(`README.md:455`)。对小件、反光件极不稳。 |

CAD asset 本身就是**米制且几何精确**的,这两个误差源直接归零。

---

## 1. 原管线全貌(先看清再裁)

两台机器。**这一点很重要:V2AP-demo 本地只有 Razor 那一半。**

| 机器 | 角色 | 代码在哪 |
|---|---|---|
| **Razor**(接机器人的笔记本) | 采集 RGB-D + 标定 → 上传 → 下载结果 → IK + 抓取 | ✅ `V2AP-demo/demo/phase2/`,本地就有 |
| **Titan**(GPU 服务器) | SAM → SAM3D → 尺度 → FoundationPose → PDM 抓取候选 | ❌ **不在本地**。另一个仓库:`UCB_Project` 的 `titan` 分支(`demo/PHASE2_PIPELINE.md` 里给了 GitHub 链接) |

**但 I/O 契约在 `demo/phase2/README.md`(37 KB)里写得极完整** —— 每一步的输入文件、输出文件、JSON schema、坐标系约定全都有。感知那半边照着契约自己实现是可行的,不用拿到他们的仓库。

```text
Razor                                          Titan (segment_daemon 常驻)
─────                                          ──────────────────────────
 R1  摆物体,机器人回起始位姿
 R2  capture_session.py → sessions/<id>/input/
 R3  rsync 上传 input/  ────────────────────►  demo/sessions/<id>/input/
 R3b ssh 打 .upload_complete 标记 ──────────►  daemon 开始处理
                                               T2  SAM2 网页 UI(Flask :7860)
                                                   操作员经 SSH 隧道点一下物体
                                               T3–T7 批处理
                                               status.json success=true
 R4  轮询 status.json / daemon_state.json
 R5  rsync 下载 output/ ◄───────────────────  demo/sessions/<id>/output/
 R6  逐张看 T3–T6 的 PNG(阻塞弹窗)
 R7  run_auto_grasp.py → IK → OMPL → 合拢 → 抬起
```

Session ID 格式:`YYYYMMDD_HHMMSS_<object_slug>`,例:`20260602_192346_chips`。

---

## 2. 我们的三步管线

### 输入包 `input/`(Razor 侧,已有现成脚本)

`demo/phase2/capture_session.py` 一条命令产出,结构见 `pack_session.py:58`:

```text
input/
├── session.json                # schema 1.1 元数据
├── rgb/left_rgb.png            # RGB 帧
├── depth/depth.npy             # float32 **米**
├── calib/
│   ├── intrinsics.json         # K + 畸变(见配套文档 §2)
│   ├── K.npy                   # (3,3) float64,与 intrinsics.json 必须逐位一致
│   ├── extrinsics.json         # T_base_cam,URDF FK 算的
│   └── robot_state.json        # 与 RGB-D 同一瞬间的关节角
├── scene/table.json            # 桌面高度
└── segment/prompt.json         # 可选:SAM 点提示
```

命令(V2AP 原样可用):

```bash
python demo/phase2/capture_session.py --object-name nut --sam-point 320 180
```

`--sam-point X Y` 就是「教会它关注哪里」那个信号,落到 `segment/prompt.json`
(`capture_session.py:219` `_optional_segment_prompt()`,写盘在 `pack_session.py:141`):

```json
{"tool": "sam2", "prompts": [{"type": "point", "xy": [320.0, 180.0], "label": 1}]}
```

### T2 — 分割,产出 mask

**输入**:`rgb/left_rgb.png` + 可选 `segment/prompt.json`
**输出**:

- `output/segment/mask.png` —— uint8,**0 = 背景,255 = 物体**,尺寸与 RGB 一致
- `output/segment/prompt_used.json` —— 实际用的提示(可复现)

三种拿 mask 的方式,按工作量排:

1. **人点一下**(V2AP 默认)—— SAM2 网页 UI,或离线用 `--sam-point` 批处理。
2. **几何先验自动出 mask** —— 工作区裁剪 → RANSAC 去桌面 → 聚类 → 按 CAD 包围盒筛。工位固定时比学习方法稳,且能免掉人工点击。
3. 文本提示(GroundingDINO)/ 自训检测器 —— 以后再说。

### T5 — FoundationPose 配准(**喂 CAD,不是喂重建的 mesh**)

这是整条管线唯一真正做位姿估计的一步。

**T5.1 构造 FP 场景目录**(`README.md:481`):

```text
fp_scene/
├── rgb/000000.png       # BGR uint8,与采集同分辨率
├── depth/000000.png     # uint16,**毫米**(FP 的 YcbineoatReader 要毫米)
├── masks/000000.png     # uint8 0/255,来自 T2
└── cam_K.txt            # 3×3 K,与 input 一致;若降采样必须同步缩放
```

单位转换(整条链上最容易静默错 1000 倍的地方):

```python
depth_mm = (depth_m * 1000.0).clip(0, 65535).astype(np.uint16)
K = np.load("input/calib/K.npy")
```

常量在 `demo/phase2/constants.py`:`FP_DEPTH_MM_SCALE = 1000.0`(:22)、`FP_FRAME_INDEX = "000000"`(:23)。

**T5.2 调用**(`README.md:503`,单帧 register 就够):

```python
pose = est.register(K=reader.K, rgb=color, depth=depth, ob_mask=mask, iteration=est_iter)
# pose 是 4×4 → 就是 T_cam_mesh
```

**我们与 V2AP 的差别**:`est` 构造时喂的 mesh,V2AP 用的是 `object_scaled.glb`(SAM3D 重建 + 估尺度的产物),**我们直接喂 sim asset**。FoundationPose 的 model-based 模式本来就是为已知 CAD 设计的,这是它的主用法,不是 hack。

> 若显存/速度需要降采样(UCB 用 `SHORTER_SIDE=480`),**K 必须同比例缩放**,并把最终 `(H_fp, W_fp)` 记进 `foundationpose_meta.json`。
> mesh 面数建议 ≤ 5000(`fast_simplification`)。

**T5.3 合成到 base 系**(`README.md:519`):

```python
T_base_cam  = np.array(json.load(open("input/calib/extrinsics.json"))["T_base_cam"])
T_cam_mesh  = pose                      # FP 输出
T_base_mesh = T_base_cam @ T_cam_mesh   # ← 螺母在机器人 base 系的位姿
```

**坐标系约定**(`README.md:132`,所有接口必须一致):
所有 4×4 都是 `T_dst_src`,列向量约定 `p_dst = T_dst_src @ p_src`。

| 符号 | 含义 | 关系 |
|---|---|---|
| `T_cam_mesh` | mesh → 相机 | `p_cam = T_cam_mesh @ p_mesh`(UCB 里叫 `ob_in_cam`) |
| `T_base_cam` | 相机 → base | `p_base = T_base_cam @ p_cam` |
| `T_base_mesh` | mesh → base | `T_base_mesh = T_base_cam @ T_cam_mesh` |

**T5.4 输出文件**:`register/T_cam_mesh.json`、`register/T_base_mesh.json`、`register/ob_in_cam/000000.txt`(UCB 格式的同一个 4×4)、`vis/T5_foundationpose_overlay.png`。

### 自检(V2AP 自带,别跳过)

- **投影核对**:用 `T_cam_mesh` 把 CAD 包围盒投回 RGB,与 mask 的 IoU 应当合理。反向投影函数现成:`demo/phase2/visualize_grasp.py:58` `_project_cam()`。
- **桌高核对**:把物体底面变换到 base 系,z 应接近 `table_height_m` ±5 cm。现成实现 `demo/phase2/table_height.py:14`,采集后自检在 `validate_input.py:171`。

---

## 3. 本地已有 vs 需要自己补

| 环节 | 状态 |
|---|---|
| 采集 RGB-D + 内外参 + 打包 | ✅ `capture_session.py` 全套,直接能用 |
| 点提示 → `prompt.json` | ✅ `capture_session.py:219` |
| 输入包校验 | ✅ `validate_input.py` |
| session 读写 | ✅ `session_io.py:33`、`pack_session.py` |
| 上传/下载/轮询编排 | ✅ `run_server_client_pipeline.py`、`server_client_transport.py` |
| 桌高 / 投影 / 可视化工具 | ✅ `table_height.py`、`visualize_grasp.py` |
| **SAM2 服务端** | ❌ 需自备(或本地跑 SAM2,不必走他们的 Flask UI) |
| **FoundationPose 本体** | ❌ 需自备:`FP_ROOT=$TITAN_ROOT/third_party/FoundationPose`,conda env `bundlesdf` |
| **T3 SAM3D / T4 尺度** | 🚫 **我们不需要**(有 CAD) |
| **T6 PDM 抓取候选** | ⏸ 装配任务未必用得上,先不接 |

**结论**:Razor 半边直接复用;我们要补的只有「SAM2 出 mask」和「FoundationPose 跑 CAD 配准」两块,而且**不必复刻他们的 Razor↔Titan 双机架构** —— 单机跑通即可,双机是他们的算力约束,不是我们的。

---

## 4. ⚠ 适用边界(必须写在前面)

这条管线的精度上限由**像素**决定,不由算法决定。用我们实测的头相机内参(fx=190.53 @640,即 571.6 @1920),0.7 m 处:

| 目标 | @640×360 | @1920×1080 |
|---|---|---|
| M12 螺母(对边 19 mm) | 5.2 px | 15.5 px |

**15 px 的物体上,1 px ≈ 1.3 mm。** 而装配对孔的公差是 ±0.5 mm。另外:

- FoundationPose **重度依赖深度**,而**发亮的金属螺母是 ZED 双目深度的最差情况**(高光 + 无纹理),mask 内很可能大面积无效深度。
- 深度无效值是 NaN/inf/0(`constants.py:30`),不是 0 而已。

**所以这条管线的定位**:

- ✅ 适合 **stage ① 粗定位**(±10–20 mm),把手臂送到腕相机看得见的位置;
- ✅ 适合**尺寸较大的零件**(支架、板件、托盘、夹具本体);
- ❌ **不足以**单独支撑 ±0.5 mm 的装配对孔 —— 那需要腕相机近距 + 相对位姿闭环 + 触觉插入(见上一轮讨论的三段式架构)。

把它当感知管线的**骨架**建起来是对的:接口、坐标系、session 格式都能直接复用到腕相机那一段;换的只是相机和距离。

---

## 5. 建议落地顺序

1. **先解掉内参分歧** —— fx 到底 190 还是 350,卷尺法核对(配套文档 §2 的 ⚠1)。这一步不做,下面全部白干。
2. **切 HD1080 采集** —— `zed_streamer --resolution HD1080`,内参按 1920 存(`calibrate_charuco.py --output-width 1920 --output-height 1080`),别再缩到 640。
3. **`capture_session.py` 采一帧螺母**,`--preview` 看 RGB + 深度,确认 mask 区域内深度不是一片空洞。**这一步就能判断方案 B 在你的零件上到底可不可行**,成本半小时。
4. 本地跑通 SAM2 出 mask。
5. FoundationPose 喂 sim asset,单帧 register,出 `T_cam_mesh`。
6. 合成 `T_base_mesh`,跑 §2 的两个自检。
7. 再决定要不要上腕相机。

第 3 步是**决策点**,不是流程步骤 —— 它的结果决定后面还值不值得投。
