# 代码溯源对照表（Extraction Map）

本表记录 `uav_flight_sim` 各模块从作者自有 OpenHarmony 无人机控制 App（项目名 **AR无人机**）的剥离来源与取舍，供 SIG 毕业评审溯源。

> 商标说明：AR无人机 为作者自有项目名称，与任何第三方厂商及其产品商标无关联；本仓库不依赖、不引用任何第三方专有 SDK 或商标化能力。

## 1. 已纳入（Included）

| 源 App 模块（.ets） | 库模块（.ts） | 说明 |
| --- | --- | --- |
| `ar/DroneController.ets`（运动学 + 真实动力学双内核 + M4/M5 保真层） | `src/model/DroneController.ts` | 运动学内核（默认）+ 真实刚体动力学双内核；`setFlightMode`/`useDynamics` 切换，默认运动学（零回归）；`stepDynamics` 串起级联控制 / 混控 / 执行器一阶滞后 / 刚体积分（v0.3.0 并入）；v0.4.0 新增 L1 速率环 / L2 增强气动 / L3 湍流 / L4 执行器非线性 / L5 EKF（诊断级）/ L6 传感器噪声，全部默认关，并提供 `applyHighFidelityPreset` / `setHighFidelityEnabled` 一键装配与可逆开关；v0.6.0 新增 `dcaEnabled` / `vpndEnabled` 主开关与 DCA/VPND 接线（默认关，零回归） |
| `ar/DelayCompensator.ets`（P4 · DCPA） | `src/avoidance/DelayCompensator.ts` | 延迟补偿预测式避障：PDC 感知延迟抵消（`compensateSectorDistance`/`compensateSectors`）+ TBB 时间预算刹车（`requiredStoppingDistance`/`timeBudgetMargin`/`shouldBrakeEarly`）+ EFP 急停快路径（`emergencyFastPath`），`dcaOverlay` 一键叠加；默认关（零回归）；`selfTest` 39 项，与 App 同数 |
| `ar/VisionParallax.ets`（P4 · VPND） | `src/avoidance/VisionParallax.ts` | 视觉视差负空间检测：识别深度盲点（透明物/丝线）；`VisionFlowSource` 契约 + `MockVisionSource` + `contradictionGate` 矛盾闸门 + `VisionParallax` 薄状态类；默认关（零回归）；`selfTest` 17 项，与 App 同数 |
| `ar/WorldObject.ets` | `src/model/WorldObject.ts` | 世界对象基类（锚定 + 偏移位姿） |
| `ar/CollisionDetector.ets` | `src/avoidance/CollisionDetector.ts` | 碰撞 / 威胁评估（纯函数） |
| `ar/SteeringBehavior.ets`（含 bypassMode） | `src/avoidance/SteeringBehavior.ts` | 转向避障；商标化能力去商标化为 `bypassMode`；硬停面随速度伸缩（`avoidMarginFor`）、绕行方向按障碍方位择优（`pickAvoidSide`，2026-10-06 同步） |
| `ar/ARDepthSampler.ets`（**仅算法本质子集**） | `src/avoidance/FusionGeometry.ts` | 多源融合几何的**纯函数**部分：近场闸门 `DEPTH_MIN_VALID_M`、支撑面/上方判据 `isSupportSurfaceHit`/`isOverheadHit`、前向净距折算 `forwardGapFromWorldHit`、射线求交 `rayPlaneT`/`rayTriangleT`。`Vec3` 由元组改对象（唯一命名分叉）；阈值复用 `CollisionDetector` 的 `AVOID_*` |
| `ar/HudModel.ets` | `src/telemetry/HudModel.ts` | HUD 威胁等级 / 配色 / 盲态（Blind）· 代测（degraded）· 抑制态（suppressed）三条「不是安全」语义 |
| `ar/FlightDynamics.ets` | `src/dynamics/FlightDynamics.ts` | 刚体动力学积分器（半隐式欧拉） |
| `ar/RotorMixer.ets` | `src/dynamics/RotorMixer.ts` | 四旋翼混控 + 执行器饱和 + 电机一阶滞后 |
| `ar/CascadeController.ets`（含 L1/L2 扩展） | `src/dynamics/CascadeController.ts` | 级联控制纯函数；v0.4.0 新增 `attitudeRateCmd`（中环）/ `rateTorque`（内环）/ `bodyDragForce`（机体气动矩阵 + 入流） |
| `ar/Px4Params.ets`（M5-1） | `src/dynamics/Px4Params.ts` | PX4 公开默认参数 → 本仓高保真层增益 / EKF 噪声的**唯一真值源**（静态标定，非逐位抄实现） |
| `ar/EkfEstimator.ets`（L5） | `src/dynamics/EkfEstimator.ts` | 误差状态 EKF（15 状态：p/v/att/gyro bias/accel bias） |
| `ar/SensorModel.ets`（L6） | `src/dynamics/SensorModel.ts` | 传感器噪声 / 丢包模型（种子化，可复现） |
| `ar/Turbulence.ets`（L3） | `src/environment/Turbulence.ts` | 种子化 OU 湍流风（σ≈1.2 m/s、τ=0.5 s） |
| `core/SeedPRNG.ets` | `src/core/SeedPRNG.ts` | Mulberry32 + Box-Muller 种子化 PRNG（L3/L6 公共随机源） |
| `ar/PathPlanner.ets` | `src/planning/PathPlanner.ts` | 占用栅格 + A* + 可见性平滑 |
| `ar/MeshGrid.ets` | `src/planning/MeshGrid.ts` | 场景 mesh → 占用栅格 |
| `ar/EnvironmentModel.ets`（纯换算部分） | `src/environment/Atmosphere.ts` | 风矢量 / 空气密度 / 阵风 |
| `service/WeatherService.ets`（纯逻辑部分） | `src/environment/WeatherCode.ts` | 风速单位折算 / WMO 代码 |
| `utils/Logger.ets`（思路） | `src/core/Logger.ts` | 自实现日志，替代 `hilog` |

契约（`src/contract/SpatialPerception.ts` / `EnvironmentPerception.ts`）为新增抽象边界；App 侧以 `ARDepthSampler` / `AtmosphereSource` 等实现之，但实现本身因依赖 `@kit.AREngine` 等专有套件未入库。

## 2. 故意未纳入（Excluded by design）

| 源 App 模块 | 未纳入原因 |
| --- | --- |
| `ar/DroneController.ets`（真实动力学内核 `useDynamics` / `setFlightMode`） | **已于 v0.3.0 纳入**：见上表 `DroneController.ts` 双内核行；默认运动学（零回归） |
| `algorithms/FlightAlgorithm.ets` + `algorithms/{WaypointFlight,RthFlight,PoiOrbit,ApasAvoidance}.ets` | 算法竞技框架（`FlightAlgorithm` 契约 + 注册表）按项目决策暂不纳入；其中「智能绕障」本体已以 `bypassMode` 并入 `avoidance/SteeringBehavior` |
| `ai/*`（MindSporeLite 端侧推理） | 依赖 `@kit.MindSporeLiteKit`，属设备能力层 |
| `service/AlertService.ets` | 依赖 `@kit.SensorServiceKit` / `AudioKit` / `MediaKit`，属 UI 反馈层 |
| `ar/ARDepthSampler.ets` / `ARPlaneTracker.ets` / `MeshProbe.ets` / `MeshSampler.ets` / `AtmosphereSource.ets` | 依赖 `@kit.AREngine` / `@kit.ArkGraphics3D`，属数据源 / 渲染层；其**接口**已抽象为 `SpatialPerception` / `EnvironmentPerception`。**注意**：`ARDepthSampler` 中**与平台无关的纯函数子集**（近场闸门、支撑面/上方判据、前向净距折算、射线-平面/三角形求交）已于 0.2.5 抽取进 `src/avoidance/FusionGeometry.ts`；**未纳入**的是其采样编排（`@kit.AREngine` 依赖、锥角加密、跨来源平滑、`FUSION_*` 开关及运行时路径） |
| `ar/ARCameraIntrinsics.ets` / `ARSessionManager.ets` / `ARFrameStream.ets` / `AnchorManager.ets` / `DroneVisuals.ets` / `HoldTargetVisual.ets` / `HorizonVisual.ets` / `ARDroneCallback.ets` / `ARDroneSession.ets` | AR 渲染 / 会话管理层，超出算法库范围 |
| `signing/`（release.p12 等密钥） | **严禁入库**：私有签名 / 密钥 / 资源 |

## 3. 零回归保障

- 所有搬运文件**公式、常量、阈值、内置 `selfTest` 原样保留**，仅改写 import 路径；
- 新增环境 / 动力学输入**默认关闭**，`useDynamics` 等同理，单测逐位对比旧行为钉死零回归；
- 黄金用例数值：盲飞 `−21.67m` 恒定；感知驱动硬停面自 2026-10-06 起为 **`0.25m`**
  （静止基线 `margin(0)`；硬停面改为随速度伸缩 `margin(v)=0.25+v²/3`，
  旧值 `0.40m` 为定值时代的语义，变更属 App 侧有意的语义演进而非回归）。

详见 `CHANGELOG.md`。
