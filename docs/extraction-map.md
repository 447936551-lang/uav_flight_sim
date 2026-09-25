# 代码溯源对照表（Extraction Map）

本表记录 `uav_flight_sim` 各模块从作者自有 OpenHarmony 无人机控制 App（项目名 **AR无人机**）的剥离来源与取舍，供 SIG 毕业评审溯源。

> 商标说明：AR无人机 为作者自有项目名称，与任何第三方厂商及其产品商标无关联；本仓库不依赖、不引用任何第三方专有 SDK 或商标化能力。

## 1. 已纳入（Included）

| 源 App 模块（.ets） | 库模块（.ts） | 说明 |
| --- | --- | --- |
| `ar/DroneController.ets`（运动学内核） | `src/model/DroneController.ts` | 仅搬运动学内核；双内核（真实动力学）切换逻辑留待 M3 |
| `ar/WorldObject.ets` | `src/model/WorldObject.ts` | 世界对象基类（锚定 + 偏移位姿） |
| `ar/CollisionDetector.ets` | `src/avoidance/CollisionDetector.ts` | 碰撞 / 威胁评估（纯函数） |
| `ar/SteeringBehavior.ets`（含 apasMode） | `src/avoidance/SteeringBehavior.ts` | 转向避障；商标化能力去商标化为 `bypassMode` |
| `ar/HudModel.ets` | `src/telemetry/HudModel.ts` | HUD 威胁等级 / 配色 / 盲态语义 |
| `ar/FlightDynamics.ets` | `src/dynamics/FlightDynamics.ts` | 刚体动力学积分器（半隐式欧拉） |
| `ar/RotorMixer.ets` | `src/dynamics/RotorMixer.ts` | 四旋翼混控 + 执行器饱和 + 电机一阶滞后 |
| `ar/CascadeController.ets` | `src/dynamics/CascadeController.ts` | 级联控制纯函数 |
| `ar/PathPlanner.ets` | `src/planning/PathPlanner.ts` | 占用栅格 + A* + 可见性平滑 |
| `ar/MeshGrid.ets` | `src/planning/MeshGrid.ts` | 场景 mesh → 占用栅格 |
| `ar/EnvironmentModel.ets`（纯换算部分） | `src/environment/Atmosphere.ts` | 风矢量 / 空气密度 / 阵风 |
| `service/WeatherService.ets`（纯逻辑部分） | `src/environment/WeatherCode.ts` | 风速单位折算 / WMO 代码 |
| `utils/Logger.ets`（思路） | `src/core/Logger.ts` | 自实现日志，替代 `hilog` |

契约（`src/contract/SpatialPerception.ts` / `EnvironmentPerception.ts`）为新增抽象边界；App 侧以 `ARDepthSampler` / `AtmosphereSource` 等实现之，但实现本身因依赖 `@kit.AREngine` 等专有套件未入库。

## 2. 故意未纳入（Excluded by design）

| 源 App 模块 | 未纳入原因 |
| --- | --- |
| `ar/DroneController.ets`（真实动力学内核 `useDynamics` / `setFlightMode`） | 双内核集成归 M3；当前库内控制器仍为单内核（运动学） |
| `algorithms/FlightAlgorithm.ets` + `algorithms/{WaypointFlight,RthFlight,PoiOrbit,ApasAvoidance}.ets` | 算法竞技框架（`FlightAlgorithm` 契约 + 注册表）按项目决策暂不纳入；其中「智能绕障」本体已以 `bypassMode` 并入 `avoidance/SteeringBehavior` |
| `ai/*`（MindSporeLite 端侧推理） | 依赖 `@kit.MindSporeLiteKit`，属设备能力层 |
| `service/AlertService.ets` | 依赖 `@kit.SensorServiceKit` / `AudioKit` / `MediaKit`，属 UI 反馈层 |
| `ar/ARDepthSampler.ets` / `ARPlaneTracker.ets` / `MeshProbe.ets` / `MeshSampler.ets` / `AtmosphereSource.ets` | 依赖 `@kit.AREngine` / `@kit.ArkGraphics3D`，属数据源 / 渲染层；其**接口**已抽象为 `SpatialPerception` / `EnvironmentPerception` |
| `ar/ARCameraIntrinsics.ets` / `ARSessionManager.ets` / `ARFrameStream.ets` / `AnchorManager.ets` / `DroneVisuals.ets` / `HoldTargetVisual.ets` / `HorizonVisual.ets` / `ARDroneCallback.ets` / `ARDroneSession.ets` | AR 渲染 / 会话管理层，超出算法库范围 |
| `signing/`（release.p12 等密钥） | **严禁入库**：私有签名 / 密钥 / 资源 |

## 3. 零回归保障

- 所有搬运文件**公式、常量、阈值、内置 `selfTest` 原样保留**，仅改写 import 路径；
- 新增环境 / 动力学输入**默认关闭**，`useDynamics` 等同理，单测逐位对比旧行为钉死零回归；
- 黄金用例数值恒定：盲飞 `−21.67m`、感知驱动硬停面 `0.40m`。

详见 `CHANGELOG.md`。
