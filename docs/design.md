# uav_flight_sim 设计说明

## 1. 背景与目标

AR无人机 是一套基于 `@kit.AREngine` 的 AR 无人机 App（作者自有项目，与任何第三方厂商及其产品商标无关联）。其飞控与避障逻辑质量较高，但强耦合于 OpenHarmony 专有套件，无法在普通开发机 / CI 中编译与单测，也难以被社区复用。源码溯源与取舍对照见 [`extraction-map.md`](./extraction-map.md)。

本仓库目标：**把可独立验证的算法子集剥离成零专有依赖的纯逻辑层**，作为向 OpenHarmony **UAV SIG** 孵化的第一步（里程碑 M1）；M2 起按领域拆分为 `core` / `model` / `avoidance` / `telemetry` / `contract` 五个模块。

已实现的可提取代码约 1500 行，覆盖：

- 飞控状态机与速度积分（DroneController）
- 转向 / 避障（SteeringBehavior，Craig Reynolds Seek+Avoid，速度域集成）
- 碰撞 / 威胁评估（CollisionDetector）
- HUD 威胁等级推导（HudModel）
- AR 世界对象基类（WorldObject）

## 2. 依赖剥离策略

| 原依赖 | 处理方式 |
| --- | --- |
| `@kit.ArkGraphics3D` 的 `Vec3` | 自实现 `src/core/Vec3.ts`（`{x,y,z}` 纯数据 + 纯函数运算） |
| `@kit.PerformanceAnalysisKit` 的 `hilog` | 自实现 `src/core/Logger.ts`（同签名，底层映射到 console） |
| `@kit.AREngine`（深度 / 锚点） | 抽象为 `SpatialPerception` 接口，由外部实现 |

所有 `.ets` 源已转换为 `.ts`，可在 Node / CI 直接编译运行。

## 3. 核心设计：SpatialPerception 契约

```
DroneController
   │  update(dt)
   │
   └─► perception.getObstacleDistance()   ← 唯一的数据来源边界
       perception.getObstacleNormal()?    ← 可选，横移参考
```

- 控制器**不持有、不依赖**任何传感器 / AR 引擎；每帧通过接口取障碍距离。
- 同一份 `DroneController` 可被三类数据源驱动：
  1. 真机 `DepthSampler`（AREngine 深度）
  2. 离线 `MockPerception`（确定性注入，用于 CI / 单测）
  3. 仿真引擎（联合仿真模式）

## 4. 模块分层与依赖方向（M2 起按领域切分）

```
src/
├── core/          零依赖基础设施：Vec3 / Logger / LoggerLevel
├── model/         飞行动力学状态机（运动学内核）：DroneController / WorldObject
├── avoidance/     碰撞评估 + 转向力合成：CollisionDetector / SteeringBehavior
├── dynamics/      真实刚体动力学：RotorMixer / FlightDynamics / CascadeController
├── planning/      占用栅格 + 路径规划：PathPlanner / MeshGrid
├── environment/   大气换算 + 天气归一化：Atmosphere / WeatherCode
├── telemetry/     对外可观测状态推导：HudModel
└── contract/      输入契约：SpatialPerception / EnvironmentPerception
```

自底向上**单向依赖**（禁止反向 import）：

| 模块 | 依赖 | 职责 |
| --- | --- | --- |
| `core` | 无 | 数值 / 向量 / 日志原语，被所有模块共享 |
| `model` | `core`, `avoidance`, `contract`, `environment` | 状态机 + 速度积分 + 姿态推导；锚定 + 偏移位姿推导（**运动学**内核）；消费两个输入契约 |
| `avoidance` | `core` | 碰撞评估、转向避障（纯函数） |
| `dynamics` | `core` | 混控 / 执行器饱和 / 刚体积分 / 级联控制（**真实动力学**内核） |
| `planning` | `core`, `avoidance` | 占用栅格构建、A\* 规划、可见性平滑、mesh 栅格化 |
| `environment` | `core` | 风矢量 / 空气密度 / 平滑阵风 / 天气单位归一化 |
| `telemetry` | `core`, `avoidance` | 威胁等级 / 配色 / 文案（纯函数） |
| `contract` | `core`（仅 `Vec3` 类型） | 算法与数据源的唯一边界（**纯接口，不含实现**） |

> 纪律：`model` / `avoidance` / `dynamics` 为零依赖底层（只依赖 `core`），
> 不得反向依赖 `telemetry` / `contract` 的实现；`contract` 为纯接口。
>
> **两套内核并存**：`model`（运动学）与 `dynamics`（刚体）是**并列的可选内核**，
> 不是替换关系。切换由调用方决定，默认仍走运动学路径 —— 这保证既有真机验证过的
> 手感与刹停表现（gap≈0.38m）**零回归**。

> **网络依赖纪律**：任何 HTTP / 设备传感器取数**不得进库**。
> 天气数据只保留纯换算（`environment/`），取数与刷新由 App 侧实现 `EnvironmentPerception` 注入。
> 违反此条会让 CI 失去离线确定性与可复现性。

## 5. 验收口径（单一事实来源）

见 `README.md` 阈值表。`CollisionDetector.evaluate()` 同时产出 `fwdScale` 与 `avoidStrength`，二者同源，保证"读数 = 刹车力度"。

防撞关键性质（已被单测与 `selfTest` 守护）：

- 只限制"朝障碍去"的前进分量；横移 / 后退一律放行 → "贴墙滑行不撞墙"。
- gap=1.0m 时前进分量仍 ≥ 满速 45%（防"防撞误刹前向"回归）。
- 测距不可用（∞）时**不限制**速度（盲 ≠ 安全，但不僵死）。
- 自定义 `band / margin / safeDist` 必须真正生效（参数透传守护）。

## 6. 验证手段

- **vitest 单测**：`tests/*.test.ts`，覆盖状态机、契约接入、避障回归。
- **内置 selfTest**：各模块保留 `selfTest()`，可在真机 / Node / CI 直接验收，无需框架。
- **无头仿真**：`sim/sim_flight.ts` 用 `MockPerception` 演示"无感知撞墙 vs 感知驱动急停"。

## 7. 里程碑

| 阶段 | 交付件 | 状态 |
| --- | --- | --- |
| M1 建仓与规范 | 独立仓库 / Apache-2.0 / OWNERS / DCO / CI | ✅ 完成 |
| M2 能力剥离与重构 | `core`/`model`/`avoidance`/`telemetry`/`contract` 五模块 + 单测 + 仿真黄金用例入 CI | ✅ 完成 |
| M2+ 第二批剥离 | `dynamics` / `planning` / `environment` 三模块 + `EnvironmentPerception` 契约 | ✅ 完成（v0.2.0） |
| M3 仿真方案与文档 | 脚本化仿真方案、场景示例、接入指南（含对接 Simulator SIG） | ⏳ 待办 |
| M4 毕业准备 | 架构 SIG 毕业评审材料、QA SIG 准出材料 | ⏳ 待办 |

> M2 交付后，原 App 仍以"源码依赖"方式接入本仓（见 `README.md` 接入说明），替换内联阈值以验证零回归；这是 M2 的回归验证手段，不改变本仓作为独立算法库的定位。
