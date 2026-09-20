# uav_flight_sim 设计说明

## 1. 背景与目标

ARDroneHarmony 是一套基于 `@kit.AREngine` 的 AR 无人机 App。其飞控与避障逻辑质量较高，但强耦合于 OpenHarmony 专有套件，无法在普通开发机 / CI 中编译与单测，也难以被社区复用。

本仓库目标：**把可独立验证的算法子集剥离成零专有依赖的纯逻辑层**，作为向 OpenHarmony **UAV SIG** 孵化的第一步（里程碑 M1）。

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

## 4. 分层与职责

| 层 | 文件 | 职责 |
| --- | --- | --- |
| 基础 | `Vec3`, `Logger`, `LoggerLevel` | 数值 / 日志原语 |
| 物理层 L5 | `CollisionDetector`, `SteeringBehavior` | 碰撞评估、转向避障（纯函数） |
| 控制层 | `DroneController` | 状态机 + 速度积分 + 姿态推导 |
| UI 层 L7 | `HudModel` | 威胁等级 / 配色 / 文案（纯函数） |
| 世界层 L3 | `WorldObject` | 锚定 + 偏移的位姿推导 |
| 契约 | `perception/SpatialPerception` | 算法与数据源的边界 |

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

## 7. 后续里程碑

- M2：被原 App 以"源码依赖"方式接入，替换内联阈值，验证零回归。
- M3：联合仿真（物理交给引擎，本仓负责决策 / 契约）。
- M4：向上游 `openharmony-robot/uav` 贡献，按 SIG 流程合入。
