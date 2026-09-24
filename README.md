# uav_flight_sim

无人机飞行仿真 **纯算法层** —— 一套零专有依赖（不引用 `@kit.AREngine` / `ArkGraphics3D` / `hilog` / `MindSporeLiteKit`）的飞行控制、转向避障、刚体动力学、路径规划与环境感知逻辑。

> 本仓库是 ARDroneHarmony（AR 无人机 App）向 **OpenHarmony UAV SIG** 孵化的算法子集：把可独立验证的"决策 / 物理"逻辑从渲染层剥离，使其能在 **CI / 离线仿真 / 真机** 三端复用同一份代码。

---

## 为什么存在这个仓库

原 App 的飞控逻辑与 AR 渲染强耦合，且依赖 OpenHarmony 专有套件，无法在开箱即用的开发机上编译与单测。孵化到 UAV SIG 时，社区更关心的是**算法本身**（飞控手感、避障策略、状态机），而非 AR 渲染。

因此本仓库只保留纯逻辑，并用一个抽象接口 `SpatialPerception`（"空间感知输入" 契约）作为算法与数据源之间的**唯一边界**：

```
   AR 引擎 DepthSampler  ┐
   离线仿真器           ├─ 实现 SpatialPerception ─▶ DroneController（零改动复用）
   真实传感器           ┘
```

## 目录结构

```
src/
  core/                 零依赖基础设施
    Vec3.ts             自实现三维向量（替代 ArkGraphics3D.Vec3）
    Logger.ts           自实现日志（替代 hilog）
    LoggerLevel.ts      日志级别
  model/                飞行动力学状态机（运动学内核）
    DroneController.ts    飞控状态机 + 速度积分 + 姿态推导
    WorldObject.ts        AR 世界对象基类（锚定 + 偏移位姿）
  avoidance/            碰撞评估 + 转向力合成
    CollisionDetector.ts  碰撞 / 威胁评估（纯函数）
    SteeringBehavior.ts   Seek/Avoid 转向 + 速度域避障
  dynamics/             真实刚体动力学（v0.2.0 新增）
    RotorMixer.ts         四旋翼混控 + 执行器饱和 + 电机一阶滞后
    FlightDynamics.ts     刚体动力学积分器（半隐式欧拉）
    CascadeController.ts  级联控制：位置环 / 速度环 / 姿态环（纯函数）
  planning/             占用栅格与路径规划（v0.2.0 新增）
    PathPlanner.ts        世界平面 → 2.5D 栅格 → A* → 可见性平滑
    MeshGrid.ts           场景 mesh → 占用栅格（平面栅格的替代数据源）
  environment/          大气换算与天气归一化（v0.2.0 新增）
    Atmosphere.ts         风矢量 / 空气密度 / 低通平滑 / 阵风
    WeatherCode.ts        风速单位折算 / 异常值防护 / WMO 代码
  telemetry/            对外可观测状态推导
    HudModel.ts          HUD 威胁等级 / 配色 / 文案（纯函数）
  contract/             输入契约（纯接口）
    SpatialPerception.ts     空间感知输入（几何：前方障碍）
    EnvironmentPerception.ts 环境感知输入（大气：风场 / 密度）
  index.ts              统一导出（barrel）
tests/                 vitest 单测 + 零依赖自检 + 仿真黄金用例
sim/
  sim_flight.ts         无头飞行仿真演示（含 MockPerception）
docs/
  design.md             设计说明与验收口径
CHANGELOG.md            版本更新说明（新增 / 变更 / 验证 / 未纳入项）
```

依赖方向自底向上**单向**：

```
core ──▶ model / avoidance / dynamics / planning / environment ──▶ telemetry
                          ▲
                      contract（纯接口）
```

`core` 零依赖；`model` / `avoidance` / `dynamics` 只依赖 `core`；`planning` 依赖 `core` + `avoidance`；
`contract` 为纯接口（仅依赖 `core` 的 `Vec3` 类型）。**上层不得反向依赖下层实现。**

## 快速开始

```bash
npm install        # 安装 typescript / vitest / tsx
npm run typecheck  # tsc 类型校验（无输出即通过）
npm test           # 运行 vitest 单测
npm run sim        # 运行无头飞行仿真演示
npm run verify     # 一键：类型校验 + 单测 + 仿真
```

> 不装框架也能验收：`npm run selfcheck` 走零依赖自检（直接调用各模块内置 `selfTest()`）。

## 核心契约：SpatialPerception

```ts
import { SpatialPerception, Vec3 } from 'uav_flight_sim';

class MyDepthSampler implements SpatialPerception {
  getObstacleDistance(): number {
    // 返回机头前向障碍净距（米）；Infinity = 无障碍 / 深度不可用
    return this.lastDepthGap;
  }
  getObstacleNormal?(): Vec3 | null {
    // 可选：障碍远离单位向量，用于横移参考
    return this.awayNormal;
  }
}

const drone = new DroneController();
drone.perception = new MyDepthSampler(); // 接入即生效，飞控代码无需改动
```

- **纯算法模式（CI / 离线）**：用 `MockPerception` 注入确定性数据，无需任何硬件即可跑通避障与单测。
- **联合仿真模式**：物理交给仿真引擎，本仓库只负责"决策 / 契约"。
- **真机 / AR 模式**：原 App 的 `DepthSampler` 实现本接口，把 AREngine 深度直接接入。

### 第二个契约：EnvironmentPerception

同一套思路的第二个实例 —— 前者回答「前方多远有障碍」（**几何**），后者回答「周围空气什么状态」（**大气**）：

```ts
import { EnvironmentPerception, Vec3 } from 'uav_flight_sim';

class MyWindSource implements EnvironmentPerception {
  getWindVelocity(): Vec3 | null {
    // 风速矢量（m/s，世界系）；null = 无风 / 未接入
    return { x: this.windX, y: 0, z: this.windZ };
  }
  getAirDensity?(): number {
    // 空气密度相对因子（标准海平面 = 1.0）
    return this.rho;
  }
}
```

⚠️ **网络取数不入库**。实时天气来自 HTTP 接口（带时延、会失败、单位随服务商变化），
放进算法库会直接破坏「CI 可离线、确定性」的定位。因此本仓只保留**纯换算**
（`environment/Atmosphere.ts` / `WeatherCode.ts`），取数与刷新留在 App 侧。

未接入 / 感知关闭时返回 `null` 或零向量 —— 算法据此退化为无风飞行，
与原行为**逐位一致**（零回归，已被单测钉死）。

## 验收口径（与真机对齐的单一事实来源）

| 阈值 | 值 | 含义 |
| --- | --- | --- |
| `AVOID_MARGIN_M` | 0.4 m | 硬停面：再近则把前进分量清零（实测刹停 gap≈0.38m） |
| `AVOID_SLOW_BAND_M` | 1.2 m | 减速带宽度：从此处向外线性收敛前进分量 |
| `AVOID_SAFE_DIST_M` | 1.6 m | 避障生效距离 = 硬停面 + 减速带 |
| `DRONE_RADIUS_M` | 0.19 m | 机体包围球半径 |
| `AVOID_STEER_GAIN` | 1.25 | 横向绕行增益（越近越鼓励横移贴墙滑行） |

避障强度与前进分量缩放**同源**：`fwdScale = (gap − margin) / band`，`avoidStrength = 1 − fwdScale`。因此面板读数与实际刹车力度永远一致，不会出现"显示危险却没刹车"。

## 里程碑

| 阶段 | 交付件 | 状态 |
| --- | --- | --- |
| M1 建仓与规范 | 独立仓库 / Apache-2.0 / OWNERS / DCO / CI | ✅ 完成 |
| M2 能力剥离与重构 | `core`/`model`/`avoidance`/`telemetry`/`contract` 五模块 + 单测 + 仿真黄金用例入 CI | ✅ 完成 |
| M2+ 第二批剥离 | `dynamics`（刚体动力学）/ `planning`（A\* 规划）/ `environment`（大气）三模块 + `EnvironmentPerception` 契约 | ✅ 完成（v0.2.0） |
| M3 仿真方案与文档 | 脚本化仿真方案、场景示例、接入指南（含对接 Simulator SIG） | ⏳ 待办 |
| M4 毕业准备 | 架构 SIG 毕业评审材料、QA SIG 准出材料 | ⏳ 待办 |

> v0.2.0 属**超出原提案范围**的新增能力（原提案 M1–M4 未含刚体动力学与路径规划）。
> 向上游贡献时会作为新增交付物一并申报。详见 `CHANGELOG.md`。

## 许可与合规

- 采用 **Apache-2.0** 许可（与 OpenHarmony 社区一致）；所有源码头已标注版权与许可。
- 已签署 **DCO**（开发者原创声明）。
- 本仓库**不含**任何原 App 的私有签名 / 密钥 / 资源，可安全开源。

## 与 OpenHarmony UAV SIG 的关系

- 上游门户仓库：`openharmony-robot/uav`
- 本仓库作为个人孵化仓先行建设，后续按 SIG 流程向上游贡献。
- 本仓库负责"决策 / 物理 / 契约"；渲染、AR 锚点、设备适配由 App / SIG 主仓承担。
