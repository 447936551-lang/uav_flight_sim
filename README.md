# uav_flight_sim

> 中文说明 ｜ [English](./README.en.md)

无人机飞行仿真 **纯算法层** —— 一套零专有依赖（不引用 `@kit.AREngine` / `ArkGraphics3D` / `hilog` / `MindSporeLiteKit`）的飞行控制、转向避障、刚体动力学、路径规划与环境感知逻辑。

> 本仓库是作者自有 OpenHarmony 无人机控制 App（项目名 **AR无人机**）向 **OpenHarmony UAV SIG** 孵化的算法子集：把可独立验证的"决策 / 物理"逻辑从渲染层剥离，使其能在 **CI / 离线仿真 / 真机** 三端复用同一份代码。

**来源与商标声明**：AR无人机 为作者自有项目名称，与任何第三方厂商及其产品商标无关联；本仓库不依赖、不引用任何第三方专有 SDK 或商标化能力。完整溯源见 [`docs/extraction-map.md`](./docs/extraction-map.md)。

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
    SeedPRNG.ts         Mulberry32 + Box-Muller 种子化 PRNG（L3/L6 公共随机源）
  model/                飞行动力学状态机（运动学内核）
    DroneController.ts    飞控状态机（运动学 + 真实动力学双内核，setFlightMode 切换）+ 速度积分 + 姿态推导 + M4/M5 保真层开关
    WorldObject.ts        AR 世界对象基类（锚定 + 偏移位姿）
  avoidance/            碰撞评估 + 转向力合成
    CollisionDetector.ts  碰撞 / 威胁评估（纯函数）+ 上方净距 + 感知失联兜底档位
    SteeringBehavior.ts   Seek/Avoid 转向 + 速度域避障 + 绕行方向择优
    FusionGeometry.ts     多源融合几何（射线-平面 / 射线-三角形 / 走廊判据）
    TemporalGuard.ts      P1 时间维度：距离低通 + 急停迟滞 + 失联计时 + 走廊软权重
    VelocityObstacle.ts   P3 预测式：TTC 判据 + 速度障碍半空间投影 + 扇区构造
  dynamics/             真实刚体动力学（v0.2.0 新增）
    RotorMixer.ts         四旋翼混控 + 执行器饱和 + 电机一阶滞后
    FlightDynamics.ts     刚体动力学积分器（半隐式欧拉）
    CascadeController.ts  级联控制：位置环 / 速度环 / 姿态环（纯函数）+ L1/L2 扩展
    EkfEstimator.ts       误差状态 EKF（15 状态，L5）
    SensorModel.ts        传感器噪声 / 丢包模型（L6）
    Px4Params.ts          PX4 公开默认参数 → 本仓增益 / EKF 噪声映射（M5-1）
  planning/             占用栅格与路径规划（v0.2.0 新增）
    PathPlanner.ts        世界平面 → 2.5D 栅格 → A* → 可见性平滑
    MeshGrid.ts           场景 mesh → 占用栅格（平面栅格的替代数据源）
  environment/          大气换算与天气归一化（v0.2.0 新增）
    Atmosphere.ts         风矢量 / 空气密度 / 低通平滑 / 阵风
    WeatherCode.ts        风速单位折算 / 异常值防护 / WMO 代码
    Turbulence.ts         种子化 OU 湍流风（L3，σ≈1.2 m/s、τ=0.5s）
  telemetry/            对外可观测状态推导
    HudModel.ts          HUD 威胁等级 / 配色 / 文案（纯函数）
  contract/             输入契约（纯接口）
    SpatialPerception.ts     空间感知输入（几何：前方障碍）
    EnvironmentPerception.ts 环境感知输入（大气：风场 / 密度）
  index.ts              统一导出（barrel）
tests/                 vitest 单测 + 零依赖自检 + 仿真黄金用例
sim/
  sim_flight.ts         无头飞行仿真演示（含 MockPerception）
  scenarios.ts          M3 场景仿真：十一场景确定性冒烟 + P5 避障量化指标（CI 门禁）
  sim_high_fidelity.ts  M4/M5 高保真层端到端验证（速率环/气动/湍流/EKF，CI 门禁）
docs/
  design.md             设计说明与验收口径
  simulation.md         M3 脚本化仿真方案（无头 / 确定性 / 黄金值回归 / CI）
  integration.md        M3 接入指南（外部仿真器 / Simulator SIG 对接）
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
npm run sim:hf     # M4/M5 高真度层端到端验证（40 项断言）
npm run verify     # 一键：类型校验 + 单测 + 仿真 + 场景 + 高保真 + DCO
```

> 场景仿真（`npm run sim:scenarios`）是 M3 新增的 CI 门禁：**十一场景**确定性冒烟覆盖
> model / avoidance / environment / planning / dynamics 五个域，任一失败即非零退出。
> 其中 P5 五个场景输出统一量化指标（最小净距 / 侵入深度 / 到达路程 / 急停翻转 / 最小 TTC），
> 让"避障做得更好"第一次有了可回归的度量而非手感描述。
> 脚本化仿真方案与接入指南见 [`docs/simulation.md`](./docs/simulation.md) 与 [`docs/integration.md`](./docs/integration.md)。

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
  // ↓ 以下均为**可选**扩展（v0.5.0），不实现则回退到单标量路径，零改动
  getCeilingDistance?(): number {
    // 上方净距（米）；Infinity = 上方无实体 / 测不到
    return this.ceilingGap;
  }
  getObstacleLateral?(): number {
    // 障碍横向偏置（米，机体右向为正），供绕行方向择优
    return this.lateral;
  }
  isDepthSupported?(): boolean {
    // 本机是否支持深度：明确不支持时失联兜底立即降档，不必等超时
    return true;
  }
  getSectorScan?(): SectorScan | null {
    // 多扇区扫描（P2）：预测式避障与绕行择优的输入源
    return { sectors: this.sectors, timestampSec: t, confidence: 1 };
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
| `AVOID_MARGIN_BASE_M` | 0.25 m | 硬停面静态基线（速度相关硬停面 `avoidMarginFor(v) = 0.25 + v²/3`） |
| `CEILING_MARGIN_M` / `CEILING_SLOW_BAND_M` | 0.35 / 0.55 m | 上方硬停面 / 上方减速带（竖直方向更保守） |
| `DEPTH_TAU_SEC` | 0.12 s | 距离一阶低通时间常数（约 3τ≈0.36s 收敛 95%） |
| `AVOID_HYSTERESIS_M` | 0.08 m | 急停迟滞宽度（进入用 margin，退出用 margin+此值） |
| `VO_TTC_SEC` | 1.5 s | 预测式避障的期望最小碰撞时间 T |
| `VO_CLEARANCE_M` | 0.19 m | 预测式避障必须保留的净距（= 机体半径） |

避障强度与前进分量缩放**同源**：`fwdScale = (gap − margin) / band`，`avoidStrength = 1 − fwdScale`。因此面板读数与实际刹车力度永远一致，不会出现"显示危险却没刹车"。

**规划式绕行**（`bypassMode`，默认 `false`）：用户直推障碍且完全无横移输入时，主动补一个侧向速度绕过去，而不是原地急停。与 `steerGain` 叠加而不冲突——前者只在"完全没有横移"时兜底，后者只放大已有横移。默认关闭即零回归。

**盲态不是安全**：`obstacleDistance === Infinity` 有两种相反成因——前方确实无障，或这一帧防撞根本是盲的。`deriveHud()` 因此要求必传 `depthTrustworthy`，不可信时判 `Blind`（距离显示 `—`、配色刻意不用绿色）。否则"避障已失效"会被渲染成"∞ 安全"，用户无从察觉。

### 避障演进能力（v0.5.0，全部默认关闭）

反应式避障（`applyAvoidance`）回答的是"离障碍多远时该减到几成"，它有一个结构性盲区：**只看得见一个前向标量**。以下五项逐层补齐，且**每一项默认关闭** —— 关掉时与 v0.4.0 行为逐位一致（由单测与场景守门）。

| 开关 | 补齐的维度 | 关键性质 |
| --- | --- | --- |
| `ceilingAvoidEnabled` | 垂直：上方净距 | 只压上升，**绝不强推下降**（下方可能才是安全区） |
| `trustFallbackEnabled` | 失联时长分档 | normal 全量 / cautious 削弱 / hold 禁爬；本机不支持深度则立即 hold |
| `bypassEnabled` | 绕行方位 | 障碍在右→向左绕，在左→向右绕（此前恒不启用择优） |
| `temporalEnabled` | 时间：滤波 + 迟滞 | 盲帧**原样透传**（绝不插值成安全读数）；硬停面边界不再逐帧翻转 |
| `voEnabled` | 预测：TTC + 速度障碍 | 半空间投影**只减不放**，故"启用只会更安全"；切向保留 → 贴墙滑行 |

预测式层与反应式层是**叠加**关系而非替换：先跑已真机验证的反应式曲线，再把结果投影到速度障碍半空间交集上。这保证既有的飞行手感与 gap≈0.38m 刹停表现不被推翻。

## P5 避障量化指标

`npm run sim:scenarios` 的五个 P5 场景输出同一组指标，用于横向比较与回归：

| 指标 | 口径 | 为什么这样定义 |
| --- | --- | --- |
| 最小净距 | 由**真实几何**（点到平面距离）算 | 不读控制器自报读数，避免"自我证明" |
| 侵入深度 | `max(0, −最小净距)` | 是否真的撞上了 |
| 到达路程 | 水平轨迹长度 | 绕行代价（越短越好） |
| 急停翻转 | `avoidEmergency` 逐帧翻转次数 | 抖动的可量化代理（越小越好） |
| 最小 TTC | 净距 ÷ 净距变化率（有限差分） | 动态障碍同样成立；越大越安全 |

## 里程碑

| 阶段 | 交付件 | 状态 |
| --- | --- | --- |
| M1 建仓与规范 | 独立仓库 / Apache-2.0 / OWNERS / DCO / CI | ✅ 完成 |
| M2 能力剥离与重构 | `core`/`model`/`avoidance`/`telemetry`/`contract` 五模块 + 单测 + 仿真黄金用例入 CI | ✅ 完成 |
| M2+ 第二批剥离 | `dynamics`（刚体动力学）/ `planning`（A\* 规划）/ `environment`（大气）三模块 + `EnvironmentPerception` 契约 | ✅ 完成（v0.2.0） |
| M3 仿真方案与文档 | 脚本化仿真方案（`docs/simulation.md`）、场景示例（`sim/scenarios.ts`）、接入指南（`docs/integration.md`，含对接 Simulator SIG） | ✅ 完成（v0.2.4） |
| M4 高保真层 | `core/SeedPRNG` / `environment/Turbulence`(L3) / `dynamics/EkfEstimator`(L5) / `dynamics/SensorModel`(L6) + `DroneController` L1/L2/L4 接线，全部默认关 | ✅ 完成（v0.4.0） |
| M5 标定与回放 | `dynamics/Px4Params`（PX4 参数映射，静态标定）+ ULog 回放夹具 + 指标标定 | ✅ 完成（v0.4.0） |
| M5+ 避障演进 | P0 接线（垂直/失联/绕行）+ P1 时间维度 + P2 多扇区契约 + P3 预测式速度障碍 + P5 量化指标 | ✅ 完成（v0.5.0） |
| M6 毕业准备 | 架构 SIG 毕业评审材料、QA SIG 准出材料 | ⏳ 待办 |

> v0.2.0 属**超出原提案范围**的新增能力（原提案 M1–M4 未含刚体动力学与路径规划）。
> 向上游贡献时会作为新增交付物一并申报。详见 `CHANGELOG.md`。

## 支持我

如果这个仓库对你的项目有帮助，欢迎请作者喝杯咖啡 ☕：

<!-- GitHub Sponsors 待开通后启用（收款需居住在支持地区，见 docs）：
- [GitHub Sponsors](https://github.com/sponsors/447936551-lang)
-->
- [爱发电](https://afdian.com/a/uavflightsim)（国内用户推荐）

赞助仅代表对开源工作的支持，不影响本项目的技术方向与社区治理（Apache-2.0 / DCO 照常执行）。

## 许可与合规

- 采用 **Apache-2.0** 许可（与 OpenHarmony 社区一致）；所有源码头已标注版权与许可。
- 已签署 **DCO**（开发者原创声明）。
- 本仓库**不含**任何原 App 的私有签名 / 密钥 / 资源，可安全开源。

## 与 OpenHarmony UAV SIG 的关系

- 上游门户仓库：`openharmony-robot/uav`
- 本仓库作为个人孵化仓先行建设，后续按 SIG 流程向上游贡献。
- 本仓库负责"决策 / 物理 / 契约"；渲染、AR 锚点、设备适配由 App / SIG 主仓承担。
