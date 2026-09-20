# uav_flight_sim

无人机飞行仿真 **纯算法层** —— 一套零专有依赖（不引用 `@kit.AREngine` / `ArkGraphics3D` / `hilog`）的飞行控制、转向避障与碰撞检测逻辑。

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
  core/                 纯算法核心（零专有依赖）
    Vec3.ts             自实现三维向量（替代 ArkGraphics3D.Vec3）
    Logger.ts           自实现日志（替代 hilog）
    LoggerLevel.ts      日志级别
    CollisionDetector.ts  碰撞 / 威胁评估（Layer 5 物理层）
    SteeringBehavior.ts   转向 + 避障（Seek/Avoid，速度域集成）
    DroneController.ts    飞控状态机 + 速度积分
    HudModel.ts          HUD 威胁等级推导（纯函数）
    WorldObject.ts       AR 世界对象基类（自实现 Vec3）
    index.ts            统一导出
  perception/
    SpatialPerception.ts  空间感知输入抽象接口（核心契约）
tests/                 vitest 单测 + 零依赖自检入口
sim/
  sim_flight.ts         无头飞行仿真演示（含 MockPerception）
docs/
  design.md             设计说明与验收口径
```

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

## 验收口径（与真机对齐的单一事实来源）

| 阈值 | 值 | 含义 |
| --- | --- | --- |
| `AVOID_MARGIN_M` | 0.4 m | 硬停面：再近则把前进分量清零（实测刹停 gap≈0.38m） |
| `AVOID_SLOW_BAND_M` | 1.2 m | 减速带宽度：从此处向外线性收敛前进分量 |
| `AVOID_SAFE_DIST_M` | 1.6 m | 避障生效距离 = 硬停面 + 减速带 |
| `DRONE_RADIUS_M` | 0.19 m | 机体包围球半径 |
| `AVOID_STEER_GAIN` | 1.25 | 横向绕行增益（越近越鼓励横移贴墙滑行） |

避障强度与前进分量缩放**同源**：`fwdScale = (gap − margin) / band`，`avoidStrength = 1 − fwdScale`。因此面板读数与实际刹车力度永远一致，不会出现"显示危险却没刹车"。

## 许可与合规

- 采用 **Apache-2.0** 许可（与 OpenHarmony 社区一致）；所有源码头已标注版权与许可。
- 已签署 **DCO**（开发者原创声明）。
- 本仓库**不含**任何原 App 的私有签名 / 密钥 / 资源，可安全开源。

## 与 OpenHarmony UAV SIG 的关系

- 上游门户仓库：`openharmony-robot/uav`
- 本仓库作为个人孵化仓先行建设，后续按 SIG 流程向上游贡献。
- 本仓库负责"决策 / 物理 / 契约"；渲染、AR 锚点、设备适配由 App / SIG 主仓承担。
