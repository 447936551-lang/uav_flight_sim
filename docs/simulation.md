# 脚本化仿真方案（M3）

> 本文回答三个问题：**如何用一条命令跑仿真**、**为什么结果是可复现的**、**CI 如何用它做门禁**。
> 接入方如何把本仓嵌入自己的仿真器，见 [`integration.md`](./integration.md)。

## 1. 定位与原则

脚本化仿真的目标：**不接任何硬件 / AR 引擎 / 网络，在一条命令内端到端验证算法行为**。

| 原则 | 含义 | 违反的代价 |
| --- | --- | --- |
| 无头（headless） | 不依赖显示器、GPU、传感器；Node 直接跑 | 无法进 CI |
| 确定性 | 定步长、无随机源、无时钟依赖 → 同输入必同输出 | 黄金值回归失去意义 |
| 零专有依赖 | 只 import 本仓 `src/`，不引用 `@kit.*` | 破坏三端复用定位 |
| 契约注入 | 感知 / 环境全部通过 `SpatialPerception` / `EnvironmentPerception` 注入 | 算法层反向依赖数据源 |
| 退出码验收 | PASS → 0，任一失败 → 1 | 无法当 CI 门禁 |
| 常量即黄金 | 阈值断言引用导出常量（如 `AVOID_MARGIN_BASE_M`），不写死数值 | 改阈值时黄金值悄悄失效 |

## 2. 两个入口

| 命令 | 脚本 | 面向 | 内容 |
| --- | --- | --- | --- |
| `npm run sim` | `sim/sim_flight.ts` | README 读者 | 最小演示：盲飞穿墙 vs 感知急停（单场景对比） |
| `npm run sim:scenarios` | `sim/scenarios.ts` | 接入方 / CI 冒烟 | 五场景全集：覆盖 model / avoidance / environment / planning 四个域 |

```
$ npm run sim:scenarios
=== uav_flight_sim 场景仿真（M3 脚本化仿真方案） ===
定步长 dt=1/60 · 无随机源 · 感知/环境走契约注入

  [PASS] 盲飞穿墙（危险基线）            minGap=-21.67m
  [PASS] 感知驱动硬停（SpatialPerception） finalGap=0.25m（= AVOID_MARGIN_BASE_M）
  [PASS] 侧风漂移（EnvironmentPerception） 无风 0.0000m / 3m/s 侧风 0.99m
  [PASS] 绕行方向择优（bypass 纯函数）     障碍右→绕左 / 障碍左→绕右 / 死区→回落右
  [PASS] A* 规划绕障（planning）          路点=4  最大绕行|z|=2.30m

结论：5/5 场景通过 —— 纯算法层在无头环境下行为符合验收口径。
```

## 3. 仿真循环骨架

所有场景共用同一个循环形状（摘自 `sim/scenarios.ts`）：

```ts
const DT = 1 / 60;                       // 定步长：与黄金值绑定，不要改
const drone = new DroneController();
drone.placed = true;
drone.flying = true;
drone.perception = wallPerception;       // 契约注入（或 obstacleDistance = Infinity）
drone.moveY = 1;                         // 杆量：满前向推杆

for (let i = 0; i < frames; i++) {
  wallPerception.setDroneZ(drone.offsetZ); // ① 仿真器把世界状态喂给感知桩
  drone.update(DT);                        // ② 算法层前进一步（唯一入口）
  // ③ 记录观测：gap / avoidEmergency / 位置 …
}
```

三条铁律：

1. **每帧先喂感知再 update**：感知桩读到的必须是本帧起点位置，顺序错了黄金值会漂。
2. **只通过 `update(dt)` 驱动**：不要绕过控制器直接改 `velX/offsetX`——那不是在测算法。
3. **dt 恒定**：积分器（半隐式欧拉）与减速带曲线都以 dt=1/60 标定；变步长需要重新标定全部黄金值。

## 4. 黄金值回归红线

改任何物理参数（`DEFAULT_FLIGHT_PARAMS` / `AVOID_*` / `WIND_GAIN` …）都可能移动黄金值。
当前红线（由 `tests/golden-flight.test.ts` 与 `sim/scenarios.ts` 双重守护）：

| 场景 | 黄金值 | 说明 |
| --- | --- | --- |
| 盲飞 20s（满前向） | `minGap = −21.67m` | 危险基线，**自 M2 起不变** |
| 感知硬停 20s | `finalGap = AVOID_MARGIN_BASE_M = 0.25m`，零过冲（minGap = finalGap） | 0.25 = 静止硬停面基线；⚠️ 旧值 0.40 已随「阈值挂速度」废弃（v0.2.3） |
| 无风悬停 10s | 漂移 = 0（逐位） | 环境层零回归硬保证 |
| 3m/s 侧风悬停 10s | 漂移 ≈ 0.99m | 随 `WIND_GAIN` 联动 |

流程：改参数 → `npm run sim:scenarios` 看哪个场景 FAIL → **确认新值在物理上更正确** → 同步更新
`tests/golden-flight.test.ts` 断言与本表 → 让 App 侧孪生（如适用）同步 → 一次 commit 内完成。

## 5. 与单测的分工

| 手段 | 文件 | 职责 |
| --- | --- | --- |
| vitest 单测 | `tests/*.test.ts` | 模块级性质断言（参数透传、边界、回归） |
| 仿真黄金用例 | `tests/golden-flight.test.ts` | 把演示场景**钉死成确定性回归护栏**（CI 门禁） |
| 无头仿真 | `sim/sim_flight.ts` | 面向人的最小演示 |
| 场景仿真 | `sim/scenarios.ts` | 跨模块端到端冒烟 + CI 门禁 |
| 零依赖自检 | `npm run selfcheck` | 各模块内置 `selfTest()`，不装 vitest 也能验收（真机 / Node 通用） |

同一行为不重复钉两遍：**数值黄金值归 golden test 与场景脚本**，单测只钉性质（单调性、透传、零回归）。

## 6. CI 集成

`.github/workflows/ci.yml`（本地等价命令 `npm run verify`）：

```
typecheck → vitest 单测 → npm run sim → npm run sim:scenarios → DCO 检查
```

场景脚本失败（断言不过 / 抛异常）即非零退出，流水线直接红——这是「黄金值漂移」的第一发现渠道。

## 7. 如何新增一个场景

1. 在 `sim/scenarios.ts` 写一个 `function scenarioXxx(): ScenarioOutcome`（确定性：定步长、无随机源）；
2. 断言引用导出常量，不写死阈值数值；
3. 注册进 `SCENARIOS` 数组（失败自动拉低退出码）；
4. 若该场景沉淀了新的**跨模块黄金值**，同步补进 `tests/golden-flight.test.ts` 与本文 §4 表格。

> 新场景的门槛：**能讲清"它防住哪类回归"**。纯展示性的场景放 README / demo，不进门禁。
