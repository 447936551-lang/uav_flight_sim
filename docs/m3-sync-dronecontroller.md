# M3 同步方案：DroneController 双内核 + 闭环保持 + 执行器动力学 + 软降落锥减

> 状态：**规划中**（2026-10-10 起）。来源：App 端 `ar/DroneController.ets`（2327 行，Phase 9 真实飞控）。
> 仓库既有同步红线：断言数对齐 App、`tsc`/`vitest`/`sim`/`check:dco` 全绿、commit `-s` + 双远端（atomgit + github）推送。

## 0. 开源边界判定（合规优先）
- **可纳入**：`RotorMixer` / `FlightDynamics` / `CascadeController` 三个纯物理模块**已在社区**（`src/dynamics/`），本次只是把它们**接线进 `DroneController`** + 搬双内核状态机 + 闭环保持 + 执行器滞后 + 软降落锥减。全部不依赖 `@kit.*`，合规。
- **不可纳入**：App 的 `FlightAlgorithm` 算法竞技框架（`activeAlgo` / `flightAlgoRegistry` / `buildAlgoContext` / `apas` 扩展点，App `DroneController.ets:60-66, 604, 949, 985-1010`）。社区不引入该框架——`setFlightMode` 只管「内置运动学 ↔ 内置真实动力学」两态。若想保留「算法扩展点」，最多留一个可选钩子类型（不引入注册表），默认 `null`、零回归。

## 1. 目标
把 App `ar/DroneController.ets` 的双内核集成复刻到社区 `DroneController.ts`，使社区可：
- `setFlightMode(true)` 切到**真实动力学**内核（级联 + 刚体 + 混控 + 执行器滞后），手感与 App 一致；
- `setFlightMode(false)`（默认）下运动学内核**逐字节零回归**；
- 运动学保持环升级为**闭环**（复用 `CascadeController.positionHoldVelocity`，与动力学位置环同源，App `:224/370/373/1213`）；
- 软降落改为**锥形减速**触地（App `:260-267/1312/1479`）。
- **保留**社区领先于 App 的 `SpatialPerception` / `EnvironmentPerception` 契约注入（App 没有，社区有）。

## 2. 改动清单

### 2.1 第一步：运动学保持环闭环化（最小、易验）
新增字段：`holdX: number = 0`、`holdZ: number = 0`、`holdEngaged: boolean = false`、`holdTargetActive: boolean = false`（默认 false，仅作占位/语义对齐）。
新增常量：`KIN_WIND_FEEDFORWARD = 0.9`、`KIN_POS_KP = 1.5`、`KIN_HOLD_RESP = 6.0`、`KIN_HOLD_AUTHORITY = 1.0`。
改写 `update()` 的「无输入」分支：当 `environmentActive || holdTargetActive` 时，
  1. 首次接合：`holdX = offsetX; holdZ = offsetZ; holdEngaged = true`（除非 holdTargetActive）；
  2. 前馈：`velX += -windAccelX * KIN_WIND_FEEDFORWARD * dt`（顶风）；
  3. 反馈（非急停时）：`hv = positionHoldVelocity(holdX, holdZ, offsetX, offsetZ, KIN_POS_KP, maxSpeed * KIN_HOLD_AUTHORITY)`；
  4. 位置保持速度**先过避障**再融合（`applyAvoidance(..., false, obstacleLateral, curSpeed)`）；
  5. 平滑趋近：`velX += (hx - velX) * clamp(KIN_HOLD_RESP * dt, 0, 1)`。
`environmentActive=false` 且 `holdTargetActive=false` 时仍走原阻尼衰减、`holdEngaged=false` → **默认零回归**。
`CascadeController.positionHoldVelocity` 签名 `(holdX, holdZ, x, z, kp, maxSpeed) => number[]` 与 App 一致，可直接复用。

### 2.2 第二步：双内核 + 执行器动力学 + 软降落接线
新增字段：`useDynamics: boolean = false`、`dynamics: FlightDynamics | null = null`（懒建，复用 `DEFAULT_DYNAMICS`）、`rotorPlant: RotorPlant | null = null`（懒建，传 `DYN_MOTOR_TAU`）、`rotorSpeeds: number[] = [0,0,0,0]`。
新增方法：`setFlightMode(dynamics)`（App `:922`，内核切换 + 状态无缝交接）、`dyn()`/`plant()` 懒构造（App `:1022/1030`）。
`update()` 内 `if (this.useDynamics) { FlightDynamics.step + mixThrusts + RotorPlant 滞后 }` 与运动学并存（App `:1105/1439/1622/1764`）。
软降落锥减：Landing 态用 `LAND_TAPER_BAND=0.4` / `LAND_TOUCHDOWN_SPEED=0.12` 限速触地（App `:255-267`）。
**排除** `activeAlgo`/`FlightAlgorithm` 框架相关，绝不搬。

### 2.3 合并难点（必须处理）
1. **保留社区契约注入**：社区 `update()` 已从 `this.perception`/`this.environment` 取数；App 是直接赋值 `obstacleDistance`。移植时把 App 的运动学/动力学逻辑**适配**到社区契约入口，不能覆盖掉契约。
2. **Vec3 元组→对象**：App 用 `[x,y,z]`（如 `rotorSpeeds`、`positionHoldVelocity` 向量参），社区是 `{x,y,z}`，改 `.x/.y/.z` 或 `vec3()`。
3. **selfTest 断言数对齐**：App `DroneController.selfTest` 含双内核/闭环/软降落断言（如 `:1896 切换动力学→useDynamics 生效`）。社区 `selfTest` 须同步加等价断言，并核对断言总数与 App 逐项同数。

## 3. 验证红线
- `npm run verify` 全绿；`selfTest` 断言数对齐 App。
- 黄金值：盲飞 `−21.67m`、感知急停 `0.25m` **不变**；新增 `useDynamics=true` 路径须有专属 `sim/scenarios.ts` 场景验证动力学下的急停距离/手感。
- `setFlightMode(false)` 默认路径与现社区运动学**逐位一致**。

## 4. 文档与版本
- `CHANGELOG.md` 新条目；`docs/extraction-map.md` 的 DroneController 行标注「双内核集成已于 v0.3.0 纳入」；README 的 DroneController API 清单补 `setFlightMode`/`useDynamics`/`rotorSpeeds`。
- 版本号：**v0.3.0**（新增整块内核能力、公开 API 为增量、向后兼容 → semver minor）。
- commit `-s` + 双远端（atomgit + github）推送（M3 整段完成后统一版本号+推送，开发期中间 commit 暂留本地）。

## 5. 风险
- 这是一次 **DroneController 重新孪生合并**，不是简单加开关——社区运动学内核已与 App 漂移（闭环保持是新增到运动学路径的），且社区有 App 没有的契约注入。**分两步提交**：① 运动学保持环闭环化；② 双内核 + 执行器动力学 + 软降落接线。
- App `update()` 是 2327 行大函数，移植易漏边界；配合 App 侧 `selfTest` 断言逐项搬，用断言锁死行为。
