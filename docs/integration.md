# 接入指南（M3）

> 本文面向**想在本仓算法内核之上搭仿真器 / 飞行模拟器的接入方**：
> 无论是 OpenHarmony 的 **UAV SIG / Simulator SIG** 仿真子项目、独立的 Gazebo/Web 仿真前端，
> 还是只想把飞控逻辑接进自己物理引擎的开发者，都按本文接入。
>
> 脚本化仿真（无头、确定性、CI 门禁）的用法见 [`simulation.md`](./simulation.md)；
> 设计取舍与模块分层见 [`design.md`](./design.md)。

---

## 1. 一句话定位

**本仓只负责"决策 / 物理 / 契约"，不碰渲染、AR 锚点、设备适配。**
所有外部数据——深度、风场、真实位姿——都通过两个**接口契约**注入：

```
   你的仿真器 / 传感器 / 真机
        │  实现 SpatialPerception   ── 喂「前方障碍净距」
        │  实现 EnvironmentPerception ── 喂「风矢量 / 空气密度」
        ▼
   uav_flight_sim（零改动复用）
        ├─ DroneController.update(dt)   ← 唯一驱动入口
        ├─ avoidance / planning          ← 可选：避障、A* 规划
        ├─ dynamics / telemetry          ← 可选：刚体动力学、HUD 推导
        ▼
   你读回 DroneState / offsetX~Z / velX~Z / avoidStrength 做可视化与判定
```

接入的代价：**实现两个接口 + 每帧调一次 `update(dt)`**。控制器代码一行都不用改。

---

## 2. 两个契约（算法与数据源的唯一边界）

### 2.1 SpatialPerception —— 几何感知（前方障碍）

```ts
import { SpatialPerception, Vec3 } from 'uav_flight_sim';

class MyDepthSampler implements SpatialPerception {
  /** 机头前向障碍净距（米）；Infinity = 无障碍 / 这一帧防撞是盲的 */
  getObstacleDistance(): number {
    return this.lastGap;          // 来自你的深度图 / 仿真碰撞探测
  }
  /** 可选：障碍远离单位向量，用于横移参考 */
  getObstacleNormal?(): Vec3 | null {
    return this.awayNormal;
  }
}
```

- 口径必须是"机体中心 → 障碍表面沿机头前向的投影距离"，已是机体到障碍的距离。
- 返回 `Infinity` 时控制器**不限制速度**（盲 ≠ 安全，但强行限制会让无人机僵住）。
- `getObstacleNormal` 不实现，避障退化为"前向减速 + 保留横移"（仍可贴墙滑行）。

### 2.2 EnvironmentPerception —— 大气感知（风场 / 密度）

```ts
import { EnvironmentPerception, Vec3 } from 'uav_flight_sim';

class MyWindSource implements EnvironmentPerception {
  /** 风速矢量（m/s，AR 世界系 X右/Y上/Z朝用户）；null = 无风 / 未接入 */
  getWindVelocity(): Vec3 | null {
    return { x: this.windX, y: 0, z: this.windZ };
  }
  /** 可选：空气密度相对因子（标准海平面 = 1.0） */
  getAirDensity?(): number {
    return this.rho;
  }
}
```

- 风力是"空气相对地面的运动速度"；动力学层据此算**相对气流**阻力（`v_drone − v_wind`）。
- 未接入 / 关闭时返回 `null` → 算法退化为无风飞行，与旧行为**逐位一致**（零回归）。

> ⚠️ **网络取数不入库**。若你的风来自 HTTP 气象接口，请在外面取好、填进实现，
> 不要让算法层直接联网——那会破坏「CI 可离线、确定性」的定位。

---

## 3. 驱动循环（每帧三步）

```ts
import {
  DroneController, SpatialPerception, EnvironmentPerception,
} from 'uav_flight_sim';

const drone = new DroneController();
drone.placed = true;
drone.flying = true;
drone.perception   = new MyDepthSampler();   // 契约注入
drone.environment  = new MyWindSource();
drone.environmentActive = true;              // 环境契约须显式开启（false = 完全不参与）

const DT = 1 / 60;                            // 定步长：与黄金值绑定，勿改

function frame(userStick: { mx: number; my: number; climb: number; yaw: number }) {
  // ① 写入杆量（范围 −1..1；松手归零即停）
  drone.moveX = userStick.mx;
  drone.moveY = userStick.my;
  drone.climbInput = userStick.climb;
  drone.yawInput   = userStick.yaw;

  // ②（若你的感知需要当前位姿）先把无人机世界坐标写给感知桩
  myDepthSampler.setDronePose(drone.offsetX, drone.offsetY, drone.offsetZ, drone.yaw);

  // ③ 算法层前进一步（唯一入口）
  drone.update(DT);

  // ④ 读回状态做可视化 / 判定
  render(drone.offsetX, drone.offsetY, drone.offsetZ,
         drone.pitch, drone.roll, drone.yaw, drone.rotorSpeed);
  if (drone.avoidEmergency) alarm('硬停面急停保持');
}
```

**三条铁律（与 `simulation.md` §3 一致）：**

1. **先喂感知再 `update`**：感知桩读到的是本帧起点位姿，顺序错了黄金值会漂。
2. **只通过 `update(dt)` 驱动**：不要绕过控制器直接改 `velX/offsetX`——那不是在测算法。
3. **`dt` 恒定**：积分器（半隐式欧拉）与减速带曲线都以 `dt=1/60` 标定；变步长需重新标定全部黄金值。

---

## 4. DroneController 公开 API（接入方只读 / 只写清单）

| 类别 | 字段 / 方法 | 说明 |
| --- | --- | --- |
| **生命周期** | `placed: boolean` | 已放置可操作 |
| | `flying: boolean` | 是否积分 / 旋翼转动（`toggleFlight()` 翻转） |
| | `state: DroneState` | 权威态枚举：Unplaced / Grounded / Flying / Landing / EmergencyHold |
| | `reset()` | 复位到初始悬停态 |
| **杆量输入** | `moveX / moveY / climbInput / yawInput` | 各 −1..1；松手归零即停 |
| **感知接入** | `perception: SpatialPerception \| null` | 设了就通过它取障碍距离 |
| | `obstacleDistance: number` | `perception` 为 null 时的直接兜底（单测 / 纯脚本） |
| | `environment / environmentActive` | 大气契约 + 显式开关（`setEnvironment(env)` 一并管理） |
| **位姿输出（读）** | `offsetX / offsetY / offsetZ` | 相对锚点位姿（米） |
| | `velX / velY / velZ` | 速度（米/秒） |
| | `yaw / yawRate / pitch / roll` | 姿态（度） |
| | `rotorSpeed` | 旋翼归一化转速（视觉线索） |
| | `speed`（只读 get） | 水平合速度 `√(velX²+velZ²)` |
| **避障输出（读）** | `avoidanceActive: boolean` | 本帧是否真正在限速度 |
| | `avoidStrength: number` | 0..1 刹车强度；与 HUD 读数**同源**（显示=实际） |
| | `avoidEmergency: boolean` | 进入硬停面急停保持（fail-safe） |
| | `steering: SteeringParams` | 避障增益（可调，含 `bypassMode` / `steerGain`） |
| **参数** | `params: DroneFlightParams` | 飞行动力学参数（改手感用，见默认值） |
| **驱动** | `update(deltaTime: number): void` | **唯一推进入口** |

> 坐标系（AR 世界系，右手系，米）：**X 右 / Y 上 / Z 朝用户**；机头朝前（远离用户）在 yaw=0 时是 **−Z** 方向。
> 因此"满前向推杆"= `moveY = 1`，前向位移沿 **−Z** 累积（`offsetZ` 变小）。

---

## 5. 可选内核（按需取用，零强制）

本仓是分层 barrel，**用到哪个模块就 `import` 哪个**，不用的模块不会拖进你的构建。

### 5.1 规划式绕障（planning）

适合"给定起点终点，自动绕开静态障碍"的仿真任务：

```ts
import { buildOccupancyGrid, planPath, OccupancyGrid,
         DEFAULT_GRID_CONFIG, CELL_OCCUPIED, CELL_FREE } from 'uav_flight_sim';

// 世界平面（地 / 墙） → 2.5D 占用栅格
const grid: OccupancyGrid = buildOccupancyGrid(planes, startX, startZ, DEFAULT_GRID_CONFIG);
// A* 规划（含对角穿角防护 + 起终点就近吸附）+ 可见性平滑
const r = planPath(grid, 0, 0, 3.5, 0);
// r.ok / r.points（PathPoint[]） / r.expanded / r.reason
```

- `grid.at(col, row)` / `grid.isFree(col, row)` / `grid.colOf(x)` / `grid.rowOf(z)` 方便你做可视化或诊断。
- 栅格单元三态：`CELL_FREE` / `CELL_OCCUPIED` / `CELL_UNKNOWN`。**务必给一张地平面**，否则全图 UNKNOWN、起点无处可吸附。
- 场景 mesh → 栅格有替代数据源 `buildGridFromMesh`（来自 `planning/MeshGrid`），规划器一行不改。

### 5.2 真实刚体动力学（dynamics）—— 可选第二内核

`DroneController` 默认走**运动学**内核（对输入积分速度、用速度反推姿态）。
若仿真器想要"四桨推力 → 力矩 → 姿态 → 位姿"的**真实刚体积分**，直接用：

```ts
import { FlightDynamics, DEFAULT_DYNAMICS, mixThrusts, DEFAULT_QUAD } from 'uav_flight_sim';

const dyn = new FlightDynamics(DEFAULT_DYNAMICS);
// 由级联控制器算出四个桨的"期望推力"（N），再经混控器做执行器饱和
const thrusts = mixThrusts(cmd, DEFAULT_QUAD, 1.0);
// 子步推进（建议 h ≤ 1/240）；windX/windZ 为环境风速，airDensity 为相对因子
dyn.step(h, thrusts, windX, windZ, airDensity);
// 读回：dyn.vx/vy/vz、dyn.pitch/roll/yaw、dyn.lastThrust、dyn.lastTauPitch/Roll/Yaw
```

- 两套内核**并列可选**，不是替换关系；默认运动学路径保证既有真机验证的手感与刹停零回归。
- 模式切换用 `dyn.syncFrom(...)` 无缝接管，避免跳变。

### 5.3 HUD 威胁推导（telemetry）

仿真器若要做"避障告警 / 读数面板"，直接复用 HUD 推导纯函数：

```ts
import { deriveHud, HudState, HudThreat } from 'uav_flight_sim';

// distance 来自感知；depthTrustworthy 必填（盲态不渲染成"安全"）；suppressed 必填
const hud: HudState = deriveHud(ThreatLevel, distance, fwdScale, readingDegraded,
                                depthTrustworthy, suppressed);
// hud.threat ∈ { Safe, Blind, Caution, Warning, Danger }；配色 / 文案由 HUD_COLOR_* 提供
```

- `deriveHud` 第六个参数 `suppressed`（前方不可判）与第四个 `depthTrustworthy`（盲态）**必填、无默认值**——默认值会悄悄掩盖"防撞失效"。

---

## 6. 仿真器接入的确定性与离线约束

为了让你的仿真也能进 CI、可复现，请遵守本仓同一套纪律（详见 `simulation.md` §1）：

| 约束 | 接入方注意 |
| --- | --- |
| 无头 | 不依赖 GPU / 显示器；Node 直接跑即可门禁 |
| 确定性 | 定步长、无随机源、无时钟依赖 → 同输入必同输出 |
| 契约注入 | 感知 / 环境全部走两个接口，不要绕过直接改内部状态 |
| 退出码验收 | 关键行为钉成断言，PASS→0 / FAIL→1，当 CI 门禁 |
| 常量即黄金 | 阈值断言引用导出常量（`AVOID_MARGIN_BASE_M` 等），不写死数值 |

内置常量口径（与真机对齐的单一事实来源）：

| 常量 | 值 | 含义 |
| --- | --- | --- |
| `AVOID_MARGIN_BASE_M` | 0.25 m | 硬停面**静止基线**（速度越快硬停面越大，见 `avoidMarginFor(speed)`） |
| `AVOID_SLOW_BAND_M` | 1.2 m | 减速带宽度 |
| `AVOID_SAFE_DIST_M` | 1.6 m | 避障生效距离 = 硬停面 + 减速带 |
| `DRONE_RADIUS_M` | 0.19 m | 机体包围球半径 |
| `AVOID_STEER_GAIN` | 1.25 | 横向绕行增益 |
| `WIND_GAIN` | 0.1 | 风→等效加速度增益（m/s² 每 m/s 风速） |

---

## 7. 对接 Simulator SIG（贡献与协作流程）

> 本节说明本仓如何进入 OpenHarmony 社区协作体系。
> **SIG 名称与仓库路径以社区当前 roster 为准**；若命名有调整，请按 SIG 官网 / 仓库 README 的最新指向替换下述 `openharmony-robot/uav`。

### 7.1 定位与关系

- 本仓是作者自有 OpenHarmony 无人机控制 App（项目名 **AR无人机**）向 **UAV SIG** 孵化的**纯算法子集**；
- 与"仿真"相关的子方向（飞行模拟、联合仿真、回归门禁）天然落到 **Simulator SIG / 机器人仿真** 协作轨道；
- 分工边界：**本仓给"决策 / 物理 / 契约"内核实代码**，**渲染、AR 锚点、设备适配、仿真前端**由 SIG 主仓或下游仿真项目承担。

```
   上游门户：openharmony-robot/uav（UAV SIG）
        │
        ├─ 算法内核（本孵化仓 uav_flight_sim）── 可经 submodule / 源码依赖被主仓引用
        │
        └─ 仿真前端 / 渲染 / 设备适配（SIG 主仓或下游项目，调用本仓 API）
```

### 7.2 贡献流程（DCO 签署）

本仓采用 **Apache-2.0 + DCO（开发者原创声明）**，与 OpenHarmony 社区一致。向本仓或上游提代码：

1. **Fork** 目标仓（`openharmony-robot/uav` 或本孵化仓）到自己账号；
2. 从 `main` 切**特性分支**（`feat/xxx` / `fix/xxx`）；
3. 提交必须带签名：`git commit -s`（会在 commit message 末尾追加 `Signed-off-by: 你的名字 <邮箱>`）；
4. 推送到**双远端**（本孵化仓约定）：
   - `origin` = `https://atomgit.com/Derek0769/uav_flight_sim.git`
   - `github` = `https://github.com/447936551-lang/uav_flight_sim.git`
   - 一次推送：`git push origin main && git push github main`
5. 提 **PR / MR**，由 SIG 评审（代码质量、零专有依赖、黄金值回归）。

> 已签名校验：`npm run check:dco`（CI 门禁之一）。**请勿 amend 已推送的提交或 force-push。**

### 7.3 维护约定（接手方必读）

| 约定 | 含义 |
| --- | --- |
| 零专有依赖 | 任何 `@kit.*` / 网络取数都**不入库**；取数留在你的仿真/App 侧，经契约注入 |
| 黄金值回归 | 改物理参数后，必须同步更新 `tests/golden-flight.test.ts` 与 `sim/scenarios.ts` 的断言 + `simulation.md` §4 表格，**同一次提交内完成** |
| DCO | 每个提交 `-s` 签名 |
| 命名中性化 | 公开仓禁厂商商标缩写；能力用中性名（如 `bypassMode` 而非某商标缩写） |
| 双内核并存 | 运动学（默认）/ 刚体动力学并列，切换由调用方决定，不得悄悄改默认路径 |

### 7.4 最小对接清单（Checklist）

- [ ] 实现 `SpatialPerception`（必需）与 `EnvironmentPerception`（可选，用风则必需）；
- [ ] `drone.perception = ...` 注入；用风则 `drone.setEnvironment(...)` + `environmentActive = true`；
- [ ] 每帧：写杆量 →（可选）喂感知桩 → `drone.update(DT)` → 读回 `offset*/vel*/state/avoidStrength`；
- [ ] 若做自动绕障：接入 `buildOccupancyGrid` + `planPath`（记得给地平面）；
- [ ] 若做真实动力学：接入 `FlightDynamics` + `mixThrusts`（内环子步 `h ≤ 1/240`）；
- [ ] 若做告警面板：接入 `deriveHud`（必填 `depthTrustworthy` / `suppressed`）；
- [ ] 关键行为钉成断言，`process.exit(非0)` 当 CI 门禁；
- [ ] 提交带 `Signed-off-by`（`git commit -s`）。

---

## 8. 常见问题

**Q：我的仿真器用的是右手系但轴朝向不同，怎么办？**
A：本仓固定 **X 右 / Y 上 / Z 朝用户，机头朝前 = −Z**。在感知桩与可视化层做一层坐标变换即可，控制器内部约定不变。

**Q：感知延迟 / 丢帧怎么处理？**
A：感知桩返回的应是"本帧可用的距离"。若这一帧没有可靠读数，返回 `Infinity`（盲态）——控制器不限制速度但不僵死；HUD 侧会渲染成中性灰"测距不可用"，不会假装安全。

**Q：能不能把本仓直接当渲染引擎用？**
A：不能也不该。本仓不含任何渲染 / AR 锚点 / 设备代码（刻意剥离）。请在你自己的仿真前端读取 `offset*/pitch/roll/rotorSpeed` 驱动 3D 节点。

**Q：运动学内核 vs 刚体动力学，仿真器该用哪个？**
A：想要"飞控手感 / 刹停表现与真机零回归"的轻量仿真 → 用 `DroneController`（运动学，默认）。想要"四桨推力如何演化出姿态与抗风"的物理级仿真 → 用 `FlightDynamics`（刚体，需自行接级联控制器与混控）。两者可混合：运动学层算期望，刚体层做呈现。
