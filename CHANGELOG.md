# 更新日志

本项目采用 [语义化版本](https://semver.org/lang/zh-CN/)。所有条目均标注来源模块与验证方式。

源码溯源与取舍对照见 [`docs/extraction-map.md`](./docs/extraction-map.md)。

---

## [0.2.4] — 2026-10-08

M3 收口：**脚本化仿真方案 + 场景示例 + 接入指南**（三项交付件全部落地，含对接 Simulator SIG 的协作流程）。
本次**无算法/API 变更**，纯文档与仿真脚本，属 patch 级交付。

> 背景：M3 的目标是让本仓「可在一条命令内被 CI / 外部仿真器复用」。
> 前两个交付件（`docs/simulation.md` 脚本化仿真方案、`sim/scenarios.ts` 五场景确定性冒烟）
> 已在本次一并完成并验证；本版补齐第三交付件 `docs/integration.md`（外部仿真器 / Simulator SIG 接入指南）。

### Added（新增）

**`docs/integration.md` — 接入指南（含对接 Simulator SIG）**

- 外部仿真器 / 真机 / Simulator SIG 的接入路径：实现 `SpatialPerception` 与 `EnvironmentPerception`
  两个契约、每帧 `drone.update(dt)`、`drone.perception = ...` 注入、读回 `offset*/vel*/state/avoidStrength`。
- `DroneController` 公开 API 清单（生命周期 / 杆量 / 感知接入 / 位姿输出 / 避障输出 / 参数 / 驱动）。
- 可选内核取用指南：**planning**（`buildOccupancyGrid` + `planPath` 自动绕障）、
  **dynamics**（`FlightDynamics` + `mixThrusts` 真实刚体积分，内环子步 `h ≤ 1/240`）、
  **telemetry**（`deriveHud` 告警面板，必填 `depthTrustworthy` / `suppressed`）。
- 「对接 Simulator SIG」专章：与 UAV SIG 关系、上游门户 `openharmony-robot/uav`、DCO 签署的贡献流程
  （fork → 特性分支 → `git commit -s` → 双远端 `origin`(atomgit)+`github` 推送 → PR 评审）、维护约定与最小对接 Checklist。
- 常见问题：坐标轴向、感知延迟、渲染分离、运动学 vs 刚体内核选型。

**`sim/scenarios.ts` — 五场景确定性冒烟**（CI 门禁，已在本次一并落地）

- 盲飞穿墙 / 感知驱动硬停 / 侧风漂移 / 绕行方向择优 / A\* 规划绕障，退出码验收（PASS→0 / FAIL→1）。
- 详见 `docs/simulation.md` 与 `sim/scenarios.ts` 文件头注释。

**`docs/simulation.md` — 脚本化仿真方案**（已在本次一并落地）

- 六原则（无头 / 确定性 / 零专有依赖 / 契约注入 / 退出码验收 / 常量即黄金）、两个入口、循环骨架、
  黄金值回归红线、与单测分工、CI 集成、如何新增场景。

**`package.json` / `package-lock.json`**

- 新增脚本 `sim:scenarios`（`tsx sim/scenarios.ts`）；
- `verify` 链追加 `sim:scenarios`（typecheck → vitest → sim → sim:scenarios → check:dco）；
- 版本号同步升至 `0.2.4`（package.json 与 package-lock.json 两处）。

### Changed（变更）

| 项 | 说明 |
| --- | --- |
| 无 | 本次未改动任何算法逻辑 / 公开 API / 阈值常量（零回归，纯文档 + 仿真脚本） |

### Verified（验证）

| 项 | 结果 |
| --- | --- |
| `tsc --noEmit` | ✅ 通过 |
| `vitest run` | ✅ 85 passed（与 0.2.3 持平，本次无新增单测） |
| 零依赖自检 | ✅ 全部模块 `selfTest` 通过 |
| `sim/sim_flight.ts` | ✅ 退出码 0 |
| `sim/scenarios.ts`（`npm run sim:scenarios`） | ✅ **5/5 PASS，退出码 0**（盲飞 −21.67m / 感知急停 0.25m / 侧风 0.99m / 绕行方向正确 / A\* 绕行 \|z\|=2.30m） |
| `check:dco` | ✅ 通过 |

**未纳入（本次仅文档/M3，非代码改动）**：

1. App 侧 `ARDepthSampler` 锥角加密与支撑面分段容差仍依赖 `@kit.AREngine`，按抽取清单不入库。
2. `DroneController` 孪生仍只含运动学内核；级联控制 / 闭环定点 / 软降落锥减属双内核集成（M3 规划），未随本次文档工作同步。
3. 本机 vitest 并行 fork 池偶发 worker 崩溃（与测试逻辑无关；串行 85/85 稳定），仍建议 `--no-file-parallelism`。

---

## [0.2.3] — 2026-10-06

随 App 侧避障打磨同步：**硬停面从定值改为随速度伸缩**（运动学自洽），外加绕行方向择优。
这也是一次"同步即质检"的版本 —— 搬运过程中抓出并修掉了 App 侧自检的 3 条缺陷断言。

> 背景：0.2.2 及之前的硬停面 `AVOID_MARGIN_M=0.4` 是**定值**，与速度脱钩。
> 运动学上不自洽：1.5m/s 的刹车距离 `1.5²/(2×1.5)=0.75m` **大于** 0.4m 硬停面 ——
> 满速冲向障碍时，等开始减速就已经撞上了。这正是真机「明明减速了还是碰到」的根因。

### Added（新增）

**`avoidance/CollisionDetector.ts` — 速度相关阈值（与 App 侧同源）**

| 导出 | 说明 |
| --- | --- |
| `AVOID_MARGIN_BASE_M = 0.25` | 硬停面**静止基线**（机体半径 0.19 + 0.06 余量） |
| `AVOID_BRAKE_ACCEL = 1.5` | 避障制动减速度（速度包线控制实测保守值） |
| `avoidMarginFor(speed)` | `margin(v) = 0.25 + v²/3`：静止 0.25m、满速 1.0m（恰好覆盖刹车距离） |
| `avoidSafeDistFor(speed, band)` | `= margin(v) + band`：减速带宽度不变，整体预警距离随刹车需求伸缩 |
| `AVOID_CORRIDOR_HALF_M` / `AVOID_BELOW_PATH_M` | 前向碰撞走廊半宽 / 可飞越判据（App 侧走廊几何常量，随孪生同步） |
| 上方净距纯函数 | `evaluateCeiling` / `sensorClimbScale` 等（爬升维度的避障，App 侧 P1 演进） |

**`avoidance/SteeringBehavior.ts` — 绕行方向择优（App 侧 P1-1/P1-2 演进）**

- 新增 `pickAvoidSide(lateral, deadzone)`：障碍在右 → 向左绕，在左 → 向右绕；
  死区（0.12m）内返回 0，退回旧的「向右绕」行为，防读数噪声导致绕行方向逐帧翻转。
  旧实现硬编码永远向右 —— 障碍偏左时会朝障碍那侧冲，表现为「在障碍前卡住不动」。
- `AvoidResult` 新增 `avoidedLeft` / `avoidedRight`：HUD 可显示「正在向左/右绕行」。
- `applyAvoidance()` 新增 2 个**可选**参数：`obstacleLateral`（横向偏置）、
  `speed`（当前速度）。有默认值，**非破坏性**（旧 6 参调用逐字节等价）。

**`tests/` — 4 条速度语义用例**：满速急停 / 静止仅减速（正反双向）/
满速强度 ≥ 静止 / 显式 emergencyDist 优先于速度推导。

### Changed（变更）

| 项 | 说明 |
| --- | --- |
| 硬停面语义 | `applyAvoidance` 的 `DEFAULT_STEERING` 路径：定值 0.4 → `avoidMarginFor(speed)` 动态推导；params **显式**指定值仍优先（老契约不变）。`evaluate()` 独立默认值保持 `AVOID_MARGIN_M=0.4` 不变 |
| 黄金值·感知驱动 | 硬停面 `0.40m` → **`0.25m`**（静止基线 `margin(0)`）。轨迹探针证实单调逼近零过冲（`minGap = finalGap`）。**语义变更而非回归** |
| 黄金值·盲飞 | `-21.67m` **不变** ✅ |
| `model/DroneController.ts` | 运动学内核调用点接入当前实际水平速度（与 App 侧同口径）；bypass 接线归 M3 |
| 命名分叉 | App 侧 `apas*`（商标缩写）→ 仓库侧 `bypass*`，共 18 处（延续 0.2.x 去商标化规则） |
| App 侧自检修复×3 | ①「显式优先」断言用 `emergencyDist=0.4` 当显式值，恰等于默认值 → 前提自反（改 0.6）；②③ 1.0m 档位断言与公式自反（满速时 1.0m 恰在硬停面上是 `stop` 非 `slow`；静止时 1.0 < 1.45 是 `slow` 非 `safe`）→ 改用档位真正翻转的 1.5m 对照 + 边界 `stop` 钉死。**源头在 App 侧修，仓库侧随同源搬运** |

### Verified（验证）

| 项 | 结果 |
| --- | --- |
| `tsc --noEmit` | ✅ 通过 |
| `vitest run` | ✅ **85 passed**（0.2.2 为 81，新增 4 例；`--no-file-parallelism` 下稳定全绿） |
| 零依赖自检 | ✅ `SteeringBehavior.selfTest` **31 项** / `CollisionDetector.selfTest` **52 项**，与 App 侧逐项同数 |
| 仿真黄金用例 | ✅ 盲飞 `-21.67m` 不变；感知驱动 `0.25m`（新语义，见 Changed） |
| `sim/sim_flight.ts` | ✅ 退出码 0，结论对比恢复正常输出 |
| `check:dco` | ✅ 通过 |

**已知遗留（未修）**：

1. `model/DroneController.ts` 孪生仍只含**运动学内核**。App 侧自 0.2.2 以来还积累
   级联控制接入、闭环定点悬停、软降落锥形减速等演进 —— 属双内核集成（M3）范围，
   本版未同步。
2. App 侧 `ARDepthSampler` 的锥角加密（13 射线/9 档/5×5 邻域）与支撑面分段容差
   （低空 0.05m / 高空 0.25m）依赖 `@kit.AREngine`，按抽取清单不入库。
3. 本机 vitest 并行 fork 池偶发 worker 崩溃（与测试逻辑无关；串行 85/85 稳定）。

---

## [0.2.2] — 2026-09-25

补齐 0.2.1 同步时**仍然漏掉的一支**：HUD 抑制态（`suppressed`）。

> 背景：0.2.1 把 `Infinity` 拆成了「真安全」与「盲」两种语义，但同一真机缺陷的
> **第三支**当时尚未定性 —— `Infinity` 还有第三种成因：**测到了，却把所有前方命中
> 用自己的几何判据剔干净了**。这一支同样被旧实现渲染成绿色「∞ 安全」，
> 也就是用户看到的「明明贴着障碍，面板却说安全」。本次随 App 侧一起收口。

### Added（新增）

**`telemetry/HudModel.ts` — 抑制态 `suppressed`（前方不可判）**

`obstacleDistance === Infinity` 现共三种成因，各有各的话要说：

| 成因 | 真实含义 | 展示 |
| --- | --- | --- |
| 前方 3m 内确实无遮挡 | 真的安全 | 绿「安全 / ∞」 |
| 这一帧防撞是盲的 | 什么都不知道 | 灰「测距不可用 / —」 |
| **命中全被支撑面/走廊/可飞越剔除** | **没有任何结论** | 灰「前方不可判 / —」 |

- 新增 `HudState.suppressed` 与 `deriveHud()` 第 6 个**必填**参数 `suppressed`
- 抑制态**不改 `level`**：等级仍完全由物理阈值决定，因此**不产生额外响铃**
  （下游告警按枚举数值做边沿比较，动 `level` 就会凭空多响一次）
- 复用盲态中性灰而非新增色：两者同属「不知道」，用同一色系表达同一认知状态，
  仅以文案区分成因，避免与三级威胁色（黄/橙/红）抢占语义
- 抑制态仅在**本帧没有有限读数**时接管；一旦真有读数就照旧报数 ——
  把真实读数藏起来是第二重失真

### Changed（变更）

| 项 | 说明 |
| --- | --- |
| `deriveHud()` 签名 | 新增第 6 个**必填**参数 `suppressed`（**破坏性变更**，与 0.2.1 加参同规） |
| 文件同源 | 本文件与 App 侧 `utils/HudModel.ets` 做了全量 diff，确认**差异仅为两条 import 路径**，无逻辑漂移 |

### Verified（验证）

| 项 | 结果 |
| --- | --- |
| `tsc --noEmit` | ✅ 通过 |
| `vitest run` | ✅ **81 passed**（原 75，新增抑制态 6 例） |
| 零依赖自检 | ✅ `HudModel.selfTest` **43 项断言全通过**（原 20 项，与 App 侧逐项同数） |
| 仿真黄金用例 | ✅ **数值未变**（盲飞 `-21.67m` / 感知驱动 `0.40m`） |
| 反向断言 | ✅ 无抑制时必须是老实的 `安全/∞`，否则日常畅通会被整体误报成「不可判」 |
| 真机印证 | ✅ App 侧真机日志实测到 `sup=1` 帧（45 探针中 29 个判支撑面、其中 2 个属「本该采纳却被支撑面吃掉」），HUD 由绿「∞ 安全」变灰「前方不可判」 |

**已知遗留（未修）**：`suppressed` 只在本帧无有限读数时接管。若被剔除的命中
比被采纳的那条**更近**（例如被吃掉 0.3m、采纳的是 2.0m），面板仍会报 `2.00 m / 安全`。
根治需先补「被抑制命中自身的坐标 / 法向 / gap」诊断字段，否则无法裁定其为真阳性还是误报。

---

## [0.2.1] — 2026-09-25

对齐原 App 侧第二批演进，补齐**此前遗漏的「原地进化」类改动**。

> 背景：0.2.0 只搬运了 App 中**新出现**的文件，未对**已迁移过的旧文件**做二次 diff，
> 导致 `SteeringBehavior` / `HudModel` 两个文件在 App 侧的后续演进全部滞留在 App 里。
> 本次做了一次全量 diff 并补齐，其余 8 个已迁移文件经核对**无逻辑漂移**
> （差异仅为 import 路径与尾逗号）。

### Added（新增）

**`avoidance/SteeringBehavior.ts` — 规划式绕行（bypassMode，默认关闭）**

用户直推障碍且**完全无横移输入**时，主动补一个侧向速度（沿机头右向）绕过去，
而不是原地急停。与既有「横向绕行增益」(`steerGain`) 叠加而不冲突：
`steerGain` 只放大用户已有的横移，本分支在「完全没有横移」时才兜底介入。
侧向强度 ∝ 前进分量 × 避障强度，越近绕得越急；结果仍受 `maxSpeed` 矢量限幅。

**`telemetry/HudModel.ts` — 盲态（Blind）与代测（degraded）语义拆分**

修掉一个真机可观察的错误：`obstacleDistance === Infinity` 有**两种相反**成因 ——
(a) 前方确实无障碍；(b) 这一帧防撞根本是「盲」的。旧实现把 (b) 一并归入 Safe，
于是「跟踪丢失、避障彻底失效」时 HUD 反而稳稳显示「∞ 安全」，用户完全无法察觉。

- 新增 `HudThreat.Blind`（排位在 Safe 与 Caution 之间：比安全更值得注意，但不触发告警）
- 判据由调用方传入 `depthTrustworthy`（**必填**，无默认值 —— 默认值会悄悄掩盖「盲」）
- 盲态距离文案为 `—` 而非 `∞`：**不把「不知道」渲染成一个具体读数**
- 盲态配色为中性灰蓝，**刻意不用绿色**（绿色会被读成安全）
- 新增 `degraded`：距离来自「相机代测」回退时，等级仍按物理阈值，仅改写文案与配色
- 危险色 `#FF5B5B` → `#FF8080`（HUD 底板对比度 5.2:1，满足 UX 标准正文 > 4.5:1）

### Changed（变更）

| 项 | 说明 |
| --- | --- |
| `deriveHud()` 签名 | 新增第 4/5 个**必填**参数 `depthTrustworthy` / `degraded`（**破坏性变更**） |
| `applyAvoidance()` 签名 | 新增第 6 个**可选**参数 `bypassMode`（默认 `false` → 零回归） |
| 命名中性化 | 绕行能力在业界常以某厂商注册商标缩写指代，本仓库以 Apache-2.0 公开分发并拟送 SIG 评审，统一改用中性名称 `bypassMode` / `BYPASS_STEER_GAIN` /「规划式绕行」，代码中不再出现该缩写 |
| barrel | `src/index.ts` 增补 `HUD_COLOR_BLIND` / `HUD_COLOR_FALLBACK` 导出 |

### Verified（验证）

| 项 | 结果 |
| --- | --- |
| `tsc --noEmit` | ✅ 通过 |
| `vitest run` | ✅ **75 passed**（原 59，新增 16） |
| 仿真黄金用例 | ✅ **数值未变**（盲飞 `-21.67m` / 感知驱动 `0.40m`） |
| 零依赖自检 | ✅ 12 个模块全 PASS |
| DCO | ✅ 5 个提交均已签名 |

零回归由测试钉死：`bypassMode` 不传与显式传 `false` **逐位等价**；
可信前提下 HUD 原有阈值口径逐条断言不变。

---

## [0.2.0] — 2026-09-24

从作者自有 OpenHarmony 无人机控制 App（项目名 AR无人机）**第二批剥离**：在既有「飞控 + 避障」之上，
补齐**真实刚体动力学**、**路径规划**与**环境感知**三块能力，并新增第二个输入契约。

> 本次迁移**零逻辑改动**：各文件的公式、常量、阈值与内置 `selfTest` 均原样搬运，
> 只改写 import 路径。因此既有飞行手感、刹停表现与全部黄金用例数值**无回归**
> （`tsc` + 52 项单测 + 无头仿真 + DCO 门禁全绿）。

### Added（新增）

**`src/dynamics/` — 真实刚体飞行动力学**（依赖 `core`）

| 文件 | 行数 | 内容 |
| --- | --- | --- |
| `RotorMixer.ts` | 420 | 四旋翼混控（Walsh–Hadamard，正交可逆）+ 执行器饱和 + `RotorPlant` 电机一阶滞后与每桨差动 |
| `FlightDynamics.ts` | 320 | 刚体动力学积分器（半隐式欧拉）；阻力基于**相对气流**而非绝对速度 |
| `CascadeController.ts` | 219 | 级联控制纯函数：位置环 / 速度环 / 推力矢量解算 / 倾角反解 / 姿态 PD / 阻力前馈 |

关键设计：**执行器饱和做在转速上**（真实上限是 RPM 不是推力），且饱和后机身受到的
力矩由**实际**推力反解 —— 即"物理诚实"：控制器想要多少不重要，电机真给得出多少才算数。

**`src/planning/` — 占用栅格与路径规划**（依赖 `core` / `avoidance`）

| 文件 | 行数 | 内容 |
| --- | --- | --- |
| `PathPlanner.ts` | 1022 | 世界平面 → 2.5D 占用栅格 → A\*（含对角穿角防护、起终点就近吸附）→ 可见性平滑 |
| `MeshGrid.ts` | 569 | 场景 mesh → 占用栅格，作为平面栅格的**替代数据源**（规划器一行不改） |

关键设计：MeshGrid 把「FREE / OCCUPIED」分别栅格化到两张临时掩膜，最后按
`OCCUPIED > FREE > UNKNOWN` 统一合成 —— 合成结果与遍历顺序无关，**可复现**。

**`src/environment/` — 大气换算与天气归一化**（依赖 `core`）

| 文件 | 行数 | 内容 |
| --- | --- | --- |
| `Atmosphere.ts` | ~230 | 气象风向 → 风矢量、气压/气温 → 空气密度因子、低通平滑、阵风扰动 |
| `WeatherCode.ts` | ~180 | 风速单位折算、异常值防护、WMO 天气代码中文化 |

关键设计：**网络取数不入库**。天气数据来自 HTTP 接口（带时延、会失败、单位随服务商变化），
放进算法库会直接破坏「CI 可离线、确定性」的定位。因此只保留纯换算，
取数/定时刷新/写回控制器留在 App 侧。

**`src/contract/EnvironmentPerception.ts` — 第二个输入契约**

与 `SpatialPerception` 同一套设计思路的第二个实例：前者回答「前方多远有障碍」（几何），
后者回答「周围空气什么状态」（大气）。算法层只声明需要什么，不关心数据从哪来。

### Changed（变更）

- `src/model/DroneController.ts`：新增**可选**环境输入（`environment` / `environmentActive` /
  `windVelX/Z` / `windAccelX/Z` / `airDensityFactor`）与 `setEnvironment()`，
  使 `EnvironmentPerception` 契约被真正消费，而不是一个装饰性接口。
  默认关闭 ⇒ **零回归**（已由单测逐位对比钉死：不接环境 / 接入但关闭，
  与旧行为输出完全一致）
- `src/index.ts`：barrel 扩容，新增 3 个模块与第二个契约的全部公开导出（同名 `selfTest` 逐个别名化）
- `tests/run-self-tests.ts`：零依赖自检入口纳入 8 个新自检（原 4 → 现 12 个模块）
- `tests/self-tests.test.ts`：新增 dynamics / planning / environment 三个 describe 块
- `tests/dynamics-planning-environment.test.ts`（新）：25 项**行为**测试，不只跑 `selfTest`
- `tests/environment-integration.test.ts`（新）：7 项契约接入测试，
  核心是「不接环境 / 接入但关闭 ⇒ 与旧行为**逐位一致**」的零回归守护

### Verified（验证）

| 项 | 结果 |
| --- | --- |
| `tsc --noEmit` | ✅ 通过 |
| `vitest run` | ✅ **59 passed**（原 19，新增 40） |
| `npm run selfcheck` | ✅ 12 个模块自检全通过 |
| 无头仿真黄金用例 | ✅ 盲飞 `−21.67m` / 感知驱动 `0.40m`（**数值未变，零回归**） |
| DCO 校验 | ✅ 通过 |

### Not included（本次**故意不入库**的部分）

| 项 | 原因 |
| --- | --- |
| 算法竞技框架（`FlightAlgorithm` 契约与注册表）及依赖它的四个任务算法（智能绕障 / 航点 / 返航 / POI 环绕） | 按项目决策：该框架及其算法**暂不纳入本仓**；其中「绕障」的**算法本体**已随 0.2.1 以 `bypassMode` 形式并入 `avoidance/SteeringBehavior`，不依赖该框架 |
| 天气 HTTP 取数（`WeatherService` 的网络部分） | 引入网络依赖，破坏离线确定性 |
| AREngine 系列（深度采样 / 平面跟踪 / mesh 探测）、`HoldTargetVisual` / `HorizonVisual` | 依赖 `@kit.AREngine` / `@kit.ArkGraphics3D`，属专有套件 |
| 端侧推理（`AiDevice` / `AiEngine` 等） | 依赖 `@kit.MindSporeLiteKit`，属设备能力层 |
| `AlertService`（振动 / 提示音告警） | 依赖 `@kit.SensorServiceKit` / `AudioKit` / `MediaKit`，属 UI 反馈层 |

---

## [0.1.0] — 2026-09-22

首个可交付版本（M1 + M2）。

- **M1 建仓与规范**：Apache-2.0 LICENSE、OWNERS、DCO 检查（`scripts/check-dco.mjs` + `commit-msg` hook）、CI 门禁
- **M2 能力剥离与重构**：扁平 `src/core` 按领域重切为 `core` / `model` / `avoidance` / `telemetry` / `contract` 五模块，自底向上单向依赖
- 核心算法：飞控状态机（`DroneController`）、转向避障（`SteeringBehavior`）、碰撞评估（`CollisionDetector`）、HUD 威胁推导（`HudModel`）、世界对象（`WorldObject`），自实现 `Vec3` / `Logger` 替代 `ArkGraphics3D` / `hilog`
- 验证：16 项单测 + 仿真黄金用例入 CI（`−21.67m` 盲飞穿墙 vs `0.40m` 硬停面急停）
- 依赖：vitest 升级至 5.0.1，清除 dev 依赖漏洞链（5 → 0）
