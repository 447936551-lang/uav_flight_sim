# 更新日志

本项目采用 [语义化版本](https://semver.org/lang/zh-CN/)。所有条目均标注来源模块与验证方式。

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

从 ARDroneHarmony（AR 无人机 App）**第二批剥离**：在既有「飞控 + 避障」之上，
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
| 算法竞技框架（`FlightAlgorithm` 契约与注册表）及依赖它的四个任务算法（APAS 绕障 / 航点 / 返航 / POI 环绕） | 按项目决策：该框架及其算法**暂不纳入本仓** |
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
