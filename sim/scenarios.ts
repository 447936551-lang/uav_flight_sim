/**
 * M3 场景示例（脚本化仿真）：一次跑完 5 个确定性场景。
 * ---------------------------------------------------------------
 * 与 sim_flight.ts 的关系：
 *   - sim_flight.ts  —— 「最小演示」：单场景对比（盲飞 vs 感知急停），面向 README 读者；
 *   - scenarios.ts   —— 「场景全集」：覆盖五个模块域的端到端脚本，面向接入方与 CI 冒烟。
 *
 * 确定性三原则（详见 docs/simulation.md）：
 *   1. 定步长 dt = 1/60，帧数固定 —— 无时钟依赖、无随机源；
 *   2. 感知 / 环境全部走契约注入（SpatialPerception / EnvironmentPerception）；
 *   3. 阈值一律引用常量（如 AVOID_MARGIN_BASE_M），不硬编码黄金数值。
 *
 * 运行：npm run sim:scenarios
 * 退出码：任一场景验收失败 → 1（可直接当 CI 门禁 / 本地冒烟脚本用）。
 */
import {
  AvoidResult,
  DroneController,
  DroneState,
  EnvironmentPerception,
  OccupancyGrid,
  SpatialPerception,
  Vec3,
  applyAvoidance,
  buildOccupancyGrid,
  CELL_OCCUPIED,
  DEFAULT_GRID_CONFIG,
  DEFAULT_STEERING,
  planPath,
  vec3,
  WIND_GAIN,
} from '../src/index';
import { AVOID_MARGIN_BASE_M } from '../src/avoidance/CollisionDetector';
// pickAvoidSide 属于模块内实现细节，未进 index barrel —— 场景里从模块直接导入
import { pickAvoidSide } from '../src/avoidance/SteeringBehavior';

// ── 契约桩（接入方照此实现即可，控制器零改动）────────────────────

/** 前方固定一堵墙（AR -Z 方向 8m），实现 SpatialPerception 契约 */
class WallAheadPerception implements SpatialPerception {
  private droneZ: number = 0;
  private readonly wallZ: number = -8;

  /** 每帧由仿真器把无人机当前 Z 写入 */
  setDroneZ(z: number): void {
    this.droneZ = z;
  }

  getObstacleDistance(): number {
    // forward = (0, 0, -1)；gap = (wall - drone) · forward
    return (this.wallZ - this.droneZ) * -1;
  }

  getObstacleNormal(): Vec3 {
    return vec3(0, 0, 1);
  }
}

/** 恒定风场（+X 方向 3 m/s），实现 EnvironmentPerception 契约 */
class ConstantWindX implements EnvironmentPerception {
  getWindVelocity(): Vec3 {
    return vec3(3, 0, 0);
  }
}

// ── 场景骨架 ─────────────────────────────────────────────────────

const DT: number = 1 / 60;

interface ScenarioOutcome {
  ok: boolean;
  detail: string;
}

type Scenario = () => ScenarioOutcome;

/** 通用飞行器：满前向推杆飞 seconds 秒，逐帧喂感知，返回轨迹统计 */
function flyTowardWall(seconds: number, withPerception: boolean): {
  minGap: number;
  finalGap: number;
  emergencyHits: number;
  finalState: DroneState;
} {
  const d = new DroneController();
  const wall = new WallAheadPerception();
  d.placed = true;
  d.flying = true;
  if (withPerception) {
    d.perception = wall; // 接入"空间感知输入"契约，控制器零改动
  } else {
    d.obstacleDistance = Infinity; // 无感知：防撞是盲的
  }
  d.moveY = 1; // 满前向推杆

  let minGap = Infinity;
  let emergencyHits = 0;
  const frames = Math.round(60 * seconds);
  for (let i = 0; i < frames; i++) {
    wall.setDroneZ(d.offsetZ);
    d.update(DT);
    const gap = wall.getObstacleDistance();
    if (gap < minGap) {
      minGap = gap;
    }
    if (d.avoidEmergency) {
      emergencyHits++;
    }
  }
  return {
    minGap,
    finalGap: wall.getObstacleDistance(),
    emergencyHits,
    finalState: d.state,
  };
}

/** 悬停 seconds 秒（无杆量），返回 X 向漂移与保持环接合标志 */
function hoverDriftX(seconds: number, wind: boolean): { driftX: number; holdEngaged: boolean } {
  const d = new DroneController();
  d.placed = true;
  d.flying = true;
  if (wind) {
    d.environment = new ConstantWindX();
    d.environmentActive = true; // 环境契约须显式开启（false = 完全不参与计算）
  }
  const frames = Math.round(60 * seconds);
  for (let i = 0; i < frames; i++) {
    d.update(DT);
  }
  return { driftX: d.offsetX, holdEngaged: d.holdEngaged };
}

// ── 五个场景 ─────────────────────────────────────────────────────

/** 场景 1：无感知直飞 —— 防撞是盲的，直接穿过墙体（危险基线） */
function scenarioBlindFly(): ScenarioOutcome {
  const r = flyTowardWall(20, false);
  const ok = r.minGap < 0; // 穿墙 = 最近间隙为负
  return {
    ok,
    detail: `minGap=${r.minGap.toFixed(2)}m（预期 < 0，穿墙）终态=${r.finalState}`,
  };
}

/** 场景 2：感知驱动硬停 —— 在静止硬停面 AVOID_MARGIN_BASE_M 处精确停住 */
function scenarioPerceptionStop(): ScenarioOutcome {
  const r = flyTowardWall(20, true);
  const tol = 0.05; // 与 golden-flight.test.ts 同一口径
  // 零过冲：最近间隙 = 末间隙（单调逼近，不会冲进硬停面再弹回）
  const ok = Math.abs(r.finalGap - AVOID_MARGIN_BASE_M) <= tol &&
    r.minGap >= AVOID_MARGIN_BASE_M - tol && // 全程不越过硬停面
    Math.abs(r.minGap - r.finalGap) < 1e-6;
  return {
    ok,
    detail: `finalGap=${r.finalGap.toFixed(2)}m（预期 ${AVOID_MARGIN_BASE_M.toFixed(2)}±${tol}）` +
      `急停帧=${r.emergencyHits} 终态=${r.finalState}`,
  };
}

/**
 * 场景 3：侧风闭环补偿 —— EnvironmentPerception 注入恒风，闭环保持环接合并抵消风扰。
 * ------------------------------------------------------------------
 * M3 Step1 之前：无风零漂移 + 有风被动漂移（open-loop，3m/s 风 10s 约漂 1m）。
 * M3 Step1 之后：启用环境感知时闭合位置环，把风扰前馈（抵消 90%）+ 反馈（位置环拉回锚点）
 *   双重压回锚点，表现为：
 *     ① 有风时 holdEngaged=true —— 证明环境契约确被消费（否则保持环不会接合）；
 *     ② 残余漂移被压到极小（<0.1m，相较开环约 1m 收敛两个数量级）且下风方向 ——
 *        既证明风被读入（非零），又证明闭环把它压住（远小于开环）；
 *     ③ 无风 / 环境关闭时仍逐位零漂移且保持环不介入 —— 零回归硬保证。
 */
function scenarioWindDrift(): ScenarioOutcome {
  const calm = hoverDriftX(10, false);
  const windy = hoverDriftX(10, true);
  // 无风必须逐位零漂移（零回归硬保证）；保持环默认不介入
  const ok = Math.abs(calm.driftX) < 1e-9 && !calm.holdEngaged &&
    // 有风：环境契约被消费（保持环接合）、残余风扰被闭环压到极小（下风方向但受控）
    windy.holdEngaged && windy.driftX > 1e-4 && windy.driftX < 0.1 && isFinite(windy.driftX);
  return {
    ok,
    detail: `无风漂移=${calm.driftX.toFixed(4)}m（预期 0，holdEngaged=${calm.holdEngaged}）  ` +
      `3m/s 侧风（闭环保持）残留=${windy.driftX.toFixed(4)}m（增益 WIND_GAIN=${WIND_GAIN}，` +
      `holdEngaged=${windy.holdEngaged}；开环本应漂约 1m）`,
  };
}

/** 场景 4：绕行方向择优 —— bypass 主动绕行按障碍方位选边，死区回落向右 */
function scenarioBypassSide(): ScenarioOutcome {
  // 正前满舵推杆：desired 与 forward=(0,-1) 同向（fwd = desired·forward > 0）；
  // distance=1.0m 落在减速带内（safeDist(0)=0.25+1.2=1.45m）
  const run = (lateral: number): AvoidResult =>
    applyAvoidance(0, -1, 0, -1, 1.0, DEFAULT_STEERING, true, lateral, 0);

  const obstacleRight = run(0.5); // 障碍偏右（+X）→ 应向左绕
  const obstacleLeft = run(-0.5); // 障碍偏左（−X）→ 应向右绕
  const deadzone = run(0); // 死区内 → 回落传统「向右」零回归
  const side = (v: number): number => pickAvoidSide(v);

  const ok = obstacleRight.avoidedLeft && !obstacleRight.avoidedRight &&
    obstacleLeft.avoidedRight && !obstacleLeft.avoidedLeft &&
    deadzone.avoidedRight && !deadzone.avoidedLeft &&
    side(0.05) === 0 && side(0.5) === -1 && side(-0.5) === 1;
  return {
    ok,
    detail: `障碍右→绕左=${obstacleRight.avoidedLeft}  障碍左→绕右=${obstacleLeft.avoidedRight}  ` +
      `死区→回落右=${deadzone.avoidedRight}（pickAvoidSide 死区=0.12m）`,
  };
}

/** 场景 5：A* 规划绕障 —— 占用栅格 + 路径规划 + 绕行证据 */
function scenarioPlanAroundWall(): ScenarioOutcome {
  // 地平面（±4m）：把覆盖区标为 FREE，否则全图 UNKNOWN、起点无处可snap
  const floor = {
    cx: 0, cy: 0, cz: 0, nx: 0, ny: 1, nz: 0,
    poly: [-4, 0, -4, 4, 0, -4, 4, 0, 4, -4, 0, 4],
  };
  // 竖直墙：x=2 平面，z∈[-2,2]、y∈[0,2]（多边形顶点为 (x,y,z) 三元组）
  const wall = {
    cx: 2, cy: 1, cz: 0, nx: -1, ny: 0, nz: 0,
    poly: [2, 0, -2, 2, 0, 2, 2, 2, 2, 2, 2, -2],
  };
  const grid: OccupancyGrid = buildOccupancyGrid([floor, wall], 0, 0, DEFAULT_GRID_CONFIG);
  // 栅格确有障碍（桩自检：墙位应被标占用、起点应为 FREE）
  const wallMarked = grid.at(grid.colOf(2), grid.rowOf(0)) === CELL_OCCUPIED;
  const startFree = grid.isFree(grid.colOf(0), grid.rowOf(0));

  const r = planPath(grid, 0, 0, 3.5, 0);
  let maxAbsZ = 0;
  for (const p of r.points) {
    maxAbsZ = Math.max(maxAbsZ, Math.abs(p.z));
  }
  // 成功达且必须绕：路径中至少一个路点 |z| 超出墙半长（2m）+ 膨胀半径
  const ok = wallMarked && startFree && r.ok && r.points.length >= 3 && maxAbsZ > 2.0;
  return {
    ok,
    detail: `规划${r.ok ? '成功' : '失败(' + r.reason + ')'}  路点=${r.points.length}  ` +
      `展开节点=${r.expanded}  最大绕行|z|=${maxAbsZ.toFixed(2)}m（墙半长 2m）`,
  };
}

// ── 入口 ─────────────────────────────────────────────────────────

const SCENARIOS: Array<[string, Scenario]> = [
  ['盲飞穿墙（危险基线）', scenarioBlindFly],
  ['感知驱动硬停（SpatialPerception）', scenarioPerceptionStop],
  ['侧风闭环补偿（EnvironmentPerception）', scenarioWindDrift],
  ['绕行方向择优（bypass 纯函数）', scenarioBypassSide],
  ['A* 规划绕障（planning）', scenarioPlanAroundWall],
];

// biome-ignore lint/suspicious/noConsole: 仿真脚本有意输出
console.log('=== uav_flight_sim 场景仿真（M3 脚本化仿真方案） ===');
// biome-ignore lint/suspicious/noConsole: 仿真脚本有意输出
console.log('定步长 dt=1/60 · 无随机源 · 感知/环境走契约注入\n');

let failed = false;
for (const [name, fn] of SCENARIOS) {
  let outcome: ScenarioOutcome;
  try {
    outcome = fn();
  } catch (e) {
    outcome = { ok: false, detail: `异常：${(e as Error).message}` };
  }
  // biome-ignore lint/suspicious/noConsole: 仿真脚本有意输出
  console.log(`  [${outcome.ok ? 'PASS' : 'FAIL'}] ${name}`);
  // biome-ignore lint/suspicious/noConsole: 仿真脚本有意输出
  console.log(`         ${outcome.detail}`);
  if (!outcome.ok) {
    failed = true;
  }
}

// biome-ignore lint/suspicious/noConsole: 仿真脚本有意输出
console.log('');
// biome-ignore lint/suspicious/noConsole: 仿真脚本有意输出
console.log(failed ? '结论：存在失败场景，请检查最近改动是否破坏黄金行为。' :
  '结论：5/5 场景通过 —— 纯算法层在无头环境下行为符合验收口径。');
process.exit(failed ? 1 : 0);
