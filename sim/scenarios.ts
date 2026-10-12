/**
 * M3 场景示例（脚本化仿真）：一次跑完 6 个确定性场景。
 * ---------------------------------------------------------------
 * 与 sim_flight.ts 的关系：
 *   - sim_flight.ts  —— 「最小演示」：单场景对比（盲飞 vs 感知急停），面向 README 读者；
 *   - scenarios.ts   —— 「场景全集」：覆盖六个模块域的端到端脚本，面向接入方与 CI 冒烟。
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
  DRONE_RADIUS_M,
  EnvironmentPerception,
  OccupancyGrid,
  SectorScan,
  SpatialPerception,
  Vec3,
  applyAvoidance,
  buildOccupancyGrid,
  buildSectors,
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

/** 场景 6：动力学内核急停 —— useDynamics=true 时，感知硬停面停距与运动学同口径 */
function scenarioDynamicsStop(): ScenarioOutcome {
  const d: DroneController = new DroneController();
  d.placed = true;
  d.flying = true;
  d.setFlightMode(true); // 切到真实动力学内核
  const wall: WallAheadPerception = new WallAheadPerception();
  d.perception = wall; // 接入"空间感知输入"契约
  d.moveY = 1; // 满前向推杆
  let minGap: number = Infinity;
  const frames: number = Math.round(60 * 20);
  for (let i = 0; i < frames; i++) {
    wall.setDroneZ(d.offsetZ);
    d.update(DT);
    const gap: number = wall.getObstacleDistance();
    if (gap < minGap) {
      minGap = gap;
    }
  }
  const finalGap: number = wall.getObstacleDistance();
  // 动力学有刚体惯性 + 电机滞后，允许比运动学(0.05)略松的容差(0.15)
  const tol: number = 0.15;
  const ok: boolean = Math.abs(finalGap - AVOID_MARGIN_BASE_M) <= tol &&
    minGap >= AVOID_MARGIN_BASE_M - tol;
  return {
    ok,
    detail: `动力学急停 finalGap=${finalGap.toFixed(2)}m（预期 ${AVOID_MARGIN_BASE_M.toFixed(2)}±${tol}）` +
      ` minGap=${minGap.toFixed(2)}m  终态=${d.state}`,
  };
}

// ── P5：量化指标与演进回归场景 ───────────────────────────────────
// 既有 6 个场景只回答"对不对"，新增这一组回答"好到什么程度"：
// 每个场景输出同一组可比较的量化指标，于是"把避障做得更好"第一次有了
// 可回归的度量，而不是靠手感描述。
// ------------------------------------------------------------------

/** 竖直墙面（XZ 平面内的直线）：过点 (cx, cz(t))、单位法向 (nx, nz) */
interface WallPlane {
  cx: number;
  cz: number;
  nx: number;
  nz: number;
}

/** 墙面的 Z 位置（含"推进一段时间后止步"的动态行为） */
function wallCz(w: WallPlane, t: number, vz: number, moveSec: number): number {
  return w.cz + vz * Math.min(t, moveSec);
}

/**
 * 平面场感知桩 —— 同时实现 P2 的多扇区契约与 P0 的垂直 / 失联扩展。
 * 几何用解析解（射线-平面求交），因此场景完全确定性、无随机源。
 */
class PlaneFieldPerception implements SpatialPerception {
  private px: number = 0;
  private pz: number = 0;
  private py: number = 0;
  private t: number = 0;
  private blind: boolean = false;

  constructor(
    private readonly walls: WallPlane[],
    private readonly ceilingY: number = Infinity,
    private readonly wallVZ: number = 0,
    private readonly outageWin: [number, number] | null = null,
    /** 墙体推进持续时长（秒）：到点后止步 —— 模拟"行人走近后停住"这类动态障碍 */
    private readonly wallMoveSec: number = Infinity
  ) {}

  /** 每帧由仿真器写入无人机位姿与仿真时刻 */
  public setPose(x: number, z: number, y: number, t: number): void {
    this.px = x;
    this.pz = z;
    this.py = y;
    this.t = t;
    this.blind = this.outageWin !== null && t >= this.outageWin[0] && t < this.outageWin[1];
  }

  /** 沿单位方向 (ux, uz) 到最近墙面的距离；打不到 = ∞ */
  private gapAlong(ux: number, uz: number): number {
    let best: number = Infinity;
    for (const w of this.walls) {
      const cz: number = wallCz(w, this.t, this.wallVZ, this.wallMoveSec);
      const num: number = (w.cx - this.px) * w.nx + (cz - this.pz) * w.nz;
      const den: number = ux * w.nx + uz * w.nz;
      if (Math.abs(den) < 1e-9) {
        continue;
      }
      const t: number = num / den;
      if (t > 0 && t < best) {
        best = t;
      }
    }
    return best;
  }

  getObstacleDistance(): number {
    // 单标量路径：只给"沿机头前向"的距离（既有能力的口径）
    return this.blind ? Infinity : this.gapAlong(0, -1);
  }

  getCeilingDistance(): number {
    if (this.blind || !isFinite(this.ceilingY)) {
      return Infinity;
    }
    return this.ceilingY - this.py;
  }

  isDepthSupported(): boolean {
    return true;
  }

  getSectorScan(): SectorScan | null {
    // 失联期间返回 null —— 契约层表达"本帧没有扇区数据"的正确方式
    if (this.blind) {
      return null;
    }
    const N: number = 8;
    const dists: number[] = [];
    for (let i = 0; i < N; i++) {
      const th: number = i * 2 * Math.PI / N;
      dists.push(this.gapAlong(-Math.sin(th), -Math.cos(th)));
    }
    return { sectors: buildSectors(0, dists), timestampSec: this.t, confidence: 1 };
  }
}

/** 一次飞行的量化指标（五个维度，可横向比较） */
interface AvoidMetrics {
  /** 最小净距（机体表面到最近障碍，米；<0 = 已侵入） */
  minClearance: number;
  /** 最大侵入深度（米；>0 = 撞上了） */
  maxPenetration: number;
  /** 水平路程（米，到达代价） */
  pathLength: number;
  /** 急停翻转次数（抖动指标，越小越好） */
  emergencyFlips: number;
  /** 最小碰撞时间（秒，越大越安全） */
  minTtcSec: number;
  /** 末速（m/s） */
  finalSpeed: number;
}

/** 指标格式化（统一口径，便于跨场景对比） */
function fmtMetrics(m: AvoidMetrics): string {
  return `最小净距=${m.minClearance.toFixed(3)}m 侵入=${m.maxPenetration.toFixed(3)}m ` +
    `路程=${m.pathLength.toFixed(2)}m 急停翻转=${m.emergencyFlips} ` +
    `最小TTC=${isFinite(m.minTtcSec) ? m.minTtcSec.toFixed(2) + 's' : '∞'} ` +
    `末速=${m.finalSpeed.toFixed(2)}m/s`;
}

/** 飞行模式：reactive = 既有反应式；vo = 叠加预测式；+temporal = 再叠加时间守护 */
type AvoidMode = 'reactive' | 'vo' | 'vo+temporal';

/**
 * 通用飞行 + 指标采集。
 * ------------------------------------------------------------------
 * 指标口径（刻意选"与实现无关"的量，才不会被实现细节自我证明）：
 *   · 净距 / 侵入：由**真实几何**（点到平面距离）算，不读控制器自报的读数；
 *   · TTC：由净距的**有限差分**变化率算，因此动态障碍（墙在动）也算得准；
 *   · 急停翻转：avoidEmergency 的逐帧翻转次数 —— 抖动的可量化代理指标。
 */
function runFlight(o: {
  walls: WallPlane[];
  seconds: number;
  mode: AvoidMode;
  inputX?: number;
  inputY?: number;
  climb?: number;
  ceilingY?: number;
  wallVZ?: number;
  wallMoveSec?: number;
  outageWin?: [number, number];
  trustFallback?: boolean;
  ceilingAvoid?: boolean;
}): AvoidMetrics {
  const d: DroneController = new DroneController();
  const per: PlaneFieldPerception = new PlaneFieldPerception(
    o.walls,
    o.ceilingY !== undefined ? o.ceilingY : Infinity,
    o.wallVZ !== undefined ? o.wallVZ : 0,
    o.outageWin !== undefined ? o.outageWin : null,
    o.wallMoveSec !== undefined ? o.wallMoveSec : Infinity
  );
  d.placed = true;
  d.flying = true;
  d.perception = per;
  d.voEnabled = (o.mode === 'vo' || o.mode === 'vo+temporal');
  d.temporalEnabled = (o.mode === 'vo+temporal');
  d.trustFallbackEnabled = o.trustFallback === true;
  d.ceilingAvoidEnabled = o.ceilingAvoid === true;
  d.moveX = o.inputX !== undefined ? o.inputX : 0;
  d.moveY = o.inputY !== undefined ? o.inputY : 1;
  d.climbInput = o.climb !== undefined ? o.climb : 0;

  let minClearance: number = Infinity;
  let minTtc: number = Infinity;
  let flips: number = 0;
  let prevEmergency: boolean = false;
  let prevClearance: number = Infinity;
  let path: number = 0;
  let prevX: number = d.offsetX;
  let prevZ: number = d.offsetZ;

  const frames: number = Math.round(60 * o.seconds);
  for (let i = 0; i < frames; i++) {
    const t: number = i * DT;
    per.setPose(d.offsetX, d.offsetZ, d.offsetY, t);
    d.update(DT);
    // —— 真实几何：点到各墙面的垂直距离 ——
    let clearance: number = Infinity;
    for (const w of o.walls) {
      const cz: number = wallCz(w, t, o.wallVZ !== undefined ? o.wallVZ : 0,
        o.wallMoveSec !== undefined ? o.wallMoveSec : Infinity);
      const dist: number = Math.abs((d.offsetX - w.cx) * w.nx + (d.offsetZ - cz) * w.nz);
      if (dist < clearance) {
        clearance = dist;
      }
    }
    clearance -= DRONE_RADIUS_M; // 机体表面净距
    if (clearance < minClearance) {
      minClearance = clearance;
    }
    // 有限差分求接近速率 → TTC（动态障碍同样成立）
    if (isFinite(prevClearance) && i > 0) {
      const rate: number = (prevClearance - clearance) / DT;
      if (rate > 1e-6 && clearance > 0) {
        const ttc: number = clearance / rate;
        if (ttc < minTtc) {
          minTtc = ttc;
        }
      }
    }
    prevClearance = clearance;
    if (d.avoidEmergency !== prevEmergency) {
      flips++;
    }
    prevEmergency = d.avoidEmergency;
    path += Math.sqrt((d.offsetX - prevX) * (d.offsetX - prevX) +
      (d.offsetZ - prevZ) * (d.offsetZ - prevZ));
    prevX = d.offsetX;
    prevZ = d.offsetZ;
  }
  return {
    minClearance: minClearance,
    maxPenetration: Math.max(0, -minClearance),
    pathLength: path,
    emergencyFlips: flips,
    minTtcSec: minTtc,
    finalSpeed: d.speed
  };
}

/** 场景 7：斜向接近（45° 墙）—— 单标量前向距离会低估斜插过来的墙 */
function scenarioDiagonalWall(): ScenarioOutcome {
  const n: number = Math.SQRT1_2; // 45°
  const walls: WallPlane[] = [{ cx: 0, cz: -3, nx: n, nz: n }];
  const reactive: AvoidMetrics = runFlight({ walls, seconds: 12, mode: 'reactive' });
  const vo: AvoidMetrics = runFlight({ walls, seconds: 12, mode: 'vo' });
  // 验收：反应式因"只看前向投影"而低估斜墙，预测式按真实方向留出净距
  const ok: boolean = vo.minClearance > reactive.minClearance &&
    vo.minClearance >= 0 && vo.maxPenetration === 0;
  return {
    ok,
    detail: `反应式 ${fmtMetrics(reactive)}\n         ` +
      `预测式 ${fmtMetrics(vo)}\n         ` +
      `净距改善 +${(vo.minClearance - reactive.minClearance).toFixed(3)}m`,
  };
}

/** 场景 8：门框穿越（左右夹逼的窄走廊）—— 侧向障碍对"单标量前向距离"完全隐形 */
function scenarioDoorFrame(): ScenarioOutcome {
  // 走廊半宽 0.35m：机体半径 0.19m，两侧余量各 0.16m
  const walls: WallPlane[] = [
    { cx: 0.35, cz: 0, nx: 1, nz: 0 },
    { cx: -0.35, cz: 0, nx: -1, nz: 0 }
  ];
  // 向前 + 轻微右偏：会持续压向右侧墙
  const cfg = { walls, seconds: 10, inputX: 0.35, inputY: 1 };
  const reactive: AvoidMetrics = runFlight({ ...cfg, mode: 'reactive' });
  const vo: AvoidMetrics = runFlight({ ...cfg, mode: 'vo' });
  // 验收：反应式对侧向墙无感（必然贴墙甚至侵入）；预测式把侧墙纳入约束
  const ok: boolean = reactive.minClearance < 0 && vo.minClearance > reactive.minClearance &&
    vo.maxPenetration < reactive.maxPenetration;
  return {
    ok,
    detail: `反应式 ${fmtMetrics(reactive)}\n         ` +
      `预测式 ${fmtMetrics(vo)}\n         ` +
      `侧向障碍防护：侵入 ${reactive.maxPenetration.toFixed(3)}m → ${vo.maxPenetration.toFixed(3)}m`,
  };
}

/**
 * 场景 9：动态障碍（墙迎面推进 3 秒后止步）—— 用最小 TTC 度量"介入是否更早"。
 * ------------------------------------------------------------------
 * 为什么墙必须"推进后止步"：一个**无限逼近**的墙，任何只会的"刹车"的避障层
 * 都必然被撞上 —— 无人机停下后墙仍在前进，接触只是时间问题。
 * 那种局面需要**主动后退**，属于规划层（P4）职责，不是反应式/预测式能解决的。
 * 因此本场景取"走近后止步"这一现实子集（行人/推车），考察的是：
 * 谁更早开始减速 → 最小 TTC 更大 → 留给用户的反应余量更多。
 */
function scenarioMovingObstacle(): ScenarioOutcome {
  const walls: WallPlane[] = [{ cx: 0, cz: -6, nx: 0, nz: 1 }];
  const cfg = { walls, seconds: 12, wallVZ: 0.6, wallMoveSec: 3 };
  const reactive: AvoidMetrics = runFlight({ ...cfg, mode: 'reactive' });
  const vo: AvoidMetrics = runFlight({ ...cfg, mode: 'vo' });
  // 验收：两者都不接触；预测式按 TTC 约束，介入更早 → 最小 TTC 显著更大
  const ok: boolean = reactive.minClearance > 0 && vo.minClearance > 0 &&
    vo.minTtcSec > reactive.minTtcSec * 1.1;
  return {
    ok,
    detail: `反应式 ${fmtMetrics(reactive)}\n         ` +
      `预测式 ${fmtMetrics(vo)}\n         ` +
      `最小TTC ${reactive.minTtcSec.toFixed(2)}s → ${vo.minTtcSec.toFixed(2)}s` +
      `（+${((vo.minTtcSec / reactive.minTtcSec - 1) * 100).toFixed(0)}% 反应余量）`,
  };
}

/**
 * 场景 10：失联恢复 —— 持续盲飞必须降档限爬，恢复后必须自动解除。
 * ------------------------------------------------------------------
 * 爬升速度刻意压到 0.2m/s：默认 1.2m/s 在 4 秒内就撞上 maxHeight=5m，
 * 两条曲线会同时饱和到同一高度，差异被抹平 —— 那就测不出兜底是否真的生效。
 */
function scenarioBlindRecovery(): ScenarioOutcome {
  // 一堵 50m 外的墙（远到不触发任何避障）：它的作用是让**非失联期**有有效读数 ——
  // 否则全场 getObstacleDistance 恒为 ∞，控制器会认为"一直在失联"，
  // 于是 outage 窗口根本区分不出来。
  const farWall: WallPlane[] = [{ cx: 0, cz: -50, nx: 0, nz: 1 }];
  const probe = (outage: [number, number], seconds: number, fallback: boolean):
    { y: number; trust: string } => {
    const d: DroneController = new DroneController();
    const per: PlaneFieldPerception = new PlaneFieldPerception(farWall, Infinity, 0, outage);
    d.placed = true;
    d.flying = true;
    d.perception = per;
    d.trustFallbackEnabled = fallback;
    d.params.climbSpeed = 0.2;
    d.climbInput = 1;
    for (let i = 0; i < Math.round(60 * seconds); i++) {
      per.setPose(d.offsetX, d.offsetZ, d.offsetY, i * DT);
      d.update(DT);
    }
    return { y: d.offsetY, trust: d.trustLevel };
  };
  // 失联 7 秒（2~9s）→ 最高只到 cautious；恢复 3 秒后应回到 normal
  const guarded = probe([2, 9], 12, true);
  const naive = probe([2, 9], 12, false);
  // 失联 14 秒 → 越过 12 秒阈值 → hold
  const long = probe([2, 20], 16, true);
  const ok: boolean = guarded.y < naive.y - 0.05 && guarded.trust === 'normal' &&
    long.trust === 'hold' && naive.trust === 'normal';
  return {
    ok,
    detail: `失联7s：兜底高度=${guarded.y.toFixed(3)}m（档位 ${guarded.trust}）  ` +
      `不兜底=${naive.y.toFixed(3)}m（档位 ${naive.trust}，零回归）  ` +
      `失联14s：档位=${long.trust} 高度=${long.y.toFixed(3)}m（爬升被冻结）`,
  };
}

/** 场景 11：垂直避障（上方净距）—— 抬头撞天花板此前"根本没有这一维" */
function scenarioCeilingLimit(): ScenarioOutcome {
  const walls: WallPlane[] = [];
  const run = (enable: boolean): number => {
    const d: DroneController = new DroneController();
    const per: PlaneFieldPerception = new PlaneFieldPerception([], 1.5, 0, null);
    d.placed = true;
    d.flying = true;
    d.perception = per;
    d.ceilingAvoidEnabled = enable;
    d.climbInput = 1;
    const y0: number = d.offsetY;
    for (let i = 0; i < Math.round(60 * 6); i++) {
      per.setPose(d.offsetX, d.offsetZ, d.offsetY, i * DT);
      d.update(DT);
    }
    return d.offsetY - y0;
  };
  const off: number = run(false);
  const on: number = run(true);
  // 验收：开启后爬升被限速（不得贴到天花板），关闭时照常爬（零回归）
  const ok: boolean = on < off - 0.05 && on >= 0 && off > 0.05;
  return {
    ok,
    detail: `关闭时 6s 爬升=${off.toFixed(3)}m（零回归）  开启时=${on.toFixed(3)}m` +
      `（上方硬停面 0.35m + 减速带 0.55m 起作用）`,
  };
}

// ── 入口 ─────────────────────────────────────────────────────────

const SCENARIOS: Array<[string, Scenario]> = [
  ['盲飞穿墙（危险基线）', scenarioBlindFly],
  ['感知驱动硬停（SpatialPerception）', scenarioPerceptionStop],
  ['侧风闭环补偿（EnvironmentPerception）', scenarioWindDrift],
  ['绕行方向择优（bypass 纯函数）', scenarioBypassSide],
  ['A* 规划绕障（planning）', scenarioPlanAroundWall],
  ['动力学内核急停（useDynamics）', scenarioDynamicsStop],
  ['P5 斜向接近（反应式 vs 预测式）', scenarioDiagonalWall],
  ['P5 门框穿越（侧向障碍）', scenarioDoorFrame],
  ['P5 动态障碍（最小 TTC）', scenarioMovingObstacle],
  ['P5 失联恢复（兜底降档）', scenarioBlindRecovery],
  ['P5 垂直避障（上方净距）', scenarioCeilingLimit],
];

// biome-ignore lint/suspicious/noConsole: 仿真脚本有意输出
console.log('=== uav_flight_sim 场景仿真（M3 脚本化仿真方案） ===');
// biome-ignore lint/suspicious/noConsole: 仿真脚本有意输出
console.log('定步长 dt=1/60 · 无随机源 · 感知/环境走契约注入\n');

let failed = false;
let passed = 0;
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
  } else {
    passed++;
  }
}

// biome-ignore lint/suspicious/noConsole: 仿真脚本有意输出
console.log('');
// biome-ignore lint/suspicious/noConsole: 仿真脚本有意输出
console.log(failed ? '结论：存在失败场景，请检查最近改动是否破坏黄金行为。' :
  `结论：${passed}/${SCENARIOS.length} 场景通过 —— 纯算法层在无头环境下行为符合验收口径。`);
process.exit(failed ? 1 : 0);
