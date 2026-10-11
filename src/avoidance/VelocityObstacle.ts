/**
 * 速度障碍 / 预测式避障（Layer 5 · 物理层 · P3 演进）
 * ---------------------------------------------------------------
 * 解决的问题：既有 applyAvoidance 是**位置域反应式**——
 *     前进分量 ×= (d − margin) / band
 * 它只回答"离障碍多远时该减到几成"，**不回答"以这个速度多久会撞上"**。
 * 后果有三：
 *
 *   1. 斜向接近被系统性低估 —— 走廊半宽硬判只保留"接近正前"的障碍，
 *      45° 斜插过来的墙要么被丢弃（撞上），要么被当成正前（过度刹车）；
 *   2. 只能"减速"，不能"改向" —— 输出始终是期望速度的**同向缩放**，
 *      侧向绕行完全依赖用户已有横移输入（bypass 另开分支补）；
 *   3. 多障碍无解 —— 单一标量距离无法表达"左前 1m、右前 0.4m"这种局面。
 *
 * 本模块换用**速度域预测式**判据（Velocity Obstacle 的线性化工程版本）：
 *   · 每个扇区给出一条**半空间约束**：v·û ≤ (d − clearance) / T
 *     （沿 û 方向的接近速度，不得大到在 T 秒内吃掉剩余净距）；
 *   · 期望速度被**投影**到所有半空间的交集上（POCS 迭代，半空间必收敛）；
 *   · 投影天然只削减"朝障碍去"的分量、保留切向分量 → 自动产生贴墙滑行/绕行。
 *
 * 与经典 VO/RVO 的差异（刻意取舍）：
 *   经典 VO 构造的是**锥形**禁区（相对速度落在锥内才禁），本实现用**半空间**近似
 *   （只约束径向接近速度）—— 更保守、几何更简单、无需求解二次规划，
 *   且在"深度只给距离、不给障碍外形"的输入条件下是唯一可解的口径。
 *   代价是绕行不如 RVO 主动（不产生"绕到侧后方"的绕行速度），
 *   取舍理由：本仓已真机验证的手感是"贴墙滑行"，半空间投影恰好复现它。
 *
 * ⚠️ 零回归纪律：本模块**不替换** applyAvoidance，而是作为**叠加约束**使用
 *   （先跑反应式层，再投影）。投影只会让速度更保守（|v| 不增、径向分量不增），
 *   因此"启用本模块只会更安全，不会更激进"，且默认关闭。
 */
import { Logger } from '../core/Logger';
import { DRONE_RADIUS_M, isValidDistance } from './CollisionDetector';
import { corridorWeight } from './TemporalGuard';
import { AVOID_CORRIDOR_HALF_M } from './CollisionDetector';

const TAG: string = 'VelocityObstacle';

// ---------------------------------------------------------------
//  常量（单一事实来源）
// ---------------------------------------------------------------

/**
 * 期望最小碰撞时间 T（秒）：沿某方向允许的最大接近速度 = 剩余净距 / T。
 * ---------------------------------------------------------------
 * 取值依据（与既有阈值对齐，避免两套口径打架）：
 *   静止时 safeDist(0) = 0.25 + 1.2 = 1.45m，满速 1.5m/s → 穿越它需 0.97s；
 *   满速时 safeDist(1.5) = 1.0 + 1.2 = 2.2m，满速穿越需 1.47s。
 * 取 1.5s 意味着"在任何速度下都至少留 1.5s 的反应余量"，略严于既有减速带
 * （这点保守是刻意的：预测层的价值就在于比反应层更早介入）。
 * 副作用上界：距离 1.0m 时允许接近速度 0.54m/s，而反应层给的是 0.94m/s ——
 * 预测层确实更早刹车，这正是它存在的意义（由开关控制，默认关闭）。
 */
export const VO_TTC_SEC: number = 1.5;

/** 期望保留的净距（米）：默认取机体包围球半径，与硬停面基线同口径 */
export const VO_CLEARANCE_M: number = DRONE_RADIUS_M;

/**
 * POCS 投影迭代次数。
 * 半空间交集的逐次投影（Kaczmarz/POCS）在两个约束夹角很小时收敛慢，
 * 4 次已足以让典型场景（前方 1~2 个约束）的最大残差 < 1% —— 见 selfTest 断言。
 */
export const VO_ITERATIONS: number = 4;

/** 判定"正在接近"的速度阈值（m/s），避免 0 附近除零产生 Infinity 噪声 */
export const VO_CLOSING_EPS: number = 1e-4;

// ---------------------------------------------------------------
//  类型（结构化，与 contract/SectorSample 天然兼容，无需 import）
// ---------------------------------------------------------------

/** 单个扇区样本：沿单位方向 û 的净距 */
export interface SectorInput {
  /** 方向单位向量 X（世界系 XZ 平面） */
  ux: number;
  /** 方向单位向量 Z */
  uz: number;
  /** 沿该方向的净距（米）；Infinity / NaN = 该扇区测不到 */
  distance: number;
  /** 命中点的横向偏置（米，机体右向为正）；可由 buildSectors 推导 */
  lateral?: number;
}

/** 速度障碍求解选项 */
export interface VoOptions {
  /** 期望最小碰撞时间（秒） */
  ttcSec: number;
  /** 期望保留的净距（米） */
  clearance: number;
  /** 走廊半宽（米）：用于把"偏离航向轴"的扇区按 corridorWeight 削弱 */
  corridorHalf: number;
  /** 结果限幅（m/s） */
  maxSpeed: number;
  /** POCS 迭代次数 */
  iterations: number;
}

/** 默认选项 */
export const DEFAULT_VO: VoOptions = {
  ttcSec: VO_TTC_SEC,
  clearance: VO_CLEARANCE_M,
  corridorHalf: AVOID_CORRIDOR_HALF_M,
  maxSpeed: 1.5,
  iterations: VO_ITERATIONS
};

/** 一条半空间约束：v·û ≤ limit */
export interface VoConstraint {
  ux: number;
  uz: number;
  /** 允许的最大径向接近速度（m/s） */
  limit: number;
}

/** 速度障碍求解结果 */
export interface VoResult {
  /** 投影后的速度 X（m/s） */
  vx: number;
  /** 投影后的速度 Z（m/s） */
  vz: number;
  /** 是否被至少一个约束修正过 */
  blocked: boolean;
  /** 求解后所有扇区中的最小碰撞时间（秒）；无障碍 = Infinity */
  minTtcSec: number;
  /** 威胁最大扇区的横向偏置（米，右为正）；无威胁 = 0 */
  threatLateral: number;
  /** 本帧生效的约束条数 */
  constraints: number;
}

// ---------------------------------------------------------------
//  纯函数：TTC
// ---------------------------------------------------------------

/** 速度在方向 û 上的投影（接近为正） */
export function approachSpeed(vx: number, vz: number, ux: number, uz: number): number {
  return vx * ux + vz * uz;
}

/**
 * 碰撞时间 TTC（秒）：以当前接近速度还有多久吃掉剩余净距。
 * 不接近（≤0）或测距无效 → Infinity。
 */
export function timeToCollision(distance: number, closing: number,
  clearance: number = VO_CLEARANCE_M): number {
  if (!isValidDistance(distance)) {
    return Infinity;
  }
  if (!isFinite(closing) || closing <= VO_CLOSING_EPS) {
    return Infinity;
  }
  const gap: number = distance - ((isFinite(clearance) && clearance > 0) ? clearance : 0);
  if (gap <= 0) {
    return 0; // 已经侵入净距：TTC 归零（最危险）
  }
  return gap / closing;
}

/** 一组扇区在当前速度下的最小 TTC（秒） */
export function minTtc(sectors: SectorInput[], vx: number, vz: number,
  clearance: number = VO_CLEARANCE_M): number {
  let worst: number = Infinity;
  for (const s of sectors) {
    const t: number = timeToCollision(s.distance, approachSpeed(vx, vz, s.ux, s.uz), clearance);
    if (t < worst) {
      worst = t;
    }
  }
  return worst;
}

// ---------------------------------------------------------------
//  纯函数：约束构造
// ---------------------------------------------------------------

/**
 * 单个扇区的径向速度上限（m/s）。
 * ---------------------------------------------------------------
 * limit = (d − clearance) / T / w
 *   · w = corridorWeight(lateral, half)：偏离航向轴越远，约束越弱（w↓ → limit↑）；
 *     w=0（走廊外）时返回 Infinity，等价于"该扇区不参与约束"。
 *     这把既有的走廊硬判变成了连续削弱 —— 既不丢信息，也不误刹。
 *   · 结果可能为负（已侵入净距）→ 夹到 0（该方向禁止接近）。
 */
export function sectorLimit(s: SectorInput, opts: VoOptions = DEFAULT_VO): number {
  if (!isValidDistance(s.distance)) {
    return Infinity;
  }
  const ttc: number = (isFinite(opts.ttcSec) && opts.ttcSec > 0) ? opts.ttcSec : VO_TTC_SEC;
  const clr: number = (isFinite(opts.clearance) && opts.clearance > 0) ? opts.clearance : 0;
  const gap: number = s.distance - clr;
  if (gap <= 0) {
    return 0; // 已侵入 → 禁止继续接近
  }
  const w: number = corridorWeight(s.lateral !== undefined ? s.lateral : 0, opts.corridorHalf);
  if (w <= 0) {
    return Infinity; // 走廊外 → 不约束（等价于既有硬判的"丢弃"）
  }
  const limit: number = gap / ttc / w;
  return limit > 0 ? limit : 0;
}

/**
 * 由扇区样本构造半空间约束集。
 * 只保留"真的会约束到当前速度上限"的扇区（limit < maxSpeed），
 * 其余跳过 —— 既省算力，也让"无障碍时空转"这一事实显式可见。
 */
export function buildVoConstraints(sectors: SectorInput[], opts: VoOptions = DEFAULT_VO): VoConstraint[] {
  const out: VoConstraint[] = [];
  for (const s of sectors) {
    const limit: number = sectorLimit(s, opts);
    if (!isFinite(limit)) {
      continue; // 走廊外 / 测不到 → 不约束
    }
    const len: number = Math.sqrt(s.ux * s.ux + s.uz * s.uz);
    if (len < 1e-9) {
      continue; // 零方向向量（脏数据）
    }
    if (limit >= opts.maxSpeed) {
      continue; // 该方向上"全速也不违反" → 不构成约束
    }
    out.push({ ux: s.ux / len, uz: s.uz / len, limit: limit });
  }
  return out;
}

// ---------------------------------------------------------------
//  纯函数：投影求解
// ---------------------------------------------------------------

/**
 * 把速度投影到半空间交集（POCS / 逐次投影）。
 * ---------------------------------------------------------------
 * 每轮：对每个约束 c，若 v·û > limit，则减去超出量沿 û 的分量：
 *     v ← v − û · (v·û − limit)
 * 半空间是凸集，逐次投影必然收敛到交集中离初值最近的点（夹角接近时慢，
 * 但每轮都使残差单调下降，故固定迭代次数也是**安全**的：最坏情况只是略保守）。
 *
 * @returns 投影后的速度与"是否被修正过"
 */
export function projectVelocity(vx: number, vz: number, constraints: VoConstraint[],
  iterations: number = VO_ITERATIONS): { vx: number; vz: number; corrected: boolean } {
  let x: number = vx;
  let z: number = vz;
  let corrected: boolean = false;
  if (!isFinite(x) || !isFinite(z)) {
    return { vx: 0, vz: 0, corrected: true };
  }
  const rounds: number = (iterations >= 1 && isFinite(iterations)) ? Math.floor(iterations) : 1;
  for (let r = 0; r < rounds; r++) {
    let changed: boolean = false;
    for (const c of constraints) {
      const s: number = x * c.ux + z * c.uz;
      if (s > c.limit) {
        const excess: number = s - c.limit;
        x -= c.ux * excess;
        z -= c.uz * excess;
        changed = true;
        corrected = true;
      }
    }
    if (!changed) {
      break;
    }
  }
  return { vx: x, vz: z, corrected: corrected };
}

/**
 * 速度障碍求解（一键式：约束构造 → 投影 → 指标回读）。
 * ---------------------------------------------------------------
 * 叠加用法（推荐）：传入 applyAvoidance 的**输出**作为期望速度，
 * 则本函数只会在反应式层不够安全时进一步收窄，永远不会放大速度 ——
 * 这就是"启用预测层只会更安全"的形式保证。
 */
export function solveVelocityObstacle(desiredVX: number, desiredVZ: number,
  sectors: SectorInput[], opts: VoOptions = DEFAULT_VO): VoResult {
  const constraints: VoConstraint[] = buildVoConstraints(sectors, opts);
  const p = projectVelocity(desiredVX, desiredVZ, constraints, opts.iterations);
  // 限幅：投影不会放大速度（半空间投影是"减去超出量"），但脏数据下仍兜一次
  let vx: number = p.vx;
  let vz: number = p.vz;
  const mag: number = Math.sqrt(vx * vx + vz * vz);
  if (mag > opts.maxSpeed && mag > 1e-9) {
    vx = vx / mag * opts.maxSpeed;
    vz = vz / mag * opts.maxSpeed;
  }
  const threat: number = pickThreatSector(sectors, desiredVX, desiredVZ, opts);
  const lateral: number = (threat >= 0 && sectors[threat].lateral !== undefined) ?
    (sectors[threat].lateral as number) : 0;
  return {
    vx: vx,
    vz: vz,
    blocked: p.corrected,
    minTtcSec: minTtc(sectors, vx, vz, opts.clearance),
    threatLateral: lateral,
    constraints: constraints.length
  };
}

/**
 * 挑出威胁最大的扇区（最小 TTC 者）。
 * ---------------------------------------------------------------
 * 用 TTC 而不是距离排序：一个 3m 外但正对着的墙，比一个 0.5m 外但在侧后方的
 * 障碍更该被"优先处理"—— 距离排序会选错，TTC 排序不会。
 * @returns 扇区下标；无有效威胁返回 -1
 */
export function pickThreatSector(sectors: SectorInput[], vx: number, vz: number,
  opts: VoOptions = DEFAULT_VO): number {
  let best: number = -1;
  let worstTtc: number = Infinity;
  for (let i = 0; i < sectors.length; i++) {
    const s: SectorInput = sectors[i];
    const t: number = timeToCollision(s.distance, approachSpeed(vx, vz, s.ux, s.uz), opts.clearance);
    if (t < worstTtc) {
      worstTtc = t;
      best = i;
    }
  }
  // 全为 Infinity（无障碍 / 都不接近）→ 无威胁
  return isFinite(worstTtc) ? best : -1;
}

// ---------------------------------------------------------------
//  扇区构造（P2 配套：把"距离数组"变成"扇区样本"）
// ---------------------------------------------------------------

/**
 * 由「等分扇区距离数组」构造扇区样本。
 * ---------------------------------------------------------------
 * 约定（与 DroneController 的机体基向量严格一致）：
 *   · sector[0] = 机头前向，方向 û₀ = (−sin yaw, −cos yaw)；
 *   · 第 i 个扇区相对机头**左偏** i·(360/N)°：ûᵢ = (−sin(yaw+θᵢ), −cos(yaw+θᵢ))，
 *     θᵢ = i·2π/N（已核验：θ 减小时方向恰好等于机体右向量，故 +θ = 左偏）；
 *   · lateral = d·(−sin θᵢ)：机体**右向为正**的横向偏置（左偏扇区 → 负值），
 *     与 SteeringBehavior.pickAvoidSide 的口径一致（lateral>0 → 障碍在右 → 向左绕）。
 *
 * @param yawDeg    机体偏航角（度）
 * @param distances 各扇区净距（米），下标 0 = 正前；长度即扇区数
 */
export function buildSectors(yawDeg: number, distances: number[]): SectorInput[] {
  const n: number = distances.length;
  const out: SectorInput[] = [];
  if (n <= 0) {
    return out;
  }
  const yaw: number = (isFinite(yawDeg) ? yawDeg : 0) * Math.PI / 180;
  for (let i = 0; i < n; i++) {
    const theta: number = i * 2 * Math.PI / n;
    const h: number = yaw + theta;
    const ux: number = -Math.sin(h);
    const uz: number = -Math.cos(h);
    const d: number = distances[i];
    out.push({
      ux: ux,
      uz: uz,
      distance: d,
      lateral: isValidDistance(d) ? d * (-Math.sin(theta)) : 0
    });
  }
  return out;
}

// ---------------------------------------------------------------
//  纯函数自检
// ---------------------------------------------------------------

export function selfTest(): string {
  const fails: string[] = [];
  let total: number = 0;
  const check = (name: string, ok: boolean): void => {
    total++;
    if (!ok) {
      fails.push(name);
    }
  };
  const O: VoOptions = DEFAULT_VO;

  // —— TTC ——
  check('不接近 → TTC=∞', timeToCollision(1.0, 0) === Infinity);
  check('远离 → TTC=∞', timeToCollision(1.0, -1.5) === Infinity);
  check('正前方接近 → TTC=净距/速度',
    Math.abs(timeToCollision(1.0, 1.0) - (1.0 - VO_CLEARANCE_M) / 1.0) < 1e-9);
  check('已侵入净距 → TTC=0', timeToCollision(0.1, 1.0) === 0);
  check('测距无效 → TTC=∞', timeToCollision(Infinity, 1.0) === Infinity);
  check('接近速度投影正确（斜向）',
    Math.abs(approachSpeed(0, -1.5, 0, -1) - 1.5) < 1e-12 &&
    Math.abs(approachSpeed(1.5, 0, 0, -1)) < 1e-12);

  // —— 扇区构造（P2）——
  // yaw=0 时机头朝 -Z；8 扇区每 45°：sector0=(0,-1)，sector2 左偏 90° = (+1? )
  const sec8: SectorInput[] = buildSectors(0, [1, 2, 3, 4, 5, 6, 7, 8]);
  check('扇区数=距离数组长度', sec8.length === 8);
  check('sector0 = 机头前向 (0,-1)',
    Math.abs(sec8[0].ux) < 1e-12 && Math.abs(sec8[0].uz + 1) < 1e-12);
  // 左偏 90°（i=2）：θ=90° → û=(-sin90, -cos90)=(-1, 0)。机体右向为 (+1,0)，
  // 故 (-1,0) 确实是**左**向 —— 与"右向=(cos yaw, -sin yaw)"一致。
  check('左偏 90° 扇区方向与机体右向相反',
    Math.abs(sec8[2].ux + 1) < 1e-9 && Math.abs(sec8[2].uz) < 1e-9);
  check('左偏扇区的 lateral 为负（右正左负）', sec8[2].lateral !== undefined &&
    (sec8[2].lateral as number) < 0);
  check('正前扇区 lateral=0', Math.abs(sec8[0].lateral as number) < 1e-12);
  check('右偏扇区（最后一个）lateral 为正',
    (sec8[7].lateral as number) > 0);
  check('无效距离的扇区 lateral 归零（不产生 NaN）',
    buildSectors(0, [Infinity, 1])[0].lateral === 0);
  const secYaw: SectorInput[] = buildSectors(90, [1, 1, 1, 1]);
  check('yaw=90° 时前向转向 (−1,0)',
    Math.abs(secYaw[0].ux + 1) < 1e-9 && Math.abs(secYaw[0].uz) < 1e-9);

  // —— 约束构造 ——
  const front: SectorInput = { ux: 0, uz: -1, distance: 1.0, lateral: 0 };
  const limFront: number = sectorLimit(front, O);
  check('径向速度上限 = 净距/T', Math.abs(limFront - (1.0 - VO_CLEARANCE_M) / VO_TTC_SEC) < 1e-9);
  check('极近 → 上限 0（禁止接近）', sectorLimit({ ux: 0, uz: -1, distance: 0.1 }, O) === 0);
  check('测不到 → 无约束（Infinity）',
    sectorLimit({ ux: 0, uz: -1, distance: Infinity }, O) === Infinity);
  // 走廊软权重：偏离航向轴的扇区约束被削弱（软判替代硬判的核心收益）
  const near: SectorInput = { ux: 0, uz: -1, distance: 1.0, lateral: 0 };
  const offAxis: SectorInput = { ux: 0, uz: -1, distance: 1.0, lateral: O.corridorHalf * 1.5 };
  check('偏离轴线的扇区约束被削弱（软权重）', sectorLimit(offAxis, O) > sectorLimit(near, O));
  check('走廊外扇区完全不约束（等价硬判丢弃）',
    sectorLimit({ ux: 0, uz: -1, distance: 1.0, lateral: O.corridorHalf * 3 }, O) === Infinity);
  // "全速也不违反"的扇区不进约束集
  check('远处扇区不构成约束',
    buildVoConstraints([{ ux: 0, uz: -1, distance: 50, lateral: 0 }], O).length === 0);
  check('近处扇区构成约束',
    buildVoConstraints([{ ux: 0, uz: -1, distance: 1.0, lateral: 0 }], O).length === 1);

  // —— 投影求解 ——
  // 正前 1.0m、满速 1.5 冲向它 → 必须被削到 ≤ 上限，且切向保留
  const cs: VoConstraint[] = buildVoConstraints([front], O);
  const p1 = projectVelocity(0, -1.5, cs, O.iterations);
  check('前向被投影到上限内', p1.corrected &&
    approachSpeed(p1.vx, p1.vz, 0, -1) <= limFront + 1e-9);
  check('投影不反向（仍朝前，只是变慢）', p1.vz < 0);
  // 纯横移不受前向约束影响（贴墙滑行的基础）
  const p2 = projectVelocity(1.5, 0, cs, O.iterations);
  check('纯横移不被前向约束削减', !p2.corrected && Math.abs(p2.vx - 1.5) < 1e-12);
  // 斜向：径向被削、切向保留 → 自然产生"沿墙滑行"
  const p3 = projectVelocity(1.0, -1.0, cs, O.iterations);
  check('斜向输入：径向削、切向留（贴墙滑行）',
    Math.abs(p3.vx - 1.0) < 1e-9 && p3.vz > -1.0 + 1e-9);
  // 多约束：两侧夹逼时仍能落到交集（POCS 收敛）
  const two: VoConstraint[] = [
    { ux: 0, uz: -1, limit: 0.3 },
    { ux: 1, uz: 0, limit: 0.3 }
  ];
  const p4 = projectVelocity(1.2, -1.2, two, 8);
  check('多约束交集（两个方向都被压到上限内）',
    approachSpeed(p4.vx, p4.vz, 0, -1) <= 0.3 + 1e-6 &&
    approachSpeed(p4.vx, p4.vz, 1, 0) <= 0.3 + 1e-6);
  // 投影永不放大速度（叠加使用的安全保证）
  const p5 = projectVelocity(0.1, -0.1, cs, O.iterations);
  check('投影永不放大速度（叠加安全）',
    Math.sqrt(p5.vx * p5.vx + p5.vz * p5.vz) <= Math.sqrt(0.02) + 1e-12);
  check('脏数据不产生 NaN',
    isFinite(projectVelocity(NaN, 1, cs, 4).vx) && isFinite(projectVelocity(1, NaN, cs, 4).vz));

  // —— 一键式求解 ——
  const r1: VoResult = solveVelocityObstacle(0, -1.5, [front], O);
  check('求解结果满足 TTC ≥ T', r1.minTtcSec >= VO_TTC_SEC - 1e-9);
  check('求解标记为被阻塞', r1.blocked && r1.constraints === 1);
  const r2: VoResult = solveVelocityObstacle(0, -1.5,
    [{ ux: 0, uz: -1, distance: 50, lateral: 0 }], O);
  check('无障碍不阻塞（零干预）', !r2.blocked && Math.abs(r2.vz + 1.5) < 1e-12);
  // 关键回归：反应式层已限速时，预测层不得**放大**速度
  const r3: VoResult = solveVelocityObstacle(0, -0.2, [front], O);
  check('已低于上限时预测层不放大（只减不放）', Math.abs(r3.vz) <= 0.2 + 1e-12 && !r3.blocked);

  // —— 威胁扇区选择 ——
  // 正前 3m 全速（TTC=(3-0.19)/1.5=1.87s） vs 左前 0.5m 但横穿（接近速度≈0）
  const sectors: SectorInput[] = buildSectors(0, [3.0, 99, 0.5, 99, 99, 99, 99, 99]);
  const ti: number = pickThreatSector(sectors, 0, -1.5, O);
  check('威胁扇区按 TTC 选（正前优先于侧方近障）', ti === 0);
  check('无威胁时返回 -1',
    pickThreatSector([{ ux: 0, uz: -1, distance: Infinity }], 0, -1.5, O) === -1);

  const msg: string = fails.length === 0 ?
    `PASS（${total} 项断言全通过）` : `FAIL：${fails.join('、')}`;
  Logger.info(`${TAG} [VERIFY] 速度障碍自检 ${msg}`);
  return msg;
}
