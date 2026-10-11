/**
 * 时间维度守护（Layer 5 · 物理层 · P1 演进）
 * ---------------------------------------------------------------
 * 解决的问题：CollisionDetector.evaluate() 是**纯瞬时函数** —— 输入一帧距离、
 * 输出一帧判定，没有任何记忆。真机上这带来三类"单帧看着都对、飞起来就错"的故障：
 *
 *   1. 读数噪声 → 深度帧 ±5cm 抖动，避障强度随之抖动，表现为刹车忽紧忽松；
 *   2. 硬停面边界抖动 → `emergency` 每帧独立重算，距离在 margin 附近徘徊时
 *      急停位反复翻转，HUD 报警闪烁、位置环（定点保持）被反复打断；
 *   3. 感知失联无计时 → sensorTrust() 需要"连续失联时长"，但库内无人计算它，
 *      于是失联兜底档位永远停在 normal（有算法、无通路）。
 *
 * 本模块把这三件事收进同一个纯函数 / 极薄状态类：
 *   · filterDistance           一阶低通，抑制读数噪声（**盲态直通**，不做插值）
 *   · emergencyWithHysteresis  急停迟滞，进入/退出用不同阈值（防边界翻转）
 *   · BlindTracker             失联计时器（为 sensorTrust 提供 blindSec）
 *   · corridorWeight           走廊软权重（把"走廊半宽"硬判改为连续衰减）
 *
 * 与 AR 完全解耦：不引用 AREngine / ArkGraphics3D / hilog，输入纯数字、输出纯数据。
 *
 * ⚠️ 零回归纪律：本模块**不修改** CollisionDetector.evaluate() 与
 * FusionGeometry.forwardGapFromWorldHit() 的既有行为 —— 两者都有 selfTest
 * 钉死的"走廊外 → ∞"硬判断言，改动会推翻已验证口径。软权重在此**新增**为
 * 独立函数，只被新链路（多扇区 / 速度障碍）消费。
 */
import { Logger } from '../core/Logger';
import { isValidDistance } from './CollisionDetector';

const TAG: string = 'TemporalGuard';

// ---------------------------------------------------------------
//  常量（单一事实来源：阈值只在此定义一次）
// ---------------------------------------------------------------

/**
 * 距离一阶低通的时间常数（秒）。
 * ---------------------------------------------------------------
 * 取值依据：真机深度采样约 30Hz（3 帧中值滤波 ≈ 0.1s 窗口），本库仿真定步长 1/60。
 * τ=0.12s 意味着一次阶跃读数需约 3τ≈0.36s 收敛到 95% —— 既吃掉 ±5cm 的逐帧抖动，
 * 又不会迟滞到"障碍已经很近了还在用旧读数"。
 * 上限约束：τ 必须 ≪ 满速下穿越减速带的时间（safeDist(1.5)=2.2m / 1.5m/s ≈ 1.5s），
 * 否则滤波引入的滞后会吃掉本就不富裕的刹车余量。0.12s 仅为该值的 8%。
 */
export const DEPTH_TAU_SEC: number = 0.12;

/**
 * 急停迟滞宽度（米）。
 * ---------------------------------------------------------------
 * 进入 stop：distance ≤ margin；退出 stop：distance > margin + 本值。
 * 取值 0.08m 的依据：大于真机深度噪声幅度（±0.05m），小于减速带宽度
 * （AVOID_SLOW_BAND_M=1.2m）的 7% —— 迟滞只用于消抖，不改变"该在哪停"。
 */
export const AVOID_HYSTERESIS_M: number = 0.08;

/**
 * 走廊软权重的过渡跨度（以走廊半宽为单位的倍数）。
 * |lateral| ≤ half → 权重 1；|lateral| ≥ half×(1+span) → 权重 0；中间 smoothstep 过渡。
 */
export const CORRIDOR_SOFT_SPAN: number = 1.0;

// ---------------------------------------------------------------
//  纯函数：距离滤波
// ---------------------------------------------------------------

/**
 * 距离一阶低通（纯函数，无状态；状态由调用方持有 prev）。
 * ---------------------------------------------------------------
 * 三条**刻意不对称**的规则，每条都有对应的故障模式：
 *
 *   ① 本帧读数无效（∞ / NaN / ≤0）→ **原样返回 raw，不插值**。
 *      盲是"这一帧防撞瞎了"，不是"障碍变远了"。若用上一帧的有效值插值出一个
 *      有限距离，等于把盲帧伪装成安全读数 —— 正是「HUD 显绿却撞墙」的成因。
 *   ② 上一帧无效、本帧有效 → **直接采信 raw**（快速捕获）。
 *      否则要从 ∞ 指数缓升，障碍逼近的头几帧被系统性低估，等于延迟刹车。
 *   ③ 两侧都有效 → 标准指数平滑：d += (raw − d)·(1 − e^(−dt/τ))。
 *      用 e^(−dt/τ) 而非固定系数，保证不同帧率（30Hz 真机 / 60Hz 仿真）行为一致。
 *
 * @param prev 上一帧的滤波结果（米）
 * @param raw  本帧原始读数（米）
 * @param dt   步长（秒；≤0 或非法时原样返回 prev）
 * @param tau  时间常数（秒）
 */
export function filterDistance(prev: number, raw: number, dt: number,
  tau: number = DEPTH_TAU_SEC): number {
  // ① 盲态直通：绝不把"读不到"插值成一个看起来安全的距离
  if (!isValidDistance(raw)) {
    return raw;
  }
  if (!isValidDistance(dt) || dt <= 0) {
    return prev;
  }
  // ② 首次捕获：直接采信，不从 ∞ 缓升
  if (!isValidDistance(prev)) {
    return raw;
  }
  const tauSafe: number = (isFinite(tau) && tau > 0) ? tau : DEPTH_TAU_SEC;
  const alpha: number = 1 - Math.exp(-dt / tauSafe);
  return prev + (raw - prev) * alpha;
}

// ---------------------------------------------------------------
//  纯函数：急停迟滞
// ---------------------------------------------------------------

/**
 * 带迟滞的急停判定（纯函数）。
 * ---------------------------------------------------------------
 * 为什么必须有迟滞：`emergency` 同时驱动三处 —— ① 前进分量清零；
 * ② DroneController 里"刹掉朝障碍去的速度分量"；③ syncState() 的 EmergencyHold 态。
 * 若每帧独立判定，距离在 margin 附近抖动时这三者会同频翻转，
 * 表现为急停报警闪烁 + 位置环反复被"急停时不介入"分支打断。
 *
 * @param prev         上一帧是否处于急停
 * @param distance     本帧距离（米）
 * @param margin       硬停面（米）
 * @param releaseExtra 退出所需的额外余量（米），默认 AVOID_HYSTERESIS_M
 */
export function emergencyWithHysteresis(prev: boolean, distance: number, margin: number,
  releaseExtra: number = AVOID_HYSTERESIS_M): boolean {
  if (!isValidDistance(distance)) {
    // 盲帧不得维持急停（否则一次丢帧就把飞机永久钉死），也不得新触发急停
    return false;
  }
  if (!isValidDistance(margin) || margin < 0) {
    return false;
  }
  const extra: number = (isFinite(releaseExtra) && releaseExtra > 0) ? releaseExtra : 0;
  if (prev) {
    // 已在急停：要"更远一点"才解除（迟滞带）
    return distance <= margin + extra;
  }
  return distance <= margin;
}

// ---------------------------------------------------------------
//  纯函数：走廊软权重
// ---------------------------------------------------------------

/** 平滑过渡函数：t ≤ 0 → 0，t ≥ 1 → 1，中间 3t²−2t³（一阶导两端为 0，无折角） */
export function smoothstep01(t: number): number {
  if (!isFinite(t)) {
    return 0;
  }
  if (t <= 0) {
    return 0;
  }
  if (t >= 1) {
    return 1;
  }
  return t * t * (3 - 2 * t);
}

/**
 * 走廊软权重（0..1）：命中点偏离航向轴线越远，越不该参与前向刹停。
 * ---------------------------------------------------------------
 * 相对既有硬判（AVOID_CORRIDOR_HALF_M 非此即彼）的改进：
 * 硬判在 |lateral| = half 处从 1 突降到 0 —— 障碍侧向缓慢移出走廊时，
 * 避障强度会**阶跃消失**，用户感到"明明快贴上了突然不减速了"。
 * 软权重把它变成连续衰减，且两端一阶导为 0（smoothstep），无折角抖动。
 *
 * 用途：多扇区 / 速度障碍链路里**按比例削弱**偏离轴的约束强度，
 * 而不是像硬判那样直接丢弃该扇区（丢弃会丢掉"擦身而过但确实很近"的信息）。
 *
 * @param lateral   命中点到航向轴线的垂直距离（米，取绝对值）
 * @param halfWidth 走廊半宽（米）；≤0 视为"未配置走廊筛选"，恒返回 1
 */
export function corridorWeight(lateral: number, halfWidth: number): number {
  if (!(halfWidth > 0)) {
    return 1; // 未配置 → 不做走廊筛选（避免 half=0 导致全盘归零）
  }
  const a: number = Math.abs(isFinite(lateral) ? lateral : 0);
  if (a <= halfWidth) {
    return 1;
  }
  const span: number = halfWidth * ((isFinite(CORRIDOR_SOFT_SPAN) && CORRIDOR_SOFT_SPAN > 0) ?
    CORRIDOR_SOFT_SPAN : 1);
  if (span <= 0) {
    return a <= halfWidth ? 1 : 0;
  }
  const t: number = (a - halfWidth) / span;
  return 1 - smoothstep01(t);
}

// ---------------------------------------------------------------
//  状态类：失联计时 + 组合守卫
// ---------------------------------------------------------------

/**
 * 感知失联计时器（为 CollisionDetector.sensorTrust 提供 blindSec）。
 * ---------------------------------------------------------------
 * 单独成类而不是塞进控制器：sensorTrust() 在物理层已实现并自检，
 * 但一直缺"谁来数秒"这一环 —— 本类就是那一环。
 */
export class BlindTracker {
  /** 已连续失联时长（秒） */
  public blindSec: number = 0;
  /** 已连续有效时长（秒，用于"恢复确认"类策略） */
  public validSec: number = 0;

  /**
   * 推进一步。
   * @param dt    步长（秒）
   * @param valid 本帧测距是否有效
   * @returns 更新后的连续失联时长（秒）
   */
  public update(dt: number, valid: boolean): number {
    if (!isFinite(dt) || dt <= 0) {
      return this.blindSec;
    }
    if (valid) {
      this.blindSec = 0;
      this.validSec += dt;
    } else {
      this.blindSec += dt;
      this.validSec = 0;
    }
    return this.blindSec;
  }

  /** 复位（换场景 / 重置仿真时调用） */
  public reset(): void {
    this.blindSec = 0;
    this.validSec = 0;
  }
}

/** 时间维度守护的单帧输出 */
export interface TemporalState {
  /** 滤波后的距离（米）；无效读数原样透传（保持"盲 ≠ 安全"） */
  distance: number;
  /** 本帧测距是否有效 */
  valid: boolean;
  /** 带迟滞的急停判定 */
  emergency: boolean;
  /** 连续失联时长（秒） */
  blindSec: number;
}

/**
 * 时间维度守护（组合状态机）：滤波 + 迟滞 + 失联计时，一帧一步。
 * ---------------------------------------------------------------
 * 之所以做成类而不是三个散落字段：三者共用同一个"上一帧"概念，
 * 散落存放极易出现"滤波用了旧值、迟滞用了新值"的错位，
 * 而这类错位在真机上表现为偶发抖动，极难复现定位。
 */
export class TemporalGuard {
  /** 最近一次滤波结果（米） */
  public filteredDistance: number = Infinity;
  /** 最近一次带迟滞的急停判定 */
  public emergency: boolean = false;
  /** 失联计时器 */
  public readonly blind: BlindTracker = new BlindTracker();

  /**
   * 推进一步。
   * @param dt      步长（秒）
   * @param raw     本帧原始前向净距（米）
   * @param margin  当前硬停面（米，通常取 avoidMarginFor(speed)）
   * @param tau     低通时间常数（秒）
   */
  public step(dt: number, raw: number, margin: number, tau: number = DEPTH_TAU_SEC): TemporalState {
    const valid: boolean = isValidDistance(raw);
    const distance: number = filterDistance(this.filteredDistance, raw, dt, tau);
    // 盲帧必须重置滤波记忆：否则恢复后第一帧会与失联前的旧读数插值
    this.filteredDistance = valid ? distance : Infinity;
    this.emergency = emergencyWithHysteresis(this.emergency, distance, margin);
    const blindSec: number = this.blind.update(dt, valid);
    return {
      distance: distance,
      valid: valid,
      emergency: this.emergency,
      blindSec: blindSec
    };
  }

  /** 复位 */
  public reset(): void {
    this.filteredDistance = Infinity;
    this.emergency = false;
    this.blind.reset();
  }
}

// ---------------------------------------------------------------
//  纯函数自检
// ---------------------------------------------------------------

/**
 * 纯函数自检（真机 hilog / CI / Node 均可直接 grep 验收）。
 * 用计数器统计断言总数，增删断言不会打印出错误的"共 N 项"。
 */
export function selfTest(): string {
  const fails: string[] = [];
  let total: number = 0;
  const check = (name: string, ok: boolean): void => {
    total++;
    if (!ok) {
      fails.push(name);
    }
  };

  // —— 滤波：盲态直通（最关键的一条）——
  check('盲帧不插值（∞ 原样透传）', filterDistance(1.0, Infinity, 1 / 60) === Infinity);
  check('盲帧不插值（NaN 原样透传）', Number.isNaN(filterDistance(1.0, NaN, 1 / 60)));
  check('盲帧不把旧值伪装成安全读数', filterDistance(0.3, Infinity, 1 / 60) !== 0.3);

  // —— 滤波：首次捕获直接采信（不从 ∞ 缓升）——
  check('首次捕获直接采信', filterDistance(Infinity, 2.0, 1 / 60) === 2.0);
  check('首次捕获逐位相等（无缓升延迟）',
    filterDistance(Infinity, 0.5, 1 / 60) === 0.5);

  // —— 滤波：指数平滑的方向与收敛性 ——
  const step1: number = filterDistance(2.0, 1.0, 1 / 60);
  check('向读数收敛（单调逼近）', step1 < 2.0 && step1 > 1.0);
  let d: number = 2.0;
  for (let i = 0; i < 60; i++) {
    d = filterDistance(d, 1.0, 1 / 60);
  }
  check('1 秒内基本收敛到读数', Math.abs(d - 1.0) < 0.02);
  // 帧率无关性：同样 0.24s，60Hz 与 30Hz 的结果应几乎一致
  let d60: number = 2.0;
  for (let i = 0; i < 14; i++) {
    d60 = filterDistance(d60, 1.0, 1 / 60);
  }
  let d30: number = 2.0;
  for (let i = 0; i < 7; i++) {
    d30 = filterDistance(d30, 1.0, 1 / 30);
  }
  check('帧率无关（60Hz 与 30Hz 一致）', Math.abs(d60 - d30) < 0.01);

  // —— 滤波：噪声抑制（同一真值上下抖 ±0.05，滤波后抖幅显著变小）——
  let noisy: number = 1.0;
  let maxDev: number = 0;
  for (let i = 0; i < 120; i++) {
    const raw: number = 1.0 + ((i % 2 === 0) ? 0.05 : -0.05);
    noisy = filterDistance(noisy, raw, 1 / 60);
    maxDev = Math.max(maxDev, Math.abs(noisy - 1.0));
  }
  check('噪声被抑制（滤波后偏离 < 原始幅度的 1/4）', maxDev < 0.0125);
  check('非法步长不改变状态', filterDistance(1.0, 2.0, 0) === 1.0 &&
    filterDistance(1.0, 2.0, NaN) === 1.0);
  check('非法时间常数回退默认（不产生 NaN）',
    isFinite(filterDistance(1.0, 2.0, 1 / 60, 0)) &&
    isFinite(filterDistance(1.0, 2.0, 1 / 60, NaN)));

  // —— 迟滞：进入用 margin，退出用 margin + extra ——
  check('未急停时 margin 内触发', emergencyWithHysteresis(false, 0.25, 0.25) === true);
  check('未急停时 margin 外不触发', emergencyWithHysteresis(false, 0.26, 0.25) === false);
  check('已急停时迟滞带内仍保持', emergencyWithHysteresis(true, 0.30, 0.25) === true);
  check('已急停时超出迟滞带才解除',
    emergencyWithHysteresis(true, 0.25 + AVOID_HYSTERESIS_M + 0.01, 0.25) === false);
  // 核心不变量：在 margin 边界反复抖动时，emergency **不得翻转**
  let prev: boolean = false;
  let flips: number = 0;
  for (let i = 0; i < 200; i++) {
    const dist: number = 0.25 + ((i % 2 === 0) ? 0.01 : -0.01);
    const now: boolean = emergencyWithHysteresis(prev, dist, 0.25);
    if (now !== prev) {
      flips++;
    }
    prev = now;
  }
  check('边界抖动下零翻转（迟滞消抖）', flips <= 1); // 只允许进入时的那一次
  // 盲帧不得维持急停（否则一次丢帧把飞机永久钉死）
  check('盲帧解除急停（不永久钉死）', emergencyWithHysteresis(true, Infinity, 0.25) === false);
  check('非法 margin 不误判急停',
    emergencyWithHysteresis(false, 0.1, NaN) === false &&
    emergencyWithHysteresis(false, 0.1, -1) === false);
  check('迟滞宽度可配置（收紧为 0 时退化为无迟滞）',
    emergencyWithHysteresis(true, 0.26, 0.25, 0) === false);

  // —— 走廊软权重 ——
  const half: number = 0.44; // DRONE_RADIUS_M + 0.25
  check('走廊内权重=1', corridorWeight(0, half) === 1 && corridorWeight(half, half) === 1);
  check('走廊外远处权重=0', corridorWeight(half * 2 + 0.01, half) === 0);
  const wMid: number = corridorWeight(half * 1.5, half);
  check('过渡区权重居中（单调连续）', wMid > 0 && wMid < 1 && Math.abs(wMid - 0.5) < 1e-9);
  check('过渡区单调递减',
    corridorWeight(half * 1.2, half) > corridorWeight(half * 1.5, half) &&
    corridorWeight(half * 1.5, half) > corridorWeight(half * 1.8, half));
  check('软权重对称（左右同权）',
    Math.abs(corridorWeight(-0.5, half) - corridorWeight(0.5, half)) < 1e-12);
  check('半宽未配置 → 不做走廊筛选（避免全盘归零）', corridorWeight(9, 0) === 1);
  check('非法 lateral 按 0 处理（不产生 NaN）',
    isFinite(corridorWeight(NaN, half)) && corridorWeight(NaN, half) === 1);

  // —— 失联计时器 ——
  const bt: BlindTracker = new BlindTracker();
  check('初始未失联', bt.blindSec === 0);
  bt.update(1 / 60, false);
  check('失联累加', Math.abs(bt.blindSec - 1 / 60) < 1e-12);
  for (let i = 0; i < 179; i++) {
    bt.update(1 / 60, false);
  }
  check('3 秒失联计时正确', Math.abs(bt.blindSec - 3) < 1e-6);
  bt.update(1 / 60, true);
  check('恢复后失联计时归零', bt.blindSec === 0 && bt.validSec > 0);
  check('非法步长不污染计时', bt.update(0, false) === 0 && bt.update(NaN, false) === 0);
  bt.reset();
  check('复位清零', bt.blindSec === 0 && bt.validSec === 0);

  // —— 组合守卫 ——
  const g: TemporalGuard = new TemporalGuard();
  const s1: TemporalState = g.step(1 / 60, 2.0, 0.25);
  check('守卫首帧直接采信', s1.distance === 2.0 && s1.valid && !s1.emergency);
  // 滤波滞后是**设计而非缺陷**：单帧近读数不足以立刻急停，
  // 否则一次深度噪声（读数突然跳到 0.2m）就能把飞机刹死。
  const s2: TemporalState = g.step(1 / 60, 0.2, 0.25);
  check('单帧近读数不立刻急停（抗噪是设计）', !s2.emergency && s2.distance < 2.0);
  let sHold: TemporalState = s2;
  for (let i = 0; i < 60; i++) {
    sHold = g.step(1 / 60, 0.2, 0.25);
  }
  check('持续近距离后触发急停（约 0.35s 内收敛）',
    sHold.emergency && sHold.distance < 0.25);
  const s3: TemporalState = g.step(1 / 60, Infinity, 0.25);
  check('守卫盲帧解除急停并累计失联', !s3.emergency && !s3.valid &&
    Math.abs(s3.blindSec - 1 / 60) < 1e-12);
  // 关键：盲帧后滤波记忆必须清零，恢复首帧不得与失联前旧值插值
  const s4: TemporalState = g.step(1 / 60, 1.0, 0.25);
  check('恢复首帧不插值旧值（直接采信）', s4.distance === 1.0);
  g.reset();
  check('守卫复位', g.filteredDistance === Infinity && !g.emergency && g.blind.blindSec === 0);

  const msg: string = fails.length === 0 ?
    `PASS（${total} 项断言全通过）` : `FAIL：${fails.join('、')}`;
  Logger.info(`${TAG} [VERIFY] 时间维度守护自检 ${msg}`);
  return msg;
}
