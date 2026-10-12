/**
 * 延迟补偿预测式避障（Layer 5 · 物理层 · P4 演进 · DCPA）
 * ---------------------------------------------------------------
 * 解决的问题：既有避障链（P0 反应式 / P1 时间维 / P3 预测式速度障碍）全部建立在
 * "这一帧读到的距离 = 此刻的真实距离" 的隐含假设上。真机上这不成立，带来三类
 * "算得都对、飞起来仍慢半拍" 的延迟故障：
 *
 *   1. 感知延迟 τ_p：深度帧从采集到送达决策，AREngine 约 30Hz + 3 帧中值滤波
 *      ≈ 0.1s。决策用的"距离"其实是 0.1s 前机体位置量到的 —— 飞机已经又飞近了
 *      v·τ_p 米，却还在按旧距离算刹车余量，等于凭空少算了这段刹车距离。
 *   2. 时间预算漏算延迟：既有硬停面 avoidMarginFor(v) 只算了"刹车距离"，
 *      没算"从发现到开始减速"这段延迟里飞机还要往前冲 v·τ_total 米。
 *      于是高速逼近时，等真开始减速，已经把延迟距离也吃掉了。
 *   3. 急停被低通拖慢：P1 的 TemporalGuard 一阶低通专为抑制读数噪声设计，
 *      但临界 TTC 时低通反而把"该立刻满刹"的信号也平滑掉了，延迟了急停。
 *
 * 本模块用纯数学、零新硬件、覆盖式（不替换既有层）的方式同时修补这三处：
 *   · PDC（Perception-Delay Cancellation）感知延迟抵消：用机体速度把扇区云
 *     沿飞行方向"前推" τ_p，把 0.1s 前的旧距离修正成"此刻"的距离；
 *   · TBB（Time-Budget Braking）时间预算刹车：把硬停面改成
 *     margin(v) + v·τ_total 的"时间预算"口径，早刹但不误刹；
 *   · EFP（Emergency Fast-Path）急停快路径：TTC < τ_total + ε 时直接满刹，
 *     跳过 TemporalGuard 低通，专治临界急停被平滑拖慢。
 *
 * 与既有层的关係（零回归纪律）：本模块**不修改** CollisionDetector /
 * TemporalGuard / VelocityObstacle 的任何既有行为，只作为**叠加输入**使用——
 * ① PDC 修正后的扇区仍交给 solveVelocityObstacle 投影；② TBB 仅在早刹判定上
 * 比反应式层更保守；③ EFP 只在 TTC 临界时额外强制急停。全部由 dcaEnabled 开关
 * 控，默认关闭 —— 关闭时本模块的所有函数原样透传，绝不改变任何下游输入。
 */
import { Logger } from '../core/Logger';
import {
  AVOID_BRAKE_ACCEL,
  AVOID_MARGIN_BASE_M,
  isValidDistance,
} from './CollisionDetector';
import { SectorInput, approachSpeed } from './VelocityObstacle';

const TAG: string = 'DelayCompensator';

// ---------------------------------------------------------------
//  常量（单一事实来源）
// ---------------------------------------------------------------

/**
 * 感知延迟 τ_p（秒）：深度帧从采集到送达决策的端到端延迟。
 * ---------------------------------------------------------------
 * 取值依据：AREngine 深度约 30Hz 输出 + 工程常用 3 帧中值滤波 ≈ 0.1s。
 * 这是"决策用的距离比真实位置旧了多久"的核心参数。标定可改：真机若换了
 * 更低延迟的管线，调小即可；保守偏大不会更危险，只会更早刹。
 */
export const DCA_TAU_PERCEPT_SEC: number = 0.10;

/**
 * 控制延迟 τ_ctrl（秒）：决策算出期望速度 → 控制环发出指令的延迟。
 * 取一帧定步长上限 + 指令流水线余量 ≈ 0.05s。
 */
export const DCA_TAU_CTRL_SEC: number = 0.05;

/**
 * 执行延迟 τ_act（秒）：指令 → 电机响应 → 飞机实际开始减速的延迟。
 * 与 RotorPlant 的 MOTOR_TAU=0.05 同口径（电机一阶滞后）。
 */
export const DCA_TAU_ACT_SEC: number = 0.05;

/**
 * 急停快路径的容差 ε（秒）：TTC 与阈值比对时允许的小余量，避免临界抖动。
 */
export const DCA_FASTPATH_EPS_SEC: number = 0.05;

/** DCA 配置（把上面四个延迟 + 制动减速度 + 基线余量打包，便于真机标定覆盖） */
export interface DcaConfig {
  /** 感知延迟 τ_p（秒） */
  tauPerceptSec: number;
  /** 控制延迟 τ_ctrl（秒） */
  tauCtrlSec: number;
  /** 执行延迟 τ_act（秒） */
  tauActSec: number;
  /** 制动减速度（m/s²），默认复用 AVOID_BRAKE_ACCEL */
  brakeAccel: number;
  /** 硬停面静态基线（m），默认 AVOID_MARGIN_BASE_M */
  marginBase: number;
  /** 急停快路径容差 ε（秒） */
  fastPathEpsSec: number;
}

/** 默认 DCA 配置（与既有阈值同源，避免两套口径打架） */
export const DEFAULT_DCA: DcaConfig = {
  tauPerceptSec: DCA_TAU_PERCEPT_SEC,
  tauCtrlSec: DCA_TAU_CTRL_SEC,
  tauActSec: DCA_TAU_ACT_SEC,
  brakeAccel: AVOID_BRAKE_ACCEL,
  marginBase: AVOID_MARGIN_BASE_M,
  fastPathEpsSec: DCA_FASTPATH_EPS_SEC,
};

/** 总延迟 = 感知 + 控制 + 执行（秒） */
export function totalLatency(cfg: DcaConfig = DEFAULT_DCA): number {
  const t: number = cfg.tauPerceptSec + cfg.tauCtrlSec + cfg.tauActSec;
  return isFinite(t) && t > 0 ? t : DCA_TAU_PERCEPT_SEC + DCA_TAU_CTRL_SEC + DCA_TAU_ACT_SEC;
}

// ---------------------------------------------------------------
//  PDC：感知延迟抵消（纯函数）
// ---------------------------------------------------------------

/**
 * 单扇区感知延迟补偿（PDC 核心）。
 * ---------------------------------------------------------------
 * 把"τ_p 秒前机体位置量到的距离"修正成"此刻机体位置下，沿该扇区方向到障碍的距离"。
 * 设 û 为扇区方向（机体指向障碍），接近速度 closing = v·û（>0 表示正在接近）。
 * 在 τ_p 内飞机沿 û 又飞近了 closing·τ_p 米，故：
 *     d_now = d_captured − closing·τ_p
 * 符号自洽：远离（closing<0）时 d_now > d_captured（障碍实际更远了，修正后更宽松）；
 * 接近（closing>0）时 d_now < d_captured（障碍实际更近了，修正后更保守）。
 *
 * 三条**刻意不对称**规则（与 TemporalGuard 同样的"盲 ≠ 安全"纪律）：
 *   ① 测距无效（∞/NaN/≤0）→ **原样返回**，绝不把"读不到"伪装成更近的安全距离；
 *   ② τ_p ≤ 0 或非法 → 返回 d_captured（无延迟可补偿，零回归）；
 *   ③ 修正后距离不得为负（已穿过/贴近）→ 夹到 0（最危险，不是"穿过去了变安全"）。
 *
 * @param dCaptured 该扇区在 τ_p 秒前量到的净距（米）
 * @param vx,vz     机体世界系速度（m/s），与扇区方向 ux,uz 同坐标系
 * @param ux,uz     扇区方向单位向量（世界系 XZ 平面）
 * @param tauP      感知延迟（秒）
 */
export function compensateSectorDistance(dCaptured: number, vx: number, vz: number,
  ux: number, uz: number, tauP: number = DCA_TAU_PERCEPT_SEC): number {
  // ① 盲态直通：绝不把"读不到"伪装成"更近了"（那会变成"凭空少算刹车距离"的反面错误）
  if (!isValidDistance(dCaptured)) {
    return dCaptured;
  }
  // ② 无延迟可补偿 → 原样返回（零回归）
  if (!(isFinite(tauP) && tauP > 0)) {
    return dCaptured;
  }
  const closing: number = approachSpeed(vx, vz, ux, uz); // >0 接近
  const dNow: number = dCaptured - closing * tauP;
  // ③ 不得为负：夹到 0（最危险，而非"穿过去就安全了"）
  return dNow > 0 ? dNow : 0;
}

/**
 * 整组扇区的感知延迟补偿（PDC）。
 * ---------------------------------------------------------------
 * 逐扇区调用 compensateSectorDistance，保留方向、横向偏置等其余字段。
 * 无效扇区、τ_p≤0 时逐项原样透传。返回新数组（不修改入参）。
 */
export function compensateSectors(sectors: SectorInput[], vx: number, vz: number,
  tauP: number = DCA_TAU_PERCEPT_SEC): SectorInput[] {
  if (!Array.isArray(sectors)) {
    return [];
  }
  return sectors.map((s: SectorInput): SectorInput => ({
    ux: s.ux,
    uz: s.uz,
    distance: compensateSectorDistance(s.distance, vx, vz, s.ux, s.uz, tauP),
    lateral: s.lateral
  }));
}

// ---------------------------------------------------------------
//  TBB：时间预算刹车（纯函数）
// ---------------------------------------------------------------

/**
 * 所需刹车距离（米）：从当前速度到完全停住，飞机还要前进多远。
 * ---------------------------------------------------------------
 * 拆成两段（与运动学严格自洽）：
 *   ① 延迟段：在 τ_total 内飞机仍以 v 前进 → 吃掉 v·τ_total 米；
 *   ② 制动段：匀减速到 0 → v²/(2·a) 米。
 * 两段相加即"再不减速就一定会撞上"的临界距离。
 *
 * @param speed  当前速度大小（m/s，非负；非法/负按 0）
 * @param aMax   制动减速度（m/s²）
 * @param tauTotal 总延迟（秒）= τ_p + τ_ctrl + τ_act
 */
export function requiredStoppingDistance(speed: number, aMax: number = AVOID_BRAKE_ACCEL,
  tauTotal: number = DCA_TAU_PERCEPT_SEC + DCA_TAU_CTRL_SEC + DCA_TAU_ACT_SEC): number {
  const v: number = (isFinite(speed) && speed > 0) ? speed : 0;
  const a: number = (isFinite(aMax) && aMax > 0) ? aMax : AVOID_BRAKE_ACCEL;
  const tau: number = (isFinite(tauTotal) && tauTotal > 0) ?
    tauTotal : DCA_TAU_PERCEPT_SEC + DCA_TAU_CTRL_SEC + DCA_TAU_ACT_SEC;
  return v * tau + (v * v) / (2 * a);
}

/**
 * 时间预算硬停面（米）：TBB 触发早刹的距离阈值。
 * ---------------------------------------------------------------
 * = 静态基线余量 + 速度相关刹车距离 + 延迟段距离
 *   = marginBase + v²/(2·a) + v·τ_total
 * 前两项正是既有 avoidMarginFor(v)，本函数在此之上**追加**延迟段 v·τ_total，
 * 把"延迟里还要往前冲"也纳入硬停面。由此：
 *   · 沿用既有手感（减速带宽度 band 不变）；
 *   · 高速时比反应式层更早刹（保守是刻意的，只更安全）。
 */
export function timeBudgetMargin(speed: number, cfg: DcaConfig = DEFAULT_DCA): number {
  const v: number = (isFinite(speed) && speed > 0) ? speed : 0;
  const a: number = (isFinite(cfg.brakeAccel) && cfg.brakeAccel > 0) ?
    cfg.brakeAccel : AVOID_BRAKE_ACCEL;
  const base: number = (isFinite(cfg.marginBase) && cfg.marginBase > 0) ?
    cfg.marginBase : AVOID_MARGIN_BASE_M;
  const tau: number = totalLatency(cfg);
  return base + (v * v) / (2 * a) + v * tau;
}

/**
 * 早刹判定（TBB）：修正后的距离是否已进入"时间预算硬停面"。
 * ---------------------------------------------------------------
 * 与反应式层 avoidMarginFor 的区别仅在于多了 v·τ_total 这一项 ——
 * 因此本判定在任何速度下都 ≥ 反应式层判定，绝不会比它更激进（零回归保证）。
 */
export function shouldBrakeEarly(dNow: number, speed: number,
  cfg: DcaConfig = DEFAULT_DCA): boolean {
  if (!isValidDistance(dNow)) {
    return false; // 盲：不靠 TBB 触发（盲由 sensorTrust 另行处理）
  }
  return dNow <= timeBudgetMargin(speed, cfg);
}

// ---------------------------------------------------------------
//  EFP：急停快路径（纯函数）
// ---------------------------------------------------------------

/**
 * 急停快路径判定（EFP）。
 * ---------------------------------------------------------------
 * 当剩余碰撞时间 TTC 已经小到"延迟 + 容差"以内，意味着：留给"发现→减速→停下"
 * 的总时间已经不够，必须**立刻满刹**，不能再走 TemporalGuard 的低通（那会把
 * 临界急停信号也平滑掉，反而更慢）。
 *
 * 判定：TTC ≤ τ_total + ε 即触发（含边界，更保守：临界时宁肯多刹不可少刹）。
 * TTC=∞（无威胁）永不触发。
 *
 * @param ttc  当前最小碰撞时间（秒），Infinity 表示无威胁
 * @param cfg  DCA 配置（取 τ_total 与 ε）
 */
export function emergencyFastPath(ttc: number, cfg: DcaConfig = DEFAULT_DCA): boolean {
  if (!isFinite(ttc)) {
    return false; // 无威胁 → 不触发（绝不误刹）
  }
  const tau: number = totalLatency(cfg);
  const eps: number = (isFinite(cfg.fastPathEpsSec) && cfg.fastPathEpsSec > 0) ?
    cfg.fastPathEpsSec : DCA_FASTPATH_EPS_SEC;
  return ttc <= tau + eps;
}

// ---------------------------------------------------------------
//  组合叠加接口（DroneController 接线用）
// ---------------------------------------------------------------

/** DCA 一帧叠加结果 */
export interface DcaOverlay {
  /** PDC 修正后的扇区（未启用时 == 入参，原样透传） */
  sectors: SectorInput[];
  /** TBB 早刹触发 */
  braking: boolean;
  /** EFP 急停快路径触发 */
  emergencyFast: boolean;
  /** 修正后"最接近障碍"的前向净距（米）；无有效扇区 = Infinity */
  compensatedForwardDist: number;
}

/**
 * 一键式 DCA 叠加：PDC 修正扇区 → TBB 早刹 → EFP 急停。
 * ---------------------------------------------------------------
 * 设计为"只产出叠加信号"，不修改任何既有层：
 *   · sectors 修正后直接喂给 solveVelocityObstacle（投影只会更保守）；
 *   · braking / emergencyFast 由 DroneController 在既有急停分支里**额外**参考，
 *     且只在 enabled 时可能产生 true —— 关闭时三项全部回落到"不介入"，零回归。
 *
 * @param enabled  总开关（dcaEnabled）；false 时原样透传、不触发
 * @param sectors  原始扇区（世界系方向与速度同坐标系）
 * @param vx,vz    机体世界系速度（m/s）
 * @param cfg      DCA 配置
 */
export function dcaOverlay(enabled: boolean, sectors: SectorInput[], vx: number, vz: number,
  cfg: DcaConfig = DEFAULT_DCA): DcaOverlay {
  if (!enabled) {
    // 零回归：关闭时完全不介入，扇区原样、判定全 false
    return {
      sectors: sectors,
      braking: false,
      emergencyFast: false,
      compensatedForwardDist: Infinity
    };
  }
  const comp: SectorInput[] = compensateSectors(sectors, vx, vz, cfg.tauPerceptSec);
  const speed: number = Math.sqrt(
    (isFinite(vx) ? vx : 0) * (isFinite(vx) ? vx : 0) +
    (isFinite(vz) ? vz : 0) * (isFinite(vz) ? vz : 0));
  // 取修正后最小的、"正在接近"的扇区距离作为前向净距
  let fwd: number = Infinity;
  for (const s of comp) {
    if (!isValidDistance(s.distance)) {
      continue;
    }
    const closing: number = approachSpeed(vx, vz, s.ux, s.uz);
    if (closing > 0 && s.distance < fwd) {
      fwd = s.distance;
    }
  }
  const braking: boolean = shouldBrakeEarly(fwd, speed, cfg);
  // EFP 用"修正后最小 TTC"判定（与 VelocityObstacle.minTtc 同口径，但输入是修正扇区）
  let ttc: number = Infinity;
  for (const s of comp) {
    if (!isValidDistance(s.distance)) {
      continue;
    }
    const closing: number = approachSpeed(vx, vz, s.ux, s.uz);
    if (closing <= 1e-4) {
      continue;
    }
    const gap: number = s.distance - 0; // 修正距离本身已含余量，按净距算 TTC
    if (gap <= 0) {
      ttc = 0;
      break;
    }
    const t: number = gap / closing;
    if (t < ttc) {
      ttc = t;
    }
  }
  const emergencyFast: boolean = emergencyFastPath(ttc, cfg);
  return {
    sectors: comp,
    braking: braking,
    emergencyFast: emergencyFast,
    compensatedForwardDist: fwd
  };
}

/**
 * 薄状态类（持有配置 + 启用标志），封装 dcaOverlay 调用。
 * 默认关闭 —— 与既有 temporalEnabled / voEnabled 同样的"默认关、零回归"纪律。
 */
export class DelayCompensator {
  public enabled: boolean = false;
  public config: DcaConfig = DEFAULT_DCA;

  public overlay(sectors: SectorInput[], vx: number, vz: number): DcaOverlay {
    return dcaOverlay(this.enabled, sectors, vx, vz, this.config);
  }
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
  const C: DcaConfig = DEFAULT_DCA;
  const tauP: number = C.tauPerceptSec; // 0.10
  const tauTotal: number = totalLatency(C); // 0.20

  // —— PDC：盲态直通（最关键的一条）——
  check('PDC 盲帧不伪造成更近距离（∞ 原样）',
    compensateSectorDistance(Infinity, 1.5, 0, 0, -1, tauP) === Infinity);
  check('PDC 盲帧不伪造成更近距离（NaN 原样）',
    Number.isNaN(compensateSectorDistance(NaN, 1.5, 0, 0, -1, tauP)));
  check('PDC 无效距离不被夹到 0（保持盲）',
    compensateSectorDistance(Infinity, 9, 0, 0, -1, tauP) !== 0);

  // —— PDC：接近时距离被修正得更近 ——
  // 扇区方向 û=(0,−1) 即机头前向（−Z）；"以 1.5m/s 接近"对应机体速度 (0,−1.5)，
  // closing = v·û = (−1.5)×(−1) = 1.5；d_now = 1.0 − 1.5×0.1 = 0.85
  const dApproach: number = compensateSectorDistance(1.0, 0, -1.5, 0, -1, tauP);
  check('PDC 接近时修正为更近（1.0−0.15=0.85）', Math.abs(dApproach - 0.85) < 1e-9);
  // 远离时修正为更远（不应误刹）：机体速度 (0,+1.5)，closing = −1.5
  const dAway: number = compensateSectorDistance(1.0, 0, 1.5, 0, -1, tauP);
  check('PDC 远离时修正为更远（不误刹）', Math.abs(dAway - 1.15) < 1e-9);
  // 斜向：closing = v·û 只取径向分量，切向分量不计入修正
  // 速度 (1.0,−1.0)：径向 = vz·uz = (−1.0)×(−1) = 1.0，切向 vx·ux = 0
  const dSlant: number = compensateSectorDistance(1.0, 1.0, -1.0, 0, -1, tauP);
  check('PDC 只修正径向分量（切向不计）', Math.abs(dSlant - 0.90) < 1e-9);

  // —— PDC：夹零不夹负（已穿过→最危险）——
  // 速度 (0,−1.5)：d_now = 0.05 − 1.5×0.1 = −0.10 → 夹到 0
  const dClamp: number = compensateSectorDistance(0.05, 0, -1.5, 0, -1, tauP);
  check('PDC 修正为负时夹到 0（不穿越变安全）', dClamp === 0);

  // —— PDC：τ_p≤0 原样透传（零回归）——
  check('PDC τ_p=0 原样透传', compensateSectorDistance(1.0, 1.5, 0, 0, -1, 0) === 1.0);
  check('PDC τ_p 非法原样透传',
    compensateSectorDistance(1.0, 1.5, 0, 0, -1, NaN) === 1.0);

  // —— PDC：整组扇区（保留方向/横向）——
  const secs: SectorInput[] = [
    { ux: 0, uz: -1, distance: 1.0, lateral: 0 },
    { ux: 1, uz: 0, distance: 5.0, lateral: 1.0 }
  ];
  const comp: SectorInput[] = compensateSectors(secs, 0, -1.5, tauP);
  check('PDC 整组长度不变', comp.length === 2);
  check('PDC 整组修正前向扇区',
    Math.abs(comp[0].distance - 0.85) < 1e-9 && comp[0].ux === 0 && comp[0].uz === -1);
  check('PDC 整组保留横向偏置', comp[1].lateral === 1.0);
  // 纯横移（vx=1.5, vz=0）前向扇区不应被径向修正（closing=0）
  const compSide: SectorInput[] = compensateSectors(secs, 1.5, 0, tauP);
  check('PDC 纯横移不修正前向（径向=0）',
    Math.abs(compSide[0].distance - 1.0) < 1e-9);
  check('PDC 空数组返回空', compensateSectors([], 1, 1, tauP).length === 0);

  // —— TBB：所需刹车距离 ——
  // v=0 → 0；v=1.5,a=1.5,τ=0.2 → 1.5×0.2 + 1.5²/(2×1.5) = 0.3 + 0.75 = 1.05
  check('TBB 静止刹车距离=0', requiredStoppingDistance(0, 1.5, 0.2) === 0);
  check('TBB 刹车距离 = vτ + v²/2a（1.05）',
    Math.abs(requiredStoppingDistance(1.5, 1.5, 0.2) - 1.05) < 1e-9);
  check('TBB 负速度按 0', requiredStoppingDistance(-1, 1.5, 0.2) === 0);
  check('TBB 非法参数不产生 NaN',
    isFinite(requiredStoppingDistance(NaN, 1.5, 0.2)) &&
    isFinite(requiredStoppingDistance(1.5, 0, 0.2)));

  // —— TBB：时间预算硬停面（含延迟段）——
  // marginBase=0.25, v=1.5 → 0.25 + 0.75 + 1.5×0.2 = 1.30
  check('TBB 时间预算硬停面 = base + v²/2a + vτ（1.30）',
    Math.abs(timeBudgetMargin(1.5, C) - 1.30) < 1e-9);
  check('TBB 静止时间预算硬停面 = base（0.25）',
    Math.abs(timeBudgetMargin(0, C) - 0.25) < 1e-9);
  // 零回归对照：TBB 阈值在任何速度下 ≥ 反应式 avoidMarginFor
  // （avoidMarginFor(1.5)=1.0；TBB(1.5)=1.30 ≥ 1.0 ✓）
  check('TBB 阈值 ≥ 反应式层（只更安全）', timeBudgetMargin(1.5, C) >= 1.0);

  // —— TBB：早刹判定 ——
  check('TBB 进入时间预算硬停面即早刹（d=1.0<1.30）',
    shouldBrakeEarly(1.0, 1.5, C) === true);
  check('TBB 远距不早刹（d=2.0>1.30）', shouldBrakeEarly(2.0, 1.5, C) === false);
  check('TBB 盲帧不早刹（不误用盲）', shouldBrakeEarly(Infinity, 1.5, C) === false);

  // —— EFP：急停快路径 ——
  // τ_total=0.20, ε=0.05 → 阈值 0.25
  check('EFP TTC=0.3 不触发（>0.25）', emergencyFastPath(0.3, C) === false);
  check('EFP TTC=0.25 触发（=阈值）', emergencyFastPath(0.25, C) === true);
  check('EFP TTC=0.10 触发（<阈值）', emergencyFastPath(0.10, C) === true);
  check('EFP 无威胁（∞）不触发', emergencyFastPath(Infinity, C) === false);
  check('EFP 已侵入（TTC=0）触发', emergencyFastPath(0, C) === true);

  // —— 组合叠加（dcaOverlay）——
  // 关闭：完全不介入（扇区原样、判定全 false）
  const off: DcaOverlay = dcaOverlay(false, secs, 0, -1.5, C);
  check('DCA 关闭时原样透传扇区（不修正）',
    Math.abs(off.sectors[0].distance - 1.0) < 1e-9 && !off.braking && !off.emergencyFast);
  // 开启 + 正前 1.0m、以 1.5m/s 接近 → 修正 0.85、未进硬停面(1.30)、EFP 看 TTC
  const on: DcaOverlay = dcaOverlay(true, secs, 0, -1.5, C);
  check('DCA 开启修正前向扇区', Math.abs(on.sectors[0].distance - 0.85) < 1e-9);
  check('DCA 开启取最小接近距离为前向净距（0.85）',
    Math.abs(on.compensatedForwardDist - 0.85) < 1e-9);
  // 该场景 TTC = 0.85 / 1.5 ≈ 0.567 > 0.25 → 不急停；但 0.85<1.30 → 早刹
  check('DCA 早刹触发（修正距离<时间预算硬停面）', on.braking === true);
  check('DCA 该场景未到 EFP（TTC>阈值）', on.emergencyFast === false);
  // 临界场景：正前 0.3m、以 1.5m/s 接近 → TTC=0.3/1.5=0.2 < 0.25 → EFP 触发
  const crit: SectorInput[] = [{ ux: 0, uz: -1, distance: 0.3, lateral: 0 }];
  const onCrit: DcaOverlay = dcaOverlay(true, crit, 0, -1.5, C);
  check('DCA 临界 TTC 触发 EFP 急停快路径',
    onCrit.emergencyFast === true && Math.abs(onCrit.compensatedForwardDist - 0.15) < 1e-9);
  // 纯横移（vx=1.5）前向 closing=0 → 不触发任何 DCA（贴墙滑行基础不被破坏）
  const onSide: DcaOverlay = dcaOverlay(true, secs, 1.5, 0, C);
  check('DCA 纯横移不触发（径向=0）', !onSide.braking && !onSide.emergencyFast);

  // —— 薄状态类 ——
  const dc: DelayCompensator = new DelayCompensator();
  check('DelayCompensator 默认关闭', dc.enabled === false);
  const dcOff: DcaOverlay = dc.overlay(secs, 0, -1.5);
  check('DelayCompensator 关闭原样透传', Math.abs(dcOff.sectors[0].distance - 1.0) < 1e-9);
  dc.enabled = true;
  const dcOn: DcaOverlay = dc.overlay(secs, 0, -1.5);
  check('DelayCompensator 开启修正扇区', Math.abs(dcOn.sectors[0].distance - 0.85) < 1e-9);

  const msg: string = fails.length === 0 ?
    `PASS（${total} 项断言全通过）` : `FAIL：${fails.join('、')}`;
  Logger.info(`${TAG} [VERIFY] 延迟补偿预测式避障自检 ${msg}`);
  return msg;
}
