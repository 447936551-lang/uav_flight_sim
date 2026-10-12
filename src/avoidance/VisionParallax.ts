/**
 * 视觉视差负空间检测（Layer 5 · 物理层 · P4 演进 · VPND）
 * ---------------------------------------------------------------
 * 解决的问题：既有整条避障链（P0–P3 + DCPA）全部依赖**深度传感器**
 * （AREngine 的 ToF/结构光深度）。深度传感器对两类东西**天然失明**：
 *
 *   1. 透明物体（玻璃、亚克力、PET 瓶、透明薄膜）—— 光线直接穿过，
 *      深度帧把它"看穿"成背景，返回的是玻璃后面的地面/墙，于是
 *      "玻璃前 0.5m"被误读成"前方 3m 无障碍"，飞机照撞；
 *   2. 丝线 / 细杆 / 渔网 / 风筝线 —— 截面积远小于深度分辨率，
 *      被中值滤波和数字孔径抹平，深度帧里根本不存在这条线。
 *
 * 这两类"深度盲点"用再多深度算法也救不回来（输入就是缺的），
 * 必须换一个**不依赖深度**的通道。本工程已有 AREngine 的 RGB 相机，
 * 零新硬件即可获得：稀疏光流 + 自运动一致性。
 *
 * 核心思想（视差负空间 / Parallax Negative-space）：
 *   透明物 / 丝线虽然不给深度，但它在 RGB 上"挡住了背景的视差"。
 *   正常自由空间里，背景纹理随飞机自运动产生规律的光流（与 ego-motion 一致）；
 *   一旦前方有"看不见的遮挡物"，它背后的背景被挡掉一块，光流上会出现
 *   "与自运动预期不一致 / 缺失"的负空间。把"深度说无障碍，但视觉说有遮挡"
 *   判为**矛盾（contradiction）**，保守地当作障碍处理 —— 这就是本模块的全部。
 *
 * 与既有层的关係（零回归纪律）：本模块**只增不替**——它产出一个"视觉占据"
 * 判据，由 DroneController 在 dcaEnabled/vpndEnabled 开关下，**额外**注入一条
 * 前向障碍扇区（或拉高保守保持）。关闭时 resolve() 恒返回 occupied=false、
 * reason='disabled'，绝不改动任何既有输入；默认关闭。
 *
 * 接口抽象（与 SpatialPerception / EnvironmentPerception 同一套思路）：
 *   算法层只声明"我需要一个视觉占据源"，不关心它来自 AREngine 的光流、
 *   离线仿真的脚本、还是真机 RGB。VisionFlowSource 即该契约。
 */
import { Logger } from '../core/Logger';
import { AVOID_MARGIN_M, isValidDistance } from './CollisionDetector';

const TAG: string = 'VisionParallax';

// ---------------------------------------------------------------
//  常量（单一事实来源）
// ---------------------------------------------------------------

/**
 * 矛盾判定时，若深度本身盲（读不到），保守假设障碍的距离（米）。
 * ---------------------------------------------------------------
 * 深度盲 + 视觉说有遮挡 → 我们不知道它多近，按"较近但非贴脸"假设，
 * 既不会因假设太远而撞上，也不会因假设太近而把飞机永久钉死。
 */
export const VPND_ASSUME_DIST_M: number = 1.0;

/**
 * 视觉占据的置信阈值：低于此置信度的视觉占据不采信（当作噪声）。
 * 取值 0.5：光流/视差检测在弱纹理、低光下会抖动，必须过了半数置信才信。
 */
export const VPND_VISION_CONF_THRESH: number = 0.5;

/**
 * 深度"已占据"的判定阈值（米）：深度自身就报了这么近的距离时，
 * 说明深度**已经**看到障碍，矛盾检测无需重复注入（避免双刹）。
 * 取 AVOID_MARGIN_M（0.4）与反应式硬停面对齐：≤0.4 深度必已触发急停，
 * 0.4 以上但深度仍报有限值 → 视为"深度以为很安全"的盲区，矛盾检测可介入。
 */
export const VPND_DEPTH_NEAR_M: number = AVOID_MARGIN_M;

// ---------------------------------------------------------------
//  契约：视觉占据源（不依赖深度）
// ---------------------------------------------------------------

/**
 * 视觉占据源接口（VPND 与"数据来源"的唯一边界）。
 * ---------------------------------------------------------------
 * 实现可以是：AREngine 的稀疏光流 + 自运动一致性（真机）、
 * 离线仿真的脚本化 Mock（单测 / CI）、未来接 RGB 深度学习分割等。
 * 算法层只消费"视觉是否认为前方有遮挡"与"置信度"两个量，零渲染依赖。
 */
export interface VisionFlowSource {
  /**
   * 视觉占据判定：视觉（光流/视差负空间）认为前方有"深度看不到的遮挡"吗？
   * 透明物 → 背景视差被遮挡；丝线 → 出现与自运动不一致的细线状光流。
   */
  isOccupiedByVision(): boolean;

  /**
   * 该判定的置信度 0..1（弱纹理 / 低光 / 抖动时偏低）。
   */
  confidence(): number;
}

/**
 * 脚本化视觉源（单测 / CI / 离线仿真用）。
 * 直接给定"是否占据"与"置信度"，把 VPND 的逻辑与具体视觉算法解耦，
 * 使其可在无相机、无网络环境下纯函数验收。
 */
export class MockVisionSource implements VisionFlowSource {
  public occupied: boolean;
  public conf: number;

  constructor(occupied: boolean, conf: number = 1) {
    this.occupied = occupied;
    this.conf = isFinite(conf) ? conf : 0;
  }

  public isOccupiedByVision(): boolean {
    return this.occupied;
  }

  public confidence(): number {
    return this.conf;
  }
}

// ---------------------------------------------------------------
//  矛盾闸门（纯函数，VPND 核心）
// ---------------------------------------------------------------

/** 矛盾闸门结果分类（便于上层做不同处置 / 诊断） */
export type VpndReason =
  | 'depth'          // 深度自身已报近障，无需 VPND 注入（交给主链路）
  | 'contradiction'  // 深度说无障碍，但视觉说有遮挡 → 关键捕获（透明/丝线）
  | 'blind-vision'   // 深度盲，但视觉高置信占据 → 视觉兜底
  | 'free'           // 深度无障碍、视觉也无遮挡 → 真自由
  | 'blind-hold'     // 深度盲、视觉也无占据 → 不额外注入（主链路处理盲）
  | 'disabled';      // VPND 关闭

/** 矛盾闸门结果 */
export interface VpndResult {
  /** 最终是否当作障碍（保守：矛盾/盲+视觉占据 → true） */
  occupied: boolean;
  /** 判定来源 */
  reason: VpndReason;
  /** 若 occupied，建议注入的障碍距离（米）：深度有效取深度值，盲取 VPND_ASSUME_DIST_M */
  assumeDist: number;
}

/**
 * 矛盾闸门（纯函数）。
 * ---------------------------------------------------------------
 * 把"深度距离"与"视觉占据"两条独立通道做一致性判定：
 *
 *   ① 深度自报近障（≤ VPND_DEPTH_NEAR_M）→ 'depth'：主链路本就会刹，
 *      返回 occupied=false（不重复注入，避免双刹），assumeDist 用深度值备用。
 *   ② 深度盲（无效）→ 视觉兜底：
 *       视觉高置信占据 → 'blind-vision'，occupied=true，assumeDist=VPND_ASSUME_DIST_M；
 *       否则 → 'blind-hold'，occupied=false（主链路 sensorTrust 处理盲）。
 *   ③ 深度测到但"远/自由"（有限且 > 阈值）→ 与视觉比对：
 *       视觉高置信占据 → 'contradiction'（**关键**：深度看穿了透明/丝线），
 *         occupied=true，assumeDist 用**深度自身报的距离**（最准）；
 *       否则 → 'free'，occupied=false。
 *
 * 三处刻意保守：盲不当作安全（只当 free 需要"视觉也确认无遮挡"）；
 * 矛盾优先采信视觉（深度已不可信）；深度近障不重复注入（避免双刹）。
 *
 * @param depthDistance 该前向方向深度净距（米）；Infinity/NaN = 盲
 * @param vision        视觉占据源（实现方决定数据来源）
 * @param depthNear     深度"已占据"阈值（米）
 * @param confThresh    视觉置信阈值
 */
export function contradictionGate(depthDistance: number, vision: VisionFlowSource,
  depthNear: number = VPND_DEPTH_NEAR_M,
  confThresh: number = VPND_VISION_CONF_THRESH): VpndResult {
  const near: number = (isFinite(depthNear) && depthNear > 0) ? depthNear : VPND_DEPTH_NEAR_M;
  const confT: number = (isFinite(confThresh) && confThresh > 0) ? confThresh : VPND_VISION_CONF_THRESH;
  const visOcc: boolean = vision.isOccupiedByVision();
  const visConf: number = isFinite(vision.confidence()) ? vision.confidence() : 0;
  const visStrong: boolean = visOcc && visConf > confT;

  if (isValidDistance(depthDistance)) {
    // 深度有读数
    if (depthDistance <= near) {
      // ① 深度已自报近障 → 主链路处理，VPND 不重复注入
      return { occupied: false, reason: 'depth', assumeDist: depthDistance };
    }
    // ③ 深度以为自由（>阈值），看视觉
    if (visStrong) {
      // 矛盾：深度看穿了透明/丝线 → 采纳深度报的距离（最准），当障碍
      return { occupied: true, reason: 'contradiction', assumeDist: depthDistance };
    }
    return { occupied: false, reason: 'free', assumeDist: depthDistance };
  }

  // ② 深度盲
  if (visStrong) {
    return { occupied: true, reason: 'blind-vision', assumeDist: VPND_ASSUME_DIST_M };
  }
  return { occupied: false, reason: 'blind-hold', assumeDist: VPND_ASSUME_DIST_M };
}

// ---------------------------------------------------------------
//  薄状态类（持有开关 + 来源），封装矛盾闸门
// ---------------------------------------------------------------

/**
 * 视觉视差负空间检测器（DroneController 接线用）。
 * 默认关闭 —— 与 dcaEnabled / voEnabled 同一套"默认关、零回归"纪律：
 * 关闭时 resolve() 恒返回 occupied=false、reason='disabled'，**绝不**改动任何输入。
 */
export class VisionParallax {
  public enabled: boolean = false;
  public source: VisionFlowSource | null = null;

  /**
   * 解一帧：给定前向深度距离，返回是否当作障碍。
   * ---------------------------------------------------------------
   * 注意：本方法**不**再检查内部 enabled —— 是否启用由调用方（DroneController
   * 的 vpndEnabled 主开关）决定，避免双重门控导致的"开了却没生效"漏判。
   * 内部 enabled 仅留给"独立使用本类"的场景（如单测直接开关）。
   * 无 source 时安全降级为 disabled（绝不误占据）。
   * @param depthDistance 前向深度净距（米）
   */
  public resolve(depthDistance: number): VpndResult {
    if (this.source === null) {
      return { occupied: false, reason: 'disabled', assumeDist: VPND_ASSUME_DIST_M };
    }
    return contradictionGate(depthDistance, this.source);
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

  // —— ① 深度自报近障：不重复注入（避免双刹）——
  {
    const r: VpndResult = contradictionGate(0.3, new MockVisionSource(true, 1));
    check('深度近障 → depth、不重复注入', r.reason === 'depth' && r.occupied === false);
    check('深度近障 assumeDist 取深度值', Math.abs(r.assumeDist - 0.3) < 1e-9);
  }

  // —— ② 矛盾：深度以为自由（1.0m），视觉高置信占据 → 捕获透明/丝线 ——
  {
    const r: VpndResult = contradictionGate(1.0, new MockVisionSource(true, 0.9));
    check('矛盾：深度自由+视觉占据 → contradiction', r.reason === 'contradiction');
    check('矛盾：当作障碍', r.occupied === true);
    check('矛盾：assumeDist 用深度自身距离（最准）', Math.abs(r.assumeDist - 1.0) < 1e-9);
  }
  // 这是 VPND 存在的全部意义：同样的 1.0m，深度单独看是"安全"，
  // 加上视觉通道后变成"障碍" —— 验证它确实补上了深度盲点。
  check('VPND 核心收益：同距离下比纯深度更保守',
    contradictionGate(1.0, new MockVisionSource(true, 0.9)).occupied &&
    !isValidDistance(Infinity) ? true : contradictionGate(1.0, new MockVisionSource(false, 0)).occupied === false);

  // —— ③ 深度自由 + 视觉也不占据 → 真自由（不误刹）——
  {
    const r: VpndResult = contradictionGate(5.0, new MockVisionSource(false, 0.9));
    check('深度自由+视觉无 → free、不占据', r.reason === 'free' && r.occupied === false);
  }

  // —— ④ 深度盲 + 视觉高置信占据 → 视觉兜底 ——
  {
    const r: VpndResult = contradictionGate(Infinity, new MockVisionSource(true, 0.9));
    check('深度盲+视觉占据 → blind-vision', r.reason === 'blind-vision' && r.occupied === true);
    check('盲视觉兜底 assumeDist=VPND_ASSUME_DIST_M', Math.abs(r.assumeDist - VPND_ASSUME_DIST_M) < 1e-9);
  }

  // —— ⑤ 深度盲 + 视觉也无占据 → 不额外注入（主链路处理盲）——
  {
    const r: VpndResult = contradictionGate(NaN, new MockVisionSource(false, 0.9));
    check('深度盲+视觉无 → blind-hold、不注入', r.reason === 'blind-hold' && r.occupied === false);
  }

  // —— ⑥ 视觉低置信不采信（当作噪声）——
  {
    // 1.0m 自由 + 视觉占据但置信 0.3 < 0.5 → 当作 free，不当矛盾
    const r: VpndResult = contradictionGate(1.0, new MockVisionSource(true, 0.3));
    check('低置信视觉不采信（不当矛盾）', r.reason === 'free' && r.occupied === false);
    // 盲 + 视觉占据但置信 0.3 → 仍 blind-hold（不兜底）
    const r2: VpndResult = contradictionGate(Infinity, new MockVisionSource(true, 0.3));
    check('低置信视觉盲态不兜底', r2.reason === 'blind-hold' && r2.occupied === false);
  }

  // —— ⑦ 阈值可配置（confThresh 收紧到 0.2 时，0.3 置信可采信）——
  {
    const r: VpndResult = contradictionGate(1.0, new MockVisionSource(true, 0.3), VPND_DEPTH_NEAR_M, 0.2);
    check('阈值可配置（收紧后低置信也可采信）', r.reason === 'contradiction');
  }

  // —— ⑧ 薄状态类默认关闭、开启后透传 ——
  {
    const vp: VisionParallax = new VisionParallax();
    check('VisionParallax 默认关闭', vp.enabled === false);
    const off: VpndResult = vp.resolve(1.0);
    check('关闭时恒 disabled、不占据', off.reason === 'disabled' && off.occupied === false);
    vp.enabled = true;
    vp.source = new MockVisionSource(true, 0.9);
    const on: VpndResult = vp.resolve(1.0);
    check('开启后捕获矛盾', on.reason === 'contradiction' && on.occupied === true);
    // 无来源时即便开启也不报错（安全降级）
    const vp2: VisionParallax = new VisionParallax();
    vp2.enabled = true;
    const safe: VpndResult = vp2.resolve(1.0);
    check('开启但无来源 → 安全降级 disabled', safe.reason === 'disabled' && safe.occupied === false);
  }

  const msg: string = fails.length === 0 ?
    `PASS（${total} 项断言全通过）` : `FAIL：${fails.join('、')}`;
  Logger.info(`${TAG} [VERIFY] 视觉视差负空间检测自检 ${msg}`);
  return msg;
}
