/**
 * HUD 状态推导（Layer 7 · UI / 告警 · 纯函数）
 * ---------------------------------------------------------------
 * 把 DroneController 每帧给出的「前向障碍距离 / 避障强度 / 急停标志」
 * 收敛成 UI 可用的威胁等级、配色与文案。本模块**零 ArkUI 依赖**，
 * 因此可以脱离设备直接单测（与 CollisionDetector / DroneController 同思路）。
 *
 * 阈值口径（单一事实来源，禁止在此重新定义）：
 *   - AVOID_MARGIN_M    硬停面（= 0.4m）：< 此距离即「刹停 / 急停」。
 *   - AVOID_SAFE_DIST_M 避障生效距离（= 1.6m = margin + 减速带）：≥ 此距离即「安全」。
 *   - HUD_WARN_DIST_M   HUD 减速带中段（= (margin + safeDist) / 2 = 1.0m）：
 *                       减速带内、硬停面外的中段，归为「接近(Warning)」。
 *
 * 与真实刹停行为对齐（不另起一套阈值，避免「面板红、飞机却没刹」的错位）：
 *   - Danger  ：avoidEmergency === true，或（有限距离 且 distance < AVOID_MARGIN_M）。
 *   - Warning ：有限距离 且 distance < HUD_WARN_DIST_M（减速带中段）。
 *   - Caution ：有限距离 且 distance < AVOID_SAFE_DIST_M（减速带外段）。
 *   - Safe    ：其余（含 distance >= 1.6m 或 distance 非有限 / Infinity）。
 *
 * 注意：avoidanceEnabled（是否开启辅助避障）**不参与**颜色判定——
 * 「近障碍」就是「近障碍」，与用户有没有开辅助无关（关了辅助照样会撞）。
 */
import { AVOID_MARGIN_M, AVOID_SAFE_DIST_M } from '../avoidance/CollisionDetector';
import { Logger } from '../core/Logger';

/**
 * HUD 威胁等级（序即严重程度：Safe < Caution < Warning < Danger）。
 * 数值序被 AlertService 直接用作「边沿比较」的依据，不要调换枚举值顺序。
 */
export enum HudThreat {
  Safe,
  Caution,
  Warning,
  Danger,
}

/**
 * HUD 减速带中段阈值 = (硬停面 + 生效距离) / 2 = (0.4 + 1.6) / 2 = 1.0m。
 * 放在减速带中段作为「接近」与「注意」的分界。
 */
const HUD_WARN_DIST_M: number = AVOID_MARGIN_M + (AVOID_SAFE_DIST_M - AVOID_MARGIN_M) * 0.5;

/** UI 层配色（ArkUI 十六进制字符串，区别于 DroneVisuals 的 0..1 RGB 浮点） */
export const HUD_COLOR_SAFE: string = '#3DD68C';
const HUD_COLOR_CAUTION: string = '#FFD93D';
const HUD_COLOR_WARNING: string = '#FFA53D';
const HUD_COLOR_DANGER: string = '#FF5B5B';

/** HUD 派生状态：UI 直接消费的不可变快照 */
export interface HudState {
  /** 威胁等级 */
  level: HudThreat;
  /** 当前配色（ArkUI 十六进制） */
  colorHex: string;
  /** 中文标签：安全 / 注意 / 接近 / 危险 */
  label: string;
  /** 距离文案：有限 → "x.xx m"，非有限 → "∞" */
  distanceText: string;
}

/**
 * 由无人机当前感知状态推导 HUD 快照。
 *
 * @param obstacleDistance 前向障碍距离（米），Infinity 表示无障碍 / 深度不可用
 * @param avoidStrength    当前避障强度 0..1（本函数不用于配色，保留为接口一致性）
 * @param avoidEmergency  是否处于急停（已进入硬停面）
 * @returns 不可变 HudState 快照
 */
export function deriveHud(obstacleDistance: number, avoidStrength: number, avoidEmergency: boolean): HudState {
  // —— 等级判定（与真实刹停对齐）——
  let level: HudThreat = HudThreat.Safe;
  if (avoidEmergency || (isFinite(obstacleDistance) && obstacleDistance < AVOID_MARGIN_M)) {
    level = HudThreat.Danger;
  } else if (isFinite(obstacleDistance) && obstacleDistance < HUD_WARN_DIST_M) {
    level = HudThreat.Warning;
  } else if (isFinite(obstacleDistance) && obstacleDistance < AVOID_SAFE_DIST_M) {
    level = HudThreat.Caution;
  } else {
    level = HudThreat.Safe;
  }

  // —— 文案 / 配色（按等级映射，集中映射避免散落）——
  let label: string = '安全';
  let colorHex: string = HUD_COLOR_SAFE;
  if (level === HudThreat.Caution) {
    label = '注意';
    colorHex = HUD_COLOR_CAUTION;
  } else if (level === HudThreat.Warning) {
    label = '接近';
    colorHex = HUD_COLOR_WARNING;
  } else if (level === HudThreat.Danger) {
    label = '危险';
    colorHex = HUD_COLOR_DANGER;
  }

  // 距离文案：有限 → 保留两位小数 + " m"；非有限（Infinity / NaN）→ "∞"
  let distanceText: string = '∞';
  if (isFinite(obstacleDistance)) {
    distanceText = `${obstacleDistance.toFixed(2)} m`;
  }

  return { level: level, colorHex: colorHex, label: label, distanceText: distanceText };
}

/**
 * 纯函数自检（与 CollisionDetector / DroneController 同风格：动态计数断言，
 * 失败抛 Error 由初始化逻辑的 try/catch 吸收，绝不影响启动）。
 * 断言项：Infinity→Safe 且 distanceText='∞'；2.0→Safe；1.2→Caution；
 * 0.8→Warning；0.3→Danger；avoidEmergency=true 且 distance=1.0→Danger；
 * 1.6 边界→Safe；distanceText 格式。
 */
export function selfTest(): void {
  const fails: string[] = [];
  let total: number = 0;
  const check = (name: string, ok: boolean): void => {
    total++;
    if (!ok) {
      fails.push(name);
    }
  };

  const inf = deriveHud(Infinity, 0, false);
  check('Infinity→Safe', inf.level === HudThreat.Safe);
  check('Infinity→distanceText=∞', inf.distanceText === '∞');

  const far = deriveHud(2.0, 0, false);
  check('2.0m→Safe', far.level === HudThreat.Safe);

  const caution = deriveHud(1.2, 0.3, false);
  check('1.2m→Caution', caution.level === HudThreat.Caution);

  const warning = deriveHud(0.8, 0.6, false);
  check('0.8m→Warning', warning.level === HudThreat.Warning);

  const danger = deriveHud(0.3, 1.0, false);
  check('0.3m→Danger', danger.level === HudThreat.Danger);

  const emergency = deriveHud(1.0, 0.5, true);
  check('emergency=true 且 1.0m→Danger', emergency.level === HudThreat.Danger);

  const boundary = deriveHud(1.6, 0.1, false);
  check('1.6m 边界→Safe', boundary.level === HudThreat.Safe);

  check('distanceText 格式(2.0m)', far.distanceText === '2.00 m');
  check('colorHex 与 Warning 对齐', warning.colorHex === HUD_COLOR_WARNING);
  check('label 与 Danger 对齐', danger.label === '危险');
  check('label 与 Caution 对齐', caution.label === '注意');

  if (fails.length !== 0) {
    throw new Error(`HudModel 自检失败：${fails.join('、')}`);
  }
  Logger.info(`[VERIFY] HudModel 自检 PASS（${total} 项断言全通过）`);
}
