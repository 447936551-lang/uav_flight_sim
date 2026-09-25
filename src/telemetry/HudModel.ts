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
 *   - Blind   ：测距链路没有可信读数（depthTrustworthy === false）。
 *   - Safe    ：其余（有限距离 且 distance >= 1.6m）。
 *
 * 【除等级外，还有两条正交的「不是安全」通道】——它们都**只**改写文案/配色/距离呈现，
 * 不参与等级判定，因此既不改变飞行行为，也不产生额外响铃：
 *   - degraded（相机代测）：量的地方不对（手机前方 ≠ 无人机前方）。
 *   - suppressed（前方不可判）：量对了，但所有前方命中都被自己的几何判据剔除了
 *     —— ∞ 在这里不是「畅通」，而是「没得出任何结论」。
 *
 * 【Blind 为什么必须独立存在（本文件最重要的一处修正）】
 *   DroneController.obstacleDistance === Infinity 有**两种完全相反**的成因：
 *     (a) 前方确实没有障碍（探测正常，只是都没命中）→ 真的安全；
 *     (b) 这一帧防撞根本是「盲」的（深度降级 / 未就绪 / 位姿不可用 / 探测点全在
 *         相机视野外）→ 什么都不知道，绝不能说安全。
 *   旧实现把 (b) 一并归入 Safe，于是「跟踪丢失、防撞彻底失效」时 HUD 反而稳稳显示
 *   「∞ 安全」—— 真机上用户看到的就是「一直都是安全，并没有避障」，完全无法察觉
 *   避障已经瞎了。这与 CollisionDetector / SteeringBehavior 里「Infinity 不是安全，
 *   而是这一帧防撞是盲的」的既有语义直接矛盾，故在此拆出 Blind。
 *   判据由调用方（页面）从 ARDepthSampler 取「测距是否有可信读数」传入，
 *   本模块不重新推导，保持纯函数与单一事实来源。
 *
 * 注意：avoidanceEnabled（是否开启辅助避障）**不参与**颜色判定——
 * 「近障碍」就是「近障碍」，与用户有没有开辅助无关（关了辅助照样会撞）。
 */
import { AVOID_MARGIN_M, AVOID_SAFE_DIST_M } from '../avoidance/CollisionDetector';
import { Logger } from '../core/Logger';

/**
 * HUD 威胁等级（序即严重程度，数值序被 AlertService 直接用作「边沿比较」的依据，
 * 不要调换枚举值顺序）。
 *
 * 2026-09-22 新增 Blind（测距不可用），插在 Safe 与 Caution 之间：
 *   · 排位高于 Safe —— 它比「安全」更值得用户注意（虽然两者都不告警）；
 *   · 排位低于 Caution —— 它不是威胁，不能触发任何告警。
 *   配套：AlertService.evaluate 显式豁免 Blind（不响铃），并在 Blind 时复位到 Safe。
 */
export enum HudThreat {
  Safe,
  Blind,
  Caution,
  Warning,
  Danger
}

/**
 * HUD 减速带中段阈值 = (硬停面 + 生效距离) / 2 = (0.4 + 1.6) / 2 = 1.0m。
 * 放在减速带中段作为「接近」与「注意」的分界。
 */
const HUD_WARN_DIST_M: number = AVOID_MARGIN_M + (AVOID_SAFE_DIST_M - AVOID_MARGIN_M) * 0.5;

/** UI 层配色（ArkUI 十六进制字符串，区别于 DroneVisuals 的 0..1 RGB 浮点） */
// 这些色值直接压在相机画面上的 HUD 底板上，须满足《通用应用 UX 体验标准》
// 2.1.4.1：标题/图标 > 3:1，正文 > 4.5:1。底板为 #CC000000 时，最暗的红
// 在 HUD 底板上的对比度最低，故把危险色由 #FF5B5B 提亮到 #FF8080（5.2:1），
// 其余三色本就在 6:1 以上。
export const HUD_COLOR_SAFE: string = '#3DD68C';
const HUD_COLOR_CAUTION: string = '#FFD93D';
const HUD_COLOR_WARNING: string = '#FFA53D';
const HUD_COLOR_DANGER: string = '#FF8080';
/**
 * 「测距不可用」配色：中性灰蓝，刻意**不用绿色**——绿色会被读成「安全」。
 * 该色在 #CC000000 底板上的对比度约 8:1，满足《通用应用 UX 体验标准》正文 > 4.5:1。
 */
export const HUD_COLOR_BLIND: string = '#9AA7B8';
/**
 * 「相机代测」配色：冷蓝，与 Blind 的灰、Safe 的绿都拉开距离。
 * 语义是「有读数，但量的不是无人机前方」，既不能显示成安全，也不是完全无数据。
 */
export const HUD_COLOR_FALLBACK: string = '#6FB7FF';

/** HUD 派生状态：UI 直接消费的不可变快照 */
export interface HudState {
  /** 威胁等级 */
  level: HudThreat;
  /** 当前配色（ArkUI 十六进制） */
  colorHex: string;
  /** 中文标签：安全 / 测距不可用 / 注意 / 接近 / 危险 */
  label: string;
  /** 距离文案：有限 → "x.xx m"；盲 → "—"；其余非有限 → "∞" */
  distanceText: string;
  /**
   * 本帧距离是否来自「相机代测」回退（无人机本体探针处于盲区）。
   * 为真时 label 固定为「相机代测」：量到的是手机前方的净距，不是无人机前方的。
   */
  degraded: boolean;
  /**
   * 本帧是否属于「有读数、但所有前方命中都被几何判据剔除」的抑制态。
   * 为真时 label 固定为「前方不可判」、距离不报数（"—"）。
   *
   * 与 degraded 的区别：degraded 是「量的地方不对」（相机代测），
   * suppressed 是「量的地方对、但结论被自己的剔除规则吃掉了」。
   * 两者都必须与「安全」区分开，这是 2026-09-25 真机「贴近障碍仍显示安全」的收口。
   */
  suppressed: boolean;
}

/**
 * 由无人机当前感知状态推导 HUD 快照。
 *
 * @param obstacleDistance 前向障碍距离（米），Infinity 表示「无读数」（无障 或 盲）
 * @param avoidStrength    当前避障强度 0..1（本函数不用于配色，保留为接口一致性）
 * @param avoidEmergency   是否处于急停（已进入硬停面）
 * @param depthTrustworthy 测距链路本帧是否给出**可信**读数。
 *                         false = 深度降级 / 未就绪 / 位姿不可用 / 探测点全在图外
 *                         —— 此时 obstacleDistance 的 ∞ 只代表「不知道」，判 Blind。
 *                         必填而非默认 true：默认值会悄悄掩盖「盲」，宁可让调用方漏传时编译报错。
 * @param degraded        本帧距离是否来自「相机代测」回退（无人机本体探针在盲区）。
 *                        为真时 label 固定为「相机代测」——量到的是手机前方的净距，
 *                        不是无人机前方的；照旧显示成「安全」就是又一次面板说谎。
 *                        该距离已在采样器侧按 CAM_FALLBACK_AUTH_CAP 封顶（≥1.0m），
 *                        因此这里最重只会落到 Caution，绝不可能出现「危险」。
 * @param suppressed      本帧是否属于「有可信读数、但所有前方命中都被几何判据剔除」。
 *                        ---------------------------------------------------------------
 *                        这是 2026-09-25 真机缺陷（无人机贴着杂物堆/柜体，HUD 仍是绿「∞
 *                        安全」）的收口，与 Blind 是**同一个病根的第二支**：
 *                        obstacleDistance === Infinity 原本有三种成因，
 *                          (a) 前方确实 3m 内无遮挡（探测正常）      → 真的安全
 *                          (b) 这一帧防撞是盲的                      → Blind（已有）
 *                          (c) 测到了，但命中全被支撑面/走廊/可飞越剔掉 → 什么结论都没有
 *                        (c) 曾被并进 (a) —— 于是「自己把障碍丢掉」被渲染成「安全」。
 *                        采样器以「在走廊内、不在机体下方、在探针量程内，却仅因
 *                        支撑面判据被丢弃」的命中数 > 0 给出判据，本模块不重新推导。
 *                        为真时距离不报数（"—"）并用「不知道」的中性灰：
 *                        ∞ 是「没结论」，渲染成一个具体读数同样是失真。
 *                        **不改 level**：等级仍完全由物理阈值决定，从而不产生额外响铃。
 * @returns 不可变 HudState 快照
 */
export function deriveHud(obstacleDistance: number, avoidStrength: number,
  avoidEmergency: boolean, depthTrustworthy: boolean, degraded: boolean,
  suppressed: boolean): HudState {
  // —— 等级判定（与真实刹停对齐）——
  let level: HudThreat = HudThreat.Safe;
  if (avoidEmergency || (isFinite(obstacleDistance) && obstacleDistance < AVOID_MARGIN_M)) {
    // 急停优先于一切：即便测距自称不可信，硬停面是实测出来的，仍按危险报。
    level = HudThreat.Danger;
  } else if (!depthTrustworthy) {
    level = HudThreat.Blind;
  } else if (isFinite(obstacleDistance) && obstacleDistance < HUD_WARN_DIST_M) {
    level = HudThreat.Warning;
  } else if (isFinite(obstacleDistance) && obstacleDistance < AVOID_SAFE_DIST_M) {
    level = HudThreat.Caution;
  } else {
    level = HudThreat.Safe;
  }

  // —— 文案 / 配色（按等级映射，集中映射避免散落）——
  // 「抑制态」与「代测」都只在**非 Blind / 非 Danger** 时改写文案：
  //   · Blind 是「连读数都没有」，语义比抑制更强，不能被降级覆盖；
  //   · Danger 是实测到了硬停面，是既成事实，宁可重复报警也不能被淡化。
  // 另外抑制态只在**没有数可报**（非有限）时才接管：一旦这帧真有有限读数，
  // 那就是真实信息，藏起来反而是第二重失真；何况等级已在报警。
  const suppressedActive: boolean = suppressed && !isFinite(obstacleDistance) &&
    level !== HudThreat.Blind && level !== HudThreat.Danger;
  let label: string = '安全';
  let colorHex: string = HUD_COLOR_SAFE;
  if (level === HudThreat.Blind) {
    label = '测距不可用';
    colorHex = HUD_COLOR_BLIND;
  } else if (level === HudThreat.Caution) {
    label = '注意';
    colorHex = HUD_COLOR_CAUTION;
  } else if (level === HudThreat.Warning) {
    label = '接近';
    colorHex = HUD_COLOR_WARNING;
  } else if (level === HudThreat.Danger) {
    label = '危险';
    colorHex = HUD_COLOR_DANGER;
  }
  // —— 代测覆盖：有读数但语义降级，标签必须改写，颜色单独一档 ——
  // 放在等级映射之后：等级（含告警行为）仍由物理阈值决定，只有文案/配色被改写，
  // 这样 AlertService 的边沿比较不会因为「代测」而产生额外的响铃。
  if (degraded && level !== HudThreat.Blind && level !== HudThreat.Danger) {
    label = '相机代测';
    colorHex = HUD_COLOR_FALLBACK;
  }
  // —— 抑制覆盖：测到了，但结论被自己的剔除规则吃掉了 ——
  // 放在代测之后：抑制态意味着「本体/相机都测到过东西，却被判据丢弃」，
  // 比「量的地方不对」更接近事故（可能真有障碍被丢），故优先级更高。
  // 复用 Blind 的中性灰：两者同属「不知道」，用同一色系表达同一认知状态，
  // 文案区分成因；刻意不引入新色，避免与三级威胁色（黄/橙/红）抢占语义。
  if (suppressedActive) {
    label = '前方不可判';
    colorHex = HUD_COLOR_BLIND;
  }

  // 距离文案：盲 / 抑制 → "—"（不报数，避免把「不知道」渲染成一个具体读数）；
  //          有限 → 保留两位小数 + " m"；其余非有限 → "∞"（理论上已不可达，保留兜底）。
  let distanceText: string = '∞';
  if (level === HudThreat.Blind || suppressedActive) {
    distanceText = '—';
  } else if (isFinite(obstacleDistance)) {
    distanceText = `${obstacleDistance.toFixed(2)} m`;
  }

  return {
    level: level,
    colorHex: colorHex,
    label: label,
    distanceText: distanceText,
    degraded: degraded,
    suppressed: suppressed
  };
}

/**
 * 纯函数自检（与 CollisionDetector / DroneController 同风格：动态计数断言，
 * 失败抛 Error 由 ARDroneSession.init 的 try/catch 吸收，绝不影响 AR 启动）。
 *
 * 2026-09-22 随 Blind 一起补：核心是**把「真安全」与「测距不可用」分开钉死**——
 *   同一条 obstacleDistance=Infinity，仅凭 depthTrustworthy 一个参数就应给出
 *   两种截然不同的展示（Safe/∞  vs  Blind/—）。
 *   真机现场取到的 2.53m 也一并入断言（那次是正常的 Safe）。
 *
 * 2026-09-25 随 suppressed 一起补：把「真安全」与「命中全被剔除」也分开钉死——
 *   同一条 Infinity + depthTrustworthy=true，仅凭 suppressed 一个参数就应从
 *   Safe/∞ 变成「前方不可判」/—。反向断言同样重要：suppressed=false 时
 *   必须仍是 Safe/∞，否则日常「前方 3m 内无遮挡」会被整体误报成不可判。
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

  // —— 核心：同一 ∞，三种语义（第六参数 suppressed 依次为 false/false/true）——
  const infTrusted = deriveHud(Infinity, 0, false, true, false, false);
  check('Infinity + 可信 → Safe', infTrusted.level === HudThreat.Safe);
  check('Infinity + 可信 → ∞', infTrusted.distanceText === '∞');

  const infBlind = deriveHud(Infinity, 0, false, false, false, false);
  check('Infinity + 不可信 → Blind', infBlind.level === HudThreat.Blind);
  check('Blind → label=测距不可用', infBlind.label === '测距不可用');
  check('Blind → 距离不报数(—)', infBlind.distanceText === '—');
  check('Blind → 不用安全绿', infBlind.colorHex !== HUD_COLOR_SAFE);

  // —— 抑制态：测到了，但命中全被剔除 ——
  const infSuppressed = deriveHud(Infinity, 0, false, true, false, true);
  check('Infinity + 可信 + 抑制 → 仍判 Safe（不得产生响铃）',
    infSuppressed.level === HudThreat.Safe);
  check('抑制 → label=前方不可判', infSuppressed.label === '前方不可判');
  check('抑制 → 不得显示安全', infSuppressed.label !== '安全');
  check('抑制 → 距离不报数(—)', infSuppressed.distanceText === '—');
  check('抑制 → 不用安全绿', infSuppressed.colorHex !== HUD_COLOR_SAFE);
  check('抑制 → suppressed 透传', infSuppressed.suppressed === true);
  // 反向：没有抑制时必须是老实的安全（否则日常畅通会被整体误报）
  check('无抑制 + Infinity → label=安全', infTrusted.label === '安全');
  check('无抑制 → suppressed=false', infTrusted.suppressed === false);
  // 抑制与代测同时为真 → 抑制优先（「判据把障碍丢了」比「量的地方不对」更接近事故）
  const bothFlags = deriveHud(Infinity, 0, false, true, true, true);
  check('抑制 + 代测 → label=前方不可判', bothFlags.label === '前方不可判');
  // 抑制不得盖掉真实读数：有有限距离时照旧报数（藏的才是第二重失真）
  const suppressedWithReading = deriveHud(0.8, 0.6, false, true, false, true);
  check('抑制但 0.8m 有限读数 → label=接近',
    suppressedWithReading.label === '接近' &&
    suppressedWithReading.distanceText === '0.80 m');
  // Blind / Danger 优先于抑制
  const supBlind = deriveHud(Infinity, 0, false, false, false, true);
  check('不可信 + 抑制标记 → 仍是 Blind', supBlind.level === HudThreat.Blind &&
    supBlind.label === '测距不可用');
  const supDanger = deriveHud(0.2, 1.0, false, true, false, true);
  check('0.2m + 抑制标记 → 仍是危险', supDanger.level === HudThreat.Danger &&
    supDanger.label === '危险');

  // 真机现场值：2.53m 是「测到但超出避障生效距离」，属正常 Safe
  const realDevice = deriveHud(2.53, 0, false, true, false, false);
  check('真机 2.53m → Safe', realDevice.level === HudThreat.Safe);

  // 盲 + 有限距离也必须是 Blind（例如降级前残留的旧值，不可当成安全）
  const blindWithStale = deriveHud(2.53, 0, false, false, false, false);
  check('不可信 + 2.53m → Blind（旧值不得冒充安全）', blindWithStale.level === HudThreat.Blind);

  // —— 相机代测：有读数，但量的不是无人机前方 ——
  // 采样器侧已把代测距离封顶到 ≥1.0m（只减速不硬停），所以最重只能到 Caution。
  const fbFar = deriveHud(2.0, 0.1, false, true, true, false);
  check('代测 2.0m → 等级仍按物理阈值(Safe)', fbFar.level === HudThreat.Safe);
  check('代测 2.0m → label=相机代测', fbFar.label === '相机代测');
  check('代测 → 不得显示安全', fbFar.label !== '安全');
  check('代测 → 不用安全绿', fbFar.colorHex !== HUD_COLOR_SAFE);
  check('代测 → 与盲态配色不同', fbFar.colorHex !== HUD_COLOR_BLIND);
  check('代测 → degraded 透传', fbFar.degraded === true);
  const fbNear = deriveHud(1.0, 0.5, false, true, true, false);
  check('代测封顶 1.0m → 最重只到 Caution', fbNear.level === HudThreat.Caution);
  check('代测 1.0m → label 仍为相机代测', fbNear.label === '相机代测');
  // 代测绝不许冒出「危险」：那是硬停面语义，只有无人机本体实测才配
  const fbDanger = deriveHud(0.2, 1.0, false, true, true, false);
  check('代测 0.2m → Danger 但 label 不被改成代测', fbDanger.level === HudThreat.Danger &&
    fbDanger.label === '危险');
  // 盲态优先于代测：什么都没测到时不能显示「相机代测」
  const fbBlind = deriveHud(Infinity, 0, false, false, true, false);
  check('不可信 + 代标记 → 仍是 Blind', fbBlind.level === HudThreat.Blind &&
    fbBlind.label === '测距不可用');

  // —— 急停优先于一切 ——
  const emergencyBlind = deriveHud(Infinity, 0, true, false, false, false);
  check('急停即使测距不可信 → Danger', emergencyBlind.level === HudThreat.Danger);
  const danger = deriveHud(0.3, 1.0, false, false, false, false);
  check('0.3m + 不可信 → Danger（实测硬停面优先）', danger.level === HudThreat.Danger);

  // —— 原有阈值口径（可信前提下不得回归）——
  const far = deriveHud(2.0, 0, false, true, false, false);
  check('2.0m→Safe', far.level === HudThreat.Safe);

  const caution = deriveHud(1.2, 0.3, false, true, false, false);
  check('1.2m→Caution', caution.level === HudThreat.Caution);

  const warning = deriveHud(0.8, 0.6, false, true, false, false);
  check('0.8m→Warning', warning.level === HudThreat.Warning);

  const boundary = deriveHud(1.6, 0.1, false, true, false, false);
  check('1.6m 边界→Safe', boundary.level === HudThreat.Safe);

  const emergency = deriveHud(1.0, 0.5, true, true, false, false);
  check('emergency=true 且 1.0m→Danger', emergency.level === HudThreat.Danger);

  check('非代测 → degraded=false', far.degraded === false);

  check('distanceText 格式(2.0m)', far.distanceText === '2.00 m');
  check('colorHex 与 Warning 对齐', warning.colorHex === HUD_COLOR_WARNING);
  check('label 与 Danger 对齐', danger.label === '危险');
  check('label 与 Caution 对齐', caution.label === '注意');
  // 严重度序必须保持单调递增（AlertService 直接吃枚举数值做边沿比较）
  check('等级序单调', HudThreat.Safe < HudThreat.Blind && HudThreat.Blind < HudThreat.Caution &&
    HudThreat.Caution < HudThreat.Warning && HudThreat.Warning < HudThreat.Danger);

  if (fails.length !== 0) {
    throw new Error(`HudModel 自检失败：${fails.join('、')}`);
  }
  Logger.info(`[VERIFY] HudModel 自检 PASS（${total} 项断言全通过）`);
}
