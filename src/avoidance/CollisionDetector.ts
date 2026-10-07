/**
 * 碰撞检测（Layer 5 · 物理层 · Phase 4）
 * ---------------------------------------------------------------
 * PRD §5.2 定义的 CollisionDetector：**球-AABB / 球-球碰撞**。
 *
 * 本模块与 AR 完全解耦 —— 不引用 AREngine、不碰 ArkGraphics3D、不读帧，
 * 输入是纯数字 / 纯结构体，输出是纯数据。因此它可以脱离设备直接单测
 * （PRD §7.2：Phase 4「与 AR 解耦，可纯函数单测」）。
 *
 * 距离口径（重要，避免重复扣减）：
 *   Phase 2 的 DepthSampler 给出的 obstacleDistance（gap）定义为
 *       gap = (障碍表面点 − 无人机中心) · 机头前向
 *   即「机体中心 → 障碍表面」沿前向的投影距离，已经是机体到障碍的距离，
 *   **不需要再减相机偏移**。
 *   本模块的硬停阈值 AVOID_MARGIN_M=0.4 也已包含机身安全余量
 *   （机体包围球半径约 0.19m；实测刹停 gap≈0.38m，见 Phase 2 真机验收报告）。
 *   因此 evaluate() 里 clearance 只用于**诊断展示**，不参与是否刹停的判定 ——
 *   若用它去扣阈值，等于把余量算两遍，会让无人机提前 0.19m 停下。
 */
import { Logger } from '../core/Logger';

const TAG: string = 'CollisionDetector';

/** 物理层用的最小向量（刻意不复用 ArkGraphics3D 的 Vec3，保持本层零渲染依赖） */
export interface PhysicsVec3 {
  x: number;
  y: number;
  z: number;
}

// ---------------------------------------------------------------
//  避障阈值 —— 单一事实来源
// ---------------------------------------------------------------
// 原先这两个常量定义在 DroneController 里（Phase 2 为了「面板显示的危险等级」
// 与「实际刹停行为」用同一组数）。Phase 4 把它们下沉到物理层：
// 它们是**避障物理参数**，理应由物理层拥有；DroneController 与 DepthSampler
// 都改为从这里引用，避免同一组阈值散落三处。
/** 硬停面（距障碍多近必须把前进分量清零）—— 实测刹停 gap≈0.38m */
export const AVOID_MARGIN_M: number = 0.4;
/** 减速带宽度（从硬停面向外多远开始平滑收敛前进分量） */
export const AVOID_SLOW_BAND_M: number = 1.2;
/**
 * 避障开始生效的距离 = 硬停面 + 减速带 = 1.6m。
 * 对应 PRD §6.3.2 的 safeDist（PRD 字面 0.5m 是按 maxSpeed=0.4m/s 定的；
 * 本 App 实际 maxSpeed=1.5m/s，制动距离需同比放大，故取 1.6m —— 偏差已记录）。
 */
export const AVOID_SAFE_DIST_M: number = AVOID_MARGIN_M + AVOID_SLOW_BAND_M;
/** 无人机包围球半径（GLB 整机含桨约 0.37m） */
export const DRONE_RADIUS_M: number = 0.19;
/**
 * 前向碰撞走廊半宽（m）：只有「命中点到机体航向轴线的垂直距离」落在此半径内，
 * 才视为**前方障碍**并参与刹停。
 * ---------------------------------------------------------------
 * 为什么必须有它：深度探针的净距是按**前向投影**算的
 *     gap = (命中点 − 探针原点) · 机头前向
 * 而投影**对侧向偏移完全不敏感** —— 一堵在机体侧方 2m、只是略微偏前的墙，
 * 其前向投影同样可能只有 0.3m，于是被判成「正前方 0.3m 有障碍」直接硬停，
 * 可机体根本不会撞上它（要撞上得先横移过去）。
 * 垂直距离是**旋转不变量**（与坐标系无关），因此它同时覆盖两种"擦身而过"：
 *   · 侧方掠过的墙面       → 前进撞不到
 *   · 机体下方的地板/桌面  → 可直接飞越
 * 取 机体半径 + 0.25m 余量：既接纳真实航路上的障碍（含轻微擦碰），
 * 又排除只是擦身而过的表面。真机 ADY-AL10 表现为「推杆不动 / 避障乱刹停」。
 */
export const AVOID_CORRIDOR_HALF_M: number = DRONE_RADIUS_M + 0.25;
/**
 * 「低于飞行高度」（m）：命中点比探针原点低超过此值 → 视为可**飞越**的下方表面
 * （地板/桌面/低矮物），不计入前方障碍。
 * 依据：机体是半径 DRONE_RADIUS_M 的包围球，比球底还低的表面前进必然不会撞上。
 * 单独设一条而不是只靠走廊，是因为"正下方略偏前"的命中点横向偏移≈竖向偏移，
 * 单靠走廊半宽可能刚好放行。
 */
export const AVOID_BELOW_PATH_M: number = DRONE_RADIUS_M + 0.10;
/** PRD §6.3.2 maxForce（m/s²） */
export const AVOID_MAX_FORCE: number = 1.0;

/**
 * 避障制动减速度（m/s²）—— 2026-10-05 新增，运动学自洽性用。
 * ---------------------------------------------------------------
 * 取值依据：本工程是**速度包线控制**（DroneController 直接给目标速度，
 * 位置环 PD 把它拉过去），实测从 1.5m/s 收敛到 0 的减速度约 1.5m/s²。
 * 取得略保守（1.5），宁可阈值留大一点，也不要算出来的余量不够刹住。
 */
export const AVOID_BRAKE_ACCEL: number = 1.5;
/**
 * 硬停面的**静态基线**（m）：速度为 0 时用它。
 * 0.25 而非原来的 0.4：机体半径 0.19m + 0.06m 余量即可，
 * 刹车距离那部分交给速度相关项按需追加（见 avoidMarginFor）。
 */
export const AVOID_MARGIN_BASE_M: number = 0.25;

/**
 * 速度相关的硬停面（m，纯函数，2026-10-05）。
 * ---------------------------------------------------------------
 * 解决的问题：原实现 `AVOID_MARGIN_M=0.4` 是**定值**，与速度脱钩。
 * 于是运动学上不自洽 ——
 *     刹车距离 s = v² / (2a) = 1.5² / (2×1.5) = **0.75m**  >  硬停面 0.4m
 * 也就是说：以最大速度冲向障碍时，**等开始减速已经撞上了**。
 * 减速带（1.2m）勉强够用，但只要叠加一层滤波延迟或一次读数偏大，
 * 就会真撞 —— 这正是「明明减速了还是碰到」的原因。
 *
 * 修法：硬停面按当前速度的**刹车距离**动态给出
 *     margin(v) = AVOID_MARGIN_BASE_M + v² / (2 × AVOID_BRAKE_ACCEL)
 * 速度为 0 → 0.25m（低速时不多占用视野）；
 * 1.5m/s → 0.25 + 0.75 = **1.0m**（恰好等于刹车距离，绝不撞）。
 *
 * 关键取舍：高速时提前刹停更多，看起来"反应变迟钝"，
 * 但那本来就是物理要求 —— 1.5m/s 覆盖 1m 只需 0.67s，
 * 而 3ms 一帧的采样 + 3 帧中值滤波已经吃掉 0.1s，仍有余量。
 *
 * @param speed 当前速度大小（m/s，非负；非法值按 0 处理）
 */
export function avoidMarginFor(speed: number): number {
  const v: number = (isFinite(speed) && speed > 0) ? speed : 0;
  return AVOID_MARGIN_BASE_M + (v * v) / (2 * AVOID_BRAKE_ACCEL);
}

/**
 * 速度相关的避障生效距离（m，纯函数）。
 * = 硬停面 + 减速带。随速度一起变，保证「减速带宽度」这个手感常量不变，
 * 而整体预警距离随刹车需求自动伸缩。
 */
export function avoidSafeDistFor(speed: number, band: number = AVOID_SLOW_BAND_M): number {
  return avoidMarginFor(speed) + (band > 0 ? band : AVOID_SLOW_BAND_M);
}

/**
 * 横向绕行增益：越接近障碍，越强地放大横移分量，形成「贴墙滑行 / 绕行」。
 * 1.0 = 不放大（纯刹停）；1.25 = 全阻塞时横移放大 25%。
 * 注意：只在用户**已经有横移输入**时起作用 —— 直推墙壁且无横移输入时
 * 不会凭空产生侧向速度（自动绕行需 Phase 7/8 的走廊绕行策略）。
 */
export const AVOID_STEER_GAIN: number = 1.25;

/** 威胁等级（与 Phase 2 DepthSample.threatLevel 语义一致） */
export type ThreatLevel = 'safe' | 'slow' | 'stop';

/** 碰撞评估结果 */
export interface CollisionInfo {
  /** 原始前向净距（m） */
  distance: number;
  /** 机体表面到障碍的余量 = distance − radius（仅诊断用，不参与刹停判定） */
  clearance: number;
  /** 威胁等级 */
  threatLevel: ThreatLevel;
  /** 是否进入急停（distance ≤ 硬停面） */
  emergency: boolean;
  /** 避障强度 0..1（PRD §6.3.2：随距离减小而增大；1 = 完全阻塞） */
  avoidStrength: number;
  /** 侵入深度（>0 表示机体已嵌入障碍，正常刹停下应恒 ≤ 0） */
  penetration: number;
}

/** 把 v 限幅到 [0,1] */
export function clamp01(v: number): number {
  if (v < 0) {
    return 0;
  }
  if (v > 1) {
    return 1;
  }
  return v;
}

/** 限幅到 [lo,hi] */
export function clampRange(v: number, lo: number, hi: number): number {
  if (v < lo) {
    return lo;
  }
  if (v > hi) {
    return hi;
  }
  return v;
}

/** 归一化；零向量返回 {0,0,0}（避免除零产生 NaN 污染整条物理链） */
export function normalize(v: PhysicsVec3): PhysicsVec3 {
  const len: number = Math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
  if (len < 1e-9) {
    return { x: 0, y: 0, z: 0 };
  }
  return { x: v.x / len, y: v.y / len, z: v.z / len };
}

/** 球-球相交（PRD §5.2「球-球碰撞」） */
export function sphereSphereHit(a: PhysicsVec3, ra: number, b: PhysicsVec3, rb: number): boolean {
  const dx: number = a.x - b.x;
  const dy: number = a.y - b.y;
  const dz: number = a.z - b.z;
  const r: number = ra + rb;
  return dx * dx + dy * dy + dz * dz <= r * r;
}

/** 取点到 AABB 的最近点（球-AABB 的基础） */
export function closestPointOnAabb(p: PhysicsVec3, min: PhysicsVec3, max: PhysicsVec3): PhysicsVec3 {
  return {
    x: clampRange(p.x, min.x, max.x),
    y: clampRange(p.y, min.y, max.y),
    z: clampRange(p.z, min.z, max.z)
  };
}

/** 球-AABB 相交（PRD §5.2「球-AABB 碰撞」） */
export function sphereAabbHit(center: PhysicsVec3, radius: number,
  min: PhysicsVec3, max: PhysicsVec3): boolean {
  const q: PhysicsVec3 = closestPointOnAabb(center, min, max);
  const dx: number = center.x - q.x;
  const dy: number = center.y - q.y;
  const dz: number = center.z - q.z;
  return dx * dx + dy * dy + dz * dz <= radius * radius;
}

/**
 * 距离是否有效（可用于避障判定）。
 * Infinity / NaN / 负数都不是「安全」，而是「这一帧防撞是盲的」——
 * 必须显式区分，否则"不刹停"无法判定原因（Phase 2 覆盖状态的设计意图）。
 */
export function isValidDistance(d: number): boolean {
  return isFinite(d) && d > 0;
}

/**
 * 评估「无人机 ↔ 前方障碍」的碰撞态势。
 *
 * 避障强度与前进分量缩放置信同源：
 *   fwdScale     = (d − margin) / band           （沿用 Phase 2 已真机验证的减速曲线）
 *   avoidStrength = 1 − fwdScale
 * 二者都由同一组阈值推导，因此面板显示的强度与实际刹车力度永远一致。
 *
 * @param distance 前向净距（m）
 * @param radius   机体包围球半径，默认 DRONE_RADIUS_M
 * @param margin   硬停面（m），默认 AVOID_MARGIN_M
 * @param band     减速带宽度（m），默认 AVOID_SLOW_BAND_M
 * @param safeDist 避障生效距离（m），默认 AVOID_SAFE_DIST_M
 *
 * 后三个阈值做成**可选入参**而不是写死常量，是为了让 SteeringParams 里
 * 的 band / emergencyDist / safeDist 真正生效 —— 否则调用方改了参数却
 * 看不到任何行为变化（"声明了却被忽略的参数"是最难查的一类问题）。
 * 不传时与模块常量完全一致，因此既有调用点零回归。
 */
export function evaluate(distance: number, radius: number = DRONE_RADIUS_M,
  margin: number = AVOID_MARGIN_M, band: number = AVOID_SLOW_BAND_M,
  safeDist: number = AVOID_SAFE_DIST_M): CollisionInfo {
  if (!isValidDistance(distance)) {
    // 测距不可用：不是"安全"，而是"盲"。返回 safe 但 distance 保持原值，
    // 由调用方结合 isValidDistance() 决定是否启用避障（与 Phase 2 降级语义一致）。
    return {
      distance: distance,
      clearance: distance - radius,
      threatLevel: 'safe',
      emergency: false,
      avoidStrength: 0,
      penetration: 0
    };
  }
  const emergency: boolean = distance <= margin;
  // 前进分量缩放：硬停面内为 0，减速带内线性收敛，safeDist 外为 1
  const fwdScale: number = clamp01((distance - margin) / (band > 0 ? band : 1e-6));
  const avoidStrength: number = 1 - fwdScale;
  let level: ThreatLevel = 'safe';
  if (emergency) {
    level = 'stop';
  } else if (distance < safeDist) {
    level = 'slow';
  }
  return {
    distance: distance,
    clearance: distance - radius,
    threatLevel: level,
    emergency: emergency,
    avoidStrength: avoidStrength,
    penetration: radius - distance
  };
}

/**
 * 上方净距（m）：深度采样器测到的「机体正上方最近的实体」沿**竖直向上**的净距。
 * Infinity = 上方无实体（天空 / 超出量程 / 测不到）。
 *
 * 为什么需要它（P1-1，2026-10-05）：
 * 整套避障链从 Phase 2 起就只算**前向水平**净距 —— 9 条锥形射线全部落在
 * CONE_HALF_DEG=15° 的水平锥内，垂直方向的分量被 `normalize3` 归一化掉。
 * 后果是**爬升完全不参与避障**：
 *   · 抬头撞上门框 / 天花板 / 树枝 → 深度射线打不到，obstacleDistance 恒 ∞，
 *     HUD 显绿「安全」，飞机照撞；
 *   · 顶着天花板横移 → 螺旋桨打墙，机身翻滚。
 * 二者都不是「避障弱」，而是「这一维根本没有避障」。
 *
 * 为什么不复用前向那套射线：15° 锥是围绕**机头前向**张开的世界系射线，
 * 要覆盖正上方就得额外加一组垂直向上的探针，那是采样器侧的事（本常量
 * 只是在物理层声明「上方也算一个需要避让的自由度」）。
 */
export const CEILING_MARGIN_M: number = 0.35;
/**
 * 上方减速带宽度（m）。比水平方向**窄**（水平 1.2m），刻意为之：
 * 竖直方向留给用户的反应空间更小（推杆是连续量，脚下没有"绕"的余地），
 * 且向上减速必须比向前更早介入 —— 否则用户会先撞上再看到提示。
 */
export const CEILING_SLOW_BAND_M: number = 0.55;
/** 上方避障生效距离 = 上方硬停面 + 上方减速带 = 0.9m */
export const CEILING_SAFE_DIST_M: number = CEILING_MARGIN_M + CEILING_SLOW_BAND_M;

/** 上方净距的评估结果（与 CollisionInfo 同构，便于上层统一消费） */
export interface CeilingInfo {
  /** 上方净距（m）；Infinity = 上方无实体或测不到 */
  distance: number;
  /** 是否进入急停（上方净距 ≤ 上方硬停面） */
  emergency: boolean;
  /** 上方避障强度 0..1（1 = 完全阻塞上升） */
  avoidStrength: number;
  /** 威胁等级（与水平同一套语义，便于 HUD/告警复用） */
  threatLevel: ThreatLevel;
}

/**
 * 评估「无人机 ↔ 正上方实体」的态势。
 * ---------------------------------------------------------------
 * 与 evaluate() **同源同构**：同样的 clamp01 收敛、同样的三档威胁分级，
 * 差别只在阈值组（上方更保守，见上方常量注释）。
 * 保持同构的意义在于：上层一旦接入，HUD/告警/自检全都能复用现有那条链路，
 * 不会出现「水平避障有强度条、垂直避障只有一个布尔」的割裂。
 *
 * 关键取舍：**只拦上升，不强推下降**。上方有障碍时把向上分量清零即可，
 * 由飞控的稳定层自然把飞机悬停；主动往下压是危险的——用户可能正下方
 * 才是安全的（下有平台），强推下降会把它送到地面上去。
 */
export function evaluateCeiling(distance: number,
  margin: number = CEILING_MARGIN_M, band: number = CEILING_SLOW_BAND_M,
  safeDist: number = CEILING_SAFE_DIST_M): CeilingInfo {
  if (!isValidDistance(distance)) {
    // 上方读不到（天空 / 超量程 / 盲）：不是「可以放心爬升」，而是「这一维是盲的」。
    // 与水平链路同策略：强度 0（不干预），由 coverage 类状态另行呈现。
    return { distance: distance, emergency: false, avoidStrength: 0, threatLevel: 'safe' };
  }
  const emergency: boolean = distance <= margin;
  const scale: number = clamp01((distance - margin) / (band > 0 ? band : 1e-6));
  const avoidStrength: number = 1 - scale;
  let level: ThreatLevel = 'safe';
  if (emergency) {
    level = 'stop';
  } else if (distance < safeDist) {
    level = 'slow';
  }
  return { distance: distance, emergency: emergency, avoidStrength: avoidStrength, threatLevel: level };
}

/**
 * 感知失联兜底档位（P1-3，2026-10-05）。
 * ---------------------------------------------------------------
 * 「深度读不到」在物理层是一个**无量**，没有距离就没有避障输入。
 * 此前链路对此的处理是「obstacleDistance=∞，不刹车，飞机照常飞」——
 * 这在**短暂**失联（低头、转身、纹理不足）下是对的，但在**持续**失联下
 * 变成事故：用户看着 HUD 的绿「安全」，实际已经是盲飞。
 *
 * 所以要按**失联时长**分档，而不是一律「不干预」或一律「立即降落」
 * （后者会导致每次掏手机看地图都触发降落）。分档依据是「还能飞多久」：
 *   Normal   刚失联 —— 绝大多数是瞬态，无脑干预反而制造误降落
 *   Cautious 持续失联 —— 提示用户，且**主动限制爬升**（盲飞最怕撞天花板）
 *   Hold     严重失联 —— 只允许水平悬停，等用户把视角转回来
 */
export type SensorTrust = 'normal' | 'cautious' | 'hold';

/** 短暂失联容忍时长（秒）：低于此值视为瞬态，不升级档位 */
const SENSOR_CAUTIOUS_AFTER_SEC: number = 3;
/** 严重失联阈值（秒）：超过此值进入 Hold（只许水平悬停） */
const SENSOR_HOLD_AFTER_SEC: number = 12;

/**
 * 按「本机支持深度」与「连续失联时长」给出当前应采取的兜底档位（纯函数）。
 * ---------------------------------------------------------------
 * @param supported  本机是否支持深度（AREngine 明确说不支持 → 直接 Hold，
 *                   不必等：永远不会自己恢复，等下去只是白飞）
 * @param blindSec   已连续失联的时长（秒）
 */
export function sensorTrust(supported: boolean, blindSec: number): SensorTrust {
  if (!supported) {
    return 'hold';
  }
  if (!isFinite(blindSec) || blindSec < 0) {
    return 'normal';
  }
  if (blindSec >= SENSOR_HOLD_AFTER_SEC) {
    return 'hold';
  }
  if (blindSec >= SENSOR_CAUTIOUS_AFTER_SEC) {
    return 'cautious';
  }
  return 'normal';
}

/** 兜底档位对应的爬升许可系数：只压上升，绝不强推下降（同 evaluateCeiling 的取舍） */
export function sensorClimbScale(trust: SensorTrust): number {
  if (trust === 'hold') {
    return 0;
  }
  if (trust === 'cautious') {
    return 0.4;
  }
  return 1;
}

/**
 * 纯函数自检（可在真机 hilog 里直接验收，无需单测框架）。
 * ---------------------------------------------------------------
 * 本工程没有 ohosTest 测试模块，而本层是「可纯函数单测」的核心，
 * 因此把断言写进代码里、在 AR 初始化时跑一次并打日志：
 * 真机/模拟器只要能跑起来，就能用 grep 证明这些纯函数是对的。
 */
export function selfTest(): string {
  const fails: string[] = [];
  // 用计数器而不是写死数字：断言增删时不会打印出错误的"共 N 项"
  let total: number = 0;
  const check = (name: string, ok: boolean): void => {
    total++;
    if (!ok) {
      fails.push(name);
    }
  };

  // —— 纯几何 ——
  check('球-球相交', sphereSphereHit({ x: 0, y: 0, z: 0 }, 1, { x: 1.5, y: 0, z: 0 }, 1) === true);
  check('球-球分离', sphereSphereHit({ x: 0, y: 0, z: 0 }, 1, { x: 3, y: 0, z: 0 }, 1) === false);
  check('球-AABB相交',
    sphereAabbHit({ x: 0, y: 0, z: 0 }, 1, { x: 0.5, y: -1, z: -1 }, { x: 2, y: 1, z: 1 }) === true);
  check('球-AABB分离',
    sphereAabbHit({ x: 0, y: 0, z: 0 }, 1, { x: 5, y: -1, z: -1 }, { x: 6, y: 1, z: 1 }) === false);
  check('最近点限幅', ((): boolean => {
    const q: PhysicsVec3 = closestPointOnAabb({ x: 9, y: 0, z: -9 },
      { x: -1, y: -1, z: -1 }, { x: 1, y: 1, z: 1 });
    return q.x === 1 && q.y === 0 && q.z === -1;
  })());

  // —— 阈值 ——
  check('阈值关系', AVOID_SAFE_DIST_M === AVOID_MARGIN_M + AVOID_SLOW_BAND_M);

  // —— evaluate ——
  const far: CollisionInfo = evaluate(2.5);
  check('远处=安全', far.threatLevel === 'safe' && !far.emergency && far.avoidStrength === 0);
  const near: CollisionInfo = evaluate(0.4);
  check('硬停面=stop', near.threatLevel === 'stop' && near.emergency && near.avoidStrength === 1);
  const mid: CollisionInfo = evaluate(1.0);
  check('减速带内=slow', mid.threatLevel === 'slow' && !mid.emergency);
  check('减速带单调性', mid.avoidStrength < near.avoidStrength && mid.avoidStrength > far.avoidStrength);
  const invalid: CollisionInfo = evaluate(Infinity);
  check('测距不可用不误判为危险', !invalid.emergency && invalid.avoidStrength === 0);
  check('余量=距离-半径', Math.abs(evaluate(1.0, 0.2).clearance - 0.8) < 1e-9);

  // —— 自定义阈值必须真的生效（否则调用方改参数却看不到变化）——
  // gap=1.0 在默认减速带内（avoidStrength=0.5）；把 band 收窄到 0.5 后同一距离应视为安全
  check('默认减速带内=0.5', Math.abs(evaluate(1.0).avoidStrength - 0.5) < 1e-9);
  check('自定义阈值生效', evaluate(1.0, DRONE_RADIUS_M, 0.4, 0.5, 0.9).avoidStrength === 0);
  check('自定义硬停面生效', evaluate(0.5, DRONE_RADIUS_M, 0.6, 1.2, 1.8).emergency === true);

  // —— 上方避障（P1-1，2026-10-05）——
  // 不变量：与水平链路同构（同样的三档分级、同样的 0/0.5/1 强度端点），
  // 但阈值更保守（上方 0.9m 生效 vs 水平 1.6m）—— 竖直方向留给用户的余地更小。
  check('上方阈值关系', CEILING_SAFE_DIST_M === CEILING_MARGIN_M + CEILING_SLOW_BAND_M);
  check('上方生效距离比水平更近（竖直方向更保守）',
    CEILING_SAFE_DIST_M < AVOID_SAFE_DIST_M);
  const cFar: CeilingInfo = evaluateCeiling(2.0);
  check('上方远处=安全', cFar.threatLevel === 'safe' && !cFar.emergency && cFar.avoidStrength === 0);
  const cNear: CeilingInfo = evaluateCeiling(CEILING_MARGIN_M);
  check('上方硬停面=stop', cNear.threatLevel === 'stop' && cNear.emergency && cNear.avoidStrength === 1);
  const cMid: CeilingInfo = evaluateCeiling(0.6);
  check('上方减速带内=slow', cMid.threatLevel === 'slow' && !cMid.emergency);
  check('上方强度单调', cMid.avoidStrength < cNear.avoidStrength && cMid.avoidStrength > cFar.avoidStrength);
  // 与水平同构：减速带正中（中值）强度恒为 0.5，两条链路口径一致
  check('上方减速带正中=0.5（与水平同构）',
    Math.abs(evaluateCeiling(CEILING_MARGIN_M + CEILING_SLOW_BAND_M / 2).avoidStrength - 0.5) < 1e-9);
  // 盲态不误判为「可以放心爬升」：强度 0（不干预），但绝不能判成 emergency
  const cBlind: CeilingInfo = evaluateCeiling(Infinity);
  check('上方读不到不误判为危险', !cBlind.emergency && cBlind.avoidStrength === 0);
  // 自定义阈值必须真的生效（否则调参者改了却看不到变化）：
  // margin=0.35 / band=0.2 → fwdScale=(0.5-0.35)/0.2=0.75 → avoidStrength=0.25
  check('上方自定义阈值生效',
    Math.abs(evaluateCeiling(0.5, CEILING_MARGIN_M, 0.2, 0.6).avoidStrength - 0.25) < 1e-9);
  check('上方自定义安全距离生效（收窄到 0.3 后 0.5m 视为安全）',
    evaluateCeiling(0.5, CEILING_MARGIN_M, CEILING_SLOW_BAND_M, 0.3).threatLevel === 'safe');

  // —— 感知失联兜底（P1-3）——
  // 核心不变量：**短暂失联不得升级**。低头 / 转身 / 低纹理都是瞬态，
  // 一升级就立即降落，用户会频繁遇到「掏手机看地图飞机自己降落」。
  check('刚失联=normal（瞬态不升级）', sensorTrust(true, 0) === 'normal');
  check('1.9s 仍 normal', sensorTrust(true, 1.9) === 'normal');
  check('3s 起 cautious', sensorTrust(true, 3.0) === 'cautious');
  check('11.9s 仍 cautious', sensorTrust(true, 11.9) === 'cautious');
  check('12s 起 hold', sensorTrust(true, 12.0) === 'hold');
  check('超时后保持 hold', sensorTrust(true, 999) === 'hold');
  // 本机不支持深度 → 立即 hold，不等（等下去只是白飞，它永远不会自己恢复）
  check('本机不支持深度 → 立即 hold', sensorTrust(false, 0) === 'hold');
  // 非法输入按「没失联」处理，绝不因为一个 NaN 就把飞机锁死
  check('非法时长按 normal 处理（不得因 NaN 锁死）',
    sensorTrust(true, NaN) === 'normal' && sensorTrust(true, -1) === 'normal');
  // 爬升许可必须单调递减，且 hold 档为 0
  check('爬升许可：normal 全量', sensorClimbScale('normal') === 1);
  check('爬升许可：cautious 削弱', sensorClimbScale('cautious') > 0 &&
    sensorClimbScale('cautious') < 1);
  check('爬升许可：hold 完全禁止', sensorClimbScale('hold') === 0);
  check('爬升许可单调不增',
    sensorClimbScale('hold') <= sensorClimbScale('cautious') &&
    sensorClimbScale('cautious') <= sensorClimbScale('normal'));

  // —— 速度相关阈值（P0-1 运动学自洽，2026-10-05）——
  // 核心不变量：**硬停面必须 ≥ 当前速度的刹车距离**，否则减速来不及必撞。
  // 这条是本轮修掉"明明减速了还是碰到"的根据，用真实数字钉死。
  check('静止时硬停面=基线', Math.abs(avoidMarginFor(0) - AVOID_MARGIN_BASE_M) < 1e-9);
  check('刹车距离自洽（1.5m/s → 0.25+0.75=1.0m）',
    Math.abs(avoidMarginFor(1.5) - 1.0) < 1e-9);
  check('硬停面随速度单调不增（高速留更多余量）',
    avoidMarginFor(0) < avoidMarginFor(0.5) &&
    avoidMarginFor(0.5) < avoidMarginFor(1.0) &&
    avoidMarginFor(1.0) < avoidMarginFor(1.5));
  // 关键：任意速度下，margin ≥ v²/(2a)，即「刹车距离永远被覆盖」。
  for (const v of [0.3, 0.8, 1.5, 2.2, 3.0]) {
    const brakeDist: number = (v * v) / (2 * AVOID_BRAKE_ACCEL);
    check(`margin 覆盖刹车距离 v=${v}`, avoidMarginFor(v) >= brakeDist - 1e-9);
  }
  // 非法输入不得产生 NaN / 负余量（否则一次脏读数就把飞机锁死）
  check('非法速度按 0 处理（不产生 NaN）',
    avoidMarginFor(NaN) === AVOID_MARGIN_BASE_M &&
    avoidMarginFor(-1) === AVOID_MARGIN_BASE_M &&
    avoidMarginFor(Infinity) === AVOID_MARGIN_BASE_M);
  // safeDist = margin + band：减速带宽度这个手感常量在任何速度下都保持不变
  check('safeDist 恒等于 margin+band（减速带宽度不随速度变）',
    Math.abs(avoidSafeDistFor(1.5) - (avoidMarginFor(1.5) + AVOID_SLOW_BAND_M)) < 1e-9);
  check('safeDist 随速度单调递增', avoidSafeDistFor(0) < avoidSafeDistFor(1.5));
  check('safeDist 非法 band 回退默认（不产生 NaN）',
    Math.abs(avoidSafeDistFor(1.0, 0) - (avoidMarginFor(1.0) + AVOID_SLOW_BAND_M)) < 1e-9);
  // 阈值随速提前介入：同一距离 1.5m，静止时是安全的（1.5 > safeDist(0)=1.45），
  // 满速时已落进减速带（1.5 < safeDist(1.5)=2.2）。
  // ⚠️ 2026-10-06 修：旧断言拿 1.0m 当样本，但满速时 margin(1.5)=1.0，
  // 1.0m 恰在硬停面上（真实档位 stop 而非 slow）；静止时 1.0 < 1.45 也本就是
  // slow 而非 safe——两个前提都与公式自反。改用 1.5m 这个在两速度下档位
  // 真正翻转的距离，并把 1.0m 的边界语义单独钉成一条 stop 断言。
  check('满速时 1.5m 已属减速带（静止时它是安全的）',
    evaluate(1.5, DRONE_RADIUS_M, avoidMarginFor(1.5), AVOID_SLOW_BAND_M,
      avoidSafeDistFor(1.5)).threatLevel === 'slow');
  check('静止时 1.5m 仍是安全的（低速不多占用视野）',
    evaluate(1.5, DRONE_RADIUS_M, avoidMarginFor(0), AVOID_SLOW_BAND_M,
      avoidSafeDistFor(0)).threatLevel === 'safe');
  // 边界钉死：满速时 1.0m 恰在硬停面上（margin(1.5)=1.0）→ 必须是 stop
  check('满速时 1.0m 恰在硬停面上 → stop',
    evaluate(1.0, DRONE_RADIUS_M, avoidMarginFor(1.5), AVOID_SLOW_BAND_M,
      avoidSafeDistFor(1.5)).threatLevel === 'stop');

  const msg: string = fails.length === 0 ?
    `PASS（${total} 项断言全通过）` : `FAIL：${fails.join('、')}`;
  Logger.info(`${TAG} [VERIFY] 碰撞层自检 ${msg}`);
  return msg;
}
