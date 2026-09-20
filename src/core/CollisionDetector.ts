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
import { Logger } from './Logger';

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
/** PRD §6.3.2 maxForce（m/s²） */
export const AVOID_MAX_FORCE: number = 1.0;
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
    z: clampRange(p.z, min.z, max.z),
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
      penetration: 0,
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
    penetration: radius - distance,
  };
}

/**
 * 纯函数自检（可在真机 / Node / CI 里直接验收，无需单测框架）。
 * ---------------------------------------------------------------
 * 本工程把断言写进代码里、在初始化时跑一次并打日志：
 * 只要能跑起来，就能用日志证明这些纯函数是对的。
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

  const msg: string = fails.length === 0 ?
    `PASS（${total} 项断言全通过）` : `FAIL：${fails.join('、')}`;
  Logger.info(`${TAG} [VERIFY] 碰撞层自检 ${msg}`);
  return msg;
}
