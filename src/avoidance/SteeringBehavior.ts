/**
 * Steering Behavior（Layer 5 · 物理层 · Phase 4）
 * ---------------------------------------------------------------
 * PRD §5.2 定义的 SteeringBehavior：**Seek + Avoid 行为合成**；
 * 算法取自 PRD §5.4 / §8.1 指定的 Craig Reynolds Steering。
 *
 * 与 AR 完全解耦：本模块只认数字，不认 AREngine / 帧 / 3D 节点。
 *
 * 两套 API 的分工（重要，避免误用）：
 *   1) seekForce / avoidForce / combineForces —— PRD §6.3.2 的**力模型**，
 *      逐字实现文档公式，纯函数、可单测，供 Phase 7 的告警/可视化与单测消费。
 *   2) applyAvoidance —— **速度域集成**，本 App 实际使用的路径。
 *
 * 为什么实际集成走速度域而不是直接加力：
 *   本 App 的 DroneController 是「目标速度 + 有限加速度逼近」模型（Phase 2 已真机验证
 *   手感与刹停距离），而 PRD 公式假定的是「力 → 加速度 → 速度」的质点模型。
 *   直接改成加力会推翻已验证的飞行手感与 gap≈0.38m 的刹停表现。
 *   因此这里保留速度域集成，并让它与力模型**共用同一组阈值**
 *   （avoidStrength 同时驱动两者），保证面板读数与实际刹车力度永远一致。
 *
 * 法向约定（与 PRD 的差异）：
 *   PRD 写 avoidForce = -obstacleNormal * ...，那里的 normal 指向障碍内部。
 *   本工程 DepthSample.normal 是「由障碍表面指向相机」（即指向无人机、远离障碍），
 *   所以本模块的入参直接叫 away（远离障碍的单位向量），不再取负号 ——
 *   避免把符号约定埋在调用方里造成反向加速。
 */
import {
  AVOID_MARGIN_M,
  AVOID_MAX_FORCE,
  AVOID_SAFE_DIST_M,
  AVOID_SLOW_BAND_M,
  AVOID_STEER_GAIN,
  DRONE_RADIUS_M,
  CollisionInfo,
  PhysicsVec3,
  ThreatLevel,
  clamp01,
  evaluate,
  isValidDistance,
  normalize,
} from './CollisionDetector';
import { Logger } from '../core/Logger';

const TAG: string = 'SteeringBehavior';

/** 转向/避障参数（可整体替换，便于调参与单测） */
export interface SteeringParams {
  /** 避障开始生效的距离（m） */
  safeDist: number;
  /** 急停硬停面（m） */
  emergencyDist: number;
  /** 减速带宽度（m） */
  band: number;
  /** 最大推力（m/s²，PRD maxForce） */
  maxForce: number;
  /** 最大速度（m/s，用于结果限幅） */
  maxSpeed: number;
  /** 横向绕行增益（1.0 = 不放大） */
  steerGain: number;
}

/** 默认参数（阈值取自物理层单一事实来源） */
export const DEFAULT_STEERING: SteeringParams = {
  safeDist: AVOID_SAFE_DIST_M,
  emergencyDist: AVOID_MARGIN_M,
  band: AVOID_SLOW_BAND_M,
  maxForce: AVOID_MAX_FORCE,
  maxSpeed: 1.5,
  steerGain: AVOID_STEER_GAIN,
};

/**
 * PRD §6.3.2：seekForce = normalize(desiredVel − currentVel) * maxForce
 * 物理含义：把当前速度拉向期望速度所需的力，方向是两者的差。
 */
export function seekForce(desired: PhysicsVec3, current: PhysicsVec3,
  maxForce: number): PhysicsVec3 {
  const diff: PhysicsVec3 = {
    x: desired.x - current.x,
    y: desired.y - current.y,
    z: desired.z - current.z,
  };
  const dir: PhysicsVec3 = normalize(diff);
  return { x: dir.x * maxForce, y: dir.y * maxForce, z: dir.z * maxForce };
}

/**
 * PRD §6.3.2：avoidForce = away * (1 − distance/safeDist) * maxForce
 * 距离越近力越大；distance ≥ safeDist 时为 0（不产生吸引，clamp 到 0）。
 * @param away 远离障碍的单位向量（本工程 normal 的朝向，见文件头说明）
 */
export function avoidForce(distance: number, safeDist: number, away: PhysicsVec3,
  maxForce: number): PhysicsVec3 {
  if (!isValidDistance(distance) || safeDist <= 0) {
    return { x: 0, y: 0, z: 0 };
  }
  const strength: number = clamp01(1 - distance / safeDist);
  const dir: PhysicsVec3 = normalize(away);
  return {
    x: dir.x * strength * maxForce,
    y: dir.y * strength * maxForce,
    z: dir.z * strength * maxForce,
  };
}

/**
 * PRD §6.3.2：totalForce = clamp(seekForce + avoidForce, maxForce)
 * 按**矢量长度**限幅（不是逐分量），保证合力方向不被扭曲。
 */
export function combineForces(seek: PhysicsVec3, avoid: PhysicsVec3,
  maxForce: number): PhysicsVec3 {
  const sx: number = seek.x + avoid.x;
  const sy: number = seek.y + avoid.y;
  const sz: number = seek.z + avoid.z;
  const len: number = Math.sqrt(sx * sx + sy * sy + sz * sz);
  if (len <= maxForce || len < 1e-9) {
    return { x: sx, y: sy, z: sz };
  }
  const k: number = maxForce / len;
  return { x: sx * k, y: sy * k, z: sz * k };
}

/** 避障后的目标速度结果 */
export interface AvoidResult {
  /** 处理后的目标速度 X 分量（m/s，AR 世界系） */
  vx: number;
  /** 处理后的目标速度 Z 分量 */
  vz: number;
  /** 是否正在因避障限制速度 */
  braking: boolean;
  /** 是否进入急停（调用方应据此刹掉当前朝障碍的速度） */
  emergency: boolean;
  /** 避障强度 0..1 */
  avoidStrength: number;
  /** 威胁等级 */
  threatLevel: ThreatLevel;
  /** 是否触发了横向绕行放大 */
  steering: boolean;
}

/**
 * 速度域避障：把「期望速度」按前方障碍改写成「安全的目标速度」。
 * ---------------------------------------------------------------
 * 处理顺序：
 *   1. 把期望速度拆成 前进(fwd) + 横移(lat) 两个分量（沿机头前向右手分解）
 *   2. 前进分量按减速曲线收敛：硬停面内归零，减速带内线性收敛，safeDist 外不限制
 *   3. 横移分量按绕行增益放大 —— 越接近障碍越鼓励横向移动，形成贴墙滑行/绕行
 *   4. 结果按 maxSpeed 做**矢量限幅**（不是逐分量，避免对角线超速）
 *
 * 关键：只限制「朝障碍去」的那部分，横移/后退一律放行 ——
 * 这正是 Phase 2 真机验证过的「贴墙滑行不撞墙」行为。
 *
 * @param desiredVX 期望速度 X（m/s）
 * @param desiredVZ 期望速度 Z
 * @param forwardX  机头前向单位向量 X
 * @param forwardZ  机头前向单位向量 Z
 * @param distance  前向净距（m）；Infinity/NaN 表示测距不可用 → 不做避障
 */
export function applyAvoidance(desiredVX: number, desiredVZ: number,
  forwardX: number, forwardZ: number, distance: number,
  params: SteeringParams = DEFAULT_STEERING): AvoidResult {
  // 测距不可用（∞ / NaN / ≤0）时不做任何限制：
  // 那是「防撞是盲的」，不是「前方安全」—— 强行限制只会让无人机僵住。
  if (!isValidDistance(distance)) {
    return {
      vx: desiredVX, vz: desiredVZ, braking: false, emergency: false,
      avoidStrength: 0, threatLevel: 'safe', steering: false,
    };
  }

  // 阈值透传给 evaluate：SteeringParams 的 band / emergencyDist / safeDist
  // 必须真正参与计算，否则调参者会以为改了却毫无变化。
  // 不传（默认值）时与模块常量一致，行为与 Phase 2 完全相同。
  const info: CollisionInfo = evaluate(distance, DRONE_RADIUS_M,
    params.emergencyDist, params.band, params.safeDist);
  const fwdScale: number = 1 - info.avoidStrength;

  // —— 分解为 前进 + 横移 ——
  const fwd: number = desiredVX * forwardX + desiredVZ * forwardZ;
  let latX: number = desiredVX - fwd * forwardX;
  let latZ: number = desiredVZ - fwd * forwardZ;

  // —— 横向绕行增益 ——
  // 只在用户已有横移输入时放大；直推墙壁且无横移输入时 lat=0，不会凭空侧移。
  const gain: number = 1 + (params.steerGain - 1) * info.avoidStrength;
  const steering: boolean = info.avoidStrength > 0 && (Math.abs(latX) > 1e-6 || Math.abs(latZ) > 1e-6);
  latX = latX * gain;
  latZ = latZ * gain;

  let vx: number = latX + fwd * fwdScale * forwardX;
  let vz: number = latZ + fwd * fwdScale * forwardZ;

  // —— 矢量限幅到 maxSpeed ——
  const mag: number = Math.sqrt(vx * vx + vz * vz);
  if (mag > params.maxSpeed && mag > 1e-9) {
    vx = vx / mag * params.maxSpeed;
    vz = vz / mag * params.maxSpeed;
  }

  return {
    vx: vx,
    vz: vz,
    braking: info.avoidStrength > 0,
    emergency: info.emergency,
    avoidStrength: info.avoidStrength,
    threatLevel: info.threatLevel,
    steering: steering,
  };
}

/**
 * 纯函数自检（真机 / Node / CI 可直接验收）。
 * 覆盖 PRD 力模型与速度域集成的关键性质。
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
  const P: SteeringParams = DEFAULT_STEERING;

  // —— PRD §6.3.2 力模型 ——
  const sf = seekForce({ x: 1, y: 0, z: 0 }, { x: 0, y: 0, z: 0 }, P.maxForce);
  check('seek 指向速度差', sf.x > 0.99 && Math.abs(sf.y) < 1e-9);
  const magSf: number = Math.sqrt(sf.x * sf.x + sf.y * sf.y + sf.z * sf.z);
  check('seek 幅值=maxForce', Math.abs(magSf - P.maxForce) < 1e-6);

  const af0 = avoidForce(P.safeDist, P.safeDist, { x: 1, y: 0, z: 0 }, P.maxForce);
  check('avoid 在 safeDist 处为 0', Math.abs(af0.x) < 1e-9);
  const af1 = avoidForce(0.01, P.safeDist, { x: 1, y: 0, z: 0 }, P.maxForce);
  check('avoid 贴近障碍≈maxForce', af1.x > P.maxForce * 0.9 && af1.x <= P.maxForce + 1e-9);
  check('avoid 方向=远离障碍', af1.x > 0);

  const big = combineForces({ x: 5, y: 0, z: 0 }, { x: 5, y: 0, z: 0 }, P.maxForce);
  const magBig: number = Math.sqrt(big.x * big.x + big.y * big.y + big.z * big.z);
  check('合力限幅到 maxForce', Math.abs(magBig - P.maxForce) < 1e-6);

  // —— 速度域集成 ——
  // 机头朝 -Z（与 DroneController 的 yaw=0 约定一致）：forward = (0, -1)
  const FX: number = 0;
  const FZ: number = -1;
  // 用户推杆"向前"→ 期望速度沿前向 (0,-1.5)
  const fwdDesiredVZ: number = -1.5;

  const rFar = applyAvoidance(0, fwdDesiredVZ, FX, FZ, 2.5, P);
  check('远处不限制', !rFar.braking && Math.abs(rFar.vz - fwdDesiredVZ) < 1e-9);

  const rStop = applyAvoidance(0, fwdDesiredVZ, FX, FZ, 0.3, P);
  check('硬停面前进归零', rStop.emergency && Math.abs(rStop.vz) < 1e-9);

  const rSlow = applyAvoidance(0, fwdDesiredVZ, FX, FZ, 1.0, P);
  check('减速带内减速', rSlow.braking && Math.abs(rSlow.vz) < Math.abs(fwdDesiredVZ) - 1e-6);
  check('减速单调（越近越慢）', Math.abs(rSlow.vz) > Math.abs(rStop.vz));
  // 回归守护（本次 bug）：gap=1.0 时前进分量必须 ≥ 满速 45%。
  // fwdScale 在 gap=1.0 = (1.0-0.4)/1.2 = 0.5，故 |vz| = 1.5*0.5 = 0.75 ≥ 0.675。
  // 这条断言锁死"防撞层把前向压到接近 0"的回归——正是右摇杆前后失灵的现场。
  check('gap=1.0 前进≥满速45%', Math.abs(rSlow.vz) >= Math.abs(fwdDesiredVZ) * 0.45 - 1e-9);

  // 横移（沿 X）必须被完整保留 —— 贴墙滑行的基础
  const rSlide = applyAvoidance(1.0, 0, FX, FZ, 0.3, P);
  check('急停时横移保留', Math.abs(rSlide.vx) > 0.99);

  // 绕行增益：接近障碍且有横移输入时，横移被放大
  const rGain = applyAvoidance(0.5, 0, FX, FZ, 0.5, P);
  check('横移绕行放大', rGain.steering && rGain.vx > 0.5 + 1e-6);

  // 测距不可用 → 不限制（盲 ≠ 安全，但不能僵住）
  const rBlind = applyAvoidance(0, fwdDesiredVZ, FX, FZ, Infinity, P);
  check('测距不可用不限制', !rBlind.braking && Math.abs(rBlind.vz - fwdDesiredVZ) < 1e-9);

  // 参数透传：收窄 band 后同一距离不再减速 —— 证明 params.band 真的进了 evaluate，
  // 而不是被忽略（这类"改了没反应"的问题在真机上极难定位）
  const narrow: SteeringParams = {
    safeDist: 0.9, emergencyDist: 0.4, band: 0.5,
    maxForce: P.maxForce, maxSpeed: P.maxSpeed, steerGain: P.steerGain,
  };
  const rNarrow = applyAvoidance(0, fwdDesiredVZ, FX, FZ, 1.0, narrow);
  check('自定义参数透传生效', !rNarrow.braking && Math.abs(rNarrow.vz - fwdDesiredVZ) < 1e-9);

  // 结果不得超过 maxSpeed
  const rCap = applyAvoidance(3.0, 3.0, FX, FZ, 0.3, P);
  const magCap: number = Math.sqrt(rCap.vx * rCap.vx + rCap.vz * rCap.vz);
  check('结果不超过 maxSpeed', magCap <= P.maxSpeed + 1e-6);

  const msg: string = fails.length === 0 ?
    `PASS（${total} 项断言全通过）` : `FAIL：${fails.join('、')}`;
  Logger.info(`${TAG} [VERIFY] 转向层自检 ${msg}`);
  return msg;
}
