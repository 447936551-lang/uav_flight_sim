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
  avoidMarginFor,
  avoidSafeDistFor,
  clamp01,
  evaluate,
  isValidDistance,
  normalize
} from './CollisionDetector';
import { Logger } from '../core/Logger';

const TAG: string = 'SteeringBehavior';

/**
 * 规划式绕行增益：障碍越近、主动侧向绕行速度越大（仅 bypassMode 生效）。
 *
 * 命名说明：业界同类能力常以某厂商的注册商标缩写指代，而本仓库以 Apache-2.0
 * 公开分发并面向 OpenHarmony 社区评审，故一律使用中性名称「bypass / 规划式绕行」。
 * App 源文件中的对应标识为该商标缩写——这是两侧唯一允许的命名分叉
 * （见 docs/extraction-map.md §1），算法语义与数值行为不受影响。
 */
const BYPASS_STEER_GAIN: number = 1.2;

/**
 * 障碍横向偏置的**死区**（m）。
 * ---------------------------------------------------------------
 * 正前方极小的横向偏移（|lateral| < 0.12m）说明障碍基本就在航线上，
 * 此时左右绕行等价（绕哪边都要横移差不多的距离），没有"择优"的依据。
 * 死区的作用是避免读数噪声（±0.05m 的抖动）导致绕行方向**逐帧翻转** ——
 * 那会让无人机在障碍前左右摇摆，看起来比直撞还糟。
 *
 * 死区外则严格按符号择优（见 pickAvoidSide）。
 */
const BYPASS_SIDE_DEADZONE_M: number = 0.12;

/**
 * 绕行方向择优：+1 = 向右绕，-1 = 向左绕，0 = 不主动绕（保持原行为）。
 * ---------------------------------------------------------------
 * P1-2（2026-10-05）。旧实现**永远向右绕**（rxA = -forwardZ 硬编码），
 * 完全不看障碍在哪一侧。真机后果很具体：
 *   · 障碍偏左 0.8m → 本该左绕（横移距离 0.8m），却向右绕（要横移 2.2m
 *     才能过去），于是贴着障碍右冲，减速带内反复被刹停 → 表现为
 *     「bypass 在障碍前卡住不动」；
 *   · 左侧是墙、右侧是空旷 → 向右绕恰好是对的，但那是运气不是设计。
 *
 * 判据用**横向偏移的符号**：lateral > 0 表示障碍在机体右侧，
 * 此时向右绕是**迎面撞上去**，正确做法是往左让。反之亦然。
 * 所以返回值的符号与 lateral 相反 —— 这是本函数最容易写反的一处，
 * 已用 selfTest 的两个镜像场景钉死。
 *
 * 死区内返回 0：调用方据此跳过主动绕行块（退化为纯减速），
 * 而不是用噪声符号随便挑一边。
 */
export function pickAvoidSide(lateral: number, deadzone: number = BYPASS_SIDE_DEADZONE_M): number {
  if (!isFinite(lateral) || Math.abs(lateral) < deadzone) {
    return 0;
  }
  return lateral > 0 ? -1 : 1;
}

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
  steerGain: AVOID_STEER_GAIN
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
    z: desired.z - current.z
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
    z: dir.z * strength * maxForce
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
  /**
   * bypass 主动绕行方向（P1-1，2026-10-05 新增）：true = 正在向左绕。
   * 两个都为 false = 本帧没主动绕（不在 bypass 模式 / 障碍在正前方死区内 /
   * 用户已有横移输入）。HUD 与诊断据此显示「向左绕行中」而不是笼统的
   * 「避障中」—— 用户能看懂飞机准备往哪边走。
   */
  avoidedLeft: boolean;
  /** bypass 主动绕行方向：true = 正在向右绕 */
  avoidedRight: boolean;
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
  params: SteeringParams = DEFAULT_STEERING, bypassMode: boolean = false,
  obstacleLateral: number = 0, speed: number = 0): AvoidResult {
  // 测距不可用（∞ / NaN / ≤0）时不做任何限制：
  // 那是「防撞是盲的」，不是「前方安全」—— 强行限制只会让无人机僵住。
  if (!isValidDistance(distance)) {
    return {
      vx: desiredVX, vz: desiredVZ, braking: false, emergency: false,
      avoidStrength: 0, threatLevel: 'safe', steering: false,
      avoidedLeft: false, avoidedRight: false
    };
  }

  // 阈值透传给 evaluate：SteeringParams 的 band / emergencyDist / safeDist
  // 必须真正参与计算，否则调参者会以为改了却毫无变化。
  //
  // 2026-10-05：emergencyDist / safeDist 改为**按当前速度动态推导**
  // （见 CollisionDetector.avoidMarginFor 的刹车距离推导）。
  // 但 params 里显式传了值的场合（自检、离线调用）必须优先，
  // 否则「调用方改了却看不到变化」—— 这是本文件一直坚守的契约。
  // 判定方式：与默认值不同即视为「调用方显式指定」。
  const margin: number = (params.emergencyDist !== DEFAULT_STEERING.emergencyDist) ?
    params.emergencyDist : avoidMarginFor(speed);
  const safeDist: number = (params.safeDist !== DEFAULT_STEERING.safeDist) ?
    params.safeDist : avoidSafeDistFor(speed, params.band);
  const info: CollisionInfo = evaluate(distance, DRONE_RADIUS_M, margin, params.band, safeDist);
  const fwdScale: number = 1 - info.avoidStrength;
  // bypass 主动绕行方向（供返回值/HUD 展示）。默认都 false = 本帧没主动绕。
  let avoidedLeft: boolean = false;
  let avoidedRight: boolean = false;

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

  // —— bypass 规划式绕行（仅 bypassMode）：用户直推墙且无横移输入时，主动生成
  // 侧向速度，使轨迹平滑偏向一侧绕过，而非急停在原地。
  // 这里在「横向绕行增益」(steerGain) 之后叠加，二者叠加亦不冲突：
  // steerGain 只放大用户已有的横移；本块在「完全没横移」时也主动给一个侧向分量。
  //
  // 绕行**方向**按障碍实际方位择优（P1-1 2026-10-05 修，见 pickAvoidSide）：
  // 旧实现硬编码「永远向右」，不看障碍在哪一侧 —— 障碍偏左时会朝障碍那侧冲，
  // 表现为在障碍前卡住。side=0（横向偏置在死区内 / 调用方未提供）时
  // 退回旧的向右行为，保证零回归。
  if (bypassMode && info.avoidStrength > 0) {
    const fwdA: number = desiredVX * forwardX + desiredVZ * forwardZ;
    const latAX: number = desiredVX - fwdA * forwardX;
    const latAZ: number = desiredVZ - fwdA * forwardZ;
    const hasLatA: boolean = Math.abs(latAX) > 1e-6 || Math.abs(latAZ) > 1e-6;
    if (fwdA > 0 && !hasLatA) {
      // 右向 = 前向逆时针转 90°。side=-1（向左绕）时取反。
      const side: number = pickAvoidSide(obstacleLateral);
      // side=0（死区内 / 方位未知）→ dirSign=1，即旧实现那个「永远向右」的行为。
      // 这样未提供方位的既有调用点行为完全不变（零回归），
      // 而不是突然变成「不绕行」——后者对已有 bypass 体验是破坏性的。
      const dirSign: number = side === 0 ? 1 : side;
      const rxA: number = -forwardZ * dirSign;
      const rzA: number = forwardX * dirSign;
      const steer: number = fwdA * info.avoidStrength * BYPASS_STEER_GAIN;
      latX += rxA * steer;
      latZ += rzA * steer;
      // 方向标记按**实际施加**的方向报（与 dirSign 严格一致），
      // 而不是按 side 报 —— 否则 side=0 时会既不标左也不标右，
      // 出现「明明在往右绕、HUD 却什么都不显示」的错位。
      avoidedLeft = dirSign < 0;
      avoidedRight = dirSign > 0;
    }
  }

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
    avoidedLeft: avoidedLeft,
    avoidedRight: avoidedRight
  };
}

/**
 * 纯函数自检（真机 hilog 可直接 grep 验收）。
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

  // ⚠️ 2026-10-05：以下用例**显式传 speed**，不再依赖默认 0。
  // 原因：硬停面改为随速度伸缩（avoidMarginFor）后，默认 speed=0 的
  // margin 是 0.25m，而旧用例用 distance=0.3 断言「硬停面前进归零」——
  // 0.3 > 0.25 就不再是 emergency，断言会假失败。
  // 显式传速让每条用例的阈值可推算，日后调阈值不会互相掩盖。
  const V_HI: number = 1.5;   // 满速：margin = 0.25 + 1.5²/3 = 1.0m
  const V_LO: number = 0.0;   // 静止：margin = 0.25m

  const rFar = applyAvoidance(0, fwdDesiredVZ, FX, FZ, 2.5, P, false, 0, V_HI);
  check('远处不限制', !rFar.braking && Math.abs(rFar.vz - fwdDesiredVZ) < 1e-9);

  // 满速时硬停面 1.0m → 0.3m 必然在硬停面内。这正是本轮修掉的运动学不自洽：
  // 旧版 margin 恒为 0.4，而 1.5m/s 的刹车距离 0.75m 反而大于它。
  const rStop = applyAvoidance(0, fwdDesiredVZ, FX, FZ, 0.3, P, false, 0, V_HI);
  check('硬停面前进归零', rStop.emergency && Math.abs(rStop.vz) < 1e-9);

  const rSlow = applyAvoidance(0, fwdDesiredVZ, FX, FZ, 1.0, P, false, 0, V_LO);
  check('减速带内减速', rSlow.braking && Math.abs(rSlow.vz) < Math.abs(fwdDesiredVZ) - 1e-6);
  check('减速单调（越近越慢）', Math.abs(rSlow.vz) > Math.abs(rStop.vz));
  // 回归守护（历史 bug）：gap=1.0 时前进分量必须 ≥ 满速 45%。
  // 静止时 margin=0.25 → fwdScale = (1.0-0.25)/1.2 = 0.625 → |vz| = 0.94。
  // 这条断言锁死"防撞层把前向压到接近 0"的回归——正是右摇杆前后失灵的现场。
  check('gap=1.0 前进≥满速45%', Math.abs(rSlow.vz) >= Math.abs(fwdDesiredVZ) * 0.45 - 1e-9);

  // 横移（沿 X）必须被完整保留 —— 贴墙滑行的基础
  const rSlide = applyAvoidance(1.0, 0, FX, FZ, 0.3, P, false, 0, V_HI);
  check('急停时横移保留', Math.abs(rSlide.vx) > 0.99);

  // 绕行增益：接近障碍且有横移输入时，横移被放大
  const rGain = applyAvoidance(0.5, 0, FX, FZ, 0.5, P, false, 0, V_HI);
  check('横移绕行放大', rGain.steering && rGain.vx > 0.5 + 1e-6);

  // 测距不可用 → 不限制（盲 ≠ 安全，但不能僵住）
  const rBlind = applyAvoidance(0, fwdDesiredVZ, FX, FZ, Infinity, P, false, 0, V_HI);
  check('测距不可用不限制', !rBlind.braking && Math.abs(rBlind.vz - fwdDesiredVZ) < 1e-9);

  // —— 速度相关阈值真的接进来了（2026-10-05）——
  // 同一距离 0.8m：满速时已在硬停面内（急停），静止时只是减速带内。
  // 这条锁死"speed 参数真的进了 evaluate"——否则改了速度却毫无变化。
  const rHiSpeed = applyAvoidance(0, fwdDesiredVZ, FX, FZ, 0.8, P, false, 0, V_HI);
  const rLoSpeed = applyAvoidance(0, fwdDesiredVZ, FX, FZ, 0.8, P, false, 0, V_LO);
  check('同一距离：满速急停 / 静止仅减速（阈值随速生效）',
    rHiSpeed.emergency && !rLoSpeed.emergency);
  check('满速时更早介入（强度更高）', rHiSpeed.avoidStrength >= rLoSpeed.avoidStrength);
  // 显式 params 仍优先于 speed 推导（守护"改参数却看不到变化"的老契约）。
  // ⚠️ emergencyDist 必须取**不同于 DEFAULT(0.4)** 的值：等于默认值时
  // 会被视为「未显式指定」而走速度推导，断言前提自反（2026-10-06 修）。
  const customM: SteeringParams = {
    safeDist: 0.9, emergencyDist: 0.6, band: 0.5, maxForce: P.maxForce,
    maxSpeed: P.maxSpeed, steerGain: P.steerGain
  };
  const rCustom = applyAvoidance(0, fwdDesiredVZ, FX, FZ, 0.8, customM, false, 0, V_HI);
  check('显式 emergencyDist 优先于速度推导（不被 speed 覆盖）',
    rCustom.emergency === (0.8 <= customM.emergencyDist));

  // 参数透传：收窄 band 后同一距离不再减速 —— 证明 params.band 真的进了 evaluate，
  // 而不是被忽略（这类"改了没反应"的问题在真机上极难定位）
  const narrow: SteeringParams = {
    safeDist: 0.9, emergencyDist: 0.4, band: 0.5,
    maxForce: P.maxForce, maxSpeed: P.maxSpeed, steerGain: P.steerGain
  };
  const rNarrow = applyAvoidance(0, fwdDesiredVZ, FX, FZ, 1.0, narrow);
  check('自定义参数透传生效', !rNarrow.braking && Math.abs(rNarrow.vz - fwdDesiredVZ) < 1e-9);

  // 结果不得超过 maxSpeed
  const rCap = applyAvoidance(3.0, 3.0, FX, FZ, 0.3, P);
  const magCap: number = Math.sqrt(rCap.vx * rCap.vx + rCap.vz * rCap.vz);
  check('结果不超过 maxSpeed', magCap <= P.maxSpeed + 1e-6);

  // —— bypass 绕行方向择优（P1-1，2026-10-05）——
  // 旧实现永远向右绕（硬编码），不看障碍在哪一侧。真机表现：障碍偏左时
  // 朝障碍那侧冲，在减速带里反复被刹停，表现为「bypass 在障碍前卡住不动」。
  //
  // 约定：lateral > 0 = 障碍在机体**右**侧 → 必须往**左**绕（迎面绕是撞上去）。
  // 机头朝 -Z 时右向为 (1,0)……注意 applyAvoidance 里 rxA = -forwardZ*dirSign，
  // forward=(0,-1) → -(-1)=1，即 dirSign=+1 时朝 +X 走。AR 系下 +X 是屏幕右，
  // 故 dirSign=+1 = 向右绕，dirSign=-1 = 向左绕。两条都要验，防止符号写反。
  check('障碍在右侧 → 判定为向左绕',
    pickAvoidSide(0.8) === -1);
  check('障碍在左侧 → 判定为向右绕',
    pickAvoidSide(-0.8) === 1);
  // 死区：正前方极小偏置不挑边（否则读数噪声会让绕行方向逐帧翻转 → 左右摇摆）
  check('死区内不挑边（防左右摇摆）', pickAvoidSide(0.05) === 0);
  check('死区边界内侧仍不挑边', pickAvoidSide(0.11) === 0);
  check('死区边界外侧开始挑边', pickAvoidSide(0.13) === -1);
  // 未知方位（NaN/∞）不得挑边，退回旧行为
  check('方位未知不挑边', pickAvoidSide(NaN) === 0 && pickAvoidSide(Infinity) === 0);
  check('死区可配置', pickAvoidSide(0.3, 0.5) === 0);

  // 端到端：bypass 模式下，绕行方向必须真的体现在输出速度上。
  // 机头朝 -Z，向左绕 = -X 方向速度；向右绕 = +X。
  const rApasRight = applyAvoidance(0, fwdDesiredVZ, FX, FZ, 0.8, P, true, 0.8);
  check('bypass·障碍在右 → 输出向左侧向速度（不撞上去）',
    rApasRight.avoidedLeft && !rApasRight.avoidedRight && rApasRight.vx < 0);
  const rApasLeft = applyAvoidance(0, fwdDesiredVZ, FX, FZ, 0.8, P, true, -0.8);
  check('bypass·障碍在左 → 输出向右侧向速度',
    rApasLeft.avoidedRight && !rApasLeft.avoidedLeft && rApasLeft.vx > 0);
  // 死区内 / 未提供方位 → 保持旧的「向右绕」行为（零回归守护）
  const rApasLegacy = applyAvoidance(0, fwdDesiredVZ, FX, FZ, 0.8, P, true, 0);
  check('bypass·方位未知 → 退回旧的向右绕（零回归）',
    rApasLegacy.avoidedRight && !rApasLegacy.avoidedLeft && rApasLegacy.vx > 0);
  // 非 bypass 模式绝不主动绕行（这是主动行为，不能污染普通避障）
  const rNotApas = applyAvoidance(0, fwdDesiredVZ, FX, FZ, 0.8, P, false, 0.8);
  check('非 bypass 模式不主动绕行',
    !rNotApas.avoidedLeft && !rNotApas.avoidedRight && Math.abs(rNotApas.vx) < 1e-9);
  // 用户已有横移输入时，bypass 不再叠加（叠加会与用户意图打架）
  const rApasHasLat = applyAvoidance(1.0, fwdDesiredVZ, FX, FZ, 0.8, P, true, 0.8);
  check('bypass·用户已有横移时不叠加主动绕行',
    !rApasHasLat.avoidedLeft && !rApasHasLat.avoidedRight);

  const msg: string = fails.length === 0 ?
    `PASS（${total} 项断言全通过）` : `FAIL：${fails.join('、')}`;
  Logger.info(`${TAG} [VERIFY] 转向层自检 ${msg}`);
  return msg;
}
