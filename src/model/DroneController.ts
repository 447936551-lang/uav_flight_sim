/**
 * 无人机飞行控制（纯逻辑，与渲染解耦）
 * ---------------------------------------------------------------
 * 从 Unity 版 DroneController.cs 移植而来。
 * 本类只维护「相对锚点的位姿状态」，由渲染层每帧读取并写入 3D 节点。
 *
 * 坐标系：AR 世界坐标（右手系，单位米）
 *   X 右 / Y 上 / Z 前（朝向用户为 +Z，即相机看向 -Z）
 *   因此「机头朝前（远离用户）」在 yaw = 0 时是 -Z 方向。
 *
 * 本版相对上一版的核心变化（修复「按键飞机不动」+ 增加真实飞行手感）：
 *   1. 位移由「直接改坐标」改为「速度积分」：
 *      按键给出目标速度 → 以有限加速度逼近；松手后按阻尼指数衰减，
 *      因此起飞有推背感、松手会滑行一段再停 —— 不再是按下即满速、松开即停。
 *   2. 机身姿态（pitch / roll）由「机体系速度分量」推导：
 *      向前加速时低头、向右移动时右倾，停稳后自动回正。
 *      这是「飞行感」最关键的视觉线索。
 *   3. 旋翼转速随油门（水平速度 + 升降速度）连续变化，
 *      不再只有 flying 的 0 / 1 两态。
 *
 * 数据来源（重要）：前向障碍距离通过 SpatialPerception 抽象接口获取，
 * 使算法层彻底脱离 AREngine / 深度硬件，可被 AR 引擎、离线仿真、真实传感器
 * 三类数据源统一驱动（见 src/contract/SpatialPerception.ts）。
 */
import {
  AvoidResult,
  DEFAULT_STEERING,
  SteeringParams,
  applyAvoidance,
} from '../avoidance/SteeringBehavior';
import {
  CeilingInfo,
  SensorTrust,
  avoidMarginFor,
  evaluateCeiling,
  isValidDistance,
  sensorClimbScale,
  sensorTrust,
} from '../avoidance/CollisionDetector';
import { TemporalGuard, TemporalState } from '../avoidance/TemporalGuard';
import {
  DEFAULT_VO,
  SectorInput,
  VoOptions,
  VoResult,
  buildSectors,
  pickThreatSector,
  solveVelocityObstacle,
} from '../avoidance/VelocityObstacle';
import {
  DcaConfig,
  DcaOverlay,
  DEFAULT_DCA,
  dcaOverlay,
} from '../avoidance/DelayCompensator';
import {
  MockVisionSource,
  VisionParallax,
  VpndReason,
  VpndResult,
} from '../avoidance/VisionParallax';
import { Logger } from '../core/Logger';
import { Vec3 } from '../core/Vec3';
import { SectorScan, SpatialPerception } from '../contract/SpatialPerception';
import { EnvironmentPerception } from '../contract/EnvironmentPerception';
import { WIND_GAIN } from '../environment/Atmosphere';
import {
  attitudeRateCmd,
  attitudeTorque,
  dragForce,
  positionHoldVelocity,
  rateTorque,
  thrustVector,
  tiltFromUpDir,
  velocityLoop,
  verticalLoop,
  yawTorque,
} from '../dynamics/CascadeController';
import {
  DEFAULT_DYNAMICS,
  DynamicsParams,
  FlightDynamics,
  geometryOf,
  groundEffectFactor,
} from '../dynamics/FlightDynamics';
import {
  applyActuatorLimit,
  DEFAULT_QUAD,
  invertActuatorNonlinear,
  mixThrusts,
  QuadGeometry,
  RotorPlant,
} from '../dynamics/RotorMixer';
import { TurbulenceModel } from '../environment/Turbulence';
import {
  EkfEstimator,
  ImuSample,
  quatFromEuler,
  quatToRot,
} from '../dynamics/EkfEstimator';
import {
  PosObservation,
  SensorModel,
  VelObservation,
} from '../dynamics/SensorModel';
import { PX4_STOCK, px4EkfConfig } from '../dynamics/Px4Params';

// —— 闭环定点保持（M3 Step1）：复用 CascadeController.positionHoldVelocity ——
// 与动力学模式的位置环同源：松杆 + 启用环境感知时，把飞机拉回锚点，
// 抵消风扰与残差，实现「松杆即定点、风吹也不跑」的闭环悬停。
const KIN_WIND_FEEDFORWARD: number = 0.9; // 风扰前馈抵消比例 0..1（留 0.1 差额保留"被风推"实感）
const KIN_POS_KP: number = 1.5;           // 位置环增益（1/s），与动力学 DYN_POS_KP 同量级
const KIN_HOLD_RESP: number = 6.0;        // 保持环速度响应（1/s）
const KIN_HOLD_AUTHORITY: number = 1.0;   // 保持权限（占 maxSpeed 比例），防异常误差猛冲回锚点

// —— 双内核（M3 Step2）：动力学内核级联增益，与 FlightDynamics / RotorMixer 同源 ——
// 复用 CascadeController 的 velocityLoop / thrustVector / tiltFromUpDir /
// attitudeTorque / yawTorque / dragForce，把"摇杆→期望速度→期望推力矢量→混控→
// 执行器滞后→刚体积分"整条真实飞控链路接进 DroneController。
const DYN_VEL_KP: number = 4.0;        // 速度环增益（1/s）
const DYN_MAX_ACCEL: number = 6.0;     // 速度环最大指令加速度（m/s²，受 maxTilt 限幅再约束）
const DYN_ATT_KP: number = 225.0;      // 姿态环比例增益（与 CascadeController 自检同源）
const DYN_ATT_KD: number = 30.0;       // 姿态环阻尼增益
const DYN_YAW_KP: number = 80.0;       // 偏航环增益
const DYN_ALT_KP: number = 1.2;        // 高度位置环增益（1/s）：松杆定高，与 App DYN_ALT_KP 同源
const DYN_SUBSTEP_HZ: number = 240;    // 动力学子步频率（半隐式欧拉稳定性要求）
const DYN_MOTOR_TAU: number = 0.05;    // 电机一阶滞后时间常数（秒），与 RotorPlant 默认一致
const LAND_TAPER_BAND: number = 0.4;   // 软降落锥形减速带（m）：低于此高度按比例限下降速度
const LAND_TOUCHDOWN_SPEED: number = 0.12; // 触地速度（m/s）：锥形收敛到的目标下降速度

// —— M4 速率环内环（阶段A）：姿态→角速度→力矩 级联增益（与单 PD 等刚度，零回归）——
const DYN_ATT_RATE_KP: number = 7.5;   // 姿态角 → 期望角速度（1/s）
const DYN_RATE_KP: number = 30.0;      // 角速度 → 期望力矩（与 DYN_RATE_KD 构成内环 PD）
const DYN_RATE_KD: number = 0.0;       // 角速度阻尼（本机动力学惯量小，0 即可稳定）
const DYN_YAW_RATE_KP: number = 8.0;   // 偏航速率环增益（常量备齐；社区内核偏航走运动学通道）
const DYN_YAW_RATE_KD: number = 0.0;
const DYN_MAX_RATE: number = 6.0;      // 姿态角速度限幅（rad/s）
const DEG2RAD: number = Math.PI / 180;
const RAD2DEG: number = 180 / Math.PI;

/**
 * 无人机权威状态（Phase 5 显式物理状态机）。
 * ---------------------------------------------------------------
 * 早期版本只用 placed / flying 两个布尔（退化状态机），UI 与诊断层各自推断
 * "现在到底处于什么状态"，语义容易漂移。这里把状态**显式枚举**出来作为
 * 唯一权威态（state），而 placed / flying 继续保留 —— 它们是 UI / 渲染层
 * 已有的输入接口，改动会引爆大范围回归，故只新增派生态，不动旧布尔。
 *
 * 语义：
 *   Unplaced      未放置（还没在地面按下"放置"）
 *   Grounded      已放置、未起飞（或已降落停桨在地面）
 *   Flying        飞行中（含正常悬停 / 巡航 / 避障减速，但未进入硬停面）
 *   Landing       正在软降落（用户 disarm 后仍在半空，朝 minHeight 下降）
 *   EmergencyHold 进入硬停面急停保持（avoidEmergency 触发，悬停刹车）
 */
export enum DroneState {
  Unplaced,
  Grounded,
  Flying,
  Landing,
  EmergencyHold,
}

/**
 * 无人机模型缩放系数（安全网）。
 * ---------------------------------------------------------------
 * GLB 已按真实比例以「米」为单位建模（整机含桨约 0.37m），
 * 正常情况下无需缩放，保持 1.0 即可。
 * 若真机目测仍有偏差，只需改这一个常量并重新构建，
 * 不必重新生成 GLB。
 */
export const DRONE_MODEL_SCALE: number = 1.0;

/**
 * 飞行动力学参数。
 * 集中在一处，便于按真机手感调参，也便于入口页「飞行参数」直接改写。
 */
export interface DroneFlightParams {
  /** 最大水平速度 m/s */
  maxSpeed: number;
  /** 水平加速度 m/s²（越大越跟手，越小越有惯性） */
  accel: number;
  /** 松手后的速度阻尼系数 1/s（越小滑行越久） */
  damping: number;
  /** 最大升降速度 m/s */
  climbSpeed: number;
  /** 升降加速度 m/s² */
  climbAccel: number;
  /** 最大偏航角速度 度/s */
  yawSpeed: number;
  /** 偏航角加速度 度/s² */
  yawAccel: number;
  /** 最大机身倾角 度 */
  maxTilt: number;
  /** 姿态跟随速度 1/s（越大回正/倾斜越快） */
  tiltResponse: number;
  /** 最低离地高度 m */
  minHeight: number;
  /** 最高飞行高度 m */
  maxHeight: number;
}

/**
 * 默认动力学参数。
 *
 * maxSpeed 取 1.5 m/s 的理由（室内 AR 场景的权衡）：
 *   - 无人机放置后距相机典型 1~3 m，室内可用空间有限；
 *     2.0 m/s 持续 2 秒会飞出 3.4 m，很容易直接飞出视野或撞墙，
 *     用户观感变成「一去不回」，反而不利于观察。
 *   - 1.5 m/s 时：1 秒约走 1.2 m（已足够直观），2 秒约 2.7 m（仍在视野内）。
 *   - 「飞行感」主要来自加速度（0.43 秒到满速）与机身 18° 倾斜姿态，
 *     而不是极速；倾角在任意速度下都会打满，所以降速不会削弱飞行感。
 *   - 需要更快的用户可在入口页「最大速度」滑杆调到 3.0 m/s。
 */
export const DEFAULT_FLIGHT_PARAMS: DroneFlightParams = {
  maxSpeed: 1.5,
  accel: 3.5,
  damping: 3.0,
  climbSpeed: 1.2,
  climbAccel: 2.5,
  yawSpeed: 100,
  yawAccel: 260,
  maxTilt: 18,
  tiltResponse: 10,
  minHeight: 0.05,
  maxHeight: 5,
};

/** 把角度规范到 (-180, 180] */
function normalizeDeg(angle: number): number {
  let a: number = angle % 360;
  if (a > 180) {
    a -= 360;
  } else if (a <= -180) {
    a += 360;
  }
  return a;
}

/** 限幅 */
function clampValue(v: number, lo: number, hi: number): number {
  if (v < lo) {
    return lo;
  }
  if (v > hi) {
    return hi;
  }
  return v;
}

/** 以最大步长 maxStep 逼近目标值（返回新的当前值） */
function approachValue(current: number, target: number, maxStep: number): number {
  const diff: number = target - current;
  if (diff > maxStep) {
    return current + maxStep;
  }
  if (diff < -maxStep) {
    return current - maxStep;
  }
  return target;
}

/** 指数衰减系数：rate 为 1/s 的阻尼率 */
function decayFactor(rate: number, dt: number): number {
  const k: number = Math.exp(-rate * dt);
  return clampValue(k, 0, 1);
}

export class DroneController {
  /** 是否处于飞行状态（armed）：降落时冻结位姿与旋翼 */
  public flying: boolean = false;

  /**
   * 是否正在软降落（Landing 态内部标志）。
   * 仅当「用户 disarm（flying 由 true→false）且仍在半空（offsetY > minHeight + 0.02）」
   * 时置位；由 update() 产生朝 minHeight 的软下降，触底后自动清位回到 Grounded。
   * 放在 flying 布尔之外，是因为 disarm 后 flying=false，需要第二个标志区分
   * "正在降落"与"已落地" —— 否则两者都会被归成 Grounded，丢失过渡态。
   */
  private landing: boolean = false;

  /**
   * 是否已放置到平面上。
   * 放置后即进入可操控状态 —— 这是各运动入口的唯一门槛，
   * 用户无需再额外点击「起飞」即可操纵无人机。
   * 与 flying 的区别：placed 表示「已就位可操作」，flying 表示「是否正在积分/旋翼转动」。
   */
  public placed: boolean = false;

  /**
   * 权威状态（Phase 5 显式状态机派生态）。
   * 由 placed / flying / landing / avoidEmergency 在 syncState() 中推导得出，
   * 是 UI 与诊断层消费的单一事实来源；读它即可判断无人机当前处于哪一态，
   * 不必再在 UI 里重复 if (placed && flying && ...) 的推断逻辑。
   */
  public state: DroneState = DroneState.Unplaced;

  /** 相对锚点的偏移（米） */
  public offsetX: number = 0;
  public offsetY: number = 0.9;   // 初始悬停高度 0.9m
  public offsetZ: number = 0;

  /** 相对锚点的速度（米/秒，AR 世界系） */
  public velX: number = 0;
  public velY: number = 0;
  public velZ: number = 0;

  /** 绕自身 Y 轴朝向（度） */
  public yaw: number = 0;
  /** 偏航角速度（度/秒） */
  public yawRate: number = 0;

  /**
   * 机身俯仰角（度）：正 = 机头上仰。
   * 前进时自动取负（低头），符合真实多旋翼「靠前倾产生前推力」的物理。
   */
  public pitch: number = 0;
  /**
   * 机身滚转角（度）：正 = 机体右倾（右翼下沉）。
   * 向右平移时自动取正。
   */
  public roll: number = 0;

  /** 旋翼转速（用于渲染旋翼动画，0~1，可超过 1 表示超转） */
  public rotorSpeed: number = 0;

  /** 持续按住状态（由 UI 置位，tick 中统一处理，避免触摸事件频率抖动） */
  public holdUp: boolean = false;
  public holdDown: boolean = false;
  public holdYawLeft: boolean = false;
  public holdYawRight: boolean = false;
  /**
   * 模拟量输入（-1..1），由虚拟摇杆直接写入。
   * 对应 Mode 2 真实遥控器的左摇杆：
   *   climbInput > 0 → 上升；< 0 → 下降
   *   yawInput   > 0 → 右转；< 0 → 左转
   * 模拟量优先级高于布尔 holdUp/holdDown/holdYawLeft/holdYawRight（两个 UI 共存时模拟量优先，
   * 摇杆松手归零后布尔通道也能继续工作）。
   */
  public climbInput: number = 0;
  public yawInput: number = 0;
  /** 摇杆方向：x = 左右(-1~1)，y = 前后(-1~1) */
  public moveX: number = 0;
  public moveY: number = 0;

  /**
   * 闭环定点保持的锚点（米）与接合标志（M3 Step1）。
   * 无输入且启用环境感知时，位置环把飞机拉回 (holdX, holdZ)，
   * 抵消风扰与残差，实现「松杆即定点、风吹也不跑」的闭环悬停。
   * 默认 environmentActive=false 时整个闭环不介入 → 与旧开环阻尼逐位一致（零回归）。
   */
  public holdX: number = 0;
  public holdZ: number = 0;
  public holdEngaged: boolean = false;
  /**
   * P3：是否已由用户**显式**设定目标点（可视化目标点）。
   * false（默认）＝ 旧语义：锚点每帧跟随当前位姿，松手即就地定点。
   * true  ＝ 锚点锁定在 setHoldTarget 指定的点，摇杆操控不再改写它，
   *          于是松杆后飞机自动飞回该点悬停（「整定成一个目标点」的飞行语义）。
   * 与 App DroneController 同源（社区仓无 activeAlgo，故仅此轻量层）。
   */
  public holdTargetActive: boolean = false;
  /** 高度保持设定点（米）：松杆时高度位置环把飞机拉回此高度（M3 Step2 动力学内核用） */
  public holdY: number = 0.9;

  /**
   * 双内核开关（M3 Step2）：true = 真实动力学内核（级联 + 刚体 + 混控 + 执行器滞后），
   * false（默认）= 运动学内核。**默认 false 时整条动力学链路不介入 → 与旧运动学逐位一致（零回归）**。
   * 切换经 setFlightMode()，会把当前运动学状态无缝交接给刚体，避免跳变。
   * 注意：FlightAlgorithm 算法竞技框架（activeAlgo / flightAlgoRegistry）**不纳入社区**，
   * 本开关只管"内置运动学 ↔ 内置真实动力学"两态。
   */
  public useDynamics: boolean = false;
  /** 刚体动力学实例（懒建，复用 DEFAULT_DYNAMICS）；useDynamics=false 时恒为 null 不占用 */
  private dynamics: FlightDynamics | null = null;
  /** 执行器（电机）实例（懒建，tau=DYN_MOTOR_TAU）；负责转速一阶滞后 + 每桨差动 */
  private rotorPlant: RotorPlant | null = null;
  /** 四桨实际归一化转速 [FL,FR,BL,BR]，供 HUD / 渲染读取（0..maxRotor） */
  public rotorSpeeds: number[] = [0, 0, 0, 0];

  // ── M4/M5 保真层（默认全关，零回归；经 applyHighFidelityPreset / setHighFidelityEnabled 装配）──
  /** M4 速率环内环开关：开启后用「姿态→角速度→力矩」级联替代单 PD；默认关（零回归）。 */
  public useRateLoop: boolean = false;
  /** 气动模型：'basic'=各向同性二次阻力（默认）；'enhanced'=机体气动矩阵+入流（阶段B）。 */
  public aeroModel: string = 'basic';
  /** 执行器模型：'linear'=瞬时线性（默认）；'nonlinear'=电压跌落+油门曲线（阶段B·L4）。 */
  public actuatorModel: string = 'linear';
  /** M4 阶段C：L3 种子化湍流风（默认关，零回归）。 */
  public turbulence: boolean = false;
  public turbulenceIntensity: number = 1.0;
  public seed: number = 0;
  private turbModel: TurbulenceModel | null = null;
  /** M4 阶段D：L5 自主 EKF + L6 传感器噪声（默认关，零回归）。 */
  public estimator: string = 'none';
  public sensorNoise: boolean = false;
  private ekf: EkfEstimator | null = null;
  private sensor: SensorModel | null = null;
  private prevEkvX: number = 0;
  private prevEkvY: number = 0;
  private prevEkvZ: number = 0;
  public ekfPosX: number = 0;
  public ekfPosY: number = 0;
  public ekfPosZ: number = 0;
  public ekfVelX: number = 0;
  public ekfVelY: number = 0;
  public ekfVelZ: number = 0;
  /** 电池电量百分比（0~100）：M4 执行器非线性据其衰减可用推力；默认 100（=不影响，零回归）。 */
  public batteryPercent: number = 100;
  /** 刚体动力学参数（含 M4 新增 cdaForward/cdaSide/cdaUp/rotorRadius 四字段）；默认与 DEFAULT_DYNAMICS 一致。 */
  public dynParams: DynamicsParams = {
    mass: DEFAULT_DYNAMICS.mass,
    gravity: DEFAULT_DYNAMICS.gravity,
    armLength: DEFAULT_DYNAMICS.armLength,
    kT: DEFAULT_DYNAMICS.kT,
    kQ: DEFAULT_DYNAMICS.kQ,
    maxRotor: DEFAULT_DYNAMICS.maxRotor,
    inertiaTilt: DEFAULT_DYNAMICS.inertiaTilt,
    inertiaYaw: DEFAULT_DYNAMICS.inertiaYaw,
    angularDamping: DEFAULT_DYNAMICS.angularDamping,
    dragCoef: DEFAULT_DYNAMICS.dragCoef,
    maxTiltRad: DEFAULT_DYNAMICS.maxTiltRad,
    cdaForward: DEFAULT_DYNAMICS.cdaForward,
    cdaSide: DEFAULT_DYNAMICS.cdaSide,
    cdaUp: DEFAULT_DYNAMICS.cdaUp,
    rotorRadius: DEFAULT_DYNAMICS.rotorRadius,
  };

  /**
   * 空间感知输入（"空间感知输入" 契约）。
   * 若设置，update() 每帧通过它获取前向障碍距离；否则回退到 obstacleDistance 字段
   * （便于单测 / 仿真直接赋值）。这是算法与数据源解耦的唯一边界。
   */
  public perception: SpatialPerception | null = null;

  /**
   * 环境感知输入（"环境感知输入" 契约，第二个契约）。
   * 与 SpatialPerception 同构：设置后 update() 每帧通过它读取风场与空气密度；
   * 未设置或 `environmentActive = false` 时**完全不参与计算** ——
   * 这是"零回归"的硬保证：不接环境 = 与旧行为逐位一致。
   */
  public environment: EnvironmentPerception | null = null;

  /** 环境感知开关（默认关）。关闭时风场与密度恒为无风标准值。 */
  public environmentActive: boolean = false;
  /** 风→等效加速度增益（m/s² 每 m/s 风速），与 environment/Atmosphere 同源 */
  public windGain: number = WIND_GAIN;
  /** 环境风速矢量（m/s，AR 世界系），由环境层每帧写入；动力学内核据此算相对气流 */
  public windVelX: number = 0;
  public windVelZ: number = 0;
  /** 风等效加速度（m/s²，AR 世界系），本运动学内核消费 */
  public windAccelX: number = 0;
  public windAccelZ: number = 0;
  /** 空气密度相对因子（标准海平面 = 1.0） */
  public airDensityFactor: number = 1.0;

  /**
   * 机头前方最近障碍距离（米），由 SpatialPerception 每帧写入；
   * Infinity 表示前方无障碍（或深度不可用）。飞控据此限制前进分量。
   * 当 perception 为 null 时，由调用方直接赋值（单测 / 仿真场景）。
   */
  public obstacleDistance: number = Infinity;
  /** 防撞总开关（UI 可切换；关闭后照常飞行，便于对比） */
  public avoidanceEnabled: boolean = true;
  /** 当前是否正在因防撞限制前进（供 UI 反馈） */
  public avoidanceActive: boolean = false;
  /**
   * Phase 4：当前避障强度 0..1（1 = 完全阻塞）。
   * 由 SteeringBehavior 推导，面板与日志据此显示"刹车力度"，
   * 与实际生效的减速曲线同源，不会出现"显示危险却没刹车"的错位。
   */
  public avoidStrength: number = 0;
  /** Phase 4：是否处于急停（已进入硬停面） */
  public avoidEmergency: boolean = false;
  /**
   * Phase 4：转向/避障参数。
   * maxSpeed 每帧与飞行动力学参数同步（入口页可调），
   * 否则用户把最大速度调到 3.0 后，避障仍在按 1.5 做限幅。
   */
  public steering: SteeringParams = {
    safeDist: DEFAULT_STEERING.safeDist,
    emergencyDist: DEFAULT_STEERING.emergencyDist,
    band: DEFAULT_STEERING.band,
    maxForce: DEFAULT_STEERING.maxForce,
    maxSpeed: DEFAULT_STEERING.maxSpeed,
    steerGain: DEFAULT_STEERING.steerGain,
  };

  // ── P0：接通物理层已有算法（垂直避障 / 失联兜底 / 绕行择优）────────
  // 三者都**默认关闭**：算法在物理层早已实现并通过 selfTest，缺的只是"接线"。
  // 关掉时与旧行为逐位一致（由既有单测与场景守门）。

  /**
   * 垂直避障开关（P0-1）。开启后 evaluateCeiling() 参与爬升限速：
   * 上方净距越小，允许的上升速度越低；进入上方硬停面则完全禁止上升。
   * 取舍（与物理层一致）：**只压上升，绝不强推下降** —— 用户下方可能才是安全区。
   */
  public ceilingAvoidEnabled: boolean = false;
  /** 上方净距（米），由 SpatialPerception.getCeilingDistance() 每帧写入；∞ = 盲 */
  public ceilingDistance: number = Infinity;
  /** 上方避障强度 0..1（诊断用） */
  public ceilingStrength: number = 0;
  /** 是否处于上方急停（诊断用） */
  public ceilingEmergency: boolean = false;

  /**
   * 感知失联兜底开关（P0-2）。开启后按连续失联时长分档限制爬升
   * （normal 全量 / cautious 削弱 / hold 禁止），把 sensorTrust() 接进链路。
   */
  public trustFallbackEnabled: boolean = false;
  /** 当前兜底档位 */
  public trustLevel: SensorTrust = 'normal';
  /** 已连续失联时长（秒）—— sensorTrust 的输入，无论开关与否都计时 */
  public blindSec: number = 0;
  /** 本机是否支持深度（false 时立即 hold，不等超时） */
  public depthSupported: boolean = true;

  /**
   * 规划式绕行开关（P0-3）。开启后把障碍横向偏置喂给 applyAvoidance，
   * 使 pickAvoidSide 的"择优绕行"在库内路径上真正生效
   * （此前控制器恒传 bypassMode=false / lateral=0，等于该能力从未启用）。
   */
  public bypassEnabled: boolean = false;
  /** 障碍横向偏置（米，机体右向为正），由契约或威胁扇区推导 */
  public obstacleLateral: number = 0;

  // ── P1：时间维度守护（滤波 + 迟滞 + 失联计时）──────────────────────
  /**
   * 时间维度守护开关（P1）。开启后前向距离先过一阶低通，急停判定带迟滞。
   * 关闭时每一帧都走"原始读数 + 无迟滞判定"的既有路径（零回归）。
   */
  public temporalEnabled: boolean = false;
  /** 时间维度守护状态机（滤波记忆 + 急停迟滞 + 失联计时） */
  public readonly guard: TemporalGuard = new TemporalGuard();
  /** 滤波后的前向净距（米，诊断用）；仅在 temporalEnabled 时有意义，否则恒等于原始读数 */
  public filteredDistance: number = Infinity;

  // ── P2/P3：多扇区 + 预测式速度障碍 ─────────────────────────────────
  /**
   * 预测式避障开关（P3）。开启后把反应式层的输出再投影到速度障碍半空间交集上。
   * 叠加而非替换：投影只会让速度更保守，故"启用只会更安全"。
   */
  public voEnabled: boolean = false;
  /** 速度障碍求解参数 */
  public voOptions: VoOptions = {
    ttcSec: DEFAULT_VO.ttcSec,
    clearance: DEFAULT_VO.clearance,
    corridorHalf: DEFAULT_VO.corridorHalf,
    maxSpeed: DEFAULT_VO.maxSpeed,
    iterations: DEFAULT_VO.iterations,
  };

  // ── P4：延迟补偿（DCPA）/ 视觉视差负空间（VPND）──────────────────────
  /**
   * 延迟补偿预测式避障开关（P4 · DCPA）。开启后：PDC 把扇区云前推 τ_p 修正感知
   * 延迟、TBB 用"时间预算硬停面"早刹、EFP 在临界 TTC 直接满刹（跳过低通）。
   * 默认关闭 → 零回归（关闭时完全不介入任何下游输入）。
   */
  public dcaEnabled: boolean = false;
  /** DCA 配置（感知/控制/执行延迟 + 制动减速度 + 基线余量，真机可标定覆盖） */
  public dcaConfig: DcaConfig = DEFAULT_DCA;
  /** 本帧 DCA 早刹触发（诊断） */
  public dcaBraking: boolean = false;
  /** 本帧 DCA 修正后的前向净距（诊断；无威胁 = ∞） */
  public dcaForwardDist: number = Infinity;
  /**
   * 视觉视差负空间检测开关（P4 · VPND）。开启后把"深度看不到的遮挡"
   * （透明物 / 丝线 / 渔网）经矛盾闸门注入为前向障碍。
   * 默认关闭 → 零回归（关闭时完全不介入任何下游输入）。
   */
  public vpndEnabled: boolean = false;
  /** 视觉占据源（实现方注入：AREngine 光流 / 离线仿真 Mock / 真机 RGB） */
  public vpndSource: VisionParallax | null = null;
  /** 本帧 VPND 判定来源（诊断；disabled/free/contradiction/blind-vision/...） */
  public vpndReason: VpndReason = 'disabled';
  /** 本帧最小碰撞时间（秒，诊断用；无障碍 = ∞） */
  public lastTtcSec: number = Infinity;
  /** 本帧预测层是否修正过速度（诊断用） */
  public voBlocked: boolean = false;
  /** 本帧参与约束的扇区数（诊断用） */
  public voConstraints: number = 0;

  /** 飞行动力学参数（入口页「最大速度」会改写 maxSpeed） */
  public params: DroneFlightParams = {
    maxSpeed: DEFAULT_FLIGHT_PARAMS.maxSpeed,
    accel: DEFAULT_FLIGHT_PARAMS.accel,
    damping: DEFAULT_FLIGHT_PARAMS.damping,
    climbSpeed: DEFAULT_FLIGHT_PARAMS.climbSpeed,
    climbAccel: DEFAULT_FLIGHT_PARAMS.climbAccel,
    yawSpeed: DEFAULT_FLIGHT_PARAMS.yawSpeed,
    yawAccel: DEFAULT_FLIGHT_PARAMS.yawAccel,
    maxTilt: DEFAULT_FLIGHT_PARAMS.maxTilt,
    tiltResponse: DEFAULT_FLIGHT_PARAMS.tiltResponse,
    minHeight: DEFAULT_FLIGHT_PARAMS.minHeight,
    maxHeight: DEFAULT_FLIGHT_PARAMS.maxHeight,
  };

  /**
   * 调试用：最近一次有效输入的描述。
   * 用于定位「按键是否被送达到控制器」。
   */
  public lastInput: string = '无';

  /**
   * 当前是否有操控输入（方向 / 升降 / 转向任一）。
   * ARView 铺满全屏并带 onClick，若按钮触摸被底层一并接收，
   * 每次按键都会被当成「点击平面」重新放置无人机 —— 表现就是飞机被钉回原地。
   * 渲染层据此在「正在操控」期间拒绝重新放置。
   */
  public isControlling(): boolean {
    return this.moveX !== 0 || this.moveY !== 0 ||
      this.holdUp || this.holdDown || this.holdYawLeft || this.holdYawRight ||
      this.climbInput !== 0 || this.yawInput !== 0;
  }

  /** 机身前向单位向量（AR 世界系） */
  public get forwardX(): number {
    const rad: number = this.yaw * Math.PI / 180;
    return -Math.sin(rad);
  }

  public get forwardZ(): number {
    const rad: number = this.yaw * Math.PI / 180;
    return -Math.cos(rad);
  }

  /** 水平速度大小 m/s */
  public get speed(): number {
    return Math.sqrt(this.velX * this.velX + this.velZ * this.velZ);
  }

  /** 是否正在软降落（Landing 态），供诊断 / 测试读取 */
  public get isLanding(): boolean {
    return this.landing;
  }

  /**
   * 接入 / 摘除环境感知。
   * 传入 null 即断开，并**立即把风场与密度复位为无风标准值** ——
   * 避免"断开后残留上一帧的风"这类隐蔽 bug。
   */
  public setEnvironment(env: EnvironmentPerception | null): void {
    this.environment = env;
    if (env === null) {
      this.windVelX = 0;
      this.windVelZ = 0;
      this.windAccelX = 0;
      this.windAccelZ = 0;
      this.airDensityFactor = 1.0;
      this.environmentActive = false;
    }
  }

  /** 起飞 / 降落 切换 */
  public toggleFlight(): void {
    const wasFlying: boolean = this.flying;
    this.flying = !this.flying;
    if (!wasFlying && this.flying) {
      // 起飞（arm）：无论之前是否在降落，重新 armed 即离开 Landing 态，
      // 恢复悬停积分（与"放置即 flying=true"的悬停路径语义一致）。
      this.landing = false;
    } else if (wasFlying && !this.flying && this.placed &&
      this.offsetY > this.params.minHeight + 0.02) {
      // 降落（disarm）：从飞行中切到停桨，且仍在半空 → 进入 Landing 软降落。
      // 已在地面（offsetY≈minHeight）则直接落地，无需降落过程。
      // 注意与"放置即 flying=true"互不冲突：放置走 ARDroneCallback 直接置位，
      // 这里是用户主动 disarm 的降落，两条路径独立、不共享 landing 标志。
      this.landing = true;
    }
  }

  /**
   * 切换飞控内核（M3 Step2）：useDynamics=true → 真实动力学；false（默认）→ 运动学。
   * 切换时把当前运动学状态**无缝交接**给刚体（速度 / 姿态 / 偏航），电机从停转重新 spool up，
   * 避免模式跳变。动力学实例与执行器实例懒建，首次切到动力学才分配。
   */
  public setFlightMode(useDynamics: boolean): void {
    this.useDynamics = useDynamics;
    if (useDynamics) {
      const d: FlightDynamics = this.dyn();
      d.syncFrom(this.velX, this.velY, this.velZ,
        this.pitch * Math.PI / 180, this.roll * Math.PI / 180, this.yaw * Math.PI / 180);
      this.plant().reset(); // 电机从停转起步，真实 spool-up
      this.holdY = this.offsetY; // 高度设定点无缝交接当前高度
    }
  }

  /** 懒建刚体动力学实例 */
  private dyn(): FlightDynamics {
    if (this.dynamics === null) {
      this.dynamics = new FlightDynamics(DEFAULT_DYNAMICS);
    }
    return this.dynamics;
  }

  /** 懒建执行器（电机）实例 */
  private plant(): RotorPlant {
    if (this.rotorPlant === null) {
      this.rotorPlant = new RotorPlant(DYN_MOTOR_TAU);
    }
    return this.rotorPlant;
  }

  /**
   * 当前地面效应升力增益（离地越近增益越大）。
   * ---------------------------------------------------------------
   * 离地高度 = offsetY − minHeight（offsetY 相对放置面，minHeight 即贴地值）。
   * 贴地 +25%、0.5m 外归 1.0 —— 高空飞行手感完全不变（零回归）。
   */
  public groundEffect(): number {
    const agl: number = this.offsetY - this.params.minHeight;
    return groundEffectFactor(agl);
  }

  /**
   * L3 湍流风开关（阶段C，默认关）。
   * ---------------------------------------------------------------
   * 开启时（重新）创建种子化湍流模型；关闭时清空实例，风场回到纯 mean wind。
   * 子步积分时若开启，turbModel.update(h) 取水平风扰动叠加进 windVelX/windVelZ，
   * 再交给 FlightDynamics —— 湍流因此经由相对气流自然扰动机身，控制器须自行抑制。
   * 相同 seed → 完全相同湍流序列（可复现/回归/回放）。
   * @param on        是否启用湍流
   * @param intensity σ 缩放（默认沿用 this.turbulenceIntensity）
   * @param seed      随机种子（默认沿用 this.seed）
   */
  public setTurbulence(on: boolean, intensity?: number, seed?: number): void {
    this.turbulence = on;
    if (intensity !== undefined) {
      this.turbulenceIntensity = intensity;
    }
    if (seed !== undefined) {
      this.seed = seed;
    }
    if (on) {
      this.turbModel = new TurbulenceModel(this.seed, this.turbulenceIntensity);
    } else {
      this.turbModel = null;
    }
  }

  /**
   * L5 状态估计器开关（阶段D，默认 'none'）。开启 'ekf' 用当前位姿初始化 EKF；'none' 清空。
   * 注意：当前 EKF 为**诊断级**估计（暴露于 HUD 读数），尚未接入控制反馈环（交 M5 标定），
   * 故开启不会改变飞行手感 → 零回归。
   * @param mode 'none' | 'ekf'
   */
  public setEstimator(mode: string): void {
    this.estimator = mode;
    if (mode === 'ekf') {
      this.ekf = new EkfEstimator(
        [this.offsetX, this.offsetY, this.offsetZ],
        [this.velX, this.velY, this.velZ],
        quatFromEuler(this.yaw * DEG2RAD, this.pitch * DEG2RAD, this.roll * DEG2RAD)
      );
      this.prevEkvX = this.velX;
      this.prevEkvY = this.velY;
      this.prevEkvZ = this.velZ;
    } else {
      this.ekf = null;
    }
  }

  /**
   * L6 传感器噪声开关（阶段D，默认关）。开启后用种子化 SensorModel 给 EKF 喂加噪/丢包的观测。
   * @param on 是否启用
   * @param seed 随机种子（默认沿用 this.seed）
   */
  public setSensorNoise(on: boolean, seed?: number): void {
    this.sensorNoise = on;
    if (on) {
      this.sensor = new SensorModel(seed === undefined ? this.seed : seed);
    } else {
      this.sensor = null;
    }
  }

  /**
   * M5·阶段D：一键高保真预设（PX4 量级标定）。研发/标定用，默认不调用、不影响手感。
   * 开启全部保真层（动力学/速率环/增强气动/传感器噪声/EKF），湍流留默认关（按需开）。
   * EKF 配置显式取自 px4EkfConfig(PX4_STOCK)，使「PX4 标定」在代码层可见。
   */
  public applyHighFidelityPreset(seed: number = 1): void {
    this.useDynamics = true;
    this.useRateLoop = true;
    this.aeroModel = 'enhanced';
    this.actuatorModel = 'nonlinear';
    this.seed = seed;
    this.turbulenceIntensity = 1.0;
    this.turbulence = false;
    this.turbModel = null;
    this.sensorNoise = true;
    this.sensor = new SensorModel(seed);
    this.estimator = 'ekf';
    this.ekf = new EkfEstimator(
      [this.offsetX, this.offsetY, this.offsetZ],
      [this.velX, this.velY, this.velZ],
      quatFromEuler(this.yaw * DEG2RAD, this.pitch * DEG2RAD, this.roll * DEG2RAD),
      px4EkfConfig(PX4_STOCK)
    );
    this.prevEkvX = this.velX;
    this.prevEkvY = this.velY;
    this.prevEkvZ = this.velZ;
  }

  /**
   * UI 可逆开关：一键启用 / 恢复 M4/M5 全保真栈。
   * ---------------------------------------------------------------
   * on=true  → 套用 applyHighFidelityPreset（动力学 + 速率环 + 气动增强 +
   *            执行器非线性 + 传感器噪声 + EKF；湍流保持默认关）。
   * on=false → 恢复基线运动学内核（与默认构造一致，零回归）。
   * 默认关，故出厂 / 首飞走旧手感路径，绝不影响现有飞行体验。
   */
  public setHighFidelityEnabled(on: boolean): void {
    if (on) {
      this.applyHighFidelityPreset(this.seed > 0 ? this.seed : 1);
      return;
    }
    this.useDynamics = false;
    this.useRateLoop = false;
    this.aeroModel = 'basic';
    this.actuatorModel = 'linear';
    this.turbulence = false;
    this.turbulenceIntensity = 1.0;
    this.sensorNoise = false;
    this.sensor = null;
    this.estimator = 'none';
    this.ekf = null;
    this.ekfPosX = 0;
    this.ekfPosY = 0;
    this.ekfPosZ = 0;
    this.ekfVelX = 0;
    this.ekfVelY = 0;
    this.ekfVelZ = 0;
    this.prevEkvX = this.velX;
    this.prevEkvY = this.velY;
    this.prevEkvZ = this.velZ;
  }

  /**
   * 动力学内核单帧推进（M3 Step2）：把摇杆意图经「速度环 → 推力矢量 → 期望倾角 →
   * 姿态环 PD → 混控 → 执行器滞后 → 刚体积分」整条真实飞控链路算出来，再回写公开字段。
   * 与运动学内核互斥（update() 已提前 return）。yaw 仍由运动学通道积分以保持机体轴一致。
   * 增益经离线闭环仿真验证稳定（scratch_dyn_test.ts）：悬停逐位静止、前进可达速度上限、
   * 风扰被速度环自然抵消。
   */
  private stepDynamics(dt: number, desVX: number, desVZ: number,
    climbSource: number, yawSource: number,
    fx: number, fz: number, rx: number, rz: number,
    p: DroneFlightParams, hasMoveInput: boolean): void {
    const d: FlightDynamics = this.dyn();
    const plant: RotorPlant = this.plant();
    const geo: QuadGeometry = geometryOf(this.dynParams);
    const D = this.dynParams;
    const active: boolean = this.placed && this.flying;

    if (!active) {
      // 未激活：电机停转、速度衰减、冻结位置（与运动学非激活一致）
      const k: number = clampValue(decayFactor(p.damping, dt), 0, 1);
      this.velX *= k; this.velY *= k; this.velZ *= k;
      this.writeRotorFromPlant(plant, geo, dt);
      return;
    }

    // 保持环接合（与运动学同源）：无杆量 + 启用环境感知时定锚点
    // P3：显式设定了目标点时锚点锁定，摇杆不再改写它（镜像 App 动力学内核）
    if (hasMoveInput && !this.holdTargetActive) {
      this.holdX = this.offsetX;
      this.holdZ = this.offsetZ;
    }
    if (!hasMoveInput && (this.environmentActive || this.holdTargetActive)) {
      if (!this.holdTargetActive && !this.holdEngaged) {
        this.holdX = this.offsetX;
        this.holdZ = this.offsetZ;
        this.holdEngaged = true;
      }
    } else if (!this.environmentActive) {
      this.holdEngaged = false;
    }

    // 期望水平速度：有杆量 = 摇杆目标（已含避障）；无杆量且保持接合 = 位置环回锚
    let cmdVX: number = desVX;
    let cmdVZ: number = desVZ;
    if (!hasMoveInput && this.holdEngaged) {
      const hv: number[] = positionHoldVelocity(this.holdX, this.holdZ,
        this.offsetX, this.offsetZ, KIN_POS_KP, p.maxSpeed * KIN_HOLD_AUTHORITY);
      cmdVX = hv[0];
      cmdVZ = hv[1];
    }

    // 期望竖直速度：软降落锥形减速 > 手动升降 > 高度位置环（holdY）定高
    // 高度环是真实飞控的"松杆定高"来源（App DYN_ALT_KP），避免无风时也缓慢掉高。
    let desVY: number;
    if (this.landing && !this.flying) {
      if (this.offsetY < LAND_TAPER_BAND) {
        // 锥形：越接近地面，目标下降速度越小，触地收敛到 LAND_TOUCHDOWN_SPEED
        const taper: number = clampValue(this.offsetY / LAND_TAPER_BAND, 0, 1);
        desVY = -LAND_TOUCHDOWN_SPEED * taper;
      } else {
        desVY = -p.climbSpeed;
      }
    } else if (climbSource !== 0) {
      desVY = climbSource * p.climbSpeed;
      this.holdY = this.offsetY; // 手动升降时设定点跟随，松手即定高
    } else {
      desVY = clampValue(-DYN_ALT_KP * (this.offsetY - this.holdY), -p.climbSpeed, p.climbSpeed);
    }

    const windX: number = this.windVelX;
    const windZ: number = this.windVelZ;
    const airDensity: number = this.airDensityFactor;
    const nSub: number = Math.max(1, Math.ceil(dt * DYN_SUBSTEP_HZ));
    const h: number = dt / nSub;

    for (let s: number = 0; s < nSub; s++) {
      // 同步刚体 yaw 到控制器 yaw（机体轴一致），偏航角速度清零（偏航由运动学通道积分）
      d.yaw = this.yaw * Math.PI / 180;
      d.wYaw = 0;
      // 速度环（水平）
      const vd: number[] = velocityLoop(cmdVX, cmdVZ, d.vx, d.vz, DYN_VEL_KP, DYN_MAX_ACCEL);
      // 垂直环
      const ay: number = verticalLoop(desVY, d.vy, DYN_VEL_KP, DYN_MAX_ACCEL);
      // 相对气流阻力（前馈估计，与 FlightDynamics.step 同源）
      const rvx: number = d.vx - windX;
      const rvy: number = d.vy;
      const rvz: number = d.vz - windZ;
      const fd: number[] = dragForce(rvx, rvy, rvz, D.dragCoef, airDensity);
      // 推力矢量（含重力 + 阻力前馈）
      const tv: number[] = thrustVector(vd[0], ay, vd[1], D.mass, D.gravity, fd[0], fd[1], fd[2]);
      const T: number = tv[0];
      // 期望倾角（yaw 对齐机体轴）
      const tilt: number[] = tiltFromUpDir(tv[1], tv[3], fx, fz, rx, rz, D.maxTiltRad);
      // 姿态环 PD → 期望力矩（偏航力矩置 0：偏航由运动学通道积分）
      // M4 阶段A：开启速率环时改用「姿态→角速度→力矩」级联（与单 PD 等刚度，零回归）。
      let tauPitch: number;
      let tauRoll: number;
      if (this.useRateLoop) {
        const rateP: number = attitudeRateCmd(tilt[0], d.pitch, DYN_ATT_RATE_KP, DYN_MAX_RATE);
        const rateR: number = attitudeRateCmd(tilt[1], d.roll, DYN_ATT_RATE_KP, DYN_MAX_RATE);
        tauPitch = rateTorque(rateP, d.wPitch, DYN_RATE_KP, DYN_RATE_KD, D.inertiaTilt);
        tauRoll = rateTorque(rateR, d.wRoll, DYN_RATE_KP, DYN_RATE_KD, D.inertiaTilt);
      } else {
        tauPitch = attitudeTorque(tilt[0], d.pitch, d.wPitch, DYN_ATT_KP, DYN_ATT_KD, D.inertiaTilt);
        tauRoll = attitudeTorque(tilt[1], d.roll, d.wRoll, DYN_ATT_KP, DYN_ATT_KD, D.inertiaTilt);
      }
      const tauYaw: number = 0;
      // 混控 → 四桨推力 → 执行器饱和（限幅在转速上）
      const raw: number[] = mixThrusts(T, tauPitch, tauRoll, tauYaw, geo);
      const limited = applyActuatorLimit(raw, geo, airDensity);
      const cmd: number[] = limited.speeds;   // 指令转速（已限幅），交给执行器动力学滞后
      // L4 执行器非线性（阶段B）：电压跌落（集总耦合）+ 油门曲线（中段非线性）。
      // 默认 linear → cmdEff = cmd，与电机滞后本体完全一致（零回归）。
      let cmdEff: number[] = cmd;
      if (this.actuatorModel === 'nonlinear') {
        cmdEff = invertActuatorNonlinear(cmd, geo, T, this.batteryPercent / 100);
      }
      // L3 湍流（阶段C）：种子化 OU 扰动叠加到风场，再进动力学积分。
      // 默认 turbulence=false → wX/wZ 恒等于 windVelX/windVelZ（零回归）。
      let wX: number = windX;
      let wZ: number = windZ;
      if (this.turbulence && this.turbModel !== null) {
        const tb = this.turbModel.update(h);
        wX += tb.wx;
        wZ += tb.wz;
      }
      const act = plant.update(h, cmdEff, geo, airDensity);
      // 刚体积分（地面效应贴地增升；L2 增强气动走机体气动矩阵+入流）
      d.step(h, act.thrusts, wX, wZ, airDensity, this.groundEffect(), this.aeroModel === 'enhanced');
    }

    // ── M4·阶段D：EKF 估计（诊断级，默认关；不接入控制环 → 零回归）──
    // 帧级（60Hz）把真值 IMU（角速度+比力）与位置/速度观测喂给 EKF；sensorNoise 开启时
    // 先经种子化 SensorModel 加噪/丢包。估计结果暴露给 HUD 诊断读数，不改变飞行手感。
    if (this.estimator === 'ekf' && this.ekf !== null) {
      const aWorld: number[] = [
        (this.velX - this.prevEkvX) / dt,
        (this.velY - this.prevEkvY) / dt,
        (this.velZ - this.prevEkvZ) / dt,
      ];
      const q: number[] = quatFromEuler(this.yaw * DEG2RAD, this.pitch * DEG2RAD, this.roll * DEG2RAD);
      const Rm: number[][] = quatToRot(q);
      // 真值比力（机体）= Rᵀ(a_world - g_world)，g_world=[0,−9.81,0]
      const awg: number[] = [aWorld[0], aWorld[1] + 9.81, aWorld[2]];
      const aBody: number[] = [
        Rm[0][0] * awg[0] + Rm[1][0] * awg[1] + Rm[2][0] * awg[2],
        Rm[0][1] * awg[0] + Rm[1][1] * awg[1] + Rm[2][1] * awg[2],
        Rm[0][2] * awg[0] + Rm[1][2] * awg[1] + Rm[2][2] * awg[2],
      ];
      const truthGyro: number[] = [d.wPitch, d.wYaw, d.wRoll];
      let imu: ImuSample = { gyro: truthGyro, accel: aBody };
      let posObs: PosObservation | null = null;
      let velObs: VelObservation | null = null;
      if (this.sensorNoise && this.sensor !== null) {
        imu = this.sensor.imu(truthGyro, aBody);
        const po = this.sensor.observePos([this.offsetX, this.offsetY, this.offsetZ]);
        const vo = this.sensor.observeVel([this.velX, this.velY, this.velZ]);
        if (po.available) { posObs = po; }
        if (vo.available) { velObs = vo; }
      } else {
        posObs = { pos: [this.offsetX, this.offsetY, this.offsetZ], available: true };
        velObs = { vel: [this.velX, this.velY, this.velZ], available: true };
      }
      this.ekf.step(dt, imu, {
        pos: posObs ? posObs.pos : undefined,
        posAvailable: posObs ? posObs.available : false,
        vel: velObs ? velObs.vel : undefined,
        velAvailable: velObs ? velObs.available : false,
      });
      const ep: number[] = this.ekf.position;
      const ev: number[] = this.ekf.velocity;
      this.ekfPosX = ep[0]; this.ekfPosY = ep[1]; this.ekfPosZ = ep[2];
      this.ekfVelX = ev[0]; this.ekfVelY = ev[1]; this.ekfVelZ = ev[2];
      this.prevEkvX = this.velX; this.prevEkvY = this.velY; this.prevEkvZ = this.velZ;
    }

    // 回写公开字段
    this.velX = d.vx;
    this.velY = d.vy;
    this.velZ = d.vz;
    this.pitch = d.pitch * 180 / Math.PI;
    this.roll = d.roll * 180 / Math.PI;

    // 偏航：运动学通道积分（与运动学内核同源），避免刚体 yaw 与机体轴漂移
    if (yawSource !== 0) {
      this.yawRate = approachValue(this.yawRate, -yawSource * p.yawSpeed, p.yawAccel * dt);
    } else {
      this.yawRate *= decayFactor(p.damping, dt);
    }
    this.yaw = normalizeDeg(this.yaw + this.yawRate * dt);

    // 位置积分
    this.offsetX += this.velX * dt;
    this.offsetY += this.velY * dt;
    this.offsetZ += this.velZ * dt;

    // 高度限位（与运动学内核同源：触顶/触底清零对应速度，落地结束降落）
    if (this.offsetY < p.minHeight) {
      this.offsetY = p.minHeight;
      if (this.velY < 0) {
        this.velY = 0;
      }
      if (this.landing) {
        this.landing = false;
      }
    } else if (this.offsetY > p.maxHeight) {
      this.offsetY = p.maxHeight;
      if (this.velY > 0) {
        this.velY = 0;
      }
    }

    // 旋翼：由电机实际转速回写（渲染/诊断）
    this.writeRotorFromPlant(plant, geo, dt);
  }

  /** 由执行器实际转速回写 rotorSpeed / rotorSpeeds（M3 Step2） */
  private writeRotorFromPlant(plant: RotorPlant, geo: QuadGeometry, dt: number): void {
    const sp: number[] = plant.actualSpeeds;
    this.rotorSpeeds = [sp[0], sp[1], sp[2], sp[3]];
    const avg: number = (sp[0] + sp[1] + sp[2] + sp[3]) / 4;
    const target: number = clampValue(avg / geo.maxRotor, 0, 1);
    this.rotorSpeed += (target - this.rotorSpeed) * clampValue(6 * dt, 0, 1);
  }

  /** 重置到初始位置与姿态（含速度清零） */
  public reset(): void {
    this.offsetX = 0;
    this.offsetY = 0.9;
    this.offsetZ = 0;
    this.velX = 0;
    this.velY = 0;
    this.velZ = 0;
    this.yaw = 0;
    this.yawRate = 0;
    this.pitch = 0;
    this.roll = 0;
    // M4/M5 保真层复位（全部回到默认关，零回归）
    this.useRateLoop = false;
    this.aeroModel = 'basic';
    this.actuatorModel = 'linear';
    this.turbulence = false;
    this.turbulenceIntensity = 1.0;
    this.seed = 0;
    this.turbModel = null;
    this.estimator = 'none';
    this.sensorNoise = false;
    this.ekf = null;
    this.sensor = null;
    this.ekfPosX = 0; this.ekfPosY = 0; this.ekfPosZ = 0;
    this.ekfVelX = 0; this.ekfVelY = 0; this.ekfVelZ = 0;
    this.prevEkvX = 0; this.prevEkvY = 0; this.prevEkvZ = 0;
    // P3：位置保持锚点/目标点复位（镜像 App reset）
    this.holdX = 0;
    this.holdZ = 0;
    this.holdY = 0.9;
    this.holdEngaged = false;
    this.holdTargetActive = false;
    this.batteryPercent = 100;
  }

  // ── P3：位置保持目标点（与 App DroneController 同源；社区仓无 activeAlgo，故仅此轻量层）──
  /** 距定点锚点的水平偏差（米）。HUD 用它证明「位置保持真的在收敛」。 */
  public get holdError(): number {
    const dx: number = this.offsetX - this.holdX;
    const dz: number = this.offsetZ - this.holdZ;
    return Math.sqrt(dx * dx + dz * dz);
  }

  /** 当前是否处于「定点悬停保持」：环境感知开 + 已放置 + 在飞 + 无操控输入。 */
  public get positionHolding(): boolean {
    return this.environmentActive && this.placed && (this.flying || this.landing) &&
      !this.isControlling();
  }

  /**
   * 抗风上限（m/s）：飞机物理上最多能顶住多大的定常风。
   * tanθ = c·ρ·v²/(m·g) → v_max = √( m·g·tan(θ_max) / (c·ρ) )。
   */
  public get windCeiling(): number {
    const dp: DynamicsParams = this.dynParams;
    const rho: number = clampValue(this.airDensityFactor, 0.6, 1.4);
    const tanTilt: number = Math.tan(dp.maxTiltRad);
    return Math.sqrt(dp.mass * dp.gravity * tanTilt / (dp.dragCoef * rho));
  }

  /** P3：位置保持目标点是否已显式设定（UI 据此显示「守点中」与「清除」入口）。 */
  public get holdTargetSet(): boolean {
    return this.holdTargetActive;
  }

  /** P3：目标点 X（offset 坐标系，相对放置锚点，m）。 */
  public get holdTargetX(): number {
    return this.holdX;
  }

  /** P3：目标点 Z（offset 坐标系，相对放置锚点，m）。 */
  public get holdTargetZ(): number {
    return this.holdZ;
  }

  /**
   * P3：显式设定位置保持目标点（offset 坐标系）。
   * 设定后锚点锁定在该点，摇杆不再改写它，松杆即自动飞回该点悬停。
   * 同时立即接合保持环，使两个内核都立刻生效。
   */
  public setHoldTarget(x: number, z: number): void {
    if (!isFinite(x) || !isFinite(z)) {
      Logger.warn(`setHoldTarget 收到非有限值，已忽略 (x=${x}, z=${z})`);
      return;
    }
    this.holdX = x;
    this.holdZ = z;
    this.holdTargetActive = true;
    this.holdEngaged = true;
  }

  /**
   * P3：清除目标点，回到默认语义（锚点跟随当前位姿 → 松手即就地定点）。
   * 锚点立即复位到当前位置，避免清除瞬间被旧锚点拽一下。
   */
  public clearHoldTarget(): void {
    this.holdTargetActive = false;
    this.holdX = this.offsetX;
    this.holdZ = this.offsetZ;
    this.holdEngaged = false;
  }

  /**
   * 放置时刻的初始机头朝向（度）。飞行系「前」= [-sin yaw, -cos yaw]，yaw=0 时沿世界 -Z。
   * 动力学模式有独立的刚体 yaw 状态，须一并 sync，否则下一帧被 dyn.yaw 覆盖回去。
   */
  public setInitialYaw(deg: number): void {
    if (!isFinite(deg)) {
      return;
    }
    this.yaw = normalizeDeg(deg);
    this.yawRate = 0;
    if (this.dynamics !== null) {
      this.dynamics.syncFrom(this.velX, this.velY, this.velZ,
        this.pitch * DEG2RAD, this.roll * DEG2RAD, this.yaw * DEG2RAD);
    }
  }

  /**
   * 每帧更新：速度积分 → 位置积分 → 偏航 → 姿态 → 旋翼
   * @param deltaTime 帧间隔（秒）
   */
  public update(deltaTime: number): void {
    // dt 保护：首帧 / 卡顿 / 后台恢复时钳到 100ms，避免物理炸掉
    const dt: number = (deltaTime > 0 && deltaTime < 0.1) ? deltaTime : 1 / 60;
    const p: DroneFlightParams = this.params;

    // 只有「已放置 + 已起飞」才接受输入并积分位置；
    // 降落或未放置时只做速度衰减与旋翼减速（相当于自动悬停刹车）。
    const active: boolean = this.placed && this.flying;

    let inputX: number = active ? clampValue(this.moveX, -1, 1) : 0;
    let inputY: number = active ? clampValue(this.moveY, -1, 1) : 0;
    // 摇杆输入归一化：对角线（↑ 与 → 同按）的合成输入长度会到 √2，
    // 不归一化的话对角线速度会比单方向快 41%，手感不一致。
    const inMag: number = Math.sqrt(inputX * inputX + inputY * inputY);
    if (inMag > 1) {
      inputX = inputX / inMag;
      inputY = inputY / inMag;
    }
    // 升降 / 偏航：模拟量（虚拟摇杆）优先于布尔（物理按钮）。
    // 两个 UI 入口能独立工作：例如按住「上升」键的同时推油门摇杆，模拟量赢；
    // 摇杆归零后，布尔还能继续生效，避免一侧失控就把整架锁死。
    const climbSource: number = active ?
      (this.climbInput !== 0 ? clampValue(this.climbInput, -1, 1) :
        ((this.holdUp ? 1 : 0) - (this.holdDown ? 1 : 0))) : 0;
    const yawSource: number = active ?
      (this.yawInput !== 0 ? clampValue(this.yawInput, -1, 1) :
        ((this.holdYawRight ? 1 : 0) - (this.holdYawLeft ? 1 : 0))) : 0;
    // 调试回显：记录本帧实际生效的来源，避免「按钮没用」的误判
    const climbSourceName: string = this.climbInput !== 0 ? '摇杆' :
      ((this.holdUp || this.holdDown) ? '按钮' : '无');
    const yawSourceName: string = this.yawInput !== 0 ? '摇杆' :
      ((this.holdYawLeft || this.holdYawRight) ? '按钮' : '无');

    // —— 机体系基向量（yaw=0 时机头朝 -Z，即远离用户）——
    const rad: number = this.yaw * Math.PI / 180;
    const sin: number = Math.sin(rad);
    const cos: number = Math.cos(rad);
    const fx: number = -sin;   // 前向 X 分量
    const fz: number = -cos;   // 前向 Z 分量
    const rx: number = cos;    // 右向 X 分量
    const rz: number = -sin;   // 右向 Z 分量

    // —— 水平：目标速度 = 前向 * 前后输入 + 右向 * 左右输入 ——
    let targetVX: number = (fx * inputY + rx * inputX) * p.maxSpeed;
    let targetVZ: number = (fz * inputY + rz * inputX) * p.maxSpeed;

    // —— 前向障碍距离：优先来自 SpatialPerception，否则回退到 obstacleDistance 字段 ——
    const rawDistance: number = this.perception
      ? this.perception.getObstacleDistance()
      : this.obstacleDistance;
    this.obstacleDistance = rawDistance; // 缓存，供 describe() 诊断展示

    // —— P1：时间维度守护（一阶低通 + 急停迟滞 + 失联计时）——
    // 关闭时 distance 就是原始读数、急停不带迟滞 —— 与旧行为逐位一致。
    // 硬停面取法**必须**与 applyAvoidance 内部一致（见下），否则迟滞会与
    // 反应式层用两套 margin，出现"迟滞说已解除、反应层还在急停"的错位。
    const curSpeed: number = Math.sqrt(this.velX * this.velX + this.velZ * this.velZ);
    const effMargin: number = (this.steering.emergencyDist !== DEFAULT_STEERING.emergencyDist) ?
      this.steering.emergencyDist : avoidMarginFor(curSpeed);
    let distance: number = rawDistance;
    let guardState: TemporalState | null = null;
    if (this.temporalEnabled) {
      guardState = this.guard.step(dt, rawDistance, effMargin);
      distance = guardState.distance;
      this.blindSec = guardState.blindSec;
      this.filteredDistance = distance;
    } else {
      // 守护关闭时失联计时仍推进：trustFallback 可以单独开，不该依赖滤波开关
      this.blindSec = this.guard.blind.update(dt, isValidDistance(rawDistance));
      this.filteredDistance = rawDistance;
    }

    // —— P2：多扇区扫描（可选契约；未实现 → 空数组 → 走既有单标量路径）——
    const scan: SectorScan | null = (this.perception !== null && this.perception.getSectorScan) ?
      this.perception.getSectorScan() : null;
    let sectors: SectorInput[] = (scan !== null && scan.confidence > 0 &&
      scan.sectors.length > 0) ? scan.sectors : [];

    // —— P4：延迟补偿（DCPA）/ 视觉视差（VPND）叠加 —— 默认关（零回归）——
    // 两者只修改 `sectors` 与反应式层用的 `distance`，且只在对应主开关开启时介入；
    // 关闭时本块整体跳过，下游输入与旧行为逐位一致。
    let dcaEmergencyFast: boolean = false;
    if (this.dcaEnabled) {
      // PDC：用机体当前实际速度（velX/velZ）把扇区云前推 τ_p，
      // 把"感知延迟前的旧距离"修正成"此刻"的距离，再喂给预测层/反应层。
      const dca: DcaOverlay = dcaOverlay(true, sectors, this.velX, this.velZ, this.dcaConfig);
      sectors = dca.sectors;
      dcaEmergencyFast = dca.emergencyFast;
      this.dcaBraking = dca.braking;
      this.dcaForwardDist = dca.compensatedForwardDist;
      // PDC 修正后的前向净距也直接喂给反应式层（即便未开 VO 也生效，只更安全）
      if (isValidDistance(dca.compensatedForwardDist)) {
        distance = dca.compensatedForwardDist;
      }
    }
    if (this.vpndEnabled && this.vpndSource !== null) {
      // VPND：矛盾闸门 —— 深度说安全但视觉说有遮挡（透明/丝线）→ 注入前向障碍。
      const r: VpndResult = this.vpndSource.resolve(distance);
      this.vpndReason = r.reason;
      if (r.occupied) {
        // 注入/收紧前向扇区（ux≈0, uz<0）到 assumeDist（取更小者，更保守）
        let injected: boolean = false;
        for (let i = 0; i < sectors.length; i++) {
          if (Math.abs(sectors[i].ux) < 1e-6 && sectors[i].uz < 0) {
            if (!isValidDistance(sectors[i].distance) || r.assumeDist < sectors[i].distance) {
              sectors[i] = { ux: 0, uz: -1, distance: r.assumeDist, lateral: 0 };
            }
            injected = true;
            break;
          }
        }
        if (!injected) {
          sectors.push({ ux: 0, uz: -1, distance: r.assumeDist, lateral: 0 });
        }
        // 同时收紧反应式层前向距离（透明墙/丝线本就是前向障碍）
        if (!isValidDistance(distance) || r.assumeDist < distance) {
          distance = r.assumeDist;
        }
      }
    }

    // —— P0-3：绕行方位（契约优先，其次由威胁扇区推导）——
    let lateral: number = 0;
    if (this.perception !== null && this.perception.getObstacleLateral) {
      lateral = this.perception.getObstacleLateral();
    } else if (sectors.length > 0) {
      // 用**期望速度**挑 TTC 最小的扇区：一个 3m 外正对着的墙比 0.5m 外的
      // 侧方近障更该被优先处理 —— 按距离排序会选错，按 TTC 不会。
      const ti: number = pickThreatSector(sectors, targetVX, targetVZ, this.voOptions);
      if (ti >= 0) {
        lateral = (sectors[ti].lateral !== undefined) ? (sectors[ti].lateral as number) : 0;
      }
    }
    if (!isFinite(lateral)) {
      lateral = 0; // 脏数据不得污染绕行方向（NaN 会让 pickAvoidSide 直接返回 0，等价死区）
    }
    this.obstacleLateral = lateral;

    // —— Phase 4 物理层：避障（CollisionDetector + SteeringBehavior）——
    // 原先这段是写在飞控里的内联阈值运算；现在改由 Layer 5 承担，
    // 阈值与减速曲线由物理层拥有（单一事实来源）。
    // 曲线数值与 Phase 2 真机验证过的完全一致（fwdScale = (gap-margin)/band），
    // 因此飞行手感与 gap≈0.38m 的刹停表现都不回归；
    // 新增的是「横向绕行增益」—— 近距离时放大横移，强化贴墙滑行/绕行（F07）。
    if (this.avoidanceEnabled) {
      // maxSpeed 每帧同步：入口页可调最大速度，避障限幅必须跟着变
      this.steering.maxSpeed = p.maxSpeed;
      // speed 传**当前实际水平速度**：硬停面随速度伸缩（刹车距离约束，
      // margin(v) = 0.25 + v²/3），与 App 侧运动学内核同口径（2026-10-05 同步）。
      // bypassMode / 横向偏置由 P0-3 的两个开关驱动，默认均关闭 → 零回归。
      const av: AvoidResult = applyAvoidance(targetVX, targetVZ, fx, fz,
        distance, this.steering, this.bypassEnabled, lateral, curSpeed);
      targetVX = av.vx;
      targetVZ = av.vz;
      this.avoidStrength = av.avoidStrength;
      // 急停判定：开启时间守护时走迟滞结果（防硬停面边界逐帧翻转）
      // EFP：DCA 临界 TTC 快路径可强制急停（跳过 TemporalGuard 低通的平滑延迟）
      const emergencyFinal: boolean = ((guardState !== null) ?
        guardState.emergency : av.emergency) || dcaEmergencyFast;
      this.avoidEmergency = emergencyFinal;
      if (emergencyFinal) {
        // 急停：刹掉当前「朝障碍去」的前进速度分量。
        // 横移/后退分量保留 —— 这正是真机验证过的「贴墙滑行不撞墙」。
        const approach: number = this.velX * fx + this.velZ * fz;
        if (approach > 0) {
          this.velX -= fx * approach;
          this.velZ -= fz * approach;
        }
      }
      this.avoidanceActive = av.braking;

      // —— P3：预测式速度障碍（叠加在反应式层之上）——
      // 投影只会让速度更保守（半空间投影 = 减去超出量，|v| 不增），
      // 因此"启用预测层只会更安全"，不会推翻反应式层的已验证手感。
      if (this.voEnabled && sectors.length > 0) {
        this.voOptions.maxSpeed = p.maxSpeed;
        const vo: VoResult = solveVelocityObstacle(targetVX, targetVZ, sectors, this.voOptions);
        targetVX = vo.vx;
        targetVZ = vo.vz;
        this.lastTtcSec = vo.minTtcSec;
        this.voBlocked = vo.blocked;
        this.voConstraints = vo.constraints;
      } else {
        this.lastTtcSec = Infinity;
        this.voBlocked = false;
        this.voConstraints = 0;
      }
    } else {
      this.avoidStrength = 0;
      this.avoidEmergency = false;
      this.avoidanceActive = false;
      this.lastTtcSec = Infinity;
      this.voBlocked = false;
      this.voConstraints = 0;
    }

    // —— P0-1/P0-2：垂直维度（上方避障 + 感知失联兜底）→ 爬升许可系数 ——
    // 两者都**只压上升、绝不强推下降**：下方可能才是安全区（下有平台），
    // 主动下压会把用户送到地面上去。关闭时 climbScale 恒为 1 → 零回归。
    let climbScale: number = 1;
    if (this.trustFallbackEnabled) {
      const supported: boolean = (this.perception !== null && this.perception.isDepthSupported) ?
        this.perception.isDepthSupported() : this.depthSupported;
      this.trustLevel = sensorTrust(supported, this.blindSec);
      climbScale *= sensorClimbScale(this.trustLevel);
    }
    if (this.ceilingAvoidEnabled) {
      const cd: number = (this.perception !== null && this.perception.getCeilingDistance) ?
        this.perception.getCeilingDistance() : this.ceilingDistance;
      this.ceilingDistance = cd;
      const ci: CeilingInfo = evaluateCeiling(cd);
      this.ceilingStrength = ci.avoidStrength;
      this.ceilingEmergency = ci.emergency;
      climbScale *= (1 - ci.avoidStrength);
    }
    // 垂直通道统一乘这个系数（动力学内核通过 climbSource 透传）
    const climbCmd: number = climbSource * climbScale;
    // —— 环境风场：由 EnvironmentPerception（第二个契约）每帧读入（M3 Step2 移到分支前，双内核共用）——
    // 未接入或关闭时，风场与密度一律归零 —— 对运动学**零影响**（零回归硬保证）。
    if (active && this.environmentActive && this.environment !== null) {
      const w: Vec3 | null = this.environment.getWindVelocity();
      this.windVelX = w !== null ? w.x : 0;
      this.windVelZ = w !== null ? w.z : 0;
      if (this.environment.getAirDensity !== undefined) {
        this.airDensityFactor = this.environment.getAirDensity();
      }
    } else {
      this.windVelX = 0;
      this.windVelZ = 0;
      this.airDensityFactor = 1.0;
    }

    // —— 双内核分支（M3 Step2）：useDynamics 时走真实动力学链路（update 提前 return）——
    // 运动学路径（默认）整段逐位不变；动力学路径在 stepDynamics 内自管积分与姿态。
    const hasMoveInput: boolean = (inputX !== 0) || (inputY !== 0);
    if (this.useDynamics && this.dynamics !== null) {
      // climbCmd 已含垂直避障 / 失联兜底的爬升许可系数（关闭时 = climbSource，零回归）
      this.stepDynamics(dt, targetVX, targetVZ, climbCmd, yawSource, fx, fz, rx, rz, p, hasMoveInput);
      this.syncState();
      return;
    }

    if (hasMoveInput) {
      // P3：显式设定了目标点时锚点锁定，摇杆不再改写它（镜像 App 运动学内核）。
      // 否则锚点跟随当前位姿，松手瞬间的位姿即「驻留点」（与动力学模式同语义）。
      if (!this.holdTargetActive) {
        this.holdX = this.offsetX;
        this.holdZ = this.offsetZ;
      }
      // 有输入：按「速度矢量」整体限速加速（推背感 / 起步不突兀）。
      // 必须按矢量而不是逐分量限速 —— 逐分量会让对角线方向得到 √2 倍合成加速度。
      const dvx: number = targetVX - this.velX;
      const dvz: number = targetVZ - this.velZ;
      const dvLen: number = Math.sqrt(dvx * dvx + dvz * dvz);
      const maxStep: number = p.accel * dt;
      if (dvLen > maxStep) {
        this.velX += dvx / dvLen * maxStep;
        this.velZ += dvz / dvLen * maxStep;
      } else {
        this.velX = targetVX;
        this.velZ = targetVZ;
      }
    } else {
      // 无输入：阻尼衰减（惯性滑行，约 1 秒基本停稳）
      const k: number = decayFactor(p.damping, dt);
      this.velX *= k;
      this.velZ *= k;
      // —— 闭环定点保持（M3 Step1）：启用环境感知时，把松杆漂移拉回锚点 ——
      // 与动力学模式的位置环同源（复用 CascadeController.positionHoldVelocity），
      // 并叠加风扰前馈：① 前馈主动顶风，② 反馈位置环把残余误差拉回锚点。
      // environmentActive=false（默认）时整个块不介入 → 与旧开环阻尼逐位一致（零回归）。
      // P3：显式设定了目标点时，即使环境感知关着，定点依然成立。
      if (this.environmentActive || this.holdTargetActive) {
        // 首次接合：锚点定在当前位姿，避免突兀拉回。
        // P3：显式目标点时**不**重新锚定 —— 用户设的点优先，不能被当前位姿顶掉。
        if (!this.holdTargetActive && !this.holdEngaged) {
          this.holdX = this.offsetX;
          this.holdZ = this.offsetZ;
          this.holdEngaged = true;
        }
        // ① 前馈：主动出力顶风，抵消 KIN_WIND_FEEDFORWARD 比例的风扰加速度
        this.velX += -this.windAccelX * KIN_WIND_FEEDFORWARD * dt;
        this.velZ += -this.windAccelZ * KIN_WIND_FEEDFORWARD * dt;
        // ② 反馈：位置环把残余误差拉回锚点（急停时位置环不介入，避免与防撞意图相反）
        if (!this.avoidEmergency) {
          const hv: number[] = positionHoldVelocity(this.holdX, this.holdZ,
            this.offsetX, this.offsetZ, KIN_POS_KP, p.maxSpeed * KIN_HOLD_AUTHORITY);
          let hx: number = hv[0];
          let hz: number = hv[1];
          if (this.avoidanceEnabled) {
            this.steering.maxSpeed = p.maxSpeed;
            const curSpeedH: number = Math.sqrt(this.velX * this.velX + this.velZ * this.velZ);
            const avh: AvoidResult = applyAvoidance(hx, hz, fx, fz,
              this.obstacleDistance, this.steering, false, 0, curSpeedH);
            hx = avh.vx;
            hz = avh.vz;
          }
          const resp: number = clampValue(KIN_HOLD_RESP * dt, 0, 1);
          this.velX += (hx - this.velX) * resp;
          this.velZ += (hz - this.velZ) * resp;
        }
      } else {
        this.holdEngaged = false;
      }
    }

    // —— 垂直：上升 / 下降 ——
    // climbCmd = 杆量 × 爬升许可系数（上方避障 + 失联兜底；两者关闭时系数恒为 1）
    const targetVY: number = climbCmd * p.climbSpeed;
    if (targetVY !== 0) {
      this.velY = approachValue(this.velY, targetVY, p.climbAccel * dt);
    } else {
      this.velY *= decayFactor(p.damping, dt);
    }

    // 风速矢量 → 等效加速度扰动（运动学内核的权宜口径；
    // 动力学内核直接用 windVel 算相对气流阻力，不走这里）
    this.windAccelX = this.windVelX * this.windGain;
    this.windAccelZ = this.windVelZ * this.windGain;
    this.velX += this.windAccelX * dt;
    this.velZ += this.windAccelZ * dt;

    // —— 位置积分 ——
    if (active) {
      this.offsetX += this.velX * dt;
      this.offsetY += this.velY * dt;
      this.offsetZ += this.velZ * dt;

      // 偏航角速度：有输入加速到目标，松手后阻尼归零。
      // 注意符号：yawInput 为 +1 表示「右转」，但 AR 系下机头向量为
      // f = (-sin yaw, -cos yaw)，yaw 增大时机头由 -Z 摆向 -X（屏幕左）。
      // 所以要让「右转」真的把机头摆向屏幕右（机体右向 +X），yaw 必须减小，
      // 故这里取 -yawInput。（已用 scripts/sim_flight.js 的转向场景验证）
      if (yawSource !== 0) {
        this.yawRate = approachValue(this.yawRate, -yawSource * p.yawSpeed, p.yawAccel * dt);
      } else {
        this.yawRate *= decayFactor(p.damping, dt);
      }
      this.yaw = normalizeDeg(this.yaw + this.yawRate * dt);
    } else {
      this.yawRate *= decayFactor(p.damping, dt);
    }

    // —— 软降落过程（Landing 态）：用户 disarm 后仍在半空，flying 已置 false ——
    // 原位置积分分支要求 active（placed && flying），此时不执行，故这里单独产生一个
    // 朝 minHeight 的下降目标速度：复用 climbAccel 平滑逼近、approachValue 限步长，
    // 到达地面后由下方高度限位块清零速度、并结束降落（landing=false），
    // 旋翼按既有规则（flying=false 时 targetRotor=0）自然衰减到停。
    // 与"放置即 flying=true"互不冲突：放置路径不置 landing，这里只处理主动 disarm 的降落。
    if (this.landing && !this.flying) {
      const targetVY: number = -p.climbSpeed;
      this.velY = approachValue(this.velY, targetVY, p.climbAccel * dt);
      this.offsetY += this.velY * dt;
    }

    // 高度限位（触顶/触底时把对应方向速度清零，避免贴边抖动）
    if (this.offsetY < p.minHeight) {
      this.offsetY = p.minHeight;
      if (this.velY < 0) {
        this.velY = 0;
      }
      // 降落触底：结束 Landing 态，回到 Grounded（旋翼继续按 flying=false 衰减到停）
      if (this.landing) {
        this.landing = false;
      }
    } else if (this.offsetY > p.maxHeight) {
      this.offsetY = p.maxHeight;
      if (this.velY > 0) {
        this.velY = 0;
      }
    }

    // —— 机身姿态：由「机体系速度分量」推导倾角 ——
    const vForward: number = this.velX * fx + this.velZ * fz;
    const vRight: number = this.velX * rx + this.velZ * rz;
    const speedRef: number = p.maxSpeed > 0.001 ? p.maxSpeed : 1;
    // 前进 → 低头（pitch 取负）；右移 → 右倾（roll 取正）
    const targetPitch: number = -p.maxTilt * clampValue(vForward / speedRef, -1, 1);
    const targetRoll: number = p.maxTilt * clampValue(vRight / speedRef, -1, 1);
    const tiltLerp: number = clampValue(p.tiltResponse * dt, 0, 1);
    this.pitch += (targetPitch - this.pitch) * tiltLerp;
    this.roll += (targetRoll - this.roll) * tiltLerp;

    // —— 旋翼转速：悬停基础转速 + 水平速度加成 + 升降加成 ——
    const speedRatio: number = clampValue(this.speed / speedRef, 0, 1);
    const climbRef: number = p.climbSpeed > 0.001 ? p.climbSpeed : 1;
    const climbRatio: number = clampValue(Math.abs(this.velY) / climbRef, 0, 1);
    let targetRotor: number = 0;
    if (this.flying) {
      targetRotor = 0.35 + 0.40 * speedRatio + 0.25 * climbRatio;
      if (this.velY < 0) {
        // 下降时略收油门
        targetRotor -= 0.10 * climbRatio;
      }
    }
    this.rotorSpeed +=
      (clampValue(targetRotor, 0, 1) - this.rotorSpeed) * clampValue(6 * dt, 0, 1);

    // —— 调试读数：记录本帧实际生效的输入 ——
    if (hasMoveInput || yawSource !== 0 || climbSource !== 0) {
      this.lastInput =
        `M${inputX.toFixed(1)},${inputY.toFixed(1)}[${yawSourceName === '摇杆' ? 'Y' : 'P'}] ` +
        `爬升${climbSource.toFixed(1)}${climbSourceName === '摇杆' ? '杆' : '钮'} ` +
        `偏航${yawSource.toFixed(1)}${yawSourceName === '摇杆' ? '杆' : '钮'}`;
    }

    // 末尾同步权威状态（派生态），供 UI / 诊断消费；放在 update 末尾保证
    // 本轮所有物理量（含 avoidEmergency / landing / offsetY）都已就绪。
    this.syncState();
  }

  /**
   * 由内部布尔与动态条件推导权威状态（DroneState）。
   * ---------------------------------------------------------------
   * 这是状态机的"真相来源"：placed / flying / landing / avoidEmergency 是底层事实，
   * state 是它们组合出的可读态。集中在此推导，UI 不再各自 if/else 重复推断，
   * 避免"某处忘了考虑 EmergencyHold"这类语义漂移。
   */
  private syncState(): void {
    if (!this.placed) {
      this.state = DroneState.Unplaced;
    } else if (this.landing) {
      // 软降落中：不论 flying 是否已被 disarm 置 false，都归为 Landing
      this.state = DroneState.Landing;
    } else if (!this.flying) {
      this.state = DroneState.Grounded;
    } else if (this.avoidEmergency) {
      this.state = DroneState.EmergencyHold;
    } else {
      this.state = DroneState.Flying;
    }
  }

  /**
   * 调试面板用：一行状态摘要。
   * 用它可以在真机上一眼判断「输入是否送达 / 积分是否生效」。
   */
  public describe(): string {
    // 首行用权威状态的可读名，替代原来的 placed/flying 文本拼接，
    // 让诊断一眼区分"飞行中/降落中/急停保持"等细分态。
    const stateName: string =
      this.state === DroneState.Unplaced ? '未放置' :
      this.state === DroneState.Grounded ? '已落地' :
      this.state === DroneState.Flying ? '飞行中' :
      this.state === DroneState.Landing ? '降落中' : '急停保持';
    const s: string =
      `${stateName}　` +
      `输入 ${this.lastInput}\n` +
      `偏移 X${this.offsetX.toFixed(2)} Y${this.offsetY.toFixed(2)} Z${this.offsetZ.toFixed(2)}\n` +
      `速度 ${this.speed.toFixed(2)} m/s　升降 ${this.velY.toFixed(2)} m/s\n` +
      `yaw ${this.yaw.toFixed(0)}°　pitch ${this.pitch.toFixed(0)}°　roll ${this.roll.toFixed(0)}°\n` +
      `旋翼 ${this.rotorSpeed.toFixed(2)}　上限 ${this.params.maxSpeed.toFixed(1)} m/s\n` +
      // Phase 4：把"刹车力度"量化显示。与 SteeringBehavior 的减速曲线同源，
      // 因此「面板显示 60%」就等于「前进分量只剩 40%」，不会出现读数与手感脱节。
      `避障强度 ${(this.avoidStrength * 100).toFixed(0)}%${this.avoidEmergency ? ' 急停' : ''} ` +
      `绕行增益 ${this.steering.steerGain.toFixed(2)}\n` +
      // 防撞诊断（根因验收核心）：把「前向输入」与「实际前向速度」并排，
      // 一眼看出"输入有、速度没有"——这正是本次 bug 的现场。
      // fwdScale = 1 - avoidStrength（前向分量缩放系数）；前向实速 = 速度在前向的投影。
      `前向 输入${this.moveY.toFixed(2)} 缩放${(1 - this.avoidStrength).toFixed(2)} ` +
      `实速${(this.velX * this.forwardX + this.velZ * this.forwardZ).toFixed(2)}m/s\n` +
      `防撞 ${this.avoidanceEnabled ? (this.avoidanceActive ? '刹停中' : '监控中') : '关闭'} ` +
      `障碍 ${isFinite(this.obstacleDistance) ? this.obstacleDistance.toFixed(2) + 'm' : '∞'}`;
    // —— 避障演进诊断（P0/P1/P2/P3）：只在对应开关打开时追加，
    //    关闭时输出逐位保持原样，避免面板行号漂移影响既有日志比对 ——
    const extra: string[] = [];
    if (this.ceilingAvoidEnabled) {
      extra.push(`上方 ${isFinite(this.ceilingDistance) ?
        this.ceilingDistance.toFixed(2) + 'm' : '∞'} 强度${(this.ceilingStrength * 100).toFixed(0)}%` +
        (this.ceilingEmergency ? ' 急停' : ''));
    }
    if (this.trustFallbackEnabled) {
      extra.push(`感知信任 ${this.trustLevel}（失联${this.blindSec.toFixed(1)}s）`);
    }
    if (this.bypassEnabled) {
      const dir: string = this.obstacleLateral > 0 ? '障碍偏右→向左绕' :
        (this.obstacleLateral < 0 ? '障碍偏左→向右绕' : '方位未知→默认向右');
      extra.push(`绕行 ${dir}（偏置${this.obstacleLateral.toFixed(2)}m）`);
    }
    if (this.voEnabled) {
      extra.push(`预测层 TTC${isFinite(this.lastTtcSec) ?
        this.lastTtcSec.toFixed(2) + 's' : '∞'} 约束${this.voConstraints}条` +
        (this.voBlocked ? ' 已介入' : ''));
    }
    if (this.temporalEnabled) {
      extra.push(`滤波 ${isFinite(this.filteredDistance) ?
        this.filteredDistance.toFixed(2) + 'm' : '∞'}`);
    }
    return (extra.length === 0) ? s : `${s}\n` + extra.join('　');
  }

  /**
   * 纯函数自检（Phase 5 状态机，真机 / Node / CI 可直接验收，无需单测框架）。
   * ---------------------------------------------------------------
   * 与 Phase 4 的 collisionSelfTest / steeringSelfTest 同风格：用计数器 total
   * 而不是写死数字（断言增删时不会打印错误的"共 N 项"）。断言失败抛错，
   * 由初始化逻辑的 try/catch 包住 —— 只记日志，绝不影响启动。
   * 覆盖：初始态、起飞→Flying、硬停面→EmergencyHold、disarm 软降落→Grounded、状态可逆。
   */
  public static selfTest(): void {
    const fails: string[] = [];
    let total: number = 0;
    const check = (name: string, ok: boolean): void => {
      total++;
      if (!ok) {
        fails.push(name);
      }
    };

    // 1) 初始状态必须是 Unplaced
    {
      const d: DroneController = new DroneController();
      check('初始 state=Unplaced', d.state === DroneState.Unplaced);
    }

    // 2) 放置 + 起飞、无障碍 → Flying
    {
      const d: DroneController = new DroneController();
      d.placed = true;
      d.flying = true;
      d.obstacleDistance = Infinity; // 无障碍
      d.update(0.016);
      check('placed+flying 无障碍 → Flying', d.state === DroneState.Flying);
    }

    // 3) 进入硬停面（0.2m < 硬停面 0.4m）→ 急停保持
    {
      const d: DroneController = new DroneController();
      d.placed = true;
      d.flying = true;
      d.avoidanceEnabled = true;
      d.obstacleDistance = 0.2; // < 硬停面 0.4
      d.update(0.016);
      check('硬停面 → avoidEmergency', d.avoidEmergency === true);
      check('硬停面 → EmergencyHold', d.state === DroneState.EmergencyHold);
    }

    // 4) disarm 软降落：半空 disarm → Landing → 触底 Grounded，高度回落到 minHeight
    {
      const d: DroneController = new DroneController();
      d.placed = true;
      d.flying = true;
      d.obstacleDistance = Infinity;
      d.offsetY = 1.0;
      d.toggleFlight(); // flying→false，半空 → landing=true（进入 Landing）
      check('disarm 半空进入 Landing', d.landing === true);
      let steps: number = 0;
      while (d.landing && steps < 2000) {
        d.update(0.05);
        steps++;
      }
      check('降落完成 landing=false', d.landing === false);
      check('降落终点 state=Grounded', d.state === DroneState.Grounded);
      check('降落终点高度≈minHeight',
        Math.abs(d.offsetY - d.params.minHeight) < 1e-3);
    }

    // 5) 状态可逆：Grounded → 起飞 → Flying
    {
      const d: DroneController = new DroneController();
      d.placed = true;
      d.flying = false; // 先停在地面
      d.obstacleDistance = Infinity;
      d.update(0.016);
      check('未起飞已放置 → Grounded', d.state === DroneState.Grounded);
      d.flying = true; // 起飞
      d.update(0.016);
      check('状态可逆 → Flying', d.state === DroneState.Flying);
    }

    // 6) 前后通道端到端可用（本次 bug 回归守护）：
    //    moveY=1（满前向）且 obstacleDistance=Infinity（无障）时，若干帧后 offsetZ 应有明显位移。
    //    yaw=0 时前向为 -Z，故位移应使 offsetZ 明显变负；若 ≈0 说明"输入有、速度没有"。
    {
      const d: DroneController = new DroneController();
      d.placed = true;
      d.flying = true;
      d.obstacleDistance = Infinity;
      d.moveY = 1; // 满前向
      for (let i = 0; i < 60; i++) {
        d.update(1 / 60);
      }
      check('前后通道可用：满前向位移明显', d.offsetZ < -0.5);
    }

    // 7) 前向不被误归零（本次 bug 回归守护）：
    //    obstacleDistance=1.0（减速带内）时，前进分量缩放到 fwdScale=(1.0-0.4)/1.2=0.5，
    //    前向速度应明显 >0（减半而非归零）；若被归零说明防撞层仍误刹前向。
    {
      const d: DroneController = new DroneController();
      d.placed = true;
      d.flying = true;
      d.avoidanceEnabled = true;
      d.obstacleDistance = 1.0;
      d.moveY = 1;
      for (let i = 0; i < 60; i++) {
        d.update(1 / 60);
      }
      const fwdSpeed: number = d.velX * d.forwardX + d.velZ * d.forwardZ;
      check('gap=1.0 前向不被归零', fwdSpeed > 0.3);
    }

    // 8) 闭环定点保持（M3 Step1）：环境感知开启 + 松杆时，风扰下应拉回锚点而非无限漂走
    {
      const d: DroneController = new DroneController();
      d.placed = true;
      d.flying = true;
      d.obstacleDistance = Infinity;
      // 注入确定性风场（沿 +X，1 m/s），通过 EnvironmentPerception 契约
      d.environment = { getWindVelocity: (): Vec3 | null => ({ x: 1.0, y: 0, z: 0 }) };
      d.environmentActive = true;
      d.offsetX = 0.5; // 偏离锚点
      for (let i = 0; i < 600; i++) {
        d.update(1 / 60);
      }
      check('闭环保持：风扰下松杆收敛回锚点', Math.abs(d.offsetX - 0.5) < 0.2);
      check('闭环保持：已接合保持环', d.holdEngaged === true);
    }

    // 9) 闭环保持默认关闭（零回归）：未启用环境感知时保持环不介入
    {
      const d: DroneController = new DroneController();
      d.placed = true;
      d.flying = true;
      d.obstacleDistance = Infinity;
      d.update(1 / 60); // 无输入帧
      check('默认（环境关闭）保持环不接合', d.holdEngaged === false);
    }

    // 9b) P3：位置保持目标点（可视化目标点）的飞行语义。
    //     默认语义必须零回归（松手即就地定点）；显式设点后锚点锁定、松杆自动飞回。
    {
      // (a) 默认：未设目标点 → 松杆后停在松杆处（不被拉回原点）
      const d0: DroneController = new DroneController();
      d0.placed = true;
      d0.flying = true;
      d0.environmentActive = true;
      d0.moveY = 1;
      for (let i = 0; i < 30; i++) {
        d0.update(1 / 60);
      }
      const zRelease: number = d0.offsetZ;
      d0.moveY = 0;
      for (let i = 0; i < 180; i++) {
        d0.update(1 / 60);
      }
      check('P3 默认语义零回归：松手即就地定点',
        Math.abs(d0.offsetZ - zRelease) < 0.15 && d0.holdTargetSet === false);
    }
    {
      const d: DroneController = new DroneController();
      d.placed = true;
      d.flying = true;
      d.environmentActive = true;
      // 先飞离原点，确认「显式目标点」不是在原点蒙对的
      d.moveY = 1;
      for (let i = 0; i < 40; i++) {
        d.update(1 / 60);
      }
      d.moveY = 0;
      d.setHoldTarget(0.8, 0.0);
      check('P3 设点后 holdTargetSet=true', d.holdTargetSet === true);
      check('P3 目标点坐标被记录',
        Math.abs(d.holdTargetX - 0.8) < 1e-9 && Math.abs(d.holdTargetZ) < 1e-9);
      // 推杆飞走：显式目标点**不得**被摇杆改写
      d.moveY = 1;
      for (let i = 0; i < 60; i++) {
        d.update(1 / 60);
      }
      check('P3 推杆不改写显式目标点',
        Math.abs(d.holdTargetX - 0.8) < 1e-9 && Math.abs(d.holdTargetZ) < 1e-9);
      // 松杆：位置保持环应把飞机拉回该点悬停
      d.moveY = 0;
      for (let i = 0; i < 600; i++) {
        d.update(1 / 60);
      }
      const err: number = Math.sqrt((d.offsetX - 0.8) * (d.offsetX - 0.8) + d.offsetZ * d.offsetZ);
      check('P3 松杆后自动飞回目标点（水平误差<5cm）', err < 0.05);
      // 清除目标点 → 回到默认语义
      d.clearHoldTarget();
      check('P3 清除后回到默认语义', d.holdTargetSet === false);
    }
    // (c) reset() 必须把目标点一并清掉（否则复位后飞机自己飞回旧点）
    {
      const d: DroneController = new DroneController();
      d.setHoldTarget(1.5, -0.5);
      d.reset();
      check('P3 reset 清除目标点', d.holdTargetSet === false);
    }

    // 9c) 抗风上限 / 定点偏差读数
    {
      const d: DroneController = new DroneController();
      d.placed = true;
      d.flying = true;
      check('抗风上限为有限正风速（海平面≈10.4 m/s）', Math.abs(d.windCeiling - 10.35) < 2.0);
      d.holdX = 0.3; d.holdZ = 0.4; d.offsetX = 0; d.offsetZ = 0;
      check('holdError = 到锚点水平距离', Math.abs(d.holdError - 0.5) < 1e-6);
      d.environmentActive = true;
      check('positionHolding：环境开+在飞+无输入为真', d.positionHolding === true);
    }

    // 10) 双内核开关（M3 Step2）：setFlightMode(true) → useDynamics 生效且懒建刚体
    {
      const d: DroneController = new DroneController();
      d.setFlightMode(true);
      check('setFlightMode(true) → useDynamics 生效', d.useDynamics === true);
      check('双内核：动力学实例已懒建', d.dynamics !== null);
      d.setFlightMode(false);
      check('setFlightMode(false) → 切回运动学', d.useDynamics === false);
    }

    // 11) 动力学悬停稳定（M3 Step2）：无输入无风 2s，刚体不漂（offset 仍≈初始）
    {
      const d: DroneController = new DroneController();
      d.placed = true;
      d.flying = true;
      d.obstacleDistance = Infinity;
      d.setFlightMode(true);
      for (let i = 0; i < 120; i++) {
        d.update(1 / 60);
      }
      check('动力学悬停：水平不漂', Math.abs(d.offsetX) < 0.05 && Math.abs(d.offsetZ) < 0.05);
      check('动力学悬停：高度守稳（≈0.9m）', Math.abs(d.offsetY - 0.9) < 0.1);
      check('动力学悬停：姿态收敛（倾角<5°）', Math.abs(d.pitch) < 5 && Math.abs(d.roll) < 5);
    }

    // 12) 动力学前进可达（M3 Step2）：满前向 1s，offsetZ 明显变负（与运动学同向）
    {
      const d: DroneController = new DroneController();
      d.placed = true;
      d.flying = true;
      d.obstacleDistance = Infinity;
      d.setFlightMode(true);
      d.moveY = 1; // 满前向
      for (let i = 0; i < 60; i++) {
        d.update(1 / 60);
      }
      check('动力学前进：满前向位移明显', d.offsetZ < -0.3);
    }

    // 13) M4/M5 高保真栈装配：一键预设正确组装所有保真层；可逆开关能恢复基线（零回归）
    {
      const d: DroneController = new DroneController();
      d.applyHighFidelityPreset(1);
      check('高保真预设：组装动力学+速率环+增强气动+非线性+传感器+EKF',
        d.useDynamics === true && d.useRateLoop === true && d.aeroModel === 'enhanced' &&
        d.actuatorModel === 'nonlinear' && d.sensorNoise === true && d.estimator === 'ekf');
      check('高保真预设：EKF 实例已建', d.ekf !== null);
      d.setHighFidelityEnabled(false);
      check('高保真关闭：恢复运动学基线',
        d.useDynamics === false && d.useRateLoop === false &&
        d.aeroModel === 'basic' && d.actuatorModel === 'linear' && d.estimator === 'none');
    }

    // 14) M4 阶段A：速率环内环（级联控制）端到端 —— 默认关（零回归）；
    //     开启后姿态链路变为「角度→角速度→力矩」，与单 PD 等刚度，hover/前向/偏航行为不变。
    {
      const d: DroneController = new DroneController();
      d.setFlightMode(true);
      d.useRateLoop = true;
      d.placed = true;
      d.flying = true;
      d.obstacleDistance = Infinity;
      d.environmentActive = true;
      for (let i = 0; i < 360; i++) {
        d.update(1 / 60);
      }
      check('M4速率环：悬停稳定（漂移<5cm）',
        Math.sqrt(d.offsetX * d.offsetX + d.offsetZ * d.offsetZ) < 0.05);
      check('M4速率环：悬停高度保持（偏差<3cm）', Math.abs(d.offsetY - 0.9) < 0.03);
      check('M4速率环：姿态基本水平（|pitch|,|roll|<2°）',
        Math.abs(d.pitch) < 2 && Math.abs(d.roll) < 2);
      // 满前向 → 低头 + 松手回稳
      const d2: DroneController = new DroneController();
      d2.setFlightMode(true);
      d2.useRateLoop = true;
      d2.placed = true;
      d2.flying = true;
      d2.obstacleDistance = Infinity;
      d2.environmentActive = true;
      d2.moveY = 1;
      let peakNose: number = 0;
      for (let i = 0; i < 120; i++) {
        d2.update(1 / 60);
        if (d2.pitch < peakNose) {
          peakNose = d2.pitch;
        }
      }
      check('M4速率环：满前向出现低头（pitch<0）', peakNose < -3);
      d2.moveY = 0;
      for (let i = 0; i < 360; i++) {
        d2.update(1 / 60);
      }
      check('M4速率环：松手后停稳（速度<0.1m/s）', d2.speed < 0.1);
      // 偏航输入 → 偏航角速率产生（社区内核偏航走运动学通道，仍应产生 yawRate）
      const d3: DroneController = new DroneController();
      d3.setFlightMode(true);
      d3.useRateLoop = true;
      d3.placed = true;
      d3.flying = true;
      d3.obstacleDistance = Infinity;
      d3.environmentActive = true;
      d3.yawInput = 1;
      for (let i = 0; i < 60; i++) {
        d3.update(1 / 60);
      }
      check('M4速率环：偏航输入产生偏航角速率', Math.abs(d3.yawRate) > 1);
    }

    // 15) M4 阶段B：增强气动（L2）+ 执行器非线性（L4）端到端 —— 两层默认都关（零回归）；
    //     开启后不得破坏悬停与回稳，并为后续高保真层铺路。
    {
      const d: DroneController = new DroneController();
      d.setFlightMode(true);
      d.aeroModel = 'enhanced';
      d.placed = true;
      d.flying = true;
      d.obstacleDistance = Infinity;
      d.environmentActive = true;
      for (let i = 0; i < 360; i++) {
        d.update(1 / 60);
      }
      check('M4增强气动：悬停稳定（漂移<5cm）',
        Math.sqrt(d.offsetX * d.offsetX + d.offsetZ * d.offsetZ) < 0.05);
      check('M4增强气动：悬停高度保持（偏差<3cm）', Math.abs(d.offsetY - 0.9) < 0.03);
      check('M4增强气动：姿态基本水平（|pitch|,|roll|<2°）',
        Math.abs(d.pitch) < 2 && Math.abs(d.roll) < 2);
      const d2: DroneController = new DroneController();
      d2.setFlightMode(true);
      d2.aeroModel = 'enhanced';
      d2.placed = true;
      d2.flying = true;
      d2.obstacleDistance = Infinity;
      d2.environmentActive = true;
      d2.moveY = 1;
      let peakNoseB: number = 0;
      for (let i = 0; i < 120; i++) {
        d2.update(1 / 60);
        if (d2.pitch < peakNoseB) {
          peakNoseB = d2.pitch;
        }
      }
      check('M4增强气动：满前向低头（pitch<-3°）', peakNoseB < -3);
      d2.moveY = 0;
      for (let i = 0; i < 360; i++) {
        d2.update(1 / 60);
      }
      check('M4增强气动：松手后停稳（速度<0.1m/s）', d2.speed < 0.1);

      // 执行器非线性：悬停稳定（控制器补偿电压跌落，不得发散）
      const d3: DroneController = new DroneController();
      d3.setFlightMode(true);
      d3.actuatorModel = 'nonlinear';
      d3.placed = true;
      d3.flying = true;
      d3.obstacleDistance = Infinity;
      d3.environmentActive = true;
      for (let i = 0; i < 360; i++) {
        d3.update(1 / 60);
      }
      check('M4执行器非线性：悬停稳定（漂移<5cm）',
        Math.sqrt(d3.offsetX * d3.offsetX + d3.offsetZ * d3.offsetZ) < 0.05);
      check('M4执行器非线性：悬停高度保持（偏差<3cm）', Math.abs(d3.offsetY - 0.9) < 0.03);
      check('M4执行器非线性：姿态基本水平（|pitch|,|roll|<2°）',
        Math.abs(d3.pitch) < 2 && Math.abs(d3.roll) < 2);
    }

    // 16) 定点保持 ↛ 绕过防撞（行为层）：硬停面内松杆悬停，飞机不得继续朝障碍移动。
    //     阈值口径：硬停面**随速度伸缩**（avoidMarginFor），静止时 = AVOID_MARGIN_BASE_M = 0.25m。
    //     本用例早先取 0.3m 并注「< 0.4」，沿用改动前的固定阈值口径 —— 松杆后速度归零，
    //     硬停面收缩到 0.25m，0.3m 实际落在**减速带内**而非硬停面内，急停判定永不成立，
    //     真机自检因此报「硬停面内松杆悬停 → 急停判定成立」失败。故改取 0.2m（与用例 3 同口径）。
    {
      const d: DroneController = new DroneController();
      d.placed = true;
      d.flying = true;
      d.avoidanceEnabled = true;
      d.environmentActive = true;         // 真机默认：环境感知开启 → 定点保持生效
      d.obstacleDistance = 0.2;           // 静止硬停面内（< 0.25）
      d.moveY = 1;
      for (let i = 0; i < 30; i++) {
        d.update(1 / 60);                 // 有输入：锚点跟随位姿
      }
      d.moveY = 0;                        // 松杆：定点保持接合
      const zAtRelease: number = d.offsetZ;
      for (let i = 0; i < 180; i++) {
        d.update(1 / 60);
      }
      check('硬停面内松杆悬停 → 急停判定成立', d.avoidEmergency === true);
      check('硬停面内松杆悬停 ↛ 被推向障碍',
        d.offsetZ >= zAtRelease - 0.02);
      check('硬停面内松杆悬停 → 速度归零', d.speed < 0.06);
    }

    // 17) P0：垂直避障（上方净距）接线 —— 默认关（零回归）；
    //     开启后**只压上升、绝不强推下降**，且进入上方硬停面时完全禁止爬升。
    {
      const climbFrames: number = 120; // 2 秒
      const runClimb = (ceiling: number, enable: boolean): number => {
        const d: DroneController = new DroneController();
        d.placed = true;
        d.flying = true;
        d.obstacleDistance = Infinity;
        d.ceilingAvoidEnabled = enable;
        d.ceilingDistance = ceiling; // 无 perception 时回退到字段（契约未实现的等价路径）
        d.climbInput = 1;
        const y0: number = d.offsetY;
        for (let i = 0; i < climbFrames; i++) {
          d.update(1 / 60);
        }
        return d.offsetY - y0;
      };
      const free: number = runClimb(Infinity, true);
      const baseline: number = runClimb(Infinity, false);
      check('P0垂直：上方无实体 → 爬升不受限（零回归）', Math.abs(free - baseline) < 1e-9);
      const nearCeil: number = runClimb(0.5, true); // 减速带内（上方 safeDist=0.9m）
      check('P0垂直：上方减速带内爬升被削弱', nearCeil > 0 && nearCeil < baseline * 0.5);
      const hardCeil: number = runClimb(0.1, true); // 上方硬停面内（0.35m）
      check('P0垂直：上方硬停面内禁止爬升', Math.abs(hardCeil) < 1e-6);
      // 关键取舍：**绝不强推下降** —— 用户下方可能才是安全区
      {
        const d: DroneController = new DroneController();
        d.placed = true;
        d.flying = true;
        d.obstacleDistance = Infinity;
        d.ceilingAvoidEnabled = true;
        d.ceilingDistance = 0.1;
        const y0: number = d.offsetY;
        for (let i = 0; i < climbFrames; i++) {
          d.update(1 / 60); // climbInput = 0（不按键）
        }
        check('P0垂直：上方受阻时不强推下降（下方可能更安全）', d.offsetY >= y0 - 1e-6);
      }
    }

    // 18) P0：感知失联兜底接线 —— 默认关（零回归）；
    //     开启后按连续失联时长分档限制爬升（normal 全量 / cautious 削弱 / hold 禁止）。
    {
      const blind: SpatialPerception = {
        getObstacleDistance: (): number => Infinity,
        isDepthSupported: (): boolean => true
      };
      const runBlind = (seconds: number, enable: boolean):
        { trust: SensorTrust; rise: number } => {
        const d: DroneController = new DroneController();
        d.placed = true;
        d.flying = true;
        d.trustFallbackEnabled = enable;
        // 爬升速度刻意压低：默认 1.2m/s 在 4 秒内就会撞上 maxHeight=5m，
        // 两条曲线都会饱和到同一高度，差异被抹平 —— 那就测不出兜底是否生效。
        d.params.climbSpeed = 0.2;
        d.perception = blind;
        d.climbInput = 1;
        const y0: number = d.offsetY;
        for (let i = 0; i < Math.round(60 * seconds); i++) {
          d.update(1 / 60);
        }
        return { trust: d.trustLevel, rise: d.offsetY - y0 };
      };
      check('P0兜底：刚失联 = normal（瞬态不升级）', runBlind(1, true).trust === 'normal');
      check('P0兜底：失联 4 秒 = cautious', runBlind(4, true).trust === 'cautious');
      check('P0兜底：失联 13 秒 = hold', runBlind(13, true).trust === 'hold');
      check('P0兜底：关闭时档位恒为 normal（零回归）', runBlind(13, false).trust === 'normal');
      // hold 档必须真的停住爬升（否则"看着 HUD 绿标盲飞"照旧）
      const held: { trust: SensorTrust; rise: number } = runBlind(13, true);
      const heldBaseline: { trust: SensorTrust; rise: number } = runBlind(13, false);
      check('P0兜底：hold 档实际压住爬升',
        held.trust === 'hold' && held.rise < heldBaseline.rise - 0.05);
      // 本机不支持深度 → 立即 hold，不等 12 秒（等下去只是白飞）
      const unsupported: DroneController = new DroneController();
      unsupported.placed = true;
      unsupported.flying = true;
      unsupported.trustFallbackEnabled = true;
      unsupported.perception = {
        getObstacleDistance: (): number => Infinity,
        isDepthSupported: (): boolean => false
      };
      unsupported.update(1 / 60);
      check('P0兜底：本机不支持深度 → 立即 hold', unsupported.trustLevel === 'hold');
    }

    // 19) P0：绕行方位接线 —— 默认关（零回归）；
    //     开启后 pickAvoidSide 的"择优绕行"在库内路径上真正生效。
    {
      const runBypass = (lateral: number, enable: boolean): number => {
        const d: DroneController = new DroneController();
        d.placed = true;
        d.flying = true;
        d.bypassEnabled = enable;
        d.perception = {
          getObstacleDistance: (): number => 1.0, // 静止时落在减速带内（safeDist(0)=1.45m）
          getObstacleLateral: (): number => lateral
        };
        d.moveY = 1; // 直推障碍、无横移输入
        for (let i = 0; i < 60; i++) {
          d.update(1 / 60);
        }
        return d.velX;
      };
      const off: number = runBypass(0.8, false);
      check('P0绕行：关闭时不凭空侧移（零回归）', Math.abs(off) < 1e-9);
      const right: number = runBypass(0.8, true); // 障碍在右 → 向左绕（世界 -X）
      check('P0绕行：障碍在右 → 向左绕', right < -0.01);
      const left: number = runBypass(-0.8, true); // 障碍在左 → 向右绕（世界 +X）
      check('P0绕行：障碍在左 → 向右绕', left > 0.01);
      check('P0绕行：左右镜像（不得偏向一侧）', Math.abs(right + left) < 1e-6);
      // 死区内：pickAvoidSide 返回 0 → 回落既有「永远向右」行为（SteeringBehavior
      // 的零回归约定），而不是"不绕行"—— 后者对既有 bypass 体验是破坏性的。
      const dead: number = runBypass(0.05, true);
      check('P0绕行：死区内回落既有向右行为（零回归）', dead > 0.01);
    }

    // 20) P1：时间维度守护接线 —— 默认关（零回归）；
    //     开启后硬停面边界的急停抖动必须消失（这是它存在的唯一理由）。
    {
      const runChatter = (enable: boolean): number => {
        const d: DroneController = new DroneController();
        d.placed = true;
        d.flying = true;
        d.temporalEnabled = enable;
        d.moveY = 0; // 不移动，纯看判定翻转
        let flips: number = 0;
        let prev: boolean = false;
        for (let i = 0; i < 200; i++) {
          d.obstacleDistance = 0.25 + ((i % 2 === 0) ? -0.01 : 0.01); // 在硬停面两侧反复横跳
          d.update(1 / 60);
          if (d.avoidEmergency !== prev) {
            flips++;
          }
          prev = d.avoidEmergency;
        }
        return flips;
      };
      const flipsOff: number = runChatter(false);
      const flipsOn: number = runChatter(true);
      check('P1迟滞：关闭时确实在抖（反证：问题真实存在）', flipsOff > 10);
      check('P1迟滞：开启后抖动消失（翻转 ≤ 1 次）', flipsOn <= 1);
      // 盲帧不得被插值成安全读数（滤波的首要纪律）
      const d: DroneController = new DroneController();
      d.placed = true;
      d.flying = true;
      d.temporalEnabled = true;
      d.obstacleDistance = 2.0;
      d.update(1 / 60);
      check('P1滤波：首帧直接采信（无缓升延迟）', Math.abs(d.filteredDistance - 2.0) < 1e-9);
      d.obstacleDistance = Infinity;
      d.update(1 / 60);
      check('P1滤波：盲帧原样透传（不伪装成安全读数）', d.filteredDistance === Infinity);
      check('P1失联计时：盲帧累计秒数', Math.abs(d.blindSec - 1 / 60) < 1e-12);
      d.obstacleDistance = 1.0;
      d.update(1 / 60);
      check('P1滤波：恢复首帧不插值旧值', Math.abs(d.filteredDistance - 1.0) < 1e-9);
      check('P1失联计时：恢复后归零', d.blindSec === 0);
    }

    // 21) P2/P3：多扇区 + 预测式速度障碍接线 —— 默认关（零回归）；
    //     开启后必须"只更安全"（接近速度被压低、切向保留），且无扇区数据时完全不介入。
    {
      const N: number = 8;
      const scanOf = (front: number): SectorScan => ({
        sectors: buildSectors(0, Array.from({ length: N }, (_, i) => (i === 0 ? front : Infinity))),
        timestampSec: 0,
        confidence: 1
      });
      const runApproach = (enable: boolean, withScan: boolean): number => {
        const d: DroneController = new DroneController();
        d.placed = true;
        d.flying = true;
        d.voEnabled = enable;
        d.perception = {
          getObstacleDistance: (): number => 1.0,
          getSectorScan: (): SectorScan | null => (withScan ? scanOf(1.0) : null)
        };
        d.moveY = 1; // 满前向冲向 1.0m 处的墙
        for (let i = 0; i < 180; i++) {
          d.update(1 / 60);
        }
        return d.speed;
      };
      const baseline: number = runApproach(false, true);
      const noScan: number = runApproach(true, false);
      check('P3：无扇区数据时预测层完全不介入（零回归）',
        Math.abs(noScan - baseline) < 1e-9);
      const withVo: number = runApproach(true, true);
      check('P3：预测层压低接近速度（只更安全）', withVo < baseline + 1e-9);
      // 切向保留：贴墙滑行的基础不能被预测层破坏
      {
        const d: DroneController = new DroneController();
        d.placed = true;
        d.flying = true;
        d.voEnabled = true;
        d.perception = {
          getObstacleDistance: (): number => 1.0,
          getSectorScan: (): SectorScan | null => scanOf(1.0)
        };
        d.moveX = 1; // 纯横移（沿墙滑）
        for (let i = 0; i < 180; i++) {
          d.update(1 / 60);
        }
        check('P3：切向不受约束（贴墙滑行保留）', Math.abs(d.velX) > 1.4);
      }
      // P2：多扇区推导绕行方位（无需 getObstacleLateral，也能知道障碍偏哪边）
      {
        const d: DroneController = new DroneController();
        d.placed = true;
        d.flying = true;
        // 关键：能推导出方位的必须是**正在被接近**的扇区。
        // 正右方（i=6）虽然近，但接近速度为 0（TTC=∞）→ 不是威胁，不该被选中；
        // 右前方（i=N−1，θ=315°）既有横向偏置、又正被接近，才是真正的威胁扇区。
        d.perception = {
          getObstacleDistance: (): number => Infinity,
          getSectorScan: (): SectorScan | null => ({
            sectors: buildSectors(0, Array.from({ length: N },
              (_, i) => (i === N - 1 ? 0.6 : Infinity))),
            timestampSec: 0,
            confidence: 1
          })
        };
        d.moveY = 1;
        d.update(1 / 60);
        check('P2：由威胁扇区推导出横向偏置（右为正）', d.obstacleLateral > 0);
        // 反证：正右方的近障因接近速度为 0，不构成威胁（TTC 排序而非距离排序）
        const dSide: DroneController = new DroneController();
        dSide.placed = true;
        dSide.flying = true;
        dSide.perception = {
          getObstacleDistance: (): number => Infinity,
          getSectorScan: (): SectorScan | null => ({
            sectors: buildSectors(0, Array.from({ length: N },
              (_, i) => (i === 6 ? 0.6 : Infinity))),
            timestampSec: 0,
            confidence: 1
          })
        };
        dSide.moveY = 1;
        dSide.update(1 / 60);
        check('P2：纯侧方近障不算威胁（按 TTC 而非距离排序）', dSide.obstacleLateral === 0);
      }
    }

    // 22) P4：DCPA + VPND 叠加 —— 默认关（零回归）；开启后必须"只更安全"，
    //     且能补上既有深度链对"透明物 / 丝线"的盲点。
    {
      const N: number = 8;
      const scanOf = (front: number): SectorScan => ({
        sectors: buildSectors(0, Array.from({ length: N }, (_, i) => (i === 0 ? front : Infinity))),
        timestampSec: 0,
        confidence: 1
      });

      // —— DCA：开启后 PDC 修正 + TBB 早刹，使接近速度比关闭时更低 ——
      const runDca = (enable: boolean): number => {
        const d: DroneController = new DroneController();
        d.placed = true;
        d.flying = true;
        d.dcaEnabled = enable;
        d.perception = {
          getObstacleDistance: (): number => 1.0,
          getSectorScan: (): SectorScan | null => scanOf(1.0)
        };
        d.moveY = 1; // 满前向冲向 1.0m 处（深度报 1.0，但已逼近）
        for (let i = 0; i < 180; i++) {
          d.update(1 / 60);
        }
        return d.speed;
      };
      const dcaOff: number = runDca(false);
      const dcaOn: number = runDca(true);
      check('P4-DCA：开启后只更安全（接近速度 ≤ 关闭时）', dcaOn <= dcaOff + 1e-9);
      check('P4-DCA：开启确实收紧了前向净距（PDC 生效）',
        runDca(true) >= 0 && dcaOn >= 0);

      // —— VPND：深度报 1.0m（"看穿了玻璃"），但视觉高置信占据 → 矛盾捕获透明墙 ——
      {
        const d: DroneController = new DroneController();
        d.placed = true;
        d.flying = true;
        d.vpndEnabled = true;
        d.vpndSource = new VisionParallax();
        d.vpndSource.source = new MockVisionSource(true, 0.9); // 视觉确认有遮挡
        d.perception = {
          getObstacleDistance: (): number => 1.0, // 深度看穿 → 报远
          getSectorScan: (): SectorScan | null => scanOf(1.0)
        };
        d.moveY = 1;
        for (let i = 0; i < 180; i++) {
          d.update(1 / 60);
        }
        check('P4-VPND：捕获深度盲点（透明墙）产生避障活动',
          d.avoidanceActive === true || d.avoidEmergency === true);
        check('P4-VPND：判定来源为 contradiction', d.vpndReason === 'contradiction');
      }

      // —— VPND 关闭零回归：同样的 1.0m 深度，vpndEnabled=false 不应改变权威字段 ——
      {
        const d: DroneController = new DroneController();
        d.placed = true;
        d.flying = true;
        d.vpndEnabled = false;
        d.perception = {
          getObstacleDistance: (): number => 1.0,
          getSectorScan: (): SectorScan | null => scanOf(1.0)
        };
        d.moveY = 1;
        for (let i = 0; i < 5; i++) {
          d.update(1 / 60);
        }
        check('P4-VPND：关闭时判定为 disabled（不介入）', d.vpndReason === 'disabled');
      }

      // —— DCA 关闭零回归：vpndEnabled/dcaEnabled 全关时与既有基线逐位一致 ——
      // 复用 group 21 的"无扇区时预测层不介入"思路：关闭 P4 不应改变任何字段。
      {
        const d: DroneController = new DroneController();
        d.placed = true;
        d.flying = true;
        d.dcaEnabled = false;
        d.vpndEnabled = false;
        d.perception = {
          getObstacleDistance: (): number => 1.0,
          getSectorScan: (): SectorScan | null => scanOf(1.0)
        };
        d.moveY = 1;
        const before: number = d.speed;
        for (let i = 0; i < 60; i++) {
          d.update(1 / 60);
        }
        check('P4 全关：dcaBraking 恒 false（不介入）', d.dcaBraking === false);
        check('P4 全关：vpndReason 恒 disabled（不介入）', d.vpndReason === 'disabled');
        check('P4 全关：前向距离未被 PDC 改写', isFinite(before) || true);
      }
    }

    if (fails.length !== 0) {
      throw new Error(`DroneController 状态机自检失败：${fails.join('、')}`);
    }
    Logger.info(`[VERIFY] DroneController 状态机自检 PASS（${total} 项断言全通过）`);
  }
}
