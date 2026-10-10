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
import { Logger } from '../core/Logger';
import { Vec3 } from '../core/Vec3';
import { SpatialPerception } from '../contract/SpatialPerception';
import { EnvironmentPerception } from '../contract/EnvironmentPerception';
import { WIND_GAIN } from '../environment/Atmosphere';
import {
  attitudeTorque,
  dragForce,
  positionHoldVelocity,
  thrustVector,
  tiltFromUpDir,
  velocityLoop,
  verticalLoop,
  yawTorque,
} from '../dynamics/CascadeController';
import {
  DEFAULT_DYNAMICS,
  FlightDynamics,
  geometryOf,
} from '../dynamics/FlightDynamics';
import {
  applyActuatorLimit,
  DEFAULT_QUAD,
  mixThrusts,
  QuadGeometry,
  RotorPlant,
} from '../dynamics/RotorMixer';

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
    const geo: QuadGeometry = geometryOf(DEFAULT_DYNAMICS);
    const D = DEFAULT_DYNAMICS;
    const active: boolean = this.placed && this.flying;

    if (!active) {
      // 未激活：电机停转、速度衰减、冻结位置（与运动学非激活一致）
      const k: number = clampValue(decayFactor(p.damping, dt), 0, 1);
      this.velX *= k; this.velY *= k; this.velZ *= k;
      this.writeRotorFromPlant(plant, geo, dt);
      return;
    }

    // 保持环接合（与运动学同源）：无杆量 + 启用环境感知时定锚点
    if (!hasMoveInput && this.environmentActive) {
      if (!this.holdEngaged) {
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
      const tauPitch: number = attitudeTorque(tilt[0], d.pitch, d.wPitch, DYN_ATT_KP, DYN_ATT_KD, D.inertiaTilt);
      const tauRoll: number = attitudeTorque(tilt[1], d.roll, d.wRoll, DYN_ATT_KP, DYN_ATT_KD, D.inertiaTilt);
      const tauYaw: number = 0;
      // 混控 → 执行器限幅 → 电机一阶滞后 → 实际推力
      const raw: number[] = mixThrusts(T, tauPitch, tauRoll, tauYaw, geo);
      const limited = applyActuatorLimit(raw, geo, airDensity);
      const act = plant.update(h, limited.speeds, geo, airDensity);
      // 刚体积分
      d.step(h, act.thrusts, windX, windZ, airDensity);
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
    const distance: number = this.perception
      ? this.perception.getObstacleDistance()
      : this.obstacleDistance;
    this.obstacleDistance = distance; // 缓存，供 describe() 诊断展示

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
      // bypassMode 显式传 false、横向偏置传 0：库内控制器暂无绕行方位输入，
      // 保持「未启用绕行」的既有行为（App 侧动力学路径的 bypass 接线归 M3）。
      const curSpeed: number = Math.sqrt(this.velX * this.velX + this.velZ * this.velZ);
      const av: AvoidResult = applyAvoidance(targetVX, targetVZ, fx, fz,
        distance, this.steering, false, 0, curSpeed);
      targetVX = av.vx;
      targetVZ = av.vz;
      this.avoidStrength = av.avoidStrength;
      this.avoidEmergency = av.emergency;
      if (av.emergency) {
        // 急停：刹掉当前「朝障碍去」的前进速度分量。
        // 横移/后退分量保留 —— 这正是真机验证过的「贴墙滑行不撞墙」。
        const approach: number = this.velX * fx + this.velZ * fz;
        if (approach > 0) {
          this.velX -= fx * approach;
          this.velZ -= fz * approach;
        }
      }
      this.avoidanceActive = av.braking;
    } else {
      this.avoidStrength = 0;
      this.avoidEmergency = false;
      this.avoidanceActive = false;
    }
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
      this.stepDynamics(dt, targetVX, targetVZ, climbSource, yawSource, fx, fz, rx, rz, p, hasMoveInput);
      this.syncState();
      return;
    }

    if (hasMoveInput) {
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
      if (this.environmentActive) {
        // 首次接合：锚点定在当前位姿，避免突兀拉回。
        if (!this.holdEngaged) {
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
    const targetVY: number = climbSource * p.climbSpeed;
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
    return s;
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

    if (fails.length !== 0) {
      throw new Error(`DroneController 状态机自检失败：${fails.join('、')}`);
    }
    Logger.info(`[VERIFY] DroneController 状态机自检 PASS（${total} 项断言全通过）`);
  }
}
