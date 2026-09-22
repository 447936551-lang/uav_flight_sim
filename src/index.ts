/**
 * uav_flight_sim —— 无人机飞行仿真（纯算法层）统一出口。
 * ---------------------------------------------------------------
 * 本文件是库对外公开 API 的 barrel，消费方（OpenHarmony App / 仿真器 /
 * 单元测试）只需 `import { ... } from 'uav_flight_sim'` 即可拿到全部公共类型与函数。
 *
 * 分层（M2 起按领域切分，自底向上单向依赖）：
 *   core/        零依赖基础设施（Vec3 / Logger），被所有模块共享；
 *   model/       飞行动力学状态机（依赖 core）；
 *   avoidance/   碰撞评估 + 转向力合成（依赖 core）；
 *   telemetry/   对外可观测状态推导（依赖 core / avoidance）；
 *   contract/    空间感知输入抽象接口（纯接口，仅依赖 core 的 Vec3 类型）。
 *
 * 本层**零专有依赖**（不引用 @kit.AREngine / ArkGraphics3D / hilog），
 * 可在 Node、CI、OpenHarmony 三端复用同一份逻辑。
 */

// 基础设施：数值与向量、日志
export { Vec3, vec3, add, sub, scale, length, normalizeVec3, ZERO } from './core/Vec3';
export { Logger } from './core/Logger';
export { LoggerLevel } from './core/LoggerLevel';

// 契约：空间感知输入抽象接口（"空间感知输入" 契约）
export { SpatialPerception } from './contract/SpatialPerception';

// 避障层：碰撞评估 + 几何
export {
  PhysicsVec3,
  ThreatLevel,
  CollisionInfo,
  AVOID_MARGIN_M,
  AVOID_SLOW_BAND_M,
  AVOID_SAFE_DIST_M,
  DRONE_RADIUS_M,
  AVOID_MAX_FORCE,
  AVOID_STEER_GAIN,
  clamp01,
  clampRange,
  normalize,
  sphereSphereHit,
  closestPointOnAabb,
  sphereAabbHit,
  isValidDistance,
  evaluate,
  selfTest as collisionSelfTest,
} from './avoidance/CollisionDetector';

// 避障层：转向力合成 + 速度域避障
export {
  SteeringParams,
  DEFAULT_STEERING,
  AvoidResult,
  seekForce,
  avoidForce,
  combineForces,
  applyAvoidance,
  selfTest as steeringSelfTest,
} from './avoidance/SteeringBehavior';

// 模型层：飞控状态机
export {
  DroneState,
  DRONE_MODEL_SCALE,
  DroneFlightParams,
  DEFAULT_FLIGHT_PARAMS,
  DroneController,
} from './model/DroneController';

// 模型层：AR 世界对象（锚定 + 偏移位姿推导）
export {
  WorldPose,
  WorldObjectType,
  WorldObject,
  DroneWorldObject,
} from './model/WorldObject';

// 遥测层：HUD 威胁等级 / 配色 / 文案推导
export {
  HudThreat,
  HUD_COLOR_SAFE,
  HudState,
  deriveHud,
  selfTest as hudSelfTest,
} from './telemetry/HudModel';
