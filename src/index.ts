/**
 * uav_flight_sim —— 无人机飞行仿真（纯算法层）统一出口。
 * ---------------------------------------------------------------
 * 本文件是库对外公开 API 的 barrel，消费方（OpenHarmony App / 仿真器 /
 * 单元测试）只需 `import { ... } from 'uav_flight_sim'` 即可拿到全部公共类型与函数。
 *
 * 本层**零专有依赖**（不引用 @kit.AREngine / ArkGraphics3D / hilog），
 * 可在 Node、CI、OpenHarmony 三端复用同一份逻辑。
 */

// 数值与向量基础
export { Vec3, vec3, add, sub, scale, length, normalizeVec3, ZERO } from './core/Vec3';

// 自实现日志（替代 hilog）
export { Logger } from './core/Logger';
export { LoggerLevel } from './core/LoggerLevel';

// 空间感知抽象接口（"空间感知输入" 契约）
export { SpatialPerception } from './perception/SpatialPerception';

// 物理层：碰撞检测
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
} from './core/CollisionDetector';

// 物理层：转向 / 避障
export {
  SteeringParams,
  DEFAULT_STEERING,
  AvoidResult,
  seekForce,
  avoidForce,
  combineForces,
  applyAvoidance,
  selfTest as steeringSelfTest,
} from './core/SteeringBehavior';

// 飞控
export {
  DroneState,
  DRONE_MODEL_SCALE,
  DroneFlightParams,
  DEFAULT_FLIGHT_PARAMS,
  DroneController,
} from './core/DroneController';

// HUD 状态推导
export {
  HudThreat,
  HUD_COLOR_SAFE,
  HudState,
  deriveHud,
  selfTest as hudSelfTest,
} from './core/HudModel';

// AR 世界对象
export {
  WorldPose,
  WorldObjectType,
  WorldObject,
  DroneWorldObject,
} from './core/WorldObject';
