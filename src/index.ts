/**
 * uav_flight_sim —— 无人机飞行仿真（纯算法层）统一出口。
 * ---------------------------------------------------------------
 * 本文件是库对外公开 API 的 barrel，消费方（OpenHarmony App / 仿真器 /
 * 单元测试）只需 `import { ... } from 'uav_flight_sim'` 即可拿到全部公共类型与函数。
 *
 * 分层（M2 起按领域切分，自底向上单向依赖）：
 *   core/         零依赖基础设施（Vec3 / Logger），被所有模块共享；
 *   model/        飞行动力学状态机（依赖 core）；
 *   avoidance/    碰撞评估 + 转向力合成（依赖 core）；
 *   dynamics/     真实刚体动力学：混控 + 积分器 + 级联控制（依赖 core）；
 *   planning/     占用栅格 + A* 路径规划 + mesh 栅格化（依赖 core / avoidance）；
 *   environment/  大气换算与天气归一化（依赖 core）；
 *   telemetry/    对外可观测状态推导（依赖 core / avoidance）；
 *   contract/     输入契约（纯接口，仅依赖 core 的 Vec3 类型）。
 *
 * 本层**零专有依赖**（不引用 @kit.AREngine / ArkGraphics3D / hilog），
 * 可在 Node、CI、OpenHarmony 三端复用同一份逻辑。
 */

// ── 基础设施：数值与向量、日志 ──────────────────────────────────
export { Vec3, vec3, add, sub, scale, length, normalizeVec3, ZERO } from './core/Vec3';
export { Logger } from './core/Logger';
export { LoggerLevel } from './core/LoggerLevel';

// ── 契约：输入抽象接口（算法与数据源的唯一边界）──────────────────
export { SpatialPerception } from './contract/SpatialPerception';
export { EnvironmentPerception } from './contract/EnvironmentPerception';

// ── 避障层：碰撞评估 + 几何 ──────────────────────────────────────
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

// ── 避障层：转向力合成 + 速度域避障 ──────────────────────────────
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

// ── 模型层：飞控状态机（运动学内核）──────────────────────────────
export {
  DroneState,
  DRONE_MODEL_SCALE,
  DroneFlightParams,
  DEFAULT_FLIGHT_PARAMS,
  DroneController,
} from './model/DroneController';

// ── 模型层：AR 世界对象（锚定 + 偏移位姿推导）────────────────────
export {
  WorldPose,
  WorldObjectType,
  WorldObject,
  DroneWorldObject,
} from './model/WorldObject';

// ── 动力学层：执行器分配 / 混控 / 电机滞后 ───────────────────────
export {
  QuadGeometry,
  DEFAULT_QUAD,
  ActuatorResult,
  mixThrusts,
  torquesFromThrusts,
  applyActuatorLimit,
  hoverThrustPerRotor,
  RotorPlant,
  selfTest as rotorMixerSelfTest,
} from './dynamics/RotorMixer';

// ── 动力学层：刚体飞行动力学（半隐式欧拉积分器）──────────────────
export {
  DynamicsParams,
  DEFAULT_DYNAMICS,
  geometryOf,
  FlightDynamics,
} from './dynamics/FlightDynamics';

// ── 动力学层：级联控制（位置环 / 速度环 / 姿态环 纯函数）─────────
export {
  positionHoldVelocity,
  velocityLoop,
  verticalLoop,
  thrustVector,
  tiltFromUpDir,
  attitudeTorque,
  yawTorque,
  dragForce,
  selfTest as cascadeSelfTest,
} from './dynamics/CascadeController';

// ── 规划层：2.5D 占用栅格 + A* 路径规划 ──────────────────────────
export {
  CELL_UNKNOWN,
  CELL_FREE,
  CELL_OCCUPIED,
  FLIGHT_BAND_TOP_Y,
  WorldPlane,
  GridConfig,
  DEFAULT_GRID_CONFIG,
  OccupancyGrid,
  GridStats,
  PathPoint,
  PlanResult,
  PlanOptions,
  DEFAULT_PLAN_OPTIONS,
  isHorizontal,
  isVertical,
  pointInPolygonXZ,
  buildOccupancyGrid,
  rasterizePolygonFill,
  rasterizeSegment,
  inflate,
  planPath,
  smoothPath,
  lineOfSight,
  selfTest as pathPlannerSelfTest,
} from './planning/PathPlanner';

// ── 规划层：场景 mesh → 占用栅格（平面栅格的替代数据源）──────────
export {
  MeshSnapshot,
  TriOrientation,
  classifyTriangle,
  markForTriangle,
  triNormalY,
  triArea2,
  buildGridFromMesh,
  meshUsable,
  selfTest as meshGridSelfTest,
} from './planning/MeshGrid';

// ── 环境层：风矢量 / 空气密度 / 低通平滑 / 阵风 ──────────────────
export {
  WIND_GAIN,
  WIND_TILT_REF,
  WIND_SMOOTH,
  GUST_AMP,
  GUST_FREQ,
  windDirUnit,
  windVelocity,
  windAccel,
  airDensityFactor,
  bearingText,
  WindSmoother,
  selfTest as atmosphereSelfTest,
} from './environment/Atmosphere';

// ── 环境层：天气快照归一化（单位换算 / 异常值防护 / WMO）──────────
export {
  WeatherSnapshot,
  offlineSnapshot,
  windUnitScale,
  clampWind,
  numOr,
  wmoText,
  selfTest as weatherCodeSelfTest,
} from './environment/WeatherCode';

// ── 遥测层：HUD 威胁等级 / 配色 / 文案推导 ───────────────────────
export {
  HudThreat,
  HUD_COLOR_SAFE,
  HudState,
  deriveHud,
  selfTest as hudSelfTest,
} from './telemetry/HudModel';
