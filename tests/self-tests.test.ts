import { describe, it, expect } from 'vitest';
import { selfTest as collisionSelfTest } from '../src/avoidance/CollisionDetector';
import { selfTest as steeringSelfTest } from '../src/avoidance/SteeringBehavior';
import { selfTest as temporalGuardSelfTest } from '../src/avoidance/TemporalGuard';
import { selfTest as velocityObstacleSelfTest } from '../src/avoidance/VelocityObstacle';
import { selfTest as delayCompensatorSelfTest } from '../src/avoidance/DelayCompensator';
import { selfTest as visionParallaxSelfTest } from '../src/avoidance/VisionParallax';
import { DroneController } from '../src/model/DroneController';
import { selfTest as hudSelfTest } from '../src/telemetry/HudModel';
import { selfTest as rotorMixerSelfTest, RotorPlant } from '../src/dynamics/RotorMixer';
import { FlightDynamics } from '../src/dynamics/FlightDynamics';
import { selfTest as cascadeSelfTest } from '../src/dynamics/CascadeController';
import { selfTest as pathPlannerSelfTest } from '../src/planning/PathPlanner';
import { selfTest as meshGridSelfTest } from '../src/planning/MeshGrid';
import { selfTest as atmosphereSelfTest } from '../src/environment/Atmosphere';
import { selfTest as weatherCodeSelfTest } from '../src/environment/WeatherCode';

// ── M4/M5 保真层（与 App 端 ARDroneSession 自检链同序）────────────
import { SeedPRNG } from '../src/core/SeedPRNG';
import { EkfEstimator } from '../src/dynamics/EkfEstimator';
import { SensorModel } from '../src/dynamics/SensorModel';
import { selfTest as turbulenceSelfTest } from '../src/environment/Turbulence';
import { selfTest as px4ParamsSelfTest } from '../src/dynamics/Px4Params';

/**
 * 复用各模块内置的纯函数自检（selfTest）。
 * 这些自检随代码一起演进，覆盖核心物理与状态机性质，是"零框架也能验收"的基础。
 */
describe('in-code selfTest (collision/steering/drone/hud)', () => {
  it('CollisionDetector.selfTest 通过', () => {
    expect(collisionSelfTest()).toContain('PASS');
  });

  it('SteeringBehavior.selfTest 通过', () => {
    expect(steeringSelfTest()).toContain('PASS');
  });

  it('TemporalGuard.selfTest 通过（P1 时间维度）', () => {
    expect(temporalGuardSelfTest()).toContain('PASS');
  });

  it('VelocityObstacle.selfTest 通过（P3 预测式）', () => {
    expect(velocityObstacleSelfTest()).toContain('PASS');
  });

  it('DelayCompensator.selfTest 通过（P4 DCPA 延迟补偿）', () => {
    expect(delayCompensatorSelfTest()).toContain('PASS');
  });

  it('VisionParallax.selfTest 通过（P4 VPND 透明物/丝线）', () => {
    expect(visionParallaxSelfTest()).toContain('PASS');
  });

  it('DroneController.selfTest 不抛错', () => {
    expect(() => DroneController.selfTest()).not.toThrow();
  });

  it('HudModel.selfTest 不抛错', () => {
    expect(() => hudSelfTest()).not.toThrow();
  });
});

describe('in-code selfTest (dynamics)', () => {
  it('RotorMixer.selfTest 不抛错', () => {
    expect(() => rotorMixerSelfTest()).not.toThrow();
  });

  it('RotorPlant.selfTest 不抛错', () => {
    expect(() => RotorPlant.selfTest()).not.toThrow();
  });

  it('FlightDynamics.selfTest 不抛错', () => {
    expect(() => FlightDynamics.selfTest()).not.toThrow();
  });

  it('CascadeController.selfTest 不抛错', () => {
    expect(() => cascadeSelfTest()).not.toThrow();
  });
});

describe('in-code selfTest (planning)', () => {
  it('PathPlanner.selfTest 不抛错', () => {
    expect(() => pathPlannerSelfTest()).not.toThrow();
  });

  it('MeshGrid.selfTest 不抛错', () => {
    expect(() => meshGridSelfTest()).not.toThrow();
  });
});

describe('in-code selfTest (environment)', () => {
  it('Atmosphere.selfTest 不抛错', () => {
    expect(() => atmosphereSelfTest()).not.toThrow();
  });

  it('WeatherCode.selfTest 不抛错', () => {
    expect(() => weatherCodeSelfTest()).not.toThrow();
  });

  it('Turbulence.selfTest 不抛错', () => {
    expect(() => turbulenceSelfTest()).not.toThrow();
  });
});

describe('in-code selfTest (M4/M5 保真层)', () => {
  it('SeedPRNG.selfTest 不抛错', () => {
    expect(() => SeedPRNG.selfTest()).not.toThrow();
  });

  it('EkfEstimator.selfTest 不抛错', () => {
    expect(() => EkfEstimator.selfTest()).not.toThrow();
  });

  it('SensorModel.selfTest 不抛错', () => {
    expect(() => SensorModel.selfTest()).not.toThrow();
  });

  it('Px4Params.selfTest 不抛错', () => {
    expect(() => px4ParamsSelfTest()).not.toThrow();
  });
});
