import { describe, it, expect } from 'vitest';
import { selfTest as collisionSelfTest } from '../src/avoidance/CollisionDetector';
import { selfTest as steeringSelfTest } from '../src/avoidance/SteeringBehavior';
import { DroneController } from '../src/model/DroneController';
import { selfTest as hudSelfTest } from '../src/telemetry/HudModel';
import { selfTest as rotorMixerSelfTest, RotorPlant } from '../src/dynamics/RotorMixer';
import { FlightDynamics } from '../src/dynamics/FlightDynamics';
import { selfTest as cascadeSelfTest } from '../src/dynamics/CascadeController';
import { selfTest as pathPlannerSelfTest } from '../src/planning/PathPlanner';
import { selfTest as meshGridSelfTest } from '../src/planning/MeshGrid';
import { selfTest as atmosphereSelfTest } from '../src/environment/Atmosphere';
import { selfTest as weatherCodeSelfTest } from '../src/environment/WeatherCode';

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
});
