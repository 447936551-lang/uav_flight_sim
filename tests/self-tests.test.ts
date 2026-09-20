import { describe, it, expect } from 'vitest';
import { selfTest as collisionSelfTest } from '../src/core/CollisionDetector';
import { selfTest as steeringSelfTest } from '../src/core/SteeringBehavior';
import { DroneController } from '../src/core/DroneController';
import { selfTest as hudSelfTest } from '../src/core/HudModel';

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
