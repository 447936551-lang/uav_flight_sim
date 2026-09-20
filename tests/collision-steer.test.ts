import { describe, it, expect } from 'vitest';
import {
  evaluate,
  AVOID_MARGIN_M,
  AVOID_SLOW_BAND_M,
  AVOID_SAFE_DIST_M,
  DRONE_RADIUS_M,
} from '../src/core/CollisionDetector';
import { applyAvoidance, DEFAULT_STEERING } from '../src/core/SteeringBehavior';

describe('碰撞与转向契约', () => {
  it('gap=1.0 落在减速带内，avoidStrength=0.5', () => {
    expect(evaluate(1.0).avoidStrength).toBeCloseTo(0.5, 6);
  });

  it('自定义阈值必须真正生效（收窄 band 后同距离不再减速）', () => {
    expect(evaluate(1.0, DRONE_RADIUS_M, 0.4, 0.5, 0.9).avoidStrength).toBe(0);
  });

  it('自定义硬停面必须真正生效', () => {
    expect(evaluate(0.5, DRONE_RADIUS_M, 0.6, 1.2, 1.8).emergency).toBe(true);
  });

  it('急停时横移分量被完整保留（贴墙滑行基础）', () => {
    const r = applyAvoidance(1.0, 0, 0, -1, 0.3, DEFAULT_STEERING);
    expect(Math.abs(r.vx)).toBeGreaterThan(0.99);
    expect(r.emergency).toBe(true);
  });

  it('测距不可用时不限制速度（盲 ≠ 安全，但不僵死）', () => {
    const r = applyAvoidance(0, -1.5, 0, -1, Infinity, DEFAULT_STEERING);
    expect(r.braking).toBe(false);
    expect(Math.abs(r.vz + 1.5)).toBeLessThan(1e-9);
  });

  it('阈值关系：safeDist = margin + band', () => {
    expect(AVOID_SAFE_DIST_M).toBeCloseTo(AVOID_MARGIN_M + AVOID_SLOW_BAND_M, 6);
  });
});
