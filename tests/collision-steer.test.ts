import { describe, it, expect } from 'vitest';
import {
  evaluate,
  AVOID_MARGIN_M,
  AVOID_SLOW_BAND_M,
  AVOID_SAFE_DIST_M,
  DRONE_RADIUS_M,
} from '../src/avoidance/CollisionDetector';
import { applyAvoidance, DEFAULT_STEERING } from '../src/avoidance/SteeringBehavior';

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
    // 显式传满速 1.5：硬停面 margin(1.5)=1.0m → 0.3m 必在硬停面内。
    // 不能依赖默认 speed=0 —— 那时 margin 只有 0.25m，0.3m 不再是 emergency
    // （阈值挂速度改动后的连带影响，与 App 侧自检同口径修法）。
    const r = applyAvoidance(1.0, 0, 0, -1, 0.3, DEFAULT_STEERING, false, 0, 1.5);
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

  // —— 硬停面随速度伸缩（2026-10-06 同步自 App 侧阈值挂速度改动）——
  // 正向：同一距离 0.8m，满速时已在硬停面内（margin(1.5)=1.0），必须急停；
  it('同一距离 0.8m：满速急停（阈值随速生效）', () => {
    const hi = applyAvoidance(0, -1.5, 0, -1, 0.8, DEFAULT_STEERING, false, 0, 1.5);
    expect(hi.emergency).toBe(true);
  });
  // 反向：静止时 0.8m 在减速带内（margin(0)=0.25 < 0.8），不得误报急停 ——
  // 否则低速场景会被整体误刹，日常畅通反而被破坏。
  it('同一距离 0.8m：静止仅减速、不急停（反向断言）', () => {
    const lo = applyAvoidance(0, -1.5, 0, -1, 0.8, DEFAULT_STEERING, false, 0, 0);
    expect(lo.emergency).toBe(false);
    expect(lo.braking).toBe(true);
    expect(lo.avoidStrength).toBeGreaterThan(0);
  });
  // 满速介入更早：同一距离下满速的避障强度不低于静止
  it('满速时避障强度 ≥ 静止（更早介入）', () => {
    const hi = applyAvoidance(0, -1.5, 0, -1, 0.8, DEFAULT_STEERING, false, 0, 1.5);
    const lo = applyAvoidance(0, -1.5, 0, -1, 0.8, DEFAULT_STEERING, false, 0, 0);
    expect(hi.avoidStrength).toBeGreaterThanOrEqual(lo.avoidStrength);
  });
  // 显式 params 仍优先于速度推导（守护「改参数却看不到变化」的老契约）：
  // emergencyDist 取 0.6 —— 必须不同于默认 0.4，否则被视为未显式指定。
  it('显式 emergencyDist 优先于速度推导（不被 speed 覆盖）', () => {
    const custom = { ...DEFAULT_STEERING, emergencyDist: 0.6 };
    const r = applyAvoidance(0, -1.5, 0, -1, 0.8, custom, false, 0, 1.5);
    expect(r.emergency).toBe(false);   // 0.8 > 显式 0.6 → 非急停
    expect(r.braking).toBe(true);      // 但仍在减速带内
  });
});
