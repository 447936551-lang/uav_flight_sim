import { describe, it, expect } from 'vitest';
import { DroneController, DroneState } from '../src/core/DroneController';
import { SpatialPerception } from '../src/perception/SpatialPerception';

/** 恒定障碍距离的感知实现，用于验证 SpatialPerception 契约接入 */
class ConstPerception implements SpatialPerception {
  constructor(private readonly d: number) {}
  getObstacleDistance(): number {
    return this.d;
  }
}

describe('DroneController 状态机', () => {
  it('初始状态为 Unplaced', () => {
    const d = new DroneController();
    expect(d.state).toBe(DroneState.Unplaced);
  });

  it('已放置 + 起飞 + 无障碍 => Flying', () => {
    const d = new DroneController();
    d.placed = true;
    d.flying = true;
    d.obstacleDistance = Infinity;
    d.update(1 / 60);
    expect(d.state).toBe(DroneState.Flying);
  });

  it('SpatialPerception 契约：感知数据驱动防撞', () => {
    const d = new DroneController();
    d.placed = true;
    d.flying = true;
    d.perception = new ConstPerception(0.2); // 进入硬停面 0.4m 内
    d.update(1 / 60);
    // 控制器每帧从 perception 取数并缓存到 obstacleDistance
    expect(d.obstacleDistance).toBeCloseTo(0.2, 6);
    expect(d.avoidEmergency).toBe(true);
    expect(d.state).toBe(DroneState.EmergencyHold);
  });

  it('前后通道端到端可用（无障碍满前向位移明显）', () => {
    const d = new DroneController();
    d.placed = true;
    d.flying = true;
    d.obstacleDistance = Infinity;
    d.moveY = 1;
    for (let i = 0; i < 60; i++) {
      d.update(1 / 60);
    }
    expect(d.offsetZ).toBeLessThan(-0.5);
  });

  it('半空 disarm => 软降落回到 Grounded 且高度≈minHeight', () => {
    const d = new DroneController();
    d.placed = true;
    d.flying = true;
    d.obstacleDistance = Infinity;
    d.offsetY = 1.0;
    d.toggleFlight();
    expect(d.isLanding).toBe(true);
    let steps = 0;
    while (d.isLanding && steps < 2000) {
      d.update(0.05);
      steps++;
    }
    expect(d.state).toBe(DroneState.Grounded);
    expect(d.offsetY).toBeCloseTo(d.params.minHeight, 3);
  });

  it('gap=1.0 时前向不被误归零（防撞回归守护）', () => {
    const d = new DroneController();
    d.placed = true;
    d.flying = true;
    d.avoidanceEnabled = true;
    d.obstacleDistance = 1.0;
    d.moveY = 1;
    for (let i = 0; i < 60; i++) {
      d.update(1 / 60);
    }
    const fwdSpeed = d.velX * d.forwardX + d.velZ * d.forwardZ;
    expect(fwdSpeed).toBeGreaterThan(0.3);
  });
});
