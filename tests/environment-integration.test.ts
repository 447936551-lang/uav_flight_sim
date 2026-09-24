import { describe, it, expect } from 'vitest';
import { DroneController } from '../src/model/DroneController';
import { EnvironmentPerception } from '../src/contract/EnvironmentPerception';
import { Vec3 } from '../src/core/Vec3';

/** 确定性风源：恒定风场，可注入单测与仿真 */
class MockEnvironment implements EnvironmentPerception {
  public windX: number;
  public windZ: number;
  public density: number;

  constructor(windX: number = 0, windZ: number = 0, density: number = 1.0) {
    this.windX = windX;
    this.windZ = windZ;
    this.density = density;
  }

  public getWindVelocity(): Vec3 | null {
    return { x: this.windX, y: 0, z: this.windZ };
  }

  public getAirDensity(): number {
    return this.density;
  }
}

/** 跑 n 帧，返回末态（用于对比） */
function fly(d: DroneController, n: number): string {
  for (let i = 0; i < n; i++) {
    d.update(1 / 60);
  }
  return `${d.offsetX.toFixed(12)}|${d.offsetZ.toFixed(12)}|${d.velX.toFixed(12)}`;
}

/** 建一架已放置、已起飞、满前向推杆的无人机 */
function readyDrone(): DroneController {
  const d: DroneController = new DroneController();
  d.placed = true;
  d.flying = true;
  d.obstacleDistance = Infinity; // 无障碍，排除避障干扰
  d.moveY = 1;
  return d;
}

describe('EnvironmentPerception 接入 DroneController', () => {
  it('未接入环境 → 与不接环境时逐位一致（零回归硬保证）', () => {
    const plain: DroneController = readyDrone();
    const withNull: DroneController = readyDrone();
    withNull.setEnvironment(null);

    expect(fly(withNull, 300)).toBe(fly(plain, 300));
  });

  it('接入但开关关闭 → 同样逐位一致（开关是真开关）', () => {
    const plain: DroneController = readyDrone();
    const off: DroneController = readyDrone();
    off.setEnvironment(new MockEnvironment(8, 3, 0.8));
    off.environmentActive = false; // 已接入，但关闭

    expect(fly(off, 300)).toBe(fly(plain, 300));
    expect(off.windVelX).toBe(0);
    expect(off.windVelZ).toBe(0);
    expect(off.airDensityFactor).toBe(1.0);
  });

  it('开启 + 顺风（+X）→ 无人机被吹向下风，位移大于无风基准', () => {
    const calm: DroneController = readyDrone();
    const windy: DroneController = readyDrone();
    windy.setEnvironment(new MockEnvironment(6, 0));
    windy.environmentActive = true;

    const calmEnd: string = fly(calm, 300);
    const windyEnd: string = fly(windy, 300);

    expect(windyEnd).not.toBe(calmEnd); // 风确实起作用了
    expect(windy.offsetX).toBeGreaterThan(calm.offsetX); // 被吹向 +X
  });

  it('开启 + 侧风（+Z）→ 位移偏向 +Z', () => {
    const calm: DroneController = readyDrone();
    const windy: DroneController = readyDrone();
    windy.setEnvironment(new MockEnvironment(0, 6));
    windy.environmentActive = true;

    fly(calm, 300);
    fly(windy, 300);

    expect(windy.offsetZ).toBeGreaterThan(calm.offsetZ);
  });

  it('空气密度被真正写入（动力学内核据此修正升力）', () => {
    const d: DroneController = readyDrone();
    d.setEnvironment(new MockEnvironment(0, 0, 0.85));
    d.environmentActive = true;
    d.update(1 / 60);
    expect(d.airDensityFactor).toBeCloseTo(0.85, 9);
  });

  it('摘除环境后风场立即归零，不残留上一帧', () => {
    const d: DroneController = readyDrone();
    d.setEnvironment(new MockEnvironment(6, 6));
    d.environmentActive = true;
    d.update(1 / 60);
    expect(d.windVelX).toBeGreaterThan(0);

    d.setEnvironment(null);
    expect(d.windVelX).toBe(0);
    expect(d.windVelZ).toBe(0);
    expect(d.windAccelX).toBe(0);
    expect(d.environmentActive).toBe(false);
  });

  it('未起飞（未 placed / 未 flying）时不吃风场', () => {
    const d: DroneController = new DroneController();
    d.placed = true;
    d.flying = false; // 停在地面
    d.setEnvironment(new MockEnvironment(9, 9));
    d.environmentActive = true;
    for (let i = 0; i < 60; i++) {
      d.update(1 / 60);
    }
    // 未起飞 ⇒ 位置不积分、且不吃风
    expect(d.offsetX).toBe(0);
    expect(d.windVelX).toBe(0);
  });
});
