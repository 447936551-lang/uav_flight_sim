import { describe, it, expect } from 'vitest';
import { DroneController, DroneState, SpatialPerception, Vec3, vec3 } from '../src/index';

/**
 * 仿真黄金用例（M2 验收：仿真黄金用例入 CI 并稳定通过）。
 * ---------------------------------------------------------------
 * 把 sim/sim_flight.ts 的演示场景固化成**确定性回归护栏**：
 * 同一场景（定步长、无随机源、固定墙位）必得同一输出，可作为 CI 门禁的黄金用例。
 *
 * 场景：无人机满前向推杆 20 秒，前方 -Z 方向 8m 处有一堵墙。
 *   - 无感知（防撞盲）      → 直接穿过墙体（minGap 变负）
 *   - 接入 SpatialPerception → 在硬停面 0.40m 处精确停住，不越界
 *
 * 黄金值（由确定性仿真测得，改动物理参数时此测试会失败并提醒同步更新）：
 *   感知驱动：末位 Z = -7.60m（= 墙位 -8 + 硬停面 0.4），最近间隙 = 0.40m
 *   无感知：  最近间隙 < 0（穿墙）
 */

/** 前方固定一堵墙（AR -Z 方向 8m），实现 SpatialPerception 契约 */
class WallAheadPerception implements SpatialPerception {
  private droneZ = 0;
  readonly wallZ = -8;
  readonly emergencyDist = 0.4;

  /** 每帧由仿真器把无人机当前 Z 写入 */
  setDroneZ(z: number): void {
    this.droneZ = z;
  }

  getObstacleDistance(): number {
    // forward = (0, 0, -1)；gap = (wall - drone) · forward
    return (this.wallZ - this.droneZ) * -1;
  }

  getObstacleNormal(): Vec3 {
    return vec3(0, 0, 1);
  }
}

interface RunResult {
  finalZ: number;
  minGap: number;
  crashed: boolean;
  finalState: DroneState;
}

/** 跑一次确定性仿真：usePerception=false 时防撞是盲的（obstacleDistance=∞） */
function runFlight(usePerception: boolean, frames: number = 60 * 20): RunResult {
  const d = new DroneController();
  const wall = new WallAheadPerception();
  d.placed = true;
  d.flying = true;
  if (usePerception) {
    d.perception = wall; // 关键：接入"空间感知输入"契约
  } else {
    d.obstacleDistance = Infinity;
  }
  d.moveY = 1; // 满前向推杆

  const dt = 1 / 60;
  let minGap = Infinity;
  for (let i = 0; i < frames; i++) {
    wall.setDroneZ(d.offsetZ);
    d.update(dt);
    const gap = wall.getObstacleDistance();
    if (gap < minGap) {
      minGap = gap;
    }
  }

  const finalGap = wall.getObstacleDistance();
  return {
    finalZ: d.offsetZ,
    minGap,
    crashed: finalGap <= wall.emergencyDist - 0.05,
    finalState: d.state,
  };
}

describe('仿真黄金用例：盲飞 vs 感知驱动（确定性回归护栏）', () => {
  it('无感知（防撞盲）：直接穿过墙体（模拟撞墙）', () => {
    const r = runFlight(false);
    expect(r.crashed).toBe(true);
    // 满前向 20 秒应远越过 8m 处的墙
    expect(r.finalZ).toBeLessThan(-8);
    // 最近间隙为负，说明机体已嵌入/穿过障碍
    expect(r.minGap).toBeLessThan(0);
  });

  it('接入 SpatialPerception：在硬停面前精确停住，不越界', () => {
    const r = runFlight(true);
    expect(r.crashed).toBe(false);
    // 黄金值：停在墙位 + 硬停面 = -8 + 0.4 = -7.60m
    expect(r.finalZ).toBeCloseTo(-7.6, 2);
    // 最近间隙贴着硬停面 0.40m（未越界，也未远距离僵停）
    expect(r.minGap).toBeCloseTo(0.4, 2);
  });

  it('确定性：同一场景两次运行结果完全一致（可作 CI 门禁）', () => {
    const a = runFlight(true);
    const b = runFlight(true);
    expect(a.finalZ).toBe(b.finalZ);
    expect(a.minGap).toBe(b.minGap);
    expect(a.finalState).toBe(b.finalState);
    expect(a.crashed).toBe(b.crashed);
  });
});
