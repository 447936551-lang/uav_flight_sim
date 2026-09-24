import { describe, it, expect } from 'vitest';
import {
  DEFAULT_QUAD,
  DEFAULT_DYNAMICS,
  RotorPlant,
  FlightDynamics,
  mixThrusts,
  torquesFromThrusts,
  applyActuatorLimit,
  hoverThrustPerRotor,
  thrustVector,
  tiltFromUpDir,
  airDensityFactor,
  windDirUnit,
  windVelocity,
  windUnitScale,
  clampWind,
  WindSmoother,
  DEFAULT_GRID_CONFIG,
  WorldPlane,
  OccupancyGrid,
  buildOccupancyGrid,
  planPath,
  CELL_OCCUPIED,
} from '../src/index';

const D2R: number = Math.PI / 180;
const SUB: number = 1 / 240; // 内环子步

describe('dynamics · RotorMixer 混控与执行器', () => {
  it('悬停（无力矩）→ 四桨推力完全相等且各占 1/4', () => {
    const t: number[] = mixThrusts(1.0, 0, 0, 0, DEFAULT_QUAD);
    expect(t[0]).toBeCloseTo(0.25, 9);
    expect(t[1]).toBeCloseTo(0.25, 9);
    expect(t[2]).toBeCloseTo(0.25, 9);
    expect(t[3]).toBeCloseTo(0.25, 9);
  });

  it('混控 ↔ 反解 严格可逆（未饱和时）', () => {
    const pts: number[] = mixThrusts(1.2, 0.05, -0.03, 0.02, DEFAULT_QUAD);
    const tau: number[] = torquesFromThrusts(pts, DEFAULT_QUAD);
    expect(tau[0]).toBeCloseTo(0.05, 9);
    expect(tau[1]).toBeCloseTo(-0.03, 9);
    expect(tau[2]).toBeCloseTo(0.02, 9);
  });

  it('俯仰力矩 > 0 → 前桨加力、后桨减力（机头上仰）', () => {
    const t: number[] = mixThrusts(1.0, 0.08, 0, 0, DEFAULT_QUAD);
    expect(t[0]).toBeGreaterThan(t[2]); // FL > BL
    expect(t[1]).toBeGreaterThan(t[3]); // FR > BR
  });

  it('执行器饱和做在转速上：超限推力被夹到 maxRotor，且回报推力自洽', () => {
    const a = applyActuatorLimit([10, 0.3, -1, 0.6], DEFAULT_QUAD, 1.0);
    expect(a.speeds[0]).toBeCloseTo(DEFAULT_QUAD.maxRotor, 9);
    expect(a.thrusts[0]).toBeCloseTo(DEFAULT_QUAD.kT * DEFAULT_QUAD.maxRotor ** 2, 9);
    expect(a.thrusts[2]).toBeCloseTo(0, 12); // 负推力夹到 0
  });

  it('空气稀薄 → 同一推力需要更高转速', () => {
    const want: number[] = [0.3, 0.3, 0.3, 0.3];
    const sea = applyActuatorLimit(want, DEFAULT_QUAD, 1.0);
    const high = applyActuatorLimit(want, DEFAULT_QUAD, 0.85);
    expect(high.speeds[0]).toBeGreaterThan(sea.speeds[0] + 0.01);
  });

  it('悬停单桨推力 = mg/4', () => {
    expect(hoverThrustPerRotor(0.12, 9.81)).toBeCloseTo(0.2943, 3);
  });
});

describe('dynamics · RotorPlant 电机一阶滞后', () => {
  it('指令突变时转速不瞬时到位（存在滞后）', () => {
    const p: RotorPlant = new RotorPlant(0.05);
    const out = p.update(SUB, [1, 1, 1, 1], DEFAULT_QUAD, 1.0);
    expect(out.speeds[0]).toBeGreaterThan(0);
    expect(out.speeds[0]).toBeLessThan(0.99);
  });

  it('长时间后收敛到指令转速', () => {
    const p: RotorPlant = new RotorPlant(0.05);
    let out = p.update(SUB, [0.6, 0.6, 0.6, 0.6], DEFAULT_QUAD, 1.0);
    for (let i = 0; i < 999; i++) {
      out = p.update(SUB, [0.6, 0.6, 0.6, 0.6], DEFAULT_QUAD, 1.0);
    }
    expect(out.speeds[0]).toBeCloseTo(0.6, 5);
  });

  it('四桨独立差动：各桨只跟随自己的指令', () => {
    const p: RotorPlant = new RotorPlant(0.05);
    const cmd: number[] = [0.2, 0.5, 0.8, 1.0];
    for (let i = 0; i < 500; i++) {
      p.update(SUB, cmd, DEFAULT_QUAD, 1.0);
    }
    expect(p.actualSpeeds[0]).toBeCloseTo(0.2, 5);
    expect(p.actualSpeeds[1]).toBeCloseTo(0.5, 5);
    expect(p.actualSpeeds[2]).toBeCloseTo(0.8, 5);
    expect(p.actualSpeeds[3]).toBeCloseTo(1.0, 5);
  });
});

describe('dynamics · FlightDynamics 刚体积分', () => {
  it('无风悬停：等推力 mg/4 时速度不漂移', () => {
    const d: FlightDynamics = new FlightDynamics(DEFAULT_DYNAMICS);
    const per: number = hoverThrustPerRotor(DEFAULT_DYNAMICS.mass, DEFAULT_DYNAMICS.gravity);
    const t: number[] = [per, per, per, per];
    for (let i = 0; i < 480; i++) {
      d.step(SUB, t, 0, 0, 1.0);
    }
    const spd: number = Math.sqrt(d.vx * d.vx + d.vy * d.vy + d.vz * d.vz);
    expect(spd).toBeLessThan(1e-6);
  });

  it('停桨 → 自由落体，且阻力使下落略慢于纯重力', () => {
    const d: FlightDynamics = new FlightDynamics(DEFAULT_DYNAMICS);
    for (let i = 0; i < 60; i++) {
      d.step(SUB, [0, 0, 0, 0], 0, 0, 1.0);
    }
    expect(d.vy).toBeLessThan(-2.0);
    expect(d.vy).toBeGreaterThan(-DEFAULT_DYNAMICS.gravity * 0.25 - 0.05);
  });

  it('风沿 +X 吹 → 机身被吹向 +X（相对气流阻力的方向正确）', () => {
    const d: FlightDynamics = new FlightDynamics(DEFAULT_DYNAMICS);
    const per: number = hoverThrustPerRotor(DEFAULT_DYNAMICS.mass, DEFAULT_DYNAMICS.gravity);
    const t: number[] = [per, per, per, per];
    for (let i = 0; i < 240; i++) {
      d.step(SUB, t, 5.0, 0, 1.0);
    }
    expect(d.vx).toBeGreaterThan(0.05);
    expect(Math.abs(d.vy)).toBeLessThan(0.05); // 不应产生竖直漂移
  });
});

describe('dynamics · CascadeController 级联控制', () => {
  it('水平悬停（a=0、无阻力）→ 推力 = mg 且方向竖直向上', () => {
    const tv: number[] = thrustVector(0, 0, 0, 0.12, 9.81, 0, 0, 0);
    expect(tv[0]).toBeCloseTo(0.12 * 9.81, 9);
    expect(tv[2]).toBeCloseTo(1, 9); // uy = 1
    expect(Math.abs(tv[1])).toBeLessThan(1e-12);
  });

  it('想向右加速 → 解出的期望倾角为右倾（roll > 0）', () => {
    const tv: number[] = thrustVector(2.0, 0, 0, 0.12, 9.81, 0, 0, 0);
    const tilt: number[] = tiltFromUpDir(tv[1], tv[3], 0, -1, 1, 0, 20 * D2R);
    expect(tilt[1]).toBeGreaterThan(0);
    expect(Math.abs(tilt[0])).toBeLessThan(1e-9); // 不应产生俯仰
  });

  it('想向前加速（yaw=0 前向 −Z）→ 低头（pitch < 0）', () => {
    const tv: number[] = thrustVector(0, 0, -2.0, 0.12, 9.81, 0, 0, 0);
    const tilt: number[] = tiltFromUpDir(tv[1], tv[3], 0, -1, 1, 0, 20 * D2R);
    expect(tilt[0]).toBeLessThan(0);
  });

  it('期望倾角被硬限幅到 maxTiltRad', () => {
    const tv: number[] = thrustVector(20, 0, 0, 0.12, 9.81, 0, 0, 0);
    const tilt: number[] = tiltFromUpDir(tv[1], tv[3], 0, -1, 1, 0, 20 * D2R);
    expect(tilt[1]).toBeLessThanOrEqual(20 * D2R + 1e-12);
  });
});

describe('environment · 大气换算', () => {
  it('气象风向约定：北风吹向南(+Z)、东风吹向西(−X)', () => {
    const n: number[] = windDirUnit(0);
    expect(n[1]).toBeGreaterThan(0.99);
    const e: number[] = windDirUnit(90);
    expect(e[0]).toBeLessThan(-0.99);
  });

  it('风速矢量 = 单位方向 × 风速', () => {
    const v: number[] = windVelocity(0, 5);
    expect(v[1]).toBeCloseTo(5, 9);
  });

  it('高温低压 → 密度 < 1；低温高压 → 密度 > 1', () => {
    expect(airDensityFactor(1000, 35)).toBeLessThan(1.0);
    expect(airDensityFactor(1030, 0)).toBeGreaterThan(1.0);
  });

  it('环境感知关闭时输出恒为无风（零回归保证）', () => {
    const sm: WindSmoother = new WindSmoother();
    sm.targetVelX = 5;
    sm.targetVelZ = 5;
    sm.targetDensity = 0.8;
    for (let i = 0; i < 200; i++) {
      const out: number[] = sm.tick(1 / 60, false);
      expect(out[0]).toBe(0);
      expect(out[1]).toBe(0);
      expect(out[2]).toBe(1.0);
    }
  });
});

describe('environment · 天气归一化', () => {
  it('km/h 必须折算回 m/s（历史 bug：4 m/s 被读成 14.6）', () => {
    expect(14.6 * windUnitScale('km/h')).toBeCloseTo(4.056, 2);
    expect(windUnitScale('m/s')).toBe(1);
    expect(windUnitScale(undefined)).toBe(1); // 未知单位不放大
  });

  it('异常风速一律归零，绝不污染飞控', () => {
    expect(clampWind(NaN)).toBe(0);
    expect(clampWind(-3)).toBe(0);
    expect(clampWind(60)).toBe(0);
    expect(clampWind(12)).toBe(12);
  });
});

describe('planning · 占用栅格与 A*', () => {
  /** 一块 10×10 的水平地板（y=0），使覆盖区域变为可行 */
  const floor: WorldPlane = {
    cx: 0, cy: 0, cz: 0,
    nx: 0, ny: 1, nz: 0,
    poly: [-5, 0, -5, 5, 0, -5, 5, 0, 5, -5, 0, 5],
  };
  /** 一道垂直墙：x=0，z ∈ [-2, 2]，从地面到 2m 高 */
  const wall: WorldPlane = {
    cx: 0, cy: 1, cz: 0,
    nx: 1, ny: 0, nz: 0,
    poly: [0, 0, -2, 0, 0, 2, 0, 2, 2, 0, 2, -2],
  };

  it('地板被栅格化为可行区域', () => {
    const grid: OccupancyGrid = buildOccupancyGrid([floor], 0, 0);
    // 起点附近（地板覆盖范围内）应可通行
    const c: number = grid.colOf(-1.5);
    const r: number = grid.rowOf(0);
    expect(grid.at(c, r)).not.toBe(CELL_OCCUPIED);
  });

  it('墙被栅格化为障碍，且规划绕行而不是穿墙', () => {
    const grid: OccupancyGrid = buildOccupancyGrid([floor, wall], 0, 0);
    const res = planPath(grid, -1.5, 0, 1.5, 0);
    expect(res.ok).toBe(true);
    expect(res.points.length).toBeGreaterThan(2);

    // 路径上的每个点都不得落在占用格里
    for (const p of res.points) {
      expect(grid.at(grid.colOf(p.x), grid.rowOf(p.z))).not.toBe(CELL_OCCUPIED);
    }

    // 绕行 ⇒ 实际路径长度明显大于直线距离 3.0m
    let len: number = 0;
    for (let i = 1; i < res.points.length; i++) {
      const dx: number = res.points[i].x - res.points[i - 1].x;
      const dz: number = res.points[i].z - res.points[i - 1].z;
      len += Math.sqrt(dx * dx + dz * dz);
    }
    expect(len).toBeGreaterThan(3.0 * 1.2);
  });

  it('无障碍时路径接近直线（不无谓绕行）', () => {
    const grid: OccupancyGrid = buildOccupancyGrid([floor], 0, 0);
    const res = planPath(grid, -1.5, 0, 1.5, 0);
    expect(res.ok).toBe(true);
    let len: number = 0;
    for (let i = 1; i < res.points.length; i++) {
      const dx: number = res.points[i].x - res.points[i - 1].x;
      const dz: number = res.points[i].z - res.points[i - 1].z;
      len += Math.sqrt(dx * dx + dz * dz);
    }
    expect(len).toBeLessThan(3.0 * 1.15);
  });
});
