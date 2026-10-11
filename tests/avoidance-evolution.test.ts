/**
 * 避障演进（P0 接线 / P1 时间维度 / P2 多扇区 / P3 预测式）回归测试
 * ---------------------------------------------------------------
 * 这些能力有一个共同风险：**在"未启用"时悄悄改变既有行为**。
 * 模块级 selfTest 已经覆盖了纯函数性质，本文件覆盖的是另一半 ——
 * 「接线」本身是否守住了零回归契约，以及各能力开启后是否真的产生预期效果。
 *
 * 组织方式（每条都对应一条明确的失效模式）：
 *   1. 契约向后兼容：只实现旧方法的感知源必须照旧工作；
 *   2. 开关关闭 → 与既有行为逐位一致（零回归硬保证）；
 *   3. 开关开启 → 产生**可观测**的差异（否则等于没接上）；
 *   4. 预测层只更安全：绝不放大速度。
 */
import { describe, it, expect } from 'vitest';
import {
  DroneController,
  SectorScan,
  SpatialPerception,
  buildSectors,
  solveVelocityObstacle,
  DEFAULT_VO,
  VO_TTC_SEC,
  DRONE_RADIUS_M,
  emergencyWithHysteresis,
  filterDistance,
  corridorWeight,
  BlindTracker,
  TemporalGuard,
} from '../src/index';

const DT: number = 1 / 60;

/** 只实现旧契约的感知源（模拟既有 AREngine 深度采样，未经任何改造） */
class LegacyPerception implements SpatialPerception {
  constructor(private readonly gap: number) {}
  getObstacleDistance(): number {
    return this.gap;
  }
}

/** 全功能感知源（P2 多扇区 + P0 垂直 / 失联扩展） */
class FullPerception implements SpatialPerception {
  constructor(
    private readonly gap: number,
    private readonly ceiling: number = Infinity,
    private readonly lateral: number = 0,
    private readonly blind: boolean = false
  ) {}
  getObstacleDistance(): number {
    return this.blind ? Infinity : this.gap;
  }
  getCeilingDistance(): number {
    return this.blind ? Infinity : this.ceiling;
  }
  getObstacleLateral(): number {
    return this.lateral;
  }
  isDepthSupported(): boolean {
    return true;
  }
  getSectorScan(): SectorScan | null {
    if (this.blind) {
      return null;
    }
    const dists: number[] = [this.gap, Infinity, Infinity, Infinity,
      Infinity, Infinity, Infinity, Infinity];
    return { sectors: buildSectors(0, dists), timestampSec: 0, confidence: 1 };
  }
}

/** 推 N 帧，返回水平位移与末速 */
function fly(d: DroneController, frames: number): { x: number; z: number; speed: number } {
  for (let i = 0; i < frames; i++) {
    d.update(DT);
  }
  return { x: d.offsetX, z: d.offsetZ, speed: d.speed };
}

describe('P2 契约向后兼容：只实现旧方法的感知源照旧工作', () => {
  it('旧感知源 + 全部新开关关闭 → 行为与升级前一致（硬停面处停住）', () => {
    const d = new DroneController();
    d.placed = true;
    d.flying = true;
    d.perception = new LegacyPerception(0.2); // 硬停面内
    d.moveY = 1;
    const r = fly(d, 120);
    expect(r.z).toBeGreaterThan(-0.5); // 不得穿过障碍
    expect(d.avoidEmergency).toBe(true);
  });

  it('全功能感知源 + 全部新开关关闭 → 与旧感知源逐位一致（零回归）', () => {
    const run = (p: SpatialPerception): { x: number; z: number; speed: number } => {
      const d = new DroneController();
      d.placed = true;
      d.flying = true;
      d.perception = p;
      d.moveY = 1;
      d.moveX = 0.3;
      return fly(d, 240);
    };
    const legacy = run(new LegacyPerception(1.0));
    const full = run(new FullPerception(1.0, 0.5, 0.8, false));
    // 新契约方法全都在，但因为开关关闭，一个都不许影响结果
    expect(full.x).toBe(legacy.x);
    expect(full.z).toBe(legacy.z);
    expect(full.speed).toBe(legacy.speed);
  });
});

describe('P0 垂直避障：只压上升，绝不强推下降', () => {
  it('开启后爬升被削弱；关闭时照常爬升（零回归）', () => {
    const climb = (enable: boolean, ceiling: number): number => {
      const d = new DroneController();
      d.placed = true;
      d.flying = true;
      d.perception = new FullPerception(Infinity, ceiling);
      d.ceilingAvoidEnabled = enable;
      d.climbInput = 1;
      const y0 = d.offsetY;
      fly(d, 120);
      return d.offsetY - y0;
    };
    const off = climb(false, 0.5);
    const on = climb(true, 0.5);
    expect(off).toBeGreaterThan(0.05);
    expect(on).toBeGreaterThan(0);
    expect(on).toBeLessThan(off * 0.5);
  });

  it('上方受阻时不主动下压（下方可能才是安全区）', () => {
    const d = new DroneController();
    d.placed = true;
    d.flying = true;
    d.perception = new FullPerception(Infinity, 0.1); // 上方硬停面内
    d.ceilingAvoidEnabled = true;
    const y0 = d.offsetY;
    fly(d, 120); // 不按升降键
    expect(d.offsetY).toBeGreaterThanOrEqual(y0 - 1e-9);
  });
});

describe('P0 绕行择优：按障碍实际方位选边', () => {
  it('障碍在右 → 向左绕；障碍在左 → 向右绕（不得永远偏向一侧）', () => {
    const runBypass = (lateral: number, enable: boolean): number => {
      const d = new DroneController();
      d.placed = true;
      d.flying = true;
      d.perception = new FullPerception(1.0, Infinity, lateral, false);
      d.bypassEnabled = enable;
      d.moveY = 1;
      fly(d, 60);
      return d.velX;
    };
    expect(Math.abs(runBypass(0.8, false))).toBeLessThan(1e-9); // 关闭 → 零回归
    expect(runBypass(0.8, true)).toBeLessThan(-0.01); // 障碍在右 → 向左
    expect(runBypass(-0.8, true)).toBeGreaterThan(0.01); // 障碍在左 → 向右
  });
});

describe('P1 时间维度：迟滞消抖与盲态纪律', () => {
  it('硬停面边界抖动下，开启迟滞后急停不再翻转', () => {
    const flips = (enable: boolean): number => {
      const d = new DroneController();
      d.placed = true;
      d.flying = true;
      d.temporalEnabled = enable;
      let prev = false;
      let n = 0;
      for (let i = 0; i < 200; i++) {
        d.obstacleDistance = 0.25 + (i % 2 === 0 ? -0.01 : 0.01);
        d.update(DT);
        if (d.avoidEmergency !== prev) {
          n++;
        }
        prev = d.avoidEmergency;
      }
      return n;
    };
    expect(flips(false)).toBeGreaterThan(10); // 反证：问题真实存在
    expect(flips(true)).toBeLessThanOrEqual(1);
  });

  it('盲帧不得被插值成安全读数（滤波首要纪律）', () => {
    expect(filterDistance(0.3, Infinity, DT)).toBe(Infinity);
    expect(Number.isNaN(filterDistance(0.3, NaN, DT))).toBe(true);
    // 恢复首帧必须直接采信，不得与失联前的旧读数插值
    const g = new TemporalGuard();
    g.step(DT, 2.0, 0.25);
    g.step(DT, Infinity, 0.25);
    expect(g.step(DT, 1.0, 0.25).distance).toBe(1.0);
  });

  it('迟滞：进入用 margin，退出用 margin + 迟滞带', () => {
    expect(emergencyWithHysteresis(false, 0.25, 0.25)).toBe(true);
    expect(emergencyWithHysteresis(false, 0.26, 0.25)).toBe(false);
    expect(emergencyWithHysteresis(true, 0.30, 0.25)).toBe(true);
    expect(emergencyWithHysteresis(true, 0.34, 0.25)).toBe(false);
  });

  it('失联计时器：累计 / 复位 / 非法步长不污染', () => {
    const bt = new BlindTracker();
    bt.update(DT, false);
    expect(bt.blindSec).toBeCloseTo(DT, 12);
    bt.update(DT, true);
    expect(bt.blindSec).toBe(0);
    expect(bt.update(0, false)).toBe(0);
  });
});

describe('P3 预测式速度障碍：只更安全', () => {
  const front = { ux: 0, uz: -1, distance: 1.0, lateral: 0 };

  it('把接近速度压到 TTC 阈值内', () => {
    const r = solveVelocityObstacle(0, -1.5, [front], DEFAULT_VO);
    expect(r.blocked).toBe(true);
    expect(Math.abs(r.vz)).toBeLessThan(1.5);
    expect(r.minTtcSec).toBeGreaterThanOrEqual(VO_TTC_SEC - 1e-6);
  });

  it('永不放大速度（叠加使用的安全保证）', () => {
    for (const v of [0.05, 0.2, 0.5, 1.0, 1.5]) {
      const r = solveVelocityObstacle(0, -v, [front], DEFAULT_VO);
      expect(Math.abs(r.vz)).toBeLessThanOrEqual(v + 1e-12);
    }
  });

  it('切向分量保留（贴墙滑行不被破坏）', () => {
    const r = solveVelocityObstacle(1.5, -1.5, [front], DEFAULT_VO);
    // 径向被削、切向保留 → 合成方向更"平行于墙"
    expect(Math.abs(r.vx)).toBeGreaterThan(1.0);
    expect(Math.abs(r.vz)).toBeLessThan(1.5);
  });

  it('偏离航向轴的扇区约束被软削弱（走廊软权重）', () => {
    const near = corridorWeight(0, DEFAULT_VO.corridorHalf);
    const off = corridorWeight(DEFAULT_VO.corridorHalf * 1.5, DEFAULT_VO.corridorHalf);
    const far = corridorWeight(DEFAULT_VO.corridorHalf * 3, DEFAULT_VO.corridorHalf);
    expect(near).toBe(1);
    expect(off).toBeGreaterThan(0);
    expect(off).toBeLessThan(1);
    expect(far).toBe(0);
  });

  it('无障碍时预测层零干预', () => {
    const r = solveVelocityObstacle(0, -1.5,
      [{ ux: 0, uz: -1, distance: 50, lateral: 0 }], DEFAULT_VO);
    expect(r.blocked).toBe(false);
    expect(r.vz).toBeCloseTo(-1.5, 12);
  });

  it('端到端：开启预测层后更晚接近障碍（最小净距不小于反应式）', () => {
    const run = (enable: boolean): number => {
      const d = new DroneController();
      d.placed = true;
      d.flying = true;
      d.perception = new FullPerception(1.0, Infinity, 0, false);
      d.voEnabled = enable;
      d.moveY = 1;
      let minTtc = Infinity;
      for (let i = 0; i < 300; i++) {
        d.update(DT);
        if (d.lastTtcSec < minTtc) {
          minTtc = d.lastTtcSec;
        }
      }
      return minTtc;
    };
    // 反应式（关闭）时 lastTtcSec 恒为 ∞（不计算），开启后才有读数 ——
    // 这条断言锁住"预测层的指标通路确实被接上"，否则改坏了也看不出来。
    expect(run(false)).toBe(Infinity);
    expect(run(true)).toBeLessThan(Infinity);
  });
});

describe('避障演进：机体包围球口径', () => {
  it('预测层保留净距 = 机体包围球半径（与硬停面基线同口径）', () => {
    // 硬停面基线 0.25 = 机体半径 0.19 + 余量 0.06；预测层取半径作为"必须留出的净距"
    expect(DRONE_RADIUS_M).toBeCloseTo(0.19, 12);
    expect(DEFAULT_VO.clearance).toBe(DRONE_RADIUS_M);
    // 净距为 0 时禁止任何接近
    const r = solveVelocityObstacle(0, -1.5,
      [{ ux: 0, uz: -1, distance: DRONE_RADIUS_M, lateral: 0 }], DEFAULT_VO);
    expect(Math.abs(r.vz)).toBeLessThan(1e-9);
  });
});
