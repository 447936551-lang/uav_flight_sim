/**
 * 规划式绕行（bypassMode）与 HUD 盲态（Blind）行为测试
 * ---------------------------------------------------------------
 * 这两个能力是从原 App 侧同步进来的第二批演进，核心风险各有一条：
 *   1. 绕行是**新增分支** —— 默认必须关闭，否则会推翻已验证的刹停手感；
 *   2. Blind 是**语义拆分** —— 同一个 Infinity 必须因「测距是否可信」而分岔，
 *      否则又会回到「避障瞎了却显示安全」那个真机 Bug。
 * 因此这里不只跑 selfTest，而是把两条性质分别钉死。
 */
import { describe, it, expect } from 'vitest';
import { applyAvoidance, DEFAULT_STEERING } from '../src/avoidance/SteeringBehavior';
import {
  deriveHud, HudThreat, HUD_COLOR_BLIND, HUD_COLOR_FALLBACK, HUD_COLOR_SAFE
} from '../src/telemetry/HudModel';

/** 机头朝 -Z：前向 (0, -1)，则右向 = (1, 0) */
const FX: number = 0;
const FZ: number = -1;

/** 沿机头方向全速前进的期望速度 */
function straightAhead(speed: number = DEFAULT_STEERING.maxSpeed): [number, number] {
  return [FX * speed, FZ * speed];
}

describe('规划式绕行 bypassMode', () => {
  it('默认关闭：直推障碍且无横移时不会凭空侧移（零回归）', () => {
    const [dvx, dvz] = straightAhead();
    const off = applyAvoidance(dvx, dvz, FX, FZ, 0.8);
    // 关闭时：前进被减速带收敛，横移仍为 0 —— 与同步前行为逐位一致
    expect(off.braking).toBe(true);
    expect(Math.abs(off.vx)).toBeLessThan(1e-9);
    expect(off.vz).toBeLessThan(0); // 仍朝 -Z（机头前方）前进，只是被减速
  });

  it('显式传 bypassMode=false 与不传完全等价', () => {
    const [dvx, dvz] = straightAhead();
    const implicit = applyAvoidance(dvx, dvz, FX, FZ, 0.8);
    const explicit = applyAvoidance(dvx, dvz, FX, FZ, 0.8, DEFAULT_STEERING, false);
    expect(explicit.vx).toBe(implicit.vx);
    expect(explicit.vz).toBe(implicit.vz);
    expect(explicit.avoidStrength).toBe(implicit.avoidStrength);
  });

  it('开启：直推障碍且无横移时主动生成侧向速度（绕过去而非急停）', () => {
    const [dvx, dvz] = straightAhead();
    const on = applyAvoidance(dvx, dvz, FX, FZ, 0.8, DEFAULT_STEERING, true);
    // 右向 = (1,0)，故应产生 +X 分量
    expect(on.vx).toBeGreaterThan(0.05);
    // 侧向分量随避障强度增长：越近绕得越急
    const nearer = applyAvoidance(dvx, dvz, FX, FZ, 0.5, DEFAULT_STEERING, true);
    expect(nearer.vx).toBeGreaterThan(on.vx);
  });

  it('开启也不得突破 maxSpeed 矢量限幅', () => {
    const [dvx, dvz] = straightAhead();
    for (const d of [1.5, 1.0, 0.8, 0.5, 0.3]) {
      const r = applyAvoidance(dvx, dvz, FX, FZ, d, DEFAULT_STEERING, true);
      const mag: number = Math.sqrt(r.vx * r.vx + r.vz * r.vz);
      expect(mag).toBeLessThanOrEqual(DEFAULT_STEERING.maxSpeed + 1e-9);
    }
  });

  it('已有横移输入时不重复叠加侧向（只在「完全没横移」时兜底）', () => {
    // 期望速度带明显 +X 横移分量
    const withLat = applyAvoidance(0.9, -0.9, FX, FZ, 0.8, DEFAULT_STEERING, true);
    const withLatOff = applyAvoidance(0.9, -0.9, FX, FZ, 0.8, DEFAULT_STEERING, false);
    expect(withLat.vx).toBe(withLatOff.vx);
    expect(withLat.vz).toBe(withLatOff.vz);
  });

  it('无障碍时不产生任何侧向（绕行只在威胁区内生效）', () => {
    const [dvx, dvz] = straightAhead();
    const far = applyAvoidance(dvx, dvz, FX, FZ, 5.0, DEFAULT_STEERING, true);
    expect(far.braking).toBe(false);
    expect(Math.abs(far.vx)).toBeLessThan(1e-9);
  });

  it('测距不可用（Infinity）时绕行不介入 —— 盲飞不等于有障碍', () => {
    const [dvx, dvz] = straightAhead();
    const blind = applyAvoidance(dvx, dvz, FX, FZ, Infinity, DEFAULT_STEERING, true);
    expect(blind.braking).toBe(false);
    expect(blind.avoidStrength).toBe(0);
    expect(blind.vz).toBe(dvz);
  });
});

describe('HUD 盲态 Blind 与代测 degraded', () => {
  it('同一个 Infinity：测距可信 → Safe/∞，不可信 → Blind/—', () => {
    const trusted = deriveHud(Infinity, 0, false, true, false, false);
    const blind = deriveHud(Infinity, 0, false, false, false, false);
    expect(trusted.level).toBe(HudThreat.Safe);
    expect(trusted.distanceText).toBe('∞');
    expect(blind.level).toBe(HudThreat.Blind);
    expect(blind.distanceText).toBe('—');
    expect(blind.label).toBe('测距不可用');
  });

  it('Blind 刻意不用安全绿 —— 否则「避障瞎了」会被读成安全', () => {
    const blind = deriveHud(Infinity, 0, false, false, false, false);
    const safe = deriveHud(Infinity, 0, false, true, false, false);
    expect(blind.colorHex).toBe(HUD_COLOR_BLIND);
    expect(blind.colorHex).not.toBe(safe.colorHex);
  });

  it('残留旧距离不得冒充安全：不可信 + 有限距离仍判 Blind', () => {
    const stale = deriveHud(2.53, 0, false, false, false, false);
    expect(stale.level).toBe(HudThreat.Blind);
  });

  it('代测：等级仍按物理阈值，但文案改写为「相机代测」且配色独立', () => {
    const fb = deriveHud(1.2, 0.3, false, true, true, false);
    expect(fb.level).toBe(HudThreat.Caution); // 等级不被改写
    expect(fb.label).toBe('相机代测');
    expect(fb.colorHex).toBe(HUD_COLOR_FALLBACK);
    expect(fb.degraded).toBe(true);
  });

  it('代测绝不允许冒出「危险」——硬停面只有无人机本体实测才配', () => {
    const fb = deriveHud(0.2, 1.0, false, true, true, false);
    expect(fb.level).toBe(HudThreat.Danger);
    expect(fb.label).toBe('危险'); // 标签不被改成「相机代测」
  });

  it('急停优先于一切：即便测距不可信也报 Danger', () => {
    const e = deriveHud(Infinity, 0, true, false, false, false);
    expect(e.level).toBe(HudThreat.Danger);
  });

  it('盲态优先于代测：什么都没测到时不能显示「相机代测」', () => {
    const b = deriveHud(Infinity, 0, false, false, true, false);
    expect(b.level).toBe(HudThreat.Blind);
    expect(b.label).toBe('测距不可用');
  });

  it('严重度序单调递增（下游按枚举数值做边沿比较）', () => {
    expect(HudThreat.Safe).toBeLessThan(HudThreat.Blind);
    expect(HudThreat.Blind).toBeLessThan(HudThreat.Caution);
    expect(HudThreat.Caution).toBeLessThan(HudThreat.Warning);
    expect(HudThreat.Warning).toBeLessThan(HudThreat.Danger);
  });

  it('可信前提下原有阈值口径零回归', () => {
    expect(deriveHud(2.0, 0, false, true, false, false).level).toBe(HudThreat.Safe);
    expect(deriveHud(1.2, 0.3, false, true, false, false).level).toBe(HudThreat.Caution);
    expect(deriveHud(0.8, 0.6, false, true, false, false).level).toBe(HudThreat.Warning);
    expect(deriveHud(1.6, 0.1, false, true, false, false).level).toBe(HudThreat.Safe);
    expect(deriveHud(0.3, 1.0, false, true, false, false).level).toBe(HudThreat.Danger);
  });
});
describe('HUD 抑制态 suppressed（前方不可判）', () => {
  it('同一个 Infinity + 测距可信：无抑制 → Safe/∞，有抑制 → 前方不可判/—', () => {
    const clear = deriveHud(Infinity, 0, false, true, false, false);
    const sup = deriveHud(Infinity, 0, false, true, false, true);
    // 「真的畅通」：老老实实的绿色安全
    expect(clear.level).toBe(HudThreat.Safe);
    expect(clear.label).toBe('安全');
    expect(clear.distanceText).toBe('∞');
    expect(clear.suppressed).toBe(false);
    // 「测到了但命中全被剔除」：等级不变（不得凭空多响一次铃），但文案必须改口
    expect(sup.level).toBe(HudThreat.Safe);
    expect(sup.label).toBe('前方不可判');
    expect(sup.distanceText).toBe('—');
    expect(sup.colorHex).toBe(HUD_COLOR_BLIND);
    expect(sup.suppressed).toBe(true);
  });

  it('抑制态绝不显示成安全，也不报出一个具体读数', () => {
    const sup = deriveHud(Infinity, 0, false, true, false, true);
    expect(sup.colorHex).not.toBe(HUD_COLOR_SAFE);
    expect(sup.distanceText).not.toBe('∞');
  });

  it('抑制 + 代测同时为真 → 抑制优先（判据把障碍丢了，比量的地方不对更接近事故）', () => {
    const both = deriveHud(Infinity, 0, false, true, true, true);
    expect(both.label).toBe('前方不可判');
  });

  it('抑制不得盖掉真实读数：同帧有有限距离时照旧报数', () => {
    const withReading = deriveHud(0.8, 0.6, false, true, false, true);
    expect(withReading.label).toBe('接近');
    expect(withReading.distanceText).toBe('0.80 m');
  });

  it('Blind / Danger 优先于抑制（语义更强的成因不被降级覆盖）', () => {
    const blind = deriveHud(Infinity, 0, false, false, false, true);
    expect(blind.level).toBe(HudThreat.Blind);
    expect(blind.label).toBe('测距不可用');
    const danger = deriveHud(0.2, 1.0, false, true, false, true);
    expect(danger.level).toBe(HudThreat.Danger);
    expect(danger.label).toBe('危险');
  });

  it('反向断言：suppressed 为 false 时日常畅通不得被误报成「前方不可判」', () => {
    for (const d of [Infinity, 5.0, 2.0, 1.6]) {
      const h = deriveHud(d, 0, false, true, false, false);
      expect(h.label).not.toBe('前方不可判');
    }
    expect(deriveHud(Infinity, 0, false, true, false, false).label).toBe('安全');
  });
});
