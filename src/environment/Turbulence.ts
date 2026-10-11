/**
 * Turbulence —— L3 种子化湍流风（M4·阶段C）。
 * =====================================================================
 * 物理模型：每轴用一阶马尔可夫（Ornstein-Uhlenbeck）过程为高斯白噪声「着色」，
 * 得到相关时间 τ 的湍流风扰动（m/s）：
 *     dX = -(1/τ)·X dt + σ·√(2/τ) dW
 * 稳态方差 = σ²，σ 由 intensity 缩放（intensity=1 → σ=baseSigma）。
 * 离散用**精确解**（指数衰减 + 增量正态），稳态方差严格 = σ²，与步长无关。
 *
 * 为什么种子化：
 *  - 相同 seed → 完全相同湍流序列 → 仿真可复现、回归可对比、回放可对齐。
 *  - 同一份 SeedPRNG 也供 L6 传感器噪声使用，保证「仿真态与观测态同源可复现」。
 *
 * 接线（DroneController）：开启 `turbulence` 后，子步循环里
 *   `turbModel.update(h)` 取水平风扰动并加进 windVelX/windVelZ 再传 dyn.step。
 * 默认关 → 与现有风场零回归。
 */

import { SeedPRNG } from '../core/SeedPRNG';
import { Logger } from '../core/Logger';

/** 基准湍流配置：intensity=1 时稳态标准差 1.2 m/s（中等湍流），τ=0.5s 平滑。 */
export const DEFAULT_TURBULENCE_SIGMA: number = 1.2;
export const DEFAULT_TURBULENCE_TAU: number = 0.5;

/** 湍流瞬时水平风扰动（m/s）。具名接口（ArkTS 禁止内联对象类型注解）。 */
export interface TurbWind {
  wx: number;
  wz: number;
}

export class TurbulenceModel {
  private prng: SeedPRNG;
  private tau: number;
  private baseSigma: number;
  private sigma: number;
  private wx: number = 0;
  private wz: number = 0;

  /**
   * @param seed       随机种子（复现用）
   * @param intensity  σ 缩放（0=关；1=基准；>1 更强阵风）
   * @param tau        相关时间常数（秒），越大风越平滑
   * @param baseSigma  intensity=1 时的稳态标准差（m/s）
   */
  constructor(seed: number, intensity: number = 1.0, tau: number = DEFAULT_TURBULENCE_TAU,
    baseSigma: number = DEFAULT_TURBULENCE_SIGMA) {
    this.prng = new SeedPRNG(seed);
    this.tau = tau;
    this.baseSigma = baseSigma;
    this.sigma = baseSigma * intensity;
  }

  /** 运行时调节强度（不重置状态，平滑过渡）。 */
  public setIntensity(intensity: number): void {
    this.sigma = this.baseSigma * intensity;
  }

  /** 复位到指定种子（重复实验：状态与 PRNG 同时清零）。 */
  public reset(seed: number): void {
    this.prng.reset(seed);
    this.wx = 0;
    this.wz = 0;
  }

  /**
   * 推进一个时间步，返回该步的水平风扰动 (m/s)。
   * 精确离散 OU：衰减 + 增量正态，稳态方差严格 = σ²（与 dt 无关）。
   */
  public update(dt: number): TurbWind {
    if (this.tau <= 1e-6) {
      // 退化配置：无相关时间 → 纯白噪声（仍确定性、仍 σ 量级）
      this.wx = this.sigma * this.prng.nextGaussian();
      this.wz = this.sigma * this.prng.nextGaussian();
      return { wx: this.wx, wz: this.wz };
    }
    const decay: number = Math.exp(-dt / this.tau);
    const q: number = Math.sqrt(1 - Math.exp(-2 * dt / this.tau));
    const incX: number = this.sigma * q * this.prng.nextGaussian();
    const incZ: number = this.sigma * q * this.prng.nextGaussian();
    this.wx = this.wx * decay + incX;
    this.wz = this.wz * decay + incZ;
    return { wx: this.wx, wz: this.wz };
  }

  /** 供诊断：当前瞬时扰动（不含均值风）。 */
  public get current(): TurbWind {
    return { wx: this.wx, wz: this.wz };
  }
}

/** 自检：确定性 / 不同种子差异 / 零均值 / 稳态方差≈σ² / intensity 缩放。失败抛 Error。 */
export function selfTest(): void {
  const fails: string[] = [];
  let total: number = 0;
  const check = (name: string, ok: boolean): void => {
    total++;
    if (!ok) {
      fails.push(name);
    }
  };

  // 1) 确定性：同 seed + 同 dt 序列 → 完全相同
  {
    const a: TurbulenceModel = new TurbulenceModel(2026, 1.0, 0.5, 1.2);
    const b: TurbulenceModel = new TurbulenceModel(2026, 1.0, 0.5, 1.2);
    let same: boolean = true;
    for (let i = 0; i < 300; i++) {
      const pa: TurbWind = a.update(1 / 240);
      const pb: TurbWind = b.update(1 / 240);
      if (pa.wx !== pb.wx || pa.wz !== pb.wz) {
        same = false;
        break;
      }
    }
    check('同种子 → 湍流序列完全一致', same);

    // reset 后严格复位
    const c: TurbulenceModel = new TurbulenceModel(55, 1.0, 0.5, 1.2);
    const d: TurbulenceModel = new TurbulenceModel(55, 1.0, 0.5, 1.2);
    for (let i = 0; i < 100; i++) {
      c.update(1 / 240);
    }
    c.reset(55);
    let resame: boolean = true;
    for (let i = 0; i < 100; i++) {
      const pc: TurbWind = c.update(1 / 240);
      const pd: TurbWind = d.update(1 / 240);
      if (pc.wx !== pd.wx || pc.wz !== pd.wz) {
        resame = false;
        break;
      }
    }
    check('reset → 湍流序列严格复位', resame);
  }

  // 2) 不同种子 → 必须不同
  {
    const a: TurbulenceModel = new TurbulenceModel(11, 1.0, 0.5, 1.2);
    const b: TurbulenceModel = new TurbulenceModel(22, 1.0, 0.5, 1.2);
    let diff: boolean = false;
    for (let i = 0; i < 300; i++) {
      const pa: TurbWind = a.update(1 / 240);
      const pb: TurbWind = b.update(1 / 240);
      if (pa.wx !== pb.wx || pa.wz !== pb.wz) {
        diff = true;
        break;
      }
    }
    check('不同种子 → 湍流序列不同', diff);
  }

  // 3) 零均值 + 稳态标准差 ≈ σ（long run）
  {
    const m: TurbulenceModel = new TurbulenceModel(31415, 1.0, 0.5, 1.2);
    const N: number = 60000;
    let sx = 0, s2x = 0, sz = 0, s2z = 0;
    for (let i = 0; i < N; i++) {
      const p: TurbWind = m.update(1 / 240);
      sx += p.wx; s2x += p.wx * p.wx;
      sz += p.wz; s2z += p.wz * p.wz;
    }
    const meanX: number = sx / N;
    const meanZ: number = sz / N;
    const stdX: number = Math.sqrt(s2x / N - meanX * meanX);
    const stdZ: number = Math.sqrt(s2z / N - meanZ * meanZ);
    check('湍流 X 零均值(|mean|<0.08)', Math.abs(meanX) < 0.08);
    check('湍流 Z 零均值(|mean|<0.08)', Math.abs(meanZ) < 0.08);
    check('湍流 X 稳态σ≈1.2(|σ-1.2|<0.1)', Math.abs(stdX - 1.2) < 0.1);
    check('湍流 Z 稳态σ≈1.2(|σ-1.2|<0.1)', Math.abs(stdZ - 1.2) < 0.1);
  }

  // 4) intensity 缩放：intensity=2 → 稳态σ≈2×
  {
    const m1: TurbulenceModel = new TurbulenceModel(900, 1.0, 0.5, 1.2);
    const m2: TurbulenceModel = new TurbulenceModel(900, 2.0, 0.5, 1.2);
    const N: number = 60000;
    let s2a = 0, s2b = 0;
    for (let i = 0; i < N; i++) {
      const p1: TurbWind = m1.update(1 / 240);
      const p2: TurbWind = m2.update(1 / 240);
      s2a += p1.wx * p1.wx;
      s2b += p2.wx * p2.wx;
    }
    const std1: number = Math.sqrt(s2a / N);
    const std2: number = Math.sqrt(s2b / N);
    check('intensity 缩放 2× → σ≈2×', Math.abs(std2 - 2 * std1) < 0.15);
  }

  // 5) 有界性：长程最大扰动应在合理倍数 σ 内（极端异常保护）
  {
    const m: TurbulenceModel = new TurbulenceModel(4242, 1.0, 0.5, 1.2);
    let maxAbs: number = 0;
    for (let i = 0; i < 200000; i++) {
      const p: TurbWind = m.update(1 / 240);
      maxAbs = Math.max(maxAbs, Math.abs(p.wx), Math.abs(p.wz));
    }
    check('湍流有界(|w|<6σ 长程)', maxAbs < 6 * 1.2);
  }

  if (fails.length > 0) {
    throw new Error(`Turbulence 自检失败 [${total - fails.length}/${total}]: ${fails.join('; ')}`);
  }
  Logger.info(`[VERIFY] Turbulence 自检 PASS（${total} 项断言全通过）`);
}
