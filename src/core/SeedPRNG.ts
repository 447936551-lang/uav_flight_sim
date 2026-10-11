/**
 * SeedPRNG —— 可种子化伪随机数发生器（M4·L3 湍流 / L6 传感器噪声 共用）。
 * =====================================================================
 * 设计要点：
 *  - 算法：Mulberry32（32-bit 状态，快速、统计性质良好）+ Box-Muller 高斯。
 *  - 关键不变量：**相同 seed → 完全相同序列**。这是「可复现湍流 / 可复现
 *    传感器噪声」的基础，也是离线回放与回归测试的硬前提。
 *  - 不依赖任何平台随机源（Math.random 不可复现，禁用）。
 *  - 全部纯函数数学，无副作用，可在 node --experimental-strip-types 下直接跑 selfTest。
 *
 * 复用约定：
 *  - L3 湍流风（TurbulenceModel）用它生成高斯白噪声 → 经一阶马尔可夫着色。
 *  - L6 传感器噪声（SensorModel，阶段D）用它生成带偏置/丢包的高斯噪声。
 *  两侧用同一份实现，保证「同一 seed 下仿真与传感器观测一致可复现」。
 */

import { Logger } from './Logger';

/** 2^32，用于把 uint32 归一化到 [0,1)。 */
const U32_MAX_F: number = 4294967296;

export class SeedPRNG {
  private s: number;
  private hasSpare: boolean = false;
  private spare: number = 0;

  constructor(seed: number) {
    this.s = seed >>> 0;
  }

  /** 复位到指定种子（重复实验用）。同时清空高斯 spare，保证序列严格复位。 */
  public reset(seed: number): void {
    this.s = seed >>> 0;
    this.hasSpare = false;
    this.spare = 0;
  }

  /** 下一个 uint32 ∈ [0, 2^32)。Mulberry32 一步。 */
  public nextU32(): number {
    this.s = (this.s + 0x6D2B79F5) >>> 0;
    let t: number = this.s;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0);
  }

  /** 下一个 float ∈ [0, 1)。 */
  public nextFloat(): number {
    return this.nextU32() / U32_MAX_F;
  }

  /** 下一个 float ∈ [min, max)。 */
  public nextRange(min: number, max: number): number {
    return min + (max - min) * this.nextFloat();
  }

  /**
   * 下一个标准正态样本 N(0,1)（Box-Muller，带 spare 缓存避免浪费）。
   * 注意：spare 的存在意味着 nextFloat 与 nextGaussian 交错调用时，
   * 序列仍完全确定（调用顺序固定 → 输出固定）。
   */
  public nextGaussian(): number {
    if (this.hasSpare) {
      this.hasSpare = false;
      return this.spare;
    }
    let u1: number = 0;
    // u1 必须 > 0（log(0) 无意义）；极小概率重试，不影响确定性。
    do {
      u1 = this.nextFloat();
    } while (u1 <= 1e-12);
    const u2: number = this.nextFloat();
    const r: number = Math.sqrt(-2 * Math.log(u1));
    const theta: number = 2 * Math.PI * u2;
    const g1: number = r * Math.cos(theta);
    const g2: number = r * Math.sin(theta);
    this.hasSpare = true;
    this.spare = g2;
    return g1;
  }

  /** 自检：确定性 + 不同种子差异 + 高斯统计。失败抛 Error（由 ARDroneSession 捕获记日志）。 */
  public static selfTest(): void {
    const fails: string[] = [];
    let total: number = 0;
    const check = (name: string, ok: boolean): void => {
      total++;
      if (!ok) {
        fails.push(name);
      }
    };

    // 1) 确定性：同种子 → 完全相同序列
    {
      const a: SeedPRNG = new SeedPRNG(12345);
      const b: SeedPRNG = new SeedPRNG(12345);
      let same: boolean = true;
      for (let i = 0; i < 200; i++) {
        if (a.nextFloat() !== b.nextFloat()) {
          same = false;
          break;
        }
      }
      check('同种子 → 序列完全一致', same);

      // reset 后序列必须严格复位
      const c: SeedPRNG = new SeedPRNG(999);
      const d: SeedPRNG = new SeedPRNG(999);
      for (let i = 0; i < 50; i++) {
        c.nextFloat();
      }
      c.reset(999);
      let resame: boolean = true;
      for (let i = 0; i < 50; i++) {
        if (c.nextFloat() !== d.nextFloat()) {
          resame = false;
          break;
        }
      }
      check('reset → 序列严格复位', resame);
    }

    // 2) 不同种子 → 必须产生差异
    {
      const a: SeedPRNG = new SeedPRNG(12345);
      const b: SeedPRNG = new SeedPRNG(54321);
      let diff: boolean = false;
      for (let i = 0; i < 200; i++) {
        if (a.nextFloat() !== b.nextFloat()) {
          diff = true;
          break;
        }
      }
      check('不同种子 → 序列不同', diff);
    }

    // 3) nextFloat 落在 [0,1)
    {
      const p: SeedPRNG = new SeedPRNG(7);
      let inRange: boolean = true;
      for (let i = 0; i < 1000; i++) {
        const v: number = p.nextFloat();
        if (!(v >= 0 && v < 1)) {
          inRange = false;
          break;
        }
      }
      check('nextFloat ∈ [0,1)', inRange);
    }

    // 4) 高斯：均值≈0、方差≈1
    {
      const g: SeedPRNG = new SeedPRNG(777);
      const N: number = 40000;
      let sum: number = 0;
      let sum2: number = 0;
      for (let i = 0; i < N; i++) {
        const x: number = g.nextGaussian();
        sum += x;
        sum2 += x * x;
      }
      const mean: number = sum / N;
      const variance: number = sum2 / N - mean * mean;
      check('高斯均值≈0(|mean|<0.05)', Math.abs(mean) < 0.05);
      check('高斯方差≈1(|var-1|<0.08)', Math.abs(variance - 1.0) < 0.08);
    }

    if (fails.length > 0) {
      throw new Error(`SeedPRNG 自检失败 [${total - fails.length}/${total}]: ${fails.join('; ')}`);
    }
    Logger.info(`[VERIFY] SeedPRNG 自检 PASS（${total} 项断言全通过）`);
  }
}
