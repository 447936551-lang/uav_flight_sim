/**
 * SensorModel —— L6 传感器噪声模型（M4·阶段D）。
 * =====================================================================
 * 把「真值信号」 corrupt 成「带噪声/零偏/丢包的观测」，供 L5 EKF 消费。
 * 全部由种子化 SeedPRNG 驱动 → 同一 seed 下仿真与观测完全一致可复现。
 *
 * 三类信号：
 *  - IMU（陀螺 ω、加速度计比力 a，均机体坐标系）：常数零偏（初始化时抽一次）+ 白噪声。
 *  - 位置/速度观测（世界系，动捕/GPS 等价）：白噪声 + 丢包（dropRate）。
 *
 * 接线（DroneController）：默认 sensorNoise=false → 把真值原样（或零噪声）喂给 EKF；
 * 开启后走本模型。零回归：sensorNoise=false 时不构造本类。
 */

import { SeedPRNG } from '../core/SeedPRNG';
import { ImuSample } from './EkfEstimator';
import { Logger } from '../core/Logger';

export interface SensorConfig {
  accelStd: number;   // 加速度计白噪声 std (m/s²)
  gyroStd: number;    // 陀螺白噪声 std (rad/s)
  accelBias: number;  // 加速零偏初始 std (m/s²)（抽一次，恒定）
  gyroBias: number;   // 陀螺零偏初始 std (rad/s)（抽一次，恒定）
  posStd: number;     // 位置观测噪声 std (m)
  velStd: number;     // 速度观测噪声 std (m/s)
  dropRate: number;   // 观测丢包概率 (0..1)
}

export const DEFAULT_SENSOR: SensorConfig = {
  accelStd: 0.05,
  gyroStd: 0.005,
  accelBias: 0.02,
  gyroBias: 0.01,
  posStd: 0.05,
  velStd: 0.03,
  dropRate: 0.05
};

type Vec = number[];

/** 位置观测（带丢包标志）。具名接口（ArkTS 禁止内联对象类型注解）。 */
export interface PosObservation {
  pos: number[];
  available: boolean;
}

/** 速度观测（带丢包标志）。 */
export interface VelObservation {
  vel: number[];
  available: boolean;
}

export class SensorModel {
  private prng: SeedPRNG;
  private cfg: SensorConfig;
  private biasA: Vec;
  private biasG: Vec;

  constructor(seed: number, cfg: SensorConfig = DEFAULT_SENSOR) {
    this.cfg = cfg;
    this.prng = new SeedPRNG(seed);
    this.biasA = this.drawBias(cfg.accelBias);
    this.biasG = this.drawBias(cfg.gyroBias);
  }

  private drawBias(std: number): Vec {
    return [this.prng.nextGaussian() * std, this.prng.nextGaussian() * std, this.prng.nextGaussian() * std];
  }

  /** 复位到指定种子（零偏重新抽取，序列严格复位）。 */
  public reset(seed: number): void {
    this.prng.reset(seed);
    this.biasA = this.drawBias(this.cfg.accelBias);
    this.biasG = this.drawBias(this.cfg.gyroBias);
  }

  /** 诊断：当前注入的加速零偏（机体）。 */
  public get accelBiasVec(): Vec { return this.biasA.slice(); }
  /** 诊断：当前注入的陀螺零偏（机体）。 */
  public get gyroBiasVec(): Vec { return this.biasG.slice(); }

  /** 由真值 IMU（机体角速度 + 比力）生成带零偏+噪声的测量。 */
  public imu(truthGyro: Vec, truthAccel: Vec): ImuSample {
    const g: Vec = [
      truthGyro[0] + this.biasG[0] + this.prng.nextGaussian() * this.cfg.gyroStd,
      truthGyro[1] + this.biasG[1] + this.prng.nextGaussian() * this.cfg.gyroStd,
      truthGyro[2] + this.biasG[2] + this.prng.nextGaussian() * this.cfg.gyroStd
    ];
    const a: Vec = [
      truthAccel[0] + this.biasA[0] + this.prng.nextGaussian() * this.cfg.accelStd,
      truthAccel[1] + this.biasA[1] + this.prng.nextGaussian() * this.cfg.accelStd,
      truthAccel[2] + this.biasA[2] + this.prng.nextGaussian() * this.cfg.accelStd
    ];
    return { gyro: g, accel: a };
  }

  /** 由真值位置生成带噪声/丢包的观测。 */
  public observePos(truthPos: Vec): PosObservation {
    if (this.prng.nextFloat() < this.cfg.dropRate) {
      return { pos: [0, 0, 0], available: false };
    }
    return {
      pos: [
        truthPos[0] + this.prng.nextGaussian() * this.cfg.posStd,
        truthPos[1] + this.prng.nextGaussian() * this.cfg.posStd,
        truthPos[2] + this.prng.nextGaussian() * this.cfg.posStd
      ],
      available: true
    };
  }

  /** 由真值速度生成带噪声/丢包的观测。 */
  public observeVel(truthVel: Vec): VelObservation {
    if (this.prng.nextFloat() < this.cfg.dropRate) {
      return { vel: [0, 0, 0], available: false };
    }
    return {
      vel: [
        truthVel[0] + this.prng.nextGaussian() * this.cfg.velStd,
        truthVel[1] + this.prng.nextGaussian() * this.cfg.velStd,
        truthVel[2] + this.prng.nextGaussian() * this.cfg.velStd
      ],
      available: true
    };
  }

  /** 自检：确定性 / 零偏近似 / 丢包率 / 无噪声配置→原样。失败抛 Error。 */
  public static selfTest(): void {
    const fails: string[] = [];
    let total: number = 0;
    const check = (name: string, ok: boolean): void => {
      total++;
      if (!ok) fails.push(name);
    };

    // 1) 确定性：同种子 → 同首帧 IMU
    {
      const a: SensorModel = new SensorModel(123);
      const b: SensorModel = new SensorModel(123);
      const ia: ImuSample = a.imu([0, 0, 0], [0, 9.81, 0]);
      const ib: ImuSample = b.imu([0, 0, 0], [0, 9.81, 0]);
      check('同种子 → IMU 一致', ia.accel[1] === ib.accel[1] && ia.gyro[0] === ib.gyro[0]);
    }

    // 2) 零偏近似：长程均值(accel - truth) ≈ 注入零偏（噪声均值 0）
    {
      const s: SensorModel = new SensorModel(7, { accelStd: 0.01, gyroStd: 0.001, accelBias: 0.05, gyroBias: 0.02, posStd: 0.05, velStd: 0.03, dropRate: 0 });
      let sx = 0;
      const N = 5000;
      for (let i = 0; i < N; i++) {
        const m: ImuSample = s.imu([0, 0, 0], [0, 0, 0]);
        sx += m.accel[0];
      }
      const mean: number = sx / N;
      // 与抽中的真实零偏对比（噪声均值 0 → 长程均值≈bias）；不再依赖 bias 抽中值恰好很小
      check('加速零偏注入正确(|mean-bias|<0.01)', Math.abs(mean - s.accelBiasVec[0]) < 0.01);
    }

    // 3) 丢包：dropRate=1 → 恒不可用；=0 → 恒可用
    {
      const always: SensorModel = new SensorModel(1, { accelStd: 0, gyroStd: 0, accelBias: 0, gyroBias: 0, posStd: 0, velStd: 0, dropRate: 1 });
      const never: SensorModel = new SensorModel(1, { accelStd: 0, gyroStd: 0, accelBias: 0, gyroBias: 0, posStd: 0, velStd: 0, dropRate: 0 });
      check('dropRate=1 → 恒丢包', !always.observePos([0, 0, 0]).available);
      check('dropRate=0 → 恒可用', never.observePos([0, 0, 0]).available);
    }

    // 4) 无噪声无零偏（dropRate=0）→ 原样
    {
      const s: SensorModel = new SensorModel(3, { accelStd: 0, gyroStd: 0, accelBias: 0, gyroBias: 0, posStd: 0, velStd: 0, dropRate: 0 });
      const m: ImuSample = s.imu([0.1, 0.2, 0.3], [1, 2, 3]);
      check('无噪声 → IMU 原样', m.gyro[0] === 0.1 && m.accel[2] === 3);
      const o = s.observePos([4, 5, 6]);
      check('无噪声 → 位置原样', o.pos[0] === 4 && o.available);
    }

    if (fails.length > 0) {
      throw new Error(`SensorModel 自检失败 [${total - fails.length}/${total}]: ${fails.join('; ')}`);
    }
    Logger.info(`[VERIFY] SensorModel 自检 PASS（${total} 项断言全通过）`);
  }
}
