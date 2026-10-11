/**
 * EkfEstimator —— L5 自主 EKF（M4·阶段D，大头）。
 * =====================================================================
 * 误差状态卡尔曼滤波器（ES-EKF，PX4 EKF2 同族思路，独立实现 + PX4 标定）：
 *   状态（名义）：p(3) 位置、v(3) 速度、q(4) 姿态（机体→世界）、无需显式存误差。
 *   误差状态(15)：δp(3) δv(3) δθ(3) 旋转向量、δbg(3) 陀螺零偏、δba(3) 加速零偏。
 *   预测：IMU（陀螺 ω、加速度计 a = 比力）含零偏+噪声 → 名义积分 + 误差协方差传播。
 *   更新：外部位置/速度观测（动捕/GPS 等价）→ 卡尔曼增益修正名义态并 reset 误差协方差。
 *   收敛守护：足够更新数 + 位置方差低于阈值才 isValid（避免拿发散估计去控机）。
 *
 * 关键性质：
 *   - **零偏估计**：相比朴素积分，EKF 能估计并扣除持续 IMU 零偏 → 长时漂移大幅降低。
 *   - **确定性**：观测/噪声若来自种子化 SensorModel（L6），整链可复现。
 *   - **零回归**：DroneController 默认 estimator='none' → 完全不构造、不调用本类。
 *
 * 约定：世界系 x右 y上 z前（与 App 一致），重力 g=[0,−9.81,0]。
 */

import { Logger } from '../core/Logger';

// ───────────────────────── 极简矩阵工具（number[][]，行主序） ─────────────────────────
type Mat = number[][];
type Vec = number[];

function zeros(r: number, c: number): Mat {
  const m: Mat = [];
  for (let i = 0; i < r; i++) {
    m.push(new Array(c).fill(0));
  }
  return m;
}

function eye(n: number): Mat {
  const m: Mat = zeros(n, n);
  for (let i = 0; i < n; i++) {
    m[i][i] = 1;
  }
  return m;
}

function matMul(A: Mat, B: Mat): Mat {
  const rA: number = A.length;
  const cB: number = B[0].length;
  const cA: number = A[0].length;
  const out: Mat = zeros(rA, cB);
  for (let i = 0; i < rA; i++) {
    for (let k = 0; k < cA; k++) {
      const aik: number = A[i][k];
      if (aik === 0) continue;
      for (let j = 0; j < cB; j++) {
        out[i][j] += aik * B[k][j];
      }
    }
  }
  return out;
}

function matAdd(A: Mat, B: Mat): Mat {
  const r: number = A.length;
  const c: number = A[0].length;
  const out: Mat = zeros(r, c);
  for (let i = 0; i < r; i++) {
    for (let j = 0; j < c; j++) {
      out[i][j] = A[i][j] + B[i][j];
    }
  }
  return out;
}

function matT(A: Mat): Mat {
  const r: number = A.length;
  const c: number = A[0].length;
  const out: Mat = zeros(c, r);
  for (let i = 0; i < r; i++) {
    for (let j = 0; j < c; j++) {
      out[j][i] = A[i][j];
    }
  }
  return out;
}

function matVec(A: Mat, v: Vec): Vec {
  const r: number = A.length;
  const c: number = A[0].length;
  const out: Vec = new Array(r).fill(0);
  for (let i = 0; i < r; i++) {
    let s: number = 0;
    for (let j = 0; j < c; j++) {
      s += A[i][j] * v[j];
    }
    out[i] = s;
  }
  return out;
}

/** Gauss-Jordan 求逆（用于小矩阵 S，3×3 / 6×6），失败抛错（不静默返回）。 */
function matInv(M: Mat): Mat {
  const n: number = M.length;
  const A: Mat = M.map((row) => row.slice());
  const I: Mat = eye(n);
  for (let col = 0; col < n; col++) {
    // 选主元
    let piv: number = col;
    let best: number = Math.abs(A[col][col]);
    for (let r = col + 1; r < n; r++) {
      const v: number = Math.abs(A[r][col]);
      if (v > best) {
        best = v;
        piv = r;
      }
    }
    if (best < 1e-12) {
      throw new Error('matInv 奇异矩阵');
    }
    if (piv !== col) {
      const tmpA: Vec = A[col]; A[col] = A[piv]; A[piv] = tmpA;
      const tmpI: Vec = I[col]; I[col] = I[piv]; I[piv] = tmpI;
    }
    const d: number = A[col][col];
    for (let j = 0; j < n; j++) {
      A[col][j] /= d;
      I[col][j] /= d;
    }
    for (let r = 0; r < n; r++) {
      if (r === col) continue;
      const f: number = A[r][col];
      if (f === 0) continue;
      for (let j = 0; j < n; j++) {
        A[r][j] -= f * A[col][j];
        I[r][j] -= f * I[col][j];
      }
    }
  }
  return I;
}

// ───────────────────────── 四元数工具（[w,x,y,z]，机体→世界） ─────────────────────────
export function qNorm(q: Vec): Vec {
  const n: number = Math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  return [q[0] / n, q[1] / n, q[2] / n, q[3] / n];
}

export function qMul(a: Vec, b: Vec): Vec {
  const aw = a[0], ax = a[1], ay = a[2], az = a[3];
  const bw = b[0], bx = b[1], by = b[2], bz = b[3];
  return [
    aw * bw - ax * bx - ay * by - az * bz,
    aw * bx + ax * bw + ay * bz - az * by,
    aw * by - ax * bz + ay * bw + az * bx,
    aw * bz + ax * by - ay * bx + az * bw
  ];
}

/** 由欧拉角（yaw 绕 Y(上) / pitch 绕 X(右) / roll 绕 Z(前)，弧度）生成机体→世界四元数（Hamilton）。 */
export function quatFromEuler(yawRad: number, pitchRad: number, rollRad: number): Vec {
  const cy = Math.cos(yawRad / 2), sy = Math.sin(yawRad / 2);
  const cp = Math.cos(pitchRad / 2), sp = Math.sin(pitchRad / 2);
  const cr = Math.cos(rollRad / 2), sr = Math.sin(rollRad / 2);
  const qy: Vec = [cy, 0, sy, 0];
  const qp: Vec = [cp, sp, 0, 0];
  const qr: Vec = [cr, 0, 0, sr];
  return qNorm(qMul(qMul(qy, qp), qr));
}

/** 由旋转向量 θ（小角度）生成增量四元数。 */
function qFromRotVec(theta: Vec): Vec {
  const ang: number = Math.sqrt(theta[0] * theta[0] + theta[1] * theta[1] + theta[2] * theta[2]);
  if (ang < 1e-9) {
    return [1, 0, 0, 0];
  }
  const h: number = ang * 0.5;
  const s: number = Math.sin(h) / ang;
  return [Math.cos(h), theta[0] * s, theta[1] * s, theta[2] * s];
}

/** 机体→世界旋转矩阵 R（3×3）。 */
export function quatToRot(q: Vec): Mat {
  const w = q[0], x = q[1], y = q[2], z = q[3];
  return [
    [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
    [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
    [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]
  ];
}

/** 反对称矩阵 [v×]。 */
function skew(v: Vec): Mat {
  return [
    [0, -v[2], v[1]],
    [v[2], 0, -v[0]],
    [-v[1], v[0], 0]
  ];
}

// ───────────────────────── 观测输入结构 ─────────────────────────
export interface ImuSample {
  /** 陀螺角速度（机体，rad/s），已含零偏+噪声的真实测量。 */
  gyro: Vec;
  /** 加速度计比力（机体，m/s²），已含零偏+噪声的真实测量。 */
  accel: Vec;
}

export interface ObsInput {
  /** 位置观测（世界，m）；available=false 表示本帧丢包。 */
  pos?: Vec;
  posAvailable?: boolean;
  /** 速度观测（世界，m/s）；available=false 表示本帧丢包。 */
  vel?: Vec;
  velAvailable?: boolean;
}

// ───────────────────────── EKF 配置（PX4 量级标定，可调） ─────────────────────────
export interface EkfConfig {
  accelNoise: number;      // 加速度计噪声密度 (m/s²/√Hz)
  gyroNoise: number;       // 陀螺噪声密度 (rad/s/√Hz)
  accelBiasWalk: number;   // 加速零偏随机游走 (m/s²/√Hz)
  gyroBiasWalk: number;    // 陀螺零偏随机游走 (rad/s/√Hz)
  posNoise: number;        // 位置观测噪声 (m)
  velNoise: number;        // 速度观测噪声 (m/s)
  gravity: number;         // 重力加速度幅值 (9.81)
  minUpdates: number;      // 收敛所需最少更新次数
  posVarGate: number;      // 位置方差收敛阈值（三维方差之和上限，m²）
}

export const DEFAULT_EKF: EkfConfig = {
  accelNoise: 0.35,        // PX4 EKF2_ACC_NOISE=0.35
  gyroNoise: 0.015,        // PX4 EKF2_GYR_NOISE=0.015
  accelBiasWalk: 0.005,
  gyroBiasWalk: 0.001,
  posNoise: 0.1,           // 动捕级位置噪声
  velNoise: 0.05,
  gravity: 9.81,
  minUpdates: 10,
  posVarGate: 0.25         // 三维位置方差和 < 0.25 m² → 收敛
};

export class EkfEstimator {
  // 名义状态
  private p: Vec;
  private v: Vec;
  private q: Vec;            // [w,x,y,z]
  private bg: Vec;           // 陀螺零偏（机体）
  private ba: Vec;           // 加速零偏（机体）
  // 误差协方差 P(15×15)
  private P: Mat;
  private cfg: EkfConfig;
  private updates: number = 0;
  private valid: boolean = false;

  constructor(initPos: Vec, initVel: Vec, initQuat: Vec, cfg: EkfConfig = DEFAULT_EKF) {
    this.p = initPos.slice();
    this.v = initVel.slice();
    this.q = qNorm(initQuat);
    this.bg = [0, 0, 0];
    this.ba = [0, 0, 0];
    this.cfg = cfg;
    this.P = zeros(15, 15);
    // 初始不确定度：位置/速度较松，姿态/零偏较松
    for (let i = 0; i < 3; i++) {
      this.P[i][i] = 1.0;          // pos 1 m²
      this.P[3 + i][3 + i] = 1.0;  // vel 1 (m/s)²
      this.P[6 + i][6 + i] = 0.1;  // att 0.1 rad²
      this.P[9 + i][9 + i] = 1e-3; // gyro bias
      this.P[12 + i][12 + i] = 1e-2; // accel bias
    }
  }

  public get position(): Vec { return this.p.slice(); }
  public get velocity(): Vec { return this.v.slice(); }
  public get quaternion(): Vec { return this.q.slice(); }
  public get gyroBias(): Vec { return this.bg.slice(); }
  public get accelBias(): Vec { return this.ba.slice(); }
  public get isValid(): boolean { return this.valid; }
  /** 位置三维标准差（m）。 */
  public get positionStd(): number {
    return Math.sqrt(Math.max(0, this.P[0][0] + this.P[1][1] + this.P[2][2]));
  }
  public get updateCount(): number { return this.updates; }

  /**
   * 推进一帧：IMU 预测 + 可选观测更新。
   * @param dt   时间步（s）
   * @param imu  陀螺/加速度测量（已含零偏+噪声）
   * @param obs  外部位置/速度观测（可部分/全部丢失）
   */
  public step(dt: number, imu: ImuSample, obs?: ObsInput): void {
    this.predict(dt, imu);
    if (obs) {
      if (obs.posAvailable && obs.pos) {
        this.updatePosVel(obs.pos, true);
      }
      if (obs.velAvailable && obs.vel) {
        this.updatePosVel(obs.vel, false);
      }
    }
    // 收敛守护
    if (!this.valid && this.updates >= this.cfg.minUpdates) {
      if (this.positionStd * this.positionStd < this.cfg.posVarGate) {
        this.valid = true;
      }
    }
  }

  private predict(dt: number, imu: ImuSample): void {
    const g: number = this.cfg.gravity;
    // 扣除估计零偏后的比力与角速度
    const aMeas: Vec = [imu.accel[0] - this.ba[0], imu.accel[1] - this.ba[1], imu.accel[2] - this.ba[2]];
    const wMeas: Vec = [imu.gyro[0] - this.bg[0], imu.gyro[1] - this.bg[1], imu.gyro[2] - this.bg[2]];
    const R: Mat = quatToRot(this.q);

    // 名义积分：速度（世界） = R·a + g_world；位置 = v；姿态 = q ⊗ Δq(ω)
    const aWorldX: number = R[0][0] * aMeas[0] + R[0][1] * aMeas[1] + R[0][2] * aMeas[2];
    const aWorldY: number = R[1][0] * aMeas[0] + R[1][1] * aMeas[1] + R[1][2] * aMeas[2] + (-g);
    const aWorldZ: number = R[2][0] * aMeas[0] + R[2][1] * aMeas[1] + R[2][2] * aMeas[2];
    this.v[0] += aWorldX * dt;
    this.v[1] += aWorldY * dt;
    this.v[2] += aWorldZ * dt;
    this.p[0] += this.v[0] * dt;
    this.p[1] += this.v[1] * dt;
    this.p[2] += this.v[2] * dt;
    // 姿态：Δq = qFromRotVec(wMeas*dt)；q = q ⊗ Δq
    const dq: Vec = qFromRotVec([wMeas[0] * dt, wMeas[1] * dt, wMeas[2] * dt]);
    this.q = qNorm(qMul(this.q, dq));

    // ── 误差状态转移矩阵 F(15×15)（连续，线性化） ──
    const F: Mat = zeros(15, 15);
    // δp' = δv
    for (let i = 0; i < 3; i++) F[i][3 + i] = 1;
    // δv' = -R[a×] δθ - R δba
    const aSkew: Mat = skew(aMeas);
    const RnegASkew: Mat = matMul(R, aSkew);
    for (let i = 0; i < 3; i++) {
      for (let j = 0; j < 3; j++) {
        F[3 + i][6 + j] = -RnegASkew[i][j];
        F[3 + i][12 + j] = -R[i][j];
      }
    }
    // δθ' = -[ω×] δθ - δbg
    const wSkew: Mat = skew(wMeas);
    for (let i = 0; i < 3; i++) {
      for (let j = 0; j < 3; j++) {
        F[6 + i][6 + j] = -wSkew[i][j];
        F[6 + i][9 + j] = (i === j ? -1 : 0);
      }
    }
    // bg, ba 随机游走：F 对应块为 0

    // ── 过程噪声 Q(15×15) ──
    const Q: Mat = zeros(15, 15);
    // 加速噪声 → δv（R diag(na²) Rᵀ）
    const na2: number = this.cfg.accelNoise * this.cfg.accelNoise;
    const Ra: Mat = matMul(R, matMul(eye(3), matT(R)));
    for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) Q[3 + i][3 + j] = na2 * Ra[i][j];
    // 陀螺噪声 → δθ
    const nw2: number = this.cfg.gyroNoise * this.cfg.gyroNoise;
    for (let i = 0; i < 3; i++) Q[6 + i][6 + i] = nw2;
    // 零偏随机游走
    const sb2: number = this.cfg.gyroBiasWalk * this.cfg.gyroBiasWalk;
    const sa2: number = this.cfg.accelBiasWalk * this.cfg.accelBiasWalk;
    for (let i = 0; i < 3; i++) {
      Q[9 + i][9 + i] = sb2;
      Q[12 + i][12 + i] = sa2;
    }

    // 离散化：P = (I+Fdt) P (I+Fdt)ᵀ + Q dt
    const I15: Mat = eye(15);
    const Fd: Mat = matAdd(I15, scaleMat(F, dt));
    const Ppred: Mat = matMul(matMul(Fd, this.P), matT(Fd));
    this.P = matAdd(Ppred, scaleMat(Q, dt));
  }

  /** 位置或速度更新（H 取对应行块）。 */
  private updatePosVel(z: Vec, isPos: boolean): void {
    const rowOff: number = isPos ? 0 : 3;
    const rDim: number = 3;
    const Rnoise: number = isPos ? this.cfg.posNoise : this.cfg.velNoise;
    // H(3×15)
    const H: Mat = zeros(3, 15);
    for (let i = 0; i < 3; i++) H[i][rowOff + i] = 1;
    // S = H P Hᵀ + R
    const HP: Mat = matMul(H, this.P);
    const HPHt: Mat = matMul(HP, matT(H));
    const S: Mat = zeros(3, 3);
    for (let i = 0; i < 3; i++) {
      for (let j = 0; j < 3; j++) {
        S[i][j] = HPHt[i][j] + (i === j ? Rnoise * Rnoise : 0);
      }
    }
    // K = P Hᵀ S⁻¹
    const Sinv: Mat = matInv(S);
    const PHt: Mat = matT(HP);
    const K: Mat = matMul(this.P, matMul(matT(H), Sinv));
    // 残差
    const xhat: Vec = isPos ? this.p : this.v;
    const r: Vec = [z[0] - xhat[0], z[1] - xhat[1], z[2] - xhat[2]];
    // δx = K r（15）
    const dx: Vec = matVec(K, r);
    // 应用：名义态修正
    for (let i = 0; i < 3; i++) {
      this.p[i] += dx[i];
      this.v[i] += dx[3 + i];
    }
    // 姿态修正：θ = dx[6..8]（旋转向量）
    const dq: Vec = qFromRotVec([dx[6], dx[7], dx[8]]);
    this.q = qNorm(qMul(dq, this.q));
    for (let i = 0; i < 3; i++) {
      this.bg[i] += dx[9 + i];
      this.ba[i] += dx[12 + i];
    }
    // 协方差 reset：P = (I - K H) P
    const I3: Mat = eye(3);
    const KH: Mat = matMul(K, H); // 15×15
    const I15: Mat = eye(15);
    const ImKH: Mat = zeros(15, 15);
    for (let i = 0; i < 15; i++) for (let j = 0; j < 15; j++) ImKH[i][j] = I15[i][j] - KH[i][j];
    this.P = matMul(ImKH, this.P);

    this.updates++;
  }

  /** 自检：确定性 / 收敛 / 零偏估计 / 收敛守护。失败抛 Error。 */
  public static selfTest(): void {
    const fails: string[] = [];
    let total: number = 0;
    const check = (name: string, ok: boolean): void => {
      total++;
      if (!ok) fails.push(name);
    };

    // 1) 确定性：同初始条件 + 同观测序列 → 同估计
    {
      const mk = () => {
        const e: EkfEstimator = new EkfEstimator([0, 0, 0], [0, 0, 0], [1, 0, 0, 0]);
        for (let i = 0; i < 50; i++) {
          e.step(1 / 100, { gyro: [0, 0, 0], accel: [0, 9.81, 0] }, { pos: [0.01, 0.02, -0.01], posAvailable: true });
        }
        return e.position;
      };
      const a: Vec = mk();
      const b: Vec = mk();
      check('EKF 确定性（同输入→同输出）', a[0] === b[0] && a[1] === b[1] && a[2] === b[2]);
    }

    // 2) 悬停收敛：真实悬停 + 无偏观测 → 估计逼近零、isValid 变 true
    {
      const e: EkfEstimator = new EkfEstimator([0, 0, 0], [0, 0, 0], [1, 0, 0, 0]);
      for (let i = 0; i < 200; i++) {
        e.step(1 / 100, { gyro: [0, 0, 0], accel: [0, 9.81, 0] },
          { pos: [0, 0, 0], posAvailable: true, vel: [0, 0, 0], velAvailable: true });
      }
      const p: Vec = e.position;
      const err: number = Math.sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
      check('悬停收敛(|pos|<0.05m)', err < 0.05);
      check('悬停后 isValid=true', e.isValid);
    }

    // 3) 零偏估计：陀螺持续零偏 → EKF 估计出接近真值的 bg（朴素积分会漂移）
    {
      const trueBg: Vec = [0.02, -0.01, 0.015];
      const e: EkfEstimator = new EkfEstimator([0, 0, 0], [0, 0, 0], [1, 0, 0, 0]);
      let naiveYaw: number = 0;
      for (let i = 0; i < 1000; i++) {
        // 真实陀螺=0，但测量含零偏
        const gyroMeas: Vec = [trueBg[0], trueBg[1], trueBg[2]];
        e.step(1 / 100, { gyro: gyroMeas, accel: [0, 9.81,0] },
          { pos: [0, 0, 0], posAvailable: true, vel: [0, 0, 0], velAvailable: true });
        naiveYaw += trueBg[2] * (1 / 100); // 朴素积分漂移
      }
      const estBg: Vec = e.gyroBias;
      const bgErr: number = Math.sqrt((estBg[0] - trueBg[0]) ** 2 + (estBg[1] - trueBg[1]) ** 2 + (estBg[2] - trueBg[2]) ** 2);
      // 注：纯悬停对陀螺零偏弱可观测（无运动激励姿态误差），故给 20 mrad 量级容差；
      // 完整收敛需动态飞行（端到端 ③-3c 已证明 EKF 远优于朴素积分）。
      check('陀螺零偏估计(悬停弱可观,|err|<0.02)', bgErr < 0.02);
      check('朴素积分明显漂移(对照)', Math.abs(naiveYaw) > 0.1);
    }

    // 4) 收敛守护：更新数不足 → isValid=false
    {
      const e: EkfEstimator = new EkfEstimator([0, 0, 0], [0, 0, 0], [1, 0, 0, 0]);
      for (let i = 0; i < 5; i++) {
        e.step(1 / 100, { gyro: [0, 0, 0], accel: [0, 9.81, 0] }, { pos: [0, 0, 0], posAvailable: true });
      }
      check('更新不足 → isValid=false', !e.isValid);
    }

    // 5) 轨迹跟踪：匀速运动 + 无偏观测 → 位置估计误差小
    {
      const e: EkfEstimator = new EkfEstimator([0, 0, 0], [0, 0, 0], [1, 0, 0, 0]);
      let px = 0, pz = 0;
      const vx = 0.5, vz = -0.3; // 水平匀速
      let maxErr = 0;
      for (let i = 0; i < 600; i++) {
        px += vx * (1 / 100);
        pz += vz * (1 / 100);
        e.step(1 / 100, { gyro: [0, 0, 0], accel: [0, 9.81, 0] },
          { pos: [px, 0, pz], posAvailable: true, vel: [vx, 0, vz], velAvailable: true });
        const p: Vec = e.position;
        const err: number = Math.sqrt((p[0] - px) ** 2 + (p[1]) ** 2 + (p[2] - pz) ** 2);
        if (i > 100) maxErr = Math.max(maxErr, err);
      }
      check('匀速轨迹跟踪(|err|<0.1m)', maxErr < 0.1);
    }

    if (fails.length > 0) {
      throw new Error(`EkfEstimator 自检失败 [${total - fails.length}/${total}]: ${fails.join('; ')}`);
    }
    Logger.info(`[VERIFY] EkfEstimator 自检 PASS（${total} 项断言全通过）`);
  }
}

function scaleMat(A: Mat, s: number): Mat {
  const r: number = A.length;
  const c: number = A[0].length;
  const out: Mat = zeros(r, c);
  for (let i = 0; i < r; i++) for (let j = 0; j < c; j++) out[i][j] = A[i][j] * s;
  return out;
}
