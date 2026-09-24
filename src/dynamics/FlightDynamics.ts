/**
 * 刚体飞行动力学（Layer 5.6 · Flight Dynamics）
 * ===============================================================
 * 把「四个桨的推力」变成「位姿随时间的演化」——这是与旧版**运动学**模型
 * （直接对输入积分的速度、用速度反推姿态）最本质的区别：
 *
 *   旧：输入 → 目标速度 → 速度 → 位置；姿态是「画」上去的装饰
 *   新：四桨推力 → 力矩 → 角加速度 → 姿态；
 *                总推力（沿机体上轴） + 重力 + 气动阻力 → 加速度 → 速度 → 位置
 *
 * 于是这些东西不再是「补丁」，而是**自然涌现**的：
 *   · 迎风倾斜：位置环要产生水平力 → 只能倾斜推力矢量 → 姿态自动倾
 *   · 抗风保持：相对气流阻力把飞机吹偏 → 控制器反向补偿 → 稳住位置
 *   · 空气密度影响升力：ρ 小 → 同样转速升力小 → 必须提转速
 *   · 倾斜必须提转速：T = mg / cosθ（倾角越大，维持高度所需推力越大）
 *
 * 平动方程（AR 世界系，X 右 / Y 上 / Z 朝用户）：
 *   F_thrust = T · û,  û = 机体上轴在世界系的单位向量
 *   F_drag   = −c · ρ_rel · |v − v_wind| · (v − v_wind)     ← 相对气流，不是绝对速度
 *   a        = (F_thrust + F_drag) / m + (0, −g, 0)
 *
 * 转动方程（小角度线性化，倾角 ≤ 20° 时误差 < 2%）：
 *   α = τ / I − c_ω · ω            ← 含气动角阻尼
 *   ω += α·h ;  angle += ω·h
 *
 * 积分器：半隐式欧拉（先更新速度/角速度，再用新速度更新位置/角度），
 * 在 240Hz 子步下对本文的刚度完全够用，且不会像显式欧拉那样发散。
 */
import { Logger } from '../core/Logger';
import {
  DEFAULT_QUAD,
  QuadGeometry,
  torquesFromThrusts
} from './RotorMixer';

/** 刚体与气动参数 */
export interface DynamicsParams {
  /** 整机质量（kg） */
  mass: number;
  /** 重力加速度（m/s²） */
  gravity: number;
  /** 臂长（m）：同时供混控器使用 */
  armLength: number;
  /** 推力系数 kT（N） */
  kT: number;
  /** 反扭矩系数 kQ（N·m） */
  kQ: number;
  /** 归一化转速上限 */
  maxRotor: number;
  /** 俯仰/滚转转动惯量（kg·m²） */
  inertiaTilt: number;
  /** 偏航转动惯量（kg·m²） */
  inertiaYaw: number;
  /** 气动角阻尼系数（1/s）：抑制角速度，等效真实机身的阻尼力矩 */
  angularDamping: number;
  /** 平动阻力系数 c（N·(s/m)²）：已含 ½·Cd·A·ρ₀ */
  dragCoef: number;
  /** 最大允许倾角（rad）：期望倾角解算后的硬限幅 */
  maxTiltRad: number;
}

/** 默认动力学参数（整机含桨约 0.37m，取 m≈0.12kg 量级） */
export const DEFAULT_DYNAMICS: DynamicsParams = {
  mass: 0.12,
  gravity: 9.81,
  armLength: DEFAULT_QUAD.armLength,
  kT: DEFAULT_QUAD.kT,
  kQ: DEFAULT_QUAD.kQ,
  maxRotor: DEFAULT_QUAD.maxRotor,
  // 4 个 0.03kg 桨组分布在臂长 0.12m 处：I ≈ m·L² 量级
  inertiaTilt: 1.4e-3,
  inertiaYaw: 2.6e-3,
  angularDamping: 6.0,
  // 5m/s 相对气流 → 0.1N 阻力（≈0.83 m/s²，约 5° 倾角即可抵消），量级真实且可见
  dragCoef: 0.004,
  maxTiltRad: 20 * Math.PI / 180
};

function clamp(v: number, lo: number, hi: number): number {
  if (v < lo) {
    return lo;
  }
  if (v > hi) {
    return hi;
  }
  return v;
}

/** 把 DynamicsParams 折算成混控器需要的几何 */
export function geometryOf(p: DynamicsParams): QuadGeometry {
  const g: QuadGeometry = {
    armLength: p.armLength,
    kT: p.kT,
    kQ: p.kQ,
    maxRotor: p.maxRotor
  };
  return g;
}

/**
 * 刚体状态 + 积分器。
 * 状态（速度 / 姿态 / 角速度）由本类独占持有，DroneController 只负责
 * 「读出 → 写入自己的公开字段」与「把自己的锚点/环境量喂进来」。
 */
export class FlightDynamics {
  /** 世界系速度（m/s） */
  public vx: number = 0;
  public vy: number = 0;
  public vz: number = 0;

  /** 姿态（rad）：pitch 正=机头上仰，roll 正=机体右倾 */
  public pitch: number = 0;
  public roll: number = 0;
  public yaw: number = 0;

  /** 机体角速度（rad/s）：绕机体右轴 / 前轴 / 竖轴 */
  public wPitch: number = 0;
  public wRoll: number = 0;
  public wYaw: number = 0;

  /** 最近一步由四桨实际产生的三轴力矩（N·m），供 HUD/调试读数 */
  public lastTauPitch: number = 0;
  public lastTauRoll: number = 0;
  public lastTauYaw: number = 0;
  /** 最近一步的总推力（N） */
  public lastThrust: number = 0;

  private params: DynamicsParams;
  private geo: QuadGeometry;

  constructor(params: DynamicsParams) {
    this.params = params;
    this.geo = geometryOf(params);
  }

  /** 复位到静止水平悬停态 */
  public reset(): void {
    this.vx = 0;
    this.vy = 0;
    this.vz = 0;
    this.pitch = 0;
    this.roll = 0;
    this.yaw = 0;
    this.wPitch = 0;
    this.wRoll = 0;
    this.wYaw = 0;
    this.lastTauPitch = 0;
    this.lastTauRoll = 0;
    this.lastTauYaw = 0;
    this.lastThrust = 0;
  }

  /** 用外部状态覆盖刚体状态（模式切换时无缝接管，避免跳变） */
  public syncFrom(vx: number, vy: number, vz: number,
    pitch: number, roll: number, yaw: number): void {
    this.vx = vx;
    this.vy = vy;
    this.vz = vz;
    this.pitch = pitch;
    this.roll = roll;
    this.yaw = yaw;
    this.wPitch = 0;
    this.wRoll = 0;
    this.wYaw = 0;
  }

  /** 机体上轴在世界系的单位向量 û = R(yaw)·R(pitch)·R(roll)·[0,1,0] */
  public upVector(): number[] {
    const sp: number = Math.sin(this.pitch);
    const cp: number = Math.cos(this.pitch);
    const sr: number = Math.sin(this.roll);
    const cr: number = Math.cos(this.roll);
    const sy: number = Math.sin(this.yaw);
    const cy: number = Math.cos(this.yaw);
    const ux: number = cy * sr + sy * sp * cr;
    const uy: number = cp * cr;
    const uz: number = -sy * sr + cy * sp * cr;
    return [ux, uy, uz];
  }

  /**
   * 推进一个子步。
   * @param h 子步时长（秒）—— 应由调用方按内环频率切分，建议 ≤ 1/240
   * @param thrusts 四个桨的**实际**推力（N，已含执行器饱和）；顺序 [FL,FR,BL,BR]
   * @param windX/windZ 环境风速矢量（m/s，AR 世界系）
   * @param airDensity 空气密度相对因子（标准海平面 = 1.0）
   */
  public step(h: number, thrusts: number[], windX: number, windZ: number,
    airDensity: number): void {
    const p: DynamicsParams = this.params;

    // ── 转动：四桨实际力矩 → 角加速度 → 角速度 → 姿态 ──
    // 注意用**实际**推力反解力矩（而非控制器想要的力矩），
    // 因此执行器饱和会真实地削弱控制力，这是物理诚实的关键。
    const tau: number[] = torquesFromThrusts(thrusts, this.geo);
    this.lastTauPitch = tau[0];
    this.lastTauRoll = tau[1];
    this.lastTauYaw = tau[2];

    const aPitch: number = tau[0] / p.inertiaTilt - p.angularDamping * this.wPitch;
    const aRoll: number = tau[1] / p.inertiaTilt - p.angularDamping * this.wRoll;
    const aYaw: number = tau[2] / p.inertiaYaw - p.angularDamping * this.wYaw;

    this.wPitch += aPitch * h;
    this.wRoll += aRoll * h;
    this.wYaw += aYaw * h;
    this.pitch += this.wPitch * h;
    this.roll += this.wRoll * h;
    this.yaw += this.wYaw * h;

    // ── 平动：总推力（沿机体上轴） + 重力 + 相对气流阻力 ──
    let T: number = 0;
    for (let i = 0; i < 4; i++) {
      T += thrusts[i];
    }
    this.lastThrust = T;

    const u: number[] = this.upVector();

    const rvx: number = this.vx - windX;
    const rvy: number = this.vy;
    const rvz: number = this.vz - windZ;
    const rs: number = Math.sqrt(rvx * rvx + rvy * rvy + rvz * rvz);
    const dk: number = p.dragCoef * clamp(airDensity, 0.4, 1.6);
    const fdx: number = -dk * rs * rvx;
    const fdy: number = -dk * rs * rvy;
    const fdz: number = -dk * rs * rvz;

    const m: number = p.mass;
    const ax: number = (T * u[0] + fdx) / m;
    const ay: number = (T * u[1] + fdy) / m - p.gravity;
    const az: number = (T * u[2] + fdz) / m;

    this.vx += ax * h;
    this.vy += ay * h;
    this.vz += az * h;
  }

  /** 当前悬停所需总推力（N）= mg / cos(倾角) —— 用于诊断读数 */
  public hoverThrust(): number {
    const u: number[] = this.upVector();
    const uy: number = u[1] > 0.05 ? u[1] : 0.05;
    return this.params.mass * this.params.gravity / uy;
  }

  /**
   * 纯函数自检。覆盖：无风悬停平衡、自由落体、风致漂移方向、抬头→后退。
   */
  public static selfTest(): void {
    const fails: string[] = [];
    let total: number = 0;
    const check = (name: string, ok: boolean): void => {
      total++;
      if (!ok) {
        fails.push(name);
      }
    };
    const p: DynamicsParams = DEFAULT_DYNAMICS;
    const hoverPerRotor: number = p.mass * p.gravity / 4;
    const hoverThrusts: number[] = [hoverPerRotor, hoverPerRotor, hoverPerRotor, hoverPerRotor];
    const h: number = 1 / 240;

    // 1) 水平 + 四桨等推力 mg/4 → 悬停平衡，速度与姿态都不漂
    {
      const d: FlightDynamics = new FlightDynamics(p);
      for (let i = 0; i < 480; i++) {
        d.step(h, hoverThrusts, 0, 0, 1.0);
      }
      const spd: number = Math.sqrt(d.vx * d.vx + d.vy * d.vy + d.vz * d.vz);
      check('无风悬停 → 速度保持 0', spd < 1e-9);
      check('无风悬停 → 姿态保持 0',
        Math.abs(d.pitch) < 1e-9 && Math.abs(d.roll) < 1e-9);
    }

    // 2) 四桨停转 → 自由落体（0.25s 后 vy 明显为负；因有气动阻力，略慢于纯 -g·t）
    {
      const d: FlightDynamics = new FlightDynamics(p);
      let v1: number = 0;
      for (let i = 0; i < 60; i++) {
        d.step(h, [0, 0, 0, 0], 0, 0, 1.0);
        v1 = d.vy;
      }
      check('停桨自由落体 → 0.25s 后 vy 明显为负', v1 < -2.0);
      check('停桨自由落体 → 阻力使下落略慢于纯重力', v1 > -p.gravity * 0.25 - 0.02);
    }

    // 3) 风从 −X 吹向 +X（windX>0）：静止的机身被吹向 +X（vx 增大）
    {
      const d: FlightDynamics = new FlightDynamics(p);
      for (let i = 0; i < 240; i++) {
        d.step(h, hoverThrusts, 5.0, 0, 1.0);
      }
      check('顺风 → 被吹向 +X（vx>0）', d.vx > 0.05);
      check('顺风 → 不产生竖直漂移（|vy| 很小）', Math.abs(d.vy) < 0.05);
    }

    // 4) 抬头（pitch>0）→ 推力矢量偏向机尾(+Z) → 产生 +Z 速度（后退）
    {
      const d: FlightDynamics = new FlightDynamics(p);
      d.pitch = 10 * Math.PI / 180;
      d.step(h, hoverThrusts, 0, 0, 1.0);
      check('抬头 → 产生 +Z 速度（后退）', d.vz > 1e-6);
      check('抬头 → 竖直分力减小（uy<1）', d.upVector()[1] < 1);
    }

    // 5) 正俯仰力矩 → 机头上仰（角速度为增方向）
    {
      const d: FlightDynamics = new FlightDynamics(p);
      const t: number[] = [hoverPerRotor + 0.08, hoverPerRotor + 0.08,
        hoverPerRotor - 0.08, hoverPerRotor - 0.08];
      d.step(h, t, 0, 0, 1.0);
      check('前桨加力 → 机头上仰（pitch>0）', d.pitch > 0);
    }

    if (fails.length !== 0) {
      throw new Error(`FlightDynamics 自检失败：${fails.join('、')}`);
    }
    Logger.info(`[VERIFY] FlightDynamics 刚体动力学自检 PASS（${total} 项断言全通过）`);
  }
}
