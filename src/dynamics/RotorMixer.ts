/**
 * 四旋翼混控器（Layer 5.5 · 执行器分配 / Actuator Allocation）
 * ===============================================================
 * 真实飞控链路的最后一环：上层只给「总推力 + 三轴期望力矩」，
 * 「能不能实现、怎么分摊到四个电机」由混控器负责 —— 就是常说的
 * 姿态 PID → 混控器（Mixer）→ 四路 PWM。
 *
 * 输入/输出都是**物理量**（N、N·m），不掺任何渲染语义，
 * 因此它可以像 CollisionDetector / SteeringBehavior 一样被纯函数自检。
 *
 * 桨位编号（X 型布局，俯视，机头朝上）：
 *
 *        FL(0)      FR(1)
 *           \        /
 *            \      /
 *            [机身]
 *            /      \
 *           /        \
 *        BL(2)      BR(3)
 *
 * 机体坐标系（与 DroneController 完全一致）：
 *   +X = 右    +Y = 上    +Z = 机尾（机头方向为 -Z）
 * 桨位坐标（臂长 L）：
 *   FL(-L, 0, -L)   FR(+L, 0, -L)   BL(-L, 0, +L)   BR(+L, 0, +L)
 *
 * 分配公式（Walsh–Hadamard 变换，正交且**可逆**，因此「反向解算出的
 * 力矩」与「正向分配的输入」在未饱和时严格相等 —— 这正是自检要覆盖的点）：
 *   a = τ_pitch / L          （俯仰力偶强度，N）
 *   b = τ_roll  / L          （滚转力偶强度，N）
 *   c = τ_yaw · kT / kQ      （偏航力偶强度，N）
 *   T_FL = (T + a + b + c) / 4
 *   T_FR = (T + a - b - c) / 4
 *   T_BL = (T - a + b - c) / 4
 *   T_BR = (T - a - b + c) / 4
 *
 * 符号约定（与 DroneController 的 pitch / roll 正方向对齐）：
 *   τ_pitch > 0 → 机头上仰（前桨 0/1 加力、后桨 2/3 减力）
 *   τ_roll  > 0 → 机体右倾（左桨 0/2 加力、右桨 1/3 减力）
 *   τ_yaw   > 0 → 绕竖轴正向（对角桨反扭差；方向由飞控 yaw 符号决定）
 *
 * 推力/转速关系： T_i = kT · s_i²   （s_i = 归一化转速 0..maxRotor）
 * 反扭矩关系：   Q_i = kQ · s_i²
 */
import { Logger } from '../core/Logger';

/** 四旋翼几何与动力系数 */
export interface QuadGeometry {
  /** 臂长 L（m）：桨心到机身中心的水平距离 */
  armLength: number;
  /** 推力系数 kT（N）：单桨在归一化转速 1.0 时的推力 */
  kT: number;
  /** 反扭矩系数 kQ（N·m）：单桨在归一化转速 1.0 时的反扭矩 */
  kQ: number;
  /** 归一化转速上限（执行器饱和边界） */
  maxRotor: number;
}

/**
 * 默认几何/动力系数（整机含桨约 0.37m，取 m≈0.12kg 量级的典型值）。
 * 悬停时单桨推力 ≈ mg/4 ≈ 0.29N → 归一化转速 ≈ 0.50（留足上下裕量）。
 */
export const DEFAULT_QUAD: QuadGeometry = {
  armLength: 0.12,
  kT: 1.2,
  kQ: 0.024,
  maxRotor: 1.0
};

/** 限幅 */
function clamp(v: number, lo: number, hi: number): number {
  if (v < lo) {
    return lo;
  }
  if (v > hi) {
    return hi;
  }
  return v;
}

/**
 * 混控：由总推力与三轴力矩解算四个桨的推力（N）。
 * 返回顺序 [FL, FR, BL, BR]。**未做饱和限幅** —— 由调用方决定是否
 * 以及如何限幅（限幅后力矩会失真，这是真实执行器的行为，必须显式发生）。
 */
export function mixThrusts(totalThrust: number, tauPitch: number, tauRoll: number,
  tauYaw: number, g: QuadGeometry): number[] {
  const a: number = tauPitch / g.armLength;
  const b: number = tauRoll / g.armLength;
  const c: number = tauYaw * g.kT / g.kQ;
  const q: number = totalThrust / 4;
  const fl: number = q + (a + b + c) / 4;
  const fr: number = q + (a - b - c) / 4;
  const bl: number = q + (-a + b - c) / 4;
  const br: number = q + (-a - b + c) / 4;
  return [fl, fr, bl, br];
}

/**
 * 反向解算：由四个桨的实际推力求机身受到的合力矩 [τ_pitch, τ_roll, τ_yaw]。
 * 与 mixThrusts 构成严格互逆对（未饱和时），是「物理诚实」的关键：
 * 饱和之后机身**真正**收到的力矩由本函数给出，而不是控制器想要的那个。
 */
export function torquesFromThrusts(thrusts: number[], g: QuadGeometry): number[] {
  const fl: number = thrusts[0];
  const fr: number = thrusts[1];
  const bl: number = thrusts[2];
  const br: number = thrusts[3];
  const tauPitch: number = g.armLength * ((fl + fr) - (bl + br));
  const tauRoll: number = g.armLength * ((fl + bl) - (fr + br));
  const tauYaw: number = (g.kQ / g.kT) * ((fl + br) - (fr + bl));
  return [tauPitch, tauRoll, tauYaw];
}

/** 执行器限幅结果：各桨归一化转速 + 限幅后**实际**能提供的推力 */
export interface ActuatorResult {
  /** 四桨归一化转速 [FL, FR, BL, BR]，已限幅到 [0, maxRotor] */
  speeds: number[];
  /** 四桨实际推力（N）—— 饱和时小于期望值，机身据此受力 */
  thrusts: number[];
}

/**
 * 执行器饱和（真实 RPM 上限 + 空气密度）。
 * ---------------------------------------------------------------
 * 关键物理：**真实执行器的上限是最高转速（RPM），不是推力**；
 * 且升力 T = kT · ρ_rel · s²。
 *   · 空气稀薄（ρ<1）→ 同样转速给出的推力更小 → 必须提转速；
 *   · 转速触顶后，再多要推力也给不出来 —— 这就是为什么大机动时
 *     姿态会「跟不上」：饱和真实地削弱了可用力矩。
 * 因此限幅必须做在**转速**上，再用限幅后的转速回报各桨真正给出的推力。
 * 注意 kQ / kT 都是气动系数、与 ρ 成正比，故其比值与高度无关 ——
 * 混控的偏航通道无需做密度修正（已在数学上自动成立）。
 */
export function applyActuatorLimit(want: number[], g: QuadGeometry,
  airDensity: number): ActuatorResult {
  const rho: number = clamp(airDensity, 0.4, 1.6);
  const speeds: number[] = [0, 0, 0, 0];
  const real: number[] = [0, 0, 0, 0];
  for (let i = 0; i < 4; i++) {
    const w: number = want[i] > 0 ? want[i] : 0;
    const s: number = clamp(Math.sqrt(w / (g.kT * rho)), 0, g.maxRotor);
    speeds[i] = s;
    real[i] = g.kT * rho * s * s;
  }
  const out: ActuatorResult = { speeds: speeds, thrusts: real };
  return out;
}

/** 悬停所需的单桨推力（N）：mg / 4 */
export function hoverThrustPerRotor(mass: number, gravity: number): number {
  return mass * gravity / 4;
}

/**
 * 纯函数自检（与工程内其他自检同风格：失败抛错，由 AR 初始化 try/catch 吸收）。
 * 覆盖：悬停对称、俯仰/滚转差动的**方向**、混控↔反解的可逆性、转速换算端点。
 */
/**
 * 执行器动力学：电机一阶滞后 + 每桨独立差动（P2）。
 * ---------------------------------------------------------------
 * P1 的混控链路在「指令转速 → 实际转速」这一步是**瞬时**的：
 *   applyActuatorLimit 把期望推力直接换算成转速，没有任何时间常数。
 * 真实电机+桨叶是惯性执行器，转速不能突变 —— 给定指令 s_cmd 后，
 * 实际转速 s 以一阶滞后逼近：  ṡ = (s_cmd − s) / τ_motor
 * 离散化（无条件稳定，子步 h 可大于 τ）：  s ← s + (s_cmd − s)·(1 − e^{−h/τ})
 *
 * 「每桨差动」指四桨是**四个相互独立的执行器**，各自以自己的时间常数逼近
 * 各自指令，因此大机动时四桨实际转速天然出现差速 —— 这正是真实四旋翼
 * 「迅猛动作会受制于执行器响应」的来源，也是姿态环要预留相位裕度的原因。
 * 每桨还可带独立的推力增益 kT_i（电机/桨叶个体差异），默认全等以保持 P1
 * 的全部对称性自检；校准后可分别写入以补偿个体偏差。
 *
 * 与 P1 的边界对齐：
 *   · 输入 cmd 即 applyActuatorLimit 给出的限幅后指令转速 [FL,FR,BL,BR]；
 *   · 输出 ActuatorResult 与旧接口完全一致（speeds=实际转速, thrusts=实际推力），
 *     因此只需在 DroneController 子步循环里把「稳态推力」换成「本步滞后推力」；
 *   · 推力仍用 T = kT_i·ρ·s_i²（与 FlightDynamics 同源，饱和语义自洽）。
 */
export class RotorPlant {
  /** 四桨实际（滞后后的）归一化转速 [FL,FR,BL,BR] */
  public actualSpeeds: number[] = [0, 0, 0, 0];
  /** 一阶时间常数（秒）：电机+桨叶 spool 时间，默认 50ms（小型多旋翼典型值） */
  private tauMotor: number;
  /** 每桨推力增益倍数 [FL,FR,BL,BR]，默认全等；校准后可分别写入补偿个体偏差 */
  private gains: number[];

  constructor(tauMotor?: number, gains?: number[]) {
    this.tauMotor = (tauMotor !== undefined && tauMotor > 0) ? tauMotor : 0.05;
    this.gains = gains ? gains.slice(0, 4) : [1, 1, 1, 1];
  }

  /** 复位到停转 */
  public reset(): void {
    this.actualSpeeds = [0, 0, 0, 0];
  }

  /** 设置每桨增益（kT 倍数） */
  public setGains(g: number[]): void {
    if (g !== undefined && g.length >= 4) {
      this.gains = g.slice(0, 4);
    }
  }

  /**
   * 推进一个子步：四桨各自以一阶滞后逼近指令转速，并用**实际**转速回报推力。
   * @param h 子步时长（秒）
   * @param cmd 指令转速（已限幅，0..maxRotor）
   * @param g 混控几何（提供 kT 与 maxRotor）
   * @param airDensity 空气密度相对因子
   */
  public update(h: number, cmd: number[], g: QuadGeometry, airDensity: number): ActuatorResult {
    const rho: number = clamp(airDensity, 0.4, 1.6);
    const k: number = 1 - Math.exp(-h / Math.max(this.tauMotor, 1e-4));
    const speeds: number[] = [0, 0, 0, 0];
    const thrusts: number[] = [0, 0, 0, 0];
    for (let i = 0; i < 4; i++) {
      let s: number = this.actualSpeeds[i] + (cmd[i] - this.actualSpeeds[i]) * k;
      if (s < 0) {
        s = 0;
      } else if (s > g.maxRotor) {
        s = g.maxRotor;
      }
      this.actualSpeeds[i] = s;
      speeds[i] = s;
      thrusts[i] = g.kT * this.gains[i] * rho * s * s;
    }
    return { speeds: speeds, thrusts: thrusts };
  }

  /**
   * 纯函数自检。覆盖：一阶滞后方向、稳态收敛、时间常数量级、
   * 每桨独立（无交叉耦合）、增益个体差、限幅仍生效、过渡期推力 < 稳态推力。
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
    const g: QuadGeometry = DEFAULT_QUAD;

    // 1) 一阶滞后方向：从 0 给指令 1.0，第一步实际转速应 >0 且 <指令（非瞬时到位）
    {
      const p: RotorPlant = new RotorPlant(0.05);
      const out: ActuatorResult = p.update(1 / 240, [1, 1, 1, 1], g, 1.0);
      check('滞后：第一步实际转速>0', out.speeds[0] > 0);
      check('滞后：第一步实际<指令（非瞬时）', out.speeds[0] < 0.99);
      check('滞后：四桨同步推进（对称）', Math.abs(out.speeds[0] - out.speeds[3]) < 1e-12);
    }

    // 2) 稳态收敛：长时间积分后应≈指令；且 3τ(0.15s) 内达到 ≥95% 且不超调
    {
      const p: RotorPlant = new RotorPlant(0.05);
      let out: ActuatorResult = p.update(1 / 240, [0.6, 0.6, 0.6, 0.6], g, 1.0);
      for (let i = 0; i < 999; i++) {
        out = p.update(1 / 240, [0.6, 0.6, 0.6, 0.6], g, 1.0);
      }
      check('收敛：稳态≈指令', Math.abs(out.speeds[0] - 0.6) < 1e-6);
      const p2: RotorPlant = new RotorPlant(0.05);
      let o2: ActuatorResult = p2.update(1 / 240, [0.6, 0.6, 0.6, 0.6], g, 1.0);
      const nSteps: number = Math.round(0.15 / (1 / 240));
      for (let i = 1; i < nSteps; i++) {
        o2 = p2.update(1 / 240, [0.6, 0.6, 0.6, 0.6], g, 1.0);
      }
      check('时间常数：3τ 内达到 ≥95%', o2.speeds[0] >= 0.6 * 0.95);
      check('时间常数：3τ 内未过冲（≤指令）', o2.speeds[0] <= 0.6 + 1e-9);
    }

    // 3) 每桨独立（无交叉耦合）：给四桨不同指令，各桨只跟随自己的指令
    {
      const p: RotorPlant = new RotorPlant(0.05);
      const cmd: number[] = [0.2, 0.5, 0.8, 1.0];
      for (let i = 0; i < 500; i++) {
        p.update(1 / 240, cmd, g, 1.0);
      }
      check('独立：FL 收敛到 0.2', Math.abs(p.actualSpeeds[0] - 0.2) < 1e-6);
      check('独立：FR 收敛到 0.5', Math.abs(p.actualSpeeds[1] - 0.5) < 1e-6);
      check('独立：BL 收敛到 0.8', Math.abs(p.actualSpeeds[2] - 0.8) < 1e-6);
      check('独立：BR 收敛到 1.0', Math.abs(p.actualSpeeds[3] - 1.0) < 1e-6);
      check('独立：四桨互不相同（的确是差动）',
        p.actualSpeeds[0] < p.actualSpeeds[1] && p.actualSpeeds[1] < p.actualSpeeds[2] &&
        p.actualSpeeds[2] < p.actualSpeeds[3]);
    }

    // 4) 每桨增益个体差：同指令下，增益大的桨给出更大推力（稳态比 = 增益比）
    {
      const p: RotorPlant = new RotorPlant(0.05, [1.0, 1.3, 1.0, 1.0]);
      let out: ActuatorResult = p.update(1 / 240, [0.6, 0.6, 0.6, 0.6], g, 1.0);
      check('个体差：增益大的桨推力更大（第一步）', out.thrusts[1] > out.thrusts[0] + 1e-9);
      for (let i = 0; i < 500; i++) {
        out = p.update(1 / 240, [0.6, 0.6, 0.6, 0.6], g, 1.0);
      }
      const ratio: number = out.thrusts[1] / out.thrusts[0];
      check('个体差：稳态推力比≈增益比(1.3)', Math.abs(ratio - 1.3) < 0.02);
    }

    // 5) 限幅仍生效：超限指令被夹到 maxRotor
    {
      const p: RotorPlant = new RotorPlant(0.05);
      for (let i = 0; i < 500; i++) {
        p.update(1 / 240, [5, 5, 5, 5], g, 1.0);
      }
      check('限幅：实际转速夹到 maxRotor', Math.abs(p.actualSpeeds[0] - g.maxRotor) < 1e-9);
    }

    // 6) 过渡期推力 < 稳态推力：一阶滞后意味着「要加速得等转速上来」
    {
      const p: RotorPlant = new RotorPlant(0.05);
      let transient: number = 0;
      for (let i = 0; i < 5; i++) {
        const o: ActuatorResult = p.update(1 / 240, [0.8, 0.8, 0.8, 0.8], g, 1.0);
        transient = o.thrusts[0];
      }
      let steady: number = 0;
      for (let i = 0; i < 1000; i++) {
        const o: ActuatorResult = p.update(1 / 240, [0.8, 0.8, 0.8, 0.8], g, 1.0);
        steady = o.thrusts[0];
      }
      check('过渡：前 5 步推力 < 稳态推力', transient < steady - 1e-6);
    }

    if (fails.length !== 0) {
      throw new Error(`RotorPlant 执行器动力学自检失败：${fails.join('、')}`);
    }
    Logger.info(`[VERIFY] RotorPlant 电机一阶滞后+每桨差动自检 PASS（${total} 项断言全通过）`);
  }
}

export function selfTest(): void {
  const fails: string[] = [];
  let total: number = 0;
  const check = (name: string, ok: boolean): void => {
    total++;
    if (!ok) {
      fails.push(name);
    }
  };
  const g: QuadGeometry = DEFAULT_QUAD;

  // 1) 纯悬停（无力矩）：四桨推力必须完全相等
  {
    const t: number[] = mixThrusts(1.0, 0, 0, 0, g);
    const eq: boolean = Math.abs(t[0] - t[1]) < 1e-9 && Math.abs(t[1] - t[2]) < 1e-9 &&
      Math.abs(t[2] - t[3]) < 1e-9 && Math.abs(t[0] - 0.25) < 1e-9;
    check('悬停 → 四桨等推力', eq);
  }

  // 2) 纯俯仰力矩：前桨(FL,FR)同增、后桨(BL,BR)同减 → 机头上仰
  {
    const t: number[] = mixThrusts(1.0, 0.08, 0, 0, g);
    check('俯仰 → FL=FR', Math.abs(t[0] - t[1]) < 1e-9);
    check('俯仰 → BL=BR', Math.abs(t[2] - t[3]) < 1e-9);
    check('俯仰>0 → 前桨>后桨(抬头)', t[0] > t[2] + 1e-6);
  }

  // 3) 纯滚转力矩：左桨(FL,BL)同增、右桨(FR,BR)同减 → 右倾
  {
    const t: number[] = mixThrusts(1.0, 0, 0.08, 0, g);
    check('滚转 → FL=BL', Math.abs(t[0] - t[2]) < 1e-9);
    check('滚转 → FR=BR', Math.abs(t[1] - t[3]) < 1e-9);
    check('滚转>0 → 左桨>右桨(右倾)', t[0] > t[1] + 1e-6);
  }

  // 4) 混控 ↔ 反解 严格可逆（未饱和）
  {
    const pts: number[] = mixThrusts(1.2, 0.05, -0.03, 0.02, g);
    const tau: number[] = torquesFromThrusts(pts, g);
    check('可逆：τ_pitch', Math.abs(tau[0] - 0.05) < 1e-9);
    check('可逆：τ_roll', Math.abs(tau[1] - (-0.03)) < 1e-9);
    check('可逆：τ_yaw', Math.abs(tau[2] - 0.02) < 1e-9);
  }

  // 5) 转速-推力换算端点（ρ=1）：0→0；kT→1；超限夹到 maxRotor；0.25kT→0.5
  {
    const a: ActuatorResult = applyActuatorLimit([0, g.kT, g.kT * 4, g.kT * 0.25], g, 1.0);
    const s: number[] = a.speeds;
    check('转速端点：0', Math.abs(s[0]) < 1e-9);
    check('转速端点：1', Math.abs(s[1] - 1) < 1e-9);
    check('转速饱和：>maxRotor 被夹到 1', Math.abs(s[2] - 1) < 1e-9);
    check('转速：0.25kT → 0.5', Math.abs(s[3] - 0.5) < 1e-9);
  }

  // 6) 悬停推力换算：m=0.12, g=9.81 → 单桨 0.2943N
  {
    const hv: number = hoverThrustPerRotor(0.12, 9.81);
    check('悬停单桨推力 ≈ mg/4', Math.abs(hv - 0.2943) < 1e-3);
  }

  // 7) 执行器饱和：转速上限生效，且回报的推力与转速严格自洽
  {
    const want: number[] = [10, 0.3, -1, 0.6];   // 第一个远超上限、第三个为负
    const a: ActuatorResult = applyActuatorLimit(want, g, 1.0);
    check('饱和：超限桨被夹到 maxRotor', Math.abs(a.speeds[0] - g.maxRotor) < 1e-9);
    check('饱和：超限桨实际推力 = kT·maxRotor²',
      Math.abs(a.thrusts[0] - g.kT * g.maxRotor * g.maxRotor) < 1e-9);
    check('饱和：负推力被夹到 0', Math.abs(a.speeds[2]) < 1e-12 &&
      Math.abs(a.thrusts[2]) < 1e-12);
    check('饱和：未限幅桨的推力原样还原', Math.abs(a.thrusts[1] - 0.3) < 1e-9);
    check('饱和：转速-推力自洽 T=kT·ρ·s²',
      Math.abs(a.thrusts[3] - g.kT * 1.0 * a.speeds[3] * a.speeds[3]) < 1e-12);
  }

  // 8) 空气稀薄 → 同一推力需要更高转速（真实物理，别退化成「密度无影响」）
  {
    const want: number[] = [0.3, 0.3, 0.3, 0.3];
    const sea: ActuatorResult = applyActuatorLimit(want, g, 1.0);
    const high: ActuatorResult = applyActuatorLimit(want, g, 0.85);
    check('稀薄空气 → 悬停转速更高', high.speeds[0] > sea.speeds[0] + 0.01);
    check('稀薄空气 → 同样转速给出的推力更小',
      g.kT * 0.85 * sea.speeds[0] * sea.speeds[0] < 0.3);
  }

  if (fails.length !== 0) {
    throw new Error(`RotorMixer 混控自检失败：${fails.join('、')}`);
  }
  Logger.info(`[VERIFY] RotorMixer 混控自检 PASS（${total} 项断言全通过）`);
}
