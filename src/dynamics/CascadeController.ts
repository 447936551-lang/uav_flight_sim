/**
 * 级联飞行控制器（Layer 5.7 · Cascade Controller）
 * ===============================================================
 * 现实飞控是**双环级联**结构，本模块就是它的「外环 + 姿态解算」部分：
 *
 *   用户摇杆 ─▶ 位置环（很慢）─▶ 期望速度
 *                                  │
 *                       速度环（慢）─▶ 期望加速度 a_des
 *                                  │
 *          期望推力矢量 F = m·(a_des + g) − F_drag   ← 重力前馈 + 阻力前馈
 *                                  │
 *              由 F 的方向反解 期望倾角 (θ_des, φ_des)
 *                                  │
 *                       姿态环（快）─▶ 期望力矩 τ       ← 混控器接手
 *
 * 关键点（也是好多人做「假动力学」时漏掉的）：
 *   1) **期望倾角不是拍出来的，是解出来的** —— 要让飞机产生水平加速度，
 *      推力矢量必须倾斜；倾角大小由所需水平力 / 竖直力决定，风大即倾角大。
 *   2) **重力与阻力前馈**：F 里先扣掉重力与当前阻力，剩下的才是「机动所需」，
 *      否则飞机连悬停都稳不住。
 *   3) **姿态环必须显著快于速度环**（约 3~5 倍带宽），否则两环耦合成振荡。
 *
 * 本模块全部是纯函数（无状态），便于像其他物理层一样做离线断言自检。
 */
import { Logger } from '../core/Logger';

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
 * 位置环：由「位置误差」给出期望速度（真实飞控的 GPS 定点保持）。
 * 无操控输入时调用，因此这正是「松手即定点、风吹也不跑」的来源。
 */
export function positionHoldVelocity(holdX: number, holdZ: number,
  x: number, z: number, kp: number, maxSpeed: number): number[] {
  const vx: number = clamp(kp * (holdX - x), -maxSpeed, maxSpeed);
  const vz: number = clamp(kp * (holdZ - z), -maxSpeed, maxSpeed);
  return [vx, vz];
}

/**
 * 速度环：由「期望速度 − 当前速度」给出期望加速度，并按矢量限幅。
 * 必须按**矢量**限幅而不是逐分量，否则对角线方向的合成加速度会到 √2 倍。
 */
export function velocityLoop(vDesX: number, vDesZ: number,
  vx: number, vz: number, kp: number, maxAccel: number): number[] {
  let ax: number = kp * (vDesX - vx);
  let az: number = kp * (vDesZ - vz);
  const mag: number = Math.sqrt(ax * ax + az * az);
  if (mag > maxAccel && mag > 1e-9) {
    ax = ax / mag * maxAccel;
    az = az / mag * maxAccel;
  }
  return [ax, az];
}

/** 垂直速度环：同上，标量版本 */
export function verticalLoop(vyDes: number, vy: number,
  kp: number, maxAccel: number): number {
  return clamp(kp * (vyDes - vy), -maxAccel, maxAccel);
}

/**
 * 由期望加速度 + 重力 + 当前阻力，解算**期望推力矢量**。
 *   F = m·(a_des + g_vec) − F_drag        （g_vec = (0, g, 0) 因为重力是 −g·ŷ）
 * 返回 [|F|, ûx, ûy, ûz]；|F| 即总推力指令 T，û 即期望的机体上轴方向。
 */
export function thrustVector(ax: number, ay: number, az: number,
  mass: number, gravity: number,
  fdx: number, fdy: number, fdz: number): number[] {
  const rx: number = mass * ax - fdx;
  const ry: number = mass * (ay + gravity) - fdy;
  const rz: number = mass * az - fdz;
  const T: number = Math.sqrt(rx * rx + ry * ry + rz * rz);
  if (T < 1e-6) {
    // 极端退化（几乎无推力需求）：维持「竖直向上」以免方向解算除零
    return [0, 0, 1, 0];
  }
  return [T, rx / T, ry / T, rz / T];
}

/**
 * 由期望机体上轴方向反解期望倾角（pitch/roll），并硬限幅到 maxTiltRad。
 * 在 yaw 对齐的机体系上投影：
 *   u·r = sin(φ)   →  φ_des = asin(u·r)        （r = 机体系右向水平单位向量）
 *   u·f = −sin(θ)  →  θ_des = asin(−(u·f))     （f = 机体系前向水平单位向量）
 * 返回 [θ_des, φ_des]（rad），符号与 DroneController 的 pitch/roll 一致。
 */
export function tiltFromUpDir(ux: number, uz: number,
  fx: number, fz: number, rx: number, rz: number, maxTiltRad: number): number[] {
  const uDotR: number = ux * rx + uz * rz;
  const uDotF: number = ux * fx + uz * fz;
  const phi: number = Math.asin(clamp(uDotR, -1, 1));
  const theta: number = Math.asin(clamp(-uDotF, -1, 1));
  return [clamp(theta, -maxTiltRad, maxTiltRad), clamp(phi, -maxTiltRad, maxTiltRad)];
}

/**
 * 姿态环（PD）→ 期望力矩。
 *   α_des = kp·(angle_des − angle) − kd·ω      （角加速度指令）
 *   τ     = I · α_des
 * 用「角加速度指令 × 转动惯量」的形式，使增益与机型惯量**解耦**：
 * 换一套惯量参数时手感不变，只影响所需的力矩量级。
 */
export function attitudeTorque(angleDes: number, angle: number,
  rate: number, kp: number, kd: number, inertia: number): number {
  return inertia * (kp * (angleDes - angle) - kd * rate);
}

/** 偏航环（一阶转速控制）→ 期望力矩 */
export function yawTorque(rateDes: number, rate: number,
  kp: number, inertia: number): number {
  return inertia * kp * (rateDes - rate);
}

/**
 * 空气密度修正的阻力前馈：给定相对气流速度，返回阻力三分量。
 * 与 FlightDynamics.step 内的阻力公式**同源**，保证前馈真的抵消得掉。
 */
export function dragForce(rvx: number, rvy: number, rvz: number,
  dragCoef: number, airDensity: number): number[] {
  const rs: number = Math.sqrt(rvx * rvx + rvy * rvy + rvz * rvz);
  const dk: number = dragCoef * clamp(airDensity, 0.4, 1.6);
  return [-dk * rs * rvx, -dk * rs * rvy, -dk * rs * rvz];
}

/**
 * 纯函数自检。覆盖：位置环限速、速度环矢量限幅、悬停推力解算、
 * 倾角反解的**方向与量级**（含限幅）、姿态 PD 的符号。
 */
export function selfTest(): void {
  const fails: string[] = [];
  let total: number = 0;
  const check = (name: string, ok: boolean): void => {
    total++;
    if (!ok) {
      fails.push(name);
    }
  };
  const D2R: number = Math.PI / 180;

  // 1) 位置环：误差方向正确且被限速
  {
    const v: number[] = positionHoldVelocity(0, 0, 1, 0, 1.2, 1.5);
    check('位置环：目标在 −X 侧 → 期望速度朝 −X', v[0] < 0);
    const v2: number[] = positionHoldVelocity(100, 0, 0, 0, 1.2, 1.5);
    check('位置环：限速到 maxSpeed', Math.abs(v2[0] - 1.5) < 1e-9);
  }

  // 2) 速度环：零误差 → 零指令；大误差按矢量限幅（对角线不超速）
  {
    const z: number[] = velocityLoop(0.5, 0.5, 0.5, 0.5, 2.6, 4.5);
    check('速度环：零误差 → 零加速度', Math.abs(z[0]) < 1e-12 && Math.abs(z[1]) < 1e-12);
    const d: number[] = velocityLoop(10, 10, 0, 0, 2.6, 4.5);
    const mag: number = Math.sqrt(d[0] * d[0] + d[1] * d[1]);
    check('速度环：对角线按矢量限幅', Math.abs(mag - 4.5) < 1e-9);
  }

  // 3) 推力矢量：水平悬停（a=0, 无阻力）→ T=mg、方向竖直向上
  {
    const tv: number[] = thrustVector(0, 0, 0, 0.12, 9.81, 0, 0, 0);
    check('悬停 → T=mg', Math.abs(tv[0] - 0.12 * 9.81) < 1e-9);
    check('悬停 → 上轴竖直', Math.abs(tv[1]) < 1e-12 && Math.abs(tv[3]) < 1e-12);
    check('悬停 → uy=1', Math.abs(tv[2] - 1) < 1e-12);
  }

  // 4) 想向右加速 → 推力矢量偏向 +X（即需右倾），倾角为正
  {
    const tv: number[] = thrustVector(2.0, 0, 0, 0.12, 9.81, 0, 0, 0);
    const fx: number = 0;
    const fz: number = -1;
    const rx: number = 1;
    const rz: number = 0;
    const t: number[] = tiltFromUpDir(tv[1], tv[3], fx, fz, rx, rz, 20 * D2R);
    check('向右加速 → 右倾（roll>0）', t[1] > 0);
    check('向右加速 → 不产生俯仰', Math.abs(t[0]) < 1e-9);
  }

  // 5) 想向前加速（yaw=0，前向 = −Z）→ 低头（pitch<0）
  {
    const tv: number[] = thrustVector(0, 0, -2.0, 0.12, 9.81, 0, 0, 0);
    const t: number[] = tiltFromUpDir(tv[1], tv[3], 0, -1, 1, 0, 20 * D2R);
    check('向前加速 → 低头（pitch<0）', t[0] < 0);
  }

  // 6) 倾角硬限幅
  {
    const tv: number[] = thrustVector(20, 0, 0, 0.12, 9.81, 0, 0, 0);
    const t: number[] = tiltFromUpDir(tv[1], tv[3], 0, -1, 1, 0, 20 * D2R);
    check('倾角限幅到 maxTilt', t[1] <= 20 * D2R + 1e-12);
  }

  // 7) 姿态 PD：误差为正 → 力矩为正；角速度阻尼使其减小
  {
    const tp: number = attitudeTorque(0.2, 0, 0, 225, 30, 1.4e-3);
    check('姿态 PD：正误差 → 正力矩', tp > 0);
    const tp2: number = attitudeTorque(0.2, 0, 5, 225, 30, 1.4e-3);
    check('姿态 PD：角速度阻尼削弱力矩', tp2 < tp);
  }

  // 8) 阻力前馈：与相对气流方向相反
  {
    const f: number[] = dragForce(3, 0, 0, 0.004, 1.0);
    check('阻力前馈：与相对气流反向', f[0] < 0);
    check('阻力前馈：大小 = c·|v|²', Math.abs(f[0] + 0.004 * 9) < 1e-9);
  }

  if (fails.length !== 0) {
    throw new Error(`CascadeController 自检失败：${fails.join('、')}`);
  }
  Logger.info(`[VERIFY] CascadeController 级联控制自检 PASS（${total} 项断言全通过）`);
}
