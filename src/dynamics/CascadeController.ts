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
 * 姿态 → 角速度指令（级联控制的「中环」）。
 * ---------------------------------------------------------------
 * 真实飞控是三级结构：外环给期望姿态角 → 中环把姿态误差变成期望角速度 →
 * 内环把角速度误差变成力矩。本函数即中环：把角度误差映射成**受最大角速度约束**
 * 的期望角速度。
 *   rateDes = clamp(kp·(angleDes − angle), −maxRate, maxRate)
 * 这样增益 kp 只决定「多快趋向目标角度」，最大角速度由飞控/机械上限约束，
 * 与单机 PX4 的 MC_ROLL_P 同构（PX4 MC_ROLL_P=4.0；本工程取 7.5 以在
 * 与既有单 PD（Kp=225）等刚度下保持临界阻尼手感）。
 */
export function attitudeRateCmd(angleDes: number, angle: number,
  kp: number, maxRate: number): number {
  return clamp(kp * (angleDes - angle), -maxRate, maxRate);
}

/**
 * 角速度内环（PD）→ 期望力矩（级联控制的「内环」）。
 *   τ = I · (kp·(rateDes − rate) − kd·rate)
 * 与 attitudeTorque（角度 PD）对应，但控制变量是**角速度**而非角度 ——
 * 这正是真实飞控「速率环」的物理层；与 attitudeRateCmd 串联即组成完整级联。
 * 当 kp·maxRate 与单 PD 等刚度时两条链路数学等价（仅结构不同），
 * 故开启速率环默认不改变飞行手感（零回归），却为湍流注入、执行器非线性、
 * 状态估计（EKF）等后续高保真层提供了「角速度」这一可注入/可观测的状态。
 */
export function rateTorque(rateDes: number, rate: number,
  kp: number, kd: number, inertia: number): number {
  return inertia * (kp * (rateDes - rate) - kd * rate);
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
 * 机体气动阻力（增强模型，M4 阶段B · L2 机体气动矩阵 + 入流）。
 * ---------------------------------------------------------------
 * 与 isotropic 的 dragForce 不同：真实机体的阻力在三个机体轴方向**并不各向同性** ——
 * 沿机头前后（forward）近似流线型、阻力小；沿机体左右（side）与上下（vertical，即推力轴）
 * 接近平板、阻力大。本函数用一组机体轴阻力面积 cdaF/cdaS/cdaU 实现这一「气动矩阵」。
 *
 * 入流（inflow / downwash）：旋翼把空气向下加速（动量理论诱导速度
 *   v_i = √(T / (2·ρ·A_disk))）。它改变机体实际感受到的相对气流 —— 尤其在前飞/升降时。
 * 为避免在悬停时把「诱导速度 = 推力来源」重复计成阻力，入流项只在机体**确有运动**
 * （activity = clamp(|v_rel|/v_ref, 0, 1)）时才叠加，悬停（v_rel=0）时归零。
 *
 * @param rvx/rvy/rvz 世界系相对气流速度（机体速度 − 风速）
 * @param upX/upY/upZ 机体上轴（世界系，含姿态）；yaw 机体偏航（用于前向/侧向分解）
 * @param cdaF/cdaS/cdaU 机体轴阻力面积（forward/side/up，默认 = 0.5·Cd·A，ρ=1 时）
 * @param totalThrust 当前总推力（N），用于动量理论诱导速度
 * @param diskArea 四桨总桨盘面积（m²）
 * @returns 世界系三分量 [Fx, Fy, Fz]，符号与 dragForce 一致（阻碍相对运动）
 */
export function bodyDragForce(rvx: number, rvy: number, rvz: number,
  upX: number, upY: number, upZ: number, yaw: number,
  airDensity: number, cdaF: number, cdaS: number, cdaU: number,
  totalThrust: number, diskArea: number): number[] {
  const rho: number = clamp(airDensity, 0.4, 1.6);
  const rs: number = Math.sqrt(rvx * rvx + rvy * rvy + rvz * rvz);
  const activity: number = clamp(rs / 3.0, 0, 1);   // v_ref=3 m/s；悬停=0 → 入流不重复计阻力
  const vI: number = activity * Math.sqrt(Math.max(totalThrust, 0) / (2 * rho * Math.max(diskArea, 1e-6)));

  // 机体基（世界系）：up 已知；forward 取 yaw 对齐的水平前向（机头 = −Z），
  // 投影到与 up 垂直平面并归一化；right = forward × up（右手系，+X = 机体右向）。
  const fxw: number = -Math.sin(yaw);
  const fzw: number = -Math.cos(yaw);
  const f0dotUp: number = fxw * upX + fzw * upZ;
  let fdx: number = fxw - f0dotUp * upX;
  let fdy: number = -f0dotUp * upY;
  let fdz: number = fzw - f0dotUp * upZ;
  const fmag: number = Math.sqrt(fdx * fdx + fdy * fdy + fdz * fdz);
  if (fmag > 1e-9) { fdx /= fmag; fdy /= fmag; fdz /= fmag; }
  const rdx: number = fdy * upZ - fdz * upY;
  const rdy: number = fdz * upX - fdx * upZ;
  const rdz: number = fdx * upY - fdy * upX;

  // 三轴相对风速分量（up 轴叠加诱导速度：下洗使机体上方相对气流增大）
  const vUp: number = rvx * upX + rvy * upY + rvz * upZ + vI;
  const vFwd: number = rvx * fdx + rvy * fdy + rvz * fdz;
  const vLat: number = rvx * rdx + rvy * rdy + rvz * rdz;

  // 各轴二次阻力 F = −ρ·cda·|v|·v（decoupled）
  const fUp: number = -rho * cdaU * Math.abs(vUp) * vUp;
  const fFwd: number = -rho * cdaF * Math.abs(vFwd) * vFwd;
  const fLat: number = -rho * cdaS * Math.abs(vLat) * vLat;

  return [
    fUp * upX + fFwd * fdx + fLat * rdx,
    fUp * upY + fFwd * fdy + fLat * rdy,
    fUp * upZ + fFwd * fdz + fLat * rdz
  ];
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

  // 9) 速率环级联（M4 阶段A）：中环姿态→角速度指令 + 内环角速度→力矩
  {
    const D2R: number = Math.PI / 180;
    // 9a) 中环：角度误差→受约束的角速度指令；正误差→正速率指令；超 maxRate 被夹
    {
      const r: number = attitudeRateCmd(0.2, 0.0, 7.5, 6.0);
      check('速率环·中环：正角度误差→正速率指令', r > 0);
      const r2: number = attitudeRateCmd(10, 0, 7.5, 6.0);   // 误差巨大 → 必被限幅
      check('速率环·中环：速率指令被 maxRate 限幅', Math.abs(r2 - 6.0) < 1e-9);
      const r3: number = attitudeRateCmd(0, 0, 7.5, 6.0);
      check('速率环·中环：零误差→零速率指令', Math.abs(r3) < 1e-12);
    }
    // 9b) 内环：正速率误差→正力矩；角速度阻尼削弱力矩
    {
      const tp: number = rateTorque(0.2, 0, 30, 0, 1.4e-3);
      check('速率环·内环：正速率误差→正力矩', tp > 0);
      const tp2: number = rateTorque(0.2, 0.1, 30, 0, 1.4e-3);
      check('速率环·内环：当前角速度削减所需力矩', tp2 < tp);
    }
    // 9c) 级联与单 PD 等刚度：1-DOF 俯仰阶跃，级联(7.5/30) 应等价于单PD(225/30)
    {
      const inertia: number = 1.4e-3;
      const angKp: number = 225, angKd: number = 30;
      const midKp: number = 7.5, rateKp: number = 30, maxRate: number = 6.0;
      const h: number = 1 / 240;
      let pC: number = 0, wC: number = 0;   // 级联
      let pP: number = 0, wP: number = 0;   // 单 PD
      const des: number = 10 * D2R;
      for (let i = 0; i < 480; i++) {
        const rc: number = attitudeRateCmd(des, pC, midKp, maxRate);
        const tc: number = rateTorque(rc, wC, rateKp, 0, inertia);
        wC += (tc / inertia) * h;
        pC += wC * h;
        const tp2: number = inertia * (angKp * (des - pP) - angKd * wP);
        wP += (tp2 / inertia) * h;
        pP += wP * h;
      }
      check('级联≡单PD：稳态角度一致', Math.abs(pC - pP) < 1e-3);
      check('级联≡单PD：稳态角速度一致', Math.abs(wC - wP) < 1e-3);
      check('级联：阶跃无发散（角度有界）', Math.abs(pC) < 0.5);
    }
  }

  // 10) 机体气动矩阵 + 入流（M4 阶段B · L2 增强气动）
  // 默认 cda 与 DEFAULT_DYNAMICS 对齐（forward=0.004 / side=0.012 / up=0.010 / rotorRadius=0.045）。
  {
    const cdaF: number = 0.004, cdaS: number = 0.012, cdaU: number = 0.010;
    const r: number = 0.045;
    const disk: number = 4 * Math.PI * r * r;
    const yaw: number = 0;
    const up: number[] = [0, 1, 0];   // 水平姿态：机体上轴竖直
    const T: number = 0.12 * 9.81;    // 悬停集总推力

    // 10a) 方向：+X 相对风（沿机体侧向）→ 阻力沿 −X（阻碍运动）
    {
      const f: number[] = bodyDragForce(5, 0, 0, up[0], up[1], up[2], yaw, 1.0, cdaF, cdaS, cdaU, T, disk);
      check('增强气动·方向：+X 风 → −X 阻力', f[0] < 0);
      // 侧向 cdaS(0.012) > 前向 cdaF(0.004)：侧向阻力幅值 3× 前向（证明非各向同性）
      const fFwd: number[] = bodyDragForce(0, 0, 5, up[0], up[1], up[2], yaw, 1.0, cdaF, cdaS, cdaU, T, disk);
      check('增强气动·气动矩阵：侧向阻力幅值 > 前向（cdaS>cdaF）', Math.abs(f[0]) > Math.abs(fFwd[2]) + 1e-9);
    }
    // 10b) 风向反转 → 阻力反向
    {
      const f1: number[] = bodyDragForce(5, 0, 0, up[0], up[1], up[2], yaw, 1.0, cdaF, cdaS, cdaU, T, disk);
      const f2: number[] = bodyDragForce(-5, 0, 0, up[0], up[1], up[2], yaw, 1.0, cdaF, cdaS, cdaU, T, disk);
      check('增强气动·方向反转：阻力随之反向', f2[0] > 0 && f1[0] < 0);
    }
    // 10c) 悬停（无相对风）→ 零阻力（activity=0，入流不重复计）
    {
      const f: number[] = bodyDragForce(0, 0, 0, up[0], up[1], up[2], yaw, 1.0, cdaF, cdaS, cdaU, T, disk);
      check('增强气动·悬停零相对风 → 零阻力', Math.abs(f[0]) < 1e-12 && Math.abs(f[1]) < 1e-12 && Math.abs(f[2]) < 1e-12);
    }
    // 10d) 入流注入：有推力+水平风 → 机体上轴出现额外阻力；无推力（T=0）→ 入流归零、上轴无阻
    {
      const fT: number[] = bodyDragForce(5, 0, 0, up[0], up[1], up[2], yaw, 1.0, cdaF, cdaS, cdaU, T, disk);
      const f0: number[] = bodyDragForce(5, 0, 0, up[0], up[1], up[2], yaw, 1.0, cdaF, cdaS, cdaU, 0, disk);
      check('增强气动·入流：有推力时上轴出现诱导阻力(Fy<0)', fT[1] < -1e-9);
      check('增强气动·入流：无推力时上轴零诱导阻力', Math.abs(f0[1]) < 1e-12);
    }
    // 10e) 增强相对 basic 各向同性：同侧向风，增强(侧)阻力 ≠ basic(isotropic)阻力
    {
      const enh: number[] = bodyDragForce(5, 0, 0, up[0], up[1], up[2], yaw, 1.0, cdaF, cdaS, cdaU, T, disk);
      const basic: number[] = dragForce(5, 0, 0, cdaF, 1.0);
      check('增强气动·区别于 basic：侧向阻力幅值 ≠ isotropic', Math.abs(Math.abs(enh[0]) - Math.abs(basic[0])) > 1e-9);
    }
  }

  if (fails.length !== 0) {
    throw new Error(`CascadeController 自检失败：${fails.join('、')}`);
  }
  Logger.info(`[VERIFY] CascadeController 级联控制自检 PASS（${total} 项断言全通过）`);
}
