/**
 * 大气换算（风矢量 / 空气密度 / 低通平滑 / 阵风）
 * ---------------------------------------------------------------
 * 本文件是 App 侧 `EnvironmentModel` 的**纯换算部分**。
 * 原文件同时做了三件事：① 纯换算 ② 每 5 分钟网络取数 ③ 写回 DroneController。
 * 其中 ②③ 与本仓库「零依赖、可离线、确定性」的定位冲突，故剥离在 App 侧；
 * 这里只保留 ①，且全部是纯函数 / 无副作用状态推进。
 *
 * 对外提供两条链路（与原实现一致，互不影响）：
 *   · 等效加速度链路 —— 把风当成施加在机身上的加速度扰动（m/s²），
 *     供**运动学**内核消费；
 *   · 风速矢量链路   —— 真实的 m/s 风矢量，供**动力学**内核算相对气流阻力。
 *
 * 坐标约定（必须与 DroneController / SpatialPerception 一致）：
 *   北 = -Z（前方 / 远离用户）　南 = +Z　东 = +X（右）　西 = -X
 *   气象风向 = 风**吹来**的方向；风实际吹向「来向 + 180°」。
 *   方位角 θ（北为 0，顺时针）的单位向量：x = sinθ，z = -cosθ。
 */
import { Logger } from '../core/Logger';

/** 风→等效加速度增益（m/s² 每 m/s 风速）。5m/s 风 → 约 0.5 m/s² 漂移 */
export const WIND_GAIN: number = 0.10;
/** 倾角参考加速度（m/s²）：达到此风扰即打满机身倾角。10m/s 风→满倾 */
export const WIND_TILT_REF: number = 1.0;
/** 风场低通平滑速率（1/s）：越大收敛越快，时间常数约 1/该值 秒 */
export const WIND_SMOOTH: number = 0.5;
/** 阵风扰动幅度：占当前风加速的比例 */
export const GUST_AMP: number = 0.35;
/** 阵风主频（rad/s） */
export const GUST_FREQ: number = 1.7;

const TAG: string = 'Atmosphere';

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
 * 气象风向 → 风矢量单位方向。
 * @param windDirectionDeg 气象风向（度）：风吹来的方向，0 = 北，顺时针
 * @returns [x, z] 单位向量，指向风**吹去**的方向
 */
export function windDirUnit(windDirectionDeg: number): number[] {
  const toBearing: number = (windDirectionDeg + 180) % 360;
  const r: number = toBearing * Math.PI / 180;
  return [Math.sin(r), -Math.cos(r)];
}

/**
 * 气象风向 + 风速 → 风速矢量（m/s，世界系 X/Z）。
 */
export function windVelocity(windDirectionDeg: number, windSpeed: number): number[] {
  const u: number[] = windDirUnit(windDirectionDeg);
  return [u[0] * windSpeed, u[1] * windSpeed];
}

/**
 * 风速 → 等效加速度扰动（m/s²，世界系 X/Z）。运动学内核消费。
 */
export function windAccel(windDirectionDeg: number, windSpeed: number): number[] {
  const u: number[] = windDirUnit(windDirectionDeg);
  const a: number = WIND_GAIN * windSpeed;
  return [u[0] * a, u[1] * a];
}

/**
 * 由海平面气压与气温推算空气密度**相对因子**（标准海平面 = 1.0）。
 *   rho ∝ P / T(K)
 * 高温低压 → <1（空气稀薄，同转速升力更小）；低温高压 → >1。
 */
export function airDensityFactor(pressureHpa: number, temperatureC: number): number {
  const tK: number = temperatureC + 273.15;
  const rho: number = (pressureHpa / 1013.25) * (288.15 / tK);
  return clamp(rho, 0.6, 1.4);
}

/** 气象方位（0..360）转中文方向，如 0→「北风」 */
export function bearingText(bearing: number): string {
  const dirs: string[] = ['北', '东北', '东', '东南', '南', '西南', '西', '西北'];
  const idx: number = Math.round(((bearing % 360) / 45)) % 8;
  return dirs[idx] + '风';
}

/**
 * 风场平滑器：把「目标风场」低通过渡到「当前风场」，并叠加阵风扰动。
 * ---------------------------------------------------------------
 * 为什么不直接用目标值：真实天气是 5 分钟刷一次、且阵风是低频起伏的，
 * 直接跳变会让飞机「一顿一顿」。低通 + 正弦阵风让它像真实大气那样渐变。
 *
 * 无状态依赖：gustPhase 由调用方推进的 dt 累加，故**确定性**（同输入必同输出），
 * 可以直接用于单测与仿真黄金用例。
 */
export class WindSmoother {
  /** 当前平滑后的风速矢量（m/s） */
  public velX: number = 0;
  public velZ: number = 0;
  /** 当前平滑后的空气密度相对因子 */
  public density: number = 1.0;
  /** 阵风相位累加器（每帧递增，驱动正弦扰动） */
  public gustPhase: number = 0;

  /** 目标风场（由外部按天气快照写入） */
  public targetVelX: number = 0;
  public targetVelZ: number = 0;
  public targetDensity: number = 1.0;

  /**
   * 推进一帧。
   * @param dt 帧间隔（秒）
   * @param enabled 环境感知是否开启；关闭时输出**恒为 0 / 1.0**（零回归保证）
   * @returns 本帧实际生效的 [windVelX, windVelZ, factor]，factor 为阵风倍率
   */
  public tick(dt: number, enabled: boolean): number[] {
    const k: number = clamp(WIND_SMOOTH * dt, 0, 1);
    this.velX += (this.targetVelX - this.velX) * k;
    this.velZ += (this.targetVelZ - this.velZ) * k;
    this.density += (this.targetDensity - this.density) * k;

    this.gustPhase += dt * GUST_FREQ;
    const gust: number = Math.sin(this.gustPhase) * GUST_AMP;

    if (!enabled) {
      return [0, 0, 1.0];
    }
    return [this.velX * (1 + gust), this.velZ * (1 + gust), this.density];
  }

  /** 复位到无风标准大气 */
  public reset(): void {
    this.velX = 0;
    this.velZ = 0;
    this.density = 1.0;
    this.gustPhase = 0;
    this.targetVelX = 0;
    this.targetVelZ = 0;
    this.targetDensity = 1.0;
  }
}

/**
 * 纯函数自检。覆盖风向约定（四条方位）、密度单调性、平滑器关闭时的零回归。
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

  // —— 风向约定（必须与 DroneController 一致）——
  // 北风（从北来）→ 吹向南 = +Z
  const n: number[] = windDirUnit(0);
  check('北风→吹向南(z>0)', n[1] > 0.99 && Math.abs(n[0]) < 1e-6);
  // 南风（从南来）→ 吹向北 = -Z
  const s: number[] = windDirUnit(180);
  check('南风→吹向北(z<0)', s[1] < -0.99 && Math.abs(s[0]) < 1e-6);
  // 东风（从东来）→ 吹向西 = -X
  const e: number[] = windDirUnit(90);
  check('东风→吹向西(x<0)', e[0] < -0.99 && Math.abs(e[1]) < 1e-6);
  // 西风（从西来）→ 吹向东 = +X
  const w: number[] = windDirUnit(270);
  check('西风→吹向东(x>0)', w[0] > 0.99 && Math.abs(w[1]) < 1e-6);

  // —— 风速矢量与等效加速度同向、且量级成比例 ——
  {
    const v: number[] = windVelocity(0, 5);
    const a: number[] = windAccel(0, 5);
    check('风速矢量与等效加速度同向', Math.sign(v[1]) === Math.sign(a[1]));
    check('等效加速度 = 风速 × WIND_GAIN',
      Math.abs(a[1] - v[1] * WIND_GAIN) < 1e-9);
  }

  // —— 空气密度单调性 ——
  check('高温低压→密度<1', airDensityFactor(1000, 35) < 1.0);
  check('低温高压→密度>1', airDensityFactor(1030, 0) > 1.0);
  check('标准海平面→密度=1', Math.abs(airDensityFactor(1013.25, 15) - 1.0) < 0.01);
  check('密度被限幅到 [0.6,1.4]',
    airDensityFactor(5000, 100) <= 1.4 && airDensityFactor(1, -200) >= 0.6);

  // —— 平滑器：关闭时必须恒为无风（零回归的核心保证）——
  {
    const sm: WindSmoother = new WindSmoother();
    sm.targetVelX = 5;
    sm.targetVelZ = 5;
    sm.targetDensity = 0.8;
    for (let i: number = 0; i < 200; i++) {
      sm.tick(1 / 60, false);
    }
    const out: number[] = sm.tick(1 / 60, false);
    check('关闭时输出风速 0', out[0] === 0 && out[1] === 0);
    check('关闭时输出密度 1.0', out[2] === 1.0);
  }

  // —— 平滑器：开启后单调逼近目标 ——
  {
    const sm: WindSmoother = new WindSmoother();
    sm.targetVelX = 4;
    sm.targetVelZ = 0;
    sm.tick(1 / 60, true);
    const first: number = Math.abs(sm.velX);
    for (let i: number = 0; i < 600; i++) {
      sm.tick(1 / 60, true);
    }
    check('开启后向目标逼近', Math.abs(sm.velX) > first);
    check('最终收敛到目标', Math.abs(sm.velX - 4) < 0.05);
  }

  if (fails.length !== 0) {
    throw new Error(`Atmosphere 自检失败：${fails.join('、')}`);
  }
  Logger.info(`${TAG} [VERIFY] 大气换算自检 PASS（${total} 项断言全通过）`);
}
