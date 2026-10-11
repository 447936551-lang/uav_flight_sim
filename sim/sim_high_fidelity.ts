/**
 * M4/M5 高保真层离线验证（社区原生）
 * ================================================================
 * 验证对象（**全部默认关闭 ⇒ 零回归**）：
 *   阶段A  L1 速率环内环        DroneController.useRateLoop
 *   阶段B  L2 增强气动          DroneController.aeroModel = 'enhanced'
 *          L4 执行器非线性      DroneController.actuatorModel = 'nonlinear'
 *   阶段C  L3 种子化湍流风      TurbulenceModel + SeedPRNG
 *   阶段D  L5 自主 EKF          DroneController.setEstimator('ekf')
 *          L6 传感器噪声        DroneController.setSensorNoise(true, seed)
 *   M5-1   PX4 参数映射         Px4Params.deriveControllerGains / px4EkfConfig
 *
 * 与 sim/ 既有脚本同约定：无外部随机源、定步长 dt=1/60、结论用真实计数，
 * 退出码 0 = 全过、1 = 有失败。结果写入 docs/high_fidelity_sim_result.txt。
 *
 * 与 App 侧 scripts/sim_stage_*.js 的关系：App 侧脚本靠正则**解析 .ets 源码**取常数
 * （因为 Node 无法直接 import ArkTS）；本仓已有真实模块可 import，故本脚本直接
 * 消费 src/ 下模块，断言同一批物理性质，不再重复内联物理公式。
 */
import { writeFileSync } from 'node:fs';
import { join } from 'node:path';

import { SeedPRNG } from '../src/core/SeedPRNG';
import { TurbulenceModel } from '../src/environment/Turbulence';
import { EkfEstimator } from '../src/dynamics/EkfEstimator';
import { SensorModel } from '../src/dynamics/SensorModel';
import {
  PX4_STOCK,
  deriveControllerGains,
  px4EkfConfig,
} from '../src/dynamics/Px4Params';
import { DroneController } from '../src/model/DroneController';

const DT: number = 1 / 60;

const fails: string[] = [];
const checks: string[] = [];
let total: number = 0;

function check(name: string, ok: boolean): void {
  total++;
  checks.push((ok ? 'PASS ' : 'FAIL ') + name);
  if (!ok) {
    fails.push(name);
  }
}

/**
 * 构造「已放置 + 起飞 + 无障碍 + 环境感知开」的机，
 * 先按 cfg 装配保真层，再跑 frames 帧。
 */
function runDrone(cfg: (d: DroneController) => void, frames: number): DroneController {
  const d: DroneController = new DroneController();
  d.placed = true;
  d.flying = true;
  d.obstacleDistance = Infinity;
  d.environmentActive = true;
  cfg(d);
  for (let i: number = 0; i < frames; i++) {
    d.update(DT);
  }
  return d;
}

/** 水平漂移（m） */
function horizDrift(d: DroneController): number {
  return Math.sqrt(d.offsetX * d.offsetX + d.offsetZ * d.offsetZ);
}

// ══════════════════════════════════════════════════════════════════
//  阶段A —— L1 速率环内环（姿态→角速度→力矩）
//  断言：① 开/关都能自稳；② 开与关行为等价（等刚度标定 ⇒ 零回归）
// ══════════════════════════════════════════════════════════════════
{
  const off: DroneController = runDrone((d) => {
    d.setFlightMode(true);
  }, 360);
  const on: DroneController = runDrone((d) => {
    d.setFlightMode(true);
    d.useRateLoop = true;
  }, 360);

  check('阶段A/速率环关：悬停漂移<5cm', horizDrift(off) < 0.05);
  check('阶段A/速率环开：悬停漂移<5cm', horizDrift(on) < 0.05);
  check('阶段A/速率环开：高度保持偏差<3cm', Math.abs(on.offsetY - 0.9) < 0.03);
  check('阶段A/速率环开：姿态基本水平(|pitch|,|roll|<2°)',
    Math.abs(on.pitch) < 2 && Math.abs(on.roll) < 2);
  // 等刚度标定：开/关行为必须一致（零回归核心断言）
  check('阶段A/零回归：速率环开与关漂移差<1cm',
    Math.abs(horizDrift(on) - horizDrift(off)) < 0.01);

  // 满前向：低头是「靠倾斜推力矢量产生前推力」的直接证据，而非装饰姿态。
  // 取**过程峰值**：加速段倾角最大（趋近 maxTilt），进入稳态巡航后只需抵消阻力，
  // 倾角会回落到很小的负值（实测约 −0.5°），故断言必须看峰值而不是末值。
  const fwd: DroneController = new DroneController();
  fwd.placed = true;
  fwd.flying = true;
  fwd.obstacleDistance = Infinity;
  fwd.environmentActive = true;
  fwd.setFlightMode(true);
  fwd.useRateLoop = true;
  fwd.moveY = 1;
  let peakNose: number = 0;
  for (let i: number = 0; i < 120; i++) {
    fwd.update(DT);
    if (fwd.pitch < peakNose) {
      peakNose = fwd.pitch;
    }
  }
  check('阶段A/速率环开：满前向低头(峰值 pitch<-3°)', peakNose < -3);
  check('阶段A/速率环开：满前向位移明显(offsetZ<-0.3)', fwd.offsetZ < -0.3);
  fwd.moveY = 0;
  for (let i: number = 0; i < 360; i++) {
    fwd.update(DT);
  }
  check('阶段A/速率环开：松手后停稳(速度<0.1m/s)', fwd.speed < 0.1);

  // 偏航：输入须真的产生偏航角速率（社区内核偏航走运动学通道，仍应产生 yawRate）
  const yw: DroneController = runDrone((d) => {
    d.setFlightMode(true);
    d.useRateLoop = true;
    d.yawInput = 1;
  }, 60);
  check('阶段A/速率环开：偏航输入产生偏航角速率', Math.abs(yw.yawRate) > 1);
}

// ══════════════════════════════════════════════════════════════════
//  阶段B —— L2 增强气动（机体气动矩阵+入流） + L4 执行器非线性
//  断言：① 两层各自开启后悬停不发散；② 非线性满爬升稳态竖直速度 ≤ 线性对照
// ══════════════════════════════════════════════════════════════════
{
  const enh: DroneController = runDrone((d) => {
    d.setFlightMode(true);
    d.aeroModel = 'enhanced';
  }, 360);
  check('阶段B/增强气动：悬停漂移<5cm', horizDrift(enh) < 0.05);
  check('阶段B/增强气动：高度保持偏差<3cm', Math.abs(enh.offsetY - 0.9) < 0.03);
  check('阶段B/增强气动：姿态基本水平(|pitch|,|roll|<2°)',
    Math.abs(enh.pitch) < 2 && Math.abs(enh.roll) < 2);

  const non: DroneController = runDrone((d) => {
    d.setFlightMode(true);
    d.actuatorModel = 'nonlinear';
  }, 360);
  check('阶段B/执行器非线性：悬停漂移<5cm', horizDrift(non) < 0.05);
  check('阶段B/执行器非线性：高度保持偏差<3cm', Math.abs(non.offsetY - 0.9) < 0.03);
  check('阶段B/执行器非线性：姿态基本水平(|pitch|,|roll|<2°)',
    Math.abs(non.pitch) < 2 && Math.abs(non.roll) < 2);

  // 满爬升（集总需求大）→ 实际旋翼转速受电压跌落压制，
  // 稳态竖直速度不快于线性对照。
  const lin: DroneController = runDrone((d) => {
    d.setFlightMode(true);
    d.actuatorModel = 'linear';
    d.climbInput = 1;
  }, 120);
  const nonC: DroneController = runDrone((d) => {
    d.setFlightMode(true);
    d.actuatorModel = 'nonlinear';
    d.climbInput = 1;
  }, 120);
  check('阶段B/执行器非线性：满爬升稳态竖直速度 ≤ 线性对照（电压跌落削减可用推力）',
    nonC.velY <= lin.velY + 1e-6);
}

// ══════════════════════════════════════════════════════════════════
//  阶段C —— L3 种子化湍流风（OU 过程）
//  断言：① PRNG 可复现；② 湍流可复现；③ 湍流有界；④ 强度按比例缩放
// ══════════════════════════════════════════════════════════════════
{
  const a: SeedPRNG = new SeedPRNG(42);
  const b: SeedPRNG = new SeedPRNG(42);
  const c: SeedPRNG = new SeedPRNG(43);
  let sameSeq: boolean = true;
  let diffSeq: boolean = false;
  let inRange: boolean = true;
  for (let i: number = 0; i < 200; i++) {
    const va: number = a.nextFloat();
    const vb: number = b.nextFloat();
    const vc: number = c.nextFloat();
    if (va !== vb) { sameSeq = false; }
    if (va !== vc) { diffSeq = true; }
    if (!(va >= 0 && va < 1)) { inRange = false; }
  }
  check('阶段C/SeedPRNG：同种子同序列', sameSeq);
  check('阶段C/SeedPRNG：异种子异序列', diffSeq);
  check('阶段C/SeedPRNG：输出落在 [0,1)', inRange);

  const t1: TurbulenceModel = new TurbulenceModel(1, 1.0);
  const t2: TurbulenceModel = new TurbulenceModel(1, 1.0);
  let turbSame: boolean = true;
  let turbMax: number = 0;
  for (let i: number = 0; i < 2000; i++) {
    const p1 = t1.update(1 / 240);
    const p2 = t2.update(1 / 240);
    if (p1.wx !== p2.wx || p1.wz !== p2.wz) { turbSame = false; }
    turbMax = Math.max(turbMax, Math.abs(p1.wx), Math.abs(p1.wz));
  }
  check('阶段C/湍流：同种子同序列（可复现）', turbSame);
  check('阶段C/湍流：有界(|w| < 6σ)', turbMax < 6 * 1.2);

  const weak: TurbulenceModel = new TurbulenceModel(7, 0.5);
  const strong: TurbulenceModel = new TurbulenceModel(7, 2.0);
  let sumWeak: number = 0;
  let sumStrong: number = 0;
  for (let i: number = 0; i < 2000; i++) {
    sumWeak += Math.abs(weak.update(1 / 240).wx);
    sumStrong += Math.abs(strong.update(1 / 240).wx);
  }
  check('阶段C/湍流：强度缩放按比例放大扰动', sumStrong > sumWeak * 2);
}

// ══════════════════════════════════════════════════════════════════
//  阶段D —— L5 自主 EKF + L6 传感器噪声（**诊断级**：不接入控制环）
//  断言：① EKF 输出全为有限值；② 无噪声时跟踪真值；③ 噪声开启后仍有限；
//        ④ 湍流+噪声 同种子端到端可复现
// ══════════════════════════════════════════════════════════════════
{
  const d: DroneController = new DroneController();
  d.placed = true;
  d.flying = true;
  d.obstacleDistance = Infinity;
  d.environmentActive = true;
  d.setFlightMode(true);
  d.setEstimator('ekf');
  for (let i: number = 0; i < 600; i++) {
    d.update(DT);
  }
  const est: number[] = [d.ekfPosX, d.ekfPosY, d.ekfPosZ, d.ekfVelX, d.ekfVelY, d.ekfVelZ];
  check('阶段D/EKF：估计量全为有限值', est.every((v: number) => isFinite(v)));
  const posErr: number = Math.sqrt(
    (d.ekfPosX - d.offsetX) * (d.ekfPosX - d.offsetX) +
    (d.ekfPosZ - d.offsetZ) * (d.ekfPosZ - d.offsetZ));
  check('阶段D/EKF：无噪声时位置跟踪真值(水平偏差<20cm)', posErr < 0.2);

  const d2: DroneController = new DroneController();
  d2.placed = true;
  d2.flying = true;
  d2.obstacleDistance = Infinity;
  d2.environmentActive = true;
  d2.setFlightMode(true);
  d2.setSensorNoise(true, 7);
  d2.setEstimator('ekf');
  for (let i: number = 0; i < 600; i++) {
    d2.update(DT);
  }
  check('阶段D/传感器噪声：开启后 EKF 输出仍有限',
    isFinite(d2.ekfPosY) && isFinite(d2.ekfVelY));

  // 可复现性：同种子两轮跑出**逐位相同**的轨迹
  function makeRun(): DroneController {
    const dd: DroneController = new DroneController();
    dd.placed = true;
    dd.flying = true;
    dd.obstacleDistance = Infinity;
    dd.environmentActive = true;
    dd.setFlightMode(true);
    dd.setTurbulence(true, 1.0, 5);
    dd.setSensorNoise(true, 5);
    dd.setEstimator('ekf');
    for (let i: number = 0; i < 300; i++) {
      dd.update(DT);
    }
    return dd;
  }

  const r1: DroneController = makeRun();
  const r2: DroneController = makeRun();
  check('阶段D：湍流+噪声 同种子端到端可复现',
    r1.offsetZ === r2.offsetZ && r1.ekfPosZ === r2.ekfPosZ);
}

// ══════════════════════════════════════════════════════════════════
//  M5-1 —— PX4 参数映射一致性（防止标定漂移）
// ══════════════════════════════════════════════════════════════════
{
  const g = deriveControllerGains(PX4_STOCK);
  // 与本仓 DroneController 的 DYN_* 常量对照（等刚度标定锚点）
  check('M5-1：推导 attRateKp == DYN_ATT_RATE_KP(7.5)', Math.abs(g.attRateKp - 7.5) < 1e-9);
  check('M5-1：推导 rateKp == DYN_RATE_KP(30)', Math.abs(g.rateKp - 30.0) < 1e-9);
  check('M5-1：推导 yawRateKp == DYN_YAW_RATE_KP(8)', Math.abs(g.yawRateKp - 8.0) < 1e-6);
  check('M5-1：推导 maxRate == DYN_MAX_RATE(6)', Math.abs(g.maxRate - 6.0) < 1e-9);
  check('M5-1：等刚度 attRateKp*rateKp == 225', Math.abs(g.attRateKp * g.rateKp - 225.0) < 1e-6);

  const cfg = px4EkfConfig(PX4_STOCK);
  check('M5-1：EKF accelNoise 等量标定 EKF2_ACC_NOISE',
    Math.abs(cfg.accelNoise - PX4_STOCK.EKF2_ACC_NOISE) < 1e-12);
  check('M5-1：EKF gyroNoise 等量标定 EKF2_GYR_NOISE',
    Math.abs(cfg.gyroNoise - PX4_STOCK.EKF2_GYR_NOISE) < 1e-12);
}

// ══════════════════════════════════════════════════════════════════
//  零回归总闸 —— 默认（全部关闭）轨迹必须与运动学基线逐位一致
// ══════════════════════════════════════════════════════════════════
{
  // ① 从未触碰保真层（默认构造）→ 运动学基线
  const baseline: DroneController = runDrone((_d) => {
    // 默认：useDynamics=false，全部保真层关闭
  }, 300);

  // ② 高保真开→关往返后，须回到与基线完全一致的状态与轨迹
  const roundTrip: DroneController = runDrone((d) => {
    d.setHighFidelityEnabled(true);
    d.setHighFidelityEnabled(false);
  }, 300);

  check('零回归：高保真开→关往返后轨迹与默认逐位一致',
    roundTrip.offsetX === baseline.offsetX &&
    roundTrip.offsetY === baseline.offsetY &&
    roundTrip.offsetZ === baseline.offsetZ);

  // ③ 默认路径不得因新增模块而改变（M4/M5 均为 no-op）
  check('零回归：默认 estimator 仍为 none', baseline.estimator === 'none');
  check('零回归：默认 useRateLoop 仍为 false', baseline.useRateLoop === false);
  check('零回归：默认 aeroModel 仍为 basic', baseline.aeroModel === 'basic');
  check('零回归：默认 actuatorModel 仍为 linear', baseline.actuatorModel === 'linear');
  check('零回归：默认 turbulence 仍为 false', baseline.turbulence === false);
  check('零回归：默认 sensorNoise 仍为 false', baseline.sensorNoise === false);
}

// ══════════════════════════════════════════════════════════════════
//  报告
// ══════════════════════════════════════════════════════════════════
const allPass: boolean = fails.length === 0;
const lines: string[] = [];
lines.push('=== M4/M5 高保真层离线验证 ===');
lines.push('');
lines.push(`总断言 ${total} 项 —— ${allPass ? 'ALL PASS' : 'FAIL (' + fails.length + ')'}`);
lines.push('');
lines.push('--- 断言明细 ---');
for (const c of checks) {
  lines.push(c.startsWith('PASS') ? '  [PASS] ' + c.slice(5) : '  [FAIL] ' + c.slice(5));
}
if (!allPass) {
  lines.push('');
  lines.push('FAILED: ' + fails.join(' | '));
}
const out: string = lines.join('\n');
console.log(out);

writeFileSync(join(__dirname, '..', 'docs', 'high_fidelity_sim_result.txt'), out + '\n');

process.exit(allPass ? 0 : 1);
