/**
 * 无头飞行仿真演示（不依赖任何 AR 引擎 / 传感器）。
 * ---------------------------------------------------------------
 * 目的：证明「纯算法层」可脱离硬件独立运行与验收。
 *
 * 关键演示点 —— "空间感知输入" 契约：
 *   仿真器实现 SpatialPerception，把"墙在无人机前方多近"喂给 DroneController。
 *   控制器代码**零改动**即可被仿真器驱动，与真机 DepthSampler 走的是同一份契约。
 *
 * 运行：npm run sim  （底层用 tsx 直接执行本 TS 文件）
 */
import {
  DroneController,
  DroneState,
  SpatialPerception,
  Vec3,
  vec3,
} from '../src/index';

/**
 * 模拟"前方一堵墙"：墙固定在 AR -Z 方向 8m 处。
 * 无人机向前飞（offsetZ 越负越靠近），gap = 墙 - 无人机 在前向的投影。
 */
class WallAheadPerception implements SpatialPerception {
  private droneZ: number = 0;
  private readonly wallZ: number = -8; // 墙在 -Z 方向 8m
  readonly emergencyDist: number = 0.4; // 硬停面（public 供仿真断言读取）

  /** 每帧由仿真器把无人机当前 Z 写入 */
  setDroneZ(z: number): void {
    this.droneZ = z;
  }

  getObstacleDistance(): number {
    // forward = (0, 0, -1)；gap = (wall - drone) · forward
    const gap: number = (this.wallZ - this.droneZ) * -1;
    return gap;
  }

  /** 远离障碍的单位向量（朝 +Z，即机尾方向），供横移参考 */
  getObstacleNormal(): Vec3 {
    return vec3(0, 0, 1);
  }

  get wallStopZ(): number {
    // 在硬停面处无人机应停下的 Z（wallZ + margin，margin 沿 -Z）
    return this.wallZ + this.emergencyDist;
  }
}

interface RunResult {
  label: string;
  finalZ: number;
  minGap: number;
  emergencyHits: number;
  crashed: boolean;
  finalState: DroneState;
}

function runFlight(label: string, usePerception: boolean): RunResult {
  const d = new DroneController();
  const wall = new WallAheadPerception();
  d.placed = true;
  d.flying = true;
  if (usePerception) {
    d.perception = wall; // 关键：把仿真器接入"空间感知输入"契约
  } else {
    d.obstacleDistance = Infinity; // 无感知：防撞是盲的
  }

  const dt = 1 / 60;
  const frames = 60 * 20; // 模拟 20 秒
  let minGap = Infinity;
  let emergencyHits = 0;
  d.moveY = 1; // 满前向推杆

  for (let i = 0; i < frames; i++) {
    wall.setDroneZ(d.offsetZ);
    d.update(dt);
    const gap = wall.getObstacleDistance();
    if (gap < minGap) {
      minGap = gap;
    }
    if (d.avoidEmergency) {
      emergencyHits++;
    }
  }

  const finalGap = wall.getObstacleDistance();
  const crashed = finalGap <= wall.emergencyDist - 0.05; // 越过硬停面即视为"撞上"
  return {
    label,
    finalZ: d.offsetZ,
    minGap,
    emergencyHits,
    crashed,
    finalState: d.state,
  };
}

function printResult(r: RunResult): void {
  const stateName =
    r.finalState === DroneState.Unplaced ? 'Unplaced' :
    r.finalState === DroneState.Grounded ? 'Grounded' :
    r.finalState === DroneState.Flying ? 'Flying' :
    r.finalState === DroneState.Landing ? 'Landing' : 'EmergencyHold';
  // biome-ignore lint/suspicious/noConsole: 演示有意输出
  console.log(
    `  [${r.label}] 末位 Z=${r.finalZ.toFixed(2)}m  最近间隙=${r.minGap.toFixed(2)}m  ` +
    `急停帧=${r.emergencyHits}  终态=${stateName}  ${r.crashed ? '❌ 撞墙' : '✅ 未撞墙'}`,
  );
}

// biome-ignore lint/suspicious/noConsole: 演示有意输出
console.log('=== uav_flight_sim 无头飞行仿真 ===');
// biome-ignore lint/suspicious/noConsole: 演示有意输出
console.log('场景：无人机满前向推杆 20 秒，前方 8m 处有一堵墙。');

const baseline = runFlight('无感知(防撞盲)', false);
const avoided = runFlight('感知驱动避障', true);

// biome-ignore lint/suspicious/noConsole: 演示有意输出
console.log('');
printResult(baseline);
printResult(avoided);

// biome-ignore lint/suspicious/noConsole: 演示有意输出
console.log('');
if (baseline.crashed && !avoided.crashed) {
  // biome-ignore lint/suspicious/noConsole: 演示有意输出
  console.log('结论：无感知时无人机直接穿过墙体（模拟"撞墙"）；');
  // biome-ignore lint/suspicious/noConsole: 演示有意输出
  console.log('      接入 SpatialPerception 后，控制器在硬停面前自动急停，未越界。');
  // biome-ignore lint/suspicious/noConsole: 演示有意输出
  console.log('      同一份 DroneController 既能被仿真器、也能被真机 DepthSampler 驱动。');
} else {
  // biome-ignore lint/suspicious/noConsole: 演示有意输出
  console.log('注意：本轮仿真未呈现预期对比，请检查参数。');
}

// 退出码：避免撞墙才算成功验收
process.exit(avoided.crashed ? 1 : 0);
