/**
 * 零依赖自检入口：直接调用各模块的 selfTest()，失败则非零退出。
 * 即使不安装 vitest，也能用 `npm run selfcheck`（tsx）完成纯逻辑验收。
 */
import { selfTest as collisionSelfTest } from '../src/avoidance/CollisionDetector';
import { selfTest as steeringSelfTest } from '../src/avoidance/SteeringBehavior';
import { DroneController } from '../src/model/DroneController';
import { selfTest as hudSelfTest } from '../src/telemetry/HudModel';
import { selfTest as rotorMixerSelfTest, RotorPlant } from '../src/dynamics/RotorMixer';
import { FlightDynamics } from '../src/dynamics/FlightDynamics';
import { selfTest as cascadeSelfTest } from '../src/dynamics/CascadeController';
import { selfTest as pathPlannerSelfTest } from '../src/planning/PathPlanner';
import { selfTest as meshGridSelfTest } from '../src/planning/MeshGrid';
import { selfTest as atmosphereSelfTest } from '../src/environment/Atmosphere';
import { selfTest as weatherCodeSelfTest } from '../src/environment/WeatherCode';

let failed = false;
function run(name: string, fn: () => void | string): void {
  try {
    const r = fn();
    const msg = typeof r === 'string' ? r : 'PASS';
    // biome-ignore lint/suspicious/noConsole: 自检有意输出
    console.log(`[selfcheck] ${name}: ${msg}`);
  } catch (e) {
    failed = true;
    // biome-ignore lint/suspicious/noConsole: 自检有意输出
    console.error(`[selfcheck] ${name}: FAIL -> ${(e as Error).message}`);
  }
}

// ── 既有模块 ────────────────────────────────────────────────────
run('CollisionDetector', () => collisionSelfTest());
run('SteeringBehavior', () => steeringSelfTest());
run('DroneController', () => {
  DroneController.selfTest();
});
run('HudModel', () => {
  hudSelfTest();
});

// ── 动力学层 ────────────────────────────────────────────────────
run('RotorMixer', () => rotorMixerSelfTest());
run('RotorPlant', () => {
  RotorPlant.selfTest();
});
run('FlightDynamics', () => {
  FlightDynamics.selfTest();
});
run('CascadeController', () => cascadeSelfTest());

// ── 规划层 ──────────────────────────────────────────────────────
run('PathPlanner', () => pathPlannerSelfTest());
run('MeshGrid', () => meshGridSelfTest());

// ── 环境层 ──────────────────────────────────────────────────────
run('Atmosphere', () => atmosphereSelfTest());
run('WeatherCode', () => weatherCodeSelfTest());

if (failed) {
  process.exit(1);
}
