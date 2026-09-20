/**
 * 零依赖自检入口：直接调用各模块的 selfTest()，失败则非零退出。
 * 即使不安装 vitest，也能用 `npm run selfcheck`（tsx）完成纯逻辑验收。
 */
import { selfTest as collisionSelfTest } from '../src/core/CollisionDetector';
import { selfTest as steeringSelfTest } from '../src/core/SteeringBehavior';
import { DroneController } from '../src/core/DroneController';
import { selfTest as hudSelfTest } from '../src/core/HudModel';

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

run('CollisionDetector', () => collisionSelfTest());
run('SteeringBehavior', () => steeringSelfTest());
run('DroneController', () => {
  DroneController.selfTest();
});
run('HudModel', () => {
  hudSelfTest();
});

if (failed) {
  process.exit(1);
}
