/**
 * 多源融合几何（算法本质层 · 与 AR 完全解耦）
 * ---------------------------------------------------------------
 * 本模块从 OpenHarmony App 的 `ARDepthSampler` 中**抽离出的纯算法本质**
 * （见仓库 `docs/extraction-map.md` C2 条款）：
 *   · 近场最小距离闸门 `DEPTH_MIN_VALID_M`；
 *   · 支撑面 / 上方命中判据（`isSupportSurfaceHit` / `isOverheadHit`）；
 *   · 单世界系命中点的「前向障碍净距」折算 `forwardGapFromWorldHit`；
 *   · 射线-平面 / 射线-三角形求交 `rayPlaneT` / `rayTriangleT`。
 *
 * 这些函数**不引用任何 @kit.AREngine / ArkGraphics3D / hilog**，
 * 输入纯数字 / 纯结构体，输出纯数据，因此可在 Node、CI、OpenHarmony 三端
 * 复用同一份逻辑（App 侧的完整采样编排仍留在闭源层，不进公开仓）。
 *
 * 与 App 的唯一命名 / 类型差异（见 extraction-map.md 第 5 节「唯一允许的命名分叉」）：
 *   · App 的 `Vec3` 是元组 `[number,number,number]`，本仓是对象 `{x,y,z}`；
 *     故本文件所有向量访问由 `[0]/[1]/[2]` 改为 `.x/.y/.z`，几何含义逐位一致。
 *   · 阈值常量（`AVOID_CORRIDOR_HALF_M` / `AVOID_BELOW_PATH_M` / `AVOID_MARGIN_M`）
 *     由 `CollisionDetector` 单一事实来源导出，不在此重复定义。
 *   · App 私有探针档 `PROBES_M[0]=0.3m` 以 `MIN_PROBE_DIST_M` 镜像到本仓，
 *     仅用于近场闸门断言（闸门必须 ≪ 最小停障距离与最小探针档，确保刹停零回归）。
 */

import { Logger } from '../core/Logger';
import {
  AVOID_CORRIDOR_HALF_M,
  AVOID_BELOW_PATH_M,
  AVOID_MARGIN_M,
} from './CollisionDetector';
import { Vec3 } from '../core/Vec3';

const TAG: string = 'FusionGeometry';

// ---------------------------------------------------------------
//  近场最小距离闸门（2026-10-08）
// ---------------------------------------------------------------
// 取值 0.05m（5cm）：既干净剔除近零噪声，又远低于「最小有效停障距离」——
// 硬停面基准 AVOID_MARGIN_BASE_M=0.25m（见 CollisionDetector）、最小探针档 0.3m，
// 真机（Pura70）实测刹停那一帧像素读数约 0.29m，远在此门之上，
// 故本闸门不会误伤任何已验证行为。**默认开启**（仅剔噪声，不改变正常刹停）。
const DEPTH_MIN_VALID_M: number = 0.05;
const DEPTH_MIN_VALID_MM: number = DEPTH_MIN_VALID_M * 1000;

/** 最小探针档（米）—— 镜像 App `PROBES_M[0]=0.3m`，仅用于近场闸门断言。 */
const MIN_PROBE_DIST_M: number = 0.3;

// ---------------------------------------------------------------
//  支撑面 / 上方命中判据（2026-09-25 收紧 + 2026-10-05 高度分段）
// ---------------------------------------------------------------
/**
 * 高空时的支撑面容差（米）—— 沿用原 0.25。机体离支撑面很远时，
 * 落在这条带内的命中几乎只可能是真正的地面/桌面，放宽是安全的。
 */
const GROUND_PLANE_EPS: number = 0.25;
/**
 * 低空时的支撑面容差（米）—— 显著收紧。锚点放桌面时，
 * 桌面上 0~25cm 的真实障碍（笔记本 / 水杯 / 盒子 / 书本）若用 0.25 会被当成地面剔掉，
 * 造成低空掠过桌面时它们完全隐形。低空严判只剔「与支撑面几乎共面」的命中。
 */
const GROUND_PLANE_EPS_LOW: number = 0.05;
/** 高度分段阈值（米）：≤ 此值按低空严判，> 此值按高空宽松。 */
const GROUND_LOW_ALT: number = 0.45;

/**
 * 「该命中点是不是脚下的支撑面（地面/桌面）」——纯函数，无副作用。
 * 两个条件**同时**成立才算：
 *   ① 命中点世界 Y 落在放置面高度附近（|surfY − baseY| < 容差，容差随高度分段）；
 *   ② 命中点在机体（探针原点）**下方**（surfY < originWorldY）。
 * 基准是**机体自身高度**，不是相机/手机 —— 修掉了旧版拿视线方向当表面法向、
 * 把前方竖直面误当地面剔掉的系统性漏检。
 * @param surfWorldY  命中点的世界 Y（米）
 * @param baseY       放置面高度（支撑面判据基准，米）
 * @param originWorldY 探针原点的世界 Y（机体高度基准，米）
 */
export function isSupportSurfaceHit(surfWorldY: number, baseY: number,
  originWorldY: number): boolean {
  const airHeight: number = originWorldY - baseY;
  // 低空严判、高空宽松
  const eps: number = (airHeight <= GROUND_LOW_ALT) ? GROUND_PLANE_EPS_LOW : GROUND_PLANE_EPS;
  const onSupportPlane: boolean = Math.abs(surfWorldY - baseY) < eps;
  const belowAirframe: boolean = (surfWorldY - originWorldY) < 0;
  return onSupportPlane && belowAirframe;
}

/**
 * 上方命中判据（与 isSupportSurfaceHit 方向相反）：命中点是否算「正上方的实体」。
 * 上方探针朝正上方打，命中点按构造位于机体上方；地面在机体下方，靠这一条被剔掉。
 * 阈值为 0（非数）：等高也算命中（螺旋桨盘面半径远大于球半径，等高仍会扫到）。
 */
export function isOverheadHit(surfWorldY: number, originWorldY: number): boolean {
  return surfWorldY >= originWorldY;
}

/**
 * 多源融合：单个世界系命中点的「前向障碍净距」折算（纯函数，可离线断言）。
 * ---------------------------------------------------------------
 * 与深度主循环完全同一套口径 —— 走廊 / 支撑面 / 可飞越三道判据，
 * 阈值全部来自物理层（AVOID_CORRIDOR_HALF_M / AVOID_BELOW_PATH_M / isSupportSurfaceHit），
 * 保证平面源、网格源与深度源「什么算前方障碍」定义一致，融合才不会引入幽灵刹停。
 * 不达标（被任一判据剔除）→ 返回 Infinity（等于「这源没看到障碍」）。
 *
 * @param hit     命中点世界坐标（米）
 * @param origin  射线原点（无人机世界坐标，米）
 * @param fwd     机头前向单位向量（世界系）
 * @param baseY   放置面高度（支撑面判据基准）
 * @param originY 射线原点的世界 Y（支撑面/可飞越判据的「机体高度」基准）
 */
export function forwardGapFromWorldHit(hit: Vec3, origin: Vec3, fwd: Vec3,
  baseY: number, originY: number): number {
  const rx: number = hit.x - origin.x;
  const ry: number = hit.y - origin.y;
  const rz: number = hit.z - origin.z;
  // 前向净距 = 命中点相对原点在机头前向的投影（旋转不变量）
  const gap: number = rx * fwd.x + ry * fwd.y + rz * fwd.z;
  if (!(gap > 0)) {
    return Infinity; // 在机体后方或正侧方，不算前方障碍
  }
  // 到航向轴线的垂直距离（旋转不变量，与坐标系无关）
  const relLenSq: number = rx * rx + ry * ry + rz * rz;
  const lateral: number = Math.sqrt(Math.max(0, relLenSq - gap * gap));
  if (lateral > AVOID_CORRIDOR_HALF_M) {
    return Infinity; // 偏离走廊，侧方掠过不刹停
  }
  if (isSupportSurfaceHit(hit.y, baseY, originY)) {
    return Infinity; // 贴放置面的地板，不刹停
  }
  if (hit.y - originY < -AVOID_BELOW_PATH_M) {
    return Infinity; // 在机体下方超过一个机体半径，可飞越
  }
  return gap;
}

/**
 * 射线（origin + t·dir, t>0）与平面（过点 c、单位法向 n）求交，返回 t（米）或 -1。
 * 纯函数。dir 不必归一化，但必须非零。与深度主循环共用同一坐标系（AR 世界系）。
 */
export function rayPlaneT(origin: Vec3, dir: Vec3, c: Vec3, n: Vec3): number {
  const denom: number = dir.x * n.x + dir.y * n.y + dir.z * n.z;
  if (Math.abs(denom) < 1e-9) {
    return -1; // 射线与平面平行
  }
  const t: number = ((c.x - origin.x) * n.x + (c.y - origin.y) * n.y +
    (c.z - origin.z) * n.z) / denom;
  return t > 1e-6 ? t : -1; // 交点在背后 → 忽略
}

/**
 * 射线与三角形（Möller–Trumbore）求交，返回 t（米）或 -1。纯函数。
 * 用于网格源：把前向射线投进场景重建网格，找最近命中面。dir 不必归一化。
 */
export function rayTriangleT(orig: Vec3, dir: Vec3, v0: Vec3, v1: Vec3, v2: Vec3): number {
  const EPS: number = 1e-9;
  const e1x: number = v1.x - v0.x, e1y: number = v1.y - v0.y, e1z: number = v1.z - v0.z;
  const e2x: number = v2.x - v0.x, e2y: number = v2.y - v0.y, e2z: number = v2.z - v0.z;
  const px: number = dir.y * e2z - dir.z * e2y;
  const py: number = dir.z * e2x - dir.x * e2z;
  const pz: number = dir.x * e2y - dir.y * e2x;
  const det: number = e1x * px + e1y * py + e1z * pz;
  if (Math.abs(det) < EPS) {
    return -1; // 平行或退化三角形
  }
  const inv: number = 1 / det;
  const tx: number = orig.x - v0.x, ty: number = orig.y - v0.y, tz: number = orig.z - v0.z;
  const u: number = (tx * px + ty * py + tz * pz) * inv;
  if (u < 0 || u > 1) {
    return -1;
  }
  const qx: number = ty * e1z - tz * e1y;
  const qy: number = tz * e1x - tx * e1z;
  const qz: number = tx * e1y - ty * e1x;
  const v: number = (dir.x * qx + dir.y * qy + dir.z * qz) * inv;
  if (v < 0 || u + v > 1) {
    return -1;
  }
  const t: number = (e2x * qx + e2y * qy + e2z * qz) * inv;
  return t > 1e-6 ? t : -1;
}

/** 暴露近场闸门常量供上游消费 / 断言 */
export { DEPTH_MIN_VALID_M, DEPTH_MIN_VALID_MM, MIN_PROBE_DIST_M };

/**
 * 纯函数自检（可在 CI / 真机 / 离线仿真直接验收，无需单测框架）。
 * 复刻 App `ARDepthSampler.selfTest` 的「近场闸门 + 多源融合几何」断言簇，
 * 用计数器统计而非写死数字，断言增删时不会打印错误的总项数。
 */
export function selfTest(): string {
  const fails: string[] = [];
  let total: number = 0;
  const check = (name: string, ok: boolean): void => {
    total++;
    if (!ok) {
      fails.push(name);
    }
  };

  // —— 近场最小距离闸门（2026-10-08）：只剔极近噪声，不误伤已验证刹停 ——
  // DEPTH_MIN_VALID_M 必须 ≪ 最小停障距离与最小探针档，才能确保 0.29m 刹停零回归。
  check('近场闸门 ≪ 最小停障距离（不误伤刹停）',
    DEPTH_MIN_VALID_M < AVOID_MARGIN_M && DEPTH_MIN_VALID_M < MIN_PROBE_DIST_M);
  check('近场闸门物理合理（>0 且 ≪ 探针量程）',
    DEPTH_MIN_VALID_M > 0 && DEPTH_MIN_VALID_M < 0.1);

  // —— 多源融合几何（纯函数，离线钉死口径与深度主循环一致）——
  // forwardGapFromWorldHit：机头前向取世界系 (0,0,-1) 约定，正前方命中 → 净距=真实距离。
  const fwdZ: Vec3 = { x: 0, y: 0, z: -1 };
  check('前向正前方命中 → 净距=真实距离',
    Math.abs(forwardGapFromWorldHit({ x: 0, y: 0, z: -2 }, { x: 0, y: 0, z: 0 }, fwdZ, 0, 0) - 2) < 1e-9);
  check('后方命中 → ∞（非前向）',
    forwardGapFromWorldHit({ x: 0, y: 0, z: 2 }, { x: 0, y: 0, z: 0 }, fwdZ, 0, 0) === Infinity);
  check('走廊外命中 → ∞（幽灵刹停防护）',
    forwardGapFromWorldHit({ x: 1, y: 0, z: -1 }, { x: 0, y: 0, z: 0 }, fwdZ, 0, 0) === Infinity);
  check('支撑面命中 → ∞（地面不刹停）',
    forwardGapFromWorldHit({ x: 0, y: -0.1, z: -2 }, { x: 0, y: 0.9, z: 0 }, fwdZ, 0, 0.9) === Infinity);
  check('可飞越命中 → ∞（下方表面不刹停）',
    forwardGapFromWorldHit({ x: 0, y: -0.5, z: -2 }, { x: 0, y: 0.2, z: 0 }, fwdZ, 0, 0.2) === Infinity);

  // rayPlaneT：平面 z=-2（法向 +Z）、从原点朝 -Z → t=2。
  check('射线与平面求交 → t=2',
    Math.abs(rayPlaneT({ x: 0, y: 0, z: 0 }, { x: 0, y: 0, z: -1 }, { x: 0, y: 0, z: -2 }, { x: 0, y: 0, z: 1 }) - 2) < 1e-9);
  check('射线与平面平行 → -1',
    rayPlaneT({ x: 0, y: 0, z: 0 }, { x: 1, y: 0, z: 0 }, { x: 0, y: 0, z: -2 }, { x: 0, y: 0, z: 1 }) === -1);
  check('交点在背后 → -1',
    rayPlaneT({ x: 0, y: 0, z: 0 }, { x: 0, y: 0, z: 1 }, { x: 0, y: 0, z: -2 }, { x: 0, y: 0, z: 1 }) === -1);

  // rayTriangleT：三角形 (0,0,0)/(2,0,0)/(0,0,-2)（位于 y=0 平面），
  // 射线须带 y 分量以不共面：从 (0.5,1,1) 朝 (0,-1,-1) 命中内部 → t>0。
  check('射线与三角形内部求交命中 → t>0',
    rayTriangleT({ x: 0.5, y: 1, z: 1 }, { x: 0, y: -1, z: -1 },
      { x: 0, y: 0, z: 0 }, { x: 2, y: 0, z: 0 }, { x: 0, y: 0, z: -2 }) > 0);
  check('射线背离三角形 → -1',
    rayTriangleT({ x: 0.5, y: 1, z: 1 }, { x: 0, y: 1, z: 1 },
      { x: 0, y: 0, z: 0 }, { x: 2, y: 0, z: 0 }, { x: 0, y: 0, z: -2 }) === -1);

  const msg: string = fails.length === 0 ?
    `PASS（${total} 项断言全通过）` : `FAIL：${fails.join('、')}`;
  Logger.info(`${TAG} [VERIFY] 多源融合几何自检 ${msg}`);
  return msg;
}
