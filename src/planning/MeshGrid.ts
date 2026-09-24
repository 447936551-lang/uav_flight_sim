/**
 * 阶段4：场景网格（mesh） → 2.5D 占用栅格
 * ---------------------------------------------------------------
 * 定位：**阶段 1（平面栅格化）的替代数据源**，不是替代规划器。
 *   MeshGrid 只负责「顶点/索引 → OccupancyGrid」，
 *   之后的 A*（PathPlanner.planPath）完全复用，一行不改 ——
 *   阶段 1 与阶段 4 因此彻底解耦：mesh 不可用就退回平面图，规划器无感。
 *
 * 为什么值得做：平面检测只能给出「地板 + 墙 + 桌面」这几张规整大面，
 * 表达不了椅子腿、纸箱、斜靠的板子这类非平面障碍；
 * 场景网格是真实重建出来的三角面，能把它们照进地图。
 *
 * 与平面版的三个关键差异（都是必须想清楚的地方）：
 *
 * 1. **高度判定从「一个点」变成「一段区间」**
 *    平面有确定的 cy，可以「一个数定生死」；三角面是三维的，
 *    必须用它的 y 区间 [ymin, ymax] 与飞行带 [obstacleMinY, FLIGHT_BAND_TOP_Y]
 *    求交集：有交集才算障碍。否则一面从地面顶到天花板的墙，
 *    会因为它「最高点在天花板之上」而被误判成天花板。
 *
 * 2. **垂直面不能填内部**
 *    与平面版同一个坑：墙在 XZ 上的投影是一条线，填内部等于把墙加厚成一片，
 *    会把可行区域整块吃掉。垂直三角形只栅格化三条边（Bresenham 保证连续无孔）。
 *
 * 3. **优先序必须先算清楚再写**
 *    一个格可能同时被「地板」（要 FREE）和「墙」（要 OCCUPIED）覆盖。
 *    直接边算边写，结果是**后处理的那个赢**，与遍历顺序有关 —— 不可复现。
 *    这里把两类标记分别栅格化到两张临时掩膜，最后按
 *    OCCUPIED > FREE > UNKNOWN 统一合成，与顺序无关。
 */
import { Logger } from '../core/Logger';
import {
  CELL_FREE,
  CELL_OCCUPIED,
  CELL_UNKNOWN,
  DEFAULT_GRID_CONFIG,
  FLIGHT_BAND_TOP_Y,
  GridConfig,
  OccupancyGrid,
  WorldPlane,
  inflate,
  isHorizontal,
  isVertical,
  rasterizePolygonFill,
  rasterizeSegment
} from './PathPlanner';

const TAG: string = 'MeshGrid';

/** 「什么都不标」的哨兵值（既不是 FREE 也不是 OCCUPIED） */
const MARK_SKIP: number = -2;

/**
 * 退化三角形下限（叉积模长 = 2×面积，单位 m²）。
 * 场景网格重建常产出「三点几乎共线」的碎三角形，它们的法向数值上是噪声，
 * 拿去分类等于掷骰子 —— 直接跳过，不表态（把决定权交给附近的正常三角形）。
 */
const TRI_MIN_AREA2: number = 1e-8;

/**
 * 场景网格快照（纯数据，**AR 原生世界坐标**）。
 * 与平面快照同样的理由：ARSceneMesh 是帧作用域对象，出了帧回调就不能再碰，
 * 必须在帧内把顶点/索引摊平成普通数组存下来。
 */
export interface MeshSnapshot {
  /** 顶点：每 3 个连续分量为一个点 (x,y,z) */
  vertices: number[];
  /** 三角形索引：每 3 个连续分量构成一个三角形 */
  indices: number[];
  /** 顶点个数（诊断用） */
  vertexCount: number;
  /** 三角形个数（诊断用） */
  triangleCount: number;
}

/** 三角形朝向分类 */
export type TriOrientation = 'horizontal' | 'vertical' | 'sloped';

/** 由三角形法向的 y 分量分类（与平面版同一套阈值口径） */
export function classifyTriangle(ny: number, cfg: GridConfig): TriOrientation {
  if (Math.abs(ny) >= cfg.horizontalNy) {
    return 'horizontal';
  }
  if (Math.abs(ny) <= cfg.verticalNy) {
    return 'vertical';
  }
  return 'sloped';
}

/**
 * 一个三角形该在栅格上标什么（纯函数，可离线断言）。
 *
 * 返回 CELL_FREE / CELL_OCCUPIED，或 MARK_SKIP（与飞行带无关，不表态）。
 * 注意返回值是「这个三角形的主张」，最终谁是 FREE 谁是 OCCUPIED 由合成阶段裁决。
 */
export function markForTriangle(orient: TriOrientation, ymin: number, ymax: number,
  cfg: GridConfig): number {
  if (orient === 'horizontal') {
    // 与平面版完全同口径：够高 → 障碍（桌面/床面）；够低 → 地板
    return ymin >= cfg.obstacleMinY ? CELL_OCCUPIED
      : (ymax < cfg.floorMaxY ? CELL_FREE : CELL_OCCUPIED);
  }
  // 垂直 / 斜面：看是否与飞行带相交
  if (ymax >= cfg.obstacleMinY && ymin <= FLIGHT_BAND_TOP_Y) {
    return CELL_OCCUPIED;
  }
  if (ymax < cfg.floorMaxY) {
    // 很矮的台阶 / 门槛：低于飞行带下沿，可从上方越过
    return CELL_FREE;
  }
  // 整体都在飞行带之上（天花板、屋梁）—— 不表态，交点交给别的三角形
  return MARK_SKIP;
}

/**
 * 三角形**单位法向**的 y 分量（n = u × v 归一化后取 y；退化时返回 0）。
 *
 * ⚠️ 必须归一化 —— 这里踩过一个真机才暴露的坑：
 *   未归一化的叉积模长 = 2 × 三角形在 XZ 上的投影**面积**，于是同样朝向的
 *   水平三角形，边长 1m 时 |n_y| ≈ 1、边长 5cm 时 |n_y| ≈ 0.0025。
 *   拿它去比 `horizontalNy = 0.9` 这种**归一化口径**的阈值（与平面版
 *   `isHorizontal` 同一套），小三角形会被**全部判成垂直** ——
 *   真机日志就是「水平 0 / 垂直 60006」，地板一块都没认出来，
 *   free 格只能靠零星大三角形产生。场景网格恰是几万个小三角形，必踩。
 *   平面版没这个问题：ARPlane 的 ny 本身就是归一化过的。
 */
export function triNormalY(ax: number, ay: number, az: number,
  bx: number, by: number, bz: number,
  cx: number, cy: number, cz: number): number {
  const ux: number = bx - ax;
  const uy: number = by - ay;
  const uz: number = bz - az;
  const vx: number = cx - ax;
  const vy: number = cy - ay;
  const vz: number = cz - az;
  // n = u × v
  const nx: number = uy * vz - uz * vy;
  const ny: number = uz * vx - ux * vz;
  const nz: number = ux * vy - uy * vx;
  const len: number = Math.sqrt(nx * nx + ny * ny + nz * nz);
  if (len < 1e-12) {
    return 0; // 退化：朝向无定义
  }
  return ny / len;
}

/**
 * 三角形叉积模长（= 2 × 三维面积，m²）。
 * 单独抽出来是为了让调用方**先判退化再分类**：退化三角形的归一化法向是噪声，
 * 直接分类等于掷骰子（真实 mesh 里这类碎三角形并不少见）。
 */
export function triArea2(ax: number, ay: number, az: number,
  bx: number, by: number, bz: number,
  cx: number, cy: number, cz: number): number {
  const ux: number = bx - ax;
  const uy: number = by - ay;
  const uz: number = bz - az;
  const vx: number = cx - ax;
  const vy: number = cy - ay;
  const vz: number = cz - az;
  const nx: number = uy * vz - uz * vy;
  const ny: number = uz * vx - ux * vz;
  const nz: number = ux * vy - uy * vx;
  return Math.sqrt(nx * nx + ny * ny + nz * nz);
}

/** 造一张与主图同几何、格子全为 UNKNOWN 的临时掩膜 */
function newMask(width: number, height: number, originX: number,
  originZ: number, cell: number): OccupancyGrid {
  const cells: Int8Array = new Int8Array(width * height);
  cells.fill(CELL_UNKNOWN);
  return new OccupancyGrid(originX, originZ, cell, width, height, cells);
}

/**
 * 场景网格快照 → 2.5D 占用栅格。
 *
 * 输入顶点必须是**与航点同口径的坐标**（AR 世界系需先减去放置锚点，见调用方）；
 * 否则规划出的路径会整体偏掉一个锚点位移。
 *
 * @param snap 网格快照（顶点 + 三角形索引）
 * @param startX 起点世界 X（m）—— 地图以它为中心
 * @param startZ 起点世界 Z（m）
 * @param cfg 栅格化配置（默认与平面版同一份 DEFAULT_GRID_CONFIG）
 */
export function buildGridFromMesh(snap: MeshSnapshot, startX: number, startZ: number,
  cfg: GridConfig = DEFAULT_GRID_CONFIG): OccupancyGrid {
  const cell: number = cfg.cellSize > 1e-3 ? cfg.cellSize : 0.1;
  const half: number = cfg.halfExtent > cell ? cfg.halfExtent : cell * 8;
  const width: number = Math.max(1, Math.ceil((half * 2) / cell));
  const height: number = width;
  const originX: number = startX - half;
  const originZ: number = startZ - half;

  const grid: OccupancyGrid = newMask(width, height, originX, originZ, cell);
  // 两张掩膜：FREE 与 OCCUPIED 分开收集，最后统一合成（与遍历顺序无关）
  const freeMask: OccupancyGrid = newMask(width, height, originX, originZ, cell);
  const occMask: OccupancyGrid = newMask(width, height, originX, originZ, cell);

  const v: number[] = snap.vertices;
  const idx: number[] = snap.indices;
  const triCount: number = Math.floor(idx.length / 3);
  let horizontals: number = 0;
  let verticals: number = 0;
  let slopeds: number = 0;

  for (let t = 0; t < triCount; t++) {
    const i0: number = idx[t * 3] * 3;
    const i1: number = idx[t * 3 + 1] * 3;
    const i2: number = idx[t * 3 + 2] * 3;
    // 索引越界保护：坏索引不能把整张地图带崩
    if (i0 + 2 >= v.length || i1 + 2 >= v.length || i2 + 2 >= v.length) {
      continue;
    }
    const ax: number = v[i0];
    const ay: number = v[i0 + 1];
    const az: number = v[i0 + 2];
    const bx: number = v[i1];
    const by: number = v[i1 + 1];
    const bz: number = v[i1 + 2];
    const cx: number = v[i2];
    const cy: number = v[i2 + 1];
    const cz: number = v[i2 + 2];

    // 退化三角形（三点几乎共线）：法向是数值噪声，跳过而不是拿它分类
    if (triArea2(ax, ay, az, bx, by, bz, cx, cy, cz) < TRI_MIN_AREA2) {
      continue;
    }

    const ny: number = triNormalY(ax, ay, az, bx, by, bz, cx, cy, cz);
    const orient: TriOrientation = classifyTriangle(ny, cfg);
    if (orient === 'horizontal') {
      horizontals++;
    } else if (orient === 'vertical') {
      verticals++;
    } else {
      slopeds++;
    }

    let ymin: number = ay;
    let ymax: number = ay;
    if (by < ymin) {
      ymin = by;
    }
    if (by > ymax) {
      ymax = by;
    }
    if (cy < ymin) {
      ymin = cy;
    }
    if (cy > ymax) {
      ymax = cy;
    }

    const mark: number = markForTriangle(orient, ymin, ymax, cfg);
    if (mark === MARK_SKIP) {
      continue;
    }
    const target: OccupancyGrid = mark === CELL_OCCUPIED ? occMask : freeMask;

    if (orient === 'vertical') {
      // 墙：只打三条边，绝不填内部（填了会把可行区域吃掉）
      rasterizeSegment(target, ax, az, bx, bz, mark);
      rasterizeSegment(target, bx, bz, cx, cz, mark);
      rasterizeSegment(target, cx, cz, ax, az, mark);
    } else {
      // 水平面 / 斜面：填内部（三角形摊平成 9 元组后复用平面版的填充分支）
      const poly: number[] = [ax, ay, az, bx, by, bz, cx, cy, cz];
      rasterizePolygonFill(target, poly, 3, mark);
      // 小三角形补丁：`rasterizePolygonFill` 按「**格中心**是否落在多边形内」判定，
      // 边长小于格宽（0.1m）的三角形很可能一个格中心都套不住 → 明明扫到的地板
      // 却仍是一片 unknown（真机 free=174 / unknown=24070 就是这么来的）。
      // 重心**必在**三角形内部，补标它所在的那一格是正确性补丁，不是保守膨胀。
      const gcx: number = (ax + bx + cx) / 3;
      const gcz: number = (az + bz + cz) / 3;
      const gc: number = target.colOf(gcx);
      const gr: number = target.rowOf(gcz);
      if (target.inBounds(gc, gr)) {
        target.cells[gr * target.width + gc] = mark;
      }
    }
  }

  // 合成：OCCUPIED > FREE > UNKNOWN
  const cells: Int8Array = grid.cells;
  const fc: Int8Array = freeMask.cells;
  const oc: Int8Array = occMask.cells;
  for (let i = 0; i < cells.length; i++) {
    if (oc[i] === CELL_OCCUPIED) {
      cells[i] = CELL_OCCUPIED;
    } else if (fc[i] === CELL_FREE) {
      cells[i] = CELL_FREE;
    }
  }

  // 障碍膨胀：与平面版同一份实现（含「先收集再写回」防滚雪球）
  if (cfg.inflateRadius > 0) {
    inflate(grid, cfg.inflateRadius);
  }

  const st = grid.stats();
  Logger.info(`${TAG} [VERIFY] mesh 栅格化：三角形 ${triCount} 个` +
    `（水平 ${horizontals} / 垂直 ${verticals} / 斜面 ${slopeds}），` +
    `${width}x${height} 格 @${cell}m，free=${st.free} occ=${st.occupied} unknown=${st.unknown}`);
  return grid;
}

/** 快照是否可用于规划（点数与索引都够） */
export function meshUsable(snap: MeshSnapshot | null): boolean {
  if (snap === null) {
    return false;
  }
  return snap.vertexCount >= 3 && snap.triangleCount >= 1;
}

// ---------------------------------------------------------------
//  自检（纯函数，无副作用；由 ARDroneSession 在 AR 初始化时跑一次）
// ---------------------------------------------------------------

/** 造一个单三角形快照 */
function triSnap(pts: number[]): MeshSnapshot {
  const s: MeshSnapshot = {
    vertices: pts,
    indices: [0, 1, 2],
    vertexCount: 3,
    triangleCount: 1
  };
  return s;
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
  const cfg: GridConfig = {
    cellSize: 0.5, halfExtent: 5.0, floorMaxY: 0.3, obstacleMinY: 0.3,
    inflateRadius: 0, horizontalNy: 0.9, verticalNy: 0.3
  };

  // —— 法向 y 分量：按 n = u × v 只取 y 分量（n_y = u_z·v_x − u_x·v_z）——
  // 顶点逆序（从上方看是逆时针）→ 朝上 n_y=+1；正序 → 朝下 n_y=−1；竖直面 → 0
  check('法向：逆序朝上 n_y=+1', triNormalY(0, 0, 0, 0, 0, 1, 1, 0, 0) > 0.9);
  check('法向：正序朝下 n_y=-1', triNormalY(0, 0, 0, 1, 0, 0, 0, 0, 1) < -0.9);
  check('法向：竖直面 n_y=0', Math.abs(triNormalY(0, 0, 0, 1, 0, 0, 0, 2, 0)) < 1e-9);

  // —— 朝向分类 ——
  check('分类：|ny|=1 → horizontal', classifyTriangle(1, cfg) === 'horizontal');
  check('分类：|ny|=0.95 → horizontal', classifyTriangle(0.95, cfg) === 'horizontal');
  check('分类：|ny|=0 → vertical', classifyTriangle(0, cfg) === 'vertical');
  check('分类：|ny|=0.2 → vertical', classifyTriangle(0.2, cfg) === 'vertical');
  check('分类：|ny|=0.6 → sloped', classifyTriangle(0.6, cfg) === 'sloped');
  check('分类：|ny|=-0.6 → sloped（法向朝下也算）', classifyTriangle(-0.6, cfg) === 'sloped');

  // —— 归一化回归（真机踩过的坑，见 triNormalY 注释）——
  // 边长 5cm 的水平三角形：未归一化时 |n_y| ≈ 0.0025，会被误判成「垂直」；
  // 真机症状就是「水平 0 / 垂直 60006」，地板一块都认不出来。
  {
    const s: number = 0.05;
    check('归一化：5cm 水平三角形 |ny|≈1（不是 0.0025）',
      Math.abs(Math.abs(triNormalY(0, 0, 0, s, 0, 0, 0, 0, s)) - 1) < 1e-6);
    check('归一化：5cm 水平三角形 → 判为 horizontal（回归）',
      classifyTriangle(triNormalY(0, 0, 0, s, 0, 0, 0, 0, s), cfg) === 'horizontal');
    check('归一化：5cm 竖直三角形 → 判为 vertical',
      classifyTriangle(triNormalY(0, 0, 0, s, 0, 0, 0, s, 0), cfg) === 'vertical');
    check('退化：三点共线 → 面积 0 且 ny=0',
      triArea2(0, 0, 0, 1, 0, 0, 2, 0, 0) < 1e-12 &&
      Math.abs(triNormalY(0, 0, 0, 1, 0, 0, 2, 0, 0)) < 1e-12);
  }

  // —— 端到端回归：小三角形拼成的地板必须能标出 FREE ——
  // 上一版 `rasterizePolygonFill` 只认「格中心落在多边形内」，
  // 边长 < 格宽的三角形一格都填不到 → 扫到的地板仍是 unknown。
  {
    const verts: number[] = [];
    const idx: number[] = [];
    // 4x4 = 16 个边长 5cm 的小正方形（每个拆 2 个三角形），覆盖 x/z ∈ [0, 0.2]
    let base: number = 0;
    const step: number = 0.05;
    for (let i = 0; i < 4; i++) {
      for (let j = 0; j < 4; j++) {
        const x0: number = i * step;
        const z0: number = j * step;
        const x1: number = x0 + step;
        const z1: number = z0 + step;
        verts.push(x0, 0, z0, x1, 0, z0, x1, 0, z1);
        verts.push(x0, 0, z0, x1, 0, z1, x0, 0, z1);
        idx.push(base, base + 1, base + 2, base + 3, base + 4, base + 5);
        base += 6;
      }
    }
    const snap: MeshSnapshot = {
      vertices: verts, indices: idx, vertexCount: base, triangleCount: base / 3
    };
    const cfgFine: GridConfig = {
      cellSize: 0.5, halfExtent: 5.0, floorMaxY: 0.3, obstacleMinY: 0.3,
      inflateRadius: 0, horizontalNy: 0.9, verticalNy: 0.3
    };
    const g: OccupancyGrid = buildGridFromMesh(snap, 0, 0, cfgFine);
    check('小三角形地板：全部判为水平', g.stats().free > 0);
    check('小三角形地板：中心格 FREE（重心补丁生效）',
      g.isFree(g.colOf(0.1), g.rowOf(0.1)));
  }

  // —— 高度判定：水平面 ——
  check('水平面 y=0 → FREE', markForTriangle('horizontal', 0, 0, cfg) === CELL_FREE);
  check('水平面 y=0.8 → OCCUPIED', markForTriangle('horizontal', 0.8, 0.8, cfg) === CELL_OCCUPIED);
  check('水平面 y=0.3 → OCCUPIED（正好在下沿）',
    markForTriangle('horizontal', 0.3, 0.3, cfg) === CELL_OCCUPIED);

  // —— 高度判定：垂直/斜面按「与飞行带求交集」——
  // 关键回归：一面从地面顶到天花板的墙，绝不能因为「最高点在天花板之上」被判成天花板
  check('墙 0→2.5m → OCCUPIED（穿过飞行带）',
    markForTriangle('vertical', 0, 2.5, cfg) === CELL_OCCUPIED);
  check('墙 0→0.2m → FREE（低于飞行带，可越过）',
    markForTriangle('vertical', 0, 0.2, cfg) === CELL_FREE);
  check('天花板 2.2→2.5m → SKIP（整体在飞行带之上）',
    markForTriangle('vertical', 2.2, 2.5, cfg) === MARK_SKIP);
  check('刚好贴带上沿 ymin=1.6 → OCCUPIED',
    markForTriangle('vertical', 1.6, 2.0, cfg) === CELL_OCCUPIED);
  check('带上沿之上一点点 ymin=1.7 → SKIP',
    markForTriangle('vertical', 1.7, 2.0, cfg) === MARK_SKIP);

  // —— 端到端：地板块 → 内部 FREE ——
  {
    // 两个三角形拼成 x∈[-2,2] z∈[-2,2] 的地板
    const quad: number[] = [
      -2, 0, -2, 2, 0, -2, 2, 0, 2,
      -2, 0, -2, 2, 0, 2, -2, 0, 2
    ];
    const snap: MeshSnapshot = {
      vertices: quad,
      indices: [0, 1, 2, 3, 4, 5],
      vertexCount: 6,
      triangleCount: 2
    };
    const g: OccupancyGrid = buildGridFromMesh(snap, 0, 0, cfg);
    check('地板块：中心格 free', g.isFree(g.colOf(0), g.rowOf(0)));
    check('地板块：有 free 格', g.stats().free > 0);
    check('地板块：地图角落仍 unknown', g.at(0, 0) === CELL_UNKNOWN);
  }

  // —— 端到端：竖墙（两个三角形）→ 沿边线 OCCUPIED 且连续 ——
  {
    // 墙位于 x=2 平面，y∈[0,2]，z∈[-2,2]
    const quad: number[] = [
      2, 0, -2, 2, 0, 2, 2, 2, 2,
      2, 0, -2, 2, 2, 2, 2, 2, -2
    ];
    const snap: MeshSnapshot = {
      vertices: quad,
      indices: [0, 1, 2, 3, 4, 5],
      vertexCount: 6,
      triangleCount: 2
    };
    const g: OccupancyGrid = buildGridFromMesh(snap, 0, 0, cfg);
    const col: number = g.colOf(2);
    let firstOcc: number = -1;
    let lastOcc: number = -1;
    for (let row = 0; row < g.height; row++) {
      if (g.at(col, row) === CELL_OCCUPIED) {
        if (firstOcc < 0) {
          firstOcc = row;
        }
        lastOcc = row;
      }
    }
    check('竖墙：有障碍格', firstOcc >= 0);
    let hole: boolean = false;
    for (let row = firstOcc; row <= lastOcc; row++) {
      if (g.at(col, row) !== CELL_OCCUPIED) {
        hole = true;
        break;
      }
    }
    check('竖墙：边线连续无孔', !hole);
    // 墙内部（x=2 两侧的格）不该被填满成一大片
    check('竖墙：未把墙加厚（相邻列保持 unknown）',
      g.at(col - 3, 10) === CELL_UNKNOWN);
  }

  // —— 端到端：地板 + 墙 同时存在时，OCCUPIED 优先于 FREE ——
  {
    const verts: number[] = [
      // 地板（y=0，x/z ∈ [-4,4]）
      -4, 0, -4, 4, 0, -4, 4, 0, 4,
      -4, 0, -4, 4, 0, 4, -4, 0, 4,
      // 穿过地板的墙（x=0，y∈[0,2]，z∈[-4,4]）
      0, 0, -4, 0, 0, 4, 0, 2, 4,
      0, 0, -4, 0, 2, 4, 0, 2, -4
    ];
    const snap: MeshSnapshot = {
      vertices: verts,
      indices: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11],
      vertexCount: 12,
      triangleCount: 4
    };
    const g: OccupancyGrid = buildGridFromMesh(snap, 0, 0, cfg);
    check('地板+墙：墙脚下是 OCCUPIED（障碍赢过地板）',
      g.at(g.colOf(0), g.rowOf(0)) === CELL_OCCUPIED);
    check('地板+墙：远处地板仍是 FREE',
      g.isFree(g.colOf(3), g.rowOf(3)));
    check('地板+墙：free/occ 都非 0', g.stats().free > 0 && g.stats().occupied > 0);
  }

  // —— 端到端：膨胀生效且不滚雪球 ——
  {
    const table: number[] = [
      -0.5, 0.8, -0.5, 0.5, 0.8, -0.5, 0.5, 0.8, 0.5,
      -0.5, 0.8, -0.5, 0.5, 0.8, 0.5, -0.5, 0.8, 0.5
    ];
    const snap: MeshSnapshot = {
      vertices: table,
      indices: [0, 1, 2, 3, 4, 5],
      vertexCount: 6,
      triangleCount: 2
    };
    const gNo: OccupancyGrid = buildGridFromMesh(snap, 0, 0, cfg);
    const cfgInf: GridConfig = {
      cellSize: 0.5, halfExtent: 5.0, floorMaxY: 0.3, obstacleMinY: 0.3,
      inflateRadius: 0.5, horizontalNy: 0.9, verticalNy: 0.3
    };
    const gInf: OccupancyGrid = buildGridFromMesh(snap, 0, 0, cfgInf);
    check('膨胀：障碍格变多', gInf.stats().occupied > gNo.stats().occupied);
    check('膨胀：未滚雪球（占用数 < 原 5 倍 + 8）',
      gInf.stats().occupied < gNo.stats().occupied * 5 + 8);
  }

  // —— 坏数据防御 ——
  {
    const empty: MeshSnapshot = {
      vertices: [], indices: [], vertexCount: 0, triangleCount: 0
    };
    check('空快照：不可用于规划', !meshUsable(empty));
    const g: OccupancyGrid = buildGridFromMesh(empty, 0, 0, cfg);
    check('空快照：全 unknown', g.stats().unknown === g.stats().total);

    const bad: MeshSnapshot = {
      vertices: [0, 0, 0], indices: [0, 1, 9], vertexCount: 1, triangleCount: 1
    };
    const g2: OccupancyGrid = buildGridFromMesh(bad, 0, 0, cfg);
    check('越界索引：不崩且全 unknown', g2.stats().unknown === g2.stats().total);

    const one: MeshSnapshot = triSnap([0, 0, 0, 1, 0, 0, 1, 0, 1]);
    check('单三角形：可用', meshUsable(one));
  }

  // —— 与平面版口径一致性（同一份 DEFAULT_GRID_CONFIG 下阈值必须一致）——
  {
    check('口径：平面版与网格版共用同一份默认配置',
      DEFAULT_GRID_CONFIG.floorMaxY === cfg.floorMaxY &&
      DEFAULT_GRID_CONFIG.obstacleMinY === cfg.obstacleMinY);
    const flatPlane: WorldPlane = { cx: 0, cy: 0, cz: 0, nx: 0, ny: 1, nz: 0, poly: [] };
    const wallPlane: WorldPlane = { cx: 0, cy: 0, cz: 0, nx: 1, ny: 0, nz: 0, poly: [] };
    check('口径：水平面 ny=1 两边同判为水平',
      isHorizontal(flatPlane, cfg) && classifyTriangle(1, cfg) === 'horizontal');
    check('口径：竖直面 ny=0 两边同判为垂直',
      isVertical(wallPlane, cfg) && classifyTriangle(0, cfg) === 'vertical');
  }

  if (fails.length !== 0) {
    throw new Error(`MeshGrid 自检失败：${fails.join('、')}`);
  }
  Logger.info(`[VERIFY] MeshGrid 场景网格栅格化自检 PASS（${total} 项断言全通过）`);
}
