/*
 * Copyright (c) 2024-2025 Huawei Device Co., Ltd.
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * PathPlanner —— 平面地图 → 2.5D 占用栅格 → A* 路径规划
 * =======================================================================
 * 阶段 1（栅格化）+ 阶段 2（搜索），**纯函数、零 AR 依赖**：
 * 输入是纯数字（平面几何），输出是纯数据（栅格 / 路点序列），
 * 因此它可以脱离设备直接离线仿真（scripts/sim_path_planner.js），
 * 与 CollisionDetector / SteeringBehavior 一样属于「物理层可单测」的那一半。
 *
 * ── 为什么是 2.5D 而不是 3D ─────────────────────────────────────────────
 * 本 App 的无人机按巡航高度（约 0.9m）水平飞行，垂直自由度很小。
 * 因此把环境压成一张「俯视占用地图」就够用：水平面贡献可行区域，
 * 垂直面贡献障碍边界。这样地图规模从「几十万顶点」降到「几千格」，
 * 完全避开了逐帧处理场景网格的性能问题。
 *
 * ⚠ 有一条必须如实说明的边界：2.5D 表达不了「悬空障碍」与
 *   「有腿的家具」（椅子）。椅子在俯视图里要么整块算障碍（偏保守，
 *   可能把可穿过的区域封掉），要么整块算空地（偏激进）。
 *   本实现选择**偏保守**：只要平面高度落在飞行带内就整块算障碍。
 *   要突破这个边界需要 3D 体素，那是阶段 4（mesh）的范畴。
 *
 * ── 数据来源与解耦（关键设计）───────────────────────────────────────────
 * 本模块**不认 AREngine**。平面数据由上层（ARPlaneTracker）从 AREngine
 * 提取成 `WorldPlane` 纯数据后传进来。这样带来两个好处：
 *   1) 可离线仿真 —— 不需要真机、不需要 AR 会话；
 *   2) 阶段 4 可热插拔 —— 将来用场景网格（mesh）体素化建图时，
 *      只要它最终也产出 `OccupancyGrid`，**A* 那一半一行都不用改**。
 *
 * ── 与既有避障层的关系 ──────────────────────────────────────────────────
 * 本模块是**规划**（先算好路径再飞），既有 applyAvoidance 是**反应式避障**
 * （每帧看当前帧深度图）。两者不是替代关系：
 *   规划负责「怎么绕」，反应式负责「没绕开时的最后一道保险」。
 * 因此接线时**必须保留** WaypointFlight 里那层 applyAvoidance。
 */

import { Logger } from '../core/Logger';
import { DRONE_RADIUS_M } from '../avoidance/CollisionDetector';

const TAG: string = 'PathPlanner';

// ---------------------------------------------------------------
//  栅格取值
// ---------------------------------------------------------------
/** 未知：没被任何平面覆盖到。**不可通行** —— 看不见的地方不该赌它可以飞 */
export const CELL_UNKNOWN: number = -1;
/** 空闲（可行区域） */
export const CELL_FREE: number = 0;
/** 占用（障碍） */
export const CELL_OCCUPIED: number = 1;

/**
 * 世界系下的一个平面（纯数据，由 ARPlaneTracker 从 AREngine 提取）。
 *
 * `poly` 是边界多边形顶点、世界系、每 3 个连续分量构成一个点 (x,y,z)：
 * AREngine 给的原始数据是「平面局部坐标系下的 (x,z) 二元组 + 位姿矩阵」，
 * 由提取方做矩阵变换后摊平成这个数组 —— 本模块只认世界系，不碰位姿。
 */
export interface WorldPlane {
  /** 平面中心（世界系 m） */
  cx: number;
  cy: number;
  cz: number;
  /** 平面法向（世界系，单位向量）。水平面 |ny|→1，垂直面 |ny|→0 */
  nx: number;
  ny: number;
  nz: number;
  /** 边界多边形顶点（世界系），每 3 个分量 = 一个点 */
  poly: number[];
}

/** 栅格化配置 */
export interface GridConfig {
  /** 每格边长（m）。0.1 是「机身半径的一半」量级，再细就只是拖慢搜索 */
  cellSize: number;
  /** 地图半边长（m）：以起点为中心，覆盖 [-halfExtent, +halfExtent] */
  halfExtent: number;
  /** 低于此高度的水平面 = 可行地板（m）。无人机在它上方飞，不算障碍 */
  floorMaxY: number;
  /** 高于此高度的水平面 = 障碍（m，桌面/床面）—— 飞机会撞上去 */
  obstacleMinY: number;
  /** 障碍膨胀半径（m）—— 按机身半径膨胀，否则规划出的路径会贴着墙飞 */
  inflateRadius: number;
  /** 水平面判定阈值：|ny| ≥ 此值视为水平 */
  horizontalNy: number;
  /** 垂直面判定阈值：|ny| ≤ 此值视为垂直（墙） */
  verticalNy: number;
}

/**
 * 飞行带上沿（m）—— 判断「某个三维面是否算障碍」时的上方依据。
 *
 * 语义：无人机巡航高度在 [floorMaxY, FLIGHT_BAND_TOP_Y] 这一带内，
 * 因此**只要几何与这条带有交集，就必须算障碍**；高于带上沿的面（如天花板）
 * 不与飞行带冲突，不该因为它把地图判死。
 *
 * 为什么单独抽成常量而不是写死在各处：平面栅格化（本文件）与网格栅格化
 * （MeshGrid）都要用它，两处口径必须一致，否则「平面图能过、网格图不能过」
 * 这种不一致极难排查。
 */
export const FLIGHT_BAND_TOP_Y: number = 1.6;

/** 默认栅格配置 */
export const DEFAULT_GRID_CONFIG: GridConfig = {
  cellSize: 0.1,
  halfExtent: 8.0,
  floorMaxY: 0.3,
  obstacleMinY: 0.3,
  inflateRadius: DRONE_RADIUS_M,
  horizontalNy: 0.9,
  verticalNy: 0.3
};

/**
 * 占用栅格。
 *
 * 用 class 而不是 interface：`cells` 是 TypedArray，
 * ArkTS 的 `arkts-no-untyped-obj-literals` 对这类字段的对象字面量匹配较严，
 * 走构造器最稳（与 AiFrameSource.DepthTensorResult 同一个坑同一个解法）。
 */
export class OccupancyGrid {
  /** 格 (0,0) **中心**的世界 X（m） */
  public originX: number;
  /** 格 (0,0) 中心的世界 Z（m） */
  public originZ: number;
  public cellSize: number;
  public width: number;
  public height: number;
  /** 栅格值，取值见 CELL_* */
  public cells: Int8Array;

  constructor(originX: number, originZ: number, cellSize: number,
    width: number, height: number, cells: Int8Array) {
    this.originX = originX;
    this.originZ = originZ;
    this.cellSize = cellSize;
    this.width = width;
    this.height = height;
    this.cells = cells;
  }

  /** 世界 X → 列号（可能越界，调用方需自行判断） */
  public colOf(worldX: number): number {
    return Math.round((worldX - this.originX) / this.cellSize);
  }

  /** 世界 Z → 行号 */
  public rowOf(worldZ: number): number {
    return Math.round((worldZ - this.originZ) / this.cellSize);
  }

  /** 列号 → 世界 X（格中心） */
  public xOf(col: number): number {
    return this.originX + col * this.cellSize;
  }

  /** 行号 → 世界 Z（格中心） */
  public zOf(row: number): number {
    return this.originZ + row * this.cellSize;
  }

  public inBounds(col: number, row: number): boolean {
    return col >= 0 && row >= 0 && col < this.width && row < this.height;
  }

  /** 取格值；越界返回 CELL_OCCUPIED（把地图外当作障碍，防止规划出界） */
  public at(col: number, row: number): number {
    if (!this.inBounds(col, row)) {
      return CELL_OCCUPIED;
    }
    return this.cells[row * this.width + col];
  }

  /** 该格是否可通行（仅 CELL_FREE 可走） */
  public isFree(col: number, row: number): boolean {
    return this.at(col, row) === CELL_FREE;
  }

  /** 统计各类格数（诊断用） */
  public stats(): GridStats {
    let free: number = 0;
    let occ: number = 0;
    let unknown: number = 0;
    for (let i = 0; i < this.cells.length; i++) {
      const v: number = this.cells[i];
      if (v === CELL_FREE) {
        free++;
      } else if (v === CELL_OCCUPIED) {
        occ++;
      } else {
        unknown++;
      }
    }
    const st: GridStats = { free: free, occupied: occ, unknown: unknown, total: this.cells.length };
    return st;
  }
}

/** 栅格统计 */
export interface GridStats {
  free: number;
  occupied: number;
  unknown: number;
  total: number;
}

/** 路点（offset 坐标系，与 WaypointFlight.Waypoint 的 x/z 同口径） */
export interface PathPoint {
  x: number;
  z: number;
}

/** 规划结果 */
export interface PlanResult {
  /** 路点序列（含起点与终点）；失败时为空数组 */
  points: PathPoint[];
  /** 是否成功 */
  ok: boolean;
  /** 展开的节点数（诊断：反映搜索代价） */
  expanded: number;
  /** 失败原因（成功时为空串） */
  reason: string;
}

// ---------------------------------------------------------------
//  阶段 1：栅格化
// ---------------------------------------------------------------

/** 判断一个平面是否「水平」（地板/桌面） */
export function isHorizontal(plane: WorldPlane, cfg: GridConfig): boolean {
  return Math.abs(plane.ny) >= cfg.horizontalNy;
}

/** 判断一个平面是否「垂直」（墙壁） */
export function isVertical(plane: WorldPlane, cfg: GridConfig): boolean {
  return Math.abs(plane.ny) <= cfg.verticalNy;
}

/**
 * 点 (px,pz) 是否在平面多边形的 XZ 投影内（射线法）。
 *
 * 只看 XZ：水平面的多边形本来就躺在水平面上，投影即其真实形状；
 * 垂直面用不到本函数（走线段栅格化）。
 */
export function pointInPolygonXZ(poly: number[], px: number, pz: number): boolean {
  const n: number = Math.floor(poly.length / 3);
  if (n < 3) {
    return false;
  }
  let inside: boolean = false;
  let j: number = n - 1;
  for (let i = 0; i < n; i++) {
    const xi: number = poly[i * 3];
    const zi: number = poly[i * 3 + 2];
    const xj: number = poly[j * 3];
    const zj: number = poly[j * 3 + 2];
    // 射线沿 +X 方向：判断边 (j→i) 是否跨越该行且交点在右侧
    const crosses: boolean = ((zi > pz) !== (zj > pz));
    if (crosses) {
      const t: number = (pz - zi) / (zj - zi);
      const xHit: number = xi + t * (xj - xi);
      if (px < xHit) {
        inside = !inside;
      }
    }
    j = i;
  }
  return inside;
}

/**
 * 把世界系平面集合栅格化为占用地图。
 *
 * 规则（每条都对应一种真机形态）：
 *   1) 水平面、高度 < floorMaxY            → 投影内标 **FREE**（这是地板）
 *   2) 水平面、高度 ≥ obstacleMinY         → 投影内标 **OCCUPIED**（桌面/床，
 *      无人机在那个高度会撞；偏保守，见文件头说明）
 *   3) 垂直面                              → 沿多边形边线在 XZ 上栅格化成线，
 *      标 **OCCUPIED**（墙）
 *   4) 其余（斜面）                        → 投影内标 OCCUPIED（保守）
 *   5) 最后按 inflateRadius **膨胀**所有障碍格
 *
 * 起点 (startX, startZ) 决定地图中心 —— 地图不需要覆盖全世界，
 * 覆盖「飞机附近能飞到的地方」就够了，这也把栅格规模压到可控范围。
 */
export function buildOccupancyGrid(planes: WorldPlane[], startX: number, startZ: number,
  cfg: GridConfig = DEFAULT_GRID_CONFIG): OccupancyGrid {
  const cell: number = cfg.cellSize > 1e-3 ? cfg.cellSize : 0.1;
  const half: number = cfg.halfExtent > cell ? cfg.halfExtent : cell * 8;
  const width: number = Math.max(1, Math.ceil((half * 2) / cell));
  const height: number = width;

  // 地图以起点为中心：格 (0,0) 中心落在「起点 - 半边长」处，
  // 这样起点恒落在栅格正中，避免它被地图边界裁掉。
  const originX: number = startX - half;
  const originZ: number = startZ - half;
  const cells: Int8Array = new Int8Array(width * height);
  cells.fill(CELL_UNKNOWN);

  const grid: OccupancyGrid = new OccupancyGrid(originX, originZ, cell, width, height, cells);
  const halfDiag: number = Math.sqrt(2) * half + cell;
  let horizontalCount: number = 0;
  let verticalCount: number = 0;
  let otherCount: number = 0;

  for (let p = 0; p < planes.length; p++) {
    const pl: WorldPlane = planes[p];
    const n: number = Math.floor(pl.poly.length / 3);
    if (n < 2) {
      continue;
    }

    // 与地图中心相距过远的平面直接跳过（省掉无谓的逐格测试）
    const dcx: number = pl.cx - startX;
    const dcz: number = pl.cz - startZ;
    if (Math.sqrt(dcx * dcx + dcz * dcz) > halfDiag) {
      continue;
    }

    if (isHorizontal(pl, cfg)) {
      horizontalCount++;
      // 高度决定这块水平面是「地板」还是「桌子」：
      //   低于 floorMaxY    → 地板，可行
      //   达到 obstacleMinY → 障碍（桌面/床面，飞机会撞上去）
      //   两者之间（本工程两阈值相等，区间为空）→ 按障碍处理，保守优先
      const useMark: number = (pl.cy >= cfg.obstacleMinY) ? CELL_OCCUPIED
        : (pl.cy < cfg.floorMaxY ? CELL_FREE : CELL_OCCUPIED);
      rasterizePolygonFill(grid, pl.poly, n, useMark);
    } else if (isVertical(pl, cfg)) {
      verticalCount++;
      // 墙：只把边线打成障碍。多边形内部不填 —— 墙的 XZ 投影是一条线，
      // 填内部会把墙「加厚」成一大片，把可行区域吃掉。
      for (let i = 0; i < n; i++) {
        const j: number = (i + 1) % n;
        rasterizeSegment(grid,
          pl.poly[i * 3], pl.poly[i * 3 + 2],
          pl.poly[j * 3], pl.poly[j * 3 + 2],
          CELL_OCCUPIED);
      }
    } else {
      otherCount++;
      // 斜面：保守处理成障碍（既不确认是地板也不确认是墙）
      rasterizePolygonFill(grid, pl.poly, n, CELL_OCCUPIED);
    }
  }

  // 障碍膨胀：把每个障碍格按其半径向外扩，等价于「把机身当成一个点」。
  // 不做这一步，A* 会给出贴着墙的路径，飞起来就是一路擦墙。
  if (cfg.inflateRadius > 0) {
    inflate(grid, cfg.inflateRadius);
  }

  const st: GridStats = grid.stats();
  Logger.info(`${TAG} [VERIFY] 栅格化：平面 ${planes.length} 个` +
    `（水平 ${horizontalCount} / 垂直 ${verticalCount} / 斜面 ${otherCount}），` +
    `${width}x${height} 格 @${cell}m，free=${st.free} occ=${st.occupied} unknown=${st.unknown}`);
  return grid;
}

/** 在栅格上填充一个多边形的投影（点在内 → 标 mark） */
export function rasterizePolygonFill(grid: OccupancyGrid, poly: number[], n: number, mark: number): void {
  // 计算多边形的 XZ 包围盒，只扫盒内格子（避免全图扫描）
  let minX: number = poly[0];
  let maxX: number = poly[0];
  let minZ: number = poly[2];
  let maxZ: number = poly[2];
  for (let i = 1; i < n; i++) {
    const x: number = poly[i * 3];
    const z: number = poly[i * 3 + 2];
    if (x < minX) {
      minX = x;
    }
    if (x > maxX) {
      maxX = x;
    }
    if (z < minZ) {
      minZ = z;
    }
    if (z > maxZ) {
      maxZ = z;
    }
  }
  let c0: number = grid.colOf(minX);
  let c1: number = grid.colOf(maxX);
  let r0: number = grid.rowOf(minZ);
  let r1: number = grid.rowOf(maxZ);
  if (c0 > c1) {
    const t: number = c0;
    c0 = c1;
    c1 = t;
  }
  if (r0 > r1) {
    const t: number = r0;
    r0 = r1;
    r1 = t;
  }
  if (c0 < 0) {
    c0 = 0;
  }
  if (r0 < 0) {
    r0 = 0;
  }
  if (c1 > grid.width - 1) {
    c1 = grid.width - 1;
  }
  if (r1 > grid.height - 1) {
    r1 = grid.height - 1;
  }
  for (let row = r0; row <= r1; row++) {
    for (let col = c0; col <= c1; col++) {
      if (pointInPolygonXZ(poly, grid.xOf(col), grid.zOf(row))) {
        grid.cells[row * grid.width + col] = mark;
      }
    }
  }
}

/**
 * 把 XZ 平面上的一条线段栅格化（Bresenham 整数算法）。
 *
 * 用于垂直面（墙）的边线。选 Bresenham 而不是「按步长采样」：
 * 采样步长取小了浪费、取大了会漏格，Bresenham 保证**连续无孔**，
 * 而墙上有孔就等于规划器能穿墙 —— 这是安全相关的正确性要求。
 */
export function rasterizeSegment(grid: OccupancyGrid, x0: number, z0: number,
  x1: number, z1: number, mark: number): void {
  let c0: number = grid.colOf(x0);
  let r0: number = grid.rowOf(z0);
  const c1: number = grid.colOf(x1);
  const r1: number = grid.rowOf(z1);
  const dc: number = Math.abs(c1 - c0);
  const dr: number = Math.abs(r1 - r0);
  const sc: number = c0 < c1 ? 1 : -1;
  const sr: number = r0 < r1 ? 1 : -1;
  let err: number = dc - dr;
  // guard：防止极端坐标造成死循环（理论上步数 = dc+dr+1）
  let guard: number = dc + dr + 2;
  while (guard > 0) {
    guard--;
    if (grid.inBounds(c0, r0)) {
      grid.cells[r0 * grid.width + c0] = mark;
    }
    if (c0 === c1 && r0 === r1) {
      break;
    }
    const e2: number = 2 * err;
    if (e2 > -dr) {
      err -= dr;
      c0 += sc;
    }
    if (e2 < dc) {
      err += dc;
      r0 += sr;
    }
  }
}

/**
 * 障碍膨胀（圆形结构元）。
 *
 * 实现是「先收集障碍坐标、再统一写回」而不是边扫边写：
 * 边扫边写会把**本轮的膨胀结果**当成下一格的原障碍，导致膨胀半径
 * 逐格滚雪球（连锁膨胀），最终吃掉整张地图。这是形态学里最常见的一个坑。
 */
export function inflate(grid: OccupancyGrid, radiusM: number): void {
  const rad: number = Math.max(1, Math.ceil(radiusM / grid.cellSize));
  const rad2: number = rad * rad;
  const src: Int8Array = new Int8Array(grid.cells);
  for (let row = 0; row < grid.height; row++) {
    for (let col = 0; col < grid.width; col++) {
      if (src[row * grid.width + col] !== CELL_OCCUPIED) {
        continue;
      }
      for (let dr = -rad; dr <= rad; dr++) {
        for (let dc = -rad; dc <= rad; dc++) {
          if (dc * dc + dr * dr > rad2) {
            continue;   // 圆形结构元（而非方形），避免对角方向多出一截
          }
          const nc: number = col + dc;
          const nr: number = row + dr;
          if (grid.inBounds(nc, nr)) {
            grid.cells[nr * grid.width + nc] = CELL_OCCUPIED;
          }
        }
      }
    }
  }
}

// ---------------------------------------------------------------
//  阶段 2：A* 搜索
// ---------------------------------------------------------------

/** 规划选项 */
export interface PlanOptions {
  /** 最多展开多少节点（防御：地图异常时不让搜索卡死 UI） */
  maxExpanded: number;
  /** 是否做可见性平滑（去掉栅格锯齿） */
  smooth: boolean;
}

/** 默认规划选项 */
export const DEFAULT_PLAN_OPTIONS: PlanOptions = {
  maxExpanded: 40000,
  smooth: true
};

/** 二叉堆节点（优先队列，按 f 最小出队） */
class HeapNode {
  public idx: number;
  public f: number;

  constructor(idx: number, f: number) {
    this.idx = idx;
    this.f = f;
  }
}

/**
 * 简易二叉小顶堆。
 *
 * 为什么不用「每轮线性扫描 openList 取最小」：地图可达 160x160=25600 格，
 * 线性扫描会让 A* 退化成 O(n²) 量级；装载航线时一次性调用也要几百 ms。
 * 堆实现只有 40 行，代价可控。
 */
class MinHeap {
  private items: HeapNode[] = [];

  public get size(): number {
    return this.items.length;
  }

  public push(node: HeapNode): void {
    this.items.push(node);
    let i: number = this.items.length - 1;
    while (i > 0) {
      const parent: number = Math.floor((i - 1) / 2);
      if (this.items[parent].f <= this.items[i].f) {
        break;
      }
      const t: HeapNode = this.items[parent];
      this.items[parent] = this.items[i];
      this.items[i] = t;
      i = parent;
    }
  }

  public pop(): HeapNode | null {
    if (this.items.length === 0) {
      return null;
    }
    const top: HeapNode = this.items[0];
    const last: HeapNode = this.items[this.items.length - 1];
    this.items.pop();
    if (this.items.length > 0) {
      this.items[0] = last;
      let i: number = 0;
      while (true) {
        const l: number = i * 2 + 1;
        const r: number = i * 2 + 2;
        let smallest: number = i;
        if (l < this.items.length && this.items[l].f < this.items[smallest].f) {
          smallest = l;
        }
        if (r < this.items.length && this.items[r].f < this.items[smallest].f) {
          smallest = r;
        }
        if (smallest === i) {
          break;
        }
        const t: HeapNode = this.items[smallest];
        this.items[smallest] = this.items[i];
        this.items[i] = t;
        i = smallest;
      }
    }
    return top;
  }
}

/** 8 邻域方向（前 4 个是直线，后 4 个是对角） */
const DIRS_COL: number[] = [1, -1, 0, 0, 1, 1, -1, -1];
const DIRS_ROW: number[] = [0, 0, 1, -1, 1, -1, 1, -1];
const SQRT2: number = Math.sqrt(2);

/**
 * 在栅格上做 A* 搜索。
 *
 * 代价：直线 1、对角 √2（与欧氏启发式一致，保证最优且不会高估）。
 * **对角移动要求两个相邻正交格都可行**，否则会「斜着穿过墙角的缝」——
 * 栅格地图里最典型的漏检。
 *
 * 起终点若落在障碍/未知格上，会**就近吸附**到最近可行格（有上限），
 * 而不是直接失败：用户在 AR 里点的航点很可能刚好压在桌子边缘那一格上。
 *
 * @returns 世界系路点序列（含起终点）；失败时 points 为空
 */
export function planPath(grid: OccupancyGrid, startX: number, startZ: number,
  goalX: number, goalZ: number, opts: PlanOptions = DEFAULT_PLAN_OPTIONS): PlanResult {
  const n: number = grid.width * grid.height;
  const gScore: Float64Array = new Float64Array(n);
  gScore.fill(Number.POSITIVE_INFINITY);
  const cameFrom: Int32Array = new Int32Array(n);
  cameFrom.fill(-1);
  const closed: Uint8Array = new Uint8Array(n);

  const sc: number = grid.colOf(startX);
  const sr: number = grid.rowOf(startZ);
  const gc: number = grid.colOf(goalX);
  const gr: number = grid.rowOf(goalZ);

  const startIdx: number = snapToFree(grid, sc, sr);
  const goalIdx: number = snapToFree(grid, gc, gr);
  if (startIdx < 0 || goalIdx < 0) {
    const fail: PlanResult = {
      points: [], ok: false, expanded: 0,
      reason: startIdx < 0 ? '起点附近无可行格' : '终点附近无可行格'
    };
    Logger.warn(`${TAG} 规划失败：${fail.reason}`);
    return fail;
  }

  const goalCol: number = goalIdx % grid.width;
  const goalRow: number = Math.floor(goalIdx / grid.width);

  const heap: MinHeap = new MinHeap();
  gScore[startIdx] = 0;
  const h0: number = heuristic(startIdx % grid.width, Math.floor(startIdx / grid.width), goalCol, goalRow);
  heap.push(new HeapNode(startIdx, h0));

  let expanded: number = 0;
  let found: boolean = false;

  while (heap.size > 0) {
    const cur: HeapNode | null = heap.pop();
    if (cur === null) {
      break;
    }
    const idx: number = cur.idx;
    if (closed[idx] === 1) {
      continue;   // 堆里可能有同一节点的旧副本（无 decrease-key 的标准做法）
    }
    closed[idx] = 1;
    expanded++;

    if (idx === goalIdx) {
      found = true;
      break;
    }
    if (expanded >= opts.maxExpanded) {
      const fail: PlanResult = {
        points: [], ok: false, expanded: expanded,
        reason: `展开节点超上限(${opts.maxExpanded})`
      };
      Logger.warn(`${TAG} 规划失败：${fail.reason}`);
      return fail;
    }

    const col: number = idx % grid.width;
    const row: number = Math.floor(idx / grid.width);
    for (let d = 0; d < 8; d++) {
      const nc: number = col + DIRS_COL[d];
      const nr: number = row + DIRS_ROW[d];
      if (!grid.inBounds(nc, nr) || !grid.isFree(nc, nr)) {
        continue;
      }
      const diagonal: boolean = d >= 4;
      if (diagonal) {
        // 禁止穿角：对角两侧的正交格必须都可通行
        if (!grid.isFree(col + DIRS_COL[d], row) || !grid.isFree(col, row + DIRS_ROW[d])) {
          continue;
        }
      }
      const nIdx: number = nr * grid.width + nc;
      if (closed[nIdx] === 1) {
        continue;
      }
      const step: number = diagonal ? SQRT2 : 1;
      const tentative: number = gScore[idx] + step;
      if (tentative < gScore[nIdx]) {
        gScore[nIdx] = tentative;
        cameFrom[nIdx] = idx;
        heap.push(new HeapNode(nIdx, tentative + heuristic(nc, nr, goalCol, goalRow)));
      }
    }
  }

  if (!found) {
    const fail: PlanResult = { points: [], ok: false, expanded: expanded, reason: '无可行路径' };
    Logger.warn(`${TAG} 规划失败：${fail.reason}（展开 ${expanded} 节点）`);
    return fail;
  }

  // —— 回溯并转成世界坐标 ——
  const rev: PathPoint[] = [];
  let walk: number = goalIdx;
  let guard: number = n + 1;
  while (walk >= 0 && guard > 0) {
    guard--;
    const pt: PathPoint = {
      x: grid.xOf(walk % grid.width),
      z: grid.zOf(Math.floor(walk / grid.width))
    };
    rev.push(pt);
    if (walk === startIdx) {
      break;
    }
    walk = cameFrom[walk];
  }
  rev.reverse();
  // 首尾对齐到用户真正给的点（吸附会带来最多半格的偏移，末端要精确）
  if (rev.length > 0) {
    rev[0] = { x: startX, z: startZ };
    rev[rev.length - 1] = { x: goalX, z: goalZ };
  }

  const pts: PathPoint[] = opts.smooth ? smoothPath(grid, rev) : rev;
  const ok: PlanResult = { points: pts, ok: true, expanded: expanded, reason: '' };
  Logger.info(`${TAG} [VERIFY] 规划成功：${rev.length} 格 → 平滑后 ${pts.length} 点，` +
    `展开 ${expanded} 节点`);
  return ok;
}

/** 欧氏距离启发式（对 8 邻域 + √2 对角代价是可采纳的，不会高估） */
function heuristic(c0: number, r0: number, c1: number, r1: number): number {
  const dc: number = Math.abs(c0 - c1);
  const dr: number = Math.abs(r0 - r1);
  // 对角优先的八向距离：比纯欧氏更贴合真实代价，扩展的节点更少
  const diag: number = Math.min(dc, dr);
  const straight: number = Math.max(dc, dr) - diag;
  return SQRT2 * diag + straight;
}

/** 就近吸附到最近可行格；找不到（或超出搜索半径）返回 -1 */
function snapToFree(grid: OccupancyGrid, col: number, row: number): number {
  if (grid.isFree(col, row)) {
    return row * grid.width + col;
  }
  const MAX_R: number = 6;
  for (let r = 1; r <= MAX_R; r++) {
    for (let dr = -r; dr <= r; dr++) {
      for (let dc = -r; dc <= r; dc++) {
        // 只测「环」上的格，避免重复检查内部（内部上一轮已查过）
        if (Math.abs(dr) !== r && Math.abs(dc) !== r) {
          continue;
        }
        const nc: number = col + dc;
        const nr: number = row + dr;
        if (grid.isFree(nc, nr)) {
          return nr * grid.width + nc;
        }
      }
    }
  }
  return -1;
}

/**
 * 可见性平滑：若「当前保留点 → 再下一点」的直线不穿障碍，
 * 就把中间点丢掉。
 *
 * 为什么必要：A* 在 8 邻域栅格上产出的是锯齿形折线（一斜一正交替），
 * 直接交给位置环会让飞机走出「一顿一顿」的抖动轨迹。
 * 平滑后既短又平滑，且**逐个候选点都做穿障检测**，不会为了平滑而穿墙。
 */
export function smoothPath(grid: OccupancyGrid, pts: PathPoint[]): PathPoint[] {
  if (pts.length <= 2) {
    return pts;
  }
  const out: PathPoint[] = [];
  let anchor: number = 0;
  out.push(pts[0]);
  while (anchor < pts.length - 1) {
    let next: number = anchor + 1;
    // 尽量往后找最远的可见点
    for (let k = pts.length - 1; k > anchor + 1; k--) {
      if (lineOfSight(grid, pts[anchor], pts[k])) {
        next = k;
        break;
      }
    }
    out.push(pts[next]);
    anchor = next;
  }
  return out;
}

/**
 * 两点之间是否直线可见（沿线逐步采样 + 每格判可行）。
 *
 * 采样步长取半格：比一格细，保证不会「跨格漏检」；
 * 又不会细到失去意义（0.1m 格 → 5cm 采样）。
 */
export function lineOfSight(grid: OccupancyGrid, a: PathPoint, b: PathPoint): boolean {
  const dx: number = b.x - a.x;
  const dz: number = b.z - a.z;
  const dist: number = Math.sqrt(dx * dx + dz * dz);
  const steps: number = Math.max(1, Math.ceil(dist / (grid.cellSize * 0.5)));
  for (let i = 0; i <= steps; i++) {
    const t: number = i / steps;
    const px: number = a.x + dx * t;
    const pz: number = a.z + dz * t;
    if (!grid.isFree(grid.colOf(px), grid.rowOf(pz))) {
      return false;
    }
  }
  return true;
}

// ---------------------------------------------------------------
//  自检（纯函数，可离线 / 可真机 grep）
// ---------------------------------------------------------------
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

  // —— 几何工具 ——
  check('点含于正方形', pointInPolygonXZ([0, 0, 0, 4, 0, 0, 4, 0, 4, 0, 0, 4], 2, 2) === true);
  check('点在正方形外', pointInPolygonXZ([0, 0, 0, 4, 0, 0, 4, 0, 4, 0, 0, 4], 5, 2) === false);
  check('顶点不足 → false', pointInPolygonXZ([0, 0, 0, 1, 0, 0], 0, 0) === false);

  // —— ① 空平面 → 全 unknown ——
  {
    const g: OccupancyGrid = buildOccupancyGrid([], 0, 0, cfg);
    check('空平面：全 unknown', g.stats().unknown === g.stats().total);
    check('空平面：无 free', g.stats().free === 0);
  }

  // —— ② 地平面 → 内部 free，外部仍 unknown ——
  {
    const floor: WorldPlane = {
      cx: 0, cy: 0, cz: 0, nx: 0, ny: 1, nz: 0,
      poly: [-2, 0, -2, 2, 0, -2, 2, 0, 2, -2, 0, 2]
    };
    const g: OccupancyGrid = buildOccupancyGrid([floor], 0, 0, cfg);
    check('地板：中心格 free', g.isFree(g.colOf(0), g.rowOf(0)));
    check('地板：有 free 格', g.stats().free > 0);
    check('地板：地图角落仍 unknown', g.at(0, 0) === CELL_UNKNOWN);
  }

  // —— ③ 桌面（高于飞行带）→ occupied ——
  {
    const table: WorldPlane = {
      cx: 0, cy: 0.8, cz: 0, nx: 0, ny: 1, nz: 0,
      poly: [-1, 0.8, -1, 1, 0.8, -1, 1, 0.8, 1, -1, 0.8, 1]
    };
    const g: OccupancyGrid = buildOccupancyGrid([table], 0, 0, cfg);
    check('高桌面：中心格 occupied', g.at(g.colOf(0), g.rowOf(0)) === CELL_OCCUPIED);
  }

  // —— ④ 墙（垂直面）→ 沿边线 occupied，且线连续无孔 ——
  {
    const wall: WorldPlane = {
      cx: 2, cy: 1, cz: 0, nx: -1, ny: 0, nz: 0,
      poly: [2, 0, -2, 2, 0, 2, 2, 2, 2, 2, 2, -2]
    };
    const g: OccupancyGrid = buildOccupancyGrid([wall], 0, 0, cfg);
    let hit: number = 0;
    for (let row = 0; row < g.height; row++) {
      if (g.at(g.colOf(2), row) === CELL_OCCUPIED) {
        hit++;
      }
    }
    check('墙：目标行被标为障碍', hit > 0);
    // 连续性：先取出「被占用行」的首尾，再检查两者之间没有空洞。
    // ⚠️ 不能简单地「遇到空白行就记空洞」—— 墙结束之后到地图边缘之间的
    //    那些空白行同样满足「之前见过占用、当前不占用」，会被误判成墙上有洞。
    //    （本断言 2026-09-24 真机首跑就是栽在这里：实现是对的，测试是错的。）
    let firstOcc: number = -1;
    let lastOcc: number = -1;
    for (let row = 0; row < g.height; row++) {
      if (g.at(g.colOf(2), row) === CELL_OCCUPIED) {
        if (firstOcc < 0) {
          firstOcc = row;
        }
        lastOcc = row;
      }
    }
    let hole: boolean = false;
    for (let row = firstOcc; row <= lastOcc; row++) {
      if (g.at(g.colOf(2), row) !== CELL_OCCUPIED) {
        hole = true;
        break;
      }
    }
    check('墙：边线连续（Bresenham 无孔）', firstOcc >= 0 && !hole);
  }

  // —— ⑤ 膨胀：障碍影响范围变大，且不是滚雪球 ——
  {
    const table: WorldPlane = {
      cx: 0, cy: 0.8, cz: 0, nx: 0, ny: 1, nz: 0,
      poly: [-0.5, 0.8, -0.5, 0.5, 0.8, -0.5, 0.5, 0.8, 0.5, -0.5, 0.8, 0.5]
    };
    const gNo: OccupancyGrid = buildOccupancyGrid([table], 0, 0, cfg);
    const cfgInf: GridConfig = {
      cellSize: 0.5, halfExtent: 5.0, floorMaxY: 0.3, obstacleMinY: 0.3,
      inflateRadius: 0.5, horizontalNy: 0.9, verticalNy: 0.3
    };
    const gInf: OccupancyGrid = buildOccupancyGrid([table], 0, 0, cfgInf);
    check('膨胀后障碍格变多', gInf.stats().occupied > gNo.stats().occupied);
    // 滚雪球检测：膨胀 0.5m（=1 格）后，占地应为「原格 + 一圈」，
    // 若实现错误连锁膨胀，占用数会暴涨到接近原图的数倍
    check('膨胀未滚雪球（占用数 < 原 5 倍）',
      gInf.stats().occupied < gNo.stats().occupied * 5 + 8);
  }

  // —— ⑥ A*：无障碍 → 直达 ——
  {
    const floor: WorldPlane = {
      cx: 0, cy: 0, cz: 0, nx: 0, ny: 1, nz: 0,
      poly: [-4, 0, -4, 4, 0, -4, 4, 0, 4, -4, 0, 4]
    };
    const g: OccupancyGrid = buildOccupancyGrid([floor], 0, 0, cfg);
    const r: PlanResult = planPath(g, -2, 0, 2, 0, DEFAULT_PLAN_OPTIONS);
    check('无障碍：规划成功', r.ok);
    check('无障碍：平滑后仅首尾两点', r.points.length === 2);
    check('无障碍：终点精确', Math.abs(r.points[r.points.length - 1].x - 2) < 1e-6);
  }

  // —— ⑦ A*：中间有墙 → 绕行，且路径全程不穿障碍 ——
  {
    const floor: WorldPlane = {
      cx: 0, cy: 0, cz: 0, nx: 0, ny: 1, nz: 0,
      poly: [-4, 0, -4, 4, 0, -4, 4, 0, 4, -4, 0, 4]
    };
    // 墙的 XZ 投影退化成一条线段（x=0，z∈[-1,1]），正好横在 (-2,0)→(2,0) 之间。
    // 刻意用退化多边形而不是有面积的矩形：有面积时四条边都会被栅格化，
    // 测出来的是「一个框」而不是「一道墙」，测试意图就不清楚了。
    const wall: WorldPlane = {
      cx: 0, cy: 1, cz: 0, nx: 0, ny: 0, nz: 1,
      poly: [0, 0, -1, 0, 0, 1, 0, 2, 1, 0, 2, -1]
    };
    const g: OccupancyGrid = buildOccupancyGrid([floor, wall], 0, 0, cfg);
    const r: PlanResult = planPath(g, -2, 0, 2, 0, DEFAULT_PLAN_OPTIONS);
    check('有墙：规划成功（绕过墙端）', r.ok);
    check('有墙：路径被拉长（确实绕了）', r.points.length > 2);
    let allFree: boolean = true;
    for (let i = 0; i < r.points.length; i++) {
      if (!g.isFree(g.colOf(r.points[i].x), g.rowOf(r.points[i].z))) {
        allFree = false;
      }
    }
    check('有墙：路径全程在可行格上', allFree);
  }

  // —— ⑧ A*：终点被障碍完全包围 → 失败但不崩 ——
  {
    const floor: WorldPlane = {
      cx: 0, cy: 0, cz: 0, nx: 0, ny: 1, nz: 0,
      poly: [-4, 0, -4, 4, 0, -4, 4, 0, 4, -4, 0, 4]
    };
    // 一个「口」字型高墙，把中心围死（每条边都是垂直面）
    const mk = (cx: number, cz: number, poly: number[]): WorldPlane => {
      const p: WorldPlane = { cx: cx, cy: 1, cz: cz, nx: 0, ny: 0, nz: 1, poly: poly };
      return p;
    };
    const ring: WorldPlane[] = [
      mk(2, 0, [2, 0, -2, 2, 0, 2, 2, 2, 2, 2, 2, -2]),
      mk(-2, 0, [-2, 0, -2, -2, 0, 2, -2, 2, 2, -2, 2, -2]),
      mk(0, -2, [-2, 0, -2, 2, 0, -2, 2, 2, -2, -2, 2, -2]),
      mk(0, 2, [-2, 0, 2, 2, 0, 2, 2, 2, 2, -2, 2, 2])
    ];
    // 地板必须一起给：只给围墙的话，框内框外全是 UNKNOWN，
    // 失败原因会退化成「起点附近无可行格」，就测不到「无可行路径」这条分支了。
    const ringFloor: WorldPlane = {
      cx: 0, cy: 0, cz: 0, nx: 0, ny: 1, nz: 0,
      poly: [-5, 0, -5, 5, 0, -5, 5, 0, 5, -5, 0, 5]
    };
    const ringAll: WorldPlane[] = [ringFloor, ring[0], ring[1], ring[2], ring[3]];
    const g: OccupancyGrid = buildOccupancyGrid(ringAll, 0, 0, cfg);
    const r: PlanResult = planPath(g, 0, 0, 3.5, 3.5, DEFAULT_PLAN_OPTIONS);
    check('被围死：规划失败但不抛异常', !r.ok);
    check('被围死：返回空路点', r.points.length === 0);
  }

  // —— ⑨ 直线可见性 ——
  {
    const floor: WorldPlane = {
      cx: 0, cy: 0, cz: 0, nx: 0, ny: 1, nz: 0,
      poly: [-4, 0, -4, 4, 0, -4, 4, 0, 4, -4, 0, 4]
    };
    const wall: WorldPlane = {
      cx: 0, cy: 1, cz: 0, nx: 0, ny: 0, nz: 1,
      poly: [0, 0, -4, 0, 0, 4, 0, 2, 4, 0, 2, -4]
    };
    const g: OccupancyGrid = buildOccupancyGrid([floor, wall], 0, 0, cfg);
    const a: PathPoint = { x: -2, z: 0 };
    const b: PathPoint = { x: 2, z: 0 };
    check('被墙挡住 → 不可见', !lineOfSight(g, a, b));
    const c: PathPoint = { x: -2, z: 3 };
    check('无遮挡 → 可见', lineOfSight(g, a, c));
  }

  if (fails.length !== 0) {
    throw new Error(`PathPlanner 自检失败：${fails.join('、')}`);
  }
  Logger.info(`[VERIFY] PathPlanner 路径规划自检 PASS（${total} 项断言全通过）`);
}
