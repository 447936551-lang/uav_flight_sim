/**
 * 三维向量（自实现，替代 @kit.ArkGraphics3D 的 Vec3）。
 * ---------------------------------------------------------------
 * 原工程 WorldObject 引用的 ArkGraphics3D.Vec3 属于专有渲染套件，
 * 无法在 OpenHarmony / 纯 Node 环境编译。本文件提供等价的纯数据结构与
 * 基础运算，使核心算法层彻底脱离专有依赖，可在 CI 与离线仿真中直接运行。
 *
 * 设计要点：
 *   - Vec3 仅作为 {x, y, z} 纯数据接口，零方法、零专有类型；
 *   - 所有运算以纯函数形式提供（add / scale / length / ...），
 *     避免引入对象方法导致的序列化 / 跨边界传递问题。
 */
export interface Vec3 {
  x: number;
  y: number;
  z: number;
}

/** 构造一个 Vec3 */
export function vec3(x: number = 0, y: number = 0, z: number = 0): Vec3 {
  return { x, y, z };
}

/** 零向量常量 */
export const ZERO: Vec3 = { x: 0, y: 0, z: 0 };

/** 逐分量相加 */
export function add(a: Vec3, b: Vec3): Vec3 {
  return { x: a.x + b.x, y: a.y + b.y, z: a.z + b.z };
}

/** 逐分量相减 */
export function sub(a: Vec3, b: Vec3): Vec3 {
  return { x: a.x - b.x, y: a.y - b.y, z: a.z - b.z };
}

/** 标量缩放 */
export function scale(a: Vec3, s: number): Vec3 {
  return { x: a.x * s, y: a.y * s, z: a.z * s };
}

/** 向量长度（模） */
export function length(a: Vec3): number {
  return Math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
}

/** 归一化；零向量返回 ZERO（避免除零产生 NaN 污染整条物理链） */
export function normalizeVec3(a: Vec3): Vec3 {
  const len: number = length(a);
  if (len < 1e-9) {
    return { x: 0, y: 0, z: 0 };
  }
  return { x: a.x / len, y: a.y / len, z: a.z / len };
}
