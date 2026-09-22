/**
 * AR 世界对象（Layer 3 · 锚点 + 物体） —— Phase 3
 * ---------------------------------------------------------------
 * PRD §5.2 定义的 WorldObject：AR 世界中虚拟物体的基类。
 * 无人机（Phase 3 落地的第一个实例）、未来的障碍物、标记点都继承自它。
 *
 * 设计要点：
 *   - 每个 WorldObject 可选地"锚定"到一个 AR 锚点（ARAnchor）。
 *   - 锚点由 AREngine 持续校正位姿，因此物体真正"钉"在现实世界的某点，
 *     即使设备追踪发生漂移也不会漂走（这是引入锚点的核心价值）。
 *   - 物体的世界位姿 = 锚点当前世界位姿（base）+ 自身相对锚点的本地偏移（localOffset）。
 *   - 若没有锚点（锚点创建失败 / 追踪丢失），回退到静态 base（placement），
 *     保证"放不下锚点也能飞"—— 不引入新失败模式。
 *
 * 坐标约定：本类内部一律用 AR 世界坐标（X右 / Y上 / Z前（朝向用户为 +Z），米）；
 * 具体怎么渲染到 AGP（绕 Y 90°）由消费方（ARDroneCallback.updateDrone）负责，
 * 本类不碰渲染坐标系。
 *
 * 注意：原工程的 Vec3 来自专有套件 @kit.ArkGraphics3D，本仓库改用自实现的
 * src/core/Vec3.ts，使本模块可在 Node / CI 中直接编译与单测。
 */
import { Vec3 } from '../core/Vec3';

/** AR 世界坐标下的一个点（米） */
export interface WorldPose {
  x: number;
  y: number;
  z: number;
}

/** 世界对象类型 */
export type WorldObjectType = 'drone' | 'obstacle' | 'marker';

/**
 * AR 世界对象基类。
 * ---------------------------------------------------------------
 * 只持有"在世界里处于哪里"的状态，不持有任何渲染节点（渲染由消费方负责），
 * 也不持有 AREngine 对象（锚点句柄由 AnchorManager 管理，本类只记一个字符串 id）。
 * 这样上层模块（无人机 / 障碍物）可以纯粹地复用"锚定 + 偏移"的位姿推导逻辑。
 */
export abstract class WorldObject {
  /** 对象唯一 id（业务侧分配，区别于 AREngine 的锚点 id） */
  public readonly id: string;
  /** 对象类型 */
  public readonly kind: WorldObjectType;
  /**
   * 锚点 id（由 AnchorManager 分配）；null 表示尚未锚定。
   * 尚未锚定时，本对象使用 `anchorBase` 的静态初值（即放置点 placement）。
   */
  public anchorId: string | null = null;
  /** 相对锚点的本地偏移（AR 世界坐标，米）。无人机飞行时即控制器 offset。 */
  public localOffset: Vec3 = { x: 0, y: 0, z: 0 };
  /**
   * 由锚点位姿推导出的"底座"世界位置（AR 世界坐标，米）。
   * - 有锚点且追踪正常：每帧被 AnchorManager 的最新位姿刷新。
   * - 锚点追踪丢失：保留上次已知值（不跳变）。
   * - 从未成功锚定：等于放置点 placement（静态）。
   */
  public anchorBase: WorldPose = { x: 0, y: 0, z: 0 };
  /** 是否已拿到过至少一次有效锚点位姿 */
  public anchorResolved: boolean = false;

  protected constructor(kind: WorldObjectType, id: string) {
    this.kind = kind;
    this.id = id;
  }

  /**
   * 写入锚点最新世界位姿（来自 AnchorManager）。
   * 仅在 p 非 null（追踪正常）时更新，追踪丢失时保留上次值。
   */
  public setAnchorBase(p: WorldPose | null): void {
    if (p !== null) {
      this.anchorBase = p;
      this.anchorResolved = true;
    }
  }

  /** 当前世界位置（AR 世界坐标）= base + localOffset */
  public worldPositionAR(): WorldPose {
    return {
      x: this.anchorBase.x + this.localOffset.x,
      y: this.anchorBase.y + this.localOffset.y,
      z: this.anchorBase.z + this.localOffset.z,
    };
  }

  /** 调试用：一行摘要 */
  public describe(): string {
    const base: WorldPose = this.anchorBase;
    return `${this.kind}#${this.id} anchor=${this.anchorId ?? '无'} ` +
      `resolved=${this.anchorResolved} base(${base.x.toFixed(2)},${base.y.toFixed(2)},${base.z.toFixed(2)}) ` +
      `off(${this.localOffset.x.toFixed(2)},${this.localOffset.y.toFixed(2)},${this.localOffset.z.toFixed(2)})`;
  }
}

/**
 * 无人机世界对象（Phase 3 第一个落地实例）。
 * 当前只比基类多一个语义标签，预留后续无人机专属状态（如电量、航灯）。
 */
export class DroneWorldObject extends WorldObject {
  public constructor(id: string) {
    super('drone', id);
  }
}
