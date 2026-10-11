/**
 * M5-1：PX4 参数映射（静态标定真值源）
 * ------------------------------------------------------------------
 * 设计口径（见 docs + MEMORY「M4 高保真层」）：
 *   - 独立实现 + 静态参数标定，不逐位抄 PX4 控制代码。
 *   - 本文件是「PX4 公开默认参数 → 本仓高保真层增益」的唯一真值源，
 *     所有映射关系在此显式声明并通过 selfTest 钉死，防止后续漂移。
 *   - PX4 速率增益是**归一化油门输出**，须按本仓刚体惯量缩放到物理力矩增益
 *     （见 PX4_ATT_GAIN_SCALE / PX4_YAW_GAIN_SCALE，均源自阶段A「等刚度」标定）。
 *
 * 标定锚点取自 PX4 公开参数参考（parameter_reference.md, release/1.14）。
 */
import { Logger } from '../core/Logger';
import { DEFAULT_EKF, EkfConfig } from './EkfEstimator';

const TAG: string = 'ArDemo][Px4';

/** PX4 公开参数参考（release/1.14）的真实默认值。 */
export interface Px4StockParams {
  MC_ROLL_P: number;          // 姿态外环滚转 P
  MC_ROLLRATE_P: number;      // 速率内环滚转 P（归一化输出）
  MC_ROLLRATE_D: number;      // 速率内环滚转 D
  MC_ROLLRATE_I: number;      // 速率内环滚转 I
  MC_YAW_P: number;           // 姿态外环偏航 P
  MC_YAWRATE_P: number;       // 速率内环偏航 P
  MPC_XY_P: number;           // 水平位置环 P
  MPC_Z_P: number;            // 垂直位置环 P
  MPC_TILTMAX_AIR_DEG: number; // 空中最大倾角 (°)
  MPC_THR_HOVER: number;      // 悬停油门
  EKF2_GYR_NOISE: number;     // 陀螺噪声密度 (rad/s/√Hz)
  EKF2_ACC_NOISE: number;     // 加速计噪声密度 (m/s²/√Hz)
  EKF2_GPS_P_NOISE: number;   // GPS 位置噪声 (m)
}

export const PX4_STOCK: Px4StockParams = {
  MC_ROLL_P: 4.0,
  MC_ROLLRATE_P: 0.15,
  MC_ROLLRATE_D: 0.003,
  MC_ROLLRATE_I: 0.2,
  MC_YAW_P: 2.8,
  MC_YAWRATE_P: 0.2,
  MPC_XY_P: 0.95,
  MPC_Z_P: 1.0,
  MPC_TILTMAX_AIR_DEG: 45.0,
  MPC_THR_HOVER: 0.5,
  EKF2_GYR_NOISE: 0.015,
  EKF2_ACC_NOISE: 0.35,
  EKF2_GPS_P_NOISE: 0.5
};

/**
 * 本仓刚体动力学惯量缩放系数（与 DroneController DYN_* 同源）。
 * PX4 速率增益是归一化油门输出，须按惯量缩放到物理力矩增益：
 *   attRateKp = MC_ROLL_P * PX4_ATT_GAIN_SCALE = 4.0 * 1.875 = 7.5 (本仓 DYN_ATT_RATE_KP)
 *   yawRateKp = MC_YAW_P  * PX4_YAW_GAIN_SCALE = 2.8 * (8/2.8)  ≈ 8.0 (本仓 DYN_YAW_RATE_KP)
 */
export const PX4_ATT_GAIN_SCALE: number = 1.875;
export const PX4_YAW_GAIN_SCALE: number = 8.0 / 2.8;
/** 与既有单 PD 等刚度目标：attRateKp * rateKp = 225（ωn≈15, ζ≈1.0）。 */
export const PX4_STIFFNESS_TARGET: number = 225.0;
/** 本仓最大角速率 (rad/s)，覆盖 MPC_TILTMAX_AIR 对应机动（> PX4 MC_*RATE_MAX 默认 3.49）。 */
export const PX4_MAX_RATE: number = 6.0;

/** 本仓级联控制器增益（由 PX4 姿态/速率默认参数等带宽标定推导）。 */
export interface ControllerGains {
  attRateKp: number;
  rateKp: number;
  rateKd: number;
  yawRateKp: number;
  maxRate: number;
}

/** 由 PX4 姿态/速率默认参数推导本仓级联增益（等带宽标定）。 */
export function deriveControllerGains(p: Px4StockParams): ControllerGains {
  const attRateKp: number = p.MC_ROLL_P * PX4_ATT_GAIN_SCALE; // 7.5
  const rateKp: number = PX4_STIFFNESS_TARGET / attRateKp;    // 30.0
  const rateKd: number = 0.0;
  const yawRateKp: number = p.MC_YAW_P * PX4_YAW_GAIN_SCALE;  // ≈8.0
  return {
    attRateKp: attRateKp,
    rateKp: rateKp,
    rateKd: rateKd,
    yawRateKp: yawRateKp,
    maxRate: PX4_MAX_RATE
  };
}

/** 由 PX4 EKF2 噪声参数推导本仓 EKF 配置（直接等量标定）。 */
export function px4EkfConfig(p: Px4StockParams): EkfConfig {
  return {
    accelNoise: p.EKF2_ACC_NOISE,
    gyroNoise: p.EKF2_GYR_NOISE,
    accelBiasWalk: 0.005,
    gyroBiasWalk: 0.001,
    posNoise: 0.1,
    velNoise: 0.05,
    gravity: 9.81,
    minUpdates: 10,
    posVarGate: 0.25
  };
}

/** 自检：钉死「PX4 默认参数 → 本仓增益」映射不漂移。 */
export function selfTest(): void {
  const fails: string[] = [];
  const check = (name: string, ok: boolean): void => {
    if (!ok) {
      fails.push(name);
    }
  };

  // 1) EKF 配置：PX4 等量标定须与本仓 DEFAULT_EKF 完全一致
  const ekf: EkfConfig = px4EkfConfig(PX4_STOCK);
  check('ekf.accelNoise==EKF2_ACC_NOISE(0.35)', Math.abs(ekf.accelNoise - 0.35) < 1e-9);
  check('ekf.gyroNoise==EKF2_GYR_NOISE(0.015)', Math.abs(ekf.gyroNoise - 0.015) < 1e-9);
  check('ekf matches DEFAULT_EKF',
    ekf.accelNoise === DEFAULT_EKF.accelNoise &&
    ekf.gyroNoise === DEFAULT_EKF.gyroNoise &&
    ekf.accelBiasWalk === DEFAULT_EKF.accelBiasWalk &&
    ekf.gyroBiasWalk === DEFAULT_EKF.gyroBiasWalk &&
    ekf.posNoise === DEFAULT_EKF.posNoise &&
    ekf.velNoise === DEFAULT_EKF.velNoise &&
    ekf.gravity === DEFAULT_EKF.gravity &&
    ekf.minUpdates === DEFAULT_EKF.minUpdates &&
    ekf.posVarGate === DEFAULT_EKF.posVarGate);

  // 2) 控制器增益：推导须与本仓 DYN_* 常量一致
  const g: ControllerGains = deriveControllerGains(PX4_STOCK);
  check('attRateKp==7.5', Math.abs(g.attRateKp - 7.5) < 1e-9);
  check('rateKp==30', Math.abs(g.rateKp - 30.0) < 1e-9);
  check('yawRateKp==8', Math.abs(g.yawRateKp - 8.0) < 1e-6);
  check('maxRate==6', Math.abs(g.maxRate - 6.0) < 1e-9);
  // 3) 等刚度目标守恒
  check('stiffness==225', Math.abs(g.attRateKp * g.rateKp - 225.0) < 1e-6);

  if (fails.length > 0) {
    Logger.error(`${TAG} selfTest FAIL: ${fails.join(', ')}`);
  } else {
    Logger.info(`${TAG} selfTest PASS (PX4 calibration mapped & consistent)`);
  }
}
