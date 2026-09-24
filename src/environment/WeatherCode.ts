/**
 * 天气快照归一化（单位换算 / 异常值防护 / WMO 代码）
 * ---------------------------------------------------------------
 * 本文件是 App 侧 `WeatherService` 的**纯逻辑部分**。
 * 原文件还负责 HTTP 请求（`@ohos.net.http`）——那是网络依赖，会破坏
 * 本仓库「CI 可离线、确定性」的定位，故剥离在 App 侧。
 *
 * 这里保留的两件事，恰恰是**最该被单测钉死**的部分：
 *
 * 1) 风速单位换算。Open-Meteo 的**默认风速单位是 km/h**（不是 m/s）。
 *    历史上请求侧漏传 `wind_speed_unit=ms`，把 4.06 m/s 的微风读成 14.6，
 *    风扰被放大 3.6 倍、越过抗风上限，真机表现为「无人机被风吹走、
 *    定点保持完全失效」。因此这里做**二次纵深防御**：不信任请求参数，
 *    一律按服务端回报的 current_units 折算回 m/s。
 *
 * 2) 异常值防护。NaN / 负值 / 超物理上限一律归零 —— 宁可无风，
 *    也不要让脏数据把飞机吹跑（放大风扰比低估更危险）。
 */
import { Logger } from '../core/Logger';

const TAG: string = 'WeatherCode';

/** 一次天气快照（已归一化，单位见字段注释） */
export interface WeatherSnapshot {
  /** 10m 高度风速 m/s */
  windSpeed: number;
  /** 气象风向（度）：风吹来的方向，0 = 北，顺时针。非风矢量方向 */
  windDirection: number;
  /** 阵风风速 m/s */
  windGust: number;
  /** 气温 ℃ */
  temperature: number;
  /** 海平面气压 hPa */
  pressure: number;
  /** 相对湿度 % */
  humidity: number;
  /** 能见度 m */
  visibility: number;
  /** WMO 天气代码 */
  code: number;
  /** 是否成功取到数据（false 时其余字段为保守默认值） */
  online: boolean;
}

/** 离线兜底快照：零风 + 标准温压，保证无人机照常悬停 */
export function offlineSnapshot(): WeatherSnapshot {
  return {
    windSpeed: 0,
    windDirection: 0,
    windGust: 0,
    temperature: 20,
    pressure: 1013,
    humidity: 50,
    visibility: 10000,
    code: 0,
    online: false
  };
}

/**
 * 风速单位 → m/s 的换算系数。
 * @param unit 服务端 current_units 里的单位字符串
 * @returns 乘到原始数值上即得 m/s 的系数
 */
export function windUnitScale(unit: string | undefined): number {
  switch ((unit ?? '').trim().toLowerCase()) {
    case 'm/s':
      return 1;
    case 'km/h':
      return 1 / 3.6;
    case 'mph':
      return 0.44704;
    case 'kn':
      return 0.514444;
    default:
      // 单位未知/缺失：保守按 m/s，不做任何放大
      // （放大风扰比低估更危险，会把飞机直接吹跑）
      return 1;
  }
}

/**
 * 风速合理性防护（m/s）。
 * 非有限值、负值、或超过 45 m/s（≈14 级风，远超本机抗风上限）一律归零。
 */
export function clampWind(v: number): number {
  if (!isFinite(v) || v < 0 || v > 45) {
    return 0;
  }
  return v;
}

/** 安全取数：非有限数一律回退默认值（避免 NaN 污染飞控） */
export function numOr(v: number | undefined, d: number): number {
  if (typeof v === 'number' && isFinite(v)) {
    return v;
  }
  return d;
}

/** 把 WMO 天气代码转成中文短标签 */
export function wmoText(code: number): string {
  const map: Record<number, string> = {
    0: '晴',
    1: '少云',
    2: '多云',
    3: '阴',
    45: '雾',
    48: '雾凇',
    51: '毛毛雨',
    53: '小雨',
    55: '中雨',
    61: '小雨',
    63: '中雨',
    65: '大雨',
    71: '小雪',
    73: '中雪',
    75: '大雪',
    80: '阵雨',
    81: '阵雨',
    82: '强阵雨',
    95: '雷暴',
    96: '雷暴伴雹',
    99: '强雷暴'
  };
  return map[code] ?? '未知';
}

/**
 * 纯函数自检。覆盖：单位折算（含历史 bug 现场）、异常值防护。
 */
export function selfTest(): void {
  const fails: string[] = [];
  let total: number = 0;
  const check = (name: string, ok: boolean): void => {
    total++;
    if (!ok) {
      fails.push(name);
    }
  };

  // —— 单位折算：这是曾经把飞机吹走的那条路径，必须钉死 ——
  check('m/s 不折算', Math.abs(windUnitScale('m/s') - 1) < 1e-9);
  check('km/h → ÷3.6', Math.abs(windUnitScale('km/h') - 1 / 3.6) < 1e-9);
  check('km/h 的 14.6 应为 4.06 m/s（旧 bug 现场）',
    Math.abs(14.6 * windUnitScale('km/h') - 4.056) < 1e-3);
  check('mph → ×0.44704', Math.abs(windUnitScale('mph') - 0.44704) < 1e-9);
  check('kn → ×0.514444', Math.abs(windUnitScale('kn') - 0.514444) < 1e-9);
  check('单位缺失按 m/s（不放大）', Math.abs(windUnitScale(undefined) - 1) < 1e-9);
  check('大小写不敏感', Math.abs(windUnitScale('KM/H') - 1 / 3.6) < 1e-9);

  // —— 异常值防护 ——
  check('NaN → 0', clampWind(NaN) === 0);
  check('负风速 → 0', clampWind(-3) === 0);
  check('超上限(60) → 0', clampWind(60) === 0);
  check('12 m/s 保留', Math.abs(clampWind(12) - 12) < 1e-9);
  check('安全取数：undefined → 默认', numOr(undefined, 20) === 20);
  check('安全取数：NaN → 默认', numOr(NaN, 20) === 20);

  // —— 离线快照必须零风（零回归保证）——
  const off: WeatherSnapshot = offlineSnapshot();
  check('离线快照零风', off.windSpeed === 0 && off.windGust === 0);
  check('离线快照 online=false', off.online === false);

  // —— WMO 代码 ——
  check('WMO 0 = 晴', wmoText(0) === '晴');
  check('WMO 95 = 雷暴', wmoText(95) === '雷暴');
  check('WMO 未知道码 → 未知', wmoText(777) === '未知');

  if (fails.length !== 0) {
    throw new Error(`WeatherCode 自检失败：${fails.join('、')}`);
  }
  Logger.info(`${TAG} [VERIFY] 天气归一化自检 PASS（${total} 项断言全通过）`);
}
