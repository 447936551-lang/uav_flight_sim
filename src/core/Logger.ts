/*
 * Copyright (c) 2024-2026 陈鹏至 (Derek0769)
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
 * 日志（自实现，替代 @kit.PerformanceAnalysisKit 的 hilog）。
 * ---------------------------------------------------------------
 * 原工程的 Logger 直接调用专有 hilog，无法在 Node / CI / 离线仿真中运行。
 * 本实现保留完全一致的方法签名（debug/info/warn/error），底层映射到
 * 标准 console，使核心算法层完全脱离专有依赖：
 *   - 在真机 / 模拟器中：可替换为 hilog 适配；
 *   - 在 Node / CI 中：直接输出到标准错误流，便于 grep 验收。
 *
 * 注意：为便于 CI 与日志采集，所有级别统一输出到 console（info→stdout，
 * warn/error→stderr，debug→stderr），且前缀固定为 `[uav_flight_sim]`。
 */
import { LoggerLevel } from './LoggerLevel';

class LoggerModel {
  private prefix: string;

  constructor(prefix: string) {
    this.prefix = prefix;
  }

  /** 把若干参数格式化为可读字符串 */
  private format(args: unknown[]): string {
    return args
      .map((a) => (typeof a === 'string' ? a : safeStringify(a)))
      .join(' ');
  }

  debug(...args: unknown[]): void {
    if (LoggerLevel.DEBUG < LoggerModel.minLevel) {
      return;
    }
    // biome-ignore lint/suspicious/noConsole: 自实现 Logger 的有意输出
    console.debug(`[${this.prefix}][D] ${this.format(args)}`);
  }

  info(...args: unknown[]): void {
    if (LoggerLevel.INFO < LoggerModel.minLevel) {
      return;
    }
    // biome-ignore lint/suspicious/noConsole: 自实现 Logger 的有意输出
    console.info(`[${this.prefix}][I] ${this.format(args)}`);
  }

  warn(...args: unknown[]): void {
    if (LoggerLevel.WARN < LoggerModel.minLevel) {
      return;
    }
    // biome-ignore lint/suspicious/noConsole: 自实现 Logger 的有意输出
    console.warn(`[${this.prefix}][W] ${this.format(args)}`);
  }

  error(...args: unknown[]): void {
    if (LoggerLevel.ERROR < LoggerModel.minLevel) {
      return;
    }
    // biome-ignore lint/suspicious/noConsole: 自实现 Logger 的有意输出
    console.error(`[${this.prefix}][E] ${this.format(args)}`);
  }

  /** 全局最低输出级别（默认 INFO） */
  public static minLevel: LoggerLevel = LoggerLevel.INFO;
}

/** 安全的 JSON 序列化（避免循环引用导致崩溃） */
function safeStringify(v: unknown): string {
  try {
    return JSON.stringify(v);
  } catch {
    return String(v);
  }
}

export const Logger = new LoggerModel('uav_flight_sim');
