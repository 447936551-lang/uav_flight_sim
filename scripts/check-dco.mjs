#!/usr/bin/env node
// DCO gate: 校验历史上每个提交都携带 Signed-off-by 行。
// 用法： node scripts/check-dco.mjs [git range, 默认 HEAD]
//   - 默认检查 HEAD 可达的全部提交
//   - 传入范围如 "origin/main..HEAD" 可只查本次 PR 的提交

import { execSync } from 'node:child_process';
import process from 'node:process';

const range = process.argv[2] || 'HEAD';

let raw;
try {
  raw = execSync(`git log ${range} --format=%B%x00`, { encoding: 'utf8' });
} catch (e) {
  console.error(`无法读取 git 历史（范围：${range}）。错误：${e.message}`);
  process.exit(1);
}

const commits = raw.split('\0').map((s) => s.trim()).filter(Boolean);
if (commits.length === 0) {
  console.error(`范围内没有提交（范围：${range}）。`);
  process.exit(1);
}

let ok = true;
commits.forEach((msg) => {
  if (!/^Signed-off-by:/m.test(msg)) {
    const firstLine = (msg.split('\n')[0] || '(空提交信息)').slice(0, 80);
    console.error(`✗ 缺少 Signed-off-by: ${firstLine}`);
    ok = false;
  }
});

if (ok) {
  console.log(`✓ DCO 校验通过：${commits.length} 个提交均已签名`);
  process.exit(0);
} else {
  console.error('✗ DCO 校验失败：部分提交缺少 Signed-off-by。请用 `git commit -s` 或安装 tools/dco/commit-msg hook。');
  process.exit(1);
}
