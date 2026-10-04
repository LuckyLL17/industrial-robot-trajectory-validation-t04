// 执行状态链端到端验证：正常完成、活动中停机、迟到消息、重复消息与审计。
// 启动独立服务实例（PORT=6399），跑完后自动退出；任一断言失败则以非零码退出。
import { spawn } from 'node:child_process';
import { setTimeout as delay } from 'node:timers/promises';

const PORT = 6399;
const BASE = `http://127.0.0.1:${PORT}`;
let passed = 0, failed = 0;
const ok = (cond, name, extra = '') => {
  if (cond) { passed++; console.log(`  ✓ ${name}`); }
  else { failed++; console.error(`  ✗ ${name}${extra ? ' — ' + extra : ''}`); }
};

async function api(method, path, body) {
  const init = { method, headers: {} };
  if (body !== undefined) { init.headers['content-type'] = 'application/json'; init.body = JSON.stringify(body); }
  const res = await fetch(BASE + path, init);
  let json = null;
  try { json = await res.json(); } catch { /* empty body */ }
  return { status: res.status, body: json };
}

async function waitFor(cond, timeoutMs = 8000, stepMs = 40) {
  const t0 = Date.now();
  while (Date.now() - t0 < timeoutMs) { const v = await cond(); if (v) return v; await delay(stepMs); }
  return null;
}

// 远离围栏/夹具碰撞体且在工作半径内的轨迹，保证通过校验。
async function makeValidatedPath() {
  const waypoints = [
    { x: 1000, y: -120, z: 300, rx: 0, ry: 0, rz: 0, speed: 120, blend: 10 },
    { x: 1050, y: -40, z: 330, rx: 0, ry: 0, rz: 0, speed: 120, blend: 10 },
    { x: 1100, y: 60, z: 370, rx: 0, ry: 0, rz: 0, speed: 120, blend: 10 },
    { x: 1160, y: 160, z: 420, rx: 0, ry: 0, rz: 0, speed: 120, blend: 10 },
  ];
  const p = (await api('POST', '/api/paths', { name: 'lifecycle-test', cellId: 'cell-01', robotId: 'robot-01', toolId: 'gun-01', waypoints })).body;
  const v = (await api('POST', `/api/paths/${p.id}/validate`)).body;
  if (!v.valid) throw new Error('test path failed validation: ' + JSON.stringify(v.errors));
  return p;
}

async function main() {
  const server = spawn('node_modules/.bin/tsx', ['src/index.ts'], {
    env: { ...process.env, PORT: String(PORT) }, detached: true, stdio: ['ignore', 'pipe', 'pipe'],
  });
  server.stderr.on('data', d => process.stderr.write(d));
  try {
    const healthy = await waitFor(async () => { try { return (await fetch(BASE + '/health')).ok; } catch { return false; } }, 20000, 250);
    if (!healthy) throw new Error('server did not start');
    const path = await makeValidatedPath();

    console.log('\n[1] 正常完成：running -> completed，终态字段齐备');
    const created = (await api('POST', '/api/executions', { pathId: path.id, stepMs: 40 })).body;
    ok(created.status === 'running' && created.progress === 0, '创建后处于 running 而不是直接 completed');
    const done = await waitFor(async () => { const e = (await api('GET', `/api/executions/${created.id}`)).body; return e.status !== 'running' ? e : null; });
    ok(done && done.status === 'completed', '执行最终正常完成');
    ok(done.progress === 100 && !!done.finishedAt && !done.stoppedAt && !done.stopReason, 'completed 只有 finishedAt，无停止字段');
    ok(done.safety.protectiveStop === false, '正常完成无 protectiveStop 标志');
    const simTel = (await api('GET', `/api/executions/${created.id}/telemetry`)).body;
    ok(simTel.count === 4 && simTel.telemetry.every(t => t.source === 'simulation'), '仿真遥测逐 waypoint 记录且可查询');

    console.log('\n[2] 迟到消息：终态不被 stop / 超阈值遥测改写，原始遥测仍可查询');
    const lateStop = (await api('POST', `/api/executions/${created.id}/stop`)).body;
    ok(lateStop.status === 'completed' && !lateStop.stoppedAt && !lateStop.stopReason, 'completed 后迟到 stop 不改写状态');
    const lateTel = (await api('POST', `/api/executions/${created.id}/telemetry`, { seq: 9001, jointErrorDeg: 5 })).body;
    ok(lateTel.status === 'completed' && lateTel.safety.protectiveStop === false, 'completed 后超阈值遥测不改写状态与安全标志');
    const afterLate = (await api('GET', `/api/executions/${created.id}`)).body;
    ok(afterLate.finishedAt === done.finishedAt, 'finishedAt 不被迟到消息改动');
    const lateTelList = (await api('GET', `/api/executions/${created.id}/telemetry`)).body;
    ok(lateTelList.telemetry.some(t => t.seq === 9001 && t.jointErrorDeg === 5), '迟到的原始遥测样本仍被记录并可查询');

    console.log('\n[3] 活动中人工停机：保留原因，仿真不再推进，重复 stop 幂等');
    const run = (await api('POST', '/api/executions', { pathId: path.id, stepMs: 150 })).body;
    await delay(220); // 让仿真推进约 1 个 waypoint，确保处于活动阶段
    const stopped = (await api('POST', `/api/executions/${run.id}/stop`)).body;
    ok(stopped.status === 'protective-stopped', '活动阶段 stop 进入 protective-stopped');
    ok(stopped.stopReason === 'manual stop' && stopped.stopSource === 'operator', '人工停止原因与来源被保留');
    ok(!!stopped.stoppedAt && !stopped.finishedAt, 'protective-stopped 只有 stoppedAt，无 finishedAt');
    ok(stopped.progress < 100, '进度冻结在 100 以下');
    const stoppedAgain = (await api('POST', `/api/executions/${run.id}/stop`)).body;
    ok(stoppedAgain.status === 'protective-stopped' && stoppedAgain.stoppedAt === stopped.stoppedAt && stoppedAgain.stopReason === stopped.stopReason, '重复 stop 返回一致结果（stoppedAt/原因不变）');
    await delay(700); // 远超剩余仿真时长
    const afterStop = (await api('GET', `/api/executions/${run.id}`)).body;
    ok(afterStop.status === 'protective-stopped' && afterStop.stoppedAt === stopped.stoppedAt, '仿真循环不会把停机改写成 completed');
    ok(afterStop.telemetry.length === stopped.telemetry.length, '停机后不再追加仿真遥测');
    const lateOnStopped = (await api('POST', `/api/executions/${run.id}/telemetry`, { seq: 7001, tcpForceN: 500 })).body;
    ok(lateOnStopped.status === 'protective-stopped' && lateOnStopped.stopReason === 'manual stop', '停机后迟到遥测不覆盖原始停止原因');

    console.log('\n[4] 活动中关节误差超限：保护停机并保留原因，同一遥测重试幂等');
    const run2 = (await api('POST', '/api/executions', { pathId: path.id, stepMs: 150 })).body;
    await delay(60);
    const hit = (await api('POST', `/api/executions/${run2.id}/telemetry`, { seq: 101, jointErrorDeg: 2.5 })).body;
    ok(hit.status === 'protective-stopped' && hit.stopReason === 'joint tracking error' && hit.stopSource === 'telemetry', '关节误差超限触发保护停机并保留原因');
    const retry = (await api('POST', `/api/executions/${run2.id}/telemetry`, { seq: 101, jointErrorDeg: 2.5 })).body;
    ok(retry.status === 'protective-stopped' && retry.stoppedAt === hit.stoppedAt && retry.stopReason === hit.stopReason, '同一遥测重试返回一致结果');
    const tel2 = (await api('GET', `/api/executions/${run2.id}/telemetry`)).body;
    ok(tel2.telemetry.filter(t => t.seq === 101).length === 1, '重复遥测只记录一次（按 seq 去重）');

    console.log('\n[5] 活动中 TCP 力超限：保护停机并保留原因');
    const run3 = (await api('POST', '/api/executions', { pathId: path.id, stepMs: 150 })).body;
    await delay(60);
    const hit2 = (await api('POST', `/api/executions/${run3.id}/telemetry`, { seq: 201, tcpForceN: 250 })).body;
    ok(hit2.status === 'protective-stopped' && hit2.stopReason === 'TCP force threshold exceeded' && hit2.stopSource === 'telemetry', 'TCP 力超限触发保护停机并保留原因');

    console.log('\n[6] 审计轨迹：创建、完成、停机、迟到与重复消息均有记录');
    const audit = (await api('GET', '/api/audit')).body;
    const actions = audit.map(a => a.action);
    ok(actions.includes('execution.create'), '审计含 execution.create');
    ok(actions.includes('execution.complete'), '审计含 execution.complete');
    ok(audit.some(a => a.action === 'execution.stop' && a.payload.reason === 'manual stop' && a.payload.source === 'operator'), '审计含人工停机及原因');
    ok(audit.some(a => a.action === 'execution.stop' && a.payload.reason === 'joint tracking error' && a.payload.source === 'telemetry'), '审计含关节误差停机及原因');
    ok(audit.some(a => a.action === 'execution.stop' && a.payload.reason === 'TCP force threshold exceeded' && a.payload.source === 'telemetry'), '审计含 TCP 力超限停机及原因');
    ok(actions.includes('execution.stop.ignored'), '审计含被忽略的迟到/重复 stop');
    ok(actions.includes('execution.telemetry.late'), '审计含迟到遥测');
    ok(actions.includes('execution.telemetry.duplicate'), '审计含重复遥测重试');
  } finally {
    try { process.kill(-server.pid, 'SIGTERM'); } catch { try { server.kill('SIGTERM'); } catch { /* already gone */ } }
  }
  console.log(`\n${passed} passed, ${failed} failed`);
  process.exit(failed ? 1 : 0);
}

main().catch(err => { console.error(err); process.exit(1); });
