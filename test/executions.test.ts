import { test, before, after } from 'node:test';
import assert from 'node:assert/strict';
import type { FastifyInstance } from 'fastify';
import { app } from '../src/index.ts';

const server = app as unknown as FastifyInstance;
const BASE = '/api';

async function call(method: string, url: string, body?: unknown, headers?: Record<string, string>) {
  const injectOptions: any = { method, url: BASE + url, headers: { ...headers } };
  if (body !== undefined) {
    injectOptions.payload = body;
    injectOptions.headers['content-type'] = 'application/json';
  }
  const res = await server.inject(injectOptions);
  let json: any = null;
  try { json = res.json(); } catch { /* empty */ }
  return { status: res.statusCode, body: json, replay: res.headers['x-idempotent-replay'] === 'true' };
}

// 创建一条通过校验的路径，返回 execution
async function setupRunningExecution() {
  const wp = (x: number, z: number) => ({ x, y: 0, z, rx: 0, ry: 0, rz: 0, speed: 100, blend: 10 });
  const path = (await call('POST', '/paths', {
    name: `t-${Date.now()}-${Math.random()}`, cellId: 'cell-01', robotId: 'robot-01', toolId: 'gun-01',
    waypoints: [wp(-950, 0), wp(-950, 80), wp(-950, 120)],
  })).body;
  const validation = (await call('POST', `/paths/${path.id}/validate`)).body;
  assert.equal(validation.valid, true, JSON.stringify(validation.errors));
  const exec = (await call('POST', '/executions', { pathId: path.id })).body;
  return { path, validation, exec };
}

before(() => { /* app 已 import，不监听端口（inject 测试） */ });
after(async () => { await server.close(); });

// ---------- 正常完成 ----------
test('创建已校验路径的执行后状态为 running，而非直接 completed', async () => {
  const { exec } = await setupRunningExecution();
  assert.equal(exec.status, 'running');
  assert.equal(exec.progress, 0);
  assert.deepEqual(exec.telemetry, []);
  assert.equal(exec.safety.protectiveStop, false);
  assert.equal(exec.safety.manualStop, false);
});

test('遥测推进并发送 completed 后正常结束，原始遥测可查询', async () => {
  const { exec } = await setupRunningExecution();
  for (const waypoint of [0, 1]) {
    const r = await call('POST', `/executions/${exec.id}/telemetry`, { messageId: `w-${waypoint}`, waypoint, tcpSpeed: 100 });
    assert.equal(r.status, 200);
    assert.equal(r.body.status, 'running');
  }
  const done = await call('POST', `/executions/${exec.id}/telemetry`, { messageId: 'w-fin', waypoint: 2, event: 'completed', tcpSpeed: 0 });
  assert.equal(done.status, 200);
  assert.equal(done.body.status, 'completed');
  assert.equal(done.body.progress, 100);
  assert.ok(done.body.finishedAt);
  assert.equal(done.body.stopReason, undefined);

  const q = await call('GET', `/executions/${exec.id}/telemetry`);
  assert.equal(q.body.count, 3);
  // 原始遥测字段完整保留
  assert.deepEqual(q.body.telemetry[0], {
    seq: 1, receivedAt: q.body.telemetry[0].receivedAt, messageId: 'w-0', waypoint: 0, tcpSpeed: 100,
  });
});

// ---------- 活动中停机：人工 / 关节误差 / TCP 力 ----------
test('活动中人工 stop 进入 stopped 并保留 manual-stop 原因，不误标保护停机', async () => {
  const { exec } = await setupRunningExecution();
  const stoppedAt = Date.now();
  const r = await call('POST', `/executions/${exec.id}/stop`, { operator: 'op-7' });
  assert.equal(r.status, 200);
  assert.equal(r.body.status, 'stopped');
  assert.equal(r.body.stopReason, 'manual-stop');
  assert.equal(r.body.safety.manualStop, true);
  assert.equal(r.body.safety.protectiveStop, false);
  assert.equal(r.body.stopDetail.source, 'operator');
  assert.equal(r.body.stopDetail.operator, 'op-7');
  assert.ok(Math.abs(Date.parse(r.body.stoppedAt) - stoppedAt) < 5000);
});

test('关节误差超阈值：protective-stopped + joint-tracking-error，原因与触发样本保留', async () => {
  const { exec } = await setupRunningExecution();
  const r = await call('POST', `/executions/${exec.id}/telemetry`, { messageId: 'joint-1', waypoint: 1, jointErrorDeg: 3.5, tcpForceN: 50 });
  assert.equal(r.status, 200);
  assert.equal(r.body.status, 'protective-stopped');
  assert.equal(r.body.stopReason, 'joint-tracking-error');
  assert.equal(r.body.safety.protectiveStop, true);
  assert.equal(r.body.safety.manualStop, false);
  assert.deepEqual(r.body.stopDetail, {
    source: 'telemetry', channel: 'jointErrorDeg', value: 3.5, limit: 2, triggerSeq: 1, triggerMessageId: 'joint-1',
  });
  // 触发保护的原始样本仍在遥测中可查
  const sample = r.body.telemetry.find((t: any) => t.messageId === 'joint-1');
  assert.equal(sample.jointErrorDeg, 3.5);
});

test('TCP 力超阈值：protective-stopped + tcp-force-exceeded', async () => {
  const { exec } = await setupRunningExecution();
  const r = await call('POST', `/executions/${exec.id}/telemetry`, { messageId: 'force-1', waypoint: 0, tcpForceN: 210 });
  assert.equal(r.status, 200);
  assert.equal(r.body.status, 'protective-stopped');
  assert.equal(r.body.stopReason, 'tcp-force-exceeded');
  assert.equal(r.body.stopDetail.channel, 'tcpForceN');
  assert.equal(r.body.stopDetail.value, 210);
  assert.equal(r.body.stopDetail.limit, 180);
});

test('阈值内遥测不会停机（边界值不触发）', async () => {
  const { exec } = await setupRunningExecution();
  const r = await call('POST', `/executions/${exec.id}/telemetry`, { messageId: 'edge', waypoint: 0, jointErrorDeg: 2, tcpForceN: 180 });
  assert.equal(r.body.status, 'running');
  assert.equal(r.body.safety.protectiveStop, false);
});

// ---------- 迟到消息不能改写终态 ----------
test('completed 后的迟到 stop 与迟到超限遥测都被拒绝且不改写终态', async () => {
  const { exec } = await setupRunningExecution();
  await call('POST', `/executions/${exec.id}/telemetry`, { messageId: 'fin', event: 'completed' });

  const lateStop = await call('POST', `/executions/${exec.id}/stop`, {});
  assert.equal(lateStop.status, 409);
  assert.equal(lateStop.body.status, 'completed');

  const lateTele = await call('POST', `/executions/${exec.id}/telemetry`, { messageId: 'late-force', tcpForceN: 999 });
  assert.equal(lateTele.status, 409);
  assert.equal(lateTele.body.status, 'completed');

  // 终态、时间戳、安全标志均未被改写
  const fresh = (await call('GET', `/executions/${exec.id}`)).body;
  assert.equal(fresh.status, 'completed');
  assert.equal(fresh.stopReason, undefined);
  assert.equal(fresh.safety.protectiveStop, false);
  assert.equal(fresh.safety.manualStop, false);
  // 迟到消息留痕，可在遥测查询中审计
  assert.equal(fresh.lateMessages.length, 2);
  assert.equal(fresh.lateMessages[0].kind, 'stop');
  assert.equal(fresh.lateMessages[1].kind, 'telemetry');
  assert.equal(fresh.lateMessages[1].statusAtReceive, 'completed');
  assert.equal(fresh.lateMessages[1].body.tcpForceN, 999);
});

test('protective-stopped 后的迟到遥测不能改写停机原因（首个原因保留）', async () => {
  const { exec } = await setupRunningExecution();
  await call('POST', `/executions/${exec.id}/telemetry`, { messageId: 'j', jointErrorDeg: 5 });
  const late = await call('POST', `/executions/${exec.id}/telemetry`, { messageId: 'f', tcpForceN: 500 });
  assert.equal(late.status, 409);
  const fresh = (await call('GET', `/executions/${exec.id}`)).body;
  assert.equal(fresh.status, 'protective-stopped');
  assert.equal(fresh.stopReason, 'joint-tracking-error');
  assert.equal(fresh.stopDetail.triggerMessageId, 'j');
  assert.equal(fresh.lateMessages.length, 1);
});

// ---------- 重复消息幂等 ----------
test('重复 stop 返回一致结果（幂等），状态与 stoppedAt 不变', async () => {
  const { exec } = await setupRunningExecution();
  const first = await call('POST', `/executions/${exec.id}/stop`, { operator: 'op-1' });
  assert.equal(first.status, 200);
  const stoppedAt = first.body.stoppedAt;

  const again = await call('POST', `/executions/${exec.id}/stop`, { operator: 'op-1' });
  assert.equal(again.status, 200);
  assert.equal(again.replay, true);
  assert.equal(again.body.status, 'stopped');
  assert.equal(again.body.stopReason, 'manual-stop');
  assert.equal(again.body.stoppedAt, stoppedAt);

  // 保护停机后的重复 stop 也一致返回该终态
  const e2 = (await setupRunningExecution()).exec;
  await call('POST', `/executions/${e2.id}/telemetry`, { messageId: 'p', tcpForceN: 300 });
  const s2 = await call('POST', `/executions/${e2.id}/stop`, {});
  assert.equal(s2.status, 200);
  assert.equal(s2.replay, true);
  assert.equal(s2.body.status, 'protective-stopped');
  assert.equal(s2.body.stopReason, 'tcp-force-exceeded');
});

test('同一遥测重试：显式 messageId 与请求体指纹两种方式都幂等，不重复入库', async () => {
  const { exec } = await setupRunningExecution();
  const payload = { messageId: 'same-1', waypoint: 0, tcpForceN: 250 };
  const first = await call('POST', `/executions/${exec.id}/telemetry`, payload);
  assert.equal(first.status, 200);
  assert.equal(first.body.status, 'protective-stopped');
  const stoppedAt = first.body.stoppedAt;

  const retry = await call('POST', `/executions/${exec.id}/telemetry`, payload);
  assert.equal(retry.status, 200);
  assert.equal(retry.replay, true);
  assert.equal(retry.body.status, 'protective-stopped');
  assert.equal(retry.body.stoppedAt, stoppedAt);
  assert.equal(retry.body.telemetry.length, 1);

  // completed 事件重试同样返回一致的 200（而非迟到 409）
  const e2 = (await setupRunningExecution()).exec;
  const done = { messageId: 'done', event: 'completed' };
  const d1 = await call('POST', `/executions/${e2.id}/telemetry`, done);
  const d2 = await call('POST', `/executions/${e2.id}/telemetry`, done);
  assert.equal(d1.status, 200);
  assert.equal(d2.status, 200);
  assert.equal(d2.replay, true);
  assert.equal(d2.body.status, 'completed');
  assert.equal(d2.body.telemetry.length, 1);

  // 无 messageId 时按规范化请求体指纹去重（字段顺序不同也算同一条）
  const e3 = (await setupRunningExecution()).exec;
  const a = await call('POST', `/executions/${e3.id}/telemetry`, { waypoint: 0, tcpSpeed: 90, jointErrorDeg: 0.1 });
  const b = await call('POST', `/executions/${e3.id}/telemetry`, { jointErrorDeg: 0.1, tcpSpeed: 90, waypoint: 0 });
  assert.equal(a.status, 200);
  assert.equal(b.status, 200);
  assert.equal(b.replay, true);
  assert.equal(b.body.telemetry.length, 1);
});

// ---------- 审计统一 ----------
test('创建、遥测、停机、迟到与重复消息均进入统一审计流', async () => {
  const { exec } = await setupRunningExecution();
  await call('POST', `/executions/${exec.id}/telemetry`, { messageId: 't1', waypoint: 0 });
  await call('POST', `/executions/${exec.id}/telemetry`, { messageId: 't1', waypoint: 0 }); // 重复
  await call('POST', `/executions/${exec.id}/stop`, {});
  await call('POST', `/executions/${exec.id}/stop`, {}); // 重复 stop

  const events = (await call('GET', '/audit')).body
    .filter((a: any) => a.payload.executionId === exec.id)
    .map((a: any) => a.action);
  assert.deepEqual(events, [
    'execution.start',
    'execution.telemetry',
    'execution.telemetry.duplicate',
    'execution.stop',
    'execution.stop.duplicate',
  ]);

  // protective-stop 审计携带原因
  const e2 = (await setupRunningExecution()).exec;
  await call('POST', `/executions/${e2.id}/telemetry`, { messageId: 'x', jointErrorDeg: 9 });
  await call('POST', `/executions/${e2.id}/telemetry`, { messageId: 'late', tcpForceN: 900 });
  const ev2 = (await call('GET', '/audit')).body
    .filter((a: any) => a.payload.executionId === e2.id)
    .map((a: any) => ({ action: a.action, reason: a.payload.reason }));
  assert.deepEqual(ev2.map((x: any) => x.action), [
    'execution.start', 'execution.telemetry', 'execution.protective-stop', 'execution.telemetry.rejected-late',
  ]);
  const ps = ev2.find((x: any) => x.action === 'execution.protective-stop');
  assert.equal(ps.reason, 'joint-tracking-error');
});

// ---------- 创建门禁 ----------
test('未通过校验的路径不能创建执行', async () => {
  const wp = (x: number, z: number) => ({ x, y: 0, z, rx: 0, ry: 0, rz: 0, speed: 100, blend: 10 });
  const path = (await call('POST', '/paths', {
    name: `bad-${Date.now()}`, cellId: 'cell-01', robotId: 'robot-01',
    waypoints: [wp(0, 700), wp(100, 700)], // 撞围栏
  })).body;
  const validation = (await call('POST', `/paths/${path.id}/validate`)).body;
  assert.equal(validation.valid, false);
  const r = await call('POST', '/executions', { pathId: path.id });
  assert.equal(r.status, 409);
});
