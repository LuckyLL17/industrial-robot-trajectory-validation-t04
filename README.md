# 工业机器人离线轨迹规划与碰撞验证服务

纯后端 Node.js + TypeScript + Fastify 服务，默认无需 Docker、数据库或消息中间件即可运行。

## 启动

```bash
npm install
npm run dev
```

默认端口：`6303`。健康检查：`GET /health`。

## 业务闭环

- 领域状态模型与事件输入
- 可回放的时间序列/仿真流程
- 约束校验与风险门禁
- 方案版本、审计轨迹与异常处置
- 所有状态暂存于进程内，便于本机演示；可扩展为数据库、事件总线和实时流处理架构

## 增强业务闭环

- TCP 工具标定误差和标定有效期管理
- 工件基准坐标系登记
- 动态障碍物上报与清除
- 轨迹发布前的碰撞、标定、动态障碍物多重安全门禁
- 执行中关节跟踪误差和 TCP 力阈值保护停机
- 轨迹 release、执行遥测和安全审计

扩展接口：`/api/calibrations`、`/api/frames`、`/api/dynamic-obstacles`、`/api/paths/:id/release`、`/api/executions/:id/telemetry`。

## 执行状态链与停机语义

`POST /api/executions` 只创建活动执行（`running`），执行由遥测推进，不再在创建时直接 `completed`。

状态机（终态一旦进入，任何迟到或重复消息都不得改写）：

| 当前 | 事件 | 迁移后 | 留存信息 |
| --- | --- | --- | --- |
| running | 遥测 `event: "completed"` | `completed` | `finishedAt`、progress=100 |
| running | 遥测 `jointErrorDeg > 2` | `protective-stopped` | `stopReason=joint-tracking-error`、`stopDetail`（通道/实测值/阈值/触发样本 seq）、`safety.protectiveStop=true` |
| running | 遥测 `tcpForceN > 180` | `protective-stopped` | `stopReason=tcp-force-exceeded`，同上 |
| running | `POST /api/executions/:id/stop` | `stopped` | `stopReason=manual-stop`、`safety.manualStop=true`、操作人 |
| 任意终态 | 新的迟到 stop / 超限遥测 | **拒绝（409）**，状态/安全标志/时间戳不变；消息记入 `lateMessages` 与审计 |
| 任意终态 | 已受理过的同一消息重试 | **200 幂等返回**，不重复入库，附 `X-Idempotent-Replay: true` |

人工停机使用独立终态 `stopped`，与安全保护的 `protective-stopped` 区分，避免“停机发生在执行中还是结束后”无法判定。

遥测幂等键：优先取 `messageId` 或 `Idempotency-Key` 请求头；缺省时对规范化（键排序）请求体做指纹，因此同一遥测原样重试天然去重。触发保护停机的原始样本仍保留在 `telemetry` 中，可通过 `GET /api/executions/:id/telemetry` 查询；迟到消息单独列在同一响应的 `lateMessages` 中。

创建、遥测受理/重复/迟到拒绝、人工停机、保护停机、重复停机统一写入 `GET /api/audit`。

测试：`npm test`（Node 内置 test runner + tsx，覆盖正常完成、活动中三类停机、迟到消息与重复消息）。
