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
- 执行状态机：`running → completed / protective-stopped`，终态不可被迟到或重复消息改写
- 活动阶段的人工停止、关节跟踪误差和 TCP 力超限保护停机均保留原因、来源与停机时间
- stop 与遥测（按 `sampleId`/`seq`）重试幂等；原始遥测始终记录并可查询
- 轨迹 release、执行遥测和安全审计（创建/完成/停机/迟到与重复消息）

扩展接口：`/api/calibrations`、`/api/frames`、`/api/dynamic-obstacles`、`/api/paths/:id/release`、`/api/executions`、`/api/executions/:id`、`/api/executions/:id/stop`、`/api/executions/:id/telemetry`（GET/POST）。

执行创建后异步推进（可用 `stepMs` 控制每 waypoint 间隔），经 `GET /api/executions/:id` 轮询状态；验证：`npm test`。
