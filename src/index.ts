import Fastify from 'fastify';
import { randomUUID, createHash } from 'node:crypto';
import { pathToFileURL } from 'node:url';

type Joint={min:number;max:number;maxVelocity:number}; type Robot={id:string;name:string;dof:number;joints:Joint[];reach:number}; type Pose={x:number;y:number;z:number;rx:number;ry:number;rz:number}; type Waypoint=Pose&{speed:number;blend:number;process?:Record<string,number>};
const app=Fastify({logger:true});const now=()=>new Date().toISOString();
const robots:Robot[]=[{id:'robot-01',name:'六轴焊接机器人 A',dof:6,reach:1450,joints:[-170,170,-100,135,-180,180].map((_,i)=>({min:[-170,-100,-150,-180,-120,-360][i],max:[170,100,150,180,120,360][i],maxVelocity:[180,160,170,360,360,500][i]}))}];
const cells:any[]=[{id:'cell-01',name:'车身侧围焊装单元',robotId:'robot-01',tools:[{id:'gun-01',mass:12,tcp:{x:0,y:0,z:220}}],obstacles:[{id:'fence',type:'box',center:{x:0,y:0,z:700},size:{x:1800,y:1800,z:1600}},{id:'fixture',type:'box',center:{x:500,y:0,z:300},size:{x:420,y:360,z:300}}]}];
const paths:any[]=[];const validations:any[]=[];const plans:any[]=[];const executions:any[]=[];const audit:any[]=[];const log=(action:string,payload:any)=>audit.push({id:randomUUID(),time:now(),action,payload});
function distance(a:Pose,b:Pose){return Math.hypot(a.x-b.x,a.y-b.y,a.z-b.z)}
function insideBox(p:Pose,b:any){return Math.abs(p.x-b.center.x)<=b.size.x/2&&Math.abs(p.y-b.center.y)<=b.size.y/2&&Math.abs(p.z-b.center.z)<=b.size.z/2}
function validatePath(path:any){const robot=robots.find(x=>x.id===path.robotId);const cell=cells.find(x=>x.id===path.cellId);const errors:any[]=[];let length=0,maxSpeed=0;for(let i=1;i<path.waypoints.length;i++){const prev=path.waypoints[i-1],cur=path.waypoints[i];length+=distance(prev,cur);maxSpeed=Math.max(maxSpeed,Number(cur.speed||0));for(const ob of cell?.obstacles||[]){if(insideBox(cur,ob)||insideBox({x:(prev.x+cur.x)/2,y:(prev.y+cur.y)/2,z:(prev.z+cur.z)/2,rx:0,ry:0,rz:0},ob))errors.push({type:'collision',obstacleId:ob.id,segment:i-1,clearance:0});}}
for(const p of path.waypoints){const reach=Math.hypot(p.x,p.y,p.z);if(reach>Number(robot?.reach||0))errors.push({type:'unreachable',waypoint:p});if(Math.abs(p.rx)>180||Math.abs(p.ry)>180||Math.abs(p.rz)>360)errors.push({type:'orientation-limit',waypoint:p});}
if(maxSpeed>220)errors.push({type:'speed-limit',maxSpeed});return{valid:errors.length===0,errors,length:Number(length.toFixed(2)),maxSpeed,estimatedSeconds:Number((length/Math.max(1,maxSpeed||100)).toFixed(2)),singularityRisk:path.waypoints.filter((p:any)=>Math.abs(p.z)<80).length};}

// ---------------------------------------------------------------------------
// 执行状态机：创建、遥测、停止共用同一套终态保护、原因留存、幂等去重与审计。
// 活动阶段(running)允许的迁移：
//   running -> completed           遥测 event=completed
//   running -> protective-stopped  关节跟踪误差 > 2° 或 TCP 力 > 180N（遥测触发）
//   running -> stopped             人工 stop
// completed / stopped / protective-stopped 均为终态，任何迟到或重复消息不得改写。
// ---------------------------------------------------------------------------
const JOINT_ERROR_LIMIT_DEG=2;
const TCP_FORCE_LIMIT_N=180;
const TERMINAL_STATUSES=new Set(['completed','stopped','protective-stopped']);

function stableStringify(value:any,seen=new WeakMap<object,string>()):string{
  if(value===null||typeof value!=='object')return JSON.stringify(value)??'null';
  const cached=seen.get(value);if(cached)return cached;
  if(Array.isArray(value)){const s='['+value.map(v=>stableStringify(v,seen)).join(',')+']';seen.set(value,s);return s;}
  const s='{'+Object.keys(value).sort().map(k=>JSON.stringify(k)+':'+stableStringify(value[k],seen)).join(',')+'}';seen.set(value,s);return s;
}
function fingerprint(body:any){return createHash('sha256').update(stableStringify(body)).digest('hex');}
// 幂等键：优先使用显式 messageId / Idempotency-Key，否则用规范化请求体指纹，
// 保证“同一遥测重试”天然可去重。
function resolveMessageKey(body:any,headerKey?:string){
  const explicit=(body&&body.messageId)||headerKey;
  if(explicit)return{key:`key:${explicit}`,explicit:true};
  const raw={...body};delete raw.messageId;
  return{key:`hash:${fingerprint(raw)}`,explicit:false};
}

function createExecution(path:any,body:any){
  const e:any={id:randomUUID(),pathId:path.id,mode:body.mode||'dry-run',startedAt:now(),status:'running',progress:0,
    waypointCount:path.waypoints.length,
    safety:{eStop:false,protectiveStop:false,manualStop:false},
    telemetry:[],lateMessages:[]};
  // seen 不对外暴露，仅用于服务端幂等判定
  Object.defineProperty(e,'seen',{value:new Map<string,{firstReceivedAt:string;seq:number}>(),enumerable:false});
  executions.push(e);
  log('execution.start',{executionId:e.id,pathId:e.pathId,mode:e.mode,status:'running'});
  return e;
}

// 阈值评估：关节误差优先，首个触发原因保留，后续原因不得覆盖。
function evaluateProtection(sample:any){
  if(sample.jointErrorDeg!=null&&Number(sample.jointErrorDeg)>JOINT_ERROR_LIMIT_DEG)
    return{reason:'joint-tracking-error' as const,channel:'jointErrorDeg',value:Number(sample.jointErrorDeg),limit:JOINT_ERROR_LIMIT_DEG};
  if(sample.tcpForceN!=null&&Number(sample.tcpForceN)>TCP_FORCE_LIMIT_N)
    return{reason:'tcp-force-exceeded' as const,channel:'tcpForceN',value:Number(sample.tcpForceN),limit:TCP_FORCE_LIMIT_N};
  return null;
}

interface IngestResult{http:number;execution:any;replay?:boolean;late?:boolean;error?:string}
function ingestTelemetry(e:any,body:any,headerKey?:string):IngestResult{
  // 1) 幂等优先：已受理过的同一消息即使在终态后重试，也返回与首次一致的结果
  const{key}=resolveMessageKey(body,headerKey);
  const prior=e.seen.get(key);
  if(prior){
    log('execution.telemetry.duplicate',{executionId:e.id,messageId:body?.messageId??null,firstReceivedAt:prior.firstReceivedAt});
    return{http:200,execution:e,replay:true};
  }
  // 2) 新消息撞上终态：迟到消息只留痕，绝不改写状态/安全标志
  if(TERMINAL_STATUSES.has(e.status)){
    const entry={kind:'telemetry',receivedAt:now(),statusAtReceive:e.status,messageId:body?.messageId??null,body};
    e.lateMessages.push(entry);
    log('execution.telemetry.rejected-late',{executionId:e.id,statusAtReceive:e.status,messageId:body?.messageId??null});
    return{http:409,execution:e,late:true,error:`execution already terminal (${e.status}); late telemetry accepted for audit only`};
  }
  // 3) 受理：原样保留遥测（原始字段全部可查），仅追加受理元数据
  const seq=e.telemetry.length+1;
  const receivedAt=now();
  const sample={seq,receivedAt,messageId:body?.messageId??null,...body};
  e.telemetry.push(sample);
  e.seen.set(key,{firstReceivedAt:receivedAt,seq});
  if(typeof body.waypoint==='number'&&Number.isFinite(body.waypoint))
    e.progress=Math.max(e.progress,Math.min(100,Math.round((body.waypoint+1)/e.waypointCount*100)));
  if(typeof body.progress==='number'&&Number.isFinite(body.progress))
    e.progress=Math.max(e.progress,Math.min(100,Math.round(body.progress)));
  log('execution.telemetry',{executionId:e.id,seq,messageId:sample.messageId,event:body?.event??null});
  // 4) 安全阈值优先：触发保护停机并永久保留首个原因/触发样本
  const trigger=evaluateProtection(body);
  if(trigger){
    e.status='protective-stopped';e.stoppedAt=receivedAt;e.stopReason=trigger.reason;
    e.safety.protectiveStop=true;
    e.stopDetail={source:'telemetry',channel:trigger.channel,value:trigger.value,limit:trigger.limit,triggerSeq:seq,triggerMessageId:sample.messageId};
    log('execution.protective-stop',{executionId:e.id,reason:trigger.reason,detail:e.stopDetail,status:e.status});
    return{http:200,execution:e};
  }
  // 5) 正常完成
  if(body.event==='completed'){
    e.status='completed';e.progress=100;e.finishedAt=receivedAt;
    log('execution.complete',{executionId:e.id,status:'completed',finishedAt:e.finishedAt,telemetryCount:e.telemetry.length});
  }
  return{http:200,execution:e};
}

interface StopResult{http:number;execution:any;replay?:boolean;late?:boolean;error?:string}
function stopExecution(e:any,body:any,headerKey?:string):StopResult{
  // 活动中人工停止：独立终态 stopped + manual-stop 原因，不误标 protectiveStop
  if(e.status==='running'){
    const stoppedAt=now();
    e.status='stopped';e.stoppedAt=stoppedAt;e.stopReason='manual-stop';
    e.safety.manualStop=true;
    e.stopDetail={source:'operator',operator:body?.operator??null,messageId:body?.messageId??headerKey??null};
    log('execution.stop',{executionId:e.id,reason:'manual-stop',detail:e.stopDetail,status:'stopped'});
    return{http:200,execution:e};
  }
  // 已因人工/保护停机：重复 stop 幂等返回同一结果，状态、原因、时间戳不变
  if(e.status==='stopped'||e.status==='protective-stopped'){
    log('execution.stop.duplicate',{executionId:e.id,status:e.status,reason:e.stopReason,messageId:body?.messageId??headerKey??null});
    return{http:200,execution:e,replay:true};
  }
  // 已正常完成后的迟到 stop：留痕审计，终态不可改写
  const entry={kind:'stop',receivedAt:now(),statusAtReceive:e.status,messageId:body?.messageId??headerKey??null,body:body??null};
  e.lateMessages.push(entry);
  log('execution.stop.rejected-late',{executionId:e.id,statusAtReceive:e.status,messageId:entry.messageId});
  return{http:409,execution:e,late:true,error:`execution already terminal (${e.status}); stop refused`};
}

app.get('/health',async()=>({ok:true,service:'industrial-robot-trajectory-validation',time:now()}));
app.get('/api/cells',async()=>cells);app.get('/api/robots',async()=>robots);
app.post('/api/paths',async(req:any,res)=>{const b=req.body as any;if(!b.cellId||!b.robotId||!Array.isArray(b.waypoints)||b.waypoints.length<2)return res.code(400).send({error:'cellId、robotId 和至少两个 waypoints 必填'});const p={id:randomUUID(),version:1,name:b.name||'未命名轨迹',cellId:b.cellId,robotId:b.robotId,toolId:b.toolId||null,waypoints:b.waypoints,process:b.process||{},status:'draft',createdAt:now()};paths.push(p);return p;});
app.post('/api/paths/:id/validate',async(req:any,res)=>{const p=paths.find(x=>x.id===(req.params as any).id);if(!p)return res.code(404).send({error:'path not found'});const result=validatePath(p);const v={id:randomUUID(),pathId:p.id,createdAt:now(),...result};validations.push(v);p.lastValidationId=v.id;p.status=result.valid?'validated':'blocked';log('path.validate',v);return v;});
app.post('/api/paths/:id/plan',async(req:any,res)=>{const p=paths.find(x=>x.id===(req.params as any).id);if(!p)return res.code(404).send({error:'path not found'});const v=validations.filter(x=>x.pathId===p.id).at(-1);if(!v?.valid)return res.code(409).send({error:'path must pass collision validation'});const b=req.body as any;const plan={id:randomUUID(),pathId:p.id,controller:b.controller||'RAPID',posture:b.posture||'elbow-up',blendPolicy:b.blendPolicy||'smooth',samplingMs:Number(b.samplingMs||20),segments:p.waypoints.slice(1).map((w:any,i:number)=>({index:i+1,from:p.waypoints[i],to:w,acceleration:Number((Math.min(2.5,(w.speed||100)/80)).toFixed(2))})),status:'planned',createdAt:now()};plans.push(plan);log('trajectory.plan',plan);return plan;});
app.post('/api/paths/:id/optimize',async(req:any,res)=>{const p=paths.find(x=>x.id===(req.params as any).id);if(!p)return res.code(404).send({error:'path not found'});const candidates=[.75,1,1.2].map(scale=>{const copy={...p,waypoints:p.waypoints.map((w:any)=>({...w,speed:Math.min(220,(w.speed||100)*scale),blend:Math.min(100,(w.blend||10)+scale*5)}))};return{scale,validation:validatePath(copy),waypoints:copy.waypoints}}).filter(x=>x.validation.valid).sort((a,b)=>a.validation.estimatedSeconds-b.validation.estimatedSeconds);return{pathId:p.id,candidates,best:candidates[0]||null};});
app.post('/api/executions',async(req:any,res)=>{
  const b=req.body as any;
  const p=paths.find(x=>x.id===b.pathId);
  if(!p)return res.code(404).send({error:'path not found'});
  const v=validations.filter(x=>x.pathId===p.id).at(-1);
  if(!v?.valid)return res.code(409).send({error:'execution blocked by validation'});
  // 仅创建活动执行：执行通过遥测推进，event=completed 才正常结束
  return createExecution(p,b);
});
app.post('/api/executions/:id/stop',async(req:any,res)=>{
  const e=executions.find(x=>x.id===(req.params as any).id);
  if(!e)return res.code(404).send({error:'execution not found'});
  const r=stopExecution(e,(req.body as any)||{},req.headers['idempotency-key'] as string|undefined);
  if(r.replay)res.header('x-idempotent-replay','true');
  if(r.http!==200)return res.code(r.http).send({error:r.error,status:e.status,executionId:e.id});
  return e;
});
app.get('/api/paths/:id/compare',async(req:any,res)=>{const p=paths.find(x=>x.id===(req.params as any).id);if(!p)return res.code(404).send({error:'path not found'});const versions=paths.filter(x=>x.name===p.name).map(x=>({id:x.id,version:x.version,validation:validations.filter(v=>v.pathId===x.id).at(-1)}));return{pathId:p.id,versions};});
app.get('/api/audit',async()=>audit.slice(-100));

// Advanced operations: calibration, workpiece frames, dynamic obstacles and safety release gates.
const calibrations:any[]=[{id:'cal-01',robotId:'robot-01',toolId:'gun-01',tcpErrorMm:0.42,angleErrorDeg:.18,status:'valid',validUntil:new Date(Date.now()+30*86400000).toISOString()}];
const dynamicObstacles:any[]=[]; const frames:any[]=[{id:'frame-body',name:'车身基准坐标系',origin:{x:0,y:0,z:0},rotation:{rx:0,ry:0,rz:0},status:'verified'}]; const safetyReleases:any[]=[];
function safetyGate(path:any){const v=validations.filter(x=>x.pathId===path.id).at(-1);const cell=cells.find(x=>x.id===path.cellId);const calibration=calibrations.find(x=>x.robotId===path.robotId&&x.toolId===path.toolId);const activeDynamic=dynamicObstacles.filter(x=>x.status==='active'&&x.cellId===path.cellId);const failures:string[]=[];if(calibration && Date.parse(calibration.validUntil)<=Date.now())calibration.status='expired';if(!v?.valid)failures.push('collision-validation');if(!calibration||calibration.status!=='valid')failures.push('tcp-calibration');if(activeDynamic.length)failures.push('dynamic-obstacle-present');if(!cell)failures.push('cell-missing');return{released:failures.length===0,failures,checkedAt:now(),calibrationId:calibration?.id||null};}
app.get('/api/calibrations',async()=>calibrations);
app.post('/api/calibrations',async(req:any,res)=>{const b=req.body as any;if(!b.robotId||!b.toolId||b.tcpErrorMm==null)return res.code(400).send({error:'robotId、toolId、tcpErrorMm 必填'});const c={id:randomUUID(),robotId:b.robotId,toolId:b.toolId,tcpErrorMm:Number(b.tcpErrorMm),angleErrorDeg:Number(b.angleErrorDeg||0),status:Number(b.tcpErrorMm)<=1&&Number(b.angleErrorDeg||0)<=.5?'valid':'invalid',validUntil:new Date(Date.now()+Number(b.validDays||30)*86400000).toISOString(),createdAt:now()};calibrations.push(c);return c;});
app.post('/api/frames',async(req:any,res)=>{const b=req.body as any;if(!b.name||!b.origin)return res.code(400).send({error:'name 和 origin 必填'});const f={id:randomUUID(),name:b.name,origin:b.origin,rotation:b.rotation||{rx:0,ry:0,rz:0},status:'pending-verification',createdAt:now()};frames.push(f);return f;});
app.post('/api/dynamic-obstacles',async(req:any,res)=>{const b=req.body as any;if(!b.cellId||!b.center||!b.size)return res.code(400).send({error:'cellId、center、size 必填'});const o={id:randomUUID(),cellId:b.cellId,center:b.center,size:b.size,source:b.source||'safety-scanner',status:'active',reportedAt:now()};dynamicObstacles.push(o);return o;});
app.post('/api/dynamic-obstacles/:id/clear',async(req:any,res)=>{const o=dynamicObstacles.find(x=>x.id===(req.params as any).id);if(!o)return res.code(404).send({error:'dynamic obstacle not found'});o.status='cleared';o.clearedAt=now();return o;});
app.post('/api/paths/:id/release',async(req:any,res)=>{const p=paths.find(x=>x.id===(req.params as any).id);if(!p)return res.code(404).send({error:'path not found'});const gate=safetyGate(p);const release={id:randomUUID(),pathId:p.id,gate,status:gate.released?'released':'blocked',operator:(req.body as any)?.operator||'cell-supervisor',createdAt:now()};safetyReleases.push(release);if(gate.released)p.status='released';log('path.release',release);return release;});
app.post('/api/executions/:id/telemetry',async(req:any,res)=>{
  const e=executions.find(x=>x.id===(req.params as any).id);
  if(!e)return res.code(404).send({error:'execution not found'});
  const b=req.body as any;
  if(!b||typeof b!=='object'||Array.isArray(b))return res.code(400).send({error:'telemetry body must be a JSON object'});
  const r=ingestTelemetry(e,b,req.headers['idempotency-key'] as string|undefined);
  if(r.replay)res.header('x-idempotent-replay','true');
  if(r.http!==200)return res.code(r.http).send({error:r.error,status:e.status,executionId:e.id});
  return e;
});
app.get('/api/executions',async()=>executions);
app.get('/api/executions/:id',async(req:any,res)=>{
  const e=executions.find(x=>x.id===(req.params as any).id);
  if(!e)return res.code(404).send({error:'execution not found'});
  return e;
});
// 原始遥测查询：返回执行期内实际受理的全部原始样本，以及终态后被拒绝的迟到消息
app.get('/api/executions/:id/telemetry',async(req:any,res)=>{
  const e=executions.find(x=>x.id===(req.params as any).id);
  if(!e)return res.code(404).send({error:'execution not found'});
  return{executionId:e.id,status:e.status,count:e.telemetry.length,telemetry:e.telemetry,lateMessages:e.lateMessages};
});
app.get('/api/safety/releases',async()=>safetyReleases);app.get('/api/dynamic-obstacles',async()=>dynamicObstacles);

// 仅在直接启动时监听端口；被测试模块 import 时不占用端口
const entry=process.argv[1]?pathToFileURL(process.argv[1]).href:'';
if(entry===import.meta.url){
  app.listen({port:Number(process.env.PORT||6303),host:'127.0.0.1'}).catch(err=>{app.log.error(err);process.exit(1)});
}
export {app,executions,audit,JOINT_ERROR_LIMIT_DEG,TCP_FORCE_LIMIT_N};
