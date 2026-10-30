/**
 * DriveJev interaction executor (observation schema 1.1: perception with vehicle occlusion) + observable
 * world-rollout teacher.
 *
 * The behaviour set, controller and candidate rules are the base ones (SemanticExecutor, unchanged). Schema 1.1 adds:
 *  - perception: vehicles occlude pedestrians and other vehicles (JevPilot's scan only models buildings);
 *    a parked car can now hide a pedestrian from the sensors as it does in the camera images;
 *  - two perception-derived junction fields: `oncoming_eta_s` / `cross_eta_s` = time until the nearest
 *    perceived oncoming / crossing vehicle reaches the junction box (constant-velocity estimate);
 *  - `obsSchema: '1.0'` reproduces the schema-1.0 student observation exactly (for evaluating schema-1.0 heads);
 *  - observableTeacher: (1) rollouts only contain road users the student's sensors have seen in the last 1 s
 *    (or that are within 8 m), so every label is explainable from the student inputs; (2) a collision is
 *    only excused when the ego is rear-ended (the base teacher excused every contact at ego speed <= 0.4 m/s, which let
 *    the teacher wait inside an oncoming lane); (3) 4.5 s horizon for gap acceptance from standstill;
 *    (4) a behaviour is only "safe" if the ego never enters the 0.6 s zone in front of a moving vehicle that is
 *    not following it (gap acceptance with a time margin instead of a 0.25 m footprint clearance);
 *    (5) when no stopping behaviour is collision-free, a motion that only lacks margin beats a colliding stop.
 */
import { angle, clamp, dist, heading } from '../third_party/jevpilot/src/math.js';
import { stopLineDistance } from '../third_party/jevpilot/src/planning.js';
import { leadVehicle, predictTrafficConflict, footprintClearance, relativeTrafficState } from '../third_party/jevpilot/src/traffic-safety.js';
import { SemanticExecutor, ACTIONS, DT, maneuverTarget, geometricLimit } from './executor.mjs';

export const VERSION = 'drivejev-exec-1.1';
export { ACTIONS, DT };
const r1 = (x) => (Number.isFinite(x) ? Math.round(x * 10) / 10 : null);

/** Does segment a->b pass through the oriented footprint of `box` (shrunk by `pad`)? */
export function segmentHitsBox(a, b, box, pad = -0.1) {
  const sin = Math.sin(box.heading || 0), cos = Math.cos(box.heading || 0);
  const loc = (p) => { const dx = p.x - box.x, dz = p.z - box.z; return [dx * sin - dz * cos, dx * cos + dz * sin]; };
  const A = loc(a), B = loc(b), half = [(box.depth || 4.2) / 2 + pad, (box.width || 1.9) / 2 + pad];
  let tmin = 0.02, tmax = 0.98;
  for (let k = 0; k < 2; k++) {
    const d = B[k] - A[k];
    if (Math.abs(d) < 1e-9) { if (A[k] < -half[k] || A[k] > half[k]) return false; continue; }
    let t1 = (-half[k] - A[k]) / d, t2 = (half[k] - A[k]) / d;
    if (t1 > t2) [t1, t2] = [t2, t1];
    tmin = Math.max(tmin, t1); tmax = Math.min(tmax, t2);
    if (tmax < tmin) return false;
  }
  return true;
}

export class InteractionExecutor extends SemanticExecutor {
  constructor(sim, { aeb = false, obsSchema = '1.1' } = {}) {
    super(sim, { aeb });
    this.obsSchema = obsSchema;
    this.vehicleOcclusion = obsSchema !== '1.0';
    this.seen = new Map();
    this.percCache = null;
  }

  eye() { const v = this.sim.player; return { x: v.x + Math.sin(v.heading) * 0.15, z: v.z - Math.cos(v.heading) * 0.15 }; }

  visible(o, occluders, eye) {
    const d = dist(o, eye);
    if (d < 6) return true;
    const blockers = occluders.filter((b) => b !== o && b.id !== o.id && dist(b, eye) < d);
    if (!blockers.length) return true;
    const hidden = (p) => blockers.some((b) => segmentHitsBox(eye, p, b));
    if (o.type === 'pedestrian') return !hidden(o);
    const f = 0.4 * (o.depth || 4.2), s = Math.sin(o.heading || 0), c = Math.cos(o.heading || 0);
    return !(hidden(o) && hidden({ x: o.x + s * f, z: o.z - c * f }) && hidden({ x: o.x - s * f, z: o.z + c * f }));
  }

  perceived() {
    const base = super.perceived();
    if (!this.vehicleOcclusion) { this.markSeen(base); return base; }
    if (this.percCache?.step === this.stepId && this.percCache.time === this.sim.time) return this.percCache.value;
    const eye = this.eye();
    const occluders = base.vehicles.filter((o) => o.type === 'car' && dist(o, eye) < 70);
    const value = { vehicles: base.vehicles.filter((o) => this.visible(o, occluders, eye)), pedestrians: base.pedestrians.filter((o) => this.visible(o, occluders, eye)) };
    this.percCache = { step: this.stepId, time: this.sim.time, value };
    this.markSeen(value);
    return value;
  }
  markSeen(p) { const t = this.sim.time; for (const o of p.vehicles) this.seen.set(o.id, t); for (const o of p.pedestrians) this.seen.set(o.id, t); }

  junctionSummary(c, p) {
    const base = super.junctionSummary(c, p);
    if (!base || this.obsSchema === '1.0') return base;
    const node = this.sim.world.byId[c.nodeId];
    let oncoming = Infinity, cross = Infinity;
    for (const o of p.vehicles) {
      const d = dist(o, node);
      if (d < 11 || d > 75) continue;
      const toNode = heading(o, node), closing = Math.cos(angle((o.heading ?? 0) - toNode)) * (o.speed ?? 0);
      if (closing < 0.5 || Math.cos(angle((o.heading ?? 0) - toNode)) < 0.8) continue;
      const eta = (d - 11) / closing, rel = angle((o.heading ?? 0) - c.approach);
      if (Math.cos(rel) < -0.8) oncoming = Math.min(oncoming, eta);
      else if (Math.abs(Math.sin(rel)) > 0.8) cross = Math.min(cross, eta);
    }
    return { ...base, oncoming_eta_s: oncoming < 12 ? r1(oncoming) : null, cross_eta_s: cross < 12 ? r1(cross) : null };
  }

  /** Evaluation only: stationary although the rule allows motion and nothing perceived blocks or conflicts. */
  unnecessaryStop() {
    const sim = this.sim, v = sim.player;
    if (Math.abs(v.speed) >= 0.2 || this.stepId <= 20 || sim.rule(v).mustStop) return false;
    const env = this.env();
    if (env.lead && env.lead.gap < 4) return false;
    const c = this.predictConflict(env.perceived);
    return !c;
  }
}

// ------------------------------------------------------------------------------------
// Observable privileged teacher (see header).
// ------------------------------------------------------------------------------------
const HORIZON_STEPS = 90; // 4.5 s
const SEEN_WINDOW_S = 1.0, NEAR_M = 8;

function observableClone(ex) {
  const sim = ex.sim.clone(), v = sim.player, t = ex.sim.time;
  // observableClone的判断是留下状态为hazard为parked的对象，或者距离在8m内的对象，或者过去1s内见到过的对象
  // ex.seen 是模拟器的ego每次感知会刷新对象的感知时间，每0.2s刷新一次，超过1s没见到的对象就会被过滤掉
  const keep = (o) => (o.hazard && o.hazard.state === 'parked') || dist(o, v) < NEAR_M || (ex.seen.get(o.id) ?? -Infinity) >= t - SEEN_WINDOW_S;
  sim.traffic = sim.traffic.filter(keep);
  sim.pedestrians = sim.pedestrians.filter(keep);
  sim.hazardPeds = sim.pedestrians.filter((p) => p.hazard);
  sim.hazardCars = sim.traffic.filter((o) => o.hazard);
  return sim;
}

function rearEnded(v, other) {
  if (!other || !['car', 'motorcycle'].includes(other.type)) return false;
  const rel = relativeTrafficState(v, other);
  return rel.ahead_m < 0 && Math.abs(rel.heading_relative_deg) < 50;
}

/** The zone a moving vehicle will cover in the next 0.6 s (its footprint stretched forward). */
function frontZone(o) {
  const b = clamp((o.speed || 0) * 0.6, 0, 8);
  if (b < 0.3) return null;
  const s = Math.sin(o.heading || 0), c = Math.cos(o.heading || 0);
  return { ...o, x: o.x + (s * b) / 2, z: o.z - (c * b) / 2, depth: (o.depth || 4.2) + b };
}

function rollout(ex, candidate) {
  // 从当前仿真状态创建副本，但只保留可观测 Teacher 允许使用的交通对象；后续模拟不会改变真实仿真。
  const sim = observableClone(ex), v = sim.player;
  // 构造本次要评估的动作；continue_current 复用当前 maneuver，stop_at_line 绑定当前停止线。
  const m = candidate.action_type === 'continue_current' && ex.maneuver ? { ...ex.maneuver, feedback: [] }
    : { action_type: candidate.action_type, line: candidate.action_type === 'stop_at_line' ? ex.line() : null, exit_s: null };
  // 记录起点状态，供 rollout 结束时计算增量违规、行驶进度和速度变化。
  const startS = v.s, startViol = sim.violations, startSpeed = v.speed;
  // 累积碰撞、最大减速度、最小安全间距，以及车辆前方区域被侵入等评估结果。
  let collision = null, maxDecel = 0, minGap = Infinity, prev = v.speed, stoppedAt = null, intrusion = null;
  // filter是保留满足条件的对象，返回一个新数组；这里保留非hazard或hazard不为parked的交通对象和行人。
  const dyn = () => [...sim.traffic.filter((o) => !o.hazard || o.hazard.state !== 'parked'), ...sim.pedestrians.filter((o) => !o.hazard || o.hazard.state !== 'parked')];
  // 按 DT=0.05 秒逐步执行候选动作；HORIZON_STEPS=90，因此最长预测 4.5 秒。
  for (let k = 0; k < HORIZON_STEPS; k++) {
    // 更新自车的车道保持预瞄参数，并计算当前前车，用于跟车限速。
    v.maneuver = { lane_offset_m: 0, lookahead_m: clamp(3 + Math.abs(v.speed) * 0.45, 3, 10) };
    // 计算前车；若前车距离小于 80 m，则传给 maneuverTarget() 用于限速。
    const lead = leadVehicle(v, sim.traffic);
    let conflict = null;
    // 只有 yield_agent 会额外预测路径冲突，并据此生成逐渐减速、停车的目标速度。
    if (m.action_type === 'yield_agent') {
      // geometricLimit(sim, v) 计算自车当前速度、路线限速和路线终点距离的几何约束，作为 yield_agent 的目标速度上限。
      const ghost = { ...v, target: geometricLimit(sim, v), maneuver: v.maneuver };
      // predictTrafficConflict() 预测自车与前方交通对象的路径冲突；若有冲突且减速可降低风险，则返回该冲突对象，否则返回 null。
      const c = predictTrafficConflict(ghost, dyn().filter((o) => dist(o, v) < 70));
      conflict = c && c.braking_reduces_risk ? c : null;
    }
    // 根据候选动作、前车和（如有）冲突计算目标速度，再让仿真物理模型推进一步。
    v.target = maneuverTarget(sim, v, m, { lead: lead && lead.gap < 80 ? lead : null, conflict }).target;
    sim.autopilot = true; sim.safety = false;
    sim.step(DT);
    // 若已在停止线前停稳，标记停车动作完成；同时记录减速度和首次停稳时间。
    if (m.action_type === 'stop_at_line' && v.speed < 0.05 && m.line && stopLineDistance(v, m.line) <= 0.65) m.status = 'completed';
    maxDecel = Math.max(maxDecel, (prev - v.speed) / DT); prev = v.speed;
    if (stoppedAt === null && v.speed < 0.05) stoppedAt = k * DT;
    // 仿真发生碰撞即记录碰撞对象、时间和是否属于后车追尾，并结束本候选的模拟。
    if (sim.crash) {
      const other = [...sim.traffic, ...sim.pedestrians].find((o) => o.id === sim.crash.object_id);
      collision = { ...sim.crash, at_s: (k + 1) * DT, rear_end: rearEnded(v, other) };
      break;
    }
    // 更新近距离最小车身间距；另检查非行人、非后车追尾对象的前方 0.6 秒占用区是否侵入自车空间。
    for (const o of dyn()) {
      const d = dist(o, v);
      // footprintClearance计算：
      // 分离量 = 两矩形中心在该方向上的距离
      //  - 两个矩形在该方向上的投影半径之和
      // 其中投影半径由矩形的长、宽及朝向算出。函数取 4 个方向中最大的分离量：

      // - 结果大于 0：矩形分开；
      // - 等于 0：接触；
      // - 小于 0：矩形重叠，负数表示 SAT 方向上的重叠深度
      if (d < 9) minGap = Math.min(minGap, footprintClearance(v, o) - (o.type === 'pedestrian' ? 0.05 : 0.02) * Math.abs(v.speed));
      if (intrusion === null && d < 20 && o.type !== 'pedestrian' && !rearEnded(v, o)) {
        const z = frontZone(o);
        if (z && footprintClearance(v, z) < 0) intrusion = { id: o.id, at_s: (k + 1) * DT };
      }
    }
    // 到达终点后无需继续模拟。
    if (sim.complete) break;
  }
  // 后车追尾不记为自车责任碰撞；返回 Teacher 后续排序/筛选要用的仿真指标。
  const atFault = collision ? !collision.rear_end : false;
  return {
    // collision 表示自车是否负有责任；collision_any 记录是否发生任何碰撞（包括后车追尾）。
    collision: atFault, collision_any: !!collision, collision_at_s: collision?.at_s ?? null, collision_type: collision?.type ?? null, collision_speed_mps: collision?.player_speed_mps ?? null,
    // collision_at_s 是 rollout 内碰撞发生时间（秒）；collision_type 是对象类型；collision_speed_mps 是碰撞时自车速度。
    // violation 表示 rollout 是否新增交通违规；progress_m 是本次行驶距离；final_speed_mps 是结束速度。
    // max_decel_mps2 是最大减速度；min_clearance_m 是近距离对象的最小调整后车身间距；start_speed_mps 是开始速度。
    violation: sim.violations > startViol, progress_m: v.s - startS, final_speed_mps: v.speed, max_decel_mps2: maxDecel, min_clearance_m: minGap, start_speed_mps: startSpeed,
    // intrusion 表示是否检测到车辆前方区域侵入自车车身；intrusion_at_s 是首次发生时间（秒）。
    intrusion: !!intrusion, intrusion_at_s: intrusion?.at_s ?? null
  };
}

export function observableTeacher(ex, obs = ex.observe()) {
  // 读取当前路口规则、停止线距离、导航方向和可选动作；每个候选只 rollout 一次。
  // sim 是仿真世界，v 是自车，ids 是本次观测提供的候选动作 ID。
  const sim = ex.sim, v = sim.player, ids = obs.candidates.map((x) => x.candidate_id);
  // c 来自自车路线的 crossings：取第一个 stopS - v.s > -19 的路口；无符合项时为 null。
  const c = ex.crossing();
  // 查询自车在当前路口的规则：根据灯色及黄灯可停车性、停车标志是否已服务、路口先行/占用状态等计算 mustStop 和 reason。
  // 此处未传 update=true，因此只读取规则，不更新停车记忆；无受控路口时返回 Clear road。
  const rule = sim.rule(v);
  // 有 crossing 时，字段是：{
  //   mustStop,       // 当前是否必须停车
  //   distance,       // stopS - v.s，沿路线计算的距离差
  //   reason,         // 判定原因
  //   color,          // 信号灯颜色或 "stop"
  //   nodeId,         // 路口 ID
  //   stopCompleted,  // 停车要求是否已完成
  //   walk            // 行人信号状态
  // }
  // 没有 crossing 时返回：{
  //   mustStop: false,
  //   distance: Infinity,
  //   reason: "Clear road",
  //   color: null
  // }
  const has = (id) => ids.includes(id), nav = obs.student_obs.nav, front = c ? ex.frontDistance(c) : null;
  //  nav = {
  //   remaining_m,          // 距终点剩余距离
  //   next_turn,             // 下一条导航指令
  //   turn_distance_m,       // 到下一导航动作的距离
  //   speed_limit_mps,       // 当前适用限速
  //   junction_control,      // 路口控制类型，如 signal、stop；没有则为 null
  //   stop_line_ahead_m,     // 车头到停止线的有符号距离；无 crossing 则为 null
  //   stop_completed         // 当前路口的停车要求是否已完成
  // }
  const moving = Math.abs(v.speed) >= 0.5;
  const scores = {};
  const score = (id) => (scores[id] ??= rollout(ex, obs.candidates.find((x) => x.candidate_id === id)));
  // score(id) 返回 rollout() 对候选动作的评估对象，不是单个数值。字段如下：
  // - collision：是否发生自车有责任的碰撞。
  // - collision_any：是否发生任何碰撞，包括非自车责任的碰撞。
  // - collision_at_s、collision_type、collision_speed_mps：碰撞时间、对象类型和自车碰撞时速度。
  // - violation：模拟期间是否新增交通规则违规。
  // - progress_m、final_speed_mps：模拟期间路线进度和结束速度。
  // - start_speed_mps：开始模拟时的速度。
  // - max_decel_mps2：模拟期间最大减速度。
  // - min_clearance_m：与附近交通参与者的最小间距。
  // - intrusion、intrusion_at_s：是否有移动车辆的前方区域侵入自车空间，以及发生时间。

  const node = c ? sim.world.byId[c.nodeId] : null;
  // 当前为绿灯时，未来灯色对学生不可见；不能仅因 rollout 中之后转红就判当前动作违法。
  const futureSignalUnknown = node?.control === 'signal' && rule.color === 'green';
  const legal = (s) => !s.violation || futureSignalUnknown;
  // safe 是完整安全门槛；clean 仅检查责任碰撞和规则，供无安全候选时降级使用。
  const safe = (id) => { const s = score(id); return !s.collision && legal(s) && s.min_clearance_m > 0.25 && !s.intrusion; };
  const clean = (id) => { const s = score(id); return !s.collision && legal(s); };
  let ruleStop = rule.mustStop;
  // 已完成停车且占用路口的车辆已驶离 12 m 以上时，解除这项过期的让行等待。
  if (ruleStop && rule.reason === 'Yield to crossing traffic' && rule.stopCompleted && node) {
    const lock = sim.locks.get(node.id), holder = lock ? sim.traffic.find((o) => o.id === lock.id) : null;
    if (holder && dist(holder, node) > 12) ruleStop = false;
  }
  // 仅在停止线位于车头后 0.3 m 到前方 85 m 范围内时应用停车规则。
  const mustStop = c && ruleStop && front !== null && front > -0.3 && front < 85;
  let preferred = null, reason = null;
  if (mustStop) {
    // 已停在线前就保持停车；否则优先停车线动作，动作不可安全执行时尝试让行，再紧急制动。
    const atLine = !moving && front <= 1.0;
    preferred = atLine ? 'hold_stop' : has('stop_at_line') ? 'stop_at_line' : !moving ? 'hold_stop' : 'emergency_brake';
    reason = `rule:${rule.reason}`;
    if (preferred === 'stop_at_line' && !safe('stop_at_line')) {
      preferred = has('yield_agent') && safe('yield_agent') ? 'yield_agent' : 'emergency_brake';
      reason += score('stop_at_line').collision ? '+collision' : '+clearance';
    }
  } else {
    // 无强制停车规则时，静止则起步；行驶中若 24 m 内有转弯则转弯，否则沿路线巡航。
    const motion = !moving ? 'proceed_route' : has('route_turn') && ['left', 'right'].includes(nav.next_turn) && nav.turn_distance_m < 24 ? 'route_turn' : 'keep_route_cruise';
    if (safe(motion)) { preferred = motion; reason = 'clear'; }
    else {
      const sm = score(motion);
      reason = `conflict:${sm.collision ? 'collision' : !legal(sm) ? 'violation' : sm.intrusion ? 'gap' : 'clearance'}`;
      // 主动作不安全时依次找安全停车动作；若没有，再退到无责任碰撞且合法的动作。
      const stops = (moving ? ['yield_agent', 'stop_at_line', 'emergency_brake'] : ['hold_stop']).filter(has);
      preferred = stops.find(safe) ?? stops.find(clean) ?? (clean(motion) ? motion : null);
      if (preferred === 'emergency_brake' && has('yield_agent') && clean('yield_agent')) preferred = 'yield_agent';
      if (!preferred) {
        // 没有安全动作或无碰撞且合法的回退动作时，先降低碰撞速度；相同时选择更晚接触。
        const harm = (id) => [score(id).collision_speed_mps ?? 0, -(score(id).collision_at_s ?? 99)];
        preferred = ids.filter((id) => id !== 'continue_current').sort((a, b) => harm(a)[0] - harm(b)[0] || harm(a)[1] - harm(b)[1])[0];
        reason += '+unavoidable';
      }
    }
  }
  // 防止首选动作因状态变化而不在当前候选中，确保返回一个可执行候选。
  if (!has(preferred)) preferred = has('hold_stop') ? 'hold_stop' : 'emergency_brake';
  const ref = score(preferred);
  // 将同类且 rollout 结果近似相同的动作纳入软标签；这一步不改变首选动作。
  // 这段在构造 Teacher 的软标签概率分布。它不会改变前面选出的 preferred，而是找出哪些候选动作在这次 rollout 中与首选动作结果近似。
  // 对每个候选 id，筛选逻辑是：
  // 1. id === preferred：首选动作一定纳入。
  // 2. 若 id 是 emergency_brake、而首选不是它，则排除。
  // 3. id 和 preferred 必须出现在同一个 equivalentTypes 分组中。
  // 4. 若 id 是 continue_current，还要求当前确实有正在执行的 maneuver，且它不是 yield_agent。
  // 5. 执行该候选的 rollout；结果必须无碰撞、合法，并且路线进度与首选相差小于 0.3 m、最终速度相差小于 0.2 m/s。
  // 通过筛选的动作组成 equivalent。然后把概率平均分给这些动作，其余候选概率为 0：
  const equivalent = ids.filter((id) => {
    if (id === preferred) return true;
    if (id === 'emergency_brake' && preferred !== 'emergency_brake') return false;
    const equivalentTypes = [['keep_route_cruise', 'route_turn', 'continue_current'], ['proceed_route', 'route_turn', 'continue_current'], ['stop_at_line', 'continue_current'], ['hold_stop', 'stop_at_line', 'yield_agent', 'emergency_brake']];
    if (!equivalentTypes.some((g) => g.includes(id) && g.includes(preferred))) return false;
    if (id === 'continue_current' && !(ex.maneuver && ex.maneuver.action_type !== 'yield_agent')) return false;
    const s = score(id);
    return !s.collision && legal(s) && Math.abs(s.progress_m - ref.progress_m) < 0.3 && Math.abs(s.final_speed_mps - ref.final_speed_mps) < 0.2;
  });
  const probabilities = Object.fromEntries(ids.map((id) => [id, equivalent.includes(id) ? 1 / equivalent.length : 0]));
  // 输出首选动作、原因、软标签概率、各 rollout 分数及路口规则诊断。
  //   这个返回对象汇总了 Teacher 的首选动作、选择原因、等价动作概率和评估信息：

  // - preferred：Teacher 最终选中的语义动作 ID。
  // - reason：Teacher 选它的原因，例如规则要求停车、没有冲突或候选动作存在冲突。
  // - probabilities：候选动作的软标签概率；等价动作平分概率，其余为 0。
  // - acceptable_set：rollout 结果与首选动作足够接近、被纳入软标签的动作 ID 集合。
  // - scores：已进行 rollout 评估的候选动作结果，如碰撞、违规、进度、最终速度。它是按需计算的，不保证包含所有候选。
  // - rule：当前路口的规则诊断：
  //   - must_stop：sim.rule(v) 原始判断的停车要求。
  //   - effective_must_stop：Teacher 实际用于本次分支判断的停车要求；可能经过停止线距离、过期路口占用等修正。
  //   - reason：规则判断原因。
  //   - color：信号灯状态。
  //   - stop_completed：停车要求是否已完成。
  // - teacher_version：Teacher 逻辑版本标识。
  // - horizon_s：每个候选动作的 rollout 时长；当前为 90 × 0.05 = 4.5 秒。
  return { preferred, reason, probabilities, acceptable_set: equivalent, scores, rule: { must_stop: rule.mustStop, effective_must_stop: !!mustStop, reason: rule.reason, color: rule.color, stop_completed: rule.stopCompleted }, teacher_version: 'observable-rollout-teacher-1.1', horizon_s: HORIZON_STEPS * DT };
}
