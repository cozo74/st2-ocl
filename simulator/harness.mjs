/**
 * DriveJev headless harness (one page = one worker). Episodes run inside the page with fixed
 * 0.05 s physics, 4 Hz capture/decision slots and a history frame exactly 10 steps back.
 *   policy 'model'   : DriveJev behind /predict
 *   policy 'teacher' : privileged observable teacher (upper bound, no cameras)
 * World: InteractionWorld (the base world + multi-agent interaction scenarios).
 * cfg.obs_schema '1.1' (default) or '1.0' (schema-1.0 heads: no vehicle occlusion, no junction ETA fields);
 * cfg.legacyHazards replays the base suites exactly (base hazard director and agent pool).
 *   runEpisode       : batch closed loop (the world waits for each answer)
 *   runRealtime      : real-time closed loop (the world never waits)
 */
import * as THREE from 'three';
import { DriveScene } from '/third_party/jevpilot/src/scene.js';
import { renderProfile } from '/third_party/jevpilot/src/render-profile.js';
import { InteractionWorld, WORLD_VERSION } from './interactions.mjs';
import { InteractionExecutor, observableTeacher, checkCandidateSafety, DT, VERSION } from './interaction_executor.mjs';

// JevPilot render profile used for every model frame.
Object.assign(renderProfile, { pixelRatio: 1, antialias: false, shadowSize: 512, detailedFoliage: false, leafCards: 8 });
export const CAMERA = {
  wide: { version: 'ego-front-v4-vfov90-pitch10', width: 640, height: 384, vfov: 90, pitch: 10 },
  tele: { version: 'ego-tele-v7-vfov15-pitch2', width: 384, height: 224, vfov: 15, pitch: 2 },
};
const canvas = document.querySelector('#sensor');
const tele2d = Object.assign(document.createElement('canvas'), { width: CAMERA.tele.width, height: CAMERA.tele.height });
const teleCtx = tele2d.getContext('2d');
const wideCam = new THREE.PerspectiveCamera(CAMERA.wide.vfov, CAMERA.wide.width / CAMERA.wide.height, 0.1, 600);
const teleCam = new THREE.PerspectiveCamera(CAMERA.tele.vfov, CAMERA.tele.width / CAMERA.tele.height, 0.5, 900);
let sim, ex, scene, leaf, config;

function aim(cam, v, pitchDeg) {
  cam.position.set(v.x + Math.sin(v.heading) * 0.15, 1.6, v.z - Math.cos(v.heading) * 0.15);
  cam.lookAt(v.x + Math.sin(v.heading) * 30, 1.6 + 30 * Math.tan((pitchDeg * Math.PI) / 180), v.z - Math.cos(v.heading) * 30);
}

/** Render the wide and tele model views (ego car and annotations hidden). Returns base64 PNGs. */
export function capture() {
  scene.render(0, false);
  scene.vectors.group.visible = false; scene.vectorLayer.hidden = true; scene.sensorCone.visible = false; scene.destination.visible = false; scene.player.visible = false;
  const v = sim.player, r = scene.renderer;
  aim(wideCam, v, CAMERA.wide.pitch);
  r.setScissorTest(false); r.setViewport(0, 0, CAMERA.wide.width, CAMERA.wide.height);
  r.render(scene.scene, wideCam);
  const wide = canvas.toDataURL('image/png');
  aim(teleCam, v, CAMERA.tele.pitch);
  r.setViewport(0, 0, CAMERA.tele.width, CAMERA.tele.height); r.setScissor(0, 0, CAMERA.tele.width, CAMERA.tele.height); r.setScissorTest(true);
  r.render(scene.scene, teleCam);
  teleCtx.drawImage(canvas, 0, CAMERA.wide.height - CAMERA.tele.height, CAMERA.tele.width, CAMERA.tele.height, 0, 0, CAMERA.tele.width, CAMERA.tele.height);
  const tele = tele2d.toDataURL('image/png');
  r.setScissorTest(false); r.setViewport(0, 0, CAMERA.wide.width, CAMERA.wide.height);
  scene.player.visible = true;
  const b64 = (d) => d.slice(d.indexOf(',') + 1);
  return { wide: b64(wide), tele: b64(tele) };
}

export async function reset(cfg) {
  config = { seed: 420000, type: 'town', hazards: [], randomRoute: true, trafficScale: 1, empty: false, aeb: false, safePass: true, obs_schema: '1.1', legacyHazards: false, ...cfg };
  config.obs_schema = { 1: '1.0', '1': '1.0', 1.1: '1.1' }[config.obs_schema] ?? String(config.obs_schema); // `--set obs_schema=1.0` arrives as a number
  sim = new InteractionWorld(config.seed, config.type, { randomRoute: config.randomRoute && config.type !== 'highway', hazards: config.hazards, trafficScale: config.trafficScale,
    empty: config.empty, hazardCooldown: config.hazardCooldown, legacyHazards: config.legacyHazards });
  ex = new InteractionExecutor(sim, { aeb: config.aeb, safePass: config.safePass, obsSchema: config.obs_schema });
  ex.episodeId = config.episode_id ?? `drivejev-${config.type}-${config.seed}`;
  if (leaf) { leaf.dispose(); leaf = null; }
  if (scene) { scene.sim = sim; scene.build(); } else scene = new DriveScene(canvas, sim, document.querySelector('#vector-layer'));
  // A fresh leaf material per world (three.js caches uniforms per material).
  const template = scene.vegetation.leafMaterial;
  leaf = template.clone(); scene.vegetation.addWind(leaf, 0.045); scene.vegetation.leafMaterial = leaf;
  scene.scene.traverse((o) => { if (o.material === template) o.material = leaf; else if (Array.isArray(o.material)) o.material = o.material.map((m) => (m === template ? leaf : m)); });
  scene.mode = 'hood'; scene.showSensors = false; scene.renderer.setPixelRatio(1); scene.renderer.shadowMap.enabled = false;
  await scene.ready;
  // Bootstrap until the first two-frame decision: cruise when moving, hold when stopped.
  ex.select(sim.player.speed >= 0.5 ? 'keep_route_cruise' : 'hold_stop'); ex.policySource = 'local_bootstrap';
  return { world: WORLD_VERSION, executor: VERSION, obs_schema: config.obs_schema, route_length_m: sim.player.route.length, crossings: sim.player.route.crossings.map((c) => ({ node: c.nodeId, control: sim.world.byId[c.nodeId].control })) };
}

const post = (url, body) => fetch(url, { method: 'POST', headers: { 'content-type': 'application/json' }, body: JSON.stringify(body) });
const requestBody = (cfg, obs, history, shot) => ({
  arm: cfg.arm, schema_version: 'drivejev-online-1.1', episode_id: ex.episodeId, observation_id: obs.observation_id,
  student_obs: obs.student_obs, candidates: obs.candidates,
  images: [{ camera: 'front', relative_time: -0.5, png_base64: history.wide }, { camera: 'front', relative_time: 0, png_base64: shot.wide }, { camera: 'front_tele', relative_time: 0, png_base64: shot.tele }],
});
const stuckStep = () => Math.abs(sim.player.speed) < 0.2 && ex.stepId > 20 && !sim.rule(sim.player).mustStop && !(ex.env().lead && ex.env().lead.gap < 4);
/** Collision partner's hazard kind (null for ordinary JevPilot traffic). */
function crashHazard() {
  if (!sim.crash) return null;
  const o = [...sim.traffic, ...sim.pedestrians].find((x) => x.id === sim.crash.object_id);
  return o?.hazard ? (o.hazard.kind ?? 'hazard') : null;
}

const recordRound = (x, digits = 2) => Number.isFinite(x) ? Math.round(x * 10 ** digits) / 10 ** digits : null;
const recordMs = (x) => recordRound(x, 3);
function safetyTimingMetrics(perDecisionMs, rolloutCount) {
  const sorted = [...perDecisionMs].sort((a, b) => a - b);
  const total = perDecisionMs.reduce((sum, value) => sum + value, 0);
  const at = (q) => sorted.length ? recordMs(sorted[Math.min(sorted.length - 1, Math.floor(sorted.length * q))]) : null;
  return {
    safety_gate_decisions: sorted.length,
    safety_check_rollouts: rolloutCount,
    safety_check_ms_total: recordMs(total),
    safety_check_ms_mean_per_rollout: rolloutCount ? recordMs(total / rolloutCount) : null,
    safety_check_ms_p50_per_decision: at(0.5),
    safety_check_ms_p95_per_decision: at(0.95),
  };
}
function egoSnapshot(speedOverride = null) {
  const v = sim.player;
  return { x_m: recordRound(v.x), z_m: recordRound(v.z), heading_rad: recordRound(v.heading),
    heading_deg: recordRound(v.heading * 180 / Math.PI, 1), speed_mps: recordRound(speedOverride ?? v.speed),
    steering: recordRound(v.steering), target_speed_mps: recordRound(v.target), route_s_m: recordRound(v.s) };
}
function recordBehaviorInterval(rows, selectedCandidate, accepted, source) {
  const now = recordRound(sim.time);
  if (rows.length) rows[rows.length - 1].interval_end_s = now;
  rows.push({ interval_start_s: now, interval_end_s: null, step_id: ex.stepId, source,
    selected_candidate_id: selectedCandidate, accepted,
    maneuver_id: ex.maneuver?.maneuver_id ?? null, action_type: ex.maneuver?.action_type ?? null,
    maneuver_status: ex.maneuver?.status ?? null, target_id: ex.maneuver?.target_id ?? null, ego: egoSnapshot() });
}
function createIncidentLog() {
  const decisions = [], imageFrames = new Map(), events = [];
  let violationsSeen = 0, frontEventsSeen = 0, collisionSeen = false;
  const rememberDecision = ({ obs, shot, history, result, acceptedResult, source, latencyMs, state }) => {
    const frameId = shot ? `step-${obs.meta.step_id}` : null, historyFrameId = history ? `step-${history.step}` : null;
    if (shot) imageFrames.set(frameId, { frame_id: frameId, sim_time_s: obs.meta.sim_time, wide_png_base64: shot.wide, tele_png_base64: shot.tele });
    if (history) imageFrames.set(historyFrameId, { frame_id: historyFrameId, sim_time_s: history.step * DT, wide_png_base64: history.wide, tele_png_base64: history.tele });
    const decision = { sim_time_s: recordRound(obs.meta.sim_time), step_id: obs.meta.step_id, observation_id: obs.observation_id,
      source, inputs: { request: { schema_version: 'drivejev-online-1.1', arm: config.arm ?? null, obs_schema: config.obs_schema },
        student_obs: obs.student_obs, candidates: obs.candidates, rule_truth: obs.meta.teacher_state, state },
      output: result, application: { accepted: acceptedResult.accepted, reason: acceptedResult.reason ?? null,
        selected_candidate_id: result.candidate_id ?? null, executing_action: ex.maneuver?.action_type ?? null,
        maneuver_id: ex.maneuver?.maneuver_id ?? null }, latency_ms: recordRound(latencyMs),
      image_inputs: shot ? [{ camera: 'front', relative_time_s: -0.5, frame_id: historyFrameId },
        { camera: 'front', relative_time_s: 0, frame_id: frameId }, { camera: 'front_tele', relative_time_s: 0, frame_id: frameId }] : [] };
    decisions.push(decision);
    // A realtime reply may arrive after its event was captured; attach it to that event window too.
    for (const event of events) if (decision.sim_time_s >= event.time_s - 6 && decision.sim_time_s <= event.time_s + 0.001 &&
      !event.decision_trace_before_event.some((d) => d.observation_id === decision.observation_id)) {
      event.decision_trace_before_event.push(decision);
      event.decision_trace_before_event.sort((a, b) => a.sim_time_s - b.sim_time_s);
      const ids = new Set(event.image_frames_before_event.map((f) => f.frame_id));
      for (const input of decision.image_inputs) if (input.frame_id && !ids.has(input.frame_id) && imageFrames.has(input.frame_id)) {
        event.image_frames_before_event.push(imageFrames.get(input.frame_id)); ids.add(input.frame_id);
      }
    }
    const cutoff = sim.time - 8;
    while (decisions.length && decisions[0].sim_time_s < cutoff) decisions.shift();
    for (const [id, frame] of imageFrames) if (frame.sim_time_s < cutoff) imageFrames.delete(id);
  };
  const addEvent = (kind, event, time) => {
    const selected = decisions.filter((d) => d.sim_time_s >= time - 6 && d.sim_time_s <= time + 0.001);
    const ids = new Set(selected.flatMap((d) => d.image_inputs.map((x) => x.frame_id).filter(Boolean)));
    events.push({ kind, time_s: recordRound(time), event, decision_trace_before_event: selected,
      image_frames_before_event: [...ids].map((id) => imageFrames.get(id)).filter(Boolean), state_at_capture: ex.diagnosticSnapshot() });
  };
  const collect = () => {
    while (violationsSeen < ex.violationEvents.length) {
      const e = ex.violationEvents[violationsSeen++]; addEvent('simulation_violation', e, e.sim_time_s);
    }
    while (frontEventsSeen < ex.frontBumperEvents.length) {
      const e = ex.frontBumperEvents[frontEventsSeen++]; addEvent('front_bumper_crossing', e, e.sim_time);
    }
    if (!collisionSeen && sim.crash) {
      collisionSeen = true;
      const other = [...sim.traffic, ...sim.pedestrians].find((o) => o.id === sim.crash.object_id);
      const partner = other ? { id: other.id, type: other.type, x_m: recordRound(other.x), z_m: recordRound(other.z),
        heading_rad: recordRound(other.heading ?? 0), speed_mps: recordRound(other.speed ?? 0), width_m: recordRound(other.width),
        depth_m: recordRound(other.depth), crossing: other.crossing ?? null, walking: other.walking ?? null,
        hazard: other.hazard ? { ...other.hazard } : null } : null;
      addEvent('collision', { ...sim.crash, collision_partner: partner }, sim.crash.time_s ?? sim.time);
    }
  };
  return { rememberDecision, collect, events };
}
function summary(info, extra) {
  const truth = ex.observe().meta.evaluation_truth, v = sim.player;
  const collisionObject = sim.crash ? [...sim.traffic, ...sim.pedestrians].find((o) => o.id === sim.crash.object_id) : null;
  const crash_context = sim.crash ? { maneuver_id: ex.maneuver?.maneuver_id ?? null,
    action_type: ex.maneuver?.action_type ?? null, maneuver_status: ex.maneuver?.status ?? null,
    ego_at_collision: { x_m: recordRound(v.x), z_m: recordRound(v.z), heading_rad: recordRound(v.heading),
      heading_deg: recordRound(v.heading * 180 / Math.PI, 1), speed_mps: recordRound(sim.crash.player_speed_mps), route_s_m: recordRound(v.s) },
    ego_after_collision: { speed_mps: recordRound(v.speed), steering: recordRound(v.steering), target_speed_mps: recordRound(v.target) },
    collision_partner: collisionObject ? { id: collisionObject.id, type: collisionObject.type, x_m: recordRound(collisionObject.x), z_m: recordRound(collisionObject.z),
      heading_rad: recordRound(collisionObject.heading ?? 0), speed_mps: recordRound(collisionObject.speed ?? 0), width_m: recordRound(collisionObject.width),
      depth_m: recordRound(collisionObject.depth), crossing: collisionObject.crossing ?? null, walking: collisionObject.walking ?? null,
      hazard: collisionObject.hazard ? { ...collisionObject.hazard } : null } : null,
    diagnostic_state: ex.diagnosticSnapshot() } : null;
  return { ...info, config, sim_time_s: sim.time, arrived: sim.complete, crash: sim.crash, crash_context, collisions: sim.collisions, violations: sim.violations,
    violation_events: ex.violationEvents, front_red_violations: ex.frontRedViolations, front_required_stop_violations: ex.frontRequiredStopViolations,
    route_completion: Math.min(1, sim.player.s / Math.max(1, sim.player.route.length)), aeb_events: ex.aebEvents, aeb_steps: ex.aebSteps,
    hard_brake_steps: truth.hard_brake_steps, jerk_rms_mps3: truth.jerk_rms_mps3, hazards: sim.director?.log ?? [], crash_hazard: crashHazard(), front_events: ex.frontBumperEvents, ...extra };
}

async function requestSafeVlmAction(ex, obs, send) {
  const candidates = [...obs.candidates];
  let result, vlmApiLatencyMs = null;
  // The model service requires 2-12 candidates. If only one is offered, its choice is implicit.
  if (candidates.length === 1) {
    const only = candidates[0];
    result = { candidate_id: only.candidate_id, probabilities: { [only.candidate_id]: 1 },
      observation_id: obs.observation_id, model_version: 'safety-gate-singleton' };
  } else {
    const apiStarted = performance.now();
    result = await send(candidates);
    vlmApiLatencyMs = performance.now() - apiStarted;
  }
  // This measures the VLM request and response parsing, separately from rollout safety checks below.
  result = { ...result, vlm_api_latency_ms: recordMs(vlmApiLatencyMs) };
  const preferred = candidates.find((candidate) => candidate.candidate_id === result.candidate_id);
  if (!preferred) throw Error(`VLM selected a candidate outside the current set: ${result.candidate_id}`);
  const probability = (candidate) => {
    const value = Number(result.probabilities?.[candidate.candidate_id]);
    return Number.isFinite(value) ? value : -Infinity;
  };
  const remaining = candidates.filter((candidate) => candidate !== preferred)
    .map((candidate, index) => ({ candidate, index }))
    .sort((a, b) => probability(b.candidate) - probability(a.candidate) || a.index - b.index)
    .map(({ candidate }) => candidate);
  const candidateChecks = [];
  const candidateCheckTimes = [];
  const safetyCheckMs = () => candidateCheckTimes.reduce((sum, value) => sum + value, 0);
  for (const candidate of [preferred, ...remaining]) {
    const checkStarted = performance.now();
    const safe = await checkCandidateSafety(ex, candidate);
    const elapsedMs = performance.now() - checkStarted;
    candidateCheckTimes.push(elapsedMs);
    candidateChecks.push({ candidate_id: candidate.candidate_id, safe, elapsed_ms: recordMs(elapsedMs) });
    if (safe) {
      const selectedBy = candidate === preferred ? 'vlm' : 'highest_remaining_probability';
      const accepted = { ...result, candidate_id: candidate.candidate_id,
        safety_gate: { safe: true, attempts: candidateChecks.length, selected_by: selectedBy,
          safety_check_ms: recordMs(safetyCheckMs()),
          candidate_checks: candidateChecks } };
      if (candidate !== preferred && Number.isFinite(probability(candidate))) accepted.confidence = { ...(result.confidence ?? {}), value: probability(candidate) };
      return accepted;
    }
  }
  // No rollout-safe VLM option remains. Use maximum braking as the emergency fallback.
  const fallback = candidates.find((candidate) => candidate.candidate_id === 'emergency_brake');
  if (!fallback) throw Error('Safety gate exhausted all candidates and emergency_brake is unavailable');
  return { ...result, candidate_id: fallback.candidate_id,
    probabilities: { ...(result.probabilities ?? {}), [fallback.candidate_id]: result.probabilities?.[fallback.candidate_id] ?? 0 },
    model_version: result.model_version ?? 'safety-gate-fallback',
    safety_gate: { safe: false, exhausted: true, fallback: fallback.candidate_id, attempts: candidateChecks.length,
      safety_check_ms: recordMs(safetyCheckMs()),
      candidate_checks: candidateChecks } };
}

export async function runEpisode(cfg) {
  const info = await reset(cfg);
  const policy = cfg.policy ?? 'model', maxTime = cfg.max_time_s ?? 120, frames = [], latencies = [], actions = {}, rejections = {}, trace = [], behaviorIntervals = [], incidents = createIncidentLog();
  const vlmApiLatencySamples = [];
  recordBehaviorInterval(behaviorIntervals, 'local_bootstrap', true, 'bootstrap');
  let decisions = 0, accepted = 0, stuck = 0, stuckClear = 0, speedSum = 0;
  const safetyGateTimes = [];
  let safetyCheckRollouts = 0;
  const t0 = performance.now();
  while (sim.time < maxTime && !sim.crash && !sim.complete) {
    if (ex.stepId % 5 === 0) {
      const obs = ex.observe();
      const shot = policy === 'model' ? capture() : null;
      if (shot) { frames.push({ step: ex.stepId, ...shot }); if (frames.length > 4) frames.shift(); }
      const history = frames.find((f) => f.step === ex.stepId - 10);
      if (ex.stepId >= 10 && (history || policy !== 'model')) {
        let result, teacherDetail = null, decisionState = ex.diagnosticSnapshot(), latencyMs = null;
        if (policy === 'teacher') {
          const t = observableTeacher(ex, obs);
          teacherDetail = t;
          result = { candidate_id: t.preferred, probabilities: t.probabilities, observation_id: obs.observation_id, model_version: 'observable-teacher' };
        } else {
          const started = performance.now();
          result = await requestSafeVlmAction(ex, obs, async (candidates) => {
            const response = await post(cfg.model_url ?? '/predict', requestBody(cfg, { ...obs, candidates }, history, shot));
            const output = await response.json();
            if (!response.ok) throw Error(`model ${response.status}: ${JSON.stringify(output).slice(0, 300)}`);
            return output;
          });
          latencyMs = performance.now() - started; latencies.push(latencyMs);
          if (Number.isFinite(result.vlm_api_latency_ms)) vlmApiLatencySamples.push(result.vlm_api_latency_ms);
        }
        if (result.safety_gate) {
          safetyGateTimes.push(result.safety_gate.safety_check_ms ?? 0);
          safetyCheckRollouts += result.safety_gate.candidate_checks?.length ?? 0;
        }
        const res = ex.accept({ ...result, episode_id: ex.episodeId }, { allowDelayed: true, maxAgeSeconds: 0.5 });
        incidents.rememberDecision({ obs, shot, history, result: { ...result, teacher_detail: teacherDetail }, acceptedResult: res,
          source: policy, latencyMs, state: decisionState });
        recordBehaviorInterval(behaviorIntervals, result.candidate_id, res.accepted, policy);
        if (res.accepted) accepted++; else rejections[res.reason] = (rejections[res.reason] ?? 0) + 1;
        decisions++; actions[result.candidate_id] = (actions[result.candidate_id] ?? 0) + 1;
        if (cfg.trace) trace.push({ t: Math.round(sim.time * 100) / 100, v: Math.round(sim.player.speed * 100) / 100, chosen: result.candidate_id, executing: ex.maneuver?.action_type, p: result.probabilities });
      }
    }
    if (stuckStep()) { stuck += DT; if (ex.unnecessaryStop()) stuckClear += DT; }
    speedSum += Math.abs(sim.player.speed) * DT;
    if (ex.step(1)) break;
    incidents.collect();
    if (ex.stepId % 200 === 0) await new Promise((r) => setTimeout(r, 0));
  }
  latencies.sort((a, b) => a - b);
  incidents.collect();
  if (behaviorIntervals.length) behaviorIntervals[behaviorIntervals.length - 1].interval_end_s = recordRound(sim.time);
  return summary(info, { policy, mode: 'batch', wall_s: (performance.now() - t0) / 1000, mean_speed_mps: speedSum / Math.max(sim.time, 1e-6), stuck_s: stuck, stuck_clear_s: stuckClear,
    decisions, accepted, rejections, action_counts: actions, behavior_intervals: behaviorIntervals, diagnostic_events: incidents.events,
    latency_ms_p50: latencies[Math.floor(latencies.length / 2)] ?? null,
    vlm_api_call_count: vlmApiLatencySamples.length,
    vlm_api_latency_ms_samples: vlmApiLatencySamples,
    vlm_api_latency_ms_mean: vlmApiLatencySamples.length ? recordMs(vlmApiLatencySamples.reduce((sum, value) => sum + value, 0) / vlmApiLatencySamples.length) : null,
    vlm_api_latency_ms_p50: vlmApiLatencySamples.length ? vlmApiLatencySamples.slice().sort((a, b) => a - b)[Math.floor(vlmApiLatencySamples.length / 2)] : null,
    vlm_api_latency_ms_p95: vlmApiLatencySamples.length ? vlmApiLatencySamples.slice().sort((a, b) => a - b)[Math.floor(vlmApiLatencySamples.length * 0.95)] : null,
    ...safetyTimingMetrics(safetyGateTimes, safetyCheckRollouts), trace: cfg.trace ? trace : undefined });
}

/** Real time: physics follows the wall clock at 20 Hz; at most one request in flight; answers older than 0.5 s are dropped. */
export async function runRealtime(cfg) {
  const info = await reset(cfg);
  const maxTime = cfg.max_time_s ?? 120, frames = [], latencies = [], rejections = {}, behaviorIntervals = [], incidents = createIncidentLog();
  const vlmApiLatencySamples = [];
  recordBehaviorInterval(behaviorIntervals, 'local_bootstrap', true, 'bootstrap');
  let inflight = false, requests = 0, accepted = 0, stuck = 0, speedSum = 0;
  const safetyGateTimes = [];
  let safetyCheckRollouts = 0;
  frames.push({ step: ex.stepId, ...capture() }); // history frame for the first request at 0.5 s
  ex.observe();
  const start = performance.now();
  const dispatch = async (obs, history, shot, decisionState) => {
    inflight = true; requests++;
    const t = performance.now();
    try {
      const result = await requestSafeVlmAction(ex, obs, async (candidates) => {
        const response = await post(cfg.model_url ?? '/predict', requestBody(cfg, { ...obs, candidates }, history, shot));
        const output = await response.json();
        if (!response.ok) throw Error(`model ${response.status}: ${JSON.stringify(output).slice(0, 300)}`);
        return output;
      });
      const latencyMs = performance.now() - t; latencies.push(latencyMs);
      if (Number.isFinite(result.vlm_api_latency_ms)) vlmApiLatencySamples.push(result.vlm_api_latency_ms);
      if (result.safety_gate) {
        safetyGateTimes.push(result.safety_gate.safety_check_ms ?? 0);
        safetyCheckRollouts += result.safety_gate.candidate_checks?.length ?? 0;
      }
      const res = ex.accept({ ...result, episode_id: ex.episodeId }, { allowDelayed: true, maxAgeSeconds: 0.5 });
      incidents.rememberDecision({ obs, shot, history, result, acceptedResult: res, source: 'model', latencyMs, state: decisionState });
      recordBehaviorInterval(behaviorIntervals, result.candidate_id, res.accepted, 'model');
      if (res.accepted) accepted++; else rejections[res.reason] = (rejections[res.reason] ?? 0) + 1;
    } finally { inflight = false; }
  };
  while (sim.time < maxTime && !sim.crash && !sim.complete) {
    const due = Math.floor((performance.now() - start) / (DT * 1000));
    let stepped = false;
    while (ex.stepId < due && !sim.crash && !sim.complete && sim.time < maxTime) {
      if (stuckStep()) stuck += DT;
      speedSum += Math.abs(sim.player.speed) * DT;
      ex.step(1); stepped = true;
      incidents.collect();
      if (ex.stepId % 5 === 0) {
        const obs = ex.observe(), shot = capture();
        frames.push({ step: ex.stepId, ...shot }); if (frames.length > 4) frames.shift();
        const history = frames.find((f) => f.step === ex.stepId - 10);
        if (history && !inflight) void dispatch(obs, history, shot, ex.diagnosticSnapshot());
      }
    }
    await new Promise((r) => setTimeout(r, stepped ? 0 : 4));
  }
  while (inflight) await new Promise((r) => setTimeout(r, 10));
  incidents.collect();
  const wall = (performance.now() - start) / 1000;
  latencies.sort((a, b) => a - b);
  if (behaviorIntervals.length) behaviorIntervals[behaviorIntervals.length - 1].interval_end_s = recordRound(sim.time);
  return summary(info, { policy: 'model', mode: 'realtime', wall_s: wall, realtime_factor: sim.time / wall, mean_speed_mps: speedSum / Math.max(sim.time, 1e-6), stuck_s: stuck,
    requests, accepted, applied_hz: accepted / Math.max(sim.time, 1e-6), rejections, behavior_intervals: behaviorIntervals, diagnostic_events: incidents.events, latency_ms_p50: latencies[Math.floor(latencies.length / 2)] ?? null,
    vlm_api_call_count: vlmApiLatencySamples.length,
    vlm_api_latency_ms_samples: vlmApiLatencySamples,
    vlm_api_latency_ms_mean: vlmApiLatencySamples.length ? recordMs(vlmApiLatencySamples.reduce((sum, value) => sum + value, 0) / vlmApiLatencySamples.length) : null,
    vlm_api_latency_ms_p50: vlmApiLatencySamples.length ? vlmApiLatencySamples.slice().sort((a, b) => a - b)[Math.floor(vlmApiLatencySamples.length / 2)] : null,
    vlm_api_latency_ms_p95: vlmApiLatencySamples.length ? vlmApiLatencySamples.slice().sort((a, b) => a - b)[Math.floor(vlmApiLatencySamples.length * 0.95)] : null,
    latency_ms_p95: latencies[Math.floor(latencies.length * 0.95)] ?? null, ...safetyTimingMetrics(safetyGateTimes, safetyCheckRollouts) });
}

window.drivejev = { reset, runEpisode, runRealtime, capture, get sim() { return sim; }, get executor() { return ex; } };
window.drivejevReady = true;
