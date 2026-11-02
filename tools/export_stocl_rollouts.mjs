#!/usr/bin/env node
/**
 * Replay one recorded DriveJev episode and export selected VLM candidate rollouts
 * as STOCL scenario-evaluation request JSON files.
 *
 * Usage:
 *   node tools/export_stocl_rollouts.mjs <episodes.jsonl> <seed> <output-dir>
 */
import fs from 'node:fs';
import path from 'node:path';
import { InteractionWorld } from '../simulator/interactions.mjs';
import { InteractionExecutor, DT, captureVlmCandidateRollout } from '../simulator/interaction_executor.mjs';

const [episodesPath, seedArg, outputArg] = process.argv.slice(2);
if (!episodesPath || !seedArg || !outputArg) {
  console.error('usage: node tools/export_stocl_rollouts.mjs <episodes.jsonl> <seed> <output-dir>');
  process.exit(2);
}

const seed = Number(seedArg);
const outputDir = path.resolve(outputArg);
if (!Number.isInteger(seed)) throw new Error(`invalid seed: ${seedArg}`);
if (fs.existsSync(outputDir) && fs.readdirSync(outputDir).length) {
  throw new Error(`${outputDir} is not empty; choose a new output directory`);
}

const episode = fs.readFileSync(episodesPath, 'utf8').split(/\r?\n/).filter(Boolean)
  .map(JSON.parse).find((row) => row.config?.seed === seed);
if (!episode) throw new Error(`seed ${seed} was not found in ${episodesPath}`);

const diagramName = 'DriveJevRolloutEvaluation';
const classDiagram = {
  class_diagram: diagramName,
  classes: [
    { name: 'Ego', attributes: [
      { attr: 'position', type: 'Geometry' },
      { attr: 'speed', type: 'Real' },
      { attr: 'heading', type: 'Real' },
      { attr: 'footprint', type: 'Geometry' },
      { attr: 'frontZone', type: 'Geometry' },
      { attr: 'targetSpeed', type: 'Real' },
      { attr: 'stopLineDist', type: 'Real' },
    ] },
    { name: 'TrafficObject', attributes: [
      { attr: 'actorType', type: 'String' },
      { attr: 'position', type: 'Geometry' },
      { attr: 'speed', type: 'Real' },
      { attr: 'heading', type: 'Real' },
      { attr: 'footprint', type: 'Geometry' },
      { attr: 'frontZone', type: 'Geometry' },
    ] },
  ],
  associations: [{
    name: 'Perception', startClass: 'Ego', endClass: 'TrafficObject', role: 'perceived',
    multi: '0..*', r_role: 'ego', r_multi: '1',
  }],
};
const specification = `Model ${diagramName}:
context Ego
  inv NearbyRisk: self.perceived->exists(t | t.position.distance(self.position) < (t.speed + self.targetSpeed) * 3 + 12)
  inv WithoutCollisionFront: self.perceived->forAll(t | self.frontZone.intersects(t.frontZone) = false)
formula Safe: globally[0,90] (not NearbyRisk or WithoutCollisionFront)
`;

function finite(value, fallback = 0) {
  return Number.isFinite(value) ? value : fallback;
}
function n(value) {
  return Number(finite(value).toFixed(6));
}
function pointWkt(position) {
  return `POINT (${n(position.x)} ${n(position.z)})`;
}
function rectangleWkt(box) {
  const h = finite(box.heading), s = Math.sin(h), c = Math.cos(h);
  const halfWidth = finite(box.width, 1.9) / 2, halfDepth = finite(box.depth, 4.2) / 2;
  const corners = [
    [-1, -1], [1, -1], [1, 1], [-1, 1], [-1, -1],
  ].map(([rightSign, forwardSign]) => {
    const right = rightSign * halfWidth, forward = forwardSign * halfDepth;
    return [box.x + c * right + s * forward, box.z + s * right - c * forward];
  });
  return `POLYGON ((${corners.map(([x, z]) => `${n(x)} ${n(z)}`).join(', ')}))`;
}
function frontZoneWkt(object) {
  const box = object.footprint;
  const extension = Math.max(0, Math.min(8, finite(object.speed) * DT));
  const sin = Math.sin(finite(object.heading)), cos = Math.cos(finite(object.heading));
  return rectangleWkt({
    x: box.x + sin * extension / 2,
    z: box.z - cos * extension / 2,
    width: box.width,
    depth: box.depth + extension,
    heading: object.heading,
  });
}

const worldConfig = episode.config;
const sim = new InteractionWorld(worldConfig.seed, worldConfig.type, {
  randomRoute: worldConfig.randomRoute && worldConfig.type !== 'highway',
  hazards: worldConfig.hazards,
  trafficScale: worldConfig.trafficScale,
  empty: worldConfig.empty,
  hazardCooldown: worldConfig.hazardCooldown,
  legacyHazards: worldConfig.legacyHazards,
});
const ex = new InteractionExecutor(sim, {
  aeb: worldConfig.aeb,
  safePass: worldConfig.safePass ?? true,
  obsSchema: worldConfig.obs_schema ?? '1.1',
});
ex.episodeId = `drivejev-${worldConfig.type}-${worldConfig.seed}`;
ex.select(sim.player.speed >= 0.5 ? 'keep_route_cruise' : 'hold_stop');
ex.policySource = 'local_bootstrap';

const behaviorAtStep = new Map(episode.behavior_intervals
  .filter((row) => Number.isInteger(row.step_id))
  .map((row) => [row.step_id, row]));
const appliedSteps = new Set();
function applyRecordedDecision(stepId) {
  if (appliedSteps.has(stepId)) return;
  appliedSteps.add(stepId);
  const row = behaviorAtStep.get(stepId);
  if (!row || row.source !== 'model' || !row.accepted || !row.selected_candidate_id) return;
  ex.select({
    candidate_id: row.selected_candidate_id,
    action_type: row.selected_candidate_id,
    target_id: row.target_id,
  });
  ex.policySource = 'recorded-model-action';
}

function advanceToStep(targetStep) {
  while (ex.stepId < targetStep) {
    if (ex.stepId % 5 === 0) {
      ex.observe();
      applyRecordedDecision(ex.stepId);
    }
    ex.step(1);
    if (sim.crash) throw new Error(`replay crashed at ${sim.time.toFixed(2)} s before step ${targetStep}`);
  }
  if (ex.stepId !== targetStep) throw new Error(`replay passed target step ${targetStep}`);
  if (ex.stepId % 5 === 0) ex.observe();
}

function candidate(id) {
  return { candidate_id: id, action_type: id, target_id: id === 'stop_at_line' ? ex.crossing()?.nodeId ?? null : null };
}

function toSnapshot(rawState, timeS, caseId, stateIndex, idsBySourceId) {
  const ego = rawState.ego;
  const egoBox = ego.footprint;
  const trafficObjects = rawState.perceived.map((object) => {
    if (!idsBySourceId.has(object.id)) idsBySourceId.set(object.id, idsBySourceId.size + 1);
    const id = idsBySourceId.get(object.id);
    return {
      id,
      actorType: object.actorType,
      position: pointWkt(object.position),
      speed: n(object.speed),
      heading: n(object.heading),
      footprint: rectangleWkt(egoBoxOf(object)),
      frontZone: frontZoneWkt(object),
    };
  });
  const egoObject = {
    id: 1,
    position: pointWkt(ego.position),
    speed: n(ego.speed),
    heading: n(ego.heading),
    footprint: rectangleWkt(egoBox),
    frontZone: frontZoneWkt(ego),
    targetSpeed: n(ego.targetSpeed, n(ego.speed)),
    stopLineDist: n(ego.stopLineDist, 1e9),
  };
  return {
    schemaVersion: 1,
    modelName: diagramName,
    time_s: n(timeS),
    caseId,
    stateIndex,
    objectsByClass: { Ego: [egoObject], TrafficObject: trafficObjects },
    linksByAssociation: {
      Perception: trafficObjects.map((object) => ({ sid: 1, eid: object.id })),
    },
  };
}

function egoBoxOf(object) {
  return object.footprint;
}

const cases = [
  { id: 't79_75_keep_route_cruise', step: 1595, candidate: 'keep_route_cruise' },
  { id: 't80_keep_route_cruise', step: 1600, candidate: 'keep_route_cruise' },
  { id: 't80_stop_at_line', step: 1600, candidate: 'stop_at_line' },
  { id: 't80_emergency_brake', step: 1600, candidate: 'emergency_brake' },
];

fs.mkdirSync(path.join(outputDir, 'cases'), { recursive: true });
fs.writeFileSync(path.join(outputDir, 'class_diagram.json'), `${JSON.stringify(classDiagram, null, 2)}\n`);
fs.writeFileSync(path.join(outputDir, 'specification.stocl'), specification);

const manifest = {
  sourceEpisode: path.resolve(episodesPath),
  seed,
  world: { type: worldConfig.type, hazards: worldConfig.hazards, legacyHazards: worldConfig.legacyHazards },
  dt_s: DT,
  horizon_s: 4.5,
  scenarioStateCount: 91,
  classDiagram: 'class_diagram.json',
  specification: 'specification.stocl',
  cases: [],
};

for (const testCase of cases) {
  advanceToStep(testCase.step);
  const startTime = sim.time;
  const result = await captureVlmCandidateRollout(ex, candidate(testCase.candidate));
  const idsBySourceId = new Map();
  const states = result.states.map((state, index) =>
    toSnapshot(state, startTime + index * DT, testCase.id, index, idsBySourceId));
  const request = {
    testCase: testCase.id,
    candidateId: testCase.candidate,
    candidateSafeFormula: result.safe_formula,
    dt_s: DT,
    horizon_s: 4.5,
    currentPosition: 1,
    strategy: 'DYNAMIC',
    specification,
    states,
  };
  fs.writeFileSync(path.join(outputDir, 'cases', `${testCase.id}.json`), `${JSON.stringify(request, null, 2)}\n`);
  manifest.cases.push({
    file: `cases/${testCase.id}.json`,
    start_time_s: n(startTime),
    candidate: testCase.candidate,
    safe_formula: result.safe_formula,
    state_count: states.length,
    traffic_object_counts: [...new Set(states.map((state) => state.objectsByClass.TrafficObject.length))],
  });
}

fs.writeFileSync(path.join(outputDir, 'manifest.json'), `${JSON.stringify(manifest, null, 2)}\n`);
console.log(JSON.stringify(manifest, null, 2));
