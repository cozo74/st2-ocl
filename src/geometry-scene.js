import * as THREE from "three";
import { CameraInput } from "./camera-input.js";
import { CANDIDATE_COUNT, VECTOR_STEPS, vectorWeights } from "./planning.js";
import { roadGeometry } from "./road-geometry.js";
import { signalState } from "./world.js";

const COLORS = {
  background: 0xf7f7f4,
  building: 0x30343b,
  buildingSurface: 0xe1e3e0,
  road: 0x8b9198,
  route: 0x246bfd,
  ego: 0x0b57d0,
  traffic: 0xd45b3f,
  pedestrian: 0x8b4fc1,
  destination: 0x299764,
  control: 0x4b5552,
  stop: 0xd3392f,
};

const SIGNAL_COLORS = {
  red: 0xf04438,
  amber: 0xf2b72f,
  green: 0x2eb85c,
  off: 0x59615e,
};

const LOCK_COLORS = {
  free: 0x4b9b77,
  locked: 0xf08c2e,
  courtesy: 0xd94c8b,
};

const LOCK_RELEASE_RADIUS = 17;
const COURTESY_RELEASE_RADIUS = 22;

function lineMaterial(color, opacity = 1) {
  return new THREE.LineBasicMaterial({
    color,
    transparent: opacity < 1,
    opacity,
    toneMapped: false,
  });
}

function lineFromPoints(points, color, { closed = false, y = 0.05 } = {}) {
  const vertices = points.map((point) =>
    point.isVector3 ? point : new THREE.Vector3(point.x, point.y ?? y, point.z),
  );
  const geometry = new THREE.BufferGeometry().setFromPoints(vertices);
  const LineType = closed ? THREE.LineLoop : THREE.Line;
  return new LineType(geometry, lineMaterial(color));
}

function boxOutline(width, height, depth, color) {
  const box = new THREE.BoxGeometry(width, height, depth);
  const edges = new THREE.EdgesGeometry(box);
  box.dispose();
  return new THREE.LineSegments(edges, lineMaterial(color));
}

function headingArrow(bodyDepth, extension, color) {
  const start = -bodyDepth / 2;
  const tip = start - extension;
  const headLength = Math.min(0.35, extension * 0.35);
  const headWidth = headLength * 0.55;
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute(
    "position",
    new THREE.Float32BufferAttribute(
      [
        0,
        0.035,
        start,
        0,
        0.035,
        tip,
        0,
        0.035,
        tip,
        -headWidth / 2,
        0.035,
        tip + headLength,
        0,
        0.035,
        tip,
        headWidth / 2,
        0.035,
        tip + headLength,
      ],
      3,
    ),
  );
  return new THREE.LineSegments(geometry, lineMaterial(color));
}

function solidBox(width, height, depth, surfaceColor, edgeColor) {
  const geometry = new THREE.BoxGeometry(width, height, depth);
  const mesh = new THREE.Mesh(
    geometry,
    new THREE.MeshBasicMaterial({
      color: surfaceColor,
      transparent: false,
      opacity: 1,
      toneMapped: false,
    }),
  );
  const outline = new THREE.LineSegments(
    new THREE.EdgesGeometry(geometry),
    lineMaterial(edgeColor),
  );
  const group = new THREE.Group();
  group.add(mesh, outline);
  return group;
}

function controlPole(height) {
  return lineFromPoints(
    [
      { x: 0, y: 0, z: 0 },
      { x: 0, y: height, z: 0 },
    ],
    COLORS.control,
  );
}

function verticalFace(radius, centerY, color, segments = 20, z = 0.22) {
  return lineFromPoints(
    Array.from({ length: segments }, (_, index) => {
      const angle = (index / segments) * Math.PI * 2;
      return {
        x: Math.sin(angle) * radius,
        y: centerY + Math.cos(angle) * radius,
        z,
      };
    }),
    color,
    { closed: true },
  );
}

function horizontalCircle(radius, color, opacity = 1, y = 0.09) {
  const circle = lineFromPoints(
    Array.from({ length: 64 }, (_, index) => {
      const angle = (index / 64) * Math.PI * 2;
      return {
        x: Math.cos(angle) * radius,
        y,
        z: Math.sin(angle) * radius,
      };
    }),
    color,
    { closed: true },
  );
  circle.material.transparent = opacity < 1;
  circle.material.opacity = opacity;
  return circle;
}

function polygonBoundarySegments(polygons) {
  const edges = new Map();
  const pointKey = (point) =>
    `${Math.round(point.x * 10000)},${Math.round(point.z * 10000)}`;
  for (const polygon of polygons) {
    polygon.points.forEach((point, index) => {
      const next = polygon.points[(index + 1) % polygon.points.length];
      const a = pointKey(point);
      const b = pointKey(next);
      const key = a < b ? `${a}|${b}` : `${b}|${a}`;
      const entry = edges.get(key);
      if (entry) entry.count++;
      else edges.set(key, { count: 1, point, next });
    });
  }
  const positions = [];
  for (const { count, point, next } of edges.values()) {
    if (count !== 1) continue;
    positions.push(point.x, 0.035, point.z, next.x, 0.035, next.z);
  }
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute(
    "position",
    new THREE.Float32BufferAttribute(positions, 3),
  );
  return new THREE.LineSegments(geometry, lineMaterial(COLORS.road));
}

function preparePaths(plan) {
  if (plan.geometryPaths) return;
  const origin = plan.origin;
  const sin = Math.sin(origin.heading);
  const cos = Math.cos(origin.heading);
  plan.geometryPaths = Object.fromEntries(
    Object.entries(plan.projections).map(([id, path]) => {
      const direction = plan.vectors[id].velocity_mps < 0 ? -1 : 1;
      return [
        id,
        path.points.map((point) => {
          const x =
            point.x +
            Math.sin(point.heading) * (origin.depth / 2 + 0.2) * direction;
          const z =
            point.z -
            Math.cos(point.heading) * (origin.depth / 2 + 0.2) * direction;
          return {
            right: (x - origin.x) * cos + (z - origin.z) * sin,
            ahead: (x - origin.x) * sin - (z - origin.z) * cos,
          };
        }),
      ];
    }),
  );
}

function vectorColor(candidate) {
  if (candidate.collision_predicted) return 0xe35d34;
  if (candidate.velocity_mps < 0) return 0x8c5ee8;
  if (!candidate.stays_on_road || !candidate.stays_in_lane) return 0xd88928;
  return candidate.steering < -0.02 ? 0x149fbb : 0x2988ef;
}

class GeometryVectors {
  constructor(scene) {
    this.group = new THREE.Group();
    this.group.renderOrder = 5;
    scene.add(this.group);
    this.enabled = true;
    this.showCandidates = false;
    this.pool = Array.from({ length: CANDIDATE_COUNT }, () => {
      const positions = new Float32Array((VECTOR_STEPS + 1) * 3);
      const geometry = new THREE.BufferGeometry();
      geometry.setAttribute(
        "position",
        new THREE.BufferAttribute(positions, 3).setUsage(
          THREE.DynamicDrawUsage,
        ),
      );
      const line = new THREE.Line(geometry, lineMaterial(0x2988ef, 0.72));
      line.frustumCulled = false;
      line.visible = false;
      this.group.add(line);
      return line;
    });
    this.clear();
  }

  setCandidates(plan) {
    preparePaths(plan);
    this.plan = plan;
  }

  setAnswer(answer, plan) {
    this.setCandidates(plan);
    this.answer = answer;
    this.answeredPlan = plan;
    this.received = performance.now();
  }

  clear() {
    this.answer = null;
    this.plan = null;
    this.answeredPlan = null;
    this.received = 0;
    this.group.visible = false;
    for (const line of this.pool) line.visible = false;
  }

  render(car, _camera, _width, _height, _dt, active, paused) {
    const age = paused ? 0 : performance.now() - this.received;
    const weights = active
      ? vectorWeights(this.answer, this.answeredPlan?.eligible, age)
      : null;
    const chosenId = weights ? this.answer.choice : null;
    const plan = chosenId ? this.answeredPlan : this.plan;
    const ids = plan
      ? Object.keys(plan.vectors).filter(
          (id) => this.showCandidates || (this.enabled && id === chosenId),
        )
      : [];
    this.group.visible = ids.length > 0;
    for (const line of this.pool) line.visible = false;
    ids.slice(0, this.pool.length).forEach((id, index) => {
      const line = this.pool[index];
      const path = plan.geometryPaths[id];
      const attribute = line.geometry.attributes.position;
      const sin = Math.sin(car.heading);
      const cos = Math.cos(car.heading);
      for (let i = 0; i <= VECTOR_STEPS; i++) {
        const point = path[Math.min(i, path.length - 1)];
        attribute.setXYZ(
          i,
          car.x + point.right * cos + point.ahead * sin,
          id === chosenId ? 0.22 : 0.16,
          car.z + point.right * sin - point.ahead * cos,
        );
      }
      attribute.needsUpdate = true;
      line.material.color.set(
        id === chosenId ? COLORS.route : vectorColor(plan.vectors[id]),
      );
      line.material.opacity = id === chosenId ? 1 : 0.66;
      line.visible = true;
    });
  }
}

export class GeometryScene {
  constructor(canvas, sim, vectorLayer) {
    this.canvas = canvas;
    this.sim = sim;
    this.mode = "chase";
    this.showSensors = false;
    this.cameraInput = new CameraInput(canvas, () => this.mode);
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
    this.renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
    this.renderer.shadowMap.enabled = false;
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.camera = new THREE.PerspectiveCamera(52, 1, 0.1, 1500);
    this.viewport = { width: canvas.clientWidth, height: canvas.clientHeight };
    this.resizeObserver = new ResizeObserver(([entry]) => {
      this.viewport = entry.contentRect;
    });
    this.resizeObserver.observe(canvas);
    this.vectorLayer = vectorLayer;
    this.build();
  }

  disposeObjects() {
    if (!this.scene) return;
    this.scene.traverse((object) => {
      object.geometry?.dispose();
      if (Array.isArray(object.material))
        object.material.forEach((material) => material.dispose());
      else object.material?.dispose();
    });
  }

  build() {
    this.disposeObjects();
    this.vectorLayer.replaceChildren();
    this.vectorLayer.hidden = false;
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(COLORS.background);
    this.scene.add(polygonBoundarySegments(roadGeometry(this.sim.world)));

    const { minX, maxX, minZ, maxZ } = this.sim.world.bounds;
    const boundary = new THREE.LineLoop(
      new THREE.BufferGeometry().setFromPoints([
        new THREE.Vector3(minX, 0.12, minZ),
        new THREE.Vector3(maxX, 0.12, minZ),
        new THREE.Vector3(maxX, 0.12, maxZ),
        new THREE.Vector3(minX, 0.12, maxZ),
      ]),
      lineMaterial(0x273746),
    );
    boundary.name = "world-boundary";
    this.scene.add(boundary);

    this.buildings = new THREE.Group();
    for (const building of this.sim.world.objects.filter(
      (object) => object.type === "building",
    )) {
      const outline = solidBox(
        building.width,
        building.height,
        building.depth,
        COLORS.buildingSurface,
        COLORS.building,
      );
      outline.position.set(building.x, building.height / 2, building.z);
      outline.rotation.y = building.rotation || 0;
      this.buildings.add(outline);
    }
    this.scene.add(this.buildings);

    this.controls = new THREE.Group();
    this.signals = [];
    for (const control of this.sim.world.objects.filter(
      (object) =>
        object.type === "traffic_light" || object.type === "stop_sign",
    )) {
      const group = new THREE.Group();
      group.position.set(control.x, 0, control.z);
      group.rotation.y = -control.approach;
      group.add(controlPole(control.height));
      if (control.type === "stop_sign") {
        const sign = verticalFace(0.65, 2.45, COLORS.stop, 8, 0.08);
        sign.name = `stop-sign-${control.id}`;
        group.add(sign);
      } else {
        const housing = boxOutline(0.65, 1.65, 0.38, COLORS.control);
        housing.position.y = 4.2;
        group.add(housing);
        const lamps = ["red", "amber", "green"].map((color, index) => {
          const lamp = verticalFace(
            0.17,
            4.73 - index * 0.5,
            SIGNAL_COLORS.off,
          );
          lamp.name = `traffic-light-${control.id}-${color}`;
          group.add(lamp);
          return { color, lamp };
        });
        this.signals.push({ control, lamps });
      }
      this.controls.add(group);
    }
    this.scene.add(this.controls);

    this.route = null;
    this.routePoints = null;
    this.updateRoute();
    this.safetyStatus = document.createElement("div");
    this.safetyStatus.className = "geometry-safety-status";
    this.safetyStatus.setAttribute("aria-live", "polite");
    this.safetyTitle = document.createElement("div");
    this.safetyTitle.className = "geometry-safety-title";
    this.safetyRows = Object.fromEntries(
      ["world", "route", "destination", "following", "conflict"].map(
        (key) => {
          const row = document.createElement("div");
          row.className = "geometry-safety-row";
          this.safetyStatus.append(row);
          return [key, row];
        },
      ),
    );
    this.safetyStatus.prepend(this.safetyTitle);
    this.vectorLayer.append(this.safetyStatus);
    this.recoveryStatus = document.createElement("div");
    this.recoveryStatus.className = "geometry-recovery-status";
    this.vectorLayer.append(this.recoveryStatus);

    this.destination = new THREE.Group();
    this.destination.add(
      lineFromPoints(
        Array.from({ length: 49 }, (_, index) => {
          const angle = (index / 48) * Math.PI * 2;
          return { x: Math.cos(angle) * 2, y: 0.12, z: Math.sin(angle) * 2 };
        }),
        COLORS.destination,
      ),
    );
    const destination = this.sim.player.route.points.at(-1);
    this.destination.position.set(destination.x, 0, destination.z);
    this.scene.add(this.destination);

    this.player = boxOutline(
      this.sim.player.width,
      1.5,
      this.sim.player.depth,
      COLORS.ego,
    );
    this.player.position.y = 0.75;
    this.scene.add(this.player);
    this.playerHeading = headingArrow(this.sim.player.depth, 1.2, COLORS.ego);
    this.scene.add(this.playerHeading);

    this.vehicles = new Map();
    this.vehicleHeadings = new Map();
    for (const vehicle of this.sim.traffic) {
      const outline = boxOutline(
        vehicle.width,
        vehicle.type === "motorcycle" ? 1.3 : 1.5,
        vehicle.depth,
        COLORS.traffic,
      );
      outline.position.y = vehicle.type === "motorcycle" ? 0.65 : 0.75;
      this.vehicles.set(vehicle.id, outline);
      this.scene.add(outline);
      const arrow = headingArrow(vehicle.depth, 1, COLORS.traffic);
      this.vehicleHeadings.set(vehicle.id, arrow);
      this.scene.add(arrow);
    }

    this.people = new Map();
    this.pedestrianHeadings = new Map();
    for (const pedestrian of this.sim.pedestrians) {
      const outline = boxOutline(
        pedestrian.width,
        pedestrian.height,
        pedestrian.depth,
        COLORS.pedestrian,
      );
      outline.position.y = pedestrian.height / 2;
      this.people.set(pedestrian.id, outline);
      this.scene.add(outline);
      const arrow = headingArrow(pedestrian.depth, 0.75, COLORS.pedestrian);
      this.pedestrianHeadings.set(pedestrian.id, arrow);
      this.scene.add(arrow);
    }

    this.vectors = new GeometryVectors(this.scene);
    this.entityLabels = [
      this.makeEntityLabel(this.sim.player, "EGO", "ego", 1.8),
      ...this.sim.traffic.map((vehicle) =>
        this.makeEntityLabel(
          vehicle,
          vehicle.type === "motorcycle" ? "MOTORCYCLE" : "NPC CAR",
          "vehicle",
          vehicle.type === "motorcycle" ? 1.55 : 1.8,
        ),
      ),
      ...this.sim.pedestrians.map((pedestrian) =>
        this.makeEntityLabel(
          pedestrian,
          "PEDESTRIAN",
          "pedestrian",
          pedestrian.height + 0.25,
        ),
      ),
    ];
    this.lockRanges = new THREE.Group();
    this.scene.add(this.lockRanges);
    this.lockViews = this.sim.world.nodes
      .filter((node) => node.control !== "none")
      .map((node) => this.makeLockView(node));
    this.ready = Promise.resolve();
    this.snap = true;
  }

  makeEntityLabel(entity, text, kind, height) {
    const anchor = document.createElement("span");
    anchor.className = "geometry-entity-anchor";
    const element = document.createElement("span");
    element.className = `geometry-entity-label ${kind}`;
    element.dataset.type = text;
    const type = document.createElement("strong");
    type.textContent = text;
    const id = document.createElement("span");
    id.textContent = `id: ${entity.id}`;
    const position = document.createElement("span");
    const velocity = document.createElement("span");
    const heading = document.createElement("span");
    element.append(type, id, position, velocity, heading);
    element.setAttribute("aria-label", `${text} ${entity.id}`);
    anchor.append(element);
    this.vectorLayer.append(anchor);
    return { anchor, element, position, velocity, heading, entity, height };
  }

  makeLockView(node) {
    const ring = horizontalCircle(LOCK_RELEASE_RADIUS, LOCK_COLORS.free, 0.24);
    ring.position.set(node.x, 0, node.z);
    const courtesyRing = horizontalCircle(
      COURTESY_RELEASE_RADIUS,
      LOCK_COLORS.courtesy,
      0.9,
      0.1,
    );
    courtesyRing.position.set(node.x, 0, node.z);
    courtesyRing.visible = false;
    this.lockRanges.add(ring, courtesyRing);
    const element = document.createElement("span");
    element.className = "geometry-lock-label free";
    const id = document.createElement("span");
    id.className = "geometry-lock-id";
    id.textContent = `id: ${node.id}`;
    const title = document.createElement("strong");
    title.textContent = "INTERSECTION";
    const status = document.createElement("span");
    const owner = document.createElement("span");
    const courtesy = document.createElement("span");
    courtesy.className = "courtesy-state";
    const range = document.createElement("span");
    range.textContent = `lock range: ${LOCK_RELEASE_RADIUS} m release radius`;
    element.append(title, id, status, owner, courtesy, range);
    this.vectorLayer.append(element);
    return {
      node,
      ring,
      courtesyRing,
      element,
      status,
      owner,
      courtesy,
    };
  }

  lockOwner(id) {
    if (id === this.sim.player.id) return "EGO";
    const vehicle = this.sim.traffic.find((candidate) => candidate.id === id);
    if (!vehicle) return id;
    return `${vehicle.type === "motorcycle" ? "MOTORCYCLE" : "NPC CAR"} ${id}`;
  }

  updateEntityLabels(width, height) {
    this.camera.updateMatrixWorld();
    const ego = this.sim.player;
    const point = new THREE.Vector3();
    for (const label of this.entityLabels) {
      const x = Math.abs(label.entity.x) < 0.05 ? 0 : label.entity.x;
      const z = Math.abs(label.entity.z) < 0.05 ? 0 : label.entity.z;
      const speed =
        Math.abs(label.entity.speed) < 0.05 ? 0 : label.entity.speed;
      const headingValue =
        Math.abs(label.entity.heading || 0) < 0.005 ? 0 : label.entity.heading;
      label.position.textContent = `(x,z): (${x.toFixed(1)}, ${z.toFixed(1)}) m`;
      label.velocity.textContent = `velocity: ${speed.toFixed(1)} m/s`;
      label.heading.textContent = `heading: ${headingValue.toFixed(2)} rad`;
      const distance = Math.hypot(
        label.entity.x - ego.x,
        label.entity.z - ego.z,
      );
      const cameraDistance = this.camera.position.distanceTo(
        point.set(label.entity.x, label.height, label.entity.z),
      );
      const scale = THREE.MathUtils.clamp(
        0.81 - cameraDistance / 346.6667,
        0.465,
        0.75,
      );
      point
        .set(label.entity.x, label.height, label.entity.z)
        .project(this.camera);
      const visible =
        distance < 220 &&
        point.z > -1 &&
        point.z < 1 &&
        Math.abs(point.x) <= 1 &&
        Math.abs(point.y) <= 1;
      label.anchor.hidden = !visible;
      if (visible) {
        label.element.style.scale = String(scale);
        label.anchor.style.transform = `translate(${(point.x * 0.5 + 0.5) * width}px,${(-point.y * 0.5 + 0.5) * height}px) translate(-50%,-100%)`;
      }
    }
  }

  updateLockViews(width, height) {
    const ego = this.sim.player;
    const point = new THREE.Vector3();
    for (const view of this.lockViews) {
      const lock = this.sim.locks.get(view.node.id);
      const grant = this.sim.courtesy.get(view.node.id);
      const state = lock ? "locked" : "free";
      view.ring.material.color.set(LOCK_COLORS[state]);
      view.ring.material.opacity = lock ? 0.95 : 0.24;
      view.status.textContent = `status: ${lock ? "LOCKED" : "FREE"}`;
      view.owner.textContent = lock
        ? `owner: ${this.lockOwner(lock.id)}  age: ${Math.max(0, this.sim.time - lock.at).toFixed(1)} s`
        : "owner: none";
      view.courtesy.textContent = grant
        ? `courtesy: GRANTED to ${this.lockOwner(grant.id)}  age: ${Math.max(0, this.sim.time - grant.at).toFixed(1)} s  limits: ${COURTESY_RELEASE_RADIUS} m / 12 s`
        : "courtesy: NONE";
      view.courtesy.classList.toggle("active", !!grant);
      view.element.className = `geometry-lock-label ${state}`;
      const distance = Math.hypot(view.node.x - ego.x, view.node.z - ego.z);
      view.ring.visible = distance < 240;
      view.courtesyRing.visible = !!grant && distance < 240;
      point.set(view.node.x, 0.45, view.node.z).project(this.camera);
      const visible =
        distance < (lock ? 220 : 100) &&
        point.z > -1 &&
        point.z < 1 &&
        Math.abs(point.x) <= 1 &&
        Math.abs(point.y) <= 1;
      view.element.hidden = !visible;
      if (visible)
        view.element.style.transform = `translate(${(point.x * 0.5 + 0.5) * width}px,${(-point.y * 0.5 + 0.5) * height}px) translate(-50%,-100%)`;
    }
  }

  updateRoute() {
    const points = this.sim.player.route.points;
    if (points === this.routePoints) return;
    if (this.route) {
      this.route.removeFromParent();
      this.route.geometry.dispose();
      this.route.material.dispose();
    }
    if (this.routeMarkers) {
      this.routeMarkers.removeFromParent();
      this.routeMarkers.geometry.dispose();
      this.routeMarkers.material.dispose();
    }
    this.route = lineFromPoints(points, COLORS.route, { y: 0.1 });
    this.route.renderOrder = 2;
    this.scene.add(this.route);
    const markerGeometry = new THREE.BufferGeometry().setFromPoints(
      points.map((point) => new THREE.Vector3(point.x, 0.22, point.z)),
    );
    this.routeMarkers = new THREE.Points(
      markerGeometry,
      new THREE.PointsMaterial({
        color: COLORS.route,
        size: 2.5,
        sizeAttenuation: false,
        toneMapped: false,
      }),
    );
    this.routeMarkers.renderOrder = 3;
    this.scene.add(this.routeMarkers);
    this.routePoints = points;
  }

  updateRecoveryStatus() {
    const active = !!this.sim.lastPlan?.recovery?.active;
    this.recoveryStatus.textContent = `planMode: ${active ? "recovery" : "normal"}`;
    this.recoveryStatus.classList.toggle("active", active);
  }

  updateSafetyStatus() {
    const envelope = this.sim.lastSafetyEnvelope,
      recovery = !!this.sim.lastPlan?.recovery?.active,
      mode = !this.sim.autopilot
        ? "not applied · autopilot off"
        : !this.sim.safety
          ? "not applied · safety=false"
          : recovery
            ? "recovery guard"
            : "applied";
    this.safetyTitle.textContent =
      `speedEnvelope max ${this.formatSafetySpeed(envelope?.max)} · ${mode}`;
    const limits = envelope?.limits;
    for (const [key, label] of [
      ["world", "world"],
      ["route", "route"],
      ["destination", "destination"],
      ["following", "following"],
      ["conflict", "traffic conflict"],
    ]) {
      this.safetyRows[key].textContent = `${label}: ${this.formatSafetySpeed(limits?.[key])}`;
    }
    this.safetyStatus.hidden = false;
  }

  formatSafetySpeed(value) {
    if (value === Infinity) return "∞";
    if (!Number.isFinite(value)) return "—";
    return `${value.toFixed(1)} m/s · ${(value * 3.6).toFixed(0)} km/h`;
  }

  async prepare() {
    this.render(0, false);
    await this.renderer.compileAsync(this.scene, this.camera);
    this.render(0);
  }

  dispose() {
    this.resizeObserver.disconnect();
    this.disposeObjects();
    this.vectorLayer.replaceChildren();
    this.renderer.dispose();
  }

  render(dt, draw = true) {
    const { width, height } = this.viewport;
    if (!width || !height) return;
    if (
      this.canvas.width !== Math.floor(width * this.renderer.getPixelRatio()) ||
      this.canvas.height !== Math.floor(height * this.renderer.getPixelRatio())
    ) {
      this.renderer.setSize(width, height, false);
      this.camera.aspect = width / height;
      this.camera.updateProjectionMatrix();
    }

    this.updateRoute();
    this.updateRecoveryStatus();
    this.updateSafetyStatus();
    const ego = this.sim.player;
    this.player.position.x = ego.x;
    this.player.position.z = ego.z;
    this.player.rotation.y = -ego.heading;
    this.playerHeading.position.set(ego.x, 0, ego.z);
    this.playerHeading.rotation.y = -ego.heading;
    for (const vehicle of this.sim.traffic) {
      const outline = this.vehicles.get(vehicle.id);
      if (!outline) continue;
      outline.position.x = vehicle.x;
      outline.position.z = vehicle.z;
      outline.rotation.y = -vehicle.heading;
      const arrow = this.vehicleHeadings.get(vehicle.id);
      arrow.position.set(vehicle.x, 0, vehicle.z);
      arrow.rotation.y = -vehicle.heading;
    }
    for (const pedestrian of this.sim.pedestrians) {
      const outline = this.people.get(pedestrian.id);
      if (!outline) continue;
      outline.position.x = pedestrian.x;
      outline.position.z = pedestrian.z;
      outline.rotation.y = -(pedestrian.heading || 0);
      const arrow = this.pedestrianHeadings.get(pedestrian.id);
      arrow.position.set(pedestrian.x, 0, pedestrian.z);
      arrow.rotation.y = -(pedestrian.heading || 0);
    }
    for (const signal of this.signals) {
      const active = signalState(
        this.sim.world.byId[signal.control.nodeId],
        this.sim.time,
        signal.control.approach,
      ).color;
      for (const { color, lamp } of signal.lamps)
        lamp.material.color.set(
          color === active ? SIGNAL_COLORS[color] : SIGNAL_COLORS.off,
        );
    }

    let position;
    let look;
    if (this.mode === "hood") {
      const view = this.cameraInput.current();
      position = new THREE.Vector3(ego.x, 1.35, ego.z);
      const yaw = ego.heading + view.yaw;
      look = position
        .clone()
        .add(
          new THREE.Vector3(
            Math.sin(yaw) * Math.cos(view.pitch),
            Math.sin(view.pitch),
            -Math.cos(yaw) * Math.cos(view.pitch),
          ).multiplyScalar(25),
        );
    } else {
      const view = this.cameraInput.current();
      const yaw = ego.heading + view.yaw;
      const horizontal = Math.cos(view.pitch) * view.distance;
      position = new THREE.Vector3(
        ego.x - Math.sin(yaw) * horizontal,
        Math.sin(view.pitch) * view.distance + 0.7,
        ego.z + Math.cos(yaw) * horizontal,
      );
      const ahead = this.mode === "map" ? 0 : 5;
      look = new THREE.Vector3(
        ego.x + Math.sin(ego.heading) * ahead,
        0.7,
        ego.z - Math.cos(ego.heading) * ahead,
      );
    }
    const snap = this.snap || this.mode === "hood";
    this.camera.position.lerp(position, snap ? 1 : 1 - Math.exp(-dt * 4));
    this.look = this.look || look.clone();
    this.look.lerp(look, snap ? 1 : 1 - Math.exp(-dt * 6));
    this.camera.lookAt(this.look);
    this.snap = false;
    this.updateEntityLabels(width, height);
    this.updateLockViews(width, height);

    this.vectors.render(
      ego,
      this.camera,
      width,
      height,
      dt,
      this.sim.autopilot,
      this.sim.paused,
    );
    if (draw) this.renderer.render(this.scene, this.camera);
  }
}
