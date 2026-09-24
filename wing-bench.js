// ---------- Geometry utilities ----------
const Geo = {
  sub: (a, b) => ({ x: a.x - b.x, y: a.y - b.y }),
  add: (a, b) => ({ x: a.x + b.x, y: a.y + b.y }),
  mul: (a, k) => ({ x: a.x * k, y: a.y * k }),
  dot: (a, b) => a.x * b.x + a.y * b.y,
  cross: (a, b) => a.x * b.y - a.y * b.x,
  len: (a) => Math.hypot(a.x, a.y),
  dist: (a, b) => Math.hypot(a.x - b.x, a.y - b.y),
  // Parameter (0..1) of the projection of p onto segment ab, clamped.
  projT(p, a, b) {
    const ab = Geo.sub(b, a), d = Geo.dot(ab, ab);
    return d === 0 ? 0 : Math.max(0, Math.min(1, Geo.dot(Geo.sub(p, a), ab) / d));
  },
  lerp: (a, b, t) => ({ x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t }),
  distToSeg(p, a, b) { return Geo.dist(p, Geo.lerp(a, b, Geo.projT(p, a, b))); },
  segIntersect(a, b, c, d) {
    const r = Geo.sub(b, a), s = Geo.sub(d, c), den = Geo.cross(r, s);
    if (Math.abs(den) < 1e-9) return null;
    const t = Geo.cross(Geo.sub(c, a), s) / den, u = Geo.cross(Geo.sub(c, a), r) / den;
    return t >= 0 && t <= 1 && u >= 0 && u <= 1 ? Geo.lerp(a, b, t) : null;
  },
  inRect: (p, r) => p.x >= r.x && p.x <= r.x + r.w && p.y >= r.y && p.y <= r.y + r.h,
  // Liang–Barsky clip; returns [t0, t1] or null.
  clipSegRect(a, b, r) {
    const dx = b.x - a.x, dy = b.y - a.y;
    let t0 = 0, t1 = 1;
    const tests = [[-dx, a.x - r.x], [dx, r.x + r.w - a.x], [-dy, a.y - r.y], [dy, r.y + r.h - a.y]];
    for (const [p, q] of tests) {
      if (p === 0) { if (q < 0) return null; continue; }
      const t = q / p;
      if (p < 0) t0 = Math.max(t0, t); else t1 = Math.min(t1, t);
      if (t0 > t1) return null;
    }
    return [t0, t1];
  },
  deg: (rad) => rad * 180 / Math.PI,
  rad: (deg) => deg * Math.PI / 180,
};

// ---------- Bar helpers (world units = cm, y down) ----------
// A bar's `len` is the span between its end pins; the cut strip adds `margin` past each end pin.
const HOLE = { diameter: 0.8, margin: 1 };

const Bar = {
  cutLength: (b) => b.len + 2 * HOLE.margin,
  a: (b) => ({ x: b.x1, y: b.y1 }),
  b: (b) => ({ x: b.x2, y: b.y2 }),
  pointAt: (b, s) => Geo.lerp(Bar.a(b), Bar.b(b), b.len ? s / b.len : 0),
  sOf: (b, p) => Geo.projT(p, Bar.a(b), Bar.b(b)) * b.len,
  angle: (b) => Geo.deg(Math.atan2(-(b.y2 - b.y1), b.x2 - b.x1)),
  setEnds(b, a, c) { b.x1 = a.x; b.y1 = a.y; b.x2 = c.x; b.y2 = c.y; },
  // Place the bar at a given angle so that the point at distance s stays on `pivot`.
  orient(b, pivot, s, deg) {
    const u = { x: Math.cos(Geo.rad(deg)), y: -Math.sin(Geo.rad(deg)) };
    const a = Geo.sub(pivot, Geo.mul(u, s));
    Bar.setEnds(b, a, Geo.add(a, Geo.mul(u, b.len)));
  },
  // Local (along, across) coordinates of p relative to the bar.
  local(b, p) {
    const u = Geo.mul(Geo.sub(Bar.b(b), Bar.a(b)), 1 / (b.len || 1));
    const d = Geo.sub(p, Bar.a(b));
    return { along: Geo.dot(d, u), across: Geo.cross(u, d) };
  },
};

// ---------- Model ----------
const DEFAULT_BASE = { x: 5.1, y: 26.7, w: 20, h: 20 };
const STORE_KEY = "wing-bench-v1";
const DEFAULT_PERSON = { visible: false, height: 175, shoulders: 45, head: 25, raise: 0 };
const emptyState = (person = DEFAULT_PERSON) => ({ bars: [], joints: [], driverId: null, nextId: 1, base: { ...DEFAULT_BASE }, person: { ...person } });
let state = emptyState();

function makeBar(p, q, w = 1.6) {
  return { id: state.nextId++, x1: p.x, y1: p.y, x2: q.x, y2: q.y, len: Geo.dist(p, q), w };
}
const barById = (id) => state.bars.find((b) => b.id === id);

// Contact point between two bars, or null.
function contact(X, Y) {
  for (const [P, Q] of [[X, Y], [Y, X]]) {
    for (const e of [Bar.a(P), Bar.b(P)]) {
      if (Geo.distToSeg(e, Bar.a(Q), Bar.b(Q)) <= Q.w / 2) return e;
    }
  }
  return Geo.segIntersect(Bar.a(X), Bar.b(X), Bar.a(Y), Bar.b(Y));
}

function baseContacts(bar) {
  const ends = [Bar.a(bar), Bar.b(bar)].filter((e) => Geo.inRect(e, state.base));
  if (ends.length) return ends;
  const clip = Geo.clipSegRect(Bar.a(bar), Bar.b(bar), state.base);
  return clip ? [Geo.lerp(Bar.a(bar), Bar.b(bar), (clip[0] + clip[1]) / 2)] : [];
}

// Base contacts are free pins until attached (`fixed`); only fixed ones hold the bar.
const isAnchor = (j) => j.b === null && j.fixed;
const isActive = (j) => j.b !== null || j.fixed;

// Pins where `bar` touches the base; a pin keeps its attachment if a previous one was at the same place.
function baseJoints(bar, previous) {
  return baseContacts(bar).map((p) => {
    const sa = Bar.sOf(bar, p);
    const fixed = previous.some((j) => isAnchor(j) && j.a === bar.id && Math.abs(j.sa - sa) < bar.w);
    return { a: bar.id, sa, b: null, fx: p.x, fy: p.y, fixed };
  });
}

// Drop the joints linking a group of bars to other bars (and to the base unless `keepBase`); joints inside the group are kept.
function detachJoints(ids, keepBase = false) {
  state.joints = state.joints.filter((j) => !ids.has(j.a) && !ids.has(j.b) || ids.has(j.a) && ids.has(j.b) || keepBase && j.b === null);
}

function recomputeJoints(...bars) {
  const ids = new Set(bars.map((b) => b.id)), previous = state.joints;
  detachJoints(ids);
  for (const bar of bars) {
    for (const other of state.bars) {
      if (ids.has(other.id)) continue;
      const p = contact(bar, other);
      if (p) state.joints.push({ a: bar.id, sa: Bar.sOf(bar, p), b: other.id, sb: Bar.sOf(other, p) });
    }
    state.joints.push(...baseJoints(bar, previous));
  }
  // Let only the edited bars settle onto their new joints.
  solve(300, ids);
  detachDriverIfFree();
}

function recomputeAll() {
  const previous = state.joints;
  state.joints = [];
  state.bars.forEach((b, i) => {
    for (const o of state.bars.slice(i + 1)) {
      const p = contact(b, o);
      if (p) state.joints.push({ a: b.id, sa: Bar.sOf(b, p), b: o.id, sb: Bar.sOf(o, p) });
    }
    state.joints.push(...baseJoints(b, previous));
  });
}

function detachDriverIfFree() {
  if (state.driverId && !driverPivot()) state.driverId = null;
}

function driverPivot() {
  return state.joints.find((j) => isAnchor(j) && j.a === state.driverId) || null;
}

// ---------- Solver: position-based constraints ----------
function solve(iterations, movable = null) {
  const inv = (bar) => (movable ? (movable.has(bar.id) ? 1 : 0) : bar.id === state.driverId ? 0 : 1);
  const joints = state.joints.filter(isActive);
  for (let k = 0; k < iterations; k++) {
    for (const j of joints) {
      const A = barById(j.a), B = j.b === null ? null : barById(j.b);
      const pA = Bar.pointAt(A, j.sa);
      const pB = B ? Bar.pointAt(B, j.sb) : { x: j.fx, y: j.fy };
      const C = Geo.sub(pA, pB);
      const ta = A.len ? j.sa / A.len : 0, tb = B && B.len ? j.sb / B.len : 0;
      const wA = inv(A), wB = B ? inv(B) : 0;
      const den = wA * ((1 - ta) ** 2 + ta ** 2) + wB * ((1 - tb) ** 2 + tb ** 2);
      if (den === 0) continue;
      const l = Geo.mul(C, 1 / den);
      A.x1 -= wA * (1 - ta) * l.x; A.y1 -= wA * (1 - ta) * l.y;
      A.x2 -= wA * ta * l.x;       A.y2 -= wA * ta * l.y;
      if (B && wB) {
        B.x1 += wB * (1 - tb) * l.x; B.y1 += wB * (1 - tb) * l.y;
        B.x2 += wB * tb * l.x;       B.y2 += wB * tb * l.y;
      }
    }
    for (const b of state.bars) {
      const w = inv(b);
      if (!w) continue;
      const d = Geo.dist(Bar.a(b), Bar.b(b)) || 1e-9;
      const k2 = (d - b.len) / d / 2;
      const dx = (b.x2 - b.x1) * k2, dy = (b.y2 - b.y1) * k2;
      b.x1 += dx; b.y1 += dy; b.x2 -= dx; b.y2 -= dy;
    }
  }
}

function maxError() {
  let e = 0;
  for (const j of state.joints.filter(isActive)) {
    const pA = Bar.pointAt(barById(j.a), j.sa);
    const pB = j.b === null ? { x: j.fx, y: j.fy } : Bar.pointAt(barById(j.b), j.sb);
    e = Math.max(e, Geo.dist(pA, pB));
  }
  for (const b of state.bars) e = Math.max(e, Math.abs(Geo.dist(Bar.a(b), Bar.b(b)) - b.len));
  return e;
}

const cloneBars = () => state.bars.map((b) => ({ ...b }));
function restoreBars(snapshot) {
  snapshot.forEach((s) => Object.assign(barById(s.id) || {}, s));
}

// ---------- Motor ----------
const MAX_STEP = 1;        // degrees per sub-step
const TOLERANCE = 0.02;    // cm
let motor = { current: 0, target: 0, blocked: false };

function syncMotor() {
  const d = barById(state.driverId);
  motor.current = motor.target = d ? Bar.angle(d) : 0;
  motor.blocked = false;
  ui.slider.value = motor.current;
  updatePanel();
}

function stepMotor() {
  const d = barById(state.driverId), pivot = driverPivot();
  if (!d || !pivot) return false;
  const diff = motor.target - motor.current;
  if (Math.abs(diff) < 1e-6) return false;
  const next = motor.current + Math.sign(diff) * Math.min(MAX_STEP, Math.abs(diff));
  const snap = cloneBars();
  Bar.orient(d, { x: pivot.fx, y: pivot.fy }, pivot.sa, next);
  for (let i = 0; i < 40; i++) { solve(25); if (maxError() < TOLERANCE / 4) break; }
  if (maxError() > TOLERANCE) {
    restoreBars(snap);
    motor.target = motor.current;
    ui.slider.value = motor.current;
    motor.blocked = true;
  } else {
    motor.current = next;
    motor.blocked = false;
  }
  return true;
}

// ---------- Persistence ----------
// Undo/redo stacks of serialized states; the last `past` entry is the current state.
const History = {
  LIMIT: 100, past: [], future: [],
  record(snap) {
    if (snap === this.past.at(-1)) return;
    this.past.push(snap);
    if (this.past.length > this.LIMIT) this.past.shift();
    this.future = [];
  },
  undo() {
    if (this.past.length < 2) return null;
    this.future.push(this.past.pop());
    return this.past.at(-1);
  },
  redo() {
    const snap = this.future.pop();
    if (snap) this.past.push(snap);
    return snap ?? null;
  },
};

function persist(snap) {
  try { localStorage.setItem(STORE_KEY, snap); } catch (_) {}
}
function save() {
  const snap = serialize();
  History.record(snap);
  persist(snap);
}
function restore(snap) {
  if (!snap) return;
  applySnapshot(JSON.parse(snap));
  persist(snap);
  deselectAll(); syncMotor(); requestDraw();
}
function load() {
  try {
    applySnapshot(JSON.parse(localStorage.getItem(STORE_KEY)));
    return true;
  } catch (_) {}
  return false;
}
const serialize = () => JSON.stringify(state, null, 2);
function applySnapshot(d) {
  if (!d || !Array.isArray(d.bars) || !Array.isArray(d.joints)) throw new Error("Invalid file: missing bars or joints");
  delete d.restPose; // Legacy field from older saves.
  d.joints.forEach((j) => { if (j.b === null && j.fixed === undefined) j.fixed = true; });
  state = { ...emptyState(), ...d, person: { ...DEFAULT_PERSON, ...d.person } };
}

function commitEdit() { save(); renderList(); }

// Scale the whole mechanism (bars, pivots and base) around the base's top-left corner.
function scaleAll(k) {
  const o = { x: state.base.x, y: state.base.y };
  const f = (p) => Geo.add(o, Geo.mul(Geo.sub(p, o), k));
  state.bars.forEach((b) => { Bar.setEnds(b, f(Bar.a(b)), f(Bar.b(b))); b.len *= k; b.w *= k; });
  state.joints.forEach((j) => {
    j.sa *= k;
    if (j.b === null) { const p = f({ x: j.fx, y: j.fy }); j.fx = p.x; j.fy = p.y; }
    else j.sb *= k;
  });
  state.base.w *= k; state.base.h *= k;
  commitEdit(); syncMotor(); fitView(); requestDraw();
}

// Cardboard prototype linkage with a 45 cm wing spar (cm).
function loadExample() {
  state = emptyState(state.person);
  const P = (x, y) => ({ x, y });
  const A = P(23.78, 33.12), B = P(21.67, 39.01), C = P(27.86, 36.34), D = P(27.19, 42.66);
  const E = P(32.04, 45.87), T1 = P(29.89, 17.21), T2 = P(36.71, 20.72), TIP = P(69.92, 37.76);
  const bars = [makeBar(A, C), makeBar(B, E), makeBar(T1, D), makeBar(T2, E), makeBar(T1, TIP)];
  state.bars.push(...bars);
  recomputeAll();
  state.joints.filter((j) => j.b === null).forEach((j) => (j.fixed = true));
  state.driverId = bars[1].id;
  commitEdit();
  fitView();
  syncMotor();
}

// ---------- View ----------
const canvas = document.getElementById("stage");
const ctx = canvas.getContext("2d");
const view = { scale: 20, ox: 0, oy: 0 };
const toScreen = (p) => ({ x: p.x * view.scale + view.ox, y: p.y * view.scale + view.oy });
const toWorld = (x, y) => ({ x: (x - view.ox) / view.scale, y: (y - view.oy) / view.scale });
const css = (name) => getComputedStyle(document.documentElement).getPropertyValue(name).trim();
const COLORS = {};
["--mat-line", "--mat-line-strong", "--mat-ink", "--kraft", "--kraft-dark", "--pin", "--pin-fixed", "--motor", "--person"].forEach((n) => (COLORS[n] = css(n)));

function resize() {
  const dpr = window.devicePixelRatio || 1;
  canvas.width = innerWidth * dpr; canvas.height = innerHeight * dpr;
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  requestDraw();
}

function fitView() {
  const pts = state.bars.flatMap((b) => [Bar.a(b), Bar.b(b)]).concat([{ x: state.base.x, y: state.base.y }, { x: state.base.x + state.base.w, y: state.base.y + state.base.h }]);
  if (state.person.visible) pts.push(...Person.bounds());
  const minX = Math.min(...pts.map((p) => p.x)) - 3, maxX = Math.max(...pts.map((p) => p.x)) + 3;
  const minY = Math.min(...pts.map((p) => p.y)) - 3, maxY = Math.max(...pts.map((p) => p.y)) + 3;
  const panelW = innerWidth > 700 ? 340 : 0, listW = innerWidth > 700 && !$("list").hidden ? $("list").offsetWidth + 30 : 0;
  const freeW = innerWidth - panelW - listW;
  view.scale = Math.min(freeW / (maxX - minX), innerHeight / (maxY - minY));
  view.ox = panelW + (freeW - (maxX - minX) * view.scale) / 2 - minX * view.scale;
  view.oy = (innerHeight - (maxY - minY) * view.scale) / 2 - minY * view.scale;
}

// Grid step (cm) grows with zoom-out so lines and labels stay readable.
function drawGrid() {
  const tl = toWorld(0, 0), br = toWorld(innerWidth, innerHeight);
  const minor = [1, 5, 10, 50, 100].find((s) => s * view.scale >= 6) ?? 100, major = minor * 5;
  ctx.lineWidth = 1;
  ctx.font = `11px ${css("--font-num")}`;
  ctx.fillStyle = COLORS["--mat-ink"];
  const lines = (from, to, draw) => {
    for (let v = Math.floor(from / minor) * minor; v <= to; v += minor) {
      ctx.strokeStyle = v % major === 0 ? COLORS["--mat-line-strong"] : COLORS["--mat-line"];
      draw(v, v % major === 0);
    }
  };
  lines(tl.x, br.x, (x, label) => {
    const sx = Math.round(toScreen({ x, y: 0 }).x) + 0.5;
    ctx.beginPath(); ctx.moveTo(sx, 0); ctx.lineTo(sx, innerHeight); ctx.stroke();
    if (label) ctx.fillText(x, sx + 3, innerHeight - 8);
  });
  lines(tl.y, br.y, (y, label) => {
    const sy = Math.round(toScreen({ x: 0, y }).y) + 0.5;
    ctx.beginPath(); ctx.moveTo(0, sy); ctx.lineTo(innerWidth, sy); ctx.stroke();
    if (label) ctx.fillText(y, innerWidth - 24, sy - 3);
  });
}

// ---------- Wearer silhouette (restroom-sign pictogram, back view) ----------
// Proportions measured on the reference sign. Widths are fractions of the shoulder
// width, heights are fractions of the body below the neck.
const Person = {
  ARM: 0.23, SLIT: 0, LEG_GAP: 0.045, CORNER: 0.136, // x / shoulders
  LIMB_END: 0.2, // corner radius of hands and feet / limb width
  SLIT_TOP: 0.143, ARM_END: 0.472, CROTCH: 0.466, BACK: 0.2, // y / body length
  HEAD: 0.85, // head diameter / (skull top to neck base)

  // Body frame: origin at the top of the head on the spine, y down, in cm.
  frame() {
    const { height: H, shoulders: S, raise } = state.person;
    const neck = Math.min(state.person.head, H / 2), L = H - neck;
    const c = { x: state.base.x + state.base.w / 2, y: state.base.y + state.base.h / 2 };
    return { S, neck, L, o: { x: c.x, y: c.y - neck - L * Person.BACK + raise } };
  },
  bounds() {
    const { S, o } = Person.frame();
    return [{ x: o.x - S / 2, y: o.y }, { x: o.x + S / 2, y: o.y + state.person.height }];
  },
  // All parts in one path: they overlap, and a single fill paints the union once.
  path() {
    const { S, neck, L, o } = Person.frame(), P = Person;
    const arm = S * P.ARM, body = S - 2 * (arm + S * P.SLIT), leg = (body - S * P.LEG_GAP) / 2, r = S * P.CORNER;
    const path = new Path2D();
    const rect = (x, y, w, h, radii = [0]) => {
      const p = toScreen({ x: o.x + x, y: o.y + y });
      path.roundRect(p.x, p.y, w * view.scale, h * view.scale, radii.map((v) => v * view.scale));
    };
    const d = neck * P.HEAD, head = toScreen({ x: o.x, y: o.y + d / 2 });
    path.arc(head.x, head.y, (d / 2) * view.scale, 0, Math.PI * 2);
    rect(-S / 2, neck, S, L * P.SLIT_TOP, [r, r, 0, 0]);
    for (const x of [-S / 2, S / 2 - arm]) rect(x, neck + r, arm, L * P.ARM_END - r, [0, 0, arm * P.LIMB_END, arm * P.LIMB_END]);
    rect(-body / 2, neck, body, L * P.CROTCH);
    for (const x of [-body / 2, body / 2 - leg]) rect(x, neck + L * P.CROTCH - 1, leg, L * (1 - P.CROTCH) + 1, [0, 0, leg * P.LIMB_END, leg * P.LIMB_END]);
    return path;
  },
  draw() {
    if (!state.person.visible) return;
    ctx.fillStyle = COLORS["--person"];
    ctx.fill(Person.path(), "nonzero");
  },
};

function drawBase() {
  const p = toScreen(state.base), w = state.base.w * view.scale, h = state.base.h * view.scale;
  ctx.fillStyle = COLORS["--kraft-dark"];
  ctx.fillRect(p.x, p.y, w, h);
  ctx.save();
  ctx.beginPath(); ctx.rect(p.x, p.y, w, h); ctx.clip();
  ctx.strokeStyle = "rgba(0,0,0,.18)";
  for (let i = -h; i < w; i += 10) { ctx.beginPath(); ctx.moveTo(p.x + i, p.y + h); ctx.lineTo(p.x + i + h, p.y); ctx.stroke(); }
  ctx.restore();
  ctx.fillStyle = "rgba(255,255,255,.75)";
  ctx.font = `700 ${Math.max(11, view.scale * 0.8)}px ${css("--font-ui")}`;
  ctx.fillText("FIXED BASE", p.x + 8, p.y + h - 8);
}

function drawBar(b) {
  const a = toScreen(Bar.a(b)), c = toScreen(Bar.b(b));
  const ang = Math.atan2(c.y - a.y, c.x - a.x), L = b.len * view.scale, W = b.w * view.scale;
  const isDriver = b.id === state.driverId, isSel = isSelected(b.id);
  ctx.save();
  ctx.translate(a.x, a.y); ctx.rotate(ang);
  ctx.fillStyle = COLORS["--kraft"];
  ctx.strokeStyle = isDriver ? COLORS["--motor"] : COLORS["--kraft-dark"];
  ctx.lineWidth = isDriver ? 3 : 1;
  ctx.shadowColor = "rgba(0,0,0,.3)"; ctx.shadowBlur = 6; ctx.shadowOffsetY = 2;
  const M = HOLE.margin * view.scale;
  ctx.fillRect(-M, -W / 2, L + 2 * M, W);
  ctx.shadowColor = "transparent";
  ctx.strokeRect(-M, -W / 2, L + 2 * M, W);
  // Corrugation line, like the cardboard strips.
  ctx.strokeStyle = "rgba(90,60,30,.25)"; ctx.lineWidth = 1;
  ctx.beginPath(); ctx.moveTo(-M, -W * 0.18); ctx.lineTo(L + M, -W * 0.18); ctx.stroke();
  if (isSel) {
    ctx.setLineDash([6, 4]); ctx.strokeStyle = "#fff"; ctx.lineWidth = 2;
    ctx.strokeRect(-M - 3, -W / 2 - 3, L + 2 * M + 6, W + 6);
    ctx.setLineDash([]);
  }
  if (isSel || b.id === ui.hoverId) {
    ctx.fillStyle = "rgba(255,255,255,.85)";
    for (const x of [0, L]) { ctx.beginPath(); ctx.arc(x, 0, Math.max(4, W * 0.18), 0, Math.PI * 2); ctx.fill(); }
  }
  ctx.restore();
  if (isSel || ui.drag?.bar === b) {
    const mid = Geo.lerp(a, c, 0.5);
    ctx.fillStyle = "#fff"; ctx.font = `12px ${css("--font-num")}`;
    ctx.fillText(`${b.len.toFixed(1)} cm`, mid.x + 8, mid.y - 8);
  }
}

const pinRadius = () => Math.max(4, view.scale * HOLE.diameter / 2);

function drawJoint(j) {
  const p = toScreen(Bar.pointAt(barById(j.a), j.sa));
  const r = pinRadius();
  ctx.fillStyle = COLORS[j.fixed ? "--pin-fixed" : "--pin"];
  ctx.beginPath(); ctx.arc(p.x, p.y, r, 0, Math.PI * 2); ctx.fill();
  ctx.fillStyle = "#3a1010";
  ctx.beginPath(); ctx.arc(p.x, p.y, r * 0.3, 0, Math.PI * 2); ctx.fill();
  if (j.b === null) {
    ctx.strokeStyle = "#fff"; ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.arc(p.x, p.y, r + 3, 0, Math.PI * 2); ctx.stroke();
  }
}

function draw() {
  ctx.clearRect(0, 0, innerWidth, innerHeight);
  drawGrid();
  Person.draw();
  drawBase();
  state.bars.forEach(drawBar);
  state.joints.forEach(drawJoint);
  if (ui.drag?.box) drawSelectionBox(ui.drag);
}

function drawSelectionBox(d) {
  const a = toScreen(d.start), b = toScreen(d.cur);
  ctx.setLineDash([5, 4]); ctx.strokeStyle = "#fff"; ctx.lineWidth = 1;
  ctx.fillStyle = "rgba(255,255,255,.08)";
  ctx.fillRect(a.x, a.y, b.x - a.x, b.y - a.y);
  ctx.strokeRect(a.x, a.y, b.x - a.x, b.y - a.y);
  ctx.setLineDash([]);
}

let drawQueued = false;
function requestDraw() {
  if (drawQueued) return;
  drawQueued = true;
  requestAnimationFrame(() => { drawQueued = false; draw(); });
}

function loop() {
  let moved = false;
  for (let i = 0; i < 4 && stepMotor(); i++) moved = true;
  if (moved) { updatePanel(); requestDraw(); }
  requestAnimationFrame(loop);
}

// ---------- UI ----------
const $ = (id) => document.getElementById(id);
const ui = {
  slider: $("slider"), menu: $("menu"), selectedId: null, group: new Set(), hoverId: null, drag: null, clearArmed: false,
};

function updatePanel() {
  const d = barById(state.driverId);
  $("angleOut").textContent = d ? `${motor.current.toFixed(1)}°` : "—";
  ui.slider.disabled = !d;
  const st = $("status");
  st.className = "chip " + (!d ? "" : motor.blocked ? "bad" : "ok");
  st.textContent = !d ? "No motor bar" : motor.blocked ? "Jammed at this position" : "Moving freely";
  $("counts").textContent = `${state.bars.length} bars · ${state.joints.length} pins`;
  $("btnExample").hidden = state.bars.length > 0;
}

function openMenu(bar, sx, sy) {
  ui.group.clear();
  ui.selectedId = bar.id;
  $("menuTitle").textContent = `Bar #${bar.id}`;
  $("inLen").value = bar.len.toFixed(1);
  $("inWidth").value = bar.w.toFixed(1);
  $("inAngle").value = Bar.angle(bar).toFixed(0);
  const isDriver = bar.id === state.driverId;
  const btn = $("btnDriver");
  btn.classList.toggle("on", isDriver);
  btn.textContent = isDriver ? "Motor ✓" : "Set as motor";
  showPanelAt(ui.menu, sx, sy);
  renderList();
  requestDraw();
}
const hidePopups = () => document.querySelectorAll(".popup").forEach((p) => (p.hidden = true));
function closeMenu() { hidePopups(); ui.selectedId = null; ui.pin = null; renderList(); requestDraw(); }
function deselectAll() { ui.group.clear(); closeMenu(); }
const isSelected = (id) => id === ui.selectedId || ui.group.has(id);

// Shift/Ctrl+click: add or remove a bar from the multi-selection (the edited bar joins it).
function toggleInGroup(bar) {
  const current = ui.selectedId;
  closeMenu();
  if (current) ui.group.add(current);
  if (!ui.group.delete(bar.id)) ui.group.add(bar.id);
  requestDraw();
}

function deleteBars(ids) {
  state.bars = state.bars.filter((b) => !ids.has(b.id));
  state.joints = state.joints.filter((j) => !ids.has(j.a) && !ids.has(j.b));
  if (ids.has(state.driverId)) state.driverId = null;
  deselectAll(); commitEdit(); syncMotor();
}

function showPanelAt(panel, sx, sy) {
  hidePopups();
  panel.hidden = false;
  const r = panel.getBoundingClientRect();
  panel.style.left = Math.max(16, Math.min(sx + 16, innerWidth - r.width - 16)) + "px";
  panel.style.top = Math.max(16, Math.min(sy + 16, innerHeight - r.height - 16)) + "px";
}

function openBaseMenu(sx, sy) {
  closeMenu();
  const { base, person } = state;
  $("inBaseW").value = base.w.toFixed(1);
  $("inBaseH").value = base.h.toFixed(1);
  $("inPersonOn").checked = person.visible;
  $("inPersonH").value = person.height;
  $("inPersonS").value = person.shoulders;
  $("inPersonHead").value = person.head;
  $("inPersonDy").value = person.raise;
  showPanelAt($("baseMenu"), sx, sy);
}

// Bind a numeric input to a field; `min` clamps it, invalid input keeps the old value.
function bindNumber(id, obj, key, min, after) {
  $(id).addEventListener("change", (e) => {
    const v = parseFloat(e.target.value);
    if (Number.isFinite(v)) obj()[key] = min === null ? v : Math.max(min, v);
    e.target.value = obj()[key];
    after();
  });
}
const onBaseResized = () => { recomputeAll(); detachDriverIfFree(); commitEdit(); syncMotor(); requestDraw(); };
const onPersonChanged = () => { save(); if (state.person.visible) fitView(); requestDraw(); };
bindNumber("inBaseW", () => state.base, "w", 1, onBaseResized);
bindNumber("inBaseH", () => state.base, "h", 1, onBaseResized);
bindNumber("inPersonH", () => state.person, "height", 50, onPersonChanged);
bindNumber("inPersonS", () => state.person, "shoulders", 10, onPersonChanged);
bindNumber("inPersonHead", () => state.person, "head", 5, onPersonChanged);
bindNumber("inPersonDy", () => state.person, "raise", null, onPersonChanged);
$("inPersonOn").addEventListener("change", (e) => { state.person.visible = e.target.checked; save(); fitView(); requestDraw(); });
$("baseMenuClose").addEventListener("click", closeMenu);

function editSelected(fn) {
  const bar = barById(ui.selectedId);
  if (!bar) return;
  fn(bar);
  recomputeJoints(bar);
  commitEdit(); syncMotor(); requestDraw();
}

$("inLen").addEventListener("change", (e) => editSelected((b) => {
  const ang = Bar.angle(b);
  b.len = Math.max(0.5, +e.target.value || b.len);
  Bar.orient(b, Bar.a(b), 0, ang);
}));
$("inWidth").addEventListener("change", (e) => editSelected((b) => { b.w = Math.max(0.3, +e.target.value || b.w); }));
$("inAngle").addEventListener("change", (e) => editSelected((b) => Bar.orient(b, Bar.a(b), 0, +e.target.value || 0)));
$("btnDriver").addEventListener("click", () => {
  const bar = barById(ui.selectedId);
  if (!bar) return;
  if (state.driverId === bar.id) state.driverId = null;
  else if (state.joints.some((j) => isAnchor(j) && j.a === bar.id)) state.driverId = bar.id;
  else { $("status").className = "chip bad"; $("status").textContent = "The motor must be pinned to the base"; return; }
  save(); syncMotor(); openMenu(bar, parseFloat(ui.menu.style.left) - 16, parseFloat(ui.menu.style.top) - 16);
});
$("btnDelete").addEventListener("click", () => deleteBars(new Set([ui.selectedId])));
$("menuClose").addEventListener("click", closeMenu);

function openPinMenu(joint, sx, sy) {
  closeMenu();
  ui.pin = joint;
  $("btnAnchor").textContent = joint.fixed ? "Detach from base" : "Attach to base";
  showPanelAt($("pinMenu"), sx, sy);
}
$("btnAnchor").addEventListener("click", () => {
  const j = ui.pin;
  if (!j) return;
  // Anchor where the pin is now: the motor may have moved the bar since the contact was found.
  const p = Bar.pointAt(barById(j.a), j.sa);
  Object.assign(j, { fx: p.x, fy: p.y, fixed: !j.fixed });
  detachDriverIfFree();
  closeMenu(); commitEdit(); syncMotor();
});
$("pinMenuClose").addEventListener("click", closeMenu);

// Hole centres along the cut strip (cm from its first cut end), one per distinct active pin.
function holes(bar) {
  const s = state.joints.filter(isActive).flatMap((j) => (j.a === bar.id ? [j.sa] : j.b === bar.id ? [j.sb] : [])).map((v) => v + HOLE.margin);
  return s.sort((x, y) => x - y).filter((v, i, all) => i === 0 || v - all[i - 1] > 0.05);
}

function renderList() {
  $("listBody").replaceChildren(...state.bars.map((b) => {
    const tr = document.createElement("tr");
    tr.classList.toggle("sel", isSelected(b.id));
    tr.innerHTML = `<td>#${b.id}${b.id === state.driverId ? " ⚙" : ""}</td><td>${Bar.cutLength(b).toFixed(1)}</td><td>${b.w.toFixed(1)}</td><td>${holes(b).map((v) => v.toFixed(1)).join(" · ")}</td>`;
    tr.addEventListener("click", () => {
      const m = toScreen(Geo.lerp(Bar.a(b), Bar.b(b), 0.5));
      openMenu(b, m.x, m.y);
    });
    return tr;
  }));
  $("listTotal").textContent = state.bars.reduce((t, b) => t + Bar.cutLength(b), 0).toFixed(1);
}
// Published artifact: platform download prompt. Local file: plain blob link.
const downloadsReady = window.claude?.use ? window.claude.use("downloads") : Promise.resolve(null);
$("btnExport").addEventListener("click", async () => {
  const filename = `wings-${new Date().toISOString().slice(0, 16).replace(/[:T]/g, "-")}.json`;
  const downloads = await downloadsReady;
  if (downloads) {
    downloads.save({ filename, data: serialize() }).catch(() => {});
    return;
  }
  const a = document.createElement("a");
  a.href = URL.createObjectURL(new Blob([serialize()], { type: "application/json" }));
  a.download = filename;
  a.click();
  setTimeout(() => URL.revokeObjectURL(a.href), 1000);
});
$("btnImport").addEventListener("click", () => $("fileImport").click());
$("fileImport").addEventListener("change", async (e) => {
  const file = e.target.files[0];
  e.target.value = "";
  if (!file) return;
  try {
    applySnapshot(JSON.parse(await file.text()));
    closeMenu(); save(); renderList(); fitView(); syncMotor(); requestDraw();
  } catch (err) {
    $("status").className = "chip bad";
    $("status").textContent = err instanceof SyntaxError ? "Unreadable JSON file" : err.message;
  }
});
$("btnList").addEventListener("click", () => { $("list").hidden = !$("list").hidden; renderList(); });
$("listClose").addEventListener("click", () => { $("list").hidden = true; });
$("btnScale").addEventListener("click", () => {
  const k = +$("inScale").value;
  if (k > 0 && k !== 1) scaleAll(k);
});

ui.slider.addEventListener("input", () => { motor.target = +ui.slider.value; });
$("btnExample").addEventListener("click", () => { closeMenu(); loadExample(); requestDraw(); });
$("btnClear").addEventListener("click", (e) => {
  if (!ui.clearArmed) {
    ui.clearArmed = true; e.target.textContent = "Confirm?";
    setTimeout(() => { ui.clearArmed = false; e.target.textContent = "Clear all"; }, 2500);
    return;
  }
  ui.clearArmed = false; e.target.textContent = "Clear all";
  state = emptyState(state.person);
  closeMenu(); commitEdit(); syncMotor();
});

// ---------- Pointer interactions ----------
function hitEndpoint(p) {
  const order = [...state.bars].reverse().sort((a, b) => (b.id === ui.selectedId) - (a.id === ui.selectedId));
  for (const b of order) {
    const r = Math.max(b.w * 0.35, 8 / view.scale);
    if (Geo.dist(p, Bar.a(b)) < r) return { bar: b, end: 1 };
    if (Geo.dist(p, Bar.b(b)) < r) return { bar: b, end: 2 };
  }
  return null;
}
function hitBar(p) {
  for (const b of [...state.bars].reverse()) {
    const l = Bar.local(b, p);
    if (l.along >= -HOLE.margin && l.along <= b.len + HOLE.margin && Math.abs(l.across) <= b.w / 2) return b;
  }
  return null;
}
function hitBasePin(p) {
  const r = (pinRadius() + 3) / view.scale;
  return state.joints.find((j) => j.b === null && Geo.dist(p, Bar.pointAt(barById(j.a), j.sa)) < r) || null;
}
// Snap a point to the endpoint of a bar outside `ids` when close enough.
function snap(p, ids) {
  const r = 10 / view.scale;
  for (const b of state.bars) {
    if (ids.has(b.id)) continue;
    for (const e of [Bar.a(b), Bar.b(b)]) if (Geo.dist(p, e) < r) return e;
  }
  return p;
}

const pointer = (e) => ({ sx: e.clientX, sy: e.clientY, w: toWorld(e.clientX, e.clientY) });

canvas.addEventListener("pointerdown", (e) => {
  canvas.setPointerCapture(e.pointerId);
  const { sx, sy, w } = pointer(e);
  const ep = hitEndpoint(w), bar = ep ? ep.bar : hitBar(w), multi = e.shiftKey || e.ctrlKey || e.metaKey;
  if (bar && multi) { toggleInGroup(bar); return; }
  if (!bar) {
    ui.drag = multi ? { box: true, start: w, cur: w, sx, sy, moved: false } : { pan: true, sx, sy, ox: view.ox, oy: view.oy, moved: false };
    return;
  }
  // Dragging the body of a selected bar moves the whole selection; an endpoint only reshapes its bar.
  const movers = !ep && ui.group.has(bar.id) ? state.bars.filter((b) => ui.group.has(b.id)) : [bar];
  const ids = new Set(movers.map((b) => b.id));
  const pin = hitBasePin(w);
  detachJoints(ids, true);
  ui.drag = { bar, ids, pin, movers: movers.map((b) => ({ bar: b, orig: { ...b } })), end: ep?.end ?? 0, start: w, sx, sy, orig: { ...bar }, moved: false };
  requestDraw();
});

canvas.addEventListener("pointermove", (e) => {
  const { sx, sy, w } = pointer(e);
  const d = ui.drag;
  if (!d) {
    const hit = hitEndpoint(w)?.bar || hitBar(w);
    const id = hit ? hit.id : null;
    canvas.style.cursor = hitEndpoint(w) ? "crosshair" : hit ? "move" : Geo.inRect(w, state.base) ? "pointer" : "default";
    if (id !== ui.hoverId) { ui.hoverId = id; requestDraw(); }
    return;
  }
  if (Math.hypot(sx - d.sx, sy - d.sy) > 3) d.moved = true;
  if (!d.moved) return;
  if (d.pan) { view.ox = d.ox + sx - d.sx; view.oy = d.oy + sy - d.sy; }
  else if (d.box) d.cur = w;
  else if (d.end) {
    const p = snap(w, d.ids), o = d.end === 1 ? { x: d.orig.x2, y: d.orig.y2 } : { x: d.orig.x1, y: d.orig.y1 };
    if (d.end === 1) Bar.setEnds(d.bar, p, o); else Bar.setEnds(d.bar, o, p);
    d.bar.len = Geo.dist(Bar.a(d.bar), Bar.b(d.bar));
  } else {
    const delta = Geo.sub(w, d.start);
    const moved = (o) => [Geo.add({ x: o.x1, y: o.y1 }, delta), Geo.add({ x: o.x2, y: o.y2 }, delta)];
    const [a, c] = moved(d.orig), sa = snap(a, d.ids), sc = snap(c, d.ids);
    const shift = Geo.add(delta, sa !== a ? Geo.sub(sa, a) : sc !== c ? Geo.sub(sc, c) : { x: 0, y: 0 });
    for (const m of d.movers) Bar.setEnds(m.bar, Geo.add({ x: m.orig.x1, y: m.orig.y1 }, shift), Geo.add({ x: m.orig.x2, y: m.orig.y2 }, shift));
  }
  requestDraw();
});

canvas.addEventListener("pointerup", (e) => {
  const d = ui.drag;
  ui.drag = null;
  if (!d) return;
  if (d.box) {
    const r = { x: Math.min(d.start.x, d.cur.x), y: Math.min(d.start.y, d.cur.y), w: Math.abs(d.cur.x - d.start.x), h: Math.abs(d.cur.y - d.start.y) };
    state.bars.filter((b) => Geo.inRect(Bar.a(b), r) && Geo.inRect(Bar.b(b), r)).forEach((b) => ui.group.add(b.id));
  } else if (d.bar) {
    recomputeJoints(...d.movers.map((m) => m.bar));
    if (d.moved) { commitEdit(); syncMotor(); if (!ui.menu.hidden && ui.selectedId === d.bar.id) openMenu(d.bar, e.clientX, e.clientY); }
    else if (d.pin) openPinMenu(state.joints.find((j) => j.b === null && j.a === d.pin.a && Math.abs(j.sa - d.pin.sa) < 1e-6) || d.pin, e.clientX, e.clientY);
    else openMenu(d.bar, e.clientX, e.clientY);
  } else if (!d.moved) {
    if (Geo.inRect(toWorld(e.clientX, e.clientY), state.base)) openBaseMenu(e.clientX, e.clientY);
    else deselectAll();
  }
  requestDraw();
});

canvas.addEventListener("dblclick", (e) => {
  const { w } = pointer(e);
  if (hitBar(w)) return;
  const bar = makeBar({ x: w.x - 5, y: w.y }, { x: w.x + 5, y: w.y });
  state.bars.push(bar);
  recomputeJoints(bar);
  commitEdit(); syncMotor();
  openMenu(bar, e.clientX, e.clientY);
});

canvas.addEventListener("wheel", (e) => {
  e.preventDefault();
  const before = toWorld(e.clientX, e.clientY);
  view.scale = Math.max(4, Math.min(150, view.scale * Math.exp(-e.deltaY * 0.0015)));
  view.ox = e.clientX - before.x * view.scale;
  view.oy = e.clientY - before.y * view.scale;
  requestDraw();
}, { passive: false });

addEventListener("keydown", (e) => {
  if (e.key === "Escape") deselectAll();
  const key = e.key.toLowerCase();
  if ((e.ctrlKey || e.metaKey) && (key === "z" || key === "y") && document.activeElement.tagName !== "INPUT") {
    e.preventDefault();
    restore(key === "y" || e.shiftKey ? History.redo() : History.undo());
  }
  if ((e.ctrlKey || e.metaKey) && key === "a" && document.activeElement.tagName !== "INPUT") {
    e.preventDefault();
    closeMenu();
    state.bars.forEach((b) => ui.group.add(b.id));
    requestDraw();
  }
  if ((e.key === "Delete" || e.key === "Backspace") && document.activeElement.tagName !== "INPUT") {
    const ids = ui.group.size ? new Set(ui.group) : new Set(ui.selectedId ? [ui.selectedId] : []);
    if (ids.size) deleteBars(ids);
  }
});
addEventListener("resize", resize);

// ---------- Boot ----------
resize();
if (load() && state.bars.length) { fitView(); syncMotor(); } else loadExample();
History.record(serialize());
updatePanel();
requestAnimationFrame(loop);
