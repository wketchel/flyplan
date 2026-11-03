'use strict';
/* Waypoint Planner — DJI WPML (KMZ) mission builder for the Air 3S.
   All distances are stored in meters and speeds in m/s; the UI converts for display. */

const FT = 3.28084, MPH = 2.236936;
const FRAME_W = 34.6, FRAME_H = 25.95;          // 35 mm-equivalent 4:3 frame (mm)
const CAMERAS = {
  wide: { label: 'Wide 24 mm (1x)', f: 24, px: 8192 },
  tele: { label: 'Medium tele 70 mm (3x)', f: 70, px: 8064 },
};
const MAX_ALT = 121.92;                          // 400 ft
const HZ_DEFAULTS = { over: { top: 15.24, buf: 15.24 }, nofly: { top: 0, buf: 10 } };
const STOP_PENALTY_S = 2.5;                      // time lost decelerating at a stop-and-turn waypoint
const native = window.Android || null;
const PC = !!window.PcHost;
const DESKTOP = PC || !!window.WinHost; // browser PC version or the Windows app
const DEV = DESKTOP ? 'this PC' : 'this tablet';

const DEFAULTS = {
  v: 1, units: 'imperial', name: 'Mission 1',
  alt: 60.96, speed: 6, finish: 'goHome', rcLost: 'goBack', turn: 'stop', defGimbal: -45,
  camera: 'wide', photoSize: '12', front: 75, side: 70, spacingAuto: true, spacing: 20,
  dir: 'auto', angle: 0, reverse: false, gridGimbal: -90, runIn: 10, pathStyle: 'anchors',
  split: { on: true, minutes: 30, points: 190, sel: -1 },
  terrain: { on: false, takeoff: null, tol: 4.5 },
  goal: { gsd: 0.04064, batteries: 4, minutes: 20, quality: 'standard', interval: 3, shutter: 1000 },
  interval: { on: false, type: 'time', auto: false, value: 3, from: 0, to: -1, method: 'camera' },
  poi: { radius: 30, points: 12, height: 0, cw: true, start: 180, gimbalAuto: true, gimbal: -30, loop: true, rings: 1, altStep: 15.24, radStep: 0, climb: 'step' },
  shape: null, poiCenter: null, mode: 'waypoint', wps: [], sel: -1, libId: null,
  droneEnum: null, droneSub: null, home: null, segs: [], seg: 0, hzClimb: true, hzAvoid: true, hzMargin: 10,
  view: { c: [39.5, -98.35], z: 4 },
};

let S = load();
function load() {
  try {
    const raw = localStorage.getItem('planner.v1');
    if (raw) {
      const st = Object.assign(clone(DEFAULTS), JSON.parse(raw));
      if (st.rcLost !== 'hover') st.rcLost = 'goBack';
      if (!st.interval.method) st.interval.method = 'points';
      if (st.interval.method === 'trigger') { st.interval.method = 'camera'; if (st.interval.type !== 'time') st.interval.value = 3; st.interval.type = 'time'; st.interval.auto = false; }
      if (!st.split) st.split = clone(DEFAULTS.split);
      if (st.pathStyle !== 'straightx') st.pathStyle = 'anchors';
      if (!st.photoSize) st.photoSize = '12';
      if (!st.terrain) st.terrain = clone(DEFAULTS.terrain);
      if (!st.goal) st.goal = clone(DEFAULTS.goal);
      st.poi = Object.assign(clone(DEFAULTS.poi), st.poi || {});
      if (!Array.isArray(st.segs)) st.segs = [];
      if (!st.def2) { // new defaults (v1.29): 30-minute battery parts and DJI Fly Timed Shot
        st.def2 = true;
        st.split = Object.assign({}, st.split, { on: true, minutes: 30, sel: -1 });
        st.interval = Object.assign({}, st.interval, { method: 'camera', type: 'time', auto: false });
        if (!(st.interval.value >= 0.5)) st.interval.value = 3;
      }
      if (!(st.seg >= 0 && st.seg < st.segs.length)) st.seg = 0;
      return st;
    }
  } catch (e) {}
  return Object.assign(clone(DEFAULTS), { def2: true });
}
function save() { try { localStorage.setItem('planner.v1', JSON.stringify(S)); } catch (e) {} }
function clone(o) { return JSON.parse(JSON.stringify(o)); }

/* ---------- units ---------- */
const imp = () => S.units === 'imperial';
const LEN = () => imp() ? 'ft' : 'm';
const SPD = () => imp() ? 'mph' : 'm/s';
const toLen = m => imp() ? m * FT : m;
const fromLen = v => imp() ? v / FT : v;
const toSpd = s => imp() ? s * MPH : s;
const fromSpd = v => imp() ? v / MPH : v;
const r1 = n => Math.round(n * 10) / 10;
const r0 = n => Math.round(n);
function fmtLen(m) { return r0(toLen(m)).toLocaleString() + ' ' + LEN(); }
function fmtDist(m) {
  if (imp()) { const ft = m * FT; return ft > 2640 ? r1(ft / 5280) + ' mi' : r0(ft).toLocaleString() + ' ft'; }
  return m > 1000 ? r1(m / 1000) + ' km' : r0(m) + ' m';
}
function fmtTime(s) { s = Math.round(s); const m = Math.floor(s / 60); return m + ':' + String(s % 60).padStart(2, '0'); }

/* ---------- geometry ---------- */
function hav(a, b) {
  const R = 6371008.8, t = Math.PI / 180;
  const dLat = (b[0] - a[0]) * t, dLng = (b[1] - a[1]) * t;
  const h = Math.sin(dLat / 2) ** 2 + Math.cos(a[0] * t) * Math.cos(b[0] * t) * Math.sin(dLng / 2) ** 2;
  return 2 * R * Math.asin(Math.sqrt(h));
}
function projector(origin) {
  const kx = 111320 * Math.cos(origin[0] * Math.PI / 180), ky = 110540;
  return {
    xy: p => [(p[1] - origin[1]) * kx, (p[0] - origin[0]) * ky],
    ll: q => [origin[0] + q[1] / ky, origin[1] + q[0] / kx],
  };
}
function offset(p, bearingDeg, dist) {
  const b = bearingDeg * Math.PI / 180;
  return [p[0] + dist * Math.cos(b) / 110540, p[1] + dist * Math.sin(b) / (111320 * Math.cos(p[0] * Math.PI / 180))];
}
function centroid(pts) { let a = 0, b = 0; pts.forEach(p => { a += p[0]; b += p[1]; }); return [a / pts.length, b / pts.length]; }
function polyArea(xy) { let s = 0; for (let i = 0; i < xy.length; i++) { const a = xy[i], b = xy[(i + 1) % xy.length]; s += a[0] * b[1] - b[0] * a[1]; } return Math.abs(s / 2); }

/* Lawnmower sweep. Lines run along bearing `ang` (0 = north–south lines, 90 = east–west). */
function sweep(P, ang, sp, ext, holes) {
  ext = ext || 0;
  const t = ang * Math.PI / 180, s = Math.sin(t), c = Math.cos(t);
  const rot = R => R.map(([x, y]) => [x * s + y * c, -x * c + y * s]);
  const Q = rot(P), rings = [Q].concat((holes || []).map(rot));
  let vmin = Infinity, vmax = -Infinity;
  Q.forEach(q => { vmin = Math.min(vmin, q[1]); vmax = Math.max(vmax, q[1]); });
  const width = vmax - vmin;
  const n = Math.max(1, Math.ceil(width / sp - 1e-9));
  const off = (width - (n - 1) * sp) / 2;
  const lines = [];
  for (let i = 0; i < n; i++) {
    const v = vmin + off + i * sp, xs = [];
    rings.forEach((R, ri) => {
      for (let k = 0; k < R.length; k++) {
        const a = R[k], b = R[(k + 1) % R.length];
        if ((a[1] <= v && b[1] > v) || (b[1] <= v && a[1] > v)) xs.push({ x: a[0] + (v - a[1]) * (b[0] - a[0]) / (b[1] - a[1]), ring: ri });
      }
    });
    xs.sort((p, q) => p.x - q.x);
    const segs = [];
    // Inside = inside the area and outside every hole (holes are no-fly zones the lines stop at, with no run-in).
    const par = new Array(rings.length).fill(false);
    let start = null, startHole = false;
    const inside = () => par[0] && par.every((v2, k) => k === 0 || !v2);
    xs.forEach(e => {
      const was = inside(); par[e.ring] = !par[e.ring]; const now = inside();
      if (!was && now) { start = e.x; startHole = e.ring > 0; }
      if (was && !now && e.x - start > 0.5) segs.push([start, e.x, startHole, e.ring > 0]);
    });
    if (segs.length) lines.push({ v, segs });
  }
  const pts = [];
  lines.forEach((L, i) => {
    let segs = L.segs.map(g => g.slice());
    if (i % 2) segs = segs.reverse().map(g => [g[1], g[0], g[3], g[2]]);
    segs.forEach(g => { const dir = g[1] >= g[0] ? 1 : -1; pts.push([g[0] - dir * (g[2] ? 0 : ext), L.v], [g[1] + dir * (g[3] ? 0 : ext), L.v]); });
  });
  return { pts: pts.map(([u, v]) => [u * s - v * c, u * c + v * s]), lines: lines.length };
}
function pathTime(pts, speed) {
  let d = 0; for (let i = 1; i < pts.length; i++) d += Math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]);
  return d / speed + pts.length * STOP_PENALTY_S;
}
const DIRS = { ns: 0, ew: 90, nesw: 45, nwse: 135 };

/* ---------- camera / coverage ---------- */
function coverage() {
  const cam = CAMERAS[S.camera] || CAMERAS.wide;
  const across = S.alt * FRAME_W / cam.f, along = S.alt * FRAME_H / cam.f;
  const spacing = S.spacingAuto ? across * (1 - S.side / 100) : S.spacing;
  const alongDist = along * (1 - S.front / 100);
  const autoInterval = S.interval.type === 'time' ? alongDist / S.speed : alongDist;
  const iv = S.interval.auto ? autoInterval : S.interval.value;
  const shotSpacing = S.interval.type === 'time' ? iv * S.speed : iv;
  return {
    across, along, spacing, sideEff: 100 * (1 - spacing / across), alongDist,
    interval: S.interval.type === 'time' ? Math.max(0.5, r1(iv)) : Math.max(1, r1(iv)),
    frontEff: 100 * (1 - shotSpacing / along), gsd: across * 100 / (S.photoSize === '12' ? 4032 : cam.px),
  };
}

/* ---------- mission generation ---------- */
function newWp(p, extra) {
  return Object.assign({ lat: p[0], lng: p[1], alt: null, speed: null, gimbal: null, heading: 'route', hdg: 0, photo: false, rec: '', hover: 0 }, extra || {});
}
let gridInfo = null;
function regen() {
  gridInfo = null;
  if (S.mode === 'grid' && S.shape && S.shape.pts.length >= 3) {
    const cov = coverage();
    const pr = projector(centroid(S.shape.pts));
    const P = S.shape.pts.map(pr.xy), holes = hzSurveyHoles(pr);
    let ang;
    if (S.dir === 'auto') {
      let best = Infinity;
      // The best line direction always runs parallel to one of the area's edges, so only those are tried.
      // (Odd in-between angles can look "shorter" only because they clip the edge lines and leave gaps.)
      const cands = [];
      P.forEach((q, k) => {
        const r = P[(k + 1) % P.length], dx = r[0] - q[0], dy = r[1] - q[1];
        if (Math.hypot(dx, dy) < 1) return;
        const b = Math.round((((Math.atan2(dx, dy) * 180 / Math.PI) % 180) + 180) % 180 * 10) / 10;
        if (!cands.some(c => Math.abs(c - b) < 0.5 || Math.abs(c - b) > 179.5)) cands.push(b % 180);
      });
      if (!cands.length) cands.push(0);
      cands.forEach(a => {
        const t = pathTime(sweep(P, a, cov.spacing, S.runIn, holes).pts, S.speed);
        if (t < best - 1e-6) { best = t; ang = a; }
      });
    } else ang = S.dir === 'custom' ? ((S.angle % 180) + 180) % 180 : DIRS[S.dir];
    let res = sweep(P, ang, cov.spacing, S.runIn, holes);
    let pts = res.pts;
    if (S.home && pts.length > 1) {                          // start at the corner nearest the pilot
      const h = pr.xy(S.home);
      const d = q => Math.hypot(q[0] - h[0], q[1] - h[1]);
      if (d(pts[pts.length - 1]) < d(pts[0])) pts = pts.slice().reverse();
    }
    if (S.reverse) pts = pts.slice().reverse();
    S.wps = pts.map((q, i) => newWp(pr.ll(q), { gimbal: i === 0 ? S.gridGimbal : null }));
    gridInfo = { angle: ang, lines: res.lines, area: polyArea(P), full: S.wps.map(w => [w.lat, w.lng]), parts: [], holes: holes.filter(H => H.some(q => hzInside(q, P)) || P.some(q => hzInside(q, H))).length };
    S.interval.from = 0; S.interval.to = -1;
    if (S.split && S.split.on && !multi()) {
      gridInfo.parts = computeParts(S.wps);
      if (S.split.sel >= gridInfo.parts.length) S.split.sel = -1;
      if (gridInfo.parts.length > 1 && S.split.sel >= 0) { const pt = gridInfo.parts[S.split.sel]; S.wps = S.wps.slice(pt.a, pt.b + 1); S.wps[0].gimbal = S.gridGimbal; }
    }
  } else if (S.mode === 'poi' && S.shape) {
    // One or more orbits. Each orbit can sit higher (or lower) and wider (or tighter) than the one before.
    // 'step': hold each orbit's height and climb on the first leg of the next orbit. 'spiral': climb a little at every point.
    const p = S.poi, c = S.shape.center, n = Math.max(3, Math.round(p.points)), rings = Math.max(1, Math.round(p.rings || 1));
    const total = rings * n + (p.loop ? 1 : 0);
    S.wps = [];
    for (let i = 0; i < total; i++) {
      const ring = Math.min(rings - 1, Math.floor(i / n));
      const pos = p.climb === 'spiral' && rings > 1 ? (rings - 1) * i / Math.max(1, total - 1) : ring;
      const alt = Math.max(2, S.alt + pos * (p.altStep || 0));
      const rad = Math.max(3, p.radius + pos * (p.radStep || 0));
      const b = p.start + (p.cw ? 1 : -1) * i * 360 / n;
      const g = p.gimbalAuto ? -Math.atan2(Math.max(0, alt - p.height), rad) * 180 / Math.PI : p.gimbal;
      S.wps.push(newWp(offset(c, b, rad), { heading: 'poi', gimbal: r0(g), alt: i === 0 && pos === 0 ? null : (Math.abs(alt - S.alt) < 0.01 ? null : alt) }));
    }
  }
  if (S.sel >= S.wps.length) S.sel = -1;
}

/* Split a survey into battery-sized parts. Waypoints come in pairs (one flight line each), so parts always break
   between lines. A part ends when adding the next line would pass the time limit or, with photo waypoints, the point limit. */
function usesPhotoPoints() { return !S.interval.method || S.interval.method === 'points'; }
const PART_MAX_POINTS = 195;   // DJI Fly's practical waypoint limit, with a little room
/* A survey's flight lines (pairs of waypoints) with the time and points each adds. */
function surveyLines(W) {
  const cov = coverage(), ext = Math.max(0, S.runIn || 0);
  const usePts = S.interval.on && usesPhotoPoints();
  const step = Math.max(1, S.interval.type === 'time' ? cov.interval * S.speed : cov.interval);
  const lines = [];
  for (let k = 0; 2 * k + 1 < W.length; k++) {
    const a = W[2 * k], b = W[2 * k + 1], nx = W[2 * k + 2];
    const L = hav([a.lat, a.lng], [b.lat, b.lng]);
    const hop = nx ? hav([b.lat, b.lng], [nx.lat, nx.lng]) : 0;
    lines.push({ k, t: (L + hop) / S.speed + 2 * STOP_PENALTY_S, p: 2 + (usePts ? Math.floor(Math.max(0, L - 2 * ext) / step) + 2 : S.pathStyle !== 'straightx' ? 2 : 0), d: L + hop });
  }
  return lines;
}
/* Packs chunks (each {t, p}) in order into parts no longer than maxT seconds and maxP points, then evens the parts out
   so the last one isn't a tiny leftover. Returns arrays of chunk indexes. */
function packChunks(chunks, maxT, maxP) {
  const greedy = (lt, lp) => {
    const parts = []; let cur = null;
    chunks.forEach((c, i) => {
      if (cur && (cur.t + c.t > lt || cur.p + c.p > lp)) { parts.push(cur); cur = null; }
      if (!cur) cur = { idx: [], t: 0, p: 0 };
      cur.idx.push(i); cur.t += c.t; cur.p += c.p;
    });
    if (cur) parts.push(cur);
    return parts;
  };
  const first = greedy(maxT, maxP), n = first.length;
  if (n < 2) return first;
  const T = chunks.reduce((a, c) => a + c.t, 0), P = chunks.reduce((a, c) => a + c.p, 0);
  for (const slack of [1.0, 1.05, 1.1, 1.2]) {
    const even = greedy(Math.min(maxT, T / n * slack), Math.min(maxP, P / n * slack));
    if (even.length === n) return even;
  }
  return first;
}
function partLimits() {
  const anyPts = multi() ? S.segs.some(sg => sg.interval && sg.interval.on && (!sg.interval.method || sg.interval.method === 'points')) : S.interval.on && usesPhotoPoints();
  return { maxT: Math.max(3, S.split.minutes) * 60, maxP: anyPts ? Math.min(PART_MAX_POINTS, Math.max(20, S.split.points)) : PART_MAX_POINTS };
}
/* Split a survey into battery-sized parts. Parts always break between flight lines. */
function computeParts(W) {
  const lines = surveyLines(W), lim = partLimits();
  return packChunks(lines, lim.maxT, lim.maxP).map(g => {
    const a = lines[g.idx[0]], b = lines[g.idx[g.idx.length - 1]];
    return { a: 2 * a.k, b: 2 * b.k + 1, t: g.t, p: g.p, d: g.idx.reduce((s2, i) => s2 + lines[i].d, 0) };
  });
}
/* Parts for a combined mission: surveys break between flight lines, orbits and waypoint runs stay whole, and each part
   stays under the time and point limits. A part is a list of slices {seg, a, b} (waypoint index ranges). */
let partsCache = null;
function missionParts() {
  if (!multi() || !S.split.on) return [];
  if (partsCache) return partsCache;
  segStore();
  const chunks = [];
  let prevEnd = null, prevSpeed = 0;
  S.segs.forEach((sg, i) => {
    if (!sg.wps || sg.wps.length < 1) return;
    const first = chunks.length;
    withSeg(sg, () => {
      if (S.mode === 'grid' && S.wps.length >= 2) surveyLines(S.wps).forEach(l => chunks.push({ seg: i, a: 2 * l.k, b: 2 * l.k + 1, t: l.t, p: l.p }));
      else { const st = stats(); chunks.push({ seg: i, a: 0, b: S.wps.length - 1, t: st.time, p: st.points }); }
    }, true);
    if (prevEnd && chunks[first]) { const d = hav(prevEnd, [sg.wps[0].lat, sg.wps[0].lng]); chunks[first].t += d / Math.max(0.5, prevSpeed) + STOP_PENALTY_S; }
    const lw = sg.wps[sg.wps.length - 1]; prevEnd = [lw.lat, lw.lng]; prevSpeed = lw.speed || sg.speed;
  });
  const lim = partLimits();
  partsCache = packChunks(chunks, lim.maxT, lim.maxP).map(g => {
    const slices = [];
    g.idx.forEach(ci => {
      const c = chunks[ci], last = slices[slices.length - 1];
      if (last && last.seg === c.seg) last.b = c.b; else slices.push({ seg: c.seg, a: c.a, b: c.b });
    });
    return { t: g.t, p: g.p, slices };
  });
  return partsCache;
}
function selectedPart() {
  if (!multi() || !S.split.on || S.split.sel < 0) return null;
  const parts = missionParts();
  return parts.length > 1 ? parts[S.split.sel] || null : null;
}
function partPoints(pt) {
  return pt.slices.reduce((out, sl) => {
    const sg = S.segs[sl.seg];
    if (sg.mode === 'grid' && (sl.a > 0 || sl.b < sg.wps.length - 1)) {
      const cut = clone(sg); cut.wps = cut.wps.slice(sl.a, sl.b + 1); cut.wps[0].gimbal = sg.gridGimbal;
      cut.interval = Object.assign({}, cut.interval, { from: 0, to: -1 });
      return out.concat(segPoints(cut, sl.seg));
    }
    return out.concat(segPoints(sg, sl.seg));
  }, []);
}
function partLabel(pt, i, n) {
  const segs = [...new Set(pt.slices.map(sl => sl.seg + 1))];
  return 'Part ' + (i + 1) + ' of ' + n + ' · ' + r0(pt.t / 60) + ' min · ' + pt.p + ' pts · segment' + (segs.length > 1 ? 's ' + segs[0] + '–' + segs[segs.length - 1] : ' ' + segs[0]);
}

/* resolved per-waypoint values */
const wpAlt = w => w.alt == null ? S.alt : w.alt;
const wpSpeed = w => w.speed == null ? S.speed : w.speed;
function wpGimbals() {
  let g = S.mode === 'grid' ? S.gridGimbal : S.defGimbal;
  return S.wps.map(w => { if (w.gimbal != null) g = w.gimbal; return g; });
}
function intervalRange() {
  const n = S.wps.length; if (!S.interval.on || n < 2) return null;
  const a = Math.min(Math.max(0, S.interval.from), n - 2);
  const b = S.interval.to < 0 ? n - 1 : Math.min(Math.max(a + 1, S.interval.to), n - 1);
  return [a, b];
}

/* ---------- combined missions (segments) ----------
   A mission can hold several segments flown in order: surveys, POI orbits and runs of waypoints, each with its own
   settings. The segment being edited lives in S like a normal single mission; the others are stored in S.segs. When the
   mission is built, every segment is turned into its own flight points (under its own settings) and they are joined.
   Orbits get an explicit heading toward their centre at every point, since a combined file can only carry one POI. */
const SEG_KEYS = ['mode', 'shape', 'poiCenter', 'wps', 'alt', 'speed', 'turn', 'defGimbal', 'camera', 'photoSize', 'front', 'side',
  'spacingAuto', 'spacing', 'dir', 'angle', 'reverse', 'gridGimbal', 'runIn', 'pathStyle', 'interval', 'poi', 'label'];
const SEG_COLORS = ['#ff7a2e', '#4dabf7', '#b197fc', '#69db7c', '#ffd43b', '#f783ac', '#63e6be', '#ffa8a8'];
const multi = () => !!(S.segs && S.segs.length);
function segSnap() { const o = {}; SEG_KEYS.forEach(k => { o[k] = clone(S[k] === undefined ? null : S[k]); }); return o; }
function segStore() { if (multi()) S.segs[S.seg] = segSnap(); }
function segLoad(sg) { SEG_KEYS.forEach(k => { if (sg[k] !== undefined) S[k] = clone(sg[k]); }); S.sel = -1; if (S.mode !== 'waypoint') regen(); }
function segLabel(sg, i) {
  const base = sg.label || (sg.mode === 'grid' && sg.shape ? 'Survey' : sg.mode === 'poi' && sg.shape ? 'POI orbit' : sg.wps && sg.wps.length ? 'Waypoints' : 'Empty');
  return (i + 1) + '. ' + base;
}
/* Runs fn with S switched to one stored segment (mission-wide settings kept). */
function withSeg(sg, fn, noTerrain) {
  const saved = S, gi = gridInfo, mc = mpCache;
  S = Object.assign({}, saved, clone(sg), { segs: [], split: Object.assign({}, saved.split, { on: false }) });
  if (noTerrain) S.terrain = Object.assign({}, saved.terrain, { on: false });
  mpCache = null;
  try { return fn(); } finally { S = saved; gridInfo = gi; mpCache = mc; }
}
/* One segment's flight points with every value resolved, so they no longer depend on the segment's settings. */
function segPoints(sg, idx) {
  return withSeg(sg, () => flightPoints().map(p => {
    const w = Object.assign({}, p.w, { alt: wpAlt(p.w), speed: wpSpeed(p.w) });
    if (w.heading === 'poi' && S.poiCenter) { w.heading = 'fixed'; w.hdg = r0(bearing(w, { lat: S.poiCenter[0], lng: S.poiCenter[1] })); }
    return Object.assign({}, p, { w, seg: idx, stop: !p.extra && (S.mode === 'grid' || S.turn === 'stop'), straight: S.mode === 'grid' && S.pathStyle === 'straightx' });
  }));
}
function allFlightPoints() {
  if (!multi()) return flightPoints();
  segStore();
  const pt = selectedPart();
  if (pt) return partPoints(pt);
  return S.segs.reduce((a, sg, i) => a.concat(segPoints(sg, i)), []);
}
function segActivate(i) {
  if (!multi() || i < 0 || i >= S.segs.length) return;
  segStore(); S.seg = i; segLoad(S.segs[i]); commit(false); fitMission();
}
function segAdd() {
  if (!multi()) {
    if (!S.wps.length && !S.shape) { toast('Draw the first part of the mission, then add the next segment.'); return; }
    S.split.sel = -1; S.segs = [segSnap()]; S.seg = 0; if (S.mode === 'grid') regen();
  }
  segStore();
  const blank = segSnap();
  Object.assign(blank, { mode: 'waypoint', shape: null, poiCenter: null, wps: [], label: null, reverse: false });
  blank.interval = Object.assign({}, blank.interval, { on: false, auto: false, from: 0, to: -1 });
  S.segs.push(blank); S.seg = S.segs.length - 1; segLoad(blank);
  commit(false);
  toast('Segment ' + S.segs.length + ' added. Draw it with Rectangle, Polygon, POI orbit or Waypoint.');
}
function segMove(i, d) {
  const j = i + d; if (!multi() || j < 0 || j >= S.segs.length) return;
  segStore();
  const a = S.segs; [a[i], a[j]] = [a[j], a[i]];
  if (S.seg === i) S.seg = j; else if (S.seg === j) S.seg = i;
  commit(false);
}
function segDelete(i) {
  if (!multi()) return;
  segStore(); S.segs.splice(i, 1);
  S.split.sel = -1;
  if (S.segs.length <= 1) { const last = S.segs[0]; S.segs = []; S.seg = 0; if (last) segLoad(last); else { S.wps = []; S.shape = null; S.mode = 'waypoint'; } toast('Back to a single-part mission'); }
  else { S.seg = Math.min(S.seg > i ? S.seg - 1 : S.seg, S.segs.length - 1); segLoad(S.segs[S.seg]); }
  commit(false);
}
/* Per-segment figures (no terrain points; terrain is handled for the whole mission). */
function segStats() {
  segStore();
  return S.segs.map((sg, i) => withSeg(sg, () => { const s = stats(); return { dist: s.dist, time: s.time, photos: s.photos, points: s.points, warns: s.warns, mode: S.mode, cam: S.interval.on && S.interval.method === 'camera' }; }, true));
}
/* Saves each part as its own mission in Missions ("Name (2 of 5)"), opening on that part. */
function savePartsToLibrary() {
  const n = multi() ? missionParts().length : (gridInfo && gridInfo.parts ? gridInfo.parts.length : 0);
  if (n < 2) { toast('It already fits in one part.'); return; }
  segStore();
  const list = libLoad(), base = snapshot(), now = Date.now();
  for (let k = 0; k < n; k++) {
    const d = clone(base);
    d.name = S.name + ' (' + (k + 1) + ' of ' + n + ')';
    d.split = Object.assign({}, d.split, { on: true, sel: k });
    list.push({ id: 'm' + (now + k), saved: now + k, data: d });
  }
  libSave(list);
  toast('Saved ' + n + ' missions in Missions: “' + S.name + ' (1 of ' + n + ')” and so on. Each opens on its own part. Fly them in order.');
}

function stats() {
  if (multi()) return statsMulti();
  const W = S.wps, out = { dist: 0, time: 0, photos: 0, warns: [] };
  const stops = S.turn === 'stop' || S.mode === 'grid';
  const legT = [];
  for (let i = 1; i < W.length; i++) {
    const d = Math.hypot(hav([W[i - 1].lat, W[i - 1].lng], [W[i].lat, W[i].lng]), wpAlt(W[i]) - wpAlt(W[i - 1]));
    const t = d / wpSpeed(W[i - 1]);
    out.dist += d; legT.push({ d, t }); out.time += t;
  }
  W.forEach(w => { out.time += (w.hover || 0); if (w.photo) out.photos++; });
  if (stops) out.time += W.length * STOP_PENALTY_S;
  const rg = intervalRange();
  if (rg) {
    const cov = coverage();
    let d = 0, t = 0; for (let i = rg[0]; i < rg[1]; i++) { d += legT[i].d; t += legT[i].t; }
    if (usesPhotoPoints()) {
      const fp = missionPoints(); out.points = fp.length;
      out.photos = fp.filter(p => p.photo || p.w.photo).length;
    } else out.photos += Math.floor((S.interval.type === 'time' ? t : d) / cov.interval) + 1;
    out.interval = cov.interval;
    if (S.interval.type === 'time' && cov.interval < 2) out.warns.push(['bad', 'Photo interval under 2 s. The Air 3S may skip shots — slow down or lower overlap.']);
  }
  const maxAlt = S.terrain.on ? S.alt : Math.max(S.alt, ...W.map(wpAlt));
  if (maxAlt > MAX_ALT + 0.5) out.warns.push(['bad', 'Altitude above 400 ft (122 m). Check your airspace limits.']);
  if (S.mode === 'grid' && S.alt < 20) out.warns.push(['', 'Low survey altitude. Watch for trees, towers and power lines.']);
  out.points = out.points || missionPoints().length;
  if (S.terrain.on) {
    const mp = missionPoints();
    if (mp.terrainState === 'notakeoff') out.warns.push(['bad', 'Terrain following is on. Set the takeoff spot.']);
    else if (mp.terrainState === 'loading') out.warns.push(['', 'Loading terrain data…']);
    else if (mp.terrainState === 'missing') out.warns.push(['bad', 'Terrain data unavailable here. Connect to the internet, or download this area in Offline maps with Terrain ticked.']);
    else if (mp.terrainInfo) {
      out.terrain = mp.terrainInfo;
      if (mp.terrainInfo.maxH > 120) out.warns.push(['', 'Highest waypoint is ' + fmtLen(mp.terrainInfo.maxH) + ' above takeoff. Set DJI Fly’s Max Altitude at least that high, or the mission may not start.']);
      if (mp.terrainInfo.clamped) out.warns.push(['bad', 'Ground drops more than your flying height below takeoff in places. Those points were held at takeoff level.']);
    }
  }
  if (S.mode === 'grid' && S.split && S.split.on && S.split.sel < 0 && gridInfo && gridInfo.parts.length > 1) out.warns.push(['bad', 'Showing all parts. Pick one part under Split into parts before sending.']);
  hzWarns(missionPoints(), out.warns); airWarns(out.warns);
  if (out.points > 200) out.warns.push(['bad', out.points + ' waypoints' + (out.points > W.length ? ' including photo points' : '') + '. DJI Fly may reject missions over ~200 points — raise the interval, fly higher or split the area.']);
  if (out.time > 30 * 60) out.warns.push(['', 'Longer than ~30 min. Plan a battery swap or split the area.']);
  if (Math.max(S.speed, ...W.map(wpSpeed)) > 15) out.warns.push(['', 'Speeds above 15 m/s (34 mph) may be capped by DJI Fly.']);
  if (S.mode !== 'grid' && W.length && W.length < 2) out.warns.push(['', 'Add at least two waypoints.']);
  return out;
}

function statsMulti() {
  const per = segStats(), out = { dist: 0, time: 0, photos: 0, warns: [], per };
  per.forEach(p => { out.dist += p.dist; out.time += p.time; out.photos += p.photos; });
  const parts = S.split.on ? missionParts() : [], pt = selectedPart();
  let prev = null;
  S.segs.forEach(sg => {
    if (!sg.wps || !sg.wps.length) return;
    if (prev) { const b = prev.wps[prev.wps.length - 1], a = sg.wps[0], d = hav([b.lat, b.lng], [a.lat, a.lng]); out.dist += d; out.time += d / (b.speed || prev.speed) + STOP_PENALTY_S; }
    prev = sg;
  });
  const mp = missionPoints(); out.points = mp.length;
  if (pt) {
    out.time = pt.t; out.dist = 0;
    for (let i = 1; i < mp.length; i++) out.dist += hav([mp[i - 1].w.lat, mp[i - 1].w.lng], [mp[i].w.lat, mp[i].w.lng]);
    const flagged = mp.filter(p => p.photo || p.w.photo).length, cams = S.segs.filter(sg => sg.interval && sg.interval.on && sg.interval.method === 'camera');
    out.photos = flagged + (cams.length ? Math.floor(pt.t / Math.max(0.5, cams[0].interval.value || 3)) : 0);
  }
  if (parts.length > 1 && S.split.sel < 0) out.warns.push(['bad', 'Showing all ' + parts.length + ' parts. Pick one part under Split into parts before sending.']);
  const seen = {};
  per.forEach((p, i) => {
    if (p.points < 2) out.warns.push(['', 'Segment ' + (i + 1) + ' has nothing drawn yet. Draw it or delete it.']);
    p.warns.forEach(([c, t]) => { if (/waypoints|Longer than|Add at least|Showing all parts|Terrain|terrain|hazard|no-fly|too low|Hazard/.test(t) || seen[t]) return; seen[t] = 1; out.warns.push([c, 'Segment ' + (i + 1) + ': ' + t]); });
  });
  if (per.some(p => p.cam)) out.warns.push(['', 'A survey here uses Timed Shot. In a combined mission it keeps shooting during the other segments and connecting legs too.']);
  if (S.terrain.on) {
    if (mp.terrainState === 'notakeoff') out.warns.push(['bad', 'Terrain following is on. Set the takeoff spot.']);
    else if (mp.terrainState === 'loading') out.warns.push(['', 'Loading terrain data…']);
    else if (mp.terrainState === 'missing') out.warns.push(['bad', 'Terrain data unavailable here. Connect to the internet, or download this area in Offline maps with Terrain ticked.']);
    else if (mp.terrainInfo) {
      out.terrain = mp.terrainInfo;
      if (mp.terrainInfo.maxH > 120) out.warns.push(['', 'Highest waypoint is ' + fmtLen(mp.terrainInfo.maxH) + ' above takeoff. Set DJI Fly’s Max Altitude at least that high, or the mission may not start.']);
      if (mp.terrainInfo.clamped) out.warns.push(['bad', 'Ground drops more than your flying height below takeoff in places. Those points were held at takeoff level.']);
    }
  }
  hzWarns(mp, out.warns); airWarns(out.warns);
  if (out.points > 200 && !(parts.length > 1 && S.split.sel < 0)) out.warns.push(['bad', out.points + ' points' + (pt ? ' in this part' : ' in total') + '. DJI Fly may reject missions over ~200 points. Turn on Split into parts' + (pt ? ' with a lower time limit' : '') + ', or trim a segment.']);
  if (out.time > 30 * 60 && !(parts.length > 1 && S.split.sel < 0)) out.warns.push(['', 'Longer than ~30 min. Turn on Split into parts or plan a battery swap.']);
  return out;
}

/* ---------- KMZ (DJI WPML) ---------- */
const NS = 'http://www.dji.com/wpmz/1.0.2';
const esc = s => String(s).replace(/[<>&'"]/g, ch => ({ '<': '&lt;', '>': '&gt;', '&': '&amp;', "'": '&apos;', '"': '&quot;' }[ch]));
const n6 = v => Number(v).toFixed(6).replace(/\.?0+$/, '') || '0';
const n9 = v => Number(v).toFixed(9);
function el(tag, inner) { return `<wpml:${tag}>${inner}</wpml:${tag}>`; }

/* Interval photos. 'points' (default) inserts pass-through photo waypoints along the route, which uses only
   actions DJI Fly itself writes. 'camera' leaves photos to DJI Fly's Timed Shot, so the file only holds the route. */
/* ---------- terrain following ----------
   Elevation comes from the free Terrarium tiles on AWS Open Data (USGS 3DEP in the US, about 10 m detail). Tiles go
   through the app's tile cache, so they work offline once seen or downloaded. Each flight leg is sampled every 10 m and
   extra pass-through points are added wherever a straight climb or descent would drift more than the tolerance from the
   wanted height above ground. Heights are written relative to the takeoff spot, which is what DJI Fly uses. */
const DEM_Z = 14;
const DEM_URL = (z, x, y) => (window.__demUrl ? window.__demUrl(z, x, y) : `https://s3.amazonaws.com/elevation-tiles-prod/terrarium/${z}/${x}/${y}.png`);
const dem = new Map();
let mpCache = null, demTimer = null;
function demPixel(lat, lng) {
  const n = 2 ** DEM_Z * 256, r = lat * Math.PI / 180;
  return [(lng + 180) / 360 * n, (1 - Math.log(Math.tan(r) + 1 / Math.cos(r)) / Math.PI) / 2 * n];
}
function demGet(px, py) {
  const tx = Math.floor(px / 256), ty = Math.floor(py / 256), t = dem.get(tx + '/' + ty);
  if (!t || !t.data) return null;
  return t.data[(py - ty * 256) * 256 + (px - tx * 256)];
}
function elevAt(lat, lng) {
  let [x, y] = demPixel(lat, lng); x -= 0.5; y -= 0.5;
  const x0 = Math.floor(x), y0 = Math.floor(y), fx = x - x0, fy = y - y0;
  const a = demGet(x0, y0), b = demGet(x0 + 1, y0), c = demGet(x0, y0 + 1), d = demGet(x0 + 1, y0 + 1);
  if (a == null || b == null || c == null || d == null) return null;
  return a * (1 - fx) * (1 - fy) + b * fx * (1 - fy) + c * (1 - fx) * fy + d * fx * fy;
}
function demLoad(k) {
  const [x, y] = k.split('/').map(Number), t = { state: 'loading' };
  dem.set(k, t);
  const img = new Image(); img.crossOrigin = 'anonymous';
  img.onload = () => {
    try {
      const c = document.createElement('canvas'); c.width = c.height = 256;
      const g = c.getContext('2d', { willReadFrequently: true }); g.drawImage(img, 0, 0);
      const px = g.getImageData(0, 0, 256, 256).data, a = new Float32Array(65536);
      for (let i = 0; i < 65536; i++) a[i] = px[i * 4] * 256 + px[i * 4 + 1] + px[i * 4 + 2] / 256 - 32768;
      t.data = a; t.state = 'ok';
    } catch (e) { t.state = 'error'; t.time = Date.now(); }
    demChanged();
  };
  img.onerror = () => { t.state = 'error'; t.time = Date.now(); demChanged(); };
  img.src = DEM_URL(DEM_Z, x, y);
}
function demChanged() { clearTimeout(demTimer); demTimer = setTimeout(() => { commit(false); if (typeof meas !== 'undefined' && meas.active) renderMeasure(); }, 150); }
/* Make sure every tile under a lat/lng box (plus a pixel of margin) is loaded: 'ok', 'loading' or 'missing'. */
function demEnsure(s, w, n, e) {
  const [x1, y1] = demPixel(n, w), [x2, y2] = demPixel(s, e);
  let st = 'ok';
  for (let tx = Math.floor((x1 - 2) / 256); tx <= Math.floor((x2 + 2) / 256); tx++) {
    for (let ty = Math.floor((y1 - 2) / 256); ty <= Math.floor((y2 + 2) / 256); ty++) {
      const k = tx + '/' + ty, t = dem.get(k);
      if (!t || (t.state === 'error' && Date.now() - t.time > 20000)) { demLoad(k); if (st === 'ok') st = 'loading'; }
      else if (t.state === 'loading') { if (st === 'ok') st = 'loading'; }
      else if (t.state === 'error') st = 'missing';
    }
  }
  return st;
}
/* Douglas–Peucker on a height profile: keep the samples needed so straight segments stay within tol. */
function keepProfile(d, A, tol) {
  const keep = new Set([0, d.length - 1]);
  const rec = (i, j) => {
    let worst = -1, wi = -1;
    for (let k = i + 1; k < j; k++) {
      const lin = A[i] + (A[j] - A[i]) * (d[k] - d[i]) / (d[j] - d[i]);
      const dev = Math.abs(A[k] - lin);
      if (dev > worst) { worst = dev; wi = k; }
    }
    if (worst > tol) { keep.add(wi); rec(i, wi); rec(wi, j); }
  };
  rec(0, d.length - 1);
  return Array.from(keep).sort((a, b) => a - b);
}
function missionPoints() {
  if (mpCache) return mpCache;
  const P = allFlightPoints();
  if (!S.terrain || !S.terrain.on || !P.length) return (mpCache = hzApply(P));
  const tk = S.terrain.takeoff;
  if (!tk) { P.terrainState = 'notakeoff'; return (mpCache = hzApply(P)); }
  const lats = P.map(p => p.w.lat).concat(tk[0]), lngs = P.map(p => p.w.lng).concat(tk[1]);
  const st = demEnsure(Math.min(...lats), Math.min(...lngs), Math.max(...lats), Math.max(...lngs));
  if (st !== 'ok') { P.terrainState = st; return (mpCache = hzApply(P)); }
  const e0 = elevAt(tk[0], tk[1]), tol = Math.max(1, S.terrain.tol || 4.5);
  const out = [], info = { e0, gMin: Infinity, gMax: -Infinity, minH: Infinity, maxH: -Infinity, added: 0, clamped: false };
  const height = (g, agl) => {
    let h = g + agl - e0;
    if (h < 2) { h = 2; info.clamped = true; }
    info.gMin = Math.min(info.gMin, g); info.gMax = Math.max(info.gMax, g);
    info.minH = Math.min(info.minH, h); info.maxH = Math.max(info.maxH, h);
    return h;
  };
  for (let i = 0; i < P.length; i++) {
    const p = P[i], g = elevAt(p.w.lat, p.w.lng);
    out.push(Object.assign({}, p, { h: height(g, wpAlt(p.w)) }));
    if (i === P.length - 1) break;
    const q = P[i + 1], a = p.w, b = q.w, Lg = hav([a.lat, a.lng], [b.lat, b.lng]);
    if (Lg < 20) continue;
    const n = Math.ceil(Lg / 10), d = [], A = [], G = [];
    for (let k = 0; k <= n; k++) {
      const f = k / n, ge = elevAt(a.lat + (b.lat - a.lat) * f, a.lng + (b.lng - a.lng) * f);
      d.push(Lg * f); G.push(ge); A.push(ge + wpAlt(a) + (wpAlt(b) - wpAlt(a)) * f);
    }
    keepProfile(d, A, tol).forEach(k => {
      if (k === 0 || k === n) return;
      const f = k / n;
      out.push({
        w: newWp([a.lat + (b.lat - a.lat) * f, a.lng + (b.lng - a.lng) * f], { speed: wpSpeed(a), heading: a.heading === 'poi' ? 'poi' : (p.seg != null && a.heading === 'fixed') ? 'fixed' : 'route', hdg: a.hdg }),
        gimbal: p.gimbal, orig: -1, extra: true, photo: false, h: height(G[k], A[k] - G[k]), seg: p.seg, stop: false, straight: p.straight,
      });
      info.added++;
    });
  }
  out.terrainState = 'ok'; out.terrainInfo = info;
  return (mpCache = hzApply(out));
}

/* In a survey, waypoints come in pairs: even legs (0→1, 2→3 …) are the flight lines, odd legs are the turn hops between
   lines. Photos go only on flight lines, and only between the area edges (the run-in at each end is skipped). */
function flightPoints() {
  const W = S.wps, gims = wpGimbals(), rg = intervalRange();
  const out = [];
  const usePoints = rg && usesPhotoPoints();
  const grid = S.mode === 'grid';
  const ext = grid ? Math.max(0, S.runIn || 0) : 0;
  const photoPt = (a, b, leg, d, i) => {
    const f = d / leg;
    return {
      w: newWp([a.lat + (b.lat - a.lat) * f, a.lng + (b.lng - a.lng) * f],
        { alt: wpAlt(a) + (wpAlt(b) - wpAlt(a)) * f, speed: wpSpeed(a), heading: a.heading === 'poi' ? 'poi' : 'route' }),
      gimbal: gims[i], orig: -1, extra: true, photo: true,
    };
  };
  let carry = 0;                                                 // meters flown since the last photo
  for (let i = 0; i < W.length; i++) {
    const endPhoto = usePoints && !grid && (i === rg[0] || i === rg[1]);
    out.push({ w: W[i], gimbal: gims[i], orig: i, extra: false, photo: endPhoto });
    if (grid && S.pathStyle !== 'straightx' && !usePoints && i % 2 === 0 && i < W.length - 1) {
      // Fallback for curved paths: two pass-through points near each end of a flight line keep DJI Fly's
      // curve from swinging wide between lines.
      const a = W[i], b = W[i + 1], leg = hav([a.lat, a.lng], [b.lat, b.lng]);
      const A = Math.min(Math.max(ext, 15), leg / 3);
      [A, leg - A].forEach(d => { const q = photoPt(a, b, leg, d, i); q.photo = false; out.push(q); });
      continue;
    }
    if (!usePoints || i < rg[0] || i >= rg[1]) continue;
    const a = W[i], b = W[i + 1];
    const leg = hav([a.lat, a.lng], [b.lat, b.lng]);
    const step = Math.max(1, S.interval.type === 'time' ? coverage().interval * wpSpeed(a) : coverage().interval);
    if (grid) {
      if (i % 2) continue;                                       // turn hop: no photos
      const d0 = Math.min(ext, leg / 2), d1 = Math.max(leg - ext, leg / 2);
      let d = d0;
      for (; d <= d1 + 0.01; d += step) out.push(photoPt(a, b, leg, Math.max(0.5, Math.min(leg - 0.5, d)), i));
      if (d1 - (d - step) > step * 0.3) out.push(photoPt(a, b, leg, Math.min(leg - 0.5, d1), i));   // photo at the far edge
      continue;
    }
    if (i === rg[0]) carry = 0;
    let d = step - carry;
    while (d < leg - 0.5) { out.push(photoPt(a, b, leg, d, i)); d += step; }
    carry = leg - (d - step);
  }
  return out;
}
function bearing(a, b) {
  const t = Math.PI / 180, y = Math.sin((b.lng - a.lng) * t) * Math.cos(b.lat * t);
  const x = Math.cos(a.lat * t) * Math.sin(b.lat * t) - Math.sin(a.lat * t) * Math.cos(b.lat * t) * Math.cos((b.lng - a.lng) * t);
  return Math.atan2(y, x) / t;
}

/* Mirrors the structure DJI Fly itself writes (wpmz 1.0.2): template.kml carries only missionConfig, the route lives in
   waylines.wpml, headings are smoothTransition, turns use continuity curvature, gimbal uses gimbalEvenlyRotate. */
function buildKmz() {
  if (!multi() && S.wps.length < 2) throw new Error('Add at least two waypoints first.');
  const P = missionPoints(), now = Date.now(), last = P.length - 1, rg = intervalRange();
  if (P.length < 2) throw new Error('Add at least two waypoints first.');
  if (S.terrain.on && P.terrainState !== 'ok') throw new Error(P.terrainState === 'notakeoff' ? 'Set the takeoff spot for terrain following first.' : 'Terrain data is not loaded yet. Connect to the internet or download this area in Offline maps.');
  const drone = el('droneInfo', el('droneEnumValue', S.droneEnum == null ? 68 : S.droneEnum) + el('droneSubEnumValue', S.droneSub == null ? 0 : S.droneSub));
  const missionConfig = el('missionConfig',
    el('flyToWaylineMode', 'safely') + el('finishAction', S.finish) +
    el('exitOnRCLost', 'executeLostAction') + el('executeRCLostAction', S.rcLost === 'hover' ? 'hover' : 'goBack') +
    el('globalTransitionalSpeed', n6(S.speed)) + drone);
  const poiStr = S.poiCenter ? `${n9(S.poiCenter[0])},${n9(S.poiCenter[1])},${n6(S.poi.height)}` : '0.000000,0.000000,0.000000';
  const head = `<?xml version="1.0" encoding="UTF-8"?>\n<kml xmlns="http://www.opengis.net/kml/2.2" xmlns:wpml="${NS}">\n<Document>\n`;
  let gid = 1, aid = 1;
  const action = (func, params) => el('action', el('actionId', aid++) + el('actionActuatorFunc', func) + el('actionActuatorFuncParam', params));
  const group = (start, end, mode, trigger, actions) => el('actionGroup', el('actionGroupId', gid++) + el('actionGroupStartIndex', start) +
    el('actionGroupEndIndex', end) + el('actionGroupMode', mode) + el('actionTrigger', trigger) + actions);
  const reach = el('actionTriggerType', 'reachPoint');
  const pay = el('payloadPositionIndex', 0);

  let wl = head + missionConfig + '\n<Folder>\n' + el('templateId', 0) + el('executeHeightMode', 'relativeToStartPoint') +
    el('waylineId', 0) + el('distance', 0) + el('duration', 0) + el('autoFlightSpeed', n6(S.speed)) + '\n';
  P.forEach((p, i) => {
    const w = p.w;
    // heading
    let mode = 'smoothTransition', ang, en = 0, poi = '0.000000,0.000000,0.000000';
    if (w.heading === 'fixed') { ang = ((w.hdg + 540) % 360) - 180; en = 1; }
    else if (w.heading === 'poi' && S.poiCenter) { mode = 'towardPOI'; ang = 0; poi = poiStr; }
    else { ang = i < last ? bearing(w, P[i + 1].w) : bearing(P[i - 1].w, w); en = i === 0 ? 1 : 0; }
    // turn
    const stop = i === 0 || (p.stop != null ? p.stop : (!p.extra && (S.mode === 'grid' || S.turn === 'stop')));
    // useStraightLine=1 makes DJI Fly fly a straight segment instead of its default curve.
    // DJI Fly does not draw or fly this flag reliably, so it is only used when chosen as an experiment.
    const straight = p.straight != null ? p.straight : S.mode === 'grid' && S.pathStyle === 'straightx';
    let x = '<Placemark>' + `<Point><coordinates>${n9(w.lng)},${n9(w.lat)}</coordinates></Point>` +
      el('index', i) + el('executeHeight', n6(p.h != null ? p.h : wpAlt(w))) + el('waypointSpeed', n6(wpSpeed(w))) +
      el('waypointHeadingParam', el('waypointHeadingMode', mode) + el('waypointHeadingAngle', Math.round(ang)) +
        el('waypointPoiPoint', poi) + el('waypointHeadingAngleEnable', en) + el('waypointHeadingPathMode', 'followBadArc')) +
      el('waypointTurnParam', el('waypointTurnMode', stop ? 'toPointAndStopWithContinuityCurvature' : 'toPointAndPassWithContinuityCurvature') +
        el('waypointTurnDampingDist', 0)) + el('useStraightLine', straight ? 1 : 0);
    if (i === 0) x += group(0, 0, 'parallel', reach, action('gimbalRotate',
      el('gimbalHeadingYawBase', 'aircraft') + el('gimbalRotateMode', 'absoluteAngle') + el('gimbalPitchRotateEnable', 1) +
      el('gimbalPitchRotateAngle', Math.round(p.gimbal)) + el('gimbalRollRotateEnable', 0) + el('gimbalRollRotateAngle', 0) +
      el('gimbalYawRotateEnable', 0) + el('gimbalYawRotateAngle', 0) + el('gimbalRotateTimeEnable', 0) + el('gimbalRotateTime', 0) + pay));
    const acts = [];
    if (w.hover > 0) acts.push(action('hover', el('hoverTime', n6(w.hover))));
    if (w.photo || p.photo) acts.push(action('takePhoto', pay));
    if (w.rec === 'start') acts.push(action('startRecord', pay));
    if (w.rec === 'stop') acts.push(action('stopRecord', pay));
    if (acts.length) x += group(i, i, 'sequence', reach, acts.join(''));
    const nx = Math.min(i + 1, last);
    x += group(i, nx, 'parallel', reach, action('gimbalEvenlyRotate', el('gimbalPitchRotateAngle', Math.round(P[nx].gimbal)) + pay));
    wl += x + '</Placemark>\n';
  });
  wl += '</Folder>\n</Document>\n</kml>\n';
  const tpl = head + el('author', 'fly') + el('createTime', now) + el('updateTime', now) + '\n' + missionConfig + '\n</Document>\n</kml>\n';
  return { template: tpl, waylines: wl, points: P.length };
}
window.__buildKmz = buildKmz;   // for testing

/* ---------- map ---------- */
const map = L.map('map', { zoomControl: false, attributionControl: true, maxZoom: 21 }).setView(S.view.c, S.view.z);
const ESRI = 'https://server.arcgisonline.com/ArcGIS/rest/services/';
const USGS = 'https://basemap.nationalmap.gov/arcgis/rest/services/';
/* Map sources. `off` = tile services saved for offline use; `max` = deepest zoom the service has real tiles for. */
const BASES = {
  sat: { label: 'Esri satellite', online: true, max: 19, bytes: 26000, off: ['World_Imagery', 'Reference/World_Boundaries_and_Places'],
    layer: L.layerGroup([
      L.tileLayer(ESRI + 'World_Imagery/MapServer/tile/{z}/{y}/{x}', { maxNativeZoom: 19, maxZoom: 21, attribution: 'Imagery © Esri, Maxar, Earthstar Geographics' }),
      L.tileLayer(ESRI + 'Reference/World_Boundaries_and_Places/MapServer/tile/{z}/{y}/{x}', { maxNativeZoom: 19, maxZoom: 21 }),
    ]) },
  usgs: { label: 'USGS aerial', max: 16, bytes: 25000, off: ['usgs:USGSImageryOnly'], layer: null },
  street: { label: 'Street map', online: true, max: 19, bytes: 12000, off: ['World_Street_Map'],
    layer: L.tileLayer(ESRI + 'World_Street_Map/MapServer/tile/{z}/{y}/{x}', { maxNativeZoom: 19, maxZoom: 21, attribution: 'Map © Esri' }) },
};
function usgsLayer(maxN) {
  return L.tileLayer(USGS + 'USGSImageryOnly/MapServer/tile/{z}/{y}/{x}', { maxNativeZoom: maxN, maxZoom: 21, attribution: 'Imagery: USGS The National Map (public domain)' });
}
try { const z = Number(localStorage.getItem('planner.usgsMax')); if (z >= 16 && z <= 20) BASES.usgs.max = z; } catch (e) {}
BASES.usgs.layer = usgsLayer(BASES.usgs.max);
let usgsProbed = false;
function probeUsgs() {
  if (usgsProbed || !native || !native.probeUsgsZoom) return;
  usgsProbed = true; const c = map.getCenter(); native.probeUsgsZoom(c.lat, c.lng);
}
function setUsgsMax(z) {
  const was = BASES.usgs.max;
  if (z <= was) { toast(z <= 16 ? 'USGS imagery stops at zoom 16 here. Use Esri satellite for close-up detail.' : 'USGS imagery goes to zoom ' + z + ' here.'); return; }
  BASES.usgs.max = z;
  try { localStorage.setItem('planner.usgsMax', z); } catch (e) {}
  const on = baseLayer === 'usgs';
  if (on) map.removeLayer(BASES.usgs.layer);
  BASES.usgs.layer = usgsLayer(z);
  if (on) BASES.usgs.layer.addTo(map);
  toast('USGS imagery now loads down to zoom ' + z + '.');
}
let baseLayer = 'sat', layerMenu = false;
try { const b = localStorage.getItem('planner.base'); if (BASES[b]) baseLayer = b; } catch (e) {}
BASES[baseLayer].layer.addTo(map);
function setBase(k) {
  if (!BASES[k] || k === baseLayer) return;
  map.removeLayer(BASES[baseLayer].layer); baseLayer = k; BASES[k].layer.addTo(map); BASES[k].layer.bringToBack && BASES[k].layer.bringToBack();
  try { localStorage.setItem('planner.base', k); } catch (e) {}
}
L.control.zoom({ position: 'bottomleft' }).addTo(map);
const photoCanvas = L.canvas({ padding: 0.2 });
const shapeLayer = L.layerGroup().addTo(map), routeLayer = L.layerGroup().addTo(map), tempLayer = L.layerGroup().addTo(map);
let homeMarker = null;
map.on('moveend', () => { const c = map.getCenter(); S.view = { c: [c.lat, c.lng], z: map.getZoom() }; save(); });

let draw = null;   // { tool, pts }
const HINTS = {
  waypoint: 'Tap the map to add waypoints. Drag a waypoint to move it.',
  rect: 'Tap one corner, then the opposite corner.',
  polygon: 'Tap each corner of the area, then press Finish.',
  poi: 'Tap the point of interest to orbit.',
  takeoff: 'Tap the spot you will take off from.',
  marker: 'Tap the map to place a marker.',
  measure: 'Tap points to measure. Tap Done when finished.',
  barea: 'Tap each corner of the area, then press Finish.',
  bline: 'Tap points along the line, then press Finish.',
  shapeedit: 'Drag the corners to reshape it. Tap Done when finished.',
};
/* While a tool is active (adding waypoints, measuring, drawing), existing features ignore taps so every tap lands on the
   map. Mission waypoints stay movable only with the Waypoint tool, whose hint says they can be dragged. */
function featuresLocked() { return !!draw; }
function setTool(t) {
  if (draw && draw.tool === 'measure' && t !== 'measure') stopMeasure();
  if (draw && draw.tool === 'shapeedit' && t !== 'shapeedit') shEditId = null;
  draw = t ? { tool: t, pts: [] } : null;
  renderTools(); renderTemp(); renderMap(); if (typeof renderMarkers === 'function') renderMarkers(); if (typeof renderShapes === 'function') renderShapes();
  const h = document.getElementById('drawhint');
  h.hidden = !t; if (t) h.textContent = HINTS[t];
}
map.on('click', e => {
  if (!draw) return;
  const p = [e.latlng.lat, e.latlng.lng];
  if (draw.tool === 'waypoint') {
    ensureManual();
    S.wps.push(newWp(p)); S.sel = S.wps.length - 1; commit(false);
  } else if (draw.tool === 'rect') {
    draw.pts.push(p);
    if (draw.pts.length === 2) {
      const [a, b] = draw.pts;
      setTool(null);
      setShape({ type: 'polygon', pts: [[a[0], a[1]], [a[0], b[1]], [b[0], b[1]], [b[0], a[1]]] });
    } else renderTemp();
  } else if (draw.tool === 'polygon' || draw.tool === 'barea' || draw.tool === 'bline') {
    draw.pts.push(p); renderTemp(); renderTools();
  } else if (draw.tool === 'measure') {
    measureAdd(p);
  } else if (draw.tool === 'marker') {
    setTool(null); const nm = addMarker(p); openMarker(nm.id, true);
  } else if (draw.tool === 'takeoff') {
    S.terrain.takeoff = p; setTool(null); commit(false); toast('Takeoff spot set');
  } else if (draw.tool === 'poi') {
    setTool(null); setShape({ type: 'poi', center: p });
  }
});
function finishPolygon() {
  if (!draw) return;
  const tool = draw.tool, pts = draw.pts;
  if (pts.length < (tool === 'bline' ? 2 : 3)) return;
  setTool(null);
  if (tool === 'barea' || tool === 'bline') { const sh = createShape(tool === 'barea' ? 'area' : 'line', pts); openShapeEditor(sh.id, true); return; }
  setShape({ type: 'polygon', pts });
}
function setShape(sh) {
  S.shape = sh;
  if (sh.type === 'poi') {
    S.mode = 'poi'; S.poiCenter = sh.center; S.turn = 'smooth';
    S.interval.auto = false;
  } else {
    S.mode = 'grid'; S.interval.on = true; S.reverse = false;
    if (S.interval.method === 'camera') { S.interval.auto = false; S.interval.type = 'time'; } else S.interval.auto = true;
  }
  S.sel = -1; regen(); commit(true);
  const b = routeBounds(); if (b) map.fitBounds(b, { padding: [40, 40], maxZoom: 19 });
}
function ensureManual() {
  if (S.mode === 'waypoint') return;
  S.mode = 'waypoint'; S.shape = null; S.interval.auto = false;
  if (S.interval.on) S.interval.value = coverage().interval;
  toast('Converted to editable waypoints');
}
function routeBounds() {
  const pts = S.wps.map(w => [w.lat, w.lng]);
  if (S.shape && S.shape.pts) pts.push(...S.shape.pts);
  return pts.length ? L.latLngBounds(pts) : null;
}

function renderTemp() {
  tempLayer.clearLayers();
  if (!draw || !draw.pts.length) return;
  const vt = L.divIcon({ className: '', html: '<div class="vtx"></div>', iconSize: [18, 18], iconAnchor: [9, 9] });
  draw.pts.forEach(p => L.marker(p, { icon: vt, interactive: false }).addTo(tempLayer));
  if (draw.pts.length > 1) L.polyline(draw.pts, { color: '#4dabf7', weight: 3, dashArray: '6 6' }).addTo(tempLayer);
}

function renderMap() {
  shapeLayer.clearLayers(); routeLayer.clearLayers();
  airDraw(shapeLayer);
  const sh = S.shape;
  if (sh && sh.type === 'polygon') {
    const poly = L.polygon(sh.pts, { color: '#4dabf7', weight: 2, fillOpacity: 0.12, interactive: false }).addTo(shapeLayer);
    const vt = L.divIcon({ className: '', html: '<div class="vtx"></div>', iconSize: [18, 18], iconAnchor: [9, 9] });
    sh.pts.forEach((p, i) => {
      const m = L.marker(p, { icon: vt, draggable: !featuresLocked(), interactive: !featuresLocked() }).addTo(shapeLayer);
      m.on('drag', e => { sh.pts[i] = [e.latlng.lat, e.latlng.lng]; poly.setLatLngs(sh.pts); });
      m.on('dragend', () => { regen(); commit(true); });
    });
  } else if (sh && sh.type === 'poi') {
    for (let k = 0; k < Math.max(1, S.poi.rings || 1); k++) {
      if (k > 0 && !S.poi.radStep) break;
      L.circle(sh.center, { radius: Math.max(3, S.poi.radius + k * (S.poi.radStep || 0)), color: '#4dabf7', weight: k ? 1.5 : 2, dashArray: k ? '4 6' : null, fillOpacity: k ? 0 : 0.08, interactive: false }).addTo(shapeLayer);
    }
    const m = L.marker(sh.center, { draggable: !featuresLocked(), interactive: !featuresLocked(), icon: L.divIcon({ className: '', html: '<div class="poim">POI</div>', iconSize: [30, 30], iconAnchor: [15, 15] }) }).addTo(shapeLayer);
    m.on('dragend', e => { sh.center = [e.target.getLatLng().lat, e.target.getLatLng().lng]; S.poiCenter = sh.center; regen(); commit(true); });
  } else if (S.poiCenter && S.wps.some(w => w.heading === 'poi')) {
    L.marker(S.poiCenter, { interactive: false, icon: L.divIcon({ className: '', html: '<div class="poim">POI</div>', iconSize: [30, 30], iconAnchor: [15, 15] }) }).addTo(shapeLayer);
  }
  const W = S.wps;
  if (S.terrain.on && S.terrain.takeoff) {
    const tm = L.marker(S.terrain.takeoff, { draggable: !featuresLocked(), interactive: !featuresLocked(), zIndexOffset: 1500, icon: L.divIcon({ className: '', html: '<div class="tkm">T</div>', iconSize: [30, 30], iconAnchor: [15, 15] }) }).addTo(routeLayer);
    tm.on('dragend', e => { S.terrain.takeoff = [e.target.getLatLng().lat, e.target.getLatLng().lng]; commit(false); });
  }
  if (S.mode === 'grid' && gridInfo && S.split && S.split.on && S.split.sel >= 0) L.polyline(gridInfo.full, { color: '#ffffff', weight: 1.5, opacity: 0.45, interactive: false }).addTo(routeLayer);
  if (multi()) segStore(), S.segs.forEach((sg, i) => {
    const col = SEG_COLORS[i % SEG_COLORS.length], pts = (sg.wps || []).map(w => [w.lat, w.lng]), nx = S.segs[i + 1];
    if (nx && pts.length && nx.wps && nx.wps.length) L.polyline([pts[pts.length - 1], [nx.wps[0].lat, nx.wps[0].lng]], { color: '#ffffff', weight: 2, dashArray: '6 8', opacity: 0.85, interactive: false }).addTo(routeLayer);
    if (i === S.seg) return;
    if (sg.shape && sg.shape.type === 'polygon') L.polygon(sg.shape.pts, { color: col, weight: 1.5, dashArray: '4 6', fillOpacity: 0.05, interactive: false }).addTo(shapeLayer);
    if (pts.length > 1) { const pl = L.polyline(pts, { color: col, weight: 3, opacity: 0.55, interactive: !draw }).addTo(routeLayer); pl.on('click', () => segActivate(i)); }
    if (pts.length) L.marker(pts[0], { interactive: !draw, zIndexOffset: 500, icon: L.divIcon({ className: '', html: `<div class="segm" style="background:${col}">${i + 1}</div>`, iconSize: [26, 26], iconAnchor: [13, 13] }) }).addTo(routeLayer).on('click', () => segActivate(i));
  });
  if (multi() && selectedPart()) { const pp = allFlightPoints().map(p => [p.w.lat, p.w.lng]); if (pp.length > 1) L.polyline(pp, { color: '#ffffff', weight: 8, opacity: 0.4, interactive: false }).addTo(routeLayer); }
  if (W.length > 1) L.polyline(W.map(w => [w.lat, w.lng]), { color: multi() ? SEG_COLORS[S.seg % SEG_COLORS.length] : '#ff7a2e', weight: 3, opacity: 0.95 }).addTo(routeLayer);
  const hzmp = hzShapes().length ? missionPoints() : null, hz = hzmp && hzmp.hazard;
  if (hz && (hz.detours || hz.climbs.length)) L.polyline(hzmp.map(p => [p.w.lat, p.w.lng]), { color: '#ffffff', weight: 2, dashArray: '2 6', opacity: 0.9, interactive: false }).addTo(routeLayer);
  if (hz) hz.spots.forEach(c => {
    L.polyline([c.a, c.b], { color: '#ffd43b', weight: 10, opacity: 0.45, lineCap: 'butt', interactive: false }).addTo(routeLayer);
    L.marker(c.at, { interactive: false, zIndexOffset: 1800, icon: L.divIcon({ className: '', html: `<div style="text-align:center"><span class="hzclimb">▲ ${fmtLen(c.to)} over ${esc(c.name)}</span></div>`, iconSize: [220, 22], iconAnchor: [110, 32] }) }).addTo(routeLayer);
  });
  if (hz) hz.problems.forEach(p => L.marker(p.at, { interactive: false, zIndexOffset: 2000, icon: L.divIcon({ className: '', html: '<div class="hzbad">!</div>', iconSize: [26, 26], iconAnchor: [13, 13] }) }).addTo(routeLayer));
  const rg = intervalRange();
  if (rg && !usesPhotoPoints()) L.polyline(W.slice(rg[0], rg[1] + 1).map(w => [w.lat, w.lng]), { color: '#ffd43b', weight: 2, dashArray: '2 8', opacity: 0.9, interactive: false }).addTo(routeLayer);
  else if (rg) {
    const shots = flightPoints().filter(p => p.photo || p.w.photo);
    if (shots.length <= 3000) shots.forEach(p => L.circleMarker([p.w.lat, p.w.lng], { renderer: photoCanvas, radius: 2.5, weight: 0, fillColor: '#ffd43b', fillOpacity: 0.95, interactive: false }).addTo(routeLayer));
  }
  const manual = S.mode === 'waypoint';
  const showAll = W.length <= 120;
  W.forEach((w, i) => {
    const first = i === 0, last = i === W.length - 1;
    if (!showAll && !first && !last && i !== S.sel) return;
    const cls = 'wpm' + (first ? ' first' : last ? ' last' : '') + (i === S.sel ? ' sel' : '') + (w.photo || w.rec ? ' photo' : '');
    const m = L.marker([w.lat, w.lng], {
      draggable: manual && (!draw || draw.tool === 'waypoint'), interactive: !draw || draw.tool === 'waypoint', zIndexOffset: i === S.sel ? 1000 : 0,
      icon: L.divIcon({ className: '', html: `<div class="${cls}">${i + 1}</div>`, iconSize: [26, 26], iconAnchor: [13, 13] }),
    }).addTo(routeLayer);
    m.on('click', () => { S.sel = S.sel === i ? -1 : i; commit(false); scrollToWp(i); });
    m.on('dragend', e => { w.lat = e.target.getLatLng().lat; w.lng = e.target.getLatLng().lng; commit(false); });
  });
}

/* ---------- toolbar ---------- */
const ICON = {
  sync: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M7 18a4.5 4.5 0 0 1-.6-9A6 6 0 0 1 18 9.5a4 4 0 0 1-.5 8.5"/><path d="M9.5 13.5a3 3 0 0 1 5.2-1.2M14.8 11v1.6h-1.6M14.5 15.5a3 3 0 0 1-5.2 1.2M9.2 18v-1.6h1.6"/></svg>',
  waypoint: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M12 21s-6-5.6-6-11a6 6 0 1 1 12 0c0 5.4-6 11-6 11z"/><circle cx="12" cy="10" r="2.2"/></svg>',
  rect: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><rect x="4" y="6" width="16" height="12" rx="1"/></svg>',
  polygon: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M5 8l7-4 7 5-2 10H7z"/></svg>',
  poi: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><circle cx="12" cy="12" r="8" stroke-dasharray="3 3"/><circle cx="12" cy="12" r="2.5" fill="currentColor"/></svg>',
  locate: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><circle cx="12" cy="12" r="4"/><path d="M12 2v3M12 19v3M2 12h3M19 12h3"/></svg>',
  layers: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M12 3l9 5-9 5-9-5z"/><path d="M3 13l9 5 9-5"/></svg>',
  check: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.4"><path d="M5 12l5 5 9-10"/></svg>',
  undo: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M9 7L4 12l5 5"/><path d="M4 12h10a6 6 0 0 1 0 12h-2" transform="translate(0 -6)"/></svg>',
  x: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.2"><path d="M6 6l12 12M18 6L6 18"/></svg>',
  measure: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M3 17L17 3l4 4L7 21z"/><path d="M7 13l2 2M10 10l2 2M13 7l2 2"/></svg>',
  markers: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M6 21V4M6 4h11l-2 4 2 4H6"/></svg>',
  missions: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M3 7a2 2 0 0 1 2-2h4l2 2h8a2 2 0 0 1 2 2v8a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2z"/></svg>',
  offline: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M7 18a4.5 4.5 0 0 1-.6-9A6 6 0 0 1 18 9.5a4 4 0 0 1-.5 8.5"/><path d="M12 11v9M8.5 16.5L12 20l3.5-3.5"/></svg>',
  trash: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M4 7h16M9 7V4h6v3M6 7l1 13h10l1-13"/></svg>',
};
function renderTools() {
  const t = draw && draw.tool;
  const b = (id, label, on, dis) => `<button class="tool${on ? ' on' : ''}" data-tool="${id}"${dis ? ' disabled' : ''}>${ICON[id === 'finish' ? 'check' : id === 'cancel' ? 'x' : id === 'clear' ? 'trash' : id] || ''}<span>${label}</span></button>`;
  let html = '<div class="toolgroup"><button class="tool hidetools" data-tool="hidetools" aria-label="Hide menu"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.4"><path d="M15 5l-7 7 7 7"/></svg><span>Hide menu</span></button></div>';
  html += '<div class="toolgroup">' + b('waypoint', 'Waypoint', t === 'waypoint') + b('rect', 'Rectangle', t === 'rect') +
    b('polygon', 'Polygon', t === 'polygon') + b('poi', 'POI orbit', t === 'poi') + '</div>';
  if (t === 'polygon' || t === 'barea' || t === 'bline') html += '<div class="toolgroup">' + b('finish', 'Finish', false, draw.pts.length < (t === 'bline' ? 2 : 3)) + b('undo', 'Undo point', false, !draw.pts.length) + b('cancel', 'Cancel') + '</div>';
  else if (t && t !== 'measure') html += '<div class="toolgroup">' + b('cancel', 'Done') + '</div>';
  html += '<div class="toolgroup">' + b('missions', 'Missions') + b('markers', 'Features') + b('sync', syLabel()) + b('measure', 'Measure', t === 'measure') + b('locate', 'My location') + b('layers', BASES[baseLayer].label, layerMenu) + b('offline', 'Offline maps') + b('clear', 'Clear', false, !S.wps.length && !S.shape) + '</div>';
  // The map menu opens beside the toolbar (not below it), so it never runs off the bottom of the screen.
  if (layerMenu) html += '<div class="flyout"><div class="toolgroup menu">' + Object.entries(BASES).map(([k, v]) => b('base:' + k, (k === baseLayer ? '✓ ' : '') + v.label)).join('') + '</div>' +
    '<div class="toolgroup menu">' + myMaps.map(m => b('my:' + m.id, (myState(m.id).on ? '✓ ' : '') + esc(m.name))).join('') + b('mymaps', 'My maps…') + '</div></div>';
  const tools = document.getElementById('tools');
  tools.innerHTML = html;
  const fly = tools.querySelector('.flyout'), lb = tools.querySelector('[data-tool=layers]');
  if (fly && lb) fly.style.top = Math.max(0, Math.min(lb.offsetTop + lb.closest('.toolgroup').offsetTop, tools.clientHeight - fly.offsetHeight)) + 'px';
}
/* Slide-away menus: the tool menu tucks off the left edge, the settings panel off the right. */
const appEl = document.getElementById('app');
function setHidden(which, hide) {
  appEl.classList.toggle(which, hide);
  try { localStorage.setItem('planner.' + which, hide ? '1' : ''); } catch (e) {}
  setTimeout(() => map.invalidateSize(), 280);
}
try { ['tools-hidden', 'panel-hidden'].forEach(k => { if (localStorage.getItem('planner.' + k)) appEl.classList.add(k); }); } catch (e) {}
document.getElementById('showtools').addEventListener('click', () => setHidden('tools-hidden', false));
document.getElementById('paneltab').addEventListener('click', () => setHidden('panel-hidden', !appEl.classList.contains('panel-hidden')));
document.getElementById('tools').addEventListener('click', e => {
  const btn = e.target.closest('[data-tool]'); if (!btn || btn.disabled) return;
  if (btn.dataset.tool === 'hidetools') { setHidden('tools-hidden', true); return; }
  const id = btn.dataset.tool;
  if (['waypoint', 'rect', 'polygon', 'poi'].includes(id)) setTool(draw && draw.tool === id ? null : id);
  else if (id === 'finish') finishPolygon();
  else if (id === 'undo') { draw.pts.pop(); renderTemp(); renderTools(); }
  else if (id === 'cancel') setTool(null);
  else if (id === 'locate') locate();
  else if (id === 'offline') openOffline();
  else if (id === 'missions') openLib();
  else if (id === 'markers') openMarkers();
  else if (id === 'sync') openSync();
  else if (id === 'measure') { if (draw && draw.tool === 'measure') setTool(null); else startMeasure(); }
  else if (id === 'layers') {
    layerMenu = !layerMenu; renderTools();
  } else if (id.startsWith('my:')) {
    const st = myState(id.slice(3)); st.on = !st.on; saveMyState(); applyMyMaps(); renderTools();
  } else if (id === 'mymaps') {
    layerMenu = false; renderTools(); openMyMaps();
  } else if (id.startsWith('base:')) {
    setBase(id.slice(5)); layerMenu = false; renderTools(); toast(BASES[baseLayer].label);
    if (baseLayer === 'usgs') probeUsgs();
  } else if (id === 'clear') {
    if (btn.dataset.armed) { S.segs = []; S.seg = 0; S.wps = []; S.shape = null; S.poiCenter = null; S.mode = 'waypoint'; S.libId = null; S.sel = -1; S.interval = clone(DEFAULTS.interval); S.split = clone(DEFAULTS.split); commit(true); toast('Mission cleared'); }
    else { btn.dataset.armed = '1'; btn.querySelector('span').textContent = 'Tap again to clear'; setTimeout(renderTools, 2500); }
  }
});

function locate() {
  if (native && native.getLocation) { native.getLocation(); toast('Finding your location…'); return; }
  if (!navigator.geolocation) return toast('Location is not available');
  navigator.geolocation.getCurrentPosition(p => gotLocation(p.coords.latitude, p.coords.longitude), () => toast('Location is not available'), { enableHighAccuracy: true, timeout: 10000 });
}
function gotLocation(lat, lng) {
  S.home = [lat, lng]; save();
  if (homeMarker) map.removeLayer(homeMarker);
  homeMarker = L.circleMarker(S.home, { radius: 8, color: '#fff', weight: 3, fillColor: '#1c7ed6', fillOpacity: 1 }).addTo(map);
  map.setView(S.home, Math.max(map.getZoom(), 17));
}

/* ---------- panel ---------- */
const panel = document.getElementById('panel');
function field(label, path, kind, opts) {
  opts = opts || {};
  let v = get(path), unit = '', type = 'number', step = opts.step || 'any';
  if (kind === 'len') { unit = LEN(); v = v == null ? '' : r0(toLen(v)); }
  else if (kind === 'spd') { unit = SPD(); v = v == null ? '' : r1(toSpd(v)); }
  else if (kind === 'pct') { unit = '%'; }
  else if (kind === 'deg') { unit = '°'; }
  else if (kind === 'sec') { unit = 's'; v = v == null ? '' : v; }
  else if (kind === 'text') { type = 'text'; }
  const ph = opts.ph != null ? ` placeholder="${esc(opts.ph)}"` : '';
  const dis = opts.disabled ? ' disabled' : '';
  // Tablet number keyboards often have no minus key, so fields that can go negative get a ± button.
  const signed = type === 'number' && !opts.disabled && (opts.signed || /gimbal|Gimbal|altStep|radStep|^poi\.height$|^hdg$|\.hdg$/.test(path));
  return `<label class="f${opts.disabled ? ' dim' : ''}"><span>${label}</span><div class="in"><input type="${type}" inputmode="${type === 'number' ? 'decimal' : 'text'}" step="${step}" data-f="${path}" data-k="${kind}" value="${v == null ? '' : esc(v)}"${ph}${dis}>${unit ? `<em>${unit}</em>` : ''}${signed ? `<button type="button" class="neg" data-neg="${path}" aria-label="Make negative or positive">±</button>` : ''}</div></label>`;
}
function select(label, path, options) {
  const v = String(get(path));
  return `<label class="f"><span>${label}</span><div class="in"><select data-f="${path}" data-k="sel">${options.map(([val, text]) => `<option value="${val}"${String(val) === v ? ' selected' : ''}>${text}</option>`).join('')}</select></div></label>`;
}
function check(label, path) { return `<label class="chk"><input type="checkbox" data-f="${path}" data-k="chk"${get(path) ? ' checked' : ''}>${label}</label>`; }
function get(path) { return path.split('.').reduce((o, k) => o == null ? undefined : o[k], S); }
function set(path, val) { const ks = path.split('.'); const last = ks.pop(); const o = ks.reduce((o, k) => o[k], S); o[last] = val; }

function renderPanel() {
  const keepScroll = panel.querySelector('.pbody') ? panel.querySelector('.pbody').scrollTop : 0;
  const st = stats();
  let h = `<div class="phead"><input data-f="name" data-k="text" value="${esc(S.name)}" aria-label="Mission name">
    <div class="seg"><button data-units="imperial" class="${imp() ? 'on' : ''}">ft · mph</button><button data-units="metric" class="${!imp() ? 'on' : ''}">m · m/s</button></div></div><div class="pbody">`;

  if (multi() && !S.wps.length && !S.shape) {
    h += `<div class="empty"><h2>Draw segment ${S.seg + 1}</h2>Pick a tool on the map: <b>Rectangle</b> or <b>Polygon</b> for a survey, <b>POI orbit</b>, or <b>Waypoint</b>. It's flown after segment ${S.seg}.</div>`;
  } else if (!S.wps.length && !S.shape) {
    h += `<div class="empty"><h2>Plan a mission</h2>Pick a tool on the map:<ol>
      <li><b>Rectangle</b> or <b>Polygon</b> — survey an area with an automatic grid.</li>
      <li><b>POI orbit</b> — circle a point with the camera locked on it.</li>
      <li><b>Waypoint</b> — tap points one at a time and set actions on each.</li></ol>
      Set altitude, speed and units below before or after you draw.</div>`;
  }

  h += segPanel(st);
  // Flight
  h += `<section class="grp"><h3>Flight${multi() ? ' · segment ' + (S.seg + 1) : ''}</h3>${multi() ? '<p class="note" style="margin-top:-4px">Altitude, speed and turns are for this segment. When finished and If signal is lost apply to the whole mission.</p>' : ''}<div class="row">${field(S.terrain.on ? 'Height above ground' : 'Altitude (above takeoff)', 'alt', 'len')}${field('Speed', 'speed', 'spd')}</div>
    <div class="row">${select('When finished', 'finish', [['goHome', 'Return to home'], ['noAction', 'Hover in place'], ['autoLand', 'Land'], ['gotoFirstWaypoint', 'Go to first waypoint']])}
    ${select('If signal is lost', 'rcLost', [['goBack', 'Return to home'], ['hover', 'Hover in place']])}</div>
    ${S.mode !== 'grid' ? `<div class="row">${select('Turns', 'turn', [['stop', 'Stop at each waypoint'], ['smooth', 'Smooth curves']])}${S.mode === 'waypoint' ? field('Default gimbal', 'defGimbal', 'deg') : '<div></div>'}</div>` : ''}
  </section>`;

  // Terrain following
  {
    const t = S.terrain, tk = t.takeoff;
    const mp = t.on ? missionPoints() : null, ti = mp && mp.terrainInfo;
    h += `<section class="grp"><h3>Terrain following</h3>
      ${check('Keep a steady height above the ground', 'terrain.on')}`;
    if (t.on) {
      h += `<p class="note">Uses USGS elevation data (about 30 ft detail, bare ground, not trees or buildings). Heights in the mission are worked out from your takeoff spot, so take off from the spot you set.</p>
        <div class="row"><label class="f"><span>Takeoff spot</span><div class="in"><input type="text" readonly value="${tk ? tk[0].toFixed(5) + ', ' + tk[1].toFixed(5) : 'Not set'}"></div></label>
        ${field('Hold height within', 'terrain.tol', 'len')}</div>
        <div class="btns" style="margin-bottom:10px"><button class="btn small" data-act="settakeoff">Set takeoff on map</button>${S.home ? '<button class="btn small" data-act="takeoffhere">Use my location</button>' : ''}</div>`;
      if (ti) h += `<div class="calc"><span>Ground at takeoff</span><b>${fmtLen(ti.e0)}</b><span>Ground under mission</span><b>${fmtLen(ti.gMin)} – ${fmtLen(ti.gMax)}</b>
        <span>Waypoint heights vs takeoff</span><b>${fmtLen(ti.minH)} – ${fmtLen(ti.maxH)}</b><span>Points added for terrain</span><b>${ti.added}</b></div>`;
    }
    h += '</section>';
  }

  // Hazards
  if (hzShapes().length) {
    const mp = missionPoints(), sum = hzSummary(mp), np = mp.hazard ? mp.hazard.problems.length : 0;
    h += `<section class="grp"><h3>Hazards · ${hzShapes().length}</h3>
      ${check('Climb over fly-over hazards (power lines, towers…)', 'hzClimb')}
      ${check('Route around no-fly zones (surveys skip them)', 'hzAvoid')}
      <div class="row">${field('Clearance above hazards', 'hzMargin', 'len')}<div></div></div>
      ${sum ? `<p class="note"><b>${sum}.</b></p>${hzProfile(mp)}` : (S.wps.length || multi()) && !np ? '<p class="note">The route stays clear of every hazard.</p>' : ''}
      ${np ? `<div class="warn bad">⚠ ${np} hazard problem${np === 1 ? '' : 's'}. See the warnings below.</div>` : ''}
      <p class="note">Mark hazards by editing a boundary or line in Features. Wires are hard for the drone's sensors to see, so plan the clearance here rather than relying on obstacle avoidance.</p>
      ${hfButton()}</section>`;
  } else if (st.points >= 2) {
    h += `<section class="grp"><h3>Hazards</h3><p class="note">No hazards marked. Look up mapped power lines, towers and wind turbines around this mission, or mark them yourself by editing a boundary or line in Features.</p>${hfButton()}</section>`;
  }
  h += airPanel();

  // Survey grid
  if (S.mode === 'grid') {
    const cov = coverage();
    h += `<section class="grp"><h3>Survey grid</h3>
      <div class="row">${select('Camera', 'camera', Object.entries(CAMERAS).map(([k, c]) => [k, c.label]))}${select('Photo size', 'photoSize', [['12', '12 MP'], ['full', S.camera === 'tele' ? '48 MP' : '50 MP']])}</div>
      <p class="note" style="margin-top:-6px">Photo size only changes the ground detail figure below. Spacing, overlap and timing are the same either way. Match it to the setting in DJI Fly.</p>
      <div class="row">${field('Front overlap', 'front', 'pct')}${field('Side overlap', 'side', 'pct', { disabled: !S.spacingAuto })}</div>
      ${check('Set path spacing from side overlap', 'spacingAuto')}
      <div class="row">${field('Distance between paths', 'spacing', 'len', { disabled: S.spacingAuto })}${field('Run-in past edges', 'runIn', 'len')}</div>
      ${select('Path between waypoints', 'pathStyle', [['anchors', 'DJI curves held straight with edge points (recommended)'], ['straightx', 'Straight-line flag (experimental)']])}
      <div class="row">${select('Line direction', 'dir', [['auto', 'Most efficient'], ['ns', 'North – South'], ['ew', 'East – West'], ['nesw', 'NE – SW'], ['nwse', 'NW – SE'], ['custom', 'Custom angle']])}
      ${S.dir === 'custom' ? field('Line bearing', 'angle', 'deg') : field('Gimbal angle', 'gridGimbal', 'deg')}</div>
      ${S.dir === 'custom' ? field('Gimbal angle', 'gridGimbal', 'deg') : ''}
      <div class="calc">
        <span>Line bearing used</span><b>${gridInfo ? r0(gridInfo.angle) + '°' : '—'}</b>
        <span>Flight lines</span><b>${gridInfo ? gridInfo.lines : '—'}</b>
        <span>Path spacing</span><b>${fmtLen(cov.spacing)}${S.spacingAuto ? '' : ` (${r0(cov.sideEff)}% side)`}</b>
        <span>Photo footprint</span><b>${fmtLen(cov.across)} × ${fmtLen(cov.along)}</b>
        <span>Ground resolution</span><b>${imp() ? (Math.round(cov.gsd / 2.54 * 100) / 100) + ' in/px' : r1(cov.gsd) + ' cm/px'}</b>
        <span>Area</span><b>${gridInfo ? (imp() ? r1(gridInfo.area / 4046.86) + ' acres' : r1(gridInfo.area / 10000) + ' ha') : '—'}</b>
      </div>
      <div class="btns"><button class="btn small primary" data-act="goal">Plan by goal…</button><button class="btn small" data-act="reverse">Reverse route</button></div>
    </section>
    ${multi() ? '' : `<section class="grp"><h3>Split into parts</h3>
      ${check('Split this survey into battery-sized parts', 'split.on')}
      ${S.split.on ? `<div class="row">${field('Max flying time per part', 'split.minutes', 'num', { step: 1 })}${S.interval.on && usesPhotoPoints() ? field('Max points per part', 'split.points', 'num', { step: 1 }) : '<div></div>'}</div>
      <p class="note" style="margin-top:-6px">Minutes of mission flying. Leave room for the trip out and back and a landing reserve.</p>
      ${select('Show / send', 'split.sel', [[-1, 'All parts (preview)'], ...(gridInfo ? gridInfo.parts : []).map((pt, i) => [i, 'Part ' + (i + 1) + ' of ' + gridInfo.parts.length + ' · ' + r0(pt.t / 60) + ' min · ' + (S.interval.on && usesPhotoPoints() ? pt.p : pt.b - pt.a + 1) + ' pts'])])}
      ${gridInfo && gridInfo.parts.length <= 1 ? `<p class="note">Fits in one part, so it sends as is.</p>` : S.split.sel < 0 ? `<p class="note">Pick a part to send it. Fly the parts in order, each with a fresh battery.</p>` : `<p class="note">Showing part ${S.split.sel + 1} of ${gridInfo.parts.length}. The rest of the survey is drawn in white.</p><div class="btns"><button class="btn small" data-act="partprev"${S.split.sel === 0 ? ' disabled' : ''}>‹ Previous part</button><button class="btn small" data-act="partnext"${S.split.sel >= gridInfo.parts.length - 1 ? ' disabled' : ''}>Next part ›</button></div>`}${gridInfo && gridInfo.parts.length > 1 ? '<div class="btns" style="margin-top:6px"><button class="btn small" data-act="partsave">Save parts to Missions</button></div>' : ''}` : ''}
    </section>`}`;
  }

  // POI
  if (S.mode === 'poi') {
    h += `<section class="grp"><h3>POI orbit</h3>
      <div class="row">${field('Radius', 'poi.radius', 'len')}${field('Waypoints', 'poi.points', 'num', { step: 1 })}</div>
      <div class="row">${field('POI height', 'poi.height', 'len')}${field('Start bearing', 'poi.start', 'deg')}</div>
      <div class="row">${select('Direction', 'poi.cw', [['true', 'Clockwise'], ['false', 'Counter-clockwise']])}${field('Gimbal angle', 'poi.gimbal', 'deg', { disabled: S.poi.gimbalAuto })}</div>
      ${check('Aim gimbal at the POI automatically', 'poi.gimbalAuto')}
      ${check('Finish where it started (full loop)', 'poi.loop')}
      <div class="row">${field('Number of orbits', 'poi.rings', 'num', { step: 1 })}${S.poi.rings > 1 ? select('Climb between orbits', 'poi.climb', [['step', 'Step up after each orbit'], ['spiral', 'Spiral up smoothly']]) : '<div></div>'}</div>
      ${S.poi.rings > 1 ? `<div class="row">${field('Height change per orbit', 'poi.altStep', 'len')}${field('Radius change per orbit', 'poi.radStep', 'len')}</div>
      <p class="note" style="margin-top:-6px">Orbit 1 flies at the mission altitude (${fmtLen(S.alt)}). Use a negative change to work downward. Top orbit: ${fmtLen(S.alt + (S.poi.rings - 1) * S.poi.altStep)}, radius ${fmtLen(Math.max(3, S.poi.radius + (S.poi.rings - 1) * S.poi.radStep))}.</p>` : ''}
    </section>`;
  }

  // Interval photos
  if (S.wps.length >= 2 || S.mode === 'grid') {
    const cov = coverage(), n = S.wps.length;
    const timeMode = S.interval.type === 'time';
    h += `<section class="grp"><h3>Interval photos</h3>
      ${check('Take photos at a fixed interval', 'interval.on')}`;
    if (S.interval.on) {
      h += select('How photos are triggered', 'interval.method', [['points', 'Photo waypoints along the route'], ['camera', 'Camera timed shot, set in DJI Fly']]);
      if (S.interval.method === 'camera') h += `<p class="note">The mission file only holds the line corners, so there are no photo waypoints. Before you start the mission, set DJI Fly's photo mode to <b>Timed Shot</b> at the interval below and start shooting once the drone reaches waypoint 1. Stop it after the last waypoint. Expect some extra photos on the turns, which mapping software ignores or you can delete.</p>`;
      if (S.interval.method === 'camera') h += `<div class="row">${field('Timed Shot interval', 'interval.value', 'sec')}<div></div></div>`;
      else h += `<div class="row">${select('Trigger', 'interval.type', [['time', 'Every N seconds'], ['distance', 'Every N ' + LEN()]])}
        ${timeMode ? field('Camera interval', 'interval.value', 'sec', { disabled: S.interval.auto }) : field('Photo spacing', 'interval.value', 'len', { disabled: S.interval.auto })}</div>`;
      if (S.mode === 'grid' && S.interval.method !== 'camera') h += check('Calculate from front overlap and speed', 'interval.auto');
      if (S.interval.auto) h += `<p class="note">Using ${timeMode ? cov.interval + ' s' : fmtLen(cov.interval)} for ${S.front}% front overlap.</p>`;
      else if (S.mode === 'grid') h += `<p class="note">Effective front overlap: <b>${r0(cov.frontEff)}%</b>. <button class="btn small" data-act="fitspeed">Match speed to interval</button></p>`;
      if (S.mode !== 'grid' && n >= 2) {
        const opts = S.wps.map((w, i) => [i, 'Waypoint ' + (i + 1)]);
        h += `<div class="row">${select('Start at', 'interval.from', opts.slice(0, -1))}${select('Stop at', 'interval.to', [[-1, 'Last waypoint'], ...opts.slice(1)])}</div>`;
      }
    }
    h += '</section>';
  }

  // Waypoints
  if (S.wps.length) {
    const gims = wpGimbals();
    h += `<section class="grp"><h3>Waypoints · ${S.wps.length}${S.mode === 'waypoint' ? '' : '<span style="text-transform:none;letter-spacing:0;font-weight:600">Edits convert to manual</span>'}</h3><div class="wpl">`;
    S.wps.forEach((w, i) => {
      const tags = [w.photo ? 'Photo' : '', w.rec === 'start' ? 'Start video' : '', w.rec === 'stop' ? 'Stop video' : '', w.hover ? `Hover ${w.hover}s` : '', w.heading === 'fixed' ? `Heading ${w.hdg}°` : '', w.heading === 'poi' ? 'Face POI' : ''].filter(Boolean).join(' · ');
      h += `<div class="wp${i === S.sel ? ' sel' : ''}" id="wp${i}"><div class="wph" data-wp="${i}"><div class="wpn">${i + 1}</div>
        <div class="wps"><b>${fmtLen(wpAlt(w))}</b> · ${r0(gims[i])}°${tags ? ' · ' + tags : ''}</div></div>`;
      if (i === S.sel) {
        const p = 'wps.' + i + '.';
        h += `<div class="wpe"><div class="row">${field('Altitude', p + 'alt', 'len', { ph: r0(toLen(S.alt)) + ' (mission)' })}${field('Speed', p + 'speed', 'spd', { ph: r1(toSpd(S.speed)) + ' (mission)' })}</div>
          <label class="f"><span>Gimbal angle${w.gimbal == null ? ' (inherited)' : ''}</span><div class="range"><input type="range" min="-90" max="0" step="1" value="${r0(gims[i])}" data-f="${p}gimbal" data-k="range"><output>${r0(gims[i])}°</output></div></label>
          <div class="row">${select('Aircraft heading', p + 'heading', [['route', 'Follow route'], ['fixed', 'Fixed bearing'], ...(S.poiCenter ? [['poi', 'Face the POI']] : [])])}
          ${w.heading === 'fixed' ? field('Bearing', p + 'hdg', 'deg') : field('Hover', p + 'hover', 'sec')}</div>
          ${w.heading === 'fixed' ? field('Hover', p + 'hover', 'sec') : ''}
          <div class="acts">${check('Take photo', p + 'photo')}
          ${select('Video', p + 'rec', [['', 'No change'], ['start', 'Start recording'], ['stop', 'Stop recording']])}</div>
          <div class="btns"><button class="btn small" data-act="prev"${i === 0 ? ' disabled' : ''}>‹ Prev</button><button class="btn small" data-act="next"${i === S.wps.length - 1 ? ' disabled' : ''}>Next ›</button><button class="btn small danger" data-act="del">Delete</button></div></div>`;
      }
      h += '</div>';
    });
    h += '</div></section>';
  }
  h += '</div>';

  // Footer
  h += `<div class="pfoot"><div class="stats">
      <div class="stat"><small>Distance</small><b>${fmtDist(st.dist)}</b></div>
      <div class="stat"><small>Time</small><b>${fmtTime(st.time)}</b></div>
      <div class="stat"><small>Photos</small><b>${st.photos}</b></div>
      <div class="stat"><small>Points</small><b>${st.points}</b></div></div>
    ${st.warns.length ? `<div class="warns">${st.warns.map(([c, t]) => `<div class="warn ${c}">${c === 'info' ? '▲' : '⚠'} ${t}</div>`).join('')}</div>` : ''}
    <div class="btns"><button class="btn" data-act="save"${st.points < 2 ? ' disabled' : ''}>Save KMZ</button>
    <button class="btn primary" data-act="send"${st.points < 2 ? ' disabled' : ''}>Send to controller</button></div></div>`;
  panel.innerHTML = h;
  const pb = panel.querySelector('.pbody'); if (pb) pb.scrollTop = keepScroll;
}
let segArmed = -1;
function segPanel(st) {
  if (!multi()) {
    if (!S.wps.length && !S.shape) return '';
    return `<section class="grp"><h3>Combine</h3><p class="note">Fly an orbit, survey or waypoints after this, all in one mission.</p><div class="btns"><button class="btn small" data-act="segadd">+ Add a segment</button></div></section>`;
  }
  const per = st.per || segStats();
  let h = `<section class="grp"><h3>Segments · ${S.segs.length}</h3><p class="note" style="margin-top:-4px">Flown in this order, joined by straight legs (dashed white). Tap one to edit it; you're editing segment ${S.seg + 1}.</p><div class="seglist">`;
  S.segs.forEach((sg, i) => {
    const p = per[i] || {};
    h += `<div class="segrow${i === S.seg ? ' on' : ''}"><i style="background:${SEG_COLORS[i % SEG_COLORS.length]}"></i>
      <button class="segname" data-act="segsel" data-i="${i}"><b>${esc(segLabel(sg, i))}</b><small>${p.points >= 2 ? fmtTime(p.time) + ' · ' + p.points + ' points' : 'Not drawn yet'}</small></button>
      <button class="btn small" data-act="segup" data-i="${i}"${i === 0 ? ' disabled' : ''} aria-label="Move up">↑</button><button class="btn small" data-act="segdown" data-i="${i}"${i === S.segs.length - 1 ? ' disabled' : ''} aria-label="Move down">↓</button>
      <button class="btn small danger" data-act="segdel" data-i="${i}" aria-label="Delete segment">${segArmed === i ? 'Delete?' : '✕'}</button></div>`;
  });
  h += `</div><div class="btns"><button class="btn small primary" data-act="segadd">+ Add a segment</button></div></section>`;
  const parts = S.split.on ? missionParts() : [];
  h += `<section class="grp"><h3>Split into parts</h3>${check('Split the mission into battery-sized parts', 'split.on')}`;
  if (S.split.on) {
    const lim = partLimits();
    h += `<div class="row">${field('Max flying time per part', 'split.minutes', 'num', { step: 1 })}${lim.maxP < PART_MAX_POINTS || S.segs.some(sg => sg.interval && sg.interval.on && sg.interval.method !== 'camera') ? field('Max points per part', 'split.points', 'num', { step: 1 }) : '<div></div>'}</div>
      <p class="note" style="margin-top:-6px">Parts break between survey lines and never in the middle of an orbit or a run of waypoints. Each also stays under ${PART_MAX_POINTS} points.</p>`;
    if (parts.length <= 1) h += `<p class="note">Fits in one part, so it sends as is.</p>`;
    else {
      h += select('Show / send', 'split.sel', [[-1, 'All parts (preview)'], ...parts.map((pt, i) => [i, partLabel(pt, i, parts.length)])]);
      h += S.split.sel < 0 ? `<p class="note">Pick a part to send it. Fly the parts in order, each with a fresh battery.</p>`
        : `<p class="note">Showing part ${S.split.sel + 1} of ${parts.length}, highlighted in white.</p><div class="btns"><button class="btn small" data-act="mpartprev"${S.split.sel === 0 ? ' disabled' : ''}>‹ Previous part</button><button class="btn small" data-act="mpartnext"${S.split.sel >= parts.length - 1 ? ' disabled' : ''}>Next part ›</button></div>`;
      h += `<div class="btns" style="margin-top:6px"><button class="btn small" data-act="partsave">Save parts to Missions</button></div>`;
    }
  }
  h += '</section>';
  return h;
}
function scrollToWp(i) { setTimeout(() => { const e = document.getElementById('wp' + i); if (e) e.scrollIntoView({ block: 'nearest', behavior: 'smooth' }); }, 30); }

const REGEN_KEYS = /^(hzAvoid|alt|speed|camera|front|side|spacing|spacingAuto|dir|angle|gridGimbal|runIn|pathStyle|split\.|interval\.|poi\.)/;
panel.addEventListener('input', e => {
  const t = e.target; if (t.dataset.k === 'range') t.nextElementSibling.textContent = t.value + '°';
});
panel.addEventListener('change', e => {
  const t = e.target, path = t.dataset.f; if (!path) return;
  const k = t.dataset.k;
  let v;
  if (k === 'chk') v = t.checked;
  else if (k === 'sel') v = t.value === 'true' ? true : t.value === 'false' ? false : /^-?\d+(\.\d+)?$/.test(t.value) ? Number(t.value) : t.value;
  else if (k === 'text') v = t.value.trim() || 'Mission';
  else if (k === 'range') v = Number(t.value);
  else {
    if (t.value === '') v = path.startsWith('wps.') ? (path.endsWith('hover') ? 0 : null) : undefined;
    else {
      const n = Number(t.value); if (!isFinite(n)) return renderPanel();
      v = k === 'len' ? fromLen(n) : k === 'spd' ? fromSpd(n) : n;
    }
    if (v === undefined) return renderPanel();
    if (path === 'alt' || path.endsWith('.alt')) v = v == null ? v : Math.max(2, v);
    if (path === 'speed' || path.endsWith('.speed')) v = v == null ? v : Math.min(Math.max(0.5, v), 20);
    // Gimbal angles are always 0 to -90, so a positive entry (no minus key on the tablet keyboard) means downward.
    if (/gimbal$|Gimbal$/.test(path)) v = v == null ? v : Math.min(0, Math.max(-90, -Math.abs(v)));
    if (k === 'pct') v = Math.min(95, Math.max(0, v));
    if (path === 'spacing') v = Math.max(1, v);
    if (path === 'runIn') v = Math.min(100, Math.max(0, v));
    if (path === 'terrain.tol') v = Math.min(30, Math.max(1, v));
    if (path === 'split.minutes') v = Math.min(40, Math.max(3, Math.round(v)));
    if (path === 'split.points') v = Math.min(500, Math.max(20, Math.round(v)));
    if (path === 'interval.value') v = Math.max(k === 'sec' ? 0.5 : 1, v);
    if (path === 'poi.radius') v = Math.max(3, v);
    if (path === 'poi.points') v = Math.min(60, Math.max(3, Math.round(v)));
    if (path === 'poi.rings') v = Math.min(8, Math.max(1, Math.round(v)));
  }
  if (path.startsWith('wps.')) {
    ensureManual();
    set(path, v);
  } else {
    set(path, v);
    if (path === 'interval.on' && v && S.mode !== 'grid') S.interval.auto = false;
    if (path === 'hzMargin') S.hzMargin = Math.min(60, Math.max(0, v));
    if (path === 'interval.method' && v === 'camera') { S.interval.auto = false; S.interval.type = 'time'; S.interval.value = 3; }
    if (path === 'interval.type' && !S.interval.auto) S.interval.value = v === 'time' ? 3 : 15;
    if (S.mode !== 'waypoint' && REGEN_KEYS.test(path)) regen();
  }
  commit(false);
});
panel.addEventListener('click', e => {
  const ng = e.target.closest('[data-neg]');
  if (ng) {
    e.preventDefault();
    const inp = panel.querySelector(`input[data-f="${ng.dataset.neg}"]`);
    if (inp && inp.value !== '') { inp.value = String(-Number(inp.value)); inp.dispatchEvent(new Event('change', { bubbles: true })); }
    return;
  }
  const u = e.target.closest('[data-units]');
  if (u) { S.units = u.dataset.units; commit(false); return; }
  const head = e.target.closest('[data-wp]');
  if (head) { const i = Number(head.dataset.wp); S.sel = S.sel === i ? -1 : i; commit(false); if (S.sel >= 0) { map.panTo([S.wps[i].lat, S.wps[i].lng]); scrollToWp(i); } return; }
  const a = e.target.closest('[data-act]'); if (!a || a.disabled) return;
  const act = a.dataset.act, si = Number(a.dataset.i);
  if (act !== 'segdel') segArmed = -1;
  if (airAct(act, a)) return;
  if (act === 'hfind') { openFindHazards(); return; }
  if (act === 'segadd') { segAdd(); return; }
  if (act === 'segsel') { if (si !== S.seg) segActivate(si); else fitMission(); return; }
  if (act === 'segup' || act === 'segdown') { segMove(si, act === 'segup' ? -1 : 1); return; }
  if (act === 'segdel') { if (segArmed === si) { segArmed = -1; segDelete(si); } else { segArmed = si; renderPanel(); } return; }
  if (act === 'segsplit' || act === 'partsave') { savePartsToLibrary(); return; }
  if (act === 'mpartprev' || act === 'mpartnext') { S.split.sel += act === 'mpartnext' ? 1 : -1; commit(false); fitMission(); return; }
  if (act === 'goal') { openGoal(); return; }
  if (act === 'settakeoff') { setTool('takeoff'); return; }
  if (act === 'takeoffhere') { if (S.home) { S.terrain.takeoff = S.home.slice(); commit(false); toast('Takeoff spot set to your location'); } return; }
  if (act === 'reverse') { S.reverse = !S.reverse; regen(); commit(false); }
  else if (act === 'partprev' || act === 'partnext') { S.split.sel += act === 'partnext' ? 1 : -1; regen(); commit(false); fitMission(); }
  else if (act === 'fitspeed') { const cov = coverage(); if (S.interval.type === 'time') { S.speed = Math.min(15, Math.max(0.5, cov.alongDist / S.interval.value)); regen(); commit(false); toast('Speed set to ' + r1(toSpd(S.speed)) + ' ' + SPD()); } }
  else if (act === 'prev') { S.sel--; commit(false); scrollToWp(S.sel); }
  else if (act === 'next') { S.sel++; commit(false); scrollToWp(S.sel); }
  else if (act === 'del') { ensureManual(); S.wps.splice(S.sel, 1); S.sel = Math.min(S.sel, S.wps.length - 1); commit(false); }
  else if (act === 'save') saveKmz();
  else if (act === 'send') openSend();
});

function commit(fit) {
  mpCache = null; partsCache = null;
  const cov = coverage();
  if (S.spacingAuto) S.spacing = cov.spacing;
  if (S.interval.auto) S.interval.value = cov.interval;
  segStore();
  if (multi() && S.split.on) { const n = missionParts().length; if (S.split.sel >= n) S.split.sel = -1; }
  save(); renderMap(); renderPanel(); renderTools(); airSchedule(); }

/* ---------- toast ---------- */
let toastT = null;
function toast(msg) { const t = document.getElementById('toast'); t.textContent = msg; t.hidden = false; clearTimeout(toastT); toastT = setTimeout(() => { t.hidden = true; }, Math.max(2800, String(msg).length * 55)); }

/* ---------- export / native bridge ---------- */
function fileName() { return (S.name || 'mission').replace(/[^\w\- ]+/g, '').trim().replace(/\s+/g, '_') || 'mission'; }
function saveKmz() { openSend('save'); }
function doSaveKmz() {
  let k; try { k = buildKmz(); } catch (e) { return toast(e.message); }
  if (native) native.saveKmz(fileName(), k.template, k.waylines);
  else toast('Saving files works in the Android app.');
}

/* Camera settings live in DJI Fly, not in the mission file, so they are listed for a quick check before each flight. */
/* ---------- mission names on the controller ----------
   DJI Fly shows each mission with the picture in waypoint/map_preview/<id>/<id>.jpg (1920 × 1080). When sending, the app
   replaces that picture with one showing the route and the mission's name, so the missions are easy to tell apart. */
function sentLog() { try { return JSON.parse(localStorage.getItem('planner.sent')) || {}; } catch (e) { return {}; } }
function partInfo() {
  if (multi()) { const parts = S.split.on ? missionParts() : []; return parts.length > 1 && S.split.sel >= 0 ? [S.split.sel + 1, parts.length] : null; }
  return S.mode === 'grid' && S.split.on && gridInfo && gridInfo.parts.length > 1 && S.split.sel >= 0 ? [S.split.sel + 1, gridInfo.parts.length] : null;
}
function missionTitle() { const p = partInfo(); return S.name + (p ? ' · part ' + p[0] + ' of ' + p[1] : ''); }
function previewJpeg() {
  const W = 1920, H = 1080, cv = document.createElement('canvas'); cv.width = W; cv.height = H;
  const g = cv.getContext('2d');
  g.fillStyle = '#e8eedf'; g.fillRect(0, 0, W, H);
  const P = missionPoints(), st = stats();
  const pts = P.map(p => [p.w.lat, p.w.lng]);
  if (pts.length < 2) throw new Error('no route');
  const lat0 = pts.reduce((a, p) => a + p[0], 0) / pts.length, kx = Math.cos(lat0 * Math.PI / 180);
  const xy = p => [p[1] * kx, -p[0]];
  const Q = pts.map(xy);
  let x1 = Infinity, x2 = -Infinity, y1 = Infinity, y2 = -Infinity;
  Q.forEach(([x, y]) => { x1 = Math.min(x1, x); x2 = Math.max(x2, x); y1 = Math.min(y1, y); y2 = Math.max(y2, y); });
  const box = { l: 260, t: 270, w: 1400, h: 600 };
  const sc = Math.min(box.w / Math.max(1e-9, x2 - x1), box.h / Math.max(1e-9, y2 - y1));
  const ox = box.l + (box.w - (x2 - x1) * sc) / 2, oy = box.t + (box.h - (y2 - y1) * sc) / 2;
  const px = p => { const [x, y] = xy(p); return [ox + (x - x1) * sc, oy + (y - y1) * sc]; };
  // property boundaries and other areas, faint
  try { shapes.forEach(sh => { if (sh.kind !== 'area' || sh.pts.length < 3) return; g.beginPath(); sh.pts.forEach((p, i) => { const [x, y] = px(p); i ? g.lineTo(x, y) : g.moveTo(x, y); }); g.closePath(); g.strokeStyle = 'rgba(80,90,60,.45)'; g.lineWidth = 4; g.setLineDash([16, 12]); g.stroke(); g.setLineDash([]); }); } catch (e) {}
  // route: white casing, then colored by segment
  g.lineJoin = 'round'; g.lineCap = 'round';
  g.beginPath(); pts.forEach((p, i) => { const [x, y] = px(p); i ? g.lineTo(x, y) : g.moveTo(x, y); });
  g.strokeStyle = '#ffffff'; g.lineWidth = 16; g.stroke();
  for (let i = 1; i < pts.length; i++) {
    const seg = P[i].seg != null ? P[i].seg : -1, a = px(pts[i - 1]), b = px(pts[i]);
    g.beginPath(); g.moveTo(a[0], a[1]); g.lineTo(b[0], b[1]);
    g.strokeStyle = seg >= 0 ? SEG_COLORS[seg % SEG_COLORS.length] : '#ff7a2e'; g.lineWidth = 9; g.stroke();
  }
  const dot = (p, col, label) => { const [x, y] = px(p); g.beginPath(); g.arc(x, y, 30, 0, Math.PI * 2); g.fillStyle = col; g.fill(); g.lineWidth = 6; g.strokeStyle = '#fff'; g.stroke(); g.fillStyle = '#fff'; g.font = 'bold 32px sans-serif'; g.textAlign = 'center'; g.textBaseline = 'middle'; g.fillText(label, x, y + 1); };
  dot(pts[0], '#2b8a3e', 'S'); dot(pts[pts.length - 1], '#c92a2a', 'E');
  // name, large enough to read on DJI Fly's small mission cards
  const text = (t, y, max, size, weight) => {
    let f = size; g.font = `${weight} ${f}px sans-serif`;
    while (g.measureText(t).width > max && f > 40) { f -= 4; g.font = `${weight} ${f}px sans-serif`; }
    g.textAlign = 'center'; g.textBaseline = 'middle'; g.lineJoin = 'round';
    g.lineWidth = Math.max(8, f / 7); g.strokeStyle = '#ffffff'; g.strokeText(t, W / 2, y);
    g.fillStyle = '#14201a'; g.fillText(t, W / 2, y);
  };
  const part = partInfo();
  text(S.name, 150, 1800, 150, '800');
  const sub = [part ? 'Part ' + part[0] + ' of ' + part[1] : '', multi() ? (n => n + (n === 1 ? ' segment' : ' segments'))(new Set(P.map(p => p.seg)).size) : MODE_LABEL[S.mode] || '', fmtTime(st.time) + ' min', st.points + ' points'].filter(Boolean).join('  ·  ');
  text(sub, 960, 1800, 64, '700');
  text('Sent ' + new Date().toLocaleDateString([], { month: 'short', day: 'numeric', year: 'numeric' }) + ' from FlyPlan', 1035, 1800, 40, '600');
  return cv.toDataURL('image/jpeg', 0.85).split(',')[1];
}
window.__previewJpeg = previewJpeg;   // for testing
function preflightChecklist() {
  const st = stats(), cov = coverage(), tele = S.camera === 'tele';
  const camMode = S.interval.on && S.interval.method === 'camera';
  // fastest shutter needed so motion stays under half a pixel, rounded up to a standard setting (never slower than 1/1000)
  const need = S.speed / (0.5 * cov.gsd / 100);
  const shutter = [1000, 1250, 1600, 2000, 2500, 3200, 4000, 5000, 6400, 8000].find(v => v >= need) || 8000;
  const items = [
    [`Camera: <b>${tele ? '3–9x camera at exactly 3x' : '1–2.9x camera at exactly 1x'}</b>`, 'Any other zoom shrinks each photo and leaves gaps between lines.'],
    [`Photo size: <b>${S.photoSize === '12' ? '12 MP' : (tele ? '48 MP' : '50 MP')}</b>`, 'Matches the detail the app planned.'],
    camMode
      ? [`Photo mode: <b>Timed Shot every ${r1(S.interval.value)} s</b>`, 'Start it when the drone reaches waypoint 1 and stop it after the last waypoint.']
      : [`Photo mode: <b>Single</b>`, S.interval.on ? 'The mission takes each photo at its photo waypoints.' : 'Photos are only taken where you set them on waypoints.'],
    [`Shutter: <b>1/${shutter} s or faster</b>`, `Keeps photos sharp at ${r1(toSpd(S.speed))} ${SPD()}. Use shutter priority or manual mode.`],
    [`Focus: <b>infinity</b>`, 'Or tap to focus on the ground once at altitude, before the first line.'],
    [`Gimbal: <b>${S.mode === 'grid' ? S.gridGimbal : S.defGimbal}°</b> at the start`, 'The mission sets it at waypoint 1. Check it moved.'],
    [`Mission time in DJI Fly: <b>about ${Math.round(st.time / 60)} min</b>`, 'A little more is normal. If it shows much longer, the speed didn’t take: set it in DJI Fly before flying.'],
  ];
  const mp = S.terrain && S.terrain.on ? missionPoints() : null;
  if (mp && mp.terrainInfo) {
    items.push([`Take off from the <b>T</b> spot you set`, 'Every height in the mission is measured from there.']);
    if (mp.terrainInfo.maxH > 120) items.push([`DJI Fly Max Altitude: <b>${fmtLen(mp.terrainInfo.maxH + 3)} or higher</b>`, 'Waypoints over low ground sit that high above takeoff.']);
  }
  return `<h3 class="subh">Before takeoff, in DJI Fly</h3><div class="checks">` +
    items.map(([t, d], i) => `<label class="ck"><input type="checkbox" id="ck${i}"><span>${t}<small>${d}</small></span></label>`).join('') + '</div>';
}

const sheet = document.getElementById('sheet');
let send = { step: 'intro', missions: [], pick: null, status: '', cls: '' };
function openSend(after) {
  sheetMode = 'send'; send = { step: 'intro', missions: [], pick: null, status: '', cls: '', after: after || 'send' };
  const hz = missionPoints().hazard;
  if ((hz && hz.problems.length) || airBlocking().length) send.step = 'hazard';
  else if (after === 'save') { doSaveKmz(); return; }
  renderSheet(); sheet.hidden = false;
}
function renderSheet() {
  let h = '<div class="card">';
  const onCtl = !!(native && native.isOnController && native.isOnController());
  if (send.step === 'hazard') {
    const hz = missionPoints().hazard, probs = hz ? hz.problems : [], ab = airBlocking();
    if (probs.length) h += `<h2>Hazards on this route</h2><div class="mlist">${probs.map((p, i) => `<div class="mitem"><div style="flex:1"><b>${esc(p.name)}</b><small>${p.kind === 'low' ? 'Passes ' + fmtLen(p.alt) + ' above takeoff; needs ' + fmtLen(p.need) : p.kind === 'inside' ? 'A waypoint is inside this no-fly zone' : 'Route enters this no-fly zone'}</small></div><button class="btn small" data-s="hzgo" data-i="${i}">Show</button></div>`).join('')}</div>
      <p class="note">Fix these first: turn on <b>Climb over</b> and <b>Route around</b> under Hazards, move the waypoints, or change the hazard. Sending anyway is at your own risk.</p>`;
    if (ab.length) h += `<h2>${probs.length ? 'Airspace' : 'Check your airspace authorization'}</h2><div class="mlist">${ab.map((it, i) => `<div class="mitem"><div style="flex:1"><b>${esc(it.title)}</b><small style="font-family:var(--font)">${it.detail}</small></div><button class="btn small" data-s="airgo" data-i="${i}">Show</button></div>`).join('')}</div>
      <p class="note">Have your LAANC or DroneZone approval before you fly. This app can't request it for you.</p>
      <div class="btns" style="margin-bottom:8px"><button class="btn small" data-s="airreq">Request authorization</button><button class="btn small" data-s="airtfr">Check TFRs</button></div>`;
    const ok = send.after === 'save' ? 'Save anyway' : 'Send anyway', okAir = probs.length ? ok : send.after === 'save' ? 'Save' : 'Continue';
    h += `<div class="btns"><button class="btn" data-s="close">${probs.length ? 'Fix the route' : 'Cancel'}</button><button class="btn ${probs.length ? 'danger' : 'primary'}" data-s="hzok">${ab.length && !probs.length ? 'I have authorization · ' + okAir : ok}</button></div>`;
    sheet.innerHTML = h + '</div>'; return;
  }
  if (send.step === 'access') {
    h += `<h2>Allow access to DJI Fly's missions</h2>
      <p class="note">Android needs your OK once before this app can write into DJI Fly's mission folder.</p>
      <ol><li>Tap <b>Grant folder access</b>.</li>
      <li>The picker should open in <b>Android › data › dji.go.v5 › files › waypoint</b>. If it doesn't, browse there.</li>
      <li>Tap <b>Use this folder</b>, then <b>Allow</b>.</li></ol>
      <p class="note">If the picker says it can't use this folder, this Android version blocks it. Run the app on the tablet and send over USB instead.</p>`;
    if (send.status) h += `<div class="status ${send.cls}">${esc(send.status)}</div>`;
    h += `<div class="btns"><button class="btn" data-s="close">Cancel</button><button class="btn primary" data-s="grant">Grant folder access</button></div>`;
  } else if ((send.step === 'intro' || send.step === 'finding') && onCtl) {
    h += `<h2>Send to DJI Fly</h2>
      <p class="note">This app is running on the controller, so no cable is needed. It replaces a placeholder mission saved in DJI Fly with “${esc(S.name)}” and backs up the original first.</p>
      <ol><li>One time only: in DJI Fly, open <b>Waypoints</b>, drop two points anywhere and <b>save</b> the mission. That becomes your placeholder.</li>
      <li>Close DJI Fly (swipe it away in recent apps) so it reloads the mission.</li></ol>`;
    if (send.status) h += `<div class="status ${send.cls}">${esc(send.status)}</div>`;
    h += `<div class="btns"><button class="btn" data-s="close">Cancel</button><button class="btn primary" data-s="find"${send.step === 'finding' ? ' disabled' : ''}>${send.step === 'finding' ? 'Looking…' : 'Find missions'}</button></div>`;
  } else if (send.step === 'intro' || send.step === 'finding') {
    h += `<h2>Send to the RC Pro 2</h2>
      <p class="note">This replaces a placeholder mission saved in DJI Fly with “${esc(S.name)}”. The original is backed up on ${DEV} first.</p>
      <ol><li>One time only: in DJI Fly, open <b>Waypoints</b>, drop two points anywhere and <b>save</b> the mission. That becomes your placeholder.</li>
      <li>Close DJI Fly on the controller.</li>
      <li>Connect the controller to ${DEV} with a USB-C data cable.</li>
      <li>If the controller asks how to use USB, choose <b>File transfer</b>.</li></ol>`;
    if (send.status) h += `<div class="status ${send.cls}">${esc(send.status)}</div>`;
    h += `<div class="btns"><button class="btn" data-s="close">Cancel</button><button class="btn primary" data-s="find"${send.step === 'finding' ? ' disabled' : ''}>${send.step === 'finding' ? 'Looking…' : 'Find controller'}</button></div>`;
  } else if (send.step === 'pick' || send.step === 'sending') {
    h += `<h2>Choose the placeholder</h2><p class="note">Newest first. The mission you pick will be overwritten.</p><div class="mlist">`;
    send.missions.forEach((m, i) => {
      const known = sentLog()[m.folder];
      h += `<label class="mitem${send.pick === i ? ' on' : ''}"><input type="radio" name="pm" value="${i}"${send.pick === i ? ' checked' : ''}><div><b>${esc(known ? known.name : m.date)}</b><small>${known ? 'Sent from this app ' + esc(new Date(known.at).toLocaleString([], { month: 'short', day: 'numeric', hour: 'numeric', minute: '2-digit' })) + ' · ' + esc(m.date) + ' · ' : ''}${esc(m.folder.slice(0, 8))}…</small></div></label>`;
    });
    h += '</div>';
    if (send.status) h += `<div class="status ${send.cls}">${esc(send.status)}</div>`;
    h += `<div class="btns"><button class="btn" data-s="close">Cancel</button><button class="btn primary" data-s="go"${send.pick == null || send.step === 'sending' ? ' disabled' : ''}>${send.step === 'sending' ? 'Sending…' : 'Replace with this mission'}</button></div>`;
  } else if (send.step === 'done') {
    h += `<h2>Mission sent</h2><div class="status ok">${esc(send.status)}</div>
      <ol>${native && native.isOnController && native.isOnController() ? '' : '<li>Unplug the cable.</li>'}<li>Open DJI Fly → Waypoints and open the placeholder mission (it may still show the old preview image).</li>
      <li>Check the route on the map: straight, parallel lines inside the boundary.</li></ol>
      ${preflightChecklist()}
      <div class="btns"><button class="btn primary" data-s="close">Done</button></div>`;
  }
  sheet.innerHTML = h + '</div>';
}
sheet.addEventListener('click', e => {
  if (phSheetClick(e)) return;
  if (sheetMode === 'offline') { offlineClick(e); return; }
  if (sheetMode === 'lib') { libClick(e); return; }
  if (sheetMode === 'goal') { goalClick(e); return; }
  if (sheetMode === 'maps') { mapsClick(e); return; }
  if (sheetMode === 'marker') { markerClick(e); return; }
  if (sheetMode === 'markers') { markerListClick(e); return; }
  if (sheetMode === 'shape') { shapeClick(e); return; }
  if (sheetMode === 'hfind') { hfClick(e); return; }
  if (sheetMode === 'sync') { syncClick(e); return; }
  if (e.target === sheet) { if (send.step !== 'sending') sheet.hidden = true; return; }
  const r = e.target.closest('input[name=pm]');
  if (r) { send.pick = Number(r.value); renderSheet(); return; }
  const b = e.target.closest('[data-s]'); if (!b || b.disabled) return;
  if (b.dataset.s === 'close') sheet.hidden = true;
  else if (b.dataset.s === 'hzgo') { const p = missionPoints().hazard.problems[+b.dataset.i]; if (p) { sheet.hidden = true; map.setView(p.at, Math.max(map.getZoom(), 17)); } }
  else if (b.dataset.s === 'airgo') { const it = airBlocking()[+b.dataset.i]; if (it) { sheet.hidden = true; map.setView(it.at, Math.max(map.getZoom(), 14)); } }
  else if (b.dataset.s === 'airreq') openUrl(AIR_LINKS.aloft);
  else if (b.dataset.s === 'airtfr') openUrl(AIR_LINKS.tfr);
  else if (b.dataset.s === 'hzok') { airAck = air.key; if (send.after === 'save') { sheet.hidden = true; doSaveKmz(); } else { send.step = 'intro'; renderSheet(); } }
  else if (b.dataset.s === 'grant') native.requestFolderAccess();
  else if (b.dataset.s === 'find') {
    if (!native || !native.findController) { send.status = PC ? 'Sending to the controller works from the tablet app. Save the mission in Missions; sync brings it to the tablet, then send it from there. Or use Save KMZ and copy the file yourself.' : 'Sending to the controller works in the Android app.'; send.cls = 'bad'; return renderSheet(); }
    send.step = 'finding'; send.status = ''; renderSheet(); native.findController();
  } else if (b.dataset.s === 'go') {
    let k; try { k = buildKmz(); } catch (err) { send.status = err.message; send.cls = 'bad'; return renderSheet(); }
    send.step = 'sending'; send.status = 'Backing up the placeholder and writing the new mission…'; send.cls = ''; renderSheet();
    send.sentFolder = send.missions[send.pick].folder; send.sentName = missionTitle();
    let pv = ''; try { pv = previewJpeg(); } catch (err) {}
    native.sendMission(send.missions[send.pick].folder, fileName(), k.template, k.waylines, pv);
  }
});

window.onNative = function (json) {
  let m; try { m = JSON.parse(json); } catch (e) { return; }
  if (/^off/.test(m.type)) { offlineMsg(m); return; }
  if (m.type === 'imported') { if (m.purpose === 'markers') importMarkers(m); else importMission(m); return; }
  if (m.type === 'places') { showPlaces(m); return; }
  if (m.type === 'maps' || m.type === 'mbProgress' || m.type === 'mbError') { mapsMsg(m); return; }
  if (m.type === 'usgsZoom') { setUsgsMax(m.zoom); return; }
  if (/^sync/.test(m.type) || m.type === 'resume') { syncMsg(m); return; }
  if (m.type === 'location') gotLocation(m.lat, m.lng);
  else if (m.type === 'toast') toast(m.message);
  else if (m.type === 'missions') {
    send.missions = m.missions; send.pick = m.missions.length ? 0 : null;
    if (!m.missions.length) { send.step = 'intro'; send.status = 'Connected to ' + m.device + ', but no saved waypoint missions were found. Save a placeholder mission in DJI Fly first.'; send.cls = 'bad'; }
    else { send.step = 'pick'; send.status = ''; }
    renderSheet();
  } else if (m.type === 'sent') {
    if (m.droneEnum != null) { S.droneEnum = m.droneEnum; S.droneSub = m.droneSub; save(); }
    if (send.sentFolder) { const log = sentLog(); log[send.sentFolder] = { name: send.sentName || S.name, at: Date.now() }; try { localStorage.setItem('planner.sent', JSON.stringify(log)); } catch (e) {} }
    send.step = 'done'; send.status = m.message; send.cls = 'ok'; renderSheet();
  } else if (m.type === 'error') {
    send.step = send.missions.length ? 'pick' : 'intro'; send.status = m.message; send.cls = 'bad';
    if (!sheet.hidden) renderSheet(); else toast(m.message);
  } else if (m.type === 'needAccess') {
    send.step = 'access'; send.status = ''; renderSheet();
  } else if (m.type === 'usbAttached') {
    if (sheet.hidden && S.wps.length >= 2) { openSend(); }
  }
};

/* ---------- offline maps ---------- */
let sheetMode = 'send';
const OFF_MINZ = 8, OFF_MAX_TILES = 60000;

const offLayer = L.layerGroup().addTo(map);
let off = { areas: [], bytes: 0, dl: null, name: '', maxZ: 18, save: null, status: '', cls: '', armed: '' };
function tileRange(b, z) {
  const n = 2 ** z, cl = v => Math.max(0, Math.min(n - 1, v));
  const lx = lng => cl(Math.floor((lng + 180) / 360 * n));
  const ly = lat => { const r = Math.max(-85.0511, Math.min(85.0511, lat)) * Math.PI / 180; return cl(Math.floor((1 - Math.log(Math.tan(r) + 1 / Math.cos(r)) / Math.PI) / 2 * n)); };
  return { x1: lx(b.w), x2: lx(b.e), y1: ly(b.n), y2: ly(b.s) };
}
function tileCount(b, minZ, maxZ) { let c = 0; for (let z = minZ; z <= maxZ; z++) { const t = tileRange(b, z); c += (t.x2 - t.x1 + 1) * (t.y2 - t.y1 + 1); } return c; }
function offBounds() { const b = map.getBounds(); return { s: b.getSouth(), w: b.getWest(), n: b.getNorth(), e: b.getEast() }; }
function offEstimate() {
  const b = offBounds(); let tiles = 0, bytes = 0;
  Object.entries(BASES).forEach(([k, v]) => {
    if (!off.save[k] || v.online) return;
    const per = tileCount(b, OFF_MINZ, Math.min(off.maxZ, v.max));
    tiles += per * v.off.length; bytes += per * v.bytes;
  });
  if (off.terrain !== false) { const per = tileCount(b, OFF_MINZ, Math.min(off.maxZ, DEM_Z)); tiles += per; bytes += per * 70000; }
  return { tiles, bytes };
}
function offLayers() { return Object.entries(BASES).filter(([k, v]) => off.save[k] && !v.online).reduce((a, [, v]) => a.concat(v.off), []).concat(off.terrain !== false ? ['terrain:terrarium'] : []); }
function areaSources(a) { return Object.values(BASES).filter(v => v.off.every(l => (a.layers || []).includes(l))).map(v => v.label).concat((a.layers || []).includes('terrain:terrarium') ? ['Terrain'] : []).join(', '); }
function fmtBytes(n) { return n >= 1e9 ? r1(n / 1e9) + ' GB' : n >= 1e6 ? r0(n / 1e6) + ' MB' : Math.max(1, r0(n / 1e3)) + ' KB'; }
function areaSize(a) { return imp() ? r1(hav([a.s, a.w], [a.s, a.e]) * FT / 5280) + ' × ' + r1(hav([a.s, a.w], [a.n, a.w]) * FT / 5280) + ' mi' : r1(hav([a.s, a.w], [a.s, a.e]) / 1000) + ' × ' + r1(hav([a.s, a.w], [a.n, a.w]) / 1000) + ' km'; }
function renderOffAreas() {
  offLayer.clearLayers();
  off.areas.forEach(a => L.rectangle([[a.s, a.w], [a.n, a.e]], { color: '#ffffff', weight: 1.5, opacity: 0.7, dashArray: '4 6', fill: false, interactive: false }).addTo(offLayer));
}
function openOffline() {
  sheetMode = 'offline'; off.armed = '';
  if (!off.dl) { off.status = ''; off.cls = ''; }
  if (!off.name) off.name = 'Area ' + (off.areas.length + 1);
  if (!off.save) { off.save = { usgs: true }; }
  renderOffline(); sheet.hidden = false;
  if (native && native.listOfflineAreas) native.listOfflineAreas();
}
function renderOffline() {
  const online = navigator.onLine;
  let h = `<div class="card"><h2>Offline maps</h2>
    <p class="note">Saves the map for the area on screen so you can plan with no internet, like out in the field. Search for your flying site or zoom the map to it first. Areas you have scrolled around are also kept automatically.</p>
    <p class="note">Internet: <b style="color:var(--${online ? 'ok' : 'bad'})">${online ? 'connected' : 'offline'}</b></p>`;
  if (off.dl) {
    const pct = off.dl.total ? Math.floor(100 * off.dl.done / off.dl.total) : 0;
    h += `<div class="status">Downloading ${esc(off.dl.name)}… ${pct}%\n${off.dl.done.toLocaleString()} of ${off.dl.total.toLocaleString()} tiles · ${fmtBytes(off.dl.bytes)}${off.dl.failed ? ` · ${off.dl.failed} failed` : ''}</div>
      <div class="bar"><i style="width:${pct}%"></i></div>
      <div class="btns"><button class="btn" data-o="close">Hide</button><button class="btn danger" data-o="cancel">Stop download</button></div>`;
  } else {
    const est = offEstimate(), big = est.tiles > OFF_MAX_TILES, none = !est.tiles;
    h += `<label class="f"><span>Name</span><div class="in"><input type="text" id="offname" data-o="name" value="${esc(off.name)}"></div></label>
      <div class="row"><label class="f"><span>Sharpest detail</span><div class="in"><select id="offzoom" data-o="zoom">
        ${[[17, 'Good (zoom 17)'], [18, 'Sharp (zoom 18)'], [19, 'Sharpest (zoom 19)']].map(([z, t]) => `<option value="${z}"${off.maxZ === z ? ' selected' : ''}>${t}</option>`).join('')}
      </select></div></label><div></div></div>
      <span class="note" style="display:block;margin:0 0 4px">Map types to save</span>
      ${Object.entries(BASES).filter(([, v]) => !v.online).map(([k, v]) => `<label class="chk"><input type="checkbox" data-base="${k}"${off.save[k] ? ' checked' : ''}>${v.label}${v.max < off.maxZ ? ` <span class="note" style="margin:0">(up to zoom ${v.max})</span>` : ''}</label>`).join('')}
      <p class="note" style="margin-top:-2px">Esri satellite and street maps work online only; Esri's terms don't allow saving them for offline use. USGS aerial imagery (US only) is public domain. For sharper offline imagery, add your own orthophoto under My maps.</p>
      <label class="chk"><input type="checkbox" id="offterrain"${off.terrain !== false ? ' checked' : ''}>Terrain heights <span class="note" style="margin:0">(for terrain following)</span></label>
      <div class="calc"><span>Area on screen</span><b>${areaSize(offBounds())}</b><span>Tiles</span><b>${est.tiles.toLocaleString()}</b><span>Estimated size</span><b>~${fmtBytes(est.bytes)}</b></div>
      ${big ? `<div class="status bad">That area is too large to save at this detail. Zoom the map in or pick a lower detail level.</div>` : ''}
      ${off.status ? `<div class="status ${off.cls}">${esc(off.status)}</div>` : ''}
      <div class="btns"><button class="btn" data-o="close">Close</button><button class="btn primary" data-o="download"${big || none || !online ? ' disabled' : ''}>Download this area</button></div>`;
  }
  h += `<h3 class="subh">Saved areas</h3>`;
  if (!native || !native.downloadOfflineArea) h += `<p class="note">${DESKTOP ? 'Offline maps are for the tablet. This PC loads maps from the internet.' : 'Saved areas are managed in the Android app.'}</p>`;
  else if (!off.areas.length) h += `<p class="note">None yet.</p>`;
  else {
    h += '<div class="mlist">' + off.areas.slice().reverse().map(a => `<div class="mitem"><div style="flex:1;min-width:0"><b>${esc(a.name)}</b><small>${esc(a.date)} · ${areaSize(a)} · ${esc(areaSources(a))} · zoom ${a.maxZ} · ${Number(a.tiles).toLocaleString()} tiles</small></div>
      <button class="btn small" data-o="show" data-id="${esc(a.id)}">Show</button>
      <button class="btn small danger" data-o="del" data-id="${esc(a.id)}">${off.armed === a.id ? 'Confirm' : 'Delete'}</button></div>`).join('') + '</div>';
  }
  if (native && native.downloadOfflineArea) h += `<div class="btns" style="align-items:center"><p class="note" style="flex:1;margin:0">Map storage used: <b>${fmtBytes(off.bytes)}</b></p>
    <button class="btn small danger" data-o="delall"${off.dl ? ' disabled' : ''}>${off.armed === '*' ? 'Tap again to delete all' : 'Delete all offline maps'}</button></div>`;
  sheet.innerHTML = h + '</div>';
}
function offlineClick(e) {
  if (e.target === sheet) { sheet.hidden = true; return; }
  const b = e.target.closest('button[data-o]'); if (!b || b.disabled) return;
  const o = b.dataset.o;
  if (o === 'close') sheet.hidden = true;
  else if (o === 'cancel') { if (native && native.cancelOfflineDownload) native.cancelOfflineDownload(); }
  else if (o === 'download') {
    if (!native || !native.downloadOfflineArea) { off.status = DESKTOP ? 'Downloading offline maps works in the tablet app.' : 'Downloading maps works in the Android app.'; off.cls = 'bad'; return renderOffline(); }
    const b4 = offBounds(), est = offEstimate();
    off.dl = { name: off.name || 'Area', done: 0, total: est.tiles, failed: 0, bytes: 0 };
    native.downloadOfflineArea(JSON.stringify(Object.assign({ name: off.dl.name, minZ: OFF_MINZ, maxZ: off.maxZ, layers: offLayers() }, b4)));
    renderOffline();
  } else if (o === 'show') {
    const a = off.areas.find(x => x.id === b.dataset.id);
    if (a) { sheet.hidden = true; map.fitBounds([[a.s, a.w], [a.n, a.e]]); }
  } else if (o === 'del') {
    if (off.armed === b.dataset.id) { off.armed = ''; native.deleteOfflineArea(b.dataset.id); }
    else { off.armed = b.dataset.id; renderOffline(); }
  } else if (o === 'delall') {
    if (off.armed === '*') { off.armed = ''; native.deleteAllOffline(); }
    else { off.armed = '*'; renderOffline(); }
  }
}
sheet.addEventListener('change', e => {
  if (sheetMode === 'lib') { const t = e.target; if (t.id === 'libname') { lib.name = t.value.trim(); } return; }
  if (sheetMode === 'goal') { goalChange(e.target); return; }
  if (sheetMode === 'maps') return;
  if (sheetMode === 'marker') { markerChange(e.target); return; }
  if (sheetMode === 'markers') { markerListChange(e.target); return; }
  if (sheetMode === 'shape') { shapeChange(e.target); return; }
  if (sheetMode !== 'offline') return;
  const t = e.target;
  if (t.id === 'offname') off.name = t.value.trim();
  else if (t.id === 'offzoom') off.maxZ = Number(t.value);
  else if (t.dataset.base) off.save[t.dataset.base] = t.checked;
  else if (t.id === 'offterrain') off.terrain = t.checked;
  renderOffline();
});
function offlineMsg(m) {
  if (m.type === 'offAreas') {
    off.areas = m.areas || []; off.bytes = m.bytes || 0;
    if (!m.running && off.dl && off.dl.finished) off.dl = null;
    renderOffAreas();
  } else if (m.type === 'offProgress') {
    if (off.doneAt && Date.now() - off.doneAt < 2000) return;
    if (!off.dl) off.dl = { name: 'area', done: 0, total: 0, failed: 0, bytes: 0 };
    Object.assign(off.dl, { done: m.done, total: m.total, failed: m.failed, bytes: m.bytes });
  } else if (m.type === 'offDone') {
    off.dl = null; off.doneAt = Date.now(); off.status = m.message; off.cls = m.cls || ''; off.name = '';
    if (sheet.hidden || sheetMode !== 'offline') toast(m.message);
    off.name = 'Area ' + (off.areas.length + 2);
  }
  if (!sheet.hidden && sheetMode === 'offline') renderOffline();
}
window.addEventListener('online', () => { if (!sheet.hidden && sheetMode === 'offline') renderOffline(); });
window.addEventListener('offline', () => { if (!sheet.hidden && sheetMode === 'offline') renderOffline(); });

/* ---------- saved missions ---------- */
const LIB_KEY = 'planner.lib.v1';
const SNAP_KEYS = ['segs', 'seg', 'label', 'name', 'alt', 'speed', 'finish', 'rcLost', 'turn', 'defGimbal', 'camera', 'front', 'side', 'spacingAuto', 'spacing',
  'dir', 'angle', 'reverse', 'gridGimbal', 'runIn', 'pathStyle', 'photoSize', 'split', 'terrain', 'interval', 'poi', 'shape', 'poiCenter', 'mode', 'wps'];
let lib = { armed: '', name: '', status: '', cls: '' };
function libLoad() { try { return JSON.parse(localStorage.getItem(LIB_KEY)) || []; } catch (e) { return []; } }
function libSave(list) { syStamp('missions', list); try { localStorage.setItem(LIB_KEY, JSON.stringify(list)); return true; } catch (e) { return false; } }
function snapshot() { const o = {}; SNAP_KEYS.forEach(k => { o[k] = clone(S[k] === undefined ? null : S[k]); }); return o; }
function isDirty() {
  if (!S.wps.length && !S.shape) return false;
  const e = libLoad().find(m => m.id === S.libId);
  return !e || JSON.stringify(e.data) !== JSON.stringify(snapshot());
}
const MODE_LABEL = { grid: 'Survey', poi: 'POI orbit', waypoint: 'Waypoints' };
/* ---------- plan by goal ----------
   Works backward from the detail you want and the batteries you have. Every candidate is run through the same grid,
   split and timing code the app uses for the real mission, so the numbers match what you will see after Apply. */
const QUALITY = {
  max: { label: 'Max (80/75)', front: 80, side: 75 },
  high: { label: 'High (75/70)', front: 75, side: 70 },
  standard: { label: 'Standard (70/65)', front: 70, side: 65 },
  light: { label: 'Light (65/60)', front: 65, side: 60 },
};
const QORDER = ['max', 'high', 'standard', 'light'];
const MAX_SPEED = 15, MIN_ALT = 30;
let goalRes = null;
function pxWidth(size) { return size === '12' ? 4032 : (CAMERAS[S.camera] || CAMERAS.wide).px; }
/* Settings that reach a detail level: altitude from detail, speed from camera interval, blur and the speed cap. */
function goalSettings(gsd, quality, size) {
  const g = S.goal, cam = CAMERAS[S.camera] || CAMERAS.wide, q = QUALITY[quality];
  let alt = gsd * pxWidth(size) * cam.f / FRAME_W, capped = false;
  if (alt > MAX_ALT) { alt = MAX_ALT; capped = true; }
  const realGsd = alt * FRAME_W / cam.f / pxWidth(size);
  const shot = alt * FRAME_H / cam.f * (1 - q.front / 100);
  const vInt = shot / Math.max(0.5, g.interval), vBlur = 0.5 * realGsd * g.shutter;
  const speed = Math.min(vInt, vBlur, MAX_SPEED);
  return { alt, capped, gsd: realGsd, front: q.front, side: q.side, quality, size, speed,
    limit: speed === MAX_SPEED ? 'top speed' : speed === vBlur ? 'motion blur' : 'camera interval', interval: shot / speed };
}
/* Run a candidate through the real planner, then put everything back. */
function goalEval(c) {
  const saved = JSON.stringify(S);
  try {
    Object.assign(S, { alt: c.alt, front: c.front, side: c.side, spacingAuto: true, speed: c.speed, photoSize: c.size });
    S.interval.on = true;
    if (S.interval.method === 'camera') { S.interval.auto = false; S.interval.type = 'time'; S.interval.value = Math.round(c.interval * 10) / 10; }
    else S.interval.auto = true;
    S.terrain.on = false;
    S.split.on = true; S.split.minutes = S.goal.minutes; S.split.sel = -1;
    regen(); mpCache = null;
    const parts = gridInfo ? gridInfo.parts.length : 1;
    S.split.on = false; regen(); mpCache = null;
    const st = stats();
    return Object.assign({}, c, { time: st.time, photos: st.photos, points: st.points, dist: st.dist, parts, fits: parts <= S.goal.batteries });
  } finally {
    S = JSON.parse(saved); regen(); mpCache = null;
  }
}
function goalSolve() {
  const g = S.goal, out = [];
  const base = goalEval(goalSettings(g.gsd, g.quality, S.photoSize));
  base.kind = 'Your goal'; out.push(base);
  if (!base.fits) {
    // 1. Coarser detail, same quality: find the finest detail that fits (detail is limited by the 400 ft cap).
    const top = goalEval(goalSettings(1, g.quality, S.photoSize));
    if (top.fits) {
      let lo = g.gsd, hi = top.gsd, best = top;
      for (let k = 0; k < 9; k++) {
        const mid = (lo + hi) / 2, r = goalEval(goalSettings(mid, g.quality, S.photoSize));
        if (r.fits) { hi = mid; best = r; } else lo = mid;
      }
      best.kind = 'Fits with less detail'; out.push(best);
    }
    // 2. Same detail, lighter overlap.
    for (const q of QORDER.slice(QORDER.indexOf(g.quality) + 1)) {
      const r = goalEval(goalSettings(g.gsd, q, S.photoSize));
      if (r.fits) { r.kind = 'Fits with less overlap'; out.push(r); break; }
    }
  }
  // 3. Same detail from higher up with 50 MP photos, when that saves batteries.
  if (S.photoSize === '12') {
    const r = goalEval(goalSettings(g.gsd, g.quality, 'full'));
    if (r.parts < base.parts) { r.kind = 'Same detail with 50 MP photos'; out.push(r); }
  }
  if (!out.some(r => r.fits)) {
    const r = goalEval(goalSettings(1, 'light', 'full'));
    out.minParts = r.parts;
  }
  return out;
}
function fmtGsd(m) { return imp() ? (Math.round(m / 0.0254 * 100) / 100) + ' in' : (Math.round(m * 1000) / 10) + ' cm'; }
function openGoal() {
  if (S.mode !== 'grid') return toast('Draw or import a survey area first');
  sheetMode = 'goal'; goalRes = null; renderGoal(); sheet.hidden = false;
}
function renderGoal() {
  const g = S.goal, unit = imp() ? 'in' : 'cm', gv = imp() ? Math.round(g.gsd / 0.0254 * 100) / 100 : Math.round(g.gsd * 1000) / 10;
  let h = `<div class="card"><h2>Plan by goal</h2>
    <p class="note">Set the detail you want and what you're bringing. The app works out altitude, overlap, speed and how to split the area.</p>
    <div class="row"><label class="f"><span>Ground detail per pixel</span><div class="in"><input type="number" inputmode="decimal" step="any" id="g_gsd" value="${gv}"><em>${unit}</em></div></label>
    <label class="f"><span>Photo size</span><div class="in"><select id="g_size"><option value="12"${S.photoSize === '12' ? ' selected' : ''}>12 MP</option><option value="full"${S.photoSize !== '12' ? ' selected' : ''}>50 MP</option></select></div></label></div>
    <div class="row"><label class="f"><span>Batteries</span><div class="in"><input type="number" inputmode="numeric" id="g_bat" value="${g.batteries}"></div></label>
    <label class="f"><span>Mission minutes per battery</span><div class="in"><input type="number" inputmode="numeric" id="g_min" value="${g.minutes}"><em>min</em></div></label></div>
    <div class="row"><label class="f"><span>Map quality (overlap)</span><div class="in"><select id="g_q">${QORDER.map(k => `<option value="${k}"${g.quality === k ? ' selected' : ''}>${QUALITY[k].label}</option>`).join('')}</select></div></label>
    <label class="f"><span>Shortest photo interval</span><div class="in"><input type="number" inputmode="decimal" step="any" id="g_int" value="${g.interval}"><em>s</em></div></label></div>
    <div class="row"><label class="f"><span>Shutter speed</span><div class="in"><em style="padding:0 0 0 10px">1/</em><input type="number" inputmode="numeric" id="g_sh" value="${g.shutter}"><em>s</em></div></label><div></div></div>
    <p class="note">1.6 in shows trails, fences and tanks. 0.8 in shows individual fence posts and brush. 0.5 in or finer is for small details. Use High or Max overlap over thick brush and trees.</p>
    <div class="btns"><button class="btn" data-g="close">Close</button><button class="btn primary" data-g="solve">Work it out</button></div>`;
  h += '<div id="goalres">';
  if (goalRes) {
    h += `<h3 class="subh">Options</h3><div class="mlist">`;
    goalRes.forEach((r, i) => {
      h += `<div class="mitem${r.fits ? ' on' : ''}" style="align-items:flex-start"><div style="flex:1;min-width:0">
        <b>${r.kind}${r.fits ? '' : ' · needs ' + r.parts + ' batteries'}</b>
        <small>${fmtGsd(r.gsd)} detail${r.capped ? ' (400 ft limit)' : ''} · ${r.size === '12' ? '12 MP' : '50 MP'} · ${QUALITY[r.quality].label}</small>
        <small>${fmtLen(r.alt)} · ${r1(toSpd(r.speed))} ${SPD()} (set by ${r.limit}) · photo every ${r1(r.interval)} s</small>
        <small>${fmtTime(r.time)} flying · ${r.parts} part${r.parts > 1 ? 's' : ''} · ~${r.photos.toLocaleString()} photos · ${fmtDist(r.dist)}</small>
        ${r.alt < MIN_ALT ? '<small style="color:var(--bad)">Very low. Check for trees, towers and power lines.</small>' : ''}</div>
        <button class="btn small${r.fits ? ' primary' : ''}" data-g="apply" data-i="${i}">Apply</button></div>`;
    });
    h += '</div>';
    if (goalRes.minParts) h += `<div class="status bad">This area needs at least ${goalRes.minParts} batteries even at 400 ft with light overlap. Bring more batteries, allow more minutes per battery, or split the area across days.</div>`;
  }
  sheet.innerHTML = h + '</div></div>';
}
function goalChange(t) {
  const g = S.goal, v = Number(t.value);
  if (t.id === 'g_gsd' && v > 0) g.gsd = imp() ? v * 0.0254 : v / 100;
  else if (t.id === 'g_size') S.photoSize = t.value;
  else if (t.id === 'g_bat' && v >= 1) g.batteries = Math.round(v);
  else if (t.id === 'g_min' && v >= 3) g.minutes = Math.min(40, Math.round(v));
  else if (t.id === 'g_q') g.quality = t.value;
  else if (t.id === 'g_int' && v > 0) g.interval = Math.max(0.5, v);
  else if (t.id === 'g_sh' && v >= 100) g.shutter = Math.round(v);
  // Inputs change on blur, often while a button is being tapped, so don't redraw the form here; just clear old results.
  goalRes = null; save();
  const r = document.getElementById('goalres'); if (r) r.innerHTML = '';
}
function goalClick(e) {
  if (e.target === sheet) { sheet.hidden = true; return; }
  const b = e.target.closest('button[data-g]'); if (!b) return;
  if (b.dataset.g === 'close') { sheet.hidden = true; commit(false); }
  else if (b.dataset.g === 'solve') { goalRes = goalSolve(); renderGoal(); }
  else if (b.dataset.g === 'apply') {
    const r = goalRes[+b.dataset.i];
    Object.assign(S, { alt: r.alt, front: r.front, side: r.side, spacingAuto: true, speed: r.speed, photoSize: r.size });
    S.goal.quality = r.quality;
    S.interval.on = true;
    if (S.interval.method === 'camera') { S.interval.auto = false; S.interval.type = 'time'; S.interval.value = Math.round(r.interval * 10) / 10; }
    else S.interval.auto = true;
    S.split.on = true; S.split.minutes = S.goal.minutes; S.split.sel = 0;
    regen(); commit(false); fitMission(); sheet.hidden = true;
    toast('Applied: ' + fmtGsd(r.gsd) + ' detail, ' + r.parts + ' part' + (r.parts > 1 ? 's' : ''));
  }
}

function openLib() {
  sheetMode = 'lib'; lib.armed = ''; lib.status = ''; lib.cls = ''; lib.name = S.name;
  renderLib(); sheet.hidden = false;
}
function renderLib() {
  const list = libLoad().sort((a, b) => b.saved - a.saved);
  const cur = list.find(m => m.id === S.libId), has = S.wps.length >= 1 || !!S.shape, dirty = isDirty();
  const fmt = t => new Date(t).toLocaleString([], { month: 'short', day: 'numeric', year: 'numeric', hour: 'numeric', minute: '2-digit' });
  let h = `<div class="card"><h2>Missions</h2>`;
  if (has) {
    h += `<label class="f"><span>Name</span><div class="in"><input type="text" id="libname" value="${esc(lib.name)}"></div></label>
      <p class="note">${cur ? (dirty ? 'Unsaved changes since ' + fmt(cur.saved) + '.' : 'Saved ' + fmt(cur.saved) + '.') : 'This mission has not been saved yet.'}</p>
      <div class="btns">${cur ? `<button class="btn primary" data-l="save">Save</button><button class="btn" data-l="saveas">Save as new</button>` : `<button class="btn primary" data-l="saveas">Save mission</button>`}</div>`;
  } else h += `<p class="note">Draw a mission, then come back here to save it.</p>`;
  if (lib.status) h += `<div class="status ${lib.cls}" style="margin-top:12px">${esc(lib.status)}</div>`;
  h += `<div class="btns" style="margin-top:12px"><button class="btn" data-l="import">Import KMZ / KML…</button><button class="btn" data-l="new">${lib.armed === 'new' ? 'Discard changes?' : 'New mission'}</button></div>`;
  h += `<h3 class="subh">Saved missions · ${list.length}</h3>`;
  if (!list.length) h += `<p class="note">None yet.</p>`;
  else h += '<div class="mlist">' + list.map(m => {
    const n = m.data.mode === 'waypoint' ? m.data.wps.length + ' waypoints' : MODE_LABEL[m.data.mode];
    const open = lib.armed === 'open:' + m.id;
    return `<div class="mitem${m.id === S.libId ? ' on' : ''}"><div style="flex:1;min-width:0"><b>${esc(m.data.name)}</b><small>${MODE_LABEL[m.data.mode] || ''}${m.data.mode === 'waypoint' ? ' · ' + n : ''} · ${fmt(m.saved)}</small></div>
      <button class="btn small${open ? ' danger' : ''}" data-l="open" data-id="${m.id}">${open ? 'Discard current & open' : 'Open'}</button>
      <button class="btn small" data-l="dup" data-id="${m.id}">Copy</button>
      <button class="btn small danger" data-l="del" data-id="${m.id}">${lib.armed === 'del:' + m.id ? 'Confirm' : 'Delete'}</button></div>`;
  }).join('') + '</div>';
  h += `<div class="btns" style="margin-top:4px"><button class="btn" data-l="close">Close</button></div>`;
  sheet.innerHTML = h + '</div>';
}
function libClick(e) {
  if (e.target === sheet) { sheet.hidden = true; return; }
  const b = e.target.closest('button[data-l]'); if (!b || b.disabled) return;
  const act = b.dataset.l, id = b.dataset.id;
  let list = libLoad();
  const ok = msg => { lib.status = msg; lib.cls = 'ok'; lib.armed = ''; };
  if (act === 'close') { sheet.hidden = true; return; }
  if (act === 'save' || act === 'saveas') {
    if (lib.name) S.name = lib.name;
    const data = snapshot(), now = Date.now();
    const e2 = act === 'save' && list.find(m => m.id === S.libId);
    if (e2) { e2.data = data; e2.saved = now; }
    else { S.libId = 'm' + now; list.push({ id: S.libId, saved: now, data }); }
    if (libSave(list)) ok('Saved “' + S.name + '”.'); else { lib.status = 'Could not save: storage is full.'; lib.cls = 'bad'; }
    commit(false);
  } else if (act === 'open' || act === 'new') {
    const key = act === 'open' ? 'open:' + id : 'new';
    if (isDirty() && lib.armed !== key) { lib.armed = key; lib.status = 'The current mission has unsaved changes.'; lib.cls = 'bad'; return renderLib(); }
    if (act === 'new') {
      S.segs = []; S.seg = 0; S.wps = []; S.shape = null; S.poiCenter = null; S.mode = 'waypoint'; S.sel = -1; S.libId = null; S.name = 'Mission ' + (list.length + 1);
      S.interval = clone(DEFAULTS.interval); S.split = clone(DEFAULTS.split); commit(false); sheet.hidden = true; toast('New mission'); return;
    }
    const m = list.find(x => x.id === id); if (!m) return;
    S.segs = []; S.seg = 0; S.label = null;
    Object.assign(S, clone(m.data)); S.libId = m.id; S.sel = -1;
    if (!Array.isArray(S.segs)) S.segs = [];
    if (!(S.seg >= 0 && S.seg < S.segs.length)) S.seg = 0;
    if (S.pathStyle !== 'straightx') S.pathStyle = 'anchors';
    if (S.interval.method === 'trigger') { S.interval.method = 'camera'; S.interval.auto = false; if (S.interval.type !== 'time') S.interval.value = 3; S.interval.type = 'time'; }
    if (S.mode !== 'waypoint') regen();
    commit(false); fitAll(); sheet.hidden = true; toast('Opened “' + S.name + '”' + (multi() ? ' · ' + S.segs.length + ' segments' : '')); return;
  } else if (act === 'dup') {
    const m = list.find(x => x.id === id); if (!m) return;
    const c = { id: 'm' + Date.now(), saved: Date.now(), data: clone(m.data) }; c.data.name = m.data.name + ' copy';
    list.push(c); libSave(list); ok('Copied as “' + c.data.name + '”.');
  } else if (act === 'del') {
    if (lib.armed !== 'del:' + id) { lib.armed = 'del:' + id; return renderLib(); }
    list = list.filter(x => x.id !== id); libSave(list); if (S.libId === id) S.libId = null; ok('Deleted.');
  } else if (act === 'import') {
    if (!native || !native.importKmz) { lib.status = 'Importing files works in the Android app.'; lib.cls = 'bad'; return renderLib(); }
    if (isDirty() && lib.armed !== 'import') { lib.armed = 'import'; lib.status = 'The current mission has unsaved changes. Tap Import again to replace it.'; lib.cls = 'bad'; return renderLib(); }
    lib.armed = ''; native.importKmz(); return;
  }
  renderLib();
}
function fitMission() { const b = routeBounds(); if (b) map.fitBounds(b, { padding: [40, 40], maxZoom: 19 }); }
function fitAll() { const pts = []; (multi() ? S.segs : [S]).forEach(sg => (sg.wps || []).forEach(w => pts.push([w.lat, w.lng]))); if (pts.length) map.fitBounds(L.latLngBounds(pts), { padding: [40, 40], maxZoom: 19 }); }

/* ---------- import ---------- */
function importMission(m) {
  try {
    S.segs = []; S.seg = 0;
    if (!m.waylines && !m.template && m.kml) {
      const area = parseBoundary(m.kml);
      if (area) {
        S.name = area.name || (m.name || 'Imported area').replace(/\.(kmz|kml)$/i, '');
        S.libId = null; S.poiCenter = null; sheet.hidden = true;
        setShape({ type: 'polygon', pts: area.pts });
        toast('Imported survey area “' + S.name + '” · ' + (imp() ? r0(gridInfo.area / 4046.86) + ' acres' : r1(gridInfo.area / 10000) + ' ha'));
        return;
      }
    }
    const n = parseMissionFile(m.waylines || m.template || '', m.kml || '');
    if (!n.wps.length) throw new Error('No waypoints found in that file.');
    Object.assign(S, n.settings);
    S.wps = n.wps; S.mode = 'waypoint'; S.shape = null; S.sel = -1; S.libId = null;
    S.name = (m.name || 'Imported').replace(/\.(kmz|kml|wpml)$/i, '');
    commit(false); fitMission(); sheet.hidden = true;
    toast('Imported ' + n.wps.length + ' waypoints' + (n.note ? ' · ' + n.note : ''));
  } catch (e) {
    lib.status = 'Could not import: ' + e.message; lib.cls = 'bad';
    if (!sheet.hidden && sheetMode === 'lib') renderLib(); else toast(lib.status);
  }
}
window.__import = importMission;   // for testing
/* A Google Earth / Google My Maps polygon becomes a survey area (largest polygon if there are several). */
function parseBoundary(kml) {
  const doc = new DOMParser().parseFromString(kml, 'application/xml');
  if (doc.getElementsByTagName('parsererror').length) return null;
  let best = null;
  Array.from(doc.getElementsByTagNameNS('*', 'Polygon')).forEach(poly => {
    const outer = poly.getElementsByTagNameNS('*', 'outerBoundaryIs')[0] || poly;
    const c = outer.getElementsByTagNameNS('*', 'coordinates')[0]; if (!c) return;
    let pts = c.textContent.trim().split(/\s+/).map(t => t.split(',').map(Number)).filter(a => isFinite(a[0]) && isFinite(a[1])).map(a => [a[1], a[0]]);
    if (pts.length > 3 && Math.abs(pts[0][0] - pts[pts.length - 1][0]) < 1e-9 && Math.abs(pts[0][1] - pts[pts.length - 1][1]) < 1e-9) pts.pop();
    if (pts.length < 3) return;
    const pr = projector(centroid(pts)), area = polyArea(pts.map(pr.xy));
    let pm = poly.parentNode; while (pm && pm.localName !== 'Placemark') pm = pm.parentNode;
    const nm = pm && Array.from(pm.childNodes).find(n => n.localName === 'name');
    if (!best || area > best.area) best = { pts, area, name: nm ? nm.textContent.trim() : '' };
  });
  return best;
}
function parseMissionFile(wpml, kml) {
  const doc = new DOMParser().parseFromString(wpml || kml, 'application/xml');
  if (doc.getElementsByTagName('parsererror').length) throw new Error('the file is not valid KML.');
  const kids = (el, name) => Array.from(el.childNodes).filter(c => c.nodeType === 1 && (c.localName === name));
  const one = (el, name) => { const k = kids(el, name); return k.length ? k[0] : null; };
  const deep = (el, name) => el.getElementsByTagNameNS('*', name)[0] || null;
  const txt = (el, name) => { const x = el && one(el, name); return x ? x.textContent.trim() : null; };
  const num = (el, name) => { const t = txt(el, name); return t == null || t === '' ? null : Number(t); };
  const settings = {};
  const mc = deep(doc, 'missionConfig');
  if (mc) {
    const fa = txt(mc, 'finishAction'); if (fa && ['goHome', 'noAction', 'autoLand', 'gotoFirstWaypoint'].includes(fa)) settings.finish = fa;
    const rl = txt(mc, 'executeRCLostAction'); if (rl) settings.rcLost = rl === 'hover' ? 'hover' : 'goBack';
  }
  const placemarks = Array.from(doc.getElementsByTagNameNS('*', 'Placemark'));
  const isWpml = placemarks.some(p => one(p, 'index'));
  const wps = []; let note = '';
  if (isWpml) {
    const folder = deep(doc, 'Folder');
    const fs = folder && (num(folder, 'autoFlightSpeed'));
    const gh = folder && num(folder, 'globalHeight');
    if (fs) settings.speed = fs;
    const sorted = placemarks.map(p => ({ p, i: num(p, 'index') })).sort((a, b) => a.i - b.i);
    const gimbalAt = {}; let interval = null;
    sorted.forEach(({ p, i }) => {
      const c = (deep(p, 'coordinates') || {}).textContent || '';
      const [lng, lat] = c.trim().split(/[,\s]+/).map(Number);
      if (!isFinite(lat) || !isFinite(lng)) return;
      const h = num(p, 'executeHeight') ?? num(p, 'height') ?? num(p, 'ellipsoidHeight') ?? gh;
      const w = newWp([lat, lng], { alt: h, speed: num(p, 'waypointSpeed') });
      const hp = one(p, 'waypointHeadingParam');
      if (hp) {
        const mode = txt(hp, 'waypointHeadingMode'), en = num(hp, 'waypointHeadingAngleEnable'), ang = num(hp, 'waypointHeadingAngle');
        if (mode === 'towardPOI') {
          const pp = (txt(hp, 'waypointPoiPoint') || '').split(',').map(Number);
          if (pp.length >= 2 && (pp[0] || pp[1])) { settings.poiCenter = [pp[0], pp[1]]; w.heading = 'poi'; }
        } else if (mode === 'fixed' || (en === 1 && i > 0 && ang != null)) { w.heading = 'fixed'; w.hdg = ((Math.round(ang) % 360) + 360) % 360; }
      }
      kids(p, 'actionGroup').forEach(g => {
        const start = num(g, 'actionGroupStartIndex'), end = num(g, 'actionGroupEndIndex');
        const tr = one(g, 'actionTrigger'), ttype = txt(tr, 'actionTriggerType');
        kids(g, 'action').forEach(a => {
          const f = txt(a, 'actionActuatorFunc'), par = one(a, 'actionActuatorFuncParam');
          if (ttype === 'multipleTiming' || ttype === 'multipleDistance') {
            if (f === 'takePhoto') interval = { type: ttype === 'multipleTiming' ? 'time' : 'distance', value: num(tr, 'actionTriggerParam'), from: start, to: end };
            return;
          }
          if (f === 'takePhoto') w.photo = true;
          else if (f === 'startRecord') w.rec = 'start';
          else if (f === 'stopRecord') w.rec = 'stop';
          else if (f === 'hover') w.hover = num(par, 'hoverTime') || 0;
          else if (f === 'gimbalRotate' && num(par, 'gimbalPitchRotateEnable') !== 0) gimbalAt[start] = num(par, 'gimbalPitchRotateAngle');
          else if (f === 'gimbalEvenlyRotate') gimbalAt[end] = num(par, 'gimbalPitchRotateAngle');
        });
      });
      w._i = i; wps.push(w);
    });
    // gimbal: store only where it changes
    let prev = null;
    wps.forEach(w => {
      const g = gimbalAt[w._i];
      if (g != null && isFinite(g) && g !== prev) { w.gimbal = Math.max(-90, Math.min(0, Math.round(g))); prev = w.gimbal; }
      delete w._i;
    });
    if (wps.length && wps[0].gimbal == null) wps[0].gimbal = S.defGimbal;
    if (interval && interval.value) {
      settings.interval = Object.assign(clone(S.interval), { on: true, auto: false, method: 'camera', type: 'time', value: interval.type === 'time' ? interval.value : 3, from: interval.from || 0, to: interval.to >= wps.length - 1 ? -1 : interval.to });
    } else settings.interval = Object.assign(clone(S.interval), { on: false, auto: false });
  } else {
    // plain Google Earth KML: first path, otherwise all points
    const line = doc.getElementsByTagNameNS('*', 'LineString')[0];
    const coordEls = line ? [deep(line, 'coordinates')] : Array.from(doc.getElementsByTagNameNS('*', 'Point')).map(p => deep(p, 'coordinates'));
    coordEls.forEach(el => {
      if (!el) return;
      el.textContent.trim().split(/\s+/).forEach(tok => {
        const [lng, lat, alt] = tok.split(',').map(Number);
        if (isFinite(lat) && isFinite(lng)) wps.push(newWp([lat, lng], { alt: alt > 0 ? alt : null }));
      });
    });
    note = 'heights set to mission altitude where the file had none';
    settings.interval = Object.assign(clone(S.interval), { on: false, auto: false });
  }
  // use the most common height as the mission altitude and clear matching overrides
  const counts = {}; wps.forEach(w => { if (w.alt != null) { const k = Math.round(w.alt * 10) / 10; counts[k] = (counts[k] || 0) + 1; } });
  const common = Object.keys(counts).sort((a, b) => counts[b] - counts[a])[0];
  if (common != null && wps.every(w => w.alt != null)) { settings.alt = Number(common); wps.forEach(w => { if (w.alt != null && Math.abs(w.alt - settings.alt) < 0.05) w.alt = null; }); }
  const sc = {}; wps.forEach(w => { if (w.speed != null) { const k = Math.round(w.speed * 10) / 10; sc[k] = (sc[k] || 0) + 1; } });
  const commonSp = Object.keys(sc).sort((a, b) => sc[b] - sc[a])[0];
  if (commonSp != null) settings.speed = Number(commonSp);
  const sp = settings.speed || S.speed; wps.forEach(w => { if (w.speed != null && Math.abs(w.speed - sp) < 0.05) w.speed = null; });
  return { wps, settings, note };
}

/* ---------- place search ---------- */
const qEl = document.getElementById('q'), qRes = document.getElementById('qres');
let searchMarker = null, lastQuery = '';
function recentPlaces() { try { return JSON.parse(localStorage.getItem('planner.recent')) || []; } catch (e) { return []; } }
function addRecent(r) {
  const list = recentPlaces().filter(x => !(Math.abs(x.lat - r.lat) < 1e-5 && Math.abs(x.lng - r.lng) < 1e-5));
  list.unshift({ name: r.name, detail: r.detail || '', lat: r.lat, lng: r.lng, ext: r.ext || null });
  try { localStorage.setItem('planner.recent', JSON.stringify(list.slice(0, 8))); } catch (e) {}
}
/* Decimal ("32.6348, -97.1562") or degrees-minutes-seconds ("32°38'05"N 97°09'22"W"). */
function parseCoords(q) {
  const d = q.trim().match(/^(-?\d{1,2}(?:\.\d+)?)\s*[,;\s]\s*(-?\d{1,3}(?:\.\d+)?)$/);
  if (d) { const lat = +d[1], lng = +d[2]; if (Math.abs(lat) <= 90 && Math.abs(lng) <= 180) return [lat, lng]; }
  const re = /(\d{1,3})\s*[°d\s]\s*(\d{1,2})?\s*['′m]?\s*(\d{1,2}(?:\.\d+)?)?\s*["″s]?\s*([NSEW])/gi;
  const parts = []; let m;
  while ((m = re.exec(q)) && parts.length < 2) {
    let v = +m[1] + (+m[2] || 0) / 60 + (+m[3] || 0) / 3600;
    if (/[SW]/i.test(m[4])) v = -v;
    parts.push({ v, ns: /[NS]/i.test(m[4]) });
  }
  if (parts.length === 2) { const lat = parts.find(p => p.ns), lng = parts.find(p => !p.ns); if (lat && lng) return [lat.v, lng.v]; }
  return null;
}
function renderResults(html) { qRes.innerHTML = html; qRes.hidden = !html; }
let qResults = [];
function listHtml(list, heading) {
  qResults = list;
  return (heading ? `<div class="hd">${heading}</div>` : '') + list.map((r, i) => `<button class="r" data-r="${i}">${esc(r.name)}${r.detail && r.detail !== r.name ? `<small>${esc(r.detail)}</small>` : ''}</button>`).join('');
}
function showRecents() {
  const rec = recentPlaces();
  if (!qEl.value.trim() && rec.length) renderResults(listHtml(rec, 'Recent'));
}
function runSearch(q) {
  q = q.trim(); if (!q) return showRecents();
  const c = parseCoords(q);
  if (c) { goToPlace({ name: c[0].toFixed(5) + ', ' + c[1].toFixed(5), detail: 'Coordinates', lat: c[0], lng: c[1] }); return; }
  lastQuery = q;
  renderResults('<div class="msg">Searching…</div>');
  const ctr = map.getCenter();
  if (native && native.searchPlace) { native.searchPlace(q, ctr.lat, ctr.lng); return; }
  const url = 'https://geocode.arcgis.com/arcgis/rest/services/World/GeocodeServer/findAddressCandidates?f=json&maxLocations=8&outFields=Match_addr,Type,Place_addr&SingleLine=' +
    encodeURIComponent(q) + '&location=' + ctr.lng + ',' + ctr.lat;
  fetch(url).then(r => r.json()).then(j => {
    showPlaces({ query: q, results: (j.candidates || []).map(c => ({ name: c.address, detail: [c.attributes && c.attributes.Type, c.attributes && c.attributes.Place_addr].filter(Boolean).join(' · '), lat: c.location.y, lng: c.location.x, ext: c.extent ? [c.extent.ymin, c.extent.xmin, c.extent.ymax, c.extent.xmax] : null })) });
  }).catch(() => showPlaces({ query: q, results: [], error: 'offline' }));
}
function showPlaces(m) {
  if (m.query && m.query !== lastQuery) return;
  if (m.results && m.results.length) return renderResults(listHtml(m.results));
  renderResults(`<div class="msg">${m.error === 'offline' ? 'No internet connection. You can still type coordinates, like 32.6348, -97.1562.' : 'No places found for “' + esc(m.query || '') + '”. Try adding the city or state.'}</div>`);
}
function goToPlace(r) {
  renderResults(''); qEl.value = r.name; qEl.blur();
  if (searchMarker) map.removeLayer(searchMarker);
  searchMarker = L.marker([r.lat, r.lng], { interactive: false, zIndexOffset: 2000,
    icon: L.divIcon({ className: '', html: `<div class="pin"><i></i><b>${esc(r.name.split(',')[0])}</b></div>`, iconSize: [220, 50], iconAnchor: [110, 24] }) }).addTo(map);
  const e = r.ext;
  if (e && e[0] !== e[2] && hav([e[0], e[1]], [e[2], e[3]]) < 50000) map.fitBounds([[e[0], e[1]], [e[2], e[3]]], { maxZoom: 17 });
  else map.setView([r.lat, r.lng], Math.max(map.getZoom(), 16));
  addRecent(r);
}
document.getElementById('qform').addEventListener('submit', e => { e.preventDefault(); runSearch(qEl.value); });
qEl.addEventListener('focus', showRecents);
qEl.addEventListener('input', () => { if (!qEl.value.trim()) { if (searchMarker) { map.removeLayer(searchMarker); searchMarker = null; } showRecents(); } });
qRes.addEventListener('click', e => { const b = e.target.closest('[data-r]'); if (b) goToPlace(qResults[+b.dataset.r]); });
map.on('click', () => { renderResults(''); });
map.on('dragstart', () => { renderResults(''); qEl.blur(); });

/* ---------- my maps (MBTiles) ----------
   Your own basemaps, such as a WebODM orthophoto exported as MBTiles. The app copies the file into its storage and
   serves it locally, so it works with no signal. They draw on top of the regular map, which still shows outside them. */
let myMaps = [], myLayers = {}, myImport = null, myArmed = null;
function myStates() { try { return JSON.parse(localStorage.getItem('planner.mymaps')) || {}; } catch (e) { return {}; } }
const myStateCache = myStates();
function myState(id) { return myStateCache[id] || (myStateCache[id] = { on: true, opacity: 1 }); }
function saveMyState() { try { localStorage.setItem('planner.mymaps', JSON.stringify(myStateCache)); } catch (e) {} }
function myBounds(m) { return [[m.bounds[1], m.bounds[0]], [m.bounds[3], m.bounds[2]]]; }
function applyMyMaps() {
  Object.keys(myLayers).forEach(id => { if (!myMaps.some(m => m.id === id)) { map.removeLayer(myLayers[id]); delete myLayers[id]; } });
  myMaps.forEach((m, i) => {
    const st = myState(m.id);
    if (!myLayers[m.id]) {
      const opt = { maxNativeZoom: m.maxzoom, minNativeZoom: m.minzoom, minZoom: Math.max(0, m.minzoom - 2), maxZoom: 22, zIndex: 20 + i, opacity: st.opacity };
      if (m.bounds) opt.bounds = L.latLngBounds(myBounds(m));
      myLayers[m.id] = PC ? window.PcHost.mapLayer(m, opt) : L.tileLayer((window.MB_BASE || 'https://mbtiles.invalid/') + `${m.id}/{z}/{x}/{y}`, opt);
    }
    const lyr = myLayers[m.id];
    lyr.setOpacity(st.opacity);
    if (st.on && !map.hasLayer(lyr)) lyr.addTo(map);
    if (!st.on && map.hasLayer(lyr)) map.removeLayer(lyr);
  });
}
function openMyMaps() { sheetMode = 'maps'; myArmed = null; renderMyMaps(); sheet.hidden = false; if (native && native.listMaps) native.listMaps(); }
function renderMyMaps() {
  let h = `<div class="card"><h2>My maps</h2>
    <p class="note">${DESKTOP ? 'Add your own high-resolution basemap, such as an orthophoto from WebODM or QGIS saved as <b>MBTiles</b>. The PC reads the file where it is (it isn’t copied), so leave it in place, for example in your Google Drive folder. It shows on top of the regular map.' : 'Add your own high-resolution basemap, such as an orthophoto from WebODM. In WebODM, export the orthophoto as <b>MBTiles</b> and copy the file to this tablet. Maps are stored in the app, work without internet, and show on top of the regular map.'}</p>`;
  if (myImport) {
    const pct = myImport.total > 0 ? Math.floor(100 * myImport.done / myImport.total) : null;
    h += `<div class="status${myImport.error ? ' bad' : ''}">${myImport.error ? esc(myImport.error) : 'Copying the map into the app… ' + (pct != null ? pct + '%' : fmtBytes(myImport.done))}</div>`;
    if (pct != null && !myImport.error) h += `<div class="bar"><i style="width:${pct}%"></i></div>`;
  }
  const busy = myImport && !myImport.error;
  h += `<div class="btns" style="margin-bottom:6px"><button class="btn" data-m="close">Close</button><button class="btn primary" data-m="import"${busy ? ' disabled' : ''}>Import MBTiles…</button></div>`;
  h += `<h3 class="subh">Your maps</h3>`;
  if (!native || !native.importMbtiles) h += `<p class="note">${PC ? 'Opening map files needs Microsoft Edge or Google Chrome.' : 'Importing maps works in the Android app.'}</p>`;
  else if (!myMaps.length) h += `<p class="note">None yet.</p>`;
  else h += '<div class="mlist">' + myMaps.map(m => {
    const st = myState(m.id);
    if (m.locked) return `<div class="mitem"><div style="flex:1;min-width:0"><b>${esc(m.name)}</b><small>Allow access to the file again to show it</small></div><button class="btn small primary" data-m="unlock" data-id="${m.id}">Allow access</button><button class="btn small danger" data-m="del" data-id="${m.id}">${myArmed === m.id ? 'Confirm' : 'Remove'}</button></div>`;
    return `<div class="mitem" style="flex-wrap:wrap"><div style="flex:1;min-width:0"><b>${esc(m.name)}</b><small>${fmtBytes(m.size)} · zoom ${m.minzoom}–${m.maxzoom}${m.missing ? ' · <span style="color:var(--bad)">file not found</span>' : ''}</small></div>
      <button class="btn small" data-m="toggle" data-id="${m.id}">${st.on ? 'Hide' : 'Show'}</button>
      <button class="btn small" data-m="zoom" data-id="${m.id}"${m.bounds ? '' : ' disabled'}>Go to</button>
      <button class="btn small danger" data-m="del" data-id="${m.id}">${myArmed === m.id ? 'Confirm' : DESKTOP ? 'Remove' : 'Delete'}</button>
      <div class="range" style="width:100%"><span class="note" style="margin:0;min-width:64px">Opacity</span><input type="range" min="0.2" max="1" step="0.05" value="${st.opacity}" data-op="${m.id}"><output>${Math.round(st.opacity * 100)}%</output></div></div>`;
  }).join('') + '</div>';
  sheet.innerHTML = h + '</div>';
}
sheet.addEventListener('input', e => {
  const t = e.target; if (sheetMode !== 'maps' || !t.dataset.op) return;
  const st = myState(t.dataset.op); st.opacity = Number(t.value); t.nextElementSibling.textContent = Math.round(st.opacity * 100) + '%';
  if (myLayers[t.dataset.op]) myLayers[t.dataset.op].setOpacity(st.opacity);
  saveMyState();
});
function mapsClick(e) {
  if (e.target === sheet) { sheet.hidden = true; return; }
  const b = e.target.closest('button[data-m]'); if (!b || b.disabled) return;
  const a = b.dataset.m, id = b.dataset.id, m = myMaps.find(x => x.id === id);
  if (a === 'close') { sheet.hidden = true; return; }
  if (a === 'import') { if (native && native.importMbtiles) { myImport = null; native.importMbtiles(); } return; }
  if (a === 'unlock') { native.unlockMap(id); return; }
  if (a === 'toggle') { const st = myState(id); st.on = !st.on; saveMyState(); applyMyMaps(); renderTools(); }
  else if (a === 'zoom' && m && m.bounds) {
    const st = myState(id); if (!st.on) { st.on = true; saveMyState(); applyMyMaps(); renderTools(); }
    sheet.hidden = true; map.fitBounds(myBounds(m)); return;
  } else if (a === 'del') {
    if (myArmed === id) { myArmed = null; native.deleteMap(id); delete myStateCache[id]; saveMyState(); }
    else myArmed = id;
  }
  renderMyMaps();
}
function mapsMsg(m) {
  if (m.type === 'mbProgress') myImport = { done: m.done, total: m.total };
  else if (m.type === 'mbError') myImport = { error: m.message };
  else if (m.type === 'maps') {
    myMaps = m.maps || [];
    if (myImport && !myImport.error) myImport = null;
    applyMyMaps(); renderTools();
    if (m.added) {
      const nm = myMaps.find(x => x.id === m.added);
      if (nm) { toast('Added ' + nm.name); if (nm.bounds) { sheet.hidden = true; map.fitBounds(myBounds(nm)); return; } }
    }
  }
  if (!sheet.hidden && sheetMode === 'maps') renderMyMaps();
}

/* ---------- markers ----------
   Landmarks that stay on the map for every mission: stands, feeders, trail cams, tanks, gates and so on. Stored in the
   app (works offline), shared with HuntStand, Google Earth or Avenza through KML import and export. */
const MK_TYPES = {
  stand: { label: 'Stand / blind', color: '#8d5524', path: 'M8 4v16M16 4v16M8 8h8M8 12h8M8 16h8' },
  feeder: { label: 'Feeder', color: '#e67700', path: 'M12 4c3 4 5 6.5 5 9a5 5 0 0 1-10 0c0-2.5 2-5 5-9z' },
  camera: { label: 'Trail camera', color: '#343a40', path: 'M4 8h16v10H4zM9 8l1.5-2h3L15 8M12 16a3 3 0 1 0 0-6 3 3 0 0 0 0 6z' },
  water: { label: 'Water / tank', color: '#1c7ed6', path: 'M3 10c2-2 4-2 6 0s4 2 6 0 4-2 6 0M3 15c2-2 4-2 6 0s4 2 6 0 4-2 6 0' },
  gate: { label: 'Gate', color: '#2b8a3e', path: 'M4 5v14M20 5v14M4 9h16M4 15h16M4 15L20 9' },
  crossing: { label: 'Road / crossing', color: '#5c677d', path: 'M8 3L5 21M16 3l3 18M12 5v3M12 11v3M12 17v3' },
  landmark: { label: 'Landmark', color: '#c2255c', path: 'M6 21V4M6 4h11l-2 4 2 4H6' },
  general: { label: 'Marker', color: '#7048e8', path: 'M12 15a3 3 0 1 0 0-6 3 3 0 0 0 0 6z' },
};
let markers = mkLoad(), mkVis = mkLoadVis(), mkEdit = null, mkArmed = false, mkLastType = 'general';
const mkLayer = L.layerGroup().addTo(map);
function mkLoad() { try { return JSON.parse(localStorage.getItem('planner.markers')) || []; } catch (e) { return []; } }
function mkSave() { syStamp('markers', markers); try { localStorage.setItem('planner.markers', JSON.stringify(markers)); } catch (e) { toast('Could not save markers: storage is full'); } }
function mkLoadVis() { try { return Object.assign({ all: true, labels: true, hidden: {} }, JSON.parse(localStorage.getItem('planner.markerVis')) || {}); } catch (e) { return { all: true, labels: true, hidden: {} }; } }
function mkSaveVis() { try { localStorage.setItem('planner.markerVis', JSON.stringify(mkVis)); } catch (e) {} }
function mkIcon(type, size) {
  const t = MK_TYPES[type] || MK_TYPES.general;
  return `<svg viewBox="0 0 24 24" width="${size}" height="${size}" fill="none" stroke="#fff" stroke-width="2.2" stroke-linecap="round" stroke-linejoin="round"><path d="${t.path}"/></svg>`;
}
function renderMarkers() {
  mkLayer.clearLayers();
  if (!mkVis.all) return;
  markers.forEach(m => {
    if (mkVis.hidden[m.type]) return;
    const t = MK_TYPES[m.type] || MK_TYPES.general;
    const html = `<div class="mk"><i style="background:${t.color}">${mkIcon(m.type, 16)}${m.photos && m.photos.length ? '<em class="mkph"></em>' : ''}</i>${mkVis.labels && m.name ? `<b>${esc(m.name)}</b>` : ''}</div>`;
    const mk = L.marker([m.lat, m.lng], { draggable: !featuresLocked(), interactive: !featuresLocked(), zIndexOffset: 800, icon: L.divIcon({ className: '', html, iconSize: [160, 44], iconAnchor: [80, 14] }) }).addTo(mkLayer);
    mk.on('click', () => openMarker(m.id));
    mk.on('dragend', e => { m.lat = e.target.getLatLng().lat; m.lng = e.target.getLatLng().lng; mkSave(); });
  });
}
function addMarker(p, extra) {
  const m = Object.assign({ id: 'k' + Date.now() + Math.floor(Math.random() * 1000), type: mkLastType, name: '', notes: '', lat: p[0], lng: p[1] }, extra || {});
  markers.push(m); mkSave();
  if (!mkVis.all) { mkVis.all = true; mkSaveVis(); }
  if (mkVis.hidden[m.type]) { delete mkVis.hidden[m.type]; mkSaveVis(); }
  renderMarkers();
  return m;
}
/* ----- one marker ----- */
function openMarker(id, isNew) {
  const m = markers.find(x => x.id === id); if (!m) return;
  mkEdit = { id, isNew: !!isNew }; mkArmed = false; sheetMode = 'marker'; renderMarker(); sheet.hidden = false;
  if (isNew) setTimeout(() => { const n = document.getElementById('mkname'); if (n) n.focus(); }, 50);
}
function renderMarker() {
  const m = markers.find(x => x.id === mkEdit.id); if (!m) { sheet.hidden = true; return; }
  const coords = m.lat.toFixed(6) + ', ' + m.lng.toFixed(6);
  sheet.innerHTML = `<div class="card"><h2>${mkEdit.isNew ? 'New marker' : 'Marker'}</h2>
    <div class="mktypes">${Object.entries(MK_TYPES).map(([k, t]) => `<button class="mkt${m.type === k ? ' on' : ''}" data-k="type" data-v="${k}"><i style="background:${t.color}">${mkIcon(k, 16)}</i><span>${t.label}</span></button>`).join('')}</div>
    <label class="f"><span>Name</span><div class="in"><input type="text" id="mkname" value="${esc(m.name)}" placeholder="${esc(MK_TYPES[m.type].label)}"></div></label>
    <label class="f"><span>Notes</span><div class="in"><textarea id="mknotes" rows="3" placeholder="Height, direction, last checked…">${esc(m.notes)}</textarea></div></label>
    ${phStrip(m, 'm')}
    <p class="note">Location: <b>${coords}</b> <button class="btn small" data-k="copy">Copy</button> · Drag the marker on the map to move it.</p>
    <div class="btns" style="margin-bottom:8px"><button class="btn small" data-k="orbit">Orbit this (POI)</button><button class="btn small" data-k="wp">Add as waypoint</button><button class="btn small" data-k="go">Center map</button></div>
    <div class="btns"><button class="btn danger" data-k="del">${mkArmed ? 'Tap again to delete' : 'Delete'}</button><button class="btn primary" data-k="done">Done</button></div></div>`;
  phFillThumbs(sheet);
}
function markerClick(e) {
  if (e.target === sheet) { sheet.hidden = true; return; }
  const b = e.target.closest('button[data-k]'); if (!b) return;
  const m = markers.find(x => x.id === mkEdit.id); if (!m) return;
  const k = b.dataset.k;
  if (k === 'type') { m.type = b.dataset.v; mkLastType = m.type; mkSave(); renderMarkers(); renderMarker(); return; }
  if (k === 'done') { sheet.hidden = true; return; }
  if (k === 'del') { if (mkArmed) { markers = markers.filter(x => x.id !== m.id); mkSave(); renderMarkers(); sheet.hidden = true; toast('Marker deleted'); } else { mkArmed = true; renderMarker(); } return; }
  if (k === 'copy') { const t = m.lat.toFixed(6) + ', ' + m.lng.toFixed(6); try { navigator.clipboard.writeText(t).then(() => toast('Copied ' + t), () => toast(t)); } catch (err) { toast(t); } return; }
  if (k === 'go') { sheet.hidden = true; map.setView([m.lat, m.lng], Math.max(map.getZoom(), 17)); return; }
  if (k === 'orbit') { sheet.hidden = true; setShape({ type: 'poi', center: [m.lat, m.lng] }); toast('Orbit set around ' + (m.name || MK_TYPES[m.type].label)); return; }
  if (k === 'wp') { sheet.hidden = true; ensureManual(); S.wps.push(newWp([m.lat, m.lng])); S.sel = S.wps.length - 1; commit(false); toast('Added waypoint ' + S.wps.length); return; }
}
function markerChange(t) {
  const m = markers.find(x => x.id === mkEdit.id); if (!m) return;
  if (t.id === 'mkname') m.name = t.value.trim();
  else if (t.id === 'mknotes') m.notes = t.value;
  mkSave(); renderMarkers();
}
/* ----- list ----- */
function openMarkers() { sheetMode = 'markers'; renderMarkerList(); sheet.hidden = false; }
function renderMarkerList() {
  const c = map.getCenter(), here = [c.lat, c.lng];
  const counts = {}; markers.forEach(m => { counts[m.type] = (counts[m.type] || 0) + 1; });
  const list = markers.slice().sort((a, b) => hav(here, [a.lat, a.lng]) - hav(here, [b.lat, b.lng]));
  let h = `<div class="card"><h2>Features</h2>
    <p class="note">Tap <b>Add on map</b> then tap a spot, or press and hold anywhere on the map. Markers stay on the map for every mission and work offline.</p>
    <div class="btns" style="margin-bottom:8px"><button class="btn primary" data-l2="add">Add on map</button>${S.home ? '<button class="btn" data-l2="here">Add at my location</button>' : ''}<button class="btn" data-l2="photos">Photos → markers…</button></div>
    <label class="chk"><input type="checkbox" id="mkall"${mkVis.all ? ' checked' : ''}>Show markers on the map</label>
    <label class="chk"><input type="checkbox" id="mklabels"${mkVis.labels ? ' checked' : ''}>Show names</label>
    <div class="mktypes">${Object.entries(MK_TYPES).map(([k, t]) => `<button class="mkt${mkVis.hidden[k] ? '' : ' on'}" data-l2="vis" data-v="${k}"><i style="background:${t.color}">${mkIcon(k, 16)}</i><span>${t.label}${counts[k] ? ' · ' + counts[k] : ''}</span></button>`).join('')}</div>
    <h3 class="subh">${markers.length} marker${markers.length === 1 ? '' : 's'} · nearest first</h3>`;
  if (!markers.length) h += `<p class="note">None yet.</p>`;
  else h += '<div class="mlist">' + list.slice(0, 200).map(m => {
    const t = MK_TYPES[m.type] || MK_TYPES.general;
    return `<div class="mitem"><i class="mkdot" style="background:${t.color}">${mkIcon(m.type, 14)}</i><div style="flex:1;min-width:0"><b>${esc(m.name || t.label)}</b><small>${t.label} · ${fmtDist(hav(here, [m.lat, m.lng]))} from map center${m.photos && m.photos.length ? ' · ' + m.photos.length + ' photo' + (m.photos.length === 1 ? '' : 's') : ''}</small></div>
      <button class="btn small" data-l2="go" data-id="${m.id}">Go to</button><button class="btn small" data-l2="edit" data-id="${m.id}">Edit</button></div>`;
  }).join('') + '</div>';
  h += `<h3 class="subh">Boundaries & lines · ${shapes.length}</h3>
    <div class="btns" style="margin-bottom:8px"><button class="btn" data-l2="barea">Draw area</button><button class="btn" data-l2="bline">Draw line</button><button class="btn" data-l2="hfind">Find power lines & towers…</button></div>
    <label class="chk"><input type="checkbox" id="shall"${mkVis.shapes !== false ? ' checked' : ''}>Show boundaries and lines on the map</label>`;
  if (shapes.length) h += '<div class="mlist">' + shapes.map(s => {
    const t = shType(s);
    return `<div class="mitem"><i class="shsw" style="border-color:${t.color};background:${s.kind === 'area' ? t.color + '33' : 'transparent'}${t.dash ? ';border-style:dashed' : ''}"></i><div style="flex:1;min-width:0"><b>${esc(s.name || t.label)}</b><small>${t.label} · ${shSize(s)}${s.hazard === 'over' ? ' · ⚡ fly-over hazard' : s.hazard === 'nofly' ? ' · ⛔ no-fly zone' : ''}</small></div>
      <button class="btn small" data-l2="sgo" data-id="${s.id}">Go to</button><button class="btn small" data-l2="sedit" data-id="${s.id}">Edit</button></div>`;
  }).join('') + '</div>';
  h += `<h3 class="subh">Share</h3><div class="btns"><button class="btn" data-l2="import">Import KML / KMZ / GPX…</button><button class="btn" data-l2="export"${markers.length || shapes.length ? '' : ' disabled'}>Export KML</button></div>
    <p class="note">Works with HuntStand, Google Earth, Avenza and onX exports. Imported points become markers; types are guessed from their names. Points become markers; outlines and lines become boundaries and lines. GPS tracks are skipped.</p>
    <div class="btns" style="margin-top:4px"><button class="btn" data-l2="close">Close</button></div></div>`;
  sheet.innerHTML = h;
}
function markerListClick(e) {
  if (e.target === sheet) { sheet.hidden = true; return; }
  const b = e.target.closest('button[data-l2]'); if (!b || b.disabled) return;
  const k = b.dataset.l2, m = markers.find(x => x.id === b.dataset.id);
  if (k === 'close') { sheet.hidden = true; return; }
  if (k === 'photos') { phToMarkers(); return; }
  if (k === 'add') { sheet.hidden = true; setTool('marker'); return; }
  if (k === 'barea' || k === 'bline') { sheet.hidden = true; setTool(k); return; }
  if (k === 'hfind') { openFindHazards(); return; }
  if (k === 'sgo' || k === 'sedit') { const sh = shapes.find(x => x.id === b.dataset.id); if (!sh) return; if (k === 'sedit') { openShapeEditor(sh.id); return; } sheet.hidden = true; map.fitBounds(L.latLngBounds(sh.pts).pad(0.15)); return; }
  if (k === 'here' && S.home) { const nm = addMarker(S.home); openMarker(nm.id, true); return; }
  if (k === 'vis') { const v = b.dataset.v; if (mkVis.hidden[v]) delete mkVis.hidden[v]; else mkVis.hidden[v] = true; mkSaveVis(); renderMarkers(); renderMarkerList(); return; }
  if (k === 'go' && m) { sheet.hidden = true; map.setView([m.lat, m.lng], Math.max(map.getZoom(), 17)); return; }
  if (k === 'edit' && m) { openMarker(m.id); return; }
  if (k === 'import') { if (native && native.importFor) native.importFor('markers'); else toast('Importing works in the Android app.'); return; }
  if (k === 'export') {
    const kml = markersToKml();
    if (native && native.saveText) native.saveText('FlyPlan features.kml', 'application/vnd.google-earth.kml+xml', kml);
    else toast('Exporting works in the Android app.');
  }
}
function markerListChange(t) {
  if (t.id === 'mkall') { mkVis.all = t.checked; mkSaveVis(); renderMarkers(); }
  else if (t.id === 'mklabels') { mkVis.labels = t.checked; mkSaveVis(); renderMarkers(); renderShapes(); }
  else if (t.id === 'shall') { mkVis.shapes = t.checked; mkSaveVis(); renderShapes(); }
}
/* ----- KML ----- */
function markersToKml() {
  const styles = Object.entries(MK_TYPES).map(([k, t]) => {
    const c = t.color.replace('#', ''); const abgr = 'ff' + c.slice(4, 6) + c.slice(2, 4) + c.slice(0, 2);
    return `<Style id="${k}"><IconStyle><color>${abgr}</color><Icon><href>http://maps.google.com/mapfiles/kml/paddle/wht-blank.png</href></Icon></IconStyle></Style>`;
  }).join('\n');
  const pms = markers.map(m => `<Placemark><name>${esc(m.name || MK_TYPES[m.type].label)}</name>${m.notes ? `<description>${esc(m.notes)}</description>` : ''}<styleUrl>#${m.type}</styleUrl>` +
    `<ExtendedData><Data name="type"><value>${m.type}</value></Data></ExtendedData><Point><coordinates>${m.lng.toFixed(7)},${m.lat.toFixed(7)},0</coordinates></Point></Placemark>`).join('\n');
  return `<?xml version="1.0" encoding="UTF-8"?>\n<kml xmlns="http://www.opengis.net/kml/2.2"><Document><name>FlyPlan features</name>\n${styles}\n${pms}\n${shapesToKml()}\n</Document></kml>\n`;
}
function guessType(text) {
  const t = (text || '').toLowerCase();
  if (/stand|blind|ladder|tower|box/.test(t)) return 'stand';
  if (/feed|corn|protein/.test(t)) return 'feeder';
  if (/cam|camera|trail ?cam/.test(t)) return 'camera';
  if (/tank|pond|water|trough|creek|well|windmill/.test(t)) return 'water';
  if (/gate/.test(t)) return 'gate';
  if (/road|crossing|culvert|bridge|trail/.test(t)) return 'crossing';
  return 'landmark';
}
function importMarkers(m) {
  const text = m.kml || m.waylines || m.template || '';
  const doc = new DOMParser().parseFromString(text, 'application/xml');
  if (doc.getElementsByTagName('parsererror').length) { toast('Could not read that file as KML or GPX'); return; }
  let added = 0, skipped = 0;
  // GPX waypoints (<wpt lat lon><name/><desc/><sym/></wpt>)
  Array.from(doc.getElementsByTagNameNS('*', 'wpt')).forEach(w => {
    const lat = Number(w.getAttribute('lat')), lng = Number(w.getAttribute('lon'));
    if (!isFinite(lat) || !isFinite(lng)) { skipped++; return; }
    const child = n => { const e = Array.from(w.childNodes).find(x => x.localName === n); return e ? e.textContent.trim() : ''; };
    const name = child('name'), desc = (child('desc') || child('cmt')).replace(/<[^>]+>/g, ' ').replace(/\s+/g, ' ').trim();
    if (markers.some(x => Math.abs(x.lat - lat) < 1e-6 && Math.abs(x.lng - lng) < 1e-6 && x.name === name)) { skipped++; return; }
    markers.push({ id: 'k' + Date.now() + '_g' + added, type: guessType(name + ' ' + desc + ' ' + child('sym') + ' ' + child('type')), name, notes: desc, lat, lng });
    added++;
  });
  skipped += doc.getElementsByTagNameNS('*', 'trk').length + doc.getElementsByTagNameNS('*', 'rte').length;
  const cnt = { shapes: 0, skipped: 0 };
  Array.from(doc.getElementsByTagNameNS('*', 'Placemark')).forEach(pm => {
    const pt = pm.getElementsByTagNameNS('*', 'Point')[0];
    if (!pt) { if (!importShapePlacemark(pm, cnt)) skipped++; return; }
    const c = (pt.getElementsByTagNameNS('*', 'coordinates')[0] || {}).textContent || '';
    const [lng, lat] = c.trim().split(/[,\s]+/).map(Number);
    if (!isFinite(lat) || !isFinite(lng)) { skipped++; return; }
    const child = n => { const e = Array.from(pm.childNodes).find(x => x.localName === n); return e ? e.textContent.trim() : ''; };
    const name = child('name'), desc = child('description').replace(/<[^>]+>/g, ' ').replace(/\s+/g, ' ').trim();
    let type = null;
    Array.from(pm.getElementsByTagNameNS('*', 'Data')).forEach(d => { if (d.getAttribute('name') === 'type') { const v = d.textContent.trim(); if (MK_TYPES[v]) type = v; } });
    if (!type) { const su = child('styleUrl').replace('#', ''); if (MK_TYPES[su]) type = su; }
    if (!type) type = guessType(name + ' ' + desc + ' ' + child('styleUrl'));
    if (markers.some(x => Math.abs(x.lat - lat) < 1e-6 && Math.abs(x.lng - lng) < 1e-6 && x.name === name)) { skipped++; return; }
    markers.push({ id: 'k' + Date.now() + '_' + added, type, name, notes: desc, lat, lng });
    added++;
  });
  mkSave(); if (!mkVis.all) { mkVis.all = true; mkSaveVis(); }
  renderMarkers();
  skipped += cnt.skipped;
  if (cnt.shapes) { shSave(); if (mkVis.shapes === false) { mkVis.shapes = true; mkSaveVis(); } renderShapes(); }
  const parts = [];
  if (added) parts.push(`${added} marker${added === 1 ? '' : 's'}`);
  if (cnt.shapes) parts.push(`${cnt.shapes} boundar${cnt.shapes === 1 ? 'y or line' : 'ies and lines'}`);
  toast(parts.length ? 'Imported ' + parts.join(' and ') + (skipped ? ` (${skipped} skipped: duplicates or GPS tracks)` : '') : 'Nothing new found in that file');
  const fitPts = markers.slice(markers.length - added).map(x => [x.lat, x.lng]).concat(...shapes.slice(shapes.length - cnt.shapes).map(x => x.pts));
  if (fitPts.length) map.fitBounds(L.latLngBounds(fitPts).pad(0.2), { maxZoom: 17 });
  if (!sheet.hidden && sheetMode === 'markers') renderMarkerList();
}
map.on('contextmenu', e => {   // press and hold (or right-click) adds a marker
  if (draw) return;
  const m = addMarker([e.latlng.lat, e.latlng.lng]); openMarker(m.id, true);
});
renderMarkers();

/* ---------- boundaries & lines ----------
   Permanent areas (property line, pastures, food plots, sanctuaries) and lines (fences, roads, creeks). Stored in the
   app with the markers, shown on every mission, and shared through the same KML import and export. */
const SH_TYPES = {
  area: {
    boundary: { label: 'Property boundary', color: '#ffd43b', fill: 0.03 },
    pasture: { label: 'Pasture / field', color: '#94d82d', fill: 0.10 },
    plot: { label: 'Food plot', color: '#40c057', fill: 0.18 },
    bedding: { label: 'Bedding area', color: '#ff922b', fill: 0.14 },
    sanctuary: { label: 'Sanctuary / no hunting', color: '#fa5252', fill: 0.12, dash: '10 6' },
    water: { label: 'Water', color: '#4dabf7', fill: 0.25 },
    nofly: { label: 'No-fly zone', color: '#e03131', fill: 0.18, dash: '6 5', hz: 'nofly' },
    tower: { label: 'Tower / obstacle', color: '#ff6b6b', fill: 0.35, hz: 'over' },
    other: { label: 'Other area', color: '#cc5de8', fill: 0.10 },
  },
  line: {
    fence: { label: 'Fence', color: '#f1f3f5', dash: '10 6' },
    road: { label: 'Road / trail', color: '#ffa94d' },
    creek: { label: 'Creek / drainage', color: '#4dabf7' },
    power: { label: 'Power line', color: '#ff6b6b', dash: '3 7', hz: 'over' },
    other: { label: 'Other line', color: '#cc5de8' },
  },
};
let shapes = shLoad(), shEditId = null, shOpen = null, shArmed = false;
const shpLayer = L.layerGroup().addTo(map);
const shRenderer = L.canvas({ padding: 0.3, tolerance: 12 });
function shLoad() { try { return JSON.parse(localStorage.getItem('planner.shapes')) || []; } catch (e) { return []; } }
let hzSig = '';
function hzSignature() { return JSON.stringify(hzShapes().map(s => [s.id, s.hazard, s.top, s.buf, s.kind, s.pts])); }
function shSave() { syStamp('shapes', shapes); try { localStorage.setItem('planner.shapes', JSON.stringify(shapes)); } catch (e) { toast('Could not save: storage is full'); } const sig = hzSignature(); if (sig !== hzSig) { hzSig = sig; hzChanged(); } }
function shType(s) { return (SH_TYPES[s.kind] || SH_TYPES.area)[s.type] || SH_TYPES[s.kind].other; }
function shLength(pts, closed) { let d = 0; for (let i = 1; i < pts.length; i++) d += hav(pts[i - 1], pts[i]); if (closed && pts.length > 2) d += hav(pts[pts.length - 1], pts[0]); return d; }
function shArea(pts) { return pts.length > 2 ? polyArea(pts.map(projector(centroid(pts)).xy)) : 0; }
function shSize(s) { return s.kind === 'area' ? fmtArea(shArea(s.pts)) + ' · ' + fmtDist(shLength(s.pts, true)) + ' around' : fmtDist(shLength(s.pts, false)); }
function renderShapes() {
  shpLayer.clearLayers();
  if (mkVis.shapes === false) return;
  const locked = featuresLocked();
  shapes.forEach(s => {
    const t = shType(s), editing = draw && draw.tool === 'shapeedit' && shEditId === s.id;
    // Only the outline is tappable (with a generous touch margin), so tapping inside a big area doesn't open it.
    const closed = () => s.kind === 'area' ? s.pts.concat([s.pts[0]]) : s.pts;
    const fill = s.kind === 'area' ? L.polygon(s.pts, { stroke: false, fillColor: t.color, fillOpacity: t.fill || 0.08, interactive: false }).addTo(shpLayer) : null;
    const lyr = L.polyline(closed(), { renderer: shRenderer, color: t.color, weight: editing ? 4 : 3, opacity: 0.95, dashArray: t.dash || null, interactive: !locked }).addTo(shpLayer);
    if (!locked) lyr.on('click', ev => { L.DomEvent.stop(ev); openShapeEditor(s.id); });
    const redraw = () => { lyr.setLatLngs(closed()); if (fill) fill.setLatLngs(s.pts); };
    if ((s.hazard === 'over' || s.hazard === 'nofly') && s.pts.length >= (s.kind === 'area' ? 3 : 2)) {
      const pr = projector(centroid(s.pts)), d = HZ_DEFAULTS[s.hazard], buf = s.buf != null ? s.buf : d.buf;
      if (buf > 0) L.polygon(hzBuffer(s.pts.map(pr.xy), s.kind === 'area', buf).map(pr.ll), { color: '#e03131', weight: 1, opacity: 0.6, dashArray: '4 4', fillColor: '#e03131', fillOpacity: s.hazard === 'nofly' ? 0.16 : 0.10, interactive: false }).addTo(shpLayer);
      const at = s.kind === 'area' ? centroid(s.pts) : s.pts[Math.floor((s.pts.length - 1) / 2)];
      L.marker(at, { interactive: false, icon: L.divIcon({ className: '', html: `<div class="hzlabel">${s.hazard === 'nofly' ? '⛔ No-fly' : '⚡ ' + fmtLen(s.top != null ? s.top : d.top)}</div>`, iconSize: [110, 20], iconAnchor: [55, -6] }) }).addTo(shpLayer);
    }
    if (mkVis.labels && s.name) {
      const at = s.kind === 'area' ? centroid(s.pts) : s.pts[Math.floor((s.pts.length - 1) / 2)];
      L.marker(at, { interactive: false, icon: L.divIcon({ className: '', html: `<div class="shlabel" style="color:${t.color}">${esc(s.name)}</div>`, iconSize: [160, 18], iconAnchor: [80, 9] }) }).addTo(shpLayer);
    }
    if (editing) s.pts.forEach((p, i) => {
      const v = L.marker(p, { draggable: true, zIndexOffset: 1500, icon: L.divIcon({ className: '', html: '<div class="vtx"></div>', iconSize: [18, 18], iconAnchor: [9, 9] }) }).addTo(shpLayer);
      v.on('drag', e => { s.pts[i] = [e.latlng.lat, e.latlng.lng]; redraw(); });
      v.on('dragend', () => { shSave(); renderShapes(); });
    });
  });
}
function createShape(kind, pts) {
  const s = { id: 's' + Date.now(), kind, type: kind === 'area' ? 'boundary' : 'fence', name: '', notes: '', pts };
  shapes.push(s); shSave();
  if (mkVis.shapes === false) { mkVis.shapes = true; mkSaveVis(); }
  renderShapes();
  return s;
}
/* ----- editor ----- */
function openShapeEditor(id, isNew) { shOpen = { id, isNew: !!isNew }; shArmed = false; sheetMode = 'shape'; renderShapeEditor(); sheet.hidden = false; }
function renderShapeEditor() {
  const s = shapes.find(x => x.id === shOpen.id); if (!s) { sheet.hidden = true; return; }
  const types = SH_TYPES[s.kind];
  sheet.innerHTML = `<div class="card"><h2>${shOpen.isNew ? 'New ' : ''}${s.kind === 'area' ? 'area' : 'line'}</h2>
    <div class="mktypes">${Object.entries(types).map(([k, t]) => `<button class="mkt${s.type === k ? ' on' : ''}" data-sh="type" data-v="${k}"><i class="shsw" style="border-color:${t.color};background:${s.kind === 'area' ? t.color + '33' : 'transparent'}${t.dash ? ';border-style:dashed' : ''}"></i><span>${t.label}</span></button>`).join('')}</div>
    <label class="f"><span>Name</span><div class="in"><input type="text" id="shname" value="${esc(s.name)}" placeholder="${esc(shType(s).label)}"></div></label>
    <label class="f"><span>Notes</span><div class="in"><textarea id="shnotes" rows="2">${esc(s.notes)}</textarea></div></label>
    ${phStrip(s, 's')}
    <h3 class="subh">Flight hazard</h3>
    <div class="mktypes">${[['none', 'Not a hazard'], ['over', 'Fly over only'], ['nofly', 'No-fly zone']].map(([k, l]) => `<button class="mkt${(s.hazard || 'none') === k ? ' on' : ''}" data-sh="hz" data-v="${k}"><span>${l}</span></button>`).join('')}</div>
    ${s.hazard === 'over' ? `<div class="row"><label class="f"><span>Height of its top (above ground)</span><div class="in"><input type="number" inputmode="decimal" id="shtop" value="${r0(toLen(s.top != null ? s.top : HZ_DEFAULTS.over.top))}"><em>${LEN()}</em></div></label>
      <label class="f"><span>Keep-away distance</span><div class="in"><input type="number" inputmode="decimal" id="shbuf" value="${r0(toLen(s.buf != null ? s.buf : HZ_DEFAULTS.over.buf))}"><em>${LEN()}</em></div></label></div>
      <p class="note">Within the keep-away distance, routes stay at least the clearance margin (Hazards, in the mission panel) above this height.</p>` : ''}
    ${s.hazard === 'nofly' ? `<div class="row"><label class="f"><span>Keep-away distance</span><div class="in"><input type="number" inputmode="decimal" id="shbuf" value="${r0(toLen(s.buf != null ? s.buf : HZ_DEFAULTS.nofly.buf))}"><em>${LEN()}</em></div></label><div></div></div>
      <p class="note">Missions never enter this ${s.kind === 'area' ? 'area' : 'line'} or come within the keep-away distance, at any height.</p>` : ''}
    <p class="note"><b>${shSize(s)}</b> · ${s.pts.length} points</p>
    <div class="btns" style="margin-bottom:8px"><button class="btn small" data-sh="edit">Move corners</button>${s.kind === 'area' ? '<button class="btn small" data-sh="survey">Use as survey area</button>' : ''}<button class="btn small" data-sh="go">Center map</button></div>
    <div class="btns"><button class="btn danger" data-sh="del">${shArmed ? 'Tap again to delete' : 'Delete'}</button><button class="btn primary" data-sh="done">Done</button></div></div>`;
  phFillThumbs(sheet);
}
function shapeClick(e) {
  if (e.target === sheet) { sheet.hidden = true; return; }
  const b = e.target.closest('button[data-sh]'); if (!b) return;
  const s = shapes.find(x => x.id === shOpen.id); if (!s) return;
  const k = b.dataset.sh;
  if (k === 'type') { s.type = b.dataset.v; const d = SH_TYPES[s.kind][s.type].hz; if (d && !s.hazard) { s.hazard = d; s.top = HZ_DEFAULTS[d].top; s.buf = HZ_DEFAULTS[d].buf; } shSave(); renderShapes(); renderShapeEditor(); return; }
  if (k === 'hz') { s.hazard = b.dataset.v === 'none' ? null : b.dataset.v; if (s.hazard) { if (s.top == null) s.top = HZ_DEFAULTS[s.hazard].top; if (s.buf == null) s.buf = HZ_DEFAULTS[s.hazard].buf; } shSave(); renderShapes(); renderShapeEditor(); return; }
  if (k === 'done') { sheet.hidden = true; return; }
  if (k === 'del') { if (shArmed) { shapes = shapes.filter(x => x.id !== s.id); shSave(); renderShapes(); sheet.hidden = true; toast('Deleted'); } else { shArmed = true; renderShapeEditor(); } return; }
  if (k === 'go') { sheet.hidden = true; map.fitBounds(L.latLngBounds(s.pts).pad(0.15)); return; }
  if (k === 'edit') { sheet.hidden = true; shEditId = s.id; setTool('shapeedit'); map.fitBounds(L.latLngBounds(s.pts).pad(0.15)); return; }
  if (k === 'survey') { sheet.hidden = true; S.name = s.name || shType(s).label; S.libId = null; setShape({ type: 'polygon', pts: s.pts.map(p => p.slice()) }); toast('Survey area set from ' + (s.name || shType(s).label)); }
}
function shapeChange(t) {
  const s = shapes.find(x => x.id === shOpen.id); if (!s) return;
  if (t.id === 'shname') s.name = t.value.trim();
  else if (t.id === 'shnotes') s.notes = t.value;
  else if (t.id === 'shtop' || t.id === 'shbuf') { const v = Number(t.value); if (!isFinite(v) || t.value === '') return renderShapeEditor(); s[t.id === 'shtop' ? 'top' : 'buf'] = Math.min(300, Math.max(0, fromLen(v))); }
  shSave(); renderShapes();
}
/* ----- KML ----- */
function shapesToKml() {
  return shapes.map(s => {
    const t = shType(s), c = t.color.replace('#', ''), abgr = c.slice(4, 6) + c.slice(2, 4) + c.slice(0, 2);
    const coords = s.pts.concat(s.kind === 'area' ? [s.pts[0]] : []).map(p => `${p[1].toFixed(7)},${p[0].toFixed(7)},0`).join(' ');
    const geom = s.kind === 'area' ? `<Polygon><outerBoundaryIs><LinearRing><coordinates>${coords}</coordinates></LinearRing></outerBoundaryIs></Polygon>` : `<LineString><coordinates>${coords}</coordinates></LineString>`;
    return `<Placemark><name>${esc(s.name || t.label)}</name>${s.notes ? `<description>${esc(s.notes)}</description>` : ''}` +
      `<Style><LineStyle><color>ff${abgr}</color><width>3</width></LineStyle><PolyStyle><color>33${abgr}</color></PolyStyle></Style>` +
      `<ExtendedData><Data name="kind"><value>${s.kind}</value></Data><Data name="type"><value>${s.type}</value></Data></ExtendedData>${geom}</Placemark>`;
  }).join('\n');
}
function guessShapeType(kind, text) {
  const t = (text || '').toLowerCase();
  if (kind === 'area') {
    if (/boundar|property|ranch|lease|line/.test(t)) return 'boundary';
    if (/plot|oats|clover|wheat/.test(t)) return 'plot';
    if (/bed/.test(t)) return 'bedding';
    if (/sanct|no hunt|refuge/.test(t)) return 'sanctuary';
    if (/pond|tank|lake|water/.test(t)) return 'water';
    if (/pasture|field|meadow|trap/.test(t)) return 'pasture';
    return 'other';
  }
  if (/fence/.test(t)) return 'fence';
  if (/road|trail|path|lane/.test(t)) return 'road';
  if (/creek|draw|drain|river|branch/.test(t)) return 'creek';
  if (/power|electric/.test(t)) return 'power';
  return 'other';
}
/* Polygons and lines from a KML Placemark; returns true if it was a shape (added or duplicate). */
function importShapePlacemark(pm, counts) {
  const poly = pm.getElementsByTagNameNS('*', 'Polygon')[0], line = pm.getElementsByTagNameNS('*', 'LineString')[0];
  if (!poly && !line) return false;
  const kind = poly ? 'area' : 'line';
  const cEl = (poly ? (poly.getElementsByTagNameNS('*', 'outerBoundaryIs')[0] || poly) : line).getElementsByTagNameNS('*', 'coordinates')[0];
  let pts = ((cEl || {}).textContent || '').trim().split(/\s+/).map(t => t.split(',').map(Number)).filter(a => isFinite(a[0]) && isFinite(a[1])).map(a => [a[1], a[0]]);
  if (kind === 'area' && pts.length > 3 && Math.abs(pts[0][0] - pts[pts.length - 1][0]) < 1e-9 && Math.abs(pts[0][1] - pts[pts.length - 1][1]) < 1e-9) pts.pop();
  if (pts.length < (kind === 'area' ? 3 : 2)) { counts.skipped++; return true; }
  const child = n => { const e = Array.from(pm.childNodes).find(x => x.localName === n); return e ? e.textContent.trim() : ''; };
  const name = child('name'), notes = child('description').replace(/<[^>]+>/g, ' ').replace(/\s+/g, ' ').trim();
  let type = null;
  Array.from(pm.getElementsByTagNameNS('*', 'Data')).forEach(d => { if (d.getAttribute('name') === 'type' && SH_TYPES[kind][d.textContent.trim()]) type = d.textContent.trim(); });
  if (!type) type = guessShapeType(kind, name + ' ' + notes);
  if (shapes.some(x => x.kind === kind && x.name === name && x.pts.length === pts.length && Math.abs(x.pts[0][0] - pts[0][0]) < 1e-6 && Math.abs(x.pts[0][1] - pts[0][1]) < 1e-6)) { counts.skipped++; return true; }
  shapes.push({ id: 's' + Date.now() + '_' + counts.shapes, kind, type, name, notes, pts });
  counts.shapes++;
  return true;
}

renderShapes();

/* ---------- measure ----------
   Tap points to measure a path: total and straight-line distance, bearing, optional area, ground elevation at each
   point, climb and descent, steepest grade, and an elevation profile. Elevation comes from the same USGS terrain tiles
   as terrain following, so it works offline once the area's terrain is saved. */
const meas = { pts: [], area: false, active: false, min: false };
const measLayer = L.layerGroup().addTo(map);
const measEl = document.getElementById('measure');
function startMeasure() { meas.pts = []; meas.area = false; meas.active = true; setTool('measure'); renderMeasure(); }
function stopMeasure() { meas.active = false; meas.pts = []; measLayer.clearLayers(); measEl.hidden = true; }
function measureAdd(p) { meas.pts.push(p); renderMeasure(); }
function segDist(a, b) { return hav(a, b); }
function measProfile(n) {
  // sample the path evenly for the profile; returns [{d, e}] or null while terrain loads
  const P = meas.pts; if (P.length < 2) return [];
  const legs = []; let total = 0;
  for (let i = 1; i < P.length; i++) { const d = segDist(P[i - 1], P[i]); legs.push(d); total += d; }
  if (meas.area && P.length > 2) { const d = segDist(P[P.length - 1], P[0]); legs.push(d); total += d; }
  const pts = meas.area && P.length > 2 ? P.concat([P[0]]) : P;
  const out = [], step = Math.max(2, total / n);
  let acc = 0;
  for (let i = 1; i < pts.length; i++) {
    const a = pts[i - 1], b = pts[i], L0 = legs[i - 1];
    for (let d = Math.ceil(acc / step - 1e-9) * step - acc; d < L0; d += step) {
      const f = L0 ? d / L0 : 0, e = elevAt(a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f);
      if (e == null) return null;
      out.push({ d: acc + d, e });
    }
    acc += L0;
  }
  const last = pts[pts.length - 1], el = elevAt(last[0], last[1]);
  if (el == null) return null;
  out.push({ d: acc, e: el });
  return out;
}
function fmtElev(m) { return imp() ? r0(m * FT).toLocaleString() + ' ft' : r0(m).toLocaleString() + ' m'; }
function fmtArea(m2) { return imp() ? (m2 / 4046.86 >= 10 ? r0(m2 / 4046.86) : r1(m2 / 4046.86)) + ' acres' : (m2 / 10000 >= 10 ? r0(m2 / 10000) : r1(m2 / 10000)) + ' ha'; }
function renderMeasure() {
  measLayer.clearLayers();
  if (!meas.active) { measEl.hidden = true; return; }
  const P = meas.pts, closed = meas.area && P.length > 2;
  if (P.length > 1) {
    if (closed) L.polygon(P, { color: '#ffd43b', weight: 3, fillOpacity: 0.15, interactive: false }).addTo(measLayer);
    else L.polyline(P, { color: '#ffd43b', weight: 3, interactive: false }).addTo(measLayer);
    const segs = P.slice(1).map((p, i) => [P[i], p]);
    if (closed) segs.push([P[P.length - 1], P[0]]);
    if (segs.length <= 30) segs.forEach(([a, b]) => {
      const mid = [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2];
      L.marker(mid, { interactive: false, icon: L.divIcon({ className: '', html: `<div class="mlabel">${fmtDist(segDist(a, b))}</div>`, iconSize: [90, 20], iconAnchor: [45, 10] }) }).addTo(measLayer);
    });
  }
  P.forEach((p, i) => {
    const mk = L.marker(p, { draggable: true, zIndexOffset: 1200, icon: L.divIcon({ className: '', html: `<div class="mpt">${i + 1}</div>`, iconSize: [22, 22], iconAnchor: [11, 11] }) }).addTo(measLayer);
    mk.on('drag', e => { P[i] = [e.latlng.lat, e.latlng.lng]; });
    mk.on('dragend', () => renderMeasure());
  });
  // numbers
  let total = 0; for (let i = 1; i < P.length; i++) total += segDist(P[i - 1], P[i]);
  const perim = closed ? total + segDist(P[P.length - 1], P[0]) : total;
  let st = 'ok';
  if (P.length) {
    const lats = P.map(p => p[0]), lngs = P.map(p => p[1]);
    st = demEnsure(Math.min(...lats), Math.min(...lngs), Math.max(...lats), Math.max(...lngs));
  }
  let h = `<div class="mhead"><b>Measure</b><div class="mbtns">
    <button class="btn small" data-ms="min" aria-label="${meas.min ? 'Show details' : 'Hide details'}">${meas.min ? '▴' : '▾'}</button>
    <button class="btn small${meas.area ? ' primary' : ''}" data-ms="area">Area</button>
    <button class="btn small" data-ms="undo"${P.length ? '' : ' disabled'}>Undo</button>
    <button class="btn small" data-ms="clear"${P.length ? '' : ' disabled'}>Clear</button>
    <button class="btn small" data-ms="done">Done</button></div></div>`;
  if (!P.length) h += `<p class="note" style="margin:6px 0 0">Tap the map to add points. Drag a point to move it.</p>`;
  else {
    const stats = [];
    if (P.length === 1) stats.push(['Location', P[0][0].toFixed(5) + ', ' + P[0][1].toFixed(5)]);
    else {
      stats.push([closed ? 'Perimeter' : 'Distance', fmtDist(perim)]);
      if (closed) stats.push(['Area', fmtArea(polyArea(P.map(projector(centroid(P)).xy)))]);
      else {
        stats.push(['Straight line', fmtDist(segDist(P[0], P[P.length - 1]))]);
        stats.push(['Bearing', r0((bearing({ lat: P[0][0], lng: P[0][1] }, { lat: P[P.length - 1][0], lng: P[P.length - 1][1] }) + 360) % 360) + '°']);
      }
    }
    let prof = null;
    if (st === 'ok') {
      const e0 = elevAt(P[0][0], P[0][1]);
      if (P.length === 1) stats.push(['Ground elevation', e0 == null ? '—' : fmtElev(e0)]);
      else {
        prof = measProfile(160);
        if (prof && prof.length > 1) {
          let up = 0, down = 0, maxG = 0, ref = prof[0].e, lo = Infinity, hi = -Infinity;
          prof.forEach((q, i) => {
            lo = Math.min(lo, q.e); hi = Math.max(hi, q.e);
            if (Math.abs(q.e - ref) >= 1) { if (q.e > ref) up += q.e - ref; else down += ref - q.e; ref = q.e; }
            if (i) { const dd = q.d - prof[i - 1].d; if (dd > 5) maxG = Math.max(maxG, Math.abs(q.e - prof[i - 1].e) / dd); }
          });
          stats.push(['Start / end elevation', fmtElev(prof[0].e) + ' / ' + fmtElev(prof[prof.length - 1].e)]);
          stats.push(['Lowest / highest', fmtElev(lo) + ' / ' + fmtElev(hi)]);
          stats.push(['Climb / descent', '↑ ' + fmtElev(up) + ' · ↓ ' + fmtElev(down)]);
          stats.push(['Steepest grade', r0(maxG * 100) + '%']);
        }
      }
    } else stats.push(['Elevation', st === 'loading' ? 'Loading…' : 'Not available here. Connect to the internet or save this area’s terrain in Offline maps.']);
    h += '<div class="mstats">' + (meas.min ? stats.slice(0, 1) : stats).map(([k, v]) => `<span>${k}</span><b>${v}</b>`).join('') + '</div>';
    if (prof && prof.length > 1 && !meas.min) h += profileSvg(prof);
  }
  measEl.innerHTML = h; measEl.hidden = false;
}
function profileSvg(prof) {
  const W = 600, H = 110, padL = 46, padB = 18, padT = 8, padR = 8;
  const lo = Math.min(...prof.map(q => q.e)), hi = Math.max(...prof.map(q => q.e));
  const span = Math.max(hi - lo, 3), dmax = prof[prof.length - 1].d || 1;
  const x = d => padL + (W - padL - padR) * d / dmax, y = e => padT + (H - padT - padB) * (1 - (e - lo) / span);
  const line = prof.map((q, i) => (i ? 'L' : 'M') + x(q.d).toFixed(1) + ' ' + y(q.e).toFixed(1)).join(' ');
  const area = line + ` L${x(dmax).toFixed(1)} ${H - padB} L${padL} ${H - padB} Z`;
  return `<svg class="mprof" viewBox="0 0 ${W} ${H}" preserveAspectRatio="none" role="img" aria-label="Elevation profile">
    <path d="${area}" fill="var(--accent)" fill-opacity="0.18"/>
    <path d="${line}" fill="none" stroke="var(--accent)" stroke-width="2" vector-effect="non-scaling-stroke"/>
    <line x1="${padL}" y1="${padT}" x2="${W - padR}" y2="${padT}" stroke="var(--line)" stroke-dasharray="3 4"/>
    <line x1="${padL}" y1="${H - padB}" x2="${W - padR}" y2="${H - padB}" stroke="var(--line)"/>
    <text x="${padL - 6}" y="${padT + 4}" text-anchor="end" font-size="11" fill="var(--muted)">${fmtElev(lo + span)}</text>
    <text x="${padL - 6}" y="${H - padB}" text-anchor="end" font-size="11" fill="var(--muted)">${fmtElev(lo)}</text>
    <text x="${padL}" y="${H - 4}" font-size="11" fill="var(--muted)">0</text>
    <text x="${W - padR}" y="${H - 4}" text-anchor="end" font-size="11" fill="var(--muted)">${fmtDist(dmax)}</text></svg>`;
}
measEl.addEventListener('click', e => {
  const b = e.target.closest('[data-ms]'); if (!b || b.disabled) return;
  const a = b.dataset.ms;
  if (a === 'min') meas.min = !meas.min;
  else if (a === 'area') meas.area = !meas.area;
  else if (a === 'undo') meas.pts.pop();
  else if (a === 'clear') meas.pts = [];
  else if (a === 'done') { setTool(null); return; }
  renderMeasure();
});


/* ---------- photos ----------
   Photos attached to markers and boundaries. Each photo is resized (longest side 1280 px) and kept in the app's
   IndexedDB store, which has far more room than the rest of the app's storage; the original stays where it was.
   Features list their photo IDs, and the resized copies travel in the sync file. A photo no feature uses any more is
   removed from this device and from the sync file. */
const PH_MAX = 1280, PH_Q = 0.78, PH_THUMB = 200;
const ICON_CAM = '<svg viewBox="0 0 24 24" width="22" height="22" fill="none" stroke="currentColor" stroke-width="2"><path d="M4 8h3l2-3h6l2 3h3v11H4z"/><circle cx="12" cy="13" r="3.5"/></svg>';
let phDbP = null, phView = null;
function phDb() {
  if (!phDbP) phDbP = new Promise((res, rej) => {
    const r = indexedDB.open('planner-photos', 1);
    r.onupgradeneeded = () => r.result.createObjectStore('p', { keyPath: 'id' });
    r.onsuccess = () => res(r.result); r.onerror = () => rej(r.error);
  });
  return phDbP;
}
function phTx(mode, fn) {
  return phDb().then(d => new Promise((res, rej) => {
    const tx = d.transaction('p', mode), rq = fn(tx.objectStore('p'));
    tx.oncomplete = () => res(rq && rq.result); tx.onerror = () => rej(tx.error); tx.onabort = () => rej(tx.error || new Error('storage is full'));
  }));
}
const phGet = id => phTx('readonly', s => s.get(id));
const phPut = rec => phTx('readwrite', s => s.put(rec));
const phDel = id => phTx('readwrite', s => s.delete(id));
const phKeys = () => phTx('readonly', s => s.getAllKeys());

/* GPS position and time from a JPEG's EXIF block. */
function exifInfo(buf) {
  const v = new DataView(buf), out = {};
  if (v.byteLength < 4 || v.getUint16(0) !== 0xFFD8) return out;
  let p = 2;
  while (p + 10 < v.byteLength) {
    const mk = v.getUint16(p), len = v.getUint16(p + 2);
    if (mk === 0xFFE1 && v.getUint32(p + 4) === 0x45786966) { try { exifTiff(v, p + 10, out); } catch (e) {} break; }
    if ((mk & 0xFF00) !== 0xFF00 || mk === 0xFFDA) break;
    p += 2 + len;
  }
  return out;
}
function exifTiff(v, t, out) {
  const le = v.getUint16(t) === 0x4949, u16 = o => v.getUint16(t + o, le), u32 = o => v.getUint32(t + o, le);
  const ifd = (off, cb) => {
    if (!off || t + off + 2 > v.byteLength) return;
    const n = u16(off);
    for (let i = 0; i < n; i++) { const e = off + 2 + i * 12; if (t + e + 12 > v.byteLength) return; cb(u16(e), u16(e + 2), u32(e + 4), e + 8); }
  };
  const rats = (vo, k) => { const off = u32(vo), a = []; for (let i = 0; i < k; i++) { const d = u32(off + i * 8 + 4); a.push(d ? u32(off + i * 8) / d : 0); } return a; };
  const str = (vo, n) => { const off = n > 4 ? u32(vo) : vo; let s = ''; for (let i = 0; i < n - 1 && t + off + i < v.byteLength; i++) s += String.fromCharCode(v.getUint8(t + off + i)); return s; };
  let gps = 0, ex = 0, dt0 = '';
  ifd(u32(4), (tag, type, n, vo) => { if (tag === 0x8825) gps = u32(vo); else if (tag === 0x8769) ex = u32(vo); else if (tag === 0x0132) dt0 = str(vo, n); });
  ifd(ex, (tag, type, n, vo) => { if (tag === 0x9003) out.dt = str(vo, n); });
  const g = {};
  ifd(gps, (tag, type, n, vo) => {
    if (tag === 1 || tag === 3) g[tag] = String.fromCharCode(v.getUint8(t + vo));
    else if ((tag === 2 || tag === 4) && type === 5 && n === 3) g[tag] = rats(vo, 3);
  });
  if (g[2] && g[4]) {
    let lat = g[2][0] + g[2][1] / 60 + g[2][2] / 3600, lng = g[4][0] + g[4][1] / 60 + g[4][2] / 3600;
    if (g[1] === 'S') lat = -lat; if (g[3] === 'W') lng = -lng;
    if (isFinite(lat) && isFinite(lng) && (lat || lng) && Math.abs(lat) <= 90 && Math.abs(lng) <= 180) { out.lat = lat; out.lng = lng; }
  }
  const m = /^(\d{4}):(\d\d):(\d\d) (\d\d):(\d\d):(\d\d)/.exec(out.dt || dt0);
  if (m) out.taken = new Date(+m[1], m[2] - 1, +m[3], +m[4], +m[5], +m[6]).getTime();
}
async function phProcess(file) {
  const buf = await file.arrayBuffer();
  const ex = exifInfo(buf);
  let bmp;
  try { bmp = await createImageBitmap(new Blob([buf], { type: file.type || 'image/jpeg' }), { imageOrientation: 'from-image' }); }
  catch (e) { throw new Error(/hei[cf]/i.test((file.type || '') + file.name) ? 'HEIC photos can’t be read. Set the camera to save JPEG, or share the photo as JPEG.' : 'couldn’t read ' + (file.name || 'a photo')); }
  const sc = Math.min(1, PH_MAX / Math.max(bmp.width, bmp.height)), w = Math.round(bmp.width * sc), h = Math.round(bmp.height * sc);
  const cv = document.createElement('canvas'); cv.width = w; cv.height = h;
  cv.getContext('2d').drawImage(bmp, 0, 0, w, h);
  if (bmp.close) bmp.close();
  const blob = await new Promise(r => cv.toBlob(r, 'image/jpeg', PH_Q));
  const ts = Math.min(1, PH_THUMB / Math.max(w, h)), tc = document.createElement('canvas');
  tc.width = Math.max(1, Math.round(w * ts)); tc.height = Math.max(1, Math.round(h * ts));
  tc.getContext('2d').drawImage(cv, 0, 0, tc.width, tc.height);
  return { id: 'p' + Date.now().toString(36) + Math.random().toString(36).slice(2, 7), blob, thumb: tc.toDataURL('image/jpeg', 0.7), w, h,
    lat: ex.lat != null ? ex.lat : null, lng: ex.lng != null ? ex.lng : null, taken: ex.taken || file.lastModified || Date.now(), name: file.name || '', added: Date.now() };
}
function phPick() {
  return new Promise(res => {
    const i = document.createElement('input');
    i.type = 'file'; i.accept = 'image/*'; i.multiple = true; i.style.display = 'none';
    i.onchange = () => { res(Array.from(i.files || [])); i.remove(); };
    document.body.appendChild(i); i.click();
  });
}
function phFeature(kind, id) { return kind === 's' ? shapes.find(x => x.id === id) : markers.find(x => x.id === id); }
function phSaveFeature(kind) { if (kind === 's') { shSave(); renderShapes(); } else { mkSave(); renderMarkers(); } }
function phRefreshSheet() {
  if (sheet.hidden) return;
  if (sheetMode === 'marker') renderMarker(); else if (sheetMode === 'shape') renderShapeEditor(); else if (sheetMode === 'markers') renderMarkerList();
}
function phStrip(rec, kind) {
  const ids = rec.photos || [];
  return `<div class="phstrip">${ids.map(id => `<button class="phth" data-ph="${id}" data-pk="${kind}" data-pf="${rec.id}"><img data-thumb="${id}" alt=""></button>`).join('')}` +
    `<button class="phadd" data-phadd="${kind}" data-pf="${rec.id}">${ICON_CAM}<span>Add photo</span>${DESKTOP || PC ? '<small class="phdrop">or drag in</small>' : ''}</button></div>`;
}
function phFillThumbs(root) {
  root.querySelectorAll('img[data-thumb]').forEach(img => phGet(img.dataset.thumb).then(r => {
    if (r) img.src = r.thumb; else img.parentNode.classList.add('missing');
  }).catch(() => {}));
}
async function phAddTo(kind, fid, given) {
  const files = given || await phPick(); if (!files.length) return;
  toast(files.length > 1 ? 'Adding ' + files.length + ' photos…' : 'Adding photo…');
  let ok = 0, err = '';
  for (const f of files) {
    try {
      const r = await phProcess(f); await phPut(r);
      const rec = phFeature(kind, fid); if (!rec) break;
      (rec.photos = rec.photos || []).push(r.id); ok++;
    } catch (e) { err = (e && e.message) || 'couldn’t add a photo'; }
  }
  if (ok) phSaveFeature(kind);
  phRefreshSheet();
  toast(err ? (ok ? 'Added ' + ok + '. ' : '') + 'Problem: ' + err : 'Added ' + ok + ' photo' + (ok === 1 ? '' : 's'));
}
/* ----- drag and drop (PC): photo files, or a picture dragged from a browser such as Google Photos ----- */
function phDropTarget() {
  if (sheet.hidden) return null;
  const b = sheet.querySelector('[data-phadd]'); if (!b) return null;
  const rec = phFeature(b.dataset.phadd, b.dataset.pf);
  return rec ? { kind: b.dataset.phadd, fid: b.dataset.pf, name: rec.name || (b.dataset.phadd === 's' ? shType(rec).label : 'this marker') } : null;
}
function phDropUrl(dt) {
  let url = '';
  const html = dt.getData('text/html') || '', m = html.match(/<img[^>]+src=["']([^"']+)["']/i);
  if (m) url = m[1].replace(/&amp;/g, '&');
  if (!url) url = (dt.getData('text/uri-list') || '').split(/\r?\n/).find(l => l && l[0] !== '#') || '';
  if (!url) url = (dt.getData('text/plain') || '').trim();
  if (!/^https?:\/\/\S+$/.test(url)) return '';
  // Google Photos: ask for the full-size original instead of the screen-sized copy.
  if (/googleusercontent\.com\//.test(url)) url = /=[\w-]+(\?.*)?$/.test(url) ? url.replace(/=[\w-]+(\?.*)?$/, '=d$1') : url + '=d';
  return url;
}
async function phFromDrop(dt) {
  const files = Array.from(dt.files || []).filter(f => /^image\//.test(f.type) || /\.(jpe?g|png|heic|heif|webp|gif)$/i.test(f.name));
  if (files.length) return files;
  if (Array.from(dt.files || []).length) throw new Error('only photos can be dropped here');
  const url = phDropUrl(dt);
  if (!url) throw new Error('nothing usable was dropped. Drag the photo itself, or save it first and drop the file');
  let r;
  try { r = await fetch(url, { credentials: 'omit' }); } catch (e) { throw new Error('couldn’t download that picture. Save it first, then drop the file'); }
  if (!r.ok) throw new Error('the site refused the download (' + r.status + '). Save the photo first, then drop the file');
  const blob = await r.blob();
  if (!/^image\//.test(blob.type)) throw new Error(/photos\.google\.com/.test(url) ? 'that was a link to the Google Photos page. Open the photo, then drag the picture itself' : 'that link isn’t a picture');
  const ext = (blob.type.split('/')[1] || 'jpg').replace('jpeg', 'jpg');
  return [new File([blob], 'dropped-photo.' + ext, { type: blob.type, lastModified: Date.now() })];
}
const dropEl = document.createElement('div'); dropEl.id = 'dropzone'; dropEl.hidden = true; document.body.appendChild(dropEl);
let dropDepth = 0;
const dropOk = e => { const t = e.dataTransfer && e.dataTransfer.types; return t && Array.from(t).some(x => x === 'Files' || x === 'text/uri-list' || x === 'text/html'); };
document.addEventListener('dragenter', e => {
  if (!dropOk(e)) return; e.preventDefault(); dropDepth++;
  const t = phDropTarget();
  dropEl.innerHTML = `<div>${ICON_CAM}<b>${t ? 'Drop to add to ' + esc(t.name) : 'Drop photos to place them on the map'}</b><small>${t ? 'Photo files, or a picture dragged from Google Photos in your browser' : 'Uses the GPS location saved in each photo. To attach a photo to a feature, open the feature first.'}</small></div>`;
  dropEl.hidden = false;
});
document.addEventListener('dragover', e => { if (dropOk(e)) { e.preventDefault(); e.dataTransfer.dropEffect = 'copy'; } });
document.addEventListener('dragleave', e => { if (!dropOk(e)) return; if (--dropDepth <= 0) { dropDepth = 0; dropEl.hidden = true; } });
document.addEventListener('drop', async e => {
  if (!dropOk(e)) return;
  e.preventDefault(); dropDepth = 0; dropEl.hidden = true;
  const t = phDropTarget();
  let files;
  try { files = await phFromDrop(e.dataTransfer); } catch (err) { toast('Problem: ' + err.message); return; }
  if (t) phAddTo(t.kind, t.fid, files); else phToMarkers(files);
});
/* ----- viewer ----- */
const pvEl = document.getElementById('photoview');
function phOpen(kind, fid, id) {
  const rec = phFeature(kind, fid);
  const ids = rec && rec.photos ? rec.photos.slice() : [id];
  phView = { kind, fid, ids, i: Math.max(0, ids.indexOf(id)), armed: false, url: '' };
  phRender();
}
function phClose() { if (phView && phView.url) URL.revokeObjectURL(phView.url); phView = null; pvEl.hidden = true; pvEl.innerHTML = ''; }
async function phRender() {
  if (!phView) return phClose();
  const id = phView.ids[phView.i];
  let r = null; try { r = await phGet(id); } catch (e) {}
  if (!phView) return;
  if (phView.url) URL.revokeObjectURL(phView.url);
  phView.url = r ? URL.createObjectURL(r.blob) : '';
  const when = r && r.taken ? new Date(r.taken).toLocaleString([], { month: 'short', day: 'numeric', year: 'numeric', hour: 'numeric', minute: '2-digit' }) : '';
  const where = r && r.lat != null ? r.lat.toFixed(5) + ', ' + r.lng.toFixed(5) : '';
  pvEl.innerHTML = `<div class="pvbar"><span>${phView.ids.length > 1 ? (phView.i + 1) + ' of ' + phView.ids.length + ' · ' : ''}${[when, where].filter(Boolean).join(' · ') || 'Photo'}</span>
      ${r && r.lat != null ? '<button class="btn small" data-pv="map">Show on map</button>' : ''}
      <button class="btn small danger" data-pv="del">${phView.armed ? 'Tap again to remove' : 'Remove'}</button><button class="btn small primary" data-pv="close">Close</button></div>
    <div class="pvbody">${r ? `<img class="pvimg" src="${phView.url}" alt="">` : '<p class="pvmissing">This photo hasn’t reached this device yet. It arrives after the device that added it syncs.</p>'}</div>
    ${phView.ids.length > 1 ? '<button class="pvnav prev" data-pv="prev" aria-label="Previous">‹</button><button class="pvnav next" data-pv="next" aria-label="Next">›</button>' : ''}`;
  pvEl.hidden = false;
}
pvEl.addEventListener('click', async e => {
  const b = e.target.closest('[data-pv]');
  if (!b) { if (e.target === pvEl || e.target.classList.contains('pvbody')) phClose(); return; }
  const a = b.dataset.pv, v = phView; if (!v) return;
  if (a === 'close') return phClose();
  if (a === 'prev' || a === 'next') { v.i = (v.i + (a === 'next' ? 1 : v.ids.length - 1)) % v.ids.length; v.armed = false; return phRender(); }
  if (a === 'map') { const r = await phGet(v.ids[v.i]); if (r && r.lat != null) { phClose(); sheet.hidden = true; map.setView([r.lat, r.lng], Math.max(map.getZoom(), 18)); } return; }
  if (a === 'del') {
    if (!v.armed) { v.armed = true; return phRender(); }
    const id = v.ids[v.i], rec = phFeature(v.kind, v.fid);
    if (rec && rec.photos) { rec.photos = rec.photos.filter(x => x !== id); phSaveFeature(v.kind); }
    phDel(id).catch(() => {});
    v.ids.splice(v.i, 1); v.armed = false;
    phRefreshSheet();
    if (!v.ids.length) return phClose();
    v.i = Math.min(v.i, v.ids.length - 1); phRender();
  }
});
document.addEventListener('keydown', e => {
  if (!phView) return;
  if (e.key === 'Escape') phClose(); else if (e.key === 'ArrowRight') pvEl.querySelector('[data-pv=next]')?.click(); else if (e.key === 'ArrowLeft') pvEl.querySelector('[data-pv=prev]')?.click();
});
/* Handles photo buttons in any sheet before the sheet's own click handler. */
function phSheetClick(e) {
  const t = e.target.closest('[data-ph]'); if (t) { phOpen(t.dataset.pk, t.dataset.pf, t.dataset.ph); return true; }
  const a = e.target.closest('[data-phadd]'); if (a) { phAddTo(a.dataset.phadd, a.dataset.pf); return true; }
  return false;
}
/* ----- geotagged photos to markers ----- */
async function phToMarkers(given) {
  const files = given || await phPick(); if (!files.length) return;
  sheet.hidden = true; toast('Reading ' + files.length + ' photo' + (files.length === 1 ? '' : 's') + '…');
  const recs = []; let noGps = 0, bad = 0, err = '';
  for (const f of files) {
    try { const r = await phProcess(f); if (r.lat == null) { noGps++; continue; } await phPut(r); recs.push(r); }
    catch (e) { bad++; err = (e && e.message) || ''; }
  }
  recs.sort((a, b) => a.taken - b.taken);
  const groups = []; // shots taken within 15 m of each other share one marker
  recs.forEach(r => { const g = groups.find(x => hav([x.lat, x.lng], [r.lat, r.lng]) < 15); if (g) g.items.push(r); else groups.push({ lat: r.lat, lng: r.lng, items: [r] }); });
  let made = 0, joined = 0;
  groups.forEach((g, i) => {
    const ids = g.items.map(r => r.id), near = markers.find(m => hav([m.lat, m.lng], [g.lat, g.lng]) < 10);
    if (near) { near.photos = (near.photos || []).concat(ids); joined++; return; }
    const d = new Date(g.items[0].taken);
    markers.push({ id: 'k' + Date.now() + '_p' + i, type: 'landmark', name: 'Photo ' + d.toLocaleDateString([], { month: 'short', day: 'numeric' }),
      notes: 'From ' + (g.items.length === 1 ? 'a photo' : g.items.length + ' photos') + ' taken ' + d.toLocaleString([], { month: 'short', day: 'numeric', year: 'numeric', hour: 'numeric', minute: '2-digit' }) + '.',
      lat: g.lat, lng: g.lng, photos: ids });
    made++;
  });
  if (made || joined) {
    if (!mkVis.all) { mkVis.all = true; } delete mkVis.hidden.landmark; mkSaveVis();
    mkSave(); renderMarkers();
    const pts = groups.map(g => [g.lat, g.lng]);
    if (pts.length === 1) map.setView(pts[0], Math.max(map.getZoom(), 17)); else map.fitBounds(L.latLngBounds(pts).pad(0.2), { maxZoom: 18 });
  }
  const parts = [];
  if (made) parts.push('Added ' + made + ' marker' + (made === 1 ? '' : 's'));
  if (joined) parts.push((made ? 'added photos to ' : 'Added photos to ') + joined + ' existing marker' + (joined === 1 ? '' : 's'));
  if (noGps) parts.push(noGps + ' photo' + (noGps === 1 ? ' has' : 's have') + ' no location' + (DESKTOP ? '' : ' (if the app asks to see photo locations, allow it)'));
  if (bad) parts.push(bad + ' couldn’t be read' + (err ? ': ' + err : ''));
  toast((parts.join('; ') || 'No photos added').replace(/\.$/, '') + '.');
}
/* ----- sync ----- */
function phB64(blob) { return new Promise((res, rej) => { const fr = new FileReader(); fr.onload = () => res(String(fr.result).split(',')[1] || ''); fr.onerror = () => rej(fr.error); fr.readAsDataURL(blob); }); }
function phBlob(b64) { const s = atob(b64), a = new Uint8Array(s.length); for (let i = 0; i < s.length; i++) a[i] = s.charCodeAt(i); return new Blob([a], { type: 'image/jpeg' }); }
async function phToSync(r) { return { d: await phB64(r.blob), th: r.thumb, w: r.w, h: r.h, lat: r.lat, lng: r.lng, t: r.taken, n: r.name }; }
function phFromSync(id, s) { return { id, blob: phBlob(s.d), thumb: s.th || '', w: s.w, h: s.h, lat: s.lat == null ? null : s.lat, lng: s.lng == null ? null : s.lng, taken: s.t, name: s.n || '', added: Date.now() }; }
function phWanted(sets) { const w = new Set(); ['markers', 'shapes'].forEach(k => (sets[k] || []).forEach(r => (r.photos || []).forEach(id => w.add(id)))); return w; }
/* Photos for the sync file: everything a feature still uses, from the file or from this device. */
async function syPhotos(R, merged) {
  const want = phWanted(merged.sets), have = new Set(await phKeys()), map2 = {};
  let got = 0, sent = 0, changed = false;
  for (const id of want) {
    if (R[id] && R[id].d) {
      map2[id] = R[id];
      if (!have.has(id)) { try { await phPut(phFromSync(id, R[id])); got++; } catch (e) {} }
    } else if (have.has(id)) {
      const r = await phGet(id);
      if (r) { map2[id] = await phToSync(r); sent++; changed = true; }
    }
  }
  if (Object.keys(R).some(id => !want.has(id))) changed = true;
  return { map: map2, got, sent, changed };
}
/* Removes photos on this device that no feature uses (kept for 10 minutes in case one is being added). */
async function phGC() {
  try {
    const want = phWanted({ markers, shapes }), keys = await phKeys();
    for (const id of keys) {
      if (want.has(id)) continue;
      const r = await phGet(id);
      if (r && Date.now() - (r.added || 0) > 600000) await phDel(id);
    }
  } catch (e) {}
}

/* ---------- flight hazards ----------
   Any boundary or line can be a hazard. "Fly over only" (power lines, towers, tall trees) means the route must stay at
   least the clearance margin above the hazard's top inside its keep-away distance. "No-fly" means the route must never
   enter it at any height. The finished route (after terrain following) is adjusted automatically: legs that would
   cross a no-fly zone detour around it, surveys skip no-fly zones, and legs over a fly-over hazard climb before it and
   come back down after. Whatever can't be fixed is reported and blocks sending until you confirm.
   Heights compare like for like: ground elevation (USGS) + the hazard's height vs. takeoff ground + the route height. */
function hzShapes() { return shapes.filter(s => (s.hazard === 'over' || s.hazard === 'nofly') && s.pts && s.pts.length >= (s.kind === 'area' ? 3 : 2)); }
function hzName(s) { return s.name || shType(s).label; }
/* Outline of a shape grown by r metres, in a local x/y frame (metres). Areas grow outward; lines become a band. */
function hzBuffer(xy, closed, r) {
  const n = xy.length, unit = (x, y) => { const l = Math.hypot(x, y) || 1; return [x / l, y / l]; };
  if (closed) {
    let a = 0; for (let i = 0; i < n; i++) { const p = xy[i], q = xy[(i + 1) % n]; a += p[0] * q[1] - q[0] * p[1]; }
    const P = a < 0 ? xy.slice().reverse() : xy;      // counter-clockwise, so the right-hand normal points outward
    if (r <= 0) return P.map(p => p.slice());
    return P.map((p, i) => {
      const pr = P[(i - 1 + n) % n], nx = P[(i + 1) % n];
      const e1 = unit(p[0] - pr[0], p[1] - pr[1]), e2 = unit(nx[0] - p[0], nx[1] - p[1]);
      const n1 = [e1[1], -e1[0]], n2 = [e2[1], -e2[0]], m = unit(n1[0] + n2[0], n1[1] + n2[1]);
      const k = Math.min(3, 1 / Math.max(0.34, m[0] * n1[0] + m[1] * n1[1]));
      return [p[0] + m[0] * r * k, p[1] + m[1] * r * k];
    });
  }
  const L = [], R = [];
  for (let i = 0; i < n; i++) {
    const pr = xy[Math.max(0, i - 1)], nx = xy[Math.min(n - 1, i + 1)];
    const e1 = i > 0 ? unit(xy[i][0] - pr[0], xy[i][1] - pr[1]) : unit(nx[0] - xy[i][0], nx[1] - xy[i][1]);
    const e2 = i < n - 1 ? unit(nx[0] - xy[i][0], nx[1] - xy[i][1]) : e1;
    const n1 = [-e1[1], e1[0]], n2 = [-e2[1], e2[0]], m = unit(n1[0] + n2[0], n1[1] + n2[1]);
    const k = Math.min(3, 1 / Math.max(0.34, m[0] * n1[0] + m[1] * n1[1]));
    let p = xy[i];
    if (i === 0) p = [p[0] - e2[0] * r, p[1] - e2[1] * r];                 // square caps past each end
    if (i === n - 1) p = [p[0] + e1[0] * r, p[1] + e1[1] * r];
    L.push([p[0] + m[0] * r * k, p[1] + m[1] * r * k]); R.push([p[0] - m[0] * r * k, p[1] - m[1] * r * k]);
  }
  return L.concat(R.reverse());
}
function hzInside(q, P) {
  let c = false;
  for (let i = 0, j = P.length - 1; i < P.length; j = i++) {
    const a = P[i], b = P[j];
    if ((a[1] > q[1]) !== (b[1] > q[1]) && q[0] < (b[0] - a[0]) * (q[1] - a[1]) / (b[1] - a[1]) + a[0]) c = !c;
  }
  return c;
}
function hzBox(P) { let a = Infinity, b = Infinity, c = -Infinity, d = -Infinity; P.forEach(p => { a = Math.min(a, p[0]); b = Math.min(b, p[1]); c = Math.max(c, p[0]); d = Math.max(d, p[1]); }); return [a, b, c, d]; }
function hzBoxHit(A, B, bx) { return !(Math.max(A[0], B[0]) < bx[0] || Math.min(A[0], B[0]) > bx[2] || Math.max(A[1], B[1]) < bx[1] || Math.min(A[1], B[1]) > bx[3]); }
/* Hazards in a local frame. extra grows each outline (used for detour paths and survey holes). */
function hzZones(pr, extra) {
  return hzShapes().map(s => {
    const d = HZ_DEFAULTS[s.hazard], buf = Math.max(0, s.buf != null ? s.buf : d.buf), top = Math.max(0, s.top != null ? s.top : d.top);
    const xy = s.pts.map(pr.xy);
    const Q = hzBuffer(xy, s.kind === 'area', buf), Qx = extra ? hzBuffer(xy, s.kind === 'area', buf + extra) : Q;
    return { s, kind: s.hazard, top, buf, Q, Qx, box: hzBox(Q), boxX: hzBox(Qx) };
  });
}
/* Survey holes: no-fly zones the grid lines should stop at (in the survey's own frame). */
function hzSurveyHoles(pr) { return S.hzAvoid === false ? [] : hzZones(pr, 4).filter(z => z.kind === 'nofly').map(z => z.Qx); }
/* Shortest way around polygon Q between where A→B enters and leaves it; null if A→B doesn't cross it. */
function hzDetour(A, B, Q) {
  const n = Q.length, hits = [];
  const dx = B[0] - A[0], dy = B[1] - A[1];
  for (let k = 0; k < n; k++) {
    const p = Q[k], q = Q[(k + 1) % n], ex = q[0] - p[0], ey = q[1] - p[1], den = dx * ey - dy * ex;
    if (Math.abs(den) < 1e-9) continue;
    const t = ((p[0] - A[0]) * ey - (p[1] - A[1]) * ex) / den, u = ((p[0] - A[0]) * dy - (p[1] - A[1]) * dx) / den;
    if (t > 1e-6 && t < 1 - 1e-6 && u >= 0 && u <= 1) hits.push({ t, k, pt: [A[0] + dx * t, A[1] + dy * t] });
  }
  if (hits.length < 2) return null;
  hits.sort((a, b) => a.t - b.t);
  const f = hits[0], l = hits[hits.length - 1];
  if (f.k === l.k) return null;
  const walk = dir => {
    const out = []; let i = dir > 0 ? (f.k + 1) % n : f.k;
    const stop = dir > 0 ? l.k : (l.k + 1) % n;
    for (let g = 0; g < n + 1; g++) { out.push(Q[i]); if (i === stop) break; i = (i + dir + n) % n; }
    return out;
  };
  const len = path => { let d = 0, prev = f.pt; path.forEach(p => { d += Math.hypot(p[0] - prev[0], p[1] - prev[1]); prev = p; }); return d + Math.hypot(l.pt[0] - prev[0], l.pt[1] - prev[1]); };
  const a = walk(1), b = walk(-1);
  return len(a) <= len(b) ? a : b;
}
const hzH = p => p.h != null ? p.h : wpAlt(p.w);
function hzPoint(a, b, A, B, q, pr, h) {
  const dx = B[0] - A[0], dy = B[1] - A[1], L2 = dx * dx + dy * dy || 1;
  const f = Math.max(0, Math.min(1, ((q[0] - A[0]) * dx + (q[1] - A[1]) * dy) / L2));
  return { w: newWp(pr.ll(q), { speed: wpSpeed(a.w), heading: 'route' }), gimbal: a.gimbal, orig: -1, extra: true, photo: false,
    h: h != null ? h : hzH(a) + (hzH(b) - hzH(a)) * f, seg: a.seg, stop: false, straight: false, hz: true };
}
/* Applies the hazard rules to the finished list of flight points and reports what it did and what is still wrong. */
function hzApply(P) {
  const list = hzShapes();
  if (!list.length || P.length < 2) return P;
  const info = { detours: 0, climbs: [], spots: [], problems: [], flat: false, loading: false, high: 0 };
  const pts = P.map(p => [p.w.lat, p.w.lng]);
  const pr = projector(centroid(pts));
  const zones = hzZones(pr, 3);
  // Ground heights: the mission's terrain data if terrain following is on; otherwise USGS elevation if it's available.
  let e0 = 0, ground = () => 0;
  if (P.terrainState === 'ok' && P.terrainInfo) { e0 = P.terrainInfo.e0; ground = (lat, lng) => elevAt(lat, lng); }
  else {
    const tk = (S.terrain && S.terrain.takeoff) || S.home || pts[0];
    const all = pts.concat([tk], ...list.map(s => s.pts));
    const st = demEnsure(Math.min(...all.map(p => p[0])), Math.min(...all.map(p => p[1])), Math.max(...all.map(p => p[0])), Math.max(...all.map(p => p[1])));
    if (st === 'ok') { e0 = elevAt(tk[0], tk[1]); ground = (lat, lng) => elevAt(lat, lng); }
    else { info.flat = true; info.loading = st === 'loading'; }
  }
  const margin = Math.max(0, S.hzMargin != null ? S.hzMargin : 10);
  let out = P.slice();
  // 1. Detour around no-fly zones.
  if (S.hzAvoid !== false) {
    const nf = zones.filter(z => z.kind === 'nofly');
    for (let pass = 0; pass < 4 && nf.length; pass++) {
      let changed = false; const res = [out[0]];
      for (let i = 1; i < out.length; i++) {
        const a = res[res.length - 1], b = out[i], A = pr.xy([a.w.lat, a.w.lng]), B = pr.xy([b.w.lat, b.w.lng]);
        for (const z of nf) {
          if (!hzBoxHit(A, B, z.boxX) || hzInside(A, z.Q) || hzInside(B, z.Q)) continue;
          const path = hzDetour(A, B, z.Qx);
          if (path && path.length) { path.forEach(q => res.push(hzPoint(a, b, A, B, q, pr))); changed = true; info.detours++; break; }
        }
        res.push(b);
      }
      out = res; if (!changed) break;
    }
  }
  // 2. Climb over fly-over hazards.
  const ov = zones.filter(z => z.kind === 'over');
  const reqAt = (q, ll) => { let r = -Infinity, who = null; for (const z of ov) { if (hzInside(q, z.Q)) { const v = ground(ll[0], ll[1]) + z.top + margin - e0; if (v > r) { r = v; who = z; } } } return [r, who]; };
  if (S.hzClimb !== false && ov.length) {
    for (let pass = 0; pass < 3; pass++) {
      let changed = false; const res = [out[0]];
      for (let i = 1; i < out.length; i++) {
        const a = res[res.length - 1], b = out[i], A = pr.xy([a.w.lat, a.w.lng]), B = pr.xy([b.w.lat, b.w.lng]);
        if (!ov.some(z => hzBoxHit(A, B, z.box))) { res.push(b); continue; }
        const Lg = Math.hypot(B[0] - A[0], B[1] - A[1]), n = Math.max(2, Math.ceil(Lg / 4));
        let k1 = -1, k2 = -1, need = -Infinity, who = null;
        for (let k = 0; k <= n; k++) {
          const f = k / n, q = [A[0] + (B[0] - A[0]) * f, A[1] + (B[1] - A[1]) * f], [r, z] = reqAt(q, pr.ll(q));
          const alt = hzH(a) + (hzH(b) - hzH(a)) * f;
          if (r > alt + 0.3) { if (k1 < 0) k1 = k; k2 = k; if (r > need) { need = r; who = z; } }
        }
        if (k1 < 0) { res.push(b); continue; }
        const altIn = hzH(a) + (hzH(b) - hzH(a)) * k1 / n, ramp = Math.max(5, need - altIn);
        const dIn = k1 / n * Lg - ramp, dOut = k2 / n * Lg + ramp;
        const at = d => [A[0] + (B[0] - A[0]) * d / Lg, A[1] + (B[1] - A[1]) * d / Lg];
        if (dIn <= 1) a.h = Math.max(hzH(a), need); else res.push(hzPoint(a, b, A, B, at(dIn), pr, need));
        if (dOut >= Lg - 1) b.h = Math.max(hzH(b), need); else res.push(hzPoint(a, b, A, B, at(dOut), pr, need));
        const nm = hzName(who.s), prev = info.climbs.find(c => c.name === nm);
        if (prev) prev.to = Math.max(prev.to, need); else info.climbs.push({ name: nm, to: need });
        info.spots.push({ name: nm, to: need, from: altIn, top: need - margin, at: pr.ll(at((k1 + k2) / 2 / n * Lg)), a: pr.ll(at(Math.max(0, dIn))), b: pr.ll(at(Math.min(Lg, dOut))) });
        info.high = Math.max(info.high, need);
        changed = true; res.push(b);
      }
      out = res; if (!changed) break;
    }
  }
  // 3. Report whatever is still wrong (one entry per hazard).
  const seen = {};
  const report = (z, kind, ll, extra) => { const key = z.s.id + kind; if (seen[key]) return; seen[key] = 1; info.problems.push(Object.assign({ name: hzName(z.s), kind, at: ll }, extra || {})); };
  for (let i = 0; i < out.length; i++) {
    const a = out[i], A = pr.xy([a.w.lat, a.w.lng]);
    zones.forEach(z => { if (z.kind === 'nofly' && hzInside(A, z.Q)) report(z, 'inside', [a.w.lat, a.w.lng]); });
    if (i === out.length - 1) break;
    const b = out[i + 1], B = pr.xy([b.w.lat, b.w.lng]);
    if (!zones.some(z => hzBoxHit(A, B, z.box))) continue;
    const Lg = Math.hypot(B[0] - A[0], B[1] - A[1]), n = Math.max(2, Math.ceil(Lg / 4));
    for (let k = 0; k <= n; k++) {
      const f = k / n, q = [A[0] + (B[0] - A[0]) * f, A[1] + (B[1] - A[1]) * f], ll = pr.ll(q), alt = hzH(a) + (hzH(b) - hzH(a)) * f;
      zones.forEach(z => {
        if (!hzInside(q, z.Q)) return;
        if (z.kind === 'nofly') report(z, 'nofly', ll);
        else { const r = ground(ll[0], ll[1]) + z.top + margin - e0; if (r > alt + 0.5) report(z, 'low', ll, { alt, need: r }); }
      });
    }
  }
  for (const k of ['terrainState', 'terrainInfo']) if (P[k] !== undefined) out[k] = P[k];
  out.hazard = info;
  return out;
}
function hzWarns(mp, warns) {
  const hz = mp && mp.hazard; if (!hz) return;
  hz.climbs.forEach(c => warns.push(['info', `Climbs to ${fmtLen(c.to)} over ${esc(c.name)}, then back down.`]));
  hz.problems.forEach(p => warns.push(['bad', p.kind === 'low' ? `Route passes too low over ${esc(p.name)} (${fmtLen(p.alt)} vs. ${fmtLen(p.need)} needed). Turn on Climb over hazards or raise the altitude.`
    : p.kind === 'inside' ? `A waypoint is inside no-fly zone ${esc(p.name)}. Move it out.` : `Route enters no-fly zone ${esc(p.name)}. Turn on Route around no-fly zones or change the route.`]));
  if (hz.high > MAX_ALT + 0.5) warns.push(['bad', `Climbing over hazards takes the drone to ${fmtLen(hz.high)} above takeoff, over 400 ft.`]);
  if (hz.flat && !hz.loading) warns.push(['', 'Hazard heights assume flat ground (no elevation data here). Allow extra clearance.']);
}
/* Side view of the route: height above takeoff along the way, with the hazards it climbs over. */
function hzProfile(mp) {
  const hz = mp && mp.hazard; if (!hz || !hz.spots.length || mp.length < 2) return '';
  const d = [0]; for (let i = 1; i < mp.length; i++) d.push(d[i - 1] + hav([mp[i - 1].w.lat, mp[i - 1].w.lng], [mp[i].w.lat, mp[i].w.lng]));
  const total = d[d.length - 1] || 1, H = mp.map(hzH);
  const along = ll => { let best = Infinity, at = 0; const pr = projector(ll); for (let i = 1; i < mp.length; i++) { const A = pr.xy([mp[i - 1].w.lat, mp[i - 1].w.lng]), B = pr.xy([mp[i].w.lat, mp[i].w.lng]), dx = B[0] - A[0], dy = B[1] - A[1], L2 = dx * dx + dy * dy || 1, f = Math.max(0, Math.min(1, -(A[0] * dx + A[1] * dy) / L2)), x = A[0] + dx * f, y = A[1] + dy * f, r = Math.hypot(x, y); if (r < best) { best = r; at = d[i - 1] + (d[i] - d[i - 1]) * f; } } return at; };
  const top = Math.max(...H, ...hz.spots.map(c => c.to)) * 1.15 || 1, W = 320, Ht = 110, px = 34, py = 8;
  const X = v => px + (W - px - 6) * v / total, Y = v => Ht - 18 - (Ht - 18 - py) * Math.max(0, v) / top;
  let g = `<svg viewBox="0 0 ${W} ${Ht}" class="hzprof" role="img" aria-label="Route height profile">`;
  [0, top / 2 / 1.15, top / 1.15].forEach(v => { g += `<line x1="${px}" x2="${W - 6}" y1="${Y(v)}" y2="${Y(v)}" class="gl"/><text x="${px - 4}" y="${Y(v) + 3}" text-anchor="end">${r0(toLen(v))}</text>`; });
  hz.spots.forEach(c => { const x = X(along(c.at)); g += `<rect x="${x - 3}" y="${Y(c.top)}" width="6" height="${Y(0) - Y(c.top)}" class="hzbar"/><line x1="${x - 14}" x2="${x + 14}" y1="${Y(c.to)}" y2="${Y(c.to)}" class="need"/>`; });
  g += `<polyline points="${mp.map((p, i) => X(d[i]).toFixed(1) + ',' + Y(H[i]).toFixed(1)).join(' ')}" class="fl"/>`;
  g += `<text x="${px}" y="${Ht - 4}">0</text><text x="${W - 6}" y="${Ht - 4}" text-anchor="end">${fmtDist(total)}</text></svg>`;
  return `<div class="hzprofwrap">${g}<small>Height above takeoff (${LEN()}) along the route. Red bars are hazards, dashes the height needed to clear them.</small></div>`;
}
function hzSummary(mp) {
  const hz = mp && mp.hazard; if (!hz) return '';
  const parts = [];
  hz.climbs.forEach(c => parts.push(`Climbs to ${fmtLen(c.to)} over ${esc(c.name)}`));
  if (hz.detours) parts.push(`${hz.detours} detour${hz.detours === 1 ? '' : 's'} around no-fly zones`);
  if (gridInfo && gridInfo.holes) parts.push(`Survey skips ${gridInfo.holes} no-fly zone${gridInfo.holes === 1 ? '' : 's'} (no photos there)`);
  return parts.join(' · ');
}
/* After a hazard changes: rebuild surveys (they skip no-fly zones) and recheck the route. */
function hzChanged() {
  if (multi()) { segStore(); S.segs.forEach((sg, i) => { if (i !== S.seg && sg.mode === 'grid' && sg.shape) sg.wps = withSeg(sg, () => { regen(); return clone(S.wps); }); }); }
  if (S.mode === 'grid') regen();
  commit(false);
}

/* ---------- find hazards ----------
   Looks up power lines, towers and wind turbines around the mission in OpenStreetMap and the FAA
   Digital Obstacle File, and lets the pilot pick which to add as hazards. Neither source is complete. */
const HF_OVERPASS = 'https://overpass-api.de/api/interpreter';
const HF_DOF = 'https://services6.arcgis.com/ssFJjBXIUyZDrSYZ/arcgis/rest/services/Digital_Obstacle_File/FeatureServer/0/query';
const HF_TOWER_TYPES = /TOWER|TWR|MAST|ANTENNA|STACK|WINDMILL|POLE|CRANE|RIG/;
var hf = null;
const hfLayer = L.layerGroup().addTo(map);
function hfArea() {
  const P = (S.wps.length || multi()) ? allFlightPoints() : [];
  let box;
  if (P.length) {
    const la = P.map(p => p.w.lat), ln = P.map(p => p.w.lng), padY = 0.005, padX = 0.005 / Math.cos(la[0] * Math.PI / 180);
    box = [Math.min(...la) - padY, Math.min(...ln) - padX, Math.max(...la) + padY, Math.max(...ln) + padX];
  } else { const b = map.getBounds(); box = [b.getSouth(), b.getWest(), b.getNorth(), b.getEast()]; }
  return { box, mission: !!P.length };
}
function hfNum(v) { const m = String(v == null ? '' : v).match(/[\d.]+/); if (!m) return null; let n = parseFloat(m[0]); if (/ft|'/.test(String(v))) n /= FT; return n > 0 ? n : null; }
/* Long lines: keep only the stretches inside the search area (plus one point either side). */
function hfClip(pts, box) {
  const m = 0.002, B = [box[0] - m, box[1] - m, box[2] + m, box[3] + m];
  const inb = p => p[0] >= B[0] && p[0] <= B[2] && p[1] >= B[1] && p[1] <= B[3];
  const edge = (q, p) => {           // where the segment from inside point q towards p leaves the area
    let t = 1;
    [[0, B[0]], [0, B[2]], [1, B[1]], [1, B[3]]].forEach(([k, v]) => { const d = p[k] - q[k]; if (d) { const u = (v - q[k]) / d; if (u > 0 && u < t) t = u; } });
    return [q[0] + (p[0] - q[0]) * t, q[1] + (p[1] - q[1]) * t];
  };
  const runs = []; let cur = [];
  for (let i = 0; i < pts.length; i++) {
    const p = pts[i];
    if (inb(p)) { if (!cur.length && i > 0) cur.push(edge(p, pts[i - 1])); cur.push(p); }
    else if (cur.length) { cur.push(edge(cur[cur.length - 1], p)); runs.push(cur); cur = []; }
    else if (i > 0 && !inb(pts[i - 1])) {   // a span crossing the area with both ends outside
      const a = pts[i - 1], c = edge([(a[0] + p[0]) / 2, (a[1] + p[1]) / 2], a), d = edge([(a[0] + p[0]) / 2, (a[1] + p[1]) / 2], p), mid = [(a[0] + p[0]) / 2, (a[1] + p[1]) / 2];
      if (inb(mid)) runs.push([c, d]);
    }
  }
  if (cur.length >= 2) runs.push(cur);
  return runs.filter(r => r.length >= 2);
}
/* How a point obstacle is treated: tall towers (often guyed) are routed around; shorter ones flown over. */
function hfPointRule(h, tower, turbine, rotor) {
  if (turbine) return { hazard: 'nofly', top: h, buf: Math.max(60, (rotor || 120) / 2 + 30) };
  if (tower && h >= 45) return { hazard: 'nofly', top: h, buf: Math.min(150, Math.max(30, h * 0.8)) };
  return { hazard: 'over', top: h, buf: 15 };
}
function hfFromOsm(j, box) {
  const out = [];
  (j.elements || []).forEach(e => {
    const t = e.tags || {};
    if (e.type === 'way' && /^(line|minor_line)$/.test(t.power) && e.geometry) {
      const kv = Math.max(0, ...String(t.voltage || '').split(/[;,]/).map(Number).filter(isFinite)) / 1000;
      const major = t.power === 'line', h = hfNum(t.height) || (major ? (kv >= 100 ? 40 : 25) : 12);
      const nm = t.name || (major ? 'Transmission line' : 'Power line') + (kv ? ' ' + Math.round(kv) + ' kV' : '');
      hfClip(e.geometry.map(g => [g.lat, g.lon]), box).forEach((pts, k) => out.push({ id: 'osm:way/' + e.id + '#' + k, src: 'OpenStreetMap', kind: 'line', pts, name: nm, h, est: !hfNum(t.height), hazard: 'over', top: h, buf: major ? 25 : 15, what: 'Power line' }));
    } else if (e.type === 'node') {
      const turbine = t.power === 'generator';
      const rotor = hfNum(t['rotor:diameter']), hub = hfNum(t['hub:height']);
      const h = hfNum(t.height) || (hub && rotor ? hub + rotor / 2 : null) || (turbine ? 150 : 30);
      const known = !!(hfNum(t.height) || (hub && rotor));
      const what = turbine ? 'Wind turbine' : t.man_made === 'mast' || t.man_made === 'communications_tower' ? 'Communication tower' : t.man_made === 'chimney' ? 'Chimney' : 'Tower';
      out.push(Object.assign({ id: 'osm:node/' + e.id, src: 'OpenStreetMap', kind: 'pt', pt: [e.lat, e.lon], name: t.name || t.operator || what, h, est: !known, what }, hfPointRule(h, !turbine, turbine, rotor)));
    }
  });
  return out;
}
function hfFromDof(j) {
  return (j.features || []).map(f => {
    const a = f.properties || {}, g = f.geometry, ty = String(a.Type_Code || '').trim(), h = (Number(a.AGL) || 0) / FT;
    if (!g || g.type !== 'Point' || !h) return null;
    const turbine = /WIND/.test(ty), tower = HF_TOWER_TYPES.test(ty);
    const NAMES = { BLDG: 'Building', 'T-L TWR': 'Power line tower', 'TOWER': 'Tower', 'ANTENNA': 'Antenna', 'STACK': 'Smokestack', 'WINDMILL': 'Wind turbine', 'POLE': 'Pole', 'TANK': 'Tank', 'ELEVATOR': 'Grain elevator', 'SILO': 'Silo', 'BLDG-TWR': 'Building tower', 'CTRL TWR': 'Control tower', 'RIG': 'Rig', 'CRANE': 'Crane', 'LGTHOUSE': 'Lighthouse', 'MET': 'Weather tower', 'UTILITY POLE': 'Utility pole' };
    const what = NAMES[ty] || ty.toLowerCase().replace(/\b\w/g, c => c.toUpperCase());
    return Object.assign({ id: 'faa:' + a.OAS_Number, src: 'FAA obstacle file', kind: 'pt', pt: [g.coordinates[1], g.coordinates[0]], name: what + (a.City ? ' (' + String(a.City).trim().toLowerCase().replace(/\b\w/g, c => c.toUpperCase()) + ')' : ''), h, est: false, what, lit: a.Lighting && a.Lighting !== 'N' && a.Lighting !== 'U' }, hfPointRule(h, tower, turbine, null));
  }).filter(Boolean);
}
/* Same tower in both sources: keep the FAA height, the OSM name. */
function hfMerge(osm, dof) {
  const out = dof.slice();
  osm.forEach(o => {
    if (o.kind === 'pt') { const d = out.find(x => x.kind === 'pt' && x.src === 'FAA obstacle file' && hav(x.pt, o.pt) < 80); if (d) { d.id2 = o.id; if (o.name !== o.what) d.name = o.name; d.src = 'FAA + OpenStreetMap'; return; } }
    out.push(o);
  });
  const have = new Set(shapes.map(s => s.src).filter(Boolean));
  out.forEach(it => { it.dup = have.has(it.id) || (it.id2 && have.has(it.id2)); it.on = !it.dup; });
  return out.sort((a, b) => (a.kind === b.kind ? 0 : a.kind === 'line' ? -1 : 1) || b.h - a.h);
}
function hfButton() {
  const n = hf && hf.state === 'done' ? hf.items.filter(i => !i.dup).length : 0;
  return `<div class="btns"><button class="btn small${n ? ' primary' : ''}" data-act="hfind">${n ? 'Review found hazards (' + n + ')' : 'Find power lines & towers…'}</button></div>`;
}
function openFindHazards() {
  sheetMode = 'hfind';
  if (hf && hf.state !== 'error' && hf.state !== 'big') { renderHf(); sheet.hidden = false; return; }
  const a = hfArea();
  hf = { state: 'loading', box: a.box, mission: a.mission, items: [], osm: '', faa: '' };
  sheet.hidden = false;
  if (a.box[2] - a.box[0] > 0.3 || a.box[3] - a.box[1] > 0.35) { hf.state = 'big'; renderHf(); return; }
  renderHf(); hfFetch();
}
function hfTimed(url, opts, ms) {
  const ctl = typeof AbortController !== 'undefined' ? new AbortController() : null, t = setTimeout(() => ctl && ctl.abort(), ms);
  return fetch(url, Object.assign({}, opts, ctl ? { signal: ctl.signal } : {})).then(r => { clearTimeout(t); return r; }, e => { clearTimeout(t); throw ctl && ctl.signal.aborted ? new Error('timeout') : e; });
}
/* Both sources load on their own; the list shows as soon as either answers and fills in as the other arrives. */
function hfFetch(only) {
  const b = hf.box, bb = b.map(v => v.toFixed(5)).join(','), me = hf;
  const why = e => /timeout/.test(e && e.message) ? 'timed out (its free server is busy)' : /HTTP (429|50\d)/.test(e && e.message) ? 'busy right now' : 'couldn\'t reach';
  const done = () => {
    if (hf !== me) return;
    if (me.osm !== 'loading' && me.faa !== 'loading' && me.osmItems == null && me.faaItems == null) me.state = 'error';
    else if (me.osmItems != null || me.faaItems != null) {
      const keep = {}; (me.items || []).forEach(it => { keep[it.id] = it.on; });
      me.items = hfMerge(me.osmItems || [], me.faaItems || []);
      me.items.forEach(it => { if (!it.dup && it.id in keep) it.on = keep[it.id]; });
      me.nOsm = (me.osmItems || []).length; me.nFaa = (me.faaItems || []).length; me.state = 'done';
    }
    hfDraw(); if (sheetMode === 'hfind' && !sheet.hidden) renderHf(); else if (me.state === 'done') renderPanel();
  };
  if (only !== 'faa') {
    me.osm = 'loading'; me.osmItems = null;
    const q = `[out:json][timeout:50];(way["power"~"^(line|minor_line)$"](${bb});node["man_made"~"^(mast|tower|communications_tower|chimney)$"](${bb});node["power"="generator"]["generator:source"="wind"](${bb}););out geom tags;`;
    hfTimed(HF_OVERPASS, { method: 'POST', headers: { 'Content-Type': 'application/x-www-form-urlencoded' }, body: 'data=' + encodeURIComponent(q) }, 60000)
      .then(r => { if (!r.ok) throw new Error('HTTP ' + r.status); return r.json(); })
      .then(j => { if (hf !== me) return; me.osmItems = hfFromOsm(j, b); me.osm = ''; done(); }, e => { if (hf !== me) return; me.osm = why(e); done(); });
  }
  if (only !== 'osm') {
    me.faa = 'loading'; me.faaItems = null;
    hfTimed(HF_DOF + '?where=1%3D1&geometry=' + [b[1], b[0], b[3], b[2]].map(v => v.toFixed(5)).join(',') + '&geometryType=esriGeometryEnvelope&inSR=4326&spatialRel=esriSpatialRelIntersects&outFields=OAS_Number,City,Type_Code,AGL,Lighting&returnGeometry=true&outSR=4326&resultRecordCount=2000&f=geojson', {}, 30000)
      .then(r => r.json()).then(j => { if (j.error) throw new Error(j.error.message); return j; })
      .then(j => { if (hf !== me) return; me.faaItems = hfFromDof(j); me.faa = ''; done(); }, e => { if (hf !== me) return; me.faa = why(e); done(); });
  }
}
function hfDraw() {
  hfLayer.clearLayers();
  if (!hf || hf.state !== 'done') return;
  hf.items.forEach(it => {
    const col = it.dup ? '#adb5bd' : it.on ? '#fcc419' : '#868e96';
    if (it.kind === 'line') L.polyline(it.pts, { color: col, weight: 4, dashArray: '2 7', opacity: 0.95, interactive: false }).addTo(hfLayer);
    else L.circleMarker(it.pt, { radius: 7, color: '#000', weight: 1.5, fillColor: col, fillOpacity: 1, interactive: false }).addTo(hfLayer);
  });
}
function hfDesc(it) {
  const ht = (it.est ? 'about ' : '') + fmtLen(it.h) + (it.est ? ' (estimated)' : '');
  const len = it.kind === 'line' ? fmtDist(shLength(it.pts)) + ' in the area · ' : '';
  const how = it.hazard === 'nofly' ? 'route around, keep ' + fmtLen(it.buf) + ' away' : 'fly over';
  return `${esc(it.what)} · ${len}${ht} · ${how} · ${esc(it.src)}${it.dup ? ' · already added' : ''}`;
}
function renderHf() {
  let h = '<div class="card"><h2>Find power lines & towers</h2>';
  if (hf.state === 'big') h += `<p class="note">That's too large an area to search. ${hf.mission ? 'The mission covers a very wide area.' : 'Zoom the map in closer to the property,'} or draw the mission first; the search covers the mission plus about a third of a mile around it.</p><div class="btns"><button class="btn primary" data-hf="close">OK</button></div>`;
  else if (hf.state === 'loading') h += `<p class="note">Searching OpenStreetMap and the FAA obstacle file ${hf.mission ? 'around this mission' : 'in the map view'}…</p><div class="btns"><button class="btn" data-hf="cancel">Cancel</button></div>`;
  else if (hf.state === 'error') h += `<div class="status bad">Couldn't get results. OpenStreetMap ${esc(hf.osm)}; FAA ${esc(hf.faa)}. This needs internet; if you're online, the free servers may be busy, so try again in a few minutes.</div><div class="btns"><button class="btn" data-hf="cancel">Close</button><button class="btn primary" data-hf="retry">Try again</button></div>`;
  else {
    const n = hf.items.filter(i => i.on).length;
    const src = (st, n) => st === 'loading' ? '<i>still searching…</i>' : st || n + ' found';
    const retry = (hf.osm && hf.osm !== 'loading') || (hf.faa && hf.faa !== 'loading');
    h += `<p class="note">Searched ${hf.mission ? 'around this mission' : 'the map view'}. OpenStreetMap: ${src(hf.osm, hf.nOsm)} · FAA obstacle file: ${src(hf.faa, hf.nFaa)}.${hf.osm === 'loading' ? ' OpenStreetMap can take up to a minute when it\'s busy; you can add items from the list meanwhile.' : ''}</p>
      ${retry ? `<div class="btns" style="margin-bottom:8px"><button class="btn small" data-hf="retrysrc">Try ${hf.osm && hf.osm !== 'loading' ? 'OpenStreetMap' : 'the FAA'} again</button></div>` : ''}
      <div class="warn" style="margin-bottom:8px">⚠ Neither source is complete. Small lines to houses, wells and barns are often missing, and heights marked estimated are guesses. Look over the map and mark anything else yourself.</div>`;
    if (!hf.items.length) h += hf.osm === 'loading' ? '' : '<p class="note">Nothing found here. That doesn\'t mean there\'s nothing there.</p>';
    else h += '<div class="mlist">' + hf.items.map((it, i) => `<label class="mitem${it.on ? ' on' : ''}"><input type="checkbox" data-hfi="${i}"${it.on ? ' checked' : ''}${it.dup ? ' disabled' : ''}><div style="flex:1;min-width:0"><b>${esc(it.name)}</b><small style="font-family:var(--font)">${hfDesc(it)}</small></div><button class="btn small" data-hf="show" data-i="${i}">Show</button></label>`).join('') + '</div>';
    h += `<p class="note">Added items become hazards you can edit in Features (height, keep-away distance, fly over or no-fly).</p>
      <div class="btns"><button class="btn" data-hf="cancel">Cancel</button><button class="btn primary" data-hf="add"${n ? '' : ' disabled'}>Add ${n} hazard${n === 1 ? '' : 's'}</button></div>`;
  }
  sheet.innerHTML = h + '</div>';
}
function hfRing(c, r) { const pr = projector(c); return [0, 1, 2, 3, 4, 5, 6, 7].map(k => pr.ll([r * Math.cos(k * Math.PI / 4), r * Math.sin(k * Math.PI / 4)])); }
function hfAdd() {
  const add = hf.items.filter(i => i.on && !i.dup), t0 = Date.now();
  add.forEach((it, k) => {
    const notes = `Added from ${it.src}${it.est ? '. Height is an estimate: check it on the ground' : ''}${it.lit ? '. Lit at night' : ''}.`;
    const base = { id: 's' + t0 + '_' + k, name: it.name, notes, hazard: it.hazard, top: Math.round(it.top * 10) / 10, buf: Math.round(it.buf * 10) / 10, src: it.id };
    if (it.kind === 'line') shapes.push(Object.assign(base, { kind: 'line', type: 'power', pts: it.pts.map(p => [+p[0].toFixed(7), +p[1].toFixed(7)]) }));
    else shapes.push(Object.assign(base, { kind: 'area', type: 'tower', pts: hfRing(it.pt, 4).map(p => [+p[0].toFixed(7), +p[1].toFixed(7)]) }));
  });
  if (mkVis.shapes === false) { mkVis.shapes = true; mkSaveVis(); }
  hf = null; hfDraw(); sheet.hidden = true;
  shSave(); renderShapes(); commit(false);
  toast(`Added ${add.length} hazard${add.length === 1 ? '' : 's'}. Edit them in Features.`);
}
function hfClick(e) {
  if (e.target === sheet) { sheet.hidden = true; return; }
  const c = e.target.closest('input[data-hfi]');
  if (c) { const it = hf.items[+c.dataset.hfi]; it.on = c.checked; hfDraw(); renderHf(); return; }
  const b = e.target.closest('[data-hf]'); if (!b || b.disabled) return;
  e.preventDefault();
  const k = b.dataset.hf;
  if (k === 'close') { hf = null; sheet.hidden = true; }
  else if (k === 'cancel') { hf = null; hfDraw(); sheet.hidden = true; renderPanel(); }
  else if (k === 'retry') { hf = null; openFindHazards(); }
  else if (k === 'retrysrc') { hfFetch(hf.osm && hf.osm !== 'loading' ? 'osm' : 'faa'); renderHf(); }
  else if (k === 'add') hfAdd();
  else if (k === 'show') {
    const it = hf.items[+b.dataset.i]; if (!it) return;
    sheet.hidden = true;
    if (it.kind === 'line') map.fitBounds(L.latLngBounds(it.pts).pad(0.2)); else map.setView(it.pt, Math.max(map.getZoom(), 16));
    renderPanel(); toast('Tap “Review found hazards” in the panel to go back to the list');
  }
}

/* ---------- FAA airspace ----------
   Checks the mission against the FAA's public UAS data (facility maps, airspace classes, special use,
   national security areas, stadiums). Advisory only: the pilot still checks TFRs/NOTAMs and gets
   authorization through a LAANC provider or FAA DroneZone. */
const AIR_BASE = 'https://services6.arcgis.com/ssFJjBXIUyZDrSYZ/arcgis/rest/services/';
const AIR_LAYERS = {
  grid: ['FAA_UAS_FacilityMap_Data', 'CEILING,APT1_NAME,APT1_ICAO,APT1_FAAID,AIRSPACE_1', '1=1'],
  cls: ['Class_Airspace', 'NAME,CLASS,LOCAL_TYPE,LOWER_VAL,LOWER_CODE', "CLASS IN ('B','C','D','E') AND LOWER_VAL = 0"],
  sua: ['Special_Use_Airspace', 'NAME,TYPE_CODE,LOWER_VAL,LOWER_CODE,TIMESOFUSE', "TYPE_CODE IN ('P','R','MOA','W','A')"],
  pro: ['Prohibited_Areas', 'NAME,TYPE_CODE,LOWER_VAL,LOWER_CODE,TIMESOFUSE', '1=1'],
  nsr: ['Part_Time_National_Security_UAS_Flight_Restrictions', 'Facility,Base,Reason,Floor,Ceiling,ALERTYPE', '1=1'],
  nda: ['National_Defense_Airspace_TFR_Areas', 'NAME,WKHR_RMK', '1=1'],
  std: ['Stadiums', 'NAME,CITY,STATE', '1=1'],
};
const AIR_LINKS = {
  aloft: 'https://www.aloft.ai/air-control/',
  b4u: 'https://b4ufly.aloft.ai/',
  tfr: 'https://tfr.faa.gov/',
  dz: 'https://faadronezone-access.faa.gov/',
  laanc: 'https://www.faa.gov/uas/getting_started/laanc',
};
const STADIUM_R = 5556;                     // 3 nautical miles
var air = { key: '', state: 'idle', data: null, at: 0, cached: false }, airTimer = null, airEval = null, airAck = '';
function openUrl(u) {
  if (native && native.openUrl) native.openUrl(u);
  else window.open(u, '_blank', 'noopener');
}
function airPoints() {
  if (!air) return null;
  const P = missionPoints();
  return P.length >= 2 ? P : null;
}
/* Area to fetch: the route plus a little, snapped to a 0.01° grid so small edits reuse the result. */
function airBox(P) {
  let a = 90, b = 180, c = -90, d = -180;
  P.forEach(p => { a = Math.min(a, p.w.lat); b = Math.min(b, p.w.lng); c = Math.max(c, p.w.lat); d = Math.max(d, p.w.lng); });
  const s = 100, f = v => Math.floor(v * s - 0.3) / s, g = v => Math.ceil(v * s + 0.3) / s;
  return [f(a), f(b), g(c), g(d)];
}
function airUrl(k, box, pad) {
  const [name, fields, where] = AIR_LAYERS[k];
  const env = [box[1] - pad, box[0] - pad, box[3] + pad, box[2] + pad].map(v => v.toFixed(4)).join(',');
  return AIR_BASE + name + '/FeatureServer/0/query?where=' + encodeURIComponent(where) + '&geometry=' + env +
    '&geometryType=esriGeometryEnvelope&inSR=4326&spatialRel=esriSpatialRelIntersects&outFields=' + fields +
    '&returnGeometry=true&outSR=4326&resultRecordCount=2000&f=geojson';
}
/* GeoJSON polygon → list of polygons, each a list of [lat,lng] rings, with a bounding box. */
function airPolys(geom) {
  if (!geom) return [];
  const list = geom.type === 'Polygon' ? [geom.coordinates] : geom.type === 'MultiPolygon' ? geom.coordinates : [];
  return list.map(rings => {
    const R = rings.map(r => r.map(c => [c[1], c[0]]));
    return { R, box: hzBox(R[0] || []) };
  });
}
function airIn(ll, polys) {
  for (const p of polys) {
    if (ll[0] < p.box[0] || ll[0] > p.box[2] || ll[1] < p.box[1] || ll[1] > p.box[3]) continue;
    let n = 0; p.R.forEach(r => { if (hzInside(ll, r)) n++; });
    if (n % 2) return true;
  }
  return false;
}
function airPrep(raw) {
  const out = {};
  for (const k in raw) out[k] = (raw[k] || []).map(f => ({ a: f.properties || {}, polys: airPolys(f.geometry), pt: f.geometry && f.geometry.type === 'Point' ? [f.geometry.coordinates[1], f.geometry.coordinates[0]] : null }));
  return out;
}
function airLoadStore() { try { return JSON.parse(localStorage.getItem('planner.air')) || null; } catch (e) { return null; } }
function airSchedule() {
  clearTimeout(airTimer);
  airTimer = setTimeout(() => airRun(false), 1200);
}
function airRun(force) {
  const P = airPoints(); if (!P) return;
  const box = airBox(P), key = box.join(',');
  if (!force && air.key === key && (air.state === 'ok' || air.state === 'loading')) return;
  if (!force && air.key !== key) {
    const st = airLoadStore();
    if (st && st.key === key && st.raw) { air = { key, state: 'ok', data: airPrep(st.raw), at: st.at, cached: true }; airEval = null; airRedraw(); if (Date.now() - st.at < 12 * 3600e3) return; }
  }
  const tok = {}; air.tok = tok;
  if (air.key !== key || air.state !== 'ok') { air = { key, state: 'loading', data: null, at: 0, tok }; airEval = null; airRedraw(); }
  const ctl = typeof AbortController !== 'undefined' ? new AbortController() : null;
  const timer = setTimeout(() => ctl && ctl.abort(), 20000);
  const get = (k, pad) => fetch(airUrl(k, box, pad), ctl ? { signal: ctl.signal } : undefined).then(r => r.json()).then(j => {
    if (j.error) throw new Error(j.error.message || 'FAA data error');
    return j.features || [];
  });
  const keys = Object.keys(AIR_LAYERS);
  Promise.all(keys.map(k => get(k, k === 'std' ? 0.06 : 0))).then(res => {
    clearTimeout(timer);
    if (air.tok !== tok) return;
    const raw = {}; keys.forEach((k, i) => { raw[k] = res[i]; });
    air = { key, state: 'ok', data: airPrep(raw), at: Date.now(), cached: false };
    try { localStorage.setItem('planner.air', JSON.stringify({ key, at: air.at, raw })); } catch (e) { try { localStorage.removeItem('planner.air'); } catch (e2) {} }
    airEval = null; airRedraw();
  }).catch(() => {
    clearTimeout(timer);
    if (air.tok !== tok) return;
    if (air.state === 'ok') { air.stale = true; airRedraw(); return; }   // keep the older result we already have
    air = { key, state: 'error', data: null, at: 0 }; airEval = null; airRedraw();
  });
}
function airRedraw() { if (typeof renderPanel === 'function') { renderMap(); renderPanel(); } }
const AIR_RANK = { ok: 0, caution: 1, auth: 2, bad: 3 };
/* Check the route against the fetched data. */
function airCheck() {
  const P = airPoints();
  if (!P || air.state !== 'ok' || !air.data) return null;
  if (airEval && airEval.mp === P && airEval.data === air.data) return airEval.res;
  const D = air.data, mp = P;
  // Sample the route every ~100 m with an estimated height above ground.
  const e0 = mp.terrainInfo ? mp.terrainInfo.e0 : null;
  const agl = (p, ll) => { const h = hzH(p); if (e0 != null) { const g = elevAt(ll[0], ll[1]); if (g != null) return h + e0 - g; } return h; };
  const S2 = [];
  for (let i = 0; i < mp.length; i++) {
    const a = mp[i], A = [a.w.lat, a.w.lng];
    S2.push({ ll: A, h: agl(a, A) });
    if (i === mp.length - 1) break;
    const b = mp[i + 1], B = [b.w.lat, b.w.lng], n = Math.ceil(hav(A, B) / 100);
    for (let k = 1; k < n; k++) { const f = k / n, ll = [A[0] + (B[0] - A[0]) * f, A[1] + (B[1] - A[1]) * f]; S2.push({ ll, h: hzH(a) + (hzH(b) - hzH(a)) * f + (e0 != null && elevAt(ll[0], ll[1]) != null ? e0 - elevAt(ll[0], ll[1]) : 0) }); }
  }
  const maxFt = Math.max(...S2.map(s => s.h)) * FT;
  const items = [], shapes = [];
  const hit = f => { for (const s of S2) if (airIn(s.ll, f.polys)) return s; return null; };
  // Controlled airspace and facility-map ceilings.
  const cells = []; let cmin = Infinity, cmax = -Infinity, apt = '';
  D.grid.forEach(f => { const s = hit(f); if (!s) return; const c = Number(f.a.CEILING); cells.push({ f, c, s }); cmin = Math.min(cmin, c); cmax = Math.max(cmax, c); if (!apt) apt = f.a.APT1_NAME || f.a.APT1_ICAO || ''; });
  const areas = [];
  D.cls.forEach(f => { const s = hit(f); if (s && !areas.some(x => x.a.NAME === f.a.NAME)) { areas.push({ a: f.a, s }); shapes.push({ f, level: 'auth' }); } });
  if (areas.length || cells.length) {
    const names = areas.map(x => x.a.NAME).join(', ') || ('Class ' + (cells[0].f.a.AIRSPACE_1 || '') + ' airspace near ' + apt);
    const ceil = cells.length ? (cmin === cmax ? fmtLen(cmin / FT) : fmtLen(cmin / FT) + '–' + fmtLen(cmax / FT)) : '';
    let detail;
    if (!cells.length) detail = 'No LAANC grid covers this spot. Request authorization through FAA DroneZone.';
    else if (cmin <= 0) detail = `LAANC ceiling here is 0 ft${cmax > 0 ? ' in places (up to ' + fmtLen(cmax / FT) + ')' : ''}. Automatic approval isn't available; request further coordination through a LAANC app or FAA DroneZone (can take weeks).`;
    else if (maxFt > cmin + 1) detail = `LAANC ceiling here is ${ceil}; the mission reaches about ${fmtLen(maxFt / FT)} above the ground. Fly at ${fmtLen(cmin / FT)} or lower for instant approval, or request further coordination.`;
    else detail = `LAANC ceiling here is ${ceil}. The mission (about ${fmtLen(maxFt / FT)} above ground) fits, so a LAANC app can usually approve it in seconds.`;
    items.push({ level: 'auth', kind: 'ctl', title: 'Controlled airspace: ' + names, detail, at: (areas[0] || cells[0]).s.ll });
  }
  // Special use airspace and prohibited areas.
  const seen = {};
  D.sua.concat(D.pro).forEach(f => {
    const a = f.a, t = String(a.TYPE_CODE || ''), nm = a.NAME || t; if (seen[nm]) return;
    const s = hit(f); if (!s) return; seen[nm] = 1;
    const lo = Number(a.LOWER_VAL) || 0, msl = a.LOWER_CODE === 'MSL';
    let gFt = null; if (msl) { const g = elevAt(s.ll[0], s.ll[1]); if (g != null) gFt = g * FT; }
    const floorAgl = msl ? (gFt != null ? lo - gFt : null) : lo;
    const reaches = floorAgl == null ? lo < 3000 : floorAgl <= maxFt + 100;
    const when = a.TIMESOFUSE ? ' Active: ' + String(a.TIMESOFUSE).toLowerCase() + '.' : '';
    if (t === 'P') { items.push({ level: 'bad', kind: 'sua', title: 'Prohibited area ' + nm, detail: 'Flight is not allowed here.', at: s.ll }); shapes.push({ f, level: 'bad' }); }
    else if (t === 'R') {
      if (!reaches) return;
      items.push({ level: 'bad', kind: 'sua', title: 'Restricted area ' + nm, detail: 'Don\'t fly while it\'s active without the controlling agency\'s permission.' + when, at: s.ll }); shapes.push({ f, level: 'bad' });
    } else {
      if (floorAgl == null ? lo > 3000 : floorAgl > 1500) return;
      const kind = t === 'MOA' ? 'Military operations area' : t === 'W' ? 'Warning area' : 'Alert area';
      items.push({ level: 'caution', kind: 'sua', title: kind + ': ' + nm, detail: 'No authorization needed, but watch for fast, low-flying aircraft.' + when, at: s.ll }); shapes.push({ f, level: 'caution' });
    }
  });
  D.nsr.forEach(f => {
    const s = hit(f); if (!s) return; const a = f.a, part = /intermittent/i.test(a.ALERTYPE || '');
    items.push({ level: part ? 'caution' : 'bad', kind: 'nsr', title: 'National security flight restriction: ' + (a.Facility || a.Base || ''), detail: (part ? 'Part-time restriction, active by NOTAM. Check TFRs before flying.' : 'Drones are restricted here') + (a.Ceiling ? ' (surface to ' + a.Ceiling + ').' : '.'), at: s.ll });
    shapes.push({ f, level: part ? 'caution' : 'bad' });
  });
  D.nda.forEach(f => {
    const s = hit(f); if (!s) return;
    items.push({ level: 'caution', kind: 'nda', title: f.a.NAME || 'National defense airspace', detail: 'Active by NOTAM. Check TFRs before flying.', at: s.ll });
    shapes.push({ f, level: 'caution' });
  });
  D.std.forEach(f => {
    if (!f.pt) return; let best = Infinity; S2.forEach(s => { best = Math.min(best, hav(s.ll, f.pt)); });
    if (best > STADIUM_R) return;
    items.push({ level: 'caution', kind: 'std', title: 'Within 3 NM of ' + (f.a.NAME || 'a stadium'), detail: 'No drones from 1 hour before to 1 hour after major events there (game-day TFR).', at: f.pt, std: f.pt });
  });
  const level = items.reduce((m, it) => AIR_RANK[it.level] > AIR_RANK[m] ? it.level : m, 'ok');
  const res = { level, items, shapes, cells, maxFt };
  airEval = { mp, data: air.data, res };
  return res;
}
function airHeadline(r) {
  if (!r) return '';
  if (r.level === 'bad') return 'Restricted airspace on this route';
  if (r.level === 'auth') return 'FAA authorization needed';
  if (r.level === 'caution') return 'No authorization needed · cautions';
  return 'Class G · no FAA authorization needed';
}
function airWarns(warns) {
  const r = airCheck(); if (!r) return;
  if (r.level === 'bad') warns.push(['bad', 'Route enters restricted or prohibited airspace. See Airspace.']);
  else if (r.level === 'auth') warns.push(['bad', 'Controlled airspace: get FAA authorization (LAANC) before flying. See Airspace.']);
}
function airPanel() {
  if (!airPoints()) return '';
  const r = airCheck();
  let h = `<section class="grp"><h3>Airspace</h3>`;
  if (air.state === 'loading') h += '<p class="note">Checking FAA airspace…</p>';
  else if (air.state === 'error') h += '<div class="warn">⚠ Couldn\'t reach the FAA airspace data. The check needs internet.</div>';
  else if (r) {
    h += `<div class="airbadge ${r.level}">${airHeadline(r)}</div>`;
    if (r.items.length) h += '<div class="airitems">' + r.items.map((it, i) => `<div class="airitem ${it.level}"><div style="flex:1"><b>${esc(it.title)}</b><small>${it.detail}</small></div><button class="btn small" data-act="airgo" data-i="${i}">Show</button></div>`).join('') + '</div>';
    else h += `<p class="note">Up to 400 ft above the ground, with no special use areas or stadiums nearby.</p>`;
    const d = new Date(air.at);
    h += `<p class="note">FAA data from ${d.toLocaleDateString([], { month: 'short', day: 'numeric' })}${air.stale || air.cached ? ' (saved copy)' : ''}. Always check temporary flight restrictions (TFRs) on the day you fly.</p>`;
  }
  if (r && (r.level === 'auth' || r.level === 'bad')) h += `<p class="note">This app can't file the request itself. A LAANC app such as Aloft approves most requests in seconds; FAA DroneZone handles the rest.</p>`;
  h += '<div class="btns">';
  if (r && r.level === 'auth') h += '<button class="btn small primary" data-act="airreq">Request authorization</button><button class="btn small" data-act="airdz">FAA DroneZone</button>';
  h += '<button class="btn small" data-act="airtfr">Check TFRs</button><button class="btn small" data-act="airb4u">B4UFLY map</button>';
  if (air.state !== 'loading') h += '<button class="btn small" data-act="aircheck">Check again</button>';
  h += '</div>';
  return h + '</section>';
}
/* Panel buttons. Returns true if handled. */
function airAct(act, b) {
  if (act === 'airreq') openUrl(AIR_LINKS.aloft);
  else if (act === 'airdz') openUrl(AIR_LINKS.dz);
  else if (act === 'airtfr') openUrl(AIR_LINKS.tfr);
  else if (act === 'airb4u') openUrl(AIR_LINKS.b4u);
  else if (act === 'airlaanc') openUrl(AIR_LINKS.laanc);
  else if (act === 'aircheck') airRun(true);
  else if (act === 'airgo') { const r = airCheck(), it = r && r.items[+b.dataset.i]; if (it) map.setView(it.at, Math.max(map.getZoom(), it.kind === 'std' ? 12 : 14)); }
  else return false;
  return true;
}
const AIR_COLOR = { bad: '#ff4d4f', auth: '#e64980', caution: '#fab005' };
function airDraw(layer) {
  const r = airCheck(); if (!r) return;
  r.shapes.forEach(({ f, level }) => f.polys.forEach(p => L.polygon(p.R, { color: AIR_COLOR[level], weight: 2, dashArray: '8 6', fillOpacity: 0.05, interactive: false }).addTo(layer)));
  r.cells.forEach(({ f, c }) => {
    f.polys.forEach(p => L.polygon(p.R, { color: '#e64980', weight: 1, opacity: 0.7, fillOpacity: c <= 0 ? 0.18 : 0.06, interactive: false }).addTo(layer));
    if (r.cells.length <= 80 && f.polys[0]) { const b = f.polys[0].box; L.marker([(b[0] + b[2]) / 2, (b[1] + b[3]) / 2], { interactive: false, icon: L.divIcon({ className: '', html: `<div class="airceil">${imp() ? c : Math.round(c / FT)}</div>`, iconSize: [34, 16], iconAnchor: [17, 8] }) }).addTo(layer); }
  });
  r.items.forEach(it => { if (it.std) L.circle(it.std, { radius: STADIUM_R, color: AIR_COLOR.caution, weight: 1.5, dashArray: '4 6', fillOpacity: 0.03, interactive: false }).addTo(layer); });
}
/* Items the send screen should stop for (authorization or restricted). */
function airBlocking() {
  const r = airCheck(); if (!r || AIR_RANK[r.level] < 2 || airAck === air.key) return [];
  return r.items.filter(it => AIR_RANK[it.level] >= 2);
}

/* ---------- sync ----------
   Keeps markers, boundaries and saved missions the same on the tablet and the PC through one shared JSON file, normally
   in Google Drive. Every record carries the time it last changed (_t); deletions leave a dated tombstone. A sync reads
   the file, keeps the newest version of each record, writes the result back and updates this device. Because the newest
   change always wins and deletions are explicit, a device that reads an out-of-date copy can't wipe out anything: the
   next sync from the device that has the newer change puts it back. Map files, offline areas, settings and the plan on
   screen don't sync. */
const SY_KINDS = ['markers', 'shapes', 'missions'];
const SY_LABEL = { markers: 'markers', shapes: 'boundaries and lines', missions: 'saved missions' };
const SY_TOMB_DAYS = 400;
let sy = { busy: false, again: false, manual: false, status: '', cls: '', timer: null, info: null, perm: false, armed: false };
function syGet(k) { try { return JSON.parse(localStorage.getItem(k)) || {}; } catch (e) { return {}; } }
function sySet(k, v) { try { localStorage.setItem(k, JSON.stringify(v)); } catch (e) {} }
function syHash(r) {
  const str = JSON.stringify(r, (k, v) => (k === '_t' ? undefined : v));
  let h1 = 0xdeadbeef, h2 = 0x41c6ce57;
  for (let i = 0; i < str.length; i++) { const c = str.charCodeAt(i); h1 = Math.imul(h1 ^ c, 2654435761); h2 = Math.imul(h2 ^ c, 1597334677); }
  h1 = Math.imul(h1 ^ (h1 >>> 16), 2246822507) ^ Math.imul(h2 ^ (h2 >>> 13), 3266489909);
  h2 = Math.imul(h2 ^ (h2 >>> 16), 2246822507) ^ Math.imul(h1 ^ (h1 >>> 13), 3266489909);
  return (h2 >>> 0).toString(36) + (h1 >>> 0).toString(36);
}
function syList(kind) { return kind === 'markers' ? markers : kind === 'shapes' ? shapes : libLoad(); }
/* Called from every save: stamps records that changed since the last save and records deletions. */
function syStamp(kind, arr) {
  const snap = syGet('planner.sync.snap');
  if (!snap[kind]) return; // not initialised yet (syInit does that at start-up)
  const tomb = syGet('planner.sync.tomb'), s = snap[kind], tb = tomb[kind] || (tomb[kind] = {}), now = Date.now(), seen = {};
  let changed = false;
  arr.forEach(r => {
    if (!r || !r.id) return;
    seen[r.id] = 1;
    const h = syHash(r);
    if (s[r.id] !== h) { r._t = now; s[r.id] = h; changed = true; }
    if (tb[r.id] && (r._t || 0) >= tb[r.id]) delete tb[r.id];
  });
  Object.keys(s).forEach(id => { if (!seen[id]) { delete s[id]; tb[id] = now; changed = true; } });
  if (!changed) return;
  sySet('planner.sync.snap', snap); sySet('planner.sync.tomb', tomb);
  syQueue(8000);
}
function syInit() {
  const snap = syGet('planner.sync.snap');
  SY_KINDS.forEach(kind => {
    if (snap[kind]) return;
    const s = snap[kind] = {}, arr = syList(kind);
    let fix = false;
    arr.forEach(r => { if (!r || !r.id) return; if (!r._t) { r._t = 1; fix = true; } s[r.id] = syHash(r); });
    if (fix) { // write the stamps back without restamping
      const key = kind === 'markers' ? 'planner.markers' : kind === 'shapes' ? 'planner.shapes' : LIB_KEY;
      try { localStorage.setItem(key, JSON.stringify(arr)); } catch (e) {}
    }
  });
  sySet('planner.sync.snap', snap);
}
/* Newest wins. A tie between two different versions is settled the same way on every device. */
function syMerge(local, remote) {
  const out = { sets: {}, deleted: {} }, cutoff = Date.now() - SY_TOMB_DAYS * 864e5;
  SY_KINDS.forEach(kind => {
    const L = (local.sets[kind] || []).filter(r => r && r.id), R = ((remote.sets || {})[kind] || []).filter(r => r && r.id);
    const lt = (local.deleted || {})[kind] || {}, rt = ((remote.deleted || {})[kind]) || {};
    const lm = {}, rm = {}; L.forEach(r => { lm[r.id] = r; }); R.forEach(r => { rm[r.id] = r; });
    const ids = []; const add = id => { if (!ids.includes(id)) ids.push(id); };
    L.forEach(r => add(r.id)); R.forEach(r => add(r.id)); Object.keys(lt).forEach(add); Object.keys(rt).forEach(add);
    const recs = [], dels = {};
    ids.forEach(id => {
      let best = null, bt = -1;
      const consider = (rec, t) => {
        if (t > bt) { best = rec; bt = t; return; }
        if (t === bt && rec && best && rec !== best) { if (JSON.stringify(rec) > JSON.stringify(best)) best = rec; }
        else if (t === bt && rec && !best) best = rec; // a record beats a tombstone with the same time
      };
      if (lm[id]) consider(lm[id], lm[id]._t || 0);
      if (rm[id]) consider(rm[id], rm[id]._t || 0);
      if (lt[id] != null) consider(null, lt[id]);
      if (rt[id] != null) consider(null, rt[id]);
      if (best) recs.push(best); else if (bt >= cutoff) dels[id] = bt;
    });
    out.sets[kind] = recs; out.deleted[kind] = dels;
  });
  return out;
}
function syLocal() {
  const tomb = syGet('planner.sync.tomb'), o = { sets: {}, deleted: {} };
  SY_KINDS.forEach(k => { o.sets[k] = syList(k); o.deleted[k] = tomb[k] || {}; });
  return o;
}
/* Reads the shared file leniently: some storage providers can leave old bytes after the end when a shorter file is
   written, so anything after the end of the JSON object is ignored. */
function syParse(text) {
  text = (text || '').replace(/^\uFEFF/, '').trim();
  if (!text) return { sets: {}, deleted: {} };
  let j = null;
  try { j = JSON.parse(text); } catch (e) {
    let depth = 0, str = false, esc2 = false, end = -1;
    for (let i = 0; i < text.length; i++) {
      const c = text[i];
      if (str) { if (esc2) esc2 = false; else if (c === '\\') esc2 = true; else if (c === '"') str = false; continue; }
      if (c === '"') str = true; else if (c === '{') depth++; else if (c === '}') { depth--; if (depth === 0) { end = i; break; } }
    }
    if (end > 0) try { j = JSON.parse(text.slice(0, end + 1)); } catch (e2) { j = null; }
  }
  if (!j || j.app !== 'Waypoint Planner sync') throw new Error('That file isn’t a Waypoint Planner sync file, so it was left alone. Choose the right file or create a new one.');
  if ((j.v || 1) > 1) throw new Error('The sync file was written by a newer version of the app. Update this app first.');
  return { sets: j.sets || {}, deleted: j.deleted || {}, photos: j.photos || {}, saved: j.saved, by: j.by };
}
function syInfo() {
  if (!native || !native.syncInfo) return null;
  try { return JSON.parse(native.syncInfo() || '{}'); } catch (e) { return {}; }
}
function syOn() { const i = syInfo(); return !!(i && i.name); }
function syLast() { return Number(localStorage.getItem('planner.sync.last') || 0); }
function syAgo(t) {
  if (!t) return 'never';
  const s = (Date.now() - t) / 1000;
  if (s < 60) return 'just now'; if (s < 3600) return Math.floor(s / 60) + ' min ago';
  const d = new Date(t), today = new Date();
  return (d.toDateString() === today.toDateString() ? '' : d.toLocaleDateString([], { month: 'short', day: 'numeric' }) + ' ') + d.toLocaleTimeString([], { hour: 'numeric', minute: '2-digit' });
}
function syLabel() {
  if (!syOn()) return 'Sync';
  const cls = sy.busy ? 'busy' : sy.cls === 'bad' ? 'bad' : sy.timer ? 'busy' : 'ok';
  return `Sync <i class="sydot ${cls}"></i>`;
}
function syQueue(ms) {
  if (!syOn()) return;
  clearTimeout(sy.timer);
  sy.timer = setTimeout(() => { sy.timer = null; syncNow(false); }, ms);
  renderTools();
}
function syncNow(manual) {
  if (!syOn()) return;
  if (sy.busy) { sy.again = true; if (manual) sy.manual = true; return; }
  clearTimeout(sy.timer); sy.timer = null;
  sy.busy = true; sy.manual = !!manual; sy.status = 'Syncing…'; sy.cls = ''; syRefresh();
  native.syncRead();
}
function syDone(status, cls) {
  sy.busy = false; sy.status = status; sy.cls = cls || '';
  if (cls === 'bad' && sy.manual && (sheet.hidden || sheetMode !== 'sync')) toast(status);
  sy.manual = false; syRefresh();
  if (sy.again) { sy.again = false; setTimeout(() => syncNow(false), 500); }
}
function syRefresh() { renderTools(); if (!sheet.hidden && sheetMode === 'sync') renderSync(); }
/* Put merged records into this device's storage without stamping them as new local changes. */
function syApply(merged) {
  const snap = syGet('planner.sync.snap'), counts = {};
  SY_KINDS.forEach(kind => {
    const cur = syList(kind), next = merged.sets[kind];
    if (JSON.stringify(cur) !== JSON.stringify(next)) {
      const before = {}; cur.forEach(r => { before[r.id] = syHash(r); });
      let n = 0; next.forEach(r => { if (before[r.id] !== syHash(r)) n++; }); n += cur.filter(r => !next.some(x => x.id === r.id)).length;
      counts[kind] = n;
      const s = snap[kind] = {}; next.forEach(r => { s[r.id] = syHash(r); });
      sySet('planner.sync.snap', snap);
      if (kind === 'markers') { markers = next; try { localStorage.setItem('planner.markers', JSON.stringify(markers)); } catch (e) {} renderMarkers(); }
      else if (kind === 'shapes') { shapes = next; try { localStorage.setItem('planner.shapes', JSON.stringify(shapes)); } catch (e) {} renderShapes(); const sig = hzSignature(); if (sig !== hzSig) { hzSig = sig; hzChanged(); } }
      else { try { localStorage.setItem(LIB_KEY, JSON.stringify(next)); } catch (e) {} if (!sheet.hidden && sheetMode === 'lib') renderLib(); }
    }
  });
  sySet('planner.sync.tomb', merged.deleted);
  return counts;
}
function syncMsg(m) {
  if (m.type === 'resume') { if (syOn() && Date.now() - syLast() > 60000) syncNow(false); return; }
  if (m.type === 'syncPicked') {
    sy.perm = false; sy.status = ''; sy.cls = ''; sy.info = null;
    toast('Sync file: ' + m.name); syncNow(true); return;
  }
  if (m.type === 'syncError') {
    if (m.code === 'perm') sy.perm = true;
    if (m.code === 'cancel') { sy.busy = false; syRefresh(); return; }
    syDone(m.message || 'Sync failed.', 'bad'); return;
  }
  if (m.type === 'syncData') { syReceive(m.text); return; }
  if (m.type === 'syncWritten') { localStorage.setItem('planner.sync.last', String(Date.now())); syDone(sy.pendingMsg || 'Synced.', 'ok'); }
}
async function syReceive(text) {
  sy.perm = false;
  let remote;
  try { remote = syParse(text); } catch (e) { syDone(e.message, 'bad'); return; }
  const merged = syMerge(syLocal(), remote);
  const counts = syApply(merged);
  let ph = { map: remote.photos || {}, got: 0, sent: 0, changed: false };
  try { ph = await syPhotos(remote.photos || {}, merged); } catch (e) {}
  const recv = Object.keys(counts).map(k => counts[k] + ' ' + SY_LABEL[k]);
  if (ph.got) recv.push(ph.got + ' photo' + (ph.got === 1 ? '' : 's'));
  sy.pendingMsg = recv.length ? 'Synced. Updated ' + recv.join(', ') + ' here.' : 'Synced. Everything was already up to date here.';
  if (ph.got) phRefreshSheet();
  const sameAsFile = !ph.changed && SY_KINDS.every(k => JSON.stringify((remote.sets[k] || [])) === JSON.stringify(merged.sets[k]) &&
    JSON.stringify(remote.deleted[k] || {}) === JSON.stringify(merged.deleted[k]));
  if (sameAsFile && text && text.trim()) { localStorage.setItem('planner.sync.last', String(Date.now())); syDone(sy.pendingMsg, 'ok'); phGC(); return; }
  const info = syInfo() || {};
  const file = { app: 'Waypoint Planner sync', v: 1, saved: new Date().toISOString(), by: info.device || (DESKTOP ? 'PC' : 'Tablet'), sets: merged.sets, deleted: merged.deleted, photos: ph.map };
  native.syncWrite(JSON.stringify(file, null, 1));
  phGC();
}
function openSync() { sheetMode = 'sync'; sy.armed = false; renderSync(); sheet.hidden = false; }
function renderSync() {
  const info = syInfo(), on = !!(info && info.name);
  let h = `<div class="card"><h2>Sync</h2>
    <p class="note">Keeps your <b>markers, boundaries and saved missions</b> the same on the tablet and the PC through one shared file in Google Drive. Each device syncs when it opens, a few seconds after you change something, and when you tap Sync now. The newest change wins, and deletions sync too.</p>`;
  if (!info) h += `<div class="status bad">${PC ? 'Sync needs Microsoft Edge or Google Chrome.' : 'Sync isn’t available in this version.'}</div>`;
  else if (!on) {
    h += `<div class="btns" style="margin-bottom:8px"><button class="btn primary" data-sy="create">Create sync file…</button><button class="btn" data-sy="pick">Use existing file…</button></div>
      <p class="note"><b>First device:</b> tap <b>Create sync file</b> and save it in Google Drive. ${DESKTOP ? 'With Google Drive for desktop it’s under <b>G:\\My Drive</b>.' : 'In the picker, open the menu (☰) and choose <b>Drive</b>.'}<br>
      <b>Other device:</b> tap <b>Use existing file</b> and choose that same file. ${DESKTOP ? '' : 'Wait for the file to show up in Drive first; it can take a minute after the PC saves it.'}</p>`;
  } else {
    if (sy.status) h += `<div class="status ${sy.cls}">${esc(sy.status)}</div>`;
    h += `<div class="mstats" style="margin:8px 0"><span>File</span><b>${esc(info.name)}</b><span>Last synced</span><b>${syAgo(syLast())}</b></div>`;
    h += `<div class="btns" style="margin-bottom:8px">${sy.perm && PC ? '<button class="btn primary" data-sy="now">Allow access and sync</button>' : `<button class="btn primary" data-sy="now"${sy.busy ? ' disabled' : ''}>${sy.busy ? 'Syncing…' : 'Sync now'}</button>`}
      <button class="btn" data-sy="pick">Choose a different file…</button>
      <button class="btn danger" data-sy="stop">${sy.armed ? 'Tap again to stop' : 'Stop syncing'}</button></div>`;
    h += `<p class="note">Stopping only disconnects this device. Your features stay here and the file stays in Drive.</p>`;
  }
  h += `<p class="note">Photos sync as resized copies (about 200 KB each), so many photos make syncing slower on a weak signal. Not synced: My maps files (add the .mbtiles on each device), offline map areas, settings, and the plan on screen until you save it in Missions.</p>
    <div class="btns"><button class="btn" data-sy="close">Close</button></div></div>`;
  sheet.innerHTML = h;
}
function syncClick(e) {
  if (e.target === sheet) { sheet.hidden = true; return; }
  const b = e.target.closest('button[data-sy]'); if (!b || b.disabled) return;
  const a = b.dataset.sy;
  if (a === 'close') { sheet.hidden = true; return; }
  if (a === 'create' || a === 'pick') { native.syncPick(a === 'create'); return; }
  if (a === 'now') { syncNow(true); return; }
  if (a === 'stop') {
    if (!sy.armed) { sy.armed = true; renderSync(); return; }
    sy.armed = false; native.syncForget(); localStorage.removeItem('planner.sync.last'); sy.status = ''; sy.cls = ''; clearTimeout(sy.timer); sy.timer = null;
    renderSync(); renderTools(); toast('This device stopped syncing');
  }
}
function syBoot() {
  syInit();
  if (syOn()) setTimeout(() => syncNow(false), 1500);
  setTimeout(phGC, 20000);
  document.addEventListener('visibilitychange', () => { if (document.visibilityState === 'visible' && DESKTOP) syncMsg({ type: 'resume' }); });
}

/* ---------- first-run notice ---------- */
function showNotice() {
  try { if (localStorage.getItem('planner.ack')) return; } catch (e) { return; }
  sheetMode = 'notice';
  sheet.innerHTML = `<div class="card"><h2>Before you fly</h2>
    <p>FlyPlan writes waypoint missions that DJI Fly flies. It isn't made by or affiliated with DJI, and it comes with no warranty.</p>
    <ul style="padding-left:20px;margin:8px 0">
      <li>Always check the route in DJI Fly before starting a mission.</li>
      <li>Keep the drone in sight, watch for trees, towers and power lines, and be ready to take over.</li>
      <li>Follow the rules where you fly (in the US: FAA Part 107 or the recreational rules, airspace authorizations, 400 ft limit).</li>
      <li>DJI app or firmware updates can change how missions load. Test new versions with a short, safe mission first.</li>
    </ul>
    <p class="note">You fly at your own risk.</p>
    <div class="btns"><button class="btn primary" data-ack="1">I understand</button></div></div>`;
  sheet.hidden = false;
}
sheet.addEventListener('click', e => {
  if (sheetMode !== 'notice') return;
  if (e.target.closest('[data-ack]')) { try { localStorage.setItem('planner.ack', '1'); } catch (err) {} sheet.hidden = true; sheetMode = 'send'; }
}, true);

/* ---------- boot ---------- */
if (S.mode !== 'waypoint') regen();
commit(false);
if (native && native.listOfflineAreas) native.listOfflineAreas();
if (native && native.listMaps) native.listMaps();
if (baseLayer === 'usgs') probeUsgs();
syBoot();
hzSig = hzSignature();
window.__booted = true;
(function () { // splash: fade out once the planner has drawn
  const sp = document.getElementById('splash'); if (!sp || window.__noSplash) { if (sp) sp.remove(); showNotice(); return; }
  setTimeout(() => { sp.classList.add('hide'); setTimeout(() => { sp.remove(); showNotice(); }, 450); }, 900);
})();
if (S.home) { homeMarker = L.circleMarker(S.home, { radius: 8, color: '#fff', weight: 3, fillColor: '#1c7ed6', fillOpacity: 1 }).addTo(map); }
