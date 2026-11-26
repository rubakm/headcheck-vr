/*
 * HeadCheck VR 2 - Fit-to-Ride simulator engine
 * -----------------------------------------------------------------------------
 * Pure JavaScript, no browser dependency. The SAME code runs:
 *   - in the headset (index.html renders it and feeds head-tracking input),
 *   - in the fleet dashboard (re-scoring, synthetic cohorts),
 *   - in Node for automated tests (node test-core.js).
 *
 * Conventions
 *   s        distance along the route centre line of the rider's lane (m)
 *   y        lateral offset, metres to the RIGHT of the rider's lane centre
 *            (India drives on the left: our lane is y in [-1.9, +1.9], the
 *            centre line is at +1.9, the oncoming lane centre is at +3.8)
 *   psi      heading error relative to the road tangent (rad, + = pointing right)
 *   heading  h in radians, 0 = north (-z in three.js), + = clockwise (right turn)
 *   head yaw / pitch / roll in degrees relative to the calibrated neutral pose;
 *            + yaw = looking right, + roll = head tilted to the right shoulder,
 *            - pitch = looking down
 */
(function (root) {
  'use strict';

  var VERSION = '2.0.0';
  var DEG = Math.PI / 180, RAD = 180 / Math.PI, G = 9.81;

  // ---------------------------------------------------------------------------
  // 1. Utilities
  // ---------------------------------------------------------------------------
  function mulberry32(seed) {
    seed = seed >>> 0;
    return function () {
      seed = (seed + 0x6D2B79F5) | 0;
      var t = Math.imul(seed ^ (seed >>> 15), 1 | seed);
      t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
  }
  function hashString(str) {
    var h = 2166136261 >>> 0;
    for (var i = 0; i < str.length; i++) { h ^= str.charCodeAt(i); h = Math.imul(h, 16777619) >>> 0; }
    return h >>> 0;
  }
  function rr(r, a, b) { return a + (b - a) * r(); }
  function randn(r) { var u = 1 - r(), v = r(); return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v); }
  function shuffle(r, a) {
    for (var i = a.length - 1; i > 0; i--) { var j = Math.floor(r() * (i + 1)); var t = a[i]; a[i] = a[j]; a[j] = t; }
    return a;
  }
  function clamp(v, a, b) { return v < a ? a : v > b ? b : v; }
  function round(v, d) { if (v == null || !isFinite(v)) return v; var k = Math.pow(10, d || 0); return Math.round(v * k) / k; }
  function mean(a) { if (!a.length) return NaN; var s = 0; for (var i = 0; i < a.length; i++) s += a[i]; return s / a.length; }
  function sd(a) {
    if (a.length < 2) return 0;
    var m = mean(a), s = 0;
    for (var i = 0; i < a.length; i++) s += (a[i] - m) * (a[i] - m);
    return Math.sqrt(s / (a.length - 1));
  }
  function median(a) {
    if (!a.length) return NaN;
    var b = a.slice().sort(function (x, y) { return x - y; }), n = b.length;
    return n % 2 ? b[(n - 1) / 2] : (b[n / 2 - 1] + b[n / 2]) / 2;
  }
  function mad(a) { var m = median(a); return median(a.map(function (x) { return Math.abs(x - m); })); }
  function wrapDeg(a) { return ((a + 180) % 360 + 360) % 360 - 180; }

  // ---------------------------------------------------------------------------
  // 2. Configuration
  // ---------------------------------------------------------------------------
  var VEHICLES = {
    bike: {
      id: 'bike', cruise: 8.9, accel: 1.6, brakeDecel: 6.0, control: 'lean', maxCmdDeg: 35, cmdTau: 0.2,
      wheelBase: 1.3, halfWidth: 0.42, front: 1.0, rear: 0.9, wheels: [0]
    },
    auto: {
      id: 'auto', cruise: 8.3, accel: 1.3, brakeDecel: 5.0, control: 'steer', maxCmdDeg: 14, cmdTau: 0.15,
      wheelBase: 2.0, halfWidth: 0.72, front: 1.5, rear: 1.3, wheels: [-0.6, 0, 0.6]
    }
  };

  var CONFIG = {
    dt: 1 / 120,                       // physics step (s)
    logHz: 20,                         // logging rate
    holdAfterBrakeS: 1.0,              // throttle stays closed this long after releasing the brake
    brakeRampS: 0.25, brakeReleaseS: 0.12,
    startSpeed: 5,
    steer: { deadzoneDeg: 1.2, maxRollDeg: 26, expo: 1.3 },
    filter: { minCutoff: 3.0, beta: 0.06, dCutoff: 1.0 },  // 1-euro filter on head roll
    laneHalf: 1.9, departY: 1.55, departHyst: 0.25,
    roadLeft: -2.3, roadRight: 5.9, centreLine: 1.9, oncomingY: 3.8,
    wind: { freqs: [0.07, 0.13, 0.23, 0.41], disp: [0.22, 0.13, 0.07, 0.035] },  // sum-of-sines lateral gusts
    windScale: 1.0,
    drt: { isiMin: 3, isiMax: 5, onS: 1.0, minRT: 0.1, maxRT: 2.5, quietPre: 0.9, quietPost: 2.6, firstAfter: 4 },
    eyesOffPitchDeg: -28,              // looking further down than this = eyes off the road (phone on the handlebar)
    leadIn: 30, runOut: 25,
    maxRideS: 200
  };

  // Events every session contains (order and positions vary by seed = "parallel forms")
  var BLUEPRINT = ['jaywalker', 'lead_stop', 'pothole', 'pothole_hidden', 'breaker', 'wrong_side',
    'sharp_turn', 'ambulance', 'school_zone', 'lane_change', 'animal'];
  var SLOT_LEN = {
    jaywalker: 65, lead_stop: 78, pothole: 45, pothole_hidden: 45, breaker: 50, wrong_side: 60,
    ambulance: 68, school_zone: 75, lane_change: 66, animal: 62, sharp_turn: 68
  };
  var SUDDEN = { jaywalker: 1, lead_stop: 1, animal: 1 };   // events whose brake reaction time is scored

  // ---------------------------------------------------------------------------
  // 3. Route geometry (straights + circular arcs)
  // ---------------------------------------------------------------------------
  function fwd(h) { return [Math.sin(h), -Math.cos(h)]; }
  function right(h) { return [Math.cos(h), Math.sin(h)]; }
  function segPose(g, d) {
    if (g.type === 'straight') { var f = fwd(g.h0); return { x: g.x0 + f[0] * d, z: g.z0 + f[1] * d, h: g.h0 }; }
    var r0 = right(g.h0), cx = g.x0 + g.dir * g.r * r0[0], cz = g.z0 + g.dir * g.r * r0[1];
    var h = g.h0 + g.dir * d / g.r, r1 = right(h);
    return { x: cx - g.dir * g.r * r1[0], z: cz - g.dir * g.r * r1[1], h: h };
  }
  function buildPath(segments) {
    var x = 0, z = 0, h = 0, s = 0, out = [];
    segments.forEach(function (g) {
      var len = g.type === 'straight' ? g.len : g.r * g.angleDeg * DEG;
      var seg = { type: g.type, len: len, r: g.r || 0, dir: g.dir || 0, s0: s, x0: x, z0: z, h0: h,
        k: g.type === 'arc' ? g.dir / g.r : 0 };
      out.push(seg);
      var e = segPose(seg, len); x = e.x; z = e.z; h = e.h; s += len;
    });
    return { segs: out, length: s };
  }
  function segIndex(path, s) {
    var a = path.segs, lo = 0, hi = a.length - 1;
    if (s <= 0) return 0;
    while (lo < hi) { var mid = (lo + hi + 1) >> 1; if (a[mid].s0 <= s) lo = mid; else hi = mid - 1; }
    return lo;
  }
  function poseAt(path, s, lateral) {
    s = clamp(s, 0, path.length);
    var g = path.segs[segIndex(path, s)], p = segPose(g, s - g.s0);
    if (lateral) { var r = right(p.h); p.x += r[0] * lateral; p.z += r[1] * lateral; }
    p.s = s;
    return p;
  }
  function curvatureAt(path, s) {
    if (s < 0 || s > path.length) return 0;
    return path.segs[segIndex(path, s)].k;
  }

  // ---------------------------------------------------------------------------
  // 4. Route generator: same blueprint every day, different order/positions
  // ---------------------------------------------------------------------------
  function eventParams(kind, s0, r, animal) {
    switch (kind) {
      case 'jaywalker': return { busS: s0 + 40.5, busY: -2.65, crossS: s0 + 47.6, triggerDist: 24, pedSpeed: 1.5,
        stopY: rr(r, -0.35, 0.05), pauseS: 2.0, qS: s0 + 47.6 - 24 };
      case 'lead_stop': return { parkS: s0 + 45, parkY: -2.55, mergeAt: s0 + 5, laneY: -0.2, vLead: 7.2,
        brakeGap: 17, brakeMaxS: s0 + 68, decel: 5.5, qS: s0 + 5 };
      case 'pothole': return { pS: s0 + 32, pY: rr(r, -0.45, 0.45), pR: rr(r, 0.48, 0.6) };
      case 'pothole_hidden': var py = rr(r, -0.3, 0.3);
        return { pS: s0 + 32, puddleY: py, puddleR: 1.3, pY: py + rr(r, -0.35, 0.35), pR: 0.45 };
      case 'breaker': return { signS: s0 + 8, bS: s0 + 38, okV: 5.6 };
      case 'wrong_side': return { spawnAt: s0 + 4, dist0: 62, vOpp: 6.5, laneY: 0.3, dodgeAt: 12, dodgeY: 0.95, qS: s0 + 4 };
      case 'ambulance': return { startAt: s0 + 4, back: 45, dv: 7, laneY: 1.15, passY: 2.5, qS: s0 + 4 };
      case 'school_zone': return { signS: s0 + 6, zoneS0: s0 + 8, zoneS1: s0 + 70, crossS: s0 + 40, limit: 6.94, tol: 0.85 };
      case 'lane_change': return { truckS: s0 + 50, truckLen: 8, truckY: -1.25, truckHalfY: 1.12,
        bikeSpawnAt: s0 + 12, bikeBack: 18, bikeV: 13.2, bikeY: 1.35 };
      case 'animal':
        return animal === 'cow'
          ? { animal: 'cow', aS: s0 + 42, aY: 0.35, startDist: 45, vy: -0.5 }
          : { animal: 'dog', aS: s0 + 40, startDist: 21, fromY: -3.4, toY: -0.05, vy: 5.0, pauseS: 1.1, qS: s0 + 40 - 21 };
    }
    return {};
  }

  function buildRoute(seed, opts) {
    opts = opts || {};
    seed = (seed >>> 0) || 1;
    var r = mulberry32(seed);
    var order = shuffle(r, BLUEPRINT.slice());
    // ease riders in: the first event is never a sudden or turning one
    var hard = { sharp_turn: 1, jaywalker: 1, lead_stop: 1, animal: 1, wrong_side: 1 };
    if (hard[order[0]]) {
      for (var j = 1; j < order.length; j++) if (!hard[order[j]]) { var tmp = order[0]; order[0] = order[j]; order[j] = tmp; break; }
    }
    var animal = r() < 0.5 ? 'cow' : 'dog';
    var segs = [], events = [], s = 0, heading = 0;
    function straight(len) { segs.push({ type: 'straight', len: len }); s += len; }
    function arc(R, dir, ang) { segs.push({ type: 'arc', r: R, dir: dir, angleDeg: ang }); s += R * ang * DEG; heading += dir * ang; }
    straight(CONFIG.leadIn);
    order.forEach(function (kind, i) {
      // gentle connector curve that keeps the overall heading near the start direction (no loops)
      var dir = heading > 8 ? -1 : heading < -8 ? 1 : (r() < 0.5 ? -1 : 1);
      var ang = Math.abs(heading) > 45 ? Math.min(32, Math.abs(heading) - 5) : rr(r, 7, 15);
      arc(rr(r, 45, 75), dir, ang);
      var ev = { id: 'E' + (i + 1), kind: kind, s0: s };
      if (kind === 'sharp_turn') {
        var tdir = heading > 5 ? -1 : heading < -5 ? 1 : (r() < 0.5 ? -1 : 1);
        ev.dir = tdir; ev.R = 14; ev.signS = s + 3; ev.turnS = s + 32; ev.turnLen = 14 * Math.PI / 2;
        straight(32); arc(14, tdir, 90); straight(14);
        ev.s1 = s;
      } else {
        Object.assign(ev, eventParams(kind, s, r, animal));
        straight(SLOT_LEN[kind]);
        ev.s1 = s;
      }
      events.push(ev);
    });
    straight(CONFIG.runOut);
    var path = buildPath(segs);
    var wind = CONFIG.wind.freqs.map(function (f, k) {
      return { f: f, a: CONFIG.wind.disp[k] * 2 * Math.PI * f, ph: r() * 2 * Math.PI };
    });
    var houseNo = 21 + Math.floor(r() * 68);
    if (houseNo % 11 === 0) houseNo += 1;             // avoid 33, 44 ... (digit swap gives the same number)
    var sjt = ['phone', 'ambulance', 'school'][Math.floor(r() * 3)];
    return {
      seed: seed, segs: segs, path: path, length: path.length, events: events, order: order, animal: animal,
      phoneS: events[4].s1 + 2, wind: wind, houseNo: houseNo, sjt: sjt, decorSeed: Math.floor(r() * 1e9)
    };
  }

  // ---------------------------------------------------------------------------
  // 5. Head-tilt steering: dead zone + expo curve, and a 1-euro filter
  // ---------------------------------------------------------------------------
  function steerNorm(rollDeg) {
    var c = CONFIG.steer, a = Math.abs(rollDeg);
    if (a <= c.deadzoneDeg) return 0;
    var u = Math.min(1, (a - c.deadzoneDeg) / (c.maxRollDeg - c.deadzoneDeg));
    return (rollDeg < 0 ? -1 : 1) * Math.pow(u, c.expo);
  }
  function steerInverse(u) {
    var c = CONFIG.steer, a = Math.min(1, Math.abs(u));
    if (a === 0) return 0;
    return (u < 0 ? -1 : 1) * (c.deadzoneDeg + Math.pow(a, 1 / c.expo) * (c.maxRollDeg - c.deadzoneDeg));
  }
  function OneEuro(minCutoff, beta, dCutoff) { this.mc = minCutoff; this.beta = beta; this.dc = dCutoff; this.x = null; this.dx = 0; this.t = null; }
  function lpAlpha(cutoff, dt) { var tau = 1 / (2 * Math.PI * cutoff); return 1 / (1 + tau / dt); }
  OneEuro.prototype.filter = function (x, t) {
    if (this.x === null) { this.x = x; this.t = t; return x; }
    var dt = t - this.t;
    if (dt <= 0) return this.x;
    this.t = t;
    var dx = (x - this.x) / dt;
    this.dx += lpAlpha(this.dc, dt) * (dx - this.dx);
    var cutoff = this.mc + this.beta * Math.abs(this.dx);
    this.x += lpAlpha(cutoff, dt) * (x - this.x);
    return this.x;
  };

  // ---------------------------------------------------------------------------
  // 6. The simulation
  // ---------------------------------------------------------------------------
  function Sim(route, opts) {
    opts = opts || {};
    this.route = route; this.path = route.path;
    this.veh = VEHICLES[opts.vehicle] || VEHICLES.bike;
    this.mode = opts.mode || 'check';
    this.windScale = opts.windScale != null ? opts.windScale : CONFIG.windScale;
    this.rng = mulberry32((route.seed ^ 0x9E3779B9) >>> 0);
    this.t = 0; this.done = false;
    this.st = {
      s: 0, y: 0, psi: 0, v: CONFIG.startSpeed, cmd: 0, act: 0, brake: 0, held: false, holdUntil: 0, zoneHold: false,
      roll: 0, rollF: 0, yaw: 0, pitch: 0, yawRate: 0, kappa: 0, w: 0, kerbL: false, kerbR: false
    };
    this.filter = new OneEuro(CONFIG.filter.minCutoff, CONFIG.filter.beta, CONFIG.filter.dCutoff);
    this.actors = []; this.fx = []; this.collisions = []; this.presses = []; this.releases = [];
    this.nextActorId = 1; this.kerbHits = 0; this.potholeHits = 0;
    this.events = route.events.map(function (e) { return Object.assign({}, e, { state: 'pending', onsetT: null, rec: {} }); });
    this.drt = { list: [], cur: null, next: CONFIG.drt.firstAfter + rr(this.rng, CONFIG.drt.isiMin, CONFIG.drt.isiMax) };
    this.log = { hz: CONFIG.logHz, t: [], s: [], y: [], v: [], psi: [], cmd: [], roll: [], yaw: [], pitch: [], brk: [], w: [] };
    this.nextLog = 0;
    this.depart = { out: false, list: [] };
    this.eyes = { off: false, start: 0, glances: [] };
    this.phone = { state: 'idle', t0: null, t1: null };
    this.nextAmbient = 6 + rr(this.rng, 0, 3);
  }

  Sim.prototype.wind = function (t) {
    var w = 0, a = this.route.wind;
    for (var i = 0; i < a.length; i++) w += a[i].a * Math.sin(2 * Math.PI * a[i].f * t + a[i].ph);
    return w * this.windScale;
  };

  Sim.prototype.addActor = function (o) {
    var a = Object.assign({ id: this.nextActorId++, s: 0, y: 0, rho: 0, v: 0, halfS: 1, halfY: 0.5, solid: true, gone: false }, o);
    this.actors.push(a);
    return a;
  };
  Sim.prototype.removeActor = function (a) { if (a) a.gone = true; };

  Sim.prototype.inSlowZone = function (s) {
    for (var i = 0; i < this.events.length; i++) {
      var e = this.events[i];
      if (e.kind === 'school_zone' && s >= e.signS - 10 && s <= e.zoneS1) return true;
    }
    return false;
  };

  // DRT stimuli are held back just before / after sudden hazards so that a button
  // press can always be attributed to one task (detection vs. braking).
  Sim.prototype.quiet = function () {
    var t = this.t, s = this.st.s, v = this.st.v, c = CONFIG.drt;
    for (var i = 0; i < this.events.length; i++) {
      var e = this.events[i];
      if (e.onsetT != null) { if (t >= e.onsetT && t < e.onsetT + c.quietPost) return true; }
      else if (e.qS != null && s < e.qS + 8 && s + v * c.quietPre + 3 >= e.qS) return true;
      if (e.kind === 'lead_stop' && (e.state === 'merge' || e.state === 'cruise')) return true;
    }
    return false;
  };

  Sim.prototype.markOnset = function (ev) {
    ev.onsetT = this.t;
    if (SUDDEN[ev.kind] || ev.kind === 'wrong_side') {
      if (this.st.held) { ev.rec.preBraked = true; } else { ev.awaitBrake = true; }
    }
  };

  // Button down / up. t is the exact sim time of the press (can fall between physics steps).
  Sim.prototype.press = function (t) {
    if (t == null) t = this.t;
    var st = this.st, c = CONFIG.drt, d = this.drt, used = false;
    st.held = true;
    this.presses.push(round(t, 3));
    if (d.cur && d.cur.outcome == null) {
      var rt = t - d.cur.onset;
      if (rt >= 0 && rt < c.minRT) { d.cur.outcome = 'anticipation'; d.cur.rt = rt; used = true; }
      else if (rt >= c.minRT && rt <= c.maxRT) {
        d.cur.rt = rt; d.cur.outcome = 'hit'; used = true;
        if (d.cur.visible) { d.cur.visible = false; this.fx.push({ type: 'drt_off', hit: true }); }
      }
    }
    for (var i = 0; i < this.events.length; i++) {
      var ev = this.events[i];
      if (ev.awaitBrake && ev.rec.brakeRT == null && t >= ev.onsetT && t - ev.onsetT <= 4) {
        ev.rec.brakeRT = t - ev.onsetT; ev.awaitBrake = false; used = true;
      }
    }
    if (this.phase === 'ride' && !used) this.fx.push({ type: 'press' });
  };
  Sim.prototype.release = function (t) { this.st.held = false; this.releases.push(round(t == null ? this.t : t, 3)); };

  Sim.prototype.step = function (dt, input) {
    if (this.done) return;
    input = input || {};
    var st = this.st, veh = this.veh, cfg = CONFIG, t = this.t;
    this.phase = 'ride';
    // --- input: head pose -> lean / steer command
    st.roll = input.roll || 0; st.yaw = input.yaw || 0; st.pitch = input.pitch || 0;
    st.rollF = this.filter.filter(st.roll, t);
    st.cmd = steerNorm(st.rollF) * veh.maxCmdDeg;
    st.act += (st.cmd - st.act) * (1 - Math.exp(-dt / veh.cmdTau));
    // --- longitudinal: brake ramps in, auto-throttle returns to cruise after a short hold
    if (st.held) st.brake = Math.min(1, st.brake + dt / cfg.brakeRampS);
    else st.brake = Math.max(0, st.brake - dt / cfg.brakeReleaseS);
    var inZone = this.inSlowZone(st.s);
    if (st.brake > 0) {
      st.v = Math.max(0, st.v - st.brake * veh.brakeDecel * dt);
      st.holdUntil = t + cfg.holdAfterBrakeS;
      if (inZone) st.zoneHold = true;
    } else {
      if (st.zoneHold && !inZone) st.zoneHold = false;
      if (!st.zoneHold && t >= st.holdUntil && st.v < veh.cruise) st.v = Math.min(veh.cruise, st.v + veh.accel * dt);
    }
    // --- lateral dynamics in road (Frenet) coordinates
    var k = curvatureAt(this.path, st.s), v = st.v, r;
    st.kappa = k;
    if (veh.control === 'lean') r = G * Math.tan(st.act * DEG) / Math.max(v, 2.5);
    else r = v * Math.tan(st.act * DEG) / veh.wheelBase;
    var rMax = v / 3; r = clamp(r, -rMax, rMax);
    st.yawRate = r;
    var w = this.wind(t) * Math.min(1, v / 3); st.w = w;
    var denom = Math.max(0.2, 1 - k * st.y);
    var sdot = v * Math.cos(st.psi) / denom;
    st.s += sdot * dt;
    st.y += (v * Math.sin(st.psi) + w) * dt;
    st.psi = clamp(st.psi + (r - k * sdot) * dt, -1.2, 1.2);
    // kerb on the left, road edge on the right
    var yMin = cfg.roadLeft + veh.halfWidth, yMax = cfg.roadRight - veh.halfWidth;
    if (st.y < yMin) {
      st.y = yMin; st.psi = Math.max(st.psi, 0) * 0.5;
      if (!st.kerbL) { st.kerbL = true; this.kerbHits++; st.v *= 0.7; this.fx.push({ type: 'kerb' }); }
    } else if (st.y > yMin + 0.15) st.kerbL = false;
    if (st.y > yMax) {
      st.y = yMax; st.psi = Math.min(st.psi, 0) * 0.5;
      if (!st.kerbR) { st.kerbR = true; this.kerbHits++; st.v *= 0.7; this.fx.push({ type: 'kerb' }); }
    } else if (st.y < yMax - 0.15) st.kerbR = false;
    this.t = t + dt;
    // --- world
    for (var i = 0; i < this.events.length; i++) {
      var ev = this.events[i];
      if (ev.state === 'done') continue;
      var before = ev.state;
      HANDLERS[ev.kind](this, ev, dt);
      if (before === 'pending' && ev.state !== 'pending') ev.tStart = this.t;
      if (ev.state === 'done') ev.tDone = this.t;
    }
    this.updateAmbient(dt);
    this.checkCollisions();
    this.updateDRT();
    this.updatePhone();
    this.track();
    if (this.t >= this.nextLog) {
      var L = this.log;
      L.t.push(round(this.t, 2)); L.s.push(round(st.s, 2)); L.y.push(round(st.y, 3)); L.v.push(round(st.v, 2));
      L.psi.push(round(st.psi, 4)); L.cmd.push(round(st.cmd, 2)); L.roll.push(round(st.roll, 2));
      L.yaw.push(round(st.yaw, 1)); L.pitch.push(round(st.pitch, 1)); L.brk.push(round(st.brake, 2)); L.w.push(round(w, 3));
      this.nextLog += 1 / cfg.logHz;
      if (this.nextLog < this.t) this.nextLog = this.t + 1 / cfg.logHz;
    }
    if (st.s >= this.path.length - 1 || this.t > cfg.maxRideS) this.done = true;
  };

  Sim.prototype.track = function () {
    var st = this.st, cfg = CONFIG, d = this.depart;
    var a = Math.abs(st.y);
    if (!d.out && a > cfg.departY) { d.out = true; d.list.push({ t: round(this.t, 2), s: round(st.s, 1), side: st.y > 0 ? 'right' : 'left' }); }
    else if (d.out && a < cfg.departY - cfg.departHyst) d.out = false;
    var e = this.eyes, off = st.pitch < cfg.eyesOffPitchDeg;
    if (off && !e.off) { e.off = true; e.start = this.t; }
    else if (!off && e.off) { e.off = false; e.glances.push({ t: round(e.start, 2), d: round(this.t - e.start, 2), ring: this.phone.state === 'ring' }); }
  };

  Sim.prototype.updateDRT = function () {
    var d = this.drt, t = this.t, c = CONFIG.drt;
    if (d.cur) {
      if (d.cur.visible && t - d.cur.onset >= c.onS) { d.cur.visible = false; this.fx.push({ type: 'drt_off' }); }
      if (t - d.cur.onset > c.maxRT) { if (d.cur.outcome == null) d.cur.outcome = 'miss'; d.list.push(d.cur); d.cur = null; }
    }
    if (!d.cur && t >= d.next) {
      if (this.quiet()) d.next = t + 0.25;
      else {
        d.cur = { onset: t, visible: true, rt: null, outcome: null };
        this.fx.push({ type: 'drt_on' });
        d.next = t + rr(this.rng, c.isiMin, c.isiMax);
      }
    }
  };

  Sim.prototype.updatePhone = function () {
    var p = this.phone, st = this.st;
    if (p.state === 'idle' && st.s >= this.route.phoneS) { p.state = 'ring'; p.t0 = this.t; this.fx.push({ type: 'phone_ring' }); }
    else if (p.state === 'ring' && this.t - p.t0 > 5) { p.state = 'missed'; p.t1 = this.t; this.fx.push({ type: 'phone_stop' }); }
  };

  // Oncoming traffic in the other lane (solid: drifting across the centre line is dangerous)
  Sim.prototype.updateAmbient = function (dt) {
    var st = this.st, t = this.t;
    for (var i = 0; i < this.actors.length; i++) {
      var a = this.actors[i];
      if (!a.ambient || a.gone) continue;
      a.s -= a.v * dt;
      if (a.s < st.s - 25 || a.s < 2) a.gone = true;
    }
    if (t >= this.nextAmbient) {
      this.nextAmbient = t + rr(this.rng, 3.5, 7.5);
      var sp = st.s + 95;
      if (sp > this.path.length - 10) return;
      for (var j = 0; j < this.events.length; j++) {
        var e = this.events[j];
        if (e.kind === 'ambulance' && e.state !== 'pending' && e.state !== 'done') return;
      }
      var kinds = ['car', 'bike', 'bike', 'auto', 'bus', 'car', 'bike'];
      var kind = kinds[Math.floor(this.rng() * kinds.length)];
      var dims = { car: [2.1, 0.9], bike: [1.0, 0.45], auto: [1.4, 0.72], bus: [5.5, 1.25] }[kind];
      this.addActor({ type: kind, s: sp, y: CONFIG.oncomingY + rr(this.rng, -0.35, 0.35), rho: Math.PI, v: rr(this.rng, 7, 10),
        halfS: dims[0], halfY: dims[1], ambient: true, colorSeed: Math.floor(this.rng() * 1000) });
    }
  };

  Sim.prototype.checkCollisions = function () {
    var st = this.st, veh = this.veh;
    var rs0 = st.s - veh.rear, rs1 = st.s + veh.front, ry0 = st.y - veh.halfWidth, ry1 = st.y + veh.halfWidth;
    for (var i = 0; i < this.actors.length; i++) {
      var a = this.actors[i];
      if (a.gone || !a.solid) continue;
      var ds = Math.max(0, Math.max(a.s - a.halfS - rs1, rs0 - (a.s + a.halfS)));
      var dy = Math.max(0, Math.max(a.y - a.halfY - ry1, ry0 - (a.y + a.halfY)));
      var gap = Math.sqrt(ds * ds + dy * dy);
      if (a.ev) {
        var ev = this.eventById(a.ev);
        if (ev && Math.abs(a.s - st.s) < 8 && (ev.rec.minGap == null || gap < ev.rec.minGap)) ev.rec.minGap = round(gap, 2);
      }
      if (gap === 0 && (a.hitT == null || this.t - a.hitT > 2.5)) {
        a.hitT = this.t;
        this.collisions.push({ t: round(this.t, 2), ev: a.ev || null, type: a.type, speed: round(st.v, 2), ambient: !!a.ambient });
        this.fx.push({ type: 'collision', actor: a.id });
        st.v = Math.min(st.v, 1.0);
      }
    }
  };
  Sim.prototype.eventById = function (id) {
    for (var i = 0; i < this.events.length; i++) if (this.events[i].id === id) return this.events[i];
    return null;
  };

  // ---------------------------------------------------------------------------
  // 7. Event behaviour (all distances along the route; actors use the same s/y frame)
  // ---------------------------------------------------------------------------
  var HANDLERS = {
    jaywalker: function (sim, ev) {
      var st = sim.st;
      if (ev.state === 'pending' && st.s >= ev.s0 - 40) {
        ev.bus = sim.addActor({ type: 'bus', s: ev.busS, y: ev.busY, halfS: 5.5, halfY: 1.25, ev: ev.id, parked: true });
        ev.ped = sim.addActor({ type: 'ped', s: ev.crossS, y: -2.1, rho: Math.PI / 2, halfS: 0.3, halfY: 0.3, ev: ev.id, walking: false });
        ev.state = 'armed';
      }
      if (ev.state === 'armed' && ev.crossS - st.s <= ev.triggerDist) { ev.state = 'active'; ev.tTrig = sim.t; }
      if (ev.state === 'active') {
        var p = ev.ped, d = sim.t - ev.tTrig, tWalk = (ev.stopY + 2.1) / ev.pedSpeed;
        if (d < tWalk) { p.y = -2.1 + ev.pedSpeed * d; p.walking = true; }
        else if (d < tWalk + ev.pauseS) { p.y = ev.stopY; p.walking = false; }
        else { p.y = Math.min(2.9, ev.stopY + ev.pedSpeed * (d - tWalk - ev.pauseS)); p.walking = p.y < 2.9; }
        if (ev.onsetT == null && p.y > -1.35) sim.markOnset(ev);
        if (st.s > ev.crossS + 6 || d > 14) ev.state = 'done';
      }
    },

    lead_stop: function (sim, ev, dt) {
      var st = sim.st, a = ev.auto;
      if (ev.state === 'pending' && st.s >= ev.s0 - 45) {
        ev.auto = a = sim.addActor({ type: 'auto', s: ev.parkS, y: ev.parkY, halfS: 1.4, halfY: 0.72, ev: ev.id, v: 0 });
        ev.state = 'parked';
      }
      if (ev.state === 'parked' && st.s >= ev.mergeAt) { ev.state = 'merge'; ev.tMerge = sim.t; a.indicator = 'right'; }
      if (ev.state === 'merge') {
        a.v = Math.min(ev.vLead, a.v + 2.2 * dt);
        a.y = Math.min(ev.laneY, a.y + 0.95 * dt);
        a.rho = a.y < ev.laneY - 0.05 ? 0.18 : 0;
        if (a.y >= ev.laneY) { ev.state = 'cruise'; a.indicator = null; a.rho = 0; }
      }
      if (ev.state === 'cruise') {
        a.v = Math.min(ev.vLead, a.v + 2.2 * dt);
        if (a.s - st.s <= ev.brakeGap || a.s >= ev.brakeMaxS) { ev.state = 'braking'; a.brakeLight = true; sim.markOnset(ev); }
      }
      if (ev.state === 'braking') {
        a.v = Math.max(0, a.v - ev.decel * dt);
        if (a.v === 0) { ev.state = 'stopped'; ev.tStop = sim.t; }
      }
      if (ev.state === 'stopped' && sim.t - ev.tStop > 3.2) { ev.state = 'pullover'; a.indicator = 'left'; }
      if (ev.state === 'pullover') {
        a.y = Math.max(-2.7, a.y - 0.9 * dt);
        a.rho = a.y > -2.65 ? -0.2 : 0;
        if (a.y <= -2.7) { a.brakeLight = false; a.indicator = null; }
      }
      if (a && ev.state !== 'parked') a.s += a.v * dt;
      if (a && ev.state !== 'pending' && st.s > a.s + 15) { ev.state = 'done'; }
    },

    pothole: function (sim, ev) { potholeLogic(sim, ev, false); },
    pothole_hidden: function (sim, ev) { potholeLogic(sim, ev, true); },

    breaker: function (sim, ev) {
      var st = sim.st;
      if (ev.state === 'pending' && st.s >= ev.signS - 45) ev.state = 'armed';
      if (ev.state === 'armed') {
        if (st.held && ev.rec.brakeDist == null && st.s >= ev.signS - 20) ev.rec.brakeDist = round(ev.bS - st.s, 1);
        if (st.s >= ev.bS) {
          ev.rec.speed = round(st.v, 2);
          if (st.v > ev.okV) { sim.fx.push({ type: 'bump', severity: st.v }); st.v *= 0.82; }
          else sim.fx.push({ type: 'bump', severity: st.v * 0.3 });
          ev.state = 'done';
        }
      }
    },

    wrong_side: function (sim, ev, dt) {
      var st = sim.st, a = ev.moto;
      // the wrong-side bike is already riding towards us far away (in the haze) and becomes a hazard at 62 m
      if (ev.state === 'pending' && st.s >= ev.spawnAt - 30) {
        ev.moto = a = sim.addActor({ type: 'bike', s: st.s + ev.dist0 + 60, y: ev.laneY, rho: Math.PI, v: ev.vOpp, halfS: 1.0, halfY: 0.45, ev: ev.id, headlight: true });
        ev.state = 'approach';
      }
      if (ev.state === 'approach') {
        a.s -= a.v * dt;
        if (a.s - st.s <= ev.dist0) { ev.state = 'active'; ev.rec.y0 = round(st.y, 2); sim.markOnset(ev); }
      } else if (ev.state === 'active') {
        a.s -= a.v * dt;
        var gap = a.s - st.s;
        if (gap < 22 && !ev.honked) { ev.honked = true; sim.fx.push({ type: 'horn', actor: a.id, dur: 0.8 }); }
        if (gap < ev.dodgeAt) a.y += (ev.dodgeY - a.y) * Math.min(1, 1.2 * dt);
        if (ev.rec.respT == null && st.y < ev.rec.y0 - 0.25) ev.rec.respT = round(sim.t - ev.onsetT, 3);
        if (ev.rec.respT == null && ev.rec.brakeRT != null) ev.rec.respT = round(ev.rec.brakeRT, 3);
        if (Math.abs(gap) < 1.8) ev.rec.passDy = round(Math.min(ev.rec.passDy == null ? 9 : ev.rec.passDy, Math.abs(a.y - st.y)), 2);
        if (gap < -20) { sim.removeActor(a); ev.state = 'done'; }
      }
    },

    sharp_turn: function (sim, ev) {
      var st = sim.st, R = ev.rec;
      if (ev.state === 'pending' && st.s >= ev.signS - 45) ev.state = 'armed';
      if (ev.state !== 'armed') return;
      if (st.held && R.brakeDist == null && st.s >= ev.signS - 10) R.brakeDist = round(ev.turnS - st.s, 1);
      if (R.entrySpeed == null && st.s >= ev.turnS) R.entrySpeed = round(st.v, 2);
      if (st.s >= ev.turnS && st.s <= ev.turnS + ev.turnLen + 10) {
        R.maxAbsY = round(Math.max(R.maxAbsY || 0, Math.abs(st.y)), 2);
        var wide = ev.dir > 0 ? st.y < -1.5 : st.y > 1.5;
        if (wide) R.wide = true;
      }
      if (st.s > ev.turnS + ev.turnLen + 12) ev.state = 'done';
    },

    ambulance: function (sim, ev, dt) {
      var st = sim.st, a = ev.amb, R = ev.rec;
      if (ev.state === 'pending' && st.s >= ev.startAt) {
        ev.amb = a = sim.addActor({ type: 'ambulance', s: st.s - ev.back, y: ev.laneY, v: st.v + ev.dv, halfS: 2.6, halfY: 1.0, ev: ev.id, siren: true });
        ev.state = 'active'; sim.markOnset(ev); R.blockedS = 0;
        sim.fx.push({ type: 'siren_on', actor: a.id });
      }
      if (ev.state !== 'active') return;
      var gap = st.s - a.s;                       // rider is ahead by gap
      var gaveWay = st.y <= -0.5 || st.v < 4;
      if (R.giveT == null && gaveWay) R.giveT = round(sim.t - ev.onsetT, 2);
      var want = Math.min(15, Math.max(9, st.v + ev.dv));
      if (gap < 9 && gap > -2 && !gaveWay) {
        a.v = Math.min(want, st.v + 0.4); R.blockedS += dt;
        if (!ev.honkT || sim.t - ev.honkT > 1.5) { ev.honkT = sim.t; sim.fx.push({ type: 'horn', actor: a.id, dur: 0.6, low: true }); }
      } else a.v = want;
      var targetY = gap < 14 ? (gaveWay ? ev.passY - 0.4 : ev.passY + 0.5) : ev.laneY;
      a.y += (targetY - a.y) * Math.min(1, 1.5 * dt);
      a.s += a.v * dt;
      if (a.s > st.s + 45 || sim.t - ev.onsetT > 22) {
        sim.fx.push({ type: 'siren_off', actor: a.id }); sim.removeActor(a); ev.state = 'done';
      }
    },

    school_zone: function (sim, ev, dt) {
      var st = sim.st, R = ev.rec;
      if (ev.state === 'pending' && st.s >= ev.signS - 55) {
        ev.state = 'armed'; R.overDist = 0; R.zoneDist = 0;
        for (var k = 0; k < 4; k++) sim.addActor({ type: 'child', s: ev.crossS - 3 + k * 1.4, y: -3.5 - (k % 2) * 0.6, rho: Math.PI / 2, halfS: 0.25, halfY: 0.25, solid: false, ev: ev.id });
        sim.addActor({ type: 'guard', s: ev.crossS + 2.5, y: -2.9, rho: Math.PI / 2, halfS: 0.3, halfY: 0.3, solid: false, ev: ev.id });
      }
      if (ev.state !== 'armed') return;
      var lim = ev.limit + ev.tol;
      if (st.s >= ev.zoneS0 && st.s <= ev.zoneS1) {
        var ds = st.v * dt;
        R.zoneDist += ds; if (st.v > lim) R.overDist += ds;
        R.maxV = round(Math.max(R.maxV || 0, st.v), 2);
        if (R.slowAt == null && st.v <= lim) R.slowAt = round(st.s - ev.signS, 1);
      }
      if (st.s > ev.zoneS1) { R.overPct = R.zoneDist > 0 ? round(100 * R.overDist / R.zoneDist, 1) : 0; ev.state = 'done'; }
    },

    lane_change: function (sim, ev, dt) {
      var st = sim.st, R = ev.rec;
      if (ev.state === 'pending' && st.s >= ev.truckS - 80) {
        ev.truck = sim.addActor({ type: 'truck', s: ev.truckS, y: ev.truckY, halfS: ev.truckLen / 2, halfY: ev.truckHalfY, ev: ev.id, parked: true, hazard: true });
        ev.state = 'armed'; R.maxYawRight = 0;
      }
      if (ev.state !== 'armed') return;
      if (!ev.bike && st.s >= ev.bikeSpawnAt) {
        ev.bike = sim.addActor({ type: 'bike', s: st.s - ev.bikeBack, y: ev.bikeY, v: ev.bikeV, halfS: 1.0, halfY: 0.45, ev: ev.id, overtaker: true });
      }
      if (ev.bike && !ev.bike.gone) {
        var b = ev.bike, behind = st.s - b.s;
        // an overtaker that is still behind sees a rider already out in the lane and passes wider;
        // it cannot avoid a rider who swerves out while it is alongside (the blind-spot conflict)
        var want = behind > 2.2 && behind < 12 ? Math.min(2.7, Math.max(ev.bikeY, st.y + 1.0)) : (behind <= -3 ? ev.bikeY : b.y);
        b.y += clamp(want - b.y, -1.6 * dt, 1.6 * dt);
        b.s += b.v * dt;
        if (b.s > st.s + 60) sim.removeActor(b);
      }
      var rear = ev.truckS - ev.truckLen / 2;
      // shoulder-check window: from 36 m before the truck until the rider moves out for it (or 6 m before it)
      if (st.s >= rear - 36) {
        if (!ev.winOpen) { ev.winOpen = true; ev.yAtOpen = st.y; }
        if (R.moveT == null) {
          R.maxYawRight = Math.max(R.maxYawRight, st.yaw);
          var startedOut = ev.yAtOpen > 0.3;
          if ((!startedOut && st.y > 0.3) || st.s >= rear - 6) { R.moveT = round(sim.t, 2); R.moveS = round(rear - st.s, 1); }
        }
      }
      if (st.s >= rear && st.s <= ev.truckS + ev.truckLen / 2) R.passY = round(Math.max(R.passY == null ? -9 : R.passY, st.y), 2);
      if (st.s > ev.truckS + ev.truckLen / 2 + 12) {
        R.checked = R.maxYawRight >= 45; R.maxYawRight = round(R.maxYawRight, 1); ev.state = 'done';
      }
    },

    animal: function (sim, ev, dt) {
      var st = sim.st, a = ev.ani, R = ev.rec;
      if (ev.animal === 'cow') {
        if (ev.state === 'pending' && st.s >= ev.aS - 75) {
          ev.ani = a = sim.addActor({ type: 'cow', s: ev.aS, y: ev.aY, rho: -Math.PI / 2, halfS: 0.45, halfY: 1.1, ev: ev.id, walking: false });
          ev.state = 'armed';
        }
        if (ev.state === 'armed') {
          if (ev.onsetT == null && ev.aS - st.s <= 40) sim.markOnset(ev);
          if (ev.aS - st.s <= ev.startDist) { a.walking = a.y > -2.6; if (a.walking) a.y += ev.vy * dt; }
          if (Math.abs(st.s - ev.aS) < 3) R.passSpeed = round(Math.min(R.passSpeed == null ? 99 : R.passSpeed, st.v), 2);
          if (st.s > ev.aS + 12) ev.state = 'done';
        }
      } else {
        if (ev.state === 'pending' && st.s >= ev.aS - 60) {
          ev.ani = a = sim.addActor({ type: 'dog', s: ev.aS, y: ev.fromY, rho: Math.PI / 2, halfS: 0.4, halfY: 0.25, ev: ev.id, walking: false });
          ev.state = 'armed';
        }
        if (ev.state === 'armed' && ev.aS - st.s <= ev.startDist) { ev.state = 'active'; ev.tTrig = sim.t; }
        if (ev.state === 'active') {
          var d = sim.t - ev.tTrig, tRun = (ev.toY - ev.fromY) / ev.vy;
          if (d < tRun) { a.y = ev.fromY + ev.vy * d; a.walking = true; a.rho = Math.PI / 2; }
          else if (d < tRun + ev.pauseS) { a.y = ev.toY; a.walking = false; }
          else { a.y = Math.max(ev.fromY, ev.toY - ev.vy * (d - tRun - ev.pauseS)); a.walking = a.y > ev.fromY; a.rho = -Math.PI / 2; }
          if (ev.onsetT == null && a.y > -2.2) sim.markOnset(ev);
          if (Math.abs(st.s - ev.aS) < 3) R.passSpeed = round(Math.min(R.passSpeed == null ? 99 : R.passSpeed, st.v), 2);
          if (st.s > ev.aS + 10 || d > 10) ev.state = 'done';
        }
      }
    }
  };

  function potholeLogic(sim, ev, hidden) {
    var st = sim.st, R = ev.rec;
    if (ev.state === 'pending' && st.s >= ev.pS - 35) ev.state = 'armed';
    if (ev.state !== 'armed') return;
    if (hidden && R.puddleSpeed == null && st.s >= ev.pS - 1) {
      R.inPuddle = Math.abs(st.y - ev.puddleY) < ev.puddleR;
      R.puddleSpeed = round(st.v, 2);
    }
    if (st.s >= ev.pS) {
      var veh = sim.veh, hit = false, best = 99;
      for (var i = 0; i < veh.wheels.length; i++) {
        var dy = Math.abs(st.y + veh.wheels[i] - ev.pY);
        best = Math.min(best, dy - ev.pR);
        if (dy < ev.pR + 0.12) hit = true;
      }
      R.hit = hit; R.speed = round(st.v, 2); R.clearance = round(best, 2);
      if (hit) {
        if (!hidden || st.v > 5.5) sim.potholeHits++;      // a hidden pothole taken slowly is not the rider's fault
        sim.fx.push({ type: 'pothole', severity: st.v }); st.v *= st.v > 4 ? 0.82 : 0.95;
      }
      ev.state = 'done';
    }
  }

  // Close the session: everything the analysis needs, nothing it doesn't.
  Sim.prototype.finish = function (extra) {
    extra = extra || {};
    var d = this.drt;
    if (d.cur) { if (d.cur.outcome == null) d.cur.outcome = this.t - d.cur.onset > 1.5 ? 'miss' : 'cut'; d.list.push(d.cur); d.cur = null; }
    if (this.eyes.off) this.eyes.glances.push({ t: round(this.eyes.start, 2), d: round(this.t - this.eyes.start, 2), ring: false });
    var route = this.route, self = this;
    var events = this.events.map(function (e) {
      var o = { id: e.id, kind: e.kind, s0: round(e.s0, 1), onsetT: e.onsetT == null ? null : round(e.onsetT, 3),
        tDone: e.tDone == null ? null : round(e.tDone, 2), rec: e.rec };
      if (e.kind === 'animal') o.animal = e.animal;
      if (e.kind === 'sharp_turn') o.dir = e.dir;
      o.collisions = self.collisions.filter(function (c) { return c.ev === e.id; }).length;
      if (e.rec.brakeRT != null) e.rec.brakeRT = round(e.rec.brakeRT, 3);
      return o;
    });
    return {
      app: 'HeadCheck VR', version: VERSION, schema: 2,
      riderId: extra.riderId || 'anon', lang: extra.lang || 'en', vehicle: this.veh.id, mode: this.mode,
      simulated: !!extra.simulated, condition: extra.condition || null,
      startedAt: extra.startedAt || new Date().toISOString(), device: extra.device || null, view: extra.view || null,
      calib: extra.calib || null,
      route: { seed: route.seed, length: round(route.length, 1), order: route.order, animal: route.animal, houseNo: route.houseNo, sjt: route.sjt },
      durationS: round(this.t, 2), completed: this.st.s >= this.path.length - 2,
      log: this.log,
      events: events,
      drt: d.list.map(function (x) { return { onset: round(x.onset, 3), rt: x.rt == null ? null : round(x.rt, 3), outcome: x.outcome }; }),
      presses: this.presses, releases: this.releases,
      collisions: this.collisions, departures: this.depart.list, kerbHits: this.kerbHits, potholeHits: this.potholeHits,
      glances: this.eyes.glances, phone: { t0: this.phone.t0 == null ? null : round(this.phone.t0, 2), t1: this.phone.t1 == null ? null : round(this.phone.t1, 2) },
      answers: extra.answers || [],
      sessionS: extra.sessionS || null
    };
  };

  // ---------------------------------------------------------------------------
  // 8. Questions (asked after the ride, in the rider's language; answered by gaze)
  // ---------------------------------------------------------------------------
  function makeQuestions(route) {
    var r = mulberry32((route.seed ^ 0x51ED270B) >>> 0);
    var h = route.houseNo, sw = (h % 10) * 10 + Math.floor(h / 10);
    var memOpts = shuffle(r, [h, sw === h ? h + 10 : sw, h + (r() < 0.5 ? 3 : -3)]);
    var animals = shuffle(r, ['cow', 'dog', 'goat']);
    var sjtOpts = shuffle(r, ['good', 'bad1', 'bad2']);
    return [
      { id: 'Q1', kind: 'memory', promptKey: 'q_memory', options: memOpts.map(function (n) { return { label: String(n) }; }), correct: memOpts.indexOf(h) },
      { id: 'Q2', kind: 'awareness', promptKey: 'q_animal', options: animals.map(function (a) { return { key: 'animal_' + a }; }), correct: animals.indexOf(route.animal) },
      { id: 'Q3', kind: 'judgement', promptKey: 'sjt_' + route.sjt, options: sjtOpts.map(function (k) { return { key: 'sjt_' + route.sjt + '_' + k }; }), correct: sjtOpts.indexOf('good') }
    ];
  }

  // ---------------------------------------------------------------------------
  // 9. Analysis: session -> metrics
  // ---------------------------------------------------------------------------
  function cleanMask(S) {
    var L = S.log, n = L.t.length, mask = new Array(n), ex = [], exT = [];
    S.events.forEach(function (e) {
      if (e.kind === 'breaker' || e.kind === 'school_zone') return;
      ex.push([e.s0 - 5, e.s0 + (SLOT_LEN[e.kind] || 70) + 8]);
      if (e.onsetT != null) exT.push([e.onsetT - 1, (e.tDone != null ? e.tDone : e.onsetT + 12) + 2]);
    });
    var colT = S.collisions.map(function (c) { return c.t; });
    for (var i = 0; i < n; i++) {
      var ok = L.t[i] >= 4 && L.v[i] >= 3.5;
      for (var j = 0; ok && j < ex.length; j++) if (L.s[i] >= ex[j][0] && L.s[i] <= ex[j][1]) ok = false;
      for (var q = 0; ok && q < exT.length; q++) if (L.t[i] >= exT[q][0] && L.t[i] <= exT[q][1]) ok = false;
      for (var c = 0; ok && c < colT.length; c++) if (L.t[i] >= colT[c] && L.t[i] <= colT[c] + 3) ok = false;
      mask[i] = ok;
    }
    return mask;
  }

  function runsOf(mask) {
    var runs = [], st = -1;
    for (var i = 0; i <= mask.length; i++) {
      if (i < mask.length && mask[i]) { if (st < 0) st = i; }
      else if (st >= 0) { runs.push([st, i - 1]); st = -1; }
    }
    return runs;
  }

  // Steering reversals: direction changes of the (lightly smoothed) lean command larger than `gap` degrees
  function reversalRate(sig, mask, hz, gap) {
    var runs = runsOf(mask), count = 0, samples = 0;
    runs.forEach(function (r) {
      if (r[1] - r[0] < hz * 2) return;
      samples += r[1] - r[0] + 1;
      var x = [];
      for (var i = r[0]; i <= r[1]; i++) x.push((sig[Math.max(r[0], i - 1)] + sig[i] + sig[Math.min(r[1], i + 1)]) / 3);
      var ext = x[0], dir = 0;
      for (var j = 1; j < x.length; j++) {
        if (dir === 0) { if (x[j] - ext >= gap) { dir = 1; ext = x[j]; } else if (ext - x[j] >= gap) { dir = -1; ext = x[j]; } }
        else if (dir === 1) { if (x[j] > ext) ext = x[j]; else if (ext - x[j] >= gap) { count++; dir = -1; ext = x[j]; } }
        else { if (x[j] < ext) ext = x[j]; else if (x[j] - ext >= gap) { count++; dir = 1; ext = x[j]; } }
      }
    });
    var minutes = samples / hz / 60;
    return minutes > 0.2 ? count / minutes : null;
  }

  // Human-operator identification. Riders steer to correct the error at an aim point about
  // one second ahead: z(t) = y(t) + T*v*sin(psi(t)), T = 1 s. The delay at which the lean
  // command best (negatively) correlates with that aim-point error is the rider's effective
  // control lag: perception + decision + neuromuscular time. Validated against simulated
  // riders with known delays (see test-core.js).
  function identifyLag(S, mask) {
    var L = S.log, hz = L.hz, T = 1.0;
    var runs = runsOf(mask).filter(function (r) { return r[1] - r[0] >= hz * 3; });
    if (!runs.length) return { lagMs: null, r2: null };
    var maxJ = Math.round(1.2 * hz), cs = [];
    for (var j = 0; j <= maxJ; j++) {
      var sx = 0, sy = 0, sxx = 0, sxy = 0, syy = 0, n = 0;
      runs.forEach(function (r) {
        for (var i = r[0] + maxJ; i <= r[1]; i++) {
          var x = L.y[i - j] + T * L.v[i - j] * Math.sin(L.psi[i - j]), yv = L.cmd[i];
          sx += x; sy += yv; sxx += x * x; sxy += x * yv; syy += yv * yv; n++;
        }
      });
      if (n < 40) return { lagMs: null, r2: null };
      var cov = sxy / n - (sx / n) * (sy / n), vx = sxx / n - (sx / n) * (sx / n), vy = syy / n - (sy / n) * (sy / n);
      cs.push(vx > 0 && vy > 0 ? -cov / Math.sqrt(vx * vy) : 0);
    }
    var best = 0;
    for (var q = 1; q < cs.length; q++) if (cs[q] > cs[best]) best = q;
    var off = 0;
    if (best > 0 && best < cs.length - 1) {
      var a1 = cs[best - 1], a2 = cs[best], a3 = cs[best + 1], den = a1 - 2 * a2 + a3;
      if (den < 0) off = clamp(0.5 * (a1 - a3) / den, -0.5, 0.5);
    }
    var r2 = cs[best] > 0 ? cs[best] * cs[best] : 0;
    if (cs[best] < 0.4) return { lagMs: null, r2: round(r2, 3) };
    return { lagMs: Math.round((best + off) * 1000 / hz), r2: round(r2, 3) };
  }
  function solve3(A, b) {
    var m = [A[0].concat([b[0]]), A[1].concat([b[1]]), A[2].concat([b[2]])];
    for (var c = 0; c < 3; c++) {
      var p = c; for (var r = c + 1; r < 3; r++) if (Math.abs(m[r][c]) > Math.abs(m[p][c])) p = r;
      if (Math.abs(m[p][c]) < 1e-9) return null;
      var tmp = m[c]; m[c] = m[p]; m[p] = tmp;
      for (var r2 = 0; r2 < 3; r2++) if (r2 !== c) { var f = m[r2][c] / m[c][c]; for (var k = c; k < 4; k++) m[r2][k] -= f * m[c][k]; }
    }
    return [m[0][3] / m[0][0], m[1][3] / m[1][1], m[2][3] / m[2][2]];
  }

  function analyze(S) {
    var L = S.log, m = {}, mask = cleanMask(S);
    var ys = [];
    for (var i = 0; i < L.y.length; i++) if (mask[i]) ys.push(L.y[i]);
    m.sdlp = ys.length > 100 ? round(sd(ys), 3) : null;
    m.sdlpSeconds = round(ys.length / L.hz, 1);
    m.meanLanePos = ys.length ? round(mean(ys), 3) : null;
    m.departures = S.departures.length;
    m.kerbHits = S.kerbHits;
    m.reversalRate = round(reversalRate(L.cmd, mask, L.hz, 2.0), 1);
    var lag = identifyLag(S, mask); m.controlLagMs = lag.lagMs; m.controlFitR2 = lag.r2;
    // detection-response task
    var drt = S.drt.filter(function (x) { return x.outcome !== 'cut'; });
    var hits = drt.filter(function (x) { return x.outcome === 'hit'; });
    var anti = drt.filter(function (x) { return x.outcome === 'anticipation'; }).length;
    var valid = drt.length - anti;
    m.drtN = drt.length; m.drtHits = hits.length; m.drtAnticip = anti;
    m.drtMissRate = valid > 0 ? round(1 - hits.length / valid, 3) : null;
    var rts = hits.map(function (x) { return x.rt * 1000; });
    m.drtRT = rts.length ? Math.round(mean(rts)) : null;
    m.drtRTsd = rts.length > 1 ? Math.round(sd(rts)) : null;
    m.drtSlow = rts.filter(function (x) { return x > 1000; }).length;
    // brake reaction to sudden hazards
    var br = [], noResp = 0, pre = 0;
    S.events.forEach(function (e) {
      if (!SUDDEN[e.kind] || e.onsetT == null) return;
      if (e.kind === 'animal' && e.animal === 'cow') return;
      if (e.rec.preBraked) pre++;
      else if (e.rec.brakeRT != null) br.push(e.rec.brakeRT);
      else noResp++;
    });
    m.brakeRT = br.length ? round(mean(br), 3) : null;
    m.brakeNoResponse = noResp; m.brakeAnticipated = pre;
    // hazards
    m.collisions = S.collisions.length;
    m.collisionsHazard = S.collisions.filter(function (c) { return !c.ambient; }).length;
    m.potholeHits = S.potholeHits;
    m.nearMisses = S.events.filter(function (e) { return e.collisions === 0 && e.rec.minGap != null && e.rec.minGap < 0.5; }).length;
    // rules and context
    var byKind = {}; S.events.forEach(function (e) { byKind[e.kind] = e; });
    var e;
    if ((e = byKind.breaker)) m.breakerKmh = e.rec.speed != null ? round(e.rec.speed * 3.6, 1) : null;
    if ((e = byKind.school_zone)) { m.schoolOverPct = e.rec.overPct != null ? e.rec.overPct : null; m.schoolMaxKmh = e.rec.maxV != null ? round(e.rec.maxV * 3.6, 1) : null; }
    if ((e = byKind.sharp_turn)) { m.turnEntryKmh = e.rec.entrySpeed != null ? round(e.rec.entrySpeed * 3.6, 1) : null; m.turnWide = !!e.rec.wide; }
    if ((e = byKind.ambulance)) { m.ambulanceGiveS = e.rec.giveT != null ? e.rec.giveT : null; m.ambulanceBlockedS = e.rec.blockedS != null ? round(e.rec.blockedS, 1) : null; }
    if ((e = byKind.lane_change)) { m.shoulderCheck = !!e.rec.checked; m.shoulderYaw = e.rec.maxYawRight != null ? e.rec.maxYawRight : null; }
    if ((e = byKind.pothole_hidden)) m.puddleKmh = e.rec.inPuddle ? round((e.rec.puddleSpeed || 0) * 3.6, 1) : 0;
    if ((e = byKind.wrong_side)) { m.wrongSideRespS = e.rec.respT != null ? e.rec.respT : null; m.wrongSidePassDy = e.rec.passDy != null ? e.rec.passDy : null; }
    // eyes off the road (phone on the handlebar)
    var gl = S.glances || [];
    m.eyesOffMax = gl.length ? round(Math.max.apply(null, gl.map(function (g) { return g.d; })), 2) : 0;
    m.eyesOffTotal = round(gl.reduce(function (a, g) { return a + g.d; }, 0), 2);
    m.phoneGlance = gl.filter(function (g) { return g.ring; }).reduce(function (a, g) { return Math.max(a, g.d); }, 0);
    // questions
    var ans = S.answers || [];
    m.answersN = ans.length; m.answersCorrect = ans.filter(function (a) { return a.correct; }).length;
    // validity
    var yawSD = sd(L.yaw), rollSD = sd(L.roll);
    m.headMotionDeg = round(Math.max(yawSD, rollSD), 2);
    m.presses = S.presses.length;
    m.completed = !!S.completed;
    return m;
  }

  // ---------------------------------------------------------------------------
  // 10. Assessment: metrics (+ personal history) -> fitness index and verdict
  // ---------------------------------------------------------------------------
  // PROVISIONAL anchors (good -> 100, bad -> 0). They must be re-fitted from pilot data
  // of real riders; until then they come from published effect sizes + synthetic riders.
  var NORMS = {
    sdlp: [0.24, 0.60], departures: [0, 4], brakeRT: [0.85, 1.8], drtRT: [480, 1000], drtMissRate: [0.05, 0.45], eyesOffMax: [1.0, 3.0]
  };
  var BASE_KEYS = ['sdlp', 'drtRT', 'drtMissRate', 'brakeRT', 'controlLagMs'];
  var BASE_FLOOR = { sdlp: 0.04, drtRT: 45, drtMissRate: 0.06, brakeRT: 0.12, controlLagMs: 60 };

  function goodness(x, anchors) {
    if (x == null || !isFinite(x)) return null;
    return clamp(100 * (1 - (x - anchors[0]) / (anchors[1] - anchors[0])), 0, 100);
  }
  function avg(list) { var v = list.filter(function (x) { return x != null; }); return v.length ? mean(v) : null; }

  function subscores(m) {
    var sc = {};
    sc.control = avg([goodness(m.sdlp, NORMS.sdlp), goodness(m.departures + 0.5 * m.kerbHits, NORMS.departures)]);
    sc.reaction = avg([goodness(m.brakeRT, NORMS.brakeRT), goodness(m.drtRT, NORMS.drtRT),
      m.brakeNoResponse > 0 ? Math.max(0, 100 - 50 * m.brakeNoResponse) : null]);
    sc.attention = avg([goodness(m.drtMissRate, NORMS.drtMissRate), goodness(m.eyesOffMax, NORMS.eyesOffMax)]);
    sc.hazards = clamp(100 - 30 * m.collisions - 12 * m.potholeHits - 10 * m.nearMisses, 0, 100);
    var rules = [];
    if (m.breakerKmh != null) rules.push(m.breakerKmh <= 20 ? 100 : m.breakerKmh <= 26 ? 50 : 0);
    if (m.schoolOverPct != null) rules.push(m.schoolOverPct <= 10 ? 100 : m.schoolOverPct <= 40 ? 50 : 0);
    if (m.turnEntryKmh != null) rules.push(m.turnWide ? 0 : m.turnEntryKmh <= 22 ? 100 : m.turnEntryKmh <= 27 ? 50 : 0);
    if (m.ambulanceGiveS !== undefined) rules.push(m.ambulanceGiveS == null ? 0 : m.ambulanceGiveS <= 5 ? 100 : m.ambulanceGiveS <= 9 ? 50 : 0);
    if (m.shoulderCheck !== undefined) rules.push(m.shoulderCheck ? 100 : 0);
    if (m.puddleKmh != null) rules.push(m.puddleKmh <= 18 ? 100 : m.puddleKmh <= 25 ? 50 : 0);
    sc.rules = rules.length ? mean(rules) : null;
    sc.cognition = m.answersN ? 100 * m.answersCorrect / m.answersN : null;
    Object.keys(sc).forEach(function (k) { if (sc[k] != null) sc[k] = Math.round(sc[k]); });
    return sc;
  }
  var WEIGHTS = { control: 0.2, reaction: 0.2, attention: 0.2, hazards: 0.2, rules: 0.1, cognition: 0.1 };
  function fitnessIndex(sc) {
    var tot = 0, w = 0;
    Object.keys(WEIGHTS).forEach(function (k) { if (sc[k] != null) { tot += sc[k] * WEIGHTS[k]; w += WEIGHTS[k]; } });
    return w ? Math.round(tot / w) : null;
  }

  // history: array of earlier summaries [{valid, verdict, features:{...}}] for THIS rider, oldest first
  function baselineFrom(history) {
    var good = (history || []).filter(function (h) { return h.valid && h.verdict === 'FIT' && h.features && h.mode !== 'practice'; }).slice(-10);
    if (good.length < 3) return { n: good.length, ready: false };
    var b = { n: good.length, ready: true, med: {}, mad: {} };
    BASE_KEYS.forEach(function (k) {
      var vals = good.map(function (h) { return h.features[k]; }).filter(function (x) { return x != null && isFinite(x); });
      if (vals.length >= 3) { b.med[k] = median(vals); b.mad[k] = mad(vals); }
    });
    return b;
  }

  function assess(S, history) {
    var m = analyze(S), sc = subscores(m), index = fitnessIndex(sc);
    var base = baselineFrom(history), z = {}, comp = null;
    if (base.ready) {
      var acc = [], n = 0;
      BASE_KEYS.forEach(function (k) {
        if (base.med[k] == null || m[k] == null) return;
        var spread = Math.max(1.4826 * base.mad[k], BASE_FLOOR[k]);
        z[k] = round((m[k] - base.med[k]) / spread, 2);
        acc.push(Math.max(0, z[k]) * Math.max(0, z[k])); n++;
      });
      comp = n ? round(Math.sqrt(mean(acc)), 2) : null;
    }
    // validity checks (anti-gaming / headset not worn)
    var invalid = [];
    if (!m.completed) invalid.push('incomplete');
    if (m.headMotionDeg != null && m.headMotionDeg < 0.4) invalid.push('no_head_motion');
    if (m.drtAnticip >= 4) invalid.push('button_mashing');
    if (m.presses > 90) invalid.push('button_mashing');
    var level = 0, reasons = [];
    function bump(lv, key, params, sev) { level = Math.max(level, lv); reasons.push({ key: key, params: params || {}, sev: sev }); }
    if (m.collisions >= 2) bump(2, 'reason_collisions', { n: m.collisions }, 10);
    else if (m.collisions === 1) bump(1, 'reason_collision', {}, 8);
    if (m.drtMissRate != null && m.drtMissRate >= 0.40) bump(2, 'reason_missed_lights', { n: m.drtN - m.drtHits - m.drtAnticip, total: m.drtN - m.drtAnticip }, 9);
    else if (m.drtMissRate != null && m.drtMissRate >= 0.22) bump(1, 'reason_missed_lights', { n: m.drtN - m.drtHits - m.drtAnticip, total: m.drtN - m.drtAnticip }, 6);
    if (index != null && index < 50) bump(2, 'reason_low_index', { index: index }, 7);
    else if (index != null && index < 68) bump(1, 'reason_low_index', { index: index }, 4);
    if (comp != null) {
      if (comp >= 3.2) bump(2, 'reason_below_usual', { z: comp }, 9);
      else if (comp >= 2.0) bump(1, 'reason_below_usual', { z: comp }, 6);
      if (z.drtRT >= 2 && base.med.drtRT) reasons.push({ key: 'reason_slower_than_usual', params: { pct: Math.round(100 * (m.drtRT / base.med.drtRT - 1)) }, sev: 5 });
      if (z.sdlp >= 2) reasons.push({ key: 'reason_weaving', params: {}, sev: 5 });
    } else {
      if (m.sdlp != null && m.sdlp > 0.45) reasons.push({ key: 'reason_weaving', params: {}, sev: 4 });
    }
    if (m.brakeRT != null && m.brakeRT > 1.5) reasons.push({ key: 'reason_slow_brake', params: { s: round(m.brakeRT, 1) }, sev: 5 });
    if (m.brakeNoResponse > 0) reasons.push({ key: 'reason_no_brake', params: { n: m.brakeNoResponse }, sev: 6 });
    if (m.eyesOffMax > 2.0) reasons.push({ key: 'reason_eyes_off', params: { s: round(m.eyesOffMax, 1) }, sev: 5 });
    if (m.schoolOverPct != null && m.schoolOverPct > 40) reasons.push({ key: 'reason_school_speed', params: {}, sev: 4 });
    if (m.shoulderCheck === false) reasons.push({ key: 'reason_no_shoulder_check', params: {}, sev: 3 });
    if (m.ambulanceGiveS == null && m.ambulanceGiveS !== undefined) reasons.push({ key: 'reason_ambulance', params: {}, sev: 3 });
    if (m.answersN && m.answersCorrect < 2) reasons.push({ key: 'reason_memory', params: { n: m.answersCorrect, total: m.answersN }, sev: 3 });
    reasons.sort(function (a, b) { return b.sev - a.sev; });
    var verdict = invalid.length ? 'INVALID' : ['FIT', 'CAUTION', 'NOT_FIT'][level];
    if (verdict === 'FIT' && !reasons.length) reasons.push({ key: 'reason_all_good', params: {}, sev: 0 });
    var features = {};
    BASE_KEYS.forEach(function (k) { features[k] = m[k]; });
    return {
      metrics: m, scores: sc, index: index, verdict: verdict, invalid: invalid,
      reasons: reasons.slice(0, 4), baseline: { n: base.n, ready: base.ready, z: z, composite: comp },
      summary: { at: S.startedAt, valid: !invalid.length, verdict: verdict, index: index, mode: S.mode, features: features }
    };
  }

  // ---------------------------------------------------------------------------
  // 11. Synthetic riders (closed-loop human-operator model) for tests and demos
  // ---------------------------------------------------------------------------
  var BOT_PROFILES = {
    rested: { tau: 0.24, noise: 0.5, lapsePerMin: 0, lapseDur: [0.4, 0.8], drtMu: 0.40, drtSd: 0.07, drtMiss: 0.03,
      brakeMu: 0.72, brakeSd: 0.12, check: 0.9, glanceP: 0.15, glanceDur: 0.5, mem: 0.95, sa: 0.92, sjt: 0.95, comply: 0.95, preview: 1.0, gain: 1.0 },
    drowsy: { tau: 0.28, noise: 0.7, lapsePerMin: 0.8, lapseDur: [0.5, 1.0], drtMu: 0.49, drtSd: 0.11, drtMiss: 0.07,
      brakeMu: 0.88, brakeSd: 0.17, check: 0.75, glanceP: 0.3, glanceDur: 0.9, mem: 0.8, sa: 0.8, sjt: 0.9, comply: 0.85, preview: 1.05, gain: 0.92 },
    tired: { tau: 0.32, noise: 0.9, lapsePerMin: 2.0, lapseDur: [0.6, 1.4], drtMu: 0.58, drtSd: 0.15, drtMiss: 0.14,
      brakeMu: 1.05, brakeSd: 0.22, check: 0.55, glanceP: 0.45, glanceDur: 1.4, mem: 0.65, sa: 0.65, sjt: 0.85, comply: 0.7, preview: 1.15, gain: 0.85 },
    impaired: { tau: 0.37, noise: 1.3, lapsePerMin: 3.5, lapseDur: [0.8, 1.8], drtMu: 0.72, drtSd: 0.22, drtMiss: 0.28,
      brakeMu: 1.35, brakeSd: 0.30, check: 0.35, glanceP: 0.6, glanceDur: 2.1, mem: 0.45, sa: 0.5, sjt: 0.7, comply: 0.5, preview: 1.2, gain: 0.95 }
  };

  function Bot(profile, seed, traits) {
    var base = typeof profile === 'string' ? BOT_PROFILES[profile] : profile;
    this.name = typeof profile === 'string' ? profile : 'custom';
    this.p = Object.assign({}, base);
    traits = traits || {};
    if (traits.rt) { this.p.drtMu *= traits.rt; this.p.brakeMu *= traits.rt; this.p.tau *= Math.sqrt(traits.rt); }
    if (traits.noise) this.p.noise *= traits.noise;
    this.r = mulberry32((seed >>> 0) || 7);
    this.hist = []; this.hi = 0;
    this.lapseUntil = -1;
    this.nextLapse = this.p.lapsePerMin > 0 ? 6 + this.expo(60 / this.p.lapsePerMin) : 1e9;
    this.rollOut = 0; this.noise = 0; this.yaw = 0; this.yawT = 0; this.yawTarget = 0; this.pitch = -10;
    this.actions = []; this.notice = {}; this.decided = {}; this.yTarget = 0; this.brakeHold = null;
    this.brakeCount = {}; this.brakeLast = {};
    this.drtSeen = -1; this.glance = null; this.check = null; this.phoneDecided = false;
  }
  Bot.prototype.expo = function (meanS) { return -Math.log(1 - this.r()) * meanS; };
  Bot.prototype.lognorm = function (mu, sdv) {
    var s2 = Math.log(1 + (sdv * sdv) / (mu * mu)), m = Math.log(mu) - s2 / 2;
    return Math.exp(m + Math.sqrt(s2) * randn(this.r));
  };
  Bot.prototype.schedule = function (t, fn) { this.actions.push({ t: t, fn: fn }); };
  Bot.prototype.react = function (key, t0, mu, sdv) {     // time at which the bot notices something
    if (this.notice[key] == null) this.notice[key] = t0 + Math.max(0.25, mu + sdv * randn(this.r));
    return this.notice[key];
  };

  Bot.prototype.act = function (sim) {
    var t = sim.t, st = sim.st, p = this.p, r = this.r, self = this;
    this.hist.push({ t: t, y: st.y, psi: st.psi, v: st.v, s: st.s });
    while (this.hi < this.hist.length - 1 && this.hist[this.hi + 1].t <= t - p.tau) this.hi++;
    if (this.hi > 400) { this.hist.splice(0, this.hi - 10); this.hi = 10; }
    var per = this.hist[this.hi];
    // lapses (micro-sleeps / attention drop-outs): control freezes, nothing is noticed
    if (t >= this.nextLapse) { this.lapseUntil = t + rr(r, p.lapseDur[0], p.lapseDur[1]); this.nextLapse = this.lapseUntil + this.expo(60 / p.lapsePerMin); }
    var lapsing = t < this.lapseUntil;
    // scheduled button actions
    for (var i = 0; i < this.actions.length; i++) {
      var a = this.actions[i];
      if (a.t <= t) { a.fn(); this.actions.splice(i, 1); i--; }
    }
    if (!lapsing) {
      this.plan(sim);
      var v = Math.max(per.v, 1), Lp = Math.max(4.5, v * 1.0 * p.preview);
      var kA = curvatureAt(sim.path, per.s + Lp * 0.6);
      var yPred = per.y + Lp * Math.sin(per.psi);
      var rStar = kA * v + 2 * v * (this.yTarget - yPred) / (Lp * Lp) * p.gain;
      var cmd = sim.veh.control === 'lean' ? Math.atan(rStar * Math.max(v, 2.5) / G) * RAD : Math.atan(rStar * sim.veh.wheelBase / Math.max(v, 0.5)) * RAD;
      this.rollOut = steerInverse(clamp(cmd / sim.veh.maxCmdDeg, -1, 1));
    } else {
      this.rollOut *= 1 - CONFIG.dt * 1.4;          // during a lapse the head/hands relax: the bike stops following the road
    }
    this.noise = 0.97 * this.noise + 0.25 * p.noise * randn(r);
    // DRT: respond to the light unless lapsing or missed
    var d = sim.drt.cur;
    if (d && d.onset !== this.drtSeen) {
      this.drtSeen = d.onset;
      if (r() > p.drtMiss) {
        var rt = this.lognorm(p.drtMu, p.drtSd);
        var at = d.onset + rt;
        if (at < this.lapseUntil) at = this.lapseUntil + this.lognorm(p.drtMu, p.drtSd);
        if (at - d.onset <= 2.45) this.tap(at);
      }
    }
    // head yaw: relaxed scanning, shoulder checks override
    if (this.check && t >= this.check.t0 && t <= this.check.t1) this.yawTarget = this.check.yaw;
    else if (t >= this.yawT) { this.yawTarget = rr(r, -22, 22); this.yawT = t + rr(r, 1.5, 4); }
    this.yaw += (this.yawTarget - this.yaw) * Math.min(1, CONFIG.dt * 9);
    // head pitch: road ahead, phone glances during the call
    if (sim.phone.state === 'ring' && !this.phoneDecided) {
      this.phoneDecided = true;
      if (r() < p.glanceP) { var g0 = t + rr(r, 0.6, 1.2); this.glance = { t0: g0, t1: g0 + p.glanceDur * rr(r, 0.8, 1.2) }; }
    }
    this.pitch = (this.glance && t >= this.glance.t0 && t <= this.glance.t1) ? -38 : -10 + 2 * Math.sin(t * 0.7);
    return { roll: this.rollOut + this.noise, yaw: this.yaw, pitch: this.pitch };
  };

  Bot.prototype.tap = function (at) {
    var self = this;
    this.schedule(at, function () { self.sim.press(at); });
    this.schedule(at + 0.12, function () { if (!self.brakeHold) self.sim.release(at + 0.12); });
  };
  // Hold the brake for one hazard until `untilFn` says stop. A hazard gets one brake application,
  // unless `repeat` is set (keeping speed down in a school zone, waiting for a cow).
  Bot.prototype.holdBrake = function (sim, why, untilFn, repeat) {
    if (this.brakeHold) return;
    var n = this.brakeCount[why] || 0;
    if (n > 0 && (!repeat || sim.t - (this.brakeLast[why] || 0) < 1.0)) return;
    if (untilFn()) return;
    this.brakeCount[why] = n + 1;
    this.brakeHold = { why: why, until: untilFn };
    sim.press(sim.t);
  };

  // Decide lane target and braking from what the rider can see (with reaction delays)
  Bot.prototype.plan = function (sim) {
    var st = sim.st, t = sim.t, p = this.p, r = this.r, self = this;
    this.sim = sim;
    var yT = 0;
    if (this.brakeHold && this.brakeHold.until()) { this.brakeLast[this.brakeHold.why] = t; this.brakeHold = null; sim.release(t); }
    sim.events.forEach(function (ev) {
      if (ev.state === 'pending') return;
      var key = ev.id, k = ev.kind;
      if (k === 'pothole' || k === 'pothole_hidden') {
        if (ev.state !== 'done' && ev.pS - st.s < 30 && ev.pS - st.s > -1) {
          if (t >= self.react(key, t, 0.6, 0.15)) {
            var side = ev.pY > 0 ? -1 : 1;
            var yAvoid = ev.pY + side * (ev.pR + 0.65);
            if (k === 'pothole_hidden') {
              if (self.decided[key] == null) self.decided[key] = r() < p.comply;
              if (self.decided[key] && ev.pS - st.s < 24 && st.v > 4.6) self.holdBrake(sim, key, function () { return sim.st.v < 4.3 || sim.st.s > ev.pS; });
              yAvoid = st.y;                       // the hidden pothole cannot be seen: no swerve
            }
            yT = clamp(yAvoid, -1.35, 1.35);
          }
        }
      } else if (k === 'breaker' && ev.state === 'armed') {
        if (self.decided[key] == null) self.decided[key] = r() < p.comply;
        var dB = (st.v * st.v - 17.6) / 9 + 2.5;
        if (self.decided[key] && ev.bS - st.s < dB && ev.bS - st.s > 0 && st.v > 4.6) self.holdBrake(sim, key, function () { return sim.st.v < 4.2 || sim.st.s > ev.bS; });
      } else if (k === 'sharp_turn' && ev.state === 'armed') {
        if (self.decided[key] == null) self.decided[key] = r() < p.comply;
        var target = self.decided[key] ? 5.0 : 6.6, startAt = Math.max(0, (st.v * st.v - target * target) / 8) + (self.decided[key] ? 4 : 0.5);
        if (ev.turnS - st.s < startAt && ev.turnS - st.s > 0 && st.v > target + 0.2) self.holdBrake(sim, key, function () { return sim.st.v < target || sim.st.s > ev.turnS; });
      } else if (k === 'school_zone' && ev.state === 'armed') {
        if (self.decided[key] == null) self.decided[key] = r() < p.comply;
        if (self.decided[key] && st.s > ev.signS - 6 && st.s < ev.zoneS1 && st.v > 6.4) {
          if (t >= self.react(key, t, 0.8, 0.2)) self.holdBrake(sim, key, function () { return sim.st.v < 6.2; }, true);
        }
      } else if (k === 'jaywalker' && ev.onsetT != null && ev.state !== 'done') {
        var tn = self.react(key, ev.onsetT, p.brakeMu, p.brakeSd);
        if (t >= tn && t >= self.lapseUntil) self.holdBrake(sim, key, function () { return sim.st.v < 0.6 || ev.ped.y > 0.8 || ev.state === 'done'; });
      } else if (k === 'lead_stop' && ev.auto) {
        var a = ev.auto;
        if (ev.onsetT != null && (ev.state === 'braking' || ev.state === 'stopped')) {
          var tn2 = self.react(key, ev.onsetT, p.brakeMu, p.brakeSd);
          if (t >= tn2 && t >= self.lapseUntil) self.holdBrake(sim, key, function () { return sim.st.v < 0.4 || a.y < -1.6 || (a.s - sim.st.s > 6 && sim.st.v < 2.5 && a.v === 0); });
        }
        if ((ev.state === 'stopped' || ev.state === 'pullover') && a.y > -1.6 && a.s - st.s < 14 && a.s - st.s > -3 && st.v < 1.5 && t > (self.notice[key] || 0) + 1.2) yT = 1.25;
      } else if (k === 'wrong_side' && ev.moto && ev.state === 'active') {
        var tn3 = self.react(key, ev.onsetT, p.brakeMu + 0.6, p.brakeSd);
        if (t >= tn3) { yT = -0.95; if (ev.moto.s - st.s < 16 && st.v > 6) self.holdBrake(sim, key, function () { return sim.st.v < 5.5 || ev.state === 'done'; }); }
      } else if (k === 'ambulance' && ev.state === 'active') {
        var tn4 = self.react(key, ev.onsetT, p.brakeMu * 1.6, p.brakeSd * 2);
        if (self.decided[key] == null) self.decided[key] = r() < p.comply;
        if (t >= tn4 && self.decided[key]) yT = -0.9;
        else if (t >= tn4 + 6) yT = -0.9;
      } else if (k === 'lane_change' && ev.state === 'armed' && ev.truck) {
        var rear = ev.truckS - ev.truckLen / 2, distR = rear - st.s;
        if (self.decided[key] == null) self.decided[key] = r() < p.check;
        if (self.decided[key] && distR < 30 && !self.check) self.check = { t0: t, t1: t + rr(r, 0.45, 0.7), yaw: rr(r, 55, 80) };
        var bike = ev.bike, coming = bike && !bike.gone && bike.s > st.s - 25 && bike.s < st.s + 3;
        var waitForBike = self.decided[key] && coming && t > (self.check ? self.check.t1 : 0);
        if (distR < 22 && st.s < ev.truckS + ev.truckLen / 2 + 3) {
          if (waitForBike) { yT = Math.min(st.y, 0); self.waiting = true; self.holdBrake(sim, key, function () { return !(bike && !bike.gone && bike.s < sim.st.s + 4); }); }
          else { yT = 0.85; self.waiting = false; }
        }
      } else if (k === 'animal' && ev.ani && ev.state !== 'done') {
        if (ev.animal === 'cow') {
          if (ev.onsetT != null && t >= self.react(key, ev.onsetT, p.brakeMu, p.brakeSd)) {
            var c = ev.ani, inPath = c.y + c.halfY > st.y - 0.6 && c.y - c.halfY < st.y + 0.6;
            if (inPath && ev.aS - st.s < 22 && ev.aS - st.s > 0 && st.v > 1) self.holdBrake(sim, key, function () { return sim.st.v < 0.8 || !(c.y + c.halfY > sim.st.y - 0.6); }, true);
          }
        } else if (ev.onsetT != null) {
          var tn5 = self.react(key, ev.onsetT, p.brakeMu, p.brakeSd);
          if (t >= tn5 && t >= self.lapseUntil) self.holdBrake(sim, key, function () { return sim.st.v < 1.0 || ev.ani.y < -1.6 || ev.state === 'done'; });
        }
      }
    });
    var yMin = this.waiting && this.brakeHold ? -9 : -1.35;
    if (!this.brakeHold) this.waiting = false;
    for (var q = 0; yMin > -9 && q < sim.actors.length; q++) {
      var ob = sim.actors[q];
      if (ob.gone || !ob.solid || ob.ambient || ob.y > 0 || ob.v > 0.2) continue;
      if (ob.s + ob.halfS > st.s - 3 && ob.s - ob.halfS < st.s + 14) yMin = Math.max(yMin, ob.y + ob.halfY + sim.veh.halfWidth + 0.3);
    }
    if (yT < yMin) yT = yMin;
    this.yTarget += (yT - this.yTarget) * 0.08;
  };

  Bot.prototype.answer = function (q) {
    var p = this.p, prob = q.kind === 'memory' ? p.mem : q.kind === 'awareness' ? p.sa : p.sjt;
    var choice = this.r() < prob ? q.correct : (q.correct + 1 + Math.floor(this.r() * 2)) % 3;
    return { id: q.id, choice: choice, correct: choice === q.correct, rtS: round(rr(this.r, 1.6, 4.5) * (p.tau / 0.24), 2) };
  };

  function runSession(opts) {
    opts = opts || {};
    var route = buildRoute(opts.seed || 1);
    var sim = new Sim(route, { vehicle: opts.vehicle || 'bike', mode: opts.mode || 'check', windScale: opts.windScale });
    var bot = new Bot(opts.bot || 'rested', (opts.seed || 1) * 7919 + 13, opts.traits);
    bot.sim = sim;
    while (!sim.done) sim.step(CONFIG.dt, bot.act(sim));
    var qs = makeQuestions(route);
    var answers = qs.map(function (q) { return bot.answer(q); });
    return sim.finish({
      riderId: opts.riderId || 'SIM', simulated: true, condition: bot.name, answers: answers, lang: opts.lang || 'en',
      startedAt: opts.startedAt || new Date(Date.UTC(2026, 10, 26, 4, 0, 0)).toISOString()
    });
  }

  var API = {
    VERSION: VERSION, CONFIG: CONFIG, VEHICLES: VEHICLES, BLUEPRINT: BLUEPRINT, NORMS: NORMS, WEIGHTS: WEIGHTS, BOT_PROFILES: BOT_PROFILES,
    mulberry32: mulberry32, hashString: hashString, randn: randn, round: round, mean: mean, sd: sd, median: median, mad: mad, clamp: clamp, wrapDeg: wrapDeg,
    buildPath: buildPath, poseAt: poseAt, curvatureAt: curvatureAt, segIndex: segIndex, buildRoute: buildRoute,
    steerNorm: steerNorm, steerInverse: steerInverse, OneEuro: OneEuro,
    Sim: Sim, makeQuestions: makeQuestions, analyze: analyze, assess: assess, subscores: subscores, fitnessIndex: fitnessIndex,
    baselineFrom: baselineFrom, identifyLag: identifyLag, cleanMask: cleanMask, Bot: Bot, runSession: runSession
  };
  if (typeof module !== 'undefined' && module.exports) module.exports = API;
  else root.HC2 = API;
})(typeof window !== 'undefined' ? window : this);
