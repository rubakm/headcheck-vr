/*
 * HeadCheck VR - core logic (shared by the VR ride page and the PC dashboard)
 * ---------------------------------------------------------------------------
 * Everything here is plain JavaScript with no browser dependency, so it can be
 * unit-tested in Node:  node test-core.js
 *
 * Conventions
 *   - Distance along the route is "s" in metres.
 *   - Heading h is in radians: 0 = north (-z in A-Frame), +90deg = east (a right turn).
 *   - "lateral" is metres to the RIDER'S RIGHT of the route line (negative = left).
 *   - Head yaw is in degrees RELATIVE TO THE SCOOTER: positive = looking right,
 *     negative = looking left, 0 = straight ahead.
 */
(function (root) {
  'use strict';

  var DEG = Math.PI / 180;

  // ---------------------------------------------------------------------------
  // 1. The standard test scenario (identical for every rider, so scores compare)
  // ---------------------------------------------------------------------------
  var SCENARIO = {
    name: 'City delivery route v1',
    leadInS: 3,            // seconds stationary before the ride starts
    promptLeadS: 6,        // voice prompt plays this many seconds before a manoeuvre
    segments: [
      { type: 'straight', len: 200 },
      { type: 'arc', r: 10, dir: +1, angleDeg: 90 },   // junction 1: right turn
      { type: 'straight', len: 220 },
      { type: 'arc', r: 10, dir: -1, angleDeg: 90 },   // junction 2: left turn
      { type: 'straight', len: 260 },
      { type: 'arc', r: 10, dir: +1, angleDeg: 90 },   // junction 3: right turn
      { type: 'straight', len: 180 }
    ],
    // Lane offset changes (right lane centre is 3.5 m to the right of the left lane)
    laneChanges: [
      { s0: 540, s1: 560, from: 0, to: 3.5 },   // move out to pass the parked truck
      { s0: 620, s1: 640, from: 3.5, to: 0 }    // merge back into the left lane
    ],
    // Manoeuvres: a shoulder check toward `side` is expected between prompt and action
    manoeuvres: [
      { id: 'M1', label: 'Turn right at junction 1', side: 'right', actionS: 200, prompt: 'Turn right ahead',
        blindSpot: { kind: 'bike', lateral: 2.6 } },
      { id: 'M2', label: 'Turn left at junction 2', side: 'left', actionS: 435.708, prompt: 'Turn left ahead',
        blindSpot: { kind: 'cycle', lateral: -1.4 } },
      { id: 'M3', label: 'Change lane right (parked truck)', side: 'right', actionS: 540, prompt: 'Change lane to the right',
        blindSpot: { kind: 'car', lateral: 3.5 } },
      { id: 'M4', label: 'Merge back left', side: 'left', actionS: 620, prompt: 'Merge back to the left lane',
        blindSpot: null },
      { id: 'M5', label: 'Turn right at junction 3', side: 'right', actionS: 711.416, prompt: 'Turn right ahead',
        blindSpot: null }
    ],
    // Hazards. kind 'tap' = rider must tap; kind 'look' = rider must turn head toward `side`
    hazards: [
      { id: 'H1', label: 'Horn from behind-right (overtaking bike)', kind: 'look', side: 'right', onsetS: 60, windowS: 3 },
      { id: 'H2', label: 'Pedestrian steps out between parked cars', kind: 'tap', onsetS: 300, windowS: 4, atS: 325 },
      { id: 'H3', label: 'Parked car door opens', kind: 'tap', onsetS: 470, windowS: 4, atS: 490 },
      { id: 'H4', label: 'Vehicle U-turns through median gap', kind: 'tap', onsetS: 760, windowS: 4, atS: 790 },
      { id: 'H5', label: 'Dog runs across the road', kind: 'tap', onsetS: 840, windowS: 4, atS: 862 }
    ],
    truck: { s: 575, lateral: 0 },
    medianGapS: 792
  };

  var DEFAULTS = {
    speed: 7,          // m/s (about 25 km/h)
    checkDeg: 60,      // head yaw toward the side that counts as a shoulder check
    lookDeg: 45,       // head yaw toward a horn that counts as "looked"
    minCheckS: 0.1,    // the yaw must stay past the threshold at least this long
    sampleHz: 20
  };

  // ---------------------------------------------------------------------------
  // 2. Route geometry
  // ---------------------------------------------------------------------------
  function fwd(h) { return [Math.sin(h), -Math.cos(h)]; }   // [x, z]
  function right(h) { return [Math.cos(h), Math.sin(h)]; }  // [x, z]

  function segPose(g, d) {
    if (g.type === 'straight') {
      var f = fwd(g.h0);
      return { x: g.x0 + f[0] * d, z: g.z0 + f[1] * d, h: g.h0 };
    }
    var r0 = right(g.h0);
    var cx = g.x0 + g.dir * g.r * r0[0];
    var cz = g.z0 + g.dir * g.r * r0[1];
    var h = g.h0 + g.dir * d / g.r;
    var r1 = right(h);
    return { x: cx - g.dir * g.r * r1[0], z: cz - g.dir * g.r * r1[1], h: h };
  }

  function buildPath(segments) {
    var x = 0, z = 0, h = 0, s = 0, out = [];
    segments.forEach(function (g) {
      var len = g.type === 'straight' ? g.len : g.r * g.angleDeg * DEG;
      var seg = Object.assign({}, g, { s0: s, len: len, x0: x, z0: z, h0: h });
      out.push(seg);
      var end = segPose(seg, len);
      x = end.x; z = end.z; h = end.h; s += len;
    });
    return { segs: out, length: s };
  }

  function segmentAt(path, s) {
    for (var i = 0; i < path.segs.length; i++) {
      var g = path.segs[i];
      if (s <= g.s0 + g.len) return g;
    }
    return path.segs[path.segs.length - 1];
  }

  /** World pose at distance s with a lateral offset (metres to the right). */
  function poseAt(path, s, lateral) {
    s = Math.max(0, Math.min(path.length, s));
    var g = segmentAt(path, s);
    var p = segPose(g, s - g.s0);
    if (lateral) {
      var r = right(p.h);
      p.x += r[0] * lateral; p.z += r[1] * lateral;
    }
    p.s = s;
    return p;
  }

  function smoothstep(a, b, x) {
    var t = Math.max(0, Math.min(1, (x - a) / (b - a)));
    return t * t * (3 - 2 * t);
  }

  /** The rider's lane offset at distance s (0 = left lane, 3.5 = right lane). */
  function laneOffsetAt(s, scenario) {
    scenario = scenario || SCENARIO;
    var off = 0;
    scenario.laneChanges.forEach(function (lc) {
      if (s >= lc.s0) off = lc.from + (lc.to - lc.from) * smoothstep(lc.s0, lc.s1, s);
    });
    return off;
  }

  /** Time (s from ride start) at which the rider reaches distance s, for a constant speed. */
  function timeAtS(s, speed, scenario) {
    scenario = scenario || SCENARIO;
    return scenario.leadInS + s / speed;
  }

  function wrapDeg(a) {
    a = ((a + 180) % 360 + 360) % 360 - 180;
    return a;
  }

  // ---------------------------------------------------------------------------
  // 3. Scoring
  // ---------------------------------------------------------------------------
  function sideSign(side) { return side === 'left' ? -1 : 1; }

  /**
   * Find runs where the (signed) yaw stays >= threshold for at least minDur seconds.
   * series: { t: [...], yaw: [...] }, sign: +1 (right) / -1 (left) / 0 (either side).
   */
  function findRuns(series, t0, t1, sign, threshold, minDur, hz) {
    var runs = [], start = -1, last = -1, peak = 0;
    var dtSample = 1 / (hz || DEFAULTS.sampleHz);
    function close() {
      if (start >= 0) {
        var dur = series.t[last] - series.t[start] + dtSample;
        if (dur >= minDur) runs.push({ t: series.t[start], tEnd: series.t[last], dur: dur, peak: peak });
      }
      start = -1; peak = 0;
    }
    for (var i = 0; i < series.t.length; i++) {
      var t = series.t[i];
      if (t < t0) continue;
      if (t > t1) break;
      var v = sign === 0 ? Math.abs(series.yaw[i]) : series.yaw[i] * sign;
      if (v >= threshold) {
        if (start < 0) start = i;
        last = i;
        if (v > peak) peak = v;
      } else {
        close();
      }
    }
    close();
    return runs;
  }

  function maxToward(series, t0, t1, sign) {
    var m = 0;
    for (var i = 0; i < series.t.length; i++) {
      var t = series.t[i];
      if (t < t0) continue;
      if (t > t1) break;
      var v = series.yaw[i] * sign;
      if (v > m) m = v;
    }
    return m;
  }

  function evaluateManoeuvre(series, m, opts) {
    var sign = sideSign(m.side);
    var runs = findRuns(series, m.promptT, m.actionT, sign, opts.checkDeg, opts.minCheckS, opts.sampleHz);
    var wrongRuns = findRuns(series, m.promptT, m.actionT, -sign, opts.checkDeg, opts.minCheckS, opts.sampleHz);
    var checked = runs.length > 0;
    var firstCheckT = checked ? runs[0].t : null;
    return {
      id: m.id, label: m.label, side: m.side,
      promptT: round(m.promptT, 3), actionT: round(m.actionT, 3),
      blindSpot: !!m.blindSpot,
      checked: checked,
      firstCheckT: checked ? round(firstCheckT, 3) : null,
      leadTimeS: checked ? round(m.actionT - firstCheckT, 2) : null,
      maxYawTowardSide: round(maxToward(series, m.promptT, m.actionT, sign), 1),
      checkedWrongSideOnly: !checked && wrongRuns.length > 0,
      nearMiss: !checked && !!m.blindSpot
    };
  }

  function evaluateHazards(series, taps, hazards, opts) {
    var usedTap = {};
    var results = hazards.map(function (hz) {
      var t0 = hz.onsetT, t1 = hz.onsetT + hz.windowS;
      if (hz.kind === 'look') {
        var runs = findRuns(series, t0, t1, sideSign(hz.side), opts.lookDeg, opts.minCheckS, opts.sampleHz);
        return {
          id: hz.id, label: hz.label, kind: hz.kind, onsetT: round(t0, 3),
          responded: runs.length > 0,
          rtMs: runs.length ? Math.round((runs[0].t - t0) * 1000) : null
        };
      }
      var hit = null;
      for (var i = 0; i < taps.length; i++) {
        if (taps[i] >= t0 && taps[i] <= t1 && !usedTap[i]) { hit = i; break; }
      }
      if (hit !== null) usedTap[hit] = true;
      return {
        id: hz.id, label: hz.label, kind: hz.kind, onsetT: round(t0, 3),
        responded: hit !== null,
        rtMs: hit !== null ? Math.round((taps[hit] - t0) * 1000) : null
      };
    });
    // A tap is "false" if it is not inside ANY tap-hazard window.
    var falseTaps = taps.filter(function (t) {
      return !hazards.some(function (hz) {
        return hz.kind === 'tap' && t >= hz.onsetT && t <= hz.onsetT + hz.windowS;
      });
    }).length;
    return { hazards: results, falseTaps: falseTaps };
  }

  function scanStats(series, rideStartT, opts) {
    var vals = [];
    for (var i = 0; i < series.t.length; i++) if (series.t[i] >= rideStartT) vals.push(series.yaw[i]);
    var off = vals.filter(function (v) { return Math.abs(v) > 30; }).length;
    var checks = findRuns(series, rideStartT, Infinity, 0, opts.checkDeg, opts.minCheckS, opts.sampleHz);
    return {
      yawSD: round(sd(vals), 1),
      pctTimeLookingAway30: vals.length ? round(100 * off / vals.length, 1) : 0,
      headCheckEvents: checks.length
    };
  }

  function bandFor(score) {
    if (score >= 80) return { key: 'safe', label: 'Safe' };
    if (score >= 50) return { key: 'coach', label: 'Needs coaching' };
    return { key: 'risk', label: 'High risk' };
  }

  /**
   * Evaluate a ride record. The record must contain:
   *   series {t, yaw}, taps [t...], events.manoeuvres [...promptT, actionT],
   *   events.hazards [...onsetT, windowS], settings (optional overrides)
   * Returns a NEW object = record + scores (the raw data is kept so the
   * dashboard can re-score with a different threshold).
   */
  function evaluateRide(record, override) {
    var opts = Object.assign({}, DEFAULTS, record.settings || {}, override || {});
    var series = record.series;
    var man = record.events.manoeuvres.map(function (m) { return evaluateManoeuvre(series, m, opts); });
    var hz = evaluateHazards(series, record.taps || [], record.events.hazards, opts);
    var checked = man.filter(function (m) { return m.checked; }).length;
    var detected = hz.hazards.filter(function (h) { return h.responded; }).length;
    var tapRts = hz.hazards.filter(function (h) { return h.kind === 'tap' && h.responded; }).map(function (h) { return h.rtMs; });
    var look = hz.hazards.filter(function (h) { return h.kind === 'look'; })[0];
    var headCheckPct = man.length ? Math.round(100 * checked / man.length) : 0;
    var hazardPct = hz.hazards.length ? Math.round(100 * detected / hz.hazards.length) : 0;
    var overall = clamp(Math.round(0.5 * headCheckPct + 0.5 * hazardPct - 5 * hz.falseTaps), 0, 100);
    var leadTimes = man.filter(function (m) { return m.checked; }).map(function (m) { return m.leadTimeS; });
    var out = Object.assign({}, record);
    out.settingsUsed = { checkDeg: opts.checkDeg, lookDeg: opts.lookDeg, minCheckS: opts.minCheckS };
    out.scores = {
      overall: overall,
      band: bandFor(overall).label,
      headCheckPct: headCheckPct,
      checksDone: checked, checksExpected: man.length,
      nearMisses: man.filter(function (m) { return m.nearMiss; }).length,
      meanCheckLeadS: leadTimes.length ? round(mean(leadTimes), 2) : null,
      hazardPct: hazardPct,
      hazardsSpotted: detected, hazardsTotal: hz.hazards.length,
      meanTapRtMs: tapRts.length ? Math.round(mean(tapRts)) : null,
      hornLookMs: look && look.responded ? look.rtMs : null,
      falseTaps: hz.falseTaps
    };
    out.manoeuvres = man;
    out.hazards = hz.hazards;
    out.scan = scanStats(series, (record.rideStartT || 0), opts);
    return out;
  }

  // ---------------------------------------------------------------------------
  // 4. Statistics for the dashboard (group comparison)
  // ---------------------------------------------------------------------------
  function mean(a) { return a.length ? a.reduce(function (x, y) { return x + y; }, 0) / a.length : NaN; }
  function sd(a) {
    if (a.length < 2) return 0;
    var m = mean(a);
    return Math.sqrt(a.reduce(function (acc, v) { return acc + (v - m) * (v - m); }, 0) / (a.length - 1));
  }
  function round(v, d) { var k = Math.pow(10, d || 0); return Math.round(v * k) / k; }
  function clamp(v, a, b) { return Math.max(a, Math.min(b, v)); }

  function lgamma(x) {
    var c = [76.18009172947146, -86.50532032941677, 24.01409824083091,
      -1.231739572450155, 0.1208650973866179e-2, -0.5395239384953e-5];
    var y = x, tmp = x + 5.5;
    tmp -= (x + 0.5) * Math.log(tmp);
    var ser = 1.000000000190015;
    for (var j = 0; j < 6; j++) ser += c[j] / ++y;
    return -tmp + Math.log(2.5066282746310005 * ser / x);
  }
  function betacf(a, b, x) {
    var MAXIT = 200, EPS = 3e-14, FPMIN = 1e-300;
    var qab = a + b, qap = a + 1, qam = a - 1, c = 1, d = 1 - qab * x / qap;
    if (Math.abs(d) < FPMIN) d = FPMIN;
    d = 1 / d;
    var h = d;
    for (var m = 1; m <= MAXIT; m++) {
      var m2 = 2 * m, aa = m * (b - m) * x / ((qam + m2) * (a + m2));
      d = 1 + aa * d; if (Math.abs(d) < FPMIN) d = FPMIN;
      c = 1 + aa / c; if (Math.abs(c) < FPMIN) c = FPMIN;
      d = 1 / d; h *= d * c;
      aa = -(a + m) * (qab + m) * x / ((a + m2) * (qap + m2));
      d = 1 + aa * d; if (Math.abs(d) < FPMIN) d = FPMIN;
      c = 1 + aa / c; if (Math.abs(c) < FPMIN) c = FPMIN;
      d = 1 / d;
      var del = d * c; h *= del;
      if (Math.abs(del - 1) < EPS) break;
    }
    return h;
  }
  function ibeta(a, b, x) {
    if (x <= 0) return 0;
    if (x >= 1) return 1;
    var bt = Math.exp(lgamma(a + b) - lgamma(a) - lgamma(b) + a * Math.log(x) + b * Math.log(1 - x));
    if (x < (a + 1) / (a + b + 2)) return bt * betacf(a, b, x) / a;
    return 1 - bt * betacf(b, a, 1 - x) / b;
  }
  /** Two-tailed p-value of Student's t with df degrees of freedom. */
  function tTwoTailedP(t, df) { return ibeta(df / 2, 0.5, df / (df + t * t)); }

  /** Welch's t-test (unequal variances) plus Cohen's d. Returns null if not computable. */
  function welchTest(a, b) {
    if (a.length < 2 || b.length < 2) return null;
    var m1 = mean(a), m2 = mean(b), v1 = Math.pow(sd(a), 2), v2 = Math.pow(sd(b), 2);
    var se2 = v1 / a.length + v2 / b.length;
    if (se2 === 0) return null;
    var t = (m1 - m2) / Math.sqrt(se2);
    var df = se2 * se2 / (Math.pow(v1 / a.length, 2) / (a.length - 1) + Math.pow(v2 / b.length, 2) / (b.length - 1));
    var pooled = Math.sqrt(((a.length - 1) * v1 + (b.length - 1) * v2) / (a.length + b.length - 2));
    return { meanA: m1, meanB: m2, sdA: Math.sqrt(v1), sdB: Math.sqrt(v2), t: t, df: df,
      p: tTwoTailedP(t, df), cohenD: pooled ? (m1 - m2) / pooled : null };
  }

  // ---------------------------------------------------------------------------
  // 5. Simulated riders (for the dashboard demo and for testing the scorer)
  // ---------------------------------------------------------------------------
  function mulberry32(seed) {
    return function () {
      seed |= 0; seed = seed + 0x6D2B79F5 | 0;
      var t = Math.imul(seed ^ seed >>> 15, 1 | seed);
      t = t + Math.imul(t ^ t >>> 7, 61 | t) ^ t;
      return ((t ^ t >>> 14) >>> 0) / 4294967296;
    };
  }

  /** Scenario events converted to times for a constant-speed ride. */
  function scheduledEvents(speed, scenario) {
    scenario = scenario || SCENARIO;
    return {
      manoeuvres: scenario.manoeuvres.map(function (m) {
        var actionT = timeAtS(m.actionS, speed, scenario);
        return { id: m.id, label: m.label, side: m.side, blindSpot: m.blindSpot,
          promptT: actionT - scenario.promptLeadS, actionT: actionT };
      }),
      hazards: scenario.hazards.map(function (h) {
        return { id: h.id, label: h.label, kind: h.kind, side: h.side,
          onsetT: timeAtS(h.onsetS, speed, scenario), windowS: h.windowS };
      })
    };
  }

  /**
   * Simulate one rider. profile: 'experienced' | 'new'. Returns a raw ride record
   * in exactly the format the VR page produces (with simulated: true).
   */
  function simulateRider(profile, seed, id) {
    var rnd = mulberry32(seed);
    var exp = profile === 'experienced';
    var speed = DEFAULTS.speed, hz = DEFAULTS.sampleHz;
    var ev = scheduledEvents(speed);
    var path = buildPath(SCENARIO.segments);
    var duration = timeAtS(path.length, speed) + 0.5;
    var n = Math.floor(duration * hz);
    var yaw = new Array(n).fill(0), t = [];
    var drift = 0;
    for (var i = 0; i < n; i++) {
      t.push(round(i / hz, 3));
      drift += (rnd() - 0.5) * 1.2 - drift * 0.02;
      yaw[i] = drift;
    }
    function bump(center, dur, peak) {
      for (var i = 0; i < n; i++) {
        var x = (t[i] - center) / (dur / 2);
        if (Math.abs(x) <= 1) yaw[i] += peak * (0.5 + 0.5 * Math.cos(Math.PI * x));
      }
    }
    // casual glances
    var glances = exp ? 14 : 8;
    for (var g = 0; g < glances; g++) bump(rnd() * duration, 0.6 + rnd() * 0.8, (rnd() < 0.5 ? -1 : 1) * (15 + rnd() * 20));
    // shoulder checks
    ev.manoeuvres.forEach(function (m) {
      var p = exp ? 0.88 : 0.35;
      if (rnd() < p) {
        var lead = exp ? 1.5 + rnd() * 2.5 : 0.6 + rnd() * 1.6;
        var peak = exp ? 72 + rnd() * 25 : 55 + rnd() * 30;
        bump(m.actionT - lead, 1.0 + rnd() * 0.5, sideSign(m.side) * peak);
      } else if (!exp && rnd() < 0.3) {
        bump(m.actionT - 1, 0.8, sideSign(m.side) * (25 + rnd() * 20)); // half-hearted glance
      }
    });
    // horn response + taps
    var taps = [];
    ev.hazards.forEach(function (h) {
      if (h.kind === 'look') {
        if (rnd() < (exp ? 0.85 : 0.5)) bump(h.onsetT + (exp ? 0.6 : 1.0) + rnd() * 0.8 + 0.4, 1.1, sideSign(h.side) * (55 + rnd() * 30));
      } else if (rnd() < (exp ? 0.92 : 0.7)) {
        taps.push(round(h.onsetT + (exp ? 0.7 + rnd() * 0.6 : 1.1 + rnd() * 1.2), 3));
      }
    });
    var falseN = exp ? (rnd() < 0.25 ? 1 : 0) : (rnd() < 0.6 ? 1 + Math.floor(rnd() * 2) : 0);
    // false taps are placed outside every hazard window, so they really are "false"
    var placed = 0, guard = 0;
    while (placed < falseN && guard++ < 100) {
      var ft = round(5 + rnd() * (duration - 10), 3);
      var inWindow = ev.hazards.some(function (h) { return ft >= h.onsetT - 0.5 && ft <= h.onsetT + h.windowS + 0.5; });
      if (!inWindow) { taps.push(ft); placed++; }
    }
    taps.sort(function (a, b) { return a - b; });
    return {
      app: 'HeadCheck VR', version: '1.0', simulated: true,
      riderId: id || (profile + '-' + seed),
      group: exp ? 'Experienced rider' : 'New / non-rider',
      yearsRiding: exp ? 2 + Math.floor(rnd() * 8) : 0,
      startedAt: new Date(Date.UTC(2026, 10, 26, 10, 0, 0) + seed * 60000).toISOString(),
      mode: 'simulated',
      settings: { speed: speed, checkDeg: DEFAULTS.checkDeg, lookDeg: DEFAULTS.lookDeg, sampleHz: hz },
      rideStartT: 0,
      durationS: round(duration, 1),
      series: { t: t, yaw: yaw.map(function (v) { return round(wrapDeg(v), 1); }) },
      taps: taps,
      events: ev
    };
  }

  var API = {
    SCENARIO: SCENARIO, DEFAULTS: DEFAULTS,
    buildPath: buildPath, poseAt: poseAt, segmentAt: segmentAt, laneOffsetAt: laneOffsetAt,
    timeAtS: timeAtS, wrapDeg: wrapDeg, smoothstep: smoothstep,
    findRuns: findRuns, evaluateManoeuvre: evaluateManoeuvre, evaluateHazards: evaluateHazards,
    evaluateRide: evaluateRide, bandFor: bandFor, scanStats: scanStats,
    mean: mean, sd: sd, round: round, welchTest: welchTest, tTwoTailedP: tTwoTailedP,
    scheduledEvents: scheduledEvents, simulateRider: simulateRider, mulberry32: mulberry32
  };

  if (typeof module !== 'undefined' && module.exports) module.exports = API;
  else root.HC = API;
})(typeof window !== 'undefined' ? window : this);
