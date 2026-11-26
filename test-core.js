// Unit tests for headcheck-core.js  ->  run:  node test-core.js
const HC = require('./headcheck-core.js');
let pass = 0, fail = 0;
function ok(cond, msg) { if (cond) { pass++; } else { fail++; console.log('FAIL:', msg); } }
function near(a, b, tol, msg) { ok(Math.abs(a - b) <= tol, msg + ` (got ${a}, want ${b})`); }

// --- Route geometry ---------------------------------------------------------
const path = HC.buildPath(HC.SCENARIO.segments);
near(path.length, 907.124, 0.01, 'route length');
let p = HC.poseAt(path, 200, 0);
near(p.x, 0, 1e-9, 'junction 1 x'); near(p.z, -200, 1e-9, 'junction 1 z'); near(p.h, 0, 1e-9, 'heading north');
p = HC.poseAt(path, 200 + Math.PI * 5, 0); // end of right-turn arc
near(p.x, 10, 1e-6, 'after right turn x'); near(p.z, -210, 1e-6, 'after right turn z');
near(p.h, Math.PI / 2, 1e-9, 'heading east after right turn');
const pr = HC.poseAt(path, 100, 3.5);
near(pr.x, 3.5, 1e-9, 'lateral offset goes to the rider\'s right');
// continuity: no jumps along the route
let maxJump = 0, prev = HC.poseAt(path, 0, 0);
for (let s = 0.5; s <= path.length; s += 0.5) {
  const q = HC.poseAt(path, s, 0);
  maxJump = Math.max(maxJump, Math.hypot(q.x - prev.x, q.z - prev.z)); prev = q;
}
ok(maxJump <= 0.5001, 'route is continuous (max step ' + maxJump.toFixed(4) + ')');
p = HC.poseAt(path, path.length, 0);
near(p.h, Math.PI / 2, 1e-9, 'final heading east (right, left, right)');
// lane offsets
near(HC.laneOffsetAt(500), 0, 1e-9, 'left lane before lane change');
near(HC.laneOffsetAt(590), 3.5, 1e-9, 'right lane while passing truck');
near(HC.laneOffsetAt(700), 0, 1e-9, 'back in left lane before junction 3');
// events do not overlap and stay on straights
const ev = HC.scheduledEvents(7);
for (let i = 1; i < ev.manoeuvres.length; i++)
  ok(ev.manoeuvres[i].promptT > ev.manoeuvres[i - 1].actionT, 'manoeuvre windows do not overlap ' + i);
HC.SCENARIO.hazards.forEach(h => {
  const g = HC.segmentAt(path, h.atS || h.onsetS);
  ok(g.type === 'straight', h.id + ' happens on a straight');
});

// --- Angles -----------------------------------------------------------------
near(HC.wrapDeg(190), -170, 1e-9, 'wrap 190'); near(HC.wrapDeg(-190), 170, 1e-9, 'wrap -190');
near(HC.wrapDeg(45), 45, 1e-9, 'wrap 45');

// --- Scoring on hand-made data ----------------------------------------------
function series(fn, dur) { const t = [], yaw = []; for (let i = 0; i < dur * 20; i++) { t.push(i / 20); yaw.push(fn(i / 20)); } return { t, yaw }; }
const m = { id: 'X', label: 'test', side: 'right', promptT: 10, actionT: 16, blindSpot: { kind: 'bike' } };
const opts = Object.assign({}, HC.DEFAULTS);
let r = HC.evaluateManoeuvre(series(t => (t > 13 && t < 14 ? 75 : 0), 30), m, opts);
ok(r.checked && Math.abs(r.leadTimeS - 2.95) < 0.06, 'right check 3 s before turn is detected (lead ' + r.leadTimeS + ')');
r = HC.evaluateManoeuvre(series(t => (t > 13 && t < 14 ? -75 : 0), 30), m, opts);
ok(!r.checked && r.checkedWrongSideOnly && r.nearMiss, 'looking LEFT before a RIGHT turn is not a check');
r = HC.evaluateManoeuvre(series(t => (t > 17 && t < 18 ? 75 : 0), 30), m, opts);
ok(!r.checked, 'a check AFTER the turn does not count');
r = HC.evaluateManoeuvre(series(t => (Math.abs(t - 13) < 0.02 ? 80 : 0), 30), m, opts);
ok(!r.checked, 'a single-sample spike (noise) is not a check');
r = HC.evaluateManoeuvre(series(t => (t > 13 && t < 14 ? 50 : 0), 30), m, opts);
ok(!r.checked, '50 deg glance is below the 60 deg threshold');
r = HC.evaluateManoeuvre(series(t => (t > 13 && t < 14 ? 50 : 0), 30), m, Object.assign({}, opts, { checkDeg: 45 }));
ok(r.checked, 'same glance counts if threshold lowered to 45');

const hz = [{ id: 'A', kind: 'tap', onsetT: 20, windowS: 4 }, { id: 'B', kind: 'tap', onsetT: 40, windowS: 4 },
  { id: 'C', kind: 'look', side: 'left', onsetT: 60, windowS: 3 }];
const h = HC.evaluateHazards(series(t => (t > 61 && t < 62 ? -60 : 0), 80), [5, 21.2, 21.5, 46], hz, opts);
ok(h.hazards[0].responded && h.hazards[0].rtMs === 1200, 'tap RT 1200 ms');
ok(!h.hazards[1].responded, 'tap after window misses hazard B');
ok(h.hazards[2].responded && Math.abs(h.hazards[2].rtMs - 1050) <= 50, 'look-left response to horn (' + h.hazards[2].rtMs + ' ms)');
ok(h.falseTaps === 2, 'false taps = 2 (t=5 and t=46); double tap inside a window is not false (got ' + h.falseTaps + ')');

// --- Statistics -------------------------------------------------------------
near(HC.tTwoTailedP(2.0, 10), 0.07339, 0.0005, 'p(t=2, df=10)');
near(HC.tTwoTailedP(2.228, 10), 0.05, 0.0005, 'p(t=2.228, df=10)');
near(HC.tTwoTailedP(1.96, 1e6), 0.05, 0.0005, 'p(t=1.96, df=inf)');
const w = HC.welchTest([1, 2, 3, 4, 5], [3, 4, 5, 6, 7]);
near(w.t, -2, 1e-9, 'welch t'); near(w.df, 8, 1e-9, 'welch df'); near(w.p, 0.0805, 0.0005, 'welch p');
near(w.cohenD, -1.2649, 0.0005, 'cohen d');

// --- End-to-end on simulated riders -----------------------------------------
const exp = [], nov = [];
for (let i = 1; i <= 10; i++) {
  exp.push(HC.evaluateRide(HC.simulateRider('experienced', i)).scores.overall);
  nov.push(HC.evaluateRide(HC.simulateRider('new', 100 + i)).scores.overall);
}
const wt = HC.welchTest(exp, nov);
console.log('simulated experienced:', exp.join(' '), '\nsimulated new:        ', nov.join(' '));
console.log('welch: t=%s df=%s p=%s d=%s', wt.t.toFixed(2), wt.df.toFixed(1), wt.p.toExponential(2), wt.cohenD.toFixed(2));
ok(HC.mean(exp) > HC.mean(nov), 'experienced riders score higher in simulation');
const rec = HC.evaluateRide(HC.simulateRider('experienced', 3));
ok(JSON.parse(JSON.stringify(rec)).scores.overall === rec.scores.overall, 'result survives JSON round trip');
ok(rec.manoeuvres.length === 5 && rec.hazards.length === 5, '5 manoeuvres + 5 hazards scored');

console.log(`\n${pass} passed, ${fail} failed`);
process.exit(fail ? 1 : 0);
