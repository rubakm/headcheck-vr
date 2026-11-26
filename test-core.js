// Tests for headcheck-core.js (HeadCheck VR 2)  ->  node test-core.js
// Covers geometry, controls, physics determinism, ISO 17488 DRT timing, metrics,
// operator-lag identification, and a synthetic fleet with personal baselines.
const HC = require('./headcheck-core.js');
let pass = 0, fail = 0;
function ok(c, msg) { if (c) pass++; else { fail++; console.log('FAIL:', msg); } }
function near(a, b, tol, msg) { ok(Math.abs(a - b) <= tol, `${msg} (got ${a}, want ${b} ± ${tol})`); }
const t0 = Date.now();

// ---- geometry -------------------------------------------------------------
const P = HC.buildPath([{ type: 'straight', len: 100 }, { type: 'arc', r: 20, dir: 1, angleDeg: 90 }, { type: 'straight', len: 50 }]);
near(P.length, 150 + 10 * Math.PI, 1e-9, 'path length');
let p = HC.poseAt(P, 100 + 10 * Math.PI, 0);
near(p.x, 20, 1e-9, 'right turn ends 20 m east'); near(p.z, -120, 1e-9, 'right turn ends 120 m north'); near(p.h, Math.PI / 2, 1e-9, 'heading east');
near(HC.curvatureAt(P, 110), 1 / 20, 1e-12, 'curvature + for right turn');
near(HC.poseAt(P, 50, 1.9).x, 1.9, 1e-9, 'lateral offset to the right');
let maxStep = 0, prev = HC.poseAt(P, 0, 0);
for (let s = 0.5; s <= P.length; s += 0.5) { const q = HC.poseAt(P, s, 0); maxStep = Math.max(maxStep, Math.hypot(q.x - prev.x, q.z - prev.z)); prev = q; }
ok(maxStep <= 0.5001, 'path is continuous');

// ---- route generator: parallel forms ---------------------------------------
const lens = [];
for (let seed = 1; seed <= 40; seed++) {
  const R = HC.buildRoute(seed);
  lens.push(R.length);
  ok(R.order.length === 11 && new Set(R.order).size === 11, `seed ${seed}: all 11 events present once`);
  ok(!['sharp_turn', 'jaywalker', 'lead_stop', 'animal', 'wrong_side'].includes(R.order[0]), `seed ${seed}: gentle first event`);
  for (let i = 1; i < R.events.length; i++) ok(R.events[i].s0 >= R.events[i - 1].s1, `seed ${seed}: events do not overlap`);
  // no self-intersection: points far apart along the path stay far apart in space
  const pts = []; for (let s = 0; s <= R.length; s += 5) pts.push(HC.poseAt(R.path, s, 0));
  let minD = 1e9; for (let i = 0; i < pts.length; i++) for (let j = i + 24; j < pts.length; j++) minD = Math.min(minD, Math.hypot(pts[i].x - pts[j].x, pts[i].z - pts[j].z));
  ok(minD > 30, `seed ${seed}: route never comes back on itself (min ${minD.toFixed(0)} m)`);
}
ok(Math.min(...lens) > 780 && Math.max(...lens) < 1000, `route length 780-1000 m (${Math.min(...lens).toFixed(0)}-${Math.max(...lens).toFixed(0)})`);
const A = HC.buildRoute(5), B = HC.buildRoute(5), C = HC.buildRoute(6);
ok(JSON.stringify(A.order) === JSON.stringify(B.order) && A.length === B.length, 'same seed = same route');
ok(JSON.stringify(A.order) !== JSON.stringify(C.order) || A.length !== C.length, 'different seed = different route');

// ---- head-tilt steering ---------------------------------------------------
ok(HC.steerNorm(1.0) === 0 && HC.steerNorm(-1.1) === 0, 'dead zone ±1.2°');
ok(HC.steerNorm(10) > 0 && HC.steerNorm(-10) < 0, 'tilt right = lean right');
near(HC.steerNorm(26), 1, 1e-9, 'full tilt = full lean'); near(HC.steerNorm(40), 1, 1e-9, 'saturates');
ok(HC.steerNorm(5) < 5 / 26, 'expo curve gives fine control near centre');
for (const u of [-0.9, -0.3, 0.05, 0.5, 1]) near(HC.steerNorm(HC.steerInverse(u)), u, 1e-9, `inverse map u=${u}`);
// 1-euro filter: smooths jitter but follows a fast tilt quickly
const f = new HC.OneEuro(3.0, 0.06, 1.0), rng = HC.mulberry32(3);
let out = [], tt = 0;
for (let i = 0; i < 240; i++) { tt = i / 120; out.push(f.filter((rng() - 0.5) * 1.0, tt)); }
ok(HC.sd(out.slice(60)) < 0.18, `jitter ±0.5° reduced (sd ${HC.sd(out.slice(60)).toFixed(3)})`);
let reached = null;
for (let i = 240; i < 480; i++) { tt = i / 120; const y = f.filter(15, tt); if (reached === null && y > 13.5) reached = tt - 2; }
ok(reached !== null && reached < 0.12, `a 15° tilt is followed within 120 ms (${Math.round(reached * 1000)} ms)`);

// ---- physics: steady turn matches lean geometry ----------------------------
{
  const R = { seed: 9, path: HC.buildPath([{ type: 'arc', r: 40, dir: 1, angleDeg: 300 }]), events: [], wind: [], phoneS: 1e9, order: [], length: 200 };
  const sim = new HC.Sim(R, {});
  sim.st.v = 8; sim.nextAmbient = 1e9; sim.drt.next = 1e9;
  const roll = HC.steerInverse(Math.atan(8 * 8 / (9.81 * 40)) * 180 / Math.PI / 35);   // lean for a 40 m radius at 8 m/s
  sim.st.holdUntil = 1e9;
  for (let i = 0; i < 360; i++) sim.step(1 / 120, { roll: roll });
  near(sim.st.yawRate, 8 / 40, 0.004, 'steady lean gives yaw rate v/R');
}

// ---- determinism & sim correctness ------------------------------------------
const s1 = HC.runSession({ seed: 77, bot: 'rested' }), s2 = HC.runSession({ seed: 77, bot: 'rested' });
ok(JSON.stringify(s1.log.y) === JSON.stringify(s2.log.y) && JSON.stringify(s1.drt) === JSON.stringify(s2.drt), 'identical replay for the same seed');
ok(s1.completed && s1.durationS > 110 && s1.durationS < 150, `ride takes 110-150 s (${s1.durationS})`);

// ---- DRT follows ISO 17488 timing ------------------------------------------
{
  const on = s1.drt.map(d => d.onset);
  const isi = on.slice(1).map((x, i) => x - on[i]);
  ok(isi.every(x => x >= 2.999), 'stimuli at least 3 s apart');
  ok(isi.filter(x => x <= 5.001).length / isi.length > 0.75, 'most stimuli 3-5 s apart (rest delayed around hazards)');
  ok(s1.drt.length >= 18, `enough stimuli for a stable estimate (${s1.drt.length})`);
  ok(s1.drt.filter(d => d.outcome === 'hit').every(d => d.rt >= 0.1 && d.rt <= 2.5), 'hits fall in the 100-2500 ms window');
}
{ // anticipation and late presses are not counted as hits
  const R = HC.buildRoute(3), sim = new HC.Sim(R, {});
  sim.drt.next = 1; sim.nextAmbient = 1e9;
  while (!sim.drt.cur) sim.step(1 / 120, {});
  const on = sim.drt.cur.onset;
  sim.press(on + 0.05); sim.release(on + 0.1);
  ok(sim.drt.cur.outcome === 'anticipation', '< 100 ms press = anticipation, not a hit');
  while (sim.drt.cur && sim.t < on + 3) sim.step(1 / 120, {});
  ok(sim.drt.list[sim.drt.list.length - 1].outcome === 'anticipation', 'stimulus stays an anticipation');
}

// ---- metrics on a hand-made log ---------------------------------------------
{
  const base = HC.runSession({ seed: 21, bot: 'rested' });
  const fake = JSON.parse(JSON.stringify(base));
  const m0 = HC.analyze(fake);
  fake.log.y = fake.log.y.map((y, i) => y + 0.3 * Math.sin(i / 7));       // add weaving
  const m1 = HC.analyze(fake);
  ok(m1.sdlp > m0.sdlp + 0.06, `added weaving raises SDLP (${m0.sdlp} -> ${m1.sdlp})`);
  ok(m0.sdlpSeconds > 25, `enough clean lane-keeping time for SDLP (${m0.sdlpSeconds} s)`);
}

// ---- operator-lag identification recovers slower riders --------------------
{
  const lag = tau => HC.median([31, 32, 33, 34, 35, 36].map(seed => {
    const S = HC.runSession({ seed, bot: Object.assign({}, HC.BOT_PROFILES.rested, { tau: tau, lapsePerMin: 0 }) });
    return HC.analyze(S).controlLagMs;
  }));
  const l20 = lag(0.20), l35 = lag(0.35), l50 = lag(0.50);
  console.log(`  control lag estimate: tau 200 -> ${l20} ms, 350 -> ${l35} ms, 500 -> ${l50} ms`);
  ok(l20 < l35 && l35 < l50, 'estimated control lag increases with true reaction delay');
  ok(l50 - l20 > 120, 'lag estimate tracks a 300 ms difference');
}

// ---- verdicts: a synthetic fleet with personal baselines --------------------
{
  const riders = 12, conds = ['rested', 'rested', 'rested', 'tired', 'tired', 'impaired'];
  const tally = { rested: { FIT: 0, CAUTION: 0, NOT_FIT: 0, INVALID: 0 }, tired: { FIT: 0, CAUTION: 0, NOT_FIT: 0, INVALID: 0 }, impaired: { FIT: 0, CAUTION: 0, NOT_FIT: 0, INVALID: 0 } };
  for (let r = 0; r < riders; r++) {
    const rr = HC.mulberry32(1000 + r);
    const traits = { rt: 0.85 + 0.35 * rr(), noise: 0.8 + 0.45 * rr() };
    const history = [];
    for (let d = 0; d < 4; d++) {        // enrolment: 4 rested sessions build the baseline
      const S = HC.runSession({ seed: 5000 + r * 50 + d, bot: 'rested', traits });
      history.push(HC.assess(S, history).summary);
      history[history.length - 1].verdict = 'FIT';   // supervisor-confirmed enrolment rides
    }
    conds.forEach((c, d) => {
      const S = HC.runSession({ seed: 9000 + r * 50 + d, bot: c, traits });
      const A = HC.assess(S, history);
      tally[c][A.verdict]++;
      if (c === 'rested') history.push(A.summary);
    });
  }
  const pct = (c, keys) => Math.round(100 * keys.reduce((a, k) => a + tally[c][k], 0) / Object.values(tally[c]).reduce((a, b) => a + b, 0));
  console.log('  fleet tally', JSON.stringify(tally));
  console.log(`  rested FIT ${pct('rested', ['FIT'])}% · tired flagged ${pct('tired', ['CAUTION', 'NOT_FIT'])}% · impaired NOT FIT ${pct('impaired', ['NOT_FIT'])}%`);
  ok(pct('rested', ['FIT']) >= 75, 'rested riders are mostly passed (specificity)');
  ok(pct('tired', ['CAUTION', 'NOT_FIT']) >= 70, 'tired riders are mostly flagged');
  ok(pct('impaired', ['NOT_FIT']) >= 80, 'impaired riders are stopped');
  ok(tally.rested.INVALID + tally.tired.INVALID + tally.impaired.INVALID === 0, 'no false INVALID sessions');
}

// ---- validity checks ----------------------------------------------------------
{
  const S = HC.runSession({ seed: 8, bot: 'rested' });
  const still = JSON.parse(JSON.stringify(S));
  still.log.yaw = still.log.yaw.map(() => 0); still.log.roll = still.log.roll.map(() => 0);
  ok(HC.assess(still, []).verdict === 'INVALID', 'headset lying still on a table = INVALID');
  ok(HC.assess(S, []).verdict !== 'INVALID', 'a normal ride is valid');
  const qs = HC.makeQuestions(HC.buildRoute(8));
  ok(qs.length === 3 && qs.every(q => q.options.length === 3 && q.correct >= 0 && q.correct < 3), '3 questions, 3 options each');
  ok(qs[0].options[qs[0].correct].label === String(HC.buildRoute(8).houseNo), 'memory question has the right house number');
  ok(JSON.parse(JSON.stringify(S)).log.t.length > 2000, 'session survives JSON round trip with full log');
}

// ---- speed ------------------------------------------------------------------
{
  const t1 = Date.now(); HC.runSession({ seed: 99, bot: 'tired' });
  ok(Date.now() - t1 < 600, `one full simulated session runs in < 600 ms (${Date.now() - t1} ms)`);
}
console.log(`\n${pass} passed, ${fail} failed  (${((Date.now() - t0) / 1000).toFixed(1)} s)`);
process.exit(fail ? 1 : 0);
