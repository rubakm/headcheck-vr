# HeadCheck VR 2 · Fit-to-Ride Simulator

A 2½-minute VR ride that a delivery or bike-taxi rider takes **before picking up a passenger or an order**. Like a pilot's simulator check, it measures how the rider handles a busy Indian street right now and gives one of four verdicts: **Fit to ride**, **Take a break**, **Not fit to ride now**, or **Check not valid**.

It runs in Chrome on an Android phone inside a Cardboard-type headset (tested design target: Procus with one button). No app install, no server, no downloads besides A-Frame.

- **Simulator:** https://rubakm.github.io/headcheck-vr/
- **Fleet dashboard:** https://rubakm.github.io/headcheck-vr/dashboard.html
- **Watch a demo ride without a headset:** https://rubakm.github.io/headcheck-vr/?autostart=1&bot=tired

## Why

Two-wheeler riders are the largest group of road deaths in India: 74,897 of 1,68,491 deaths in 2022 (44.5%), with pedestrians next at 19.5% (MoRTH, *Road Accidents in India 2022*). Platform riders work long shifts, often at night, and nobody checks whether they are alert before a ride. A fatigued or impaired rider is a risk to the passenger, to pedestrians and to themselves.

## How a check works

| Step | Time | What happens |
|---|---|---|
| Ready | – | Rider puts on the headset, looks ahead and presses the button. |
| Calibrate | ~3 s | Rider holds still; the resting head tilt and pitch are recorded and removed, and "straight ahead" is reset. |
| Order | 5 s | A delivery order is announced: "house number 47". The rider must remember it. |
| Ride | ~130 s | A randomised street with 11 Indian traffic situations, a red-light vigilance task, wind gusts, oncoming traffic and a ringing phone. |
| Questions | ~10 s | Memory, situation awareness and a judgement question in the rider's language, answered by looking at an option. |
| Verdict | 8 s | Verdict, score and main reason, shown and spoken in the rider's language. |

### Controls

| | VR headset | Without headset |
|---|---|---|
| Steer (lean) | Tilt the head left/right | `A`/`D` or `←`/`→`; on a phone, tilt the phone |
| Brake | Hold the headset button | Hold `Space` / touch the screen |
| Red light | Tap the button | Tap `Space` |
| Look around / shoulder check | Turn the head | Drag the mouse |
| Answer | Look at an option for 1.2 s, or look and press | Same, or keys `1` `2` `3` |

Head roll goes through a dead zone (1.2°), an expo curve and a One-Euro filter (smooths tremor without adding lag on fast moves), then into a bike lean model stepped at 120 Hz. Button presses are time-stamped between frames. An end-to-end test confirms the head-pose path reproduces the applied pose to within 0.05° across 2,701 samples, even when the rider starts 25° off-centre and tilted 3°.

## The 11 situations (order and road layout change every session)

Pedestrian stepping out from behind a parked bus · auto-rickshaw that pulls in and brakes hard · pothole · pothole hidden in a puddle · unmarked speed breaker · bike coming on the wrong side · sharp turn · ambulance siren from behind · school zone (25 km/h) with children and a guard · parked truck with a bike overtaking (shoulder check) · cow or dog walking into the road.

Every session builds a new street from a random seed (parallel test forms), so riders cannot memorise it. Signs show English and the rider's language, shop boards mix Tamil, Hindi and English, and trucks carry "HORN OK PLEASE".

## What is measured

| Measure | How | Why it matters |
|---|---|---|
| Lane weaving (SDLP) | Standard deviation of lateral position on clean stretches | The reference measure for impaired driving; +2.4 cm is the accepted "clinically relevant" change, matched to a blood alcohol level of 0.5 g/L |
| Red-light reaction (DRT) | Small red light every 3–5 s, on for 1 s; valid responses 100–2500 ms; earlier presses count as anticipation | Detection-response task timings follow ISO 17488; used here as a vigilance probe during riding |
| Brake reaction | Time from hazard onset to brake | Sudden-hazard response |
| Steering control lag | Cross-correlation between the rider's aim-point error and their steering | Slower, sluggish control when tired |
| Hazards and collisions | Collisions, near misses, potholes hit, running wide in the turn | Direct safety outcome |
| Rules and context | School-zone speed, giving way to the ambulance, shoulder check before moving out | Judgement under pressure |
| Eyes off the road | Head pitched down at the phone, longest glance | Distraction during a call |
| Memory and judgement | House number, which animal crossed, what to do when the phone rings | Working memory and situational awareness |

## Verdict

1. Six sub-scores (lane control, reactions, attention, hazards, rules, memory) combine into a **fitness index** out of 100.
2. **Gates**: 2+ collisions or 40%+ missed lights → not fit; 1 collision or 22%+ missed → take a break; index below 50 → not fit; below 68 → take a break.
3. **Personal baseline**: after 3 good rides, each check is also compared with the rider's own normal (median and spread of the last 10 good rides). A change score of 2.0 or more → take a break; 3.2 or more → not fit. This catches a rider who is still "average" but far below their own usual level.
4. **Not valid** if the head barely moved (headset not worn), the button was mashed, or the ride was not finished.

All thresholds are **provisional** until set from pilot data. The dashboard includes a what-if tool that shows the trade-off when they move.

## Validation so far

- **Engine:** 565 automated tests (`node test-core.js`): road geometry, physics, filter latency, DRT timing, SDLP sensitivity, control-lag recovery, scoring, determinism.
- **Synthetic riders:** closed-loop simulated riders (rested, drowsy, tired, impaired) that steer, brake and react with different delays and attention lapses. With personal baselines, the engine flags them as designed: in the dashboard run of 12 riders, rested 12/12 fit, tired 12/12 flagged (9 not fit), impaired 12/12 not fit, drowsy 4/12 flagged.
- **End to end:** headless browser tests run complete checks through the real page (calibration, recentering, key events, gaze answers, result files, history, practice mode, abort).

Synthetic results show the **pipeline works, not that it predicts real crashes.** That needs real riders.

## Suggested pilot study

1. Ethics approval and informed consent. No alcohol or drug dosing; use naturally occurring fatigue.
2. About 40 riders. Two practice rides first, then 3 rested checks on different days at shift start (baseline and test–retest reliability).
3. Checks at shift start and shift end, and after night shifts, alongside a 3-minute PVT-B (lapses ≥ 355 ms), Karolinska Sleepiness Scale, hours slept and time awake.
4. Analyse reliability (ICC on rested days), sensitivity to fatigue (start vs end of shift), and ROC against the PVT-B reference; set cut-offs for a target false-alarm rate (for example, at most 5% of rested checks flagged).
5. Record simulator sickness after each ride.
6. Later: compare verdicts with incident and harsh-braking data, with the operator's consent.

## Fleet dashboard

`dashboard.html` imports result files from the simulator (the `.json` the operator downloads or shares at the end of a check). It shows checks today, riders needing attention first, each rider's index history against thresholds, the selected ride's lane-position, speed-and-braking and red-light traces, the rider's normal for each measure, and CSV export. "Load demo fleet" fills it with clearly labelled simulated riders. Data stays in the browser.

## Files

| File | What it is |
|---|---|
| `index.html` | VR simulator: procedural Indian street (textures drawn in the browser, geometry merged per 50 m chunk), cockpit, traffic, audio, input, session flow, results |
| `headcheck-core.js` | Engine: route generator, 120 Hz physics, hazards, DRT, logging, metrics, scoring, simulated riders. Runs in the browser and in Node |
| `i18n.js` | Everything the rider sees or hears in English, Hindi and Tamil |
| `dashboard.html` | Fleet dashboard and synthetic validation |
| `test-core.js` | Engine test suite |

URL options for demos and testing: `bot=rested|drowsy|tired|impaired`, `autostart=1`, `lang=en|hi|ta`, `vehicle=bike|auto`, `seed=<number>`, `timescale=<0.25–8>`, `debug=1` (frame rate and draw calls).

## Limitations

- Cardboard headsets track head rotation only. Head tilt stands in for leaning the body; mirrors are not rendered (shoulder checks are measured from head turns).
- VR can cause motion sickness. A comfort vignette narrows the view in fast turns; the rider can stop at any time and the check is then not scored.
- This is a screening aid for fitness to ride. It is not a medical, alcohol or drug test and must not replace legal checks.
- Hindi and Tamil text was written for this project and needs review by native speakers.
- The auto-rickshaw is a beta. Other vehicles (cars, buses, trucks) fit the same engine through a vehicle profile.

## Sources

- MoRTH, *Road Accidents in India 2022*, summarised in [SaveLIFE Foundation: India's Road Crash Statistics 2022](https://savelifefoundation.org/wp-content/uploads/2024/07/Indias-Road-Crash-Statistics_-2022-Overview.pdf) and [Business Standard](https://bsppd.business-standard.com/amp/india-news/road-accidents-in-india-at-decade-low-but-number-of-fatalities-up-123112200002_1.html)
- [A pooled analysis of on-the-road highway driving studies measuring SDLP at a blood alcohol concentration of 0.5 g/L](https://pmc.ncbi.nlm.nih.gov/articles/PMC5306436/) (Psychopharmacology, 2017)
- Verster & Roth (2011), [Standard operation procedures for the on-the-road driving test and SDLP](https://pmc.ncbi.nlm.nih.gov/articles/PMC3100218/)
- [ISO 17488:2016, Detection-response task (DRT) for assessing attentional effects of cognitive load in driving](https://www.iso.org/standard/59887.html)
- Basner, Mollicone & Dinges (2011), [Validity and sensitivity of a brief psychomotor vigilance test (PVT-B)](https://pmc.ncbi.nlm.nih.gov/articles/PMC3197786/)
- Basner & Rubinstein (2011), [Fitness for duty: a 3-minute PVT predicts fatigue-related performance declines](https://pmc.ncbi.nlm.nih.gov/articles/PMC3190077/)
- Antler et al. (2022), [The 3-minute PVT shows inadequate convergent validity relative to the 10-minute PVT](https://pmc.ncbi.nlm.nih.gov/articles/PMC8885985/) (why short tests need their own validation)

Version 1 (head-check training ride) is in this repository's history.
