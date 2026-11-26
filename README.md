# HeadCheck VR – Rider safety test for delivery fleets

A 2½-minute virtual delivery ride for a **Procus (Cardboard-type) headset + Android phone**.
It measures two things:

1. **Shoulder checks**: does the rider turn their head toward the side *before* turning or changing lane?
2. **Hazard perception**: how fast do they spot a pedestrian, an opening car door, a U-turning car or a dog?

Results go to a **PC dashboard**. It compares experienced riders with new riders and runs the statistics.

---

## 1. What's in the folder

| File | What it is |
|---|---|
| `index.html` | The VR ride. Open it on the phone. |
| `dashboard.html` | The PC dashboard. Open it on the laptop. |
| `headcheck-core.js` | Shared logic: route, scoring, statistics. Both pages need it **in the same folder**. |
| `test-core.js` | Unit tests for the logic: `node test-core.js` (45 tests). |

You need internet on the phone, because the 3D library (A-Frame 1.6) loads from aframe.io.

---

## 2. Try it on your laptop first (5 minutes)

```bash
cd headcheck-vr
python -m http.server 8000
```
Open **http://localhost:8000**, enter a rider ID, tick consent, then click **Try without headset**.
- Drag the mouse to look around.
- **Space** = tap (spot a hazard). Esc = stop.

Open **http://localhost:8000/dashboard.html** and click **Load demo data** to see the dashboard.

---

## 3. Put it online so the phone can open it (VR mode needs HTTPS)

**GitHub Pages (free, about 5 minutes):**
1. Create a new public repository on github.com, e.g. `headcheck-vr`.
2. Click **Add file → Upload files**, drag in `index.html`, `dashboard.html` and `headcheck-core.js`, then **Commit**.
3. Go to **Settings → Pages → Source: Deploy from a branch → main / (root) → Save**.
4. After about 1 minute your link is `https://<your-username>.github.io/headcheck-vr/`.

(Alternative: Netlify Drop, where you drag the folder onto app.netlify.com/drop.)

---

## 4. Running it on the phone + Procus

1. Use **Chrome on Android**. iPhones don't support browser VR. Turn auto-rotate on.
2. Open your GitHub Pages link. The status box should say **"VR mode is supported on this device ✓"**.
3. Enter Rider ID, group and years riding. Choose **Test** mode for the study or **Training** mode for demos. Tick consent.
4. Tap **Start in VR headset**. The first time, Chrome may ask you to scan the headset's QR code; if there isn't one, skip it.
5. Put the phone in the Procus and plug in earphones (or use the Procus Pro's built-in headphones). The horn sound comes from a specific direction, so **audio matters**.
6. The rider sits on a **swivel chair**, looks straight ahead and presses the **Procus button** to start (or waits 8 s).
7. After the ride, remove the phone. On the results screen, tap **Share result file** and send it to yourself on WhatsApp, Drive or email, or tap **Download**.
8. On the PC, open `dashboard.html` and click **Load result files**. You can select or drag many files at once.

**Optional settings in the link:** `index.html?speed=6&check=60&voice=0`
- `speed`: ride speed in m/s (default 7 ≈ 25 km/h). Lower it if riders feel sick.
- `check`: degrees of head turn that count as a shoulder check (default 60). The dashboard slider can re-score later anyway.
- `voice=0`: turns off the spoken prompts (a beep and text stay).

---

## 5. What happens in the ride (same for every rider, so scores are comparable)

| Time | Event | What is measured |
|---|---|---|
| ~12 s | Bike horn from behind-right, then it overtakes | Did they look right within 3 s? |
| ~26–32 s | "Turn right ahead" (a bike sits in the blind spot) | Shoulder check right before the turn |
| ~46 s | Pedestrian steps out between parked cars | Tap reaction time |
| ~59–65 s | "Turn left ahead" (a cyclist sits in the blind spot) | Shoulder check left |
| ~70 s | Parked car door opens into the lane | Tap reaction time |
| ~74–80 s | "Change lane to the right" (parked truck ahead; a car sits in the blind spot) | Shoulder check right |
| ~86–92 s | "Merge back to the left lane" | Shoulder check left |
| ~98–105 s | "Turn right ahead" | Shoulder check right |
| ~112 s | Car U-turns through the median gap | Tap reaction time |
| ~123 s | Dog runs across the road | Tap reaction time |

**Definitions used in scoring**
- **Shoulder check:** head turned ≥ 60° toward the manoeuvre side, held ≥ 0.1 s, between the voice prompt and the manoeuvre. Looking the wrong way doesn't count.
- **Near miss:** no check before a manoeuvre that had a vehicle in the blind spot.
- **Hazard spotted:** a tap within 4 s of the hazard appearing (for the horn: looking ≥ 45° toward it within 3 s).
- **False tap:** a tap when no hazard window is open.
- **Score** = 50% × checks done + 50% × hazards spotted − 5 per false tap (0–100).
  - **≥ 80 Safe**, **50–79 Needs coaching**, **< 50 High risk**.

Head direction is sampled 20 times a second and stored in the result file, so you can re-analyse it later.

---

## 6. Study protocol for the professor (pilot validation)

**Research question:** does the HeadCheck score separate experienced riders from new or non-riders? Separating these "known groups" is evidence that the test measures real riding skill.

- **Participants:** at least 5 experienced riders (2+ years riding daily, e.g. delivery riders) and 5 new or non-riders. More is better: 10 + 10 gives a far more reliable result.
- **Before:** explain the study and get verbal or written consent. Exclude anyone prone to motion sickness or epilepsy. Say only: *"Ride as you normally would. Tap the button as soon as you spot a hazard."* **Don't mention shoulder checks.**
- **During:** use **Test mode**, one ride per person (a second ride is contaminated by learning). Stop immediately if anyone feels unwell.
- **After:** ask "Did you feel any dizziness or nausea? (0–10)" and note it.
- **Analysis (all in the dashboard):**
  - Group means with SD, Welch's t-test, Cohen's d.
  - The dot plot for each metric (score, checks, hazards, reaction time).
  - Move the **threshold slider** (45° → 75°) and check whether the group difference holds. That's a robustness check.
  - **Export CSV** for tables in your report.

**Limitations to state honestly:**
- Head direction is not the same as eye direction.
- The rider is seated, not balancing a real scooter.
- Phone gyroscopes drift slightly over minutes; the ride is 2½ minutes to keep this small.
- Small samples give a pilot result only.
- Cardboard VR can cause motion sickness at higher speeds.

---

## 7. Business angle (one paragraph for the report)

Food-delivery, quick-commerce and courier fleets onboard thousands of two-wheeler riders. HeadCheck VR runs on a ₹2,000 headset and the rider's own Android phone. In 3 minutes at the hub it gives a per-rider safety score and lists exactly which habit is missing (e.g. "never checks left"), so retraining can target it. Possible revenue models:
- per-rider assessment fees paid by fleets,
- per-hub subscriptions,
- later, safety-score data for two-wheeler insurers.

---

## 8. Troubleshooting

| Problem | Fix |
|---|---|
| "Not on HTTPS, VR blocked" | Open the GitHub Pages link, not a file or http:// link. |
| "VR mode not available" | Use Chrome on Android and update Chrome. You can still use "Try without headset" (phone held up, no stereo). |
| Picture is double or blurry in the Procus | Adjust the Procus lens distance (IPD) wheel; centre the phone in the slot. |
| Taps don't register | Press the Procus button firmly (it touches the screen). Check that the phone screen isn't cracked at the tap spot. |
| No horn sound | Turn the phone volume up; tap the screen once before entering VR. |
| The view is turned sideways at the start | The rider must look straight ahead when the ride starts; that direction becomes "forward". |
| Rider feels sick | Stop. Use `?speed=5` for the next rider. |

---

## 9. How the code works (for your report)

- **`headcheck-core.js`:**
  - the route as straight lines and 10 m-radius arcs (`buildPath`, `poseAt`),
  - the scenario timing,
  - shoulder-check detection (runs above a threshold inside a time window: `findRuns`, `evaluateManoeuvre`),
  - hazard scoring,
  - statistics (Welch's t-test via the regularised incomplete beta function, Cohen's d),
  - a simulated-rider generator for the demo.
- **`index.html`:**
  - an A-Frame/three.js scene; instanced buildings and parked cars keep it fast on phones,
  - the scooter "rig" moves along the route,
  - head yaw is read relative to the scooter and recentred at the start,
  - spatial horn audio is generated with Web Audio (no sound files),
  - prompts use speech synthesis,
  - the tap is the WebXR `select` event (the Cardboard button).
- **`dashboard.html`:**
  - plain SVG charts with no libraries,
  - re-scores every rider when the threshold changes,
  - exports CSV.
