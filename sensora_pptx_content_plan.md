# Sensora — Slide-by-Slide Content Plan for SENSORA_Template.pptx

This maps your actual project content onto the 17-slide template you uploaded. The template is a generic "SENSORA 2.0" pitch-deck skeleton with instructional placeholder text in every box — this plan replaces that placeholder text with your real content, slide by slide, in the same order and layout the template already uses.

Reference material pulled from the project: `architecture-plan.md`, `novelty-analysis.md`, `hardware-firmware-spec.md`, `feature-build-checklist.md`, `dashboard-features.md`, `wifi-json-protocol.md`.

---

## Slide 1 — Title / Cover

Template layout: big background gradient, deck title + subtitle.

- **Title:** Sensora — Intelligent Worker Safety & Resilient Emergency Response System
- **Subtitle:** Edge-computed risk detection and a self-healing ESP-NOW mesh that keeps a worker's safety state visible even when connectivity fails.
- Hackathon name / track, your team name, date — whatever the event's cover convention is.

Speaker note: this is the "elevator pitch in one breath" slide. Don't read it word for word — say it, then move.

---

## Slide 2 — Contents

Template layout: numbered list, 4 sections.

Keep the template's section structure as-is — it already matches a clean pitch arc:

1. Section 1 — Problem & Context
2. Section 2 — Solution & Technology
3. Section 3 — Implementation & Results
4. Section 4 — Impact, Scalability & Future

No content change needed here beyond the labels already in the template.

---

## Slide 3 — Section Divider "01"

- **Label:** Section 1 — Problem & Context

Just the divider; no body content.

---

## Slide 4 — Title & Overview (timeline layout: two milestones)

Template layout: two circular icons on a line, left = "2024", right = "2025" — repurpose these as **"The Problem" → "Our Answer"** rather than literal years, or just delete the year labels if the template lets you retype them as blank/step numbers.

**Left node — Problem & Team Snapshot:**
- One-line core challenge: *"Underground and confined industrial sites (mines, tunnels, plants) can't see a worker in trouble until it's too late — gas, heat, falls, and lost contact all happen out of sight, and standard WiFi/cellular don't reach down there."*
- Team name, track, and a one-line role breakdown (e.g., "4–6 person team — embedded/firmware, comms, backend, dashboard").
- Solution tagline aimed at the target user: *"Sensora gives every worker a local safety brain and a mesh that keeps talking even when a node dies."*

**Right node — Value Tagline & Visual Identity:**
- Short sub-tagline highlighting the key benefit: *"Real-time, works with zero connectivity, self-healing by design."*
- Sensora logo/wordmark as the cover visual if you have one; otherwise a simple icon (helmet, signal, or shield) in the brand color.

---

## Slide 5 — Problem & Real-World Context

Template layout: image left, two stacked text cards right.

**Who Is Affected & Why It Matters:**
- Underground miners, tunnel/confined-space workers, and plant floor workers in environments where WiFi/cellular don't reach and hazards (gas buildup, heat, falls, structural collapse) develop faster than a manual headcount can catch.
- Quantify scale if you have or can cite a number (mine workforce size, fatality/incident statistics for confined-space or underground work — cite a source if you use one; otherwise keep it qualitative and honest).
- Real-world consequence: a worker down or a gas spike goes unnoticed until the next scheduled check-in, not the moment it happens.

**Gaps in Existing Approaches & Context:**
- Existing smart-helmet and gas/fire-detection research (cited in your novelty analysis) each solve *one* signal type in isolation — motion **or** gas **or** comms — not a fused local risk state.
- Existing comms-resilience projects (e.g. solar LoRa mesh for disaster response) target long-range outdoor connectivity, not the short-range, node-dense, GPS-denied underground case.
- Most published designs assume the sensor readings are the safety layer; Sensora treats communication failure itself as a hazard, and the mesh's design reflects that.
- Transition line: *"So the requirement isn't a better sensor — it's a system that keeps functioning when any single part of it goes dark."*

---

## Slide 6 — Problem Drivers & Requirements (fishbone diagram)

Template layout: fishbone with Problem / Internal / External rows, and Context / Needs / Fit rows.

| Branch | What to put there |
|---|---|
| **Problem** (top-left) | Fragmented sensing (temp, gas, motion each siloed), no shared risk model across a fleet, and comms that fail exactly when they're needed most (emergency = when infrastructure is stressed). |
| **Internal** (top-mid) | Data/process factors: no on-device fusion of signals, fixed thresholds that don't adapt to a worker's own baseline, single-path routing that has no fallback. |
| **External** (top-right) | Underground RF attenuation, no cellular/WiFi at depth, regulatory expectation of continuous worker monitoring, cost pressure (can't mandate expensive industrial IoT per worker). |
| **Context** (bottom-left) | Regulations increasingly expect continuous, auditable worker-safety monitoring; environment is GPS-denied and RF-hostile. |
| **Needs** (bottom-mid) | Real-time local risk detection, low power (multi-shift battery life), robust to node loss, cheap enough per worker to actually deploy at scale. |
| **Fit** (bottom-right) | Combining edge sensor fusion (cheap MCU-level compute) with a priority-aware self-healing mesh meets all of the above better than either sensing-only or comms-only approaches. |
| **Core issue** callout | "Safety state must survive the same failure that caused the emergency." |

---

## Slide 7 — Section Divider "02"

- **Label:** Section 2 — Solution & Technology

---

## Slide 8 — Core Solution (WHAT / HOW / WHY nested circles)

**WHAT? → Phenomenon/Outcome:**
Sensora turns raw multi-sensor readings from a worker-worn device into an immediate, locally-computed safety state (OK / WARNING / CRITICAL / FALL_SUSPECTED / SOS), visible even with zero network, and relays it through a resilient mesh the instant connectivity allows.

**HOW? → Method/Action:**
- Worker ESP32 fuses MPU6050 motion, BME280 temp/humidity/pressure, and MQ-7 gas into one adaptive risk score using a weighted RMS-of-z-scores formula with a rolling per-worker baseline (Welford's online mean/variance) — not a fixed threshold.
- A fall/inactivity state machine (impact spike → stillness → tilt confirmation) avoids false positives a bare accelerometer threshold would trigger.
- Wall-mounted ESP-NOW anchor nodes form a gradient-routed backbone; workers associate with the strongest anchor via hysteresis-smoothed RSSI. Routine telemetry uses efficient hop-by-hop routing; DISTRESS/emergency traffic uses TTL-bounded flood-with-dedup for guaranteed delivery.
- One gateway anchor bridges to a dashboard over WiFi/JSON (schema-versioned, push-based).

**WHY? → Mission/Idea:**
Make advanced multi-sensor safety monitoring dependable exactly when infrastructure is least dependable — technology that quietly protects, and never claims more certainty (or connectivity) than it actually has.

---

## Slide 9 — System Architecture & Workflow

**01 — End-to-End Flow & Core Components:**
Sensing (MPU6050 + BME280 + MQ-7 on the worker unit) → edge fusion & state machine (on-device, C/C++, deterministic) → ESP-NOW packet to nearest wall anchor → mesh relay (gradient routing / flood-on-emergency) to the gateway anchor → WiFi/JSON push to the dashboard → dashboard render (worker roster, alert banner, event log).

**02 — AI/Logic Placement, Interactions & Scalability:**
- Edge: deterministic weighted risk-fusion + fall state machine on the worker MCU — must be 100% reliable, so no ML here by design.
- Backend/dashboard: open-string `riskState`/`hazardType` fields and a versioned JSON schema (`schemaVersion`) mean new hazard types and features (proximity mapping, in/out counting, battery) slot in without breaking existing dashboard code.
- Scalability path: `nodeId` is already in every message so a second wall anchor drops in with no protocol change; the mesh's hop-count/RSSI association logic is designed to generalize past a single-anchor topology.

---

## Slide 10 — Section Divider "03"

- **Label:** Section 3 — Implementation & Results

---

## Slide 11 — Prototype & Implementation Evidence

**Demo flow & implementation status** — be explicit about what's real vs. planned, the template literally asks for this:
- **Built and working (`LIVE`):** Feature 1 — fall/inactivity detection. Worker ESP32 → ESP-NOW → wall node → WiFi/JSON → **Sensora Watch** dashboard, running end-to-end on the defined protocol, with a live demo simulator (trigger fall / simulate movement / reset) built into the dashboard itself.
- **Scoped and specified, not yet built:** Feature 2 (proximity/"LiDAR-like" snapshot via HC-SR04 + gyro yaw) and Feature 3 (worker in/out counting — blocked on 2 additional beam sensors).
- Say this on stage rather than let a judge find it: name what's live, what's simulated, what's next.

**Visual proof of working system:**
- Screenshot(s) of the Sensora Watch dashboard: utility bar, stat tiles, worker roster with status pill/motion energy/RSSI/battery, alert banner, event log.
- A photo of the actual ESP32 hardware (worker unit + wall node) if available.
- Optionally, the JSON payload itself (a STATUS and a DISTRESS example) as a small code callout — it's concrete evidence the protocol is real, not just described.

---

## Slide 12 — Results & Metrics (4 cards)

Be honest about hackathon-scale results — the template's own copy talks about accuracy/latency/reliability/baseline, so fill each card with what you can actually measure or credibly estimate:

| Card | What to put |
|---|---|
| **accuracy / quality** | Fall-detection state machine parameters (impact ≥3.0g within 150ms, stillness ≥2000ms, tilt ≥45°) and why this rejects false positives a bare threshold wouldn't (e.g., "sitting down quickly" no longer triggers a false alarm). If you've run manual test trials, state the count and outcome plainly (e.g., "12/12 simulated falls detected in bench testing, 0 false triggers from normal movement"). |
| **latency / throughput** | End-to-end time from simulated fall to dashboard alert (measure it — likely well under 1s for a single hop). STATUS update cadence (~2s) vs. DISTRESS (sent immediately, retried up to 3x on failure). |
| **reliability / uptime** | Mesh design properties: 9-second stale-neighbor failure detection, TTL-bounded flood with dedup guarantees delivery via multiple paths for emergency traffic, dashboard's own 15-second staleness detection marks a worker OFFLINE rather than silently going stale. |
| **baseline / validation** | Compare against the "naive fixed-threshold" approach used in the two published designs you reviewed (MDPI smart-helmet paper, IJERT fire/gas paper): your adaptive per-worker baseline (Welford mean/variance + EWMA) reduces false alarms from workers with different resting motion/vitals, where a fixed threshold can't. |

If you don't have hard numbers yet, say so and give the honest estimate — judges respect a stated methodology over an invented precision.

---

## Slide 13 — Section Divider "04"

- **Label:** Section 4 — Impact, Scalability & Future

---

## Slide 14 — Innovation, Impact & Users

**Innovation & impact:**
- *Innovation engine:* the differentiator isn't any single sensor (temp/humidity/motion/gas are the same sensors published solutions use) — it's the underlying algorithms: adaptive per-worker baselining instead of fixed thresholds, a fall state machine instead of a bare accelerometer trip, and priority-aware routing (efficient path for routine data, guaranteed-delivery flood for emergencies) instead of one-size-fits-all mesh routing. Pull 4–6 of your strongest points from `novelty-analysis.md`'s 22-point list here rather than all 22 — pick the ones a judge from this specific hackathon's domain will find most concrete.
- *Impact rationale:* faster, quieter detection of a worker down (no wait for a scheduled check-in), safety state that survives the exact failure mode (lost comms) that makes an emergency worse, and a system cheap enough per worker to actually deploy at fleet scale (same commodity sensors as existing designs, no proprietary hardware).

**Users & narrative:**
- *Users & alignment:* underground/confined-space workers (wearing the device), site supervisors (watching the dashboard), and safety/compliance teams (auditable event log).
- *Before vs. after:* Before Sensora — a worker down is discovered at the next radio check-in or shift change, minutes to tens of minutes later, and a comms outage means total blackout on worker status. After Sensora — local detection is instant even with zero network, and the moment any path to the gateway exists, the alert reaches the dashboard through whichever route is still standing.

---

## Slide 15 — Future Scope & Conclusion

**Realistic Enhancements & Extensions:**
- Feature 2 (proximity/HC-SR04 "LiDAR-like" snapshot at moment of distress) and Feature 3 (worker in/out counting at the wall node, pending beam sensors) — both already schema-planned, not hypothetical.
- Battery-percentage reporting (the JSON schema already reserves the field as `null` for this).
- A second wall anchor to prove the multi-hop backbone, not just a single-hop link.
- Beyond mining: the same mesh/edge-fusion pattern extends to the broader disaster-resilient-comms and flash-flood scenarios in the original architecture plan (env/flood node, backend ML risk fusion) — worth one line if your hackathon track values breadth, but don't let it dilute the mine-safety story that's actually built.

**Value, Feasibility & Final Call:**
- Tackles a concrete, underserved problem (worker safety under active comms failure) with a working prototype, not just a concept — Feature 1 runs end-to-end today.
- Feasible to extend: the protocol and packet formats were deliberately designed so new features (schema version bump, new payload fields) don't require redesigning what's already built.
- Close with your team's invitation for questions and a live demo of Sensora Watch.

---

## Slide 16 — (Template duplicates Slide 15 — repurpose it)

The template has this slide as an exact duplicate of Slide 15. Rather than leaving two identical slides, repurpose it as a **backup/anticipated-questions slide** — genuinely useful material already exists for this in `architecture-plan.md` §8. Suggested content:

**"What We're Ready to Be Asked"**
- *"Your gas sensor isn't giving calibrated ppm readings"* → Correct, by design: MQ-7 needs a heater burn-in cycle; we detect deviation from a power-on baseline and disclose that on the dashboard, not a certified ppm value.
- *"No GPS?"* → Correct and intentional — GPS doesn't lock underground; we use anchor-relative RSSI position estimation instead, and the packet format already has a field reserved for GPS at surface deployment.
- *"Isn't flooding for emergencies wasteful?"* → It's a deliberate trade: efficient distance-vector routing for routine data, flood-with-TTL only for safety-critical traffic, because guaranteed delivery matters more than airtime when it counts.
- *"Does this scale past your demo?"* → Not as-is — single-byte-style addressing and demo-scale node counts are a known, named ceiling; a production version moves to a proper mesh protocol and a real backend. Naming this proactively reads as maturity, not weakness.

If your event doesn't expect a backup slide, an alternative use for Slide 16 is a **live-metrics or architecture-diagram recap** slide instead — whichever the judges' format rewards more.

---

## Slide 17 — Thank You

- **Thank You**
- Team name and team member names/roles (matches the template's existing prompt exactly).
- Optional: contact/repo link, and a one-line callback to the pitch's opening line for a clean close (e.g., *"Sensora — a safety state that survives the failure."*).

---

## Notes on filling this in

- Keep the visual template as-is (colors, diagrams, layout) — it's well-suited to this content; you're replacing instructional placeholder copy with the real thing, not redesigning slides.
- Slides 4, 8, 9, 11, 14, 15 have two side-by-side content blocks each — keep both sides roughly balanced in length so the layout doesn't look lopsided once real text replaces the placeholder paragraphs (the template's placeholder text runs ~40-60 words per block; aim for the same).
- Say this once, early: **the same sensor types as existing published solutions, but different underlying algorithms and different reliability guarantees** — this is your single clearest differentiator and it should show up on slides 8, 12, and 14, not just once.
