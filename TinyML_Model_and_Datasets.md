# TinyML in Bleep — Model, Training, and Datasets

This document explains how the tinyML anomaly-detection model works in Bleep's second worker/wall node pair (`firmware/worker_n/`, `firmware/wall_n/`), how it was trained, and — the part most worth reading carefully — where the training data actually came from and what that does and doesn't mean for the model's accuracy today.

## 1. Why tinyML instead of hand-picked thresholds

The original firmware (`worker_node.ino`) alarms on each sensor independently: if temperature crosses 40°C, or MQ-4 crosses a raw ADC value, that single reading alone latches a hazard. That works, but it can't express *combinations* — for example, a moderately elevated heart rate is completely normal for a worker who is moving, but the same heart rate at zero motion energy is a much stronger signal something is wrong. A bank of independent static thresholds can't see that relationship; each sensor is judged in isolation.

`worker_n.ino` layers a small **autoencoder** on top of the same sensors to catch exactly that kind of cross-sensor anomaly, without needing hand-labeled "this combination is dangerous" examples. It runs alongside the existing per-sensor thresholds (both are described in `dashboard_features.md`'s Feature 4 section) rather than replacing them.

## 2. Model architecture

- **Type:** a 1-hidden-layer autoencoder — `Linear → tanh → Linear → sigmoid`.
- **Shape:** 8 inputs → 6 hidden units → 8 outputs (reconstructing the same 8 inputs). That's roughly 8×6 + 6×8 = 96 weights plus biases — small enough to hand-implement as a couple of nested loops in C.
- **Inputs (in this fixed order, from `buildFeatureVector()` in `worker_n.ino`):**
  1. `temperatureC` (DHT11)
  2. `humidityPct` (DHT11)
  3. `airQualityRaw` (MQ-135, raw ADC)
  4. `combustibleGasRaw` (MQ-4, raw ADC)
  5. `heartRateBpm` (MAX30102)
  6. `spo2Pct` (MAX30102)
  7. `soilMoisturePct`
  8. `motionEnergy` (MPU6050-derived)

There is no `pressureHPa` in the feature set — the worker board has no BMP180 (that sensor only exists on `wall_n.ino`'s own local sensor set), so pressure is excluded entirely rather than sent as a dummy value.

**How inference works on-device:** each reading is min/max-normalized against a learned "normal" range per sensor, run through the tiny network, and compared against its own reconstruction. If the network — which has only ever learned to reconstruct *normal* combinations well — can't reconstruct a worker's current readings accurately, the reconstruction error spikes, and that error is the anomaly score. This is the standard unsupervised approach for constrained-hardware anomaly detection: no labeled "bad" examples are needed, only examples of normal operation.

**No ML framework on the device.** Inference is plain float matrix math (`tinyMLInfer()` in `worker_n.ino`) — no TensorFlow Lite Micro runtime, no third-party inference library. The only place any ML framework (`numpy`) is used at all is offline, during training on a laptop; the ESP32 only ever reads a generated header file (`tinyml_model.h`) full of float arrays and does the multiply-adds itself. This keeps the firmware's flash/RAM footprint small and avoids pinning the project to a specific TFLite Micro library version.

## 3. Training pipeline (`ml/train_tinyml_model.py`)

The trainer is a single, dependency-light Python script (`numpy` only) that:

1. Generates a training dataset (see §4 below — this is the important part).
2. Min/max-normalizes it using a fixed "plausible normal range" per sensor.
3. Trains the autoencoder for 3000 epochs of full-batch gradient descent on MSE reconstruction loss.
4. Computes the **anomaly threshold** as `mean(reconstruction error) + 3 × std(reconstruction error)` across the training set — i.e., a reading is flagged anomalous once its reconstruction error is a clear statistical outlier relative to what "normal" data reconstructs to.
5. Computes a **per-sensor threshold band** (`mean ± 3 standard deviations` of that sensor's *raw* training values), which is what the dashboard displays as each sensor's live ML-derived safe range, and what several of `worker_n.ino`'s own hazard thresholds (`TEMP_DANGER_C`, `MQ135_DANGER_RAW`, `HR_DANGER_HIGH_BPM`, etc.) are actually pulled from — see `TINYML_FEATURE_HIGH_THRESHOLD` / `TINYML_FEATURE_LOW_THRESHOLD` in the generated header.
6. Writes the trained weights, normalization ranges, anomaly threshold, and per-sensor bands into `tinyml_model.h` — generated **twice**, once into `firmware/worker_n/` and once into `firmware/wall_n/`, since the Arduino IDE only looks for a header inside the same folder as the `.ino` it's compiling.

Every per-sensor threshold band is additionally clamped to a **hard safety bound** (`HARD_SAFETY_BOUND` in the script) before being written out — e.g. the model can never report a "safe" ceiling above 40°C for temperature, or below 90% for SpO2, no matter what the training data says. This is defensive: the learned statistics can *tighten* a safety margin, but they can never loosen one past a physically-justified ceiling/floor. That guardrail matters a lot given what the training data actually is, covered next.

## 4. Where the training data comes from (read this part)

**There is no logged real-world sensor history for this project yet.** The dataset the model is trained on today is entirely **synthetic** — generated by `generate_synthetic_dataset()` in the training script, not collected from real workers or real mine conditions. Concretely:

- For each of the 8 sensors, a *plausible normal operating range* was defined by hand (`FEATURE_NORMAL_RANGE` in the script), based on:
  - the existing hand-picked danger thresholds already in `worker_node.ino` (e.g. "danger ≥ 40°C" implies "normal" tops out somewhere below that), and
  - typical sensor datasheet / operating ranges for each component (DHT11, MQ-135, MQ-4, MAX30102, generic soil moisture sensor).
- 6000 synthetic samples are drawn per training run, one row per sample, with each feature sampled from a truncated Gaussian centered in the middle of its normal range (standard deviation chosen so ~99.7% of samples land inside the defined range).
- A small amount of **deliberate cross-feature correlation** is injected — specifically, higher synthetic motion energy nudges heart rate up and SpO2 down slightly, mimicking mild exertion — because the entire point of using an autoencoder instead of independent thresholds is to let it learn correlations between sensors. Purely independent per-column noise wouldn't give it anything to learn there.
- One exception worth calling out: the MQ-4 (combustible gas) "normal" range was **widened from an original 100–1300 assumption to 100–3400** after real hardware testing across multiple independent sessions consistently showed this specific MQ-4 module's clean-air raw baseline sitting anywhere from ~2100 to ~3300 — not the much lower range originally assumed from the datasheet alone. That's the one place actual hardware behavior (not just datasheet numbers) has already fed back into the "synthetic" range definition, and it's a good example of why this needs to keep happening.

**What this means in practice:**
- The model has never seen a real worker's real vitals, a real mine's real ambient gas/temperature drift, or real sensor noise characteristics (beyond the one MQ-4 correction above). Its notion of "normal" is an informed guess, not a measurement.
- It is explicitly documented as a **placeholder appropriate for a hackathon-stage build**, not a claim of field-validated accuracy. The training script's own module docstring says so directly, and this document is repeating that on purpose so it doesn't get lost.
- The hard safety bounds (§3) exist specifically to contain the risk of that placeholder-ness — even if the synthetic data's assumptions are wrong in some direction, the model can't report a threshold outside the physically-justified danger boundary for that sensor.

## 5. Retraining on real data

The training script was written so that swapping synthetic data for real data is a small, contained change:

1. Log real STATUS packets from `worker_n.ino` (field names are documented in `wifi_json_protocol.md` / `sensora_telemetry.schema.json`) — from workers actually wearing the device under normal working conditions, ideally across a range of ambient temperature/humidity/gas conditions the site actually sees.
2. Replace `generate_synthetic_dataset()` in `ml/train_tinyml_model.py` with code that loads that logged CSV/JSON into the same 8-column layout (`FEATURE_NAMES` order).
3. Re-run `python3 ml/train_tinyml_model.py`. It overwrites both copies of `tinyml_model.h` with weights fit to the real data.

Nothing in `worker_n.ino` or `wall_n.ino` needs to change for this — the firmware only ever reads the generated header's arrays and constants, so it's agnostic to whether they came from synthetic or real training data. This is the intended next step before relying on this model's thresholds in an actual deployment, not an optional nice-to-have.

## 6. Summary

| | |
|---|---|
| Model | 8→6→8 autoencoder (tanh hidden, sigmoid output), ~96 weights |
| Training framework | numpy only, offline (laptop), full-batch gradient descent |
| On-device inference | Plain C float math, no ML runtime/library on the ESP32 |
| Training data | **Synthetic**, generated from hand-defined "plausible normal" ranges per sensor, informed by existing hardcoded thresholds + datasheets + limited real hardware calibration (MQ-4 only so far) |
| Real-world validation | None yet — explicitly flagged as a placeholder pending logged real sensor data |
| Safety guardrail | Learned thresholds are hard-clamped to physically-justified min/max bounds that training data can never loosen |
