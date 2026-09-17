#!/usr/bin/env python3
"""
Bleep worker_n.ino -- on-device anomaly-detection model trainer.

Trains a tiny autoencoder (9 -> 6 -> 9) on synthetic "normal worker
conditions" data, one feature per worker sensor. The autoencoder learns
what a NORMAL combination of readings looks like; at runtime on the
ESP32, the same 9 numbers are fed through the same weights, and a
worker's readings are flagged anomalous when the network can no longer
reconstruct them accurately (reconstruction error above a learned
threshold). This is the standard tinyML pattern for unsupervised
anomaly detection on constrained hardware: small enough to run as a
few dozen multiply-adds per loop() tick, no dataset/labels needed in
the field, and it naturally accounts for correlations between sensors
(e.g. "high heart rate is normal in isolation, but only in combination
with high motion energy") that a bank of independent static thresholds
can't express.

WHERE THE TRAINING DATA COMES FROM
-----------------------------------
There is no logged real-world sensor history for this project yet, so
this script generates synthetic training data from the plausible
"normal" ranges documented in worker_node.ino's existing static
threshold constants (TEMP_DANGER_C, MQ135_DANGER_RAW, HR_DANGER_*,
etc.) and typical datasheet/operating ranges for each sensor. This is
a reasonable placeholder for a hackathon build, but it is NOT a
substitute for real data: re-run this script on logged real sensor
values (see "RETRAINING ON REAL DATA" below) as soon as they exist, to
let the model learn the actual normal envelope for your workers and
environment instead of an assumed one.

WHAT THIS SCRIPT PRODUCES
--------------------------
Running this script regenerates firmware/tinyml_model.h, a plain C
header containing:
  - the trained weight/bias matrices (float arrays)
  - the per-feature normalization range (min/max) used before inference
  - the overall reconstruction-error anomaly threshold
  - a derived, human-readable min/max "safe" threshold PER SENSOR
    (mean +/- 3 standard deviations of the training data), which is
    what worker_n.ino also reports to the dashboard as this sensor's
    live ML-derived threshold band -- this is the "set the threshold
    for all possible sensors using the model" part: instead of a
    hand-picked magic number per sensor, each threshold comes directly
    out of the same statistics the model was trained on.

RETRAINING ON REAL DATA
------------------------
Once you have logged real STATUS packets (see wifi_json_protocol.md /
sensora_telemetry.schema.json for the field names), replace
`generate_synthetic_dataset()` below with code that loads your logged
CSV/JSON into the same 9-column layout, and re-run:
    python3 ml/train_tinyml_model.py
It will overwrite firmware/tinyml_model.h with weights fitted to your
real data. Nothing in worker_n.ino needs to change -- it only ever
reads the header's arrays and constants.

No third-party ML framework is required on purpose (no TensorFlow,
no TFLite Micro runtime on the device) -- only numpy, for training.
Inference on the ESP32 is plain float matrix math, which keeps the
firmware's flash/RAM footprint small and avoids pinning the project to
a specific TFLite Micro library version.
"""

import os
import numpy as np

RNG = np.random.default_rng(42)

FEATURE_NAMES = [
    "temperatureC",
    "humidityPct",
    "airQualityRaw",     # MQ-135
    "combustibleGasRaw", # MQ-4
    "heartRateBpm",
    "spo2Pct",
    "soilMoisturePct",
    "motionEnergy",
]
N_FEATURES = len(FEATURE_NAMES)

# Plausible NORMAL operating range per sensor -- used both to generate
# synthetic training samples and to min/max-normalize inputs before
# they hit the network. Keep in sync with worker_node.ino's existing
# *_DANGER_* constants: these ranges sit safely inside them.
#
# No pressureHPa here: worker_n.ino's worker board has no BMP180 (that
# sensor only exists on wall_n.ino's local sensor set) -- see the
# module docstring update if that ever changes.
FEATURE_NORMAL_RANGE = {
    "temperatureC":      (18.0, 34.0),   # danger >= 40 (TEMP_DANGER_C)
    "humidityPct":       (25.0, 85.0),
    "airQualityRaw":     (150.0, 1600.0),   # danger >= 4000 (MQ135_DANGER_RAW)
    # Field evidence (multiple independent test sessions on real
    # hardware) consistently showed this specific MQ-4 module's clean-
    # air raw baseline sitting anywhere from ~2100 to ~3300, not the
    # ~100-1300 originally assumed here -- that recurring pattern
    # across sessions is real sensor/divider behavior, not a one-off
    # warm-up transient, so the "normal" range is widened to match it.
    # See wifi_json_protocol.md's calibration caveat: MQ raw values
    # are never a calibrated ppm reading and always need on-site
    # tuning per physical sensor.
    "combustibleGasRaw": (100.0, 3400.0),   # danger >= 3800 (MQ4_DANGER_RAW, see HARD_SAFETY_BOUND)
    "heartRateBpm":      (52.0, 128.0),     # danger <=45 or >=130
    "spo2Pct":           (92.0, 100.0),     # danger <= 90
    "soilMoisturePct":   (3.0, 72.0),       # danger >= 80
    "motionEnergy":      (0.0, 5.0),        # ambient movement noise band
}

# Absolute physical/safety ceilings that the model's derived threshold
# must never be relaxed past, even if the synthetic/logged training
# data happened to under- or over-represent extreme values. Defense in
# depth for a safety device: the ML threshold can tighten these, never
# loosen them. `None` means "no hard ceiling in that direction".
HARD_SAFETY_BOUND = {
    "temperatureC":      (None, 40.0),
    "humidityPct":       (None, None),
    "airQualityRaw":     (None, 4000.0),
    # Raised from the original 3000 -- see FEATURE_NORMAL_RANGE
    # comment above. This still leaves headroom above the observed
    # ~2100-3300 baseline for a genuine leak to be caught, but stop
    # relying on this number alone: recalibrate the MQ-4 module's
    # onboard trim pot per its datasheet, and let it run a full
    # warm-up cycle, then update this constant from real numbers.
    "combustibleGasRaw": (None, 3800.0),
    "heartRateBpm":      (45.0, 130.0),
    "spo2Pct":           (90.0, None),
    "soilMoisturePct":   (None, 80.0),
    "motionEnergy":      (None, None),
}


def generate_synthetic_dataset(n_samples: int = 6000) -> np.ndarray:
    """Synthetic 'normal worker conditions' samples, one row per sample,
    columns in FEATURE_NAMES order. Each feature is drawn from a
    truncated Gaussian centered in its normal range, with mild
    cross-feature correlation (e.g. motion energy nudges heart rate
    up) so the autoencoder has more than independent per-column noise
    to learn from -- a bank of independent thresholds already captures
    independent noise just fine; the point of the model is the
    correlations."""
    cols = []
    motion = np.clip(RNG.normal(1.2, 1.0, n_samples), 0.0, 5.0)

    for name in FEATURE_NAMES:
        lo, hi = FEATURE_NORMAL_RANGE[name]
        mean = (lo + hi) / 2.0
        std = (hi - lo) / 6.0  # ~99.7% of samples land inside [lo, hi]
        col = RNG.normal(mean, std, n_samples)

        if name == "heartRateBpm":
            col = col + motion * 4.0  # moving worker -> mildly elevated HR
        if name == "spo2Pct":
            col = col - motion * 0.3  # mild exertion dip, still normal

        cols.append(np.clip(col, lo, hi))

    cols[FEATURE_NAMES.index("motionEnergy")] = motion
    return np.column_stack(cols)


def minmax_normalize(x: np.ndarray) -> np.ndarray:
    lo = np.array([FEATURE_NORMAL_RANGE[n][0] for n in FEATURE_NAMES])
    hi = np.array([FEATURE_NORMAL_RANGE[n][1] for n in FEATURE_NAMES])
    return (x - lo) / (hi - lo)


def sigmoid(z):
    return 1.0 / (1.0 + np.exp(-z))


def train_autoencoder(x_norm: np.ndarray, hidden_size: int = 6,
                       epochs: int = 3000, lr: float = 0.1):
    """Plain-numpy 1-hidden-layer autoencoder: Linear -> tanh -> Linear
    -> sigmoid, trained by full-batch gradient descent on MSE
    reconstruction loss. Small and simple on purpose -- this whole
    network is ~9*6 + 6*9 = 108 weights, trivial to re-implement as a
    handful of float loops in C on an ESP32."""
    n_in = x_norm.shape[1]
    w1 = RNG.normal(0, np.sqrt(1.0 / n_in), (n_in, hidden_size))
    b1 = np.zeros(hidden_size)
    w2 = RNG.normal(0, np.sqrt(1.0 / hidden_size), (hidden_size, n_in))
    b2 = np.zeros(n_in)

    n = x_norm.shape[0]
    for epoch in range(epochs):
        z1 = x_norm @ w1 + b1
        h = np.tanh(z1)
        z2 = h @ w2 + b2
        out = sigmoid(z2)

        err = out - x_norm
        loss = np.mean(err ** 2)

        d_out = (2.0 / n) * err * out * (1 - out)
        d_w2 = h.T @ d_out
        d_b2 = d_out.sum(axis=0)

        d_h = d_out @ w2.T
        d_z1 = d_h * (1 - h ** 2)
        d_w1 = x_norm.T @ d_z1
        d_b1 = d_z1.sum(axis=0)

        w1 -= lr * d_w1
        b1 -= lr * d_b1
        w2 -= lr * d_w2
        b2 -= lr * d_b2

        if epoch % 500 == 0 or epoch == epochs - 1:
            print(f"  epoch {epoch:5d}  MSE={loss:.6f}")

    return w1, b1, w2, b2


def reconstruction_errors(x_norm, w1, b1, w2, b2) -> np.ndarray:
    h = np.tanh(x_norm @ w1 + b1)
    out = sigmoid(h @ w2 + b2)
    return np.mean((out - x_norm) ** 2, axis=1)


def per_feature_threshold_band(raw: np.ndarray):
    """mean +/- 3*std per feature, clamped to the hard safety bound in
    each direction. Returned as (lowThreshold[], highThreshold[])."""
    mean = raw.mean(axis=0)
    std = raw.std(axis=0)
    lows, highs = [], []
    for i, name in enumerate(FEATURE_NAMES):
        lo = mean[i] - 3 * std[i]
        hi = mean[i] + 3 * std[i]
        hard_lo, hard_hi = HARD_SAFETY_BOUND[name]
        if hard_lo is not None:
            lo = max(lo, hard_lo)  # never call a value "safe" below the hard danger floor
        if hard_hi is not None:
            hi = min(hi, hard_hi)  # never call a value "safe" above the hard danger ceiling
        lows.append(lo)
        highs.append(hi)
    return np.array(lows), np.array(highs)


def emit_c_array(name: str, arr: np.ndarray) -> str:
    flat = arr.flatten()
    values = ", ".join(f"{v:.8f}f" for v in flat)
    dims = "][".join(str(d) for d in arr.shape) if arr.ndim > 1 else str(arr.shape[0])
    return f"const float {name}[{dims}] = {{ {values} }};"


def main():
    print("Generating synthetic training data...")
    raw = generate_synthetic_dataset()
    x_norm = minmax_normalize(raw)

    print("Training autoencoder...")
    w1, b1, w2, b2 = train_autoencoder(x_norm)

    errors = reconstruction_errors(x_norm, w1, b1, w2, b2)
    anomaly_threshold = float(errors.mean() + 3 * errors.std())
    print(f"Reconstruction error: mean={errors.mean():.6f} std={errors.std():.6f} "
          f"-> anomaly threshold={anomaly_threshold:.6f}")

    low_band, high_band = per_feature_threshold_band(raw)

    lo_arr = np.array([FEATURE_NORMAL_RANGE[n][0] for n in FEATURE_NAMES])
    hi_arr = np.array([FEATURE_NORMAL_RANGE[n][1] for n in FEATURE_NAMES])

    header = []
    header.append("// AUTO-GENERATED by ml/train_tinyml_model.py -- do not hand-edit.")
    header.append("// Re-run that script (see its module docstring) to regenerate this")
    header.append("// file from a new dataset instead of editing the numbers below.")
    header.append("#pragma once")
    header.append("")
    header.append(f"#define TINYML_N_FEATURES {N_FEATURES}")
    header.append(f"#define TINYML_HIDDEN_SIZE {w1.shape[1]}")
    header.append("")
    header.append("// Feature order used everywhere below and in worker_n.ino's")
    header.append(f"// buildFeatureVector(): {', '.join(FEATURE_NAMES)}.")
    header.append("")
    header.append(emit_c_array("TINYML_FEATURE_MIN", lo_arr))
    header.append(emit_c_array("TINYML_FEATURE_MAX", hi_arr))
    header.append("")
    header.append(emit_c_array("TINYML_W1", w1))
    header.append(emit_c_array("TINYML_B1", b1))
    header.append(emit_c_array("TINYML_W2", w2))
    header.append(emit_c_array("TINYML_B2", b2))
    header.append("")
    header.append(f"const float TINYML_ANOMALY_THRESHOLD = {anomaly_threshold:.8f}f;")
    header.append("")
    header.append("// Per-sensor threshold band derived from the training data's own")
    header.append("// mean +/- 3 standard deviations, clamped so it can never claim a")
    header.append("// value is 'safe' beyond the hard physical safety ceiling for that")
    header.append("// sensor (see HARD_SAFETY_BOUND in the training script). Reported")
    header.append("// to the dashboard as each sensor's live ML-derived threshold.")
    header.append(emit_c_array("TINYML_FEATURE_LOW_THRESHOLD", low_band))
    header.append(emit_c_array("TINYML_FEATURE_HIGH_THRESHOLD", high_band))
    header.append("")

    header_text = "\n".join(header) + "\n"

    # Written into BOTH sketch folders, not just firmware/ -- the
    # Arduino IDE only looks for #include "tinyml_model.h" inside the
    # same folder as the .ino file being compiled (each sketch folder
    # must be named exactly like its .ino file), so one copy sitting
    # next to worker_n.ino and another next to wall_n.ino is required,
    # not a nice-to-have. Keep it that way after any manual reshuffling
    # of the firmware/ folder.
    firmware_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "firmware")
    out_paths = [
        os.path.join(firmware_dir, "worker_n", "tinyml_model.h"),
        os.path.join(firmware_dir, "wall_n", "tinyml_model.h"),
    ]
    for out_path in out_paths:
        os.makedirs(os.path.dirname(out_path), exist_ok=True)
        with open(out_path, "w") as f:
            f.write(header_text)
        print(f"Wrote {out_path}")
    print("\nPer-sensor learned threshold band:")
    for i, name in enumerate(FEATURE_NAMES):
        print(f"  {name:20s} safe=[{low_band[i]:.2f}, {high_band[i]:.2f}]")


if __name__ == "__main__":
    main()
