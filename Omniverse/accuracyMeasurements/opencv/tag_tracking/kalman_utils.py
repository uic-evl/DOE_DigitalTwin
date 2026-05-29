import cv2
import numpy as np
from collections import deque

def create_kalman():
    dt = 1 / 30.0  # assume ~30 FPS

    kf = cv2.KalmanFilter(6, 3)

    # State: [x, y, z, vx, vy, vz]
    kf.transitionMatrix = np.array([
        [1, 0, 0, dt, 0,  0],
        [0, 1, 0, 0,  dt, 0],
        [0, 0, 1, 0,  0,  dt],
        [0, 0, 0, 1,  0,  0],
        [0, 0, 0, 0,  1,  0],
        [0, 0, 0, 0,  0,  1]
    ], dtype=np.float32)

    # Measurement: [x, y, z]
    kf.measurementMatrix = np.array([
        [1, 0, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 0, 1, 0, 0, 0]
    ], dtype=np.float32)

    # Tune these matrices!
    kf.processNoiseCov = np.eye(6, dtype=np.float32) * 1e-3
    kf.measurementNoiseCov = np.eye(3, dtype=np.float32) * 1e-2
    kf.errorCovPost = np.eye(6, dtype=np.float32)

    return kf












##############################
"""
Evaluate Kalman performance over recent history
raw: (x,y,z)
filtered: (x,y,z)
prediction: (x,y,z) from predict()
"""
def update_kalman_diagnostics(marker_id, raw, filtered, prediction, diag_history, diag_length):
    if marker_id not in diag_history:
        diag_history[marker_id] = {
            "raw": deque(maxlen=diag_length),
            "filtered": deque(maxlen=diag_length),
            "innovation": deque(maxlen=diag_length)
        }

    hist = diag_history[marker_id]

    raw = np.array(raw)
    filtered = np.array(filtered)
    prediction = np.array(prediction)

    # Store values
    hist["raw"].append(raw)
    hist["filtered"].append(filtered)

    innovation = np.linalg.norm(raw - prediction)
    hist["innovation"].append(innovation)

    # Only compute if enough samples
    if len(hist["raw"]) < diag_length:
        return None

    raw_arr = np.array(hist["raw"])
    filt_arr = np.array(hist["filtered"])

    # Variance (noise level)
    raw_var = np.var(raw_arr, axis=0).mean()
    filt_var = np.var(filt_arr, axis=0).mean()

    # Improvement ratio
    reduction = 1.0 - (filt_var / (raw_var + 1e-6))

    # Innovation average
    avg_innovation = np.mean(hist["innovation"])

    return {
        "raw_var": raw_var,
        "filt_var": filt_var,
        "noise_reduction": reduction,
        "innovation": avg_innovation
    }
