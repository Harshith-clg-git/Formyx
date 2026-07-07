"""
formyx_backend/tracking/target_tracker.py
-----------------------------------------
Tracks target position and velocity in 3D space using a Kalman Filter.
Implements Mahalanobis distance gating to reject outliers/clutter and
velocity damping to handle periods of target occlusion.
"""

from __future__ import annotations

import logging
from typing import Tuple

import numpy as np

from config import get

log = logging.getLogger(__name__)


class TargetTracker:
    """
    3D Kalman Filter target tracker.
    Tracks state: [x, y, z, vx, vy, vz]^T
    """

    def __init__(self) -> None:
        # Load parameters from configuration
        self.q_coeff: float = get("tracking", "kalman_process_noise", 0.1)
        self.r_coeff: float = get("tracking", "kalman_measurement_noise", 0.5)
        self.gate_dist: float = get("tracking", "mahalanobis_gate_distance", 9.21)
        self.max_lost_frames: int = get("tracking", "max_lost_frames", 30)
        
        # Velocity damping coefficient (prevents target prediction from drifting infinitely)
        self.damping: float = get("tracking", "velocity_damping", 0.1)

        # State vector: [x, y, z, vx, vy, vz]^T
        self._x = np.zeros(6, dtype=np.float64)
        # Covariance matrix
        self._P = np.eye(6, dtype=np.float64) * 10.0  # high initial uncertainty
        
        self.is_initialized = False
        self.lost_frames = 0

        # Measurement matrix (we only observe x, y, z positions)
        self._H = np.zeros((3, 6), dtype=np.float64)
        self._H[0, 0] = 1.0
        self._H[1, 1] = 1.0
        self._H[2, 2] = 1.0

        # Measurement noise covariance matrix (3x3)
        self._R = np.eye(3, dtype=np.float64) * self.r_coeff

        log.info(
            "TargetTracker initialized: process_noise=%.2f, "
            "measurement_noise=%.2f, mahalanobis_gate=%.2f, max_lost=%d",
            self.q_coeff,
            self.r_coeff,
            self.gate_dist,
            self.max_lost_frames,
        )

    def reset(self) -> None:
        """Reset the tracker state and covariances."""
        log.info("Resetting TargetTracker state.")
        self._x.fill(0.0)
        self._P = np.eye(6, dtype=np.float64) * 10.0
        self.is_initialized = False
        self.lost_frames = 0

    def predict(self, dt: float) -> None:
        """
        Propagate the Kalman Filter state estimate forward in time by dt.

        Parameters
        ----------
        dt : float
            Time elapsed since the last step in seconds.
        """
        if not self.is_initialized or dt <= 0.0:
            return

        # 1. State Transition Matrix F
        F = np.eye(6, dtype=np.float64)
        F[0, 3] = dt
        F[1, 4] = dt
        F[2, 5] = dt

        # 2. Process Noise Covariance Matrix Q (scaled by dt)
        # Position states get smaller process noise, velocity states get larger.
        Q = np.diag([
            self.q_coeff * (dt ** 2) / 2.0,
            self.q_coeff * (dt ** 2) / 2.0,
            self.q_coeff * (dt ** 2) / 2.0,
            self.q_coeff * dt,
            self.q_coeff * dt,
            self.q_coeff * dt
        ])

        # 3. Propagate state: x = F * x
        self._x = F.dot(self._x)

        # 4. Apply velocity damping to prevent infinite prediction drift
        damping_factor = max(0.0, 1.0 - self.damping * dt)
        self._x[3:6] *= damping_factor

        # 5. Propagate covariance: P = F * P * F^T + Q
        self._P = F.dot(self._P).dot(F.T) + Q

    def update(self, measurement: Tuple[float, float, float]) -> bool:
        """
        Update the filter state with a new 3D position measurement.

        Parameters
        ----------
        measurement : Tuple[float, float, float]
            The (x, y, z) measured relative target position.

        Returns
        -------
        bool
            True if the measurement was within the Mahalanobis gate and accepted,
            False if it was rejected or if the filter is not yet initialized.
        """
        z = np.array(measurement, dtype=np.float64)

        if not self.is_initialized:
            # First measurement: initialize state directly
            self._x[0:3] = z
            self._x[3:6] = 0.0  # initial velocity is 0
            # Initialize covariance
            self._P = np.diag([1.0, 1.0, 1.0, 5.0, 5.0, 5.0])
            self.is_initialized = True
            self.lost_frames = 0
            log.info("TargetTracker initialized at target position (%.2f, %.2f, %.2f)", *measurement)
            return True

        # Compute innovation residual: y = z - H * x
        y = z - self._H.dot(self._x)

        # Innovation covariance: S = H * P * H^T + R
        S = self._H.dot(self._P).dot(self._H.T) + self._R

        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            log.error("Failed to invert innovation covariance matrix S.")
            return False

        # Calculate Mahalanobis distance squared: d2 = y^T * S_inv * y
        d2 = y.T.dot(S_inv).dot(y)

        # Gating check
        if d2 > self.gate_dist:
            self.lost_frames += 1
            log.debug(
                "Measurement (%.2f, %.2f, %.2f) rejected by Mahalanobis gate: "
                "d2=%.2f > gate=%.2f. Lost frames: %d",
                *measurement, d2, self.gate_dist, self.lost_frames
            )
            return False

        # Measurement accepted -> perform Kalman update
        # Kalman Gain: K = P * H^T * S_inv
        K = self._P.dot(self._H.T).dot(S_inv)

        # Update state: x = x + K * y
        self._x = self._x + K.dot(y)

        # Update covariance using numerically stable Joseph form:
        # P = (I - K*H) * P * (I - K*H)^T + K * R * K^T
        I = np.eye(6, dtype=np.float64)
        I_KH = I - K.dot(self._H)
        self._P = I_KH.dot(self._P).dot(I_KH.T) + K.dot(self._R).dot(K.T)

        self.lost_frames = 0
        return True

    def get_state(self) -> Tuple[float, float, float, float, float, float] | None:
        """
        Retrieve the current state estimate.

        Returns
        -------
        Tuple[float, float, float, float, float, float] or None
            (x, y, z, vx, vy, vz) state estimate if valid, or None if the
            tracker is uninitialized or has lost the target for too long.
        """
        if not self.is_initialized or self.lost_frames >= self.max_lost_frames:
            return None
        return tuple(self._x)
