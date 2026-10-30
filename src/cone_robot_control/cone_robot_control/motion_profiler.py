#!/usr/bin/env python3
"""
==============================================================================
Motion Profiler: State-Driven (Position & Angle) Trajectory Generator
==============================================================================
Generates smooth velocity and heading references purely as a function of
MEASURED spatial progress (distance traveled s and heading change yaw),
NEVER open-loop wall-clock time.

Supports:
  - STRAIGHT: Linear translation X meters with locked 0° relative yaw
  - ROTATE:   In-place pivot rotation Y radians with 0 linear velocity
  - ARC:      Synchronous curved trajectory (distance X meters, turn Y radians)
              with constant geometric curvature kappa = Y / X and spatial
              heading reference yaw_ref(s) = (s / |X|) * Y.
==============================================================================
"""

import math
from typing import Tuple, Optional


class MotionPrimitiveProfiler:
    """
    State-based trajectory generator for Straight, Rotate, and Arc primitives.
    Computes target velocities from remaining measured distance/angle rather than time.
    """

    def __init__(
        self,
        default_linear_speed: float = 0.20,
        default_angular_speed: float = 1.0,
        max_linear_accel: float = 0.35,
        max_angular_accel: float = 1.5,
        min_linear_speed: float = 0.08,
        min_angular_speed: float = 0.70,
    ):
        self.default_linear_speed = default_linear_speed
        self.default_angular_speed = default_angular_speed
        self.max_linear_accel = max(0.05, max_linear_accel)
        self.max_angular_accel = max(0.10, max_angular_accel)
        self.min_linear_speed = max(0.04, min_linear_speed)
        self.min_angular_speed = max(0.20, min_angular_speed)

        self.motion_type: Optional[str] = None
        self.curvature: float = 0.0
        self.target_dist: float = 0.0
        self.target_yaw: float = 0.0

        self.v_limit: float = self.default_linear_speed
        self.w_limit: float = self.default_angular_speed
        self.a_limit: float = self.max_linear_accel
        self.alpha_limit: float = self.max_angular_accel

        self._current_v_mag: float = 0.0
        self._current_w_mag: float = 0.0

    def start_straight(
        self, distance_m: float, max_v: Optional[float] = None
    ) -> None:
        """Configures state-driven straight drive for X meters."""
        self.motion_type = "STRAIGHT"
        self.target_dist = float(distance_m)
        self.target_yaw = 0.0
        self.curvature = 0.0

        self.v_limit = max_v if (max_v and max_v > 0.0) else self.default_linear_speed
        self.a_limit = self.max_linear_accel
        self._current_v_mag = self.min_linear_speed
        self._current_w_mag = 0.0

    def start_rotate(
        self, delta_yaw_rad: float, max_w: Optional[float] = None
    ) -> None:
        """Configures state-driven in-place rotation for Y radians."""
        self.motion_type = "ROTATE"
        self.target_dist = 0.0
        self.target_yaw = float(delta_yaw_rad)
        self.curvature = 0.0

        self.w_limit = max_w if (max_w and max_w > 0.0) else self.default_angular_speed
        self.alpha_limit = self.max_angular_accel
        self._current_v_mag = 0.0
        self._current_w_mag = self.min_angular_speed

    def start_arc(
        self,
        distance_m: float,
        delta_yaw_rad: float,
        max_v: Optional[float] = None,
        max_w: Optional[float] = None,
    ) -> None:
        """
        Configures state-driven circular arc for path length X meters and heading change Y radians.
        Curvature kappa = delta_yaw_rad / signed_distance_m.
        """
        self.motion_type = "ARC"
        self.target_dist = float(distance_m)
        self.target_yaw = float(delta_yaw_rad)

        signed_dist = math.copysign(max(1e-4, abs(distance_m)), distance_m if distance_m != 0.0 else 1.0)
        self.curvature = delta_yaw_rad / signed_dist

        v_lim = max_v if (max_v and max_v > 0.0) else self.default_linear_speed
        w_lim = max_w if (max_w and max_w > 0.0) else self.default_angular_speed

        # Ensure linear speed does not force angular velocity above w_lim
        if abs(self.curvature * v_lim) > w_lim:
            v_lim = abs(w_lim / self.curvature)

        # Ensure linear acceleration does not force angular acceleration above max_angular_accel
        a_lim = self.max_linear_accel
        if abs(self.curvature * a_lim) > self.max_angular_accel:
            a_lim = abs(self.max_angular_accel / self.curvature)

        self.v_limit = max(self.min_linear_speed, v_lim)
        self.w_limit = w_lim
        self.a_limit = max(0.05, a_lim)
        self._current_v_mag = self.min_linear_speed
        self._current_w_mag = abs(self.curvature * self._current_v_mag)

    def sample_by_state(
        self,
        actual_dist_m: float,
        actual_yaw_rad: float,
        dt: float,
        dist_tolerance_m: float = 0.03,
        yaw_tolerance_rad: float = 0.031,
    ) -> Tuple[float, float, float, float, bool]:
        """
        Computes reference velocities and reference pose purely from measured state:
          - actual_dist_m:  Measured path distance traveled so far (meters, >= 0)
          - actual_yaw_rad: Measured relative yaw change so far (radians)
          - dt:             Control loop timestep (seconds, e.g. 0.02s for 50 Hz)

        Returns:
          (v_ref, omega_ref, s_ref, yaw_ref, is_finished)
        """
        if not self.motion_type:
            return 0.0, 0.0, 0.0, 0.0, True

        total_s = abs(self.target_dist)
        s = max(0.0, abs(actual_dist_m))
        dist_rem = max(0.0, total_s - s)
        dir_sign = 1.0 if self.target_dist >= 0.0 else -1.0

        # ----------------------------------------------------------------------
        # 1. STRAIGHT: Move X meters with locked heading (yaw_ref = 0.0)
        # ----------------------------------------------------------------------
        if self.motion_type == "STRAIGHT":
            if dist_rem <= dist_tolerance_m:
                self._current_v_mag = 0.0
                return 0.0, 0.0, self.target_dist, 0.0, True

            # Smooth acceleration slew from standstill
            self._current_v_mag = min(self.v_limit, self._current_v_mag + self.a_limit * dt)

            # Progressive 90 cm approach deceleration ramp down to gentle crawl speed
            # Uses a concave profile (ramp_ratio ** 1.35) so the final 15-20 cm is traversed
            # at ~4-5 cm/s, eliminating 1 Hz GPS latency & track momentum overshoot.
            crawl_speed = max(0.035, self.min_linear_speed * 0.45)
            decel_zone_m = min(0.90, max(0.25, total_s * 0.50))
            if dist_rem < decel_zone_m:
                ramp_ratio = max(
                    0.0,
                    (dist_rem - dist_tolerance_m) / max(1e-3, decel_zone_m - dist_tolerance_m),
                )
                v_approach = crawl_speed + (self.v_limit - crawl_speed) * (ramp_ratio ** 1.35)
            else:
                v_approach = self.v_limit

            v_mag = max(crawl_speed, min(self._current_v_mag, v_approach))
            self._current_v_mag = v_mag

            return dir_sign * v_mag, 0.0, dir_sign * s, 0.0, False

        # ----------------------------------------------------------------------
        # 2. ARC: Move X meters along curve while turning Y radians
        #    Heading reference yaw_ref(s) is strictly parameterized by distance s!
        # ----------------------------------------------------------------------
        if self.motion_type == "ARC":
            progress = min(1.0, max(0.0, s / max(1e-4, total_s)))
            yaw_ref = progress * self.target_yaw

            if dist_rem <= dist_tolerance_m:
                self._current_v_mag = 0.0
                return 0.0, 0.0, self.target_dist, self.target_yaw, True

            self._current_v_mag = min(self.v_limit, self._current_v_mag + self.a_limit * dt)

            crawl_speed = max(0.035, self.min_linear_speed * 0.45)
            decel_zone_m = min(0.90, max(0.25, total_s * 0.50))
            if dist_rem < decel_zone_m:
                ramp_ratio = max(
                    0.0,
                    (dist_rem - dist_tolerance_m) / max(1e-3, decel_zone_m - dist_tolerance_m),
                )
                v_approach = crawl_speed + (self.v_limit - crawl_speed) * (ramp_ratio ** 1.35)
            else:
                v_approach = self.v_limit

            v_mag = max(crawl_speed, min(self._current_v_mag, v_approach))
            self._current_v_mag = v_mag

            v_ref = dir_sign * v_mag
            omega_ref = self.curvature * v_ref
            return v_ref, omega_ref, dir_sign * s, yaw_ref, False

        # ----------------------------------------------------------------------
        # 3. ROTATE: Turn Y radians in place
        # ----------------------------------------------------------------------
        if self.motion_type == "ROTATE":
            yaw_err = self.target_yaw - actual_yaw_rad
            abs_err = abs(yaw_err)
            if abs_err <= yaw_tolerance_rad:
                self._current_w_mag = 0.0
                return 0.0, 0.0, 0.0, self.target_yaw, True

            turn_sign = 1.0 if yaw_err >= 0.0 else -1.0
            self._current_w_mag = min(self.w_limit, self._current_w_mag + self.alpha_limit * dt)

            # Proportional deceleration taper inside final 8 deg to prevent overshoot & hunting
            fine_band_rad = math.radians(8.0)
            err_above_tol = max(0.0, abs_err - yaw_tolerance_rad)
            taper_ratio = min(1.0, err_above_tol / max(1e-4, fine_band_rad))
            fine_floor_w = 0.35 + (self.min_angular_speed - 0.35) * taper_ratio

            w_brake = math.sqrt(
                fine_floor_w ** 2 + 2.0 * self.alpha_limit * err_above_tol
            )
            w_mag = max(fine_floor_w, min(self._current_w_mag, w_brake))
            self._current_w_mag = w_mag

            return 0.0, turn_sign * w_mag, 0.0, self.target_yaw, False

        return 0.0, 0.0, 0.0, 0.0, True
