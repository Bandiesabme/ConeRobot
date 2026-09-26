#!/usr/bin/env python3
"""
==============================================================================
Motion Profiler: Deterministic S-Curve / Trapezoidal Trajectory Generation
==============================================================================
Provides smooth velocity and heading profiles for:
  - STRAIGHT: Linear translation with zero angular velocity
  - ROTATE: In-place pivot rotation with zero linear velocity
  - ARC: Synchronous translation and heading change (curvature continuous)
  - STOP: Controlled deceleration profile to standstill

Designed for skid-steer / tracked robots to prevent sudden torque spikes and
track slip at motion initiation.
==============================================================================
"""

import math
from typing import Tuple, Optional


class TrapezoidalProfile1D:
    """Computes a 1D trapezoidal velocity profile for distance or angle."""

    def __init__(
        self,
        total_distance: float,
        max_speed: float,
        max_accel: float,
        start_speed: float = 0.0,
        end_speed: float = 0.0,
    ):
        self.total_dist = max(1e-5, abs(total_distance))
        self.sign = 1.0 if total_distance >= 0 else -1.0
        self.max_accel = max(1e-3, abs(max_accel))
        self.max_speed = max(1e-3, abs(max_speed))
        self.start_speed = min(self.max_speed, max(0.0, abs(start_speed)))
        self.end_speed = min(self.max_speed, max(0.0, abs(end_speed)))

        # Distance required to accelerate from start_speed to max_speed
        d_accel = (self.max_speed ** 2 - self.start_speed ** 2) / (2.0 * self.max_accel)
        # Distance required to decelerate from max_speed to end_speed
        d_decel = (self.max_speed ** 2 - self.end_speed ** 2) / (2.0 * self.max_accel)

        if d_accel + d_decel > self.total_dist:
            # Triangular profile: peak cruise speed cannot be reached
            self.v_cruise = math.sqrt(
                max(
                    0.0,
                    (2.0 * self.max_accel * self.total_dist + self.start_speed ** 2 + self.end_speed ** 2) / 2.0,
                )
            )
            self.t_accel = (self.v_cruise - self.start_speed) / self.max_accel
            self.d_accel = (self.v_cruise ** 2 - self.start_speed ** 2) / (2.0 * self.max_accel)
            self.t_cruise = 0.0
            self.d_cruise = 0.0
            self.t_decel = (self.v_cruise - self.end_speed) / self.max_accel
            self.d_decel = (self.v_cruise ** 2 - self.end_speed ** 2) / (2.0 * self.max_accel)
        else:
            # Full trapezoidal profile
            self.v_cruise = self.max_speed
            self.t_accel = (self.v_cruise - self.start_speed) / self.max_accel
            self.d_accel = d_accel
            self.d_cruise = self.total_dist - (d_accel + d_decel)
            self.t_cruise = self.d_cruise / self.v_cruise
            self.t_decel = (self.v_cruise - self.end_speed) / self.max_accel
            self.d_decel = d_decel

        self.total_duration = self.t_accel + self.t_cruise + self.t_decel

    def sample(self, t: float) -> Tuple[float, float, bool]:
        """
        Samples the profile at elapsed time t.
        Returns:
            (velocity, position, is_finished)
        """
        if t <= 0.0:
            return self.sign * self.start_speed, 0.0, False

        if t >= self.total_duration:
            return self.sign * self.end_speed, self.sign * self.total_dist, True

        # Phase 1: Acceleration
        if t < self.t_accel:
            v = self.start_speed + self.max_accel * t
            s = self.start_speed * t + 0.5 * self.max_accel * (t ** 2)
            return self.sign * v, self.sign * s, False

        # Phase 2: Cruise
        t_into_cruise = t - self.t_accel
        if t_into_cruise < self.t_cruise:
            v = self.v_cruise
            s = self.d_accel + self.v_cruise * t_into_cruise
            return self.sign * v, self.sign * s, False

        # Phase 3: Deceleration
        t_into_decel = t_into_cruise - self.t_cruise
        v = max(self.end_speed, self.v_cruise - self.max_accel * t_into_decel)
        s = (
            self.d_accel
            + self.d_cruise
            + (self.v_cruise * t_into_decel - 0.5 * self.max_accel * (t_into_decel ** 2))
        )
        return self.sign * v, self.sign * s, False


class MotionPrimitiveProfiler:
    """
    High-level coordinated trajectory generator for Straight, Rotate, and Arc primitives.
    """

    def __init__(
        self,
        default_linear_speed: float = 0.25,
        default_angular_speed: float = 1.0,
        max_linear_accel: float = 0.5,
        max_angular_accel: float = 1.5,
    ):
        self.default_linear_speed = default_linear_speed
        self.default_angular_speed = default_angular_speed
        self.max_linear_accel = max_linear_accel
        self.max_angular_accel = max_angular_accel

        self.motion_type: Optional[str] = None
        self.profile: Optional[TrapezoidalProfile1D] = None
        self.curvature: float = 0.0
        self.target_dist: float = 0.0
        self.target_yaw: float = 0.0

    def start_straight(
        self, distance_m: float, max_v: Optional[float] = None
    ) -> float:
        """Configures straight drive. Returns total expected duration in seconds."""
        self.motion_type = "STRAIGHT"
        self.target_dist = distance_m
        self.target_yaw = 0.0
        self.curvature = 0.0

        v_limit = max_v if (max_v and max_v > 0) else self.default_linear_speed
        self.profile = TrapezoidalProfile1D(
            total_distance=distance_m,
            max_speed=v_limit,
            max_accel=self.max_linear_accel,
        )
        return self.profile.total_duration

    def start_rotate(
        self, delta_yaw_rad: float, max_w: Optional[float] = None
    ) -> float:
        """Configures in-place rotation. Returns total expected duration in seconds."""
        self.motion_type = "ROTATE"
        self.target_dist = 0.0
        self.target_yaw = delta_yaw_rad
        self.curvature = 0.0

        w_limit = max_w if (max_w and max_w > 0) else self.default_angular_speed
        self.profile = TrapezoidalProfile1D(
            total_distance=delta_yaw_rad,
            max_speed=w_limit,
            max_accel=self.max_angular_accel,
        )
        return self.profile.total_duration

    def start_arc(
        self,
        distance_m: float,
        delta_yaw_rad: float,
        max_v: Optional[float] = None,
        max_w: Optional[float] = None,
    ) -> float:
        """
        Configures synchronous circular arc.
        Calculates curvature kappa = delta_yaw / distance.
        Ramps v(t) and omega(t) proportionally so heading and distance complete simultaneously.
        """
        self.motion_type = "ARC"
        self.target_dist = distance_m
        self.target_yaw = delta_yaw_rad

        dist_abs = max(1e-4, abs(distance_m))
        self.curvature = delta_yaw_rad / dist_abs

        v_limit = max_v if (max_v and max_v > 0) else self.default_linear_speed
        w_limit = max_w if (max_w and max_w > 0) else self.default_angular_speed

        # Enforce that v_limit does not cause omega to exceed w_limit
        if abs(self.curvature * v_limit) > w_limit:
            v_limit = abs(w_limit / self.curvature)

        # Enforce that linear acceleration does not exceed angular acceleration limit
        a_limit = self.max_linear_accel
        if abs(self.curvature * a_limit) > self.max_angular_accel:
            a_limit = abs(self.max_angular_accel / self.curvature)

        self.profile = TrapezoidalProfile1D(
            total_distance=distance_m,
            max_speed=v_limit,
            max_accel=a_limit,
        )
        return self.profile.total_duration

    def sample(
        self, elapsed_t: float
    ) -> Tuple[float, float, float, float, bool]:
        """
        Samples the active trajectory at elapsed time t.
        Returns:
            (v_ref, omega_ref, s_ref, yaw_ref, is_finished)
        """
        if not self.profile:
            return 0.0, 0.0, 0.0, 0.0, True

        val, pos, finished = self.profile.sample(elapsed_t)

        if self.motion_type == "STRAIGHT":
            return val, 0.0, pos, 0.0, finished

        if self.motion_type == "ROTATE":
            # For rotate: val is omega, pos is yaw
            return 0.0, val, 0.0, pos, finished

        if self.motion_type == "ARC":
            # For arc: val is linear speed v, pos is linear distance s
            v_ref = val
            omega_ref = self.curvature * v_ref
            s_ref = pos
            yaw_ref = self.curvature * s_ref
            return v_ref, omega_ref, s_ref, yaw_ref, finished

        return 0.0, 0.0, 0.0, 0.0, True
