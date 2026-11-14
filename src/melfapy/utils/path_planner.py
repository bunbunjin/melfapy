"""Straight-line path planning on top of the 7-segment S-curve profile."""

import numpy as np

from .advanced_S_curve_acceleration import AdvancedSCurvePlanner


class LinearPathPlanner:
    """Move (x, y, z) along a straight line with a synchronized rotation.

    One S-curve is planned along the path length, so v_max / a_max / j_max
    limit the motion of the tool along the path rather than each axis on its
    own, and every axis starts and finishes together.

    The rotation is planned with the same limits (in its own unit) and whichever
    of the two takes longer sets the timing of the whole move.

    snap: path distance [mm] next to the start and the target within which the
    pose is held at exactly the start / target. Positions are sent as float32,
    so a move of a few nanometres changes some axes but not others; at the edge
    of the workspace that rounded pose can be out of reach.
    """

    def __init__(
        self,
        start_xyz,
        target_xyz,
        start_angle: float,
        target_angle: float,
        v_max: float,
        a_max: float,
        j_max: float,
        snap: float = 1e-3,
    ):
        self.start_xyz = np.asarray(start_xyz, dtype=float)
        self.target_xyz = np.asarray(target_xyz, dtype=float)
        self.start_angle = float(start_angle)
        self.target_angle = float(target_angle)
        self.snap = snap

        self._delta = self.target_xyz - self.start_xyz
        self.distance = float(np.linalg.norm(self._delta))
        self._delta_angle = self.target_angle - self.start_angle

        self.linear = AdvancedSCurvePlanner(0.0, self.distance, v_max, a_max, j_max)
        self.rotation = AdvancedSCurvePlanner(
            0.0, abs(self._delta_angle), v_max, a_max, j_max
        )
        if self.linear.T >= self.rotation.T:
            self._master, self._length = self.linear, self.distance
        else:
            self._master, self._length = self.rotation, abs(self._delta_angle)
        self.T = float(self._master.T)

    def raw_progress(self, t: float) -> float:
        """Fraction of the move completed at time t (0 .. 1), without snapping."""
        if self._length == 0:
            return 1.0
        return float(self._master.get_profile(t)[0]) / self._length

    def progress(self, t: float) -> float:
        """Fraction of the move completed at time t, held at 0 / 1 near the ends."""
        if t >= self.T:
            return 1.0
        sigma = self.raw_progress(t)
        travelled = sigma * self.distance
        if self.distance > 0:
            if travelled < self.snap:
                return 0.0
            if self.distance - travelled < self.snap:
                return 1.0
        return sigma

    def pose(self, t: float) -> tuple[float, float, float, float]:
        """(x, y, z, angle) to command at time t."""
        sigma = self.progress(t)
        if sigma <= 0.0:
            xyz, angle = self.start_xyz, self.start_angle
        elif sigma >= 1.0:
            xyz, angle = self.target_xyz, self.target_angle
        else:
            xyz = self.start_xyz + self._delta * sigma
            angle = self.start_angle + self._delta_angle * sigma
        return float(xyz[0]), float(xyz[1]), float(xyz[2]), float(angle)

    def axis_profile(self, axis: int, t: float) -> tuple[float, float, float, float]:
        """Planned (position, velocity, acceleration, jerk) of x / y / z at time t."""
        if self._length == 0:
            return float(self.target_xyz[axis]), 0.0, 0.0, 0.0
        pos, vel, acc, jerk = self._master.get_profile(t)
        scale = self._delta[axis] / self._length
        return (
            float(self.start_xyz[axis] + scale * pos),
            float(scale * vel),
            float(scale * acc),
            float(scale * jerk),
        )
