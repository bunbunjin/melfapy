"""Recording and analysis of trajectories streamed to the controller over MXT."""

import csv
from pathlib import Path
from typing import NamedTuple, Sequence

import numpy as np

AXES = ("x", "y", "z", "a", "b", "c", "l1", "l2")


class TrajectorySample(NamedTuple):
    """One command packet and the controller's reply to it.

    t_command:  time the command pose was sampled from the profile [s]
    t_reply:    time the reply was received [s] (same clock as t_command)
    command:    the 8 pose values sent to the controller
    reply_type: type of the data in the reply (1: command, 4: filtered, 7: feedback ...)
    reply:      the 8 pose values reported by the controller
    ccount:     communication counter reported by the controller
    """

    t_command: float
    t_reply: float
    command: tuple
    reply_type: int
    reply: tuple
    ccount: int


def save_trajectory_csv(samples: Sequence[TrajectorySample], path) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    header = (
        ["t_command", "t_reply", "reply_type", "ccount"]
        + [f"cmd_{axis}" for axis in AXES]
        + [f"rep_{axis}" for axis in AXES]
    )
    with path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        for s in samples:
            writer.writerow(
                [repr(s.t_command), repr(s.t_reply), s.reply_type, s.ccount]
                + [repr(float(v)) for v in s.command]
                + [repr(float(v)) for v in s.reply]
            )
    return path


def load_trajectory_csv(path) -> list[TrajectorySample]:
    samples = []
    with Path(path).open(newline="") as f:
        reader = csv.reader(f)
        next(reader)
        for row in reader:
            values = [float(v) for v in row]
            samples.append(
                TrajectorySample(
                    t_command=values[0],
                    t_reply=values[1],
                    reply_type=int(values[2]),
                    ccount=int(values[3]),
                    command=tuple(values[4:12]),
                    reply=tuple(values[12:20]),
                )
            )
    return samples


def local_derivatives(t, x, window: float = 0.06):
    """Estimate position, velocity, acceleration and jerk at every sample.

    A cubic polynomial is fitted to the samples within +-window/2 of each
    sample. Each phase of an S-curve is a cubic in position, so the fit is
    exact inside a phase and only smooths across phase boundaries, while
    averaging out timing jitter and float32 rounding of the positions.

    Returns four arrays (position, velocity, acceleration, jerk).
    """
    t = np.asarray(t, dtype=float)
    x = np.asarray(x, dtype=float)
    n = len(t)
    pos = np.empty(n)
    vel = np.empty(n)
    acc = np.empty(n)
    jerk = np.empty(n)
    half = window / 2
    for i in range(n):
        lo = np.searchsorted(t, t[i] - half, side="left")
        hi = np.searchsorted(t, t[i] + half, side="right")
        # A cubic needs at least 4 points; widen the window at the edges.
        while hi - lo < 6 and (lo > 0 or hi < n):
            lo = max(lo - 1, 0)
            hi = min(hi + 1, n)
        tt = t[lo:hi] - t[i]
        degree = min(3, len(tt) - 1)
        coeffs = np.polyfit(tt, x[lo:hi], degree)[::-1]  # c0 + c1 t + c2 t^2 + c3 t^3
        coeffs = np.pad(coeffs, (0, 4 - len(coeffs)))
        pos[i] = coeffs[0]
        vel[i] = coeffs[1]
        acc[i] = 2 * coeffs[2]
        jerk[i] = 6 * coeffs[3]
    return pos, vel, acc, jerk


def summarize_axis(t, x, window: float = 0.06) -> dict:
    """Peak velocity / acceleration / jerk of one axis of a recorded trajectory."""
    _, vel, acc, jerk = local_derivatives(t, x, window)
    return {
        "travel": float(x[-1] - x[0]),
        "peak_velocity": float(np.max(np.abs(vel))),
        "peak_acceleration": float(np.max(np.abs(acc))),
        "peak_jerk": float(np.max(np.abs(jerk))),
    }
