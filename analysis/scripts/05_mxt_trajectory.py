# -*- coding: utf-8 -*-
"""
MXT で実際に送った指令と、コントローラーが返した位置から
速度・加速度・ジャークを求める（issue #7）。

  記録:  python 05_mxt_trajectory.py record --host 192.168.0.10 --port 10000 --to 300 50 --name move1
  解析:  python 05_mxt_trajectory.py analyze analysis/results/mxt_move1.csv

record は send_packet() で現在位置から --to の XY へ動かし、送信した指令と
コントローラーの応答を CSV に保存する（numpy と pydantic だけで動く）。
analyze は CSV から最大速度・加速度・ジャークを計算し、計画値と比較して
JSON とグラフ（matplotlib が必要）を出力する。
"""

import argparse
import contextlib
import io
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(ROOT / "src"))

from melfapy.Melfa import MelfaController, MelfaPose  # noqa: E402
from melfapy.utils.advanced_S_curve_acceleration import AdvancedSCurvePlanner  # noqa: E402
from melfapy.utils.trajectory import (  # noqa: E402
    load_trajectory_csv,
    local_derivatives,
    save_trajectory_csv,
)

RESULTS_DIR = ROOT / "analysis" / "results"
XYZ = ("x", "y", "z")

# 系列の色（順番固定）: 計画 / 送信した指令 / コントローラーの応答
COLORS = {"planned": "#2a78d6", "sent": "#eb6834", "reply": "#1baf7a"}
INK = "#3d3d3a"
GRID = "#e4e3de"
SURFACE = "#fcfcfb"


def record(args) -> None:
    address = (args.host, args.port)
    controller = MelfaController(
        command=1,
        send_type=1,
        recv_type=args.recv_type,
        pose=MelfaPose(values=[0] * 10),
        address=address,
        v_max=args.v_max,
        a_max=args.a_max,
        j_max=args.j_max,
    )
    with contextlib.redirect_stdout(io.StringIO()):
        start = list(controller.get_position())
    target = [args.to[0], args.to[1], start[2], 0, 0, math.degrees(start[5]), 0, 0, 4, 0]
    controller.pose = MelfaPose(values=target)
    with contextlib.redirect_stdout(io.StringIO()):
        controller.send_packet()
    trajectory = controller.trajectory

    csv_path = save_trajectory_csv(trajectory, RESULTS_DIR / f"mxt_{args.name}.csv")
    meta = {
        "name": args.name,
        "address": list(address),
        "recv_type": args.recv_type,
        "limits": {"v_max": args.v_max, "a_max": args.a_max, "j_max": args.j_max},
        "sleep_time": controller.sleep_time,
        "start": [float(v) for v in start[:8]],
        "target": [float(v) for v in target[:3]],
        "samples": len(trajectory),
    }
    csv_path.with_suffix(".meta.json").write_text(json.dumps(meta, indent=2))
    print(f"{args.name}: {start[:3]} -> {target[:3]}  samples={len(trajectory)}  saved {csv_path}")


def _planned(meta, axis_index, t):
    limits = meta["limits"]
    planner = AdvancedSCurvePlanner(
        meta["start"][axis_index],
        meta["target"][axis_index],
        limits["v_max"],
        limits["a_max"],
        limits["j_max"],
    )
    profile = np.array([planner.get_profile(v) for v in t], dtype=float)
    return planner, profile


def _peaks(vel, acc, jerk) -> dict:
    return {
        "peak_velocity": float(np.max(np.abs(vel))),
        "peak_acceleration": float(np.max(np.abs(acc))),
        "peak_jerk": float(np.max(np.abs(jerk))),
    }


def _follow(meta, t_cmd, cmd, rep, moving) -> dict:
    """How the controller's replies relate to what was commanded.

    packet_shift: n if every reply equals the command sent n packets earlier.
    lag_ms / rms_mm / max_mm: best pure time delay between the planned profile
    and the replies, and how far the replies are from that delayed profile.
    """
    limits = meta["limits"]
    shift = None
    for n in range(0, 6):
        if len(cmd) > n + 2 and np.allclose(rep[n:, moving], cmd[: len(cmd) - n, moving], atol=2e-3):
            shift = n
            break

    planners = [
        AdvancedSCurvePlanner(meta["start"][i], meta["target"][i], limits["v_max"], limits["a_max"], limits["j_max"])
        for i in moving
    ]

    def residual(tau):
        return np.concatenate(
            [rep[:, i] - np.array([pl.get_profile(v)[0] for v in t_cmd - tau]) for i, pl in zip(moving, planners)]
        )

    taus = np.arange(-0.02, 0.25, 0.0005)
    rms = [float(np.sqrt(np.mean(residual(tau) ** 2))) for tau in taus]
    best = int(np.argmin(rms))
    res = residual(taus[best])
    return {
        "packet_shift": shift,
        "lag_ms": float(taus[best] * 1000),
        "rms_mm": rms[best],
        "max_mm": float(np.max(np.abs(res))),
        "_residual": res.reshape(len(moving), -1),
    }


def analyze_one(csv_path: Path, window, plot: bool) -> dict:
    samples = load_trajectory_csv(csv_path)
    meta = json.loads(csv_path.with_suffix(".meta.json").read_text())
    limits = meta["limits"]

    t_cmd = np.array([s.t_command for s in samples])
    t_rep = np.array([s.t_reply for s in samples])
    cmd = np.array([s.command for s in samples])
    rep = np.array([s.reply for s in samples])
    intervals = np.diff(t_cmd)
    if window is None:
        # 3次式の当てはめに十分な点数が入る幅にする
        window = max(0.06, 8 * float(np.median(intervals)))

    result = {
        "name": meta["name"],
        "recv_type": meta["recv_type"],
        "limits": limits,
        "start": meta["start"][:3],
        "target": meta["target"],
        "samples": len(samples),
        "duration_s": float(t_rep[-1]),
        "derivative_window_s": window,
        "send_interval_ms": {
            "mean": float(intervals.mean() * 1000),
            "p99": float(np.percentile(intervals, 99) * 1000),
            "max": float(intervals.max() * 1000),
        },
        "reply_delay_ms": {
            "mean": float((t_rep - t_cmd).mean() * 1000),
            "max": float((t_rep - t_cmd).max() * 1000),
        },
        "final_error_mm": [float(rep[-1][i] - meta["target"][i]) for i in range(3)],
        "axes": {},
    }

    series = {}
    for i, axis in enumerate(XYZ):
        if abs(meta["target"][i] - meta["start"][i]) < 1e-6:
            continue
        planner, planned = _planned(meta, i, t_cmd)
        sent = local_derivatives(t_cmd, cmd[:, i], window)
        steps = np.abs(np.diff(cmd[:, i]))
        result["axes"][axis] = {
            "travel_mm": float(meta["target"][i] - meta["start"][i]),
            "planned": {
                "time_s": float(planner.T),
                "peak_velocity": float(planner.v_lim),
                "peak_acceleration": float(planner.a_lim),
                "peak_jerk": float(planner.j_max),
            },
            "sent": _peaks(*sent[1:]),
            "max_step_mm": float(steps.max()),
            "max_step_speed": float(np.max(steps / intervals)),
        }
        series[axis] = (planned, sent, rep[:, i])

    moving = [XYZ.index(a) for a in series]
    follow = None
    if moving:
        # XYZ の合成（経路）速度: 軸ごとに独立して計画しているので各軸の最大値より大きくなる
        sent_speed = np.sqrt(sum(series[XYZ[i]][1][1] ** 2 for i in moving))
        planned_speed = np.sqrt(sum(series[XYZ[i]][0][:, 1] ** 2 for i in moving))
        result["path_speed"] = {
            "planned": float(planned_speed.max()),
            "sent": float(sent_speed.max()),
            "v_max": limits["v_max"],
        }
        follow = _follow(meta, t_cmd, cmd, rep, moving)
        result["reply"] = {k: v for k, v in follow.items() if not k.startswith("_")}

    csv_path.with_suffix(".stats.json").write_text(json.dumps(result, indent=2))
    if plot and series:
        _plot(csv_path.with_suffix(".png"), meta, t_cmd, series, follow)
    return result


def _plot(path: Path, meta, t_cmd, series, follow) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    rows = (
        ("position [mm]", 0),
        ("velocity [mm/s]", 1),
        ("acceleration [mm/s²]", 2),
        ("jerk [mm/s³]", 3),
        ("reply - delayed plan [mm]", None),
    )
    axes_names = list(series)
    fig, axs = plt.subplots(
        len(rows),
        len(axes_names),
        figsize=(5.2 * len(axes_names), 11.5),
        sharex=True,
        squeeze=False,
        facecolor=SURFACE,
    )
    for col, axis in enumerate(axes_names):
        planned, sent, reply = series[axis]
        for row, (label, k) in enumerate(rows):
            ax = axs[row][col]
            ax.set_facecolor(SURFACE)
            if k is None:
                ax.plot(t_cmd, follow["_residual"][col], color=COLORS["reply"], lw=2)
                ax.axhline(0, color=GRID, lw=1)
            else:
                ax.plot(t_cmd, planned[:, k], color=COLORS["planned"], lw=2, ls="--", label="planned profile")
                ax.plot(t_cmd, sent[k], color=COLORS["sent"], lw=2, label="commands sent")
                if k == 0:
                    ax.plot(t_cmd, reply, color=COLORS["reply"], lw=2, ls=":", label="controller reply")
            ax.grid(True, color=GRID, lw=0.8)
            ax.tick_params(colors=INK, labelsize=9)
            for side in ("top", "right"):
                ax.spines[side].set_visible(False)
            for side in ("left", "bottom"):
                ax.spines[side].set_color(GRID)
            if col == 0:
                ax.set_ylabel(label, color=INK, fontsize=10)
            if row == 0:
                ax.set_title(f"{axis.upper()} axis", color=INK, fontsize=11, loc="left")
            if row == len(rows) - 1:
                ax.set_xlabel("time [s]", color=INK, fontsize=10)
    handles, labels = axs[0][0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper right", ncol=3, frameon=False, fontsize=10, labelcolor=INK)
    limits = meta["limits"]
    start = [round(v, 1) for v in meta["start"][:3]]
    target = [round(v, 1) for v in meta["target"]]
    fig.suptitle(
        f"{meta['name']}: {start} -> {target}\n"
        f"v_max={limits['v_max']} a_max={limits['a_max']} j_max={limits['j_max']}   "
        f"recv_type={meta['recv_type']}   reply lag {follow['lag_ms']:.0f} ms",
        x=0.01,
        ha="left",
        fontsize=11,
        color=INK,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(path, dpi=130, facecolor=SURFACE)
    plt.close(fig)


def analyze(args) -> None:
    for name in args.csv:
        r = analyze_one(Path(name), args.window, not args.no_plot)
        iv = r["send_interval_ms"]
        print(f"\n== {r['name']}  recv_type={r['recv_type']}  {[round(v, 1) for v in r['start']]} -> {[round(v, 1) for v in r['target']]}")
        print(
            f"   samples={r['samples']}  duration={r['duration_s']:.3f}s  "
            f"send interval mean={iv['mean']:.2f} p99={iv['p99']:.2f} max={iv['max']:.2f} ms  "
            f"final error={['%.3f' % e for e in r['final_error_mm']]} mm"
        )
        print(f"   {'axis':4} {'source':8} {'v [mm/s]':>10} {'a [mm/s2]':>10} {'j [mm/s3]':>10}")
        for axis, a in r["axes"].items():
            for source in ("planned", "sent"):
                p = a[source]
                print(
                    f"   {axis:4} {source:8} {p['peak_velocity']:10.1f} "
                    f"{p['peak_acceleration']:10.1f} {p['peak_jerk']:10.1f}"
                )
            print(
                f"   {axis:4} planned time={a['planned']['time_s']:.3f}s  "
                f"max step={a['max_step_mm']:.3f} mm ({a['max_step_speed']:.1f} mm/s between packets)"
            )
        if "path_speed" in r:
            ps = r["path_speed"]
            print(f"   path speed: planned={ps['planned']:.1f} sent={ps['sent']:.1f} mm/s (v_max={ps['v_max']})")
        if "reply" in r:
            f = r["reply"]
            shift = "none" if f["packet_shift"] is None else f"{f['packet_shift']} packet(s)"
            print(
                f"   reply: exact copy of an earlier command: {shift};  "
                f"lag to plan {f['lag_ms']:.1f} ms, deviation rms {f['rms_mm']:.3f} mm, max {f['max_mm']:.3f} mm"
            )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="mode", required=True)

    rec = sub.add_parser("record", help="動かして CSV に記録する")
    rec.add_argument("--host", default="192.168.0.10")
    rec.add_argument("--port", type=int, default=10000)
    rec.add_argument("--to", type=float, nargs=2, required=True, metavar=("X", "Y"))
    rec.add_argument("--name", required=True)
    rec.add_argument("--recv-type", type=int, default=1, help="1: 指令値, 4: フィルタ後, 7: エンコーダFB")
    rec.add_argument("--v-max", type=int, default=300)
    rec.add_argument("--a-max", type=int, default=500)
    rec.add_argument("--j-max", type=int, default=700)
    rec.set_defaults(func=record)

    ana = sub.add_parser("analyze", help="記録した CSV を解析する")
    ana.add_argument("csv", nargs="+")
    ana.add_argument("--window", type=float, default=None, help="微分に使う時間窓 [s]（省略時は送信間隔から決める）")
    ana.add_argument("--no-plot", action="store_true")
    ana.set_defaults(func=analyze)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
