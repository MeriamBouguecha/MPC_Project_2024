#!/usr/bin/env python3
"""Render real MATLAB trajectories as static GitHub PNG previews.

Run after MATLAB build_showcase: python3 scripts/render_assets.py
Python is used only for rendering; the control simulation runs in MATLAB.
"""
import json
import os
from pathlib import Path
import tempfile

os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "highway-mpc-mpl"))
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
BG, PANEL, FG, MUTED = "#080f18", "#101b29", "#e9f0f7", "#91a4b8"
CYAN, GOLD, GRID = "#49dfce", "#ffbc68", "#253345"


def make_figure(data):
    t = np.array(data["t"])
    states = np.array(data["states"])
    inputs = np.array(data["inputs"])
    ref = np.array(data["reference"])
    predictions = np.array(data["predictionY"])
    team = True
    nonlinear = data['id'].startswith('nonlinear')
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10,
                         "text.color": FG, "axes.labelcolor": MUTED,
                         "xtick.color": MUTED, "ytick.color": MUTED,
                         "axes.edgecolor": GRID, "axes.titlecolor": FG})
    fig = plt.figure(figsize=(14, 8.6), facecolor=BG)
    fig.text(.055, .95, "HIGHWAY / MPC", color=CYAN, weight="bold", fontsize=12)
    fig.text(.945, .95, "NONLINEAR PLANT  /  10 Hz CONTROL", ha="right", color=MUTED, fontsize=9)
    fig.text(.055, .89, data["title"], weight="bold", fontsize=27)
    fig.text(.055, .85, data["description"], color=MUTED, fontsize=11)
    stats = [fig.text(x, .785, "", fontsize=18, weight="bold") for x in [.055, .29, .53, .77]]
    for x, label in zip([.055, .29, .53, .77], ["SPEED", "LATERAL POSITION", "HEADING", "THROTTLE"]):
        fig.text(x, .82, label, fontsize=8, color=MUTED)
    road = fig.add_axes([.055, .48, .89, .265], facecolor=PANEL)
    road.set_ylim(-2.5, 5.5)
    road.set_yticks([0, 3], labels=["LANE 01", "LANE 02"])
    road.tick_params(axis="y", length=0, labelsize=8)
    road.set_xlabel("Longitudinal position [m]  ·  lateral scale enlarged for legibility", fontsize=9)
    road.axhspan(-1.5, 4.5, color="#142433")
    for y in [-1.5, 4.5]:
        road.axhline(y, color="#6a7c8d", lw=1.2)
    road.axhline(1.5, color="#8292a4", lw=1, ls=(0, (8, 10)))
    for y in [0, 3]:
        road.axhline(y, color=GRID, lw=1, ls=":")
    trail, = road.plot([], [], color=CYAN, lw=2.5, label="Nonlinear trajectory")
    forecast, = road.plot([], [], color=GOLD, lw=1.5, ls="--", label="_nolegend_" if team else "MPC lateral prediction")
    vehicle = Polygon(np.zeros((4, 2)), closed=True, facecolor=CYAN, edgecolor="#cafff3", lw=1.2, zorder=5)
    road.add_patch(vehicle)
    glass = Polygon(np.zeros((4, 2)), closed=True, facecolor="#12313b", edgecolor="none", zorder=6)
    road.add_patch(glass)
    lead = Polygon(np.zeros((4, 2)), closed=True, facecolor=GOLD, edgecolor="#ffe2b8", lw=1.2, zorder=5,
                   label="Lead vehicle" if data.get("otherStates") else "_nolegend_")
    lead.set_visible(bool(data.get("otherStates")))
    road.add_patch(lead)
    road.legend(loc="upper right", facecolor=PANEL, edgecolor="none", labelcolor=FG, fontsize=8, ncol=2)
    stamp = road.text(.015, .93, "", transform=road.transAxes, color=GOLD, fontsize=9)
    cursors = []
    specs = [
        ("Speed tracking", "km/h", [(states[:, 3]*3.6, CYAN, "Measured"),
                                     (ref[:, 1]*3.6, GOLD, "Reference")]),
        ("Lane tracking", "m", [(states[:, 1], CYAN, "Measured"),
                                 (ref[:, 0], GOLD, "Reference")]),
        ("Control inputs" if team else "Control & estimation", "normalized", [(inputs[:, 1], CYAN, "Throttle"),
                     (inputs[:, 0]*180/np.pi if team else np.array(data["disturbance"]), GOLD,
                      "Steering [deg]" if team else "Estimated disturbance")])]
    for i, (title, unit, series) in enumerate(specs):
        ax = fig.add_axes([.055+i*.31, .135, .265, .245], facecolor=PANEL)
        for values, color, label in series:
            ax.plot(t[:len(values)], values, color=color, lw=1.7,
                    ls="--" if label == "Reference" else "-", label=label)
        ax.set(title=title, xlabel="Time [s]", ylabel=unit, xlim=(0, t[-1]))
        ax.grid(color=GRID, alpha=.7, lw=.6)
        ax.spines[["top", "right"]].set_visible(False)
        ax.legend(loc="best", fontsize=8, facecolor=PANEL, edgecolor="none", labelcolor=FG)
        cursors.append(ax.axvline(0, color=FG, lw=1, ls=":", alpha=.65))
    m = data["metrics"]
    fig.text(.055, .045, "SAMPLED CONSTRAINTS SATISFIED" if m["constraintViolation"] <= 1e-5 else "CONSTRAINT VIOLATION", color=CYAN, fontsize=9)
    caption = "Team nonlinear MPC · 10 s horizon · CasADi / IPOPT" if nonlinear else "Team linear MPC · 5 s horizon · YALMIP / quadprog"
    fig.text(.945, .045, caption, ha="right", fontsize=9, color=MUTED)

    def frame(k):
        k = min(k, len(t)-1)
        x = states[k]
        u = inputs[min(k, len(inputs)-1)]
        for artist, label in zip(stats, [f"{x[3]*3.6:.1f} km/h", f"{x[1]:.2f} m", f"{np.degrees(x[2]):.2f} deg", f"{u[1]:+.2f}"]):
            artist.set_text(label)
        road.set_xlim(x[0]-22, x[0]+78)
        trail.set_data(states[max(0, k-65):k+1, 0], states[max(0, k-65):k+1, 1])
        if k < len(predictions):
            forecast.set_data(x[0]+x[3]*data["dt"]*np.arange(1, predictions.shape[1]+1), predictions[k])
        else:
            forecast.set_data([], [])
        # Stylized body length for legibility on the enlarged lateral scale.
        vertices = np.array([[-3.5, -.9], [3.5, -.9], [3.5, .9], [-3.5, .9]])
        angle = x[2]
        rotation = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
        vehicle.set_xy(vertices @ rotation.T + x[:2])
        glass.set_xy(np.array([[-1.7,-.7],[.7,-.7],[.7,.7],[-1.7,.7]]) @ rotation.T+x[:2])
        if data.get("otherStates"):
            lead.set_xy(vertices+np.array(data["otherStates"][k][:2]))
        stamp.set_text(f"t = {t[k]:04.1f} s  /  target {ref[k, 0]:.0f} m · {ref[k, 1]*3.6:.0f} km/h")
        for cursor in cursors:
            cursor.set_xdata([t[k], t[k]])
        return stats + [trail, forecast, vehicle, glass, lead, stamp] + cursors

    return fig, frame


def main():
    base = ROOT / "docs"
    output = base / "assets"
    output.mkdir(parents=True, exist_ok=True)
    names = ["nonlinear-tracking", "nonlinear-avoidance", "tracking", "offset-free"]
    for name in names:
        data = json.loads((base / "data" / f"{name}.json").read_text())
        fig, frame = make_figure(data)
        frame(round((7.0 if name == 'nonlinear-avoidance' else 4.0)/data["dt"]))
        path = output / f"{name}.png"
        fig.savefig(path, dpi=130, facecolor=BG)
        print(path.relative_to(ROOT))
        plt.close(fig)


if __name__ == "__main__":
    main()
