"""
Render PNG views of the cage with matplotlib.  The drone shown is a PLACEHOLDER
built from the [MEASURE] parameters (body box, motor points, prop disks). It is
not a DJI model.
"""
import math
import os
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import trimesh  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Line3DCollection, Poly3DCollection  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
sys.path.insert(0, HERE)
from mini4pro_cage import Params  # noqa: E402

OUT = os.path.join(ROOT, "renders")
LIGHT = np.array([0.4, -0.5, 0.75])
LIGHT = LIGHT / np.linalg.norm(LIGHT)
COL = {"front": (0.16, 0.45, 0.85), "rear": (0.95, 0.55, 0.15), "test": (0.2, 0.65, 0.4)}


def shade(m, base):
    n = m.face_normals
    k = 0.35 + 0.65 * np.clip(n @ LIGHT, 0, 1)
    return np.clip(np.outer(k, base), 0, 1)


def add_mesh(ax, m, base, alpha=1.0):
    pc = Poly3DCollection(m.triangles, facecolors=shade(m, base), edgecolors="none",
                          linewidths=0, alpha=alpha)
    ax.add_collection3d(pc)


def placeholder_drone(ax, p: Params, alpha=0.18):
    x0, x1, y0, y1, z0, z1 = p.body_box
    c = np.array([[x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0],
                  [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]])
    e = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4), (0, 4), (1, 5), (2, 6), (3, 7)]
    ax.add_collection3d(Line3DCollection([[c[a], c[b]] for a, b in e], colors="0.45",
                                         linewidths=0.8, linestyles="--"))
    t = np.linspace(0, 2 * np.pi, 72)
    for mo in (p.front_motor, p.rear_motor):
        for s in (1, -1):
            mx, my, mz = mo[0], s * mo[1], mo[2]
            verts = [list(zip(mx + p.prop_r * np.cos(t), my + p.prop_r * np.sin(t), np.full_like(t, mz)))]
            ax.add_collection3d(Poly3DCollection(verts, facecolors=(0.6, 0.6, 0.6, alpha),
                                                 edgecolors=(0.3, 0.3, 0.3, 0.6), linewidths=0.6))
            ax.plot([mx, mx], [my, my], [mz - 18, mz + 2], color="0.25", lw=2.2)
            # placeholder arm: same direction the clips assume (toward airframe centre
            # unless an arm angle parameter is set), drawn to 30 mm from the centre
            ang = p.front_arm_angle if mo == p.front_motor else p.rear_arm_angle
            dz = p.front_clip_dz if mo == p.front_motor else p.rear_clip_dz
            a = math.atan2(-my, -mx) if ang is None else math.radians(s * ang)
            L = math.hypot(mx, my) - 30
            ax.plot([mx, mx + L * math.cos(a)], [my, my + L * math.sin(a)], [mz + dz] * 2,
                    color="0.35", lw=3)
    cx, cy, cz = p.camera_pt
    ax.scatter([cx], [cy], [cz], color="red", s=18)


def frame(ax, lim, elev, azim, title):
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_zlim(-lim * 0.6, lim * 0.6)
    ax.set_box_aspect((1, 1, 0.6))
    ax.view_init(elev=elev, azim=azim)
    ax.set_title(title, fontsize=10)
    ax.set_xlabel("X fwd (mm)", fontsize=7)
    ax.set_ylabel("Y left (mm)", fontsize=7)
    ax.set_zlabel("Z (mm)", fontsize=7)
    ax.tick_params(labelsize=6)


def load(rel):
    return trimesh.load(os.path.join(ROOT, rel), force="mesh")


def main():
    os.makedirs(OUT, exist_ok=True)
    p = Params()
    b = "build"
    asm = {k: load(f"{b}/asm_{k}.stl") for k in
           ("left_front", "left_rear", "right_front", "right_rear")}

    views = [("assembly_iso", 28, -135, "Assembled cage on PLACEHOLDER drone (iso, front-left)"),
             ("assembly_top", 89, -90, "Top view: prop disks grey, front modules blue, rear orange"),
             ("assembly_side_left", 2, -90, "Left side view: rear props assumed 10 mm lower")]
    for name, el, az, title in views:
        fig = plt.figure(figsize=(9, 8), dpi=130)
        ax = fig.add_subplot(111, projection="3d")
        placeholder_drone(ax, p)
        for k, m in asm.items():
            add_mesh(ax, m, COL["front" if "front" in k else "rear"])
        frame(ax, 190, el, az, title)
        fig.text(0.02, 0.02, "v0.2 PROVISIONAL. Drone geometry is a placeholder from unmeasured parameters.",
                 fontsize=7, color="0.3")
        fig.tight_layout()
        fig.savefig(os.path.join(OUT, f"{name}.png"))
        plt.close(fig)

    # left side cage only, close iso with joint highlighted
    fig = plt.figure(figsize=(9, 7), dpi=130)
    ax = fig.add_subplot(111, projection="3d")
    for k in ("left_front", "left_rear"):
        add_mesh(ax, asm[k], COL["front" if "front" in k else "rear"])
    ax.set_xlim(-170, 170)
    ax.set_ylim(0, 230)
    ax.set_zlim(-60, 40)
    ax.set_box_aspect((340, 230, 100))
    ax.view_init(elev=22, azim=-60)
    ax.set_title("Left cage assembly: front module (blue) + rear module (orange), lap joint at outboard bridge", fontsize=9)
    ax.tick_params(labelsize=6)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "left_cage_detail.png"))
    plt.close(fig)

    # joint close-up
    import json
    rep = json.load(open(os.path.join(ROOT, b, "build_report.json")))
    M = rep["joint_point"]
    fig = plt.figure(figsize=(8, 7), dpi=130)
    ax = fig.add_subplot(111, projection="3d")
    for k in ("left_front", "left_rear"):
        add_mesh(ax, asm[k], COL["front" if "front" in k else "rear"])
    ax.set_xlim(M[0] - 22, M[0] + 22)
    ax.set_ylim(M[1] - 22, M[1] + 22)
    ax.set_zlim(-35, 10)
    ax.set_box_aspect((44, 44, 45))
    ax.view_init(elev=25, azim=-30)
    ax.set_title("Bridge lap joint: two M2 bolts (or 1.75 mm filament pins) through both halves", fontsize=9)
    ax.tick_params(labelsize=6)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "joint_closeup.png"))
    plt.close(fig)

    # clip close-up in assembled pose (front)
    cc = rep["front_clip_center"]
    fig = plt.figure(figsize=(8, 7), dpi=130)
    ax = fig.add_subplot(111, projection="3d")
    add_mesh(ax, asm["left_front"], COL["front"])
    ax.set_xlim(cc[0] - 18, cc[0] + 18)
    ax.set_ylim(cc[1] - 18, cc[1] + 18)
    ax.set_zlim(cc[2] - 18, cc[2] + 18)
    ax.set_box_aspect((1, 1, 1))
    ax.view_init(elev=-25, azim=-150)
    ax.set_title("Front arm clip from below: snap opening faces down, zip-tie groove at mid-length", fontsize=9)
    ax.tick_params(labelsize=6)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "clip_closeup_from_below.png"))
    plt.close(fig)

    # print-pose sheet
    files = [("stl/fit_tests/front_mount_fit_test.stl", "Front mount fit test (print pose)"),
             ("stl/fit_tests/front_clip_clearance_ladder.stl", "Clip clearance ladder 0.10 / 0.25 / 0.40"),
             ("stl/provisional_cage/PROVISIONAL_left_front_module.stl", "Left front module (print pose)"),
             ("stl/provisional_cage/PROVISIONAL_left_rear_module.stl", "Left rear module (print pose)")]
    fig = plt.figure(figsize=(12, 10), dpi=120)
    for i, (f, t) in enumerate(files):
        m = load(f)
        ax = fig.add_subplot(2, 2, i + 1, projection="3d")
        add_mesh(ax, m, COL["test"] if "fit" in f or "ladder" in f else COL["front" if "front" in f else "rear"])
        e = m.bounds
        c = e.mean(0)
        r = (e[1] - e[0]).max() / 2
        ax.set_xlim(c[0] - r, c[0] + r)
        ax.set_ylim(c[1] - r, c[1] + r)
        ax.set_zlim(0, max(2 * r * 0.4, e[1][2] + 2))
        ax.set_box_aspect((1, 1, 0.4))
        ax.view_init(elev=30, azim=-60)
        ax.set_title(t, fontsize=9)
        ax.tick_params(labelsize=6)
    fig.suptitle("Parts as oriented for printing: bed at Z = 0, upper ring down, clip opening up", fontsize=11)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "print_orientation_sheet.png"))
    plt.close(fig)
    print("renders written to", OUT)


if __name__ == "__main__":
    main()
