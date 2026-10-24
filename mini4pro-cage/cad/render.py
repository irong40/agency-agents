"""
PNG renders of the v0.3 cage with matplotlib.  The drone drawn is a PLACEHOLDER
built from the parameters (body box, motor posts, prop disks, arms), not a DJI model.
"""
import json
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
REF = os.environ.get("REF_IMAGES")  # optional folder with the reference photos 1.png..3.png
COL = {"body": (0.30, 0.32, 0.36), "lid": (0.16, 0.45, 0.85), "clip": (0.1, 0.1, 0.1)}


def shade(m, base, light):
    k = 0.35 + 0.65 * np.clip(m.face_normals @ light, 0, 1)
    return np.clip(np.outer(k, base), 0, 1)


def add_mesh(ax, m, base, light=np.array([0.3, -0.4, 0.86])):
    light = light / np.linalg.norm(light)
    ax.add_collection3d(Poly3DCollection(m.triangles, facecolors=shade(m, base, light),
                                         edgecolors="none", linewidths=0))


def placeholder_drone(ax, p, alpha=0.15):
    x0, x1, y0, y1, z0, z1 = p.body_box
    c = np.array([[x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0],
                  [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]])
    e = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4), (0, 4), (1, 5), (2, 6), (3, 7)]
    ax.add_collection3d(Line3DCollection([[c[a], c[b]] for a, b in e], colors="0.45",
                                         linewidths=0.8, linestyles="--"))
    t = np.linspace(0, 2 * np.pi, 90)
    for mo, lobe in ((p.front_motor, "front"), (p.rear_motor, "rear")):
        for s in (1, -1):
            mx, my, mz = mo[0], s * mo[1], mo[2]
            ax.add_collection3d(Poly3DCollection(
                [list(zip(mx + p.prop_r * np.cos(t), my + p.prop_r * np.sin(t), np.full_like(t, mz)))],
                facecolors=(0.85, 0.3, 0.3, alpha), edgecolors=(0.7, 0.2, 0.2, 0.7), linewidths=0.6))
            ax.plot([mx, mx], [my, my], [mz - 16, mz + 1], color="0.2", lw=2.5)
            a = math.atan2(-my, -mx)
            L = math.hypot(mx, my) - 25
            dz = getattr(p, f"{lobe}_clip_dz")
            ax.plot([mx, mx + L * math.cos(a)], [my, my + L * math.sin(a)], [mz + dz] * 2,
                    color="0.35", lw=3.5)


def frame(ax, lim, elev, azim, zlim=(-60, 60)):
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_zlim(*zlim)
    ax.set_box_aspect((1, 1, (zlim[1] - zlim[0]) / (2 * lim)))
    ax.view_init(elev=elev, azim=azim)
    ax.tick_params(labelsize=6)


def load(rel):
    return trimesh.load(os.path.join(ROOT, rel), force="mesh")


def side_meshes(side):
    names = ["body_front", "body_rear", "lid_front", "lid_rear"]
    return {n: load(f"build/asm_{side}_{n}.stl") for n in names
            if os.path.exists(os.path.join(ROOT, f"build/asm_{side}_{n}.stl"))}


def draw_side(ax, meshes):
    for n, m in meshes.items():
        add_mesh(ax, m, COL["lid" if n.startswith("lid") else "body"])


def main():
    os.makedirs(OUT, exist_ok=True)
    p = Params()
    L, R = side_meshes("left"), side_meshes("right")
    note = "v0.3 PROVISIONAL. Blue = lid (top level), grey = body. Red disks = assumed prop swept area. Drone is a placeholder."

    for name, el, az, title in [
        ("assembly_iso", 28, -135, "Assembled cage on placeholder drone"),
        ("assembly_top", 89.9, -90, "Top view"),
        ("assembly_side_left", 0, -90, "Left side view"),
    ]:
        fig = plt.figure(figsize=(9, 8), dpi=130)
        ax = fig.add_subplot(111, projection="3d")
        placeholder_drone(ax, p)
        draw_side(ax, L)
        draw_side(ax, R)
        frame(ax, 210, el, az)
        ax.set_title(title, fontsize=10)
        fig.text(0.02, 0.02, note, fontsize=7, color="0.3")
        fig.tight_layout()
        fig.savefig(os.path.join(OUT, f"{name}.png"))
        plt.close(fig)

    # left cage alone, from above at an angle (like reference photo 2)
    fig = plt.figure(figsize=(10, 7), dpi=130)
    ax = fig.add_subplot(111, projection="3d")
    draw_side(ax, L)
    ax.set_xlim(-175, 175)
    ax.set_ylim(-65, 285)
    ax.set_zlim(-40, 25)
    ax.set_box_aspect((350, 350, 65))
    ax.view_init(elev=35, azim=-120)
    ax.set_axis_off()
    ax.set_title("Left cage, angled from above (compare with reference photo 2)", fontsize=10)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "left_cage_iso.png"))
    plt.close(fig)

    # bottom view of both cages side by side (like reference photo 3)
    fig = plt.figure(figsize=(9, 8), dpi=130)
    ax = fig.add_subplot(111, projection="3d")
    draw_side(ax, L)
    draw_side(ax, R)
    frame(ax, 210, -89.9, 0)
    ax.set_axis_off()
    ax.set_title("Both cages from below (compare with reference photo 3)", fontsize=10)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "cages_from_below.png"))
    plt.close(fig)

    rep = json.load(open(os.path.join(ROOT, "build", "build_report.json")))
    cc = rep["clips"]["front"]["clip_center"]
    fig = plt.figure(figsize=(8, 7), dpi=130)
    ax = fig.add_subplot(111, projection="3d")
    draw_side(ax, {k: v for k, v in L.items() if k == "body_front"})
    ax.set_xlim(cc[0] - 30, cc[0] + 30)
    ax.set_ylim(cc[1] - 30, cc[1] + 30)
    ax.set_zlim(-30, 10)
    ax.set_box_aspect((60, 60, 40))
    ax.view_init(elev=30, azim=-60)
    ax.set_title("Front hub ring, post and arm clip (opening up, lips snap over the arm)", fontsize=9)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "clip_closeup.png"))
    plt.close(fig)

    M = rep["waist_midpoint"]
    fig = plt.figure(figsize=(8, 7), dpi=130)
    ax = fig.add_subplot(111, projection="3d")
    for k, c in (("body_front", (0.2, 0.5, 0.85)), ("body_rear", (0.95, 0.55, 0.15))):
        add_mesh(ax, L[k], c)
    ax.set_xlim(M[0] - 35, M[0] + 35)
    ax.set_ylim(M[1] + 20, M[1] + 90)
    ax.set_zlim(-30, 5)
    ax.set_box_aspect((70, 70, 35))
    ax.view_init(elev=55, azim=-70)
    ax.set_title("Waist split: dovetail joint on the lower outer rail (front blue, rear orange)", fontsize=9)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "waist_dovetail_closeup.png"))
    plt.close(fig)

    files = [("stl/fit_tests/front_mount_fit_test.stl", "Front mount fit test"),
             ("stl/fit_tests/front_clip_clearance_ladder.stl", "Clip ladder 0.10 / 0.25 / 0.40"),
             ("stl/provisional_cage/PROVISIONAL_left_body_front.stl", "Left body, front half"),
             ("stl/provisional_cage/PROVISIONAL_left_lid_front.stl", "Left lid, front half (top face down)")]
    fig = plt.figure(figsize=(12, 10), dpi=120)
    for i, (f, t) in enumerate(files):
        m = load(f)
        ax = fig.add_subplot(2, 2, i + 1, projection="3d")
        add_mesh(ax, m, COL["lid"] if "lid" in f else COL["body"])
        b = m.bounds
        c = b.mean(0)
        r = (b[1] - b[0])[:2].max() / 2
        ax.set_xlim(c[0] - r, c[0] + r)
        ax.set_ylim(c[1] - r, c[1] + r)
        ax.set_zlim(0, max(r * 0.5, b[1][2] + 2))
        ax.set_box_aspect((1, 1, 0.25))
        ax.view_init(elev=35, azim=-60)
        ax.set_title(t, fontsize=9)
        ax.tick_params(labelsize=6)
    fig.suptitle("Parts as oriented for printing (bed at Z = 0)", fontsize=11)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "print_orientation_sheet.png"))
    plt.close(fig)

    if REF and os.path.isdir(REF):
        import matplotlib.image as mpimg
        fig, axs = plt.subplots(1, 2, figsize=(14, 7), dpi=110)
        axs[0].imshow(mpimg.imread(os.path.join(REF, "3.png")))
        axs[0].set_title("Reference photo (view from below)")
        axs[1].imshow(mpimg.imread(os.path.join(OUT, "cages_from_below.png")))
        axs[1].set_title("v0.3 model from below")
        for a in axs:
            a.axis("off")
        fig.tight_layout()
        fig.savefig(os.path.join(OUT, "compare_photo3_vs_model.png"))
        plt.close(fig)
        fig, axs = plt.subplots(1, 2, figsize=(14, 6), dpi=110)
        axs[0].imshow(mpimg.imread(os.path.join(REF, "2.png")))
        axs[0].set_title("Reference photo (angled)")
        axs[1].imshow(mpimg.imread(os.path.join(OUT, "left_cage_iso.png")))
        axs[1].set_title("v0.3 model, left cage")
        for a in axs:
            a.axis("off")
        fig.tight_layout()
        fig.savefig(os.path.join(OUT, "compare_photo2_vs_model.png"))
        plt.close(fig)
    print("renders written to", OUT)


if __name__ == "__main__":
    main()
