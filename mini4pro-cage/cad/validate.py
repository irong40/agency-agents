"""
Mesh and geometry validation for the v0.3 cage.

Checks: mesh integrity, volume and weight, clearances against the PARAMETER values
(propeller, motor, arm, body, camera, landing plane), part interference in the
assembled pose, build-volume fit and overhang area.

Does NOT check: real fit on the drone, strength, crash behaviour, vibration,
compass effects or flight suitability.  Clearances are only as good as the
[ASSUMED] / [DERIVED] parameters.
"""
import glob
import itertools
import json
import math
import os
import sys

import numpy as np
import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
sys.path.insert(0, HERE)
from mini4pro_cage import Params  # noqa: E402

DENSITY = {"PLA": 1.24, "PETG": 1.27}
# Manufacturer build volumes (mm), recalled from public spec sheets. Confirm on the real machine.
PRINTERS = {
    "Prusa MINI+": (180, 180, 180),
    "Bambu Lab A1 mini": (180, 180, 180),
    "Prusa MK4/MK4S": (250, 210, 220),
    "Bambu Lab A1 / P1S / X1C": (256, 256, 256),
    "Creality Ender-3 (V2)": (220, 220, 250),
    "Ultimaker S3": (230, 190, 200),
    "MakerBot Replicator+": (295, 195, 165),
}


def load(path):
    return trimesh.load(path, force="mesh")


def min_footprint(m):
    v = m.vertices[:, :2]
    best = None
    for a in range(0, 180):
        t = math.radians(a)
        R = np.array([[math.cos(t), -math.sin(t)], [math.sin(t), math.cos(t)]])
        r = v @ R.T
        e = r.max(0) - r.min(0)
        if best is None or e[0] * e[1] < best[0]:
            best = (e[0] * e[1], sorted(e.tolist(), reverse=True), a)
    return best[1], best[2]


def fits_any_rotation(m, pr, margin=4.0):
    """True if some rotation about Z puts the part inside the bed with `margin` all round."""
    if m.extents[2] > pr[2]:
        return False
    v = m.vertices[:, :2]
    X, Y = pr[0] - 2 * margin, pr[1] - 2 * margin
    for a in np.arange(0, 180, 0.5):
        t = math.radians(a)
        r = v @ np.array([[math.cos(t), -math.sin(t)], [math.sin(t), math.cos(t)]]).T
        e = r.max(0) - r.min(0)
        if (e[0] <= X and e[1] <= Y) or (e[0] <= Y and e[1] <= X):
            return True
    return False


def overhang_pct(m, limit=45.0, bed_tol=0.3):
    n, a = m.face_normals, m.area_faces
    down = n[:, 2] < -math.cos(math.radians(limit))
    on_bed = m.triangles_center[:, 2] < m.bounds[0][2] + bed_tol
    return round(float(a[down & ~on_bed].sum() / a.sum() * 100), 1)


def mesh_report(path):
    m = load(path)
    vol = abs(m.volume) / 1000.0
    xy, rot = min_footprint(m)
    return {
        "file": os.path.relpath(path, ROOT),
        "watertight": bool(m.is_watertight),
        "winding_consistent": bool(m.is_winding_consistent),
        "bodies": len(m.split(only_watertight=False, repair=False)),
        "triangles": int(len(m.faces)),
        "volume_cm3": round(vol, 2),
        "mass_g_PLA": round(vol * DENSITY["PLA"], 1),
        "mass_g_PETG": round(vol * DENSITY["PETG"], 1),
        "footprint_mm": [round(x, 1) for x in xy],
        "height_mm": round(float(m.extents[2]), 1),
        "rotate_on_bed_deg": rot,
        "overhang_pct_over45": overhang_pct(m),
        "fits": [k for k, v in PRINTERS.items() if fits_any_rotation(m, v)],
    }


def interference(a, b):
    try:
        return round(abs(trimesh.boolean.intersection([a, b], engine="manifold").volume), 3)
    except Exception as e:  # pragma: no cover
        return f"error: {e}"


def clearances(p: Params, left, right):
    res = {}
    V = np.vstack([m.vertices for m in left.values()])
    for name, mo in (("front", p.front_motor), ("rear", p.rear_motor)):
        mx, my, mz = mo
        d = np.hypot(V[:, 0] - mx, V[:, 1] - my)
        band = (V[:, 2] > mz - p.prop_vertical_keepout) & (V[:, 2] < mz + p.prop_cone_up + 2.0)
        res[f"{name}_prop_min_radial_clear_mm"] = round(float(d[band].min() - p.prop_r), 2)
        res[f"{name}_prop_disk_intrusions"] = int(((d < p.prop_r + 1.0) & band).sum())
        over = (d < p.prop_r + 1.0) & (V[:, 2] > mz)
        res[f"{name}_top_level_min_height_above_prop_mm"] = round(float(V[over, 2].min() - mz), 2) if over.any() else None
        under = (d < p.prop_r + 1.0) & (V[:, 2] < mz)
        res[f"{name}_bottom_level_max_height_below_prop_mm"] = round(float(mz - V[under, 2].max()), 2) if under.any() else None
        mk = (d < p.motor_keepout_r) & (V[:, 2] > mz - 25) & (V[:, 2] < mz + 8)
        res[f"{name}_motor_keepout_intrusions"] = int(mk.sum())
    x0, x1, y0, y1, z0, z1 = p.body_box
    A = np.vstack([V] + [m.vertices for m in right.values()])
    inbox = (A[:, 0] > x0) & (A[:, 0] < x1) & (A[:, 1] > y0) & (A[:, 1] < y1) & (A[:, 2] > z0) & (A[:, 2] < z1)
    res["body_box_intrusions"] = int(inbox.sum())
    dx = np.maximum.reduce([x0 - A[:, 0], np.zeros(len(A)), A[:, 0] - x1])
    dy = np.maximum.reduce([y0 - A[:, 1], np.zeros(len(A)), A[:, 1] - y1])
    dz = np.maximum.reduce([z0 - A[:, 2], np.zeros(len(A)), A[:, 2] - z1])
    res["body_box_min_gap_mm"] = round(float(np.sqrt(dx ** 2 + dy ** 2 + dz ** 2).min()), 1)
    cam = np.array(p.camera_pt)
    rel = A - cam
    res["camera_min_dist_mm"] = round(float(np.linalg.norm(rel, axis=1).min()), 1)
    ahead = rel[:, 0] > 0
    res["structure_ahead_of_camera_within_45deg"] = int(
        (ahead & (np.degrees(np.arctan2(np.abs(rel[:, 1]), rel[:, 0])) < 45)).sum())
    res["lowest_point_z"] = round(float(A[:, 2].min()), 1)
    res["landing_plane_margin_mm"] = round(float(A[:, 2].min() - p.landing_z), 1)
    lv = V
    rv = np.vstack([m.vertices for m in right.values()])
    res["left_right_min_y_gap_mm"] = round(float(lv[:, 1].min() - rv[:, 1].max()), 1)
    return res


def main():
    p = Params()
    out = {"meshes": []}
    for f in sorted(glob.glob(os.path.join(ROOT, "stl", "**", "*.stl"), recursive=True)):
        out["meshes"].append(mesh_report(f))
    b = os.path.join(ROOT, "build")
    names = ["body_front", "body_rear", "lid_front", "lid_rear"]
    L = {n: load(f"{b}/asm_left_{n}.stl") for n in names}
    R = {n: load(f"{b}/asm_right_{n}.stl") for n in names}
    out["assembled_interference_mm3"] = {f"{x} vs {y}": interference(L[x], L[y])
                                         for x, y in itertools.combinations(names, 2)}
    out["assembled_interference_mm3"]["left vs right (all)"] = interference(
        trimesh.util.concatenate(list(L.values())), trimesh.util.concatenate(list(R.values())))
    out["clearances_vs_parameters"] = clearances(p, L, R)
    tot = sum(abs(m.volume) for m in list(L.values()) + list(R.values())) / 1000.0
    out["full_cage_both_sides"] = {"volume_cm3": round(tot, 1),
                                   "mass_g_PLA": round(tot * DENSITY["PLA"], 1),
                                   "mass_g_PETG": round(tot * DENSITY["PETG"], 1),
                                   "hardware": "none required; CA glue on pegs and dovetails recommended"}
    rep = json.load(open(os.path.join(b, "build_report.json")))
    out["keepout_trim_mm3"] = {"prop": rep.get("prop_keepout_trim_mm3"), "arm": rep.get("arm_keepout_trim_mm3")}
    with open(os.path.join(b, "validation_report.json"), "w") as f:
        json.dump(out, f, indent=2)
    print(json.dumps(out, indent=2))


if __name__ == "__main__":
    main()
