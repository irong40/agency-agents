"""
Mesh and geometry validation for the Mini 4 Pro cage.

What this checks: mesh integrity, volume and weight, clearances against the
PARAMETER values (propeller, motor, body, camera, landing plane), module
interference, build-volume fit and overhang area.

What this does NOT check: real fit on the drone, strength, crash behaviour,
vibration, or flight suitability.  Every clearance below is only as good as the
[MEASURE] parameters it is computed from.
"""
import glob
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

DENSITY = {"PLA": 1.24, "PETG": 1.27}  # g/cm3, typical datasheet values

# Manufacturer build volumes in mm (X, Y, Z).  Recalled from public spec sheets;
# confirm against the library's actual machine.
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
    m = trimesh.load(path, force="mesh")
    return m


def mesh_report(path):
    m = load(path)
    bodies = m.split(only_watertight=False, repair=False)
    vol = abs(m.volume) / 1000.0  # cm3
    return {
        "file": os.path.relpath(path, ROOT),
        "triangles": int(len(m.faces)),
        "watertight": bool(m.is_watertight),
        "winding_consistent": bool(m.is_winding_consistent),
        "bodies": len(bodies),
        "volume_cm3": round(vol, 2),
        "mass_g_PLA": round(vol * DENSITY["PLA"], 1),
        "mass_g_PETG": round(vol * DENSITY["PETG"], 1),
        "bbox_mm": [round(float(x), 1) for x in m.extents],
    }


def min_footprint(m):
    """Smallest XY bounding rectangle over rotations about Z (1 degree steps)."""
    v = m.vertices[:, :2]
    best = None
    for a in range(0, 180):
        t = math.radians(a)
        R = np.array([[math.cos(t), -math.sin(t)], [math.sin(t), math.cos(t)]])
        r = v @ R.T
        ext = r.max(0) - r.min(0)
        key = ext[0] * ext[1]
        if best is None or key < best[0]:
            best = (key, sorted(ext.tolist(), reverse=True), a)
    return best[1], best[2]


def fits(dims_xy, z, printer):
    X, Y, Z = sorted(printer[:2], reverse=True) + [printer[2]]
    return dims_xy[0] <= X - 4 and dims_xy[1] <= Y - 4 and z <= Z


def overhang(m, bed_tol=0.3, limit_deg=45.0):
    n = m.face_normals
    a = m.area_faces
    zc = m.triangles_center[:, 2]
    down = n[:, 2] < -math.cos(math.radians(limit_deg))
    on_bed = zc < m.bounds[0][2] + bed_tol
    over = down & ~on_bed
    return round(float(a[over].sum() / a.sum() * 100), 1)


def clearance_checks(p: Params, left_meshes, right_meshes):
    res = {}
    motors = {"front": p.front_motor, "rear": p.rear_motor}
    V = np.vstack([m.vertices for m in left_meshes])
    for name, mo in motors.items():
        mx, my, mz = mo
        d = np.hypot(V[:, 0] - mx, V[:, 1] - my)
        band = (V[:, 2] > mz - p.prop_vertical_keepout) & (V[:, 2] < mz + p.prop_cone_up + 2.0)
        res[f"{name}_prop_min_radial_clear_mm"] = round(float(d[band].min() - p.prop_r), 2) if band.any() else None
        inside = (d < p.prop_r + 1.0) & band
        res[f"{name}_prop_disk_intrusions"] = int(inside.sum())
        mk = (d < p.motor_keepout_r) & (V[:, 2] > mz - 25) & (V[:, 2] < mz + 8)
        res[f"{name}_motor_keepout_intrusions"] = int(mk.sum())
    x0, x1, y0, y1, z0, z1 = p.body_box
    allV = np.vstack([V] + [m.vertices for m in right_meshes])
    inbox = ((allV[:, 0] > x0) & (allV[:, 0] < x1) & (allV[:, 1] > y0) & (allV[:, 1] < y1)
             & (allV[:, 2] > z0) & (allV[:, 2] < z1))
    res["body_box_intrusions"] = int(inbox.sum())
    dx = np.maximum.reduce([x0 - allV[:, 0], np.zeros(len(allV)), allV[:, 0] - x1])
    dy = np.maximum.reduce([y0 - allV[:, 1], np.zeros(len(allV)), allV[:, 1] - y1])
    dz = np.maximum.reduce([z0 - allV[:, 2], np.zeros(len(allV)), allV[:, 2] - z1])
    res["body_box_min_gap_mm"] = round(float(np.sqrt(dx**2 + dy**2 + dz**2).min()), 1)
    cam = np.array(p.camera_pt)
    res["camera_min_dist_mm"] = round(float(np.linalg.norm(allV - cam, axis=1).min()), 1)
    # forward-looking camera: any structure ahead of the lens within +-45 deg horizontally?
    rel = allV - cam
    ahead = rel[:, 0] > 0
    ang = np.degrees(np.arctan2(np.abs(rel[:, 1]), rel[:, 0]))
    res["structure_in_camera_hfov45_vertices"] = int((ahead & (ang < 45)).sum())
    res["lowest_point_z"] = round(float(allV[:, 2].min()), 1)
    res["landing_plane_margin_mm"] = round(float(allV[:, 2].min() - p.landing_z), 1)
    gap_lr = None
    lv = V[V[:, 1] > 0]
    rv = np.vstack([m.vertices for m in right_meshes])
    if len(lv) and len(rv):
        gap_lr = float(lv[:, 1].min() - rv[:, 1].max())
    res["left_right_min_y_gap_mm"] = round(gap_lr, 1) if gap_lr is not None else None
    return res


def interference(a, b):
    try:
        i = trimesh.boolean.intersection([a, b], engine="manifold")
        return round(abs(i.volume), 3)
    except Exception as e:  # pragma: no cover
        return f"error: {e}"


def main():
    p = Params()
    out = {"meshes": [], "print_fit": [], "notes": []}
    files = sorted(glob.glob(os.path.join(ROOT, "stl", "**", "*.stl"), recursive=True))
    for f in files:
        r = mesh_report(f)
        m = load(f)
        xy, rot = min_footprint(m)
        r["print_pose_footprint_mm"] = [round(x, 1) for x in xy]
        r["print_pose_height_mm"] = round(float(m.extents[2]), 1)
        r["rotate_on_bed_deg"] = rot
        r["overhang_area_pct_over45"] = overhang(m)
        r["fits"] = [k for k, v in PRINTERS.items() if fits(xy, m.extents[2], v)]
        out["meshes"].append(r)

    b = os.path.join(ROOT, "build")
    lf, lr = load(f"{b}/asm_left_front.stl"), load(f"{b}/asm_left_rear.stl")
    rf, rr = load(f"{b}/asm_right_front.stl"), load(f"{b}/asm_right_rear.stl")
    out["assembled_interference_cm3"] = {
        "left_front_vs_left_rear_mm3": interference(lf, lr),
        "left_front_vs_right_front_mm3": interference(lf, rf),
        "left_rear_vs_right_rear_mm3": interference(lr, rr),
    }
    out["clearances_vs_parameters"] = clearance_checks(p, [lf, lr], [rf, rr])
    total = sum(abs(m.volume) for m in (lf, lr, rf, rr)) / 1000.0
    out["full_cage_both_sides"] = {
        "volume_cm3": round(total, 1),
        "mass_g_PLA": round(total * DENSITY["PLA"], 1),
        "mass_g_PETG": round(total * DENSITY["PETG"], 1),
        "plus_hardware_g": "about 2 g for 4x M2x10 screws + nuts (estimate)",
    }
    with open(os.path.join(b, "validation_report.json"), "w") as f:
        json.dump(out, f, indent=2)
    print(json.dumps(out, indent=2))


if __name__ == "__main__":
    main()
