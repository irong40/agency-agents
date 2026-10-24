"""
Parametric connected propeller cage for the DJI Mini 4 Pro.  Version 0.2 (PROVISIONAL)

Units: millimetres.  Body frame: origin at the airframe centre, +X forward,
+Y to the drone's left, +Z up.  The model is built for the LEFT side and
mirrored for the right side.

Every dimension below carries a source tag:
  [MEASURE]   placeholder.  Must be measured on the real drone before a final print.
  [DERIVED]   computed from a published figure.  See docs/DIMENSIONS.md.
  [DESIGN]    a design choice.  Safe to tune.

No third-party geometry is imported.  The Printables model 1411135 could not be
downloaded from this build environment, so the arm clips here are original,
generic C-clips sized from placeholder arm dimensions.

Run:  python mini4pro_cage.py            (builds everything, writes STLs + report)
      python mini4pro_cage.py --json p.json   (override parameters from a JSON file)
"""
from __future__ import annotations

import argparse
import json
import math
import os
from dataclasses import asdict, dataclass, field, replace

import cadquery as cq
from OCP.BRepTools import BRepTools


# ----------------------------------------------------------------------------
# Parameters
# ----------------------------------------------------------------------------
@dataclass
class Params:
    # --- Propeller -----------------------------------------------------------
    prop_diameter: float = 152.4        # [MEASURE] tip-to-tip. 6.0 in class assumed.
    radial_clearance: float = 6.0       # [DESIGN] prop tip to ring inner face, at rest
    prop_vertical_keepout: float = 5.0  # [DESIGN] no structure within this of the prop plane under the disk
    prop_cone_up: float = 4.0           # [MEASURE] blade tip rise under thrust (estimate)

    # --- Motor shaft positions, LEFT side, z = prop hub plane ------------------
    front_motor: tuple = (73.0, 110.5, 0.0)   # [DERIVED+MEASURE] see DIMENSIONS.md
    rear_motor: tuple = (-73.0, 105.0, -10.0)  # [DERIVED+MEASURE]
    motor_keepout_r: float = 11.0       # [MEASURE] motor bell radius + 1.5 mm margin

    # --- Arms and clips ---------------------------------------------------------
    # Arm direction in plan, degrees, measured from +X.  None = point at airframe centre.
    front_arm_angle: float | None = None  # [MEASURE]
    rear_arm_angle: float | None = None   # [MEASURE]
    front_clip_dist: float = 22.0       # [MEASURE] motor axis to clip centre along the arm
    rear_clip_dist: float = 22.0        # [MEASURE]
    front_clip_dz: float = -12.0        # [MEASURE] arm centre height relative to prop plane
    rear_clip_dz: float = -12.0         # [MEASURE]
    front_arm_w: float = 12.0           # [MEASURE] arm width at clip location
    front_arm_h: float = 8.0            # [MEASURE] arm height at clip location
    rear_arm_w: float = 12.0            # [MEASURE]
    rear_arm_h: float = 8.0             # [MEASURE]
    clip_len: float = 14.0              # [DESIGN] clip length along the arm
    clip_wall: float = 1.8              # [DESIGN]
    clip_clear: float = 0.25            # [DESIGN] tuned with the clearance ladder
    snap_lip: float = 0.8               # [DESIGN] lip overlap under the arm, each side
    zip_groove_w: float = 3.0           # [DESIGN] backup zip-tie groove width (0 disables)
    zip_groove_d: float = 0.6           # [DESIGN]

    # --- Cage ---------------------------------------------------------------------
    ring_w: float = 2.6                 # [DESIGN] radial width of rings and rails
    ring_h: float = 3.6                 # [DESIGN] vertical height of rings and rails
    upper_dz: float = 5.0               # [DESIGN] upper ring centre above prop plane
    lower_dz: float = -16.0             # [DESIGN] lower rail centre below prop plane
    rib_w: float = 2.4                  # [DESIGN] tangential width of vertical ribs
    rib_spacing: float = 28.0           # [DESIGN] max arc length between ribs
    front_body_gap: float = 100.0       # [DESIGN] open sector facing the airframe, degrees
    rear_body_gap: float = 100.0        # [DESIGN]
    strut_d: float = 3.2                # [DESIGN] clip-to-rail strut diameter
    strut_spread: float = 62.0          # [DESIGN] strut angle either side of the arm, degrees

    # --- Bridge joint between front and rear modules ------------------------------
    joint_len: float = 16.0             # [DESIGN] along the front-rear direction
    joint_w: float = 9.0                # [DESIGN] total width, split into two lap halves
    joint_clear: float = 0.15           # [DESIGN] gap between lap halves
    bolt_d: float = 2.2                 # [DESIGN] M2 clearance, or 1.75 filament pin

    # --- Keep-out checks only (not built) -----------------------------------------
    body_box: tuple = (-75.0, 75.0, -33.0, 33.0, -45.0, 15.0)  # [MEASURE] xmin..zmax
    camera_pt: tuple = (80.0, 0.0, -25.0)  # [MEASURE] gimbal lens centre
    landing_z: float = -48.0            # [MEASURE] table plane relative to front prop plane

    # --- Tessellation -----------------------------------------------------------
    stl_tol: float = 0.05
    stl_ang: float = 0.2

    @property
    def prop_r(self) -> float:
        return self.prop_diameter / 2.0

    @property
    def ring_rc(self) -> float:
        """Ring centreline radius."""
        return self.prop_r + self.radial_clearance + self.ring_w / 2.0


# ----------------------------------------------------------------------------
# Small geometry helpers
# ----------------------------------------------------------------------------
def _norm(a: float) -> float:
    return a % 360.0


def _pol(c, r, a_deg, z=None):
    a = math.radians(a_deg)
    p = (c[0] + r * math.cos(a), c[1] + r * math.sin(a))
    return p if z is None else (p[0], p[1], z)


def _dist2(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _angle_to(src, dst):
    return math.degrees(math.atan2(dst[1] - src[1], dst[0] - src[0]))


def _ang_diff(a, b):
    """Smallest signed difference a-b in degrees."""
    return (a - b + 180.0) % 360.0 - 180.0


def ring_arc(p: Params, motor, other, body_gap):
    """Largest contiguous arc of the ring that (a) leaves an open sector facing the
    airframe centre and (b) stays outside the other propeller's swept disk.
    Returns (a0, a1) in degrees with a1 > a0 (unwrapped)."""
    rc = p.ring_rc
    excl_other = p.prop_r + p.radial_clearance + p.ring_w / 2.0
    body_dir = _angle_to(motor, (0.0, 0.0))
    step = 0.25
    n = int(360 / step)
    ok = []
    for i in range(n):
        a = i * step
        pt = _pol(motor, rc, a)
        in_gap = abs(_ang_diff(a, body_dir)) < body_gap / 2.0
        in_other = _dist2(pt, other) < excl_other
        ok.append(not in_gap and not in_other)
    if all(ok):
        return 0.0, 360.0
    # rotate so index 0 is excluded, then find the longest run of True
    start = ok.index(False)
    best, cur, best_s, cur_s = 0, 0, 0, 0
    for k in range(1, n + 1):
        idx = (start + k) % n
        if ok[idx]:
            if cur == 0:
                cur_s = idx
            cur += 1
            if cur > best:
                best, best_s = cur, cur_s
        else:
            cur = 0
    a0 = best_s * step
    a1 = a0 + (best - 1) * step
    return a0, a1


def clamp_to_arc(a, a0, a1):
    a = a0 + _norm(a - a0)
    if a <= a1:
        return a
    d0 = abs(_ang_diff(a, a0))
    d1 = abs(_ang_diff(a, a1))
    return a0 if d0 < d1 else a1


def sector(center, r_in, r_out, a0, a1, zc, h):
    am = (a0 + a1) / 2.0
    wp = (
        cq.Workplane("XY", origin=(0, 0, zc - h / 2.0))
        .moveTo(*_pol(center, r_out, a0))
        .threePointArc(_pol(center, r_out, am), _pol(center, r_out, a1))
        .lineTo(*_pol(center, r_in, a1))
        .threePointArc(_pol(center, r_in, am), _pol(center, r_in, a0))
        .close()
        .extrude(h)
    )
    return wp.val()


def rod(p0, p1, d, extend=1.0):
    v = cq.Vector(*p1) - cq.Vector(*p0)
    L = v.Length
    u = v.normalized()
    start = cq.Vector(*p0) - u * extend
    return cq.Solid.makeCylinder(d / 2.0, L + 2 * extend, start, u)


def oriented_box(center, u, w, lu, lw, lz):
    """Box centred at `center`, length lu along plan unit u, lw along plan unit w."""
    b = cq.Solid.makeBox(lu, lw, lz, cq.Vector(-lu / 2, -lw / 2, -lz / 2))
    ang = math.degrees(math.atan2(u[1], u[0]))
    b = b.rotate(cq.Vector(0, 0, 0), cq.Vector(0, 0, 1), ang)
    # makeBox local +Y maps to w only if w is +90 deg from u; flip handled by symmetry
    return b.translate(cq.Vector(*center))


def fuse_all(solids):
    out = solids[0]
    for s in solids[1:]:
        out = out.fuse(s)
    return out.clean()


# ----------------------------------------------------------------------------
# Components
# ----------------------------------------------------------------------------
def clip_solid(p: Params, arm_w, arm_h, clear=None, notches=0):
    """C-clip in local frame: arm axis along X, opening on -Z (underside of arm)."""
    c = p.clip_clear if clear is None else clear
    t = p.clip_wall
    L = p.clip_len
    cav_w, cav_h = arm_w + 2 * c, arm_h + 2 * c
    ow, oh = cav_w + 2 * t, cav_h + 2 * t
    body = cq.Workplane("XY").box(L, ow, oh)
    body = body.edges("|X").fillet(min(1.2, t * 0.7))
    body = body.cut(cq.Workplane("XY").box(L + 2, cav_w, cav_h))
    open_w = max(arm_w - 2 * p.snap_lip, 2.0)
    body = body.cut(
        cq.Workplane("XY").box(L + 2, open_w, t + 2).translate((0, 0, -(cav_h / 2 + t / 2)))
    )
    # lead-in chamfers on the snap lips
    for s in (-1, 1):
        wedge = (
            cq.Workplane("YZ")
            .polyline([(s * open_w / 2, -oh / 2 - 0.01),
                       (s * (open_w / 2 + p.snap_lip * 0.9), -oh / 2 - 0.01),
                       (s * open_w / 2, -cav_h / 2 + 0.01)])
            .close()
            .extrude(L + 2)
            .translate((-(L + 2) / 2, 0, 0))
        )
        body = body.cut(wedge)
    if p.zip_groove_w > 0:
        gd = p.zip_groove_d
        frame = (
            cq.Workplane("XY").box(p.zip_groove_w, ow + 2, oh + 2)
            .cut(cq.Workplane("XY").box(p.zip_groove_w + 2, ow - 2 * gd, oh - 2 * gd))
        )
        body = body.cut(frame)
    for i in range(notches):
        x = -L / 2 + 2.5 + i * 2.2
        body = body.cut(cq.Workplane("XY").box(1.0, ow + 2, 1.0).translate((x, 0, oh / 2)))
    return body.val(), ow, oh


def place_clip(solid, center, ang_deg):
    return solid.rotate(cq.Vector(0, 0, 0), cq.Vector(0, 0, 1), ang_deg).translate(cq.Vector(*center))


def rib_solid(p: Params, motor, a_deg, H, zmid):
    """Vertical rib.  Starts 0.4 mm inside the ring wall and stands 0.6 mm proud of
    the outer face.  Faces flush or tangent with the ring cylinders break STL meshing."""
    r0 = p.ring_rc - p.ring_w / 2.0 + 0.4
    r1 = p.ring_rc + p.ring_w / 2.0 + 0.6
    rib = cq.Solid.makeBox(r1 - r0, p.rib_w, H, cq.Vector(r0, -p.rib_w / 2, -H / 2))
    rib = rib.rotate(cq.Vector(0, 0, 0), cq.Vector(0, 0, 1), a_deg)
    return rib.translate(cq.Vector(motor[0], motor[1], zmid))


@dataclass
class CageModule:
    name: str
    motor: tuple
    arc: tuple
    solids: list = field(default_factory=list)
    z_range: tuple = (0.0, 0.0)
    struts: list = field(default_factory=list)  # (p0, p1) for checks
    clip_center: tuple = (0, 0, 0)
    arm_angle: float = 0.0
    rib_angles: list = field(default_factory=list)


def build_module(p: Params, which: str, include_joint_side=True) -> CageModule:
    fm, rm = p.front_motor, p.rear_motor
    if which == "front":
        motor, other, gap = fm, rm, p.front_body_gap
        arm_ang, dist, dz = p.front_arm_angle, p.front_clip_dist, p.front_clip_dz
        aw, ah = p.front_arm_w, p.front_arm_h
    else:
        motor, other, gap = rm, fm, p.rear_body_gap
        arm_ang, dist, dz = p.rear_arm_angle, p.rear_clip_dist, p.rear_clip_dz
        aw, ah = p.rear_arm_w, p.rear_arm_h

    a0, a1 = ring_arc(p, motor, other, gap)
    rc, rw, rh = p.ring_rc, p.ring_w, p.ring_h
    zu, zl = motor[2] + p.upper_dz, motor[2] + p.lower_dz
    mod = CageModule(which, motor, (a0, a1))
    mod.z_range = (zl - rh / 2, zu + rh / 2)

    # upper ring and lower rail
    mod.solids.append(sector(motor, rc - rw / 2, rc + rw / 2, a0, a1, zu, rh))
    mod.solids.append(sector(motor, rc - rw / 2, rc + rw / 2, a0, a1, zl, rh))

    # clip
    if arm_ang is None:
        arm_ang = _angle_to(motor, (0.0, 0.0))
    mod.arm_angle = arm_ang
    cc = _pol(motor, dist, arm_ang, motor[2] + dz)
    mod.clip_center = cc
    clip, ow, oh = clip_solid(p, aw, ah)
    mod.solids.append(place_clip(clip, cc, arm_ang))

    # struts from both clip side walls to the lower rail
    rib_angles = []
    for s in (+1, -1):
        tgt = clamp_to_arc(arm_ang + s * p.strut_spread, a0, a1)
        rib_angles.append(tgt)
        perp = math.radians(arm_ang + s * 90.0)
        st = (cc[0] + (ow / 2 - 0.6) * math.cos(perp),
              cc[1] + (ow / 2 - 0.6) * math.sin(perp), cc[2])
        en = _pol(motor, rc, tgt, zl)
        mod.solids.append(rod(st, en, p.strut_d))
        mod.struts.append((st, en))

    # vertical ribs
    arc_len = math.radians(a1 - a0) * rc
    n = max(2, math.ceil(arc_len / p.rib_spacing) + 1)
    base = [a0 + (a1 - a0) * i / (n - 1) for i in range(n)]
    for t in rib_angles:
        if all(abs(t - b) > 4.0 for b in base):
            base.append(t)
    mod.rib_angles = sorted(base)
    H = (zu + rh / 2) - (zl - rh / 2)
    zmid = ((zu + rh / 2) + (zl - rh / 2)) / 2
    for a in mod.rib_angles:
        mod.solids.append(rib_solid(p, motor, a, H, zmid))
    return mod


def joint_geometry(p: Params, fmod: CageModule, rmod: CageModule):
    """Find the outboard meeting point of the two arcs and the lap-joint frame."""
    fm, rm = p.front_motor, p.rear_motor
    rc = p.ring_rc
    f_ends = [_pol(fm, rc, fmod.arc[0]), _pol(fm, rc, fmod.arc[1])]
    r_ends = [_pol(rm, rc, rmod.arc[0]), _pol(rm, rc, rmod.arc[1])]
    mid_m = ((fm[0] + rm[0]) / 2, (fm[1] + rm[1]) / 2)
    best = None
    for fe in f_ends:
        for re in r_ends:
            d = _dist2(fe, re)
            # prefer the pair farther from the airframe centre (outboard)
            score = d - 0.01 * math.hypot((fe[0] + re[0]) / 2, (fe[1] + re[1]) / 2)
            if best is None or score < best[0]:
                best = (score, fe, re)
    _, fe, re = best
    M = ((fe[0] + re[0]) / 2, (fe[1] + re[1]) / 2)
    u = (rm[0] - fm[0], rm[1] - fm[1])
    L = math.hypot(*u)
    u = (u[0] / L, u[1] / L)
    w = (-u[1], u[0])
    if (M[0] - mid_m[0]) * w[0] + (M[1] - mid_m[1]) * w[1] < 0:
        w = (-w[0], -w[1])
    return fe, re, M, u, w


def lap_half(p, M, u, w, side, z_range, clear=0.0):
    """side=-1: inboard half (front module).  side=+1: outboard half (rear module)."""
    hw = p.joint_w / 2.0
    shift = p.joint_w / 2.0 - p.ring_w / 2.0   # whole block sits outboard of the ring inner face
    off = shift + side * hw / 2
    c = (M[0] + w[0] * off, M[1] + w[1] * off, (z_range[0] + z_range[1]) / 2)
    lz = z_range[1] - z_range[0]
    b = cq.Solid.makeBox(p.joint_len + 2 * clear, hw + 2 * clear, lz + 2 * clear,
                         cq.Vector(-(p.joint_len / 2 + clear), -(hw / 2 + clear), -(lz / 2 + clear)))
    ang = math.degrees(math.atan2(w[1], w[0])) - 90.0  # local +Y -> w
    b = b.rotate(cq.Vector(0, 0, 0), cq.Vector(0, 0, 1), ang)
    return b.translate(cq.Vector(*c))


def prop_keepouts(p: Params):
    """Cylinders the propellers (plus radial clearance) sweep.  Subtracted from every
    printed part so no parameter combination can put structure in the prop path."""
    out = []
    for mo in (p.front_motor, p.rear_motor):
        z0 = mo[2] - p.prop_vertical_keepout
        h = p.prop_vertical_keepout + p.prop_cone_up + 2.0
        r = p.prop_r + p.radial_clearance - 0.02
        out.append(cq.Solid.makeCylinder(r, h, cq.Vector(mo[0], mo[1], z0)))
    return out


def build_side(p: Params):
    """Return dict with front/rear module solids for the LEFT side plus a report."""
    fmod = build_module(p, "front")
    rmod = build_module(p, "rear")
    fe, re, M, u, w = joint_geometry(p, fmod, rmod)
    rep = {"joint_point": M, "front_arc_deg": fmod.arc, "rear_arc_deg": rmod.arc,
           "front_arm_angle": fmod.arm_angle, "rear_arm_angle": rmod.arm_angle,
           "end_gap_mm": _dist2(fe, re)}

    # bridge beams when the arcs do not meet
    for mod, e in ((fmod, fe), (rmod, re)):
        if _dist2(e, M) > 0.5:
            for z in (mod.motor[2] + p.upper_dz, mod.motor[2] + p.lower_dz):
                mod.solids.append(rod((e[0], e[1], z), (M[0], M[1], z), p.ring_h, extend=0.5))

    f_half = lap_half(p, M, u, w, -1, fmod.z_range)
    r_half = lap_half(p, M, u, w, +1, rmod.z_range)
    f_env = lap_half(p, M, u, w, -1, fmod.z_range, clear=p.joint_clear)
    r_env = lap_half(p, M, u, w, +1, rmod.z_range, clear=p.joint_clear)

    zo = (max(fmod.z_range[0], rmod.z_range[0]), min(fmod.z_range[1], rmod.z_range[1]))
    rep["lap_overlap_z"] = zo
    rep["lap_overlap_h"] = zo[1] - zo[0]
    holes = []
    if zo[1] - zo[0] >= p.bolt_d + 3.0:
        zb = (zo[0] + zo[1]) / 2
        for s in (-1, 1):
            sh = p.joint_w / 2.0 - p.ring_w / 2.0
            c = (M[0] + u[0] * s * p.joint_len / 4 + w[0] * sh,
                 M[1] + u[1] * s * p.joint_len / 4 + w[1] * sh, zb)
            p0 = (c[0] - w[0] * p.joint_w, c[1] - w[1] * p.joint_w, zb)
            p1 = (c[0] + w[0] * p.joint_w, c[1] + w[1] * p.joint_w, zb)
            holes.append(rod(p0, p1, p.bolt_d, extend=0))
    rep["bolt_holes"] = len(holes)

    front = fuse_all(fmod.solids + [f_half]).cut(r_env)
    rear = fuse_all(rmod.solids + [r_half]).cut(f_env)
    for h in holes:
        front = front.cut(h)
        rear = rear.cut(h)
    for k in prop_keepouts(p):
        front = front.cut(k)
        rear = rear.cut(k)
    return {"front": front.clean(), "rear": rear.clean(), "fmod": fmod, "rmod": rmod,
            "report": rep}


def build_fit_test(p: Params, which: str):
    """Clip + both struts + short ring/rail segments with ribs at the strut ends.
    Checks clip fit AND ring radius at two points around the propeller."""
    mod = build_module(p, which)
    motor, rc, rw, rh = mod.motor, p.ring_rc, p.ring_w, p.ring_h
    zu, zl = motor[2] + p.upper_dz, motor[2] + p.lower_dz
    parts = []
    clip, ow, oh = clip_solid(p, *( (p.front_arm_w, p.front_arm_h) if which == "front"
                                    else (p.rear_arm_w, p.rear_arm_h)))
    parts.append(place_clip(clip, mod.clip_center, mod.arm_angle))
    H = (zu + rh / 2) - (zl - rh / 2)
    zmid = ((zu + rh / 2) + (zl - rh / 2)) / 2
    for (st, en) in mod.struts:
        parts.append(rod(st, en, p.strut_d))
        a = _angle_to(motor, en)
        a = clamp_to_arc(a, *mod.arc)
        b0 = max(mod.arc[0], a - 9)
        b1 = min(mod.arc[1], a + 9)
        if b1 - b0 < 4:
            b0, b1 = a - 4, a + 4
        parts.append(sector(motor, rc - rw / 2, rc + rw / 2, b0, b1, zu, rh))
        parts.append(sector(motor, rc - rw / 2, rc + rw / 2, b0, b1, zl, rh))
        for ra in (b0, b1, a):
            parts.append(rib_solid(p, motor, ra, H, zmid))
    out = fuse_all(parts)
    for k in prop_keepouts(p):
        out = out.cut(k)
    return out.clean()


def build_clip_ladder(p: Params, which: str, clears=(0.10, 0.25, 0.40)):
    aw, ah = (p.front_arm_w, p.front_arm_h) if which == "front" else (p.rear_arm_w, p.rear_arm_h)
    out = []
    for i, c in enumerate(clears):
        s, ow, oh = clip_solid(p, aw, ah, clear=c, notches=i + 1)
        out.append(s.translate(cq.Vector(0, i * (ow + 6), 0)))
    return cq.Compound.makeCompound(out)


def mirror_y(shape):
    return shape.mirror("XZ")


def to_print_pose(shape):
    """Flip upside down so the upper ring rests on the bed, opening of the clip faces up."""
    s = shape.rotate(cq.Vector(0, 0, 0), cq.Vector(1, 0, 0), 180)
    bb = s.BoundingBox()
    return s.translate(cq.Vector(-bb.center.x, -bb.center.y, -bb.zmin))


def clip_print_pose(shape):
    """Clips print on their side: arm axis vertical is weak, so lay the clip with
    the arm axis horizontal and the opening facing up."""
    s = shape.rotate(cq.Vector(0, 0, 0), cq.Vector(1, 0, 0), 180)
    bb = s.BoundingBox()
    return s.translate(cq.Vector(-bb.center.x, -bb.center.y, -bb.zmin))


def _faces_mesh_ok(shape, tol, ang):
    ok = all(len(f.tessellate(tol, ang)[1]) > 0 for f in shape.Faces())
    # per-face tessellation caches triangulations on the faces; clear them so the
    # STL writer meshes the whole solid at once with shared edge points
    BRepTools.Clean_s(shape.wrapped)
    return ok


def export(shape, path, p: Params):
    """Export STL only if every B-rep face tessellates; otherwise retry finer, then fail."""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    for tol, ang in ((p.stl_tol, p.stl_ang), (0.02, 0.1), (0.01, 0.05), (0.1, 0.3)):
        if _faces_mesh_ok(shape, tol, ang):
            cq.exporters.export(shape, path, tolerance=tol, angularTolerance=ang)
            return tol
    fixed = shape.fix()
    if _faces_mesh_ok(fixed, p.stl_tol, p.stl_ang):
        cq.exporters.export(fixed, path, tolerance=p.stl_tol, angularTolerance=p.stl_ang)
        return p.stl_tol
    raise RuntimeError(f"tessellation failed for {path}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--json", help="parameter overrides")
    ap.add_argument("--out", default=os.path.join(os.path.dirname(__file__), ".."))
    args = ap.parse_args()
    p = Params()
    if args.json:
        with open(args.json) as f:
            p = replace(p, **{k: (tuple(v) if isinstance(v, list) else v)
                              for k, v in json.load(f).items() if not k.startswith("_")})
    out = os.path.abspath(args.out)
    stl = os.path.join(out, "stl")

    # fit tests (left side; the clip and ring are symmetric about the arm, so a
    # left fit test also checks the right side's mirror image)
    for which in ("front", "rear"):
        export(to_print_pose(build_fit_test(p, which)),
               os.path.join(stl, "fit_tests", f"{which}_mount_fit_test.stl"), p)
        export(clip_print_pose(build_clip_ladder(p, which)),
               os.path.join(stl, "fit_tests", f"{which}_clip_clearance_ladder.stl"), p)

    side = build_side(p)
    asm = os.path.join(stl, "provisional_cage")
    for name in ("front", "rear"):
        left = side[name]
        right = mirror_y(left)
        export(to_print_pose(left), os.path.join(asm, f"PROVISIONAL_left_{name}_module.stl"), p)
        export(to_print_pose(right), os.path.join(asm, f"PROVISIONAL_right_{name}_module.stl"), p)
        # assembled pose (body frame) for checks and renders
        export(left, os.path.join(out, "build", f"asm_left_{name}.stl"), p)
        export(right, os.path.join(out, "build", f"asm_right_{name}.stl"), p)

    rep = side["report"]
    rep["params"] = asdict(p)
    rep["ring_centerline_r"] = p.ring_rc
    rep["front_struts"] = side["fmod"].struts
    rep["rear_struts"] = side["rmod"].struts
    rep["front_clip_center"] = side["fmod"].clip_center
    rep["rear_clip_center"] = side["rmod"].clip_center
    os.makedirs(os.path.join(out, "build"), exist_ok=True)
    with open(os.path.join(out, "build", "build_report.json"), "w") as f:
        json.dump(rep, f, indent=2, default=list)
    print(json.dumps({k: v for k, v in rep.items() if k != "params"}, indent=2, default=list))


if __name__ == "__main__":
    main()
