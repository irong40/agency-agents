"""
Parametric 360-degree connected propeller cage for the DJI Mini 4 Pro.
Version 0.3 (PROVISIONAL).  Layout follows the YMTIEXNG / DJI-style reference photos.

Units: mm.  Body frame: origin at the airframe centre, +X forward, +Y left, +Z up,
z = 0 at the propeller hub plane.  The LEFT side is built and mirrored for the right.

Structure per side (one figure-8 cage around the front and rear propeller):
  LID (top level, printed flat, top face down)
    top loop      figure-8 rail around both props
    top rings     one smaller ring over each prop
    top spokes    ring -> loop -> upper outer rail
    upper rail    outboard outer rail with the open "C" ends near nose and tail
  BODY (bottom level on the bed, printed upright)
    lower rail    outboard outer rail, same plan shape as the upper rail
    hub rings     one under each motor, carries the arm clip
    bottom spokes hub ring -> lower rail
    spine         hub to hub through the waist
    waist bar     across the waist, with legs up to the top loop
    ribs          lower rail -> upper rail, with pegs into the lid
    clips         snap C-clips on each arm next to the motor, opening facing up

FDM adaptation (not in the photos): the side ribs are near-vertical instead of
bulging outward, because a 26-degree bulge cannot print without supports.  The
lid/body split and the waist split (dovetail joints) keep every part on a
180 x 180 mm bed.

Source tags on parameters: [PHOTO] proportion measured from the reference photos,
scaled by [DERIVED] values; [DERIVED] computed from recalled DJI figures;
[ASSUMED] placeholder, must be measured; [DESIGN] engineering choice.
See docs/DIMENSIONS.md.

Run:  python mini4pro_cage.py [--json overrides.json] [--out DIR]
"""
from __future__ import annotations

import argparse
import json
import math
import os
from dataclasses import asdict, dataclass, replace

import numpy as np
import trimesh
from manifold3d import CrossSection, Manifold, OpType

SEG = 20  # segments for small round features


# ----------------------------------------------------------------------------
# Parameters
# ----------------------------------------------------------------------------
@dataclass
class Params:
    # --- drone, LEFT side ---------------------------------------------------
    prop_diameter: float = 154.0                 # [DERIVED] photo ratios + DJI length 298 mm
    front_motor: tuple = (72.0, 109.5, 0.0)      # [DERIVED] see DIMENSIONS.md
    rear_motor: tuple = (-72.0, 109.5, 0.0)      # [DERIVED] z ASSUMED equal to front
    prop_cone_up: float = 4.0                    # [ASSUMED] blade tip rise under thrust
    radial_clearance: float = 6.0                # [DESIGN] minimum, used for the keep-out
    prop_vertical_keepout: float = 5.0           # [DESIGN]
    motor_keepout_r: float = 11.0                # [ASSUMED]

    # --- arms and clips ------------------------------------------------------
    front_arm_angle: float | None = None         # [ASSUMED] None = toward airframe centre
    rear_arm_angle: float | None = None
    front_clip_dist: float = 18.0                # [ASSUMED] motor axis to clip centre
    rear_clip_dist: float = 18.0
    front_clip_dz: float = -12.0                 # [ASSUMED] arm centre below prop plane
    rear_clip_dz: float = -12.0
    front_arm_w: float = 12.0                    # [ASSUMED] arm section at the clip
    front_arm_h: float = 8.0
    rear_arm_w: float = 12.0
    rear_arm_h: float = 8.0
    arm_keepout_clear: float = 1.5               # [DESIGN]
    clip_len: float = 10.0                       # [DESIGN]
    clip_wall: float = 1.8
    clip_clear: float = 0.25                     # [DESIGN] pick from the clearance ladder
    snap_lip: float = 0.8
    zip_groove_w: float = 2.5
    zip_groove_d: float = 0.6

    # --- cage plan (PHOTO ratios x loop_r) -----------------------------------
    loop_r: float = 63.0                         # [PHOTO+DERIVED] top loop radius (ratio 1.00)
    top_ring_r: float = 44.0                     # [PHOTO] 0.70
    hub_ring_r: float = 21.5                     # [PHOTO] 0.34
    rail_r: float = 91.5                         # [PHOTO] 1.45, lower outer rail
    rail_upper_inset: float = 6.0                # [DESIGN] upper rail sits this far inside
    loop_waist_w: float = 36.0                   # [PHOTO] 0.57, half-width at the waist
    rail_waist_w: float = 66.0                   # [PHOTO] 1.05
    cut_theta: float = 60.0                      # [PHOTO] C-cut end, deg from inboard toward lobe end
    spoke_thetas: tuple = (60.0, 121.0, 180.0)   # [PHOTO]
    short_spoke_theta: float = -25.0             # [PHOTO] top level, ring -> loop only

    # --- heights -----------------------------------------------------------------
    z_top: float = 12.0                          # [ASSUMED] lid centre plane
    z_bot: float = -24.0                         # [ASSUMED] bottom level centre plane
    lid_h: float = 2.2                           # [DESIGN] flat bar height, lid
    bot_h: float = 2.4                           # [DESIGN] flat bar height, bottom level

    # --- member widths -----------------------------------------------------------
    loop_w: float = 3.0                          # [DESIGN] member widths; see variants/ for heavier/lighter
    rail_w: float = 2.6
    ring_w: float = 2.0
    spoke_w: float = 2.0
    hub_w: float = 2.8
    spine_w: float = 2.6
    rib_w: float = 2.6
    waist_w: float = 2.6
    rib_spacing: float = 55.0                    # [PHOTO] about 11 ribs per side, as pictured
    waist_leg_base_w: float = 56.0               # [DESIGN] inboard end of the waist bar
    waist_leg_top_w: float = 52.0                # [DESIGN] leg meets a lid link here, outside both prop zones
    waist_feature_offset: float = 3.0            # [PHOTO, shifted 3 mm] waist bar, rib and leg; keeps their pads clear of the dovetails

    # --- lid to body pegs ------------------------------------------------------------
    boss_d: float = 5.5
    boss_h: float = 4.0
    peg_d: float = 2.0
    peg_h: float = 2.4
    peg_hole_d: float = 2.4
    lid_pad_h: float = 3.0                       # lid pad depth below the lid plane

    # --- waist split ---------------------------------------------------------------
    split_waist: bool = True
    split_offset: float = 7.0                    # [DESIGN] split line this far behind the waist centreline
    dove_pad_l: float = 16.0
    dove_pad_w: float = 10.0
    dove_neck: float = 4.0
    dove_tip: float = 6.5
    dove_depth: float = 5.0
    dove_clear: float = 0.15

    # --- keep-out checks only ---------------------------------------------------------
    body_box: tuple = (-75.0, 75.0, -33.0, 33.0, -45.0, 15.0)   # [ASSUMED]
    camera_pt: tuple = (80.0, 0.0, -25.0)                       # [ASSUMED]
    landing_z: float = -48.0                                    # [ASSUMED]

    @property
    def prop_r(self):
        return self.prop_diameter / 2.0


# ----------------------------------------------------------------------------
# Plan geometry
# ----------------------------------------------------------------------------
def V(x, y):
    return np.array([float(x), float(y)])


def ang(v):
    return math.degrees(math.atan2(v[1], v[0]))


def pol(c, r, a):
    t = math.radians(a)
    return np.array([c[0] + r * math.cos(t), c[1] + r * math.sin(t)])


def cross2(u, v):
    return u[0] * v[1] - u[1] * v[0]


class Frame:
    """Plan frame for one side: motors, axis, waist midpoint, inboard normal."""

    def __init__(self, p: Params):
        self.F = V(*p.front_motor[:2])
        self.R = V(*p.rear_motor[:2])
        self.zF, self.zR = p.front_motor[2], p.rear_motor[2]
        a = self.F - self.R
        self.d = float(np.linalg.norm(a))
        self.a = a / self.d
        self.M = (self.F + self.R) / 2
        n = V(-self.a[1], self.a[0])
        if np.dot(n, -self.M) < 0:
            n = -n
        self.n_in, self.n_out = n, -n

    def center(self, lobe):
        return self.F if lobe == "front" else self.R

    def zc(self, lobe):
        return self.zF if lobe == "front" else self.zR

    def lobe_angle(self, lobe, theta):
        end = self.a if lobe == "front" else -self.a
        s = 1.0 if cross2(self.n_in, end) > 0 else -1.0
        return ang(self.n_in) + s * theta


def arc(center, r, a0, a1, step=3.0):
    n = max(2, int(abs(a1 - a0) / step) + 1)
    return [pol(center, r, a0 + (a1 - a0) * i / (n - 1)) for i in range(n)]


def arc_via(center, r, a_start, a_end, a_via, step=3.0):
    ccw = (a_end - a_start) % 360.0
    via = (a_via - a_start) % 360.0
    sweep = ccw if via < ccw else -(360.0 - ccw)
    return arc(center, r, a_start, a_start + sweep, step)


def short_arc(center, r, a0, a1, step=3.0):
    diff = (a1 - a0 + 180.0) % 360.0 - 180.0
    return arc(center, r, a0, a0 + diff, step)


def fillet(fr: Frame, R, w, side_n):
    h = fr.d / 2
    Rw = (h * h + w * w - R * R) / (2 * (R - w))
    if Rw <= 0:
        raise ValueError(f"waist half-width {w} too large for radius {R} at spacing {fr.d:.1f}")
    C = fr.M + side_n * (w + Rw)
    Tf = fr.F + (C - fr.F) * R / (R + Rw)
    Tr = fr.R + (C - fr.R) * R / (R + Rw)
    return C, Rw, Tf, Tr


def peanut_closed(fr: Frame, R, w_out, w_in):
    Co, Rwo, Tfo, Tro = fillet(fr, R, w_out, fr.n_out)
    Ci, Rwi, Tfi, Tri = fillet(fr, R, w_in, fr.n_in)
    pts = []
    pts += arc_via(fr.F, R, ang(Tfi - fr.F), ang(Tfo - fr.F), ang(fr.a))
    pts += short_arc(Co, Rwo, ang(Tfo - Co), ang(Tro - Co))[1:]
    pts += arc_via(fr.R, R, ang(Tro - fr.R), ang(Tri - fr.R), ang(-fr.a))[1:]
    pts += short_arc(Ci, Rwi, ang(Tri - Ci), ang(Tfi - Ci))[1:-1]
    return pts


def rail_open(fr: Frame, R, w_out, cut_theta):
    Co, Rwo, Tfo, Tro = fillet(fr, R, w_out, fr.n_out)
    a_cut_f = fr.lobe_angle("front", cut_theta)
    a_cut_r = fr.lobe_angle("rear", cut_theta)
    pts = []
    pts += arc_via(fr.F, R, a_cut_f, ang(Tfo - fr.F), ang(fr.a))
    pts += short_arc(Co, Rwo, ang(Tfo - Co), ang(Tro - Co))[1:]
    pts += arc_via(fr.R, R, ang(Tro - fr.R), a_cut_r, ang(-fr.a))[1:]
    return pts


def ray_hit(origin, a_deg, poly, closed):
    """First intersection of a ray with a polyline."""
    d = np.array([math.cos(math.radians(a_deg)), math.sin(math.radians(a_deg))])
    best = None
    n = len(poly)
    rng = range(n if closed else n - 1)
    for i in rng:
        p, q = poly[i], poly[(i + 1) % n]
        e = q - p
        den = cross2(d, e)
        if abs(den) < 1e-12:
            continue
        w = p - origin
        t = cross2(w, e) / den
        s = cross2(w, d) / den
        if t > 1e-6 and -1e-9 <= s <= 1 + 1e-9:
            if best is None or t < best[0]:
                best = (t, origin + d * t)
    return None if best is None else best[1]


def ray_hit_over(origin, a_deg, poly, closed, over=0.35):
    """ray_hit, pushed `over` mm past the rail centreline so a spoke end never sits
    exactly on a rail vertex (concentric pucks produce non-manifold edges)."""
    h = ray_hit(origin, a_deg, poly, closed)
    if h is None:
        return None
    return h + np.array([math.cos(math.radians(a_deg)), math.sin(math.radians(a_deg))]) * over


def nearest_on(poly, pt):
    best = None
    for i in range(len(poly) - 1):
        p, q = poly[i], poly[i + 1]
        e = q - p
        t = np.clip(np.dot(pt - p, e) / np.dot(e, e), 0, 1)
        c = p + e * t
        dd = np.linalg.norm(c - pt)
        if best is None or dd < best[0]:
            best = (dd, c)
    return best[1]


def resample(poly, spacing, must=()):
    """Points along an open polyline: both ends, the `must` points, and fill so no
    gap exceeds `spacing`."""
    seg = [np.linalg.norm(poly[i + 1] - poly[i]) for i in range(len(poly) - 1)]
    cum = np.concatenate([[0], np.cumsum(seg)])
    L = cum[-1]

    def s_of(pt):
        best = None
        for i in range(len(poly) - 1):
            p, q = poly[i], poly[i + 1]
            e = q - p
            t = np.clip(np.dot(pt - p, e) / np.dot(e, e), 0, 1)
            dd = np.linalg.norm(p + e * t - pt)
            if best is None or dd < best[0]:
                best = (dd, cum[i] + t * seg[i])
        return best[1]

    keys = []
    for k in sorted([0.0, L] + [s_of(m) for m in must]):
        if not keys or k - keys[-1] > 1.0:      # drop near-duplicates (they make coincident ribs)
            keys.append(k)
    keys[-1] = L
    out = []
    for a, b in zip(keys[:-1], keys[1:]):
        k = max(1, math.ceil((b - a) / spacing))
        out += [a + (b - a) * j / k for j in range(k)]
    out.append(L)

    def pt_at(s):
        i = min(np.searchsorted(cum, s, side="right") - 1, len(seg) - 1)
        t = (s - cum[i]) / seg[i] if seg[i] > 0 else 0
        return poly[i] + (poly[i + 1] - poly[i]) * t

    return [pt_at(s) for s in out]


# ----------------------------------------------------------------------------
# Solid primitives
# ----------------------------------------------------------------------------
# Flat members (everything in the lid and in the bottom level) are built as 2D
# outlines with exact integer polygon booleans (Clipper, inside manifold3d) and
# extruded once.  Only ribs, legs, bosses, pegs, posts and clips are 3D, and each
# of those starts 0.2 mm inside a slab so no faces coincide.  This avoids the
# sliver and pinch geometry that chains of 3D hulls produce.
def circ2(p, d):
    return CrossSection.circle(d / 2, SEG).translate([float(p[0]), float(p[1])])


def seg2(p0, p1, w):
    return CrossSection.batch_hull([circ2(p0, w), circ2(p1, w)])


def stroke2(pts, w, closed=False):
    n = len(pts)
    return [seg2(pts[i], pts[(i + 1) % n], w) for i in range(n if closed else n - 1)]


def union2(parts):
    return CrossSection.batch_boolean(parts, OpType.Add)


def rect2(center, a_deg, lx, ly):
    return CrossSection.square([lx, ly], True).rotate(a_deg).translate([float(center[0]), float(center[1])])


def halfplane2(point, a_vec, front=True, size=4000.0):
    """Half-plane on the +a side of the line through `point` (or the -a side)."""
    s = 1.0 if front else -1.0
    c = np.asarray(point) + np.asarray(a_vec) * s * size / 2
    return rect2(c, ang(a_vec), size, size)


def slab(cs, z, h):
    return cs.extrude(h).translate([0, 0, z - h / 2])


def cyl(p, d, h):
    return Manifold.cylinder(h, d / 2, d / 2, SEG, True).translate([float(p[0]), float(p[1]), float(p[2])])


def strut(p_low, p_high, w, h_low, h_high):
    """3D member from a puck inside a slab (p_low) to a puck at p_high.  The low puck is
    0.4 mm narrower than `w` so its side faces never coincide with the slab outline."""
    return Manifold.batch_hull([cyl(p_low, w - 0.4, h_low), cyl(p_high, w, h_high)])


def union(parts):
    parts = [p for p in parts if p is not None and not p.is_empty()]
    return Manifold.batch_boolean(parts, OpType.Add) if len(parts) > 1 else parts[0]


def oriented_box(center3, a_deg, lx, ly, lz):
    return (Manifold.cube([lx, ly, lz], True)
            .rotate([0, 0, a_deg]).translate([float(c) for c in center3]))


def _miter():
    jt = getattr(CrossSection, "JoinType", None)
    if jt is None:
        from manifold3d import JoinType  # type: ignore
        return JoinType.Miter
    return jt.Miter


# ----------------------------------------------------------------------------
# Clip
# ----------------------------------------------------------------------------
def clip_solid(p: Params, arm_w, arm_h, clear=None, notches=0):
    """C-clip, local frame: arm axis along X, opening on +Z (the cage is pushed up from
    below and the lips snap over the top of the arm).  Returns (solid, outer_w, outer_h)."""
    c = p.clip_clear if clear is None else clear
    t, L = p.clip_wall, p.clip_len
    cav_w, cav_h = arm_w + 2 * c, arm_h + 2 * c
    ow, oh = cav_w + 2 * t, cav_h + 2 * t
    rf = min(1.2, t * 0.7)
    # 2D in (u, v) = (-z, y); extrude along local z, then rotate z -> x
    corners = [CrossSection.circle(rf, SEG).translate([su * (oh / 2 - rf), sv * (ow / 2 - rf)])
               for su in (-1, 1) for sv in (-1, 1)]
    outer = CrossSection.batch_hull(corners)
    cav = CrossSection.square([cav_h, cav_w], True)
    open_w = max(arm_w - 2 * p.snap_lip, 2.0)
    slot = CrossSection.square([t + 2, open_w], True).translate([-(cav_h / 2 + (t + 2) / 2), 0])
    sec = outer - cav - slot
    for s in (-1, 1):
        tri = CrossSection([[(-(oh / 2 + 0.01), s * open_w / 2),
                             (-(oh / 2 + 0.01), s * (open_w / 2 + p.snap_lip * 0.9)),
                             (-(cav_h / 2 - 0.01), s * open_w / 2)]])
        sec = sec - tri
    body = sec.extrude(L).rotate([0, 90, 0]).translate([-L / 2, 0, 0])
    if p.zip_groove_w > 0:
        gd = p.zip_groove_d
        frame = (Manifold.cube([p.zip_groove_w, ow + 2, oh + 2], True)
                 - Manifold.cube([p.zip_groove_w + 2, ow - 2 * gd, oh - 2 * gd], True))
        body = body - frame
    for i in range(notches):
        x = -L / 2 + 2.0 + i * 2.0
        body = body - Manifold.cube([1.0, 1.0, oh * 0.6], True).translate([x, ow / 2, 0])
    return body, ow, oh


# ----------------------------------------------------------------------------
# Cage
# ----------------------------------------------------------------------------
class Side:
    """All members of the LEFT side cage, plus reference points for the report."""

    def __init__(self, p: Params):
        self.p = p
        fr = self.fr = Frame(p)
        self.loop = peanut_closed(fr, p.loop_r, p.loop_waist_w, p.loop_waist_w)
        self.rail_up = rail_open(fr, p.rail_r - p.rail_upper_inset,
                                 p.rail_waist_w - p.rail_upper_inset, p.cut_theta)
        self.rail_lo = rail_open(fr, p.rail_r, p.rail_waist_w, p.cut_theta)
        self.waist_M = fr.M + fr.a * p.waist_feature_offset
        self.lid_top = p.z_top + p.lid_h / 2
        self.lid_bot = p.z_top - p.lid_h / 2
        self.pad_bot = self.lid_bot - p.lid_pad_h
        self.boss_top = self.pad_bot
        self.bot_top = p.z_bot + p.bot_h / 2
        self.report = {}
        self._layout()

    # -- shared layout ------------------------------------------------------------
    def _layout(self):
        p, fr = self.p, self.fr
        self.w_out_lo = ray_hit_over(self.waist_M, ang(fr.n_out), self.rail_lo, closed=False)
        self.w_in_lo = self.waist_M + fr.n_in * p.waist_leg_base_w
        self.leg_top = self.waist_M + fr.n_in * p.waist_leg_top_w
        must = [ray_hit(fr.center(l), fr.lobe_angle(l, th), self.rail_lo, closed=False)
                for l in ("front", "rear") for th in p.spoke_thetas] + [self.w_out_lo]
        self.rib_lo = resample(self.rail_lo, p.rib_spacing, [q for q in must if q is not None])
        self.rib_up = [nearest_on(self.rail_up, q) for q in self.rib_lo]
        self.peg_points = self.rib_up + [self.leg_top]
        self.report["ribs"] = len(self.rib_lo)
        self.report["peg_points"] = len(self.peg_points)

    # -- lid ------------------------------------------------------------------
    def lid_2d(self):
        p, fr = self.p, self.fr
        s = []
        s += stroke2(self.loop, p.loop_w, closed=True)
        s += stroke2(self.rail_up, p.rail_w)
        for lobe in ("front", "rear"):
            c = fr.center(lobe)
            s += stroke2(arc(c, p.top_ring_r, 1.37, 361.37, 6)[:-1], p.ring_w, closed=True)
            for th in p.spoke_thetas:
                a = fr.lobe_angle(lobe, th)
                s1 = ray_hit_over(c, a, self.rail_up, closed=False)
                if s1 is None:
                    s1 = ray_hit_over(c, a, self.loop, closed=True)
                s.append(seg2(pol(c, p.top_ring_r, a), s1, p.spoke_w))
            a = fr.lobe_angle(lobe, p.short_spoke_theta)
            s.append(seg2(pol(c, p.top_ring_r, a), ray_hit_over(c, a, self.loop, closed=True), p.spoke_w))
        wo = ray_hit_over(self.waist_M, ang(fr.n_out), self.rail_up, closed=False)
        wl = ray_hit(self.waist_M, ang(fr.n_out), self.loop, closed=True)
        s.append(seg2(wl, wo, p.spoke_w))
        li = ray_hit(self.waist_M, ang(fr.n_in), self.loop, closed=True)
        s.append(seg2(li, self.leg_top, p.spoke_w))
        s += [circ2(q, p.boss_d + 0.5) for q in self.peg_points]     # pad tops, flush with lid top
        return union2(s)

    def lid_3d(self):
        """Pad extensions below the lid plane (overlap 0.2 mm into the slab)."""
        p = self.p
        h = self.lid_bot + 0.2 - self.pad_bot
        zc = (self.lid_bot + 0.2 + self.pad_bot) / 2
        return [cyl((q[0], q[1], zc), p.boss_d + 0.1, h) for q in self.peg_points]   # 0.4 narrower than the 2D pad

    def lid_holes(self):
        p = self.p
        h = self.lid_top - self.pad_bot + 2
        zc = (self.lid_top + self.pad_bot) / 2
        return [cyl((q[0], q[1], zc), p.peg_hole_d, h) for q in self.peg_points]

    # -- body ------------------------------------------------------------------
    def body_2d(self):
        p, fr = self.p, self.fr
        s = []
        s += stroke2(self.rail_lo, p.rail_w)
        for lobe in ("front", "rear"):
            c = fr.center(lobe)
            s += stroke2(arc(c, p.hub_ring_r, 2.11, 362.11, 8)[:-1], p.hub_w, closed=True)
            for th in p.spoke_thetas:
                a = fr.lobe_angle(lobe, th)
                s.append(seg2(pol(c, p.hub_ring_r, a), ray_hit_over(c, a, self.rail_lo, closed=False), p.spoke_w))
            # link from the hub ring to the clip post
            aa = self.arm_angle(lobe)
            s.append(seg2(pol(c, getattr(p, f"{lobe}_clip_dist"), aa), pol(c, p.hub_ring_r, aa), p.hub_w))
            s.append(circ2(pol(c, getattr(p, f"{lobe}_clip_dist"), aa), 6.0))
        s.append(seg2(fr.F - fr.a * p.hub_ring_r, fr.R + fr.a * p.hub_ring_r, p.spine_w))
        s.append(seg2(self.w_out_lo, self.w_in_lo, p.waist_w))
        return union2(s)

    def body_3d(self):
        p = self.p
        z_in = p.z_bot                      # lower pucks centred in the slab, 0.4 mm shorter
        h_in = p.bot_h - 0.4
        boss_zc = self.boss_top - p.boss_h / 2
        m = []
        for q, u in zip(self.rib_lo, self.rib_up):
            m.append(strut((*q, z_in), (*u, boss_zc), p.rib_w, h_in, p.boss_h - 0.4))
            m.append(cyl((u[0], u[1], boss_zc), p.boss_d, p.boss_h))
        m.append(strut((*self.w_in_lo, z_in), (*self.leg_top, boss_zc), p.waist_w, h_in, p.boss_h - 0.4))
        m.append(cyl((self.leg_top[0], self.leg_top[1], boss_zc), p.boss_d, p.boss_h))
        # pegs start 0.2 mm inside the boss
        zc = self.boss_top + p.peg_h / 2 - 0.2
        m += [cyl((q[0], q[1], zc), p.peg_d, p.peg_h + 0.4) for q in self.peg_points]
        return m

    def arm_angle(self, lobe):
        aa = getattr(self.p, f"{lobe}_arm_angle")
        return ang(-self.fr.center(lobe)) if aa is None else aa

    def clips(self, lobes=("front", "rear")):
        p, fr = self.p, self.fr
        out, info = [], {}
        for lobe in lobes:
            c = fr.center(lobe)
            aa = self.arm_angle(lobe)
            cc2 = pol(c, getattr(p, f"{lobe}_clip_dist"), aa)
            cz = fr.zc(lobe) + getattr(p, f"{lobe}_clip_dz")
            clip, ow, oh = clip_solid(p, getattr(p, f"{lobe}_arm_w"), getattr(p, f"{lobe}_arm_h"))
            out.append(clip.rotate([0, 0, aa]).translate([float(cc2[0]), float(cc2[1]), cz]))
            bottom = cz - oh / 2
            # post: from inside the bottom slab up into the clip floor
            out.append(strut((*cc2, p.z_bot), (*cc2, bottom + 0.6), 5.0, p.bot_h - 0.4, 1.0))
            info[lobe] = {"arm_angle": aa, "clip_center": [*cc2, cz], "clip_outer_w": ow, "clip_outer_h": oh}
        self.report["clips"] = info
        return out

    def arm_keepouts(self):
        p, fr = self.p, self.fr
        e = p.arm_keepout_clear
        out = []
        for lobe in ("front", "rear"):
            c = fr.center(lobe)
            aa = self.arm_angle(lobe)
            L = float(np.linalg.norm(c)) - 25.0
            mid = pol(c, L / 2, aa)
            out.append(oriented_box((mid[0], mid[1], fr.zc(lobe) + getattr(p, f"{lobe}_clip_dz")), aa, L,
                                    getattr(p, f"{lobe}_arm_w") + 2 * e, getattr(p, f"{lobe}_arm_h") + 2 * e))
        return out

    def prop_keepouts(self):
        p, fr = self.p, self.fr
        out = []
        for lobe in ("front", "rear"):
            c = fr.center(lobe)
            z0 = fr.zc(lobe) - p.prop_vertical_keepout
            h = p.prop_vertical_keepout + p.prop_cone_up + 2.0
            r = p.prop_r + p.radial_clearance - 0.3
            out.append(Manifold.cylinder(h, r, r, 96, False).translate([float(c[0]), float(c[1]), z0]))
        return out

    # -- waist split ------------------------------------------------------------
    def split_point(self):
        return self.fr.M - self.fr.a * self.p.split_offset

    def crossings(self, polys):
        fr, Ps = self.fr, self.split_point()
        out = []
        for poly, closed in polys:
            n = len(poly)
            for i in range(n if closed else n - 1):
                p0, p1 = poly[i], poly[(i + 1) % n]
                s0, s1 = np.dot(p0 - Ps, fr.a), np.dot(p1 - Ps, fr.a)
                if s0 * s1 < 0:
                    out.append(p0 + (p1 - p0) * (s0 / (s0 - s1)))
        return out

    def split2d(self, cs, crossing_pts):
        """Split a flat level at the waist line.  Each crossing gets a pad and a dovetail:
        the tongue is on the rear part and points forward into a pocket in the front part."""
        p, fr = self.p, self.fr
        a_deg = ang(fr.a)
        Ps = self.split_point()
        hf, hr = halfplane2(Ps, fr.a, True), halfplane2(Ps, fr.a, False)
        front, rear = cs ^ hf, cs ^ hr
        for P in crossing_pts:
            pad = rect2(P, a_deg, p.dove_pad_l, p.dove_pad_w)
            poly = [(-0.6, -p.dove_neck / 2), (p.dove_depth, -p.dove_tip / 2),
                    (p.dove_depth, p.dove_tip / 2), (-0.6, p.dove_neck / 2)]
            tongue = CrossSection([poly]).rotate(a_deg).translate([float(P[0]), float(P[1])])
            pocket = CrossSection([poly]).offset(p.dove_clear, _miter()).rotate(a_deg).translate(
                [float(P[0]), float(P[1])])
            front = (front + (pad ^ hf)) - pocket
            rear = rear + (pad ^ hr) + tongue
        return front, rear

    # -- assemble ------------------------------------------------------------------
    def build(self):
        p = self.p
        body2, lid2 = self.body_2d(), self.lid_2d()
        b3, l3 = self.body_3d(), self.lid_3d()
        parts = {}
        if p.split_waist:
            n = [float(self.fr.a[0]), float(self.fr.a[1]), 0.0]
            off = float(np.dot(self.split_point(), self.fr.a))
            nr = [-n[0], -n[1], 0.0]
            spine = [self.fr.F - self.fr.a * p.hub_ring_r, self.fr.R + self.fr.a * p.hub_ring_r]
            bx = self.crossings([(self.rail_lo, False), (spine, False)])
            lx = self.crossings([(self.loop, True), (self.rail_up, False)])
            self.report["body_joints"], self.report["lid_joints"] = len(bx), len(lx)
            bf2, br2 = self.split2d(body2, bx)
            lf2, lr2 = self.split2d(lid2, lx)
            b3u, l3u = union(b3), union(l3)
            parts["body_front"] = union([slab(bf2, p.z_bot, p.bot_h), b3u.trim_by_plane(n, off)])
            parts["body_rear"] = union([slab(br2, p.z_bot, p.bot_h), b3u.trim_by_plane(nr, -off)])
            parts["lid_front"] = union([slab(lf2, p.z_top, p.lid_h), l3u.trim_by_plane(n, off)])
            parts["lid_rear"] = union([slab(lr2, p.z_top, p.lid_h), l3u.trim_by_plane(nr, -off)])
        else:
            parts["body"] = union([slab(body2, p.z_bot, p.bot_h)] + b3)
            parts["lid"] = union([slab(lid2, p.z_top, p.lid_h)] + l3)

        self.report["arm_keepout_trim_mm3"] = {}
        self.report["prop_keepout_trim_mm3"] = {}
        for k in list(parts):
            # anything removed by a keep-out means a member entered it: must be 0
            v0 = parts[k].volume()
            for ko in self.arm_keepouts():
                parts[k] = parts[k] - ko
            v1 = parts[k].volume()
            for ko in self.prop_keepouts():
                parts[k] = parts[k] - ko
            v2 = parts[k].volume()
            if k.startswith("lid"):
                for hcyl in self.lid_holes():
                    parts[k] = parts[k] - hcyl
            self.report["arm_keepout_trim_mm3"][k] = round(v0 - v1, 3)
            self.report["prop_keepout_trim_mm3"][k] = round(v1 - v2, 3)
        # clips wrap the arm, so they go on after the arm keep-out subtraction
        clips = self.clips()
        if p.split_waist:
            n = [float(self.fr.a[0]), float(self.fr.a[1]), 0.0]
            off = float(np.dot(self.split_point(), self.fr.a))
            cu = union(clips)
            parts["body_front"] = union([parts["body_front"], cu.trim_by_plane(n, off)])
            parts["body_rear"] = union([parts["body_rear"], cu.trim_by_plane([-n[0], -n[1], 0.0], -off)])
        else:
            parts["body"] = union([parts["body"]] + clips)
        return parts

    def fit_test(self, lobe):
        """Hub ring + clip + bottom spokes + lower-rail segments + ribs with bosses.
        Checks clip fit, hub position and the real outer-wall radius."""
        p, fr = self.p, self.fr
        c = fr.center(lobe)
        aa = self.arm_angle(lobe)
        s = stroke2(arc(c, p.hub_ring_r, 2.11, 362.11, 8)[:-1], p.hub_w, closed=True)
        s.append(seg2(pol(c, getattr(p, f"{lobe}_clip_dist"), aa), pol(c, p.hub_ring_r, aa), p.hub_w))
        s.append(circ2(pol(c, getattr(p, f"{lobe}_clip_dist"), aa), 6.0))
        m3 = []
        boss_zc = self.boss_top - p.boss_h / 2
        for th in p.spoke_thetas:
            a = fr.lobe_angle(lobe, th)
            s1 = ray_hit_over(c, a, self.rail_lo, closed=False)
            s.append(seg2(pol(c, p.hub_ring_r, a), s1, p.spoke_w))
            seg = [q for q in self.rail_lo if np.linalg.norm(q - s1) < 14.0]
            if len(seg) >= 2:
                s += stroke2(seg, p.rail_w)
            u = nearest_on(self.rail_up, s1)
            m3.append(strut((*s1, p.z_bot), (*u, boss_zc), p.rib_w, p.bot_h - 0.4, p.boss_h - 0.4))
            m3.append(cyl((u[0], u[1], boss_zc), p.boss_d, p.boss_h))
        body = union([slab(union2(s), p.z_bot, p.bot_h)] + m3)
        for k in self.arm_keepouts() + self.prop_keepouts():
            body = body - k
        return union([body] + self.clips((lobe,)))


# ----------------------------------------------------------------------------
# Export
# ----------------------------------------------------------------------------
def to_trimesh(m: Manifold):
    m = m.simplify(0.002)   # collapse sub-micron slivers before float32 STL output
    mesh = m.to_mesh()
    v = np.asarray(mesh.vert_properties)[:, :3]
    f = np.asarray(mesh.tri_verts)
    return trimesh.Trimesh(vertices=v, faces=f, process=True)


def mirror_y(m: Manifold):
    return m.mirror([0, 1, 0])


def print_pose(m: Manifold, flip: bool):
    if flip:
        m = m.rotate([180, 0, 0])
    b = m.bounding_box()
    return m.translate([-(b[0] + b[3]) / 2, -(b[1] + b[4]) / 2, -b[2]])


def export(m: Manifold, path):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tm = to_trimesh(m)
    tm.export(path)
    return tm


def clip_ladder(p: Params, lobe, clears=(0.10, 0.25, 0.40)):
    aw, ah = getattr(p, f"{lobe}_arm_w"), getattr(p, f"{lobe}_arm_h")
    out = []
    for i, c in enumerate(clears):
        s, ow, oh = clip_solid(p, aw, ah, clear=c, notches=i + 1)
        out.append(s.translate([0, i * (ow + 6), 0]))
    return union(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--json")
    ap.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
    args = ap.parse_args()
    p = Params()
    if args.json:
        with open(args.json) as f:
            p = replace(p, **{k: (tuple(v) if isinstance(v, list) else v)
                              for k, v in json.load(f).items() if not k.startswith("_")})
    out = os.path.abspath(args.out)
    stl, build = os.path.join(out, "stl"), os.path.join(out, "build")

    side = Side(p)
    parts = side.build()
    for name, m in parts.items():
        flip = name.startswith("lid")
        export(print_pose(m, flip), os.path.join(stl, "provisional_cage", f"PROVISIONAL_left_{name}.stl"))
        export(print_pose(mirror_y(m), flip), os.path.join(stl, "provisional_cage", f"PROVISIONAL_right_{name}.stl"))
        export(m, os.path.join(build, f"asm_left_{name}.stl"))
        export(mirror_y(m), os.path.join(build, f"asm_right_{name}.stl"))

    for lobe in ("front", "rear"):
        export(print_pose(Side(p).fit_test(lobe), False),
               os.path.join(stl, "fit_tests", f"{lobe}_mount_fit_test.stl"))
        export(print_pose(clip_ladder(p, lobe), False),
               os.path.join(stl, "fit_tests", f"{lobe}_clip_clearance_ladder.stl"))

    rep = dict(side.report)
    rep["motor_spacing_mm"] = side.fr.d
    rep["waist_midpoint"] = side.fr.M.tolist()
    rep["params"] = asdict(p)
    os.makedirs(build, exist_ok=True)
    enc = lambda o: o.tolist() if hasattr(o, "tolist") else list(o)  # noqa: E731
    with open(os.path.join(build, "build_report.json"), "w") as f:
        json.dump(rep, f, indent=2, default=enc)
    print(json.dumps({k: v for k, v in rep.items() if k != "params"}, indent=2, default=enc))


if __name__ == "__main__":
    main()
