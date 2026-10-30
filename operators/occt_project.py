"""Fork: OCCT Project Edges — the "Convert Entities" of other CAD tools.

Copies edges of an OCCT part into the active sketch, projected along the
sketch normal. Lines stay lines; circles and arcs stay circles and arcs when
parallel to the sketch; full ellipses stay ellipses; Bezier edges are
projected exactly through their poles; anything else (B-splines, tilted
circles...) is approximated by cubic Beziers within a tolerance.

The copy is static: run it again after changing the source part.
"""
import json
import math

import bpy
from bpy.props import BoolProperty, EnumProperty, FloatProperty
from bpy.types import Context, Operator
from bpy.utils import register_classes_factory
from mathutils import Vector

from ..declarations import Operators
from ..utilities.view import refresh
from . import occt_ops


# ---------------------------------------------------------------------------
# Choice of edges
# ---------------------------------------------------------------------------

def _edges_picked(solid, scene):
    import build123d as b3d
    probes = json.loads(scene.get(occt_ops.PICKS_KEY, "[]"))
    edges = solid.edges()
    out = []
    for pr in probes:
        p = b3d.Vector(tuple(pr))
        best = min(edges, key=lambda e: (e.center() - p).length)
        if best not in out:
            out.append(best)
    return out


def _edges_face_cursor(solid, scene):
    import build123d as b3d
    p = b3d.Vector(tuple(scene.cursor.location))
    face = min(solid.faces(), key=lambda f: (f.center() - p).length)
    return list(face.edges())


def _edges_in_plane(solid, M, tol):
    Mi = M.inverted()
    out = []
    for e in solid.edges():
        if all(abs((Mi @ Vector(tuple(e @ t))).z) <= tol for t in (0.0, 0.25, 0.5, 0.75, 1.0)):
            out.append(e)
    return out


# ---------------------------------------------------------------------------
# Projection of one edge -> segments in sketch coordinates (Blender units)
# ---------------------------------------------------------------------------

def _local(Mi, v):
    p = Mi @ Vector(tuple(v))
    return Vector((p.x, p.y))


def _bezier_fit(edge, Mi, tol):
    """Cubic Beziers through the projected edge (Hermite spans), refined until within tol."""
    for spans in (1, 2, 4, 8, 16, 32):
        segs, worst = [], 0.0
        for k in range(spans):
            t0, t1 = k / spans, (k + 1) / spans
            p0, p3 = _local(Mi, edge @ t0), _local(Mi, edge @ t1)
            L = edge.length * (t1 - t0)
            d0 = Mi.to_3x3() @ Vector(tuple(edge.tangent_at(t0)))
            d1 = Mi.to_3x3() @ Vector(tuple(edge.tangent_at(t1)))
            c1 = p0 + Vector((d0.x, d0.y)) * L / 3
            c2 = p3 - Vector((d1.x, d1.y)) * L / 3
            segs.append({"type": "bezier", "p1": p0, "c1": c1, "c2": c2, "p2": p3})
            for j in (0.25, 0.5, 0.75):
                u = 1 - j
                b = u ** 3 * p0 + 3 * u * u * j * c1 + 3 * u * j * j * c2 + j ** 3 * p3
                worst = max(worst, (b - _local(Mi, edge @ (t0 + j * (t1 - t0)))).length)
        if worst <= tol:
            break
    return segs


def _fit_conic(pts, tol):
    """Closed curve sampled in 2D -> ("circle", ct, r) / ("ellipse", ct, pa, pb) if every sample lies
    within tol of the fitted conic, else None (algebraic least-squares fit, then a true distance check)."""
    import numpy as np
    P = np.array([(p.x, p.y) for p in pts])
    m = P.mean(axis=0)
    s = max(np.abs(P - m).max(), 1e-12)
    Q = (P - m) / s  # normalised for a well-conditioned fit
    x, y = Q[:, 0], Q[:, 1]
    D = np.column_stack([x * x, x * y, y * y, x, y, np.ones_like(x)])
    A, B, C, Dd, E, F = np.linalg.svd(D)[2][-1]
    if B * B - 4 * A * C >= 0:
        return None  # not an ellipse
    c = np.linalg.solve([[2 * A, B], [B, 2 * C]], [-Dd, -E])
    Fc = A * c[0] ** 2 + B * c[0] * c[1] + C * c[1] ** 2 + Dd * c[0] + E * c[1] + F
    w, V = np.linalg.eigh([[A, B / 2], [B / 2, C]])
    if np.any(-Fc / w <= 0):
        return None
    radii = np.sqrt(-Fc / w) * s
    centre = c * s + m
    order = np.argsort(-radii)  # major first
    ra, rb = radii[order]
    ua, ub = V[:, order[0]], V[:, order[1]]
    # distance of each sample to the ellipse, measured along its ray from the centre
    rel = P - centre
    u, v = rel @ ua, rel @ ub
    k = np.sqrt((u / ra) ** 2 + (v / rb) ** 2)
    dist = np.abs(k - 1) * np.hypot(u, v) / np.maximum(k, 1e-12)
    if dist.max() > tol:
        return None
    ct = Vector(centre)
    if abs(ra - rb) <= tol:
        return ("circle", ct, float((ra + rb) / 2))
    return ("ellipse", ct, ct + Vector(ua * ra), ct + Vector(ub * rb))


def project_edge(edge, M, tol):
    """Segments (sketch-local, Blender units) for one B-rep edge; [] if it projects to a point."""
    import build123d as b3d
    Mi = M.inverted()
    n_sketch = (M.to_3x3() @ Vector((0, 0, 1))).normalized()
    p1, p2 = _local(Mi, edge @ 0.0), _local(Mi, edge @ 1.0)
    gt = edge.geom_type
    closed = edge.is_closed

    if gt == b3d.GeomType.LINE:
        if (p2 - p1).length <= tol:
            return []  # edge along the sketch normal
        return [{"type": "line", "p1": p1, "p2": p2}]

    if gt == b3d.GeomType.CIRCLE:
        n_edge = Vector(tuple(edge.normal())).normalized()
        if abs(abs(n_edge.dot(n_sketch)) - 1.0) < 1e-6:  # parallel: stays a circle
            ct = _local(Mi, edge.arc_center)
            r = edge.radius
            if closed:
                return [{"type": "circle", "ct": ct, "r": r}]
            mid = _local(Mi, edge @ 0.5)
            a1 = math.atan2(*(p1 - ct)[::-1])
            a2 = math.atan2(*(p2 - ct)[::-1])
            am = math.atan2(*(mid - ct)[::-1])
            ccw = (am - a1) % (2 * math.pi) < (a2 - a1) % (2 * math.pi)
            return [{"type": "arc", "ct": ct, "p1": p1 if ccw else p2, "p2": p2 if ccw else p1}]

    if gt == b3d.GeomType.ELLIPSE and closed:
        el = edge.geom_adaptor().Ellipse()
        n_edge = Vector((el.Axis().Direction().X(), el.Axis().Direction().Y(), el.Axis().Direction().Z()))
        if abs(abs(n_edge.dot(n_sketch)) - 1.0) < 1e-6:
            c = el.Location()
            xa, ya = el.XAxis().Direction(), el.YAxis().Direction()
            ct = Vector((c.X(), c.Y(), c.Z()))
            pa = ct + Vector((xa.X(), xa.Y(), xa.Z())) * el.MajorRadius()
            pb = ct + Vector((ya.X(), ya.Y(), ya.Z())) * el.MinorRadius()
            return [{"type": "ellipse", "ct": _local(Mi, ct), "pa": _local(Mi, pa), "pb": _local(Mi, pb)}]

    if gt == b3d.GeomType.BEZIER:
        try:
            bz = edge.geom_adaptor().Bezier()
            if bz.Degree() == 3:
                poles = [Vector((bz.Pole(i).X(), bz.Pole(i).Y(), bz.Pole(i).Z())) for i in range(1, 5)]
                q = [_local(Mi, v) for v in poles]
                # pole order follows the curve: match the edge's own start
                if (q[0] - p1).length > (q[3] - p1).length:
                    q.reverse()
                if (q[0] - p1).length <= tol and (q[3] - p2).length <= tol:
                    return [{"type": "bezier", "p1": p1, "c1": q[1], "c2": q[2], "p2": p2}]
        except Exception:
            pass

    if edge.length <= tol or ((p2 - p1).length <= tol and not closed):
        return []
    if closed:
        # a closed curve that is really an ellipse or a circle (OCCT may turn them into B-splines)
        fit = _fit_conic([_local(Mi, edge @ (k / 64)) for k in range(64)], tol)
        if fit and fit[0] == "circle":
            return [{"type": "circle", "ct": fit[1], "r": fit[2]}]
        if fit:
            return [{"type": "ellipse", "ct": fit[1], "pa": fit[2], "pb": fit[3]}]
    return _bezier_fit(edge, Mi, tol)


def _merge_full_conics(edges, M, tol):
    """Pieces of one same circle or ellipse covering the whole turn (seam-split rims) -> one entity.

    Returns (segments, remaining_edges).
    """
    import build123d as b3d
    Mi = M.inverted()
    n_sketch = (M.to_3x3() @ Vector((0, 0, 1))).normalized()
    groups, rest = {}, []
    for e in edges:
        if e.geom_type not in (b3d.GeomType.CIRCLE, b3d.GeomType.ELLIPSE) or e.is_closed:
            rest.append(e)
            continue
        ad = e.geom_adaptor()
        if e.geom_type == b3d.GeomType.CIRCLE:
            g = ad.Circle()
            radii = (g.Radius(), g.Radius())
        else:
            g = ad.Ellipse()
            radii = (g.MajorRadius(), g.MinorRadius())
        c, ax, xa = g.Location(), g.Axis().Direction(), g.XAxis().Direction()
        normal = Vector((ax.X(), ax.Y(), ax.Z()))
        if abs(abs(normal.dot(n_sketch)) - 1.0) > 1e-6:
            rest.append(e)
            continue
        q = lambda v: round(v / max(tol, 1e-9))
        key = (e.geom_type, q(c.X()), q(c.Y()), q(c.Z()), q(radii[0]), q(radii[1]),
               round(abs(xa.X()), 4), round(abs(xa.Y()), 4), round(abs(xa.Z()), 4))
        groups.setdefault(key, []).append((e, g, ad.LastParameter() - ad.FirstParameter()))
    segments = []
    for (gt, *_), items in groups.items():
        if abs(sum(span for _, _, span in items) - 2 * math.pi) > 1e-6:
            rest.extend(e for e, _, _ in items)
            continue
        e, g, _ = items[0]
        c = g.Location()
        ct = Vector((c.X(), c.Y(), c.Z()))
        if gt == b3d.GeomType.CIRCLE:
            segments.append({"type": "circle", "ct": _local(Mi, ct), "r": g.Radius()})
        else:
            xa, ya = g.XAxis().Direction(), g.YAxis().Direction()
            pa = ct + Vector((xa.X(), xa.Y(), xa.Z())) * g.MajorRadius()
            pb = ct + Vector((ya.X(), ya.Y(), ya.Z())) * g.MinorRadius()
            segments.append({"type": "ellipse", "ct": _local(Mi, ct), "pa": _local(Mi, pa), "pb": _local(Mi, pb)})
    return segments, rest


def _merge_curved_loops(edges, M, tol):
    """Curved edges (no straight line) chaining into a closed loop, e.g. an ellipse split in two
    B-spline halves -> one circle / ellipse when the loop fits a conic. Returns (segments, rest)."""
    import build123d as b3d
    Mi = M.inverted()
    curved = [e for e in edges if e.geom_type != b3d.GeomType.LINE and not e.is_closed]
    rest = [e for e in edges if e not in curved]
    segments, used = [], set()
    ends = [(_local(Mi, e @ 0.0), _local(Mi, e @ 1.0)) for e in curved]
    for i in range(len(curved)):
        if i in used:
            continue
        loop, start, cur = [i], ends[i][0], ends[i][1]
        while (cur - start).length > tol:
            nxt = next((j for j in range(len(curved)) if j not in loop and j not in used
                        and ((ends[j][0] - cur).length <= tol or (ends[j][1] - cur).length <= tol)), None)
            if nxt is None:
                break
            cur = ends[nxt][1] if (ends[nxt][0] - cur).length <= tol else ends[nxt][0]
            loop.append(nxt)
        if len(loop) < 2 or (cur - start).length > tol:
            continue
        pts = [_local(Mi, curved[j] @ (k / 32)) for j in loop for k in range(32)]
        fit = _fit_conic(pts, tol)
        if not fit:
            continue
        used.update(loop)
        if fit[0] == "circle":
            segments.append({"type": "circle", "ct": fit[1], "r": fit[2]})
        else:
            segments.append({"type": "ellipse", "ct": fit[1], "pa": fit[2], "pb": fit[3]})
    rest += [curved[j] for j in range(len(curved)) if j not in used]
    return segments, rest


def _key(seg, tol):
    """Rounded signature: two edges projecting to the same curve (top and bottom rims) count once."""
    r = lambda v: (round(v.x / tol), round(v.y / tol))
    if seg["type"] == "circle":
        return ("c", r(seg["ct"]), round(seg["r"] / tol))
    if seg["type"] == "ellipse":
        return ("e", r(seg["ct"]), r(seg["pa"]))
    a, b = r(seg["p1"]), r(seg["p2"])
    mid = seg.get("c1", seg.get("ct", (seg["p1"] + seg["p2"]) / 2))
    return (seg["type"], min(a, b), max(a, b), r(mid))


# ---------------------------------------------------------------------------
# Creation in the sketch
# ---------------------------------------------------------------------------

def add_segments(context, sketch, segments, construction=False, tol=1e-6):
    """Create the segments in the sketch; coincident endpoints become one point. Returns the count."""
    sse = context.scene.sketcher.entities
    sk_i = sketch.slvs_index
    get = sse.get
    known = []

    def point(co, shared=True):
        if shared:
            for c, i in known:
                if (c - co).length <= tol:
                    return i
        i = sse.add_point_2d(tuple(co), get(sk_i), index_reference=True)
        if shared:
            known.append((co.copy(), i))
        return i

    count = 0
    for s in segments:
        t = s["type"]
        if t == "line":
            e = sse.add_line_2d(get(point(s["p1"])), get(point(s["p2"])), get(sk_i))
        elif t == "arc":
            p1, p2, ct = point(s["p1"]), point(s["p2"]), point(s["ct"], False)
            sk = get(sk_i)
            e = sse.add_arc(sk.wp.nm, get(ct), get(p1), get(p2), sk)
        elif t == "circle":
            ct = point(s["ct"], False)
            sk = get(sk_i)
            e = sse.add_circle(sk.wp.nm, get(ct), s["r"], sk)
        elif t == "ellipse":
            e = sse.add_ellipse(get(point(s["ct"], False)), get(point(s["pa"], False)),
                                get(point(s["pb"], False)), get(sk_i))
        else:
            e = sse.add_bezier(point(s["p1"]), point(s["c1"], False), point(s["c2"], False),
                               point(s["p2"]), get(sk_i))
        e.construction = construction
        count += 1
    return count


def project_edges(context, sketch, part, mode="PICKED", tol=1e-6, construction=False):
    """Project the chosen edges of an OCCT part into the sketch. Returns (created, skipped)."""
    solid = occt_ops._solid_cache.get(part)
    if solid is None:
        occt_ops.recompute_part(context, part)
        solid = occt_ops._solid_cache.get(part)
    if solid is None:
        raise RuntimeError(f"part '{part}' has no B-rep")
    M = sketch.wp.matrix_basis
    if mode == "PICKED":
        edges = _edges_picked(solid, context.scene)
    elif mode == "FACE":
        edges = _edges_face_cursor(solid, context.scene)
    else:
        edges = _edges_in_plane(solid, M, max(tol * 10, 1e-6))
    merged, edges = _merge_full_conics(edges, M, max(tol * 10, 1e-7))
    loops, edges = _merge_curved_loops(edges, M, max(tol * 10, 1e-7))
    merged += loops
    segments, seen, skipped = [], set(), 0
    for s in merged:
        k = _key(s, max(tol * 10, 1e-7))
        if k not in seen:
            seen.add(k)
            segments.append(s)
    for e in edges:
        segs = project_edge(e, M, tol)
        if not segs:
            skipped += 1
            continue
        for s in segs:
            k = _key(s, max(tol * 10, 1e-7))
            if k in seen:
                continue
            seen.add(k)
            segments.append(s)
    created = add_segments(context, sketch, segments, construction, tol=max(tol * 10, 1e-7))
    sk = context.scene.sketcher.entities.get(sketch.slvs_index)
    sk.solve(context)
    return created, skipped


class View3D_OT_slvs_occt_project_edges(Operator):
    """Project edges of an OCCT part into the active sketch (Convert Entities): picked edges, the face
    nearest the 3D cursor, or every edge lying in the sketch plane. Static copy: run again after
    changing the part"""

    bl_idname = Operators.OCCTProjectEdges
    bl_label = "OCCT Project Edges"
    bl_options = {"REGISTER", "UNDO"}

    part: EnumProperty(name="Part", items=occt_ops._parts_enum)
    mode: EnumProperty(name="Edges", items=[
        ("PICKED", "Picked Edges", "Edges selected with OCCT Pick Edges"),
        ("FACE", "Face at 3D Cursor", "Every edge of the face nearest the 3D cursor"),
        ("IN_PLANE", "In Sketch Plane", "Every edge lying in the sketch plane"),
    ], default="FACE")
    tolerance: FloatProperty(name="Tolerance", default=0.000005, min=1e-8, soft_max=0.001,
                             unit="LENGTH", precision=6,
                             description="Approximation tolerance for curves that cannot be copied exactly")
    construction: BoolProperty(name="As Construction", default=False)

    @classmethod
    def poll(cls, context):
        return (context.scene.sketcher.active_sketch is not None
                and bool(occt_ops.get_recipes(context.scene)))

    def invoke(self, context, event):
        picks = json.loads(context.scene.get(occt_ops.PICKS_KEY, "[]"))
        if picks and not self.properties.is_property_set("mode"):
            self.mode = "PICKED"
        return context.window_manager.invoke_props_dialog(self, width=300)

    def execute(self, context: Context):
        sketch = context.scene.sketcher.active_sketch
        if self.part == "NONE":
            self.report({"ERROR"}, "No OCCT part")
            return {"CANCELLED"}
        try:
            created, skipped = project_edges(context, sketch, self.part, self.mode,
                                             self.tolerance, self.construction)
        except RuntimeError as e:
            self.report({"ERROR"}, str(e))
            return {"CANCELLED"}
        if not created:
            self.report({"WARNING"}, "No edge to project (edges along the sketch normal are skipped)")
            return {"CANCELLED"}
        self.report({"INFO"}, f"{created} entit{'y' if created == 1 else 'ies'} projected"
                              + (f", {skipped} edge(s) along the normal skipped" if skipped else ""))
        refresh(context)
        return {"FINISHED"}


register, unregister = register_classes_factory((View3D_OT_slvs_occt_project_edges,))
