"""Fork: sketch automation tools that work on the user's own drawing.

The user draws; none of these tools create geometry (except a fixed construction
"Origin" point used as the dimensioning reference). They read the sketch and add
what the drawing already shows:

- Auto Constraints: tangency at smooth joints, horizontal / vertical lines,
  touching endpoints; removes redundant constraints.
- Propose Dimensions: base dimensions using the drawing's values, rounded.
- Fully Constrain Sketch: Propose Dimensions, then ellipse rotation, curve
  joints, Bezier handles and any remaining point, until no degree of freedom
  is left.
- Arrange Dimensions: places dimensions next to the part, outside, without
  overlaps.

Every dimension is tried on its own: it is kept only if the solver accepts it
and it actually removes a degree of freedom.
"""
import math
from collections import defaultdict

from bpy.props import BoolProperty, FloatProperty
from bpy.types import Context, Operator
from bpy.utils import register_classes_factory

from ..declarations import Operators
from ..model.types import SlvsArc, SlvsBezier, SlvsEllipse, SlvsLine2D
from ..utilities.view import refresh


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _outgoing(e, pt):
    """Unit direction in which curve e leaves its endpoint pt, or None."""
    co = pt.co
    if isinstance(e, SlvsLine2D):
        v = (e.p2 if pt == e.p1 else e.p1).co - co
    elif isinstance(e, SlvsBezier):
        v = (e.ctrl1 if pt == e.p1 else e.ctrl2).co - co
        if v.length < 1e-12:  # handle on its endpoint: use the other handle
            v = (e.ctrl2 if pt == e.p1 else e.ctrl1).co - co
    elif isinstance(e, SlvsArc):
        v = (co - e.ct.co).orthogonal()
        if v.dot(_arc_midpoint(e) - co) < 0:
            v = -v
    else:
        return None
    return v.normalized() if v.length > 1e-12 else None


def _arc_midpoint(arc):
    c = arc.ct.co
    a1 = math.atan2(*(arc.p1.co - c)[::-1])
    a2 = math.atan2(*(arc.p2.co - c)[::-1])
    sweep = (a2 - a1) % (2 * math.pi)
    if getattr(arc, "invert_direction", False):
        sweep -= 2 * math.pi
    am = a1 + sweep / 2
    r = (arc.p1.co - c).length
    return c + c.__class__((math.cos(am), math.sin(am))) * r


def _exists(constraints, type_, a, b=None):
    ia = a.slvs_index
    ib = b.slvs_index if b is not None else None
    for c in constraints.get_list(type_):
        idx = [x.slvs_index for x in c.entities()]
        if ia in idx and (ib is None or ib in idx):
            return True
    return False


def _state(context, sketch_i):
    sk = context.scene.sketcher.entities.get(sketch_i)
    sk.solve(context)
    return sk.solver_state, getattr(sk, "dof", None)


def _mm(context):
    return 1000.0 * (context.scene.unit_settings.scale_length or 1.0)


# ---------------------------------------------------------------------------
# Auto Constraints
# ---------------------------------------------------------------------------

def auto_constraints(context, sketch, tangent_angle=8.0, axis_angle=2.0,
                     contact_tolerance=1e-5, tangency=True, axes=True,
                     contacts=True, cleanup=True, dry_run=False):
    """Add the constraints the drawing already shows. Returns a report {kind: [details]}.

    tangent_angle: max deviation (degrees) for a joint to count as smooth
    axis_angle:    max deviation (degrees) of a line from horizontal / vertical
    contact_tolerance: distance (Blender units) under which two endpoints touch
    """
    S = context.scene.sketcher
    sse, cons = S.entities, S.constraints
    report = defaultdict(list)
    sk_i = sketch.slvs_index

    segs = [e for e in sse.all
            if getattr(e, "sketch", None) == sketch and not e.is_point()
            and isinstance(e, (SlvsLine2D, SlvsBezier, SlvsArc)) and not e.construction]

    # 1. redundancy: a point made coincident with an entity it is an endpoint of
    if cleanup:
        to_remove = []
        for c in cons.coincident:
            if c.sketch != sketch:
                continue
            a, b = c.entity1, c.entity2
            pt, ent = (a, b) if a.is_point() else (b, a)
            if pt.is_point() and not ent.is_point() and pt in getattr(ent, "connection_points", lambda: [])():
                to_remove.append(c)
        report["redundant removed"] = [f"{c.entity1.name} / {c.entity2.name}" for c in to_remove]
        if not dry_run:
            for c in to_remove:
                cons.remove(c)

    # 2. contacts: distinct endpoints at the same place
    ends = {}
    for e in segs:
        for p in e.connection_points():
            ends[p.slvs_index] = p
    if contacts:
        pts = list(ends.values())
        for i, p in enumerate(pts):
            for q in pts[i + 1:]:
                if (p.co - q.co).length <= contact_tolerance and not _exists(cons, "COINCIDENT", p, q):
                    report["coincident"].append(f"{p.name} = {q.name}")
                    if not dry_run:
                        cons.add_coincident(p.slvs_index, q.slvs_index, sketch=sk_i)

    # 3. tangency at smooth joints
    if tangency:
        by_point = defaultdict(list)
        for e in segs:
            for p in e.connection_points():
                by_point[p.slvs_index].append(e)
        for pi, ents in by_point.items():
            if len(ents) != 2:
                continue
            e1, e2 = ents
            if isinstance(e1, SlvsLine2D) and isinstance(e2, SlvsLine2D):
                continue  # two lines are a corner or collinear, never tangent
            pt = ends[pi]
            d1, d2 = _outgoing(e1, pt), _outgoing(e2, pt)
            if d1 is None or d2 is None:
                continue
            dev = math.degrees(math.acos(max(-1.0, min(1.0, -d1.dot(d2)))))
            if dev > tangent_angle:
                report["sharp corners kept"].append(f"{pt.name} ({dev:.0f}°)")
                continue
            if isinstance(e1, SlvsLine2D):  # the constraint wants a curve first
                e1, e2 = e2, e1
            if _exists(cons, "TANGENT", e1, e2):
                continue
            report["tangent"].append(f"{e1.name} ~ {e2.name} ({dev:.1f}°)")
            if not dry_run:
                cons.add_tangent(e1.slvs_index, e2.slvs_index, sketch=sk_i)

    # 4. horizontal / vertical lines
    if axes:
        for e in segs:
            if not isinstance(e, SlvsLine2D):
                continue
            v = e.p2.co - e.p1.co
            if v.length < 1e-9 or _exists(cons, "HORIZONTAL", e) or _exists(cons, "VERTICAL", e):
                continue
            a = abs(math.degrees(math.atan2(v.y, v.x))) % 180
            if min(a, 180 - a) <= axis_angle:
                report["horizontal"].append(e.name)
                if not dry_run:
                    cons.add_horizontal(e.slvs_index, sketch=sk_i)
            elif abs(a - 90) <= axis_angle:
                report["vertical"].append(e.name)
                if not dry_run:
                    cons.add_vertical(e.slvs_index, sketch=sk_i)

    if not dry_run:
        report["_state"] = list(_state(context, sk_i))
    return dict(report)


# ---------------------------------------------------------------------------
# Dimensions
# ---------------------------------------------------------------------------

class _Dimensioner:
    """Adds distance constraints one at a time, keeping only the useful ones."""

    def __init__(self, context, sketch, step_mm):
        self.context = context
        S = context.scene.sketcher
        self.sse, self.cons = S.entities, S.constraints
        self.sk_i = sketch.slvs_index
        self.mm = _mm(context)
        self.step = step_mm
        self.report = defaultdict(list)
        self.state, self.dof = _state(context, self.sk_i)

    def sketch(self):
        return self.sse.get(self.sk_i)

    def done(self):
        return self.dof == 0

    def _round(self, v):
        return round(v * self.mm / self.step) * self.step / self.mm

    def add(self, label, e1, e2=None, align="NONE"):
        if self.done():
            return False
        e1_i = e1 if isinstance(e1, int) else e1.slvs_index
        e2_i = None if e2 is None else (e2 if isinstance(e2, int) else e2.slvs_index)
        c = self.cons.add_distance(e1_i, e2_i, sketch=self.sk_i, init=True,
                                   **({"align": align} if align != "NONE" else {}))
        if abs(c.value) * self.mm < 1e-3:  # zero length: nothing to dimension
            self.cons.remove(c)
            return False
        c.value = self._round(c.value)
        value = c.value * self.mm
        if not self._keep(c):
            self.report["rejected"].append(label)
            return False
        self.report["added"].append(f"{label} = {value:g} mm")
        return True

    def _keep(self, c):
        """Keep constraint c only if the solver stays OKAY and loses a degree of freedom.

        Solved twice: a near-degenerate set can pass once and turn redundant once
        the geometry has converged.
        """
        state, dof = _state(self.context, self.sk_i)
        if state == "OKAY":
            state, dof = _state(self.context, self.sk_i)
        if state != "OKAY" or (self.dof is not None and dof is not None and dof >= self.dof):
            self.cons.remove(c)
            self.state, self.dof = _state(self.context, self.sk_i)
            return False
        self.state, self.dof = state, dof
        return True

    def add_axis(self, label, p1, p2, horizontal):
        """Geometric horizontal / vertical constraint between two points."""
        if self.done():
            return False
        add = self.cons.add_horizontal if horizontal else self.cons.add_vertical
        c = add(p1, p2, sketch=self.sk_i)
        if not self._keep(c):
            self.report["rejected"].append(label)
            return False
        self.report["added"].append(label)
        return True

    def segs(self):
        sk = self.sketch()
        return [e for e in self.sse.all if getattr(e, "sketch", None) == sk
                and not e.is_point() and not e.construction]

    def origin(self):
        """Fixed construction point at the origin (the Al-Tasrif pivot), created if needed."""
        sk = self.sketch()
        for p in self.sse.points2D:
            if p.sketch == sk and p.fixed and p.co.length < 1e-9:
                return p.slvs_index
        p = self.sse.add_point_2d((0.0, 0.0), sk, fixed=True, construction=True)
        p.name = "Origin"
        i = p.slvs_index
        self.state, self.dof = _state(self.context, self.sk_i)
        return i

    def endpoints(self):
        ends = {}
        for e in self.segs():
            if isinstance(e, SlvsEllipse):
                continue
            for p in e.connection_points():
                ends[p.slvs_index] = p.co.copy()
        return ends


def propose_dimensions(context, sketch, step_mm=0.05, lengths=True, ellipses=True,
                       anchor=True, overall=True, arrange=True):
    """Base dimensions with the drawing's values, rounded to step_mm.

    lengths:  length of every line
    ellipses: both semi-axes and the centre position
    anchor:   point nearest the origin, dimensioned from a fixed "Origin" point
    overall:  horizontal distance origin -> leftmost and rightmost endpoints
    Bezier handles stay free: the shape is the user's.
    Returns (report, solver state, degrees of freedom).
    """
    d = _Dimensioner(context, sketch, step_mm)
    if d.state != "OKAY":
        return {"rejected": [f"sketch not solved ({d.state}): run Auto Constraints first"]}, d.state, d.dof

    origin = d.origin() if (anchor or overall or ellipses) else None

    if lengths:
        for e in [e.slvs_index for e in d.segs() if isinstance(e, SlvsLine2D)]:
            d.add(f"length {d.sse.get(e).name}", e)

    if ellipses:
        for e_i in [e.slvs_index for e in d.segs() if isinstance(e, SlvsEllipse)]:
            el = d.sse.get(e_i)
            ct, pa, pb = el.ct.slvs_index, el.pa.slvs_index, el.pb.slvs_index
            d.add(f"{el.name} semi-axis a", ct, pa)
            d.add(f"{el.name} semi-axis b", ct, pb)
            d.add(f"{el.name} centre (horizontal)", origin, ct, "HORIZONTAL")
            d.add(f"{el.name} centre (vertical)", origin, ct, "VERTICAL")

    ends = d.endpoints()
    if ends and anchor:
        near = min(ends, key=lambda i: ends[i].length)
        n = d.sse.get(near).name
        d.add(f"position {n} (horizontal)", origin, near, "HORIZONTAL")
        d.add(f"position {n} (vertical)", origin, near, "VERTICAL")
    if ends and overall:
        for i in {min(ends, key=lambda i: ends[i].x), max(ends, key=lambda i: ends[i].x)}:
            d.add(f"overall {d.sse.get(i).name}", origin, i, "HORIZONTAL")

    state, dof = _state(context, d.sk_i)
    if arrange:
        arrange_dimensions(context, d.sketch())
    return dict(d.report), state, dof


def fully_constrain(context, sketch, step_mm=0.05, lock_handles=True):
    """Constrain the sketch until no degree of freedom is left, with the drawing's values.

    Order: base dimensions, ellipse rotation, curve joints (from the origin),
    Bezier handles (length, then direction), then any point still free.
    Returns (report, solver state, degrees of freedom).
    """
    report, state, dof = propose_dimensions(context, sketch, step_mm, arrange=False)
    if state != "OKAY":
        return report, state, dof
    d = _Dimensioner(context, sketch, step_mm)
    for k, v in report.items():
        d.report[k] += v
    origin = d.origin()

    # 1. ellipse rotation: offset of the first axis end across the axis direction
    for e_i in [e.slvs_index for e in d.segs() if isinstance(e, SlvsEllipse)]:
        el = d.sse.get(e_i)
        A, _B = el.axes()
        across = "VERTICAL" if abs(A.x) >= abs(A.y) else "HORIZONTAL"
        d.add(f"{el.name} rotation", el.ct.slvs_index, el.pa.slvs_index, across)

    # 2. joints between curves, from the origin
    for i in list(d.endpoints()):
        n = d.sse.get(i).name
        d.add(f"joint {n} (horizontal)", origin, i, "HORIZONTAL")
        d.add(f"joint {n} (vertical)", origin, i, "VERTICAL")

    # 3. Bezier handles: length from their endpoint, then direction
    if lock_handles:
        handles = []
        for e in d.segs():
            if isinstance(e, SlvsBezier):
                handles += [(e.p1.slvs_index, e.ctrl1.slvs_index), (e.p2.slvs_index, e.ctrl2.slvs_index)]
        for end_i, ctrl_i in handles:
            n = d.sse.get(ctrl_i).name
            d.add(f"handle {n} (length)", end_i, ctrl_i)
            # direction: the smaller component (length + larger component is
            # degenerate when the handle is nearly on an axis)
            v = d.sse.get(ctrl_i).co - d.sse.get(end_i).co
            small_is_x = abs(v.x) <= abs(v.y)
            small = abs(v.x) if small_is_x else abs(v.y)
            if small * d.mm < d.step:
                # handle on an axis: say so geometrically
                d.add_axis(f"handle {n} ({'vertical' if small_is_x else 'horizontal'})",
                           end_i, ctrl_i, horizontal=not small_is_x)
            else:
                d.add(f"handle {n} ({'horizontal' if small_is_x else 'vertical'} offset)",
                      end_i, ctrl_i, "HORIZONTAL" if small_is_x else "VERTICAL")

    # 4. anything still free: from the origin
    sk = d.sketch()
    for i in [p.slvs_index for p in d.sse.points2D if p.sketch == sk and not p.fixed]:
        if d.done():
            break
        n = d.sse.get(i).name
        d.add(f"point {n} (horizontal)", origin, i, "HORIZONTAL")
        d.add(f"point {n} (vertical)", origin, i, "VERTICAL")

    state, dof = _state(context, d.sk_i)
    arrange_dimensions(context, d.sketch())
    return dict(d.report), state, dof


# ---------------------------------------------------------------------------
# Arrange Dimensions
# ---------------------------------------------------------------------------

def _outline_samples(sse, sketch):
    """Points of the real geometry (sampled curves, handles excluded)."""
    from mathutils import Vector
    pts = []
    for e in sse.all:
        if getattr(e, "sketch", None) != sketch or e.is_point() or e.construction:
            continue
        if isinstance(e, SlvsLine2D):
            pts += [e.p1.co.copy(), e.p2.co.copy()]
        elif isinstance(e, SlvsBezier):
            a, b, c, d = e.p1.co, e.ctrl1.co, e.ctrl2.co, e.p2.co
            for k in range(17):
                t = k / 16
                u = 1 - t
                pts.append(u ** 3 * a + 3 * u * u * t * b + 3 * u * t * t * c + t ** 3 * d)
        elif isinstance(e, SlvsEllipse):
            A, B = e.axes()
            for k in range(24):
                t = 2 * math.pi * k / 24
                pts.append(e.ct.co + A * math.cos(t) + B * math.sin(t))
        elif hasattr(e, "ct") and hasattr(e, "radius"):
            for k in range(24):
                t = 2 * math.pi * k / 24
                pts.append(e.ct.co + Vector((math.cos(t), math.sin(t))) * e.radius)
    return pts


def arrange_dimensions(context, sketch, gap=None, spacing=None):
    """Place distance dimensions next to the sketch, outside, like on a drawing.

    - horizontal / vertical dimensions: just past the edge of the part, on the
      nearer side; a new row only when two dimensions overlap;
    - oblique or inner dimensions (semi-axes, slanted line): next to the element.
    gap, spacing in Blender units; default 4 % and 3.5 % of the part's diagonal.
    Changes no value, only the display. Returns the number of dimensions placed.
    """
    from mathutils import Vector

    S = context.scene.sketcher
    sse = S.entities
    ui = context.preferences.system.ui_scale or 1.0
    pts = _outline_samples(sse, sketch)
    if not pts:
        return 0
    lo = Vector((min(p.x for p in pts), min(p.y for p in pts)))
    hi = Vector((max(p.x for p in pts), max(p.y for p in pts)))
    diag = max((hi - lo).length, 1e-6)
    gap = gap or 0.04 * diag
    spacing = spacing or 0.035 * diag
    Mi = sketch.wp.matrix_basis.inverted()
    centre = (lo + hi) / 2

    def local(v3):
        v = Mi @ v3
        return Vector((v.x, v.y))

    rows = defaultdict(list)  # (axis, side) -> [(start, end, row)]
    count = 0
    infos = []
    for c in [c for c in S.constraints.distance if c.sketch == sketch]:
        B = c.matrix_basis()
        o3 = B @ Vector((0, 0, 0))
        scale = ((B @ Vector((0, 1, 0))) - o3).length
        if scale < 1e-12:
            continue
        o = local(o3)
        xd = local(B @ Vector((1, 0, 0))) - o
        yd = local(B @ Vector((0, 1, 0))) - o
        if xd.length < 1e-12 or yd.length < 1e-12:
            continue
        xd.normalize()
        yd.normalize()
        side = 1.0 if (o + yd - centre).length >= (o - yd - centre).length else -1.0
        horiz, vert = abs(xd.x) > 0.999, abs(xd.y) > 0.999
        infos.append((c, o, yd * side, side, scale, horiz, vert, abs(c.value) / 2))

    # 1. axis dimensions: rows past the edge of the part, no overlaps
    for c, o, n, side, scale, horiz, vert, half in sorted(
            [t for t in infos if t[5] or t[6]], key=lambda t: abs(t[0].value)):
        axis = "H" if horiz else "V"
        pos = o.y if horiz else o.x
        start = (o.x if horiz else o.y) - half
        end = (o.x if horiz else o.y) + half
        margin = 0.5 * spacing

        def place(nn):
            edge = (lo.y if nn.y < 0 else hi.y) if horiz else (lo.x if nn.x < 0 else hi.x)
            key = (axis, round(nn.x), round(nn.y))
            row = 0
            while any(r == row and not (end + margin < s0 or start - margin > e0)
                      for s0, e0, r in rows[key]):
                row += 1
            return abs(edge - pos) + gap + row * spacing, key, row

        # both sides are possible: keep the one where the dimension lands closer
        (dist, key, row), side = min([(place(n), side), (place(-n), -side)], key=lambda t: t[0][0])
        rows[key].append((start, end, row))
        c.draw_offset = side * dist / scale / ui
        c.draw_outset = 0.0
        count += 1

    # 2. oblique / inner dimensions: next to the element
    for c, o, n, side, scale, horiz, vert, half in [t for t in infos if not (t[5] or t[6])]:
        c.draw_offset = side * gap / scale / ui
        c.draw_outset = 0.0
        count += 1
    return count


# ---------------------------------------------------------------------------
# Operators
# ---------------------------------------------------------------------------

def _dof_text(state, dof):
    return f"solver {state}, {dof} degree(s) of freedom left"


class View3D_OT_slvs_auto_constraints(Operator):
    """Add the constraints your drawing already shows: tangency at smooth joints, horizontal / vertical lines, touching endpoints. Removes redundant constraints. Creates no geometry and no dimension"""

    bl_idname = Operators.AutoConstraints
    bl_label = "Auto Constraints"
    bl_options = {"REGISTER", "UNDO"}

    tangent_angle: FloatProperty(name="Smooth Joint Under", default=8.0, min=0.5, max=30.0,
                                 description="Deviation (degrees) under which a joint becomes tangent")
    axis_angle: FloatProperty(name="Axis Line Under", default=2.0, min=0.0, max=10.0,
                              description="Deviation (degrees) under which a line becomes horizontal / vertical")
    tangency: BoolProperty(name="Tangency", default=True)
    axes: BoolProperty(name="Horizontal / Vertical", default=True)
    contacts: BoolProperty(name="Touching Endpoints", default=True)
    cleanup: BoolProperty(name="Remove Redundant", default=True)

    @classmethod
    def poll(cls, context):
        return context.scene.sketcher.active_sketch is not None

    def execute(self, context: Context):
        report = auto_constraints(
            context, context.scene.sketcher.active_sketch, self.tangent_angle, self.axis_angle,
            tangency=self.tangency, axes=self.axes, contacts=self.contacts, cleanup=self.cleanup,
        )
        state, dof = report.pop("_state", (None, None))
        summary = ", ".join(f"{len(v)} {k}" for k, v in report.items() if v and k != "sharp corners kept")
        sharp = len(report.get("sharp corners kept", []))
        self.report({"INFO" if state in ("OKAY", None) else "WARNING"},
                    f"{summary or 'nothing to add'}; {sharp} sharp corner(s) kept; {_dof_text(state, dof)}")
        refresh(context)
        return {"FINISHED"}


class View3D_OT_slvs_auto_dimensions(Operator):
    """Add base dimensions with your drawing's values (rounded): line lengths, ellipse semi-axes and centre, anchor on the origin, overall length. A dimension the solver rejects is removed. Then edit the values: the drawing follows"""

    bl_idname = Operators.AutoDimensions
    bl_label = "Propose Dimensions"
    bl_options = {"REGISTER", "UNDO"}

    step_mm: FloatProperty(name="Rounding (mm)", default=0.05, min=0.001, max=5.0)
    lengths: BoolProperty(name="Line Lengths", default=True)
    ellipses: BoolProperty(name="Ellipses", default=True)
    anchor: BoolProperty(name="Anchor on Origin", default=True)
    overall: BoolProperty(name="Overall Length", default=True)

    @classmethod
    def poll(cls, context):
        return context.scene.sketcher.active_sketch is not None

    def execute(self, context: Context):
        report, state, dof = propose_dimensions(
            context, context.scene.sketcher.active_sketch, self.step_mm,
            self.lengths, self.ellipses, self.anchor, self.overall)
        added, rejected = len(report.get("added", [])), len(report.get("rejected", []))
        self.report({"INFO" if state == "OKAY" else "WARNING"},
                    f"{added} dimension(s) added, {rejected} rejected; {_dof_text(state, dof)}")
        refresh(context)
        return {"FINISHED"}


class View3D_OT_slvs_fully_constrain(Operator):
    """Constrain the sketch completely with your drawing's values (rounded): base dimensions, ellipse rotation, curve joints, Bezier handles, then any point still free. Only dimensions the solver accepts are kept"""

    bl_idname = Operators.FullyConstrain
    bl_label = "Fully Constrain Sketch"
    bl_options = {"REGISTER", "UNDO"}

    step_mm: FloatProperty(name="Rounding (mm)", default=0.05, min=0.001, max=5.0)
    lock_handles: BoolProperty(name="Lock Bezier Handles", default=True,
                               description="Dimension the Bezier handles too (otherwise the handles remain free)")

    @classmethod
    def poll(cls, context):
        return context.scene.sketcher.active_sketch is not None

    def execute(self, context: Context):
        report, state, dof = fully_constrain(
            context, context.scene.sketcher.active_sketch, self.step_mm, self.lock_handles)
        added, rejected = len(report.get("added", [])), len(report.get("rejected", []))
        level = "INFO" if state == "OKAY" and dof == 0 else "WARNING"
        self.report({level}, f"{added} dimension(s) added, {rejected} rejected; {_dof_text(state, dof)}")
        refresh(context)
        return {"FINISHED"}


class View3D_OT_slvs_arrange_dimensions(Operator):
    """Bring the sketch's dimensions close to the part: outside, evenly spaced, parallel dimensions stacked without overlaps"""

    bl_idname = Operators.ArrangeDimensions
    bl_label = "Arrange Dimensions"
    bl_options = {"REGISTER", "UNDO"}

    gap_mm: FloatProperty(name="Gap (mm)", default=0.0, min=0.0, max=200.0,
                          description="Distance between the part and the first dimension (0: automatic)")
    spacing_mm: FloatProperty(name="Spacing (mm)", default=0.0, min=0.0, max=200.0,
                              description="Spacing between stacked parallel dimensions (0: automatic)")

    @classmethod
    def poll(cls, context):
        return context.scene.sketcher.active_sketch is not None

    def execute(self, context: Context):
        mm = _mm(context)
        n = arrange_dimensions(context, context.scene.sketcher.active_sketch,
                               self.gap_mm / mm or None, self.spacing_mm / mm or None)
        self.report({"INFO"}, f"{n} dimension(s) arranged")
        refresh(context)
        return {"FINISHED"}


register, unregister = register_classes_factory((
    View3D_OT_slvs_auto_constraints,
    View3D_OT_slvs_auto_dimensions,
    View3D_OT_slvs_fully_constrain,
    View3D_OT_slvs_arrange_dimensions,
))
