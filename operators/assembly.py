"""Fork: assembly tools for jointed instruments (forceps, scissors, needle holders).

- Assembly Pivot: mounts two branches on their joint axis, the way Al-Tasrif expects the
  model: a "Pivot" Empty at the joint (model origin), one Empty per moving branch with an
  action named "ouverture..." (first frame closed, last frame fully open) and a Limit
  Rotation constraint so the branch cannot be turned past the maximum by hand.
- Check Interference: common volume of two parts (exact for OCCT parts, contact test for
  plain meshes), at the current pose or over the whole opening; reports the free opening
  angle and can limit the opening to it.
"""
import math

import bpy
from bpy.props import BoolProperty, EnumProperty, FloatProperty, IntProperty, StringProperty
from bpy.types import Context, Operator
from bpy.utils import register_classes_factory
from mathutils import Matrix, Vector

from ..declarations import Operators
from . import occt_ops

AXES = [("Z", "World Z", "Joint axis perpendicular to the top view (usual case)"),
        ("Y", "World Y", ""), ("X", "World X", "")]


def _mesh_objects(self, context):
    items = [(o.name, o.name, "") for o in context.scene.objects if o.type == "MESH"
             and not o.name.endswith("_Edges")]
    _mesh_objects.cache = items or [("NONE", "<no mesh>", "")]
    return _mesh_objects.cache


def _axis_matrix(axis, centre):
    """World matrix of an Empty at centre whose local Z is the joint axis."""
    rot = {"Z": Matrix.Identity(3),
           "Y": Matrix.Rotation(-math.pi / 2, 3, "X"),
           "X": Matrix.Rotation(math.pi / 2, 3, "Y")}[axis]
    m = rot.to_4x4()
    m.translation = centre
    return m


def _new_empty(context, name, matrix, parent=None, display="PLAIN_AXES", size=0.01):
    e = bpy.data.objects.new(name, None)
    e.empty_display_type = display
    e.empty_display_size = size
    context.scene.collection.objects.link(e)
    if parent is not None:
        e.parent = parent
        e.matrix_parent_inverse = Matrix.Identity(4)
        e.matrix_local = parent.matrix_world.inverted() @ matrix
    else:
        e.matrix_world = matrix
    return e


def _attach(child, parent):
    """Parent keeping the world transform."""
    mw = child.matrix_world.copy()
    child.parent = parent
    child.matrix_parent_inverse = parent.matrix_world.inverted()
    child.matrix_world = mw


def _animate(empty, angle, frame_start, frame_end, action_name):
    empty.rotation_mode = "XYZ"
    empty.rotation_euler = (0, 0, 0)
    empty.keyframe_insert("rotation_euler", index=2, frame=frame_start)
    empty.rotation_euler = (0, 0, angle)
    empty.keyframe_insert("rotation_euler", index=2, frame=frame_end)
    empty.rotation_euler = (0, 0, 0)
    act = empty.animation_data.action
    act.name = action_name
    for fc in _fcurves(act):
        for kp in fc.keyframe_points:
            kp.interpolation = "LINEAR"
    lim = empty.constraints.new("LIMIT_ROTATION")
    lim.name = "Opening limit"
    lim.use_limit_z = True
    lim.min_z, lim.max_z = (0.0, angle) if angle >= 0 else (angle, 0.0)
    lim.owner_space = "LOCAL"


def _fcurves(action):
    """F-curves of an action (Blender 4.4+ layered actions keep them in channelbags)."""
    if hasattr(action, "fcurves") and len(getattr(action, "fcurves", [])):
        return list(action.fcurves)
    out = []
    for layer in getattr(action, "layers", []):
        for strip in layer.strips:
            for bag in getattr(strip, "channelbags", []):
                out += list(bag.fcurves)
    return out


def make_pivot(context, branch_a, branch_b, axis="Z", centre=(0, 0, 0), max_angle=math.radians(30),
               mode="BOTH", flip=False, frame_start=1, frame_end=24, name="Pivot"):
    """Build the pivot assembly; returns the root Empty."""
    m = _axis_matrix(axis, Vector(centre))
    root = _new_empty(context, name, m, display="ARROWS", size=0.008)
    root["altasrif_pivot"] = True
    root["max_angle_deg"] = math.degrees(max_angle)
    s = -1.0 if flip else 1.0
    plan = [(branch_a, s * max_angle / 2), (branch_b, -s * max_angle / 2)] if mode == "BOTH" \
        else [(branch_a, 0.0), (branch_b, -s * max_angle)]
    for obj, angle in plan:
        hinge = _new_empty(context, f"{name}.{obj.name}", m, parent=root, size=0.004)
        _attach(obj, hinge)
        if abs(angle) > 1e-12:
            _animate(hinge, angle, frame_start, frame_end, f"ouverture_{obj.name}")
    return root


# ---------------------------------------------------------------------------
# Interference
# ---------------------------------------------------------------------------

def _located_solid(obj):
    """OCCT solid of the part placed where its object is now (parts are built at identity)."""
    import build123d as b3d
    from OCP.gp import gp_Trsf
    solid = occt_ops._solid_cache.get(obj.name)
    if solid is None:
        return None
    m = obj.matrix_world
    t = gp_Trsf()
    t.SetValues(m[0][0], m[0][1], m[0][2], m[0][3], m[1][0], m[1][1], m[1][2], m[1][3],
                m[2][0], m[2][1], m[2][2], m[2][3])
    return solid.moved(b3d.Location(t))


def _mesh_contact(context, a, b):
    """Contact between two plain meshes (triangle overlap, world space)."""
    from mathutils.bvhtree import BVHTree
    dg = context.evaluated_depsgraph_get()

    def tree(o):
        me = o.evaluated_get(dg).data
        mw = o.matrix_world
        return BVHTree.FromPolygons([mw @ v.co for v in me.vertices], [tuple(p.vertices) for p in me.polygons])

    ta, tb = tree(a), tree(b)
    if ta.overlap(tb):
        return True

    def inside(tree_, o):
        # one vertex of o inside the other mesh: nearest surface point faces away from it
        me = o.evaluated_get(dg).data
        if not len(me.vertices):
            return False
        p = o.matrix_world @ me.vertices[0].co
        hit = tree_.find_nearest(p)
        return hit[0] is not None and (p - hit[0]).dot(hit[1]) < 0

    return inside(ta, b) or inside(tb, a)


def interference(context, a, b):
    """(volume in mm3 or None, contact bool). Exact common volume for OCCT parts."""
    context.view_layer.update()
    sa, sb = _located_solid(a), _located_solid(b)
    mm3 = (1000.0 * (context.scene.unit_settings.scale_length or 1.0)) ** 3
    if sa is not None and sb is not None:
        try:
            common = sa & sb
            v = float(getattr(common, "volume", 0.0) or 0.0) * mm3
        except Exception:
            v = 0.0
        return v, v > 1e-3
    return None, _mesh_contact(context, a, b)


def relative_angle(a, b):
    """Opening angle between the two branches (degrees, around the joint axis)."""
    r = (b.matrix_world.to_3x3().normalized().inverted() @ a.matrix_world.to_3x3().normalized())
    return abs(math.degrees(r.to_euler().z))


def _pivot_of(obj):
    o = obj
    while o is not None:
        if o.get("altasrif_pivot"):
            return o
        o = o.parent
    return None


def _frame_range(root):
    frames = []
    for hinge in root.children:
        ad = hinge.animation_data
        if ad and ad.action:
            for fc in _fcurves(ad.action):
                frames += [kp.co.x for kp in fc.keyframe_points]
    return (int(min(frames)), int(max(frames))) if frames else None


def sweep_interference(context, a, b, steps=12):
    """Interference over the opening: list of (frame, angle deg, volume or None, contact)."""
    root = _pivot_of(a) or _pivot_of(b)
    rng = _frame_range(root) if root else None
    scene = context.scene
    if rng is None:
        v, c = interference(context, a, b)
        return [(scene.frame_current, relative_angle(a, b), v, c)]
    keep = scene.frame_current
    out = []

    def at(f):
        scene.frame_set(int(math.floor(f)), subframe=f - math.floor(f))
        v, c = interference(context, a, b)
        return (f, relative_angle(a, b), v, c)

    try:
        for k in range(steps + 1):
            out.append(at(rng[0] + (rng[1] - rng[0]) * k / steps))
        # first contact refined by bisection between the last free and the first colliding pose
        first = next((i for i, s in enumerate(out) if s[3]), None)
        if first:
            lo, hi = out[first - 1], out[first]
            for _ in range(10):
                mid = at((lo[0] + hi[0]) / 2)
                lo, hi = (lo, mid) if mid[3] else (mid, hi)
            out = out[:first] + [lo, hi] + out[first:]
            out.sort(key=lambda s: s[0])
    finally:
        scene.frame_set(keep)
    return out


def limit_opening(root, factor):
    """Scale every hinge's end keyframe (and its rotation limit) by factor (0..1)."""
    for hinge in root.children:
        ad = hinge.animation_data
        if not (ad and ad.action):
            continue
        for fc in _fcurves(ad.action):
            kps = sorted(fc.keyframe_points, key=lambda k: k.co.x)
            if len(kps) >= 2:
                kps[-1].co.y *= factor
                kps[-1].handle_left.y *= factor
                kps[-1].handle_right.y *= factor
                fc.update()
                end = kps[-1].co.y
                for con in hinge.constraints:
                    if con.type == "LIMIT_ROTATION":
                        con.min_z, con.max_z = (0.0, end) if end >= 0 else (end, 0.0)
    root["max_angle_deg"] = root.get("max_angle_deg", 0.0) * factor


# ---------------------------------------------------------------------------
# Operators
# ---------------------------------------------------------------------------

class View3D_OT_slvs_assembly_pivot(Operator):
    """Mount two branches on their joint axis: a Pivot Empty at the joint (model origin for Al-Tasrif)
    and an "ouverture" action per moving branch, first frame closed, last frame fully open"""

    bl_idname = Operators.AssemblyPivot
    bl_label = "Assembly Pivot"
    bl_options = {"REGISTER", "UNDO"}

    branch_a: EnumProperty(name="Branch A", items=_mesh_objects)
    branch_b: EnumProperty(name="Branch B", items=_mesh_objects)
    axis: EnumProperty(name="Joint Axis", items=AXES, default="Z")
    center: EnumProperty(name="Through", items=[("ORIGIN", "World Origin (pivot)", ""),
                                                ("CURSOR", "3D Cursor", "")], default="ORIGIN")
    max_angle: FloatProperty(name="Max Opening", default=math.radians(30), min=math.radians(1),
                             max=math.radians(180), subtype="ANGLE")
    mode: EnumProperty(name="Moving", items=[("BOTH", "Both Branches", "Each branch turns half the angle"),
                                             ("B_ONLY", "Branch B Only", "Branch A stays fixed")], default="BOTH")
    flip: BoolProperty(name="Flip Direction", default=False, description="Open the other way")
    frame_end: IntProperty(name="Open at Frame", default=24, min=2)

    @classmethod
    def poll(cls, context):
        return context.mode == "OBJECT"

    def invoke(self, context, event):
        sel = [o for o in context.selected_objects if o.type == "MESH"]
        if len(sel) >= 2:
            self.branch_a, self.branch_b = sel[0].name, sel[1].name
        return context.window_manager.invoke_props_dialog(self, width=300)

    def execute(self, context: Context):
        a, b = context.scene.objects.get(self.branch_a), context.scene.objects.get(self.branch_b)
        if a is None or b is None or a == b:
            self.report({"ERROR"}, "Choose two different branches")
            return {"CANCELLED"}
        centre = context.scene.cursor.location.copy() if self.center == "CURSOR" else Vector((0, 0, 0))
        root = make_pivot(context, a, b, self.axis, centre, self.max_angle, self.mode, self.flip,
                          1, self.frame_end)
        self.report({"INFO"}, f"{root.name}: {math.degrees(self.max_angle):.0f}° opening, frames 1 → {self.frame_end}")
        return {"FINISHED"}


class View3D_OT_slvs_check_interference(Operator):
    """Check whether two parts overlap: exact common volume for OCCT parts, contact for plain meshes.
    Over the whole opening when they are mounted on an Assembly Pivot"""

    bl_idname = Operators.CheckInterference
    bl_label = "Check Interference"
    bl_options = {"REGISTER", "UNDO"}

    part_a: EnumProperty(name="Part A", items=_mesh_objects)
    part_b: EnumProperty(name="Part B", items=_mesh_objects)
    sweep: BoolProperty(name="Over the Opening", default=True,
                        description="Test along the whole opening animation (otherwise the current pose only)")
    steps: IntProperty(name="Steps", default=12, min=2, max=200)
    limit: BoolProperty(name="Limit Opening to Free Angle", default=False,
                        description="If the parts collide while opening, stop the opening just before")

    @classmethod
    def poll(cls, context):
        return context.mode == "OBJECT"

    def invoke(self, context, event):
        sel = [o for o in context.selected_objects if o.type == "MESH"]
        if len(sel) >= 2:
            self.part_a, self.part_b = sel[0].name, sel[1].name
        return context.window_manager.invoke_props_dialog(self, width=300)

    def execute(self, context: Context):
        a, b = context.scene.objects.get(self.part_a), context.scene.objects.get(self.part_b)
        if a is None or b is None or a == b:
            self.report({"ERROR"}, "Choose two different parts")
            return {"CANCELLED"}
        if self.sweep:
            samples = sweep_interference(context, a, b, self.steps)
        else:
            v, c = interference(context, a, b)
            samples = [(context.scene.frame_current, relative_angle(a, b), v, c)]
        hits = [s for s in samples if s[3]]
        if not hits:
            self.report({"INFO"}, f"No interference ({len(samples)} pose(s) checked, up to {samples[-1][1]:.1f}°)")
            return {"FINISHED"}
        worst = max(hits, key=lambda s: s[2] or 0.0)
        what = f"{worst[2]:.3f} mm³" if worst[2] is not None else "contact"
        first = hits[0]
        free = [s for s in samples if s[0] < first[0]]
        msg = f"Interference: {what} at {worst[1]:.1f}°; first at {first[1]:.1f}°"
        if first is samples[0]:
            msg += " (already in the closed position: check the joint / clearance)"
        elif free:
            msg += f"; free opening up to {free[-1][1]:.1f}°"
            root = _pivot_of(a) or _pivot_of(b)
            if self.limit and root and samples[-1][1] > 1e-9:
                limit_opening(root, free[-1][1] / samples[-1][1])
                msg += " — opening limited to it"
        self.report({"WARNING"}, msg)
        return {"FINISHED"}


register, unregister = register_classes_factory((View3D_OT_slvs_assembly_pivot,
                                                 View3D_OT_slvs_check_interference))
