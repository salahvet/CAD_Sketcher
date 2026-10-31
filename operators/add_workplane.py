import logging

import math

import bpy
from bpy.props import BoolProperty, EnumProperty, FloatProperty
from bpy.types import Operator, Context
from mathutils import Matrix, Vector

from .. import global_data
from ..model.types import SlvsNormal3D
from ..model.categories import NORMAL3D

from ..utilities.geometry import get_face_orientation
from ..declarations import Operators
from ..stateful_operator.utilities.register import register_stateops_factory
from ..stateful_operator.state import state_from_args
from ..stateful_operator.utilities.geometry import get_evaluated_obj, get_mesh_element
from ..solver import solve_system
from .base_3d import Operator3d
from .constants import types_point_3d
from .utilities import ignore_hover, activate_sketch
from ..utilities.view import get_placement_pos

logger = logging.getLogger(__name__)


class View3D_OT_slvs_add_workplane(Operator, Operator3d):
    """Add a workplane"""

    bl_idname = Operators.AddWorkPlane
    bl_label = "Add Solvespace Workplane"
    bl_options = {"REGISTER", "UNDO"}

    wp_state1_doc = ("Origin", "Pick or place workplanes's origin.")
    wp_state2_doc = ("Orientation", "Set workplane's orientation.")

    states = (
        state_from_args(
            wp_state1_doc[0],
            description=wp_state1_doc[1],
            pointer="p1",
            types=types_point_3d,
        ),
        state_from_args(
            wp_state2_doc[0],
            description=wp_state2_doc[1],
            state_func="get_orientation",
            pointer="nm",
            types=NORMAL3D,
            interactive=True,
            create_element="create_normal3d",
        ),
    )

    def get_normal(self, context: Context, index: int):
        states = self.get_states_definition()
        state = states[index]
        data = self._state_data[index]
        type = data["type"]
        sse = context.scene.sketcher.entities

        if type == bpy.types.MeshPolygon:
            ob_name, nm_index = self.get_state_pointer(index=index, implicit=True)
            ob = bpy.data.objects[ob_name]
            return sse.add_ref_normal_3d(ob, nm_index)
        return getattr(self, state.pointer)

    def get_orientation(self, context: Context, coords):
        # TODO: also support edges
        data = self.state_data
        ob, type, index = get_mesh_element(context, coords, edge=False, face=True)

        p1 = self.get_point(context, 0)
        mousepos = get_placement_pos(context, coords)
        vec = mousepos - p1.location
        return global_data.Z_AXIS.rotation_difference(vec).to_euler()

    def create_normal3d(self, context: Context, values, state, state_data):
        sse = context.scene.sketcher.entities

        v = values[0].to_quaternion()
        nm = sse.add_normal_3d(v)
        state_data["type"] = SlvsNormal3D
        return nm.slvs_index

    def main(self, context: Context):
        sse = context.scene.sketcher.entities
        p1 = self.get_point(context, 0)
        nm = self.get_normal(context, 1)
        self.target = sse.add_workplane(p1, nm)
        ignore_hover(self.target)
        return True

    def fini(self, context: Context, succeede: bool):
        if hasattr(self, "target"):
            logger.debug("Add: {}".format(self.target))

        if succeede:
            if self.has_coincident():
                solve_system(context)


def _snap_axis(n, tol_deg=0.5):
    """A normal within tol_deg of a world axis becomes exactly that axis (tessellation noise)."""
    for ax in (Vector((1, 0, 0)), Vector((0, 1, 0)), Vector((0, 0, 1))):
        for a in (ax, -ax):
            if n.angle(a) < math.radians(tol_deg):
                return a.copy()
    return n


def face_frame(normal, point, origin_mode="PIVOT", snap_deg=0.5):
    """Workplane frame for a face: (origin, quaternion).

    Sketch X follows world X when possible (else world Y), Y = normal x X, so a side face reads
    like the side view and the top face like the top view. The origin is the projection of the
    world origin (the instrument's pivot) onto the plane, or the picked point."""
    n = _snap_axis(normal.normalized(), snap_deg)
    for ref in (Vector((1, 0, 0)), Vector((0, 1, 0))):
        x = ref - n * ref.dot(n)
        if x.length > 1e-6:
            break
    x.normalize()
    y = n.cross(x)
    quat = Matrix((x, y, n)).transposed().to_quaternion()
    if origin_mode == "PIVOT":
        origin = n * point.dot(n)  # projection of the world origin onto the plane
    else:
        origin = point
    return origin, quat


def _brep_face_normal(ob, point):
    """Exact normal of the OCCT B-rep face under point, or None (not an OCCT part)."""
    try:
        from . import occt_ops
        import build123d as b3d
    except Exception:
        return None, False
    solid = occt_ops._solid_cache.get(ob.name)
    if solid is None or not ob.get("occt_part"):
        return None, False
    p = b3d.Vector(*point)
    face = min(solid.faces(), key=lambda f: f.distance_to(p))
    try:
        nv = face.normal_at(p)
    except Exception:
        nv = face.normal_at(face.center())
    planar = face.geom_type == b3d.GeomType.PLANE
    m = ob.matrix_world.to_3x3()  # parts are built in world space; follow the object if moved
    return (m @ Vector((nv.X, nv.Y, nv.Z))).normalized(), planar


class View3D_OT_slvs_add_workplane_face(Operator, Operator3d):
    """Add a workplane on a mesh face: exact face of an OCCT part, origin at the projection of the
    pivot (world origin), sketch X along world X when possible"""

    bl_idname = Operators.AddWorkPlaneFace
    bl_label = "Add Solvespace Workplane"
    bl_options = {"REGISTER", "UNDO"}

    wp_face_state1_doc = (
        "Face",
        "Pick a mesh face to use as workplanes's transformation.",
    )

    states = (
        state_from_args(
            wp_face_state1_doc[0],
            description=wp_face_state1_doc[1],
            use_create=False,
            pointer="face",
            types=(bpy.types.MeshPolygon,),
            interactive=True,
        ),
    )

    origin_mode: EnumProperty(
        name="Origin",
        items=[("PIVOT", "Pivot Projection", "Projection of the world origin (the pivot) onto the face plane"),
               ("FACE", "Picked Face", "Centre of the picked mesh face (former behaviour)")],
        default="PIVOT",
    )
    snap_angle: FloatProperty(
        name="Snap to World Axis", default=math.radians(5.0), min=0.0, max=math.radians(45.0),
        subtype="ANGLE",
        description="A face normal closer than this to a world axis becomes that axis "
                    "(a curved side then gives a clean side-view plane); 0 keeps the exact normal",
    )
    start_sketch: BoolProperty(
        name="Start Sketch", default=True,
        description="Open a new sketch on the workplane right away (view aligned)",
    )

    def main(self, context: Context):
        sse = context.scene.sketcher.entities

        ob_name, face_index = self.get_state_pointer(index=0, implicit=True)
        ob = get_evaluated_obj(context, bpy.data.objects[ob_name])
        mesh = ob.data
        face = mesh.polygons[face_index]

        mat_obj = ob.matrix_world
        pos = mat_obj @ face.center
        tri_n = (mat_obj.to_3x3() @ (get_face_orientation(mesh, face) @ Vector((0, 0, 1)))).normalized()
        normal, planar = _brep_face_normal(bpy.data.objects[ob_name], pos)
        if normal is None:
            normal = tri_n
        elif normal.dot(tri_n) < 0:
            normal = -normal  # keep the outward side the user clicked
        snap = math.degrees(self.snap_angle)
        snapped = _snap_axis(normal.normalized(), snap) != normal.normalized()
        if planar is False and bpy.data.objects[ob_name].get("occt_part") and not snapped:
            self.report({"WARNING"}, "Curved face: the workplane is its tangent plane at the picked point")
        origin_co, quat = face_frame(normal, pos, self.origin_mode, snap)
        if self.origin_mode == "FACE":
            origin_co = pos
        origin = sse.add_point_3d(origin_co)
        nm = sse.add_normal_3d(quat)

        self.target = sse.add_workplane(origin, nm)
        ignore_hover(self.target)
        self._sketch_i = -1
        if self.start_sketch:
            wp_i = self.target.slvs_index
            sketch = sse.add_sketch(sse.get(wp_i))
            sk_i = sketch.slvs_index
            p = sse.add_point_2d((0.0, 0.0), sse.get(sk_i))  # fixed origin, like Add Sketch
            p.fixed = True
            self._sketch_i = sk_i
            self.target = sse.get(wp_i)
        context.area.tag_redraw() # Force re-draw of UI (Blender doesn't update after tool usage)
        return True

    def fini(self, context: Context, succeede: bool):
        if succeede and getattr(self, "_sketch_i", -1) >= 0:
            activate_sketch(context, self._sketch_i, self)
            wp = context.scene.sketcher.entities.get(self._sketch_i).wp
            wp.visible = False


register, unregister = register_stateops_factory(
    (View3D_OT_slvs_add_workplane, View3D_OT_slvs_add_workplane_face)
)
