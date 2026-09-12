# --- correctif PureMed : rotation ---
#
# Ecrit sur le modele de move.py : meme base Operator2d, meme machine a
# etats, meme appel au solveur. Seule la transformation change.
#
# Le pivot est le barycentre des points concernes, releve a l invocation.
# ALT au lancement prend le curseur 3D a la place.

import math

from bpy.types import Operator, Context, Event
from bpy.props import FloatProperty, BoolProperty
from mathutils import Vector

from ..declarations import Operators
from .base_2d import Operator2d
from ..stateful_operator.state import state_from_args
from ..stateful_operator.utilities.register import register_stateops_factory
from ..utilities.data_handling import get_flat_deps
from ..solver import solve_system
from ..utilities.view import get_pos_2d


def get_points(context: Context):
    """Les points 2D selectionnes, ou dont depend une entite selectionnee."""
    entities = context.scene.sketcher.entities.selected_active
    points = []

    def add(p):
        if p not in points:
            points.append(p)

    def is_point2d(e):
        return bool(e) and e.is_2d() and e.is_point()

    for entity in entities:
        if not entity:
            continue
        if is_point2d(entity):
            add(entity)
            continue
        for e in get_flat_deps(entity):
            if is_point2d(e):
                add(e)
    return points


class View3D_OT_slvs_rotate(Operator, Operator2d):
    """Rotate selected entities around their center, independent of constraints"""

    bl_idname = Operators.Rotate
    bl_label = "Rotate Entities"
    bl_options = {"UNDO", "REGISTER"}

    angle: FloatProperty(
        name="Angle", subtype="ANGLE", default=0.0, options={"SKIP_SAVE"}
    )
    use_cursor: BoolProperty(
        name="Pivot on 3D Cursor", default=False, options={"SKIP_SAVE"}
    )

    states = (
        state_from_args(
            "Angle",
            description="Rotation angle to apply to the selection of entities",
            property="angle",
            state_func="get_angle",
            interactive=True,
        ),
    )

    def _releve(self, context: Context):
        """Points concernes, leurs positions de depart, et le pivot."""
        self._pts = get_points(context)
        self._dep = [Vector((p.co[0], p.co[1])) for p in self._pts]
        if self.use_cursor and self.sketch:
            wp = self.sketch.wp
            loc = wp.matrix_basis.inverted() @ context.scene.cursor.location
            self._piv = Vector((loc.x, loc.y))
        elif self._dep:
            s = Vector((0.0, 0.0))
            for v in self._dep:
                s += v
            self._piv = s / len(self._dep)
        else:
            self._piv = Vector((0.0, 0.0))

    def invoke(self, context: Context, event: Event):
        coords = Vector((event.mouse_region_x, event.mouse_region_y))
        self.use_cursor = event.alt
        retval = super().invoke(context, event)
        self._releve(context)
        pos = get_pos_2d(context, self.sketch.wp, coords)
        d = Vector(pos) - self._piv if pos is not None else Vector((1.0, 0.0))
        self._ref = math.atan2(d.y, d.x) if d.length > 1e-9 else 0.0
        return retval

    def get_angle(self, context: Context, coords):
        pos = get_pos_2d(context, self.sketch.wp, coords)
        if pos is None:
            return None
        d = Vector(pos) - self._piv
        if d.length < 1e-9:
            return None
        return math.atan2(d.y, d.x) - self._ref

    def main(self, context: Context):
        if not hasattr(self, "_dep"):
            self._releve(context)
        c, s = math.cos(self.angle), math.sin(self.angle)
        for point, o in zip(self._pts, self._dep):
            if point.fixed:
                continue
            dx, dy = o.x - self._piv.x, o.y - self._piv.y
            point.co = (self._piv.x + dx * c - dy * s,
                        self._piv.y + dx * s + dy * c)
        return {"FINISHED"}

    def fini(self, context: Context, succeede: bool):
        if succeede:
            if self.sketch:
                self.sketch.geometry_solved = False
            solve_system(context, sketch=self.sketch)


register, unregister = register_stateops_factory((View3D_OT_slvs_rotate,))
