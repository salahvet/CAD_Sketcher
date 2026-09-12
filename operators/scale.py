# --- correctif PureMed : rotation ---
#
# Mise a l echelle, sur le meme modele que rotate.py et move.py.
# Le facteur suit l eloignement du curseur au pivot, rapporte a sa
# distance de depart : s eloigner agrandit, se rapprocher retrecit.

from bpy.types import Operator, Context, Event
from bpy.props import FloatProperty, BoolProperty
from mathutils import Vector

from ..declarations import Operators
from .base_2d import Operator2d
from ..stateful_operator.state import state_from_args
from ..stateful_operator.utilities.register import register_stateops_factory
from ..solver import solve_system
from ..utilities.view import get_pos_2d
from .rotate import get_points


class View3D_OT_slvs_scale(Operator, Operator2d):
    """Scale selected entities around their center, independent of constraints"""

    bl_idname = Operators.Scale
    bl_label = "Scale Entities"
    bl_options = {"UNDO", "REGISTER"}

    factor: FloatProperty(
        name="Factor", default=1.0, options={"SKIP_SAVE"}
    )
    use_cursor: BoolProperty(
        name="Pivot on 3D Cursor", default=False, options={"SKIP_SAVE"}
    )

    states = (
        state_from_args(
            "Factor",
            description="Scale factor to apply to the selection of entities",
            property="factor",
            state_func="get_factor",
            interactive=True,
        ),
    )

    def _releve(self, context: Context):
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
        d = (Vector(pos) - self._piv).length if pos is not None else 0.0
        # une reference trop petite rendrait le geste explosif
        self._ref = d if d > 1e-6 else 1e-6
        return retval

    def get_factor(self, context: Context, coords):
        pos = get_pos_2d(context, self.sketch.wp, coords)
        if pos is None:
            return None
        return (Vector(pos) - self._piv).length / self._ref

    def main(self, context: Context):
        if not hasattr(self, "_dep"):
            self._releve(context)
        k = self.factor
        for point, o in zip(self._pts, self._dep):
            if point.fixed:
                continue
            point.co = (self._piv.x + (o.x - self._piv.x) * k,
                        self._piv.y + (o.y - self._piv.y) * k)
        return {"FINISHED"}

    def fini(self, context: Context, succeede: bool):
        if succeede:
            if self.sketch:
                self.sketch.geometry_solved = False
            solve_system(context, sketch=self.sketch)


register, unregister = register_stateops_factory((View3D_OT_slvs_scale,))
