import logging

from bpy.types import Operator, Context, Event
from mathutils import Vector

from ..declarations import Operators
from ..stateful_operator.utilities.register import register_stateops_factory
from ..stateful_operator.state import state_from_args
from ..solver import solve_system
from .base_2d import Operator2d
from .constants import types_point_2d
from .utilities import ignore_hover

logger = logging.getLogger(__name__)

# En deca, l'axe ou le second rayon est nul : pas d'ellipse.
TOL_DEGENERATE = 1e-9


def project_on_normal(ct, pa, p):
    """Projette p sur la perpendiculaire a (ct, pa) passant par ct.

    Retourne None si l'axe (ct, pa) est nul ou si le second rayon l'est.
    """
    axis = pa - ct
    if axis.length < TOL_DEGENERATE:
        return None
    n = Vector((-axis.y, axis.x)).normalized()
    d = (p - ct).dot(n)
    if abs(d) < TOL_DEGENERATE:
        return None
    return ct + n * d


class View3D_OT_slvs_add_ellipse_2d(Operator, Operator2d):
    """Add an ellipse to the active sketch: center, first axis end,
    then second radius"""

    bl_idname = Operators.AddEllipse2D
    bl_label = "Add Solvespace 2D Ellipse"
    bl_options = {"REGISTER", "UNDO"}

    ellipse_state1_doc = ("Center", "Pick or place the ellipse center.")
    ellipse_state2_doc = ("Axis", "Pick or place the end of the first axis.")
    ellipse_state3_doc = ("Second radius", "Place the end of the second axis.")

    states = (
        state_from_args(
            ellipse_state1_doc[0],
            description=ellipse_state1_doc[1],
            pointer="ct",
            types=types_point_2d,
        ),
        state_from_args(
            ellipse_state2_doc[0],
            description=ellipse_state2_doc[1],
            pointer="pa",
            types=types_point_2d,
            allow_prefill=False,
        ),
        state_from_args(
            ellipse_state3_doc[0],
            description=ellipse_state3_doc[1],
            pointer="pb",
            types=types_point_2d,
            interactive=True,
        ),
    )

    def solve_state(self, context: Context, _event: Event):
        sketch = context.scene.sketcher.active_sketch
        solve_system(context, sketch=sketch)
        return True

    def main(self, context: Context):
        ct = self.get_point(context, 0)
        pa = self.get_point(context, 1)
        pb = self.get_point(context, 2)
        sketch = self.sketch
        sse = context.scene.sketcher.entities

        co = project_on_normal(ct.co, pa.co, pb.co)
        if co is None:
            # Ellipse degeneree : on laisse l'utilisateur bouger la souris.
            return False

        # Le troisieme point est pose exactement sur la perpendiculaire :
        # la contrainte interne est satisfaite des la creation, le solveur
        # n'a rien a deplacer.
        pb.co = co

        ellipse = sse.add_ellipse(ct, pa, pb, sketch)
        ignore_hover(ellipse)
        self.target = ellipse

        if context.scene.sketcher.use_construction:
            ellipse.construction = True
        return True

    def fini(self, context: Context, succeede: bool):
        if hasattr(self, "target"):
            logger.debug("Add: {}".format(self.target))
        if succeede:
            self.sketch.geometry_solved = False
            self.solve_state(context, None)


register, unregister = register_stateops_factory((View3D_OT_slvs_add_ellipse_2d,))
