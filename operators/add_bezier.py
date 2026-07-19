import logging

from bpy.types import Operator, Context, Event

from ..declarations import Operators
from ..stateful_operator.utilities.register import register_stateops_factory
from ..stateful_operator.state import state_from_args
from ..solver import solve_system
from .base_2d import Operator2d
from .constants import types_point_2d
from .utilities import ignore_hover

logger = logging.getLogger(__name__)


class View3D_OT_slvs_add_bezier2d(Operator, Operator2d):
    """Add a cubic bezier segment to the active sketch.
    Click the 4 points of the control polygon: start, control 1,
    control 2, end."""

    bl_idname = Operators.AddBezier2D
    bl_label = "Add Solvespace 2D Bezier"
    bl_options = {"REGISTER", "UNDO"}

    bezier_state1_doc = ("Startpoint", "Pick or place the startpoint.")
    bezier_state2_doc = ("Control 1", "Pick or place the first control point.")
    bezier_state3_doc = ("Control 2", "Pick or place the second control point.")
    bezier_state4_doc = ("Endpoint", "Pick or place the endpoint.")

    states = (
        state_from_args(
            bezier_state1_doc[0],
            description=bezier_state1_doc[1],
            pointer="p1",
            types=types_point_2d,
        ),
        state_from_args(
            bezier_state2_doc[0],
            description=bezier_state2_doc[1],
            pointer="ctrl1",
            types=types_point_2d,
            allow_prefill=False,
        ),
        state_from_args(
            bezier_state3_doc[0],
            description=bezier_state3_doc[1],
            pointer="ctrl2",
            types=types_point_2d,
            allow_prefill=False,
        ),
        state_from_args(
            bezier_state4_doc[0],
            description=bezier_state4_doc[1],
            pointer="p2",
            types=types_point_2d,
            interactive=True,
        ),
    )

    def solve_state(self, context: Context, _event: Event):
        sketch = context.scene.sketcher.active_sketch
        solve_system(context, sketch=sketch)
        return True

    def main(self, context):
        p1, ctrl1, ctrl2, p2 = (
            self.get_point(context, 0),
            self.get_point(context, 1),
            self.get_point(context, 2),
            self.get_point(context, 3),
        )
        sketch = self.sketch
        sse = context.scene.sketcher.entities
        bez = sse.add_bezier(p1, ctrl1, ctrl2, p2, sketch)

        ignore_hover(bez)
        self.target = bez
        if context.scene.sketcher.use_construction:
            self.target.construction = True
        return True

    def fini(self, context: Context, succeede: bool):
        if hasattr(self, "target"):
            logger.debug("Add: {}".format(self.target))
        if succeede:
            if self.has_coincident():
                self.solve_state(context, self.sketch)
            self.sketch.geometry_solved = False


register, unregister = register_stateops_factory((View3D_OT_slvs_add_bezier2d,))
