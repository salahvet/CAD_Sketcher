import logging

from bpy.types import Context

from ..declarations import Operators
from ..stateful_operator.utilities.register import register_stateops_factory
from ..stateful_operator.state import state_from_args
from .constants import types_point_2d
from .utilities import ignore_hover
from .add_rectangle import View3D_OT_slvs_add_rectangle

logger = logging.getLogger(__name__)


class View3D_OT_slvs_add_rectangle_center(View3D_OT_slvs_add_rectangle):
    """Add a rectangle from its center to the active sketch"""

    bl_idname = Operators.AddRectangleCenter
    bl_label = "Add Center Rectangle"
    bl_options = {"REGISTER", "UNDO"}

    rect_state1_doc = ("Center", "Pick or place the rectangle center.")
    rect_state2_doc = ("Corner", "Pick or place a corner.")

    states = (
        state_from_args(
            rect_state1_doc[0],
            description=rect_state1_doc[1],
            pointer="p1",
            types=types_point_2d,
        ),
        state_from_args(
            rect_state2_doc[0],
            description=rect_state2_doc[1],
            pointer="p2",
            types=types_point_2d,
            interactive=True,
            # saisie numerique relative au centre -> demi-dimensions
            create_element="create_point",
        ),
    )

    def main(self, context: Context):
        sketch = self.sketch
        sse = context.scene.sketcher.entities
        construction = context.scene.sketcher.use_construction

        center, corner = self.get_point(context, 0), self.get_point(context, 1)
        cx, cy = center.co
        x, y = corner.co
        if abs(x - cx) < 1e-9 or abs(y - cy) < 1e-9:
            return False  # rectangle plat : on attend un autre mouvement

        # ATTENTION : ajouter a une collection peut la reallouer et rendre
        # caduques les references Python deja prises. On ne garde donc que
        # des index, et on relit l'entite (sse.get) juste avant usage.
        center_i, corner_i = center.slvs_index, corner.slvs_index
        get = sse.get

        # coins par symetrie centrale ; ordre : horizontal, vertical,
        # horizontal, vertical (comme l'outil rectangle d'origine)
        pts_i = [corner_i]
        for co in ((2 * cx - x, y), (2 * cx - x, 2 * cy - y), (x, 2 * cy - y)):
            pts_i.append(sse.add_point_2d(co, sketch).slvs_index)
        if construction:
            for i in pts_i:
                get(i).construction = True

        lines_i = []
        for k in range(4):
            line = sse.add_line_2d(get(pts_i[k]), get(pts_i[(k + 1) % 4]), sketch)
            line.construction = construction
            lines_i.append(line.slvs_index)

        # diagonales de construction, comme le rectangle centre des
        # logiciels de CAO ; le centre est tenu au milieu de la premiere
        diag_i = []
        for a, b in ((0, 2), (1, 3)):
            d = sse.add_line_2d(get(pts_i[a]), get(pts_i[b]), sketch)
            d.construction = True
            diag_i.append(d.slvs_index)

        for i in (*pts_i, *lines_i, *diag_i):
            ignore_hover(get(i))

        self._center_i = center_i
        self._diag_i = diag_i
        self.lines = [get(i) for i in lines_i]
        return True

    def fini(self, context: Context, succeede: bool):
        sse = context.scene.sketcher.entities
        if succeede and getattr(self, "_diag_i", None):
            context.scene.sketcher.constraints.add_midpoint(
                sse.get(self._center_i), sse.get(self._diag_i[0]),
                sketch=self.sketch,
            )
        super().fini(context, succeede)


register, unregister = register_stateops_factory(
    (View3D_OT_slvs_add_rectangle_center,)
)
