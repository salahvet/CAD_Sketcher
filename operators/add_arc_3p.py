import logging
import math

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

# En deca de ce determinant les trois points sont alignes : pas de cercle.
TOL_COLLINEAR = 1e-9
TAU = 2.0 * math.pi


def circumcenter_2d(a, b, c):
    """Centre du cercle passant par trois points du plan.

    Retourne None si les points sont alignes, auquel cas aucun arc fini
    ne passe par les trois.
    """
    ax, ay = a.x, a.y
    bx, by = b.x, b.y
    cx, cy = c.x, c.y

    d = 2.0 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    if abs(d) < TOL_COLLINEAR:
        return None

    a2 = ax * ax + ay * ay
    b2 = bx * bx + by * by
    c2 = cx * cx + cy * cy

    ux = (a2 * (by - cy) + b2 * (cy - ay) + c2 * (ay - by)) / d
    uy = (a2 * (cx - bx) + b2 * (ax - cx) + c2 * (bx - ax)) / d
    return Vector((ux, uy))


def _sweep_ccw(angle_from, angle_to):
    """Balayage antihoraire de angle_from vers angle_to, dans [0, 2pi)."""
    return (angle_to - angle_from) % TAU


def arc_passes_through(center, start, end, through):
    """Vrai si le sens antihoraire start -> end passe par le point vise.

    C'est ce qui distingue l'arc mineur de l'arc majeur : les trois points
    definissent le meme cercle, le troisieme dit lequel des deux arcs garder.
    """
    a1 = math.atan2(start.y - center.y, start.x - center.x)
    a2 = math.atan2(end.y - center.y, end.x - center.x)
    a3 = math.atan2(through.y - center.y, through.x - center.x)
    return _sweep_ccw(a1, a3) < _sweep_ccw(a1, a2)


class View3D_OT_slvs_add_arc_3p_2d(Operator, Operator2d):
    """Add an arc through three points to the active sketch"""

    bl_idname = Operators.AddArc3P2D
    bl_label = "Add Solvespace 2D Arc (3 Points)"
    bl_options = {"REGISTER", "UNDO"}

    arc_state1_doc = ("Startpoint", "Pick or place the arc start point.")
    arc_state2_doc = ("Endpoint", "Pick or place the arc end point.")
    arc_state3_doc = ("Through", "Pick or place a point the arc passes through.")

    states = (
        state_from_args(
            arc_state1_doc[0],
            description=arc_state1_doc[1],
            pointer="p1",
            types=types_point_2d,
        ),
        state_from_args(
            arc_state2_doc[0],
            description=arc_state2_doc[1],
            pointer="p2",
            types=types_point_2d,
            allow_prefill=False,
        ),
        state_from_args(
            arc_state3_doc[0],
            description=arc_state3_doc[1],
            pointer="p3",
            types=types_point_2d,
            interactive=True,
        ),
    )

    def solve_state(self, context: Context, _event: Event):
        sketch = context.scene.sketcher.active_sketch
        solve_system(context, sketch=sketch)
        return True

    def main(self, context: Context):
        p1 = self.get_point(context, 0)
        p2 = self.get_point(context, 1)
        p3 = self.get_point(context, 2)
        sketch = self.sketch
        sse = context.scene.sketcher.entities

        center = circumcenter_2d(p1.co, p2.co, p3.co)
        if center is None:
            # Points alignes : on n'invente pas un arc, on laisse
            # l'utilisateur bouger la souris.
            return False

        # Le centre est une consequence des trois points, jamais saisi
        # directement : il est cree en construction pour ne pas encombrer
        # la geometrie exportee.
        ct = sse.add_point_2d(tuple(center), sketch, construction=True)
        ignore_hover(ct)

        arc = sse.add_arc(sketch.wp.nm, ct, p1, p2, sketch)
        arc.invert_direction = not arc_passes_through(center, p1.co, p2.co, p3.co)

        ignore_hover(arc)
        self.center = ct
        self.through = p3
        self.target = arc

        if context.scene.sketcher.use_construction:
            arc.construction = True
        return True

    def fini(self, context: Context, succeede: bool):
        if not succeede:
            return

        arc = getattr(self, "target", None)
        through = getattr(self, "through", None)
        if arc is None or through is None:
            return

        logger.debug("Add: {}".format(arc))

        # Sans cette contrainte le troisieme point ne servirait qu'au
        # placement initial : l'arc oublierait son passage par ce point des
        # la premiere resolution. Avec elle, deplacer l'un des trois points
        # met l'arc a jour.
        ssc = context.scene.sketcher.constraints
        try:
            ssc.add_coincident(through, arc, sketch=self.sketch)
        except Exception as exc:
            logger.warning("Arc 3 points : contrainte non posee (%s)", exc)

        self.sketch.geometry_solved = False
        self.solve_state(context, None)


register, unregister = register_stateops_factory((View3D_OT_slvs_add_arc_3p_2d,))
