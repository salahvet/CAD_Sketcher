import logging

from bpy.types import PropertyGroup
from bpy.utils import register_classes_factory

from ..solver import Solver
from ..global_data import WpReq
from .base_constraint import GenericConstraint
from .utilities import slvs_entity_pointer, make_coincident, get_connection_point
from .categories import CURVE
from .line_2d import SlvsLine2D
from .arc import SlvsArc
from .circle import SlvsCircle
from .bezier import SlvsBezier

logger = logging.getLogger(__name__)


class SlvsTangent(GenericConstraint, PropertyGroup):
    """Forces two curves (arc/circle) or a curve and a line to be tangent."""

    type = "TANGENT"
    label = "Tangent"
    signature = (CURVE, (SlvsLine2D, *CURVE))

    def needs_wp(self):
        return WpReq.NOT_FREE


    def create_slvs_data(self, solvesys, group=Solver.group_fixed):
        e1, e2 = self.entity1, self.entity2
        wp = self.get_workplane()

        # --- Fork : tangence des béziers via contraintes points/lignes ---
        # (le chemin générique solvesys.tangent est instable avec les cubiques)
        def _bezier_ctrl_at(bez, point):
            """Point de contrôle adjacent à l'extrémité 'point' de la bézier."""
            if point == bez.p1:
                return bez.ctrl1
            if point == bez.p2:
                return bez.ctrl2
            return None

        beziers = [e for e in (e1, e2) if isinstance(e, SlvsBezier)]
        if beziers:
            point = get_connection_point(e1, e2)
            CIRCLE_ARC_B = (SlvsCircle, SlvsArc)
            if point is None and len(beziers) == 1:
                bez = beziers[0]
                other = e2 if bez == e1 else e1
                if type(other) in CIRCLE_ARC_B:
                    # spline <-> cercle/arc sans point commun : l'extrémité la
                    # plus proche est posée sur la courbe, sa poignée est
                    # perpendiculaire au rayon en ce point
                    if isinstance(other, SlvsCircle):
                        r = other.radius
                    else:
                        r = (other.p1.co - other.ct.co).length

                    def _gap(pt):
                        return abs((pt.co - other.ct.co).length - r)

                    end = min((bez.p1, bez.p2), key=_gap)
                    ctrl = _bezier_ctrl_at(bez, end)
                    radius = solvesys.add_line_2d(
                        group, other.ct.py_data, end.py_data, wp
                    )
                    handle = solvesys.add_line_2d(
                        group, end.py_data, ctrl.py_data, wp
                    )
                    return (
                        make_coincident(solvesys, end.py_data, other, wp, group),
                        solvesys.perpendicular(
                            group, radius, handle, workplane=wp
                        ),
                    )
            if point is None:
                logger.warning("Tangent (bezier): entities must share a point")
                return None

            if len(beziers) == 2:
                # spline <-> spline : ctrl_a, jonction, ctrl_b alignés
                ca = _bezier_ctrl_at(e1, point)
                cb = _bezier_ctrl_at(e2, point)
                if ca is None or cb is None:
                    return None
                line = solvesys.add_line_2d(group, ca.py_data, cb.py_data, wp)
                return solvesys.coincident(group, point.py_data, line, wp)

            bez = beziers[0]
            other = e2 if bez == e1 else e1
            ctrl = _bezier_ctrl_at(bez, point)
            if ctrl is None:
                return None

            if other.is_line():
                # spline <-> ligne : le ctrl adjacent sur le prolongement de la ligne
                return solvesys.coincident(group, ctrl.py_data, other.py_data, wp)

            if type(other) in (SlvsCircle, SlvsArc):
                # spline <-> arc/cercle : poignée perpendiculaire au rayon
                radius = solvesys.add_line_2d(
                    group, other.ct.py_data, point.py_data, wp
                )
                handle = solvesys.add_line_2d(
                    group, point.py_data, ctrl.py_data, wp
                )
                return solvesys.perpendicular(group, radius, handle, workplane=wp)
            return None

        CIRCLE_ARC = (SlvsCircle, SlvsArc)
        if type(e1) in CIRCLE_ARC and e2.is_line():
            orig = e2.p1.co
            coords = (e1.ct.co - orig).project(e2.p2.co - orig) + orig
            p = solvesys.add_point_2d(group, *coords, wp)
            line = solvesys.add_line_2d(group, e1.ct.py_data, p, wp)
            return (
                make_coincident(solvesys, p, e1, wp, group),
                make_coincident(solvesys, p, e2, wp, group),
                solvesys.perpendicular(group, e2.py_data, line, workplane=wp),
            )
        elif type(e1) in CIRCLE_ARC and type(e2) in CIRCLE_ARC:
            coords = (e1.ct.co + e2.ct.co) / 2
            p = solvesys.add_point_2d(group, *coords, wp)
            line = solvesys.add_line_2d(group, e1.ct.py_data, e2.ct.py_data, wp)

            return (
                make_coincident(solvesys, p, e1, wp, group),
                make_coincident(solvesys, p, e2, wp, group),
                solvesys.coincident(group, p, line, wp)
            )

        return solvesys.tangent(group, e2.py_data, e1.py_data, wp)


    def placements(self):
        point = get_connection_point(self.entity1, self.entity2)
        if point is None:
            return (self.entity1, self.entity2)
        return (point,)


slvs_entity_pointer(SlvsTangent, "entity1")
slvs_entity_pointer(SlvsTangent, "entity2")
slvs_entity_pointer(SlvsTangent, "sketch")

register, unregister = register_classes_factory((SlvsTangent,))
