import logging
import math

from bpy.types import Operator, Context
from bpy.props import FloatProperty

from ..model.categories import SEGMENT
from ..model.ellipse import SlvsEllipse
from ..model.identifiers import is_line, is_circle
from ..declarations import Operators
from ..stateful_operator.utilities.register import register_stateops_factory
from ..stateful_operator.state import state_from_args
from ..utilities.view import refresh
from ..utilities.walker import EntityWalker
from ..utilities.intersect import get_offset_elements, get_intersections
from ..model.utilities import get_connection_point
from .base_2d import Operator2d
from .utilities import ignore_hover


logger = logging.getLogger(__name__)


def _get_offset_co(point, normal, distance):
    # For start or endpoint: get point translated by distance along normal
    return point.co + normal * distance


def _bool_to_signed_int(invert):
    return -1 if invert else 1

def _inverted_dist(invert, distance):
    sign = _bool_to_signed_int(invert) * _bool_to_signed_int(distance < 0)
    return math.copysign(distance, sign)


class View3D_OT_slvs_add_offset(Operator, Operator2d):
    """Copy and offset selected entities along with their constraints by the distance value"""

    bl_idname = Operators.Offset
    bl_label = "Offset"
    bl_options = {"REGISTER", "UNDO"}

    distance: FloatProperty(name="Distance")

    states = (
        state_from_args(
            "Entity",
            description="Base entity to get path from",
            pointer="entity",
            use_create=False,
            types=(*SEGMENT, SlvsEllipse),
        ),
        state_from_args(
            "Distance",
            description="Distance to offset the created entities",
            property="distance",
            interactive=True,
        ),
    )

    def main(self, context: Context):
        sketch = self.sketch
        entity = self.entity
        distance = self.distance
        sse = context.scene.sketcher.entities

        ignore_hover(entity)

        if is_circle(entity):
            c_new = entity.new(context, radius=entity.radius + distance)
            ignore_hover(c_new)

            refresh(context)
            return True

        if isinstance(entity, SlvsEllipse):
            return self._offset_ellipse(context, entity, distance)


        walker = EntityWalker(context.scene, sketch, entity=entity)
        path = walker.main_path()
        is_cyclic = walker.is_cyclic_path(path[0])

        if path is None:
            return False

        # Get intersections and create points
        points = []
        entities, directions = path
        self.entities = entities

        intersection_count = len(entities) if is_cyclic else len(entities) - 1
        point_coords = []
        for i in range(intersection_count):
            entity = entities[i]
            entity_dir = directions[i]
            neighbour_i = (i + 1) % len(entities)
            neighbour = entities[neighbour_i]
            neighbour_dir = directions[neighbour_i]
            point = get_connection_point(entity, neighbour)

            intersections = sorted(
                get_intersections(
                    get_offset_elements(entity, _inverted_dist(entity_dir, distance)),
                    get_offset_elements(neighbour, _inverted_dist(neighbour_dir, distance)),
                ),
                key=lambda i: (i - point.co).length,
            )

            has_bezier = "SlvsBezier" in (entity.type, neighbour.type)
            if has_bezier:
                # jonction tangente : les courbes decalees se touchent sans
                # se couper franchement -> point decale moyen
                d_e = _inverted_dist(entity_dir, distance)
                d_n = _inverted_dist(neighbour_dir, distance)
                fallback = (
                    _get_offset_co(point, entity.normal(position=point.co), d_e)
                    + _get_offset_co(point, neighbour.normal(position=point.co), d_n)
                ) / 2
                limit = abs(distance) * 4 + 1e-6
                if not intersections or (intersections[0] - point.co).length > limit:
                    intersections = [fallback]

            if not intersections:
                return False

            point_coords.append(intersections[0])

        points = [
            sse.add_point_2d(co, sketch, index_reference=True) for co in point_coords
        ]

        # Add start/endpoint if not cyclic
        if not is_cyclic:

            start, end = walker.get_limitpoints(path)
            start_co = _get_offset_co(
                start,
                entities[0].normal(position=start.co),
                _inverted_dist(directions[0], distance),
            )
            end_co = _get_offset_co(
                end,
                entities[-1].normal(position=end.co),
                _inverted_dist(directions[-1], distance),
            )

            points.insert(0, sse.add_point_2d(start_co, sketch, index_reference=True))
            points.append(sse.add_point_2d(end_co, sketch, index_reference=True))

        # Exclude created points from selection
        [ignore_hover(p) for p in points]

        # Create segments
        self.new_path = []
        for i, entity in enumerate(entities):
            direction = directions[i]

            i_start = (i - 1 if is_cyclic else i) % len(entities)
            i_end = (i_start + 1) % len(points)
            p1 = points[i_start]
            p2 = points[i_end]

            use_construction = context.scene.sketcher.use_construction
            if entity.type == "SlvsBezier":
                new_entity = self._offset_bezier(
                    context, entity, p1, p2, direction,
                    _inverted_dist(direction, distance), use_construction,
                )
                ignore_hover(new_entity)
                self.new_path.append(new_entity)
                continue
            new_entity = entity.new(
                context,
                p1=p1,
                p2=p2,
                sketch=sketch,
                **(
                    {"invert": direction} if hasattr(entity, "invert_direction") else {}
                ),
                construction=use_construction,
                index_reference=True,
            )
            ignore_hover(new_entity)

            self.new_path.append(new_entity)

        refresh(context)
        return True

    def _offset_bezier(self, context, bez, p1_i, p2_i, inverted, dist, construction):
        """Bezier decalee : poignees par Tiller-Hanson, recalees sur les
        jonctions deja calculees (p1_i, p2_i sont des index de points)."""
        sse = context.scene.sketcher.entities
        sketch = self.sketch
        o_p1, o_c1, o_c2, o_p2 = bez.offset_controls(dist)
        bez_i = bez.slvs_index
        # dans le sens de parcours, la nouvelle p1 correspond a l'ancienne p2
        if inverted:
            o_p1, o_c1, o_c2, o_p2 = o_p2, o_c2, o_c1, o_p1
        j1 = sse.get(p1_i).co.copy()
        j2 = sse.get(p2_i).co.copy()
        c1 = o_c1 + (j1 - o_p1)
        c2 = o_c2 + (j2 - o_p2)
        c1_i = sse.add_point_2d(c1, sketch, index_reference=True)
        c2_i = sse.add_point_2d(c2, sketch, index_reference=True)
        for i in (c1_i, c2_i):
            pt = sse.get(i)
            pt.construction = construction
            ignore_hover(pt)
        new = sse.add_bezier(
            p1_i, c1_i, c2_i, p2_i, sketch, construction=construction,
            index_reference=True,
        )
        return sse.get(new) if isinstance(new, int) else new

    def _offset_ellipse(self, context, ellipse, distance):
        """Ellipse concentrique, meme orientation, rayons + distance.

        Le decalage exact d'une ellipse n'est pas une ellipse : c'est
        l'approximation usuelle, fidele tant que la distance reste petite
        devant le petit rayon.
        """
        sse = context.scene.sketcher.entities
        sketch = self.sketch
        A, B = ellipse.axes()
        ra, rb = A.length + distance, B.length + distance
        if ra <= 1e-6 or rb <= 1e-6 or A.length < 1e-9 or B.length < 1e-9:
            return False  # decalage vers l'interieur plus grand que l'ellipse

        # index seulement : ajouter des points peut reallouer les collections
        ct_i = ellipse.ct.slvs_index
        c = ellipse.ct.co.copy()
        pa_i = sse.add_point_2d(c + A.normalized() * ra, sketch).slvs_index
        pb_i = sse.add_point_2d(c + B.normalized() * rb, sketch).slvs_index
        new = sse.add_ellipse(sse.get(ct_i), sse.get(pa_i), sse.get(pb_i), sketch)
        new.construction = context.scene.sketcher.use_construction
        new_i = new.slvs_index
        for i in (pa_i, pb_i, new_i):
            ignore_hover(sse.get(i))
        self.entities = []
        self.new_path = [sse.get(new_i)]
        refresh(context)
        return True

    def fini(self, context: Context, succeede: bool):
        if not succeede:
            return

        if self.sketch:
            self.sketch.geometry_solved = False

        constraints = context.scene.sketcher.constraints

        # Add parallel constraint
        # for entity, new_entity in zip(self.entities, self.new_path):
        #     if not is_line(entity):
        #         continue
        #     constraints.add_parallel(entity, new_entity, sketch=self.sketch)


register, unregister = register_stateops_factory((View3D_OT_slvs_add_offset,))
