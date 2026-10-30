from enum import Enum
from typing import Callable, Tuple

from mathutils.geometry import (
    intersect_line_line_2d as intersect_segment_segment_2d,
    intersect_line_sphere_2d as intersect_segment_sphere_2d,
    intersect_sphere_sphere_2d,
)

from .geometry import intersect_line_line_2d, intersect_line_sphere_2d
from ..model.base_entity import SlvsGenericEntity
from .data_handling import to_list


class ElementTypes(str, Enum):
    Line = "LINE"
    Sphere = "Sphere"
    Polyline = "POLYLINE"  # fork : Bezier decalee


def _get_intersection_func(type_a, type_b, segment=False):
    if all([t == ElementTypes.Line for t in (type_a, type_b)]):
        return intersect_line_line_2d if not segment else intersect_segment_segment_2d
    if all([t == ElementTypes.Sphere for t in (type_a, type_b)]):
        return intersect_sphere_sphere_2d
    return intersect_line_sphere_2d if not segment else intersect_segment_sphere_2d


def _order_intersection_args(arg1, arg2):
    if arg1[0] == ElementTypes.Sphere and arg2[0] == ElementTypes.Line:
        return arg2, arg1
    return arg1, arg2


# TODO: Move to entity classes
# -> entity.get_offset_props()


def _get_offset_line(line, offset):
    normal = line.normal()
    offset_vec = normal * offset
    return (line.p1.co + offset_vec, line.p2.co + offset_vec)


def _get_offset_sphere(arc, offset):
    """Return sphere_co and sphere_radius of offset sphere"""
    return arc.ct.co, arc.radius + offset


# Note: for arcs the radius might increse or decrease depending of the curvature


def get_offset_elements(
    entity: SlvsGenericEntity, offset: float
) -> Tuple[ElementTypes, Callable]:
    if entity.type == "SlvsBezier":
        return (ElementTypes.Polyline, (entity.offset_polyline(offset),))
    t = ElementTypes.Line if entity.type == "SlvsLine2D" else ElementTypes.Sphere
    func = _get_offset_sphere if t == ElementTypes.Sphere else _get_offset_line
    return (t, func(entity, offset))


def _polyline_intersections(a, b):
    """Intersections d'une polyligne (a) avec une ligne infinie, un cercle
    ou une autre polyligne (b)."""
    pts = a[1][0]
    out = []
    for p, q in zip(pts, pts[1:]):
        if b[0] == ElementTypes.Line:
            co = intersect_line_line_2d(p, q, *b[1])
            if co is None:
                continue
            # le point doit tomber sur le segment p-q (la ligne reste infinie)
            pq = q - p
            L2 = pq.length_squared
            if L2 > 0 and -1e-9 <= (co - p).dot(pq) / L2 <= 1 + 1e-9:
                out.append(co)
        elif b[0] == ElementTypes.Sphere:
            for co in to_list(intersect_segment_sphere_2d(p, q, *b[1])):
                if co:
                    out.append(co)
        else:
            for r, s in zip(b[1][0], b[1][0][1:]):
                co = intersect_segment_segment_2d(p, q, r, s)
                if co:
                    out.append(co)
    return out


def get_intersections(*element_list, segment=False):
    """Find all intersections between all combinations of elements, (type, element)"""
    intersections = []
    lenght = len(element_list)

    for i, elem_a in enumerate(element_list):
        if i + 1 == lenght:
            break
        for elem_b in element_list[i + 1 :]:
            if ElementTypes.Polyline in (elem_a[0], elem_b[0]):
                a, b = (elem_a, elem_b) if elem_a[0] == ElementTypes.Polyline else (elem_b, elem_a)
                intersections.extend(_polyline_intersections(a, b))
                continue
            a, b = _order_intersection_args(elem_a, elem_b)
            func = _get_intersection_func(a[0], b[0], segment=segment)

            retval = to_list(func(*a[1], *b[1]))

            for intr in retval:
                if not intr:
                    continue
                intersections.append(intr)
    return intersections
