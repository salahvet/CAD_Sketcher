import logging

import bpy
from bpy.types import PropertyGroup
from bpy.utils import register_classes_factory
import gpu
from gpu_extras.batch import batch_for_shader
from mathutils.geometry import interpolate_bezier

from ..solver import Solver
from .base_entity import Entity2D
from .utilities import slvs_entity_pointer, get_connection_point
from .constants import CURVE_RESOLUTION
from ..shaders import Shaders

logger = logging.getLogger(__name__)

# Batches des lignes de rappel (poignées), indexés par slvs_index
_handle_batches = {}


class SlvsBezier(Entity2D, PropertyGroup):
    """Cubic bezier segment on the sketch workplane, defined by 4 control
    points (p1, ctrl1, ctrl2, p2). Control points are regular sketch points,
    so every point constraint applies to them and the solver treats the
    segment as a native SLVS_E_CUBIC entity.
    """

    @classmethod
    def is_path(cls):
        return True

    @classmethod
    def is_segment(cls):
        return True

    def dependencies(self):
        return [self.p1, self.ctrl1, self.ctrl2, self.p2, self.sketch]

    def is_dashed(self):
        return self.construction

    def _control_locations(self):
        return (
            self.p1.location,
            self.ctrl1.location,
            self.ctrl2.location,
            self.p2.location,
        )

    def update(self):
        if bpy.app.background:
            return
        a, b, c, d = self._control_locations()
        coords = interpolate_bezier(a, b, c, d, CURVE_RESOLUTION)
        kwargs = {"pos": coords}
        self._batch = batch_for_shader(self._shader, "LINE_STRIP", kwargs)

        # Lignes de rappel des poignées : p1->ctrl1 et p2->ctrl2
        handle_shader = Shaders.uniform_color_line_3d()
        _handle_batches[self.slvs_index] = batch_for_shader(
            handle_shader, "LINES", {"pos": (a, b, d, c)}
        )
        self.is_dirty = False

    def draw(self, context):
        # Courbe principale (comportement standard)
        super().draw(context)

        # Poignées en pointillé, comme un éditeur de splines
        if not self.is_visible(context):
            return
        batch = _handle_batches.get(self.slvs_index)
        if not batch:
            return
        shader = Shaders.uniform_color_line_3d()
        shader.bind()
        gpu.state.blend_set("ALPHA")
        col = self.color(context)
        shader.uniform_float("color", (col[0], col[1], col[2], col[3] * 0.5))
        gpu.state.line_width_set(1.0)
        shader.uniform_bool("dashed", (True,))
        shader.uniform_float("dash_width", 0.03)
        shader.uniform_float("dash_factor", 0.4)
        batch.draw(shader)
        gpu.shader.unbind()
        self.restore_opengl_defaults()

    def create_slvs_data(self, solvesys, group=Solver.group_fixed):
        handle = solvesys.add_cubic(
            group,
            self.p1.py_data,
            self.ctrl1.py_data,
            self.ctrl2.py_data,
            self.p2.py_data,
            self.wp.py_data,
        )
        self.py_data = handle

    def placement(self):
        a, b, c, d = self._control_locations()
        return interpolate_bezier(a, b, c, d, 3)[1]

    def closest_picking_point(self, origin, view_vector):
        a, b, c, d = self._control_locations()
        points = interpolate_bezier(a, b, c, d, CURVE_RESOLUTION)
        vv = view_vector.normalized()
        best, best_d = points[0], None
        for p in points:
            rel = p - origin
            dist = (rel - rel.project(vv)).length
            if best_d is None or dist < best_d:
                best, best_d = p, dist
        return best

    def connection_points(self, direction: bool = False):
        points = [self.p1, self.p2]
        if direction:
            return list(reversed(points))
        return points

    def direction(self, point, is_endpoint=False):
        if is_endpoint:
            return point == self.p1
        return point == self.p2

    def tangent_toward(self, point):
        """Tangent at an endpoint, pointing toward the connection point"""
        if point == self.p1:
            vec = self.p1.co - self.ctrl1.co
        else:
            vec = self.p2.co - self.ctrl2.co
        if vec.length == 0.0:
            return None
        return vec.normalized()

    def connection_angle(self, other, connection_point=None, **kwargs):
        point = connection_point or get_connection_point(self, other)
        if not point:
            return None
        if self.is_3d() or other.is_3d():
            return None

        dir1 = self.tangent_toward(point)
        if dir1 is None:
            return None

        if other.is_line():
            dir2 = (
                other.direction_vec()
                if other.direction(point)
                else (other.direction_vec() * (-1))
            )
        elif hasattr(other, "tangent_toward"):
            dir2 = other.tangent_toward(point)
        else:
            return None
        if dir2 is None:
            return None
        return dir1.angle_signed(dir2)

    def to_bezier(
        self, spline, startpoint, endpoint, invert_direction, set_startpoint=False
    ):
        locations = [
            self.p1.co.to_3d(),
            self.ctrl1.co.to_3d(),
            self.ctrl2.co.to_3d(),
            self.p2.co.to_3d(),
        ]
        if invert_direction:
            locations.reverse()

        if set_startpoint:
            startpoint.co = locations[0]
        endpoint.co = locations[3]

        startpoint.handle_right = locations[1]
        endpoint.handle_left = locations[2]

        return endpoint

    def midpoint(self):
        return self.placement()

    def replace_point(self, old, new):
        for ptr in ("p1", "ctrl1", "ctrl2", "p2"):
            if old != getattr(self, ptr):
                continue
            setattr(self, ptr, new)
            break

    def new(self, context, **kwargs):
        kwargs.setdefault("p1", self.p1)
        kwargs.setdefault("ctrl1", self.ctrl1)
        kwargs.setdefault("ctrl2", self.ctrl2)
        kwargs.setdefault("p2", self.p2)
        kwargs.setdefault("sketch", self.sketch)
        kwargs.setdefault("construction", self.construction)
        return context.scene.sketcher.entities.add_bezier(**kwargs)


slvs_entity_pointer(SlvsBezier, "p1")
slvs_entity_pointer(SlvsBezier, "ctrl1")
slvs_entity_pointer(SlvsBezier, "ctrl2")
slvs_entity_pointer(SlvsBezier, "p2")
slvs_entity_pointer(SlvsBezier, "sketch")

register, unregister = register_classes_factory((SlvsBezier,))
