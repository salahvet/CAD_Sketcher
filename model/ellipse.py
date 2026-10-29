import logging
import math
from typing import List

import bpy
import gpu
from bpy.types import PropertyGroup
from bpy.utils import register_classes_factory
from gpu_extras.batch import batch_for_shader
from mathutils import Vector

from ..solver import Solver
from ..shaders import Shaders
from .base_entity import SlvsGenericEntity, Entity2D
from .utilities import slvs_entity_pointer
from .constants import CURVE_RESOLUTION

logger = logging.getLogger(__name__)

# Batches des axes (pointilles), indexes par slvs_index
_axis_batches = {}

# Constante des poignees pour approcher un quart d'ellipse par une cubique
KAPPA = 4.0 / 3.0 * math.tan(math.pi / 8.0)


class SlvsEllipse(Entity2D, PropertyGroup):
    """Ellipse du croquis (fork).

    SolveSpace n'a pas d'entite ellipse : comme pour SlvsBezier, la forme
    est portee par de vrais points du croquis, ce qui rend toutes les
    contraintes de points utilisables sur elle.

        ct : centre
        pa : extremite du premier demi-axe
        pb : extremite du second demi-axe

    Une contrainte interne garde (ct, pa) perpendiculaire a (ct, pb) :
    6 parametres - 1 = 5 degres de liberte, ceux d'une ellipse
    (centre, deux rayons, orientation).
    """

    @classmethod
    def is_path(cls):
        return True

    @classmethod
    def is_closed(cls):
        return True

    # Volontairement False : is_curve() suppose un rayon et un centre
    # circulaire (distance, arc.intersect...), is_segment() ouvrirait Trim
    # qui n'est pas encore gere pour l'ellipse.

    def dependencies(self) -> List[SlvsGenericEntity]:
        return [self.ct, self.pa, self.pb, self.sketch]

    def is_dashed(self):
        return self.construction

    # --- geometrie ---------------------------------------------------------

    def axes(self):
        """Demi-axes (vecteurs 2D) A = pa - ct et B, rendu orthogonal a A.

        Le solveur garde A et B perpendiculaires ; on reprojette B ici pour
        que le trace reste une vraie ellipse meme avant la resolution.
        """
        ct = self.ct.co
        A = self.pa.co - ct
        B = self.pb.co - ct
        if A.length > 1e-12:
            n = Vector((-A.y, A.x)).normalized()
            B = n * B.dot(n)
        return A, B

    @property
    def radius_a(self):
        return (self.pa.co - self.ct.co).length

    @property
    def radius_b(self):
        return self.axes()[1].length

    def point_on_curve(self, t):
        """Point 2D au parametre t : ct + A cos t + B sin t"""
        A, B = self.axes()
        return self.ct.co + A * math.cos(t) + B * math.sin(t)

    def _derivative(self, t):
        A, B = self.axes()
        return -A * math.sin(t) + B * math.cos(t)

    def _coords_2d(self, resolution=None):
        n = resolution or max(CURVE_RESOLUTION * 2, 64)
        return [self.point_on_curve(2.0 * math.pi * i / n) for i in range(n + 1)]

    def _to_3d(self, co2d):
        return self.wp.matrix_basis @ co2d.to_3d()

    # --- dessin ------------------------------------------------------------

    def update(self):
        if bpy.app.background:
            return
        coords = [self._to_3d(co)[:] for co in self._coords_2d()]
        self._batch = batch_for_shader(self._shader, "LINE_STRIP", {"pos": coords})

        # axes ct->pa et ct->pb en pointilles
        ct = self.ct.location
        _axis_batches[self.slvs_index] = batch_for_shader(
            Shaders.uniform_color_line_3d(),
            "LINES",
            {"pos": (ct, self.pa.location, ct, self.pb.location)},
        )
        self.is_dirty = False

    def draw(self, context):
        super().draw(context)

        if not self.is_visible(context):
            return
        batch = _axis_batches.get(self.slvs_index)
        if not batch:
            return
        shader = Shaders.uniform_color_line_3d()
        shader.bind()
        gpu.state.blend_set("ALPHA")
        col = self.color(context)
        shader.uniform_float("color", (col[0], col[1], col[2], col[3] * 0.4))
        gpu.state.line_width_set(1.0)
        shader.uniform_bool("dashed", (True,))
        shader.uniform_float("dash_width", 0.03)
        shader.uniform_float("dash_factor", 0.4)
        batch.draw(shader)
        gpu.shader.unbind()
        self.restore_opengl_defaults()

    # --- solveur -----------------------------------------------------------

    def create_slvs_data(self, solvesys, group=Solver.group_fixed):
        wp = self.wp.py_data
        axis_a = solvesys.add_line_2d(group, self.ct.py_data, self.pa.py_data, wp)
        axis_b = solvesys.add_line_2d(group, self.ct.py_data, self.pb.py_data, wp)
        # Contrainte interne : les deux axes restent perpendiculaires
        solvesys.perpendicular(group, axis_a, axis_b, workplane=wp)
        # L'entite est representee par son premier axe : c'est ce handle que
        # recevraient d'eventuelles contraintes ligne <-> ellipse.
        self.py_data = axis_a

    # --- placement / selection ---------------------------------------------

    def placement(self):
        return self._to_3d(self.point_on_curve(math.pi / 4.0))

    def midpoint(self):
        return self.placement()

    def closest_picking_point(self, origin, view_vector):
        vv = view_vector.normalized()
        best, best_d = None, None
        for co in self._coords_2d():
            p = self._to_3d(co)
            rel = p - origin
            dist = (rel - rel.project(vv)).length
            if best_d is None or dist < best_d:
                best, best_d = p, dist
        return best

    def project_point(self, coords):
        """Point de l'ellipse le plus proche de coords (2D)."""
        n = 256
        t = min(
            (2.0 * math.pi * i / n for i in range(n)),
            key=lambda u: (self.point_on_curve(u) - coords).length,
        )
        step = 2.0 * math.pi / n
        for _ in range(24):
            t = min(
                (t - step, t, t + step),
                key=lambda u: (self.point_on_curve(u) - coords).length,
            )
            step *= 0.5
        return self.point_on_curve(t)

    # --- topologie (comme SlvsCircle : contour ferme sans extremites) ------

    def connection_points(self, direction: bool = False):
        return []

    def direction(self, point, is_endpoint=False):
        return False

    def overlaps_endpoint(self, co):
        return False

    def intersect(self, other, extended: bool = False):
        # Trim non gere pour l'ellipse pour l'instant
        return ()

    # --- conversion en courbe Blender --------------------------------------

    def bezier_segment_count(self):
        return 4

    def bezier_point_count(self):
        return self.bezier_segment_count()

    def to_bezier(
        self,
        spline,
        startpoint,
        endpoint,
        invert_direction,
        set_startpoint=False,
        midpoints=[],
    ):
        """4 cubiques, une par quart : P(t) +/- KAPPA * P'(t)."""
        segment_count = len(midpoints) + 1
        points = [startpoint, *midpoints, startpoint]
        step = 2.0 * math.pi / segment_count
        sign = -1.0 if invert_direction else 1.0
        k = KAPPA * 4.0 / segment_count  # KAPPA est defini pour un quart

        for i in range(segment_count):
            t0 = sign * step * i
            t1 = sign * step * (i + 1)
            b0, b1 = points[i], points[i + 1]
            p0, p1 = self.point_on_curve(t0), self.point_on_curve(t1)
            d0 = self._derivative(t0) * sign
            d1 = self._derivative(t1) * sign
            b0.co = p0.to_3d()
            b0.handle_right = (p0 + d0 * k).to_3d()
            b1.handle_left = (p1 - d1 * k).to_3d()
            b1.co = p1.to_3d()
        return endpoint

    # --- edition -----------------------------------------------------------

    def replace_point(self, old, new):
        for ptr in ("ct", "pa", "pb"):
            if old != getattr(self, ptr):
                continue
            setattr(self, ptr, new)
            break

    def new(self, context, **kwargs):
        kwargs.setdefault("ct", self.ct)
        kwargs.setdefault("pa", self.pa)
        kwargs.setdefault("pb", self.pb)
        kwargs.setdefault("sketch", self.sketch)
        kwargs.setdefault("construction", self.construction)
        return context.scene.sketcher.entities.add_ellipse(**kwargs)


slvs_entity_pointer(SlvsEllipse, "ct")
slvs_entity_pointer(SlvsEllipse, "pa")
slvs_entity_pointer(SlvsEllipse, "pb")
slvs_entity_pointer(SlvsEllipse, "sketch")

register, unregister = register_classes_factory((SlvsEllipse,))
