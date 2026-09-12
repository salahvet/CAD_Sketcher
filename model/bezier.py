import logging

import bpy
from bpy.types import PropertyGroup
from bpy.utils import register_classes_factory
import gpu
from gpu_extras.batch import batch_for_shader
from mathutils.geometry import interpolate_bezier

from ..solver import Solver
from .base_entity import Entity2D
from .utilities import (slvs_entity_pointer, get_connection_point,
                        round_v)  # round_v : correctif PureMed
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


    # --- correctif PureMed : intersect ---
    def overlaps_endpoint(self, co):
        """
        Le point tombe-t-il sur une extremite de la courbe ?

        SlvsLine2D et SlvsArc ont cette methode, SlvsBezier ne l avait
        pas — et intersect() l appelle. C est ce manque qui levait
        AttributeError: 'SlvsBezier' object has no attribute
        'overlaps_endpoint' des qu on rognait une Bezier.

        Meme implementation que sur les autres entites : comparaison
        arrondie au cinq-millieme, pour absorber le bruit numerique du
        solveur.
        """
        precision = 5
        co_rounded = round_v(co, ndigits=precision)
        return any(
            co_rounded == round_v(v, ndigits=precision)
            for v in (self.p1.co, self.p2.co)
        )

    def _aplatir(self, resolution=None):
        """La courbe en polyligne, dans le plan du croquis."""
        from mathutils.geometry import interpolate_bezier
        n = resolution or max(CURVE_RESOLUTION, 24)
        pts = interpolate_bezier(
            self.p1.co.to_3d(), self.ctrl1.co.to_3d(),
            self.ctrl2.co.to_3d(), self.p2.co.to_3d(), n)
        return [p.to_2d() for p in pts]

    def intersect(self, other, extended: bool = False):
        """
        Points de coupe avec une autre entite du croquis.

        Renvoie un tuple de Vector 2D, vide s il n y en a pas — meme
        contrat que SlvsLine2D.intersect et SlvsArc.intersect.
        """
        from mathutils.geometry import (intersect_line_line_2d,
                                        intersect_line_sphere_2d)

        def sur_segment(a, b, p, marge=1e-7):
            ab = b - a
            L2 = ab.length_squared
            if L2 <= 0.0:
                return False
            t = (p - a).dot(ab) / L2
            return -marge <= t <= 1.0 + marge

        mien = self._aplatir()
        autre_pts = None
        cercle = None
        if hasattr(other, "is_line") and other.is_line():
            autre_pts = [other.p1.co, other.p2.co]
        elif type(other).__name__ == "SlvsBezier":
            autre_pts = other._aplatir()
        elif hasattr(other, "ct") and hasattr(other, "radius"):
            cercle = (other.ct.co, other.radius)
        else:
            return ()

        trouves = []
        for a, b in zip(mien, mien[1:]):
            candidats = []
            if autre_pts is not None:
                for c, d in zip(autre_pts, autre_pts[1:]):
                    co = intersect_line_line_2d(a, b, c, d)
                    if co is not None:
                        candidats.append(co)
            else:
                centre, rayon = cercle
                for co in (intersect_line_sphere_2d(a, b, centre, rayon) or ()):
                    if co is None or not sur_segment(a, b, co):
                        continue
                    # un arc ne couvre qu une portion du cercle
                    if hasattr(other, "is_inside") and not other.is_inside(co):
                        continue
                    candidats.append(co)
            for co in candidats:
                # ne PAS supposer que l autre entite possede la methode :
                # c est exactement l erreur qui a fait planter Trim la
                # premiere fois, avec AttributeError sur SlvsBezier.
                if _touche_un_bout(self, co) or _touche_un_bout(other, co):
                    continue
                # la polyligne peut rendre deux fois le meme point quand
                # l intersection tombe pile sur un sommet
                if any((co - v).length < 1e-5 for v in trouves):
                    continue
                trouves.append(co)
        return tuple(trouves)

    def _parametre(self, co, resolution=256):
        """
        Le parametre t de la courbe le plus proche du point co.

        On aplatit finement, on trouve le segment de polyligne le plus
        proche, puis on interpole a l interieur. Suffisant pour Trim,
        qui travaille a la precision du clic.
        """
        pts = self._aplatir(resolution)
        n = len(pts) - 1
        best = (1e18, 0.0)
        for i in range(n):
            a, b = pts[i], pts[i + 1]
            ab = b - a
            L2 = ab.length_squared
            u = max(0.0, min(1.0, (co - a).dot(ab) / L2)) if L2 > 0 else 0.0
            d = (co - (a + ab * u)).length
            if d < best[0]:
                best = (d, (i + u) / n)
        return best[1]

    def _abscisses(self, resolution=256):
        """Polyligne et abscisses curvilignes cumulees."""
        pts = self._aplatir(resolution)
        s = [0.0]
        for a, b in zip(pts, pts[1:]):
            s.append(s[-1] + (b - a).length)
        return pts, s

    def distance_along_segment(self, p1, p2):
        """
        Distance de p1 a p2 MESUREE LE LONG de la courbe, en avancant.

        Meme contrat que sur SlvsLine2D et SlvsArc : Trim s en sert pour
        ordonner les intersections a partir du point clique. La valeur
        est prise modulo la longueur totale, de sorte qu un point situe
        en arriere ressorte en fin de liste plutot qu en negatif.
        """
        pts, s = self._abscisses()
        total = s[-1]
        if total <= 0.0:
            return 0.0

        def abscisse(co):
            n = len(pts) - 1
            best = (1e18, 0.0)
            for i in range(n):
                a, b = pts[i], pts[i + 1]
                ab = b - a
                L2 = ab.length_squared
                u = max(0.0, min(1.0, (co - a).dot(ab) / L2)) if L2 > 0 else 0.0
                d = (co - (a + ab * u)).length
                if d < best[0]:
                    best = (d, s[i] + u * (s[i + 1] - s[i]))
            return best[1]

        seuil = 0.0000001
        return (abscisse(p2) - abscisse(p1)) % (total + seuil)

    def project_point(self, coords):
        """
        Le point de la courbe le plus proche de coords.

        Meme contrat que sur SlvsLine2D et SlvsArc. operators/bevel.py
        l appelle sans protection, ligne 122 : sans cette methode, le
        conge plante sur une Bezier exactement comme Trim le faisait.

        Passe par la polyligne, puis affine par une dichotomie autour du
        segment retenu — la polyligne seule donnerait un point legerement
        en corde, pas sur la courbe.
        """
        t = self._parametre(coords)
        # affinage : on resserre autour de t sur quelques tours
        pas = 1.0 / 256.0
        for _ in range(24):
            candidats = [max(0.0, t - pas), t, min(1.0, t + pas)]
            t = min(candidats, key=lambda u: (self._point(u) - coords).length)
            pas *= 0.5
        return self._point(t)

    def _point(self, t):
        """Le point de la courbe au parametre t, par De Casteljau."""
        a, b, c, d = (self.p1.co, self.ctrl1.co, self.ctrl2.co, self.p2.co)
        ab = a.lerp(b, t)
        bc = b.lerp(c, t)
        cd = c.lerp(d, t)
        return ab.lerp(bc, t).lerp(bc.lerp(cd, t), t)

    def replace(self, context, p1, p2, use_self=False):
        """
        Rend une Bezier limitee aux points p1 et p2, poses sur la courbe.

        LE PIEGE

        Sur une ligne ou un arc, tronquer revient a deplacer les deux
        extremites : la forme entre elles ne change pas. Sur une Bezier,
        si. Reprendre les poignees d origine avec de nouvelles
        extremites donnerait une courbe differente de la portion
        d origine — le trace bougerait sous les yeux de l utilisateur.

        On extrait donc la sous-courbe par l algorithme de De Casteljau,
        qui rend les quatre points de controle exacts de la portion
        comprise entre deux parametres. La portion conservee garde ainsi
        rigoureusement son trace.
        """
        t1 = self._parametre(p1.co)
        t2 = self._parametre(p2.co)
        if t2 < t1:
            t1, t2 = t2, t1
            p1, p2 = p2, p1

        P0, P1, P2, P3 = (self.p1.co.copy(), self.ctrl1.co.copy(),
                          self.ctrl2.co.copy(), self.p2.co.copy())

        def couper(a, b, c, d, t):
            """De Casteljau : rend la portion [0, t] de la courbe."""
            ab = a.lerp(b, t)
            bc = b.lerp(c, t)
            cd = c.lerp(d, t)
            abc = ab.lerp(bc, t)
            bcd = bc.lerp(cd, t)
            fin = abc.lerp(bcd, t)
            return a, ab, abc, fin

        # portion [0, t2], puis dans celle-ci la portion [t1/t2, 1]
        A0, A1, A2, A3 = couper(P0, P1, P2, P3, max(t2, 1e-9))
        u = t1 / t2 if t2 > 1e-9 else 0.0
        B0, B1, B2, B3 = couper(A3, A2, A1, A0, max(1.0 - u, 1e-9))
        # couper() a travaille a l envers : on remet dans le bon sens
        n1, n2 = B2, B1

        sse = context.scene.sketcher.entities
        sketch = context.scene.sketcher.active_sketch

        if use_self:
            self.p1 = p1
            self.p2 = p2
            self.ctrl1.co = n1
            self.ctrl2.co = n2
            return self

        c1 = sse.add_point_2d(n1, sketch)
        c2 = sse.add_point_2d(n2, sketch)
        c1.construction = True
        c2.construction = True
        bez = sse.add_bezier(p1, c1, c2, p2, sketch)
        bez.construction = self.construction
        return bez
    # --- fin correctif PureMed ---

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


# --- correctif PureMed : intersect ---
def _touche_un_bout(entite, co):
    """
    Vrai si co tombe sur une extremite de l entite.

    Toutes les entites n ont pas overlaps_endpoint — un cercle n a pas
    d extremite, et SlvsBezier ne l avait pas jusqu a ce correctif.
    On interroge donc l objet plutot que de supposer.
    """
    f = getattr(entite, "overlaps_endpoint", None)
    return bool(f(co)) if callable(f) else False
# --- fin correctif PureMed ---
