"""Generateur d'icones CAD Sketcher (fork).

Chaque icone est decrite une seule fois avec quelques primitives, puis
ecrite en deux formats :
  - SVG  : source lisible / retouchable
  - .dat : format "VCO" des icones d'outils Blender (triangles + couleurs)

Repere : carre 256 x 256, origine en bas a gauche (celui des .dat).
"""
import math
import struct
import sys
from pathlib import Path

# --- palette ---------------------------------------------------------------
GEOM = (235, 235, 235, 255)      # geometrie
ACCENT = (117, 255, 175, 255)    # points de definition (vert de l'icone point existante)
CONSTR = (150, 150, 150, 255)    # construction : rayons, axes, poignees
FILL = (235, 235, 235, 60)       # remplissage leger (plan)
DARK = (40, 40, 40, 255)         # liseret autour des points

W_GEOM = 14      # epaisseur des traits de geometrie
W_CONSTR = 7     # epaisseur des traits de construction
R_PT = 15        # rayon d'un point
R_PT_RING = 21   # rayon du liseret


class Icon:
    """kind :
        tool       -> .dat (barre d'outils) + SVG + PNG (ruban)
        ui         -> SVG + PNG (boutons d'interface)
        constraint -> masque blanc SVG + PNG 400 px, teinte par le gizmo
    """

    def __init__(self, name, kind="tool"):
        self.name = name
        self.kind = kind
        self.mask = kind == "constraint"
        self.tris = []   # [(p0, p1, p2, color)]
        self.svg = []

    def _c(self, col):
        """En mode masque tout devient blanc, l'alpha porte la hierarchie."""
        if not self.mask:
            return col
        alpha = col[3] if col not in (CONSTR,) else 150
        return (255, 255, 255, alpha)

    # -- primitives bas niveau --
    def _tri(self, a, b, c, col):
        self.tris.append((a, b, c, col))

    def arrow_head(self, tip, frm, size=30, col=GEOM):
        """Triangle plein en tip, oriente depuis frm"""
        col = self._c(col)
        dx, dy = tip[0] - frm[0], tip[1] - frm[1]
        L = math.hypot(dx, dy) or 1.0
        ux, uy = dx / L, dy / L
        bx, by = tip[0] - ux * size, tip[1] - uy * size
        nx, ny = -uy * size * 0.55, ux * size * 0.55
        pts = [tip, (bx + nx, by + ny), (bx - nx, by - ny)]
        self._tri(*pts, col)
        self.svg.append('<polygon points="{}" fill="{}"{}/>'.format(
            " ".join("%.1f,%.1f" % (x, 256 - y) for x, y in pts),
            _rgb(col), _op(col, "fill-opacity")))

    def arrow(self, a, b, w=W_GEOM, col=GEOM, size=34, both=False):
        """Trait termine par une (ou deux) pointes"""
        L = math.hypot(b[0] - a[0], b[1] - a[1])
        k = (L - size * 0.6) / L
        b2 = (a[0] + (b[0] - a[0]) * k, a[1] + (b[1] - a[1]) * k)
        a2 = a
        if both:
            a2 = (b[0] + (a[0] - b[0]) * k, b[1] + (a[1] - b[1]) * k)
            self.arrow_head(a, b, size, col)
        self.stroke([a2, b2], w=w, col=col)
        self.arrow_head(b, a, size, col)

    def _disc_tris(self, c, r, col, seg=20):
        pts = [(c[0] + r * math.cos(2 * math.pi * i / seg),
                c[1] + r * math.sin(2 * math.pi * i / seg)) for i in range(seg)]
        for i in range(seg):
            self._tri(c, pts[i], pts[(i + 1) % seg], col)

    def _segment_tris(self, a, b, w, col):
        dx, dy = b[0] - a[0], b[1] - a[1]
        L = math.hypot(dx, dy)
        if L == 0:
            return
        nx, ny = -dy / L * w / 2, dx / L * w / 2
        p1 = (a[0] + nx, a[1] + ny)
        p2 = (a[0] - nx, a[1] - ny)
        p3 = (b[0] - nx, b[1] - ny)
        p4 = (b[0] + nx, b[1] + ny)
        self._tri(p1, p2, p3, col)
        self._tri(p1, p3, p4, col)

    # -- primitives d'icone --
    def stroke(self, pts, w=W_GEOM, col=GEOM, closed=False):
        col = self._c(col)
        pts = list(pts)
        if closed:
            pts = pts + [pts[0]]
        for a, b in zip(pts, pts[1:]):
            self._segment_tris(a, b, w, col)
        # jointures / extremites arrondies
        for p in (pts if not closed else pts[:-1]):
            self._disc_tris(p, w / 2, col, seg=10)
        self.svg.append(
            '<polyline points="{}" fill="none" stroke="{}" stroke-width="{}" '
            'stroke-linecap="round" stroke-linejoin="round"{}/>'.format(
                " ".join("%.1f,%.1f" % (x, 256 - y) for x, y in pts),
                _rgb(col), w, _op(col, "stroke-opacity")))

    def dashed(self, pts, w=W_CONSTR, col=CONSTR, dash=18, gap=13):
        # decoupe la polyligne en tirets de longueur fixe
        col = self._c(col)
        pts = list(pts)
        segs = []
        cur, on, left = [pts[0]], True, dash
        for a, b in zip(pts, pts[1:]):
            L = math.hypot(b[0] - a[0], b[1] - a[1])
            t = 0.0
            while L - t > 1e-9:
                step = min(left, L - t)
                t += step
                p = (a[0] + (b[0] - a[0]) * t / L, a[1] + (b[1] - a[1]) * t / L)
                if on:
                    cur.append(p)
                left -= step
                if left <= 1e-9:
                    if on:
                        segs.append(cur)
                    on = not on
                    left = dash if on else gap
                    cur = [p]
        if on and len(cur) > 1:
            segs.append(cur)
        for s in segs:
            for a, b in zip(s, s[1:]):
                self._segment_tris(a, b, w, col)
        self.svg.append(
            '<polyline points="{}" fill="none" stroke="{}" stroke-width="{}" '
            'stroke-dasharray="{} {}" stroke-linecap="butt"/>'.format(
                " ".join("%.1f,%.1f" % (x, 256 - y) for x, y in pts),
                _rgb(col), w, dash, gap))

    def point(self, c, col=ACCENT, r=R_PT):
        if self.mask:
            # pas de liseret sombre dans un masque teinte
            col = self._c(col)
            self._disc_tris(c, r + 3, col)
            self.svg.append('<circle cx="%.1f" cy="%.1f" r="%d" fill="%s"/>'
                            % (c[0], 256 - c[1], r + 3, _rgb(col)))
            return
        self._disc_tris(c, r + (R_PT_RING - R_PT), DARK)
        self._disc_tris(c, r, col)
        self.svg.append('<circle cx="%.1f" cy="%.1f" r="%d" fill="%s" stroke="%s" '
                        'stroke-width="%d"/>' % (c[0], 256 - c[1], r + 3, _rgb(col),
                                                 _rgb(DARK), (R_PT_RING - R_PT)))

    def square(self, c, h=13, col=CONSTR):
        col = self._c(col)
        x, y = c
        q = [(x - h, y - h), (x + h, y - h), (x + h, y + h), (x - h, y + h)]
        self._tri(q[0], q[1], q[2], col)
        self._tri(q[0], q[2], q[3], col)
        self.svg.append('<rect x="%.1f" y="%.1f" width="%d" height="%d" fill="%s"/>'
                        % (x - h, 256 - y - h, 2 * h, 2 * h, _rgb(col)))

    def poly(self, pts, col=FILL):
        """Polygone convexe plein"""
        col = self._c(col)
        pts = list(pts)
        for i in range(1, len(pts) - 1):
            self._tri(pts[0], pts[i], pts[i + 1], col)
        self.svg.append('<polygon points="{}" fill="{}"{}/>'.format(
            " ".join("%.1f,%.1f" % (x, 256 - y) for x, y in pts),
            _rgb(col), _op(col, "fill-opacity")))

    def triad(self, o=(196, 60), s=40):
        """Petit triedre 3D : distingue les outils 3D de leurs equivalents 2D"""
        self.stroke([o, (o[0] + s, o[1])], w=7, col=(255, 80, 80, 255))
        self.stroke([o, (o[0], o[1] + s)], w=7, col=(110, 200, 80, 255))
        self.stroke([o, (o[0] - s * 0.6, o[1] - s * 0.6)], w=7, col=(80, 140, 255, 255))

    # -- sorties --
    def to_svg(self):
        return ('<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 256 256" '
                'width="256" height="256">\n  <!-- CAD Sketcher fork : {} -->\n  '
                .format(self.name) + "\n  ".join(self.svg) + "\n</svg>\n")

    def to_dat(self):
        q = lambda v: max(0, min(255, int(round(v))))
        out = bytearray(b"VCO\x00")
        out += bytes((255, 255))   # taille de l'icone
        out += bytes((0, 0))       # decalage
        for a, b, c, _ in self.tris:
            for p in (a, b, c):
                out += bytes((q(p[0]), q(p[1])))
        for *_, col in self.tris:
            out += bytes(col) * 3
        return bytes(out)


def _rgb(col):
    return "#%02x%02x%02x" % col[:3]


def _op(col, attr):
    return "" if col[3] == 255 else ' %s="%.2f"' % (attr, col[3] / 255)


def arc_pts(c, r, a0, a1, n=40, ry=None, rot=0.0):
    ry = r if ry is None else ry
    cr, sr = math.cos(rot), math.sin(rot)
    pts = []
    for i in range(n + 1):
        t = a0 + (a1 - a0) * i / n
        x, y = r * math.cos(t), ry * math.sin(t)
        pts.append((c[0] + x * cr - y * sr, c[1] + x * sr + y * cr))
    return pts


def bezier_pts(p0, p1, p2, p3, n=40):
    pts = []
    for i in range(n + 1):
        t = i / n
        u = 1 - t
        pts.append(tuple(u**3 * a + 3 * u * u * t * b + 3 * u * t * t * c + t**3 * d
                         for a, b, c, d in zip(p0, p1, p2, p3)))
    return pts


R = math.radians
ICONS = {}


def icon(name, kind="tool"):
    def deco(f):
        ic = Icon(name, kind)
        f(ic)
        ICONS[name] = ic
        return f
    return deco


# --- les icones --------------------------------------------------------------

@icon("ops.bgs.add_point")
def _(i):
    # point : croix de visee fine + point (convention CAO)
    i.stroke([(128, 58), (128, 92)], w=7, col=CONSTR)
    i.stroke([(128, 164), (128, 198)], w=7, col=CONSTR)
    i.stroke([(58, 128), (92, 128)], w=7, col=CONSTR)
    i.stroke([(164, 128), (198, 128)], w=7, col=CONSTR)
    i.point((128, 128), r=22)


@icon("ops.bgs.add_point3d")
def _(i):
    i.stroke([(112, 74), (112, 106)], w=7, col=CONSTR)
    i.stroke([(112, 170), (112, 202)], w=7, col=CONSTR)
    i.stroke([(42, 138), (74, 138)], w=7, col=CONSTR)
    i.stroke([(150, 138), (182, 138)], w=7, col=CONSTR)
    i.point((112, 138), r=22)
    i.triad()


@icon("ops.bgs.add_line")
def _(i):
    a, b = (52, 60), (204, 196)
    i.stroke([a, b])
    i.point(a)
    i.point(b)


@icon("ops.bgs.add_line3d")
def _(i):
    a, b = (40, 112), (172, 212)
    i.stroke([a, b])
    i.point(a)
    i.point(b)
    i.triad()


@icon("ops.bgs.add_circle")
def _(i):
    c, r = (128, 128), 88
    i.stroke(arc_pts(c, r, 0, 2 * math.pi, n=64), closed=False)
    p = (c[0] + r * math.cos(R(35)), c[1] + r * math.sin(R(35)))
    i.dashed([c, p])
    i.point(c)
    i.point(p, r=11)


@icon("ops.bgs.add_arc")
def _(i):
    # arc par centre : centre + rayons en pointille vers les extremites
    c, r = (96, 92), 118
    a0, a1 = R(5), R(85)
    i.dashed([c, (c[0] + r * math.cos(a0), c[1] + r * math.sin(a0))])
    i.dashed([c, (c[0] + r * math.cos(a1), c[1] + r * math.sin(a1))])
    pts = arc_pts(c, r, a0, a1)
    i.stroke(pts)
    i.point(c)
    i.point(pts[0])
    i.point(pts[-1])


@icon("ops.bgs.add_arc_3p")
def _(i):
    # arc 3 points : pas de centre, trois points sur la courbe
    c, r = (128, 70), 100
    pts = arc_pts(c, r, R(10), R(170))
    i.stroke(pts)
    i.point(pts[0])
    i.point(pts[len(pts) // 2])
    i.point(pts[-1])


@icon("ops.bgs.add_bezier")
def _(i):
    p0, p1, p2, p3 = (40, 64), (92, 216), (164, 40), (216, 192)
    i.dashed([p0, p1])
    i.dashed([p3, p2])
    i.stroke(bezier_pts(p0, p1, p2, p3))
    i.square(p1)
    i.square(p2)
    i.point(p0)
    i.point(p3)


@icon("ops.bgs.add_ellipse")
def _(i):
    c, a, b, rot = (128, 128), 100, 56, R(25)
    pts = arc_pts(c, a, 0, 2 * math.pi, n=72, ry=b, rot=rot)
    pa = (c[0] + a * math.cos(rot), c[1] + a * math.sin(rot))
    pb = (c[0] - b * math.sin(rot), c[1] + b * math.cos(rot))
    i.dashed([c, pa])
    i.dashed([c, pb])
    i.stroke(pts)
    i.point(c)
    i.point(pa, r=11)
    i.point(pb, r=11)


@icon("ops.bgs.add_rectangle")
def _(i):
    a, b = (48, 64), (208, 192)
    i.stroke([a, (b[0], a[1]), b, (a[0], b[1])], closed=True)
    i.point(a)
    i.point(b)


@icon("ops.bgs.add_workplane")
def _(i):
    # plan en perspective + normale
    q = [(28, 76), (168, 52), (228, 132), (88, 156)]
    i.poly(q)
    i.stroke(q, w=10, closed=True)
    c = (128, 104)
    i.stroke([c, (128, 214)], w=9, col=ACCENT)
    i.poly([(110, 196), (146, 196), (128, 232)], col=ACCENT)
    i.point(c, r=11)


@icon("ops.bgs.add_workplane_face")
def _(i):
    # face d'un cube mise en surbrillance + normale
    f = [(64, 112), (176, 112), (176, 40), (64, 40)]
    top = [(64, 112), (176, 112), (216, 152), (104, 152)]
    side = [(176, 112), (216, 152), (216, 80), (176, 40)]
    i.poly(side, col=(235, 235, 235, 40))
    i.poly(f, col=(235, 235, 235, 40))
    i.poly(top, col=(117, 255, 175, 110))
    i.stroke(f, w=8, closed=True)
    i.stroke(top, w=8, closed=True)
    i.stroke([(216, 152), (216, 80), (176, 40)], w=8)
    c = (140, 132)
    i.stroke([c, (140, 212)], w=9, col=ACCENT)
    i.poly([(122, 196), (158, 196), (140, 232)], col=ACCENT)


# --- outils de modification (barre d'outils) ---------------------------------

RED = (255, 95, 95, 255)


@icon("ops.bgs.select")
def _(i):
    # cadre de selection + curseur
    i.dashed([(40, 216), (176, 216), (176, 104), (40, 104), (40, 216)])
    tip = (120, 160)
    i.poly([tip, (120, 40), (150, 74)], col=GEOM)
    i.poly([tip, (150, 74), (196, 74)], col=GEOM)
    i.stroke([(146, 80), (172, 30)], w=18)


@icon("ops.bgs.trim")
def _(i):
    # ciseaux au-dessus d'un trait : partie gardee pleine, partie coupee
    # en pointille
    i.stroke([(24, 40), (128, 40)], w=12)
    i.dashed([(128, 40), (236, 40)], w=9)
    i.point((128, 40), r=11)

    pivot = (120, 150)
    for ring in ((50, 206), (50, 94)):
        dx, dy = pivot[0] - ring[0], pivot[1] - ring[1]
        L = math.hypot(dx, dy)
        ux, uy = dx / L, dy / L
        tip = (pivot[0] + ux * 108, pivot[1] + uy * 108)
        nx, ny = -uy * 14, ux * 14
        # lame effilee
        i.poly([(pivot[0] + nx, pivot[1] + ny), tip,
                (pivot[0] - nx, pivot[1] - ny)], col=GEOM)
        # branche + anneau
        r = 26
        start = (ring[0] + ux * r, ring[1] + uy * r)
        i.stroke([start, pivot], w=13)
        i.stroke(arc_pts(ring, r, 0, 2 * math.pi, n=40), w=11)
    i.point(pivot, col=RED, r=10)


@icon("ops.bgs.add_rectangle_center")
def _(i):
    a, b = (40, 64), (216, 192)
    i.dashed([a, b])
    i.dashed([(a[0], b[1]), (b[0], a[1])])
    i.stroke([a, (b[0], a[1]), b, (a[0], b[1])], closed=True)
    i.point(((a[0] + b[0]) / 2, (a[1] + b[1]) / 2))
    i.point(b, r=11)


@icon("ops.bgs.bevel")
def _(i):
    # conge : coin d'origine en pointille, arc tangent aux deux cotes
    i.dashed([(64, 120), (64, 208), (152, 208)])
    i.stroke([(64, 32), (64, 120)])
    arc = arc_pts((152, 120), 88, math.pi, math.pi / 2)
    i.stroke(arc)
    i.stroke([(152, 208), (232, 208)])
    i.point(arc[0], r=11)
    i.point(arc[-1], r=11)


@icon("ops.bgs.offset")
def _(i):
    # contour d'origine en pointille, copie decalee en trait plein
    c, r1, r2 = (128, 56), 58, 112
    i.dashed(arc_pts(c, r1, R(180), R(0)))
    i.stroke(arc_pts(c, r2, R(180), R(0)))
    a = (c[0], c[1] + r1)
    b = (c[0], c[1] + r2)
    i.arrow(a, (b[0], b[1] - 14), w=9, col=ACCENT, size=30)


# --- transformations (boutons) -----------------------------------------------

@icon("ops.bgs.move", "ui")
def _(i):
    c = (128, 128)
    for d in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        i.arrow((c[0] + d[0] * 26, c[1] + d[1] * 26),
                (c[0] + d[0] * 104, c[1] + d[1] * 104), w=12)
    i.point(c)


@icon("ops.bgs.rotate", "ui")
def _(i):
    c = (128, 128)
    pts = arc_pts(c, 84, R(110), R(380))
    i.stroke(pts[:-3])
    i.arrow_head(pts[-1], pts[-5], size=40)
    i.dashed([c, pts[0]])
    i.point(c)


@icon("ops.bgs.scale", "ui")
def _(i):
    i.dashed([(36, 36), (112, 36), (112, 112), (36, 112), (36, 36)])
    i.stroke([(36, 36), (220, 36), (220, 220), (36, 220)], closed=True)
    i.arrow((96, 96), (188, 188), w=11, col=ACCENT, size=36)
    i.point((36, 36), r=11)


@icon("ops.bgs.mirror", "ui")
def _(i):
    i.dashed([(128, 20), (128, 236)], w=8, col=ACCENT)
    left = [(104, 56), (104, 200), (32, 88)]
    i.stroke(left, closed=True)
    i.stroke([(256 - x, y) for x, y in left], closed=True, col=CONSTR)


@icon("ops.bgs.array_linear", "ui")
def _(i):
    for k, col in enumerate((GEOM, CONSTR, CONSTR)):
        x = 32 + k * 72
        sq = [(x, 112), (x + 48, 112), (x + 48, 160), (x, 160)]
        if k == 0:
            i.stroke(sq, closed=True)
        else:
            i.dashed(sq + [sq[0]], w=9)
    i.arrow((32, 64), (224, 64), w=10, col=ACCENT, size=30)


@icon("ops.bgs.merge_points", "ui")
def _(i):
    t = (188, 128)
    for s in ((56, 200), (56, 56)):
        i.dashed([s, ((s[0] + t[0]) / 2, (s[1] + t[1]) / 2 + (0 if s[1] < 128 else 0)), t])
        i.point(s, r=13)
    i.point(t, r=24)


@icon("ops.bgs.construction", "ui")
def _(i):
    # geometrie de construction : traits pointilles
    i.dashed([(40, 56), (216, 200)], w=12, col=GEOM, dash=24, gap=16)
    i.dashed(arc_pts((128, 128), 92, R(150), R(390), n=60), w=10, col=CONSTR,
             dash=20, gap=14)
    i.point((40, 56), r=13)
    i.point((216, 200), r=13)


@icon("ops.bgs.leave_sketch", "ui")
def _(i):
    # cadre ouvert + fleche de sortie
    i.stroke([(150, 196), (150, 220), (36, 220), (36, 36), (150, 36), (150, 60)])
    i.arrow((88, 128), (232, 128), w=16, col=ACCENT, size=46)


# --- contraintes (masques blancs teintes par le gizmo) -----------------------

@icon("COINCIDENT", "constraint")
def _(i):
    i.stroke([(32, 56), (128, 136)])
    i.stroke([(224, 56), (128, 136)])
    i.stroke([(128, 136), (128, 228)], col=CONSTR, w=W_CONSTR)
    i.point((128, 136), r=26)


@icon("DISTANCE", "constraint")
def _(i):
    i.stroke([(44, 48), (44, 200)], w=W_CONSTR, col=CONSTR)
    i.stroke([(212, 48), (212, 200)], w=W_CONSTR, col=CONSTR)
    i.arrow((48, 128), (208, 128), w=12, size=40, both=True)
    i.stroke([(20, 48), (236, 48)], w=10)


@icon("DIAMETER", "constraint")
def _(i):
    c, r = (128, 128), 92
    i.stroke(arc_pts(c, r, 0, 2 * math.pi, n=64), w=10)
    a = (c[0] - r * 0.707, c[1] - r * 0.707)
    b = (c[0] + r * 0.707, c[1] + r * 0.707)
    i.arrow(a, b, w=11, size=38, both=True)


@icon("ANGLE", "constraint")
def _(i):
    v = (40, 56)
    i.stroke([v, (232, 56)])
    end = (v[0] + 200 * math.cos(R(55)), v[1] + 200 * math.sin(R(55)))
    i.stroke([v, end])
    arc = arc_pts(v, 120, R(4), R(51))
    i.stroke(arc[3:-3], w=9)
    i.arrow_head(arc[-1], arc[-4], size=30)
    i.arrow_head(arc[0], arc[3], size=30)


@icon("EQUAL", "constraint")
def _(i):
    i.stroke([(48, 160), (208, 160)], w=26)
    i.stroke([(48, 96), (208, 96)], w=26)


@icon("VERTICAL", "constraint")
def _(i):
    i.stroke([(128, 44), (128, 212)], w=18)
    i.point((128, 44))
    i.point((128, 212))


@icon("HORIZONTAL", "constraint")
def _(i):
    i.stroke([(44, 128), (212, 128)], w=18)
    i.point((44, 128))
    i.point((212, 128))


@icon("PARALLEL", "constraint")
def _(i):
    i.stroke([(56, 40), (152, 216)], w=18)
    i.stroke([(112, 40), (208, 216)], w=18)


@icon("PERPENDICULAR", "constraint")
def _(i):
    i.stroke([(32, 48), (224, 48)], w=18)
    i.stroke([(128, 48), (128, 224)], w=18)
    i.stroke([(128, 96), (176, 96), (176, 48)], w=9, col=CONSTR)


@icon("TANGENT", "constraint")
def _(i):
    c, r = (128, 92), 68
    i.stroke(arc_pts(c, r, 0, 2 * math.pi, n=64), w=12)
    i.stroke([(20, 160), (236, 160)], w=14)
    i.point((128, 160), r=16)


@icon("MIDPOINT", "constraint")
def _(i):
    a, b = (32, 72), (224, 184)
    i.stroke([a, b], w=12)
    m = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
    for p in (((a[0] + m[0]) / 2, (a[1] + m[1]) / 2),
              ((b[0] + m[0]) / 2, (b[1] + m[1]) / 2)):
        i.stroke([(p[0] - 12, p[1] + 21), (p[0] + 12, p[1] - 21)], w=9)
    i.point(m, r=22)


@icon("RATIO", "constraint")
def _(i):
    i.stroke([(32, 176), (224, 176)], w=16)
    i.stroke([(32, 80), (128, 80)], w=16)
    for x in (32, 128, 224):
        i.stroke([(x, 160), (x, 192)], w=8, col=CONSTR)
    for x in (32, 80, 128):
        i.stroke([(x, 64), (x, 96)], w=8, col=CONSTR)


@icon("SYMMETRY", "constraint")
def _(i):
    i.dashed([(128, 16), (128, 240)], w=10, col=CONSTR)
    i.point((56, 128), r=22)
    i.point((200, 128), r=22)
    i.arrow((82, 128), (114, 128), w=9, size=24)
    i.arrow((174, 128), (142, 128), w=9, size=24)


if __name__ == "__main__":
    out = Path(sys.argv[1])
    (out / "svg").mkdir(parents=True, exist_ok=True)
    for name, ic in ICONS.items():
        (out / "svg" / (name + ".svg")).write_text(ic.to_svg(), encoding="utf-8")
        if ic.kind == "tool":
            (out / "dat").mkdir(parents=True, exist_ok=True)
            (out / "dat" / (name + ".dat")).write_bytes(ic.to_dat())
        print(name, ic.kind, len(ic.tris), "triangles")
