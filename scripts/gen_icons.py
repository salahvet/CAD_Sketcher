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
    def __init__(self, name):
        self.name = name
        self.tris = []   # [(p0, p1, p2, color)]
        self.svg = []

    # -- primitives bas niveau --
    def _tri(self, a, b, c, col):
        self.tris.append((a, b, c, col))

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
        self._disc_tris(c, r + (R_PT_RING - R_PT), DARK)
        self._disc_tris(c, r, col)
        self.svg.append('<circle cx="%.1f" cy="%.1f" r="%d" fill="%s" stroke="%s" '
                        'stroke-width="%d"/>' % (c[0], 256 - c[1], r + 3, _rgb(col),
                                                 _rgb(DARK), (R_PT_RING - R_PT)))

    def square(self, c, h=13, col=CONSTR):
        x, y = c
        q = [(x - h, y - h), (x + h, y - h), (x + h, y + h), (x - h, y + h)]
        self._tri(q[0], q[1], q[2], col)
        self._tri(q[0], q[2], q[3], col)
        self.svg.append('<rect x="%.1f" y="%.1f" width="%d" height="%d" fill="%s"/>'
                        % (x - h, 256 - y - h, 2 * h, 2 * h, _rgb(col)))

    def poly(self, pts, col=FILL):
        """Polygone convexe plein"""
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


def icon(name):
    def deco(f):
        ic = Icon(name)
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


if __name__ == "__main__":
    out = Path(sys.argv[1])
    (out / "svg").mkdir(parents=True, exist_ok=True)
    (out / "dat").mkdir(parents=True, exist_ok=True)
    for name, ic in ICONS.items():
        (out / "svg" / (name + ".svg")).write_text(ic.to_svg(), encoding="utf-8")
        (out / "dat" / (name + ".dat")).write_bytes(ic.to_dat())
        print(name, len(ic.tris), "triangles")
