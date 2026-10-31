"""Fork: B-rep -> all-quad mesh, built face by face from the exact geometry.

Every B-rep edge is sampled once (its points are shared by the faces on both sides), so the
mesh is closed. Faces bounded by 4 edges (extrusion sides, fillets, chamfers, rectangular caps)
get a structured grid in their (u, v) parameters; opposite edges of those faces are given the
same number of segments. Other faces (caps with holes, odd outlines) are triangulated at half
resolution in (u, v), triangles are paired into quads, then every coarse polygon is split
(quad -> 4 quads, triangle -> 3 quads): the result is quads only, and the split points that
fall on an edge are exactly that edge's own samples.

The module only needs OCP (build123d); the 2D constrained triangulation is passed in
(Blender: mathutils.geometry.delaunay_2d_cdt).
"""
import math

SHARP_ANGLE_DEG = 3.0


def _indexed_map():
    try:  # OCP 7.9+
        from OCP.collections import IndexedMap_TopoDS_Shape_TopTools_ShapeMapHasher as M
    except ImportError:
        from OCP.TopTools import TopTools_IndexedMapOfShape as M
    return M()


def _cast(kind, shape):
    from OCP.TopoDS import TopoDS
    f = getattr(TopoDS, kind + "_s", None) or getattr(TopoDS, kind)
    return f(shape)


def _occ():
    from OCP import TopAbs, TopExp, TopTools, TopoDS, BRep, BRepAdaptor, BRepTools, GCPnts, GeomAbs
    return TopAbs, TopExp, TopTools, TopoDS, BRep, BRepAdaptor, BRepTools, GCPnts, GeomAbs


class _UF:
    def __init__(self):
        self.p = {}

    def find(self, a):
        self.p.setdefault(a, a)
        while self.p[a] != a:
            self.p[a] = self.p[self.p[a]]
            a = self.p[a]
        return a

    def union(self, a, b):
        ra, rb = self.find(a), self.find(b)
        if ra != rb:
            self.p[ra] = rb


def _point_in_loops(p, loops):
    """Even-odd test against several closed polylines."""
    x, y = p
    inside = False
    for loop in loops:
        n = len(loop)
        for i in range(n):
            x1, y1 = loop[i]
            x2, y2 = loop[(i + 1) % n]
            if (y1 > y) != (y2 > y):
                xi = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
                if xi > x:
                    inside = not inside
    return inside


def _dist_seg(p, a, b):
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / L2))
    qx, qy = ax + t * dx - p[0], ay + t * dy - p[1]
    return math.hypot(qx, qy)


def _quad_quality(pts):
    """Worst corner deviation from 90 deg (degrees); None if not convex."""
    worst = 0.0
    sign = 0
    for k in range(4):
        a, b, c = pts[k - 1], pts[k], pts[(k + 1) % 4]
        v1 = (a[0] - b[0], a[1] - b[1])
        v2 = (c[0] - b[0], c[1] - b[1])
        cr = v1[0] * v2[1] - v1[1] * v2[0]
        s = 1 if cr > 0 else -1
        if sign == 0:
            sign = s
        elif s != sign or abs(cr) < 1e-30:
            return None
        n1, n2 = math.hypot(*v1), math.hypot(*v2)
        if n1 == 0 or n2 == 0:
            return None
        ang = math.degrees(math.acos(max(-1.0, min(1.0, (v1[0] * v2[0] + v1[1] * v2[1]) / (n1 * n2)))))
        worst = max(worst, abs(ang - 90.0))
    return worst


def _bad_quad(P, lo=8.0):
    for k in range(4):
        a, b, c = P[k - 1], P[k], P[(k + 1) % 4]
        u = [a[i] - b[i] for i in range(3)]
        w = [c[i] - b[i] for i in range(3)]
        nu = math.sqrt(sum(x * x for x in u))
        nw = math.sqrt(sum(x * x for x in w))
        if nu * nw == 0:
            return True
        cs = sum(u[i] * w[i] for i in range(3)) / (nu * nw)
        ang = math.degrees(math.acos(max(-1.0, min(1.0, cs))))
        if ang < lo or ang > 180.0 - lo:
            return True
    return False


def quad_mesh(solid, edge_length, max_angle_deg=15.0, cdt=None):
    """Return (verts, faces, sharp_edges, info).

    verts: list of (x, y, z); faces: lists of 4 vertex indices (outward winding);
    sharp_edges: set of (i, j) vertex pairs on non-tangent B-rep edges.
    edge_length / max_angle_deg describe the final mesh (the coarse pass uses twice as much).
    """
    TopAbs, TopExp, TopTools, TopoDS, BRep, BRepAdaptor, BRepTools, GCPnts, GeomAbs = _occ()
    shape = getattr(solid, "wrapped", solid)

    emap = _indexed_map()
    vmap = _indexed_map()
    fmap = _indexed_map()
    TopExp.TopExp.MapShapes_s(shape, TopAbs.TopAbs_EDGE, emap)
    TopExp.TopExp.MapShapes_s(shape, TopAbs.TopAbs_VERTEX, vmap)
    TopExp.TopExp.MapShapes_s(shape, TopAbs.TopAbs_FACE, fmap)

    H = 2.0 * edge_length                    # coarse spacing
    A = math.radians(2.0 * max_angle_deg)    # coarse angle per segment

    # ---- edges: coarse counts -----------------------------------------------------------
    edges = {}
    for i in range(1, emap.Extent() + 1):
        e = _cast("Edge", emap.FindKey(i))
        if BRep.BRep_Tool.Degenerated_s(e):
            edges[i] = {"degenerated": True}
            continue
        ef = _cast("Edge", e.Oriented(TopAbs.TopAbs_FORWARD))
        c = BRepAdaptor.BRepAdaptor_Curve(ef)
        t0, t1 = c.FirstParameter(), c.LastParameter()
        L = GCPnts.GCPnts_AbscissaPoint.Length_s(c)
        is_line = c.GetType() == GeomAbs.GeomAbs_Line
        va = vmap.FindIndex(TopExp.TopExp.FirstVertex_s(ef))
        vb = vmap.FindIndex(TopExp.TopExp.LastVertex_s(ef))
        closed = va == vb
        turn = 0.0
        if not is_line:
            prev = None
            from OCP.gp import gp_Pnt, gp_Vec
            for k in range(33):
                pp, vv = gp_Pnt(), gp_Vec()
                c.D1(t0 + (t1 - t0) * k / 32, pp, vv)
                if vv.Magnitude() > 1e-15:
                    if prev is not None:
                        turn += prev.Angle(vv)
                    prev = vv
        n = max(1, math.ceil(L / H - 1e-9))
        if not is_line:
            n = max(n, math.ceil(turn / A - 1e-9), 2)
        if closed:
            n = max(n, 3)
        edges[i] = {"curve": c, "t0": t0, "t1": t1, "va": va, "vb": vb, "n": n, "line": is_line}

    # ---- faces: find the 4-sided ones and match opposite edge counts ---------------------
    def wire_edges(face, wire):
        out = []
        ex = BRepTools.BRepTools_WireExplorer(wire, face)
        while ex.More():
            ce = ex.Current()
            out.append((emap.FindIndex(ce), ce))
            ex.Next()
        return out

    faces = []
    uf = _UF()
    for fi in range(1, fmap.Extent() + 1):
        face = _cast("Face", fmap.FindKey(fi))
        wires = []
        it = TopExp.TopExp_Explorer(face, TopAbs.TopAbs_WIRE)
        while it.More():
            wires.append(_cast("Wire", it.Current()))
            it.Next()
        outer = BRepTools.BRepTools.OuterWire_s(face)
        loops = [wire_edges(face, w) for w in wires]
        loops = [[(ei, ce) for ei, ce in lp if not edges[ei].get("degenerated")] for lp in loops]
        structured = len(loops) == 1 and len(loops[0]) == 4
        if structured:
            (a, _), (b, _), (c_, _), (d, _) = loops[0]
            uf.union(("e", a), ("e", c_))
            uf.union(("e", b), ("e", d))
        faces.append({"fi": fi, "face": face, "loops": loops, "structured": structured,
                      "outer": outer})
    groups = {}
    for i, ed in edges.items():
        if "n" in ed:
            r = uf.find(("e", i))
            groups[r] = max(groups.get(r, 0), ed["n"])
    for i, ed in edges.items():
        if "n" in ed:
            ed["n"] = groups[uf.find(("e", i))]

    # ---- shared edge samples (fine: 2n segments) ----------------------------------------
    keys = {}
    verts = []

    def vid(key, xyz):
        j = keys.get(key)
        if j is None:
            j = keys[key] = len(verts)
            verts.append(xyz)
        return j

    from OCP.gp import gp_Pnt, gp_Vec, gp_Pnt2d
    for i, ed in edges.items():
        if "n" not in ed:
            continue
        m = 2 * ed["n"]
        c = ed["curve"]
        if ed["line"]:
            ts = [ed["t0"] + (ed["t1"] - ed["t0"]) * k / m for k in range(m + 1)]
        else:
            ua = GCPnts.GCPnts_UniformAbscissa(c, m + 1)
            if ua.IsDone() and ua.NbPoints() == m + 1:
                ts = [ua.Parameter(k + 1) for k in range(m + 1)]
            else:
                ts = [ed["t0"] + (ed["t1"] - ed["t0"]) * k / m for k in range(m + 1)]
        ed["ts"] = ts
        ids = []
        for k, t in enumerate(ts):
            p = c.Value(t)
            xyz = (p.X(), p.Y(), p.Z())
            if k == 0:
                key = ("v", ed["va"])
            elif k == m:
                key = ("v", ed["vb"])
            else:
                key = ("e", i, k)
            ids.append(vid(key, xyz))
        ed["ids"] = ids

    out_faces = []
    normals_at_mid = {}
    info = {"structured": 0, "general": 0, "faces": len(faces), "warnings": []}

    for F in faces:
        face = F["face"]
        fi = F["fi"]
        surf = BRepAdaptor.BRepAdaptor_Surface(face)
        flip = face.Orientation() == TopAbs.TopAbs_REVERSED

        def normal(u, v):
            p, du, dv = gp_Pnt(), gp_Vec(), gp_Vec()
            surf.D1(u, v, p, du, dv)
            nn = du.Crossed(dv)
            if nn.Magnitude() < 1e-30:
                return None
            nn.Normalize()
            return (-nn.X(), -nn.Y(), -nn.Z()) if flip else (nn.X(), nn.Y(), nn.Z())

        def lift(u, v):
            p = surf.Value(u, v)
            return (p.X(), p.Y(), p.Z())

        # boundary: per edge, fine (vertex id, uv) in traversal order
        loops_pts = []
        for lp in F["loops"]:
            sides = []
            for ei, ce in lp:
                ed = edges[ei]
                c2 = BRepAdaptor.BRepAdaptor_Curve2d(ce, face)
                pts = [(ed["ids"][k], (lambda q: (q.X(), q.Y()))(c2.Value(t))) for k, t in enumerate(ed["ts"])]
                if ce.Orientation() == TopAbs.TopAbs_REVERSED:
                    pts.reverse()
                tm = 0.5 * (ed["t0"] + ed["t1"])
                q = c2.Value(tm)
                nrm = normal(q.X(), q.Y())
                if nrm:
                    normals_at_mid.setdefault(ei, []).append((fi, nrm))
                sides.append(pts)
            loops_pts.append(sides)

        new = []
        uv_of = {}
        if F["structured"]:
            info["structured"] += 1
            s0, s1, s2, s3 = loops_pts[0]
            nA, nB = len(s0) - 1, len(s1) - 1
            if len(s2) - 1 != nA or len(s3) - 1 != nB:
                info["warnings"].append(f"face {fi}: unmatched counts")
                F["structured"] = False
            else:
                bot = s0
                right = s1
                top = list(reversed(s2))
                left = list(reversed(s3))
                grid = [[None] * (nB + 1) for _ in range(nA + 1)]
                P00, P10, P11, P01 = bot[0][1], bot[-1][1], top[-1][1], top[0][1]
                for i in range(nA + 1):
                    for j in range(nB + 1):
                        if j == 0:
                            grid[i][j] = bot[i][0]
                        elif j == nB:
                            grid[i][j] = top[i][0]
                        elif i == 0:
                            grid[i][j] = left[j][0]
                        elif i == nA:
                            grid[i][j] = right[j][0]
                        else:
                            s, t = i / nA, j / nB
                            u = ((1 - t) * bot[i][1][0] + t * top[i][1][0] + (1 - s) * left[j][1][0] + s * right[j][1][0]
                                 - ((1 - s) * (1 - t) * P00[0] + s * (1 - t) * P10[0] + s * t * P11[0] + (1 - s) * t * P01[0]))
                            v = ((1 - t) * bot[i][1][1] + t * top[i][1][1] + (1 - s) * left[j][1][1] + s * right[j][1][1]
                                 - ((1 - s) * (1 - t) * P00[1] + s * (1 - t) * P10[1] + s * t * P11[1] + (1 - s) * t * P01[1]))
                            grid[i][j] = vid(("f", fi, i, j), lift(u, v))
                            uv_of[grid[i][j]] = (u, v)
                for i in range(nA):
                    for j in range(nB):
                        new.append([grid[i][j], grid[i + 1][j], grid[i + 1][j + 1], grid[i][j + 1]])
        if not F["structured"]:
            info["general"] += 1
            new = _general_face(fi, loops_pts, surf, lift, vid, H, cdt, info)

        # orientation: polygons come out counter-clockwise in (u, v), i.e. along du x dv
        if F["structured"]:
            ring = [uv for side in loops_pts[0] for _, uv in side[:-1]]
            area = sum(ring[k - 1][0] * ring[k][1] - ring[k][0] * ring[k - 1][1] for k in range(len(ring)))
            if area < 0:
                new = [list(reversed(q)) for q in new]
        if flip:
            new = [list(reversed(q)) for q in new]
        out_faces.extend(new)
        info.setdefault("face_of", []).extend([fi] * len(new))

    # ---- sharp edges -------------------------------------------------------------------
    sharp = set()
    lim = math.cos(math.radians(SHARP_ANGLE_DEG))
    for ei, lst in normals_at_mid.items():
        fis = {f for f, _ in lst}
        if len(fis) < 2:
            continue  # seam
        n1, n2 = lst[0][1], lst[1][1]
        if sum(n1[k] * n2[k] for k in range(3)) < lim:
            ids = edges[ei]["ids"]
            for a, b in zip(ids, ids[1:]):
                sharp.add((min(a, b), max(a, b)))

    # drop degenerate quads (repeated vertex)
    keep = [k for k, q in enumerate(out_faces) if len(set(q)) == 4]
    info["face_of"] = [info["face_of"][k] for k in keep]
    out_faces = [out_faces[k] for k in keep]
    info["degenerate"] = sum(1 for q in out_faces if _bad_quad([verts[k] for k in q]))
    info["verts"] = len(verts)
    info["quads"] = len(out_faces)
    return verts, out_faces, sharp, info


def _general_face(fi, loops_pts, surf, lift, vid, H, cdt, info):
    """Coarse CDT in (u, v) -> pair triangles -> split everything into quads."""
    from OCP.gp import gp_Pnt, gp_Vec
    # fine loops (drop the last point of each edge: it starts the next one)
    loops = []
    for sides in loops_pts:
        lp = []
        for pts in sides:
            lp.extend(pts[:-1])
        if len(lp) >= 3:
            loops.append(lp)
    if not loops:
        return []
    # metric scale of the parameters near the middle of the face
    us = [uv[0] for lp in loops for _, uv in lp]
    vs = [uv[1] for lp in loops for _, uv in lp]
    uc, vc = 0.5 * (min(us) + max(us)), 0.5 * (min(vs) + max(vs))
    p, du, dv = gp_Pnt(), gp_Vec(), gp_Vec()
    surf.D1(uc, vc, p, du, dv)
    su, sv = max(du.Magnitude(), 1e-12), max(dv.Magnitude(), 1e-12)
    S = lambda uv: (uv[0] * su, uv[1] * sv)
    US = lambda xy: (xy[0] / su, xy[1] / sv)

    # coarse boundary = even positions of each fine loop (edges have an even number of segments)
    coarse_ids, coarse_xy, cedges, loops_xy = [], [], [], []
    mid_of = {}
    for lp in loops:
        start = len(coarse_ids)
        cl = []
        for k in range(0, len(lp), 2):
            coarse_ids.append(lp[k][0])
            coarse_xy.append(S(lp[k][1]))
            cl.append(S(lp[k][1]))
        nloc = len(coarse_ids) - start
        for k in range(nloc):
            a, b = start + k, start + (k + 1) % nloc
            cedges.append((a, b))
            fine_mid = lp[(2 * k + 1) % len(lp)][0]
            mid_of[(a, b)] = mid_of[(b, a)] = fine_mid
        loops_xy.append(cl)
    nb = len(coarse_ids)

    # interior points on a square grid
    xs = [x for x, _ in coarse_xy]
    ys = [y for _, y in coarse_xy]
    segs = [(loops_xy[li][k], loops_xy[li][(k + 1) % len(loops_xy[li])])
            for li in range(len(loops_xy)) for k in range(len(loops_xy[li]))]
    inner = []
    x = min(xs) + H
    while x < max(xs) - 0.5 * H:
        y = min(ys) + H
        while y < max(ys) - 0.5 * H:
            if _point_in_loops((x, y), loops_xy) and min(_dist_seg((x, y), a, b) for a, b in segs) > 0.6 * H:
                inner.append((x, y))
            y += H
        x += H
    all_xy = coarse_xy + inner

    tris = cdt(all_xy, cedges)
    tris = [t for t in tris if _point_in_loops(
        tuple(sum(all_xy[k][c] for k in t) / 3 for c in range(2)), loops_xy)]

    # ids for interior coarse points
    pid = list(coarse_ids) + [vid(("fi", fi, k), lift(*US(xy))) for k, xy in enumerate(inner)]

    # pair triangles into quads
    edge_tris = {}
    for ti, t in enumerate(tris):
        for k in range(3):
            a, b = t[k], t[(k + 1) % 3]
            edge_tris.setdefault((min(a, b), max(a, b)), []).append(ti)
    cands = []
    for (a, b), tl in edge_tris.items():
        if len(tl) != 2:
            continue
        t1, t2 = tris[tl[0]], tris[tl[1]]
        c1 = [k for k in t1 if k not in (a, b)][0]
        c2 = [k for k in t2 if k not in (a, b)][0]
        # quad in order following t1's winding
        i = t1.index(c1)
        quad = [c1, t1[(i + 1) % 3], c2, t1[(i + 2) % 3]]
        q = _quad_quality([all_xy[k] for k in quad])
        if q is not None and q < 50.0:
            cands.append((q, tl[0], tl[1], quad))
    cands.sort(key=lambda c: c[0])
    used = set()
    polys = []
    for q, t1, t2, quad in cands:
        if t1 in used or t2 in used:
            continue
        used.update((t1, t2))
        polys.append(quad)
    polys += [list(t) for ti, t in enumerate(tris) if ti not in used]
    for k, poly in enumerate(polys):
        ar = sum(all_xy[poly[i - 1]][0] * all_xy[poly[i]][1] - all_xy[poly[i]][0] * all_xy[poly[i - 1]][1]
                 for i in range(len(poly)))
        if ar < 0:
            polys[k] = list(reversed(poly))

    # split every coarse polygon into quads
    out = []
    mids = {}

    def mid(a, b):
        if (a, b) in mid_of:
            return mid_of[(a, b)]
        k = (min(a, b), max(a, b))
        if k not in mids:
            xy = ((all_xy[a][0] + all_xy[b][0]) / 2, (all_xy[a][1] + all_xy[b][1]) / 2)
            mids[k] = vid(("fm", fi, k), lift(*US(xy)))
        return mids[k]

    for pi, poly in enumerate(polys):
        n = len(poly)
        cx = sum(all_xy[k][0] for k in poly) / n
        cy = sum(all_xy[k][1] for k in poly) / n
        c = vid(("fc", fi, pi), lift(*US((cx, cy))))
        for k in range(n):
            a, b, z = poly[k], poly[(k + 1) % n], poly[k - 1]
            out.append([pid[a], mid(a, b), c, mid(z, a)])
    return out


def scipy_cdt(points, edges):
    """Fallback for tests outside Blender (not constrained: dense boundaries only)."""
    from scipy.spatial import Delaunay
    import numpy as np
    d = Delaunay(np.array(points))
    return [tuple(int(k) for k in s) for s in d.simplices]


_CDT_TYPE = [1]


def blender_cdt(points, edges):
    from mathutils import Vector
    from mathutils.geometry import delaunay_2d_cdt
    vco = [Vector(p) for p in points]
    span = max(max(p[0] for p in points) - min(p[0] for p in points),
               max(p[1] for p in points) - min(p[1] for p in points), 1e-12)
    out_v, _e, out_f, orig_v, _oe, _of = delaunay_2d_cdt(vco, edges, [], _CDT_TYPE[0], span * 1e-9)
    remap = []
    for k, o in enumerate(orig_v):
        remap.append(o[0] if o else None)
    tris = []
    for f in out_f:
        if len(f) == 3 and all(remap[k] is not None for k in f):
            tris.append(tuple(remap[k] for k in f))
    return tris
