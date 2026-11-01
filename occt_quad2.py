"""Fork: B-rep -> all-quad mesh by patch layout (adaptive density, clean edge flow).

Each B-rep face is cut into 4-sided patches, and each patch gets a structured grid:
  * holes are joined to the outer boundary by cuts;
  * reflex corners shoot a cut along the continuation of an incident edge (motorcycle cut);
  * regions left with 4 corners are one patch; with 3 or 5 corners they get a centre point and
    3 / 5 patches around it (one pole of valence 3 / 5, nothing else irregular);
  * opposite sides of every patch get the same number of segments (integer solve over the
    segments shared by all faces), starting from the fewest segments that keep the shape:
    segment density follows curvature (chord error) and an optional maximum edge length.
A face that cannot be laid out (fold, unsolvable counts) falls back to the triangle-pairing mesher.
"""
import math

try:
    from . import occt_quad as _base
except ImportError:  # tests outside Blender
    import occt_quad as _base


def _occ():
    from OCP import TopAbs, TopExp, BRep, BRepAdaptor, BRepTools, GeomAbs
    from OCP.gp import gp_Pnt, gp_Vec, gp_Pnt2d
    return TopAbs, TopExp, BRep, BRepAdaptor, BRepTools, GeomAbs, gp_Pnt, gp_Vec, gp_Pnt2d


class LayoutError(Exception):
    pass


# ---------------------------------------------------------------------------------------------
# small 2D helpers (scaled parameter plane)
# ---------------------------------------------------------------------------------------------

def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1])


def _cross(a, b):
    return a[0] * b[1] - a[1] * b[0]


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1]


def _norm(a):
    return math.hypot(a[0], a[1])


def _unit(a):
    n = _norm(a)
    return (a[0] / n, a[1] / n) if n > 1e-300 else (0.0, 0.0)


def _angle(d1, d2):
    """Signed turning from direction d1 to d2."""
    return math.atan2(_cross(d1, d2), _dot(d1, d2))


def _ray_seg(o, d, a, b):
    """Ray o + t d against segment a-b: (t, u) or None."""
    e = _sub(b, a)
    den = _cross(d, e)
    if abs(den) < 1e-300:
        return None
    w = _sub(a, o)
    t = _cross(w, e) / den
    u = _cross(w, d) / den
    if t > 0 and -1e-9 <= u <= 1 + 1e-9:
        return t, min(1.0, max(0.0, u))
    return None


# ---------------------------------------------------------------------------------------------
# model
# ---------------------------------------------------------------------------------------------

class Curve:
    """A B-rep edge (param t) or a cut inside a face (param f in [0, 1], straight in (u, v))."""

    def __init__(self, cid, kind):
        self.id = cid
        self.kind = kind
        self.nodes = []  # (param, node id)

    def add_node(self, p, nid):
        for q, n in self.nodes:
            if abs(q - p) < 1e-12 * (1 + abs(p)):
                return n
        self.nodes.append((p, nid))
        return nid


class Mesher:
    def __init__(self, solid, tolerance, max_edge, max_angle_deg, corner_deg, cdt, guides=None,
                 auto=False, across=2):
        self.guides = guides
        self.auto = auto
        self.strip_mode = auto
        self.across = across
        TopAbs, TopExp, BRep, BRepAdaptor, BRepTools, GeomAbs, gp_Pnt, gp_Vec, gp_Pnt2d = _occ()
        self.O = (TopAbs, TopExp, BRep, BRepAdaptor, BRepTools, GeomAbs, gp_Pnt, gp_Vec, gp_Pnt2d)
        self.shape = getattr(solid, "wrapped", solid)
        self.tol = tolerance
        self.max_edge = max_edge
        self.amax = math.radians(max_angle_deg)
        self.corner = math.radians(corner_deg)
        self.cdt = cdt
        self.emap, self.vmap, self.fmap = _base._indexed_map(), _base._indexed_map(), _base._indexed_map()
        TopExp.TopExp.MapShapes_s(self.shape, TopAbs.TopAbs_EDGE, self.emap)
        TopExp.TopExp.MapShapes_s(self.shape, TopAbs.TopAbs_VERTEX, self.vmap)
        TopExp.TopExp.MapShapes_s(self.shape, TopAbs.TopAbs_FACE, self.fmap)
        self.info = {"patches": 0, "poles": 0, "fallback": 0, "faces": self.fmap.Extent(), "warnings": []}

    # ---- nodes ------------------------------------------------------------------------------
    def reset(self):
        self.nodes = []        # dict(kind, xyz, uv={fi: uv})
        self.curves = {}
        self.vnode = {}
        self.patches = []      # (fi, [chain0..3]); chain = list of pieces
        self.fallback_loops = {}  # fi -> loops (list of list of pieces)

    def new_node(self, xyz=None, **kw):
        self.nodes.append({"xyz": xyz, "uv": {}, **kw})
        return len(self.nodes) - 1

    # ---- edges ------------------------------------------------------------------------------
    def prepare_edges(self):
        TopAbs, TopExp, BRep, BRepAdaptor, BRepTools, GeomAbs, gp_Pnt, gp_Vec, gp_Pnt2d = self.O
        self.edges = {}
        for i in range(1, self.emap.Extent() + 1):
            e = _base._cast("Edge", self.emap.FindKey(i))
            if BRep.BRep_Tool.Degenerated_s(e):
                self.edges[i] = None
                continue
            ef = _base._cast("Edge", e.Oriented(TopAbs.TopAbs_FORWARD))
            c = BRepAdaptor.BRepAdaptor_Curve(ef)
            t0, t1 = c.FirstParameter(), c.LastParameter()
            va = self.vmap.FindIndex(TopExp.TopExp.FirstVertex_s(ef))
            vb = self.vmap.FindIndex(TopExp.TopExp.LastVertex_s(ef))
            is_line = c.GetType() == GeomAbs.GeomAbs_Line
            # cumulative "sampling measure": chord error, angle per segment, max length
            N = 2 if is_line else 96
            ts = [t0 + (t1 - t0) * k / N for k in range(N + 1)]
            pts, tans = [], []
            for t in ts:
                p, v = gp_Pnt(), gp_Vec()
                c.D1(t, p, v)
                pts.append((p.X(), p.Y(), p.Z()))
                tans.append(v)
            M = [0.0]
            length = 0.0
            for k in range(N):
                ds = math.dist(pts[k], pts[k + 1])
                length += ds
                dth = 0.0
                if not is_line and tans[k].Magnitude() > 1e-15 and tans[k + 1].Magnitude() > 1e-15:
                    dth = tans[k].Angle(tans[k + 1])
                m = math.sqrt(dth * ds / (8.0 * self.tol)) + dth / self.amax
                if self.max_edge > 0:
                    m += ds / self.max_edge
                M.append(M[-1] + m)
            # distribution measure: the same, plus some arc length so that it never stays flat
            S_acc = [0.0]
            for k in range(N):
                S_acc.append(S_acc[-1] + math.dist(pts[k], pts[k + 1]))
            lam = (3.0 * M[-1] + 1.0) / max(length, 1e-15)  # mostly arc length: rungs stay across
            Md = [M[k] + lam * S_acc[k] for k in range(N + 1)]
            self.edges[i] = {"c": c, "t0": t0, "t1": t1, "va": va, "vb": vb, "ts": ts, "M": M, "Md": Md,
                             "len": length, "line": is_line}

    def measure(self, ei, p, key="M"):
        ed = self.edges[ei]
        ts, M = ed["ts"], ed[key]
        f = (p - ed["t0"]) / (ed["t1"] - ed["t0"]) * (len(ts) - 1)
        k = min(max(int(f), 0), len(ts) - 2)
        a = f - k
        return M[k] * (1 - a) + M[k + 1] * a

    def inv_measure(self, ei, m, key="Md"):
        ed = self.edges[ei]
        ts, M = ed["ts"], ed[key]
        lo, hi = 0, len(M) - 1
        if m <= M[0]:
            return ts[0]
        if m >= M[-1]:
            return ts[-1]
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if M[mid] <= m:
                lo = mid
            else:
                hi = mid
        a = (m - M[lo]) / (M[hi] - M[lo]) if M[hi] > M[lo] else 0.0
        return ts[lo] * (1 - a) + ts[hi] * a

    def vertex_node(self, vidx):
        n = self.vnode.get(vidx)
        if n is None:
            BRep = self.O[2]
            p = BRep.BRep_Tool.Pnt_s(_base._cast("Vertex", self.vmap.FindKey(vidx)))
            n = self.vnode[vidx] = self.new_node((p.X(), p.Y(), p.Z()), kind="v")
        return n

    def edge_curve(self, ei):
        cid = ("e", ei)
        c = self.curves.get(cid)
        if c is None:
            ed = self.edges[ei]
            c = self.curves[cid] = Curve(cid, "edge")
            c.ei = ei
            c.add_node(ed["t0"], self.vertex_node(ed["va"]))
            c.add_node(ed["t1"], self.vertex_node(ed["vb"]))
        return c

    # ---- faces ------------------------------------------------------------------------------
    def face_ctx(self, fi):
        TopAbs, TopExp, BRep, BRepAdaptor, BRepTools, GeomAbs, gp_Pnt, gp_Vec, gp_Pnt2d = self.O
        face = _base._cast("Face", self.fmap.FindKey(fi))
        F = {"fi": fi, "face": face, "surf": BRepAdaptor.BRepAdaptor_Surface(face),
             "flip": face.Orientation() == TopAbs.TopAbs_REVERSED, "c2": {}}
        loops = []
        outer = BRepTools.BRepTools.OuterWire_s(face)
        wires = []
        it = TopExp.TopExp_Explorer(face, TopAbs.TopAbs_WIRE)
        while it.More():
            wires.append(_base._cast("Wire", it.Current()))
            it.Next()
        wires.sort(key=lambda w: 0 if w.IsSame(outer) else 1)
        for w in wires:
            uses = []
            ex = BRepTools.BRepTools_WireExplorer(w, face)
            while ex.More():
                ce = ex.Current()
                ei = self.emap.FindIndex(ce)
                if self.edges.get(ei) is not None:
                    rev = ce.Orientation() == TopAbs.TopAbs_REVERSED
                    key = (ei, rev)
                    if key not in F["c2"]:
                        F["c2"][key] = BRepAdaptor.BRepAdaptor_Curve2d(ce, face)
                    uses.append((ei, rev))
                ex.Next()
            if uses:
                loops.append(uses)
        F["loops"] = loops
        # metric scale of the parameters (middle of the face)
        us, vs = [], []
        for (ei, rev), c2 in F["c2"].items():
            ed = self.edges[ei]
            for k in range(5):
                q = c2.Value(ed["t0"] + (ed["t1"] - ed["t0"]) * k / 4)
                us.append(q.X())
                vs.append(q.Y())
        uc, vc = 0.5 * (min(us) + max(us)), 0.5 * (min(vs) + max(vs))
        p, du, dv = gp_Pnt(), gp_Vec(), gp_Vec()
        F["surf"].D1(uc, vc, p, du, dv)
        F["su"], F["sv"] = max(du.Magnitude(), 1e-12), max(dv.Magnitude(), 1e-12)
        return F

    def S(self, F, uv):
        return (uv[0] * F["su"], uv[1] * F["sv"])

    def US(self, F, xy):
        return (xy[0] / F["su"], xy[1] / F["sv"])

    def lift(self, F, uv):
        p = F["surf"].Value(uv[0], uv[1])
        return (p.X(), p.Y(), p.Z())

    # pieces: dict(c=curve, pa, pb, na, nb, use)
    def piece_uv(self, F, pc, p):
        """(u, v) of the point at curve param p of a piece, in face F."""
        c = pc["c"]
        if c.kind == "edge":
            q = F["c2"][pc["use"]].Value(p)
            return (q.X(), q.Y())
        return c.uv_at(p)

    def piece_poly(self, F, pc, n=None):
        if n is None:
            if pc["c"].kind == "chord":
                n = 2 if len(pc["c"].poly) == 2 else 3 * len(pc["c"].poly)
            else:
                n = 2 if self.edges[pc["c"].ei]["line"] else 24
        out = []
        for k in range(n + 1):
            p = pc["pa"] + (pc["pb"] - pc["pa"]) * k / n
            out.append((p, self.S(F, self.piece_uv(F, pc, p))))
        return out

    def piece_tangent(self, F, pc, at_end):
        span = pc["pb"] - pc["pa"]
        if at_end:
            p1, p0 = pc["pb"], pc["pb"] - span * 0.02
        else:
            p0, p1 = pc["pa"], pc["pa"] + span * 0.02
        return _unit(_sub(self.S(F, self.piece_uv(F, pc, p1)), self.S(F, self.piece_uv(F, pc, p0))))

    def piece_len(self, F, pc):
        poly = self.piece_poly(F, pc)
        return sum(_norm(_sub(poly[k + 1][1], poly[k][1])) for k in range(len(poly) - 1))

    def node_xy(self, F, region, k):
        pc = region[k]
        return self.S(F, self.piece_uv(F, pc, pc["pa"]))

    def turning(self, F, region, k, sigma):
        """Turning at the start node of region[k] (between region[k-1] and region[k])."""
        a = self.piece_tangent(F, region[k - 1], True)
        b = self.piece_tangent(F, region[k], False)
        return sigma * _angle(a, b)

    def split_piece(self, F, region, k, p):
        """Split region[k] at curve param p; returns the index of the piece starting there."""
        pc = region[k]
        c = pc["c"]
        if c.kind == "edge":
            nid = self.new_node(self.lift_edge(c.ei, p), kind="e")
        else:
            uv = self.piece_uv(F, pc, p)
            nid = self.new_node(self.lift(F, uv), kind="c", fi=F["fi"])
            self.nodes[nid]["uv"][F["fi"]] = uv
        nid = c.add_node(p, nid)
        b = dict(pc, pa=p, na=nid)
        pc["pb"], pc["nb"] = p, nid  # keep the first half as the same object (references stay valid)
        region.insert(k + 1, b)
        return k + 1

    def lift_edge(self, ei, p):
        q = self.edges[ei]["c"].Value(p)
        return (q.X(), q.Y(), q.Z())

    def new_chord(self, F, uvA, uvB, na, nb, poly=None):
        cid = ("c", len(self.curves))
        c = self.curves[cid] = Curve(cid, "chord")
        pts = [uvA] + list(poly[1:-1] if poly else []) + [uvB]
        acc = [0.0]
        for k in range(len(pts) - 1):
            acc.append(acc[-1] + _norm(_sub(self.S(F, pts[k + 1]), self.S(F, pts[k]))))
        tot = acc[-1] or 1.0
        c.fi, c.uvA, c.uvB, c.poly, c.cum = F["fi"], uvA, uvB, pts, [a / tot for a in acc]

        def uv_at(f, c=c):
            P, C = c.poly, c.cum
            if f <= 0:
                return P[0]
            if f >= 1:
                return P[-1]
            k = 0
            while k < len(C) - 2 and C[k + 1] < f:
                k += 1
            a = (f - C[k]) / (C[k + 1] - C[k]) if C[k + 1] > C[k] else 0.0
            return (P[k][0] + (P[k + 1][0] - P[k][0]) * a, P[k][1] + (P[k + 1][1] - P[k][1]) * a)

        c.uv_at = uv_at
        c.across = None
        self._last_chord = c
        c.add_node(0.0, na)
        c.add_node(1.0, nb)
        fwd = {"c": c, "pa": 0.0, "pb": 1.0, "na": na, "nb": nb, "use": None}
        bwd = {"c": c, "pa": 1.0, "pb": 0.0, "na": nb, "nb": na, "use": None}
        return fwd, bwd

    def point_at(self, F, region, k, p):
        """Make sure there is a node at param p of region[k]; return index of the piece starting there."""
        pc = region[k]
        span = abs(pc["pb"] - pc["pa"])
        if abs(p - pc["pa"]) <= 1e-9 * (1 + span):
            return k
        if abs(p - pc["pb"]) <= 1e-9 * (1 + span):
            return (k + 1) % len(region)
        return self.split_piece(F, region, k, p)

    def hit(self, F, region, origin_k, o, d, skip):
        best = None
        for k, pc in enumerate(region):
            if k in skip:
                continue
            poly = self.piece_poly(F, pc)
            for j in range(len(poly) - 1):
                r = _ray_seg(o, d, poly[j][1], poly[j + 1][1])
                if r and r[0] > 1e-12 and (best is None or r[0] < best[0]):
                    p = poly[j][0] + (poly[j + 1][0] - poly[j][0]) * r[1]
                    best = (r[0], k, p)
        return best

    def cut(self, F, region, ka, kb, poly=None):
        """Split region by a chord between the start nodes of region[ka] and region[kb]."""
        pa, pb = region[ka], region[kb]
        na, nb = pa["na"], pb["na"]
        uvA = self.piece_uv(F, pa, pa["pa"])
        uvB = self.piece_uv(F, pb, pb["pa"])
        fwd, bwd = self.new_chord(F, uvA, uvB, na, nb, poly)
        n = len(region)
        if ka < kb:
            r1 = region[ka:kb] + [bwd]
            r2 = region[kb:] + region[:ka] + [fwd]
        else:
            r1 = region[ka:] + region[:kb] + [bwd]
            r2 = region[kb:ka] + [fwd]
        return r1, r2

    def snap_or_split(self, F, region, k, p, near):
        """Hit at param p of region[k]: snap to an end node if close (in scaled length), else split."""
        pc = region[k]
        xy = self.S(F, self.piece_uv(F, pc, p))
        a = self.S(F, self.piece_uv(F, pc, pc["pa"]))
        b = self.S(F, self.piece_uv(F, pc, pc["pb"]))
        if _norm(_sub(xy, a)) < near:
            return k
        if _norm(_sub(xy, b)) < near:
            return (k + 1) % len(region)
        return self.split_piece(F, region, k, p)

    # ---- layout of one face -----------------------------------------------------------------
    def layout_face(self, F):
        fi = F["fi"]
        loops = []
        for uses in F["loops"]:
            lp = []
            for ei, rev in uses:
                c = self.edge_curve(ei)
                ed = self.edges[ei]
                pa, pb = (ed["t1"], ed["t0"]) if rev else (ed["t0"], ed["t1"])
                na = self.vertex_node(ed["vb"] if rev else ed["va"])
                nb = self.vertex_node(ed["va"] if rev else ed["vb"])
                lp.append({"c": c, "pa": pa, "pb": pb, "na": na, "nb": nb, "use": (ei, rev)})
            loops.append(lp)
        outer = loops[0]
        poly = [xy for pc in outer for _, xy in self.piece_poly(F, pc)[:-1]]
        area = sum(_cross(poly[k - 1], poly[k]) for k in range(len(poly)))
        sigma = 1.0 if area > 0 else -1.0
        F["sigma"] = sigma
        region = list(outer)

        regions = [list(outer)]
        holes = [list(h) for h in loops[1:]]
        self._reflex_skip = set()

        # guide lines drawn by the user: they cut the face first
        guides = self.face_guides(F)
        if guides:
            regions, holes = self.insert_guides(F, regions, holes, guides)
            self.info["guides_used"] = self.info.get("guides_used", 0) + 1

        # remaining holes
        for hole in holes:
            regions = self.place_hole(F, regions, hole, sigma)

        # reflex corners: motorcycle cuts
        done, guard = [], 0
        while regions:
            guard += 1
            if guard > 200:
                raise LayoutError("too many cuts")
            r = regions.pop()
            worst = None
            skip_nodes = getattr(self, "_reflex_skip", set())
            if guides:
                # nodes where a guide / skeleton cut ends are already resolved
                skip_nodes = skip_nodes | {pc["na"] for pc in r if pc["c"].kind == "chord"} | \
                    {pc["nb"] for pc in r if pc["c"].kind == "chord"}
            for k in range(len(r)):
                t = self.turning(F, r, k, sigma)
                if t < -self.corner and r[k]["na"] not in skip_nodes and (worst is None or t < worst[0]):
                    worst = (t, k)
            if worst is None:
                done.append(r)
                continue
            k = worst[1]
            o = self.node_xy(F, r, k)
            cands = []
            n = len(r)
            for d in (self.piece_tangent(F, r[k - 1], True),
                      tuple(-x for x in self.piece_tangent(F, r[k], False))):
                h = self.hit(F, r, k, o, d, {k, (k - 1) % n})
                if h is not None:
                    cands.append(h)
            if not cands:
                self._reflex_skip = skip_nodes | {r[k]["na"]}
                regions.append(r)
                continue
            L, kh, ph = min(cands)
            sref = r[k]
            kt = self.snap_or_split(F, r, kh, ph, 0.25 * L)
            tref = r[kt]
            ks = next(j for j, pc in enumerate(r) if pc is sref)
            kt = next(j for j, pc in enumerate(r) if pc is tref)
            if tref["na"] == sref["na"] or kt == ks:
                self._reflex_skip = skip_nodes | {sref["na"]}
                regions.append(r)
                continue
            r1, r2 = self.cut(F, r, ks, kt)
            regions += [r1, r2]

        for r in done:
            self.make_patches(F, r, sigma)

    # ---- skeleton (chordal axis) decomposition -----------------------------------------------
    def skeleton_cuts(self, F, loops, prune=2.0):
        """Cuts across the junctions of the shape's skeleton, as (u, v) polylines.

        The boundary is sampled and triangulated (constrained Delaunay, no inner points). Triangles
        with 3 inner edges are junctions, 2 = sleeve, 1 = end (chordal axis transform). Short
        dead-end branches (corners, bumps) are pruned; every remaining junction triangle gives its
        3 edges as cuts. The shape then splits into strips (one per skeleton branch) and junction
        triangles (3 patches around one pole)."""
        S, US = (lambda uv: self.S(F, uv)), (lambda xy: self.US(F, xy))
        per = 0.0
        polys = []
        for lp in loops:
            poly = [xy for pc in lp for _, xy in self.piece_poly(F, pc, 64)[:-1]]
            polys.append(poly)
            per += sum(_norm(_sub(poly[k], poly[k - 1])) for k in range(len(poly)))
        ds = per / 900.0
        pts, segs, loops_xy, loop_of = [], [], [], []
        for poly in polys:
            res = [poly[0]]
            acc = 0.0
            for k in range(1, len(poly) + 1):
                a, b = poly[k - 1], poly[k % len(poly)]
                L = _norm(_sub(b, a))
                while acc + L >= ds:
                    f = (ds - acc) / L if L else 0.0
                    a = (a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f)
                    L = _norm(_sub(b, a))
                    acc = 0.0
                    res.append(a)
                acc += L
            if len(res) > 2 and _norm(_sub(res[-1], res[0])) < 0.5 * ds:
                res.pop()
            start = len(pts)
            pts += res
            loop_of += [len(loops_xy)] * len(res)
            loops_xy.append(res)
            n = len(res)
            segs += [(start + k, start + (k + 1) % n) for k in range(n)]
        tris = self.cdt(pts, segs)
        loops_all = loops_xy
        tris = [t for t in tris if _base._point_in_loops(
            tuple(sum(pts[k][c] for k in t) / 3 for c in range(2)), loops_all)]
        bnd = set((min(a, b), max(a, b)) for a, b in segs)
        etri = {}
        for ti, t in enumerate(tris):
            for k in range(3):
                e = (min(t[k], t[(k + 1) % 3]), max(t[k], t[(k + 1) % 3]))
                etri.setdefault(e, []).append(ti)
        inner = {e: tl for e, tl in etri.items() if len(tl) == 2 and e not in bnd}
        tedges = [[(min(t[k], t[(k + 1) % 3]), max(t[k], t[(k + 1) % 3])) for k in range(3)] for t in tris]
        kind = [sum(1 for e in te if e in inner) for te in tedges]

        def mid(e):
            a, b = pts[e[0]], pts[e[1]]
            return ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)

        def centre(ti):
            t = tris[ti]
            return tuple(sum(pts[k][c] for k in t) / 3 for c in range(2))

        def radius(ti):
            a, b, c = (pts[k] for k in tris[ti])
            la, lb, lc = _norm(_sub(b, c)), _norm(_sub(a, c)), _norm(_sub(a, b))
            ar = abs(_cross(_sub(b, a), _sub(c, a))) / 2
            return la * lb * lc / (4 * ar) if ar > 0 else 0.0

        # branches between nodes (junctions / ends)
        nodes = [ti for ti in range(len(tris)) if kind[ti] != 2]
        branches = []  # (start tri, end tri, start edge, length)
        seen = set()
        for t0 in nodes:
            for e0 in tedges[t0]:
                if e0 not in inner or (t0, e0) in seen:
                    continue
                prev, e, length, pos = t0, e0, 0.0, centre(t0)
                while True:
                    nxt = inner[e][0] if inner[e][1] == prev else inner[e][1]
                    length += _norm(_sub(mid(e), pos))
                    pos = mid(e)
                    if kind[nxt] != 2:
                        length += _norm(_sub(centre(nxt), pos))
                        e_last = e
                        break
                    e = next(x for x in tedges[nxt] if x in inner and x != e)
                    prev = nxt
                seen.add((t0, e0))
                seen.add((nxt, e_last))
                branches.append([t0, nxt, e0, e_last, length, True])
        # prune short dead ends, repeatedly
        changed = True
        while changed:
            changed = False
            deg = {}
            for b in branches:
                if b[5]:
                    deg[b[0]] = deg.get(b[0], 0) + 1
                    deg[b[1]] = deg.get(b[1], 0) + 1
            for b in branches:
                if not b[5]:
                    continue
                for j, end in ((b[0], b[1]), (b[1], b[0])):
                    if deg.get(end, 0) == 1 and deg.get(j, 0) >= 3 and kind[j] == 3:
                        if b[4] < prune * radius(j):
                            b[5] = False
                            changed = True
                            break
                if changed:
                    break
        deg = {}
        for b in branches:
            if b[5]:
                deg[b[0]] = deg.get(b[0], 0) + 1
                deg[b[1]] = deg.get(b[1], 0) + 1
        self.info["skel_dbg"] = [(b[0], b[1], kind[b[0]], kind[b[1]], deg.get(b[0]), deg.get(b[1]), round(b[4] * 1000, 2),
                                  round(radius(b[0]) * 1000, 2), round(radius(b[1]) * 1000, 2)) for b in branches if b[5]]
        # two junctions joined by a short branch form one 4-way junction (a T: the strip runs
        # straight into the other one, no pole)
        skip_edges = set()
        merged = set()
        for b in sorted((b for b in branches if b[5]), key=lambda b: b[4]):
            j1, j2 = b[0], b[1]
            if j1 == j2 or kind[j1] != 3 or kind[j2] != 3 or deg.get(j1, 0) < 3 or deg.get(j2, 0) < 3:
                continue
            if j1 in merged or j2 in merged:
                continue
            if b[4] < 1.0 * (radius(j1) + radius(j2)):
                skip_edges.add((j1, b[2]))
                skip_edges.add((j2, b[3]))
                merged.update((j1, j2))
        # reflex corners of the outline: junction cuts start there when one is close
        poly0 = [xy for pc in loops[0] for _, xy in self.piece_poly(F, pc)[:-1]]
        sg = 1.0 if sum(_cross(poly0[k - 1], poly0[k]) for k in range(len(poly0))) > 0 else -1.0
        reflex = []
        for li, lp in enumerate(loops):
            for k in range(len(lp)):
                if self.turning(F, lp, k, sg) < -math.radians(30):
                    reflex.append((li, self.S(F, self.piece_uv(F, lp[k], lp[k]["pa"]))))

        snap_of = {}
        for ti in nodes:
            if kind[ti] != 3:
                continue
            used = {}
            for i in tris[ti]:
                best = None
                for li, c in reflex:
                    if li != loop_of[i]:
                        continue
                    d = _norm(_sub(c, pts[i]))
                    if d < 1.2 * radius(ti) and (best is None or d < best[0]):
                        best = (d, c)
                if best and (best[1] not in used or used[best[1]][0] > best[0]):
                    if best[1] in used:
                        snap_of.pop((ti, used[best[1]][1]), None)
                    used[best[1]] = (best[0], i)
                    snap_of[(ti, i)] = best[1]

        def snapped(i, ti):
            return snap_of.get((ti, i), pts[i])

        cuts = []
        for ti in nodes:
            if kind[ti] == 3 and deg.get(ti, 0) >= 3:
                for e in tedges[ti]:
                    if (ti, e) in skip_edges:
                        continue
                    a, b = snapped(e[0], ti), snapped(e[1], ti)
                    if _norm(_sub(a, b)) > 1e-9:
                        cuts.append([US(a), US(b)])
        # strip ends: where a surviving branch enters a pruned junction, cut across
        for b in branches:
            if not b[5]:
                continue
            for node, e in ((b[0], b[2]), (b[1], b[3])):
                if kind[node] == 3 and deg.get(node, 0) == 1:
                    a, b_ = snapped(e[0], node), snapped(e[1], node)
                    cuts.append([US(a), US(b_)])
        self.info["skeleton_junctions"] = self.info.get("skeleton_junctions", 0) + len(cuts) // 3
        return cuts

    # ---- user guides ------------------------------------------------------------------------
    def face_guides(self, F):
        """Guide polylines in (u, v) for a planar face facing the guide sketch, else []."""
        if not self.guides:
            return []
        from OCP.GeomAbs import GeomAbs_Plane
        surf = F["surf"]
        if surf.GetType() != GeomAbs_Plane:
            return []
        pln = surf.Plane()
        O = pln.Location()
        N = pln.Axis().Direction()
        X, Y = pln.XAxis().Direction(), pln.YAxis().Direction()
        O = (O.X(), O.Y(), O.Z())
        N = (N.X(), N.Y(), N.Z())
        X = (X.X(), X.Y(), X.Z())
        Y = (Y.X(), Y.Y(), Y.Z())
        normals = self.guides.get("normals") or [self.guides["normal"]] * len(self.guides["lines"])
        across = self.guides.get("across") or [None] * len(self.guides["lines"])
        out = []
        for line, n, acr in zip(self.guides["lines"], normals, across):
            dn = sum(n[k] * N[k] for k in range(3))
            if abs(dn) < math.cos(math.radians(15)):
                continue
            uv = []
            for p in line:
                t = sum((O[k] - p[k]) * N[k] for k in range(3)) / dn
                q = [p[k] + t * n[k] - O[k] for k in range(3)]
                uv.append((sum(q[k] * X[k] for k in range(3)), sum(q[k] * Y[k] for k in range(3))))
            out.append((uv, acr))
        return out

    def _nearest_on(self, F, pieces, xy):
        best = None
        for k, pc in enumerate(pieces):
            poly = self.piece_poly(F, pc, None if pc["c"].kind == "chord" else 48)
            for j in range(len(poly) - 1):
                a, b = poly[j][1], poly[j + 1][1]
                e = _sub(b, a)
                L2 = _dot(e, e)
                t = 0.0 if L2 == 0 else max(0.0, min(1.0, _dot(_sub(xy, a), e) / L2))
                q = (a[0] + e[0] * t, a[1] + e[1] * t)
                d = _norm(_sub(q, xy))
                if best is None or d < best[0]:
                    best = (d, k, poly[j][0] + (poly[j + 1][0] - poly[j][0]) * t)
        return best

    def _crossings(self, F, regions, g, edges_too=False):
        """Positions (segment index + t, point) where polyline g crosses a chord of the regions."""
        out = []
        for r in regions:
            for pc in r:
                if pc["c"].kind != "chord" and not edges_too:
                    continue
                poly = self.piece_poly(F, pc)
                for a in range(len(g) - 1):
                    p, q = g[a], g[a + 1]
                    d1 = _sub(q, p)
                    for j in range(len(poly) - 1):
                        r_, s_ = poly[j][1], poly[j + 1][1]
                        d2 = _sub(s_, r_)
                        den = _cross(d1, d2)
                        if abs(den) < 1e-30:
                            continue
                        t = _cross(_sub(r_, p), d2) / den
                        u = _cross(_sub(r_, p), d1) / den
                        if 0 <= u <= 1 and 0 < t < 1:
                            pos = a + t
                            x = (p[0] + d1[0] * t, p[1] + d1[1] * t)
                            total = sum(_norm(_sub(g[k + 1], g[k])) for k in range(len(g) - 1))
                            along = sum(_norm(_sub(g[k + 1], g[k])) for k in range(a)) + t * _norm(d1)
                            if 0.02 * total < along < 0.98 * total:
                                out.append((pos, x))
        return sorted(set(out))

    @staticmethod
    def _split_polyline(g, marks):
        events = [(float(k), g[k], False) for k in range(len(g))] + [(pos, x, True) for pos, x in marks]
        events.sort(key=lambda e: e[0])
        out, cur = [], []
        for pos, x, is_mark in events:
            if not cur or _norm(_sub(x, cur[-1])) > 1e-12:
                cur.append(x)
            if is_mark:
                if len(cur) >= 2:
                    out.append(cur)
                cur = [x]
        if len(cur) >= 2:
            out.append(cur)
        return out

    def _occurrence(self, F, r, nid, d):
        """Index of the piece starting at node nid whose corner sector contains direction d
        (a node can appear twice in a region joined to a hole)."""
        ks = [k for k, pc in enumerate(r) if pc["na"] == nid]
        if len(ks) <= 1:
            return ks[0] if ks else None
        sg = F.get("sigma", 1.0)
        two = 2 * math.pi
        for k in ks:
            t_out = self.piece_tangent(F, r[k], False)
            t_in = self.piece_tangent(F, r[k - 1], True)
            back = (-t_in[0], -t_in[1])
            a_d = (sg * _angle(t_out, d)) % two
            a_b = (sg * _angle(t_out, back)) % two
            if 0 < a_d < a_b:
                return k
        return ks[0]

    def insert_guides(self, F, regions, holes, guides):
        S, US = (lambda uv: self.S(F, uv)), (lambda xy: self.US(F, xy))
        raw = [([S(p) for p in g], acr) for g, acr in guides if len(g) >= 2]
        # clip the guides to this face (they may come from a parallel face of another shape)
        outer_poly = self.region_poly(F, regions[0])
        hole_polys = [self.region_poly(F, h) for h in holes]
        pieces, across_of = [], {}
        for g, acr in raw:
            xs = self._crossings(F, [regions[0]] + holes, g, edges_too=True)
            for sub in (self._split_polyline(g, xs) if xs else [g]):
                L = sum(_norm(_sub(sub[k + 1], sub[k])) for k in range(len(sub) - 1))
                if L <= 0:
                    continue
                acc, m = 0.0, sub[0]
                for k in range(len(sub) - 1):
                    l = _norm(_sub(sub[k + 1], sub[k]))
                    if acc + l >= L / 2:
                        f = (L / 2 - acc) / l if l else 0.0
                        m = (sub[k][0] + (sub[k + 1][0] - sub[k][0]) * f, sub[k][1] + (sub[k + 1][1] - sub[k][1]) * f)
                        break
                    acc += l
                if _base._point_in_loops(m, [outer_poly]) and not any(_base._point_in_loops(m, [hp]) for hp in hole_polys):
                    pieces.append(sub)
                    across_of[id(sub)] = acr
        allxy = [xy for pc in regions[0] for _, xy in self.piece_poly(F, pc, 8)]
        diag = math.hypot(max(x for x, _ in allxy) - min(x for x, _ in allxy),
                          max(y for _, y in allxy) - min(y for _, y in allxy))
        snap = max(0.0005, 0.02 * diag)
        node_snap = 0.3 * snap
        progress = True
        while pieces and progress:
            progress = False
            for g in list(pieces):
                # a guide that crosses a cut already in place is split there first
                xs = self._crossings(F, regions, g)
                if xs:
                    pieces.remove(g)
                    for sub in self._split_polyline(g, xs):
                        across_of[id(sub)] = across_of.get(id(g))
                        pieces.append(sub)
                    progress = True
                    break
                L = sum(_norm(_sub(g[k + 1], g[k])) for k in range(len(g) - 1))
                acc, mid = 0.0, g[0]
                for k in range(len(g) - 1):
                    l = _norm(_sub(g[k + 1], g[k]))
                    if acc + l >= L / 2:
                        f = (L / 2 - acc) / l if l else 0.0
                        mid = (g[k][0] + (g[k + 1][0] - g[k][0]) * f, g[k][1] + (g[k + 1][1] - g[k][1]) * f)
                        break
                    acc += l
                ri = next((i for i, r in enumerate(regions) if self.inside(F, r, mid)), None)
                if ri is None or any(_base._point_in_loops(mid, [self.region_poly(F, h)]) for h in holes):
                    pieces.remove(g)  # outside this face
                    progress = True
                    continue
                r = regions[ri]
                ends = []
                for xy in (g[0], g[-1]):
                    cand = [("r", None, self._nearest_on(F, r, xy))]
                    for hi, h in enumerate(holes):
                        if self.inside(F, r, self.region_poly(F, h)[0]):
                            cand.append(("h", hi, self._nearest_on(F, h, xy)))
                    cand = [c for c in cand if c[2] is not None]
                    best = min(cand, key=lambda c: c[2][0])
                    ends.append(best if best[2][0] <= snap else None)
                if None in ends:
                    continue
                if ends[0][0] == "h" and ends[1][0] == "h":
                    continue
                pieces.remove(g)
                progress = True
                if ends[0][0] == "h":
                    ends = ends[::-1]
                    g = g[::-1]
                (_, _, (_, kr, pr)), (kind2, hi, (_, k2, p2)) = ends
                if kind2 == "h":
                    h = holes.pop(hi)
                    ih = self.snap_or_split(F, h, k2, p2, node_snap)
                    ir = self.snap_or_split(F, r, kr, pr, node_snap)
                    regions[ri] = self._join(F, r, ir, h, ih, [US(x) for x in g])
                    self._last_chord.across = across_of.get(id(g))
                    continue
                ka = self.snap_or_split(F, r, kr, pr, node_snap)
                aref = r[ka]
                # second end: positions may have shifted after the first split
                d2 = self._nearest_on(F, r, g[-1])
                kb = self.snap_or_split(F, r, d2[1], d2[2], node_snap)
                bref = r[kb]
                ka = self._occurrence(F, r, aref["na"], _unit(_sub(g[1], g[0])))
                kb = self._occurrence(F, r, bref["na"], _unit(_sub(g[-2], g[-1])))
                if ka is None or kb is None or ka == kb or aref["na"] == bref["na"]:
                    continue
                r1, r2 = self.cut(F, r, ka, kb, [US(x) for x in g])
                self._last_chord.across = across_of.get(id(g))
                regions[ri:ri + 1] = [r1, r2]
        if pieces:
            self.info["warnings"].append(f"face {F['fi']}: {len(pieces)} guide piece(s) not connected")
        return regions, holes

    def region_poly(self, F, r):
        return [xy for pc in r for _, xy in self.piece_poly(F, pc, 8)[:-1]]

    def inside(self, F, r, xy):
        return _base._point_in_loops(xy, [self.region_poly(F, r)])

    def chord_inside(self, F, r, xa, xb, n=3):
        poly = self.region_poly(F, r)
        return all(_base._point_in_loops((xa[0] + (xb[0] - xa[0]) * f, xa[1] + (xb[1] - xa[1]) * f), [poly])
                   for f in [(k + 1) / (n + 1) for k in range(n)])

    def sub_angles(self, F, r, k, d):
        """Angles between a chord leaving the start node of r[k] (direction d) and the boundary."""
        t_out = self.piece_tangent(F, r[k], False)
        t_in = self.piece_tangent(F, r[k - 1], True)
        back = (-t_in[0], -t_in[1])
        a1 = abs(_angle(t_out, d))
        a2 = abs(_angle(d, back))
        return [a1, a2]

    def place_hole(self, F, regions, hole, sigma):
        hp = [(k, p, xy) for k, pc in enumerate(hole) for p, xy in self.piece_poly(F, pc, 24)[:-1]]
        cen = (sum(x for _, _, (x, _) in hp) / len(hp), sum(y for _, _, (_, y) in hp) / len(hp))
        # region that contains the hole
        ri = next((i for i, r in enumerate(regions) if self.inside(F, r, hp[0][2])), 0)
        region = regions.pop(ri)
        reflex = any(self.turning(F, hole, k, sigma) < -self.corner for k in range(len(hole)))
        if reflex:
            # polygonal hole: join by the shortest link; reflex cuts do the rest
            best = None
            rp = [(k, p, xy) for k, pc in enumerate(region) for p, xy in self.piece_poly(F, pc, 12)]
            for kh, ph, xh in hp:
                for kr, pr, xr in rp:
                    d = _norm(_sub(xh, xr))
                    if best is None or d < best[0]:
                        best = (d, kh, ph, kr, pr)
            _, kh, ph, kr, pr = best
            ih = self.snap_or_split(F, hole, kh, ph, 1e-12)
            ir = self.snap_or_split(F, region, kr, pr, 1e-12)
            regions.append(self._join(F, region, ir, hole, ih))
            return regions
        # smooth hole: 4 cuts. Aim at the 4 corners of the region if it has exactly 4,
        # otherwise go out along the principal axes of the hole.
        turns = [self.turning(F, region, k, sigma) for k in range(len(region))]
        rc = [k for k in range(len(region)) if turns[k] > self.corner]
        targets = []
        if len(rc) == 4:
            for k in rc:
                xy = self.node_xy(F, region, k)
                targets.append(("corner", region[k], _unit(_sub(xy, cen))))
        else:
            sxx = sum((x - cen[0]) ** 2 for _, _, (x, y) in hp)
            syy = sum((y - cen[1]) ** 2 for _, _, (x, y) in hp)
            sxy = sum((x - cen[0]) * (y - cen[1]) for _, _, (x, y) in hp)
            th = 0.5 * math.atan2(2 * sxy, sxx - syy)
            for k in range(4):
                a = th + k * math.pi / 2
                targets.append(("dir", None, (math.cos(a), math.sin(a))))
        targets.sort(key=lambda t: math.atan2(t[2][1], t[2][0]))
        # hole points: the hole sample furthest along each direction
        hole_pts = []
        for kind, ref, d in targets:
            k, p, xy = max(hp, key=lambda e: _dot(_sub(e[2], cen), d))
            hole_pts.append((k, p))
        # split the hole at those points (descending order keeps indices valid)
        refs = [None] * 4
        order = sorted(range(4), key=lambda i: (hole_pts[i][0], hole_pts[i][1] * (1 if hole[hole_pts[i][0]]["pb"] >= hole[hole_pts[i][0]]["pa"] else -1)), reverse=True)
        for i in order:
            k, p = hole_pts[i]
            j = self.snap_or_split(F, hole, k, p, 1e-12)
            refs[i] = hole[j]
        if len({id(x) for x in refs}) < 4:
            raise LayoutError("hole points")
        # first cut joins the hole to the region
        kind, tref, d = targets[0]
        o = self.S(F, self.piece_uv(F, refs[0], refs[0]["pa"]))
        if kind == "corner":
            ir = next(k for k, pc in enumerate(region) if pc is tref)
        else:
            h = self.hit(F, region, None, o, d, set())
            if h is None:
                raise LayoutError("hole cut")
            ir = self.snap_or_split(F, region, h[1], h[2], 0.25 * h[0])
        ih = next(k for k, pc in enumerate(hole) if pc is refs[0])
        region = self._join(F, region, ir, hole, ih)
        out = [region]
        for i in range(1, 4):
            kind, tref, d = targets[i]
            r = next(rr for rr in out if any(pc is refs[i] for pc in rr))
            out.remove(r)
            ks = next(k for k, pc in enumerate(r) if pc is refs[i])
            o = self.node_xy(F, r, ks)
            if kind == "corner" and any(pc is tref for pc in r):
                kt = next(k for k, pc in enumerate(r) if pc is tref)
                if not self.chord_inside(F, r, o, self.node_xy(F, r, kt)):
                    raise LayoutError("hole cut")
            else:
                h = self.hit(F, r, ks, o, d, {ks, (ks - 1) % len(r)})
                if h is None:
                    raise LayoutError("hole cut")
                kt = self.snap_or_split(F, r, h[1], h[2], 0.25 * h[0])
            tr = r[kt]
            ks = next(k for k, pc in enumerate(r) if pc is refs[i])
            kt = next(k for k, pc in enumerate(r) if pc is tr)
            if ks == kt or tr["na"] == refs[i]["na"]:
                raise LayoutError("hole cut")
            r1, r2 = self.cut(F, r, ks, kt)
            out += [r1, r2]
        return regions + out

    def _join(self, F, region, ir, hole, ih, poly=None):
        uvA = self.piece_uv(F, region[ir], region[ir]["pa"])
        uvB = self.piece_uv(F, hole[ih], hole[ih]["pa"])
        fwd, bwd = self.new_chord(F, uvA, uvB, region[ir]["na"], hole[ih]["na"], poly)
        return region[ir:] + region[:ir] + [fwd] + hole[ih:] + hole[:ih] + [bwd]

    # ---- patches ----------------------------------------------------------------------------
    def chain_split(self, F, r, ks, ke, frac):
        """Node at a fraction of the length of the chain r[ks..ke) (cyclic); returns piece index."""
        n = len(r)
        idx = [(ks + j) % n for j in range(((ke - ks) % n) or n)]
        lengths = [self.piece_len(F, r[k]) for k in idx]
        target, acc = sum(lengths) * frac, 0.0
        for k, L in zip(idx, lengths):
            if acc + L >= target - 1e-15:
                pc = r[k]
                f = (target - acc) / L if L > 0 else 0.0
                # snap to an existing junction when close (15 % of the chain)
                if f * L < 0.15 * sum(lengths):
                    return k
                if (1 - f) * L < 0.15 * sum(lengths):
                    return (k + 1) % n
                return self.split_piece(F, r, k, pc["pa"] + (pc["pb"] - pc["pa"]) * f)
            acc += L
        return ke

    def make_patches(self, F, r, sigma, depth=0):
        if depth > 12:
            raise LayoutError("patch recursion")
        turns = [self.turning(F, r, k, sigma) for k in range(len(r))]
        corners = [r[k] for k in range(len(r)) if turns[k] > self.corner]
        ix = lambda ref: next(j for j, pc in enumerate(r) if pc is ref)
        # strips: a region bounded by cuts takes its corners from the cut ends
        cut_sides = []  # (index of first chord piece, index after last chord piece)
        n_r = len(r)
        is_c = [pc["c"].kind == "chord" for pc in r]
        if any(is_c) and not all(is_c):
            k0 = next(k for k in range(n_r) if is_c[k] and not is_c[k - 1])
            k = k0
            for _ in range(n_r):
                if is_c[k] and not is_c[k - 1]:
                    j = k
                    while is_c[j % n_r]:
                        j += 1
                    cut_sides.append((k, j % n_r))
                k = (k + 1) % n_r
                if k == k0:
                    break
        if len(cut_sides) == 2 and self.strip_mode:
            (a0, a1), (b0, b1) = cut_sides
            cand = [r[a0], r[a1], r[b0], r[b1]]
            if len({id(c) for c in cand}) == 4:
                corners = sorted(cand, key=ix)
        elif len(cut_sides) == 1 and self.strip_mode:
            a0, a1 = cut_sides[0]
            ends = [r[a0], r[a1]]
            others = sorted((k for k in range(n_r) if r[k] not in ends and turns[k] > math.radians(30)
                             and not is_c[k] and not is_c[k - 1]), key=lambda k: -turns[k])
            if len(others) >= 2:
                corners = sorted(ends + [r[k] for k in others[:2]], key=ix)

        # 6 corners or more: split by the best inner chord between two corners
        if len(corners) >= 6:
            n = len(corners)
            best = None
            for i in range(n):
                for gap in (3,):
                    j = (i + gap) % n
                    if n == 6 and j < i:
                        continue
                    a, b = corners[i], corners[j]
                    xa = self.node_xy(F, r, ix(a))
                    xb = self.node_xy(F, r, ix(b))
                    L = _norm(_sub(xb, xa))
                    if L <= 0:
                        continue
                    h = self.hit(F, r, ix(a), xa, _unit(_sub(xb, xa)), {ix(a), (ix(a) - 1) % len(r)})
                    if h is not None and h[0] < L * (1 - 1e-6):
                        continue  # the chord leaves the region
                    if not self.chord_inside(F, r, xa, xb, 9):
                        continue
                    d = _unit(_sub(xb, xa))
                    ang = min(self.sub_angles(F, r, ix(a), d) + self.sub_angles(F, r, ix(b), (-d[0], -d[1])))
                    if ang < math.radians(25):
                        continue
                    n1, n2 = gap + 1, n - gap + 1
                    pen = sum(0 if k == 4 else (1 if k in (3, 5) else 3) for k in (n1, n2))
                    score = (pen, L / math.sin(min(ang, math.pi / 2)))
                    if best is None or score < best[0]:
                        best = (score, a, b)
            if best is not None:
                r1, r2 = self.cut(F, r, ix(best[1]), ix(best[2]))
                self.make_patches(F, r1, sigma, depth + 1)
                self.make_patches(F, r2, sigma, depth + 1)
                return
            order = sorted(range(len(corners)), key=lambda k: -turns[ix(corners[k])])
            corners = [corners[k] for k in sorted(order[:4])]

        def split_at(ref_from, ref_to, frac):
            k = self.chain_split(F, r, ix(ref_from), ix(ref_to), frac)
            return r[k]

        disk = False
        if len(corners) == 0:
            disk = True
            c0 = r[0]
            corners = [c0] + [split_at(c0, c0, f) for f in (0.25, 0.5, 0.75)]
        elif len(corners) == 1:
            c0 = corners[0]
            c1 = split_at(c0, c0, 1 / 3)
            c2 = split_at(c1, c0, 0.5)
            corners = [c0, c1, c2]
        elif len(corners) == 2:
            a, b = corners
            m1 = split_at(a, b, 0.5)
            m2 = split_at(b, a, 0.5)
            corners = [a, m1, b, m2]
        corners.sort(key=ix)
        if len({id(c) for c in corners}) != len(corners):
            raise LayoutError("corner collapse")

        def chain(ref_from, ref_to):
            a, b = ix(ref_from), ix(ref_to)
            out, k = [], a
            while True:
                out.append(r[k])
                k = (k + 1) % len(r)
                if k == b:
                    return out

        m = len(corners)
        if m == 4 and not disk:
            self.patches.append((F["fi"], [chain(corners[i], corners[(i + 1) % 4]) for i in range(4)]))
            self.info["patches"] += 1
            return
        # centre patch: split every side at its middle, spokes to a centre point
        mids = []
        for i in range(m):
            a, b = corners[i], corners[(i + 1) % m]
            mid = split_at(a, b, 0.5)
            if mid is a or mid is b:
                raise LayoutError("side too short for a centre patch")
            mids.append(mid)
        uvs = [self.piece_uv(F, mref, mref["pa"]) for mref in mids]
        cen = (sum(u for u, _ in uvs) / m, sum(v for _, v in uvs) / m)
        nc = self.new_node(self.lift(F, cen), kind="center", fi=F["fi"])
        self.nodes[nc]["uv"][F["fi"]] = cen
        spokes = [self.new_chord(F, uv, cen, mref["na"], nc) for mref, uv in zip(mids, uvs)]
        for i in range(m):
            c_prev = chain(mids[i - 1], corners[i])
            c_next = chain(corners[i], mids[i])
            self.patches.append((F["fi"], [c_prev, c_next, [spokes[i][0]], [spokes[i - 1][1]]]))
            self.info["patches"] += 1
        self.info["poles"] += 1

    # ---- solve ------------------------------------------------------------------------------
    def segments_of(self, pc):
        """Segments (curve id, k) covered by a piece, and their direction."""
        c = pc["c"]
        ps = c.sorted
        lo, hi = sorted((pc["pa"], pc["pb"]))
        ks = [k for k in range(len(ps) - 1) if ps[k][0] >= lo - 1e-12 * (1 + abs(lo)) and ps[k + 1][0] <= hi + 1e-12 * (1 + abs(hi))]
        fwd = pc["pb"] >= pc["pa"]
        if not fwd:
            ks = ks[::-1]
        return [((c.id, k), fwd) for k in ks]

    def seg_measure(self, s):
        cid, k = s
        c = self.curves[cid]
        p0, p1 = c.sorted[k][0], c.sorted[k + 1][0]
        if c.kind == "edge":
            return abs(self.measure(c.ei, p1) - self.measure(c.ei, p0))
        a = self.nodes[c.sorted[k][1]]["xyz"]
        b = self.nodes[c.sorted[k + 1][1]]["xyz"]
        L = math.dist(a, b)
        m = L / self.max_edge if self.max_edge > 0 else 0.0
        if getattr(c, "across", None):
            m = max(m, round(c.across * abs(p1 - p0)) - 1e-3)
        return m

    def solve(self, fallback_faces):
        """Segment counts: opposite sides of each patch equal, as few segments as possible.
        Patches that make the system inconsistent are freed (meshed on their own, a few poles
        inside) instead of failing the whole face."""
        for c in self.curves.values():
            c.sorted = sorted(c.nodes)
        lb = {}
        for c in self.curves.values():
            for k in range(len(c.sorted) - 1):
                s = (c.id, k)
                lb[s] = max(1, math.ceil(self.seg_measure(s) - 1e-6))
        self.segs_min = dict(lb)
        chains_s = []
        for pi, (fi, chains) in enumerate(self.patches):
            chains_s.append([[s for pc in ch for s, _ in self.segments_of(pc)] for ch in chains])
        par_fixed = []
        for fi, loops in self.fallback_loops.items():
            for lp in loops:
                par_fixed.append([s for pc in lp for s, _ in self.segments_of(pc)])
        self.free = set()
        meas = {s: self.seg_measure(s) + 1e-3 for s in lb}
        for attempt in range(len(self.patches) + 1):
            segs = dict(lb)
            eqs = []
            par = list(par_fixed)
            for pi, cs in enumerate(chains_s):
                if pi in self.free:
                    par.append(cs[0] + cs[1] + cs[2] + cs[3])
                    continue
                for A, B in ((cs[0], cs[2]), (cs[1], cs[3])):
                    coef = {}
                    for x in A:
                        coef[x] = coef.get(x, 0) + 1
                    for x in B:
                        coef[x] = coef.get(x, 0) - 1
                    coef = {x: c for x, c in coef.items() if c}
                    eqs.append((pi, coef))
            fires = {}
            ok = False
            for it in range(400):
                changed = False
                blown = None
                for pi, coef in eqs:
                    diff = sum(segs[x] * c for x, c in coef.items())
                    if diff == 0:
                        continue
                    cand = [x for x, c in coef.items() if (c > 0) == (diff < 0)]
                    if not cand:
                        blown = pi
                        break
                    x = max(cand, key=lambda y: meas[y] / segs[y])
                    segs[x] += math.ceil(abs(diff) / abs(coef[x]))
                    fires[pi] = fires.get(pi, 0) + 1
                    changed = True
                    if segs[x] > 4 * lb[x] + 16:
                        blown = pi
                        break
                if blown is not None:
                    break
                for L in par:
                    if sum(segs[x] for x in L) % 2:
                        segs[max(L, key=lambda y: meas[y] / segs[y])] += 1
                        changed = True
                if not changed:
                    ok = True
                    break
            if ok:
                self.segs = segs
                self.info["free_patches"] = len(self.free)
                return
            # free the patch that had to be corrected most often
            worst = max(fires, key=fires.get) if fires else blown
            if worst is None or worst in self.free:
                raise LayoutError(("count", None))
            self.free.add(worst)
        raise LayoutError(("count", None))

    # ---- build ------------------------------------------------------------------------------
    def build(self):
        verts, keys = [], {}

        def vid(key, xyz):
            j = keys.get(key)
            if j is None:
                j = keys[key] = len(verts)
                verts.append(xyz)
            return j

        self.vid = vid
        self._moved = {}
        nid_v = lambda n: vid(("n", n), self.nodes[n]["xyz"])
        # samples of every segment, in curve direction: list of (vertex id, param)
        samples = {}
        for s, n in self.segs.items():
            cid, k = s
            c = self.curves[cid]
            (p0, n0), (p1, n1) = c.sorted[k], c.sorted[k + 1]
            pts = [(nid_v(n0), p0)]
            if c.kind == "edge":
                m0, m1 = self.measure(c.ei, p0, "Md"), self.measure(c.ei, p1, "Md")
                for j in range(1, n):
                    p = self.inv_measure(c.ei, m0 + (m1 - m0) * j / n)
                    pts.append((vid(("s", cid, k, j), self.lift_edge(c.ei, p)), p))
            else:
                F = self.faces[c.fi]
                for j in range(1, n):
                    p = p0 + (p1 - p0) * j / n
                    uv = c.uv_at(p)
                    pts.append((vid(("s", cid, k, j), self.lift(F, uv)), p))
            pts.append((nid_v(n1), p1))
            samples[s] = pts
        self.samples = samples

        faces_out, face_of = [], []
        for pi, (fi, chains) in enumerate(self.patches):
            F = self.faces[fi]
            pts4 = []
            for ch in chains:
                line = []
                for pc in ch:
                    for s, fwd in self.segments_of(pc):
                        sp = samples[s] if fwd else samples[s][::-1]
                        for j, (v, p) in enumerate(sp):
                            if line and j == 0:
                                continue
                            line.append((v, self.piece_uv(F, pc, p)))
                pts4.append(line)
            quads = None
            if pi not in self.free:
                try:
                    quads = self._coons(F, pi, pts4)
                except LayoutError as e:
                    if e.args and isinstance(e.args[0], tuple) and e.args[0][0] == "fold":
                        self.free.add(pi)  # opposite sides already match: the loop count is even
                        self.info["folded_patches"] = self.info.get("folded_patches", 0) + 1
                    else:
                        raise
            if pi in self.free:
                line = []
                for ch in pts4:
                    for j, e in enumerate(ch):
                        if line and j == 0:
                            continue
                        line.append(e)
                if line[0][0] != line[-1][0]:
                    line.append(line[0])
                L = [math.dist(verts[line[k][0]], verts[line[k + 1][0]]) for k in range(len(line) - 1)]
                H = 2.0 * sum(L) / max(len(L), 1)
                lift = lambda u, v, F=F: self.lift(F, (u, v))
                quads = _base._general_face(("p", pi), [[line]], F["surf"], lift, vid, H, self.cdt, self.info)
                if F["flip"]:
                    quads = [q[::-1] for q in quads]
            faces_out.append((fi, quads))
        for k, xyz in self._moved.items():
            verts[k] = xyz
        return verts, faces_out

    def _coons(self, F, pi, pts4):
        b, r, t, l = pts4[0], pts4[1], pts4[2][::-1], pts4[3][::-1]
        nA, nB = len(b) - 1, len(r) - 1
        if len(t) - 1 != nA or len(l) - 1 != nB:
            raise LayoutError(("coons", F["fi"]))
        S = lambda uv: self.S(F, uv)

        def arc(line):
            acc = [0.0]
            for k in range(len(line) - 1):
                acc.append(acc[-1] + _norm(_sub(S(line[k + 1][1]), S(line[k][1]))))
            tot = acc[-1] or 1.0
            return [a / tot for a in acc]

        sb, st, tl, tr = arc(b), arc(t), arc(l), arc(r)
        P00, P10, P11, P01 = b[0][1], b[-1][1], t[-1][1], t[0][1]
        grid = [[None] * (nB + 1) for _ in range(nA + 1)]
        uvg = [[None] * (nB + 1) for _ in range(nA + 1)]
        for i in range(nA + 1):
            for j in range(nB + 1):
                if j == 0:
                    grid[i][j], uvg[i][j] = b[i]
                elif j == nB:
                    grid[i][j], uvg[i][j] = t[i]
                elif i == 0:
                    grid[i][j], uvg[i][j] = l[j]
                elif i == nA:
                    grid[i][j], uvg[i][j] = r[j]
                else:
                    tt = 0.5 * (tl[j] + tr[j])
                    for _ in range(3):
                        s = (1 - tt) * sb[i] + tt * st[i]
                        tt = (1 - s) * tl[j] + s * tr[j]
                    uv = tuple((1 - tt) * b[i][1][c] + tt * t[i][1][c] + (1 - s) * l[j][1][c] + s * r[j][1][c]
                               - ((1 - s) * (1 - tt) * P00[c] + s * (1 - tt) * P10[c]
                                  + s * tt * P11[c] + (1 - s) * tt * P01[c]) for c in range(2))
                    grid[i][j] = self.vid(("g", pi, i, j), self.lift(F, uv))
                    uvg[i][j] = uv
        def folds():
            signs = []
            for i in range(nA):
                for j in range(nB):
                    q = [(i, j), (i + 1, j), (i + 1, j + 1), (i, j + 1)]
                    xy = [S(uvg[a][c]) for a, c in q]
                    signs.append(sum(_cross(xy[k - 1], xy[k]) for k in range(4)))
            pos = sum(1 for a in signs if a > 0)
            neg = sum(1 for a in signs if a < 0)
            return (pos and neg), neg > pos

        folded, negative = folds()
        if folded and nA > 1 and nB > 1:
            # untangle: Laplacian smoothing of the inner (u, v) points, boundary fixed
            for it in range(300):
                for i in range(1, nA):
                    for j in range(1, nB):
                        a, b, c, d = uvg[i - 1][j], uvg[i + 1][j], uvg[i][j - 1], uvg[i][j + 1]
                        uvg[i][j] = ((a[0] + b[0] + c[0] + d[0]) / 4, (a[1] + b[1] + c[1] + d[1]) / 4)
                if it % 25 == 24:
                    folded, negative = folds()
                    if not folded:
                        break
            if not folded:
                for i in range(1, nA):
                    for j in range(1, nB):
                        self._moved[grid[i][j]] = self.lift(F, uvg[i][j])
        if folded:
            raise LayoutError(("fold", F["fi"]))
        quads = []
        for i in range(nA):
            for j in range(nB):
                q = [(i, j), (i + 1, j), (i + 1, j + 1), (i, j + 1)]
                quads.append([grid[a][c] for a, c in q])
        neg = negative
        if neg:
            quads = [q[::-1] for q in quads]
        if F["flip"]:
            quads = [q[::-1] for q in quads]
        return quads

    # ---- driver -----------------------------------------------------------------------------
    def auto_guides(self):
        """Skeleton cuts of the largest flat face of each orientation, as 3D guides."""
        from OCP.GeomAbs import GeomAbs_Plane
        from OCP.GProp import GProp_GProps
        from OCP.BRepGProp import BRepGProp
        groups = []
        for fi in range(1, self.fmap.Extent() + 1):
            F = self.face_ctx(fi)
            if not F["loops"] or F["surf"].GetType() != GeomAbs_Plane:
                continue
            d = F["surf"].Plane().Axis().Direction()
            n = (d.X(), d.Y(), d.Z())
            gp = GProp_GProps()
            fn = getattr(BRepGProp, "SurfaceProperties_s", None) or BRepGProp.SurfaceProperties
            fn(F["face"], gp)
            area = gp.Mass()
            for g in groups:
                if abs(sum(n[k] * g["n"][k] for k in range(3))) > math.cos(math.radians(5)):
                    if area > g["area"]:
                        g.update(F=F, area=area)
                    g["count"] += 1
                    g["faces"].append(F)
                    break
            else:
                groups.append({"n": n, "F": F, "area": area, "count": 1, "faces": [F]})
        lines, normals, across = [], [], []
        # the caps: the orientation with the largest flat face (and a twin face facing it)
        groups = [g for g in groups if g["count"] >= 2]
        groups = sorted(groups, key=lambda g: -g["area"])[:1]
        for g in groups:
            F = g["F"]
            loops = []
            for uses in F["loops"]:
                lp = []
                for ei, rev in uses:
                    c = self.edge_curve(ei)
                    ed = self.edges[ei]
                    pa, pb = (ed["t1"], ed["t0"]) if rev else (ed["t0"], ed["t1"])
                    lp.append({"c": c, "pa": pa, "pb": pb, "na": None, "nb": None, "use": (ei, rev)})
                loops.append(lp)
            try:
                cuts = self.skeleton_cuts(F, loops)
            except Exception as e:
                self.info["warnings"].append(f"skeleton: {e}")
                continue
            for cut in cuts:
                lines.append([self.lift(F, uv) for uv in cut])
                normals.append(g["n"])
                across.append(self.across)
            # outlines of the other flat faces (steps, pockets) become cuts of the master face,
            # so that every level of the part shares the same patches
            master_xy = [xy for lp in loops for pc in lp for _, xy in self.piece_poly(F, pc, 48)]
            master_seg = list(zip(master_xy, master_xy[1:]))
            pln = F["surf"].Plane()
            O, N = pln.Location(), pln.Axis().Direction()
            X, Y = pln.XAxis().Direction(), pln.YAxis().Direction()
            O, N, X, Y = ((O.X(), O.Y(), O.Z()), (N.X(), N.Y(), N.Z()), (X.X(), X.Y(), X.Z()), (Y.X(), Y.Y(), Y.Z()))

            def to_xy(p):
                q = [p[k] - O[k] for k in range(3)]
                d = sum(q[k] * N[k] for k in range(3))
                q = [q[k] - d * N[k] for k in range(3)]
                return self.S(F, (sum(q[k] * X[k] for k in range(3)), sum(q[k] * Y[k] for k in range(3))))

            tol = 1e-3 * max(1e-9, max(_norm(_sub(a, b)) for a, b in master_seg)) + 2e-6
            done_e = {ei for lp in loops for pc in lp for ei in [pc["use"][0]]}
            for G in g["faces"]:
                if G is F:
                    continue
                for (ei, rev) in G["c2"]:
                    if ei in done_e:
                        continue
                    done_e.add(ei)
                    ed = self.edges[ei]
                    n_s = 2 if ed["line"] else 24
                    pts3 = [self.lift_edge(ei, ed["t0"] + (ed["t1"] - ed["t0"]) * k / n_s) for k in range(n_s + 1)]
                    mids = [to_xy(pts3[n_s // 2]), to_xy(pts3[max(1, n_s // 4)])]
                    on_outline = all(min(_base._dist_seg(m, a, b) for a, b in master_seg) < tol for m in mids)
                    if on_outline:
                        continue
                    lines.append(pts3)
                    normals.append(g["n"])
                    across.append(None)
        return {"lines": lines, "normals": normals, "across": across, "normal": (0, 0, 1)} if lines else None

    def run(self):
        self.prepare_edges()
        if self.auto:
            self.reset()
            ag = self.auto_guides()
            if ag:
                if self.guides:
                    n = len(self.guides["lines"])
                    self.guides = {"lines": list(self.guides["lines"]) + ag["lines"],
                                   "normals": list(self.guides.get("normals") or [self.guides["normal"]] * n) + ag["normals"],
                                   "across": list(self.guides.get("across") or [None] * n) + ag["across"],
                                   "normal": self.guides["normal"]}
                else:
                    self.guides = ag
        fallback = set()
        for attempt in range(8):
            self.reset()
            self.faces = {}
            failed = None
            for fi in range(1, self.fmap.Extent() + 1):
                F = self.faces[fi] = self.face_ctx(fi)
                if not F["loops"]:
                    continue
                if fi in fallback:
                    self.fallback_loops[fi] = self._plain_loops(F)
                    continue
                n_p = len(self.patches)
                try:
                    self.layout_face(F)
                except LayoutError as e:
                    del self.patches[n_p:]
                    fallback.add(fi)
                    self.info["warnings"].append(f"face {fi}: {e}")
                    failed = fi
                    break
            if failed is not None:
                continue
            try:
                self.solve(fallback)
                verts, patch_faces = self.build()
            except LayoutError as e:
                fi = e.args[0][1] if isinstance(e.args[0], tuple) else None
                if fi is None or fi in fallback:
                    raise
                fallback.add(fi)
                self.info["warnings"].append(f"face {fi}: {e}")
                continue
            break
        else:
            raise LayoutError("layout did not converge")

        out_faces, face_of, patch_of = [], [], []
        for pi, (fi, quads) in enumerate(patch_faces):
            out_faces += quads
            face_of += [fi] * len(quads)
            patch_of += [pi] * len(quads)
        # fallback faces
        for fi, loops in self.fallback_loops.items():
            F = self.faces[fi]
            loops_pts = []
            seglen = []
            for lp in loops:
                line = []
                for pc in lp:
                    for s, fwd in self.segments_of(pc):
                        sp = self.samples[s] if fwd else self.samples[s][::-1]
                        for j, (v, p) in enumerate(sp):
                            if line and j == 0:
                                continue
                            line.append((v, self.piece_uv(F, pc, p)))
                if line[0][0] == line[-1][0]:
                    pass
                else:
                    line.append(line[0])
                loops_pts.append([line])
                seglen += [math.dist(verts[line[k][0]], verts[line[k + 1][0]]) for k in range(len(line) - 1)]
            H = 2.0 * (sum(seglen) / max(len(seglen), 1))
            lift = lambda u, v, F=F: self.lift(F, (u, v))
            quads = _base._general_face(fi, loops_pts, F["surf"], lift, self.vid, H, self.cdt, self.info)
            if F["flip"]:
                quads = [q[::-1] for q in quads]
            out_faces += quads
            face_of += [fi] * len(quads)
            patch_of += [-1] * len(quads)
            self.info["fallback"] += 1
        self.verts = verts
        self.patch_of = patch_of
        return verts, out_faces, face_of

    def _plain_loops(self, F):
        loops = []
        for uses in F["loops"]:
            lp = []
            for ei, rev in uses:
                c = self.edge_curve(ei)
                ed = self.edges[ei]
                pa, pb = (ed["t1"], ed["t0"]) if rev else (ed["t0"], ed["t1"])
                lp.append({"c": c, "pa": pa, "pb": pb, "na": None, "nb": None, "use": (ei, rev)})
            loops.append(lp)
        return loops

    def sharp_edges(self, verts_faces=None):
        """Mesh edges lying on non-tangent B-rep edges."""
        TopAbs, TopExp, BRep, BRepAdaptor, BRepTools, GeomAbs, gp_Pnt, gp_Vec, gp_Pnt2d = self.O
        lim = math.cos(math.radians(_base.SHARP_ANGLE_DEG))
        normals = {}
        for fi, F in self.faces.items():
            for (ei, rev), c2 in F["c2"].items():
                ed = self.edges[ei]
                q = c2.Value(0.5 * (ed["t0"] + ed["t1"]))
                p, du, dv = gp_Pnt(), gp_Vec(), gp_Vec()
                F["surf"].D1(q.X(), q.Y(), p, du, dv)
                nn = du.Crossed(dv)
                if nn.Magnitude() < 1e-30:
                    continue
                nn.Normalize()
                n = (-nn.X(), -nn.Y(), -nn.Z()) if F["flip"] else (nn.X(), nn.Y(), nn.Z())
                normals.setdefault(ei, []).append((fi, n))
        sharp = set()
        for ei, lst in normals.items():
            if len({f for f, _ in lst}) < 2:
                continue
            n1, n2 = lst[0][1], lst[1][1]
            if sum(n1[k] * n2[k] for k in range(3)) < lim:
                c = self.curves.get(("e", ei))
                if c is None:
                    continue
                for k in range(len(c.sorted) - 1):
                    ids = [v for v, _ in self.samples[(c.id, k)]]
                    for a, b in zip(ids, ids[1:]):
                        sharp.add((min(a, b), max(a, b)))
        return sharp


def quad_mesh(solid, tolerance=0.0001, max_edge=0.0, max_angle_deg=45.0, corner_deg=40.0, cdt=None,
              guides=None, skeleton=False, across=2):
    """Return (verts, faces, sharp_edges, info); lengths in the solid's units.

    guides: {"normal": (x, y, z), "lines": [[(x, y, z), ...], ...]} drawn by the user (solid
    coordinates); they cut the planar faces that face that normal before the automatic layout."""
    m = Mesher(solid, tolerance, max_edge, max_angle_deg, corner_deg, cdt, guides, skeleton, across)
    verts, faces, face_of = m.run()
    sharp = m.sharp_edges()
    keep = [k for k, q in enumerate(faces) if len(set(q)) == 4]
    faces = [faces[k] for k in keep]
    info = m.info
    info["face_of"] = [face_of[k] for k in keep]
    info["patch_of"] = [m.patch_of[k] for k in keep]
    info["degenerate"] = sum(1 for q in faces if _base._bad_quad([verts[k] for k in q]))
    info["verts"] = len(verts)
    info["quads"] = len(faces)
    return verts, faces, sharp, info
