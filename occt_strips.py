"""Fork: strip quad mesh of a flat profile, the way instrument cages are modelled by hand.

The profile (outer loop + holes, 2D) is triangulated from its outline only (constrained Delaunay).
Its chordal axis gives the skeleton: triangles with 3 inner edges are junctions, 2 = sleeves (their
inner edges are chords across the shape), 1 = ends. Each skeleton branch becomes a strip of quads:
  * rungs = a few chords of the branch, chosen so the outline between two rungs stays within the
    tolerance, rungs never further apart than the maximum length, and a rung on every corner;
  * every rung is split into `across` segments (2 by default), so the long loops run all along;
  * junctions are filled with 3 patches around one pole, ends with one row closing the strip.
Returns 2D vertices and quads; the caller extrudes them and fits them to the exact part.
"""
import math

import numpy as np


def _cr(a, b):
    return a[0] * b[1] - a[1] * b[0]


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def _pip(p, loops):
    x, y = p
    inside = False
    for loop in loops:
        n = len(loop)
        for i in range(n):
            x1, y1 = loop[i]
            x2, y2 = loop[(i + 1) % n]
            if (y1 > y) != (y2 > y):
                if x1 + (y - y1) * (x2 - x1) / (y2 - y1) > x:
                    inside = not inside
    return inside


def _dist_seg(p, a, b):
    ab = b - a
    L2 = ab @ ab
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, (p - a) @ ab / L2))
    return np.linalg.norm(a + ab * t - p)


class StripMesher:
    def __init__(self, loops, cdt, tol, max_len, across=2, corner_deg=25.0, prune=2.0, corners=None):
        """loops: list of closed polylines (outer first, then holes); each polyline is a list of
        (x, y). corners: optional set of (loop index, point index) that must be cage vertices."""
        self.tol, self.max_len, self.n = tol, max_len, max(1, int(across))
        self.cdt = cdt
        pts, segs, loop_of, self.loops = [], [], [], []
        self.is_corner = []
        for li, lp in enumerate(loops):
            s = len(pts)
            m = len(lp)
            for k in range(m):
                pts.append(tuple(lp[k]))
                loop_of.append(li)
                a, b, c = np.array(lp[k - 1]), np.array(lp[k]), np.array(lp[(k + 1) % m])
                t1 = math.atan2(*(b - a)[::-1])
                t2 = math.atan2(*(c - b)[::-1])
                corner = abs(_wrap(t2 - t1)) > math.radians(corner_deg)
                if corners is not None and (li, k) in corners:
                    corner = True
                self.is_corner.append(corner)
            segs += [(s + k, s + (k + 1) % m) for k in range(m)]
            self.loops.append(list(range(s, s + m)))
        self.P = np.array(pts, float)
        self.loop_of = loop_of
        self.next = {}
        self.prev = {}
        for idx in self.loops:
            m = len(idx)
            for k in range(m):
                self.next[idx[k]] = idx[(k + 1) % m]
                self.prev[idx[k]] = idx[k - 1]
        tris = cdt(pts, segs)
        loops_xy = [[pts[i] for i in idx] for idx in self.loops]
        self.T = [tuple(t) for t in tris if _pip(tuple(self.P[list(t)].mean(0)), loops_xy)]
        self.bnd = {(min(a, b), max(a, b)) for a, b in segs}
        self.verts, self.quads, self.keys = [], [], {}
        self.info = {"junctions": 0, "strips": 0, "ends": 0, "warnings": []}

    # ---- vertices ------------------------------------------------------------------------------
    def vid(self, key, xy):
        j = self.keys.get(key)
        if j is None:
            j = self.keys[key] = len(self.verts)
            self.verts.append((float(xy[0]), float(xy[1])))
        return j

    def bvert(self, i):
        return self.vid(("b", i), self.P[i])

    def rung(self, e):
        """Points of a chord e = (a, b) (sorted pair) split into n segments, from e[0] to e[1]."""
        a, b = e
        out = [self.bvert(a)]
        for k in range(1, self.n):
            out.append(self.vid(("r", e, k), self.P[a] + (self.P[b] - self.P[a]) * k / self.n))
        out.append(self.bvert(b))
        return out

    def oriented_rung(self, e, start):
        r = self.rung(e)
        return r if e[0] == start else r[::-1]

    def quad(self, q):
        if len(set(q)) == 4:
            self.quads.append(list(q))

    # ---- skeleton ------------------------------------------------------------------------------
    def build(self):
        T = self.T
        P = self.P
        tedges = [[(min(t[k], t[(k + 1) % 3]), max(t[k], t[(k + 1) % 3])) for k in range(3)] for t in T]
        etri = {}
        for ti, te in enumerate(tedges):
            for e in te:
                etri.setdefault(e, []).append(ti)
        inner = {e: tl for e, tl in etri.items() if len(tl) == 2 and e not in self.bnd}
        kind = [sum(1 for e in te if e in inner) for te in tedges]
        self.tedges, self.inner, self.kind = tedges, inner, kind

        def centre(ti):
            return P[list(T[ti])].mean(0)

        def radius(ti):
            a, b, c = (P[k] for k in T[ti])
            la, lb, lc = np.linalg.norm(b - c), np.linalg.norm(a - c), np.linalg.norm(a - b)
            ar = abs(_cr(b - a, c - a)) / 2
            return la * lb * lc / (4 * ar) if ar > 0 else 0.0

        nodes = [ti for ti in range(len(T)) if kind[ti] != 2]
        branches, seen = [], set()
        for t0 in nodes:
            for e0 in tedges[t0]:
                if e0 not in inner or (t0, e0) in seen:
                    continue
                prev, e, chords, tris_on = t0, e0, [e0], []
                while True:
                    nxt = inner[e][0] if inner[e][1] == prev else inner[e][1]
                    if kind[nxt] != 2:
                        break
                    tris_on.append(nxt)
                    e = next(x for x in tedges[nxt] if x in inner and x != e)
                    chords.append(e)
                    prev = nxt
                seen.add((t0, e0))
                seen.add((nxt, e))
                length = sum(np.linalg.norm(P[list(chords[k + 1])].mean(0) - P[list(chords[k])].mean(0))
                             for k in range(len(chords) - 1))
                length += np.linalg.norm(P[list(chords[0])].mean(0) - centre(t0))
                length += np.linalg.norm(P[list(chords[-1])].mean(0) - centre(nxt))
                branches.append({"a": t0, "b": nxt, "chords": chords, "tris": tris_on, "len": length,
                                 "alive": True})
        # prune short dead ends (corners, bumps)
        changed = True
        while changed:
            changed = False
            deg = {}
            for b in branches:
                if b["alive"]:
                    deg[b["a"]] = deg.get(b["a"], 0) + 1
                    deg[b["b"]] = deg.get(b["b"], 0) + 1
            for b in sorted((b for b in branches if b["alive"]), key=lambda b: b["len"]):
                for j, end in ((b["a"], b["b"]), (b["b"], b["a"])):
                    if deg.get(end, 0) == 1 and kind[j] == 3 and deg.get(j, 0) >= 3 and \
                            b["len"] < self.prune_factor * radius(j):
                        b["alive"] = False
                        changed = True
                        break
                if changed:
                    break
        deg = {}
        for b in branches:
            if b["alive"]:
                deg[b["a"]] = deg.get(b["a"], 0) + 1
                deg[b["b"]] = deg.get(b["b"], 0) + 1
        # a junction left with two branches is just a bend: join the branches through it
        for ti in nodes:
            if kind[ti] != 3 or deg.get(ti, 0) != 2:
                continue
            bs = [b for b in branches if b["alive"] and ti in (b["a"], b["b"])]
            if len(bs) != 2 or bs[0] is bs[1]:
                continue
            b1, b2 = bs
            c1 = b1["chords"] if b1["b"] == ti else b1["chords"][::-1]
            s1 = b1["a"] if b1["b"] == ti else b1["b"]
            c2 = b2["chords"] if b2["a"] == ti else b2["chords"][::-1]
            e2 = b2["b"] if b2["a"] == ti else b2["a"]
            b1.update(a=s1, b=e2, chords=c1 + c2, len=b1["len"] + b2["len"])
            b2["alive"] = False
            deg[ti] = 0
        self.branches, self.deg = branches, deg

        live = [b for b in branches if b["alive"]]
        if not live:
            self.info["warnings"].append("no skeleton branch")
            return
        # junction triangles (3 live branches) -> 3 patches around a pole
        for ti in nodes:
            if kind[ti] == 3 and deg.get(ti, 0) == 3:
                self.junction(ti)
            elif kind[ti] == 3 and deg.get(ti, 0) >= 4:
                self.info["warnings"].append("4-way junction")
        # strips
        for b in live:
            self.strip(b)
        # ends: node with one live branch (pruned junction or end triangle)
        for b in live:
            for node, e in ((b["a"], b["chords"][0]), (b["b"], b["chords"][-1])):
                if deg.get(node, 0) == 1:
                    self.end(node, e, b)

    # ---- strips ---------------------------------------------------------------------------------
    def strip(self, b):
        P = self.P
        ch = b["chords"]
        if len(ch) < 2:
            return
        self.info["strips"] += 1
        # orient every chord the same way: start = the endpoint shared with the previous chord's
        # side. Consecutive chords share exactly one endpoint (sleeve triangle).
        sides = []  # (left index, right index) per chord
        a0, b0 = ch[0]
        d = P[list(ch[1])].mean(0) - P[list(ch[0])].mean(0)
        if _cr(d, P[a0] - P[list(ch[0])].mean(0)) > 0:
            left, right = a0, b0
        else:
            left, right = b0, a0
        sides.append((left, right))
        for e in ch[1:]:
            l, r = sides[-1]
            if l in e:
                nl, nr = l, (e[0] if e[1] == l else e[1])
            elif r in e:
                nl, nr = (e[0] if e[1] == r else e[1]), r
            else:
                nl, nr = e
            sides.append((nl, nr))
        # choose the rungs
        # corner rungs: where several chords fan out of one corner, take the middle one
        forced = set()
        k = 1
        while k < len(ch) - 1:
            for s in (0, 1):
                v = sides[k][s]
                if self.is_corner[v]:
                    j = k
                    while j + 1 < len(ch) - 1 and sides[j + 1][s] == v:
                        j += 1
                    i0 = k
                    while i0 - 1 > 0 and sides[i0 - 1][s] == v:
                        i0 -= 1
                    forced.add((i0 + j) // 2)
            k += 1
        keep = [0]
        last = 0
        for k in range(1, len(ch)):
            if k == len(ch) - 1:
                keep.append(k)
                break
            if not self._span_ok(sides, last, k + 1):
                if keep[-1] != k:
                    keep.append(k)
                last = k
            elif k in forced:
                keep.append(k)
                last = k
        keep = sorted(set(keep))
        # two consecutive rungs must not share an endpoint (that quad would collapse)
        changed = True
        while changed and len(keep) > 2:
            changed = False
            for j in range(len(keep) - 1):
                (l0, r0), (l1, r1) = sides[keep[j]], sides[keep[j + 1]]
                if l0 == l1 or r0 == r1:
                    cand = [x for x in (keep[j + 1], keep[j]) if x not in (keep[0], keep[-1])]
                    # keep the rung that sits on a corner
                    cand.sort(key=lambda x: (x in forced, self.is_corner[sides[x][0]] or self.is_corner[sides[x][1]]))
                    if not cand:
                        break
                    drop = cand[0]
                    keep.remove(drop)
                    changed = True
                    break
        for k0, k1 in zip(keep, keep[1:]):
            l0, r0 = sides[k0]
            l1, r1 = sides[k1]
            R0 = self.oriented_rung(ch[k0], l0)
            R1 = self.oriented_rung(ch[k1], l1)
            for i in range(self.n):
                self.quad([R0[i], R1[i], R1[i + 1], R0[i + 1]])

    def _span_ok(self, sides, k0, k1):
        """Can rungs k0 and k1 be consecutive? (outline within tolerance, length within max)."""
        P = self.P
        for s in (0, 1):
            a, b = sides[k0][s], sides[k1][s]
            if a == b:
                continue
            pa, pb = P[a], P[b]
            if self.max_len > 0 and np.linalg.norm(pb - pa) > self.max_len:
                return False
            # boundary samples between a and b along the loop: the shorter way round
            paths = []
            for step in (self.next, self.prev):
                path, i = [], a
                for _ in range(len(P)):
                    i = step[i]
                    if i == b:
                        paths.append(path)
                        break
                    path.append(i)
            if not paths:
                return False
            path = min(paths, key=len)
            for i in path:
                if self.is_corner[i]:
                    return False
                if _dist_seg(P[i], pa, pb) > self.tol:
                    return False
        # rungs not further apart than `aspect` times the strip width
        e0 = sides[k0]
        e1 = sides[k1]
        w = max(np.linalg.norm(P[e0[0]] - P[e0[1]]), np.linalg.norm(P[e1[0]] - P[e1[1]]))
        m0 = (P[e0[0]] + P[e0[1]]) / 2
        m1 = (P[e1[0]] + P[e1[1]]) / 2
        if self.aspect > 0 and np.linalg.norm(m1 - m0) > self.aspect * w:
            return False
        return True

    # ---- junctions and ends -----------------------------------------------------------------------
    def junction(self, ti):
        self.info["junctions"] += 1
        t = self.T[ti]
        P = self.P
        n = self.n
        e = [(min(t[k], t[(k + 1) % 3]), max(t[k], t[(k + 1) % 3])) for k in range(3)]
        c = P[list(t)].mean(0)
        C = self.vid(("j", ti), c)
        if n % 2:
            self.info["warnings"].append("junction needs an even number across")
            return
        h = n // 2
        # rung of each edge from t[k] to t[k+1]
        R = [self.oriented_rung(e[k], t[k]) for k in range(3)]
        mids = [R[k][h] for k in range(3)]
        # grid between corner t[k], mid of edge k, centre, mid of edge k-1  (h x h)
        for k in range(3):
            A = R[k][: h + 1]                     # t[k] -> mid k
            B = R[k - 1][h:][::-1]                # t[k] -> mid k-1  (reversed second half of edge k-1)
            ma, mb = P[t[k]], c
            va, vb = np.array(self.verts[mids[k]]), np.array(self.verts[mids[k - 1]])
            spokeA = [mids[k]] + [self.vid(("s", ti, k, j), va + (c - va) * j / h) for j in range(1, h)] + [C]
            spokeB = [mids[k - 1]] + [self.vid(("s", ti, (k - 1) % 3, j), vb + (c - vb) * j / h)
                                      for j in range(1, h)] + [C]
            # bilinear grid: rows along B (from corner), columns along A
            grid = [[None] * (h + 1) for _ in range(h + 1)]
            for i in range(h + 1):
                for j in range(h + 1):
                    if j == 0:
                        grid[i][j] = A[i]
                    elif i == 0:
                        grid[i][j] = B[j]
                    elif i == h:
                        grid[i][j] = spokeA[j]
                    elif j == h:
                        grid[i][j] = spokeB[i]
                    else:
                        pa = np.array(self.verts[A[i]])
                        pc = np.array(self.verts[spokeB[i]])
                        grid[i][j] = self.vid(("g", ti, k, i, j), pa + (pc - pa) * j / h)
            for i in range(h):
                for j in range(h):
                    self.quad([grid[i][j], grid[i + 1][j], grid[i + 1][j + 1], grid[i][j + 1]])

    def end(self, node, e, b):
        """Close a strip: the region beyond chord e (junction pruned or end triangle)."""
        self.info["ends"] += 1
        P = self.P
        n = self.n
        a, c = e
        # boundary chain from c to a on the far side (the side that does not contain the strip)
        chain = self._far_chain(e, b)
        if chain is None or len(chain) < 1:
            return
        # pick end points: two strongest corners of the chain, else 1/4 and 3/4 of its length
        if len(chain) < 2:
            R = self.oriented_rung(e, c)
            tip = self.bvert(chain[0]) if chain else None
            if tip is not None and n == 2:
                self.quad([R[0], R[1], R[2], tip])
            return
        idx = [c] + chain + [a]
        L = np.concatenate([[0], np.cumsum([np.linalg.norm(P[idx[k + 1]] - P[idx[k]]) for k in range(len(idx) - 1)])])
        corners = [k for k in range(1, len(idx) - 1) if self.is_corner[idx[k]]]
        if len(corners) >= 2:
            corners = sorted(sorted(corners, key=lambda k: -self._turn(idx[k]))[:2])
            k1, k2 = corners
        else:
            k1 = int(np.searchsorted(L, 0.25 * L[-1]))
            k2 = int(np.searchsorted(L, 0.75 * L[-1]))
            k1 = min(max(k1, 1), len(idx) - 3)
            k2 = min(max(k2, k1 + 1), len(idx) - 2)
        p1, p2 = idx[k1], idx[k2]
        # end side p1 -> p2 split into n
        endside = [self.bvert(p1)] + [self.vid(("es", node, j), P[p1] + (P[p2] - P[p1]) * j / n)
                                      for j in range(1, n)] + [self.bvert(p2)]
        R = self.oriented_rung(e, c)  # c -> a
        # one row of n quads: rung (c..a) against end side reversed (p2..p1)?  c is next to p1
        top = endside  # p1 .. p2 ; p1 is on c's side
        for i in range(n):
            self.quad([R[i], R[i + 1], top[n - i - 1] if False else top[i + 1], top[i]][::1])

    def _turn(self, i):
        P = self.P
        a, b, c = P[self.prev[i]], P[i], P[self.next[i]]
        return abs(_wrap(math.atan2(*(c - b)[::-1]) - math.atan2(*(b - a)[::-1])))

    def _far_chain(self, e, b):
        """Boundary samples strictly between e's endpoints, the short way round (the end beyond
        the last rung)."""
        a, c = e
        P = self.P
        paths = []
        for step in (self.next, self.prev):
            path, i = [], c
            for _ in range(len(P)):
                i = step[i]
                if i == a:
                    paths.append(path)
                    break
                path.append(i)
        if not paths:
            return None
        return min(paths, key=len)


def strip_mesh(loops, cdt, tol, max_len, across=2, prune=2.0, aspect=2.0):
    m = StripMesher(loops, cdt, tol, max_len, across)
    m.prune_factor = prune
    m.aspect = aspect
    m.build()
    V = m.verts
    quads = []
    for q in m.quads:
        ar = sum(V[q[k - 1]][0] * V[q[k]][1] - V[q[k]][0] * V[q[k - 1]][1] for k in range(4))
        quads.append(q if ar > 0 else q[::-1])
    m.quads = quads
    return m.verts, m.quads, m.info, m


def strip_cage(solid, cdt, tol=0.0001, across=2, aspect=2.0, ds=None):
    """3D cage of an extruded part: strip mesh of its largest flat face, extruded through the
    part's thickness. Returns (verts, faces, sharp_edges, info)."""
    try:
        from . import occt_quad2
    except ImportError:
        import occt_quad2
    from OCP.GeomAbs import GeomAbs_Plane
    from OCP.GProp import GProp_GProps
    from OCP.BRepGProp import BRepGProp
    m = occt_quad2.Mesher(solid, tol, 0.0, 45.0, 40.0, cdt)
    m.prepare_edges()
    m.reset()
    best = None
    n_planar = {}
    for fi in range(1, m.fmap.Extent() + 1):
        F = m.face_ctx(fi)
        if not F["loops"] or F["surf"].GetType() != GeomAbs_Plane:
            continue
        gp = GProp_GProps()
        fn = getattr(BRepGProp, "SurfaceProperties_s", None) or BRepGProp.SurfaceProperties
        fn(F["face"], gp)
        if best is None or gp.Mass() > best[0]:
            best = (gp.Mass(), F)
    if best is None:
        raise ValueError("no flat face to start from")
    F = best[1]
    pln = F["surf"].Plane()
    O = np.array([pln.Location().X(), pln.Location().Y(), pln.Location().Z()])
    N = np.array([pln.Axis().Direction().X(), pln.Axis().Direction().Y(), pln.Axis().Direction().Z()])
    X = np.array([pln.XAxis().Direction().X(), pln.XAxis().Direction().Y(), pln.XAxis().Direction().Z()])
    Y = np.array([pln.YAxis().Direction().X(), pln.YAxis().Direction().Y(), pln.YAxis().Direction().Z()])
    # outline: each edge resampled, B-rep vertices kept
    per = 0.0
    pieces_loops = []
    for uses in F["loops"]:
        lp = []
        for ei, rev in uses:
            c = m.edge_curve(ei)
            ed = m.edges[ei]
            pa, pb = (ed["t1"], ed["t0"]) if rev else (ed["t0"], ed["t1"])
            pc = {"c": c, "pa": pa, "pb": pb, "na": None, "nb": None, "use": (ei, rev)}
            lp.append(pc)
            per += m.piece_len(F, pc)
        pieces_loops.append(lp)
    ds = ds or per / 1500.0
    loops = []
    for lp in pieces_loops:
        pts = []
        for pc in lp:
            n = max(1, int(math.ceil(m.piece_len(F, pc) / ds)))
            pts += [m.piece_uv(F, pc, pc["pa"] + (pc["pb"] - pc["pa"]) * k / n) for k in range(n)]
        loops.append(pts)
    V2, Q2, info, sm = strip_mesh(loops, cdt, tol, 0.0, across, aspect=aspect)
    # thickness along the face normal (from the B-rep vertices)
    d = [np.dot(np.array(m.nodes[m.vertex_node(v)]["xyz"]) - O, N) for v in range(1, m.vmap.Extent() + 1)]
    z0, z1 = min(d), max(d)
    info["thickness_mm"] = round((z1 - z0) * 1000, 4)
    # flat levels other than the two caps: steps / pockets (not handled yet)
    levels = set()
    for fi in range(1, m.fmap.Extent() + 1):
        G = m.face_ctx(fi)
        if G["loops"] and G["surf"].GetType() == GeomAbs_Plane:
            g = G["surf"].Plane()
            n2 = np.array([g.Axis().Direction().X(), g.Axis().Direction().Y(), g.Axis().Direction().Z()])
            if abs(n2 @ N) > 0.98:
                o2 = np.array([g.Location().X(), g.Location().Y(), g.Location().Z()])
                levels.add(round(float((o2 - O) @ N) * 1e5))
    if len(levels) > 2:
        info["warnings"].append("the part has steps or pockets: they are not followed yet (the cage is fitted over them)")
    nv = len(V2)
    verts = []
    for z in (z0, z1):
        for (u, v) in V2:
            p = O + u * X + v * Y + z * N
            verts.append(tuple(p))
    faces = [list(q) for q in Q2] + [[k + nv for k in q[::-1]] for q in Q2]
    # walls on the outline
    ec = {}
    for q in Q2:
        for k in range(4):
            a, b = q[k], q[(k + 1) % 4]
            ec[(a, b)] = ec.get((a, b), 0) + 1
    bnd = [(a, b) for (a, b) in ec if (b, a) not in ec]
    for a, b in bnd:
        faces.append([b, a, a + nv, b + nv])
    # sharp edges: both outline loops, and the vertical edges on outline corners
    corner2d = set()
    for key, j in sm.keys.items():
        if key[0] == "b" and sm.is_corner[key[1]]:
            corner2d.add(j)
    sharp = set()
    for a, b in bnd:
        sharp.add((min(a, b), max(a, b)))
        sharp.add((min(a, b) + nv, max(a, b) + nv))
    bverts = {a for a, b in bnd}
    for a in bverts:
        if a in corner2d:
            sharp.add((a, a + nv))
    info["quads_per_cap"] = len(Q2)
    return verts, faces, sharp, info
