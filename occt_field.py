"""Fork: cross field + separatrices on a planar region (quad layout like field-aligned remeshers).

1. The region (outer loop, holes, inner feature lines) is triangulated with inner points.
2. A smooth 4-direction field is solved: z = exp(4 i theta), fixed to the boundary / feature
   tangents, harmonic inside (Laplace, conjugate gradient, numpy only).
3. Singular triangles (index +-1/4) are where the field turns: they become the poles.
4. Separatrices are traced from every singularity (3 or 5 directions) and from every reflex
   corner (along the continuation of its edges), each stopping on the boundary or on an earlier
   trace (motorcycle graph). The traces cut the region into 4-sided patches.
All coordinates are 2D, in a metric plane.
"""
import math

import numpy as np


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def _resample(poly, h, closed=True):
    pts = [poly[0]]
    n = len(poly)
    acc = 0.0
    rng = range(1, n + 1) if closed else range(1, n)
    for k in rng:
        a = np.array(pts[-1] if acc == 0 else poly[k - 1], float)
        a = np.array(poly[k - 1], float)
        b = np.array(poly[k % n], float)
        L = np.linalg.norm(b - a)
        t = 0.0
        while acc + (L - t) >= h:
            t += h - acc
            acc = 0.0
            pts.append(tuple(a + (b - a) * (t / L)))
        acc += L - t
    if closed and len(pts) > 2 and np.linalg.norm(np.subtract(pts[-1], pts[0])) < 0.5 * h:
        pts.pop()
    if not closed and np.linalg.norm(np.subtract(pts[-1], poly[-1])) > 1e-12:
        if np.linalg.norm(np.subtract(pts[-1], poly[-1])) < 0.5 * h and len(pts) > 1:
            pts[-1] = tuple(poly[-1])
        else:
            pts.append(tuple(poly[-1]))
    return pts


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


class CrossField:
    def __init__(self, loops, features, h, cdt, corner_deg=30.0):
        """loops: closed polylines (outer first); features: open polylines inside (steps)."""
        self.h = h
        self.loops_in = loops
        pts, cons, fixed_dir = [], [], {}
        self.loop_idx = []
        for lp in loops:
            res = _resample(lp, h, True)
            s = len(pts)
            pts += res
            n = len(res)
            cons += [(s + k, s + (k + 1) % n) for k in range(n)]
            self.loop_idx.append(list(range(s, s + n)))
        self.feat_idx = []
        for fl in features:
            res = _resample(fl, h, False)
            s = len(pts)
            pts += res
            cons += [(s + k, s + k + 1) for k in range(len(res) - 1)]
            self.feat_idx.append(list(range(s, s + len(res))))
        nb = len(pts)
        # inner points: jittered grid, away from constrained points
        P = np.array(pts)
        lo, hi = P.min(0), P.max(0)
        loops_res = [[pts[i] for i in idx] for idx in self.loop_idx]
        rng = np.random.default_rng(1)
        inner = []
        from collections import defaultdict
        grid = defaultdict(list)
        cell = h
        for i, p in enumerate(pts):
            grid[(int(p[0] // cell), int(p[1] // cell))].append(p)
        y = lo[1] + h * 0.5
        row = 0
        while y < hi[1]:
            x = lo[0] + h * (0.5 + 0.5 * (row % 2))
            while x < hi[0]:
                q = (x + rng.uniform(-0.1, 0.1) * h, y + rng.uniform(-0.1, 0.1) * h)
                ok = _pip(q, loops_res)
                if ok:
                    cx, cy = int(q[0] // cell), int(q[1] // cell)
                    for dx in (-1, 0, 1):
                        for dy in (-1, 0, 1):
                            for r in grid.get((cx + dx, cy + dy), ()):
                                if (r[0] - q[0]) ** 2 + (r[1] - q[1]) ** 2 < (0.6 * h) ** 2:
                                    ok = False
                if ok:
                    inner.append(q)
                x += h
            y += h * math.sqrt(3) / 2
            row += 1
        pts += inner
        self.P = np.array(pts)
        tris = cdt(pts, cons)
        tris = [t for t in tris if _pip(tuple(self.P[list(t)].mean(0)), loops_res)]
        self.T = np.array(tris, int)
        self.nb = nb
        self._fix_directions(cons, corner_deg)
        self._solve()
        self._singularities()
        self._locator()

    # ---- boundary / feature directions ---------------------------------------------------------
    def _fix_directions(self, cons, corner_deg):
        P = self.P
        zsum = {}
        self.corners = []  # (index, turning angle, incoming dir, outgoing dir)
        for idx in self.loop_idx:
            n = len(idx)
            for k in range(n):
                a, b, c = P[idx[k - 1]], P[idx[k]], P[idx[(k + 1) % n]]
                d1, d2 = b - a, c - b
                t1, t2 = math.atan2(d1[1], d1[0]), math.atan2(d2[1], d2[0])
                z = np.exp(4j * t1) + np.exp(4j * t2)
                zsum[idx[k]] = z / abs(z) if abs(z) > 1e-9 else np.exp(4j * t2)
                turn = _wrap(t2 - t1)
                if abs(turn) > math.radians(corner_deg):
                    self.corners.append((idx[k], turn, d1 / np.linalg.norm(d1), d2 / np.linalg.norm(d2)))
        for idx in self.feat_idx:
            for k in range(len(idx)):
                a = P[idx[max(k - 1, 0)]]
                c = P[idx[min(k + 1, len(idx) - 1)]]
                d = c - a
                zsum[idx[k]] = np.exp(4j * math.atan2(d[1], d[0]))
        self.fixed = zsum
        # orientation of the outer loop (for reflex corners)
        o = P[self.loop_idx[0]]
        area = 0.5 * np.sum(o[:, 0] * np.roll(o[:, 1], -1) - np.roll(o[:, 0], -1) * o[:, 1])
        self.sigma = 1.0 if area > 0 else -1.0

    def _solve(self):
        n = len(self.P)
        T = self.T
        E = np.vstack([T[:, [0, 1]], T[:, [1, 2]], T[:, [2, 0]]])
        E = np.unique(np.sort(E, axis=1), axis=0)
        self.E = E
        fixed = np.zeros(n, bool)
        z0 = np.zeros(n, complex)
        for i, z in self.fixed.items():
            fixed[i] = True
            z0[i] = z
        free = ~fixed
        deg = np.zeros(n)
        np.add.at(deg, E[:, 0], 1)
        np.add.at(deg, E[:, 1], 1)

        def A(x):  # Laplacian restricted to free vertices (x has zeros on fixed)
            y = deg * x
            np.add.at(y, E[:, 0], -x[E[:, 1]])
            np.add.at(y, E[:, 1], -x[E[:, 0]])
            y[fixed] = 0
            return y

        b = -A(z0) * 1.0
        b[fixed] = 0
        # b = - L z0 restricted; recompute properly: rhs = sum of fixed neighbours
        b = np.zeros(n, complex)
        np.add.at(b, E[:, 0], np.where(fixed[E[:, 1]], z0[E[:, 1]], 0))
        np.add.at(b, E[:, 1], np.where(fixed[E[:, 0]], z0[E[:, 0]], 0))
        b[fixed] = 0
        x = np.zeros(n, complex)
        r = b - A(x)
        p = r.copy()
        rs = np.vdot(r, r).real
        for it in range(4 * n + 100):
            Ap = A(p)
            alpha = rs / max(np.vdot(p, Ap).real, 1e-300)
            x += alpha * p
            r -= alpha * Ap
            rs_new = np.vdot(r, r).real
            if rs_new < 1e-20 * max(1.0, np.vdot(b, b).real):
                break
            p = r + (rs_new / rs) * p
            rs = rs_new
        z = np.where(fixed, z0, x)
        mag = np.abs(z)
        z = np.where(mag > 1e-12, z / np.maximum(mag, 1e-12), 1.0)
        self.z = z
        self.phi = np.angle(z)  # = 4 theta

    def _singularities(self):
        phi = self.phi
        T = self.T
        d = (np.vectorize(_wrap)(phi[T[:, 1]] - phi[T[:, 0]]) + np.vectorize(_wrap)(phi[T[:, 2]] - phi[T[:, 1]])
             + np.vectorize(_wrap)(phi[T[:, 0]] - phi[T[:, 2]]))
        idx = np.rint(d / (2 * math.pi)).astype(int)
        self.sing = [(int(t), int(idx[t])) for t in np.nonzero(idx)[0]]

    # ---- field queries ---------------------------------------------------------------------------
    def _locator(self):
        P, T = self.P, self.T
        self.cell = 2 * self.h
        from collections import defaultdict
        g = defaultdict(list)
        for ti, t in enumerate(T):
            q = P[t]
            lo = np.floor(q.min(0) / self.cell).astype(int)
            hi = np.floor(q.max(0) / self.cell).astype(int)
            for ix in range(lo[0], hi[0] + 1):
                for iy in range(lo[1], hi[1] + 1):
                    g[(ix, iy)].append(ti)
        self.grid = g

    def locate(self, p):
        key = (int(math.floor(p[0] / self.cell)), int(math.floor(p[1] / self.cell)))
        for ti in self.grid.get(key, ()):
            a, b, c = self.P[self.T[ti]]
            v0, v1, v2 = c - a, b - a, np.asarray(p) - a
            d00, d01, d02 = v0 @ v0, v0 @ v1, v0 @ v2
            d11, d12 = v1 @ v1, v1 @ v2
            den = d00 * d11 - d01 * d01
            if den == 0:
                continue
            u = (d11 * d02 - d01 * d12) / den
            v = (d00 * d12 - d01 * d02) / den
            if u >= -1e-9 and v >= -1e-9 and u + v <= 1 + 1e-9:
                return ti, (1 - u - v, v, u)
        return None, None

    def theta(self, p):
        ti, w = self.locate(p)
        if ti is None:
            return None
        zz = sum(w[k] * self.z[self.T[ti][k]] for k in range(3))
        return np.angle(zz) / 4.0

    def direction(self, p, prev):
        th = self.theta(p)
        if th is None:
            return None
        best = None
        for k in range(4):
            d = np.array([math.cos(th + k * math.pi / 2), math.sin(th + k * math.pi / 2)])
            s = d @ prev
            if best is None or s > best[0]:
                best = (s, d)
        return best[1]

    # ---- separatrices ----------------------------------------------------------------------------
    def separatrices(self, max_len=None):
        h = self.h
        step = 0.25 * h
        traces = []
        self.dbg = []
        seeds = []
        corner_pts = [(vi, self.P[vi], turn, d_in, d_out) for vi, turn, d_in, d_out in self.corners]
        used_corners = set()
        loops_res = [[tuple(self.P[i]) for i in idx] for idx in self.loop_idx]
        for ti, ind in self.sing:
            c = self.P[self.T[ti]].mean(0)
            # a singularity next to a boundary corner belongs to the corner: start there
            near = min(corner_pts, key=lambda e: np.linalg.norm(e[1] - c), default=None)
            if near is not None and np.linalg.norm(near[1] - c) < 2.5 * h:
                vi, cp, turn, d_in, d_out = near
                used_corners.add(vi)
                bis = (-d_in + d_out)
                inward = np.array([-bis[1], bis[0]]) if np.linalg.norm(bis) < 1e-9 else bis
                th = self.theta(cp + 0.8 * h * self._inward(cp, d_in, d_out, loops_res))
                if th is None:
                    continue
                for k in range(4):
                    d = np.array([math.cos(th + k * math.pi / 2), math.sin(th + k * math.pi / 2)])
                    if not _pip(tuple(cp + d * 0.5 * h), loops_res):
                        continue
                    if min(abs(_wrap(math.atan2(d[1], d[0]) - math.atan2(e[1], e[0]))) for e in (d_in, -d_out, -d_in, d_out)) < math.radians(25):
                        continue
                    seeds.append((cp.copy(), d, ("corner", vi)))
                continue
            r = 1.5 * h
            al = np.linspace(0, 2 * math.pi, 180, endpoint=False)
            g = []
            for a in al:
                q = c + r * np.array([math.cos(a), math.sin(a)])
                th = self.theta(q)
                g.append(None if th is None else ((th - a + math.pi / 4) % (math.pi / 2)) - math.pi / 4)
            dirs = []
            for k in range(len(al)):
                g0, g1 = g[k], g[(k + 1) % len(al)]
                if g0 is None or g1 is None:
                    continue
                if g0 == 0 or (g0 < 0 < g1) or (g1 < 0 < g0):
                    if abs(g0 - g1) < math.pi / 4:
                        a = al[k] + (al[1] - al[0]) * (abs(g0) / (abs(g0) + abs(g1) + 1e-30))
                        dirs.append(a)
            for a in dirs:
                seeds.append((c.copy(), np.array([math.cos(a), math.sin(a)]), ("sing", ti)))
        for vi, turn, d_in, d_out in self.corners:
            if vi in used_corners:
                continue
            if turn * self.sigma < 0:  # reflex corner: continue both edges into the region
                seeds.append((self.P[vi].copy(), d_in.copy(), ("corner", vi)))
                seeds.append((self.P[vi].copy(), -d_out.copy(), ("corner", vi)))
        max_len = max_len or 10 * float(np.ptp(self.P, axis=0).max())
        loops = [[tuple(self.P[i]) for i in idx] for idx in self.loop_idx]
        # all traces grow together, one step each turn; a trace stops on the boundary or when it
        # meets another trace (motorcycle graph)
        act = [{"pts": [p0.copy()], "p": p0.copy(), "d": d0.copy(), "L": 0.0, "src": src, "p0": p0.copy(),
                "stop": None} for p0, d0, src in seeds]
        while any(t["stop"] is None for t in act):
            for t in act:
                if t["stop"] is not None:
                    continue
                p, d = t["p"], t["d"]
                if t["L"] > max_len:
                    t["stop"] = "long"
                    continue
                dn = self.direction(p + d * step * 0.5, d)
                if dn is None:  # leaving the region: end on the boundary just ahead
                    hb = self._hit_boundary(p, p + d * step * 2.0, loops)
                    if hb is not None:
                        t["pts"].append(hb)
                        t["stop"] = "boundary"
                    else:
                        t["stop"] = "out"
                    continue
                q = p + dn * step
                others = []
                for o in act:
                    if o is t:
                        continue
                    arr = np.array(o["pts"])
                    if len(arr) < 2:
                        continue
                    if t["L"] < 2 * h and np.linalg.norm(o["p0"] - t["p0"]) < 0.5 * h:
                        continue  # siblings from the same start
                    others.append(arr)
                hit = self._hit_traces(p, q, others)
                if hit is not None:
                    t["pts"].append(hit)
                    t["stop"] = "trace"
                    continue
                if t["L"] > 3 * h and len(t["pts"]) > 8:
                    if self._hit_traces(p, q, [np.array(t["pts"][:-6])]) is not None or \
                            np.linalg.norm(q - t["p0"]) < 0.5 * h:
                        t["stop"] = "closed"
                        continue
                if self.locate(q)[0] is None:
                    hb = self._hit_boundary(p, p + (q - p) * 2.0, loops)
                    t["pts"].append(q if hb is None else hb)
                    t["stop"] = "boundary"
                    continue
                t["pts"].append(q)
                t["p"], t["d"] = q, dn
                t["L"] += step
        for t in act:
            self.dbg.append((t["src"], tuple(np.round(t["p0"] * 1000, 2)), t["stop"], round(t["L"] * 1000, 1)))
            if len(t["pts"]) >= 2 and t["stop"] in ("trace", "boundary"):
                traces.append(np.array(t["pts"]))
        self.traces = traces
        return traces

    def _inward(self, cp, d_in, d_out, loops):
        for d in ((-d_in + d_out), (d_in - d_out), np.array([-d_out[1], d_out[0]]), np.array([d_out[1], -d_out[0]])):
            n = np.linalg.norm(d)
            if n > 1e-9 and _pip(tuple(cp + d / n * 0.3 * self.h), loops):
                return d / n
        return np.zeros(2)

    @staticmethod
    def _seg_inter(p, q, a, b):
        d1, d2 = q - p, b - a
        den = d1[0] * d2[1] - d1[1] * d2[0]
        if abs(den) < 1e-30:
            return None
        w = a - p
        t = (w[0] * d2[1] - w[1] * d2[0]) / den
        u = (w[0] * d1[1] - w[1] * d1[0]) / den
        if 1e-9 < t <= 1 and 0 <= u <= 1:
            return t, u
        return None

    def _hit_traces(self, p, q, traces, ignore_start=None):
        best = None
        for tr in traces:
            if ignore_start is not None and np.linalg.norm(tr[0] - ignore_start) < 0.5 * self.h:
                continue
            for k in range(len(tr) - 1):
                r = self._seg_inter(p, q, tr[k], tr[k + 1])
                if r and (best is None or r[0] < best[0]):
                    best = (r[0], tr[k] + (tr[k + 1] - tr[k]) * r[1])
        return None if best is None else best[1]

    def _hit_boundary(self, p, q, loops):
        best = None
        for lp in loops:
            n = len(lp)
            for k in range(n):
                a, b = np.array(lp[k]), np.array(lp[(k + 1) % n])
                r = self._seg_inter(p, q, a, b)
                if r and (best is None or r[0] < best[0]):
                    best = (r[0], a + (b - a) * r[1])
        return None if best is None else best[1]
