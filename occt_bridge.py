"""Pont CAD Sketcher -> OpenCASCADE (build123d).

Convertit un sketch contraint (lignes, arcs, cercles, splines bezier)
en Face B-rep EXACTE. La geometrie n'est pas approximee : les arcs
restent des arcs, les beziers des courbes de Bezier OCCT.
"""

import math
import sys


def _ensure_build123d():
    target = None
    try:
        import bpy
        target = bpy.utils.user_resource("SCRIPTS", path="modules")
    except Exception:
        pass
    if target and target not in sys.path:
        sys.path.insert(0, target)
    import build123d
    return build123d


def sketch_to_wires(context, sketch):
    """Retourne (wires, plane_normal) : la liste des contours fermes du
    sketch en Wires OCCT exacts, dans l'espace monde."""
    b3d = _ensure_build123d()
    from .utilities.walker import EntityWalker
    from mathutils import Vector

    M = sketch.wp.matrix_basis

    def P(co):
        v = M @ Vector((co[0], co[1], 0.0))
        return (v.x, v.y, v.z)

    def arc_edge(e, inv):
        ct = e.ct.co
        start, end = e.start.co, e.end.co
        a0 = math.atan2(start[1] - ct[1], start[0] - ct[0])
        a1 = math.atan2(end[1] - ct[1], end[0] - ct[0])
        r = math.hypot(start[0] - ct[0], start[1] - ct[1])
        da = (a1 - a0) % (2.0 * math.pi)
        if da < 1e-9:
            da = 2.0 * math.pi
        am = a0 + da / 2.0
        mid = (ct[0] + r * math.cos(am), ct[1] + r * math.sin(am))
        pa, pm, pb = P(start), P(mid), P(end)
        if inv:
            pa, pb = pb, pa
        return b3d.Edge.make_three_point_arc(pa, pm, pb)

    walker = EntityWalker(context.scene, sketch)
    wires = []
    for entities, directions in walker.paths:
        edges = []
        for e, inv in zip(entities, directions):
            t = type(e).__name__
            if t == "SlvsLine2D":
                a, b = P(e.p1.co), P(e.p2.co)
                if inv:
                    a, b = b, a
                edges.append(b3d.Edge.make_line(a, b))
            elif t == "SlvsArc":
                edges.append(arc_edge(e, inv))
            elif t == "SlvsBezier":
                pts = [P(e.p1.co), P(e.ctrl1.co), P(e.ctrl2.co), P(e.p2.co)]
                if inv:
                    pts.reverse()
                edges.append(b3d.Edge.make_bezier(*pts))
        if edges:
            wires.append(b3d.Wire(edges))

    # cercles isoles (contours fermes sans extremites, ignores du walker)
    ents = context.scene.sketcher.entities
    normal = (M.to_3x3() @ Vector((0, 0, 1))).normalized()
    for c in ents.circles:
        if c.sketch_i != sketch.slvs_index or c.construction:
            continue
        plane = b3d.Plane(origin=P(c.ct.co), z_dir=tuple(normal))
        wires.append(b3d.Wire([b3d.Edge.make_circle(c.radius, plane)]))

    # ellipses (fork) : contours fermes, comme les cercles
    for el in getattr(ents, "ellipses", ()):
        if el.sketch_i != sketch.slvs_index or el.construction:
            continue
        A, B = el.axes()
        ra, rb = A.length, B.length
        if ra < 1e-12 or rb < 1e-12:
            continue
        major = A if ra >= rb else B
        x_dir = (M.to_3x3() @ major.to_3d()).normalized()
        plane = b3d.Plane(
            origin=P(el.ct.co), x_dir=tuple(x_dir), z_dir=tuple(normal)
        )
        wires.append(
            b3d.Wire([b3d.Edge.make_ellipse(max(ra, rb), min(ra, rb), plane)])
        )

    return wires, tuple(normal)


def sketch_to_face(context, sketch):
    """Face B-rep exacte du sketch : plus grand contour = exterieur,
    les autres = trous."""
    b3d = _ensure_build123d()
    wires, normal = sketch_to_wires(context, sketch)
    closed = [w for w in wires if w.is_closed]
    if not closed:
        return None

    faces = [(w, abs(b3d.Face(w).area)) for w in closed]
    faces.sort(key=lambda t: t[1], reverse=True)
    outer = faces[0][0]
    inners = [w for w, _ in faces[1:]]
    face = b3d.Face(outer, inners) if inners else b3d.Face(outer)
    if face.normal_at(face.center()).dot(b3d.Vector(normal)) < 0:
        face = -face
    return face


def _faces_of(shape):
    if shape is None:
        return []
    try:
        return list(shape.faces())
    except Exception:
        return []


def sketch_regions(context, sketch, min_area=1e-14):
    """Every region of the sketch, like other CAD tools show them: closed contours are split
    against each other (a ring and its hole are two regions, two overlapping contours three).
    Faces in world space, oriented along the sketch normal."""
    b3d = _ensure_build123d()
    wires, normal = sketch_to_wires(context, sketch)
    contours = [b3d.Face(w) for w in wires if w.is_closed]
    pieces = []
    for f in contours:
        if not pieces:
            pieces = [f]
            continue
        new = []
        for p in pieces:
            new += [x for x in _faces_of(p & f) if x.area > min_area]
            new += [x for x in _faces_of(p - f) if x.area > min_area]
        rest = [f]
        for p in pieces:
            rest = [x for r in rest for x in _faces_of(r - p) if x.area > min_area]
        pieces = new + rest
    n = b3d.Vector(normal)
    out = []
    for p in pieces:
        if p.normal_at(p.center()).dot(n) < 0:
            p = -p
        out.append(p)
    return out


def region_interior_point(face):
    """A point inside the region (centroid of its largest triangle): the centre of a ring is not."""
    verts, tris = face.tessellate(1e-6)
    best, area = None, -1.0
    for a, b, c in tris:
        A, B, C = verts[a], verts[b], verts[c]
        ar = (B - A).cross(C - A).length
        if ar > area:
            best, area = (A + B + C) / 3, ar
    return best


def region_faces(context, sketch, probes, tol=1e-7):
    """Regions of the sketch that contain one of the probe points (world coordinates)."""
    b3d = _ensure_build123d()
    pts = [b3d.Vector(*p) for p in probes]
    return [f for f in sketch_regions(context, sketch) if any(f.distance_to(q) <= tol for q in pts)]


def exact_mesh_data(shape, lin=None, ang=0.2):
    """Per-face triangulation of a B-rep with exact surface normals.

    Returns (verts, tris, normals) with one vertex set per face (no welding across faces), so
    sharp edges stay sharp and tangent faces get matching normals. lin defaults to 1/2000 of the
    part size."""
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.BRep import BRep_Tool
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
    from OCP.TopLoc import TopLoc_Location
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    from OCP.gp import gp_Pnt, gp_Vec
    from OCP.Bnd import Bnd_Box
    from OCP.BRepBndLib import BRepBndLib
    sh = getattr(shape, "wrapped", shape)
    if lin is None:
        box = Bnd_Box()
        add = getattr(BRepBndLib, "Add_s", None) or BRepBndLib.Add
        add(sh, box)
        lo, hi = box.CornerMin(), box.CornerMax()
        x0, y0, z0, x1, y1, z1 = lo.X(), lo.Y(), lo.Z(), hi.X(), hi.Y(), hi.Z()
        lin = max(max(x1 - x0, y1 - y0, z1 - z0) / 2000.0, 1e-7)
    BRepMesh_IncrementalMesh(sh, lin, False, ang, True)
    verts, tris, normals = [], [], []
    ex = TopExp_Explorer(sh, TopAbs_FACE)
    from OCP.TopoDS import TopoDS
    to_face = getattr(TopoDS, "Face_s", None) or TopoDS.Face
    while ex.More():
        face = to_face(ex.Current())
        ex.Next()
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(face, loc)
        if tri is None:
            continue
        trsf = loc.Transformation()
        rev = face.Orientation() == TopAbs_REVERSED
        surf = BRepAdaptor_Surface(face) if tri.HasUVNodes() else None
        base = len(verts)
        for i in range(1, tri.NbNodes() + 1):
            p = tri.Node(i).Transformed(trsf)
            verts.append((p.X(), p.Y(), p.Z()))
            n = None
            if surf is not None:
                uv = tri.UVNode(i)
                pp, du, dv = gp_Pnt(), gp_Vec(), gp_Vec()
                try:
                    surf.D1(uv.X(), uv.Y(), pp, du, dv)
                    nn = du.Crossed(dv)
                    if nn.Magnitude() > 1e-20:
                        nn.Normalize()
                        n = (nn.X(), nn.Y(), nn.Z())
                        if rev:
                            n = (-n[0], -n[1], -n[2])
                except Exception:
                    n = None
            normals.append(n)
        ftris = []
        for i in range(1, tri.NbTriangles() + 1):
            a, b, c = tri.Triangle(i).Get()
            ftris.append((base + a - 1, base + b - 1, base + c - 1))
        # winding from the exact normals (they include the face orientation)
        agree = 0
        for a, b, c in ftris:
            na = normals[a]
            if na is None:
                continue
            pa, pb, pc = verts[a], verts[b], verts[c]
            u = (pb[0] - pa[0], pb[1] - pa[1], pb[2] - pa[2])
            w = (pc[0] - pa[0], pc[1] - pa[1], pc[2] - pa[2])
            cr = (u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0])
            d = cr[0] * na[0] + cr[1] * na[1] + cr[2] * na[2]
            agree += 1 if d > 0 else (-1 if d < 0 else 0)
        flip = agree < 0 if any(normals[k] is not None for k in range(base, len(verts))) else rev
        tris += [(a, c, b) for a, b, c in ftris] if flip else ftris
    return verts, tris, normals


def shape_to_mesh_object(shape, name, tolerance=None, existing=None):
    """Display mesh of a B-rep: fine per-face triangulation with the exact surface normals
    (custom normals), so fillets and curved walls shade cleanly and sharp edges stay sharp.
    The mesh is a VIEW of the part, never to be edited."""
    import bpy
    import numpy as np
    verts, tris, normals = exact_mesh_data(shape, tolerance)
    V = np.array(verts, float)
    T = np.array(tris, int).reshape(-1, 3)
    # fallback normals from the triangles (area weighted), used where the exact one is missing
    fn = np.cross(V[T[:, 1]] - V[T[:, 0]], V[T[:, 2]] - V[T[:, 0]])
    vn = np.zeros_like(V)
    for k in range(3):
        np.add.at(vn, T[:, k], fn)
    ln = np.linalg.norm(vn, axis=1)
    vn = vn / np.maximum(ln, 1e-30)[:, None]
    N = []
    for i, n in enumerate(normals):
        if n is None or float(np.dot(n, vn[i])) < 0.2:
            N.append(tuple(vn[i]))
        else:
            N.append(n)
    me = bpy.data.meshes.new(name)
    me.from_pydata([tuple(v) for v in verts], [], [tuple(t) for t in tris])
    me.update()
    me.polygons.foreach_set("use_smooth", [True] * len(me.polygons))
    try:
        me.normals_split_custom_set_from_vertices(N)
    except Exception:
        pass
    me.update()
    if existing is not None:
        old = existing.data
        existing.data = me
        if old.users == 0:
            bpy.data.meshes.remove(old)
        return existing
    obj = bpy.data.objects.new(name, me)
    bpy.context.collection.objects.link(obj)
    return obj


def shape_edges_object(shape, name, existing=None, samples=24):
    """Objet courbe des ARETES EXACTES du B-rep (lignes 2 points,
    courbes echantillonnees finement)."""
    import bpy
    cu = bpy.data.curves.new(name, type="CURVE")
    cu.dimensions = "3D"
    for edge in shape.edges():
        try:
            gtype = edge.geom_type
            gt = str(gtype() if callable(gtype) else gtype)
        except Exception:
            gt = ""
        n = 2 if "LINE" in gt.upper() else samples
        pts = []
        for i in range(n):
            t = i / (n - 1)
            v = edge @ t
            pts.append((v.X, v.Y, v.Z))
        sp = cu.splines.new("POLY")
        sp.points.add(len(pts) - 1)
        for bp, co in zip(sp.points, pts):
            bp.co = (co[0], co[1], co[2], 1.0)
    if existing is not None:
        old = existing.data
        existing.data = cu
        bpy.data.curves.remove(old)
        return existing
    obj = bpy.data.objects.new(name, cu)
    bpy.context.collection.objects.link(obj)
    return obj


def sketch_guides(context, sketch, arc_step_deg=6.0, bezier_steps=24):
    """Lines, arcs and Bézier curves of a sketch as 3D polylines (world space), chained where
    exactly two of them meet. Used as retopology guides by OCCT Quad Mesh."""
    from mathutils import Vector

    M = sketch.wp.matrix_basis
    P = lambda co: tuple(M @ Vector((co[0], co[1], 0.0)))
    ents = context.scene.sketcher.entities
    items = []  # (start point id, end point id, polyline)
    for e in ents.lines2D:
        if e.sketch_i == sketch.slvs_index:
            items.append((e.p1.slvs_index, e.p2.slvs_index, [P(e.p1.co), P(e.p2.co)]))
    for e in ents.arcs:
        if e.sketch_i != sketch.slvs_index:
            continue
        ct, a, b = e.ct.co, e.start.co, e.end.co
        a0 = math.atan2(a[1] - ct[1], a[0] - ct[0])
        a1 = math.atan2(b[1] - ct[1], b[0] - ct[0])
        r = math.hypot(a[0] - ct[0], a[1] - ct[1])
        da = (a1 - a0) % (2.0 * math.pi) or 2.0 * math.pi
        n = max(2, int(math.degrees(da) / arc_step_deg) + 1)
        pts = [P((ct[0] + r * math.cos(a0 + da * k / n), ct[1] + r * math.sin(a0 + da * k / n))) for k in range(n + 1)]
        items.append((e.start.slvs_index, e.end.slvs_index, pts))
    for e in getattr(ents, "beziers", ()):
        if e.sketch_i != sketch.slvs_index:
            continue
        c = [e.p1.co, e.ctrl1.co, e.ctrl2.co, e.p2.co]
        pts = []
        for k in range(bezier_steps + 1):
            t = k / bezier_steps
            w = ((1 - t) ** 3, 3 * t * (1 - t) ** 2, 3 * t * t * (1 - t), t ** 3)
            pts.append(P((sum(w[i] * c[i][0] for i in range(4)), sum(w[i] * c[i][1] for i in range(4)))))
        items.append((e.p1.slvs_index, e.p2.slvs_index, pts))

    # chain through points shared by exactly two items
    degree = {}
    for a, b, _ in items:
        degree[a] = degree.get(a, 0) + 1
        degree[b] = degree.get(b, 0) + 1
    unused = list(range(len(items)))
    lines = []

    def extend(chain, end, at_end):
        while degree.get(end) == 2:
            nxt = next((j for j in unused if end in (items[j][0], items[j][1])), None)
            if nxt is None:
                break
            unused.remove(nxt)
            ja, jb, jp = items[nxt]
            seg = jp if ja == end else jp[::-1]   # seg starts at `end`
            if at_end:
                chain.extend(seg[1:])
            else:
                chain[:0] = seg[::-1][:-1]
            end = jb if ja == end else ja
        return chain

    while unused:
        i = unused.pop(0)
        a, b, pts = items[i]
        chain = extend(list(pts), b, True)
        chain = extend(chain, a, False)
        lines.append(chain)
    normal = tuple((M.to_3x3() @ Vector((0, 0, 1))).normalized())
    return {"normal": normal, "lines": lines}
