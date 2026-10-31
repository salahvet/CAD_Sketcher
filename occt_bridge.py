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


def shape_to_mesh_object(shape, name, tolerance=0.0005, existing=None):
    """Tessellation vers un objet mesh Blender - shading lisse, aretes
    vives aux frontieres. Le mesh est une VUE, jamais a editer."""
    import bpy
    import bmesh
    import math
    verts, triangles = shape.tessellate(tolerance)
    me = bpy.data.meshes.new(name)
    me.from_pydata([tuple(v) for v in verts], [], [tuple(t) for t in triangles])
    me.update()

    bm = bmesh.new()
    bm.from_mesh(me)
    for f in bm.faces:
        f.smooth = True
    limit = math.radians(28)
    for e in bm.edges:
        if len(e.link_faces) == 2:
            if e.calc_face_angle(0.0) > limit:
                e.smooth = False
        else:
            e.smooth = False
    bm.to_mesh(me)
    bm.free()
    me.update()
    if existing is not None:
        old = existing.data
        existing.data = me
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
