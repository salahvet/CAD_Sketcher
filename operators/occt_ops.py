"""PartDesign parametrique sur noyau OpenCASCADE.

Chaque piece est une RECETTE (arbre de features) stockee dans la scene :
    scene["occt_recipes"][part_name] = [feature, feature, ...]
Le solide B-rep est reconstruit integralement a chaque modification du
sketch (recompute), comme dans FreeCAD. Le mesh Blender n'est que la vue.
"""

import json

import bpy
from bpy.props import BoolProperty, EnumProperty, FloatProperty, IntProperty
from bpy.types import Context, Operator
from bpy.utils import register_classes_factory
from mathutils import Vector

from ..declarations import Operators
from .. import occt_bridge

RECIPES_KEY = "occt_recipes"
PICKS_KEY = "occt_edge_picks"
REGION_PICKS_KEY = "occt_region_picks"  # {sketch name: [[x, y], ...]} in sketch coordinates


def region_picks(scene, sketch_name):
    try:
        return json.loads(scene.get(REGION_PICKS_KEY, "{}")).get(sketch_name, [])
    except Exception:
        return []


def set_region_picks(scene, sketch_name, points):
    try:
        data = json.loads(scene.get(REGION_PICKS_KEY, "{}"))
    except Exception:
        data = {}
    if points:
        data[sketch_name] = points
    else:
        data.pop(sketch_name, None)
    scene[REGION_PICKS_KEY] = json.dumps(data)

# solides B-rep vivants (pour la selection des aretes)
_solid_cache = {}


# ---------------------------------------------------------------------------
# Recettes
# ---------------------------------------------------------------------------

def get_recipes(scene):
    raw = scene.get(RECIPES_KEY, "{}")
    try:
        return json.loads(raw)
    except Exception:
        return {}


def set_recipes(scene, recipes):
    scene[RECIPES_KEY] = json.dumps(recipes)


def find_sketch(context, name):
    for s in context.scene.sketcher.entities.sketches:
        if s.name == name:
            return s
    return None


# ---------------------------------------------------------------------------
# Selecteurs d'aretes (probleme du nommage topologique : criteres
# geometriques re-evalues a chaque recompute, comme les selecteurs FreeCAD)
# ---------------------------------------------------------------------------

def _blend_tol(solid):
    """3D tolerance for the fillet approximation. OCCT's defaults (1e-4 ...) are meant for parts
    modelled in millimetres; our parts are in metres, so a default fillet is approximated to
    0.1 mm: along an ellipse or a Bezier it is cut into many patches whose corner vertices merge
    (1 mm vertex tolerance) -> wrong triangles and dark specks, and fillets that fail."""
    from OCP.Bnd import Bnd_Box
    from OCP.BRepBndLib import BRepBndLib
    box = Bnd_Box()
    (getattr(BRepBndLib, "Add_s", None) or BRepBndLib.Add)(solid.wrapped, box)
    lo, hi = box.CornerMin(), box.CornerMax()
    diag = ((hi.X() - lo.X()) ** 2 + (hi.Y() - lo.Y()) ** 2 + (hi.Z() - lo.Z()) ** 2) ** 0.5
    return min(max(diag * 1e-6, 1e-8), 1e-5)


def _solids_of(shape):
    """build123d Solids of an OCCT shape (Shape.cast returns None on some build123d versions)."""
    import build123d as b3d
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopAbs import TopAbs_SOLID
    from OCP.TopoDS import TopoDS
    to_solid = getattr(TopoDS, "Solid_s", None) or TopoDS.Solid
    out = []
    ex = TopExp_Explorer(shape, TopAbs_SOLID)
    while ex.More():
        out.append(b3d.Solid(to_solid(ex.Current())))
        ex.Next()
    return out


def blend_edges(solid, edges, op, size):
    """Fillet / chamfer. Fillets are built straight with OCCT at a tolerance scaled to the part
    (clean single surfaces, no merged vertices), then with build123d, then with OCCT defaults.
    The result must be one valid closed solid."""
    import build123d as b3d
    from OCP.BRepCheck import BRepCheck_Analyzer
    from OCP.ShapeFix import ShapeFix_Shape
    err = None
    attempts = []
    if op == "fillet":
        from OCP.BRepFilletAPI import BRepFilletAPI_MakeFillet
        from OCP.ChFi3d import ChFi3d_Rational, ChFi3d_QuasiAngular
        tol = _blend_tol(solid)

        def fillet_maker(kind, scaled):
            def make():
                mk = BRepFilletAPI_MakeFillet(solid.wrapped, kind)
                if scaled:
                    # Tang, Tesp, T2d, TApp3d, TolApp2d, Fleche
                    mk.SetParams(1e-2, tol, tol / 10.0, tol, tol / 10.0, tol * 10.0)
                return mk
            return make
        attempts = [fillet_maker(ChFi3d_Rational, True), fillet_maker(ChFi3d_QuasiAngular, True),
                    "b3d", fillet_maker(ChFi3d_Rational, False)]
    else:
        from OCP.BRepFilletAPI import BRepFilletAPI_MakeChamfer
        attempts = ["b3d", lambda: BRepFilletAPI_MakeChamfer(solid.wrapped)]
    for make in attempts:
        try:
            if make == "b3d":
                return b3d.fillet(edges, radius=size) if op == "fillet" else b3d.chamfer(edges, length=size)
            mk = make()
            for e in edges:
                mk.Add(size, e.wrapped)
            mk.Build()
            if not mk.IsDone():
                continue
            shape = mk.Shape()
            if not BRepCheck_Analyzer(shape).IsValid():
                fix = ShapeFix_Shape(shape)
                fix.Perform()
                shape = fix.Shape()
                if not BRepCheck_Analyzer(shape).IsValid():
                    continue
            solids = _solids_of(shape)
            if len(solids) != 1 or not solids[0].volume:
                continue  # OCCT built something, but not one closed solid
            return solids[0]
        except Exception as e:
            err = e
    raise err or ValueError(f"{op} of {size * 1000:.3g} mm failed")


def select_edges(solid, feat, base_origin, normal):
    import build123d as b3d
    n = b3d.Vector(normal)
    o = b3d.Vector(base_origin)
    edges = solid.edges()
    mode = feat.get("select", "ALL")

    if mode == "CURSOR":
        probe = b3d.Vector(tuple(feat["probe"]))
        return [min(edges, key=lambda e: (e.center() - probe).length)]

    if mode == "PICKED":
        out = []
        for pr in feat.get("probes", []):
            probe = b3d.Vector(tuple(pr))
            best = min(edges, key=lambda e: (e.center() - probe).length)
            if best not in out:
                out.append(best)
        return out

    heights = [(e.center() - o).dot(n) for e in edges]
    h_min, h_max = min(heights), max(heights)
    span = max(h_max - h_min, 1e-9)

    def is_vertical(e):
        t = (e @ 1.0) - (e @ 0.0)
        if t.length < 1e-9:
            return False
        return abs(t.normalized().dot(n)) > 0.7

    out = []
    for e, h in zip(edges, heights):
        f = (h - h_min) / span
        if mode == "TOP" and f > 0.9 and not is_vertical(e):
            out.append(e)
        elif mode == "BOTTOM" and f < 0.1 and not is_vertical(e):
            out.append(e)
        elif mode == "VERTICAL" and is_vertical(e):
            out.append(e)
        elif mode == "ALL":
            out.append(e)
    return out


# ---------------------------------------------------------------------------
# Moteur de recompute
# ---------------------------------------------------------------------------

MIRROR_PLANES = [
    ("XZ", "XZ (flip Y)", "Mirror across the XZ plane"),
    ("YZ", "YZ (flip X)", "Mirror across the YZ plane"),
    ("XY", "XY (flip Z)", "Mirror across the XY plane"),
]
_PLANE_NORMALS = {"XY": (0, 0, 1), "XZ": (0, 1, 0), "YZ": (1, 0, 0)}


def _mirror_plane(feat):
    import build123d as b3d
    n = _PLANE_NORMALS.get(feat.get("plane") or feat.get("mirror"), (0, 1, 0))
    origin = tuple(feat.get("center") or (0, 0, 0))
    return b3d.Plane(origin=origin, z_dir=n), Vector(n), Vector(origin)


def _reflect(v, n, o):
    """Reflet du point v par le plan (normale n, passant par o)."""
    v = Vector(v)
    return v - 2 * (v - o).dot(n) * n


def build_solid(context, recipe, report=None, skip=(), part_name="", _stack=()):
    """Solide B-rep d'une recette, sans toucher a la scene.

    skip : index des fonctions ignorees (en plus de celles marquees "suppressed").
    Retourne (solid, base_origin, normal) ; solid vaut None si rien n'est construit.
    """
    import build123d as b3d
    solid = None
    base_origin = (0, 0, 0)
    normal = (0, 0, 1)
    bodies = {}  # index de fonction -> (op, volume-outil, repere de l'esquisse)

    for index, feat in enumerate(recipe):
        op = feat["op"]
        if feat.get("suppressed") or index in skip:
            continue
        if op == "derive":
            src = feat.get("from")
            recipes = get_recipes(context.scene)
            if src in _stack or src == part_name:
                if report:
                    report({"WARNING"}, f"Derive: cycle through '{src}'")
                continue
            if src not in recipes:
                if report:
                    report({"WARNING"}, f"Derive: source part '{src}' not found")
                continue
            body, o, nv = build_solid(context, recipes[src], report, set(feat.get("suppress", [])),
                                      src, _stack + (part_name,))
            if body is None:
                continue
            if feat.get("mirror") and feat["mirror"] != "NONE":
                plane, n, c = _mirror_plane(feat)
                body = body.mirror(plane)
                o = tuple(_reflect(o, n, c))
                nv = tuple(_reflect(Vector(nv) + c, n, c) - _reflect(c, n, c))
            if solid is None:
                solid, base_origin, normal = body, o, nv
            else:
                solid = solid + body
            continue
        if op == "mirror":
            if solid is None:
                continue
            plane, _n, _c = _mirror_plane(feat)
            try:
                mirrored = solid.mirror(plane)
                solid = solid + mirrored if feat.get("fuse", True) else mirrored
            except Exception as e:
                if report:
                    report({"WARNING"}, f"Mirror failed: {e}")
            continue
        if op == "pattern":
            solid = _apply_pattern(context, solid, feat, bodies, report)
            continue
        if op in ("pad", "pocket"):
            sketch = find_sketch(context, feat["sketch"])
            if sketch is None:
                if report:
                    report({"WARNING"}, f"Sketch '{feat['sketch']}' introuvable")
                continue
            M = sketch.wp.matrix_basis
            if feat.get("regions"):
                probes = [tuple(M @ Vector((p[0], p[1], 0.0))) for p in feat["regions"]]
                faces = occt_bridge.region_faces(context, sketch, probes)
                if not faces:
                    if report:
                        report({"WARNING"}, f"No region of '{feat['sketch']}' at the picked points")
                    continue
            else:
                face = occt_bridge.sketch_to_face(context, sketch)
                if face is None:
                    if report:
                        report({"WARNING"}, f"Contour non ferme : {feat['sketch']}")
                    continue
                faces = [face]
            body = extrude_feature(faces, feat, M, solid, report)
            if body is None:
                continue
            bodies[index] = (op, body, M.copy())
            if op == "pad":
                if solid is None:
                    solid = body
                    M = sketch.wp.matrix_basis
                    base_origin = tuple(M.translation)
                    nv = (M.to_3x3() @ Vector((0, 0, 1))).normalized()
                    normal = tuple(nv)
                else:
                    solid = solid + body
            else:  # pocket
                if solid is None:
                    continue
                solid = solid - body
        elif op in ("fillet", "chamfer"):
            if solid is None:
                continue
            edges = select_edges(solid, feat, base_origin, normal)
            if not edges:
                continue
            try:
                solid = blend_edges(solid, edges, op, feat["radius"])
            except Exception as e:
                if report:
                    # tell what would work: try smaller sizes (not applied)
                    ok = None
                    for k in (0.75, 0.5, 0.3, 0.15):
                        try:
                            blend_edges(solid, edges, op, feat["radius"] * k)
                            ok = feat["radius"] * k
                            break
                        except Exception:
                            continue
                    hint = (f" A {op} of {ok * 1000:.2f} mm works on these edges." if ok else
                            " Check that the outline joins smoothly (Tangent constraint) where these edges meet.")
                    report({"WARNING"}, f"{op.capitalize()} of {feat['radius'] * 1000:.2f} mm failed on "
                                        f"{len(edges)} edge(s): the feature is kept but not applied.{hint}")

    return solid, base_origin, normal


EXTENTS = [
    ("BLIND", "Blind", "Extrude by the given depth"),
    ("MID_PLANE", "Mid-Plane", "Half the depth on each side of the sketch"),
    ("TWO_SIDES", "Two Sides", "A depth on each side of the sketch, set separately"),
    ("THROUGH_ALL", "Through All", "Through the whole part, on one side"),
    ("THROUGH_ALL_BOTH", "Through All (Both Sides)", "Through the whole part, on both sides"),
    ("UP_TO_NEXT", "Up to Next", "Up to the next face of the part"),
    ("UP_TO_LAST", "Up to Last", "Up to the last face of the part"),
    ("UP_TO_CURSOR", "Up to 3D Cursor", "Up to the plane through the 3D cursor, parallel to the sketch"),
]


def feature_extent(feat):
    """Extent of a pad / pocket; older recipes only have depth and symmetric."""
    return feat.get("extent") or ("MID_PLANE" if feat.get("symmetric") else "BLIND")


def extrude_feature(faces, feat, M, solid, report=None):
    """Tool body of a pad / pocket: every face extruded according to the feature's extent, fused."""
    import build123d as b3d
    n = (M.to_3x3() @ Vector((0, 0, 1))).normalized()
    sign = -1.0 if feat.get("reverse") else 1.0
    d = b3d.Vector(*(n * sign))
    ext = feature_extent(feat)
    depth = float(feat.get("depth", 0.001))
    depth2 = float(feat.get("depth2", 0.0))

    def signed(f, amount, direction):
        """Extrude by a signed amount: a negative depth goes the other way."""
        if abs(amount) < 1e-12:
            return None
        return b3d.extrude(f, amount=abs(amount), dir=direction if amount > 0 else -direction)

    def far():
        """A length that crosses everything: faces and part, wherever the sketch is."""
        L = 0.0
        o = b3d.Vector(*M.translation)
        for shape in [*faces] + ([solid] if solid is not None else []):
            bb = shape.bounding_box()
            L = max(L, bb.diagonal + (bb.center() - o).length)
        return 2.0 * L + 0.01

    bodies = []
    for f in faces:
        try:
            if ext == "MID_PLANE":
                b = b3d.extrude(f, amount=abs(depth) / 2.0, dir=d, both=True)
            elif ext == "TWO_SIDES":
                parts = [x for x in (signed(f, depth, d), signed(f, depth2, -d)) if x is not None]
                if not parts:
                    continue
                b = parts[0] if len(parts) == 1 else parts[0] + parts[1]
            elif ext == "THROUGH_ALL":
                b = b3d.extrude(f, amount=far(), dir=d)
            elif ext == "THROUGH_ALL_BOTH":
                b = b3d.extrude(f, amount=far(), dir=d, both=True)
            elif ext in ("UP_TO_NEXT", "UP_TO_LAST"):
                if solid is None:
                    if report:
                        report({"WARNING"}, "Up to Next / Last needs an existing solid: blind depth used")
                    b = signed(f, depth, d)
                else:
                    until = b3d.Until.NEXT if ext == "UP_TO_NEXT" else b3d.Until.LAST
                    b = b3d.extrude(f, dir=d, until=until, target=solid)
            elif ext == "UP_TO_CURSOR":
                dist = (Vector(feat.get("probe", M.translation)) - M.translation).dot(n)
                if abs(dist) < 1e-9:
                    if report:
                        report({"WARNING"}, "The 3D cursor is on the sketch plane: nothing to extrude")
                    continue
                b = b3d.extrude(f, amount=abs(dist), dir=b3d.Vector(*(n if dist > 0 else -n)))
            else:
                b = signed(f, depth, d)
            if b is None:
                continue
        except Exception as e:
            if report:
                report({"WARNING"}, f"Extrusion failed ({ext}): {e}")
            continue
        bodies.append(b)
    if not bodies:
        return None
    body = bodies[0]
    for b in bodies[1:]:
        body = body + b
    return body


def dependents(recipes, part_name):
    """Pieces derivees (directement ou non) de part_name, sources avant derivees."""
    out, todo = [], [part_name]
    while todo:
        cur = todo.pop(0)
        for name, recipe in recipes.items():
            if name not in out and name != part_name and any(
                    f.get("op") == "derive" and f.get("from") == cur for f in recipe):
                out.append(name)
                todo.append(name)
    return out


def _hide_object(ob):
    if ob is None:
        return
    try:
        ob.hide_set(True)
    except RuntimeError:  # not in the view layer
        ob.hide_viewport = True


def hide_used_sketch(context, sketch):
    """A sketch consumed by an OCCT feature: hide it, its workplane and its converted objects."""
    if sketch is None or context.scene.sketcher.active_sketch == sketch:
        return
    sketch.visible = False
    if sketch.wp is not None and not sketch.wp.origin:
        sketch.wp.visible = False
    for attr in ("target_object", "target_curve_object"):
        _hide_object(getattr(sketch, attr, None))


def tidy_view(context):
    """Hide what OCCT parts make redundant; returns (hidden, removed) counts."""
    sse = context.scene.sketcher
    entities = sse.entities
    active = sse.active_sketch
    used = {f.get("sketch") for recipe in get_recipes(context.scene).values() for f in recipe}
    hidden = 0
    for sk in entities.sketches:
        if sk.name in used and sk != active:
            before = (sk.visible, sk.wp.visible if sk.wp else False)
            hide_used_sketch(context, sk)
            hidden += before != (sk.visible, sk.wp.visible if sk.wp else False)
    # workplanes with no sketch on them (left over from Add Workplane)
    with_sketch = {sk.wp.slvs_index for sk in entities.sketches if sk.wp}
    for wp in entities.workplanes:
        if wp.visible and not wp.origin and wp.slvs_index not in with_sketch:
            wp.visible = False
            hidden += 1
    # exact-edge objects whose part was deleted
    removed = 0
    for ob in list(context.scene.objects):
        if ob.name.endswith("_Edges") and ob.type == "CURVE" and ob.parent is None \
                and ob.name[:-6] not in bpy.data.objects:
            data = ob.data
            bpy.data.objects.remove(ob)
            if data.users == 0:
                bpy.data.curves.remove(data)
            removed += 1
    return hidden, removed


def recompute_part(context, part_name, report=None, cascade=True):
    recipes = get_recipes(context.scene)
    recipe = recipes.get(part_name)
    if not recipe:
        return None

    occt_bridge.SMOOTH_REPORT.clear()
    solid, _o, _n = build_solid(context, recipe, report, part_name=part_name)
    if occt_bridge.SMOOTH_REPORT and report:
        angs = sorted({round(a, 1) for _p, a in occt_bridge.SMOOTH_REPORT}, reverse=True)
        report({"INFO"}, f"{part_name}: {len(occt_bridge.SMOOTH_REPORT)} almost-tangent sketch joint(s) "
                         f"made tangent ({', '.join(f'{a}°' for a in angs[:4])}): no crease on the walls")
    if solid is None:
        return None

    _solid_cache[part_name] = solid
    existing = context.scene.objects.get(part_name)
    obj = occt_bridge.shape_to_mesh_object(solid, part_name, existing=existing)
    obj["occt_part"] = True
    # mesh-modifier features (legacy Pad / Pocket / Fillet) do not belong on an OCCT part:
    # they cut / bevel the display mesh a second time and spoil it
    legacy = [m.name for m in obj.modifiers if m.name.startswith("SLVS_")]
    for name in legacy:
        obj.modifiers.remove(obj.modifiers[name])
    if legacy and report:
        report({"WARNING"}, f"{part_name}: removed the mesh modifiers {', '.join(legacy)} "
                            "(mesh features); OCCT parts are edited with the OCCT features")

    # objet des aretes exactes du B-rep
    edges_name = part_name + "_Edges"
    e_existing = context.scene.objects.get(edges_name)
    e_obj = occt_bridge.shape_edges_object(solid, edges_name, existing=e_existing)
    e_obj.parent = obj
    e_obj.hide_select = True

    if cascade:  # les pieces derivees suivent leur source
        for name in dependents(recipes, part_name):
            try:
                recompute_part(context, name, report, cascade=False)
            except Exception:
                continue
    return obj


# ---------------------------------------------------------------------------
# Repetition de fonction (pattern)
# ---------------------------------------------------------------------------

PATTERN_DIRECTIONS = [
    ("SKETCH_X", "Sketch X", "Along the X axis of the repeated feature's sketch"),
    ("SKETCH_Y", "Sketch Y", "Along the Y axis of the repeated feature's sketch"),
    ("NORMAL", "Sketch Normal", "Along the normal of the repeated feature's sketch"),
    ("WORLD_X", "World X", ""), ("WORLD_Y", "World Y", ""), ("WORLD_Z", "World Z", ""),
]


def _pattern_vector(name, matrix):
    """Direction unitaire (monde) d'un nom de direction, dans le repere de l'esquisse."""
    local = {"SKETCH_X": (1, 0, 0), "SKETCH_Y": (0, 1, 0), "NORMAL": (0, 0, 1)}
    world = {"WORLD_X": (1, 0, 0), "WORLD_Y": (0, 1, 0), "WORLD_Z": (0, 0, 1)}
    if name in world:
        return Vector(world[name])
    return (matrix.to_3x3() @ Vector(local.get(name, (1, 0, 0)))).normalized()


def pattern_copies(body, feat, matrix):
    """Copies 1..n-1 du volume-outil (la copie 0 est la fonction elle-meme)."""
    import build123d as b3d
    count = int(feat.get("count", 2))
    copies = []
    if feat.get("kind", "linear") == "circular":
        axis_dir = _pattern_vector(feat.get("axis", "NORMAL"), matrix)
        centre = Vector(feat["center"]) if feat.get("center") else matrix.translation
        total = float(feat.get("angle", 360.0))
        # 360 deg : la derniere copie ne doit pas retomber sur la premiere
        step = total / count if abs(total - 360.0) < 1e-6 else total / max(count - 1, 1)
        axis = b3d.Axis(tuple(centre), tuple(axis_dir))
        for k in range(1, count):
            copies.append(body.rotate(axis, step * k))
    else:
        d = _pattern_vector(feat.get("direction", "SKETCH_X"), matrix) * float(feat.get("spacing", 0.001))
        for k in range(1, count):
            copies.append(body.moved(b3d.Location((d.x * k, d.y * k, d.z * k))))
    return copies


def _apply_pattern(context, solid, feat, bodies, report=None):
    src = feat.get("feature")
    if src is None or src not in bodies:
        if report:
            report({"WARNING"}, "Pattern: the repeated feature is missing or suppressed")
        return solid
    op, body, matrix = bodies[src]
    copies = pattern_copies(body, feat, matrix)
    if not copies:
        return solid
    tool = copies[0].fuse(*copies[1:]) if len(copies) > 1 else copies[0]
    if solid is None:
        return tool if op == "pad" else None
    try:
        return solid + tool if op == "pad" else solid - tool
    except Exception as e:
        if report:
            report({"WARNING"}, f"Pattern failed: {e}")
        return solid


def recompute_all(scene):
    context = bpy.context
    for part_name in list(get_recipes(scene).keys()):
        try:
            recompute_part(context, part_name)
        except Exception:
            continue


def recompute_for_sketch(scene, sketch_name):
    """Ne recalcule que les pieces qui referencent ce sketch (et leurs derivees)."""
    context = bpy.context
    recipes = get_recipes(scene)
    direct = [n for n, r in recipes.items() if any(f.get("sketch") == sketch_name for f in r)]
    ordre = []
    for n in direct:
        for m in [n] + dependents(recipes, n):
            if m in ordre:
                ordre.remove(m)  # une derivee passe apres toutes ses sources
            ordre.append(m)
    for part_name in ordre:
        try:
            recompute_part(context, part_name, cascade=False)
        except Exception:
            continue


# hook parametrique : recompute apres chaque conversion de sketch
_wrapped = False
_busy = False


def _wrap_converter():
    global _wrapped
    if _wrapped:
        return
    from .. import converters as conv
    original = conv.update_convertor_geometry

    def wrapped(scene, *args, **kwargs):
        global _busy
        result = original(scene, *args, **kwargs)
        if _busy:
            return result
        _busy = True
        try:
            recompute_all(scene)
        except Exception:
            pass
        finally:
            _busy = False
        return result

    conv.update_convertor_geometry = wrapped

    # Hook principal : le SOLVEUR (declenche a chaque modification du
    # sketch, y compris le drag des points -> parametrique en direct)
    from .. import solver as solver_module
    original_solve = solver_module.Solver.solve

    def solve_wrapped(self, report=True):
        global _busy
        ok = original_solve(self, report)
        sk = getattr(self, "sketch", None)
        if (not _busy and ok and sk is not None
                and getattr(sk, "is_sketch", False)):
            _busy = True
            try:
                recompute_for_sketch(bpy.context.scene, sk.name)
            except Exception:
                pass
            finally:
                _busy = False
        return ok

    solver_module.Solver.solve = solve_wrapped
    _wrapped = True


# ---------------------------------------------------------------------------
# Operateurs
# ---------------------------------------------------------------------------

def _parts_enum(self, context):
    items = [(k, k, "") for k in get_recipes(context.scene).keys()]
    return items or [("NONE", "<no part>", "")]


def _ensure_unique_name(context, sketch):
    """Les references de recette sont par nom : impose l'unicite."""
    others = [s for s in context.scene.sketcher.entities.sketches
              if s != sketch]
    names = {s.name for s in others}
    if sketch.name in names:
        base = sketch.name
        i = 1
        while f"{base}.{i:03d}" in names:
            i += 1
        sketch.name = f"{base}.{i:03d}"
    return sketch


def sketch_items(self, context):
    """Sketches of the scene, the last one left first (default choice)."""
    sketches = [s.name for s in context.scene.sketcher.entities.sketches]
    last = context.scene.get("occt_last_sketch", "")
    if last in sketches:
        sketches.remove(last)
        sketches.insert(0, last)
    sketch_items.cache = [(n, n, "") for n in sketches] or [("NONE", "<no sketch>", "")]
    return sketch_items.cache


def sketch_by_name(context, name):
    return next((s for s in context.scene.sketcher.entities.sketches if s.name == name), None)


def poll_outside_sketch(cls, context):
    """Features are made outside sketch mode, on a chosen sketch, to avoid any confusion."""
    if context.mode != "OBJECT":
        return False
    if context.scene.sketcher.active_sketch is not None:
        cls.poll_message_set("Leave the sketch first, then choose the sketch in the dialog")
        return False
    if not len(context.scene.sketcher.entities.sketches):
        cls.poll_message_set("No sketch yet")
        return False
    return True


def _active_sketch(context):
    s = context.scene.sketcher.active_sketch
    if s is not None and getattr(s, "is_sketch", False):
        return s
    return None


def _pad_parts_enum(self, context):
    items = [("__NEW__", "New Part", "Start a new part from this sketch")]
    items += [(k, k, "") for k in get_recipes(context.scene).keys()]
    _pad_parts_enum.cache = items
    return items


def zero_depth(op):
    """True when the depths leave nothing to extrude."""
    if op.extent == "TWO_SIDES":
        return abs(op.depth) < 1e-9 and abs(op.depth2) < 1e-9
    return op.extent in ("BLIND", "MID_PLANE") and abs(op.depth) < 1e-9


def _draw_depths(op, col):
    if op.extent == "TWO_SIDES":
        col.prop(op, "depth", text="Depth (Side 1)")
        col.prop(op, "depth2", text="Depth (Side 2)")
    elif op.extent in ("BLIND", "MID_PLANE"):
        col.prop(op, "depth")


class _ExtrudeProps:
    sketch: EnumProperty(name="Sketch", items=sketch_items, description="Sketch to extrude (default: the last one left)")
    extent: EnumProperty(name="Extent", items=EXTENTS, default="BLIND")
    depth: FloatProperty(name="Depth", default=0.003, soft_min=-10.0, soft_max=10.0, unit="LENGTH",
                         description="Negative: extrude on the other side of the sketch")
    depth2: FloatProperty(name="Depth 2", default=0.003, soft_min=-10.0, soft_max=10.0, unit="LENGTH",
                          description="Depth on the other side of the sketch (Two Sides)")
    reverse: BoolProperty(name="Reverse", default=False, description="Extrude on the other side of the sketch")
    use_regions: BoolProperty(name="Picked Regions Only", default=True,
                              description="Extrude only the regions picked with OCCT Pick Regions")

    def _draw_extent(self, context, col):
        col.prop(self, "extent")
        _draw_depths(self, col)
        if self.extent in ("BLIND", "TWO_SIDES", "THROUGH_ALL", "UP_TO_NEXT", "UP_TO_LAST"):
            col.prop(self, "reverse")
        n = len(region_picks(context.scene, self.sketch))
        row = col.row()
        row.enabled = n > 0
        row.prop(self, "use_regions", text=f"Picked Regions Only ({n})" if n else "Picked Regions Only (none)")

    def _feature(self, context, op, sketch):
        feat = {"op": op, "sketch": sketch.name, "depth": self.depth, "depth2": self.depth2, "extent": self.extent,
                "symmetric": self.extent == "MID_PLANE", "reverse": self.reverse}
        if self.extent == "UP_TO_CURSOR":
            feat["probe"] = list(context.scene.cursor.location)
        picks = region_picks(context.scene, sketch.name)
        if self.use_regions and picks:
            feat["regions"] = picks
        return feat


class View3D_OT_slvs_occt_pad(Operator, _ExtrudeProps):
    """OCCT Pad: extrude the active sketch (or its picked regions) into an exact B-rep solid,
    as a new part or added to an existing one"""

    bl_idname = Operators.OCCTPad
    bl_label = "OCCT Pad"
    bl_options = {"REGISTER", "UNDO"}

    part: EnumProperty(name="Part", items=_pad_parts_enum)

    @classmethod
    def poll(cls, context):
        return poll_outside_sketch(cls, context)

    def invoke(self, context, event):
        self.sketch = sketch_items(self, context)[0][0]
        return context.window_manager.invoke_props_dialog(self, width=300)

    def draw(self, context):
        col = self.layout.column()
        col.prop(self, "sketch")
        col.prop(self, "part")
        self._draw_extent(context, col)

    def execute(self, context: Context):
        if zero_depth(self):
            self.report({"ERROR"}, "Depth is zero: nothing to extrude")
            return {"CANCELLED"}
        sketch = sketch_by_name(context, self.sketch)
        if sketch is None:
            self.report({"ERROR"}, "Choose a sketch")
            return {"CANCELLED"}
        sketch = _ensure_unique_name(context, sketch)
        recipes = get_recipes(context.scene)
        if self.part in ("__NEW__", "") or self.part not in recipes:
            part_name, i = f"OCCT_{sketch.name}", 1
            while part_name in recipes:
                part_name = f"OCCT_{sketch.name}.{i:03d}"
                i += 1
        else:
            part_name = self.part
        recipe = recipes.setdefault(part_name, [])
        recipe.append(self._feature(context, "pad", sketch))
        set_recipes(context.scene, recipes)
        obj = recompute_part(context, part_name, report=self.report)
        if obj is None:
            recipe.pop()
            if not recipe:
                del recipes[part_name]
            set_recipes(context.scene, recipes)
            self.report({"ERROR"}, "Recompute failed (closed contour?)")
            return {"CANCELLED"}
        hide_used_sketch(context, sketch)
        self.report({"INFO"}, f"Pad -> {part_name}")
        return {"FINISHED"}


class View3D_OT_slvs_occt_pocket(Operator, _ExtrudeProps):
    """OCCT Pocket: cut the active sketch profile (or its picked regions) out of a part"""

    bl_idname = Operators.OCCTPocket
    bl_label = "OCCT Pocket"
    bl_options = {"REGISTER", "UNDO"}

    part: EnumProperty(name="Part", items=_parts_enum)

    @classmethod
    def poll(cls, context):
        return poll_outside_sketch(cls, context)

    def invoke(self, context, event):
        self.sketch = sketch_items(self, context)[0][0]
        if not self.properties.is_property_set("extent"):
            self.extent = "THROUGH_ALL_BOTH"
        ob = context.active_object
        if ob is not None and ob.name in get_recipes(context.scene):
            self.part = ob.name
        return context.window_manager.invoke_props_dialog(self, width=300)

    def draw(self, context):
        col = self.layout.column()
        col.prop(self, "sketch")
        col.prop(self, "part")
        self._draw_extent(context, col)

    def execute(self, context: Context):
        if self.part == "NONE":
            self.report({"ERROR"}, "No part (make an OCCT Pad first)")
            return {"CANCELLED"}
        if zero_depth(self):
            self.report({"ERROR"}, "Depth is zero: nothing to extrude")
            return {"CANCELLED"}
        sketch = sketch_by_name(context, self.sketch)
        if sketch is None:
            self.report({"ERROR"}, "Choose a sketch")
            return {"CANCELLED"}
        sketch = _ensure_unique_name(context, sketch)
        recipes = get_recipes(context.scene)
        recipes[self.part].append(self._feature(context, "pocket", sketch))
        set_recipes(context.scene, recipes)
        recompute_part(context, self.part, report=self.report)
        hide_used_sketch(context, sketch)
        self.report({"INFO"}, f"Pocket -> {self.part}")
        return {"FINISHED"}


class View3D_OT_slvs_occt_fillet(Operator):
    """OCCT Fillet/Chamfer with edge selectors.
    'Nearest to 3D Cursor' targets one single edge: snap the 3D cursor
    near the edge you want, then run this"""

    bl_idname = Operators.OCCTFillet
    bl_label = "OCCT Fillet / Chamfer"
    bl_options = {"REGISTER", "UNDO"}

    part: EnumProperty(name="Part", items=_parts_enum)
    select: EnumProperty(
        name="Edges",
        items=[("TOP", "Top Rim", "Edges of the top face"),
               ("BOTTOM", "Bottom Rim", "Edges of the bottom face"),
               ("VERTICAL", "Vertical", "Edges along the extrusion"),
               ("CURSOR", "Nearest to 3D Cursor", "Single edge closest "
                "to the 3D cursor"),
               ("PICKED", "Picked Edges", "Edges selected with the "
                "Pick Edges tool"),
               ("ALL", "All Edges", "Every edge")],
        default="TOP")
    radius: FloatProperty(name="Radius", default=0.02, min=0.0001,
                          soft_max=1.0, unit="LENGTH")
    chamfer: BoolProperty(name="Chamfer", default=False)

    @classmethod
    def poll(cls, context):
        return context.mode == "OBJECT"

    def invoke(self, context, event):
        if not self.properties.is_property_set("select"):
            picks = json.loads(context.scene.get(PICKS_KEY, "[]"))
            if picks:
                self.select = "PICKED"
        return context.window_manager.invoke_props_dialog(self, width=280)

    def execute(self, context: Context):
        if self.part == "NONE":
            self.report({"ERROR"}, "No part")
            return {"CANCELLED"}
        feat = {"op": "chamfer" if self.chamfer else "fillet",
                "select": self.select, "radius": self.radius}
        if self.select == "CURSOR":
            feat["probe"] = list(context.scene.cursor.location)
        elif self.select == "PICKED":
            picks = json.loads(context.scene.get(PICKS_KEY, "[]"))
            if not picks:
                self.report({"ERROR"},
                            "No picked edges (use Pick Edges first)")
                return {"CANCELLED"}
            feat["probes"] = picks
        recipes = get_recipes(context.scene)
        recipes[self.part].append(feat)
        set_recipes(context.scene, recipes)
        recompute_part(context, self.part, report=self.report)
        self.report({"INFO"}, f"{feat['op']} -> {self.part}")
        return {"FINISHED"}


def _renumber_patterns(recipe, removed_index):
    """Apres suppression d'une fonction, les patterns pointent toujours la bonne source."""
    for f in recipe:
        if f.get("op") != "pattern" or f.get("feature") is None:
            continue
        if f["feature"] == removed_index:
            f["feature"] = None  # source supprimee : le pattern ne fait plus rien
        elif f["feature"] > removed_index:
            f["feature"] -= 1


def _renumber_derived(recipes, part, removed_index):
    for r in recipes.values():
        for f in r:
            if f.get("op") == "derive" and f.get("from") == part:
                f["suppress"] = [i - 1 if i > removed_index else i
                                 for i in f.get("suppress", []) if i != removed_index]


def _repeatable_features(self, context):
    recipe = get_recipes(context.scene).get(getattr(self, "part", ""), [])
    items = [(str(i), f"{i + 1}. {f['op'].capitalize()} [{f.get('sketch', '?')}]", "")
             for i, f in enumerate(recipe) if f.get("op") in ("pad", "pocket")]
    _repeatable_features.cache = items or [("-1", "<no pad / pocket>", "")]
    return _repeatable_features.cache


class View3D_OT_slvs_occt_pattern(Operator):
    """OCCT Pattern: repeat a pad or a pocket of the part, along a direction or around an axis
    (serrations, ratchet teeth, holes). Stays parametric: edit the sketch and every copy follows"""

    bl_idname = Operators.OCCTPattern
    bl_label = "OCCT Pattern"
    bl_options = {"REGISTER", "UNDO"}

    part: EnumProperty(name="Part", items=_parts_enum)
    feature: EnumProperty(name="Feature", items=_repeatable_features,
                          description="Pad or pocket to repeat (default: the last one)")
    kind: EnumProperty(name="Type", items=[("linear", "Linear", "Along a direction"),
                                           ("circular", "Circular", "Around an axis")], default="linear")
    direction: EnumProperty(name="Direction", items=PATTERN_DIRECTIONS, default="SKETCH_X")
    count: IntProperty(name="Count", default=10, min=2, max=1000,
                       description="Total number of instances, the original included")
    spacing: FloatProperty(name="Spacing", default=0.00055, soft_min=-1.0, soft_max=1.0,
                           unit="LENGTH", precision=5, description="Distance between two instances (negative: other way)")
    axis: EnumProperty(name="Axis", items=PATTERN_DIRECTIONS, default="NORMAL")
    angle: FloatProperty(name="Total Angle", default=360.0, min=-360.0, max=360.0,
                         description="Degrees covered by the instances (360: evenly around)")
    center: EnumProperty(name="Center", items=[("SKETCH", "Sketch Origin", ""),
                                               ("CURSOR", "3D Cursor", "")], default="SKETCH")

    @classmethod
    def poll(cls, context):
        return context.mode == "OBJECT" and bool(get_recipes(context.scene))

    def invoke(self, context, event):
        ob = context.active_object
        recipes = get_recipes(context.scene)
        if ob is not None and ob.name in recipes:
            self.part = ob.name
        items = [i for i in _repeatable_features(self, context) if i[0] != "-1"]
        if items:
            self.feature = items[-1][0]
        return context.window_manager.invoke_props_dialog(self, width=300)

    def draw(self, context):
        col = self.layout.column()
        col.prop(self, "part")
        col.prop(self, "feature")
        col.prop(self, "kind", expand=True)
        col.prop(self, "count")
        if self.kind == "linear":
            col.prop(self, "direction")
            col.prop(self, "spacing")
        else:
            col.prop(self, "axis")
            col.prop(self, "center")
            col.prop(self, "angle")

    def execute(self, context: Context):
        recipes = get_recipes(context.scene)
        if self.part not in recipes or self.feature == "-1":
            self.report({"ERROR"}, "Nothing to repeat (make an OCCT Pad or Pocket first)")
            return {"CANCELLED"}
        feat = {"op": "pattern", "feature": int(self.feature), "kind": self.kind, "count": self.count}
        if self.kind == "linear":
            feat.update(direction=self.direction, spacing=self.spacing)
        else:
            feat.update(axis=self.axis, angle=self.angle)
            if self.center == "CURSOR":
                feat["center"] = list(context.scene.cursor.location)
        recipes[self.part].append(feat)
        set_recipes(context.scene, recipes)
        obj = recompute_part(context, self.part, report=self.report)
        if obj is None:
            recipes[self.part].pop()
            set_recipes(context.scene, recipes)
            self.report({"ERROR"}, "Recompute failed")
            return {"CANCELLED"}
        self.report({"INFO"}, f"Pattern x{self.count} -> {self.part}")
        return {"FINISHED"}


class View3D_OT_slvs_occt_derive(Operator):
    """OCCT Derive Part: start a new part from another part's solid, optionally mirrored, with some of
    its features switched off (e.g. the second branch of a forceps). Stays linked: edit the source
    and the derived part follows"""

    bl_idname = Operators.OCCTDerive
    bl_label = "OCCT Derive Part"
    bl_options = {"REGISTER", "UNDO"}

    source: EnumProperty(name="Source Part", items=_parts_enum)
    name: bpy.props.StringProperty(name="New Part", default="")
    mirror: EnumProperty(name="Mirror", items=[("NONE", "None", "")] + MIRROR_PLANES, default="XZ")
    center: EnumProperty(name="Mirror Through", items=[("ORIGIN", "World Origin (pivot)", ""),
                                                       ("CURSOR", "3D Cursor", "")], default="ORIGIN")

    @classmethod
    def poll(cls, context):
        return context.mode == "OBJECT" and bool(get_recipes(context.scene))

    def invoke(self, context, event):
        ob = context.active_object
        if ob is not None and ob.name in get_recipes(context.scene):
            self.source = ob.name
        return context.window_manager.invoke_props_dialog(self, width=300)

    def execute(self, context: Context):
        recipes = get_recipes(context.scene)
        if self.source not in recipes:
            self.report({"ERROR"}, "No source part")
            return {"CANCELLED"}
        name = self.name.strip() or f"{self.source}.B"
        base, i = name, 1
        while name in recipes or (name in context.scene.objects and name not in recipes):
            name = f"{base}.{i:03d}"
            i += 1
        feat = {"op": "derive", "from": self.source, "suppress": [], "mirror": self.mirror}
        if self.mirror != "NONE" and self.center == "CURSOR":
            feat["center"] = list(context.scene.cursor.location)
        recipes[name] = [feat]
        set_recipes(context.scene, recipes)
        obj = recompute_part(context, name, report=self.report)
        if obj is None:
            del recipes[name]
            set_recipes(context.scene, recipes)
            self.report({"ERROR"}, "Recompute failed")
            return {"CANCELLED"}
        self.report({"INFO"}, f"{name} derived from {self.source}")
        return {"FINISHED"}


class View3D_OT_slvs_occt_mirror(Operator):
    """OCCT Mirror: mirror the part's solid across a plane, fused with the original (symmetric part)
    or alone"""

    bl_idname = Operators.OCCTMirror
    bl_label = "OCCT Mirror"
    bl_options = {"REGISTER", "UNDO"}

    part: EnumProperty(name="Part", items=_parts_enum)
    plane: EnumProperty(name="Plane", items=MIRROR_PLANES, default="XY")
    center: EnumProperty(name="Through", items=[("ORIGIN", "World Origin", ""),
                                                ("CURSOR", "3D Cursor", "")], default="ORIGIN")
    fuse: BoolProperty(name="Keep Original", default=True,
                       description="Fuse the mirrored copy with the original (otherwise keep only the copy)")

    @classmethod
    def poll(cls, context):
        return context.mode == "OBJECT" and bool(get_recipes(context.scene))

    def invoke(self, context, event):
        ob = context.active_object
        if ob is not None and ob.name in get_recipes(context.scene):
            self.part = ob.name
        return context.window_manager.invoke_props_dialog(self, width=280)

    def execute(self, context: Context):
        recipes = get_recipes(context.scene)
        if self.part not in recipes:
            return {"CANCELLED"}
        feat = {"op": "mirror", "plane": self.plane, "fuse": self.fuse}
        if self.center == "CURSOR":
            feat["center"] = list(context.scene.cursor.location)
        recipes[self.part].append(feat)
        set_recipes(context.scene, recipes)
        recompute_part(context, self.part, report=self.report)
        self.report({"INFO"}, f"Mirror {self.plane} -> {self.part}")
        return {"FINISHED"}


class View3D_OT_slvs_occt_toggle_feature(Operator):
    """Switch this feature off or on (suppress); for a derived part, switch off a feature of the source"""

    bl_idname = Operators.OCCTToggleFeature
    bl_label = "Suppress / Unsuppress Feature"
    bl_options = {"REGISTER", "UNDO"}

    part: bpy.props.StringProperty()
    index: IntProperty(default=-1)
    source_index: IntProperty(default=-1, description="Index in the source part (derived parts only)")

    def execute(self, context: Context):
        recipes = get_recipes(context.scene)
        recipe = recipes.get(self.part)
        if recipe is None or not (0 <= self.index < len(recipe)):
            return {"CANCELLED"}
        feat = recipe[self.index]
        if self.source_index >= 0:
            sup = set(feat.get("suppress", []))
            sup ^= {self.source_index}
            feat["suppress"] = sorted(sup)
        else:
            feat["suppressed"] = not feat.get("suppressed", False)
        set_recipes(context.scene, recipes)
        recompute_part(context, self.part, report=self.report)
        return {"FINISHED"}


class View3D_OT_slvs_occt_edit_feature(Operator):
    """Edit this feature's settings (extent, depth, radius, count...) and recompute the part"""

    bl_idname = Operators.OCCTEditFeature
    bl_label = "Edit Feature"
    bl_options = {"REGISTER", "UNDO"}

    part: bpy.props.StringProperty(options={"HIDDEN"})
    index: IntProperty(default=-1, options={"HIDDEN"})
    # pad / pocket
    extent: EnumProperty(name="Extent", items=EXTENTS, default="BLIND")
    depth: FloatProperty(name="Depth", default=0.003, soft_min=-10.0, soft_max=10.0, unit="LENGTH",
                         description="Negative: extrude on the other side of the sketch")
    depth2: FloatProperty(name="Depth 2", default=0.003, soft_min=-10.0, soft_max=10.0, unit="LENGTH",
                          description="Depth on the other side of the sketch (Two Sides)")
    reverse: BoolProperty(name="Reverse", default=False)
    keep_regions: BoolProperty(name="Picked Regions Only", default=True,
                               description="Keep the regions stored in the feature (otherwise the whole sketch)")
    update_regions: BoolProperty(name="Use Current Region Picks", default=False,
                                 description="Replace the stored regions by the ones picked now with OCCT Pick Regions")
    # fillet / chamfer
    radius: FloatProperty(name="Radius", default=0.0005, min=0.00001, soft_max=1.0, unit="LENGTH")
    chamfer: BoolProperty(name="Chamfer", default=False)
    # pattern
    kind: EnumProperty(name="Type", items=[("linear", "Linear", ""), ("circular", "Circular", "")])
    count: IntProperty(name="Count", default=2, min=2, max=1000)
    spacing: FloatProperty(name="Spacing", default=0.001, unit="LENGTH", precision=5)
    direction: EnumProperty(name="Direction", items=PATTERN_DIRECTIONS)
    axis: EnumProperty(name="Axis", items=PATTERN_DIRECTIONS, default="NORMAL")
    angle: FloatProperty(name="Total Angle", default=360.0, min=-360.0, max=360.0)
    # mirror / derive
    plane: EnumProperty(name="Plane", items=MIRROR_PLANES, default="XY")
    mirror: EnumProperty(name="Mirror", items=[("NONE", "None", "")] + MIRROR_PLANES, default="NONE")
    fuse: BoolProperty(name="Keep Original", default=True)

    def _feat(self, context):
        recipe = get_recipes(context.scene).get(self.part)
        if recipe is None or not (0 <= self.index < len(recipe)):
            return None
        return recipe[self.index]

    def invoke(self, context, event):
        f = self._feat(context)
        if f is None:
            return {"CANCELLED"}
        op = f.get("op")
        if op in ("pad", "pocket"):
            self.extent = feature_extent(f)
            self.depth = f.get("depth", 0.003)
            self.depth2 = f.get("depth2", 0.003)
            self.reverse = f.get("reverse", False)
            self.keep_regions = bool(f.get("regions"))
            self.update_regions = False
        elif op in ("fillet", "chamfer"):
            self.radius = f.get("radius", 0.0005)
            self.chamfer = op == "chamfer"
        elif op == "pattern":
            self.kind = f.get("kind", "linear")
            self.count = int(f.get("count", 2))
            self.spacing = f.get("spacing", 0.001)
            self.direction = f.get("direction", "SKETCH_X")
            self.axis = f.get("axis", "NORMAL")
            self.angle = f.get("angle", 360.0)
        elif op == "mirror":
            self.plane = f.get("plane", "XY")
            self.fuse = f.get("fuse", True)
        elif op == "derive":
            self.mirror = f.get("mirror", "NONE")
        return context.window_manager.invoke_props_dialog(self, width=300, title=f"Edit {op.capitalize()}")

    def draw(self, context):
        f = self._feat(context)
        if f is None:
            return
        op = f.get("op")
        col = self.layout.column()
        if op in ("pad", "pocket"):
            col.label(text=f"Sketch: {f.get('sketch', '?')}", icon="GREASEPENCIL")
            col.prop(self, "extent")
            _draw_depths(self, col)
            if self.extent in ("BLIND", "TWO_SIDES", "THROUGH_ALL", "UP_TO_NEXT", "UP_TO_LAST"):
                col.prop(self, "reverse")
            if f.get("regions"):
                col.prop(self, "keep_regions", text=f"Picked Regions Only ({len(f['regions'])})")
            n = len(region_picks(context.scene, f.get("sketch", "")))
            if n:
                col.prop(self, "update_regions", text=f"Use Current Region Picks ({n})")
            if self.extent == "UP_TO_CURSOR":
                col.label(text="Depth: up to the 3D cursor (its current position)", icon="INFO")
        elif op in ("fillet", "chamfer"):
            col.prop(self, "chamfer")
            col.prop(self, "radius", text="Length" if self.chamfer else "Radius")
        elif op == "pattern":
            col.prop(self, "kind", expand=True)
            col.prop(self, "count")
            if self.kind == "linear":
                col.prop(self, "direction")
                col.prop(self, "spacing")
            else:
                col.prop(self, "axis")
                col.prop(self, "angle")
        elif op == "mirror":
            col.prop(self, "plane")
            col.prop(self, "fuse")
        elif op == "derive":
            col.label(text=f"Source: {f.get('from', '?')}", icon="DUPLICATE")
            col.prop(self, "mirror")
            col.label(text="Switch source features on / off in the Part Tree", icon="INFO")

    def execute(self, context: Context):
        recipes = get_recipes(context.scene)
        recipe = recipes.get(self.part)
        if recipe is None or not (0 <= self.index < len(recipe)):
            return {"CANCELLED"}
        f = recipe[self.index]
        op = f.get("op")
        if op in ("pad", "pocket"):
            if zero_depth(self):
                self.report({"ERROR"}, "Depth is zero: nothing to extrude")
                return {"CANCELLED"}
            f.update(extent=self.extent, depth=self.depth, depth2=self.depth2, reverse=self.reverse,
                     symmetric=self.extent == "MID_PLANE")
            if self.extent == "UP_TO_CURSOR":
                f["probe"] = list(context.scene.cursor.location)
            picks = region_picks(context.scene, f.get("sketch", ""))
            if self.update_regions and picks:
                f["regions"] = picks
            elif not self.keep_regions:
                f.pop("regions", None)
        elif op in ("fillet", "chamfer"):
            f["op"] = "chamfer" if self.chamfer else "fillet"
            f["radius"] = self.radius
        elif op == "pattern":
            f.update(kind=self.kind, count=self.count)
            if self.kind == "linear":
                f.update(direction=self.direction, spacing=self.spacing)
            else:
                f.update(axis=self.axis, angle=self.angle)
        elif op == "mirror":
            f.update(plane=self.plane, fuse=self.fuse)
        elif op == "derive":
            f["mirror"] = self.mirror
        set_recipes(context.scene, recipes)
        recompute_part(context, self.part, report=self.report)
        self.report({"INFO"}, f"{op.capitalize()} {self.index + 1} updated")
        return {"FINISHED"}


class View3D_OT_slvs_occt_remove_last(Operator):
    """Remove the last feature of a part and recompute"""

    bl_idname = Operators.OCCTRemoveLast
    bl_label = "OCCT Remove Last Feature"
    bl_options = {"REGISTER", "UNDO"}

    part: EnumProperty(name="Part", items=_parts_enum)

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self, width=240)

    def execute(self, context: Context):
        recipes = get_recipes(context.scene)
        recipe = recipes.get(self.part)
        if not recipe:
            return {"CANCELLED"}
        removed = recipe.pop()
        if not recipe:
            del recipes[self.part]
        set_recipes(context.scene, recipes)
        if recipes.get(self.part):
            recompute_part(context, self.part, report=self.report)
        self.report({"INFO"}, f"Removed {removed['op']}")
        return {"FINISHED"}


class View3D_OT_slvs_occt_remove_feature(Operator):
    """Remove this feature from the part and recompute"""

    bl_idname = Operators.OCCTRemoveFeature
    bl_label = "Remove Feature"
    bl_options = {"REGISTER", "UNDO"}

    part: bpy.props.StringProperty()
    index: IntProperty(default=-1)

    def execute(self, context: Context):
        recipes = get_recipes(context.scene)
        recipe = recipes.get(self.part)
        if recipe is None or not (0 <= self.index < len(recipe)):
            return {"CANCELLED"}
        removed = recipe.pop(self.index)
        _renumber_patterns(recipe, self.index)
        _renumber_derived(recipes, self.part, self.index)
        if not recipe:
            del recipes[self.part]
            obj = context.scene.objects.get(self.part)
            if obj:
                bpy.data.objects.remove(obj, do_unlink=True)
        set_recipes(context.scene, recipes)
        if recipes.get(self.part):
            recompute_part(context, self.part, report=self.report)
        self.report({"INFO"}, f"Removed {removed['op']}")
        return {"FINISHED"}


class View3D_OT_slvs_occt_recompute(Operator):
    """Rebuild all OCCT parts from their recipes"""

    bl_idname = Operators.OCCTRecompute
    bl_label = "Recompute All"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context: Context):
        for name in list(get_recipes(context.scene).keys()):
            recompute_part(context, name, report=self.report)
        return {"FINISHED"}


class View3D_OT_slvs_occt_tidy_view(Operator):
    """Hide the sketches, workplanes and sketch objects that OCCT parts have used,
    hide workplanes with no sketch, and delete edge outlines left by deleted parts"""

    bl_idname = Operators.OCCTTidyView
    bl_label = "Tidy View"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context: Context):
        hidden, removed = tidy_view(context)
        msg = f"{hidden} item(s) hidden" + (f", {removed} orphan outline(s) deleted" if removed else "")
        self.report({"INFO"}, msg)
        context.area and context.area.tag_redraw()
        return {"FINISHED"}


class View3D_OT_slvs_occt_pick_edges(Operator):
    """CAD selection mode: hover the exact B-rep edges of the active
    part, click to select (several allowed), Enter to confirm.
    Then use OCCT Fillet with 'Picked Edges'"""

    bl_idname = Operators.OCCTPickEdges
    bl_label = "OCCT Pick Edges"

    _handle = None

    @staticmethod
    def _part(context):
        """The OCCT part to pick on: the active part, the part of the active _Quad / _Edges
        object, or the only part of the scene."""
        recipes = get_recipes(context.scene)
        ob = context.active_object
        if ob is not None:
            for name in (ob.name, ob.get("occt_quad_of"), ob.parent.name if ob.parent else None):
                if name and name in recipes:
                    return name
        return next(iter(recipes)) if len(recipes) == 1 else None

    @classmethod
    def poll(cls, context):
        if context.mode != "OBJECT":
            cls.poll_message_set("Leave Edit / Sketch mode first")
            return False
        if cls._part(context) is None:
            cls.poll_message_set("Select the OCCT part (or its _Quad mesh)")
            return False
        return True

    def invoke(self, context, event):
        self._area = context.area  # kept: context.area is None when the workspace changes
        part = self._part(context)
        ob = context.scene.objects.get(part)
        self.M = ob.matrix_world.copy() if ob is not None else None
        solid = _solid_cache.get(part)
        if solid is None:
            recompute_part(context, part)
            solid = _solid_cache.get(part)
        if solid is None:
            self.report({"ERROR"}, "Part has no B-rep (recompute failed)")
            return {"CANCELLED"}

        import gpu
        from gpu_extras.batch import batch_for_shader

        self.part = part
        self.samples = []
        self.centers = []
        self.batches = []
        shader = gpu.shader.from_builtin("UNIFORM_COLOR")
        self.shader = shader
        try:
            self.smooth = occt_bridge.smooth_edge_flags(solid)
        except Exception:
            self.smooth = []
        for edge in solid.edges():
            pts = []
            gt = str(edge.geom_type).upper().split(".")[-1]
            n_s = 2 if gt == "LINE" else 64  # (BSPLINE contains "LINE": compare exactly)
            for i in range(n_s):
                t = i / (n_s - 1)
                v = edge @ t
                p = Vector((v.X, v.Y, v.Z))
                if self.M is not None:
                    p = self.M @ p
                pts.append(tuple(p))
            self.samples.append(pts)
            c = edge.center()
            self.centers.append((c.X, c.Y, c.Z))
            self.batches.append(
                batch_for_shader(shader, "LINE_STRIP", {"pos": pts}))

        self.hover = -1
        self.picked = set()
        context.scene[PICKS_KEY] = "[]"  # a new pick replaces the previous one
        self._solid_ref = solid
        self._handle = bpy.types.SpaceView3D.draw_handler_add(
            self._draw, (context,), "WINDOW", "POST_VIEW")
        context.window_manager.modal_handler_add(self)
        self._area.header_text_set(
            "Pick Edges: click an edge to select / unselect | Enter: fillet the picked edges | Esc: cancel")
        return {"RUNNING_MODAL"}

    def _finish(self, context):
        if self._handle:
            bpy.types.SpaceView3D.draw_handler_remove(self._handle, "WINDOW")
            self._handle = None
        self._area.header_text_set(None)
        self._area.tag_redraw()

    def modal(self, context, event):
        self._area.tag_redraw()

        # la piece a ete recalculee sous nos pieds -> sortir proprement
        if _solid_cache.get(self.part) is not self._solid_ref:
            self._finish(context)
            self.report({"INFO"}, "Pick Edges closed (part was rebuilt)")
            return {"CANCELLED"}

        if event.type in {"MIDDLEMOUSE", "WHEELUPMOUSE", "WHEELDOWNMOUSE"}:
            return {"PASS_THROUGH"}

        if event.type == "MOUSEMOVE":
            from bpy_extras.view3d_utils import location_3d_to_region_2d
            from mathutils import Vector
            region = context.region
            rv3d = context.region_data
            if region is None or rv3d is None:
                # souris hors de la vue 3D (panneaux, sidebar...)
                self.hover = -1
                return {"PASS_THROUGH"}
            mouse = Vector((event.mouse_region_x, event.mouse_region_y))
            best, best_d = -1, 15.0
            for i, pts in enumerate(self.samples):
                prev = None
                for co in pts:
                    p2 = location_3d_to_region_2d(region, rv3d, co)
                    if p2 is not None and prev is not None:
                        ab = p2 - prev
                        L2 = ab.length_squared
                        t = 0.0 if L2 == 0 else max(0.0, min(1.0, (mouse - prev).dot(ab) / L2))
                        d = (prev + ab * t - mouse).length
                        if d < best_d:
                            best, best_d = i, d
                    prev = p2
            self.hover = best
            return {"PASS_THROUGH"}

        if event.type == "LEFTMOUSE" and event.value == "PRESS":
            if context.region_data is None or self.hover < 0:
                return {"PASS_THROUGH"}
            if self.hover in self.picked:
                self.picked.discard(self.hover)
            else:
                self.picked.add(self.hover)
            # saved at once: OCCT Fillet sees the picks even if opened without Enter
            context.scene[PICKS_KEY] = json.dumps([list(self.centers[i]) for i in sorted(self.picked)])
            self._area.header_text_set(
                f"Pick Edges: {len(self.picked)} edge(s) picked | click to add / remove | "
                "Enter: fillet them | Esc: done")
            return {"RUNNING_MODAL"}

        if event.type in {"RET", "NUMPAD_ENTER"} and event.value == "PRESS":
            picks = [list(self.centers[i]) for i in sorted(self.picked)]
            if picks:
                context.scene[PICKS_KEY] = json.dumps(picks)
            self._finish(context)
            if picks:
                # enchainement direct : dialogue Fillet pre-regle
                bpy.ops.view3d.slvs_occt_fillet(
                    "INVOKE_DEFAULT", part=self.part, select="PICKED")
            else:
                self.report({"INFO"}, "No edge picked")
            return {"FINISHED"}

        if event.type in {"ESC", "RIGHTMOUSE"} and event.value == "PRESS":
            self._finish(context)
            if self.picked:
                self.report({"INFO"}, f"{len(self.picked)} edge(s) picked: OCCT Fillet uses them (Edges: Picked Edges)")
            return {"FINISHED"}

        return {"PASS_THROUGH"}

    def _draw(self, context):
        import gpu
        shader = self.shader
        gpu.state.blend_set("ALPHA")
        gpu.state.depth_test_set("LESS_EQUAL")
        gpu.state.depth_mask_set(False)
        for i, batch in enumerate(self.batches):
            if i in self.picked:
                gpu.state.line_width_set(4.0)
                color = (0.1, 0.9, 0.2, 1.0)
            elif i == self.hover:
                gpu.state.line_width_set(4.0)
                color = (1.0, 0.85, 0.1, 1.0)
            elif i < len(self.smooth) and self.smooth[i]:
                gpu.state.line_width_set(1.0)
                color = (0.6, 0.75, 1.0, 0.18)  # tangent seam: faint, still pickable
            else:
                gpu.state.line_width_set(1.5)
                color = (0.9, 0.9, 0.9, 0.55)
            shader.bind()
            shader.uniform_float("color", color)
            batch.draw(shader)
        gpu.state.line_width_set(1.0)
        gpu.state.blend_set("NONE")


classes = (View3D_OT_slvs_occt_pad, View3D_OT_slvs_occt_pocket,
           View3D_OT_slvs_occt_fillet, View3D_OT_slvs_occt_pattern,
           View3D_OT_slvs_occt_derive, View3D_OT_slvs_occt_mirror,
           View3D_OT_slvs_occt_toggle_feature, View3D_OT_slvs_occt_edit_feature, View3D_OT_slvs_occt_remove_last,
           View3D_OT_slvs_occt_remove_feature, View3D_OT_slvs_occt_recompute,
           View3D_OT_slvs_occt_tidy_view, View3D_OT_slvs_occt_pick_edges)
_register, _unregister = register_classes_factory(classes)


def register():
    _register()
    _wrap_converter()


def unregister():
    _unregister()
