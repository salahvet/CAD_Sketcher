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

def recompute_part(context, part_name, report=None):
    import build123d as b3d
    recipes = get_recipes(context.scene)
    recipe = recipes.get(part_name)
    if not recipe:
        return None

    solid = None
    base_origin = (0, 0, 0)
    normal = (0, 0, 1)

    for feat in recipe:
        op = feat["op"]
        if op in ("pad", "pocket"):
            sketch = find_sketch(context, feat["sketch"])
            if sketch is None:
                if report:
                    report({"WARNING"}, f"Sketch '{feat['sketch']}' introuvable")
                continue
            face = occt_bridge.sketch_to_face(context, sketch)
            if face is None:
                if report:
                    report({"WARNING"}, f"Contour non ferme : {feat['sketch']}")
                continue
            depth = feat["depth"]
            if feat.get("symmetric"):
                body = b3d.extrude(face, amount=depth / 2.0, both=True)
            else:
                body = b3d.extrude(face, amount=depth)
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
                if op == "fillet":
                    solid = b3d.fillet(edges, radius=feat["radius"])
                else:
                    solid = b3d.chamfer(edges, length=feat["radius"])
            except Exception as e:
                if report:
                    report({"WARNING"},
                           f"{op} impossible (rayon trop grand ?) : {e}")

    if solid is None:
        return None

    _solid_cache[part_name] = solid
    existing = context.scene.objects.get(part_name)
    obj = occt_bridge.shape_to_mesh_object(solid, part_name, existing=existing)
    obj["occt_part"] = True

    # objet des aretes exactes du B-rep
    edges_name = part_name + "_Edges"
    e_existing = context.scene.objects.get(edges_name)
    e_obj = occt_bridge.shape_edges_object(solid, edges_name, existing=e_existing)
    e_obj.parent = obj
    e_obj.hide_select = True
    return obj


def recompute_all(scene):
    context = bpy.context
    for part_name in list(get_recipes(scene).keys()):
        try:
            recompute_part(context, part_name)
        except Exception:
            continue


def recompute_for_sketch(scene, sketch_name):
    """Ne recalcule que les pieces qui referencent ce sketch."""
    context = bpy.context
    for part_name, recipe in get_recipes(scene).items():
        if any(f.get("sketch") == sketch_name for f in recipe):
            try:
                recompute_part(context, part_name)
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


def _active_sketch(context):
    s = context.scene.sketcher.active_sketch
    if s is not None and getattr(s, "is_sketch", False):
        return s
    return None


class View3D_OT_slvs_occt_pad(Operator):
    """OCCT Pad: extrude the active sketch into an exact B-rep solid.
    Run again on the same part to fuse additional pads"""

    bl_idname = Operators.OCCTPad
    bl_label = "OCCT Pad"
    bl_options = {"REGISTER", "UNDO"}

    depth: FloatProperty(name="Depth", default=0.1, min=0.0001,
                         soft_max=10.0, unit="LENGTH")
    symmetric: BoolProperty(name="Symmetric", default=False)

    @classmethod
    def poll(cls, context):
        return context.mode == "OBJECT" and _active_sketch(context) is not None

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self, width=260)

    def execute(self, context: Context):
        sketch = _ensure_unique_name(context, _active_sketch(context))
        part_name = f"OCCT_{sketch.name}"
        recipes = get_recipes(context.scene)
        recipe = recipes.setdefault(part_name, [])
        recipe.append({"op": "pad", "sketch": sketch.name,
                       "depth": self.depth,
                       "symmetric": self.symmetric})
        set_recipes(context.scene, recipes)
        obj = recompute_part(context, part_name, report=self.report)
        if obj is None:
            recipe.pop()
            set_recipes(context.scene, recipes)
            self.report({"ERROR"}, "Recompute failed (closed contour?)")
            return {"CANCELLED"}
        self.report({"INFO"}, f"Pad -> {part_name}")
        return {"FINISHED"}


class View3D_OT_slvs_occt_pocket(Operator):
    """OCCT Pocket: cut the active sketch profile out of a part"""

    bl_idname = Operators.OCCTPocket
    bl_label = "OCCT Pocket"
    bl_options = {"REGISTER", "UNDO"}

    part: EnumProperty(name="Part", items=_parts_enum)
    depth: FloatProperty(name="Depth", default=0.2, min=0.0001,
                         soft_max=10.0, unit="LENGTH")
    symmetric: BoolProperty(name="Symmetric", default=True)

    @classmethod
    def poll(cls, context):
        return context.mode == "OBJECT" and _active_sketch(context) is not None

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self, width=260)

    def execute(self, context: Context):
        if self.part == "NONE":
            self.report({"ERROR"}, "No part (make an OCCT Pad first)")
            return {"CANCELLED"}
        sketch = _ensure_unique_name(context, _active_sketch(context))
        recipes = get_recipes(context.scene)
        recipes[self.part].append({"op": "pocket", "sketch": sketch.name,
                                   "depth": self.depth,
                                   "symmetric": self.symmetric})
        set_recipes(context.scene, recipes)
        recompute_part(context, self.part, report=self.report)
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



class View3D_OT_slvs_occt_pick_edges(Operator):
    """CAD selection mode: hover the exact B-rep edges of the active
    part, click to select (several allowed), Enter to confirm.
    Then use OCCT Fillet with 'Picked Edges'"""

    bl_idname = Operators.OCCTPickEdges
    bl_label = "OCCT Pick Edges"

    _handle = None

    @classmethod
    def poll(cls, context):
        ob = context.active_object
        return (context.mode == "OBJECT" and ob is not None
                and ob.get("occt_part"))

    def invoke(self, context, event):
        part = context.active_object.name
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
        for edge in solid.edges():
            pts = []
            for i in range(16):
                t = i / 15.0
                v = edge @ t
                pts.append((v.X, v.Y, v.Z))
            self.samples.append(pts)
            c = edge.center()
            self.centers.append((c.X, c.Y, c.Z))
            self.batches.append(
                batch_for_shader(shader, "LINE_STRIP", {"pos": pts}))

        self.hover = -1
        self.picked = set()
        self._solid_ref = solid
        self._handle = bpy.types.SpaceView3D.draw_handler_add(
            self._draw, (context,), "WINDOW", "POST_VIEW")
        context.window_manager.modal_handler_add(self)
        context.area.header_text_set(
            "Pick Edges: clic = selectionner | Entree = valider | Echap = annuler")
        return {"RUNNING_MODAL"}

    def _finish(self, context):
        if self._handle:
            bpy.types.SpaceView3D.draw_handler_remove(self._handle, "WINDOW")
            self._handle = None
        context.area.header_text_set(None)
        context.area.tag_redraw()

    def modal(self, context, event):
        context.area.tag_redraw()

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
            best, best_d = -1, 25.0
            for i, pts in enumerate(self.samples):
                for co in pts[::3]:
                    p2 = location_3d_to_region_2d(region, rv3d, co)
                    if p2 is None:
                        continue
                    d = (p2 - mouse).length
                    if d < best_d:
                        best, best_d = i, d
            self.hover = best
            return {"PASS_THROUGH"}

        if event.type == "LEFTMOUSE" and event.value == "PRESS":
            if context.region_data is None or self.hover < 0:
                return {"PASS_THROUGH"}
            if self.hover in self.picked:
                self.picked.discard(self.hover)
            else:
                self.picked.add(self.hover)
            return {"RUNNING_MODAL"}

        if event.type in {"RET", "NUMPAD_ENTER"}:
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

        if event.type in {"ESC", "RIGHTMOUSE"}:
            self._finish(context)
            return {"CANCELLED"}

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
            else:
                gpu.state.line_width_set(1.5)
                color = (0.9, 0.9, 0.9, 0.55)
            shader.bind()
            shader.uniform_float("color", color)
            batch.draw(shader)
        gpu.state.line_width_set(1.0)
        gpu.state.blend_set("NONE")


classes = (View3D_OT_slvs_occt_pad, View3D_OT_slvs_occt_pocket,
           View3D_OT_slvs_occt_fillet, View3D_OT_slvs_occt_remove_last,
           View3D_OT_slvs_occt_remove_feature, View3D_OT_slvs_occt_recompute,
           View3D_OT_slvs_occt_pick_edges)
_register, _unregister = register_classes_factory(classes)


def register():
    _register()
    _wrap_converter()


def unregister():
    _unregister()
