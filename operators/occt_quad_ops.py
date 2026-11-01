"""Fork: OCCT Quad Mesh — turn an OCCT part into a clean all-quad mesh ready for Subdivision Surface.

The mesh is built face by face from the exact B-rep (see occt_quad.py). Sharp B-rep edges get
support loops (or creases), then the cage is fitted so that its subdivided surface lies on
the exact part: after each pass, the limit position of every cage vertex is compared with the
exact surface and the cage vertex is moved by the gap. The part stays editable; running the
tool again rebuilds the same quad object.
"""
import math

import bmesh
import bpy
from bpy.props import BoolProperty, EnumProperty, FloatProperty, IntProperty
from bpy.types import Context, Operator
from bpy.utils import register_classes_factory
from mathutils import Vector
from mathutils.bvhtree import BVHTree

from ..declarations import Operators
from .. import occt_quad, occt_quad2
from . import occt_ops

SHARP_MODES = [
    ("SUPPORT", "Support Loops", "Add an edge loop on each side of the sharp edges (works in every program)"),
    ("CREASE", "Crease", "Full edge crease on the sharp edges (no extra geometry; Blender and some exporters only)"),
    ("NONE", "None", "Everything is rounded by the subdivision"),
]


def _exact_bvh(solid, size):
    verts, tris = solid.tessellate(size * 1e-3, 0.05)
    return BVHTree.FromPolygons([tuple(v) for v in verts], [tuple(t) for t in tris], all_triangles=True)


def _eval_coords(obj, depsgraph):
    import numpy as np
    ev = obj.evaluated_get(depsgraph)
    me = ev.to_mesh()
    co = np.empty(len(me.vertices) * 3)
    me.vertices.foreach_get("co", co)
    ev.to_mesh_clear()
    return co.reshape(-1, 3)


def fit_to_surface(context, obj, bvh, iterations):
    """Move the cage so that the limit surface lies on the exact surface. Returns (max, mean) gap."""
    import numpy as np
    me = obj.data
    n = len(me.vertices)
    cage = np.empty(n * 3)
    me.vertices.foreach_get("co", cage)
    cage = cage.reshape(-1, 3)
    stats = (0.0, 0.0)
    for it in range(iterations + 1):
        context.view_layer.update()
        dg = context.evaluated_depsgraph_get()
        ev = _eval_coords(obj, dg)
        gaps = []
        for p in ev:
            hit = bvh.find_nearest(Vector(p))
            if hit[0] is not None:
                gaps.append(hit[3])
        stats = (max(gaps) if gaps else 0.0, sum(gaps) / len(gaps) if gaps else 0.0)
        if it == iterations:
            break
        limit = ev[:n]
        for i in range(n):
            hit = bvh.find_nearest(Vector(limit[i]))
            if hit[0] is not None:
                cage[i] += np.array(hit[0]) - limit[i]
        me.vertices.foreach_set("co", cage.ravel())
        me.update()
    return stats


GUIDES_KEY = "occt_quad_guides"


def drawn_guides(obj_part):
    """Guides drawn on the part with OCCT Draw Guides: [{"n": normal, "pts": [...]}] (local space)."""
    import json
    if obj_part is None:
        return []
    try:
        return json.loads(obj_part.get(GUIDES_KEY, "[]"))
    except ValueError:
        return []


def set_drawn_guides(obj_part, lines):
    import json
    obj_part[GUIDES_KEY] = json.dumps(lines)


def guides_for(context, obj_part, guide_sketch):
    """Guide polylines (drawn on the part, or from a sketch), in the part's local space."""
    if isinstance(guide_sketch, str) and guide_sketch == "DRAWN":
        lines = drawn_guides(obj_part)
        if not lines:
            return None
        return {"lines": [ln["pts"] for ln in lines], "normals": [ln["n"] for ln in lines],
                "normal": lines[0]["n"]}
    if guide_sketch is None:
        return None
    from .. import occt_bridge
    g = occt_bridge.sketch_guides(context, guide_sketch)
    if not g["lines"]:
        return None
    if obj_part is not None:
        Mi = obj_part.matrix_world.inverted()
        g["lines"] = [[tuple(Mi @ Vector(p)) for p in line] for line in g["lines"]]
        g["normal"] = tuple((Mi.to_3x3() @ Vector(g["normal"])).normalized())
    return g


def part_solid(context, part_name, report=None):
    solid = occt_ops._solid_cache.get(part_name)
    if solid is None:
        occt_ops.recompute_part(context, part_name, report=report)
        solid = occt_ops._solid_cache.get(part_name)
    if solid is None:
        raise RuntimeError("The part cannot be rebuilt")
    return solid


# ---- patch preview (viewport overlay) -------------------------------------------------------

_PREVIEW = {"handle": None, "batches": None, "part": None}

_COLORS = [(0.95, 0.55, 0.35), (0.40, 0.70, 0.95), (0.55, 0.85, 0.45), (0.90, 0.75, 0.30),
           (0.75, 0.50, 0.90), (0.35, 0.85, 0.80), (0.95, 0.45, 0.65), (0.65, 0.65, 0.65)]


def _draw_preview():
    import gpu
    b = _PREVIEW["batches"]
    if not b:
        return
    gpu.state.blend_set("ALPHA")
    gpu.state.depth_test_set("LESS_EQUAL")
    gpu.state.depth_mask_set(False)
    b["fill_shader"].bind()
    b["fill"].draw(b["fill_shader"])
    sh = b["line_shader"]
    sh.bind()
    sh.uniform_float("color", (0.05, 0.05, 0.05, 0.55))
    gpu.state.line_width_set(1.0)
    b["wire"].draw(sh)
    sh.uniform_float("color", (0.0, 0.0, 0.0, 1.0))
    gpu.state.line_width_set(3.0)
    b["border"].draw(sh)
    gpu.state.line_width_set(1.0)
    gpu.state.depth_mask_set(True)
    gpu.state.blend_set("NONE")


def clear_preview():
    if _PREVIEW["handle"] is not None:
        bpy.types.SpaceView3D.draw_handler_remove(_PREVIEW["handle"], "WINDOW")
    _PREVIEW.update(handle=None, batches=None, part=None)


def show_preview(context, part_name, verts, faces, patch_of):
    import gpu
    from gpu_extras.batch import batch_for_shader
    ob = context.scene.objects.get(part_name)
    M = ob.matrix_world if ob is not None else None
    W = [tuple(M @ Vector(v)) if M is not None else v for v in verts]
    # small offset along the face normal so the overlay sits on the part
    pos, col, wire, border = [], [], [], []
    edge_patches = {}
    for q, pi in zip(faces, patch_of):
        a, b, c, d = (Vector(W[k]) for k in q)
        n = (c - a).cross(d - b)
        n = n.normalized() * 2e-5 if n.length > 0 else Vector()
        rgb = _COLORS[pi % len(_COLORS)] if pi >= 0 else (0.9, 0.2, 0.2)
        for tri in ((a, b, c), (a, c, d)):
            for p in tri:
                pos.append(tuple(p + n))
                col.append((*rgb, 0.45))
        for k in range(4):
            e = (min(q[k], q[(k + 1) % 4]), max(q[k], q[(k + 1) % 4]))
            edge_patches.setdefault(e, set()).add(pi)
            wire += [tuple(Vector(W[q[k]]) + n), tuple(Vector(W[q[(k + 1) % 4]]) + n)]
    for (i, j), ps in edge_patches.items():
        if len(ps) > 1:
            border += [W[i], W[j]]
    fill_shader = gpu.shader.from_builtin("SMOOTH_COLOR")
    line_shader = gpu.shader.from_builtin("UNIFORM_COLOR")
    clear_preview()
    _PREVIEW["batches"] = {
        "fill": batch_for_shader(fill_shader, "TRIS", {"pos": pos, "color": col}),
        "wire": batch_for_shader(line_shader, "LINES", {"pos": wire}),
        "border": batch_for_shader(line_shader, "LINES", {"pos": border}),
        "fill_shader": fill_shader, "line_shader": line_shader}
    _PREVIEW["part"] = part_name
    _PREVIEW["handle"] = bpy.types.SpaceView3D.draw_handler_add(_draw_preview, (), "WINDOW", "POST_VIEW")
    for area in context.screen.areas if context.screen else []:
        if area.type == "VIEW_3D":
            area.tag_redraw()


def build_quad_object(context, part_name, tolerance, max_edge, max_angle, sharp_mode, support_width,
                      levels, fit, iterations, report=None, method="PATCHES", edge_length=0.0015,
                      guide_sketch=None):
    obj_part = context.scene.objects.get(part_name)
    solid = part_solid(context, part_name, report)

    if method == "UNIFORM":
        verts, faces, sharp, info = occt_quad.quad_mesh(solid, edge_length, 15.0, occt_quad.blender_cdt)
    else:
        guides = guides_for(context, obj_part, guide_sketch)
        verts, faces, sharp, info = occt_quad2.quad_mesh(solid, tolerance, max_edge, max_angle, 40.0,
                                                         occt_quad.blender_cdt, guides=guides)

    bm = bmesh.new()
    bv = [bm.verts.new(v) for v in verts]
    for q in faces:
        try:
            bm.faces.new([bv[k] for k in q])
        except ValueError:
            pass
    bm.verts.ensure_lookup_table()
    sharp_edges = []
    for a, b in sharp:
        e = bm.edges.get((bv[a], bv[b]))
        if e is not None:
            sharp_edges.append(e)
    for f in bm.faces:
        f.smooth = True
    if sharp_mode == "SUPPORT" and sharp_edges and support_width > 0:
        bmesh.ops.bevel(bm, geom=sharp_edges + list({v for e in sharp_edges for v in e.verts}),
                        offset=support_width, offset_type="OFFSET", segments=2, profile=1.0,
                        affect="EDGES", clamp_overlap=True, loop_slide=True,
                        vmesh_method="ADJ")
    crease_edges = sharp_edges if sharp_mode == "CREASE" else []
    for e in sharp_edges:
        if e.is_valid:
            e.smooth = False

    name = part_name + "_Quad"
    obj = context.scene.objects.get(name)
    me = bpy.data.meshes.new(name)
    if crease_edges:
        layer = bm.edges.layers.float.get("crease_edge") or bm.edges.layers.float.new("crease_edge")
        for e in crease_edges:
            if e.is_valid:
                e[layer] = 1.0
    bm.to_mesh(me)
    ngons = sum(1 for f in bm.faces if len(f.verts) != 4)
    bm.free()
    if obj is None:
        obj = bpy.data.objects.new(name, me)
        cols = obj_part.users_collection if obj_part else [context.collection]
        for c in cols:
            c.objects.link(obj)
    else:
        old = obj.data
        obj.data = me
        if old.users == 0:
            bpy.data.meshes.remove(old)
    if obj_part is not None:
        obj.matrix_world = obj_part.matrix_world.copy()
    obj["occt_quad_of"] = part_name
    mod = obj.modifiers.get("Subdivision") or obj.modifiers.new("Subdivision", "SUBSURF")
    mod.levels = levels
    mod.render_levels = levels
    mod.use_limit_surface = True
    mod.show_viewport = True  # the fit reads the subdivided surface

    stats = None
    if fit:
        size = max(solid.bounding_box().size.X, solid.bounding_box().size.Y, solid.bounding_box().size.Z)
        stats = fit_to_surface(context, obj, _exact_bvh(solid, size), iterations)
    info["ngons"] = ngons
    return obj, info, stats


def _guide_items(context):
    items = [("DRAWN", "Drawn on Part", "Guides drawn on the part with Draw Guides"),
             ("NONE", "None (automatic)", "")]
    items += [(sk.name, sk.name, "") for sk in context.scene.sketcher.entities.sketches]
    _guide_items.cache = items
    return items


def _parts(self, context):
    items = [(k, k, "") for k in occt_ops.get_recipes(context.scene).keys()] or [("NONE", "No part", "")]
    _parts.cache = items
    return items


class View3D_OT_slvs_occt_quad_mesh(Operator):
    """Make a clean all-quad copy of an OCCT part, ready for Subdivision Surface and fitted to the
    exact shape (the part itself stays editable)"""

    bl_idname = Operators.OCCTQuadMesh
    bl_label = "OCCT Quad Mesh"
    bl_options = {"REGISTER", "UNDO"}

    part: EnumProperty(name="Part", items=_parts)
    method: EnumProperty(name="Layout", default="PATCHES", items=[
        ("PATCHES", "Patches (light)", "Few large patches with continuous edge loops; density follows curvature"),
        ("UNIFORM", "Uniform (dense)", "Even density everywhere (previous method)"),
    ])
    guides: EnumProperty(name="Guides", items=lambda self, context: _guide_items(context),
                         description="Sketch whose lines, arcs and curves cut the flat faces into patches "
                                     "(draw them across the face, from edge to edge)")
    edge_length: FloatProperty(name="Edge Length", default=0.0015, min=1e-5, soft_max=0.02, unit="LENGTH",
                               description="Target edge length (Uniform layout)")
    tolerance: FloatProperty(name="Tolerance", default=0.0001, min=1e-6, soft_max=0.001, unit="LENGTH",
                             precision=4,
                             description="Largest gap allowed between a cage edge and the exact curve: "
                                         "segments are added only where the shape curves")
    max_edge: FloatProperty(name="Max Edge Length", default=0.0, min=0.0, soft_max=0.05, unit="LENGTH",
                            description="Longest cage edge (0 = no limit: flat areas stay as large quads)")
    max_angle: FloatProperty(name="Max Angle", default=45.0, min=5.0, max=90.0,
                             description="Largest angle covered by one segment on curved edges (degrees)")
    sharp_mode: EnumProperty(name="Sharp Edges", items=SHARP_MODES, default="SUPPORT")
    support_width: FloatProperty(name="Support Width", default=0.0001, min=0.0, soft_max=0.002, unit="LENGTH",
                                 description="Distance of the support loops from the sharp edges")
    levels: IntProperty(name="Subdivision Levels", default=2, min=0, max=4)
    fit: BoolProperty(name="Fit to Exact Shape", default=True,
                      description="Move the cage so that the subdivided surface lies on the exact part")
    iterations: IntProperty(name="Fit Passes", default=6, min=1, max=30)

    @classmethod
    def poll(cls, context):
        return bool(occt_ops.get_recipes(context.scene))

    def invoke(self, context, event):
        ob = context.active_object
        if ob is not None:
            name = ob.get("occt_quad_of", ob.name)
            if name in occt_ops.get_recipes(context.scene):
                self.part = name
        return context.window_manager.invoke_props_dialog(self, width=320)

    def draw(self, context):
        col = self.layout.column()
        col.prop(self, "part")
        col.prop(self, "method")
        if self.method == "UNIFORM":
            col.prop(self, "edge_length")
        else:
            col.prop(self, "guides")
            col.prop(self, "tolerance")
            col.prop(self, "max_edge")
            col.prop(self, "max_angle")
        col.prop(self, "sharp_mode")
        row = col.row()
        row.enabled = self.sharp_mode == "SUPPORT"
        row.prop(self, "support_width")
        col.prop(self, "levels")
        col.prop(self, "fit")
        row = col.row()
        row.enabled = self.fit
        row.prop(self, "iterations")

    def _guide_sketch(self, context):
        if self.method != "PATCHES" or self.guides in ("", "NONE"):
            return None
        if self.guides == "DRAWN":
            return "DRAWN"
        return next((sk for sk in context.scene.sketcher.entities.sketches if sk.name == self.guides), None)

    def execute(self, context: Context):
        if self.part == "NONE":
            return {"CANCELLED"}
        try:
            obj, info, stats = build_quad_object(
                context, self.part, self.tolerance, self.max_edge, self.max_angle, self.sharp_mode,
                self.support_width, self.levels if self.fit or self.levels else 0, self.fit,
                self.iterations, report=self.report, method=self.method, edge_length=self.edge_length,
                guide_sketch=self._guide_sketch(context))
        except Exception as e:
            self.report({"ERROR"}, f"Quad mesh failed: {e}")
            return {"CANCELLED"}
        msg = f"{obj.name}: {len(obj.data.polygons)} faces"
        for w in info.get("warnings", []):
            if "guide" in w:
                self.report({"WARNING"}, w)
        if info.get("free_patches"):
            msg += f", {info['free_patches']} patch(es) filled freely"
        if info.get("ngons"):
            msg += f" ({info['ngons']} not quads)"
        if stats:
            msg += f", gap to exact shape max {stats[0] * 1000:.3f} mm / mean {stats[1] * 1000:.4f} mm"
        self.report({"INFO"}, msg)
        return {"FINISHED"}


class View3D_OT_slvs_occt_patch_preview(Operator):
    """Show the quad patches of an OCCT part in the viewport (one colour per patch, thick lines
    between patches), using the chosen guide sketch. Run it again after editing the guides"""

    bl_idname = Operators.OCCTPatchPreview
    bl_label = "Preview Patches"
    bl_options = {"REGISTER"}

    part: EnumProperty(name="Part", items=_parts)
    guides: EnumProperty(name="Guides", items=lambda self, context: _guide_items(context))
    tolerance: FloatProperty(name="Tolerance", default=0.0001, min=1e-6, soft_max=0.001, unit="LENGTH", precision=4)
    max_edge: FloatProperty(name="Max Edge Length", default=0.0, min=0.0, soft_max=0.05, unit="LENGTH")
    max_angle: FloatProperty(name="Max Angle", default=45.0, min=5.0, max=90.0)
    clear: BoolProperty(name="Clear", default=False, options={"HIDDEN"})

    @classmethod
    def poll(cls, context):
        return bool(occt_ops.get_recipes(context.scene))

    def invoke(self, context, event):
        if self.clear:
            return self.execute(context)
        ob = context.active_object
        if ob is not None:
            name = ob.get("occt_quad_of", ob.name)
            if name in occt_ops.get_recipes(context.scene):
                self.part = name
        return context.window_manager.invoke_props_dialog(self, width=300)

    def execute(self, context: Context):
        if self.clear:
            clear_preview()
            for area in context.screen.areas:
                if area.type == "VIEW_3D":
                    area.tag_redraw()
            return {"FINISHED"}
        if self.part == "NONE":
            return {"CANCELLED"}
        try:
            solid = part_solid(context, self.part, self.report)
            sk = None
            if self.guides == "DRAWN":
                sk = "DRAWN"
            elif self.guides not in ("", "NONE"):
                sk = next((x for x in context.scene.sketcher.entities.sketches if x.name == self.guides), None)
            guides = guides_for(context, context.scene.objects.get(self.part), sk)
            verts, faces, sharp, info = occt_quad2.quad_mesh(solid, self.tolerance, self.max_edge, self.max_angle,
                                                             40.0, occt_quad.blender_cdt, guides=guides)
        except Exception as e:
            self.report({"ERROR"}, f"Preview failed: {e}")
            return {"CANCELLED"}
        show_preview(context, self.part, verts, faces, info["patch_of"])
        n = len(set(p for p in info["patch_of"] if p >= 0))
        msg = f"{n} patches, {len(faces)} quads before support loops"
        if info.get("free_patches"):
            msg += f"; {info['free_patches']} filled freely (red)"
        for w in info.get("warnings", []):
            if "guide" in w:
                self.report({"WARNING"}, w)
        self.report({"INFO"}, msg)
        return {"FINISHED"}


def unregister_preview():
    clear_preview()


register, _unregister = register_classes_factory((View3D_OT_slvs_occt_quad_mesh, View3D_OT_slvs_occt_patch_preview))


def unregister():
    clear_preview()
    _unregister()
