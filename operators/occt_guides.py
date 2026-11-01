"""Fork: OCCT Draw Guides — draw retopology lines straight on an OCCT part.

Click on the part to place points; Enter, Space or a double click ends the line. After each line
the patch preview is rebuilt, so the layout is seen while drawing. The lines are kept on the part
(local space) and used by OCCT Quad Mesh / Preview Patches with Guides = "Drawn on Part".
"""
import bpy
from bpy.types import Operator
from bpy.utils import register_classes_factory
from bpy_extras import view3d_utils
from mathutils import Vector
from mathutils.bvhtree import BVHTree
from mathutils.geometry import intersect_line_plane

from ..declarations import Operators
from . import occt_ops, occt_quad_ops

HELP = ("Draw Guides: click on the part to add points | Enter/Space/double click: end the line | "
        "Backspace/Ctrl+Z: undo | D: delete the line under the cursor | Esc/right click: done")


def _part_of(context):
    ob = context.active_object
    if ob is None:
        return None
    name = ob.get("occt_quad_of", ob.name)
    return name if name in occt_ops.get_recipes(context.scene) else None


class View3D_OT_slvs_occt_draw_guides(Operator):
    """Draw retopology guide lines directly on the active OCCT part; the patch preview updates
    after each line. OCCT Quad Mesh then follows them (Guides: Drawn on Part)"""

    bl_idname = Operators.OCCTDrawGuides
    bl_label = "Draw Guides"
    bl_options = {"REGISTER", "UNDO"}

    clear: bpy.props.BoolProperty(name="Clear All Guides", default=False, options={"SKIP_SAVE"})

    @classmethod
    def poll(cls, context):
        if context.area is None or context.area.type != "VIEW_3D":
            return False
        if _part_of(context) is None:
            cls.poll_message_set("Select an OCCT part (or its _Quad mesh)")
            return False
        return True

    # ---- helpers ------------------------------------------------------------------------------
    def _refresh_preview(self, context):
        try:
            solid = occt_quad_ops.part_solid(context, self.part)
            guides = occt_quad_ops.guides_for(context, self.obj, "DRAWN")
            verts, faces, sharp, info = occt_quad_ops.occt_quad2.quad_mesh(
                solid, 0.0001, 0.0, 45.0, 40.0, occt_quad_ops.occt_quad.blender_cdt, guides=guides)
            occt_quad_ops.show_preview(context, self.part, verts, faces, info["patch_of"])
            n = len(set(p for p in info["patch_of"] if p >= 0))
            self.status = f"{n} patches, {len(faces)} quads"
            if info.get("free_patches"):
                self.status += f", {info['free_patches']} red (not regular)"
            loose = [w for w in info.get("warnings", []) if "guide" in w]
            if loose:
                self.status += " — a guide does not reach an edge"
        except Exception as e:
            self.status = f"Preview failed: {e}"
        context.area.header_text_set(HELP + "   ▸ " + self.status)

    def _ray(self, context, event):
        region, rv3d = context.region, context.region_data
        co = (event.mouse_region_x, event.mouse_region_y)
        o = view3d_utils.region_2d_to_origin_3d(region, rv3d, co)
        d = view3d_utils.region_2d_to_vector_3d(region, rv3d, co)
        return self.Mi @ o, (self.Mi.to_3x3() @ d).normalized()

    def _pick(self, context, event):
        """Point (local space) under the mouse: on the part, or on the plane of the current line."""
        o, d = self._ray(context, event)
        loc, nrm, idx, dist = self.bvh.ray_cast(o, d)
        if loc is not None:
            # snap to an existing guide point close on screen
            p2 = Vector((event.mouse_region_x, event.mouse_region_y))
            for line in self.lines + [{"pts": self.current}]:
                for q in line["pts"]:
                    s = self._to_screen(context, Vector(q))
                    if s is not None and (s - p2).length < 12:
                        return Vector(q), nrm
            return loc, nrm
        if self.current:
            n = Vector(self.current_n)
            hit = intersect_line_plane(o, o + d, Vector(self.current[0]), n)
            if hit is not None:
                return hit, n
        return None, None

    def _to_screen(self, context, p_local):
        return view3d_utils.location_3d_to_region_2d(context.region, context.region_data, self.M @ p_local)

    def _save(self):
        occt_quad_ops.set_drawn_guides(self.obj, self.lines)

    def _end_line(self, context):
        if len(self.current) >= 2:
            self.lines.append({"n": list(self.current_n), "pts": [list(p) for p in self.current]})
            self._save()
            self.current = []
            self._refresh_preview(context)
        else:
            self.current = []

    # ---- drawing --------------------------------------------------------------------------------
    def _draw(self, context):
        import gpu
        from gpu_extras.batch import batch_for_shader
        sh = gpu.shader.from_builtin("UNIFORM_COLOR")
        gpu.state.blend_set("ALPHA")
        gpu.state.depth_test_set("NONE")
        W = lambda p: tuple(self.M @ Vector(p))
        segs = []
        for line in self.lines:
            pts = line["pts"]
            for a, b in zip(pts, pts[1:]):
                segs += [W(a), W(b)]
        if segs:
            sh.bind()
            sh.uniform_float("color", (1.0, 0.45, 0.05, 1.0))
            gpu.state.line_width_set(3.0)
            batch_for_shader(sh, "LINES", {"pos": segs}).draw(sh)
        cur = [W(p) for p in self.current]
        if self.hover is not None and cur:
            cur.append(W(self.hover))
        if len(cur) >= 2:
            sh.bind()
            sh.uniform_float("color", (1.0, 0.9, 0.1, 1.0))
            gpu.state.line_width_set(2.0)
            batch_for_shader(sh, "LINE_STRIP", {"pos": cur}).draw(sh)
        pts = [W(p) for line in self.lines for p in line["pts"]] + [W(p) for p in self.current]
        if pts:
            gpu.state.point_size_set(7.0)
            sh.bind()
            sh.uniform_float("color", (1.0, 1.0, 1.0, 1.0))
            batch_for_shader(sh, "POINTS", {"pos": pts}).draw(sh)
        gpu.state.line_width_set(1.0)
        gpu.state.blend_set("NONE")

    # ---- operator -------------------------------------------------------------------------------
    def invoke(self, context, event):
        self.part = _part_of(context)
        self.obj = context.scene.objects.get(self.part)
        if self.obj is None or self.obj.type != "MESH":
            self.report({"ERROR"}, "The part has no mesh to draw on")
            return {"CANCELLED"}
        if self.clear:
            occt_quad_ops.set_drawn_guides(self.obj, [])
            occt_quad_ops.clear_preview()
            context.area.tag_redraw()
            self.report({"INFO"}, "Guides cleared")
            return {"FINISHED"}
        self.M = self.obj.matrix_world.copy()
        self.Mi = self.M.inverted()
        me = self.obj.data
        self.bvh = BVHTree.FromPolygons([v.co.copy() for v in me.vertices], [tuple(p.vertices) for p in me.polygons])
        self.lines = occt_quad_ops.drawn_guides(self.obj)
        self.current, self.current_n, self.hover = [], (0, 0, 1), None
        self.status = ""
        self._handle = bpy.types.SpaceView3D.draw_handler_add(self._draw, (context,), "WINDOW", "POST_VIEW")
        context.window_manager.modal_handler_add(self)
        self._refresh_preview(context)
        return {"RUNNING_MODAL"}

    def _finish(self, context):
        bpy.types.SpaceView3D.draw_handler_remove(self._handle, "WINDOW")
        context.area.header_text_set(None)
        context.area.tag_redraw()

    def modal(self, context, event):
        context.area.tag_redraw()
        if event.type == "MOUSEMOVE":
            p, n = self._pick(context, event)
            self.hover = p
            return {"RUNNING_MODAL"}
        if event.type == "LEFTMOUSE" and event.value == "DOUBLE_CLICK":
            self._end_line(context)
            return {"RUNNING_MODAL"}
        if event.type == "LEFTMOUSE" and event.value == "PRESS":
            p, n = self._pick(context, event)
            if p is not None:
                if not self.current:
                    self.current_n = tuple(n)
                if not self.current or (Vector(self.current[-1]) - p).length > 1e-7:
                    self.current.append(tuple(p))
            return {"RUNNING_MODAL"}
        if event.type in {"RET", "NUMPAD_ENTER", "SPACE"} and event.value == "PRESS":
            self._end_line(context)
            return {"RUNNING_MODAL"}
        if (event.type == "BACK_SPACE" or (event.type == "Z" and event.ctrl)) and event.value == "PRESS":
            if self.current:
                self.current.pop()
            elif self.lines:
                self.lines.pop()
                self._save()
                self._refresh_preview(context)
            return {"RUNNING_MODAL"}
        if event.type in {"D", "DEL", "X"} and event.value == "PRESS" and self.lines:
            p2 = Vector((event.mouse_region_x, event.mouse_region_y))
            best = None
            for i, line in enumerate(self.lines):
                pts = [self._to_screen(context, Vector(q)) for q in line["pts"]]
                for a, b in zip(pts, pts[1:]):
                    if a is None or b is None:
                        continue
                    ab = b - a
                    t = 0.0 if ab.length == 0 else max(0.0, min(1.0, (p2 - a).dot(ab) / ab.length_squared))
                    dd = (a + ab * t - p2).length
                    if best is None or dd < best[0]:
                        best = (dd, i)
            if best is not None and best[0] < 25:
                self.lines.pop(best[1])
                self._save()
                self._refresh_preview(context)
            return {"RUNNING_MODAL"}
        if event.type in {"ESC", "RIGHTMOUSE"} and event.value == "PRESS":
            self._end_line(context)
            self._finish(context)
            self.report({"INFO"}, f"{len(self.lines)} guide line(s) on {self.part}")
            return {"FINISHED"}
        return {"PASS_THROUGH"}


register, unregister = register_classes_factory((View3D_OT_slvs_occt_draw_guides,))
