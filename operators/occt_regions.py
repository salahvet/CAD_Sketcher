"""Fork: OCCT Pick Regions — choose which regions of the active sketch a Pad or a Pocket uses.

The sketch is split into its regions (closed contours cut against each other). Hover to
highlight, click to select or unselect, Enter to confirm, Esc to cancel. Each picked region
is stored as one interior point in sketch coordinates, so it is found again after small edits.
"""
import bpy
from bpy.types import Operator
from bpy.utils import register_classes_factory
from bpy_extras import view3d_utils
from mathutils import Vector

from ..declarations import Operators
from .. import occt_bridge
from . import occt_ops


def _point_in_tri(p, a, b, c):
    def side(p1, p2, p3):
        return (p1.x - p3.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p3.y)
    d1, d2, d3 = side(p, a, b), side(p, b, c), side(p, c, a)
    neg = d1 < 0 or d2 < 0 or d3 < 0
    pos = d1 > 0 or d2 > 0 or d3 > 0
    return not (neg and pos)


class View3D_OT_slvs_occt_pick_regions(Operator):
    """Pick the regions of the active sketch to extrude: click to select / unselect,
    Enter to confirm, Esc to cancel. Then OCCT Pad / Pocket use only those regions"""

    bl_idname = Operators.OCCTPickRegions
    bl_label = "OCCT Pick Regions"

    @classmethod
    def poll(cls, context):
        return (context.area is not None and context.area.type == "VIEW_3D"
                and context.scene.sketcher.active_sketch is not None)

    def invoke(self, context, event):
        import gpu
        from gpu_extras.batch import batch_for_shader

        sketch = context.scene.sketcher.active_sketch
        try:
            faces = occt_bridge.sketch_regions(context, sketch)
        except Exception as e:
            self.report({"ERROR"}, f"Cannot compute the regions: {e}")
            return {"CANCELLED"}
        if not faces:
            self.report({"ERROR"}, "The sketch has no closed region")
            return {"CANCELLED"}

        self.sketch_name = sketch.name
        self.M = sketch.wp.matrix_basis.copy()
        self.Mi = self.M.inverted()
        self.normal = (self.M.to_3x3() @ Vector((0, 0, 1))).normalized()
        self.shader = gpu.shader.from_builtin("UNIFORM_COLOR")
        self.regions = []  # (fill batch, outline batch, 2D triangles, interior point 2D)
        for f in faces:
            verts, tris = f.tessellate(1e-6)
            v3 = [Vector((v.X, v.Y, v.Z)) for v in verts]
            v2 = [(self.Mi @ v).to_2d() for v in v3]
            fill = batch_for_shader(self.shader, "TRIS", {"pos": [tuple(v) for v in v3]},
                                    indices=[tuple(t) for t in tris])
            lines = []
            for e in f.edges():
                pts = [e @ (k / 24) for k in range(25)]
                for a, b in zip(pts, pts[1:]):
                    lines += [(a.X, a.Y, a.Z), (b.X, b.Y, b.Z)]
            outline = batch_for_shader(self.shader, "LINES", {"pos": lines})
            t2 = [(v2[a], v2[b], v2[c]) for a, b, c in tris]
            inside = (self.Mi @ Vector(tuple(occt_bridge.region_interior_point(f)))).to_2d()
            self.regions.append((fill, outline, t2, inside))

        # regions already picked for this sketch
        self.picked = set()
        for x, y in occt_ops.region_picks(context.scene, self.sketch_name):
            i = self._region_at(Vector((x, y)))
            if i is not None:
                self.picked.add(i)
        self.hover = None
        self._handle = bpy.types.SpaceView3D.draw_handler_add(self._draw, (context,), "WINDOW", "POST_VIEW")
        context.window_manager.modal_handler_add(self)
        context.area.header_text_set("Pick Regions: click to select / unselect, Enter to confirm, Esc to cancel")
        context.area.tag_redraw()
        return {"RUNNING_MODAL"}

    def _region_at(self, p2):
        for i, (_f, _o, tris, _c) in enumerate(self.regions):
            if any(_point_in_tri(p2, a, b, c) for a, b, c in tris):
                return i
        return None

    def _mouse_on_plane(self, context, event):
        region, rv3d = context.region, context.region_data
        coord = (event.mouse_region_x, event.mouse_region_y)
        origin = view3d_utils.region_2d_to_origin_3d(region, rv3d, coord)
        ray = view3d_utils.region_2d_to_vector_3d(region, rv3d, coord)
        denom = ray.dot(self.normal)
        if abs(denom) < 1e-12:
            return None
        t = (self.M.translation - origin).dot(self.normal) / denom
        return (self.Mi @ (origin + ray * t)).to_2d()

    def _finish(self, context):
        bpy.types.SpaceView3D.draw_handler_remove(self._handle, "WINDOW")
        context.area.header_text_set(None)
        context.area.tag_redraw()

    def modal(self, context, event):
        context.area.tag_redraw()
        if event.type == "MOUSEMOVE":
            p = self._mouse_on_plane(context, event)
            self.hover = self._region_at(p) if p is not None else None
            return {"RUNNING_MODAL"}
        if event.type == "LEFTMOUSE" and event.value == "PRESS":
            if self.hover is not None:
                self.picked ^= {self.hover}
            return {"RUNNING_MODAL"}
        if event.type in {"RET", "NUMPAD_ENTER", "SPACE"} and event.value == "PRESS":
            points = [[self.regions[i][3].x, self.regions[i][3].y] for i in sorted(self.picked)]
            occt_ops.set_region_picks(context.scene, self.sketch_name, points)
            self._finish(context)
            self.report({"INFO"}, f"{len(points)} region(s) picked" if points else "Region picks cleared")
            return {"FINISHED"}
        if event.type in {"ESC", "RIGHTMOUSE"} and event.value == "PRESS":
            self._finish(context)
            return {"CANCELLED"}
        return {"PASS_THROUGH"}

    def _draw(self, context):
        import gpu
        gpu.state.blend_set("ALPHA")
        gpu.state.depth_test_set("NONE")
        sh = self.shader
        for i, (fill, outline, _t, _c) in enumerate(self.regions):
            if i in self.picked:
                col = (0.15, 0.85, 0.35, 0.45)
            elif i == self.hover:
                col = (1.0, 0.85, 0.15, 0.35)
            else:
                col = (0.6, 0.6, 0.65, 0.12)
            sh.bind()
            sh.uniform_float("color", col)
            fill.draw(sh)
            sh.uniform_float("color", (col[0], col[1], col[2], 0.9))
            gpu.state.line_width_set(2.0 if (i in self.picked or i == self.hover) else 1.0)
            outline.draw(sh)
        gpu.state.line_width_set(1.0)
        gpu.state.blend_set("NONE")


register, unregister = register_classes_factory((View3D_OT_slvs_occt_pick_regions,))
