import bpy
from bpy.props import BoolProperty, FloatProperty
from bpy.types import Context, Operator
from bpy.utils import register_classes_factory
from mathutils import Vector

from ..declarations import Operators
from ..solver import solve_system
from ..utilities.merge import merge_point_indices, delete_collapsed_lines


class View3D_OT_slvs_merge_points(Operator):
    """Weld sketch points together.
    - With 2+ points selected: merges them onto the first one.
    - Without selection: scans the sketch and welds every pair closer
      than the threshold (adjust live in the redo panel)."""

    bl_idname = Operators.MergePoints
    bl_label = "Merge Points"
    bl_options = {"REGISTER", "UNDO"}

    threshold: FloatProperty(
        name="Threshold", description="Maximum merge distance (scan mode)",
        default=0.005, min=0.00001, soft_max=0.2, precision=4)
    use_midpoint: BoolProperty(
        name="At Midpoint",
        description="Place the merged point at the midpoint instead of "
                    "keeping the first point position", default=False)

    @classmethod
    def poll(cls, context):
        return context.scene.sketcher.active_sketch_i != -1

    def execute(self, context: Context):
        sse = context.scene.sketcher.entities
        sketch = context.scene.sketcher.active_sketch

        sel_points = [e for e in sse.selected_active
                      if e.is_point() and hasattr(e, "co")]

        merged = 0
        if len(sel_points) >= 2:
            # --- mode manuel : souder la selection sur le premier point ---
            target = sel_points[0]
            if self.use_midpoint:
                mid = sum((Vector(p.co) for p in sel_points), Vector((0, 0)))
                target.co = mid / len(sel_points)
            target_i = target.slvs_index
            dup_indices = sorted((p.slvs_index for p in sel_points[1:]),
                                 reverse=True)
            for di in dup_indices:
                target_i = merge_point_indices(context, target_i, [di])
                merged += 1
        else:
            # --- mode scan : paires sous la tolerance, iterativement ---
            for _guard in range(500):
                pts = [p for p in sse.points2D
                       if p.sketch_i == sketch.slvs_index
                       and not p.origin and not p.fixed]
                pair = None
                for i, a in enumerate(pts):
                    ax, ay = a.co
                    for b in pts[i + 1:]:
                        bx, by = b.co
                        if ((bx - ax) ** 2 + (by - ay) ** 2) ** 0.5 \
                                <= self.threshold:
                            pair = (a, b)
                            break
                    if pair:
                        break
                if not pair:
                    break
                a, b = pair
                if self.use_midpoint:
                    a.co = (Vector(a.co) + Vector(b.co)) / 2
                merge_point_indices(context, a.slvs_index, [b.slvs_index])
                merged += 1

        collapsed = delete_collapsed_lines(context)
        solve_system(context, sketch=sketch)
        if context.area:
            context.area.tag_redraw()
        self.report({"INFO"},
                    f"Merged {merged} point(s)"
                    + (f", removed {collapsed} degenerate line(s)" if collapsed else ""))
        return {"FINISHED"}

    def draw(self, context):
        layout = self.layout
        layout.prop(self, "threshold")
        layout.prop(self, "use_midpoint")


register, unregister = register_classes_factory((View3D_OT_slvs_merge_points,))
