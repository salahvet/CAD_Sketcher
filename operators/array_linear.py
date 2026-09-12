from copy import deepcopy

import bpy
from bpy.props import FloatVectorProperty, IntProperty
from bpy.types import Context, Operator
from bpy.utils import register_classes_factory
from mathutils import Vector

from .. import global_data
from ..declarations import Operators
from ..serialize import iter_elements_dict, paste
from ..solver import solve_system
from ..utilities.select import deselect_all
from ..utilities.merge import merge_point_indices, delete_collapsed_lines



def _filter_geometry_selection(sse):
    """Ne garde dans la selection que la geometrie 2D legitime
    (lignes, arcs, cercles, beziers, points 2D). Deselectionne le reste
    (sketchs, plans de travail, normales...) qui ferait planter le collage."""
    kept = 0
    for e in list(sse.selected_active):
        type_name = type(e).__name__
        if type_name in ("SlvsLine2D", "SlvsArc", "SlvsCircle",
                         "SlvsBezier", "SlvsPoint2D"):
            kept += 1
            continue
        e.selected = False
    return kept


class View3D_OT_slvs_array_linear(Operator):
    """Repeat the selected entities along a direction (linear pattern).
    Tip: select a construction line together with the entities to use it
    as the step vector (it won't be duplicated)."""

    bl_idname = Operators.ArrayLinear
    bl_label = "Linear Array"
    bl_options = {"REGISTER", "UNDO"}

    count: IntProperty(
        name="Count", description="Total number of instances",
        default=3, min=2, max=100)
    offset: FloatVectorProperty(
        name="Vector", description="Repeat vector (sketch X, Y)",
        size=2, default=(0.1, 0.0), subtype="COORDINATES")
    auto_merge: __import__("bpy").props.BoolProperty(
        name="Auto Merge",
        description="Merge near-coincident points after the repetition "
                    "(continuous chained contour)",
        default=True)
    merge_threshold: __import__("bpy").props.FloatProperty(
        name="Merge Threshold", description="Maximum merge distance",
        default=0.005, min=0.00001, soft_max=0.1, precision=4)
    distribute: __import__("bpy").props.BoolProperty(
        name="Distribute Along Vector",
        description="The vector is the TOTAL span: instances are evenly "
                    "distributed along it (construction line = full guide). "
                    "Uncheck for a fixed step per instance",
        default=True)

    @classmethod
    def poll(cls, context):
        return (context.mode == "OBJECT"
                and context.scene.sketcher.active_sketch_i != -1
                and len(context.scene.sketcher.entities.selected_active) > 0)

    def invoke(self, context: Context, event):
        # Si une ligne de construction est selectionnee : vecteur de pas auto
        selected = context.scene.sketcher.entities.selected_active
        for e in selected:
            if getattr(e, "construction", False) and hasattr(e, "p1") \
                    and hasattr(e, "p2") and e.is_line():
                vec = Vector(e.p2.co) - Vector(e.p1.co)
                self.offset = (vec.x, vec.y)
                self.distribute = True
                e.selected = False   # ne pas dupliquer le guide
                break
        else:
            self.distribute = False
        # Boîte de dialogue : nombre + décalage avant exécution
        return context.window_manager.invoke_props_dialog(self, width=260)

    def _merge_close_points(self, context, sketch):
        """Fusionne iterativement les paires de points du sketch plus proches
        que la tolerance. Reconstruit la liste a chaque passe pour rester
        immunise contre le remappage d'indices."""
        sse = context.scene.sketcher.entities
        merged = 0
        for _guard in range(500):
            pts = [p for p in sse.points2D
                   if p.sketch_i == sketch.slvs_index
                   and not p.origin and not p.fixed]
            def would_collapse(ia, ib):
                for coll in (sse.lines2D, sse.arcs, sse.beziers, sse.circles):
                    for e in coll:
                        ptrs = [getattr(e, n) for n in
                                ("p1_i", "p2_i", "ctrl1_i", "ctrl2_i", "ct_i")
                                if hasattr(e, n)]
                        if ia in ptrs and ib in ptrs:
                            return True
                return False

            pair = None
            for i, a in enumerate(pts):
                ax, ay = a.co
                for b in pts[i + 1:]:
                    bx, by = b.co
                    if ((bx - ax) ** 2 + (by - ay) ** 2) ** 0.5 \
                            <= self.merge_threshold:
                        if would_collapse(a.slvs_index, b.slvs_index):
                            continue
                        pair = (a.slvs_index, b.slvs_index)
                        break
                if pair:
                    break
            if not pair:
                break
            merge_point_indices(context, pair[0], [pair[1]])
            merged += 1
        return merged

    def draw(self, context):
        layout = self.layout
        layout.prop(self, "count")
        col = layout.column(align=True)
        col.prop(self, "offset")
        layout.prop(self, "distribute")
        layout.prop(self, "auto_merge")
        if self.auto_merge:
            layout.prop(self, "merge_threshold")

    def execute(self, context: Context):
        sse = context.scene.sketcher.entities
        if not sse.selected_active:
            self.report({"WARNING"}, "No entity selected")
            return {"CANCELLED"}

        # Construire le tampon via l'operateur copy existant
        saved_buffer = getattr(global_data, "COPY_BUFFER", None)
        bpy.ops.view3d.slvs_copy()
        array_buffer = deepcopy(global_data.COPY_BUFFER)
        if saved_buffer is not None:
            global_data.COPY_BUFFER = saved_buffer

        if self.distribute and self.count > 1:
            offset = Vector(self.offset) / (self.count - 1)
        else:
            offset = Vector(self.offset)
        created = 0
        for k in range(1, self.count):
            buf = deepcopy(array_buffer)
            for element in iter_elements_dict(buf):
                if "sketch_i" in element.keys():
                    element["sketch_i"] = context.scene.sketcher.active_sketch_i
            before_pts = {p.slvs_index for p in sse.points2D}
            paste(context, buf)

            # Decaler les points nouvellement colles de k * offset
            # (identification par difference : fiable sur grosses selections)
            for p in sse.points2D:
                if p.slvs_index in before_pts:
                    continue
                p.co = Vector(p.co) + offset * k
                created += 1

        deselect_all(context)
        sketch = context.scene.sketcher.active_sketch

        if self.auto_merge:
            merged = self._merge_close_points(context, sketch)
            delete_collapsed_lines(context)
            if merged:
                self.report({"INFO"}, f"Merged {merged} point(s)")

        solve_system(context, sketch=sketch)
        if context.area:
            context.area.tag_redraw()
        self.report({"INFO"},
                    f"Array: {self.count} instances")
        return {"FINISHED"}


register, unregister = register_classes_factory((View3D_OT_slvs_array_linear,))
