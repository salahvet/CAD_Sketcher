from copy import deepcopy

import bpy
from bpy.props import BoolProperty, EnumProperty, FloatProperty, StringProperty
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


class View3D_OT_slvs_mirror(Operator):
    """Mirror the selected entities across a construction line.
    Select the entities to mirror plus one construction line (the axis,
    it won't be duplicated)."""

    bl_idname = Operators.Mirror
    bl_label = "Mirror"
    bl_options = {"REGISTER", "UNDO"}

    destination: EnumProperty(
        name="Destination",
        description="Where to place the mirrored copy",
        items=[
            ("CURRENT", "Current Sketch",
             "Mirror inside the active sketch (weldable to the original)"),
            ("NEW", "New Sketch",
             "Create a separate sketch on the same workplane"),
        ],
        default="CURRENT")
    new_name: StringProperty(
        name="Sketch Name", description="Name of the new sketch",
        default="Sketch_Mirror")
    auto_merge: BoolProperty(
        name="Auto Merge",
        description="Merge near-coincident points after mirroring "
                    "(welds endpoints lying on the axis)",
        default=True)
    merge_threshold: FloatProperty(
        name="Merge Threshold", description="Maximum merge distance",
        default=0.005, min=0.00001, soft_max=0.1, precision=4)

    @classmethod
    def poll(cls, context):
        return (context.mode == "OBJECT"
                and context.scene.sketcher.active_sketch_i != -1
                and len(context.scene.sketcher.entities.selected_active) > 0)

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self, width=280)

    def execute(self, context: Context):
        sse = context.scene.sketcher.entities
        sketch = context.scene.sketcher.active_sketch

        # Axe : la DERNIERE ligne de construction selectionnee
        # (convention : on selectionne l'axe en dernier)
        constr_lines = [e for e in sse.selected_active
                        if getattr(e, "construction", False) and e.is_line()]
        if not constr_lines:
            self.report({"ERROR"},
                        "Select a construction line as the mirror axis (last)")
            return {"CANCELLED"}
        order = {idx: i for i, idx in enumerate(global_data.selected)}
        constr_lines.sort(key=lambda e: order.get(e.slvs_index, -1))
        axis = constr_lines[-1]

        A = Vector(axis.p1.co)
        d = Vector(axis.p2.co) - A
        if d.length < 1e-9:
            self.report({"ERROR"}, "Axis line has zero length")
            return {"CANCELLED"}
        d.normalize()

        # Aucune geometrie de construction n'est dupliquee (axe et guides)
        for e in list(sse.selected_active):
            if getattr(e, "construction", False):
                e.selected = False

        if not sse.selected_active:
            self.report({"ERROR"}, "No entity selected besides the axis")
            return {"CANCELLED"}

        # Copie via le mecanisme standard
        saved_buffer = getattr(global_data, "COPY_BUFFER", None)
        bpy.ops.view3d.slvs_copy()
        buf = deepcopy(global_data.COPY_BUFFER)
        if saved_buffer is not None:
            global_data.COPY_BUFFER = saved_buffer

        # Destination : sketch actif ou nouveau sketch sur le meme plan
        if self.destination == "NEW":
            target_sketch = sse.add_sketch(sketch.wp)
            target_sketch.name = self.new_name
        else:
            target_sketch = sketch
        target_index = target_sketch.slvs_index

        for element in iter_elements_dict(buf):
            if "sketch_i" in element.keys():
                element["sketch_i"] = target_index

        # Capture AVANT collage : les nouvelles entites sont identifiees
        # par difference (les indices du tampon ne sont pas fiables
        # sur les grosses selections chainees)
        before_pts = {p.slvs_index for p in sse.points2D}
        before_arcs = {a.slvs_index for a in sse.arcs}

        paste(context, buf)

        def reflect(co):
            v = Vector(co) - A
            return A + d * (2.0 * v.dot(d)) - v

        mirrored_pts = 0
        for p in sse.points2D:
            if p.slvs_index in before_pts:
                continue
            p.co = reflect(p.co)
            mirrored_pts += 1
        for arc in sse.arcs:
            if arc.slvs_index in before_arcs:
                continue
            # un miroir inverse le sens de parcours des arcs
            arc.invert_direction = not arc.invert_direction

        deselect_all(context)

        if self.auto_merge and self.destination == "CURRENT":
            merged = self._merge_close_points(context, sketch)
            delete_collapsed_lines(context)
        else:
            merged = 0

        solve_system(context, sketch=target_sketch)
        if target_sketch != sketch:
            solve_system(context, sketch=sketch)
        if context.area:
            context.area.tag_redraw()
        self.report({"INFO"},
                    f"Mirror: {mirrored_pts} point(s)"
                    + (f", merged {merged}" if merged else ""))
        return {"FINISHED"}

    def _merge_close_points(self, context, sketch):
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
        layout.prop(self, "destination", expand=True)
        if self.destination == "NEW":
            layout.prop(self, "new_name")
        else:
            layout.prop(self, "auto_merge")
            if self.auto_merge:
                layout.prop(self, "merge_threshold")


register, unregister = register_classes_factory((View3D_OT_slvs_mirror,))
