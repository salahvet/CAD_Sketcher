import bpy
from bpy.props import BoolProperty, EnumProperty, FloatProperty, IntProperty
from bpy.types import Context, Operator
from bpy.utils import register_classes_factory

from ..declarations import Operators


# ---------------------------------------------------------------------------
# Utilitaires
# ---------------------------------------------------------------------------

def get_active_sketch(context):
    sketch = context.scene.sketcher.active_sketch
    if sketch is not None and getattr(sketch, "is_sketch", False):
        return sketch
    sketches = list(context.scene.sketcher.entities.sketches)
    return sketches[0] if sketches else None


def ensure_sketch_solid(context, sketch):
    """Sketch ferme -> objet mesh rempli, regenere par CAD Sketcher
    a chaque modification du sketch."""
    sketch.convert_type = "MESH"
    sketch.fill_shape = True
    from .. import converters
    converters.update_convertor_geometry(context.scene)
    return sketch.target_object


def mesh_objects_enum(self, context):
    items = []
    for o in context.scene.objects:
        if o.type != "MESH":
            continue
        if o.name.startswith("SLVS_cutter"):
            continue
        if o.get("occt_part") or o.get("occt_quad_of"):
            continue  # OCCT parts are edited with the OCCT features, not with mesh modifiers
        items.append((o.name, o.name, ""))
    return items or [("NONE", "<no mesh object>", "")]


# ---------------------------------------------------------------------------
# Pad (extrusion)
# ---------------------------------------------------------------------------

class View3D_OT_slvs_feature_pad(Operator):
    """Extrude the active closed sketch into a solid (parametric:
    edit the sketch and the solid follows)"""

    bl_idname = Operators.FeaturePad
    bl_label = "Pad (Extrude)"
    bl_options = {"REGISTER", "UNDO"}

    depth: FloatProperty(
        name="Depth", description="Extrusion depth",
        default=0.1, min=0.0001, soft_max=10.0, unit="LENGTH")
    symmetric: BoolProperty(
        name="Symmetric", description="Extrude on both sides of the sketch plane",
        default=False)

    @classmethod
    def poll(cls, context):
        return get_active_sketch(context) is not None

    def execute(self, context: Context):
        sketch = get_active_sketch(context)
        obj = ensure_sketch_solid(context, sketch)
        if obj is None:
            self.report({"ERROR"}, "Sketch conversion failed (closed contour?)")
            return {"CANCELLED"}

        mod = obj.modifiers.get("SLVS_Pad")
        if mod is None:
            mod = obj.modifiers.new("SLVS_Pad", "SOLIDIFY")
        mod.thickness = self.depth
        mod.offset = 0.0 if self.symmetric else -1.0
        mod.use_even_offset = True

        self.report({"INFO"}, f"Pad: {obj.name}, depth {self.depth:.4g}")
        return {"FINISHED"}


# ---------------------------------------------------------------------------
# Pocket (enlevement de matiere)
# ---------------------------------------------------------------------------

class View3D_OT_slvs_feature_pocket(Operator):
    """Cut the active sketch profile into a target solid
    (parametric boolean: edit the sketch and the cut follows)"""

    bl_idname = Operators.FeaturePocket
    bl_label = "Pocket (Cut)"
    bl_options = {"REGISTER", "UNDO"}

    depth: FloatProperty(
        name="Depth", description="Cut depth",
        default=0.2, min=0.0001, soft_max=10.0, unit="LENGTH")
    symmetric: BoolProperty(
        name="Symmetric", description="Cut on both sides of the sketch plane",
        default=True)
    target: EnumProperty(
        name="Target", description="Solid to cut into",
        items=mesh_objects_enum)

    @classmethod
    def poll(cls, context):
        return get_active_sketch(context) is not None

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self, width=280)

    def execute(self, context: Context):
        if self.target == "NONE":
            self.report({"ERROR"}, "No target solid")
            return {"CANCELLED"}
        target = context.scene.objects.get(self.target)
        sketch = get_active_sketch(context)
        if target is None:
            self.report({"ERROR"}, "Target not found")
            return {"CANCELLED"}
        if sketch.target_object == target:
            self.report({"ERROR"}, "Target cannot be the sketch own solid")
            return {"CANCELLED"}

        obj = ensure_sketch_solid(context, sketch)
        if obj is None:
            self.report({"ERROR"}, "Sketch conversion failed (closed contour?)")
            return {"CANCELLED"}

        # Le solide du sketch devient un outil de coupe cache
        cutter_name = f"SLVS_cutter_{sketch.name}"
        obj.name = cutter_name
        mod = obj.modifiers.get("SLVS_Pad")
        if mod is None:
            mod = obj.modifiers.new("SLVS_Pad", "SOLIDIFY")
        mod.thickness = self.depth
        mod.offset = 0.0 if self.symmetric else -1.0
        mod.use_even_offset = True
        obj.display_type = "WIRE"
        obj.hide_render = True

        bool_name = f"SLVS_Pocket_{sketch.name}"
        bmod = target.modifiers.get(bool_name)
        if bmod is None:
            bmod = target.modifiers.new(bool_name, "BOOLEAN")
        bmod.operation = "DIFFERENCE"
        bmod.object = obj
        bmod.solver = "EXACT"

        self.report({"INFO"}, f"Pocket: {target.name} - {sketch.name}")
        return {"FINISHED"}

    def draw(self, context):
        layout = self.layout
        layout.prop(self, "target")
        layout.prop(self, "depth")
        layout.prop(self, "symmetric")


# ---------------------------------------------------------------------------
# Fillet / Chamfer
# ---------------------------------------------------------------------------

class View3D_OT_slvs_feature_fillet(Operator):
    """Add a fillet (rounded) or chamfer (straight) on the edges of a solid
    (angle-based bevel modifier)"""

    bl_idname = Operators.FeatureFillet
    bl_label = "Fillet / Chamfer"
    bl_options = {"REGISTER", "UNDO"}

    width: FloatProperty(
        name="Width", description="Fillet/chamfer width",
        default=0.01, min=0.0001, soft_max=1.0, unit="LENGTH")
    chamfer: BoolProperty(
        name="Chamfer", description="Straight cut instead of rounded fillet",
        default=False)
    segments: IntProperty(
        name="Segments", description="Roundness (ignored for chamfer)",
        default=3, min=1, max=12)
    angle_limit: FloatProperty(
        name="Angle Limit", description="Only bevel edges sharper than this",
        default=0.523599, min=0.0175, max=3.1415, subtype="ANGLE")
    target: EnumProperty(
        name="Target", description="Solid to bevel",
        items=mesh_objects_enum)

    def invoke(self, context, event):
        # cible par defaut : l'objet actif s'il est un mesh
        ob = context.active_object
        if ob and ob.type == "MESH" and not ob.name.startswith("SLVS_cutter") and not ob.get("occt_part"):
            self.target = ob.name
        return context.window_manager.invoke_props_dialog(self, width=280)

    def execute(self, context: Context):
        if self.target == "NONE":
            self.report({"ERROR"}, "No target solid")
            return {"CANCELLED"}
        target = context.scene.objects.get(self.target)
        if target is None:
            self.report({"ERROR"}, "Target not found")
            return {"CANCELLED"}

        name = "SLVS_Chamfer" if self.chamfer else "SLVS_Fillet"
        mod = target.modifiers.get(name)
        if mod is None:
            mod = target.modifiers.new(name, "BEVEL")
        mod.width = self.width
        mod.segments = 1 if self.chamfer else self.segments
        mod.limit_method = "ANGLE"
        mod.angle_limit = self.angle_limit
        mod.miter_outer = "MITER_ARC"

        self.report({"INFO"}, f"{name[5:]}: {target.name}, width {self.width:.4g}")
        return {"FINISHED"}

    def draw(self, context):
        layout = self.layout
        layout.prop(self, "target")
        layout.prop(self, "width")
        layout.prop(self, "chamfer")
        if not self.chamfer:
            layout.prop(self, "segments")
        layout.prop(self, "angle_limit")


register, unregister = register_classes_factory((
    View3D_OT_slvs_feature_pad,
    View3D_OT_slvs_feature_pocket,
    View3D_OT_slvs_feature_fillet,
))
