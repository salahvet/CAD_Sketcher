import bmesh
import bpy
from bpy.props import BoolProperty, EnumProperty, FloatProperty, IntProperty
from bpy.types import Context, Operator
from bpy.utils import register_classes_factory
from mathutils import Vector

from ..declarations import Operators
from .features_3d import get_active_sketch, ensure_sketch_solid


# ---------------------------------------------------------------------------
# Construction du solide structure
# ---------------------------------------------------------------------------

def build_pad_solid(context, sketch, depth, symmetric, existing=None):
    """Extrusion REELLE du profil du sketch en mesh structure :
    - vertex groups 'rim_bottom' / 'rim_top' (couronnes d'aretes)
    - bevel weight = 1.0 sur les aretes laterales
    Retourne l'objet solide (independant de l'objet regenere par
    CAD Sketcher, qui reste la source du profil)."""
    profile_obj = ensure_sketch_solid(context, sketch)
    if profile_obj is None or len(profile_obj.data.polygons) == 0:
        return None

    bm = bmesh.new()
    bm.from_mesh(profile_obj.data)
    bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=1e-6)

    # normale du profil (plan du sketch)
    bm.faces.ensure_lookup_table()
    normal = bm.faces[0].normal.copy()
    if normal.length < 0.5:
        normal = Vector((0, 0, 1))

    bottom_verts = list(bm.verts)
    bottom_boundary = [e for e in bm.edges if e.is_boundary]

    # extrusion
    res = bmesh.ops.extrude_face_region(bm, geom=list(bm.faces))
    new_geom = res["geom"]
    top_verts = [g for g in new_geom if isinstance(g, bmesh.types.BMVert)]
    offset = normal * depth
    if symmetric:
        bmesh.ops.translate(bm, verts=top_verts, vec=offset * 0.5)
        bmesh.ops.translate(bm, verts=bottom_verts, vec=-offset * 0.5)
    else:
        bmesh.ops.translate(bm, verts=top_verts, vec=offset)

    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)

    bottom_set = set(bottom_verts)
    top_set = set(top_verts)

    # aretes laterales : un sommet en bas, un en haut -> bevel weight
    key = bm.edges.layers.float.get("bevel_weight_edge")
    if key is None:
        key = bm.edges.layers.float.new("bevel_weight_edge")
    for e in bm.edges:
        va, vb = e.verts
        is_side = (va in bottom_set) != (vb in bottom_set)
        e[key] = 1.0 if is_side else 0.0

    # indices pour les vertex groups (avant to_mesh)
    bottom_idx = [v.index for v in bm.verts if v in bottom_set]
    top_idx = [v.index for v in bm.verts if v in top_set]

    name = f"Part_{sketch.name}"
    if existing is not None:
        obj = existing
        bm.to_mesh(obj.data)
    else:
        me = bpy.data.meshes.new(name)
        bm.to_mesh(me)
        obj = bpy.data.objects.new(name, me)
        context.collection.objects.link(obj)
    bm.free()
    obj.data.update()

    # vertex groups des couronnes
    for gname, idxs in (("rim_bottom", bottom_idx), ("rim_top", top_idx)):
        vg = obj.vertex_groups.get(gname)
        if vg is None:
            vg = obj.vertex_groups.new(name=gname)
        else:
            vg.remove(range(len(obj.data.vertices)))
        vg.add(idxs, 1.0, "REPLACE")

    # parametres pour la regeneration
    obj["slvs_pad_sketch"] = sketch.slvs_index
    obj["slvs_pad_depth"] = depth
    obj["slvs_pad_symmetric"] = symmetric

    # le profil CAD Sketcher devient une simple reference filaire
    profile_obj.display_type = "WIRE"
    profile_obj.hide_render = True
    return obj


def rebuild_pads(scene):
    """Regenere tous les solides Part_* dont le sketch a change."""
    context = bpy.context
    try:
        ents = scene.sketcher.entities
    except Exception:
        return
    for obj in scene.objects:
        if obj.type != "MESH" or "slvs_pad_sketch" not in obj:
            continue
        sketch = ents.get(obj["slvs_pad_sketch"])
        if sketch is None:
            continue
        try:
            build_pad_solid(context, sketch,
                            obj["slvs_pad_depth"],
                            bool(obj["slvs_pad_symmetric"]),
                            existing=obj)
        except Exception:
            continue


# ---------------------------------------------------------------------------
# Operateurs
# ---------------------------------------------------------------------------

class View3D_OT_slvs_feature_pad_solid(Operator):
    """Extrude the active closed sketch into a STRUCTURED solid:
    top/bottom edge rims and side edges are tagged independently
    so fillets can target each family separately"""

    bl_idname = Operators.FeaturePadSolid
    bl_label = "Pad Solid"
    bl_options = {"REGISTER", "UNDO"}

    depth: FloatProperty(
        name="Depth", default=0.1, min=0.0001, soft_max=10.0, unit="LENGTH")
    symmetric: BoolProperty(name="Symmetric", default=False)

    @classmethod
    def poll(cls, context):
        return (context.mode == "OBJECT"
                and get_active_sketch(context) is not None)

    def execute(self, context: Context):
        sketch = get_active_sketch(context)
        existing = context.scene.objects.get(f"Part_{sketch.name}")
        obj = build_pad_solid(context, sketch, self.depth,
                              self.symmetric, existing=existing)
        if obj is None:
            self.report({"ERROR"}, "Sketch conversion failed (closed contour?)")
            return {"CANCELLED"}
        self.report({"INFO"},
                    f"Pad Solid: {obj.name} "
                    f"({len(obj.data.polygons)} faces, rims tagged)")
        return {"FINISHED"}


class View3D_OT_slvs_feature_fillet_edges(Operator):
    """Fillet or chamfer ONE edge family of a Pad Solid:
    top rim, bottom rim or side edges - each with its own width"""

    bl_idname = Operators.FeatureFilletEdges
    bl_label = "Fillet Edges"
    bl_options = {"REGISTER", "UNDO"}

    edge_set: EnumProperty(
        name="Edges",
        items=[
            ("TOP", "Top Rim", "Contour edges of the top face"),
            ("BOTTOM", "Bottom Rim", "Contour edges of the bottom face"),
            ("SIDE", "Side Edges", "Vertical edges between the two faces"),
        ],
        default="TOP")
    width: FloatProperty(
        name="Width", default=0.01, min=0.0001, soft_max=1.0, unit="LENGTH")
    segments: IntProperty(name="Segments", default=3, min=1, max=12)
    chamfer: BoolProperty(name="Chamfer", default=False)

    def _targets(self, context):
        return [o for o in context.scene.objects
                if o.type == "MESH" and "slvs_pad_sketch" in o]

    @classmethod
    def poll(cls, context):
        ob = context.active_object
        return (context.mode == "OBJECT" and ob is not None
                and "slvs_pad_sketch" in ob)

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self, width=280)

    def execute(self, context: Context):
        obj = context.active_object
        name = f"SLVS_Fillet_{self.edge_set}"
        mod = obj.modifiers.get(name)
        if mod is None:
            mod = obj.modifiers.new(name, "BEVEL")
        mod.width = self.width
        mod.segments = 1 if self.chamfer else self.segments
        mod.miter_outer = "MITER_ARC"

        if self.edge_set == "SIDE":
            mod.limit_method = "WEIGHT"
        else:
            mod.limit_method = "VGROUP"
            mod.vertex_group = ("rim_top" if self.edge_set == "TOP"
                                else "rim_bottom")

        self.report({"INFO"},
                    f"{'Chamfer' if self.chamfer else 'Fillet'} "
                    f"{self.edge_set.lower()}: {obj.name}, "
                    f"width {self.width:.4g}")
        return {"FINISHED"}

    def draw(self, context):
        layout = self.layout
        layout.prop(self, "edge_set", expand=True)
        layout.prop(self, "width")
        layout.prop(self, "chamfer")
        if not self.chamfer:
            layout.prop(self, "segments")


# ---------------------------------------------------------------------------
# Regeneration parametrique : on enveloppe le convertisseur CAD Sketcher
# ---------------------------------------------------------------------------

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
            # deja dans un rebuild : ne pas boucler
            return result
        _busy = True
        try:
            rebuild_pads(scene)
        except Exception:
            pass
        finally:
            _busy = False
        return result

    conv.update_convertor_geometry = wrapped
    _wrapped = True


classes = (View3D_OT_slvs_feature_pad_solid,
           View3D_OT_slvs_feature_fillet_edges)
_register, _unregister = register_classes_factory(classes)


def register():
    _register()
    _wrap_converter()


def unregister():
    _unregister()
