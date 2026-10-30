from bpy.types import Menu

from ..declarations import Menus, Operators


class VIEW3D_MT_sketcher_functions(Menu):
    """Extra sketch functions (fork)"""

    bl_label = "Functions"
    bl_idname = Menus.FunctionsMenu

    def draw(self, context):
        layout = self.layout
        layout.operator_context = "INVOKE_DEFAULT"
        layout.operator(Operators.ArrayLinear, icon="MOD_ARRAY")
        layout.operator(Operators.Mirror, icon="MOD_MIRROR")
        layout.separator()
        layout.label(text="3D Features")
        layout.label(text="OCCT Part (parametric)")
        layout.operator(Operators.OCCTPad, icon="MESH_CUBE")
        layout.operator(Operators.OCCTPocket, icon="MOD_BOOLEAN")
        layout.operator(Operators.OCCTPickEdges, icon="RESTRICT_SELECT_OFF")
        layout.operator(Operators.OCCTFillet, icon="MOD_BEVEL")
        layout.operator(Operators.OCCTPattern, icon="MOD_ARRAY")
        layout.operator(Operators.OCCTMirror, icon="MOD_MIRROR")
        layout.operator(Operators.OCCTDerive, icon="DUPLICATE")
        layout.operator(Operators.OCCTRemoveLast, icon="TRASH")
        layout.separator()
        layout.operator(Operators.FeaturePadSolid, icon="MESH_CUBE")
        layout.operator(Operators.FeatureFilletEdges, icon="MOD_BEVEL")
        layout.separator()
        layout.operator(Operators.FeaturePad, icon="MOD_SOLIDIFY")
        layout.operator(Operators.FeaturePocket, icon="MOD_BOOLEAN")
        layout.operator(Operators.FeatureFillet, icon="MOD_BEVEL")
