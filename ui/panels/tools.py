from bpy.types import Context

from .. import declarations
from .. import icon_manager
from ...utilities.preferences import is_experimental
from . import VIEW3D_PT_sketcher_base


class VIEW3D_PT_sketcher_tools(VIEW3D_PT_sketcher_base):
    """
    Tools Menu: List of useful tools for sketching
    """

    bl_label = "Tools"
    bl_idname = declarations.Panels.SketcherTools
    bl_options = {"DEFAULT_CLOSED"}

    def draw(self, context: Context):
        layout = self.layout

        # Selection
        layout.label(text="Select:")
        row = layout.row(align=True)
        row.operator(declarations.Operators.SelectAll,
                     text="All").deselect = False
        row.operator(declarations.Operators.SelectAll,
                     text="None").deselect = True
        row.operator(declarations.Operators.SelectInvert, text="Invert")
        layout.operator(declarations.Operators.SelectExtendAll,
                        text="Select Chain")

        layout.separator()
        layout.operator(declarations.Operators.MergePoints)

        # --- correctif PureMed : rotation ---
        layout.separator()
        layout.label(text="Transform:")
        col = layout.column(align=True)
        col.enabled = bool(
            len(context.scene.sketcher.entities.selected_active))
        col.operator(declarations.Operators.Move,
                     text="Move", icon='ORIENTATION_GLOBAL')
        col.operator(declarations.Operators.Rotate,
                     text="Rotate",
                     icon='DRIVER_ROTATIONAL_DIFFERENCE')
        col.operator(declarations.Operators.Scale,
                     text="Scale", icon='MOD_MESHDEFORM')
        layout.menu(declarations.Menus.FunctionsMenu)

        # Constraints
        layout.label(text="Constraints:")
        col = layout.column(align=True)

        for op in declarations.ConstraintOperators:
            col.operator(op, icon_value=icon_manager.get_constraint_icon(op))

        # Fork : automatisation sur le dessin de l'utilisateur
        col = layout.column(align=True)
        col.operator(declarations.Operators.AutoConstraints, icon="AUTO")
        col.operator(declarations.Operators.AutoDimensions, icon="DRIVER_DISTANCE")

        layout.separator()

        # Drawing
        layout.label(text="Drawing:")
        layout.prop(context.scene.sketcher, "use_construction")

        # Node modifier operators
        if is_experimental():
            layout.label(text="Node Tools:")
            col = layout.column(align=True)
            #col.operator(declarations.Operators.NodeFill)
            col.operator(declarations.Operators.NodeExtrude)
            col.operator(declarations.Operators.NodeArrayLinear)
