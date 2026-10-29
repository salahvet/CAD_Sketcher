from bpy.types import WorkSpaceTool

from ..icon_manager import get_icon
from ..declarations import GizmoGroups, Operators, WorkSpaceTools
from ..keymaps import tool_generic
from ..stateful_operator.tool import GenericStateTool
from ..stateful_operator.utilities.keymap import operator_access


class VIEW3D_T_slvs_add_rectangle(GenericStateTool, WorkSpaceTool):
    bl_space_type = "VIEW_3D"
    bl_context_mode = "OBJECT"
    bl_idname = WorkSpaceTools.AddRectangle
    bl_label = "Add Rectangle"
    bl_operator = Operators.AddRectangle
    bl_icon = get_icon("ops.bgs.add_rectangle")
    bl_widget = GizmoGroups.Preselection
    bl_keymap = (
        *tool_generic,
        *operator_access(Operators.AddRectangle),
    )

    def draw_settings(context, layout, tool):
        layout.prop(context.scene.sketcher, "auto_axis_constraints", text="Auto Constraints")
