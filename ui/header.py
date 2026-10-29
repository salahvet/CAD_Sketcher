"""Fork : ruban d'outils d'esquisse dans l'en-tete de la vue 3D.

Affiche dans la barre des reglages d'outil (View > Tool Settings), au centre,
uniquement quand une esquisse est active :

    Dessin | Modification | Transformation | Contraintes | Construction  Quitter
"""
from bpy.types import Context

from .. import declarations
from .. import icon_manager
from ..declarations import Operators, WorkSpaceTools

# (id de l'outil, nom de l'icone, libelle)
DRAW_TOOLS = (
    (WorkSpaceTools.AddPoint2D, "ops.bgs.add_point", "Point"),
    (WorkSpaceTools.AddLine2D, "ops.bgs.add_line", "Line"),
    (WorkSpaceTools.AddRectangle, "ops.bgs.add_rectangle", "Rectangle"),
    (WorkSpaceTools.AddCircle2D, "ops.bgs.add_circle", "Circle"),
    (WorkSpaceTools.AddEllipse2D, "ops.bgs.add_ellipse", "Ellipse"),
    (WorkSpaceTools.AddArc2D, "ops.bgs.add_arc", "Arc (center)"),
    (WorkSpaceTools.AddArc3P2D, "ops.bgs.add_arc_3p", "Arc (3 points)"),
    (WorkSpaceTools.AddBezier2D, "ops.bgs.add_bezier", "Bezier"),
)

MODIFY_TOOLS = (
    (WorkSpaceTools.Trim, "ops.bgs.trim", "Trim"),
    (WorkSpaceTools.Bevel, "ops.bgs.bevel", "Bevel"),
    (WorkSpaceTools.Offset, "ops.bgs.offset", "Offset"),
)

TRANSFORM_OPS = (
    (Operators.Move, "ops.bgs.move"),
    (Operators.Rotate, "ops.bgs.rotate"),
    (Operators.Scale, "ops.bgs.scale"),
    (Operators.Mirror, "ops.bgs.mirror"),
    (Operators.ArrayLinear, "ops.bgs.array_linear"),
    (Operators.MergePoints, "ops.bgs.merge_points"),
)


def _active_tool_id(context: Context):
    try:
        return context.workspace.tools.from_space_view3d_mode(context.mode).idname
    except Exception:
        return ""


def _tool_buttons(layout, tools, active):
    row = layout.row(align=True)
    for idname, icon, label in tools:
        op = row.operator(
            "wm.tool_set_by_id",
            text="",
            icon_value=icon_manager.get_ui_icon(icon),
            depress=(active == idname),
        )
        op.name = idname
    return row


def draw_sketch_ribbon(self, context: Context):
    sketcher = getattr(context.scene, "sketcher", None)
    if not sketcher or not sketcher.active_sketch:
        return

    layout = self.layout
    layout.operator_context = "INVOKE_DEFAULT"
    active = _active_tool_id(context)

    layout.separator_spacer()

    _tool_buttons(layout, DRAW_TOOLS, active)
    layout.separator()
    _tool_buttons(layout, MODIFY_TOOLS, active)
    layout.separator()

    row = layout.row(align=True)
    for op, icon in TRANSFORM_OPS:
        row.operator(op, text="", icon_value=icon_manager.get_ui_icon(icon))
    layout.separator()

    row = layout.row(align=True)
    for op in declarations.ConstraintOperators:
        row.operator(op, text="", icon_value=icon_manager.get_constraint_icon(op))
    layout.separator()

    layout.prop(sketcher, "use_construction", text="", icon="MOD_WIREFRAME")
    layout.operator(
        Operators.SetActiveSketch, text="", icon="BACK", depress=True
    ).index = -1

    layout.separator_spacer()
