# Please keep this file in alphabetical order
from enum import Enum


class Gizmos(str, Enum):
    Angle = "VIEW3D_GT_slvs_angle"
    Constraint = "VIEW3D_GT_slvs_constraint"
    ConstraintValue = "VIEW3D_GT_slvs_constraint_value"
    Diameter = "VIEW3D_GT_slvs_diameter"
    Distance = "VIEW3D_GT_slvs_distance"
    Preselection = "VIEW3D_GT_slvs_preselection"


class GizmoGroups(str, Enum):
    Angle = "VIEW3D_GGT_slvs_angle"
    Constraint = "VIEW3D_GGT_slvs_constraint"
    Diameter = "VIEW3D_GGT_slvs_diameter"
    Distance = "VIEW3D_GGT_slvs_distance"
    Preselection = "VIEW3D_GGT_slvs_preselection"


class Operators(str, Enum):
    AddAngle = "view3d.slvs_add_angle"
    AddArc2D = "view3d.slvs_add_arc2d"
    AddArc3P2D = "view3d.slvs_add_arc_3p_2d"
    AddBezier2D = "view3d.slvs_add_bezier2d"
    AddEllipse2D = "view3d.slvs_add_ellipse_2d"
    ArrayLinear = "view3d.slvs_array_linear"
    Mirror = "view3d.slvs_mirror"
    AddCircle2D = "view3d.slvs_add_circle2d"
    AddCoincident = "view3d.slvs_add_coincident"
    AddDiameter = "view3d.slvs_add_diameter"
    AddDistance = "view3d.slvs_add_distance"
    AddEqual = "view3d.slvs_add_equal"
    AddHorizontal = "view3d.slvs_add_horizontal"
    AddLine2D = "view3d.slvs_add_line2d"
    AddLine3D = "view3d.slvs_add_line3d"
    AddMidPoint = "view3d.slvs_add_midpoint"
    AddParallel = "view3d.slvs_add_parallel"
    AddPerpendicular = "view3d.slvs_add_perpendicular"
    AddPoint2D = "view3d.slvs_add_point2d"
    AddPoint3D = "view3d.slvs_add_point3d"
    AddPresetTheme = "bgs.theme_preset_add"
    AddRatio = "view3d.slvs_add_ratio"
    AddSymmetry = "view3d.slvs_add_symmetry"
    AddRectangle = "view3d.slvs_add_rectangle"
    AddRectangleCenter = "view3d.slvs_add_rectangle_center"
    Rotate = "view3d.slvs_rotate"
    Scale = "view3d.slvs_scale"
    AddSketch = "view3d.slvs_add_sketch"
    AddTangent = "view3d.slvs_add_tangent"
    AddVertical = "view3d.slvs_add_vertical"
    AddWorkPlane = "view3d.slvs_add_workplane"
    AddWorkPlaneFace = "view3d.slvs_add_workplane_face"
    AlignWorkplaneCursor = "view3d.slvs_align_workplane_cursor"
    AlignView = "view3d.slvs_align_view"
    AutoConstraints = "view3d.slvs_auto_constraints"
    AutoDimensions = "view3d.slvs_auto_dimensions"
    ArrangeDimensions = "view3d.slvs_arrange_dimensions"
    FullyConstrain = "view3d.slvs_fully_constrain"
    BatchSet = "view3d.slvs_batch_set"
    ContextMenu = "view3d.slvs_context_menu"
    Copy = "view3d.slvs_copy"
    DeleteConstraint = "view3d.slvs_delete_constraint"
    DeleteEntity = "view3d.slvs_delete_entity"
    InstallPackage = "view3d.slvs_install_package"
    MergePoints = "view3d.slvs_merge_points"
    FeaturePad = "view3d.slvs_feature_pad"
    FeaturePocket = "view3d.slvs_feature_pocket"
    FeatureFillet = "view3d.slvs_feature_fillet"
    FeaturePadSolid = "view3d.slvs_feature_pad_solid"
    FeatureFilletEdges = "view3d.slvs_feature_fillet_edges"
    OCCTPad = "view3d.slvs_occt_pad"
    OCCTPocket = "view3d.slvs_occt_pocket"
    OCCTFillet = "view3d.slvs_occt_fillet"
    OCCTPattern = "view3d.slvs_occt_pattern"
    OCCTDerive = "view3d.slvs_occt_derive"
    OCCTMirror = "view3d.slvs_occt_mirror"
    OCCTToggleFeature = "view3d.slvs_occt_toggle_feature"
    OCCTEditFeature = "view3d.slvs_occt_edit_feature"
    OCCTProjectEdges = "view3d.slvs_occt_project_edges"
    OCCTPickRegions = "view3d.slvs_occt_pick_regions"
    AssemblyPivot = "view3d.slvs_assembly_pivot"
    CheckInterference = "view3d.slvs_check_interference"
    OCCTRemoveLast = "view3d.slvs_occt_remove_last"
    OCCTRemoveFeature = "view3d.slvs_occt_remove_feature"
    OCCTRecompute = "view3d.slvs_occt_recompute"
    OCCTTidyView = "view3d.slvs_occt_tidy_view"
    OCCTQuadMesh = "view3d.slvs_occt_quad_mesh"
    OCCTPickEdges = "view3d.slvs_occt_pick_edges"
    Paste = "view3d.slvs_paste"
    Move = "view3d.slvs_move"
    Offset = "view3d.slvs_offset"
    NodeFill = "view3d.slvs_node_fill"
    NodeExtrude = "view3d.slvs_node_extrude"
    NodeArrayLinear = "view3d.slvs_node_array_linear"
    RegisterDrawCB = "view3d.slvs_register_draw_cb"
    Restore = "view3d.slvs_restore"
    Select = "view3d.slvs_select"
    SelectAll = "view3d.slvs_select_all"
    SelectBox = "view3d.slvs_select_box"
    SelectInvert = "view3d.slvs_select_invert"
    SelectExtendAll = "view3d.slvs_select_extend_all"
    SelectExtend = "view3d.slvs_select_extend"
    SetActiveSketch = "view3d.slvs_set_active_sketch"
    SetAllConstraintsVisibility = "view3d.slvs_set_all_constraints_visibility"
    ShowSolverState = "view3d.slvs_show_solver_state"
    Snapshot = "view3d.slvs_snapshot"
    Solve = "view3d.slvs_solve"
    Update = "view3d.slvs_update"
    Trim = "view3d.slvs_trim"
    Bevel = "view3d.slvs_bevel"
    Tweak = "view3d.slvs_tweak"
    TweakConstraintValuePos = "view3d.slvs_tweak_constraint_value_pos"
    UnregisterDrawCB = "view3d.slvs_unregister_draw_cb"
    WriteSelectionTexture = "view3d.slvs_write_selection_texture"


class Macros(str, Enum):
    DuplicateMove = "view3d.slvs_duplicate_move"


class Menus(str, Enum):
    SelectedMenu = "VIEW3D_MT_selected_menu"
    FunctionsMenu = "VIEW3D_MT_sketcher_functions"


class Panels(str, Enum):
    SketcherPartTree = "VIEW3D_PT_sketcher_part_tree"
    Sketcher = "VIEW3D_PT_sketcher"
    SketcherDebugPanel = "VIEW3D_PT_sketcher_debug_panel"
    SketcherTools = "VIEW3D_PT_sketcher_tools"
    SketcherConstraints = "VIEW3D_PT_sketcher_constraints"
    SketcherEntities = "VIEW3D_PT_sketcher_entities"


class VisibilityTypes(str, Enum):
    Hide = "HIDE"
    Show = "SHOW"


class WorkSpaceTools(str, Enum):
    AddArc2D = "sketcher.slvs_add_arc2d"
    AddArc3P2D = "sketcher.slvs_add_arc_3p_2d"
    AddBezier2D = "sketcher.slvs_add_bezier2d"
    AddEllipse2D = "sketcher.slvs_add_ellipse_2d"
    AddCircle2D = "sketcher.slvs_add_circle2d"
    AddLine2D = "sketcher.slvs_add_line2d"
    AddLine3D = "sketcher.slvs_add_line3d"
    AddPoint2D = "sketcher.slvs_add_point2d"
    AddPoint3D = "sketcher.slvs_add_point3d"
    AddRectangle = "sketcher.slvs_add_rectangle"
    AddRectangleCenter = "sketcher.slvs_add_rectangle_center"
    AddWorkplane = "sketcher.slvs_add_workplane"
    AddWorkplaneFace = "sketcher.slvs_add_workplane_face"
    Offset = "sketcher.slvs_offset"
    Select = "sketcher.slvs_select"
    Trim = "sketcher.slvs_trim"
    Bevel = "sketcher.slvs_bevel"


ConstraintOperators = (
    Operators.AddDistance,
    Operators.AddDiameter,
    Operators.AddAngle,
    Operators.AddCoincident,
    Operators.AddEqual,
    Operators.AddVertical,
    Operators.AddHorizontal,
    Operators.AddParallel,
    Operators.AddPerpendicular,
    Operators.AddTangent,
    Operators.AddMidPoint,
    Operators.AddRatio,
    Operators.AddSymmetry,
)
