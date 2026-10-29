"""Fork : barre d'outils flottante en haut de la vue 3D (mode esquisse).

Chaque bouton est un gizmo 2D : il dessine son icone (texture GPU) a la
taille voulue, se met en surbrillance au survol, affiche son nom et lance
son operateur au clic (target_set_operator). Les groupes passent a la
ligne quand la vue est trop etroite.

Taille et affichage : Preferences de l'add-on > Ribbon.
"""
import blf
import bpy
import gpu
import numpy as np
from bpy.types import Gizmo, GizmoGroup
from gpu_extras.batch import batch_for_shader
from mathutils import Matrix

from .. import declarations, icon_manager
from ..declarations import Operators, WorkSpaceTools
from ..utilities.preferences import get_prefs

BASE = 20  # taille d'un bouton d'en-tete Blender, avant agrandissement

# (type, cle, icone, libelle)
#   type : "tool" -> wm.tool_set_by_id, "op" -> operateur, "toggle", "leave"
DRAW = (
    ("tool", WorkSpaceTools.Select, "ops.bgs.select", "Select"),
    ("tool", WorkSpaceTools.AddPoint2D, "ops.bgs.add_point", "Point"),
    ("tool", WorkSpaceTools.AddLine2D, "ops.bgs.add_line", "Line"),
    ("tool", WorkSpaceTools.AddRectangle, "ops.bgs.add_rectangle", "Rectangle"),
    ("tool", WorkSpaceTools.AddRectangleCenter, "ops.bgs.add_rectangle_center", "Center Rectangle"),
    ("tool", WorkSpaceTools.AddCircle2D, "ops.bgs.add_circle", "Circle"),
    ("tool", WorkSpaceTools.AddEllipse2D, "ops.bgs.add_ellipse", "Ellipse"),
    ("tool", WorkSpaceTools.AddArc2D, "ops.bgs.add_arc", "Arc (center)"),
    ("tool", WorkSpaceTools.AddArc3P2D, "ops.bgs.add_arc_3p", "Arc (3 points)"),
    ("tool", WorkSpaceTools.AddBezier2D, "ops.bgs.add_bezier", "Bezier"),
)
MODIFY = (
    ("tool", WorkSpaceTools.Trim, "ops.bgs.trim", "Trim"),
    ("tool", WorkSpaceTools.Bevel, "ops.bgs.bevel", "Bevel"),
    ("tool", WorkSpaceTools.Offset, "ops.bgs.offset", "Offset"),
)
TRANSFORM = (
    ("op", Operators.Move, "ops.bgs.move", "Move"),
    ("op", Operators.Rotate, "ops.bgs.rotate", "Rotate"),
    ("op", Operators.Scale, "ops.bgs.scale", "Scale"),
    ("op", Operators.Mirror, "ops.bgs.mirror", "Mirror"),
    ("op", Operators.ArrayLinear, "ops.bgs.array_linear", "Linear Array"),
    ("op", Operators.MergePoints, "ops.bgs.merge_points", "Merge Points"),
)


def _constraint_group():
    out = []
    for op in declarations.ConstraintOperators:
        label = op.value.split("slvs_add_")[-1].replace("_", " ").title()
        out.append(("constraint", op, op, label))
    return tuple(out)


MISC = (
    ("toggle", "scene.sketcher.use_construction", "ops.bgs.construction", "Construction Mode"),
    ("leave", Operators.SetActiveSketch, "ops.bgs.leave_sketch", "Leave Sketch"),
)


def groups():
    return (DRAW, MODIFY, TRANSFORM, _constraint_group(), MISC)


# --- textures ----------------------------------------------------------------

_textures = {}


def _texture(kind, key):
    """Texture GPU d'une icone, chargee une fois depuis les previews."""
    cache_key = (kind, str(key))
    if cache_key in _textures:
        return _textures[cache_key]
    tex = None
    try:
        if kind == "constraint":
            if not icon_manager.preview_icons:
                icon_manager.load_preview_icons()
            prev = icon_manager.preview_icons.get(key)
        else:
            if not icon_manager.ui_icons:
                icon_manager.load_ui_icons()
            prev = icon_manager.ui_icons.get(key)
        if prev:
            size, pixels = tuple(prev.image_size), prev.image_pixels_float
            if not size[0]:
                size, pixels = tuple(prev.icon_size), prev.icon_pixels_float
            if size[0]:
                arr = np.empty(size[0] * size[1] * 4, dtype=np.float32)
                pixels.foreach_get(arr)
                buf = gpu.types.Buffer("FLOAT", len(arr), arr)
                tex = gpu.types.GPUTexture(size=size, format="RGBA16F", data=buf)
    except Exception:
        tex = None
    _textures[cache_key] = tex
    return tex


def clear_textures():
    _textures.clear()


_quad_batch = {}


def _rect_batch(shader):
    key = id(shader)
    if key not in _quad_batch:
        _quad_batch[key] = batch_for_shader(
            shader, "TRI_FAN", {"pos": ((0, 0), (1, 0), (1, 1), (0, 1))}
        )
    return _quad_batch[key]


def _image_batch(shader):
    key = ("img", id(shader))
    if key not in _quad_batch:
        _quad_batch[key] = batch_for_shader(shader, "TRI_FAN", {
            "pos": ((0, 0), (1, 0), (1, 1), (0, 1)),
            "texCoord": ((0, 0), (1, 0), (1, 1), (0, 1)),
        })
    return _quad_batch[key]


def _pixel_space(region):
    """Projection orthographique en pixels de la region."""
    w, h = region.width, region.height
    return Matrix((
        (2.0 / w, 0, 0, -1),
        (0, 2.0 / h, 0, -1),
        (0, 0, 1, 0),
        (0, 0, 0, 1),
    ))


def _draw_rect(x, y, w, h, color):
    shader = gpu.shader.from_builtin("UNIFORM_COLOR")
    with gpu.matrix.push_pop():
        gpu.matrix.multiply_matrix(
            Matrix.Translation((x, y, 0)) @ Matrix.Diagonal((w, h, 1, 1))
        )
        shader.bind()
        shader.uniform_float("color", color)
        _rect_batch(shader).draw(shader)


def _draw_round_rect(x, y, w, h, r, color, seg=6):
    import math
    r = min(r, w / 2, h / 2)
    pts = [(x + w / 2, y + h / 2)]
    corners = ((x + w - r, y + r, -90), (x + w - r, y + h - r, 0),
               (x + r, y + h - r, 90), (x + r, y + r, 180))
    for cx, cy, a0 in corners:
        for i in range(seg + 1):
            a = math.radians(a0 + 90 * i / seg)
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    pts.append(pts[1])
    shader = gpu.shader.from_builtin("UNIFORM_COLOR")
    shader.bind()
    shader.uniform_float("color", color)
    batch_for_shader(shader, "TRI_FAN", {"pos": pts}).draw(shader)


BAND_COLOR = (0.13, 0.13, 0.13, 0.55)


# --- bouton ------------------------------------------------------------------

class VIEW3D_GT_slvs_ribbon_button(Gizmo):
    bl_idname = "VIEW3D_GT_slvs_ribbon_button"

    def setup(self):
        self.spec = None
        self.rect = (0, 0, 0, 0)
        self.active = False
        self.enabled = True
        self.band = None

    def test_select(self, context, location):
        x, y, w, h = self.rect
        if not self.enabled or self.hide:
            return -1
        mx, my = location
        return 0 if (x <= mx <= x + w and y <= my <= y + h) else -1

    def draw(self, context):
        if self.spec is None or self.hide:
            return
        kind, key, icon, label = self.spec
        x, y, w, h = self.rect
        region = context.region

        gpu.state.blend_set("ALPHA")
        gpu.state.depth_test_set("NONE")
        with gpu.matrix.push_pop(), gpu.matrix.push_pop_projection():
            gpu.matrix.load_identity()
            gpu.matrix.load_projection_matrix(_pixel_space(region))

            if self.band:
                bx, by, bw, bh = self.band
                _draw_round_rect(bx, by, bw, bh, bh * 0.22, BAND_COLOR)

            radius = w * 0.18
            if self.active:
                _draw_round_rect(x, y, w, h, radius, (0.28, 0.45, 0.72, 0.95))
            elif self.is_highlight:
                _draw_round_rect(x, y, w, h, radius, (1.0, 1.0, 1.0, 0.16))

            tex = _texture("constraint" if kind == "constraint" else "ui", icon)
            if tex is not None:
                pad = w * 0.1
                alpha = 1.0 if self.enabled else 0.3
                shader = icon_manager._get_shader()
                with gpu.matrix.push_pop():
                    gpu.matrix.multiply_matrix(
                        Matrix.Translation((x + pad, y + pad, 0))
                        @ Matrix.Diagonal((w - 2 * pad, h - 2 * pad, 1, 1))
                    )
                    shader.bind()
                    shader.uniform_float("color", (1, 1, 1, alpha))
                    shader.uniform_sampler("image", tex)
                    _image_batch(shader).draw(shader)

            if self.is_highlight:
                _draw_tooltip(label, x + w / 2, y - 6, w)
        gpu.state.blend_set("NONE")


def _draw_tooltip(text, cx, top, button_size):
    font = 0
    size = max(11, int(button_size * 0.26))
    blf.size(font, size)
    tw, th = blf.dimensions(font, text)
    pad = size * 0.45
    x = cx - tw / 2
    y = top - th - 2 * pad
    _draw_rect(x - pad, y - pad, tw + 2 * pad, th + 2 * pad, (0.08, 0.08, 0.08, 0.9))
    blf.color(font, 0.95, 0.95, 0.95, 1.0)
    blf.position(font, x, y, 0)
    blf.draw(font, text)


# --- groupe ------------------------------------------------------------------

def _top_offset(context):
    """Hauteur des en-tetes superposes a la vue (Region Overlap)."""
    win = context.region
    off = 0
    for r in context.area.regions:
        if r.type in {"HEADER", "TOOL_HEADER"} and r.height > 1:
            if win.y <= r.y < win.y + win.height:  # en-tete au-dessus de la vue
                off += r.height
    return off


def _side_offsets(context):
    left = right = 0
    win = context.region
    for r in context.area.regions:
        if r.width <= 1 or not (win.x <= r.x < win.x + win.width):
            continue
        if r.type == "TOOLS":
            left = max(left, r.width)
        elif r.type == "UI":
            right = max(right, r.width)
    return left, right


class VIEW3D_GGT_slvs_ribbon(GizmoGroup):
    bl_idname = "VIEW3D_GGT_slvs_ribbon"
    bl_label = "Sketcher Ribbon"
    bl_space_type = "VIEW_3D"
    bl_region_type = "WINDOW"
    bl_options = {"PERSISTENT", "SHOW_MODAL_ALL"}

    @classmethod
    def poll(cls, context):
        try:
            if not get_prefs().show_ribbon:
                return False
        except Exception:
            return False
        sketcher = getattr(context.scene, "sketcher", None)
        return bool(sketcher and sketcher.active_sketch)

    def setup(self, context):
        self.buttons = []
        for group in groups():
            row = []
            for spec in group:
                gz = self.gizmos.new(VIEW3D_GT_slvs_ribbon_button.bl_idname)
                gz.spec = spec
                kind, key, icon, label = spec
                if kind == "tool":
                    gz.target_set_operator("wm.tool_set_by_id").name = key
                elif kind == "toggle":
                    gz.target_set_operator("wm.context_toggle").data_path = key
                elif kind == "leave":
                    gz.target_set_operator(key).index = -1
                else:
                    gz.target_set_operator(key)
                gz.use_draw_modal = True
                row.append(gz)
            self.buttons.append(row)

    def draw_prepare(self, context):
        region = context.region
        prefs = get_prefs()
        ui = context.preferences.system.ui_scale
        size = BASE * prefs.ribbon_scale * ui
        gap = size * 0.08
        group_gap = size * 0.55
        band_pad = size * 0.14
        margin = size * 0.25

        try:
            active = context.workspace.tools.from_space_view3d_mode(
                context.mode
            ).idname
        except Exception:
            active = ""
        construction = context.scene.sketcher.use_construction

        left, right = _side_offsets(context)
        # place pour le texte de vue (haut gauche) et le gizmo de navigation
        # (haut droite), pour ne pas les recouvrir
        left += 130 * ui
        right += 110 * ui
        avail = region.width - left - right - 2 * margin

        # repartition des groupes en lignes
        lines, cur, cur_w = [], [], 0.0
        for row in self.buttons:
            w = len(row) * size + (len(row) - 1) * gap
            add = w if not cur else w + group_gap
            if cur and cur_w + add > avail:
                lines.append((cur, cur_w))
                cur, cur_w = [], 0.0
                add = w
            cur.append(row)
            cur_w += add
        if cur:
            lines.append((cur, cur_w))

        y = region.height - _top_offset(context) - margin - size
        for rows, width in lines:
            x = left + margin + max(0.0, (avail - width) / 2)
            for row in rows:
                row_w = len(row) * size + (len(row) - 1) * gap
                for k, gz in enumerate(row):
                    # la bande grise du groupe est dessinee par son 1er bouton
                    gz.band = (
                        (x - band_pad, y - band_pad, row_w + 2 * band_pad,
                         size + 2 * band_pad) if k == 0 else None
                    )
                    kind, key, icon, label = gz.spec
                    gz.rect = (x, y, size, size)
                    gz.active = (kind == "tool" and active == key) or (
                        kind == "toggle" and construction
                    )
                    gz.enabled = True
                    if kind == "op":
                        try:
                            mod, name = str(key).split(".")
                            gz.enabled = getattr(getattr(bpy.ops, mod), name).poll()
                        except Exception:
                            gz.enabled = True
                    x += size + gap
                x += group_gap - gap
            y -= size + 2 * band_pad + gap * 2


classes = (VIEW3D_GT_slvs_ribbon_button, VIEW3D_GGT_slvs_ribbon)
