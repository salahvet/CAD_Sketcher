from pathlib import Path
from functools import cache

import gpu
import bpy
import bpy.utils.previews
from gpu_extras.batch import batch_for_shader
from bpy.app import background

from .declarations import Operators
from .shaders import Shaders

icons = {}
preview_icons = None
# Fork : icones PNG des outils et transformations (ruban, boutons)
ui_icons = None
_operator_types = {
    Operators.AddDistance: "DISTANCE",
    Operators.AddDiameter: "DIAMETER",
    Operators.AddAngle: "ANGLE",
    Operators.AddCoincident: "COINCIDENT",
    Operators.AddEqual: "EQUAL",
    Operators.AddVertical: "VERTICAL",
    Operators.AddHorizontal: "HORIZONTAL",
    Operators.AddParallel: "PARALLEL",
    Operators.AddPerpendicular: "PERPENDICULAR",
    Operators.AddTangent: "TANGENT",
    Operators.AddMidPoint: "MIDPOINT",
    Operators.AddRatio: "RATIO",
    Operators.AddSymmetry: "SYMMETRY",
}

@cache
def _get_shader():
    return Shaders.uniform_color_image_2d()

@cache
def _get_batch():
    return batch_for_shader(_get_shader(), "TRI_FAN", {
    "pos": ((-.5, -.5), (.5, -.5), (.5, .5), (-.5, .5)),
    "texCoord": ((0, 0), (1, 0), (1, 1), (0, 1)),
})

def get_folder_path():
    return Path(__file__).parent / "resources" / "icons"


def get_icon(name: str):
    return str(get_folder_path() / name)


def load_icon(type, icon):
    size, pixels = icon.icon_size, icon.icon_pixels_float
    buffer = gpu.types.Buffer('FLOAT', (1, len(pixels)), [pixels])
    texture = gpu.types.GPUTexture(size=size, data=buffer)
    icons[type] = texture


def load():
    global preview_icons

    if preview_icons:
        return

    if background:
        return

    load_preview_icons()
    load_ui_icons()

    for operator, icon in preview_icons.items():
        load_icon(_operator_types[operator], icon)


def unload():
    global icons
    unload_preview_icons()
    unload_ui_icons()
    icons = {}


def load_preview_icons():
    global preview_icons

    if preview_icons:
        return

    preview_icons = bpy.utils.previews.new()

    for operator, type in _operator_types.items():
        icon_path = get_folder_path() / f"{type}.png"

        if not icon_path.exists():
            continue

        preview_icons.load(operator, str(icon_path), 'IMAGE')


def unload_preview_icons():
    global preview_icons

    if not preview_icons:
        return

    preview_icons.clear()
    bpy.utils.previews.remove(preview_icons)
    preview_icons = None


def get_constraint_icon(operator: str):
    if not preview_icons:
        return -1

    icon = preview_icons.get(operator)

    if not icon:
        return -1

    return icon.icon_id


def draw(type, color):
    texture = icons.get(type)

    if not texture:
        return

    gpu.state.blend_set("ALPHA")

    shader, batch = _get_shader(), _get_batch()
    shader.bind()
    shader.uniform_float("color", color)
    shader.uniform_sampler("image", texture)
    batch.draw(shader)

    gpu.state.blend_set("NONE")


# --- Fork : icones d'interface ------------------------------------------------

def load_ui_icons():
    """Charge resources/icons/png/*.png, cle = nom sans extension
    (ex. "ops.bgs.add_ellipse")."""
    global ui_icons
    if ui_icons or background:
        return
    ui_icons = bpy.utils.previews.new()
    folder = get_folder_path() / "png"
    if not folder.exists():
        return
    for f in sorted(folder.glob("*.png")):
        ui_icons.load(f.stem, str(f), "IMAGE")


def unload_ui_icons():
    global ui_icons
    if not ui_icons:
        return
    ui_icons.clear()
    bpy.utils.previews.remove(ui_icons)
    ui_icons = None


def get_ui_icon(name: str) -> int:
    """icon_value utilisable dans layout.operator(..., icon_value=...)"""
    if not ui_icons:
        load_ui_icons()
    icon = ui_icons.get(name) if ui_icons else None
    return icon.icon_id if icon else 0
