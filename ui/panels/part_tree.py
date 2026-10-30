import json

from bpy.types import Context

from ... import declarations
from . import VIEW3D_PT_sketcher_base


_ICONS = {"pad": "MESH_CUBE", "pocket": "MOD_BOOLEAN",
          "fillet": "MOD_BEVEL", "chamfer": "MOD_BEVEL", "pattern": "MOD_ARRAY"}


class VIEW3D_PT_sketcher_part_tree(VIEW3D_PT_sketcher_base):
    """Build tree of the OCCT parts (fork)"""

    bl_label = "Part Tree (OCCT)"
    bl_idname = declarations.Panels.SketcherPartTree

    def draw(self, context: Context):
        layout = self.layout
        raw = context.scene.get("occt_recipes", "{}")
        try:
            recipes = json.loads(raw)
        except Exception:
            recipes = {}

        if not recipes:
            layout.label(text="No OCCT part yet", icon="INFO")
            layout.label(text="Use Functions > OCCT Pad")
            return

        for part_name, recipe in recipes.items():
            box = layout.box()
            row = box.row()
            row.label(text=part_name, icon="OBJECT_DATA")
            for i, feat in enumerate(recipe):
                op = feat.get("op", "?")
                if op == "pattern":
                    src = feat.get("feature")
                    what = f"#{src + 1}" if src is not None else "(source removed)"
                    if feat.get("kind") == "circular":
                        how = f"{feat.get('angle', 360):g}°"
                    else:
                        how = f"step {feat.get('spacing', 0) * 1000:.3g} mm"
                    txt = f"{i+1}. Pattern x{feat.get('count')} of {what}  {how}"
                elif op in ("pad", "pocket"):
                    txt = (f"{i+1}. {op.capitalize()}  "
                           f"[{feat.get('sketch','?')}]  "
                           f"d={feat.get('depth', 0):.3g}")
                else:
                    txt = (f"{i+1}. {op.capitalize()}  "
                           f"({feat.get('select','?')})  "
                           f"r={feat.get('radius', 0):.3g}")
                row = box.row(align=True)
                row.label(text=txt, icon=_ICONS.get(op, "DOT"))
                p = row.operator(declarations.Operators.OCCTRemoveFeature,
                                 text="", icon="TRASH")
                p.part = part_name
                p.index = i

        layout.operator(declarations.Operators.OCCTPickEdges,
                        icon="RESTRICT_SELECT_OFF")
        layout.operator(declarations.Operators.OCCTRecompute,
                        icon="FILE_REFRESH")
