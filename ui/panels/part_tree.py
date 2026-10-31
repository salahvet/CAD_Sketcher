import json

from bpy.types import Context

from ... import declarations
from . import VIEW3D_PT_sketcher_base


_ICONS = {"pad": "MESH_CUBE", "pocket": "MOD_BOOLEAN",
          "fillet": "MOD_BEVEL", "chamfer": "MOD_BEVEL", "pattern": "MOD_ARRAY",
          "derive": "DUPLICATE", "mirror": "MOD_MIRROR"}


def _feature_text(i, feat):
    op = feat.get("op", "?")
    if op == "pattern":
        src = feat.get("feature")
        what = f"#{src + 1}" if src is not None else "(source removed)"
        if feat.get("kind") == "circular":
            how = f"{feat.get('angle', 360):g}°"
        else:
            how = f"step {feat.get('spacing', 0) * 1000:.3g} mm"
        return f"{i+1}. Pattern x{feat.get('count')} of {what}  {how}"
    if op == "derive":
        m = feat.get("mirror", "NONE")
        return f"{i+1}. Derived from {feat.get('from', '?')}" + (f"  (mirror {m})" if m != "NONE" else "")
    if op == "mirror":
        return f"{i+1}. Mirror {feat.get('plane', '?')}" + ("" if feat.get("fuse", True) else "  (copy only)")
    if op in ("pad", "pocket"):
        ext = feat.get("extent") or ("MID_PLANE" if feat.get("symmetric") else "BLIND")
        how = {"BLIND": f"{feat.get('depth', 0) * 1000:.3g} mm",
               "MID_PLANE": f"mid-plane {abs(feat.get('depth', 0)) * 1000:.3g} mm",
               "TWO_SIDES": f"{feat.get('depth', 0) * 1000:.3g} / {feat.get('depth2', 0) * 1000:.3g} mm",
               "THROUGH_ALL": "through all", "THROUGH_ALL_BOTH": "through all (both)",
               "UP_TO_NEXT": "up to next", "UP_TO_LAST": "up to last",
               "UP_TO_CURSOR": "up to cursor"}.get(ext, ext)
        if feat.get("reverse") and ext in ("BLIND", "TWO_SIDES", "THROUGH_ALL", "UP_TO_NEXT", "UP_TO_LAST"):
            how += ", reversed"
        regions = f"  ({len(feat['regions'])} region{'s' if len(feat['regions']) > 1 else ''})" if feat.get("regions") else ""
        return f"{i+1}. {op.capitalize()}  [{feat.get('sketch','?')}]  {how}{regions}"
    return f"{i+1}. {op.capitalize()}  ({feat.get('select','?')})  r={feat.get('radius', 0):.3g}"


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
                row = box.row(align=True)
                off = feat.get("suppressed", False)
                t = row.operator(declarations.Operators.OCCTToggleFeature, text="",
                                 icon="HIDE_ON" if off else "HIDE_OFF", emboss=False)
                t.part, t.index, t.source_index = part_name, i, -1
                label = row.row()
                label.enabled = not off
                label.label(text=_feature_text(i, feat), icon=_ICONS.get(op, "DOT"))
                if op in ("pad", "pocket"):
                    sk = next((x for x in context.scene.sketcher.entities.sketches
                               if x.name == feat.get("sketch")), None)
                    if sk is not None:
                        row.operator(declarations.Operators.SetActiveSketch, text="",
                                     icon="GREASEPENCIL").index = sk.slvs_index
                e = row.operator(declarations.Operators.OCCTEditFeature, text="", icon="PREFERENCES")
                e.part, e.index = part_name, i
                p = row.operator(declarations.Operators.OCCTRemoveFeature,
                                 text="", icon="TRASH")
                p.part = part_name
                p.index = i
                if op == "derive" and feat.get("from") in recipes:
                    # fonctions de la source : decocher pour les retirer de la piece derivee
                    sub = box.column(align=True)
                    sup = set(feat.get("suppress", []))
                    for k, sf in enumerate(recipes[feat["from"]]):
                        r = sub.row(align=True)
                        r.separator(factor=3.0)
                        t = r.operator(declarations.Operators.OCCTToggleFeature, text="",
                                       icon="CHECKBOX_DEHLT" if k in sup else "CHECKBOX_HLT", emboss=False)
                        t.part, t.index, t.source_index = part_name, i, k
                        lr = r.row()
                        lr.enabled = k not in sup
                        lr.label(text=_feature_text(k, sf), icon=_ICONS.get(sf.get("op"), "DOT"))

        layout.operator(declarations.Operators.OCCTPickEdges,
                        icon="RESTRICT_SELECT_OFF")
        row = layout.row(align=True)
        row.operator(declarations.Operators.OCCTRecompute, icon="FILE_REFRESH")
        row.operator(declarations.Operators.OCCTTidyView, icon="HIDE_ON")
        layout.operator(declarations.Operators.OCCTQuadMesh, icon="MOD_SUBSURF")
