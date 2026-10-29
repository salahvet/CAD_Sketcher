import logging

from bpy.types import Operator
from bpy.props import FloatProperty, BoolProperty

from ..declarations import Operators
from ..stateful_operator.utilities.register import register_stateops_factory
from .base_constraint import GenericConstraintOp

from ..model.diameter import SlvsDiameter

logger = logging.getLogger(__name__)


class VIEW3D_OT_slvs_add_diameter(Operator, GenericConstraintOp):
    """Add a diameter constraint"""

    bl_idname = Operators.AddDiameter
    bl_label = "Diameter"
    bl_options = {"UNDO", "REGISTER"}

    # Either Radius or Diameter
    value: FloatProperty(
        name="Size",
        subtype="DISTANCE",
        unit="LENGTH",
        precision=5,
        options={"SKIP_SAVE"},
    )
    setting: BoolProperty(name="Use Radius")
    type = "DIAMETER"
    property_keys = ("value", "setting")

    def main(self, context):
        if not self.exists(context, SlvsDiameter):
            self.target = context.scene.sketcher.constraints.add_diameter(
                self.entity1,
                sketch=self.sketch,
                init=not self.initialized,
                **self.get_settings(),
            )
        elif not getattr(self, "target", None):
            # Fork : un diametre/rayon existe deja sur ce cercle -> on le
            # reprend (et on le reaffiche s'il etait masque) au lieu de ne
            # rien faire en silence
            deps = {i for i in (self.entity1, self.sketch) if i is not None}
            for c in context.scene.sketcher.constraints.all:
                if isinstance(c, SlvsDiameter) and set(c.dependencies()) == deps:
                    self.target = c
                    if not c.visible:
                        c.visible = True
                    self.report(
                        {"INFO"},
                        "{} already exists on this curve: reusing it".format(
                            c.label
                        ),
                    )
                    break

        return super().main(context)


register, unregister = register_stateops_factory((VIEW3D_OT_slvs_add_diameter,))
