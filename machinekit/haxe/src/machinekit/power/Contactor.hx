package machinekit.power;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import materia.assembly.AssemblyFrames as F;

/** Assumed catalogue-class power contactor with auxiliary feedback. Dimensions mm; SI ratings unless stated.
 * Geometry is a nominal layout envelope, not manufacturing or supplier geometry. */
class Contactor extends AssumedCatalogComponent {
  static var recipe:Null<ComponentType>;
  static function inputs():Array<ComponentParameter> return [
    P.quantity("ratedCurrent", 32, "A"),
    P.quantity("ratedVoltage", 400, "V"),
    P.quantity("coilVoltage", 24, "V"),
    P.count("poles", 3),
    P.length("width", 55),
    P.length("depth", 80),
    P.length("height", 90)
  ];
  public function new(?v:ComponentValues) {
    super("contactor", "power contactor with auxiliary feedback", "device housing", inputs(), v);
    positive(["ratedCurrent", "ratedVoltage", "coilVoltage", "width", "depth", "height"]);
    require(parameterCount("poles") > 0, "contactor needs poles");
    addConnector("din", Mount, F.translation(0, 0, 0));
    for (name in["line", "load"]) addPort({
      name: name,
      kind: ElectricalPower,
      role: name == "line" ? Consumer : Supply,
      iface : Plug(
        "power",
        parameterCount("poles")
      ),
      required : true
    }
    );
    addPort({name: "coil", kind: ElectricalPower, role: Consumer, iface: Plug("24V", 2), required: true}
    );
  }
  public static function recipeType():ComponentType {
    if (recipe == null) recipe = new ComponentType("machinekit.power.contactor", inputs(), v -> new Contactor(v));
    return recipe;
  }
  override public function componentType():Null<ComponentType> return recipeType();
  /** A housing, not a solid block: mass from the envelope at an assumed effective density. */
  override function effectiveDensity():Null<Float>
    return machinekit.component.EffectiveDensities.DinRailSwitchgear;

  override public function geometry(detail:ComponentDetail = Preview):Part {
    return Solids.named(Part.box(n("width"), n("depth"), n("height")), "contactor-envelope");
  }
  override public function io():MachineIo return {inputs:["feedback"], outputs:["coil"], analogOutputs:[]};

}
