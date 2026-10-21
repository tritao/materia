package machinekit.control;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import materia.assembly.AssemblyFrames as F;

/** Assumed catalogue-class dual-channel safety relay with delayed contacts. Dimensions mm; SI ratings unless stated.
 * Geometry is a nominal layout envelope, not manufacturing or supplier geometry. */
class SafetyRelay extends AssumedCatalogComponent {
  static var recipe:Null<ComponentType>;
  static function inputs():Array<ComponentParameter> return [
    P.quantity("supplyVoltage", 24, "V"),
    P.quantity("stopDelay", 2, "s"),
    P.count("immediateContacts", 2),
    P.count("delayedContacts", 2),
    P.length("width", 45),
    P.length("depth", 120),
    P.length("height", 100)
  ];
  public function new(?v:ComponentValues) {
    super("safety-relay", "dual-channel safety relay with delayed contacts", "device housing", inputs(), v);
    positive(["supplyVoltage", "stopDelay", "width", "depth", "height"]);
    require(
      parameterCount("immediateContacts") > 0 && parameterCount("delayedContacts") > 0,
      "relay needs immediate and delayed contacts"
    );
    addConnector("din", Mount, F.translation(0, 0, 0));
    for (i in 1...3) addPort({
      name: "safety_input" + i,
      kind: Signal,
      role: Consumer,
      iface: Plug("24V", 1),
      required: true
    }
    );
    for (i in 0...parameterCount("immediateContacts")) addPort({
      name: "immediate" +(i + 1),
      kind: Signal,
      role: Supply,
      iface: Plug(
        "dry-contact",
        2
      ),
      required: true
    }
    );
    for (i in 0...parameterCount("delayedContacts")) addPort({
      name: "delayed" +(i + 1),
      kind: Signal,
      role: Supply,
      iface: Plug(
        "dry-contact",
        2
      ),
      required: true
    }
    );
    addPort({name: "feedback_loop", kind: Signal, role: Consumer, iface: Plug("dry-contact", 2), required: true}
    );
  }
  public static function recipeType():ComponentType {
    if (recipe == null) recipe = new ComponentType("machinekit.control.safety-relay",
      inputs(), v -> new SafetyRelay(v));
    return recipe;
  }
  override public function componentType():Null<ComponentType> return recipeType();
  /** A housing, not a solid block: mass from the envelope at an assumed effective density. */
  override function effectiveDensity():Null<Float>
    return machinekit.component.EffectiveDensities.DinRailElectronics;

  override public function geometry(detail:ComponentDetail = Preview):Part {
    return Solids.named(Part.box(n("width"), n("depth"), n("height")), "relay-envelope");
  }
  override public function io():MachineIo return {inputs:["healthy"], outputs:[], analogOutputs:[]};

}
