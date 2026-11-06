package machinekit.power;

import cadkit.modeling.Part;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import materia.assembly.AssemblyFrames as F;

/** Assumed catalogue-class three-phase spindle VFD with STO terminals. Dimensions mm; SI ratings unless stated.
 * Geometry is a nominal layout envelope, not manufacturing or supplier geometry. */
class Vfd extends AssumedCatalogComponent {
  static final recipe = new machinekit.cache.Once<ComponentType>();
  static function inputs():Array<ComponentParameter> return [
    P.quantity("power", 7500, "W"),
    P.quantity("inputVoltage", 400, "V"),
    P.quantity("outputCurrent", 18, "A"),
    P.quantity("maxFrequency", 400, "Hz"),
    P.length("width", 180),
    P.length("depth", 180),
    P.length("height", 300),
    P.flag("sto", true)
  ];
  public function new(?v:ComponentValues) {
    super("vfd", "three-phase spindle VFD with STO terminals", "device housing dark", inputs(), v);
    positive(["power", "inputVoltage", "outputCurrent", "maxFrequency", "width", "depth", "height"]);
    addConnector("mount", Mount, F.translation(0, 0, 0));
    addPort({name: "mains", kind: ElectricalPower, role: Consumer, iface: Plug("3-phase-PE", 4), required: true});
    addPort({name: "motor", kind: ElectricalPower, role: Supply, iface: Plug("3-phase-PE", 4), required: true});
    addPort({name: "brake_resistor", kind: ElectricalPower, role: Supply, iface: Plug("DC-brake", 2), required: false});
    if (flag("sto")) for (i in 1...3) addPort({
      name: "sto" + i,
      kind: Signal,
      role: Consumer,
      iface: Plug("24V", 1),
      required: true
    });
  }
  public static function recipeType():ComponentType {
    return recipe.get(() -> new ComponentType("machinekit.power.vfd", inputs(), v -> new Vfd(v))
      .numbered(v -> 'VFD-ASSUMED-${Dimension.format(v.number("power"))}W-${Dimension.format(v.number("inputVoltage"))}V-'
        + '${Dimension.format(v.number("outputCurrent"))}A-${Dimension.format(v.number("maxFrequency"))}HZ-'
        + (v.boolean("sto") ? "STO" : "NO-STO")).procured(_ -> Buy(null, null, null)).requiringInputs(["inputVoltage", "sto"], ["power", "outputCurrent", "maxFrequency"], []));
  }
  override public function componentType():Null<ComponentType> return recipeType();
  /** A housing, not a solid block: mass from the envelope at an assumed effective density. */
  override function effectiveDensity():Null<Float> return machinekit.component.EffectiveDensities.Drive;

  /** Keypad and display on its front, louvred sides and a finned heat sink behind (`DeviceHousing.enclosure`). */
  override public function geometry(detail:ComponentDetail = Preview):Part {
    return detail == Envelope ? Solids.named(Part.box(n("width"),
      n("depth"), n("height")), "vfd-envelope") : DeviceHousing.enclosure(
        n("width"),
        n("depth"),
        n("height"),
        {keypad : true, louvres : true, fins : 9}
      );
  }
  override public function io():MachineIo return {
    inputs: ["at_speed", "zero_speed", "alarm"],
    outputs: ["run", "reset"],
    analogOutputs: ["speed"]
  };

}
