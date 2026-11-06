package machinekit.power;

import cadkit.modeling.Part;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import materia.assembly.AssemblyFrames as F;

/** Assumed catalogue-class VFD braking resistor. Dimensions mm; SI ratings unless stated.
 * Geometry is a nominal layout envelope, not manufacturing or supplier geometry. */
class BrakeResistor extends AssumedCatalogComponent {
  static final recipe = new machinekit.cache.Once<ComponentType>();
  static function inputs():Array<ComponentParameter> return [
    P.quantity("resistance", 80, "ohm"),
    P.quantity("continuousPower", 1000, "W"),
    P.quantity("pulseEnergy", 15000, "J"),
    P.length("width", 80),
    P.length("length", 300),
    P.length("height", 60)
  ];
  public function new(?v:ComponentValues) {
    super("brake-resistor", "VFD braking resistor", "aluminium 6061", inputs(), v);
    positive(["resistance", "continuousPower", "pulseEnergy", "width", "length", "height"]);
    addConnector("mount", Mount, F.translation(0, 0, 0));
    addPort({name: "brake", kind: ElectricalPower, role: Consumer, iface: Plug("DC-brake", 2), required: true});
  }
  public static function recipeType():ComponentType {
    return recipe.get(() -> new ComponentType(
      "machinekit.power.brake-resistor",
      inputs(),
      v -> new BrakeResistor(v)
    ).numbered(v -> 'BRK-RESISTOR-ASSUMED-${Dimension.format(v.number("resistance"))}OHM-'
      + '${Dimension.format(v.number("continuousPower"))}W-${Dimension.format(v.number("pulseEnergy"))}J').procured(_ -> Buy(null, null, null)).requiringInputs(["resistance"], ["continuousPower", "pulseEnergy"], []));
  }
  override public function componentType():Null<ComponentType> return recipeType();
  /** A housing, not a solid block: mass from the envelope at an assumed effective density. */
  override function effectiveDensity():Null<Float> return machinekit.component.EffectiveDensities.BrakeResistor;

  /** A finned aluminium extrusion on slotted mounting tabs (`DeviceHousing.finnedResistor`). */
  override public function geometry(detail:ComponentDetail = Preview):Part {
    return detail == Envelope ? Solids.named(Part.box(n("width"),
      n("length"), n("height")), "resistor-envelope") : DeviceHousing.finnedResistor(
        n("width"),
        n("length"),
        n("height")
      );
  }

}
