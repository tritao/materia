package machinekit.power;

import machinekit.component.ComponentRecipeSupport;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.Dimension;
import machinekit.motion.PowerSupply;

/**
 * Assumed rail-mounted supply from three-phase mains to a regulated low-voltage DC bus (24 or 48 V) that has to take back what its
 * loads regenerate: a supply of this class alone cannot, so it carries a brake chopper and resistor, and `regenerativePower` is the
 * power that resistor takes continuously (`DriveSystem.checkElectricalSources` compares the servos' regeneration with it). Line voltage
 * is phase-to-phase RMS. The ratings are assumed; there is no product behind them yet.
 */
class ThreePhaseDcSupply extends PowerSupply {
  public final mains:AcMains;
  public function new(
    mains:AcMains,
    volts:Float,
    amps:Float,
    outlets:Int = 1,
    width:Float = 125,
    height:Float = 125,
    depth:Float = 120,
    regenerativePower:Float = 0
  ) {
    if (mains == null || mains.system != ThreePhase) throw "This supply needs three-phase mains";
    super(new DcVoltage(volts), new DcCurrent(amps), outlets, width, height, depth, regenerativePower);
    this.mains = mains;
    addPort({name: "mains",
      kind: ElectricalPower, role: Consumer, iface: Plug("three-phase-mains", 4), required: true});
    for (index in 1...outlets + 1) addConversion("mains", 'power$index');
  }

  static final dcRecipe = new machinekit.cache.Once<ComponentType>();

  public static function recipeType():ComponentType {
    return dcRecipe.get(() -> new ComponentType(
      "machinekit.power.three-phase-dc-supply",
      [
        ComponentRecipeSupport.scalar("lineVoltage", 400, 0.001),
        ComponentRecipeSupport.scalar("frequency", 50, 0.001),
        ComponentRecipeSupport.scalar("voltage", 48, 0.001),
        ComponentRecipeSupport.scalar("current", 20, 0.001),
        ComponentRecipeSupport.count("outlets", 1),
        ComponentRecipeSupport.length("width", 125),
        ComponentRecipeSupport.length("height", 125),
        ComponentRecipeSupport.length("depth", 120),
        ComponentRecipeSupport.scalar("regenerativePower", 300, 0)
      ],
      v -> new ThreePhaseDcSupply(
        new AcMains(ThreePhase, new AcRmsVoltage(v.number("lineVoltage")), v.number("frequency")),
        v.number("voltage"),
        v.number("current"),
        v.integer("outlets"),
        v.number("width"),
        v.number("height"),
        v.number("depth"),
        v.number("regenerativePower")
      )
    ).numbered(v -> 'DCSUP-ASSUMED-${Dimension.format(v.number("lineVoltage"))}VAC-${Dimension.format(v.number("voltage"))}VDC-'
      + '${Dimension.format(v.number("current"))}A-${v.integer("outlets")}O-${Dimension.format(v.number("regenerativePower"))}W')
      .procured(_ -> Buy(null, null, null)).requiring(v -> {
        var req = machinekit.component.Procurement.ProcurementCodec.requirement;
        return [req("lineVoltage", "V", null, null, v.number("lineVoltage")),
          req("frequency", "Hz", null, null, v.number("frequency")), req("voltage", "V", null, null, v.number("voltage")),
          req("current", "A", v.number("current")), req("outlets", "count", v.integer("outlets")),
          req("regenerativePower", "W", v.number("regenerativePower"))];
      }));
  }

  override public function componentType():Null<ComponentType> return recipeType();

  override public function values():ComponentValues return new ComponentValues().setNumber(
    "lineVoltage",
    mains.rmsVoltage.volts()
  ).setNumber("frequency", mains.frequencyHz).setNumber("voltage", voltage.volts()).setNumber("current", current.amps())
    .setInteger("outlets", outlets).setNumber("width", width).setNumber("height", height).setNumber("depth", depth)
    .setNumber("regenerativePower", regenerativePower).setToken("material", materialSpec());
}
