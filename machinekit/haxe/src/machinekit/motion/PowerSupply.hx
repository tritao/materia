package machinekit.motion;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentParameter;
import machinekit.component.ComponentParameterType;
import machinekit.component.ComponentValue;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** A DC supply with stated output ratings and an assumed rectangular envelope.
 * Current is a DC output rating shared by all outlets. A regenerative rating
 * represents an integrated shunt and braking resistor, with an assumed envelope.
 */
class PowerSupply extends MachineComponent implements ElectricalSource {
  static final recipe = new machinekit.cache.Once<ComponentType>();
  public final voltage:machinekit.power.DcVoltage;
  public final current:machinekit.power.DcCurrent;
  public final regenerativePower:Float;
  public final outlets:Int;
  public final width:Float;
  public final height:Float;
  public final depth:Float;

  public function new(
    voltage:machinekit.power.DcVoltage,
    current:machinekit.power.DcCurrent,
    outlets:Int = 1,
    width:Float = 120,
    height:Float = 60,
    depth:Float = 30,
    regenerativePower:Float = 0
  ) {
    if (outlets < 1 || outlets > 64) throw "A power supply needs from 1 to 64 output ports";
    for (value in [voltage.volts(), current.amps(), width, height, depth]) if (!(value > 0)
      || !Math.isFinite(value)) throw "A power supply needs finite positive ratings and dimensions";
    if (!(regenerativePower >= 0)
      || !Math.isFinite(regenerativePower)) throw "Regenerative capacity must be finite and nonnegative";
    super(
      numberFor(voltage.volts(), current.amps(), width, height, depth, outlets, regenerativePower),
      'DC supply, ${Dimension.format(voltage.volts())} V, ${Dimension.format(current.amps())} A',
      "aluminium 6061"
    );
    addFacet(new machinekit.component.EngineeringMetadata(new ComponentValues()
      .setToken("ratingsBasis", "Stated").setToken("errorsBasis", "Assumed").setToken("source", "")));
    this.voltage = voltage;
    this.current = current;
    this.regenerativePower = regenerativePower;
    this.outlets = outlets;
    this.width = width;
    this.height = height;
    this.depth = depth;
    addConnector("mount", Mount, Solids.axial(0, 0, 0));
    for (index in 1...outlets + 1) addPort({
      name: 'power$index',
      kind: ElectricalPower,
      role: Supply,
      iface: Unspecified,
      required: false
    });
  }

  override public function electricalSource():Null<ElectricalSource> return this;

  public function outputRating(name:String):ElectricalSupplyRating {
    var output = port(name);
    return {voltage: voltage, current: current, regenerativePower: regenerativePower};
  }

  static function numberFor(voltage:Float, current:Float, width:Float, height:Float, depth:Float,
    outlets:Int, regenerativePower:Float):String {
    var readable = 'PSU-ASSUMED-${Dimension.format(voltage)}V${Dimension.format(current)}A-'
      + '${Dimension.format(width)}x${Dimension.format(height)}x${Dimension.format(depth)}-'
      + '${outlets}O-${Dimension.format(regenerativePower)}';
    if (readable.length <= machinekit.component.PartNumberLint.MAX_LENGTH) return readable;
    var key = [for (value in [voltage, current, width, height, depth, outlets, regenerativePower])
      machinekit.component.ComponentNumberKey.encode(value)].join("|");
    return machinekit.component.PartNumbers.geometry("PSU", "ASSUMED", key);
  }

  public static function recipeType():ComponentType {
    return recipe.get(() -> new ComponentType(
      "machinekit.motion.power-supply",
      [
        new ComponentParameter("voltage", ComponentParameterType.Scalar, ComponentValue.Number(24), "V", 0.001),
        new ComponentParameter("current", ComponentParameterType.Scalar, ComponentValue.Number(10), "A DC", 0.001),
        ComponentRecipeSupport.count("outlets", 1),
        ComponentRecipeSupport.length("width", 120),
        ComponentRecipeSupport.length("height", 60),
        ComponentRecipeSupport.length("depth", 30),
        ComponentRecipeSupport.scalar("regenerativePower", 0, 0)
      ],
      v -> new PowerSupply(
        new machinekit.power.DcVoltage(v.number("voltage")),
        new machinekit.power.DcCurrent(v.number("current")),
        v.integer("outlets"),
        v.number("width"),
        v.number("height"),
        v.number("depth"),
        v.number("regenerativePower")
      )
    ).numbered(v -> numberFor(v.number("voltage"), v.number("current"), v.number("width"),
      v.number("height"), v.number("depth"), v.integer("outlets"), v.number("regenerativePower")))
      .procured(_ -> Buy(null, null, null)).requiring(v -> {
        var req = machinekit.component.Procurement.ProcurementCodec.requirement;
        return [req("voltage", "V", null, null, v.number("voltage")), req("current", "A", v.number("current")),
          req("outlets", "count", v.integer("outlets")), req("regenerativePower", "W", v.number("regenerativePower"))];
      }));
  }

  override public function componentType():Null<ComponentType> return Std.isExactType(this,
    PowerSupply) ? recipeType() : null;
  override public function values():ComponentValues return new ComponentValues().setNumber("voltage",
    voltage.volts()).setNumber(
      "current",
      current.amps()
    ).setInteger(
      "outlets",
      outlets
    ).setNumber(
      "width",
      width
    ).setNumber(
      "height",
      height
    ).setNumber(
      "depth",
      depth
    ).setNumber(
      "regenerativePower",
      regenerativePower
    ).setToken(
      "material",
      materialSpec()
    );
  override public function hasGeometry():Bool return true;
  /** A housing, not a solid block: mass from the envelope at an assumed effective density. */
  override function effectiveDensity():Null<Float> return machinekit.component.EffectiveDensities.PowerSupply;

  /** A vented cover with the screw-terminal strip along its top (`DeviceHousing.panelModule`). */
  override public function geometry(detail:ComponentDetail = Preview):Part return detail == Envelope ? Solids.named(
    Part.box(width, height, depth),
    "body"
  ) : machinekit.power.DeviceHousing.panelModule(
    width,
    height,
    depth,
    {vents : true, terminals : Std.int(Math.min(12, 3 + 2 * outlets))}
  );

  override public function presentationView():Null<machinekit.component.ComponentView
    > return machinekit.power.DeviceHousing.panelView();
}
