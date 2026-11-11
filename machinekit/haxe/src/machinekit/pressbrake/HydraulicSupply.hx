package machinekit.pressbrake;

import cadkit.modeling.Part;
import machinekit.fluid.PressureSource;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** Assumed pressure-regulated hydraulic service, including its tank return.
 * Pressure is gauge Pa. The envelope declares no pump flow or power rating.
 */
class HydraulicSupply extends MachineComponent implements PressureSource {
  public final pressurePa:Float;
  static final recipe = new machinekit.cache.Once<ComponentType>();
  public function new(pressurePa:Float) {
    if (!Math.isFinite(pressurePa) || pressurePa < 0) throw "Hydraulic supply pressure must be finite and nonnegative";
    super('HYDRAULIC-SUPPLY-$pressurePa', "Assumed regulated hydraulic service", "steel", true);
    addFacet(machinekit.component.EngineeringMetadata.assumed(""));
    this.pressurePa = pressurePa;
    addConnector("mount", Mount, Solids.axial(0, 0, 0));
    addPort({name: "P", kind: Hydraulic, role: Supply, iface: Thread("G1/2"), required: false});
    addPort({name: "T", kind: Hydraulic, role: Consumer, iface: Thread("G1/2"), required: false});
  }
  public function outputPressure(port:String):Float {
    if (port != "P") throw 'Unknown hydraulic pressure outlet "$port"';
    return pressurePa;
  }
  override public function pressureSource():Null<PressureSource> return this;
  override public function hasGeometry():Bool return true;
  /** A power pack: the oil tank, the pump bell on it and the motor above. Envelope detail is the box, which also sets the mass. */
  override public function geometry(detail:ComponentDetail = Preview):Part {
    if (detail == Envelope) return massGeometry();
    return Solids.union([
      Solids.named(Part.box(100, 100, 110), "tank"),
      Solids.named(Part.cylinderSpan(12, 109.99, 112, 30, 30), "filler-cap"),
      Solids.named(
        Part.revolve([
          {r: 0.0, z: 109.99},
          {r: 28.0, z: 109.99},
          {r: 28.0, z: 128.0},
          {r: 46.0, z: 128.0},
          {r: 46.0, z: 192.0},
          {r: 40.0, z: 200.0},
          {r: 0.0, z: 200.0}
        ]),
        "motor"
      )
    ]);
  }

  override public function massGeometry():Null<Part> return Solids.named(Part.box(100, 100, 200), "hydraulic-service");
  public static function recipeType():ComponentType {
    return recipe.get(() -> new ComponentType(
      "machinekit.pressbrake.hydraulic-supply",
      [new machinekit.component.ComponentParameter("pressurePa", Scalar, Number(0), "Pa", 0)],
      v -> new HydraulicSupply(v.number("pressurePa"))
    ).numbered(v -> 'HYD-SUPPLY-ASSUMED-${Dimension.format(v.number("pressurePa"))}PA')
      .procured(_ -> Buy(null, null, null)).requiring(v -> [
        machinekit.component.Procurement.ProcurementCodec.requirement("pressure", "Pa", v.number("pressurePa")),
        machinekit.component.Procurement.ProcurementCodec.choice("portThread", "G1/2")
      ]));
  }
  override public function componentType():Null<ComponentType> return recipeType();
  override public function values():ComponentValues return new ComponentValues().setNumber("pressurePa",
    pressurePa).setToken(
      "material",
      materialSpec()
    );
}
