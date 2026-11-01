package machinekit.printing;

import cadkit.modeling.Part;
import machinekit.catalog.Catalog;
import machinekit.catalog.CatalogMetadata.DimensionKind;
import machinekit.catalog.CatalogMetadata.Conformance;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

typedef HotendSpec = {
  var designation:String;
  /** Heatsink: diameter and length, mm. The filament enters at its top. */
  var heatsinkDiameter:Float;
  var heatsinkLength:Float;
  /** Heater block, mm. It carries the cartridge, the thermistor and the nozzle. */
  var blockWidth:Float;
  var blockDepth:Float;
  var blockHeight:Float;
  /** Thermal model (assumed): heat capacity of the block with nozzle and cartridge, J/K; heat lost to still air, W/K;
	 * extra loss at full part-cooling fan, W/K. */
  var heatCapacity:Float;
  var lossCoefficient:Float;
  var fanLossCoefficient:Float;
  /** Nominal melt-zone throughput for this hardware (assumed), mm^3/s. */
  var maxVolumetricFlow:Float;
  var maxTemperature:Float;
}

/**
 * A hotend without its nozzle, cartridge or thermistor, which are separate parts: a heatsink and the heater block they
 * clamp to. All thermal and flow values are assumptions for a V6-style hotend, to be replaced by a measured part.
 * Origin: the block's underside on the nozzle axis, which runs along +Z; `nozzleSeat` is there, `filamentEntry` is at
 * the top of the heatsink and the cartridge and thermistor seats are in the block.
 */
class Hotend extends MachineComponent {
  static var table:Null<Catalog<HotendSpec>>;
  static var recipe:Null<ComponentType>;
  public final spec:HotendSpec;

  public static function catalog():Catalog<HotendSpec> {
    if (table == null) table = new Catalog(
      "hotend",
      s -> s.designation,
      [{
        designation: "GENERIC-V6-STYLE",
        heatsinkDiameter: 22.0,
        heatsinkLength: 30.0,
        blockWidth: 16.0,
        blockDepth: 20.0,
        blockHeight: 11.5,
        heatCapacity: 13.5,
        lossCoefficient: 0.12,
        fanLossCoefficient: 0.05,
        maxVolumetricFlow: 15.0,
        maxTemperature: 285.0
      }],
      _ -> ({
        source: "Assumed V6-style hotend; thermal and flow values are not measured",
        standard: null,
        standardEdition: null,
        dimensionKind: Unverified,
        conformance: GenericApproximation,
        verifiedFields: []
      })
    );
    return table;
  }

  public function new(designation:String = "GENERIC-V6-STYLE") {
    var row = catalog().get(designation);
    super(designation, "Hotend heatsink and heater block", "aluminium 6061");
    spec = row;
    addConnector("nozzleSeat", Mount, Solids.axial(0, 0, 0));
    addConnector("filamentEntry", Axis, Solids.axial(0, 0, row.blockHeight + row.heatsinkLength));
    addConnector("mount", Mount, Solids.axial(0, 0, row.blockHeight + row.heatsinkLength - 6));
    // The cartridge lies along y through the block's middle, the thermistor along x behind it.
    addConnector("cartridgeSeat", Axis, Solids.axial(0, -row.blockDepth / 2, row.blockHeight / 2));
    addConnector("thermistorSeat", Axis, Solids.axial(-row.blockWidth / 2, 0, row.blockHeight / 2 + 3));
    addPort({name: "heater", kind: ElectricalPower, role: Consumer, iface: Unspecified, required: false});
  }

  /** Heat leaving the block at `celsius` above `ambient` with the part-cooling fan at `fanDuty` (0 to 1), watts. */
  public function loss(celsius:Float, ambient:Float,
    fanDuty:Float):Float return (spec.lossCoefficient + spec.fanLossCoefficient * fanDuty) * (celsius - ambient);

  /** The fastest filament speed the melt zone delivers through a bead of `area` m^2, m/s of travel: flow over area. */
  public function maxSpeed(area:Float):Float {
    if (!(area > 0)) throw "A bead needs a positive area";
    return spec.maxVolumetricFlow * 1e-9 / area;
  }

  public static function recipeType():ComponentType {
    if (recipe == null) recipe = new ComponentType(
      "machinekit.printing.hotend",
      [ComponentRecipeSupport.catalog("designation", catalog(), "GENERIC-V6-STYLE")],
      v -> new Hotend(v.token("designation"))
    ).numbered(v -> v.token("designation"));
    return recipe;
  }
  override public function componentType():Null<ComponentType> return Std.isExactType(this,
    Hotend) ? recipeType() : null;
  override public function values():ComponentValues return new ComponentValues().setToken("designation",
    designation).setToken(
      "material",
      materialSpec()
    );
  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail = Preview):Part {
    var block = Solids.named(Part.box(spec.blockWidth, spec.blockDepth, spec.blockHeight), "block");
    var sink = Solids.named(
      Part.cylinderSpan(spec.heatsinkDiameter / 2, spec.blockHeight, spec.blockHeight + spec.heatsinkLength),
      "heatsink"
    );
    if (detail == Envelope) return Solids.union([block, sink]);
    var breakNeck = Solids.named(Part.cylinderSpan(3.5, spec.blockHeight - 0.5, spec.blockHeight + 3), "neck");
    // The heatsink's fins: a core with a fin every 2.5 mm below a solid mounting collar at its top.
    sink.close();
    var r = spec.heatsinkDiameter / 2,
      base = spec.blockHeight + 3, top = spec.blockHeight + spec.heatsinkLength, collar = top - 8;
    var profile:Array<{r:Float, z:Float}> = [{r: 0.0, z: base}, {r: 0.4 * r, z: base}];
    var z = base;
    while (z + 2.5 <= collar) {
      profile.push({r: 0.4 * r, z: z + 1.5});
      profile.push({r: r, z: z + 1.5});
      profile.push({r: r, z: z + 2.5});
      profile.push({r: 0.4 * r, z: z + 2.5});
      z += 2.5;
    }
    if (z < collar) profile.push({r: 0.4 * r, z: collar});
    profile = profile.concat([{r: r, z: collar}, {r: r, z: top}, {r: 0.0, z: top}]);
    return Solids.union([block, breakNeck, Solids.named(Part.revolve(profile), "heatsink")]);
  }

  /** The turned heatsink without its fins, as before they were drawn. */
  override public function massGeometry():Null<Part> {
    var block = Solids.named(Part.box(spec.blockWidth, spec.blockDepth, spec.blockHeight), "block");
    var sink = Solids.named(
      Part.cylinderSpan(spec.heatsinkDiameter / 2, spec.blockHeight, spec.blockHeight + spec.heatsinkLength),
      "heatsink"
    );
    return Solids.union([
      block,
      Solids.named(Part.cylinderSpan(3.5, spec.blockHeight - 0.5, spec.blockHeight + 3), "neck"),
      sink
    ]);
  }
}
