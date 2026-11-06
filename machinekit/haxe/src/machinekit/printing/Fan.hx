package machinekit.printing;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.catalog.Catalog;
import machinekit.catalog.CatalogMetadata.DimensionKind;
import machinekit.catalog.CatalogMetadata.Conformance;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

typedef FanSpec = {
  var designation:String;
  /** True for a blower, whose air leaves sideways through the nozzle at the end of its housing. */
  var blower:Bool;
  var width:Float;
  var depth:Float;
  var height:Float;
  var volts:Float;
  var watts:Float;
}

/**
 * A DC fan or blower. Origin: the middle of the underside of the housing, with the fan's axis along +Z. A
 * blower's outlet is the face on +X; `outlet` is there. Rows are assumed 24 V parts.
 */
class Fan extends MachineComponent {
  static final table = new machinekit.cache.Once<Catalog<FanSpec>>();
  static final recipe = new machinekit.cache.Once<ComponentType>();
  public final spec:FanSpec;

  public static function catalog():Catalog<FanSpec> {
    return table.get(() -> new Catalog(
      "fan",
      s -> s.designation,
      [
        {designation: "FAN-4010-24V", blower: false, width: 40.0, depth: 40.0, height: 10.0, volts: 24.0, watts: 1.5},
        {designation: "BLOWER-5015-24V", blower: true, width: 50.0, depth: 50.0, height: 15.0, volts: 24.0, watts: 2.4}
      ],
      _ -> ({
        source: "Assumed common axial fan and blower sizes; no vendor verification",
        standard: null,
        standardEdition: null,
        dimensionKind: Unverified,
        conformance: GenericApproximation,
        verifiedFields: []
      })
    ));
  }

  public function new(designation:String = "FAN-4010-24V") {
    var row = catalog().get(designation);
    super(
      designation,
      '${row.blower ? "Blower" : "Fan"}, ${Std.string(Math.round(row.width))} x ${Std.string(Math.round(row.height))} mm, ${Std.string(Math.round(row.volts))} V',
      "plastic"
    );
    if (Std.isExactType(this, Fan)) addFacet(new machinekit.component.EngineeringMetadata(new ComponentValues()
      .setToken("ratingsBasis", "Assumed").setToken("errorsBasis", "Assumed").setToken("source", "")));
    spec = row;
    addConnector("mount", Mount, Solids.axial(0, 0, 0));
    if (row.blower) addConnector("outlet", Face, Solids.axial(row.width / 2, 0, row.height / 2));
    addPort({name: "power", kind: ElectricalPower, role: Consumer, iface: Unspecified, required: false});
  }

  public static function recipeType():ComponentType {
    return recipe.get(() -> new ComponentType(
      "machinekit.printing.fan",
      [ComponentRecipeSupport.catalog("designation", catalog(), "FAN-4010-24V")],
      v -> new Fan(v.token("designation"))
    ).numbered(v -> v.token("designation")).procured(_ -> Buy(null, null, null)).requiring(v -> {
      var row = catalog().get(v.token("designation"));
      return [machinekit.component.Procurement.ProcurementCodec.choice("kind", row.blower ? "blower" : "fan"),
        machinekit.component.Procurement.ProcurementCodec.requirement("width", "mm", null, null, row.width),
        machinekit.component.Procurement.ProcurementCodec.requirement("depth", "mm", null, null, row.depth),
        machinekit.component.Procurement.ProcurementCodec.requirement("height", "mm", null, null, row.height),
        machinekit.component.Procurement.ProcurementCodec.requirement("voltage", "V", null, null, row.volts)];
    }));
  }
  override public function componentType():Null<ComponentType> return Std.isExactType(this, Fan) ? recipeType() : null;
  override public function values():ComponentValues return new ComponentValues().setToken("designation",
    designation).setToken(
      "material",
      materialSpec()
    );
  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail = Preview):Part {
    if (detail == Envelope) return envelope();
    if (!spec.blower) {
      // The square frame around the rotor, its hub, and the four corner screw holes.
      var pitch = 0.8 * spec.width, frame = Solids.cut(
        envelope(),
        [Solids.named(Part.cylinderSpan(0.47 * spec.width,
        -0.1, spec.height + 0.1), "rotor-bore")].concat([for (i in 0...4) Solids.named(
          Part.cylinderSpan(
            1.6,
            -0.1,
            spec.height + 0.1,
            (i % 2 == 0 ? -1 : 1) * pitch / 2,
            (i < 2 ? -1 : 1) * pitch / 2
          ),
          "screw-hole"
        )])
      );
      var hub = Solids.named(Part.cylinderSpan(0.2 * spec.width, 0.1 * spec.height, 0.9 * spec.height), "hub");
      var struts = [for (i in 0...4) Solids.named(
        Part.cylinderAlong(
          0.08 * spec.height,
          new Vector(0, 0, 0.3 * spec.height),
          new Vector(Math.cos(i * Math.PI / 2), Math.sin(i * Math.PI / 2), 0),
          0.49 * spec.width
        ),
        "strut"
      )];
      return Solids.union([frame, hub].concat(struts));
    }
    // The volute is a cylinder with a duct leaving it towards +X.
    var wheel = Solids.named(Part.cylinderSpan(spec.width / 2 - 1, 0, spec.height), "volute");
    var duct = Solids.named(
      Part.box(spec.width / 2, spec.depth * 0.4, spec.height).translated(new Vector(spec.width / 4, 0, 0)),
      "duct"
    );
    return Solids.union([wheel, duct]);
  }

  override public function massGeometry():Null<Part> return spec.blower ? null : envelope();

  function envelope():Part return Solids.named(Part.box(spec.width, spec.depth, spec.height), "housing");
}
