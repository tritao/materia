package machinekit.motion;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.catalog.Catalog;
import machinekit.catalog.CatalogMetadata.DimensionKind;
import machinekit.catalog.CatalogMetadata.Conformance;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** Assumed generic roller micro-switch; catalog dimensions are not vendor claims. */
typedef LimitSwitchSpec = {
  var designation:String;
  var width:Float;
  var height:Float;
  var depth:Float;
  var rollerDiameter:Float;
  var pretravel:Float;
  var hysteresis:Float;
  var repeatability:Float;
}

/** Housing, lever and roller envelope. Local +Z is the approaching trigger direction. */
class LimitSwitch extends MachineComponent implements SwitchPart {
  static final table = new machinekit.cache.Once<Catalog<LimitSwitchSpec>>();
  static final recipe = new machinekit.cache.Once<ComponentType>();
  public final spec:LimitSwitchSpec;

  public static function catalog():Catalog<LimitSwitchSpec> {
    return table.get(() -> new Catalog(
      "assumed roller switch",
      s -> s.designation,
      [{
        designation: "GENERIC-ROLLER-MICROSWITCH",
        width: 20.0,
        height: 10.0,
        depth: 8.0,
        rollerDiameter: 4.0,
        pretravel: 1.0,
        hysteresis: 0.2,
        repeatability: 0.02
      }],
      _ -> ({
        source: "Authored generic switch design assumptions; no vendor verification",
        standard: null,
        standardEdition: null,
        dimensionKind: Unverified,
        conformance: GenericApproximation,
        verifiedFields: []
      })
    ));
  }

  public function new(designation:String = "GENERIC-ROLLER-MICROSWITCH") {
    var row = catalog().get(designation);
    super(designation, "Assumed generic roller micro-switch", "plastic");
    addFacet(machinekit.component.EngineeringMetadata.assumed(""));
    spec = row;
    addConnector("mount", Mount, Solids.axial(0, 0, 0));
    addConnector("trip", Face, Solids.axial(row.width * 0.4, 0, row.depth + row.rollerDiameter - row.pretravel));
    addPort({name: "signal", kind: Signal, role: Supply, iface: Unspecified, required: false});
  }

  public function tripConnector():String return "trip";
  public function switchHysteresis():Float return spec.hysteresis;
  public function switchRepeatability():Float return spec.repeatability;

  public static function recipeType():ComponentType {
    return recipe.get(() -> new ComponentType(
      "machinekit.motion.limit-switch",
      [ComponentRecipeSupport.catalog("designation", catalog(), "GENERIC-ROLLER-MICROSWITCH")],
      v -> new LimitSwitch(v.token("designation"))
    ).numbered(v -> StringTools.startsWith(v.token("designation"), "GENERIC-")
      ? "SWITCH-ASSUMED-" + v.token("designation").substr("GENERIC-".length) : v.token("designation"))
      .procured(_ -> Buy(null, null, null)).requiring(v -> {
        var row = catalog().get(v.token("designation"));
        return [machinekit.component.Procurement.ProcurementCodec.choice("actuator", "roller"),
          machinekit.component.Procurement.ProcurementCodec.requirement("pretravel", "mm", null, row.pretravel),
          machinekit.component.Procurement.ProcurementCodec.requirement("hysteresis", "mm", null, row.hysteresis),
          machinekit.component.Procurement.ProcurementCodec.requirement("repeatability", "mm", null, row.repeatability)];
      }));
  }
  override public function componentType():Null<ComponentType> return Std.isExactType(this,
    LimitSwitch) ? recipeType() : null;
  override public function values():ComponentValues return new ComponentValues().setToken("designation",
    designation).setToken(
      "material",
      materialSpec()
    );
  override public function hasGeometry():Bool return true;

  override public function geometry(detail:ComponentDetail = Preview):Part {
    return Solids.building([], parts -> {var body = Solids.named(
      Part.box(spec.width, spec.height, spec.depth),
      "housing"
    );
    parts.push(body);
    if (detail != Envelope) {
      var holes:Array<Part> = [];
      for (x in [-6.0, 6.0]) {
        var hole = Solids.named(Part.cylinderSpan(1.2,
          -0.1, spec.depth + 0.1, x), x < 0 ? "mount.left" : "mount.right");
        parts.push(hole);
        holes.push(hole);
      }
      body = Solids.cut(body, holes);
      parts.push(body);
    }
    // The lever ends at the roller axle rather than crossing its full diameter.
    var leverBase = Part.box(spec.width * 0.9, 3, 1.5);
    parts.push(leverBase);
    var lever = Solids.named(leverBase.translated(new Vector(-spec.width * 0.05, 0, spec.depth - 0.5)), "lever");
    leverBase.close();
    parts.push(lever);
    var roller = Solids.named(
      Part.cylinderAlongY(spec.rollerDiameter / 2, -3, 3, spec.width * 0.4, spec.depth + spec.rollerDiameter / 2),
      "roller"
    );
    parts.push(roller);
    return Solids.union([body, lever, roller]);
    });
  }
}
