package machinekit.motion;

import cadkit.modeling.Part;
import machinekit.catalog.Catalog;
import machinekit.catalog.CatalogMetadata.DimensionKind;
import machinekit.catalog.CatalogMetadata.Conformance;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

typedef ProximitySwitchSpec = {
  var designation:String;
  var diameter:Float;
  var length:Float;
  var sensingDistance:Float;
  var hysteresis:Float;
  var repeatability:Float;
}

/** Assumed generic M8/M12 inductive sensor envelope, with an explicit sensing gap. */
class ProximitySwitch extends MachineComponent implements SwitchPart {
  static var table:Null<Catalog<ProximitySwitchSpec>>;
  static var recipe:Null<ComponentType>;
  public final spec:ProximitySwitchSpec;

  public static function catalog():Catalog<ProximitySwitchSpec> {
    if (table == null) table = new Catalog("assumed inductive switch", s -> s.designation, [
      {designation: "GENERIC-INDUCTIVE-M8", diameter: 8.0, length: 30.0,
        sensingDistance: 2.0, hysteresis: 0.2, repeatability: 0.02},
      {designation: "GENERIC-INDUCTIVE-M12", diameter: 12.0, length: 40.0,
        sensingDistance: 4.0, hysteresis: 0.4, repeatability: 0.04}
    ], _ -> ({source: "Authored generic sensor design assumptions; no vendor verification",
      standard: null, standardEdition: null, dimensionKind: Unverified,
      conformance: GenericApproximation, verifiedFields: []}));
    return table;
  }

  public function new(designation:String = "GENERIC-INDUCTIVE-M8") {
    var row = catalog().get(designation);
    super(designation, "Assumed generic inductive proximity switch", "steel");
    spec = row;
    addConnector("mount", Mount, Solids.axial(0, 0, 0));
    addConnector("face", Face, Solids.axial(0, 0, row.length));
    addConnector("trip", Face, Solids.axial(0, 0, row.length + row.sensingDistance));
    addPort({name: "signal", kind: Signal, role: Supply, iface: Unspecified, required: false});
    addPort({name: "power", kind: ElectricalPower, role: Consumer, iface: Unspecified, required: false});
  }
  public function tripConnector():String return "trip";
  public function switchHysteresis():Float return spec.hysteresis;
  public function switchRepeatability():Float return spec.repeatability;
  public static function recipeType():ComponentType {
    if (recipe == null) recipe = new ComponentType("machinekit.motion.proximity-switch", [
      ComponentRecipeSupport.catalog("designation", catalog(), "GENERIC-INDUCTIVE-M8")
    ], v -> new ProximitySwitch(v.token("designation")));
    return recipe;
  }
  override public function componentType():Null<ComponentType>
    return Std.isExactType(this, ProximitySwitch) ? recipeType() : null;
  override public function values():ComponentValues
    return new ComponentValues().setToken("designation", designation).setToken("material", materialSpec());
  override public function hasGeometry():Bool return true;
  /**
   * The envelope is the barrel's cylinder. The preview has a connector end at the mount (z = 0), a fine-pitch
   * metric threaded barrel (a cosmetic thread, M8x1 or M12x1) and the sensing cap at the face (z = length).
   */
  override public function geometry(detail:ComponentDetail = Preview):Part {
    if (detail == Envelope) return envelope();
    var d = spec.diameter, l = spec.length, connector = l * 0.22, cap = l * 0.08;
    return Solids.building([], owned -> {
      var plug = Solids.named(Part.cylinderSpan(d * 0.4, 0, connector * 0.6), "connector");
      var collar = Solids.named(Part.cylinderSpan(d * 0.46, connector * 0.6, connector), "collar");
      var barrel = Solids.named(Part.cylinderSpan(d / 2, connector, l - cap), "barrel");
      var face = Solids.named(Part.cylinderSpan(d * 0.45, l - cap, l), "sensing-face");
      for (part in [plug, collar, barrel, face]) owned.push(part);
      var threaded = new Part(cadkit.threads.Threads.attach(barrel.shape, "barrel:cyl.side",
        machinekit.standard.FastenerThreads.metric(d, 1), cadkit.threads.ThreadedFace.ThreadStart.AxisMinimum, l - cap - connector));
      barrel.close();
      owned.push(threaded);
      return Solids.union([plug, collar, threaded, face]);
    });
  }

  /** Its mass is that of the plain barrel cylinder (the drawn connector and cap do not set it). */
  override public function massGeometry():Null<Part> return envelope();

  function envelope():Part return Solids.named(Part.cylinderSpan(spec.diameter / 2, 0, spec.length), "body");
}
