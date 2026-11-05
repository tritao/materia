package machinekit.motion;

import cadkit.InertiaTensor;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import machinekit.motion.SpindleBearingData.SpindleBearingRow;

/** Catalogue angular-contact spindle bearing. Frame: front z0, rear z width.
 * The seal/race envelope and annular inertia distribution are approximations;
 * catalogue mass, boundary dimensions, ratings and preload class are retained.
 * Two GA bearings mounted DB form the outboard's preloaded pair. */
class SpindleBearing extends MachineComponent {
  public final row:SpindleBearingRow;
  static var recipe:Null<ComponentType>;
  public static function parameters():Array<ComponentParameter> return EngineeringMetadata.parameters().concat([
    P.choice("model", SpindleBearingData.catalog().designations(), SpindleBearingData.SKF_MODEL),
    P.quantity("radialRunout", 0.0015, "mm"),
    P.quantity("axialRunout", 0.0015, "mm")
  ]);
  public static function recipeType():ComponentType {
    if (recipe == null) recipe = new ComponentType("machinekit.motion.spindle-bearing", parameters(), v -> new SpindleBearing(v), false, false, null, false)
      .numbered(v -> "SKF-" + v.token("model").split(" ").join("-").split("/").join("-"))
      .procured(v -> Buy("SKF", v.token("model"), "P4A; universally matchable GA"));
    return recipe;
  }
  public function new(?overrides:ComponentValues) {
    var v = P.resolve(parameters(), overrides);
    v.setToken("ratingsBasis", "Catalog").setToken("source", SpindleBearingData.SKF_SOURCE);
    super("SKF-" + v.token("model"), "SKF universally matchable spindle bearing " + v.token("model"), "bearing steel", false, v);
    row = SpindleBearingData.catalog().get(v.token("model"));
    addFacet(new EngineeringMetadata(v));
    addFacet(PrecisionErrors.fromValues(v, [{name: "radialRunout", unit: "mm"}, {name: "axialRunout", unit: "mm"}]));
    addConnector("mount", Mount, Solids.axial(0, 0, 0));
    addConnector("axis", Axis, Solids.axial(0, 0, row.width / 2));
    addConnector("rear", Mount, Solids.axial(0, 0, row.width));
    var r2 = (row.outside * row.outside + row.bore * row.bore) / 4;
    var transverse = row.mass * (3 * r2 + row.width * row.width) / 12;
    declareMass(row.mass, new Vector(0, 0, row.width / 2),
      new InertiaTensor(transverse, 0, 0, transverse, 0, row.mass * r2 / 2));
  }
  override public function componentType():Null<ComponentType> return recipeType();
  override public function carriesJournal():Bool return true;
  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail = Preview):Part return detail == Envelope
    ? Solids.cut(Part.cylinderSpan(row.outside / 2, 0, row.width), [Part.cylinderSpan(row.bore / 2, -1, row.width + 1)])
    : RingParts.bearing(row.outside, row.bore, row.width);
}
