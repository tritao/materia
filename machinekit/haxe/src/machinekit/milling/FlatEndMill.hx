package machinekit.milling;

import cadkit.parametric.Feature;
import cadkit.parametric.recording.DocumentBuilder;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Visible flat cutter envelope, origin at tool tip. The buried shank is not
 * represented; stickout is measured from the collet's front face. */
class FlatEndMill extends MachineComponent {
  static final recipe = new machinekit.cache.Once<ComponentType>();
  public static function parameters():Array<ComponentParameter> return [
    P.length("diameter", 6),
    P.length("fluteLength", 22),
    P.length("stickout", 35)
  ];
  public static function recipeType():ComponentType {
    return recipe.get(() -> new ComponentType(
      "machinekit.milling.flat-end-mill",
      parameters(),
      v -> new FlatEndMill(v)
    ).numbered(v -> 'ENDMILL-ASSUMED-D${Dimension.format(v.number("diameter"))}-F${Dimension.format(v.number("fluteLength"))}-S${Dimension.format(v.number("stickout"))}'));
  }
  public function new(?values:ComponentValues) {
    var v = P.resolve(parameters(), values);
    if (v.number("diameter") <= 0 || v.number("fluteLength") <= 0 || v.number("stickout")
      < v.number("fluteLength")) throw "Cutter needs positive dimensions and flutes within stickout";
    super("FLAT-END-MILL", "Flat end mill — nominal visible envelope", "steel", false, v);
    addConnector("tip", Mount, Solids.axial(0, 0, 0));
  }
  override public function componentType():Null<ComponentType> return recipeType();
  override public function hasGeometry():Bool return true;
  override public function record(b:DocumentBuilder, detail:ComponentDetail):Null<Feature> {
    PartRecordings.recipeInputs(this, b);
    var radius = parameterNumber("diameter") / 2, flute = parameterNumber("fluteLength"), stickout = parameterNumber("stickout");
    var axis = Vector.Z(), origin = new Vector(0, 0, 0);
    var cutter:Feature = b.label(b.cylinderAlong(origin, axis, radius, 0, flute), "Flutes");
    var blank:Feature = stickout == flute ? cutter : b.label(b.unionAll([
      cutter,
      b.label(b.cylinderAlong(origin, axis, radius * 0.9333333333, flute, stickout), "Neck")
    ]), "Cutter blank");
    if (detail == Envelope) return blank;
    // Two assumed 30-degree helical gullets make orientation readable at preview fidelity; the envelope stays the blank.
    var helix = Math.tan(Math.PI / 6) / radius, twist = flute * helix;
    var steps = Std.int(Math.min(96, Math.max(8, Math.ceil(twist / (Math.PI / 12)))));
    var gullets:Array<Feature> = [for (phase in [0.0, Math.PI]) b.loft([for (i in 0...steps + 1) {
      var z = -0.02 + (flute + 0.04) * i / steps, a = phase + z * helix;
      b.wire(b.circle(radius * 0.48, new Plane(new Vector(radius * 0.88 * Math.cos(a), radius * 0.88 * Math.sin(a), z), Vector.X(), Vector.Z())));
    }])];
    return b.label(b.cutAll(blank, gullets, ["gullet 1", "gullet 2"]), "Helical gullets");
  }
}
