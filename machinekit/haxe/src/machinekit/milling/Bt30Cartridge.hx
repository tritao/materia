package machinekit.milling;

import cadkit.modeling.Vector;
import cadkit.parametric.Feature;
import cadkit.parametric.recording.DocumentBuilder;
import machinekit.component.*;

/** BT30 packaging study. Ø90 body and Ø125 flange follow the DAIYA BT30SP
 * envelope; axial interfaces, bearing/shaft internals and mounting holes are
 * explicit project assumptions, not a supplier interchangeable cartridge. */
class Bt30Cartridge extends MachineComponent {
  public static inline var DIAMETER:Float = 90;
  public static inline var SEAT:Float = 60;
  public static inline var REAR:Float = 242;
  static final recipe = new machinekit.cache.Once<ComponentType>();
  public static function recipeType():ComponentType {
    return recipe.get(() -> new ComponentType(
      "machinekit.milling.bt30-cartridge-study",
      [],
      v -> new Bt30Cartridge()
    ).numbered(_ -> "BT30-CARTRIDGE-ASSUMED-D90"));
  }
  public function new() {
    super("BT30-CARTRIDGE-STUDY", "BT30 stationary housing — assumed interfaces", "steel");
    for (row in [{id: "gaugeLine",
      z: 0.0}, {id: "mount", z: SEAT}, {id: "reaction", z: REAR}, {id: "pulley", z: 250.0}]) addConnector(
        row.id,
        Mount,
        Solids.axial(0, 0, row.z)
      );
  }
  override public function componentType():Null<ComponentType> return recipeType();
  override public function values():ComponentValues return new ComponentValues().setToken("material", materialSpec());
  override public function hasGeometry():Bool return true;
  override public function record(b:DocumentBuilder, detail:ComponentDetail):Null<Feature> {
    machinekit.component.PartRecordings.recipeInputs(this, b);
    var axis = Vector.Z();
    function post(r:Float, lo:Float, hi:Float, x:Float = 0, y:Float = 0):Feature return b.cylinderAlong(new Vector(x, y, 0), axis, r, lo, hi);
    var body = b.label(b.unionAll([b.label(post(45, SEAT, REAR), "Housing"), b.label(post(62.5, 40, SEAT), "Flange")]), "Housing and flange");
    // The nose, receiving taper and drawbar spring seat belong to the rotor.
    var tools = [post(26, 39, REAR + 1)], labels = ["rotor bore"];
    for (i in 0...6) {
      var a = i * Math.PI / 3;
      tools.push(post(3.3, 39, 61, 53.5 * Math.cos(a), 53.5 * Math.sin(a)));
      labels.push('flange bolt ${i + 1}');
    }
    for (p in [[-37.5, 0.0], [37.5, 0.0], [0.0, -37.5], [0.0, 37.5]]) {
      tools.push(post(2.1, 231, 243, p[0], p[1]));
      labels.push("reaction tap " + (p[0] < 0 ? "left" : p[0] > 0 ? "right" : p[1] < 0 ? "front" : "back"));
    }
    return b.label(b.cutAll(body, tools, labels), "Rotor bore, flange bolts and reaction taps");
  }
}
