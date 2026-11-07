package machinekit.structural;

import cadkit.modeling.Part;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Physical study layer. It cannot supply steel threads, a product selection,
 * thickness/creep qualification or machining accuracy evidence. */
class LinearMountReplication extends MachineComponent {
  static final recipe = new machinekit.cache.Once<ComponentType>();
  public final width:Float;
  public final length:Float;
  public final thickness:Float;
  final holes:Array<materia.casting.InsertHole>;
  static function inputs():Array<ComponentParameter> return [
    P.length("width", 140),
    P.length("length", 540),
    P.length("thickness", 4),
    P.text("holes", InsertHoles.fromNeutral([])),
    P.text("reference", "Engineering sensitivity: E=4 GPa, density=1800 kg/m3; product/creep unqualified")
  ];
  public static function recipeType():ComponentType {
    return recipe.get(() -> new ComponentType(
      "machinekit.structural.linear-mount-replication",
      inputs(),
      v -> new LinearMountReplication(v),
      false,
      false,
      null,
      false
    ).numbered(v -> 'LM-REPLICATION-${Dimension.format(v.number("width"))}x${Dimension.format(v.number("length"))}x${Dimension.format(v.number("thickness"))}-'
      + haxe.crypto.Sha256.encode(v.token("holes") + "|" + v.token("reference")).substr(0, 12)).purchaseKeyed(v -> {
      var holes = InsertHoles.records(v.token("holes"));
      for (i in 0...holes.length) holes[i].id = Std.string(i);
      return [ComponentNumberKey.encode(v.number("width")), ComponentNumberKey.encode(v.number("length")),
        ComponentNumberKey.encode(v.number("thickness")), InsertHoles.fromNeutral(holes), v.token("reference")].join("|");
    }).procured(_ -> Make("casting replication compound"))
      .requiringInputs(["width", "length", "thickness"], [], []));
  }
  public function new(?overrides:ComponentValues) {
    var v = P.resolve(inputs(), overrides, false);
    width = v.number("width");
    length = v.number("length");
    thickness = v.number("thickness");
    for (n in [width, length, thickness]) if (!Math.isFinite(n) || n <= 0) throw "Invalid replication study geometry";
    if (StringTools.trim(v.token("reference")) == "") throw "Replication assumptions need provenance";
    holes = InsertHoles.records(v.token("holes"));
    for (hole in holes) {
      if (hole.thread != null || hole.extent != Through) throw "Replication is not a threaded clamp member";
      // This layer supplies neither steel threads nor steel insert ligament.
      // Its inherited clearance must remain inside the footprint, with the
      // declared one-mm compound edge wall (engineering assumption).
      var mouth = materia.assembly.AssemblyFrames.transformPoint(hole.frame, 0, 0, 0);
      var axis = materia.assembly.AssemblyFrames.transformVector(hole.frame, 0, 0, 1);
      var wall = hole.diameter / 2 + 1.0;
      if (Math.abs(axis.x) > 1e-9 || Math.abs(axis.y) > 1e-9 || Math.abs(axis.z) < 0.999999
        || Math.abs(mouth.x) + wall > width / 2 + 1e-9
        || Math.abs(mouth.y) + wall > length / 2 + 1e-9
        || Math.abs(mouth.z - thickness) > 1e-9 || axis.z > 0
        || Math.abs(hole.depth - thickness) > 1e-9)
        throw "Replication clearance must cross the layer normally and retain one mm ASSUMED edge wall: " + hole.id;
    }
    super("LINEAR-MOUNT-REPLICATION",
      "Unqualified replication study layer", "replication compound (assumed)", false, v);
    addFacet(new EngineeringMetadata(new ComponentValues().setToken("errorsBasis", "Assumed").setToken("source", "").setToken("ratingsBasis", "Assumed")));
    lockManufacturingMaterial("replication compound (assumed)");
    addFacet(new InterfaceLayer(thickness));
    addConnector("mount", Mount, Solids.axial(0, 0, 0));
    addConnector("top", Mount, Solids.axial(0, 0, thickness));
  }
  override public function componentType():Null<ComponentType> return recipeType();
  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail = Preview):Part return Solids.cut(
    Part.box(width, length, thickness),
    InsertHoles.facet(holes, width, length, thickness).drillingTools()
  );
}
