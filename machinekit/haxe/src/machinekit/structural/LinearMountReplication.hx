package machinekit.structural;

import cadkit.modeling.Part;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Physical study layer. It cannot supply steel threads, a product selection,
 * thickness/creep qualification or machining accuracy evidence. */
class LinearMountReplication extends MachineComponent {
  static var recipe:Null<ComponentType>;
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
    if (recipe == null) recipe = new ComponentType(
      "machinekit.structural.linear-mount-replication",
      inputs(),
      v -> new LinearMountReplication(v),
      false,
      false,
      null,
      false
    );
    return recipe;
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
      materia.casting.InsertStages.validateServiceHole(
        hole,
        {x: -width / 2, y: -length / 2, z: 0.0, width: width, depth: length, height: thickness}
      );
    }
    super("LINEAR-MOUNT-REPLICATION",
      "Unqualified replication study layer", "replication compound (assumed)", false, v);
    lockManufacturingMaterial("replication compound (assumed)");
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
