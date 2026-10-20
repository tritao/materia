package machinekit.routing;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import materia.assembly.AssemblyFrames as F;

/** Assumed catalogue-class linear covered tool rack. Dimensions mm; SI ratings unless stated.
 * Geometry is a nominal layout envelope, not manufacturing or supplier geometry. */
class ToolRack extends AssumedCatalogComponent {
  static var recipe:Null<ComponentType>;
  static function inputs():Array<ComponentParameter> return [
    P.count("pockets", 8),
    P.length("pitch", 110),
    P.length("depth", 140),
    P.length("height", 80),
    P.length("forkHeight", 100),
    P.length("coverStroke", 180),
    P.length("coverBore", 25),
    P.length("wall", 4)
  ];
  public function new(?v:ComponentValues) {
    super("tool-rack", "linear covered tool rack", "steel", inputs(), v);
    positive(["pitch", "depth", "height", "forkHeight", "coverStroke", "coverBore", "wall"]);
    require(parameterCount("pockets") > 0, "rack has no pockets");
    require(2 * n("wall") < Math.min(n("height"), n("depth")), "rack walls consume section");
    coverCylinder();
    require(n("forkHeight") >= n("height"), "fork pockets below rack base");
    addConnector("mount", Mount, F.translation(0, 0, 0));
    for (i in 0...parameterCount("pockets")) addConnector("pocket" +(i + 1), Mount, pocketFrame(i));
    addConnector("cover", Mount, F.translation(0, 0, n("height")));
    addPort({name: "cover_air", kind: Pneumatic, role: Consumer, iface: PushIn(6), required: true}
    );
  }
  public static function recipeType():ComponentType {
    if (recipe == null) recipe = new ComponentType("machinekit.routing.tool-rack", inputs(), v -> new ToolRack(v));
    return recipe;
  }
  override public function componentType():Null<ComponentType> return recipeType();
  override public function geometry(detail:ComponentDetail = Preview):Part {
    var length = parameterCount("pockets") * n("pitch");
    var outer = Solids.named(Part.box(length, n("depth"), n("height")), "rack-base");
    var blank = Part.box(length + 2, n("depth") - 2 * n("wall"), n("height") - 2 * n("wall"));
    var bore = blank.translated(new Vector(0, 0, n("wall")));
    blank.close();
    return Solids.cut(outer, [Solids.named(bore, "hollow-section")]);
  }
  public function coverCylinder():machinekit
    .pneumatic.PneumaticCylinder return new machinekit.pneumatic.PneumaticCylinder(
      "ISO6432",
      n("coverBore"),
      n("coverStroke"),
      200
    );
  public function pocketFrame(index:Int):materia.assembly.AssemblyRecord.AssemblyFrame {
    require(index >= 0 && index < parameterCount("pockets"), "pocket index outside rack");
    return Solids.axial((index -(parameterCount("pockets") - 1) / 2) * n("pitch"), 0, n("forkHeight"));
  }
  override public function io():MachineIo return {
    inputs:["cover_open",
    "cover_closed"].concat([for (i in 0...parameterCount("pockets")) "pocket" +(i + 1) + "_occupied"]),
    outputs:["cover"],
    analogOutputs:[]
  };

}
