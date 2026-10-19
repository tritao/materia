package machinekit.routing;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import materia.assembly.AssemblyFrames as F;

/** Assumed catalogue-class zoned vacuum plenum table. Dimensions mm; SI ratings unless stated.
 * Geometry is a nominal layout envelope, not manufacturing or supplier geometry. */
class VacuumTable extends AssumedCatalogComponent {
  static var recipe:Null<ComponentType>;
  static function inputs():Array<ComponentParameter> return [
    P.length("width", 1600),
    P.length("length", 2600),
    P.length("height", 50),
    P.length("wall", 15),
    P.count("columns", 2),
    P.count("rows", 3),
    P.length("gridPitch", 50),
    P.length("grooveWidth", 6),
    P.length("grooveDepth", 5),
    P.length("zonePortDiameter", 50)
  ];
  public function new(?v:ComponentValues) {
    super("vacuum-table", "zoned vacuum plenum table", "aluminium 6061", inputs(), v);
    positive(["width", "length", "height", "wall", "gridPitch", "grooveWidth", "grooveDepth", "zonePortDiameter"]);
    require(parameterCount("columns") > 0 && parameterCount("rows") > 0, "vacuum table needs zones");
    require(
      n("height") > 2 * n("wall") && n("wall") > n("grooveDepth") && n("grooveWidth") < n("gridPitch"),
      "invalid table wall/grooves"
    );
    require(
      n("width") / parameterCount("columns") > 2 * n("wall") && n("length") / parameterCount("rows") > 2 * n("wall"),
      "zone walls consume plenum"
    );
    require(n("zonePortDiameter")<Math.min(n("width")/parameterCount("columns")-2*n("wall"),n("length")/parameterCount("rows")-2*n("wall")),
      "vacuum port crosses zone walls");
    zoneValve(0);
    addConnector("mount", Mount, F.translation(0, 0, 0));
    addConnector("top", Face, F.translation(0, 0, n("height")));
    for (i in 0...zoneCount()) {
      addConnector("zone" +(i + 1), Mount, zoneFrame(i));
      addPort({
        name: "zone" +(i + 1),
        kind: Vacuum,
        role: Consumer,
        iface: Thread("G2"),
        required: true,
        connector: "zone" +(i + 1)
      }
      );
    }
  }
  public static function recipeType():ComponentType {
    if (recipe == null) recipe = new ComponentType("machinekit.routing.vacuum-table",
      inputs(), v -> new VacuumTable(v));
    return recipe;
  }
  override public function componentType():Null<ComponentType> return recipeType();
  override public function geometry(detail:ComponentDetail = Preview):Part {
    var tools:Array<Part> = [];
    return Solids.building(tools, owned -> {var base = Solids.named(
      Part.box(n("width"), n("length"), n("height")),
      "table"
    );
    owned.push(base);
    // Sealed plenums below a grooved top; each zone remains isolated.
    var w = n("width") / parameterCount("columns"), l = n("length") / parameterCount("rows");
    for (i in 0...zoneCount()) {
      var frame = zoneFrame(i), blank = Part.box(w - 2 * n("wall"), l - 2 * n("wall"), n("height") - 2 * n("wall"));
      var cut = blank.translated(new Vector(frame.x, frame.y, n("wall")));
      blank.close();
      owned.push(cut);
    }
    if (detail == Preview) for (i in 0...zoneCount()) {
      var f = zoneFrame(i), margin = n("wall"), gw = n("grooveWidth"), z = n("height") - n("grooveDepth");
      var nx = Std.int((w - 2 * margin) / n("gridPitch")), ny = Std.int((l - 2 * margin) / n("gridPitch"));
      for (ix in 0...nx) {
        var blank = Part.box(gw, l - 2 * margin, n("grooveDepth") + 1);
        var cut = blank.translated(new Vector(f.x +(ix -(nx - 1) / 2) * n("gridPitch"), f.y, z));
        blank.close();
        owned.push(cut);
      }
      for (iy in 0...ny) {
        var blank = Part.box(w - 2 * margin, gw, n("grooveDepth") + 1);
        var cut = blank.translated(new Vector(f.x, f.y +(iy -(ny - 1) / 2) * n("gridPitch"), z));
        blank.close();
        owned.push(cut);
      }
      // A nominal through-port ties each zone's top grid to its plenum.
      owned.push(Part.cylinderSpan(n("zonePortDiameter") / 2, -1, n("height") + 1, f.x, f.y));
    }
    return Solids.cut(base, owned.slice(1));
    }
    );
  }
  /** Separate catalogue parts so the machine's BOM includes every valve. */
  public function zoneValve(index:Int):VacuumZoneValve {
    zoneFrame(index);
    return new VacuumZoneValve(new ComponentValues().setNumber("bore", n("zonePortDiameter")));
  }
  public function zoneCount():Int return parameterCount("columns") * parameterCount("rows");
  public function zoneFrame(index:Int):materia.assembly.AssemblyRecord.AssemblyFrame {
    require(index >= 0 && index < zoneCount(), "zone index outside table");
    var c = index % parameterCount("columns"), r = Std.int(index / parameterCount("columns"));
    return Solids.axial(
      (c + 0.5) * n("width") / parameterCount("columns") - n("width") / 2,
      (r + 0.5) * n("length") / parameterCount("rows") - n("length") / 2,
      n("height")
    );
  }
  override public function io():MachineIo return {
    inputs:[for (i in 0...zoneCount()) "zone" +(i + 1) + "_pressure_ok"],
    outputs:[],
    analogOutputs:[]
  };

}
