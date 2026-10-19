package machinekit.control;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import materia.assembly.AssemblyFrames as F;

/** Assumed catalogue-class control cabinet with DIN rows and derived I/O. Dimensions mm; SI ratings unless stated.
 * Geometry is a nominal layout envelope, not manufacturing or supplier geometry. */
class ControlCabinet extends AssumedCatalogComponent {
  static var recipe:Null<ComponentType>;
  static function inputs():Array<ComponentParameter> return [
    P.length("width", 800),
    P.length("depth", 300),
    P.length("height", 1000),
    P.length("wall", 2),
    P.count("dinRows", 4),
    P.count("inputs", 0),
    P.count("outputs", 0),
    P.count("analogOutputs", 0)
  ];
  public function new(?v:ComponentValues) {
    super("control-cabinet", "control cabinet with DIN rows and derived I/O", "steel", inputs(), v);
    positive(["width", "depth", "height", "wall"]);
    require(parameterCount("dinRows") > 0, "cabinet needs DIN rows");
    require(
      n("height") /(parameterCount("dinRows") + 1) > 35
      && n("width") > 4 * n("wall") && n("depth") > 2 * n("wall") + 7.5,
      "DIN rows exceed cabinet interior"
    );
    require(2 * n("wall") < Math.min(n("width"), Math.min(n("depth"), n("height"))), "cabinet wall consumes interior");
    addConnector("mount", Mount, F.translation(0, 0, 0));
    for (i in 0...parameterCount("dinRows")) addConnector(
      "din" +(i + 1),
      Mount,
      F.translation(
        0,
        -n("depth") / 2 + n("wall") + 7.5,
        (i + 1) * n("height") /(parameterCount("dinRows") + 1)
      )
    );
  }
  public static function recipeType():ComponentType {
    if (recipe == null) recipe = new ComponentType(
      "machinekit.control.control-cabinet",
      inputs(),
      v -> new ControlCabinet(v)
    );
    return recipe;
  }
  override public function componentType():Null<ComponentType> return recipeType();
  override public function geometry(detail:ComponentDetail = Preview):Part {
    var outer = Solids.named(Part.box(n("width"), n("depth"), n("height")), "cabinet");
    var blank = Part.box(n("width") - 2 * n("wall"), n("depth"), n("height") - 2 * n("wall"));
    var tool = blank.translated(new Vector(0, n("wall"), n("wall")));
    blank.close();
    var shell = Solids.cut(outer, [Solids.named(tool, "interior")]);
    return Solids.building([shell], parts -> {for (i in 0...parameterCount("dinRows")) {
      var blank = Part.box(n("width") - 4 * n("wall"), 7.5, 35);
      var z =(i + 1) * n("height") /(parameterCount("dinRows") + 1) - 17.5;
      var rail = blank.translated(new Vector(0, -n("depth") / 2 + n("wall") + 3.75, z));
      blank.close();
      parts.push(Solids.named(rail, "din-row" +(i + 1)));
    }
    return Solids.union(parts);
    }
    );
  }
  /** Build-time count from actual components; channels remain named for SR17 wiring. */
  public static function forParts(parts:Array < {
    id:String, part:AssumedCatalogComponent
  }
  >, ?dimensions:ComponentValues):ControlCabinet {
    var v = dimensions == null ? new ComponentValues() : dimensions.copy(),
      inputs = 0, outputs = 0, analog = 0, seen : Map<
        String,
        Bool
    > = [];
    for (entry in parts) {
      if (entry.id == null
        || entry.id.length == 0 || seen.exists(entry.id)) throw "Cabinet component ids must be unique and nonempty";
      seen.set(entry.id, true);
      var channels = entry.part.io();
      inputs += channels.inputs.length;
      outputs += channels.outputs.length;
      analog += channels.analogOutputs.length;
    }
    return new ControlCabinet(v.setInteger("inputs", inputs).setInteger("outputs", outputs).setInteger(
      "analogOutputs",
      analog
    ));
  }

}
