package machinekit.component;

import machinekit.structural.GroundStock.GroundStockRow;

/** How a made part is produced. */
enum RouteKind {
  /** Cut from ground plate or flat bar; only the faces named in `machinedFaces` are touched after grinding. */
  GroundStock;
  /** Laser- or waterjet-cut plate, flat as rolled. */
  CutPlate;
  /** CNC milled from a block, in `setups` setups on a 3-axis mill. */
  Milled;
  /** Turned on a lathe, in `setups` setups. */
  Turned;
}

/** A machined face and what is asked of it. Faces not listed stay as the stock or the cut leaves them. */
typedef MachinedFace = {
  /** The part's own name for the face. */
  var name:String;
  /** Flatness (or position, for a bore) the function needs, mm. */
  var tolerance:Float;
  /** Surface roughness Ra, micrometres, or null when the process leaves it. */
  var roughnessRa:Null<Float>;
}

/**
 * A hole a made part carries, in the part's own frame: what the drawing's hole table lists. `kind` is `"tapped"` (cut at the tap
 * drill `diameter`, thread `thread`, to `depth`), `"clearance"` (a bolt of size `thread` passes), `"dowel"` (drilled `diameter`,
 * reamed to `fit` at assembly) or `"bore"` (a plain bore, `nominal` the diameter it is for). `position` is the mouth, `direction`
 * the unit vector into the part, `depth` measured from the mouth along it.
 */
typedef RouteHole = {
  var id:String;
  var kind:String;
  var thread:Null<String>;
  /** As cut in the model's solid, mm. */
  var diameter:Float;
  /** The thread's major diameter, the dowel's or the bore's finished diameter, mm. */
  var nominal:Float;
  var fit:Null<String>;
  var depth:Float;
  var position:Array<Float>;
  var direction:Array<Float>;
}

/**
 * The production route of a made part: which stock or process, how many setups and what tolerance class, and which faces
 * are machined. The route states its process text (`process()`), which the BOM carries as the part's
 * `manufacturingProcess`, and which supplier group the part goes to (`supplier()`); the faces stay with the route for the
 * drawing package.
 *
 * Rules from the hobber frame plan: a milled part fits a 3-axis mill (at most `MAX_SETUPS` setups, longest side at most
 * `MAX_EXTENT` mm) and asks tight tolerances only of its listed functional faces.
 */
class ManufacturingRoute implements ComponentFacet {
  public static inline var MAX_SETUPS:Int = 2;
  public static inline var MAX_EXTENT:Float = 600;
  /** Tolerance classes of a milled or turned part, loosest first. */
  public static function toleranceClasses():Array<String> return ["general", "fine", "tight"];

  public final kind:RouteKind;
  public final setups:Int;
  public final toleranceClass:String;
  public final stock:Null<GroundStockRow>;
  final faces:Array<MachinedFace>;

  function new(kind:RouteKind, setups:Int, toleranceClass:String, stock:Null<GroundStockRow>, machined:Array<MachinedFace>) {
    if (setups < 0) throw "A route has a non-negative number of setups";
    if (toleranceClasses().indexOf(toleranceClass) < 0) throw 'Unknown tolerance class "$toleranceClass"';
    for (face in machined) {
      if (face.name == null || face.name.length == 0) throw "A machined face needs a name";
      if (!(face.tolerance > 0) || !Math.isFinite(face.tolerance)) throw 'Machined face "${face.name}" needs a positive tolerance';
    }
    this.kind = kind;
    this.setups = setups;
    this.toleranceClass = toleranceClass;
    this.stock = stock;
    faces = [for (face in machined) {name: face.name, tolerance: face.tolerance, roughnessRa: face.roughnessRa}];
  }

  /** Cut from ground stock `row`; `machined` lists the faces that are drilled, tapped or finished after cutting. */
  public static function groundStock(row:GroundStockRow, ?machined:Array<MachinedFace>):ManufacturingRoute {
    if (row == null) throw "A ground stock route needs its stock row";
    var list = machined == null ? [] : machined;
    return new ManufacturingRoute(GroundStock, list.length == 0 ? 0 : 1, "general", row, list);
  }

  /** Laser- or waterjet-cut plate; holes and the profile come with the cut. */
  public static function cutPlate():ManufacturingRoute return new ManufacturingRoute(CutPlate, 0, "general", null, []);

  /**
	 * Milled in `setups` setups. `extent` is the longest side of the finished part, mm. A tight class needs at least one
	 * listed face to be tight about; listing a face is how a drawing says which ones.
	 */
  public static function milled(setups:Int, toleranceClass:String, machined:Array<MachinedFace>, extent:Float):ManufacturingRoute {
    if (setups < 1 || setups > MAX_SETUPS) throw 'A milled part takes 1 to $MAX_SETUPS setups, not $setups';
    if (!(extent > 0) || extent > MAX_EXTENT) throw 'A milled part is at most ${Dimension.format(MAX_EXTENT)} mm long, not ${Dimension.format(extent)}';
    if (toleranceClass != "general" && machined.length == 0) throw "A milled part with a fine or tight class lists the faces it applies to";
    return new ManufacturingRoute(Milled, setups, toleranceClass, null, machined);
  }

  /** Turned in `setups` setups (one per chucking). */
  public static function turned(setups:Int, toleranceClass:String, machined:Array<MachinedFace>):ManufacturingRoute {
    if (setups < 1 || setups > MAX_SETUPS) throw 'A turned part takes 1 to $MAX_SETUPS setups, not $setups';
    return new ManufacturingRoute(Turned, setups, toleranceClass, null, machined);
  }

  /** The machined faces, copied. */
  public function machinedFaces():Array<MachinedFace> return [for (face in faces) {name: face.name, tolerance: face.tolerance, roughnessRa: face.roughnessRa}];

  /** The process text a `Make` procurement and the BOM's `manufacturingProcess` carry. */
  public function process():String return switch kind {
    case GroundStock:
      var row = stock;
      if (row == null) throw "Ground stock route lost its row";
      "ground " + row.kind + " " + Dimension.format(row.thickness) + " mm, cut" + (faces.length == 0 ? "" : " and machined");
    case CutPlate:
      "cut plate";
    case Milled:
      "CNC milling, " + setups + (setups == 1 ? " setup, " : " setups, ") + toleranceClass + " tolerance";
    case Turned:
      "turning, " + setups + (setups == 1 ? " setup, " : " setups, ") + toleranceClass + " tolerance";
  };

  /** Who the part goes to. */
  public function supplier():String return switch kind {
    case GroundStock:
      faces.length == 0 ? "stock supplier" : "stock supplier and machine shop";
    case CutPlate:
      "plate cutter";
    case Milled:
      "machine shop";
    case Turned:
      "lathe shop";
  };

  public function check(component:MachineComponent):Void {
    var duplicate = ManufacturingRoute.of(component);
    if (duplicate != null) throw 'Duplicate manufacturing route on "${component.designation}"';
  }

  public function describe():String {
    var text = "route: " + process() + " (" + supplier() + ")";
    if (faces.length > 0) text += "; machined: " + [for (face in faces) face.name + " ±" + Dimension.format(face.tolerance)].join(", ");
    return text;
  }

  public static function of(component:MachineComponent):Null<ManufacturingRoute> {
    var result:Null<ManufacturingRoute> = null;
    for (facet in component.facets()) if (Std.isOfType(facet, ManufacturingRoute)) {
      if (result != null) throw 'Duplicate manufacturing route on "${component.designation}"';
      result = cast facet;
    }
    return result;
  }
}
