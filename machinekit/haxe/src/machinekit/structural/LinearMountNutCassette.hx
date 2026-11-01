package machinekit.structural;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import materia.assembly.AssemblyFrames as F;

typedef MountNutStation = {
  var connector:String;
  var x:Float;
  var y:Float;
}

/** Removable anti-rotation cage. Nuts/washer bearing bypass the cage and bear
 * directly on the host underside. A separate drilled cover prevents loss when
 * the bolts are withdrawn. Neither cage nor cover substitutes for a nut.
 * Frame: XY stock centred on origin, bottom z=0, washer mouths at z=10.
 */
class LinearMountNutCassette extends MachineComponent {
  static var recipe:Null<ComponentType>;
  public final length:Float;
  public final width:Float;
  public final height:Float;
  public final nut:machinekit.standard.HexNut;
  public final washer:machinekit.standard.FlatWasher;
  final stations:Array<MountNutStation>;
  public final sourceFingerprint:String;
  public final compact:Bool;
  static function inputs():Array<ComponentParameter> return [
    P.length("length", 440),
    P.length("width", 24),
    P.choice("size", ["M6"], "M6"),
    P.choice("layout", ["end-service", "compact-centre-service"], "end-service"),
    P.text("stations", "[]"),
    P.text("sourceFingerprint", "")
  ];
  public static function recipeType():ComponentType {
    if (recipe == null) recipe = new ComponentType(
      "machinekit.structural.linear-mount-nut-cassette",
      inputs(),
      v -> new LinearMountNutCassette(v),
      false,
      false,
      null,
      false,
      new ComponentValues().setToken(
        "sourceFingerprint",
        haxe.crypto.Sha256.encode("metadata-only; derive production source from MountedElement")
      ).setToken(
        "stations",
        '[{"connector":"metadata-example","x":0,"y":0}]'
      )
    ).numbered(v -> 'NUT-CASSETTE-${v.token("size")}-${v.token("layout").toUpperCase()}').purchaseKeyed(v -> {
      var rows:Array<Dynamic> = haxe.Json.parse(v.token("stations"));
      var coordinates = [for (row in rows) ComponentNumberKey.encode(row.x) + "," + ComponentNumberKey.encode(row.y)];
      return [ComponentNumberKey.encode(v.number("length")), ComponentNumberKey.encode(v.number("width")),
        v.token("size"), v.token("layout"), coordinates.join(";")].join("|");
    });
    return recipe;
  }
  public function new(?overrides:ComponentValues) {
    var v = P.resolve(inputs(), overrides, false);
    length = v.number("length");
    width = v.number("width");
    height = 10;
    compact = v.token("layout") == "compact-centre-service";
    nut = machinekit.standard.HexNut.metric(v.token("size"));
    washer = machinekit.standard.FlatWasher.metric(v.token("size"));
    sourceFingerprint = v.token("sourceFingerprint");
    if (!~/^[0-9a-f]{64}$/.match(sourceFingerprint)) throw "Nut cassette needs its mounted-element source identity";
    if (!Math.isFinite(length) || !Math.isFinite(width)
      || length <= 0 || width < washer.outerDiameter + 4) throw "Nut cassette stock does not enclose its hardware";
    stations = [];
    var raw:Array<Dynamic> = haxe.Json.parse(v.token("stations")), seen:Map<String, Bool> = [];
    if (raw == null || raw.length == 0) throw "Nut cassette needs actual element connectors";
    for (row in raw) {
      var id:String = row.connector;
      if (id == null || StringTools.trim(id) == "" || seen.exists(id)) throw "Invalid nut cassette connector identity";
      seen.set(id, true);
      if (row.x == null || row.y == null) throw "Missing nut cassette coordinates";
      var x:Float = row.x, y:Float = row.y, r = washer.outerDiameter / 2 + 0.25;
      if (!Math.isFinite(x) || !Math.isFinite(y) || Math.abs(x) + r + 2 > width / 2 || Math.abs(y) + r + (compact
        ? 2 : 14) > length / 2) throw "Nut cassette pocket breaks its stock/end fastener allowance: " + id;
      for (previous in stations) if (Math.sqrt(Math.pow(x - previous.x,
        2) + Math.pow(y - previous.y, 2)) < 2 * r + 2) throw "Nut cassette pockets overlap";
      stations.push({connector: id, x: x, y: y});
    }
    if (compact && (stations.length != 2
      || Math.abs(stations[0].x) + Math.abs(stations[1].x) > 1e-9 || Math.abs(stations[0].y + stations[1].y) > 1e-9
      || Math.abs(stations[0].y) < 18)) throw "Compact cassette requires the opposed two-bolt cleat pattern";
    v.setToken("stations", haxe.Json.stringify(stations));
    super("LINEAR-MOUNT-NUT-CASSETTE", "Removable M6 mount nut cassette", "steel C45", false, v);
    lockManufacturingMaterial("steel C45");
    var stock = {x: -width / 2, y: -length / 2, z: 0.0, width: width, depth: length, height: height};
    for (hole in serviceThreads()) materia.casting.InsertStages.validateServiceHole(hole, stock);
    addFacet(InsertHoles.facet(serviceThreads(), width, length, height));
    addConnector("mount", Mount, Solids.axial(0, 0, 0));
  }
  public static function fromElement(
    element:MountedElement,
    placement:materia.assembly.AssemblyRecord.AssemblyFrame,
    length:Float,
    width:Float = 24,
    compact:Bool = false
  ):LinearMountNutCassette {
    machinekit.design.DesignGeometry.validateFrame(placement);
    var rows:Array<MountNutStation> = [];
    for (c in element.connectors) {
      var pose = F.compose(placement, c.frame), normal = F.transformVector(pose, 0, 1, 0);
      if (Math.abs(normal.x) + Math.abs(normal.y) > 1e-6
        || normal.z < 1 - 1e-6) throw "Cassette requires mounted-element bolt axes normal to its top plane";
      rows.push({connector: c.name, x: pose.x, y: pose.y});
    }
    return new LinearMountNutCassette(new ComponentValues().setNumber("length",
      length).setNumber("width", width).setToken(
        "layout",
        compact ? "compact-centre-service" : "end-service"
      ).setToken(
        "size",
        element.thread.designation
      ).setToken(
        "stations",
        haxe.Json.stringify(rows)
      ).setToken(
        "sourceFingerprint",
        element.fingerprint("nut-cassette")
      ));
  }
  public function pocketStations():Array<MountNutStation> return stations.map(s -> ({
    connector: s.connector,
    x: s.x,
    y: s.y
  }));
  public function coverThreads():Array<materia.casting.InsertHole> {
    var choice = InsertHoles.metricThread(machinekit.standard.FastenerThreads.coarse(3).mating());
    return [for (end in [-1, 1]) {
      var hole = InsertHoles.records(InsertHoles.tappedBlind(choice, height, C45Bright))[0];
      hole.id = end < 0 ? "cover-front" : "cover-back";
      hole.frame = compact ? F.translation(0, end * 4, 0) : F.translation(
        end < 0 ? 6.5 : -6.5,
        end * (length / 2 - 8),
        0
      );
      hole;
    }
    ];
  }
  public function serviceThreads():Array<materia.casting.InsertHole> {
    var holes = coverThreads();
    if (compact) return holes; // Centre cover threads also take the retrieval tool.
    var choice = InsertHoles.metricThread(machinekit.standard.FastenerThreads.coarse(3).mating());
    for (end in [-1, 1]) {
      var hole = InsertHoles.records(InsertHoles.tappedBlind(choice, 7.75, C45Bright))[0];
      hole.id = end < 0 ? "pull-front" : "pull-back";
      hole.frame = F.fromRotationMatrix(
        0,
        end * length / 2,
        height / 2,
        end < 0 ? [1, 0, 0, 0, 0, 1, 0, -1, 0] : [1, 0, 0, 0, 0, -1, 0, 1, 0]
      );
      holes.push(hole);
    }
    return holes;
  }
  public function cover():LinearMountSteel {
    var holes:Array<materia.casting.InsertHole> = [];
    for (s in stations) {
      var hole = InsertHoles.records(InsertHoles.through(nut.spec.diameter + 1, 2))[0];
      if (compact) {
        // Loss-prevention sheet only; nuts and washers bear directly on the RHS.
        // A 100 N handling load is below 2 * 2 mm ligaments * 2 mm sheet *
        // 235/1.5 MPa. This is not a relaxation of any structural bolt/thread rule.
        hole.limits.minWall = 2;
        hole.limits.overrides.push({name: "minWall", value: 2});
      }
      hole.id = s.connector;
      hole.frame.x = s.x;
      hole.frame.y = s.y;
      holes.push(hole);
    }
    for (h in coverThreads()) {
      var hole = InsertHoles.records(InsertHoles.through(3.4, 2))[0];
      hole.id = h.id;
      hole.frame.x = h.frame.x;
      hole.frame.y = h.frame.y;
      holes.push(hole);
    }
    return new LinearMountSteel(new ComponentValues().setNumber("width", width).setNumber("length", length).setNumber(
      "thickness",
      2
    ).setToken(
      "role",
      "carrier"
    ).setToken(
      "holes",
      InsertHoles.fromNeutral(holes)
    ));
  }
  override public function componentType():Null<ComponentType> return recipeType();
  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail = Preview):Part {
    var tools = InsertHoles.facet(serviceThreads(), width, length, height).drillingTools();
    for (s in stations) {
      tools.push(Part.prism(cadkit.modeling.Polygon.regular(6,
        nut.acrossFlats + 0.5), -0.1, height - washer.thickness).translated(new Vector(
          s.x,
          s.y,
          0
        )));
      tools.push(Part.cylinderSpan((washer.outerDiameter + 0.5) / 2,
        height - washer.thickness, height + 0.1, s.x, s.y));
    }
    return Solids.cut(Part.box(width, length, height), tools);
  }
}
