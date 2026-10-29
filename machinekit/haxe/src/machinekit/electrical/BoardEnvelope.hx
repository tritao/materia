package machinekit.electrical;

import cadkit.PlanarRegion;
import cadkit.modeling.Part;
import machinekit.assembly.MachineAssembly;
import machinekit.assembly.PosedParts;
import machinekit.component.MachineComponent;
import machinekit.document.CanonicalJson;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Where a value comes from: a document (its URL and where in it), or a placeholder that says what is assumed. */
class EnvelopeSource {
  public final url:Null<String>;
  public final where:Null<String>;
  public final placeholder:Null<String>;

  function new(url:Null<String>, where:Null<String>, placeholder:Null<String>) {
    this.url = url;
    this.where = where;
    this.placeholder = placeholder;
  }

  public static function document(url:String, where:String):EnvelopeSource return new EnvelopeSource(url, where, null);

  public static function assumed(text:String):EnvelopeSource return new EnvelopeSource(null, null, text);

  public function json():JsonValue {
    var fields = new Map<String, JsonValue>();
    if (placeholder != null) fields.set("placeholder", JString(placeholder)) else {
      fields.set("url", JString(url));
      fields.set("where", JString(where));
    }
    return JObject(fields);
  }
}

/** What stands on one side of a mounting hole: a screw head, a standoff, a nut. */
class EnvelopeHardware {
  public final what:String;
  public final keepoutDiameter:Float;
  public final height:Float;
  public final source:EnvelopeSource;

  public function new(what:String, keepoutDiameter:Float, height:Float, source:EnvelopeSource) {
    this.what = what;
    this.keepoutDiameter = keepoutDiameter;
    this.height = height;
    this.source = source;
  }
}

/** A mounting hole of the board, at (x, y) of the board member's own plan, and the hardware on each side of it. */
class EnvelopeHole {
  public final name:String;
  public final x:Float;
  public final y:Float;
  public final drill:Float;
  public final plated:Bool;
  public final conductive:Bool;
  /** On the board's top side and on its bottom side. */
  public final top:EnvelopeHardware;
  public final bottom:EnvelopeHardware;
  public final source:EnvelopeSource;

  public function new(
    name:String,
    x:Float,
    y:Float,
    drill:Float,
    plated:Bool,
    conductive:Bool,
    top:EnvelopeHardware,
    bottom:EnvelopeHardware,
    source:EnvelopeSource
  ) {
    this.name = name;
    this.x = x;
    this.y = y;
    this.drill = drill;
    this.plated = plated;
    this.conductive = conductive;
    this.top = top;
    this.bottom = bottom;
    this.source = source;
  }
}

/** Wires that reach the board where no member of the assembly models their end yet (an opening for a part still to come). */
class EnvelopeOpening {
  public final name:String;
  public final x:Float;
  public final y:Float;
  public final wires:Int;
  public final ports:Array<String>;
  public final source:EnvelopeSource;

  public function new(name:String, x:Float, y:Float, wires:Int, ports:Array<String>, source:EnvelopeSource) {
    this.name = name;
    this.x = x;
    this.y = y;
    this.wires = wires;
    this.ports = ports;
    this.source = source;
  }
}

/** What the product says about its board; everything else is read from the assembly. */
class BoardEnvelopeSpec {
  /** pcbkit's id of the board. */
  public var board:String = "";

  public var productName:String = "";
  /** The model that writes the record (a class name), and its revision. */
  public var model:String = "";

  public var revision:String = "";
  /** The assembly member whose solid is the board: its outline, holes and thickness. */
  public var member:String = "";

  /** The board's top side (pcbkit's, where its parts stand) faces +z of the member's own frame; -z when false. */
  public var topUp:Bool = true;

  public var topFaces:String = "";
  public var bottomFaces:String = "";
  public var thicknessSource:EnvelopeSource = EnvelopeSource.assumed("the board solid's thickness");

  /** Source of what is read from the model's geometry (outline, height zones): placeholders until its parts are measured. */
  public var geometrySource:EnvelopeSource = EnvelopeSource.assumed("the product model's geometry");

  public var holes:Array<EnvelopeHole> = [];
  /** Members that are the board's own mounting hardware (their room is in `holes`), by path or path prefix ending in "/". */
  public var mounting:Array<String> = [];

  public var openings:Array<EnvelopeOpening> = [];
  /** How far from where its wires arrive a connector may sit, and what that rests on. */
  public var connectionDistance:Float = 20.0;

  public var connectionSource:EnvelopeSource = EnvelopeSource.assumed("harness slack");
  public var connectionSide:String = "top";
  public var connectionEntry:String = "horizontal";

  /** The height map's resolution and how far from the board it looks, mm; the outline's chord tolerance for arcs. */
  public var step:Float = 0.5;

  public var reach:Float = 40.0;
  public var chord:Float = 0.05;

  public function new() {
  }
}

/**
 * A board's enclosure envelope (pcbkit's docs/ENCLOSURE.md, schema 1), read from the assembly the board is built into, so it
 * follows the product model and no number is copied by hand:
 *
 * - the outline is the board member's solid cut at mid-thickness: its outer loop, arcs cut into chords; inner loops must
 *   be the declared mounting holes (pcbkit's outline is one simple polygon);
 * - the height zones are a height map of every other member over the board, side by side: the member's solid is cut at
 *   `step`, 2 `step`, ... from the board's surface, and the first cut that meets it gives, over that cut's bounding
 *   rectangles, the room to it (one `step` less, so it never overstates). The back wall, ribs, posts and the bodies of
 *   panel parts all come out of this; zones only restrict, so a conservative rectangle stays safe;
 * - the wire arrivals are the board's port connections: each other end's connector frame, projected onto the board, with
 *   as many wires as its ports' plugs have pins; openings add wires no member models yet;
 * - the frame maps the board member's own plan (x right, y up, seen from +z) to pcbkit's board frame (x right, y down,
 *   seen from the board's top side, origin at the outline's top left corner).
 */
class BoardEnvelope {
  public static function derive(assembly:MachineAssembly, spec:BoardEnvelopeSpec):JsonValue {
    var components = new Map<String, MachineComponent>();
    for (c in assembly.components()) components.set(c.id, c.component);
    var boardComponent = need(components, spec.member);
    var poses = assembly.solvedPoses();
    var toBoard = AssemblyFrames.inverse(need(poses, spec.member));
    var result:Null<JsonValue> = null;
    PosedParts.scope(posed -> {var solid = posed.local(boardComponent);
    var box = PosedParts.boxOf(solid);
    var outline = outlinePoints(solid, (box.minZ + box.maxZ) / 2, spec);
    var sx = spec.topUp ? 1.0 : -1.0;
    var ox = 1e18, oy = -1e18;
    for (p in outline) {
      ox = Math.min(ox, sx * p.x);
      oy = Math.max(oy, p.y);
    }
    ox = -ox;
    function bx(x:Float):Float return sx * x + ox;
    function by(y:Float):Float return -y + oy;
    function rect(x0:Float, y0:Float, x1:Float, y1:Float):JsonValue {
      var a = bx(x0), b = bx(x1), c = by(y0), d = by(y1);
      return JArray([JNumber(Math.min(a,
        b)), JNumber(Math.min(c, d)), JNumber(Math.max(a, b)), JNumber(Math.max(c, d))]);
    }
    var fields = new Map<String, JsonValue>();
    fields.set("schema", JNumber(1));
    fields.set("board", JString(spec.board));
    fields.set(
      "product",
      obj(["name" => JString(spec.productName), "model" => JString(spec.model), "revision" => JString(spec.revision)])
    );
    fields.set(
      "frame",
      obj([
        "note" => JString("the board member's own plan (x right, y up, seen from +z); the board's top side faces "
      + (spec.topUp ? "+z" : "-z, so the plan is seen mirrored")),
      "matrix" => JArray([JArray([JNumber(sx), JNumber(0)]), JArray([JNumber(0), JNumber(-1)])]),
      "offset_mm" => JArray([JNumber(ox), JNumber(oy)])
    ])
    );
    fields.set("thickness_mm", sourced(box.maxZ - box.minZ, spec.thicknessSource));
    fields.set(
      "sides",
      obj(["top" => obj(["faces" => JString(spec.topFaces)]), "bottom" => obj(["faces" => JString(spec.bottomFaces)])])
    );
    fields.set(
      "outline",
      obj([
        "polygon_mm" => JArray([for (p in outline) JArray([JNumber(bx(p.x)), JNumber(by(p.y))])]),
        "source" => spec.geometrySource.json()
      ])
    );
    var holes = new Map<String, JsonValue>();
    for (h in spec.holes) holes.set(
      h.name,
      obj([
        "pos_mm" => JArray([JNumber(bx(h.x)), JNumber(by(h.y))]),
        "drill_mm" => sourced(h.drill, h.source),
        "plated" => JBool(h.plated),
        "conductive" => JBool(h.conductive),
        "hardware" => obj(["top" => hardware(h.top), "bottom" => hardware(h.bottom)]),
        "source" => h.source.json()
      ])
    );
    fields.set("holes", JObject(holes));
    var footprint = PlanarRegion.section(solid.shape, (box.minZ + box.maxZ) / 2);
    try fields.set("zones",
      zones(posed, components, poses, toBoard, box, footprint, spec, rect)) catch (error:Dynamic) {
      footprint.shape.close();
      throw error;
    }
    footprint.shape.close();
    fields.set("connections", connections(assembly, components, poses, toBoard, spec, bx, by));
    result = JObject(fields);
    });
    return result;
  }

  /** The record as pcbkit reads it: canonical JSON. */
  public static function write(assembly:MachineAssembly,
    spec:BoardEnvelopeSpec):String return CanonicalJson.write(derive(
      assembly,
      spec
    ));

  static function outlinePoints(solid:Part, z:Float, spec:BoardEnvelopeSpec):Array<{x:Float, y:Float}> {
    var region = PlanarRegion.section(solid.shape, z);
    try {
      var loops = region.loops();
      var outer = [for (l in loops) if (l.outer) l];
      if (outer.length
        != 1) throw 'BoardEnvelope: the board solid has ${outer.length} pieces at mid-thickness, not one';
      for (l in loops) if (!l.outer && !isDeclaredHole(l,
        spec)) throw "BoardEnvelope: the board has an interior cut-out that is "
        + "not a declared mounting hole; pcbkit's outline is one simple polygon, so open it to an edge or declare the hole";
      var points:Array<{x:Float, y:Float}> = [];
      for (s in outer[0].segments) {
        if (!s.arc) {
          points.push({x: s.x0, y: s.y0});
          continue;
        }
        var step = s.radius > spec.chord ? 2 * Math.acos(1 - spec.chord / s.radius) : Math.PI / 4;
        var n = Std.int(Math.max(1, Math.ceil(Math.abs(s.sweep) / step)));
        for (i in 0...n) {
          var a = s.startAngle + s.sweep * i / n;
          points.push({x: s.cx + s.radius * Math.cos(a), y: s.cy + s.radius * Math.sin(a)});
        }
      }
      region.shape.close();
      return points;
    } catch (error:Dynamic) {
      region.shape.close();
      throw error;
    }
  }

  static function isDeclaredHole(loop:cadkit.PlanarRegion.PlanarLoop, spec:BoardEnvelopeSpec):Bool {
    if (loop.segments.length == 0 || !loop.segments[0].arc) return false;
    var s = loop.segments[0];
    for (h in spec.holes) if (Math.abs(s.cx - h.x) < 0.01
      && Math.abs(s.cy - h.y) < 0.01 && Math.abs(2 * s.radius - h.drill) < 0.01) return true;
    return false;
  }

  static function isMounting(id:String, spec:BoardEnvelopeSpec):Bool {
    for (m in spec.mounting) if (id == m || (StringTools.endsWith(m,
      "/") && StringTools.startsWith(id, m))) return true;
    return false;
  }

  static function zones(
    posed:PosedParts,
    components:Map<String, MachineComponent>,
    poses:Map<String, AssemblyFrame>,
    toBoard:AssemblyFrame,
    board:machinekit.assembly.PosedParts.PartBox,
    footprint:PlanarRegion,
    spec:BoardEnvelopeSpec,
    rect:(Float, Float, Float, Float) -> JsonValue
  ):JsonValue {
    // Every rectangle of material over the board, with the room under it; then the ones a lower one already covers go.
    var found:Array<{side:String, id:String, level:Int, x0:Float, y0:Float, x1:Float, y1:Float}> = [];
    var ids = [for (id in components.keys()) id];
    ids.sort((a, b) -> a < b ? -1 : (a > b ? 1 : 0));
    for (side in ["top", "bottom"]) {
      var dir = (side == "top") == spec.topUp ? 1.0 : -1.0;
      var surface = dir > 0 ? board.maxZ : board.minZ;
      for (id in ids) {
        if (id == spec.member || isMounting(id, spec)) continue;
        var component = need(components, id);
        var pose = AssemblyFrames.compose(toBoard, need(poses, id));
        var b = posed.box(component, pose);
        if (b.maxX <= board.minX || b.minX >= board.maxX || b.maxY <= board.minY || b.minY >= board.maxY) continue;
        var near = dir > 0 ? b.minZ - surface : surface - b.maxZ;
        var far = dir > 0 ? b.maxZ - surface : surface - b.minZ;
        if (far <= 0 || near >= spec.reach) continue;
        var part = posed.posed(component, pose);
        var first = Std.int(Math.max(1, Math.ceil(Math.max(near, 0) / spec.step)));
        var last = Std.int(Math.floor(Math.min(far, spec.reach) / spec.step));
        for (k in first...last + 1) {
          var cut = PlanarRegion.section(part.shape, surface + dir * k * spec.step);
          // Only material over the board itself: a wall around it or a post in a notch leaves the board all its room.
          var over = cut.isEmpty() ? null : cut.common(footprint);
          cut.shape.close();
          if (over == null) continue;
          if (!over.isEmpty()) for (loop in over.loops()) if (loop.outer) {
            var e = extent(loop);
            if (e.x1 - e.x0 > 1e-6 && e.y1 - e.y0 > 1e-6) found.push({
              side: side,
              id: id,
              level: k - 1,
              x0: e.x0,
              y0: e.y0,
              x1: e.x1,
              y1: e.y1
            });
          }
          over.shape.close();
        }
        part.close();
      }
    }
    // Zones only restrict: a rectangle inside a kept one of the same side with no more room adds nothing.
    found.sort((a, b) -> a.level != b.level ? a.level - b.level : (a.id < b.id ? -1 : (a.id > b.id ? 1 : 0)));
    var kept:Array<{side:String, id:String, level:Int, x0:Float, y0:Float, x1:Float, y1:Float}> = [];
    for (f in found) {
      var covered = false;
      for (k in kept) if (k.side == f.side
        && k.x0 <= f.x0 + 1e-6 && k.y0 <= f.y0 + 1e-6 && k.x1 >= f.x1 - 1e-6 && k.y1 >= f.y1 - 1e-6) covered = true;
      if (!covered) kept.push(f);
    }
    var groups = new Map<String, {side:String, id:String, level:Int, rects:Array<JsonValue>}>();
    for (k in kept) {
      var key = name(k.id) + "-" + k.side + "-" + k.level;
      if (!groups.exists(key)) groups.set(key, {side: k.side, id: k.id, level: k.level, rects: []});
      groups.get(key).rects.push(rect(k.x0, k.y0, k.x1, k.y1));
    }
    var result = new Map<String, JsonValue>();
    for (key => g in groups) result.set(
      key,
      obj([
        "side" => JString(g.side),
        "rects_mm" => JArray(g.rects),
        "max_height_mm" => sourced(g.level * spec.step, spec.geometrySource),
        "note"
      => JString('the room to ${g.id} (${need(components, g.id).designation}), read from its solid in ${spec.step} mm steps')
    ])
    );
    return JObject(result);
  }

  static function connections(
    assembly:MachineAssembly,
    components:Map<String, MachineComponent>,
    poses:Map<String, AssemblyFrame>,
    toBoard:AssemblyFrame,
    spec:BoardEnvelopeSpec,
    bx:Float -> Float,
    by:Float -> Float
  ):JsonValue {
    // Group the board's connections by the other end's connector: one place where a set of wires arrives.
    var groups = new Map<String, {instance:String, connector:Null<String>, ports:Array<String>, wires:Int}>();
    for (c in assembly.portConnections()) {
      var other = c.fromInstance == spec.member ? {
        instance:c.toInstance, port:c.toPort
      }
      :c.toInstance == spec.member ? {
        instance:c.fromInstance, port:c.fromPort
      }
      :null;
      if (other == null) continue;
      var component = need(components, other.instance);
      var found:Null<machinekit.component.ComponentPort> = null;
      for (p in component.ports()) if (p.name == other.port) found = p;
      if (found == null) throw 'BoardEnvelope: ${other.instance} has no port ${other.port}';
      var port:machinekit.component.ComponentPort = found;
      var key = other.instance + "|" + (port.connector == null ? "" : port.connector);
      if (!groups.exists(key)) groups.set(
        key,
        {instance: other.instance, connector: port.connector, ports: [], wires: 0}
      );
      var g = groups.get(key);
      g.ports.push(other.instance + ":" + other.port);
      g.wires += switch port.iface {
        case Plug(_, pins):
          pins;
        default:
          1;
      };
    }
    var perInstance = new Map<String, Int>();
    for (g in groups) {
      var seen:Null<Int> = perInstance.get(g.instance);
      perInstance.set(g.instance, (seen == null ? 0 : seen) + 1);
    }
    var result = new Map<String, JsonValue>();
    for (g in groups) {
      var connector = g.connector;
      var frame = connector == null ? AssemblyFrames.identity() : need(components,
        g.instance).connector(connector).frame;
      var at = AssemblyFrames.compose(toBoard, AssemblyFrames.compose(need(poses, g.instance), frame));
      g.ports.sort((a, b) -> a < b ? -1 : (a > b ? 1 : 0));
      var count:Null<Int> = perInstance.get(g.instance);
      var key = name(g.instance) + (count != null && count > 1 && connector != null ? "-" + name(connector) : "");
      result.set(key, connection(bx(at.x), by(at.y), g.wires, g.ports, spec, spec.geometrySource));
    }
    for (o in spec.openings) result.set(o.name, connection(bx(o.x), by(o.y), o.wires, o.ports, spec, o.source));
    return JObject(result);
  }

  static function connection(
    x:Float,
    y:Float,
    wires:Int,
    ports:Array<String>,
    spec:BoardEnvelopeSpec,
    source:EnvelopeSource
  ):JsonValue return obj([
    "point_mm" => JArray([JNumber(x), JNumber(y)]),
    "max_distance_mm" => sourced(spec.connectionDistance, spec.connectionSource),
    "wires" => JNumber(wires),
    "ports" => JArray([for (p in ports) JString(p)]),
    "interface" => JNull,
    "side" => JString(spec.connectionSide),
    "entry" => JString(spec.connectionEntry),
    "source" => source.json()
  ]);

  static function hardware(h:EnvelopeHardware):JsonValue return obj([
    "what" => JString(h.what),
    "keepout_diameter_mm" => sourced(h.keepoutDiameter, h.source),
    "height_mm" => sourced(h.height, h.source)
  ]);

  static function sourced(value:Float, source:EnvelopeSource):JsonValue return obj([
    "value" => JNumber(value),
    "source" => source.json()
  ]);

  static function obj(fields:Map<String, JsonValue>):JsonValue return JObject(fields);

  /** A member path as a record key: lower case, every other character a dash. */
  static function name(id:String):String {
    var out = new StringBuf();
    for (i in 0...id.length) {
      var c = id.charAt(i).toLowerCase();
      out.add((c >= "a" && c <= "z") || (c >= "0" && c <= "9") || c == "_" ? c : "-");
    }
    return out.toString();
  }

  /** The bounding rectangle of a loop; an arc counts with its whole circle, which only makes the rectangle larger. */
  static function extent(loop:cadkit.PlanarRegion.PlanarLoop): {x0: Float, y0: Float, x1: Float, y1: Float} {
    var x0 = 1e18, y0 = 1e18, x1 = -1e18, y1 = -1e18;
    for (s in loop.segments) {
      if (s.arc) {
        x0 = Math.min(x0, s.cx - s.radius);
        x1 = Math.max(x1, s.cx + s.radius);
        y0 = Math.min(y0, s.cy - s.radius);
        y1 = Math.max(y1, s.cy + s.radius);
      }
      x0 = Math.min(x0, Math.min(s.x0, s.x1));
      x1 = Math.max(x1, Math.max(s.x0, s.x1));
      y0 = Math.min(y0, Math.min(s.y0, s.y1));
      y1 = Math.max(y1, Math.max(s.y0, s.y1));
    }
    return {x0: x0, y0: y0, x1: x1, y1: y1};
  }

  static function need<T>(map:Map<String, T>, key:String):T {
    var value = map.get(key);
    if (value == null) throw 'BoardEnvelope: no member "$key"';
    return value;
  }
}
