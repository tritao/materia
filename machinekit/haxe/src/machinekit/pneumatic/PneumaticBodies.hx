package machinekit.pneumatic;

import cadkit.modeling.Align;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.Solids;

/**
 * Drawings of purchased pneumatic parts inside their envelope, frame x across (centred), y through (centred, front
 * at -y), z up from the mounting face at 0: valves with solenoid coils, manifolds, vacuum generators and sensors,
 * the FRL unit, suction cups and cylinder rods. Ports are drilled into faces rather than standing proud, so the
 * envelope stays the clearance shape; fittings are their own parts.
 */
class PneumaticBodies {
  /**
	 * A spool valve: a chamfered body over the spool length `spool`, solenoid coil blocks beyond it at one end, or
	 * both when `coils` is 2, and push-in ports drilled into the top at `ports` (x).
	 */
  public static function valve(width:Float,
    depth:Float, height:Float, spool:Float, coils:Int, ports:Array<Float>):Part {
    return build(keep -> {var c = Math.min(depth, height) * 0.08, coil = (width - spool) / coils;
    var pieces = [keep(chamferedBox(spool, depth, height, c, -spool / 2), "spool-body")];
    for (end in (coils == 2 ? [-1.0, 1.0] : [1.0])) {
      var x0 = end > 0 ? spool / 2 - 0.01 : -width / 2, x1 = end > 0 ? width / 2 : -spool / 2 + 0.01;
      pieces.push(keep(chamferedBox(x1 - x0, depth * 0.9, height * 0.95, c, x0), "coil"));
      // The manual-override button on the coil's top.
      pieces.push(keep(
        Part.cylinderSpan(Math.min(coil, depth) * 0.15, height * 0.95 - 0.01, height, (x0 + x1) / 2, 0),
        "override"
      ));
    }
    var tools = [for (x in ports) keep(
      Part.cylinderSpan(Math.min(depth * 0.18, 3.5), height * 0.6, height + 0.1, x, 0),
      "port"
    )];
    return {pieces: pieces, tools: tools};
    });
  }

  /** A manifold bar along z with `outlets` ports drilled into its front and the inlet into its base. */
  public static function manifold(width:Float, depth:Float, height:Float, outlets:Int):Part {
    return build(keep -> {var pieces = [keep(
      chamferedBox(width, depth, height, Math.min(width, depth) * 0.08, -width / 2),
      "bar"
    )];
    var pitch = (height - 12) / outlets, r = Math.min(width, height / outlets) * 0.2;
    var tools = [for (i in 0...outlets) keep(
      Part.cylinderAlongY(r, -depth / 2 - 0.1, 0, 0, 6 + pitch * (i + 0.5)),
      "outlet"
    )];
    tools.push(keep(Part.cylinderSpan(r, -0.1, 5), "inlet"));
    return {pieces: pieces, tools: tools};
    });
  }

  /** A vacuum generator: a venturi body, a silencer on top, and its air and vacuum ports in the front. */
  public static function vacuumGenerator(width:Float, depth:Float, height:Float):Part {
    return build(keep -> {var body = height * 0.62, r = Math.min(width, depth) * 0.42;
    var pieces = [
      keep(chamferedBox(width, depth, body, Math.min(width, depth) * 0.08, -width / 2), "venturi-body"),
      keep(Part.cylinderSpan(r, body - 0.01, height), "silencer")
    ];
    var port = Math.min(width, body) * 0.14;
    var tools = [for (z in [body * 0.3,
      body * 0.7]) keep(Part.cylinderAlongY(port, -depth / 2 - 0.1, 0, 0, z), "port")];
    return {pieces: pieces, tools: tools};
    });
  }

  /** A pressure sensor or switch: a body with a display window on its front, on a port boss. */
  public static function sensor(width:Float, depth:Float, height:Float):Part {
    return build(keep -> {var boss = height * 0.22;
    var pieces = [
      keep(Part.cylinderSpan(Math.min(width, depth) * 0.25, 0, boss + 0.01), "port-boss"),
      keep(
        chamferedBox(width, depth, height - boss, Math.min(width, depth) * 0.08, -width / 2).translated(new Vector(
          0,
          0,
          boss
        )),
        "body"
      )
    ];
    var tools = [keep(
      Part.box(width * 0.65, 0.8, (height - boss) * 0.35, Center, Min, Min).translated(new Vector(
        0,
        -depth / 2 - 0.1,
        boss + (height - boss) * 0.55
      )),
      "display"
    )];
    return {pieces: pieces, tools: tools};
    });
  }

  /** A filter-regulator: a head block with a gauge on its front, the filter bowl below it and a drain. */
  public static function frl(width:Float, depth:Float, height:Float):Part {
    return build(keep -> {var head = height * 0.38, bowl = Math.min(width, depth) * 0.42, drain = height * 0.06;
    var pieces = [
      keep(
        chamferedBox(width, depth, head, Math.min(width, depth) * 0.06, -width / 2).translated(new Vector(
          0,
          0,
          height - head
        )),
        "head"
      ),
      keep(
        Part.revolve([
          {r: bowl * 0.35, z: drain},
          {r: bowl, z: drain + bowl * 0.4},
          {r: bowl, z: height - head + 0.01},
          {r: 0, z: height - head + 0.01},
          {r: 0, z: drain}
        ]),
        "bowl"
      ),
      keep(Part.cylinderSpan(bowl * 0.2, 0, drain + 0.01), "drain")
    ];
    var gauge = Math.min(width, head) * 0.3;
    var tools = [keep(Part.cylinderAlongY(gauge, -depth / 2 - 0.1, -depth / 2 + 1.2, 0, height - head / 2), "gauge")];
    return {pieces: pieces, tools: tools};
    });
  }

  /** A bellows-free flat suction cup: a stem with its fitting at the base, flaring to the sealing lip at the top. */
  public static function suctionCup(diameter:Float, height:Float):Part {
    var r = diameter / 2, stem = Math.max(r * 0.28, Math.min(r, 3.0)), neck = height * 0.45;
    return Solids.named(
      Part.revolve([
        {r: 0, z: 0},
        {r: stem, z: 0},
        {r: stem, z: neck},
        {r: r * 0.9, z: height * 0.85},
        {r: r, z: height},
        {r: r * 0.8, z: height},
        {r: stem * 0.6, z: neck + height * 0.12},
        {r: 0, z: neck + height * 0.12}
      ]),
      "cup"
    );
  }

  /** A piston rod from z = -`length` to 0: chamfered ends, a wrench neck and a threaded-end shoulder at its tip. */
  public static function rod(diameter:Float, length:Float):Part {
    var r = diameter / 2, c = r * 0.15, flats = Math.min(length * 0.08, diameter), tip = -length;
    return Solids.named(
      Part.revolve([
        {r: 0, z: tip},
        {r: r * 0.7 - c, z: tip},
        {r: r * 0.7, z: tip + c},
        {r: r * 0.7, z: tip + flats},
        {r: r, z: tip + flats + c},
        {r: r, z: -c},
        {r: r - c, z: 0},
        {r: 0, z: 0}
      ]),
      "rod"
    );
  }

  static function chamferedBox(dx:Float, dy:Float, dz:Float, c:Float, x0:Float):Part return Part.prism(
    [
      new Vector(-dy / 2, 0),
      new Vector(dy / 2, 0),
      new Vector(dy / 2, dz - c),
      new Vector(dy / 2 - c, dz),
      new Vector(-dy / 2 + c, dz),
      new Vector(-dy / 2, dz - c)
    ],
    x0,
    x0 + dx
  ).placed(alongX());

  /** Places a prism drawn in (y, z) and extruded along its z so that the extrusion runs along +x. */
  static function alongX():cadkit.modeling.Location return new cadkit.modeling.Location(new cadkit.modeling.Plane(
    new Vector(0, 0, 0),
    Vector.Y(),
    Vector.X()
  ));

  static function build(fn:(Part -> String -> Part) -> {pieces: Array<Part>,
    tools: Array<Part>}):Part return Solids.building([], owned -> {function keep(
      part: Part,
      tag: String
    ): Part {
    var named = Solids.named(part, tag);
    owned.push(named);
    return named;
  }
  var made = fn(keep);
  return Solids.cut(made.pieces.length == 1 ? made.pieces[0] : Solids.union(made.pieces), made.tools);
  });
}
