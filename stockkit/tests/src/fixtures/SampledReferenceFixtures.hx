package fixtures;

import cadkit.modeling.Align;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import toolpathkit.path.PathGeometry;
import toolpathkit.path.Point3;
import toolpathkit.tool.CutterProfile;
import toolpathkit.tool.CutterSegment;
import toolpathkit.tool.CutterZone;
import oracle.ExactOracle;
import oracle.SampledReference;

/**
  Checks the sampled reference against closed forms and the exact OCCT
  oracle before it is trusted for moves the oracle refuses.
**/
class SampledReferenceFixtures {
  static inline final SLACK = 1e-12;
  static inline final R = 0.003;

  public static function run():Void {
    closedForms();
    convergence();
    helix();
    againstOracle();
    horizontalClosedForms();
    horizontalGrowth();
    horizontalBruteForce();
    horizontalConvergence();
  }

  /** Horizontal rays through known swept sets: X rays at (y, z), Y rays at (x, z). */
  static function horizontalClosedForms():Void {
    var flat = CutterProfile.flat(2 * R, 0.02);
    var ball = CutterProfile.ball(2 * R, 0.02);
    var depth = 0.002;
    var slot:PathGeometry = Line(new Point3(0.01, 0.01, -depth),
      new Point3(0.03, 0.01, -depth));

    // A level flat slot along X: a Y ray across it spans the tool's diameter,
    // an X ray along it the slot plus the chord of the end caps.
    bracketsAlong(Y, flat, slot, 0.02, -depth + 0.001, [{lo: 0.01 - R, hi: 0.01 + R}],
      "Y ray across a flat slot");
    var dy = 0.001, w = Math.sqrt(R * R - dy * dy);
    bracketsAlong(X, flat, slot, 0.01 + dy, -depth + 0.001, [{lo: 0.01 - w, hi: 0.03 + w}],
      "X ray along a flat slot");
    bracketsAlong(X, flat, slot, 0.01, -depth - 0.0005, [], "X ray below a flat slot");
    bracketsAlong(X, flat, slot, 0.01, -depth + 0.0205, [], "X ray above a flat slot");
    bracketsAlong(Y, flat, slot, 0.0345, -depth + 0.001, [], "Y ray past a flat slot's cap");

    // A ball slot below the ball's centre: the section there is a disc of
    // radius sqrt(R² - (R - h)²).
    var h = 0.001, rb = Math.sqrt(R * R - (R - h) * (R - h));
    bracketsAlong(Y, ball, slot, 0.02, -depth + h, [{lo: 0.01 - rb, hi: 0.01 + rb}],
      "Y ray across a ball slot below its centre");
    var wb = Math.sqrt(rb * rb - dy * dy);
    bracketsAlong(X, ball, slot, 0.01 + dy, -depth + h, [{lo: 0.01 - wb, hi: 0.03 + wb}],
      "X ray along a ball slot below its centre");

    // A full circle of radius 5 mm: a ray through its centre crosses the
    // swept ring twice.
    var c = new Point3(0.02, 0.01, -0.001), ring = 0.005;
    var circle:PathGeometry = Arc(c, ring, 0, 2 * Math.PI);
    bracketsAlong(X, flat, circle, c.y, c.z + 0.002, [
      {lo: c.x - ring - R, hi: c.x - ring + R}, {lo: c.x + ring - R, hi: c.x + ring + R}
    ], "X ray through a flat full circle's centre");
    bracketsAlong(Y, flat, circle, c.x, c.z + 0.002, [
      {lo: c.y - ring - R, hi: c.y - ring + R}, {lo: c.y + ring - R, hi: c.y + ring + R}
    ], "Y ray through a flat full circle's centre");

    // A descending ramp from (0, 0, 0) to (10, 0, -2) mm: a ray 1 mm down is
    // reached by the tip from halfway on, and the top from 19 mm up only
    // until halfway.
    var ramp:PathGeometry = Line(new Point3(0, 0, 0), new Point3(0.01, 0, -0.002));
    bracketsAlong(X, flat, ramp, 0, -0.001, [{lo: 0.005 - R, hi: 0.01 + R}],
      "X ray under a descending ramp");
    bracketsAlong(X, flat, ramp, 0, 0.019, [{lo: -R, hi: 0.005 + R}],
      "X ray near the top of a descending ramp");
    bracketsAlong(Y, flat, ramp, 0.007, -0.001, [{lo: -R, hi: R}],
      "Y ray under a descending ramp");

    // The last half turn of a descending helix reaches a ray 1.5 mm down;
    // the tool's axis sweeps x in [19, 21] mm.
    var helix:PathGeometry = Circular(new Point3(0.02, 0.01, 0.0), 0.001, 0.0, 4 * Math.PI,
      XY, -0.002);
    bracketsAlong(X, CutterProfile.flat(2 * R, 0.01), helix, 0.01, -0.0015,
      [{lo: 0.02 - 0.001 - R, hi: 0.02 + 0.001 + R}], "X ray through a helix axis");

    var refused = false;
    try SampledReference.sweepAlong(X, flat, Circular(new Point3(0, 0, 0), 0.001, 0, 1, XZ, 0),
      0, 0, 10) catch (_:Dynamic) refused = true;
    Assert.check(refused, "horizontal rays refuse arcs outside the XY plane");
  }

  /**
    One sample step, so the outer set is exactly the two end tools grown by
    rho, whose sections are known: a ball grown by rho is a ball of radius
    R + rho, and a flat mill's grown rim is rounded by rho.
  **/
  static function horizontalGrowth():Void {
    var step = 0.001, rho = step / 2;
    var a = new Point3(0.01, 0.01, 0.0);
    var move:PathGeometry = Line(a, new Point3(a.x + step, a.y, a.z));
    var ball = CutterProfile.ball(2 * R, 0.02);
    for (h in [R, 0.001, 0.0, -0.0003]) {
      var g = Math.sqrt((R + rho) * (R + rho) - (R - h) * (R - h));
      grownAlong(ball, move, a.y, h, g, 'ball grown at height $h');
    }
    grownAlong(ball, move, a.y, 0.02 + 0.0004, R + Math.sqrt(rho * rho - 0.0004 * 0.0004),
      "ball grown above its top");
    var flat = CutterProfile.flat(2 * R, 0.02);
    for (h in [-0.0003, 0.02 + 0.0003])
      grownAlong(flat, move, a.y, h, R + Math.sqrt(rho * rho - 0.0003 * 0.0003),
        'flat grown at height $h');
    grownAlong(flat, move, a.y, 0.005, R + rho, "flat grown along its side");
    var bounds = SampledReference.sweepAlong(X, flat, move, a.y, -0.0006, 1);
    Assert.check(bounds.outer.length == 0, "grown flat mill does not reach 0.6 mm below its tip");
  }

  /** The X ray at (y, h) meets the two grown end tools, each a disc of radius `g`. */
  static function grownAlong(profile:CutterProfile, move:PathGeometry, y:Float, h:Float,
      g:Float, message:String):Void {
    var bounds = SampledReference.sweepAlong(X, profile, move, y, h, 1);
    var from = 0.01, to = 0.011;
    Assert.check(bounds.outer.length == 1, '$message: one outer span, got ${describe(bounds.outer)}');
    if (bounds.outer.length != 1) return;
    Assert.near(bounds.outer[0].lo, from - g, '$message: low end', 1e-11);
    Assert.near(bounds.outer[0].hi, to + g, '$message: high end', 1e-11);
    Assert.check(SampledReference.contains(bounds.inner, bounds.inner, bounds.outer, SLACK),
      '$message: inner inside outer');
  }

  /**
    Random points near a one-step move: those within rho of an end tool,
    measured against a finely divided polygon of the tool's half section,
    must be in the outer set on both horizontal rays through them, and those
    clearly further must not be; points of the inner set must be in a tool.
  **/
  static function horizontalBruteForce():Void {
    var concave = new CutterProfile([
      Line(0, 0, 0.002, 0, Cutting),
      Arc(0.003, 0, 0.002, 0, 0.003, 0.001, Cutting),
      Line(0.003, 0.001, 0.003, 0.004, Cutting),
      Line(0.003, 0.004, 0.005, 0.004, Shank),
      Line(0.005, 0.004, 0.005, 0.008, Shank)
    ]);
    var profiles = [
      concave,
      CutterProfile.bullNose(2 * R, 0.001, 0.006).withShank(0.004, 0.003),
      CutterProfile.taperedBall(0.002, 20 * Math.PI / 180, 2 * R, 0.008),
      CutterProfile.vee(2 * R, Math.PI / 2, 0.005)
    ];
    var seed = 12345.0;
    function random():Float {
      // Park-Miller; the modulus is a Float so the product cannot overflow.
      var modulus = 2147483647.0;
      seed = seed * 16807 - modulus * Math.floor(seed * 16807 / modulus);
      return seed / modulus;
    }
    var within = 0, beyond = 0;
    for (profile in profiles) {
      var polygon = halfSection(profile);
      for (step in [0.0016, 0.003, 0.0001]) {
        var rho = step / 2;
        var a = new Point3(0.01, 0.01, 0.0), b = new Point3(0.01 + 0.6 * step, 0.01 - 0.8 * step, 0.0);
        var move:PathGeometry = Line(a, b);
        for (k in 0...3000) {
          var x:Float, y:Float, z:Float;
          if (k % 2 == 0) {
            // Anywhere around the tools.
            var reach = 0.006 + 2 * rho;
            x = a.x + (2 * random() - 1) * reach;
            y = a.y + (2 * random() - 1) * reach;
            z = -2 * rho + random() * (profile.height() + 4 * rho);
          } else {
            // Near a tool's surface, where the bounds are decided: a random
            // boundary point of the half section moved up to 2 rho.
            var on = boundaryPoint(profile, random(), random());
            var direction = 2 * Math.PI * random(), offset = 2 * rho * random();
            var r = on.r + offset * Math.cos(direction);
            z = on.z + offset * Math.sin(direction);
            var azimuth = 2 * Math.PI * random(), tip = random() < 0.5 ? a : b;
            x = tip.x + Math.abs(r) * Math.cos(azimuth);
            y = tip.y + Math.abs(r) * Math.sin(azimuth);
            z += tip.z;
          }
          var distance = Math.min(distanceTo(polygon, x - a.x, y - a.y, z - a.z),
            distanceTo(polygon, x - b.x, y - b.y, z - b.z));
          var xRay = SampledReference.sweepAlong(X, profile, move, y, z, 1);
          var yRay = SampledReference.sweepAlong(Y, profile, move, x, z, 1);
          var where = 'brute force point ($x, $y, $z) at distance $distance, rho $rho';
          if (distance <= rho * (1 - 1e-4)) {
            within++;
            Assert.check(onSpans(xRay.outer, x) && onSpans(yRay.outer, y), '$where is outside the outer set');
          } else if (distance >= rho * (1 + 1e-4)) {
            beyond++;
            Assert.check(!onSpans(xRay.outer, x) && !onSpans(yRay.outer, y), '$where is inside the outer set');
          }
          if (distance > 1e-9)
            Assert.check(!onSpans(xRay.inner, x) && !onSpans(yRay.inner, y), '$where is inside the inner set');
        }
      }
    }
    Assert.check(within > 5000 && beyond > 5000, 'brute force covers both sides: $within within, $beyond beyond');
  }

  /** A point on the profile or its flat top: `pick` chooses the piece, `t` the place along it. */
  static function boundaryPoint(profile:CutterProfile, pick:Float, t:Float):{r:Float, z:Float} {
    var index = Math.floor(pick * (profile.segments.length + 1));
    if (index >= profile.segments.length) return {r: t * profile.topRadius(), z: profile.height()};
    return switch profile.segments[index] {
      case Line(r0, z0, r1, z1, _): {r: r0 + t * (r1 - r0), z: z0 + t * (z1 - z0)};
      case Arc(cr, cz, r0, z0, r1, z1, _):
        var start = Math.atan2(z0 - cz, r0 - cr);
        var sweep = Math.atan2(z1 - cz, r1 - cr) - start;
        while (sweep > Math.PI) sweep -= 2 * Math.PI;
        while (sweep <= -Math.PI) sweep += 2 * Math.PI;
        var radius = Math.sqrt((r0 - cr) * (r0 - cr) + (z0 - cz) * (z0 - cz));
        {r: cr + radius * Math.cos(start + t * sweep), z: cz + radius * Math.sin(start + t * sweep)};
    };
  }

  /** The closed half section as a polygon in (r, z): axis, profile with arcs finely divided, top. */
  static function halfSection(profile:CutterProfile):Array<{r:Float, z:Float}> {
    var points = [{r: 0.0, z: 0.0}];
    for (segment in profile.segments) switch segment {
      case Line(_, _, r1, z1, _): points.push({r: r1, z: z1});
      case Arc(cr, cz, r0, z0, r1, z1, _):
        var start = Math.atan2(z0 - cz, r0 - cr);
        var sweep = Math.atan2(z1 - cz, r1 - cr) - start;
        while (sweep > Math.PI) sweep -= 2 * Math.PI;
        while (sweep <= -Math.PI) sweep += 2 * Math.PI;
        var radius = Math.sqrt((r0 - cr) * (r0 - cr) + (z0 - cz) * (z0 - cz));
        for (i in 1...4001) {
          var angle = start + sweep * i / 4000;
          points.push({r: cr + radius * Math.cos(angle), z: cz + radius * Math.sin(angle)});
        }
    }
    points.push({r: 0.0, z: profile.height()});
    return points;
  }

  /** Distance from the point at offset (dx, dy, dz) from the tip to the tool: 0 inside. */
  static function distanceTo(polygon:Array<{r:Float, z:Float}>, dx:Float, dy:Float,
      dz:Float):Float {
    var r = Math.sqrt(dx * dx + dy * dy);
    var inside = false, nearest = Math.POSITIVE_INFINITY;
    for (i in 0...polygon.length) {
      var p = polygon[i], q = polygon[(i + 1) % polygon.length];
      if ((p.z > dz) != (q.z > dz) && r < p.r + (dz - p.z) / (q.z - p.z) * (q.r - p.r))
        inside = !inside;
      var er = q.r - p.r, ez = q.z - p.z, length2 = er * er + ez * ez;
      var t = length2 == 0 ? 0.0 : Math.max(0.0, Math.min(1.0, ((r - p.r) * er + (dz - p.z) * ez) / length2));
      var cr = p.r + t * er - r, cz = p.z + t * ez - dz;
      nearest = Math.min(nearest, Math.sqrt(cr * cr + cz * cz));
    }
    return inside ? 0.0 : nearest;
  }

  static function onSpans(spans:Array<Span>, s:Float):Bool {
    for (span in spans) if (span.lo - 1e-12 <= s && s <= span.hi + 1e-12) return true;
    return false;
  }

  /** Outer minus inner shrinks on horizontal rays as samples are added. */
  static function horizontalConvergence():Void {
    var bull = CutterProfile.bullNose(2 * R, 0.001, 0.01);
    var ramp:PathGeometry = Line(new Point3(0.01, 0.01, 0.0), new Point3(0.03, 0.015, -0.002));
    var arc:PathGeometry = Arc(new Point3(0.02, 0.01, -0.001), 0.002, 0.3, 2.5);
    var cases:Array<{move:PathGeometry, axis:ReferenceAxis, u:Float, v:Float}> = [
      {move: ramp, axis: X, u: 0.0125, v: -0.001},
      {move: ramp, axis: Y, u: 0.021, v: -0.0005},
      {move: arc, axis: X, u: 0.0115, v: -0.0005},
      {move: arc, axis: Y, u: 0.019, v: 0.0}
    ];
    for (fixture in cases) {
      var previous = Math.POSITIVE_INFINITY;
      for (samples in [16, 128, 1024]) {
        var bounds = SampledReference.sweepAlong(fixture.axis, bull, fixture.move, fixture.u,
          fixture.v, samples);
        var label = '${fixture.axis} ray at ${fixture.u}, ${fixture.v} with $samples samples';
        Assert.check(bounds.inner.length > 0, '$label is cut');
        Assert.check(SampledReference.contains(bounds.inner, bounds.inner, bounds.outer, SLACK),
          '$label: inner inside outer');
        var gap = SampledReference.measure(bounds.outer) - SampledReference.measure(bounds.inner);
        Assert.check(gap >= 0 && gap < previous / 4, '$label: bounds tighten to $gap after $previous');
        previous = gap;
      }
    }
  }

  /** The reference brackets a known swept set along a horizontal ray. */
  static function bracketsAlong(axis:ReferenceAxis, profile:CutterProfile, move:PathGeometry,
      u:Float, v:Float, exact:Array<Span>, message:String):Void {
    var bounds = SampledReference.sweepAlong(axis, profile, move, u, v, 1000);
    Assert.check(SampledReference.contains(bounds.inner, exact, bounds.outer, SLACK),
      '$message: exact ${describe(exact)} within ${describe(bounds.inner)} .. ${describe(bounds.outer)}');
    Assert.check(SampledReference.measure(bounds.outer) - SampledReference.measure(bounds.inner) < 1e-4,
      '$message: bounds are tight');
  }

  static function closedForms():Void {
    var flat = CutterProfile.flat(2 * R, 0.02);
    var ball = CutterProfile.ball(2 * R, 0.02);
    var depth = 0.002;
    var slot:PathGeometry = Line(new Point3(0.01, 0.01, -depth),
      new Point3(0.03, 0.01, -depth));

    // Flat slot: the tool bottom to its top, wherever the ray is under the stadium.
    brackets(flat, slot, 0.02, 0.012, [{lo: -depth, hi: -depth + 0.02}], "flat slot middle");
    brackets(flat, slot, 0.0325, 0.01, [{lo: -depth, hi: -depth + 0.02}], "flat slot end cap");
    brackets(flat, slot, 0.034, 0.01, [], "flat slot misses past the cap");
    brackets(flat, slot, 0.02, 0.01 + 0.999 * R, [{lo: -depth, hi: -depth + 0.02}],
      "flat slot just inside its wall");
    // A ray grazing the wall may round to either side: the bounds must still
    // hold, though they cannot be tight there.
    var grazing = SampledReference.sweep(flat, slot, 0.02, 0.01 + R, 1000);
    Assert.check(SampledReference.contains(grazing.inner, grazing.inner, grazing.outer, SLACK)
      && SampledReference.contains([], [{lo: -depth, hi: -depth + 0.02}], grazing.outer, SLACK),
      "flat slot grazing its wall");

    // Ball slot: the floor follows the ball across the slot.
    var offset = 0.002;
    brackets(ball, slot, 0.02, 0.01 + offset,
      [{lo: -depth + R - Math.sqrt(R * R - offset * offset), hi: -depth + 0.02}],
      "ball slot floor");

    // Plunge: the lowest tip height to the highest top.
    var plunge:PathGeometry = Line(new Point3(0.02, 0.01, 0.001),
      new Point3(0.02, 0.01, -0.003));
    brackets(flat, plunge, 0.021, 0.01, [{lo: -0.003, hi: 0.021}], "plunge");

    // Ball ramp: the ball's centre sweeps a 3D segment, so the lowest point
    // on the ray is the lowest point of a capsule of the ball's radius.
    var hemisphere = CutterProfile.ball(2 * R, R);
    var a = new Point3(0.01, 0.01, -0.001), b = new Point3(0.03, 0.012, -0.004);
    var ramp:PathGeometry = Line(a, b);
    for (ray in [{x: 0.02, y: 0.0125}, {x: 0.012, y: 0.0085}, {x: 0.031, y: 0.013},
        {x: 0.0285, y: 0.009}]) {
      var lowest = capsuleLowest(a, b, R, ray.x, ray.y);
      var bounds = SampledReference.sweep(hemisphere, ramp, ray.x, ray.y, 2000);
      Assert.check(bounds.inner.length > 0, 'ball ramp at ${ray.x}, ${ray.y} is cut');
      Assert.check(bounds.outer[0].lo <= lowest + SLACK && lowest <= bounds.inner[0].lo + SLACK,
        'ball ramp at ${ray.x}, ${ray.y}: capsule floor $lowest within '
        + '${bounds.outer[0].lo}..${bounds.inner[0].lo}');
      // A sampled pose touches the floor to within the sample spacing's sag;
      // the grown tools reach further below it where the ball is steep.
      Assert.check(bounds.inner[0].lo - lowest < 1e-8,
        'ball ramp at ${ray.x}, ${ray.y}: inner floor ${bounds.inner[0].lo} is tight');
      Assert.check(lowest - bounds.outer[0].lo < 5e-5,
        'ball ramp at ${ray.x}, ${ray.y}: outer floor ${bounds.outer[0].lo} is tight');
      Assert.check(SampledReference.contains(bounds.inner, bounds.inner, bounds.outer, SLACK),
        'ball ramp at ${ray.x}, ${ray.y}: inner inside outer');
    }
  }

  /** Outer minus inner shrinks as samples are added. */
  static function convergence():Void {
    var bull = CutterProfile.bullNose(2 * R, 0.001, 0.01);
    var ramp:PathGeometry = Line(new Point3(0.01, 0.01, 0.0), new Point3(0.03, 0.015, -0.002));
    var arc:PathGeometry = Arc(new Point3(0.02, 0.01, -0.001), 0.002, 0.3, 2.5);
    for (move in [ramp, arc]) for (ray in [{x: 0.021, y: 0.0135}, {x: 0.019, y: 0.0105}]) {
      var previous = Math.POSITIVE_INFINITY;
      for (samples in [16, 128, 1024]) {
        var bounds = SampledReference.sweep(bull, move, ray.x, ray.y, samples);
        Assert.check(SampledReference.contains(bounds.inner, bounds.inner, bounds.outer, SLACK),
          'inner inside outer with $samples samples');
        var gap = SampledReference.measure(bounds.outer) - SampledReference.measure(bounds.inner);
        // The gap scales with the sample step, so eight times the samples
        // must shrink it at least fourfold.
        Assert.check(gap >= 0 && gap < previous / 4,
          'bounds tighten with $samples samples: $gap after $previous');
        previous = gap;
      }
    }
  }

  static function helix():Void {
    // Two descending turns of radius 1 mm with a 3 mm flat mill: the ray on
    // the helix axis is always under the tool, so it is cut from the final
    // tip height up to the starting tool top.
    var flat = CutterProfile.flat(2 * R, 0.01);
    var helix:PathGeometry = Circular(new Point3(0.02, 0.01, 0.0), 0.001, 0.0,
      4 * Math.PI, XY, -0.002);
    brackets(flat, helix, 0.02, 0.01, [{lo: -0.002, hi: 0.01}], "helix axis");
    // Off-axis rays: bounds stay nested and tight.
    var bounds = SampledReference.sweep(flat, helix, 0.0235, 0.01, 4000);
    Assert.check(bounds.inner.length == 1, "helix edge ray is cut once");
    Assert.check(SampledReference.contains(bounds.inner, bounds.inner, bounds.outer, SLACK),
      "helix edge ray: inner inside outer");
    Assert.check(SampledReference.measure(bounds.outer) - SampledReference.measure(bounds.inner) < 1e-5,
      "helix edge ray: bounds are tight");
    var refused = false;
    try SampledReference.sweep(flat, Circular(new Point3(0, 0, 0), 0.001, 0, 1, XZ, 0),
      0, 0, 10) catch (_:Dynamic) refused = true;
    Assert.check(refused, "arcs outside the XY plane are refused");
  }

  /** The exact oracle's stock lies within the reference's remaining-material bounds. */
  static function againstOracle():Void {
    var ball = CutterProfile.ball(2 * R, 0.02);
    var cases:Array<{name:String, moves:Array<PathGeometry>, rays:Array<{x:Float, y:Float}>}> = [
      {name: "ball slot", moves: [Line(new Point3(0.01, 0.01, -0.004),
        new Point3(0.03, 0.01, -0.004))],
        rays: [{x: 0.02, y: 0.012}, {x: 0.0315, y: 0.0105}, {x: 0.02, y: 0.0129}]},
      {name: "ball arc", moves: [Arc(new Point3(0.02, 0.01, -0.004), 0.005, 0, Math.PI)],
        rays: [{x: 0.02, y: 0.015}, {x: 0.0245, y: 0.0115}, {x: 0.0172, y: 0.0131}]}
    ];
    for (fixture in cases) {
      var blank = Part.box(0.04, 0.02, 0.01, Min, Min, Max);
      var stock = ExactOracle.cut(blank, ExactOracle.pathMoves(ball, fixture.moves));
      blank.close();
      var moves = [for (geometry in fixture.moves) {profile: ball, geometry: geometry}];
      for (ray in fixture.rays) {
        var top = 0.05;
        var exact = ExactOracle.rayIntervals(stock, new Vector(ray.x, ray.y, top),
          new Vector(0, 0, -1), 0.1);
        var candidate = [for (hit in exact) {lo: top - hit.exit, hi: top - hit.enter}];
        var bounds = SampledReference.remaining([{lo: -0.01, hi: 0.0}], moves,
          ray.x, ray.y, 2000);
        // The oracle is exact only to OCCT's tolerance.
        Assert.check(SampledReference.contains(bounds.inner, candidate, bounds.outer, 1e-9),
          '${fixture.name} at ${ray.x}, ${ray.y}: oracle ${describe(candidate)} within '
          + '${describe(bounds.inner)} .. ${describe(bounds.outer)}');
      }
      stock.close();
    }
  }

  /** The reference brackets a known swept set. */
  static function brackets(profile:CutterProfile, move:PathGeometry, x:Float, y:Float,
      exact:Array<Span>, message:String):Void {
    var bounds = SampledReference.sweep(profile, move, x, y, 1000);
    Assert.check(SampledReference.contains(bounds.inner, exact, bounds.outer, SLACK),
      '$message: exact ${describe(exact)} within ${describe(bounds.inner)} .. ${describe(bounds.outer)}');
    Assert.check(SampledReference.measure(bounds.outer) - SampledReference.measure(bounds.inner) < 1e-4,
      '$message: bounds are tight');
  }

  /**
    Lowest z within `radius` of the segment between ball centres above `a`
    and `b` on the vertical line at (x, y): the lower of the end spheres and
    the cylinder around the segment, clipped to its length.
  **/
  static function capsuleLowest(a:Point3, b:Point3, radius:Float, x:Float, y:Float):Float {
    var lowest = Math.POSITIVE_INFINITY;
    for (end in [a, b]) {
      var dx = x - end.x, dy = y - end.y, h = radius * radius - dx * dx - dy * dy;
      if (h >= 0) lowest = Math.min(lowest, end.z + radius - Math.sqrt(h));
    }
    var ux = b.x - a.x, uy = b.y - a.y, uz = b.z - a.z;
    var length = Math.sqrt(ux * ux + uy * uy + uz * uz);
    ux /= length; uy /= length; uz /= length;
    // p - c0 = w + z e_z with w = (x - a.x, y - a.y, -(a.z + radius));
    // |p - c0|^2 - ((p - c0).u)^2 = radius^2 is quadratic in z.
    var wx = x - a.x, wy = y - a.y, wz = -(a.z + radius);
    var wu = wx * ux + wy * uy + wz * uz;
    var qa = 1 - uz * uz;
    var qb = 2 * (wz - wu * uz);
    var qc = wx * wx + wy * wy + wz * wz - wu * wu - radius * radius;
    var disc = qb * qb - 4 * qa * qc;
    if (disc >= 0) for (sign in [-1.0, 1.0]) {
      var z = (-qb + sign * Math.sqrt(disc)) / (2 * qa);
      var along = wu + z * uz;
      if (along >= 0 && along <= length) lowest = Math.min(lowest, z);
    }
    return lowest;
  }

  static function describe(spans:Array<Span>):String
    return '[' + [for (span in spans) '${span.lo}..${span.hi}'].join(', ') + ']';
}
