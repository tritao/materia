package fixtures;

import cadkit.modeling.Align;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import cnckit.ir.CncGeometry;
import cnckit.ir.CncPoint;
import cnckit.tool.CutterProfile;
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
  }

  static function closedForms():Void {
    var flat = CutterProfile.flat(2 * R, 0.02);
    var ball = CutterProfile.ball(2 * R, 0.02);
    var depth = 0.002;
    var slot:CncGeometry = Line(new CncPoint(0.01, 0.01, -depth),
      new CncPoint(0.03, 0.01, -depth));

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
    var plunge:CncGeometry = Line(new CncPoint(0.02, 0.01, 0.001),
      new CncPoint(0.02, 0.01, -0.003));
    brackets(flat, plunge, 0.021, 0.01, [{lo: -0.003, hi: 0.021}], "plunge");

    // Ball ramp: the ball's centre sweeps a 3D segment, so the lowest point
    // on the ray is the lowest point of a capsule of the ball's radius.
    var hemisphere = CutterProfile.ball(2 * R, R);
    var a = new CncPoint(0.01, 0.01, -0.001), b = new CncPoint(0.03, 0.012, -0.004);
    var ramp:CncGeometry = Line(a, b);
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
    var ramp:CncGeometry = Line(new CncPoint(0.01, 0.01, 0.0), new CncPoint(0.03, 0.015, -0.002));
    var arc:CncGeometry = Arc(new CncPoint(0.02, 0.01, -0.001), 0.002, 0.3, 2.5);
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
    var helix:CncGeometry = Circular(new CncPoint(0.02, 0.01, 0.0), 0.001, 0.0,
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
    try SampledReference.sweep(flat, Circular(new CncPoint(0, 0, 0), 0.001, 0, 1, XZ, 0),
      0, 0, 10) catch (_:Dynamic) refused = true;
    Assert.check(refused, "arcs outside the XY plane are refused");
  }

  /** The exact oracle's stock lies within the reference's remaining-material bounds. */
  static function againstOracle():Void {
    var ball = CutterProfile.ball(2 * R, 0.02);
    var cases:Array<{name:String, moves:Array<CncGeometry>, rays:Array<{x:Float, y:Float}>}> = [
      {name: "ball slot", moves: [Line(new CncPoint(0.01, 0.01, -0.004),
        new CncPoint(0.03, 0.01, -0.004))],
        rays: [{x: 0.02, y: 0.012}, {x: 0.0315, y: 0.0105}, {x: 0.02, y: 0.0129}]},
      {name: "ball arc", moves: [Arc(new CncPoint(0.02, 0.01, -0.004), 0.005, 0, Math.PI)],
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
  static function brackets(profile:CutterProfile, move:CncGeometry, x:Float, y:Float,
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
  static function capsuleLowest(a:CncPoint, b:CncPoint, radius:Float, x:Float, y:Float):Float {
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
