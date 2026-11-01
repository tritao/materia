package gearkit;

/** A concrete circular-spline tooth: one internal tooth, as a `ToothProfile`, plus the signed clearance of any point
    from the ring of teeth it makes. This is the part the fixed-circular-spline mesh solver meshes against
    (TOOTH_DESIGN_PLAN.md TD11), and what a later EDM / shaper output and metrology read through `profile`.
    Two ways to make one:
    - `derived`: from a design state (form, cam, kappa) by the swept-envelope circular spline of that state, as a
      `SampledInternalProfile` with the sweep's deviation bound;
    - `involute`: a nominal involute internal gear (analytic, bound 0), for comparison and for the rigid limit.
    The clearance works from the profile's outline (`wallSamples` points per segment), so it is the same for any form.
    Sign: positive in the open space (distance to the nearest tooth surface), negative inside circular-spline material,
    including the rim beyond the root circle. */
class CircularSpline {
  public final label:String;
  public final profile:ToothProfile;
  /** Largest known difference (mm) between the profile and the surface it stands for; 0 for an analytic involute. */
  public final deviationBound:Float;
  public final teeth:Int;
  public final tipRadius:Float;
  public final rootRadius:Float;
  final pitch:Float;
  // Right-hand wall from the tip centre to the end of the root circle, in the tooth frame.
  final wx:Array<Float> = [];
  final wy:Array<Float> = [];
  final wr:Array<Float> = [];
  final wa:Array<Float> = [];
  var landEnd:Int = 0;

  public function new(label:String, profile:ToothProfile, deviationBound:Float, wallSamples:Int = 800) {
    var spec = profile.gearSpec();
    if (!spec.internal) throw "a circular spline is an internal gear";
    this.label = label;
    this.profile = profile;
    this.deviationBound = deviationBound;
    teeth = spec.teeth;
    pitch = 2 * Math.PI / teeth;
    tipRadius = spec.tipRadius();
    rootRadius = spec.rootRadius();
    var o = profile.outline(wallSamples);
    wx.push(0.0);
    wy.push(tipRadius);
    var started = false;
    for (i in 0...o.length - 1) {
      if (!started && o[i].x >= 0 && o[i].radius() <= tipRadius + 1e-9) started = true;
      if (started && o[i].x > 0) {
        wx.push(o[i].x);
        wy.push(o[i].y);
      }
    }
    for (k in 0...wx.length) {
      wr.push(Math.sqrt(wx[k] * wx[k] + wy[k] * wy[k]));
      wa.push(Math.atan2(wx[k], wy[k]));
    }
    for (k in 0...wr.length) if (wr[k] <= tipRadius + 1e-9) landEnd = k;
  }

  /** The circular spline of a design state: the envelope of its flexspline tooth swept through the wave, cut off where it is
      `minThicknessFraction` x module thick, with the nominal root circle (pitch + dedendum of the flexspline's form). */
  public static function derived(label:String, pair:StrainWavePair, minThicknessFraction:Float = 0.05):CircularSpline {
    var space = pair.space(), m = pair.input.module, minThickness = minThicknessFraction * m;
    var tip = space.maxRadius(), n = 800;
    for (i in 0...n + 1) {
      var r = space.maxRadius() - (space.maxRadius() - space.minRadius()) * i / n;
      if (space.toothThickness(r) >= minThickness) tip = r;
      else break;
    }
    var root = Math.max(pair.circularRootRadius(), space.maxRadius());
    var spec = new GearSpec(m, pair.input.circularTeeth, pair.flex.faceWidth, new SampledInternalForm(space, m, tip, root), true);
    return new CircularSpline(label, spec.profile(), space.deviationBound);
  }

  /** A nominal involute internal gear: module, teeth, pressure angle, profile shift (positive thins the teeth). */
  public static function involute(
    label:String,
    module:Float,
    teeth:Int,
    pressureAngle:Float,
    profileShift:Float = 0,
    ?cutterTipRadius:Float,
    ?tipFilletRadius:Float
  ):CircularSpline {
    var spec = new GearSpec(module, teeth, 8, new Involute(pressureAngle), true, profileShift, cutterTipRadius, tipFilletRadius);
    return new CircularSpline(label, spec.profile(), 0);
  }

  function segmentAt(r:Float):Int {
    var lo = 0, hi = wr.length - 1;
    while (hi - lo > 1) {
      var mid = (lo + hi) >> 1;
      if (wr[mid] <= r) lo = mid;
      else hi = mid;
    }
    return lo;
  }

  function segmentDistance(k:Int, px:Float, py:Float):Float {
    var ax = wx[k], ay = wy[k], dx = wx[k + 1] - ax, dy = wy[k + 1] - ay;
    var l2 = dx * dx + dy * dy;
    var t = l2 > 0 ? ((px - ax) * dx + (py - ay) * dy) / l2 : 0.0;
    t = Math.max(0, Math.min(1, t));
    var ex = ax + t * dx - px, ey = ay + t * dy - py;
    return Math.sqrt(ex * ex + ey * ey);
  }

  /** Signed clearance (mm) of a point in the circular-spline frame. The ring is placed with a tooth *space* centred on +Y,
      where the wave generator's major axis starts (theta = 0) and flexspline tooth 0 sits, as in `SweptSpace`. */
  public function clearance(p:Point2):Float {
    var r = p.radius();
    if (r >= rootRadius) return rootRadius - r;
    var a = Math.atan2(p.x, p.y) - pitch / 2;
    return single(r, Math.abs(a - pitch * Math.round(a / pitch)));
  }

  /** Signed clearance to the one tooth on a given side of the space the point is in: `side` +1 the tooth towards +X
      (the one a flexspline turning in the drive direction runs into), -1 the tooth towards -X. Unlike `clearance` it
      does not take the nearer tooth, so it is monotone in a rotation of the point about the axis. */
  public function sideClearance(p:Point2, side:Int):Float {
    var r = p.radius();
    if (r >= rootRadius) return rootRadius - r;
    var a = Math.atan2(p.x, p.y), k = Math.round(a / pitch);
    var u = Math.abs(a - (k + 0.5 * side) * pitch);
    return single(r, Math.min(u, 2 * Math.PI - u));
  }

  /** Signed clearance to a single tooth with its axis at +Y, for a point at radius r (below the root radius) and angle u >= 0 from the axis. */
  function single(r:Float, u:Float):Float {
    if (r < tipRadius && u <= wa[landEnd]) return tipRadius - r;
    var px = r * Math.sin(u), py = r * Math.cos(u);
    var i = r < tipRadius ? landEnd : segmentAt(r);
    // Nearest wall segment: start at the segment at this radius and walk downhill (the distance to the segments is
    // unimodal near the nearest point), then take the best of a few neighbours against the sampling roughness of the wall.
    var last = wr.length - 2, k = Std.int(Math.min(last, i)), d = segmentDistance(k, px, py);
    while (k > 0) {
      var e = segmentDistance(k - 1, px, py);
      if (e >= d) break;
      d = e;
      k--;
    }
    while (k < last) {
      var e = segmentDistance(k + 1, px, py);
      if (e >= d) break;
      d = e;
      k++;
    }
    var best = d;
    for (q in Std.int(Math.max(0, k - 6))...Std.int(Math.min(last, k + 6)) + 1) best = Math.min(best, segmentDistance(q, px, py));
    if (r < tipRadius) return best;
    var j = Std.int(Math.min(wr.length - 2, i)), f = wr[j + 1] > wr[j] ? Math.min(1.0, (r - wr[j]) / (wr[j + 1] - wr[j])) : 0.0;
    var wall = wa[j] + f * (wa[j + 1] - wa[j]);
    return u <= wall ? -best : best;
  }
}
