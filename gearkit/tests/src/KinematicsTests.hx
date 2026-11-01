import gearkit.Cam;
import gearkit.Cams;
import gearkit.CosineCam;
import gearkit.EllipseCam;
import gearkit.FourRollerCam;
import gearkit.Point2;
import gearkit.Quadrature;
import gearkit.ResalCam;
import gearkit.StrainWaveKinematics;
import gearkit.WaveDeformation;

/** TD3: cams, neutral-line w/v/mu, and flexspline tooth poses. */
class KinematicsTests {
  static var assertions = 0;
  static var worstClosedForm = 0.0;
  static var worstInextensible = 0.0;
  static var worstClosure = 0.0;

  static function check(ok:Bool, message:String):Void {
    assertions++;
    if (!ok) throw message;
  }
  static function near(a:Float, b:Float, tolerance:Float, message:String):Void check(
    Math.isFinite(a) && Math.abs(a - b) <= tolerance, '$message: $a versus $b (tol $tolerance)');
  static function rejects(f:Void -> Void, fragment:String):Void {
    var error = "";
    try f() catch (e:Dynamic) error = Std.string(e);
    check(error.indexOf(fragment) >= 0, 'expected "$fragment", got "$error"');
  }

  /** The reference flexspline D1: neutral line 0.3 mm under the root of m 0.3, Z 200 (assumed wall 0.6 mm). */
  static final RN = 29.7;
  static final W0 = 0.3;

  public static function run():Void {
    cosine();
    for (cam in cams()) general(cam);
    fourRollerArcs();
    poses();
    for (n in ["cosine", "ellipse", "four-roller", "resal"]) check(Cams.make(n, RN, W0).name() == n, 'cam $n by name');
    resal();
    rejects(() -> new ResalCam(0), "amplitude");
    rejects(() -> Cams.make("triangle", RN, W0), "unknown cam");
    rejects(() -> new CosineCam(0), "amplitude");
    rejects(() -> new EllipseCam(0.2, 0.3), "neutral radius");
    rejects(() -> new FourRollerCam(RN, W0, RN), "lobe radius");
    Sys.println('KinematicsTests: $assertions assertions passed; closed form ${worstClosedForm}; inextensibility ${worstInextensible}; closure ${worstClosure}');
  }

  static function cams():Array<Cam> return [new CosineCam(W0), new EllipseCam(RN, W0), new FourRollerCam(RN, W0), new ResalCam(W0)];

  /** The numerical v and mu of the cosine cam equal the closed form to 1e-12. */
  static function cosine():Void {
    var d = new WaveDeformation(new CosineCam(W0), RN);
    for (i in -200...401) {
      var phi = i * 0.0173;
      var w = W0 * Math.cos(2 * phi), v = -(W0 / 2) * Math.sin(2 * phi), mu = 1.5 * W0 / RN * Math.sin(2 * phi);
      worstClosedForm = Math.max(worstClosedForm, Math.max(Math.abs(d.v(phi) - v), Math.abs(d.mu(phi) - mu)));
      near(d.w(phi), w, 1e-15, "cosine w");
      near(d.v(phi), v, 1e-12, "cosine v closed form");
      near(d.mu(phi), mu, 1e-12, "cosine mu closed form");
    }
  }

  /** Gravagno 2016 eq 18: values, zero mean, symmetry, flat at both axes. */
  static function resal():Void {
    var c = new ResalCam(W0), pi = Math.PI;
    near(c.w(0), W0, 1e-15, "resal w(0) = w0 at the rollers");
    near(c.w(pi), W0, 1e-15, "resal w(pi)");
    near(c.w(pi / 2), -W0 * (1 / pi - 0.25) / (pi / 8 - 1 / pi), 1e-15, "resal minor-axis value");
    near(c.w(pi / 2) / W0, -0.9180, 5e-4, "resal minor axis is about -0.918 w0");
    near(c.dw(0), 0, 1e-15, "resal dw at the major axis");
    near(c.dw(pi / 2), 0, 1e-15, "resal dw at the minor axis");
    near(c.dw(pi - 1e-9), 0, 1e-8, "resal dw just before pi");
    var mean = Quadrature.integrate(x -> c.w(x), 0, pi, [], 1e-3) / pi;
    near(mean, 0, 1e-12, "resal mean is zero");
    for (i in 0...60) {
      var phi = 0.0211 * i + 0.004;
      near(c.w(-phi), c.w(phi), 1e-14, "resal even about the major axis");
      near(c.w(pi - phi), c.w(phi), 1e-14, "resal even about the minor axis");
      near(c.w(phi + pi), c.w(phi), 1e-14, "resal period pi");
      near(c.dw(-phi), -c.dw(phi), 1e-14, "resal dw odd");
      var h = 1e-6;
      near((c.w(phi + h) - c.w(phi - h)) / (2 * h), c.dw(phi), 1e-7, "resal dw is the derivative of w");
    }
    // Different from the cosine: not symmetric about its own mean.
    check(Math.abs(c.w(pi / 2) + W0) > 0.05 * W0, "resal is not a cosine");
  }

  static function general(cam:Cam):Void {
    var name = cam.name(), d = new WaveDeformation(cam, RN);
    near(cam.w(0), W0, 1e-12, '$name w(0) = w0');
    check(cam.w(Math.PI / 2) < 0, '$name minor axis is pulled in');
    // Closure: the mean of w vanishes, so the tangential displacement is periodic and v(pi/2) = 0.
    worstClosure = Math.max(worstClosure, Math.abs(d.closure()));
    near(d.closure(), 0, 1e-9, '$name quarter-turn integral of w');
    near(d.v(Math.PI / 2), 0, 1e-9, '$name v closes at the minor axis');
    near(d.v(Math.PI), 0, 1e-9, '$name v closes after half a turn');
    near(d.v(0), 0, 1e-15, '$name v(0)');
    // Inextensibility w + dv/dphi = 0, by a fourth-order central difference of v.
    var h = 1e-3;
    for (i in 0...181) {
      var phi = 0.0173 * i + 0.003;
      // The stencil is only valid away from arc joins, where w'' jumps.
      var nearJoin = false;
      for (b in cam.breakpoints()) for (m in [b, Math.PI - b, Math.PI + b, 2 * Math.PI - b, 2 * Math.PI + b]) if (Math.abs(phi - m) < 4 * h) nearJoin = true;
      if (nearJoin) continue;
      var dv = (-d.v(phi + 2 * h) + 8 * d.v(phi + h) - 8 * d.v(phi - h) + d.v(phi - 2 * h)) / (12 * h);
      worstInextensible = Math.max(worstInextensible, Math.abs(cam.w(phi) + dv));
      near(cam.w(phi) + dv, 0, 1e-9, '$name inextensibility w + v\'');
      // analytic derivative agrees with a difference of w
      var dwNum = (-cam.w(phi + 2 * h) + 8 * cam.w(phi + h) - 8 * cam.w(phi - h) + cam.w(phi - 2 * h)) / (12 * h);
      near(cam.dw(phi), dwNum, 1e-8, '$name dw');
    }
    // Two-lobe symmetry: w even, v and mu odd about the major axis; period pi.
    for (i in 1...100) {
      var x = 0.0157 * i;
      near(d.w(-x), d.w(x), 1e-12, '$name w even');
      near(d.v(-x), -d.v(x), 1e-12, '$name v odd');
      near(d.mu(-x), -d.mu(x), 1e-12, '$name mu odd');
      near(d.w(x + Math.PI), d.w(x), 1e-12, '$name w period');
      near(d.v(x + Math.PI), d.v(x), 1e-12, '$name v period');
      near(d.w(Math.PI - x), d.w(x), 1e-12, '$name w even about the minor axis');
    }
    // The exact arc length of the deformed neutral line differs from 2 pi rn only at second order in w0/rn.
    var strain = d.lengthStrain(), bound = 3 * (W0 / RN) * (W0 / RN);
    check(Math.abs(strain) < bound, '$name exact length strain $strain exceeds second-order bound $bound');
  }

  /** The four-roller profile is two circular arcs, tangent at the join. */
  static function fourRollerArcs():Void {
    var cam = new FourRollerCam(RN, W0);
    var a = RN + W0, b = RN - W0;
    near(cam.profilePoint(0).x, a, 1e-12, "major semi-axis");
    near(cam.profilePoint(Math.PI / 2).y, b, 1e-12, "minor semi-axis");
    var join = cam.joinAngle;
    check(join > 0 && join < Math.PI / 2, "join angle lies between the axes");
    for (i in 0...101) {
      var phi = join * i / 100;
      var p = cam.profilePoint(phi);
      near(Math.sqrt(Math.pow(p.x - cam.lobeCentre, 2) + p.y * p.y), cam.lobeRadius, 1e-12, "lobe contact arc");
      var q = cam.profilePoint(join + (Math.PI / 2 - join) * i / 100);
      near(Math.sqrt(q.x * q.x + Math.pow(q.y - cam.sideCentre, 2)), cam.sideRadius, 1e-12, "side contact arc");
    }
    // Tangent at the join: both circles share the point and the normal (centres collinear with it).
    var j = cam.profilePoint(join);
    var n1 = new Point2(j.x - cam.lobeCentre, j.y), n2 = new Point2(j.x, j.y - cam.sideCentre);
    near((n1.x * n2.y - n1.y * n2.x) / (n1.radius() * n2.radius()), 0, 1e-12, "arcs tangent at the join");
    near(cam.dw(join - 1e-9), cam.dw(join + 1e-9), 1e-6, "w' continuous at the join");
    // Four-fold symmetry of the oval.
    var q1 = cam.profilePoint(0.4), q2 = cam.profilePoint(Math.PI - 0.4), q3 = cam.profilePoint(Math.PI + 0.4);
    near(q2.x, -q1.x, 1e-12, "mirror about the minor axis");
    near(q2.y, q1.y, 1e-12, "mirror about the minor axis");
    near(q3.x, -q1.x, 1e-12, "point symmetry");
    near(q3.y, -q1.y, 1e-12, "point symmetry");
  }

  /** Reflecting the whole picture about the major axis maps the tooth at +delta to the mirrored tooth at -delta. */
  static function poses():Void {
    for (cam in cams()) {
      var kin = new StrainWaveKinematics(200, 202, new WaveDeformation(cam, RN));
      var q = new Point2(0.07, 29.9);
      for (theta in [0.0, 0.3, 1.1, 2.9]) for (delta in [0.0, 0.05, 0.4, 0.9, 1.4]) {
        var p = kin.poseAt(theta, delta).apply(q);
        var m = kin.poseAt(theta, -delta).apply(new Point2(-q.x, q.y));
        // reflect p about the axis through the origin at angle theta from +Y
        var r = p.rotated(-theta);
        var reflected = new Point2(-r.x, r.y).rotated(theta);
        near(reflected.x, m.x, 1e-11, '${cam.name()} mirrored pose x');
        near(reflected.y, m.y, 1e-11, '${cam.name()} mirrored pose y');
      }
      // At the major axis the tooth sits W0 farther out, untilted; at the minor axis W < 0.
      var major = kin.poseAt(0, 0);
      near(major.angle, 0, 1e-15, "no tilt on the major axis");
      near(major.apply(new Point2(0, RN)).y, RN + cam.w(0), 1e-12, "neutral point on the deformed line (major axis)");
      var minor = kin.poseAt(0, Math.PI / 2);
      near(minor.apply(new Point2(0, RN)).radius(), RN + cam.w(Math.PI / 2), 1e-9, "neutral point on the deformed line (minor axis)");
    }
    var kin = new StrainWaveKinematics(200, 202, new WaveDeformation(new CosineCam(W0), RN));
    // CS fixed: one wave-generator turn moves the flexspline two teeth the other way (eta = -2 theta / Zf).
    near(kin.meanRotation(2 * Math.PI), -4 * Math.PI / 200, 1e-15, "flexspline mean rotation per wave-generator turn");
    near(kin.sectionAngle(0, 0), 0, 0, "tooth 0 on the major axis at rest");
    near(kin.sectionAngle(1, 0), 2 * Math.PI / 200, 1e-15, "tooth pitch");
    // Tooth positions repeat with the wave-generator half turn only up to a pitch shift of one tooth.
    var p0 = kin.pose(0, 0), p1 = kin.pose(1, 0);
    near(p1.angle - p0.angle, 2 * Math.PI / 200 + (kin.deformation.mu(2 * Math.PI / 200) - kin.deformation.mu(0)), 1e-12, "tooth angles differ by the pitch plus tilt");
  }
}
