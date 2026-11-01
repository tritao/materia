package gearkit;

/** A flexspline and its wave-generator kinematics built from a `StrainWaveInput`, with the
    circular-spline tooth space derived from them (TOOTH_DESIGN_PLAN.md TD-D1, TD-D3).
    The double-circular-arc form is Liu 2025's optimum scaled to the input module (`DoubleCircularArc.liu`).
    **Assumed:** wall under the root `0.01 m Z_f` (so the neutral line is half a wall below the
    root circle), face width 8 mm (the analysis is planar). */
class StrainWavePair {
  public final input:StrainWaveInput;
  public final flex:GearSpec;
  public final profile:ToothProfile;
  public final kinematics:StrainWaveKinematics;
  public final cam:Cam;
  public final wall:Float;
  var cached:Null<SweptSpace>;

  public function new(input:StrainWaveInput, ?cutterTipRadius:Float, ?tipFilletRadius:Float) {
    this.input = input;
    var form:ToothForm;
    if (input.form == "involute") {
      var angle = input.pressureAngle;
      if (angle == null) throw "involute design without a pressure angle";
      form = new Involute(angle, input.addendumFactor, input.tipRelief);
    } else if (input.form == "double-circular-arc") form = DoubleCircularArc.liu(input.module);
    else throw 'tooth form "${input.form}" is not implemented';
    flex = new GearSpec(input.module, input.flexTeeth, 8, form, false, input.profileShift, cutterTipRadius, tipFilletRadius);
    profile = flex.profile();
    wall = 0.01 * input.module * input.flexTeeth;
    var rn = flex.rootRadius() - wall / 2;
    cam = Cams.make(input.cam, rn, input.w0());
    kinematics = new StrainWaveKinematics(input.flexTeeth, input.circularTeeth, new WaveDeformation(cam, rn));
  }

  public function circularPitchRadius():Float return input.circularPitchRadius();
  /** Nominal circular-spline tip (inner) and root (outer) radii (assumed): pitch -/+ the addendum and dedendum of the
      nominal full-depth mate (1 and 1.25 modules for an involute, whatever the flexspline's own addendum or shift), or
      the flexspline form's own for a form that defines them. */
  public function circularTipRadius():Float {
    var inv = Std.isOfType(flex.form, Involute);
    return circularPitchRadius() - (inv ? input.module : flex.form.addendum(input.module));
  }
  public function circularRootRadius():Float return circularPitchRadius() + flex.form.dedendum(input.module);

  /** Open tooth boundary in the tooth frame: one pitch of outline without the closing chord. */
  public function boundary(samples:Int):Array<Point2> {
    var o = profile.outline(samples);
    return o.slice(0, o.length - 1);
  }

  /** Largest radius any point of the tooth reaches, at the major axis. */
  public function reach():Float {
    var pose = kinematics.pose(0, 0), best = 0.0;
    for (p in boundary(64)) best = Math.max(best, pose.apply(p).radius());
    return best;
  }

  /** The derived circular-spline tooth space. Sweeps tooth 0 through section angles |delta| <= pi/2. */
  public function space(coarseSteps:Int = 2400, coarseBins:Int = 400):SweptSpace {
    if (cached != null) return cached;
    var theta = Math.PI / 2 * input.flexTeeth / input.circularTeeth;
    var rHi = reach() - 1e-6 * input.module;
    cached = CircularSplineEnvelope.derive(
      boundary(32), boundary(64), t -> kinematics.pose(0, t), -theta, theta, coarseSteps, input.circularTeeth,
      circularTipRadius(), rHi, coarseBins
    );
    return cached;
  }
}
