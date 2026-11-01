package gearkit;

/** Inputs of one strain-wave tooth design (TOOTH_DESIGN_PLAN.md, TD-D1): tooth counts, module,
    deflection coefficient, cam, and the flexspline tooth form by name. Lengths in mm, angles in
    radians. The deformation is `w0 = kappa * module * (circularTeeth - flexTeeth) / 2`. Validation
    here covers only the inputs; whether the resulting pair meshes is the meshing analysis's job. */
class StrainWaveInput {
  public static final FORMS = ["involute", "double-circular-arc"];
  public static final CAMS = ["cosine", "ellipse", "four-roller", "resal"];

  public final name:String;
  public final form:String;
  public final module:Float;
  public final flexTeeth:Int;
  public final circularTeeth:Int;
  public final kappa:Float;
  public final cam:String;
  /** Involute pressure angle; null for forms that define their own flank. */
  public final pressureAngle:Null<Float>;

  public function new(
    name:String,
    form:String,
    module:Float,
    flexTeeth:Int,
    circularTeeth:Int,
    kappa:Float,
    cam:String,
    ?pressureAngle:Float
  ) {
    if (FORMS.indexOf(form) < 0) throw 'tooth form "$form" is not one of ${FORMS.join(", ")}';
    if (CAMS.indexOf(cam) < 0) throw 'cam "$cam" is not one of ${CAMS.join(", ")}';
    if (!Math.isFinite(module) || module <= 0) throw "module must be finite and positive";
    if (flexTeeth < 20) throw "flexspline needs at least twenty teeth";
    if (circularTeeth - flexTeeth != 2) throw "circular spline must have exactly two more teeth than the flexspline (two-lobe cam)";
    if (!Math.isFinite(kappa) || kappa <= 0) throw "deflection coefficient kappa must be finite and positive";
    if (form == "involute") {
      if (pressureAngle == null || !Math.isFinite(pressureAngle) || pressureAngle <= 0
        || pressureAngle >= Math.PI / 2) throw "involute pressure angle must be between zero and pi/2 radians";
    } else if (pressureAngle != null) throw 'tooth form "$form" defines its own flank; pressure angle is not an input';
    this.name = name;
    this.form = form;
    this.module = module;
    this.flexTeeth = flexTeeth;
    this.circularTeeth = circularTeeth;
    this.kappa = kappa;
    this.cam = cam;
    this.pressureAngle = pressureAngle;
  }

  /** Radial deflection amplitude of the flexspline neutral line, mm. */
  public function w0():Float return kappa * module * (circularTeeth - flexTeeth) / 2;

  public function flexPitchRadius():Float return module * flexTeeth / 2;
  public function circularPitchRadius():Float return module * circularTeeth / 2;
}
