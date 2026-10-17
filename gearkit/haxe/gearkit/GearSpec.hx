package gearkit;

/** Full-depth normal-system spur gear. Lengths in mm, angles in radians.
    Positive shift thickens external teeth and thins internal teeth.
    Fillet defaults are design assumptions, not catalogue/standard claims. */
class GearSpec {
  public final module:Float;
  public final teeth:Int;
  public final faceWidth:Float;
  public final pressureAngle:Float;
  public final internal:Bool;
  public final profileShift:Float;
  public final cutterTipRadius:Float;
  public final tipFilletRadius:Float;
  public final evaluation:Null<GearEvaluationRange>;

  public function new(
    module:Float,
    teeth:Int,
    faceWidth:Float,
    pressureAngle:Float,
    internal:Bool = false,
    profileShift:Float = 0,
    ?cutterTipRadius:Float,
    ?tipFilletRadius:Float,
    ?evaluation:GearEvaluationRange
  ) {
    if (!Math.isFinite(module) || module <= 0) throw "gear module must be finite and positive";
    if (teeth < 3) throw "gear must have at least three teeth";
    if (!Math.isFinite(faceWidth) || faceWidth <= 0) throw "gear face width must be finite and positive";
    if (!Math.isFinite(pressureAngle) || pressureAngle <= 0
      || pressureAngle >= Math.PI / 2) throw "gear pressure angle must be between zero and pi/2 radians";
    if (!Math.isFinite(profileShift)
      || profileShift <= -1 || profileShift >= 1) throw "profile shift must be finite and strictly between -1 and 1";
    var corner = cutterTipRadius == null ? 0.38 * module : cutterTipRadius;
    var tip = tipFilletRadius == null ? 0.05 * module : tipFilletRadius;
    if (!Math.isFinite(corner)
      || corner <= 0 || corner >= module) throw "cutter tip radius must be positive and less than module";
    if (!Math.isFinite(tip)
      || tip < 0 || tip >= module) throw "tip fillet radius must be non-negative and less than module";
    if (!Math.isFinite(module * teeth)) throw "gear pitch diameter must be finite";
    this.module = module;
    this.teeth = teeth;
    this.faceWidth = faceWidth;
    this.pressureAngle = pressureAngle;
    this.internal = internal;
    this.profileShift = profileShift;
    this.cutterTipRadius = corner;
    this.tipFilletRadius = tip;
    this.evaluation = evaluation;
    if (rootRadius() <= 0
      || tipRadius() <= baseRadius()) throw "gear root must be positive and tooth tip must lie above the base circle";
    if (pitchToothThickness() <= 0
      || pitchToothThickness() >= Math.PI * module) throw "profile shift leaves no tooth or tooth space";
  }

  public function sign():Float return internal ? -1.0 : 1.0;
  public function pitchDiameter():Float return module * teeth;
  public function pitchRadius():Float return pitchDiameter() / 2;
  public function baseRadius():Float return pitchRadius() * Math.cos(pressureAngle);
  public function tipRadius():Float return pitchRadius() + sign() * module + profileShift * module;
  public function rootRadius():Float return pitchRadius() - sign() * 1.25 * module + profileShift * module;
  public function pitchToothThickness():Float return Math.PI
    * module / 2 + sign() * 2 * profileShift * module * Math.tan(pressureAngle);
  /** Default is the entire unrounded flank and face; HB4 supplies standard-specific windows. */
  public function evaluationRange():GearEvaluationRange return new ToothProfile(this).evaluationRange();
}
