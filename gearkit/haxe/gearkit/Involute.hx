package gearkit;

/** Involute tooth form of a rack-generated spur gear. Defaults: full depth, addendum 1.0 m, dedendum 1.25 m.
    `addendumFactor` shortens (or lengthens) the tooth by turning the blank to a different outside diameter: the
    dedendum, the hob and the rack are unchanged. `tipRelief` removes material from the flank near the tip (`TipRelief`).
    Profile shift is the `GearSpec`'s. */
class Involute implements ToothForm {
  /** Dedendum in modules, before profile shift. */
  public static final DEDENDUM = 1.25;

  final angle:Float;
  /** Addendum in modules, before profile shift. */
  public final addendumFactor:Float;
  public final tipRelief:Null<TipRelief>;

  public function new(pressureAngle:Float, addendumFactor:Float = 1.0, ?tipRelief:TipRelief) {
    if (!Math.isFinite(pressureAngle) || pressureAngle <= 0
      || pressureAngle >= Math.PI / 2) throw "gear pressure angle must be between zero and pi/2 radians";
    if (!Math.isFinite(addendumFactor) || addendumFactor <= 0) throw "gear addendum factor must be finite and positive";
    angle = pressureAngle;
    this.addendumFactor = addendumFactor;
    this.tipRelief = tipRelief;
  }

  public function name():String return "involute";
  public function pressureAngle():Float return angle;
  public function addendum(module:Float):Float return addendumFactor * module;
  public function dedendum(module:Float):Float return DEDENDUM * module;

  public function pitchToothThickness(spec:GearSpec):Float return Math.PI
    * spec.module / 2 + spec.sign() * 2 * spec.profileShift * spec.module * Math.tan(angle);

  /** Base circle radius of an involute gear. Other forms have no base circle and are rejected. */
  public static function baseRadius(spec:GearSpec):Float {
    var involute = Std.downcast(spec.form, Involute);
    if (involute == null) throw 'tooth form "${spec.form.name()}" has no base circle';
    return spec.pitchRadius() * Math.cos(involute.angle);
  }

  public function validate(spec:GearSpec):Void {
    if (spec.tipRadius() <= spec.pitchRadius() * Math.cos(angle)) throw "gear tooth tip must lie above the base circle";
  }

  public function profile(spec:GearSpec):ToothProfile return new InvoluteProfile(spec, tipRelief);
}
