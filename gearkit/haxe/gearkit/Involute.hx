package gearkit;

/** Involute tooth form of a rack-generated spur gear: full depth, addendum 1.0 m, dedendum 1.25 m. */
class Involute implements ToothForm {
  final angle:Float;

  public function new(pressureAngle:Float) {
    if (!Math.isFinite(pressureAngle) || pressureAngle <= 0
      || pressureAngle >= Math.PI / 2) throw "gear pressure angle must be between zero and pi/2 radians";
    angle = pressureAngle;
  }

  public function name():String return "involute";
  public function pressureAngle():Float return angle;
  public function addendum(module:Float):Float return module;
  public function dedendum(module:Float):Float return 1.25 * module;

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

  public function profile(spec:GearSpec):ToothProfile return new InvoluteProfile(spec);
}
