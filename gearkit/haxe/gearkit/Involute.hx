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

  public function baseRadius(spec:GearSpec):Float return spec.pitchRadius() * Math.cos(angle);

  public function validate(spec:GearSpec):Void {
    if (spec.tipRadius() <= baseRadius(spec)) throw "gear root must be positive and tooth tip must lie above the base circle";
  }

  public function profile(spec:GearSpec):ToothProfile return new InvoluteProfile(spec);
}
