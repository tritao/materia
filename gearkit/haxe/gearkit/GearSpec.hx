package gearkit;

/** Nominal spur-gear dimensions. Lengths are millimetres; angles are radians.
    Tooth geometry and metrology evaluation ranges are added in HB1. */
class GearSpec {
  public final module:Float;
  public final teeth:Int;
  public final faceWidth:Float;
  public final pressureAngle:Float;

  public function new(module:Float, teeth:Int, faceWidth:Float, pressureAngle:Float) {
    if (!Math.isFinite(module) || module <= 0) throw "gear module must be finite and positive";
    if (teeth < 3) throw "gear must have at least three teeth";
    if (!Math.isFinite(faceWidth) || faceWidth <= 0) throw "gear face width must be finite and positive";
    if (!Math.isFinite(pressureAngle) || pressureAngle <= 0 || pressureAngle >= Math.PI / 2)
      throw "gear pressure angle must be between zero and pi/2 radians";
    if (!Math.isFinite(module * teeth)) throw "gear pitch diameter must be finite";
    this.module = module;
    this.teeth = teeth;
    this.faceWidth = faceWidth;
    this.pressureAngle = pressureAngle;
  }

  public function pitchDiameter():Float return module * teeth;
}
