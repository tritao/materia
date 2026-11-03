package machinekit.motion;

import cadkit.modeling.Part;
import machinekit.component.MachineComponent;

/** Mechanical interface of an axis motor: its mounting flange, its shaft, and the tool that cuts a plate to receive
 * it. The owning part supplies them; builders never infer a motor's interface from its body shape. */
class AxisMotor {
  public final part:MachineComponent;
  public final flange:MotorFlange;
  public final face:Float;
  public final shaftDiameter:Float;
  public final shaftLength:Float;
  final cutout:Float -> Part;
  final tappedCutout:Null<Float -> Part>;
  public function new(
    part:MachineComponent,
    flange:MotorFlange,
    shaftDiameter:Float,
    shaftLength:Float,
    cutout:Float -> Part,
    ?tappedCutout:Float -> Part
  ) {
    this.tappedCutout = tappedCutout;
    if (part.motorDrive() == null) throw 'Axis motor ' + part.designation + ' has no drive ratings';
    var face = flange.face;
    for (value in [face,
      shaftDiameter, shaftLength]) if (!Math.isFinite(value) || value <= 0) throw 'Invalid axis motor interface';
    this.part = part;
    this.flange = flange;
    this.face = face;
    this.shaftDiameter = shaftDiameter;
    this.shaftLength = shaftLength;
    this.cutout = cutout;
  }
  public function mountingCutout(depth:Float):Part return cutout(depth);
  /** The cut-out for a tapped host (tap-drill holes), where the flange has clearance holes; null otherwise. */
  public function hostCutout():Null<Float -> Part> return tappedCutout;
  public static function of(part:MachineComponent):AxisMotor {
    var result = part.axisMotor();
    if (result == null) throw 'Component ' + part.designation + ' does not declare an axis motor interface';
    return result;
  }
}
