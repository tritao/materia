package machinekit.motion;

/**
 * A motor's mounting flange, the interface every motor mount and plate receives: the square `face`, the flange
 * `flangeThickness`, the locating pilot (`pilotDiameter`, standing `pilotHeight` proud of the face) and the bolt
 * `holes` in the face plane (mm, about the shaft at z = 0). A flange is either tapped, `thread` set (NEMA steppers:
 * the mount takes clearance holes and screws into the motor), or drilled, `holeDiameter` > 0 (servos: the motor takes
 * clearance and the mount is tapped). Null flange data means the interface has not been specified.
 */
typedef MotorFlange = {
  var face:Float;
  var flangeThickness:Float;
  var pilotDiameter:Float;
  var pilotHeight:Float;
  var holeDiameter:Float;
  var holes:Array<{x:Float, y:Float}>;
  @:optional var thread:String;
  /** The screw size its holes take, where the maker states it (a drilled NEMA flange's M5). */
  @:optional var screw:String;
  @:optional var threadDepth:Float;
}
