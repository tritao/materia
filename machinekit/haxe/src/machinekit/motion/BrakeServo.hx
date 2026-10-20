package machinekit.motion;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import materia.assembly.AssemblyFrames as F;

/** Assumed catalogue-class AC servo with spring-applied holding brake. Dimensions mm; SI ratings unless stated.
 * Geometry is a nominal layout envelope, not manufacturing or supplier geometry. */
class BrakeServo extends AssumedCatalogComponent {
  static var recipe:Null<ComponentType>;
  static function inputs():Array<ComponentParameter> return [
    P.choice("power", ["400", "750"], "400"),
    P.quantity("brakeTorque", 2.5, "N m"),
    P.length("brakeLength", 35),
    P.quantity("releaseDelay", 0.05, "s"),
    P.quantity("applicationDelay", 0.03, "s")
  ];
  public function new(?v:ComponentValues) {
    super("brake-servo", "AC servo with spring-applied holding brake", "aluminium 6061", inputs(), v);
    positive(["brakeTorque", "brakeLength", "releaseDelay", "applicationDelay"]);
    motor = new AxisServoMotor("GENERIC-HV-SERVO-" + parameterToken("power") + "W");
    require(n("brakeTorque") >= motor.rating.ratedTorque, "holding brake below motor rated torque");
    addConnector("mountFace", Mount, Solids.axial(0, 0, 0));
    addConnector("shaftAxis", Axis, Solids.axial(0, 0, 0));
    addConnector("shaftEnd", Axis, Solids.axial(0, 0, motor.rating.shaftLength));
  }
  public static function recipeType():ComponentType {
    if (recipe == null) recipe = new ComponentType("machinekit.motion.brake-servo", inputs(), v -> new BrakeServo(v));
    return recipe;
  }
  override public function componentType():Null<ComponentType> return recipeType();
  override public function geometry(detail:ComponentDetail = Preview):Part {
    return Solids.union([
      Solids.named(motor.geometry(detail), "motor"),
      Solids.named(
        Part.cylinderSpan(
          motor.rating.bodyDiameter / 2,
          -motor.rating.bodyLength - n("brakeLength"),
          -motor.rating.bodyLength
        ),
        "brake"
      )
    ]);
  }
  public final motor:ServoMotor;
  override public function axisMotor():Null<AxisMotor> {
    var face=AxisMotor.of(motor);
    return new AxisMotor(this,face.face,face.shaftDiameter,face.shaftLength,depth->face.mountingCutout(depth));
  }
  override public function motorDrive():Null<MotorDrive> return motor;
  override public function motorBrakeSpec():Null<MotorBrakeSpec> return new BrakeServoSpec(this);
}
private class BrakeServoSpec implements MotorBrakeSpec {
  final part:BrakeServo;
  public function new(part:BrakeServo) this.part = part;
  public function holdingTorque():Float return part.n("brakeTorque");
  public function releaseDelaySeconds():Float return part.n("releaseDelay");
  public function applicationDelaySeconds():Float return part.n("applicationDelay");

}
