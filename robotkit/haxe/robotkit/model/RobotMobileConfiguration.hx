package robotkit.model;

/** Mobile-base roles, command limits, and optional rectangular footprint. */
class RobotMobileConfiguration {
  public final drive:RobotDriveConfiguration;
  public final maxLinearSpeed:Float;
  public final maxAngularSpeed:Float;
  public final maxLinearAcceleration:Float;
  public final maxAngularAcceleration:Float;
  public final footprintLength:Null<Float>;
  public final footprintWidth:Null<Float>;

  public function new(
    drive:RobotDriveConfiguration,
    maxLinearSpeed:Float,
    maxAngularSpeed:Float,
    ? maxLinearAcceleration:Float = 1.0e300,
    ? maxAngularAcceleration:Float = 1.0e300,
    ? footprintLength:Float,
    ? footprintWidth:Float
  ) {
    this.drive = drive;
    this.maxLinearSpeed = maxLinearSpeed;
    this.maxAngularSpeed = maxAngularSpeed;
    this.maxLinearAcceleration = maxLinearAcceleration;
    this.maxAngularAcceleration = maxAngularAcceleration;
    this.footprintLength = footprintLength;
    this.footprintWidth = footprintWidth;
  }
}
