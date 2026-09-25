package robotkit.runtime;

/** Immutable resolved configuration used to construct a user-facing MobileBase. */
class RobotRuntimeMobileConfiguration {
  public final drive:RobotRuntimeDriveConfiguration;
  public final maxLinearSpeed:Float;
  public final maxAngularSpeed:Float;
  public final maxLinearAcceleration:Float;
  public final maxAngularAcceleration:Float;
  public final footprintLength:Null<Float>;
  public final footprintWidth:Null<Float>;

  public function new(
    drive:RobotRuntimeDriveConfiguration,
    maxLinearSpeed:Float,
    maxAngularSpeed:Float,
    maxLinearAcceleration:Float,
    maxAngularAcceleration:Float,
    footprintLength:Null<Float>,
    footprintWidth:Null<Float>
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
