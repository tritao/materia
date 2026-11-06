package toolpathkit.motion;

import motionkit.robot.CouplingProjection;
import robotkit.runtime.RobotRuntimeBlueprint;
import robotkit.runtime.RobotRuntimeVelocityDriveBlueprint;

/** A process drive's channels expressed in the physical spindle's coordinates. */
class SpindleDriveBinding {
  public final drive:RobotRuntimeVelocityDriveBlueprint;
  public final name:String;
  /** Signed spindle radians/second per positive speed-channel unit and positive direction. */
  public final radiansPerSpeedUnit:Float;

  function new(drive:RobotRuntimeVelocityDriveBlueprint, ratio:Float) {
    this.drive = drive;
    var suffix = ".speed";
    if (!StringTools.endsWith(drive.speedChannel, suffix)) throw "Spindle needs named speed/direction channels";
    name = drive.speedChannel.substr(0, drive.speedChannel.length - suffix.length);
    if (name.length == 0 || drive.directionChannel != name + ".direction")
      throw "Spindle channels must share their declared name";
    radiansPerSpeedUnit = ratio * drive.radiansPerSpeedUnit;
    if (!Math.isFinite(radiansPerSpeedUnit) || radiansPerSpeedUnit == 0)
      throw "Spindle needs a finite nonzero drive speed scale";
  }

  /** Resolve through compiled mechanical couplings, with no assumptions about motor/load joint ordering. */
  public static function resolve(blueprint:RobotRuntimeBlueprint, joint:Int):SpindleDriveBinding {
    var projection = CouplingProjection.of([for (term in blueprint.couplings) {
      leader: term.leader, follower: term.follower, ratio: term.ratio, offset: term.offset
    }], blueprint.jointCount);
    var matches:Array<{drive:RobotRuntimeVelocityDriveBlueprint, ratio:Float}> = [];
    for (drive in blueprint.velocityDrives) {
      var ratio = projection.relativeRate(drive.joint, joint, blueprint.jointCount);
      if (ratio != null) matches.push({drive: drive, ratio: ratio});
    }
    if (matches.length != 1) throw "Spindle needs one declared velocity drive through its mechanical couplings";
    return new SpindleDriveBinding(matches[0].drive, matches[0].ratio);
  }

  /** Magnitude sent to the declared analog speed channel for physical spindle RPM. */
  public function speed(rpm:Float):Float return rpm * (2 * Math.PI / 60) / Math.abs(radiansPerSpeedUnit);
  public function direction(sign:Float):Float return radiansPerSpeedUnit < 0 ? -sign : sign;
}
