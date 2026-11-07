import machinekit.hobbing.HobberDrives;
import machinekit.motion.BallNut;
import machinekit.motion.ServoMotor;
import machinekit.sizing.CrashProtection;
import machinekit.sizing.CrashProtection.CrashLimit;

/** One linear axis's crash protection (see `CrashProtection`), with the weight the axis carries along its screw. */
typedef AxisCrash = {id:String, limit:CrashLimit, motorPeakThrust:Float, gravityN:Float, movingKg:Float, note:String};

/**
 * The crash protection of the hobber's X, Y and Z servo drives (FRAME_PLAN, "Crash protection"): the torque limit and the following-error trip each
 * drive is to be set to, derived from the axis's sizing duty. The model has no drive parameter store (`MotorDriver` has a current rating, and the actuator
 * record only the motor's peak effort), so the settings are declared here and the joint checks (`HobberJointLoads`) take the limited thrust from them.
 *
 * To be set in the drives (Leadshine EL8 servo drives on the X, Y and Z servos): the internal torque limit (a share of rated torque, per direction) to
 * `ratedShare` below, and the excessive-position-error alarm (following-error limit) to the trip below. The parameter numbers are in the drive manual, which
 * the repository does not hold, so they are not named here; the limit must be written to the drive at commissioning and read back.
 */
class HobberCrashProtection {
  public static final AXES = ["X", "Y", "Z"];

  /** The duty and the crash protection of axis `id` on `machine`. */
  public static function axis(machine:FlexsplineHobber, id:String):AxisCrash {
    var motor:ServoMotor = HobberJointLoads.member(machine, HobberDrives.member(id,"motor")), nut:BallNut = HobberJointLoads.member(machine, HobberDrives.member(id,"nut"));
    var g = 9.80665, q = HobberJointLoads.LEAD / (2 * Math.PI), first = switch id {
      case "X": "blockXLeft";
      case "Y": "Ymodule/blockLeftFront";
      default: "blockZLeft";
    }, moving = HobberJointLoads.movingMass(machine, first);
    var gravity = id == "Z" ? moving * g : id == "Y" ? moving * g * Math.sin(Math.PI / 12) : 0.0;
    var duty = machinekit.sizing.LinearDriveSizing.demand({
      massKg: moving, motors: 1, metresPerRadian: q, efficiency: nut.efficiency(), rotaryInertiaKgM2: 0.4e-4, acceleration: 0.5, velocity: 0.2,
      strokeM: id == "Y" ? 0.03 : id == "X" ? 0.095 : 0.2, cuttingForceN: HobberJointLoads.CUTTING_FORCE, dragForceN: 100, gravityForceN: gravity,
      rotaryDragNm: 0.1, reserve: 1.5
    }, motor.rating);
    var perNm = nut.efficiency() / q;
    return {
      id: id, limit: CrashProtection.limit(duty.peakNm, perNm, motor.rating), motorPeakThrust: motor.rating.peakTorque * perNm, gravityN: gravity,
      movingKg: moving, note: id == "Z" ? "gravity along the screw" : id == "Y" ? "gravity component along the tilted screw" : ""
    };
  }

  /** The report lines: each axis's settings, the thrust a jam makes with them and the motor-peak thrust it replaces. */
  public static function report(machine:FlexsplineHobber):Array<String> {
    var lines = [];
    for (id in AXES) {
      var a = axis(machine, id);
      lines.push(CrashProtection.describe(id, a.limit) + '; motor peak would be ${Math.round(a.motorPeakThrust)} N, weight along the screw ${Math.round(a.gravityN)} N (${Math.round(a.movingKg * 10) / 10} kg)');
    }
    return lines;
  }
}
