package motionkit.bridge;

import machinekit.assembly.LinearAxis;
import motionkit.axis.MotionAxisBlueprint;
import motionkit.axis.MotionSystemBlueprint;
import robotkit.model.Actuator;
import robotkit.model.Joint;
import robotkit.model.JointType;
import robotkit.model.Link;
import robotkit.model.RobotModel;

/** MachineKit mechanical-design to RobotKit execution-model bridge. */
class MachineKitRobotCompiler {
  public static inline final MILLIMETRES_TO_METRES:Float = 0.001;
  public static inline final DEFAULT_MAX_VELOCITY:Float = 0.15;
  public static inline final DEFAULT_MAX_ACCELERATION:Float = 0.5;

  /**
   * Compiles one MachineKit LinearAxis and its logical axis mapping. The
   * returned blueprint exposes the RobotModel used to create the runtime, so
   * design, simulation, and deployment all share one compiled description.
   */
  public static function compileLinearAxis(axis:LinearAxis, axisId:String,
      ?maxVelocity:Float = DEFAULT_MAX_VELOCITY,
      ?maxAcceleration:Float = DEFAULT_MAX_ACCELERATION):MotionSystemBlueprint {
    var model = compileLinearAxisModel(axis, axisId, maxVelocity);
    var joint = model.joints[0];
    var axisBlueprint = new MotionAxisBlueprint(axisId, [joint.id], 0.0,
      axis.stroke * MILLIMETRES_TO_METRES, maxVelocity, maxAcceleration);
    return MotionSystemBlueprint.fromRobotModel(model, [axisBlueprint]);
  }

  /** Compiles only the stable RobotKit model for callers that own the runtime step. */
  public static function compileLinearAxisModel(axis:LinearAxis, axisId:String,
      ?maxVelocity:Float = DEFAULT_MAX_VELOCITY):RobotModel {
    if (axis == null) throw "MachineKit LinearAxis is required";
    requireId(axisId);
    if (!Math.isFinite(maxVelocity) || maxVelocity < 0.0)
      throw "Linear-axis maximum velocity must be finite and non-negative";

    var model = new RobotModel('linear-axis-$axisId');
    var base = model.addLink(new Link('${axisId}.base'));
    var carriage = model.addLink(new Link('${axisId}.carriage'));
    var joint = model.addJoint(new Joint(axisId, JointType.Prismatic, base, carriage, axisId));
    // MachineKit's LinearAxis is authored along +Z and reports millimetres;
    // RobotKit uses metres and a zeroed machine coordinate at travelMin.
    joint.parentFramePosition = [0.0, 0.0,
      (axis.screwStart + axis.travelMin) * MILLIMETRES_TO_METRES];
    joint.axis = [0.0, 0.0, 1.0];
    joint.limits.lower = 0.0;
    joint.limits.upper = axis.stroke * MILLIMETRES_TO_METRES;
    joint.limits.velocity = maxVelocity;
    joint.drive = new Actuator('${axis.motor.designation} / lead-screw ${axis.transmission.lead} mm/rev',
      0.0, maxVelocity);
    return model;
  }

  static function requireId(id:String):Void {
    if (id == null || StringTools.trim(id).length == 0)
      throw "MachineKit axis ID must be non-empty";
  }
}
