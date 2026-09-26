package motionkit;

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
    var model = compileLinearAxisModel(axis, axisId, maxVelocity, maxAcceleration);
    var joint = model.joints[0];
    var axisBlueprint = new MotionAxisBlueprint(axisId, [joint.id], 0.0,
      axis.stroke * MILLIMETRES_TO_METRES, maxVelocity, maxAcceleration);
    return MotionSystemBlueprint.fromRobotModel(model, [axisBlueprint]);
  }

  /** Compiles only the stable RobotKit model for callers that own the runtime step. */
  public static function compileLinearAxisModel(axis:LinearAxis, axisId:String,
      ?maxVelocity:Float = DEFAULT_MAX_VELOCITY,
      ?maxAcceleration:Float = DEFAULT_MAX_ACCELERATION):RobotModel {
    requireAxis(axis, axisId);
    requireId(axisId);
    if (!Math.isFinite(maxVelocity) || maxVelocity < 0.0)
      throw "Linear-axis maximum velocity must be finite and non-negative";
    if (!Math.isFinite(maxAcceleration) || maxAcceleration < 0.0)
      throw "Linear-axis maximum acceleration must be finite and non-negative";

    var model = new RobotModel('linear-axis-$axisId');
    var base = model.addLink(new Link('${axisId}.base'));
    var carriage = model.addLink(new Link('${axisId}.carriage'));
    addPrismaticJoint(model, axisId, base, carriage, axis,
      [0.0, 0.0, 0.0, 1.0], [0.0, 0.0, axis.screwStart + axis.travelMin],
      maxVelocity, maxAcceleration);
    return model;
  }

  /** Compiles three MachineKit axes into one serial XYZ gantry robot. */
  public static function compileXYZGantry(xAxis:LinearAxis, yAxis:LinearAxis, zAxis:LinearAxis,
      ?maxVelocity:Float = DEFAULT_MAX_VELOCITY,
      ?maxAcceleration:Float = DEFAULT_MAX_ACCELERATION):MotionSystemBlueprint {
    requireAxis(xAxis, "x");
    requireAxis(yAxis, "y");
    requireAxis(zAxis, "z");
    if (!Math.isFinite(maxVelocity) || maxVelocity < 0.0)
      throw "Gantry maximum velocity must be finite and non-negative";
    if (!Math.isFinite(maxAcceleration) || maxAcceleration < 0.0)
      throw "Gantry maximum acceleration must be finite and non-negative";

    var model = new RobotModel("xyz-gantry");
    var base = model.addLink(new Link("gantry.base"));
    var xCarriage = model.addLink(new Link("x.carriage"));
    var yCarriage = model.addLink(new Link("y.carriage"));
    var zCarriage = model.addLink(new Link("z.carriage"));
    var halfSqrt = Math.sqrt(0.5);
    // MachineKit LinearAxis is authored along local +Z. Rotate each carriage
    // frame into the corresponding machine coordinate direction.
    addPrismaticJoint(model, "x", base, xCarriage, xAxis,
      [0.0, halfSqrt, 0.0, halfSqrt],
      [xAxis.screwStart + xAxis.travelMin, 0.0, 0.0], maxVelocity, maxAcceleration);
    addPrismaticJoint(model, "y", xCarriage, yCarriage, yAxis,
      [-halfSqrt, 0.0, 0.0, halfSqrt],
      [0.0, yAxis.screwStart + yAxis.travelMin, 0.0], maxVelocity, maxAcceleration);
    // The inherited Y-carriage frame is R_y(90°) * R_x(-90°). Use its
    // inverse for Z and express the travel origin along local -X so both the
    // Z joint axis and its origin land on world +Z.
    addPrismaticJoint(model, "z", yCarriage, zCarriage, zAxis,
      [0.5, -0.5, -0.5, 0.5],
      [-(zAxis.screwStart + zAxis.travelMin), 0.0, 0.0], maxVelocity, maxAcceleration);

    return MotionSystemBlueprint.fromRobotModel(model, [
      axisBlueprint("x", xAxis, maxVelocity, maxAcceleration),
      axisBlueprint("y", yAxis, maxVelocity, maxAcceleration),
      axisBlueprint("z", zAxis, maxVelocity, maxAcceleration)
    ]);
  }

  static function axisBlueprint(id:String, axis:LinearAxis, maxVelocity:Float,
      maxAcceleration:Float):MotionAxisBlueprint {
    return new MotionAxisBlueprint(id, [id], 0.0,
      axis.stroke * MILLIMETRES_TO_METRES, maxVelocity, maxAcceleration);
  }

  static function addPrismaticJoint(model:RobotModel, id:String, parent:Link, child:Link,
      axis:LinearAxis, frameRotation:Array<Float>, framePositionMillimetres:Array<Float>,
      maxVelocity:Float, maxAcceleration:Float):Joint {
    var joint = model.addJoint(new Joint(id, JointType.Prismatic, parent, child, id));
    // MachineKit reports millimetres; RobotKit uses metres and a zeroed
    // machine coordinate at travelMin.
    joint.parentFramePosition = [
      framePositionMillimetres[0] * MILLIMETRES_TO_METRES,
      framePositionMillimetres[1] * MILLIMETRES_TO_METRES,
      framePositionMillimetres[2] * MILLIMETRES_TO_METRES
    ];
    joint.parentFrameRotation = frameRotation.copy();
    joint.axis = [0.0, 0.0, 1.0];
    joint.limits.lower = 0.0;
    joint.limits.upper = axis.stroke * MILLIMETRES_TO_METRES;
    joint.limits.velocity = maxVelocity;
    joint.limits.maxAcceleration = maxAcceleration;
    joint.drive = new Actuator('${axis.motor.designation} / lead-screw ${axis.transmission.lead} mm/rev',
      0.0, maxVelocity);
    return joint;
  }

  static function requireAxis(axis:LinearAxis, id:String):Void {
    if (axis == null) throw 'MachineKit LinearAxis "$id" is required';
  }

  static function requireId(id:String):Void {
    if (id == null || StringTools.trim(id).length == 0)
      throw "MachineKit axis ID must be non-empty";
  }
}
