package motionkit.robot;

import cadkit.modeling.AssemblyModel;
import machinekit.assembly.LinearAxis;
import motionkit.axis.MotionAxisBlueprint;
import robotkit.model.Actuator;
import robotkit.model.Joint;
import robotkit.model.JointType;
import robotkit.model.Link;
import robotkit.model.RobotModel;
import robotkit.model.Transmission;

/** Stable joint IDs from a physical MachineKit axis assembly. */
typedef AssemblyAxisBinding = {
  var id:String;
  var axis:LinearAxis;
  /** The link carrying the motor body; fixed parts share one link per rigid body. */
  var motorLinkId:String;
  var shaftJointId:String;
  var travelJointId:String;
}

/** MachineKit mechanical-design to RobotKit execution-model bridge. */
class MachineKitRobotCompiler {
  public static inline final MILLIMETRES_TO_METRES:Float = 0.001;
  public static inline final DEFAULT_MAX_VELOCITY:Float = 0.15;
  public static inline final DEFAULT_MAX_ACCELERATION:Float = 0.5;

  /** Attach MachineKit drives to the part-level assembly model. The motor
   * drives its shaft joint; the existing lead-screw JointCoupling relates
   * shaft rotation to carriage travel. No second joint topology is built. */
  public static function compileAssemblyAxes(model:RobotModel,
      bindings:Array<AssemblyAxisBinding>,
      ?maxVelocity:Float = DEFAULT_MAX_VELOCITY,
      ?maxAcceleration:Float = DEFAULT_MAX_ACCELERATION):MotionSystemBlueprint {
    if (model == null || bindings == null || bindings.length == 0)
      throw "Assembly axes need a physical robot model and bindings";
    if (!Math.isFinite(maxVelocity) || maxVelocity <= 0 ||
        !Math.isFinite(maxAcceleration) || maxAcceleration <= 0)
      throw "Assembly axis limits must be finite and positive";
    var axes:Array<MotionAxisBlueprint> = [];
    var used = new Map<String, Bool>();
    // Validate the entire mapping before changing the supplied model.
    for (binding in bindings) {
      if (binding == null) throw "Assembly axis binding is required";
      requireId(binding.id);
      requireAxis(binding.axis, binding.id);
      if (used.exists(binding.id)) throw 'Duplicate assembly axis "${binding.id}"';
      used.set(binding.id, true);
      var shaft:Null<Joint> = null, travel:Null<Joint> = null;
      for (joint in model.joints) {
        if (joint.id == binding.shaftJointId) shaft = joint;
        if (joint.id == binding.travelJointId) travel = joint;
      }
      if (shaft == null || travel == null || shaft.type != JointType.Continuous ||
          travel.type != JointType.Prismatic || shaft.parent.id != binding.motorLinkId)
        throw 'Assembly axis "${binding.id}" has no matching motor shaft and carriage joints';
      var expected = 2.0 * Math.PI /
        (binding.axis.transmission.lead * binding.axis.transmission.direction *
          MILLIMETRES_TO_METRES);
      var matched = false;
      for (coupling in model.couplings)
        if (coupling.leader == travel.id && coupling.follower == shaft.id &&
            Math.abs(coupling.ratio - expected) <= Math.abs(expected) * 1e-8)
          matched = true;
      if (!matched)
        throw 'Assembly axis "${binding.id}" has no matching lead-screw joint coupling';
      for (actuator in model.actuators) switch actuator.transmission {
        case SimpleTransmission(jointId, _, _) if (jointId == shaft.id):
          throw 'Assembly motor shaft "${shaft.id}" already has an actuator';
        case _:
      }
      axes.push(new MotionAxisBlueprint(binding.id, [travel.id, shaft.id],
        0.0, binding.axis.stroke * MILLIMETRES_TO_METRES,
        maxVelocity, maxAcceleration));
    }
    for (index in 0...bindings.length) {
      var binding = bindings[index];
      var shaftId = binding.shaftJointId;
      var ratio = 2.0 * Math.PI /
        (binding.axis.transmission.lead * binding.axis.transmission.direction *
          MILLIMETRES_TO_METRES);
      var motor = new Actuator('${binding.id}.motor.${binding.axis.motor.designation}',
        0.0, maxVelocity * Math.abs(ratio),
        Transmission.SimpleTransmission(shaftId, 1.0, 0.0));
      motor.fullStepsPerRevolution = fullSteps(binding.axis.motor);
      model.addActuator(motor);
    }
    return MotionSystemBlueprint.fromRobotModel(model, axes);
  }

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
    var carriageStart = machineAxisStart(axis, 'axis-$axisId');
    addPrismaticJoint(model, axisId, base, carriage, axis,
      [0.0, 0.0, 0.0, 1.0], [0.0, 0.0, carriageStart],
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
    var xStart = machineAxisStart(xAxis, "x-axis");
    var yStart = machineAxisStart(yAxis, "y-axis");
    var zStart = machineAxisStart(zAxis, "z-axis");
    // MachineKit LinearAxis is authored along local +Z. Rotate each carriage
    // frame into the corresponding machine coordinate direction.
    addPrismaticJoint(model, "x", base, xCarriage, xAxis,
      [0.0, halfSqrt, 0.0, halfSqrt],
      [xStart, 0.0, 0.0], maxVelocity, maxAcceleration);
    addPrismaticJoint(model, "y", xCarriage, yCarriage, yAxis,
      [-halfSqrt, 0.0, 0.0, halfSqrt],
      [0.0, yStart, 0.0], maxVelocity, maxAcceleration);
    // The inherited Y-carriage frame is R_y(90°) * R_x(-90°). Use its
    // inverse for Z and express the travel origin along local -X so both the
    // Z joint axis and its origin land on world +Z.
    addPrismaticJoint(model, "z", yCarriage, zCarriage, zAxis,
      [0.5, -0.5, -0.5, 0.5],
      [-zStart, 0.0, 0.0], maxVelocity, maxAcceleration);

    return MotionSystemBlueprint.fromRobotModel(model, [
      axisBlueprint("x", xAxis, maxVelocity, maxAcceleration),
      axisBlueprint("y", yAxis, maxVelocity, maxAcceleration),
      axisBlueprint("z", zAxis, maxVelocity, maxAcceleration)
    ]);
  }

  /** Read the carriage datum through MachineAssembly's prefix-aware connector API. */
  static function machineAxisStart(axis:LinearAxis, prefix:String):Float {
    var mechanical = new AssemblyModel();
    axis.addTo(mechanical, prefix);
    var connector = axis.connector("carriageBore", prefix);
    return mechanical.initialState('machinekit-$prefix').worldConnector(connector.instanceId,
      connector.connectorName).z;
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
    var travelPerRevolutionMetres = axis.nut.travelPerRevolution() * MILLIMETRES_TO_METRES;
    var ratio = 2.0 * Math.PI / travelPerRevolutionMetres;
    var motor = new Actuator('$id.motor.${axis.motor.designation}',
      0.0, maxVelocity * Math.abs(ratio),
      Transmission.SimpleTransmission(joint.id, ratio, 0.0));
    motor.fullStepsPerRevolution = fullSteps(axis.motor);
    model.addActuator(motor);
    return joint;
  }

  /** Full steps in a turn of a stepper: its rating's step angle, or the NEMA standard 1.8 degrees. */
  static function fullSteps(motor:machinekit.motion.NemaStepper):Float {
    var rating = motor.rating();
    return rating == null ? 200.0 : 360.0 / rating.stepAngle;
  }

  static function requireAxis(axis:LinearAxis, id:String):Void {
    if (axis == null) throw 'MachineKit LinearAxis "$id" is required';
  }

  static function requireId(id:String):Void {
    if (id == null || StringTools.trim(id).length == 0)
      throw "MachineKit axis ID must be non-empty";
  }
}
