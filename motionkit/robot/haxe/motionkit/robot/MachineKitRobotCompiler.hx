package motionkit.robot;

import cadkit.modeling.AssemblyModel;
import cadbridge.AssemblySimulationBridge;
import machinekit.assembly.MachineAssembly;
import materia.assembly.AssemblyFrames;
import robotkit.model.ActuatorDrive.StepperDrive;
import robotkit.model.TorqueSpeedCurve;
import machinekit.assembly.LinearAxis;
import motionkit.axis.MotionAxisBlueprint;
import robotkit.model.Actuator;
import robotkit.model.Joint;
import robotkit.model.JointType;
import robotkit.model.RobotModel;
import robotkit.model.Transmission;

/** Stable joint IDs from a physical MachineKit axis assembly. */
private typedef AssemblyAxisBinding = {
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

  /** Compile the authored gantry, including its drives, through the app's bridge.
   * Logical ceilings restrict a job; they do not replace physical drive ratings. */
  public static function compileGantry(gantry:machinekit.gantry.Gantry,
      maxVelocity:Float = DEFAULT_MAX_VELOCITY,
      maxAcceleration:Float = DEFAULT_MAX_ACCELERATION):MotionSystemBlueprint {
    if (gantry == null) throw "A physical gantry assembly is required";
    if (!Math.isFinite(maxVelocity) || maxVelocity <= 0 ||
        !Math.isFinite(maxAcceleration) || maxAcceleration <= 0)
      throw "Gantry job ceilings must be finite and positive";
    var model = physicalAssemblyModel(gantry);
    var ids = [for (axis in gantry.axes) axis.id];
    // Preserve the logical XYZ order while retaining every physical shaft joint.
    var order = [for (joint in model.joints) joint.id];
    model.joints.sort((a, b) -> {
      var first = ids.indexOf(a.id), second = ids.indexOf(b.id);
      if (first < 0) first = ids.length + order.indexOf(a.id);
      if (second < 0) second = ids.length + order.indexOf(b.id);
      return first - second;
    });
    var axes:Array<MotionAxisBlueprint> = [];
    for (axis in gantry.axes) {
      var joints = [axis.id];
      for (_ in 0...model.joints.length) for (coupling in model.couplings) {
        var inputs = 0;
        for (candidate in model.couplings) if (candidate.follower == coupling.follower) inputs++;
        if (inputs == 1 && joints.indexOf(coupling.leader) >= 0 && joints.indexOf(coupling.follower) < 0)
          joints.push(coupling.follower);
      }
      var limits = model.coupledLimits(axis.id);
      axes.push(new MotionAxisBlueprint(axis.id, joints,
        axis.lower * MILLIMETRES_TO_METRES, axis.upper * MILLIMETRES_TO_METRES,
        Math.min(maxVelocity, limits.requireVelocity()),
        Math.min(maxAcceleration, limits.requireAcceleration())));
    }
    return MotionSystemBlueprint.fromRobotModel(model, axes);
  }

  /** Physical mass/inertia and saved actuator metadata enter one bridge invocation. */
  static function physicalAssemblyModel(assembly:MachineAssembly):RobotModel {
    var mechanical = new AssemblyModel();
    assembly.addTo(mechanical, "");
    var definition = mechanical.definition("machinekit-machine");
    machinekit.assembly.AssemblyPreview.preserveOwnership(definition, assembly.describe().mechanical);
    var components = assembly.components();
    var parts:Array<cadbridge.AssemblySimulationBridge.AssemblyPhysicalPart> = [];
    for (entry in definition.definitions) {
      var occurrence = [for (candidate in definition.occurrences) if (candidate.definition == entry.id) candidate][0];
      var component = [for (candidate in components) if (candidate.id == occurrence.id) candidate.component][0];
      var mass = component.massProperties();
      var inertia = mass.inertia;
      if (inertia == null) throw 'Machine member "${occurrence.id}" needs inertia';
      parts.push({id: entry.id, materialId: component.materialSpec(), density: 1.0,
        volume: mass.mass * 1e9,
        centerOfMass: [mass.centreOfMass.x, mass.centreOfMass.y, mass.centreOfMass.z],
        inertia: [inertia.xx * 1e9, inertia.xy * 1e9, inertia.xz * 1e9,
          inertia.xy * 1e9, inertia.yy * 1e9, inertia.yz * 1e9,
          inertia.xz * 1e9, inertia.yz * 1e9, inertia.zz * 1e9]});
    }
    return AssemblySimulationBridge.toRobotModel(definition,
      {metresPerUnit: MILLIMETRES_TO_METRES, parts: parts}).model;
  }

  /** Attach MachineKit drives to the part-level assembly model. The motor
   * drives its shaft joint; the existing lead-screw JointCoupling relates
   * shaft rotation to carriage travel. No second joint topology is built. */
  static function attachLinearAxisDrives(model:RobotModel,
      bindings:Array<AssemblyAxisBinding>,
      ?maxVelocity:Float = DEFAULT_MAX_VELOCITY,
      ?maxAcceleration:Float = DEFAULT_MAX_ACCELERATION):MotionSystemBlueprint {
    if (model == null || bindings == null || bindings.length == 0)
      throw "Assembly axes need a physical robot model and bindings";
    if (!Math.isFinite(maxVelocity) || maxVelocity <= 0 ||
        !Math.isFinite(maxAcceleration) || maxAcceleration <= 0)
      throw "Assembly axis limits must be finite and positive";
    var axes:Array<MotionAxisBlueprint> = [];
    var ratios:Array<Float> = [];
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
      var expected = assemblyScrewRatio(binding.axis);
      var matched = false;
      for (coupling in model.couplings)
        if (coupling.leader == travel.id && coupling.follower == shaft.id &&
            Math.abs(coupling.ratio - expected) <= Math.abs(expected) * 1e-8) {
          matched = true;
          ratios.push(coupling.ratio);
        }
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
      var ratio = ratios[index];
      // This convenience compiler states a 24 V, rated-current drive. The motor part owns its curve.
      var source = binding.axis.motor.actuator('${binding.id}.motor.${binding.axis.motor.designation}', shaftId, 24, 0.5);
      var motor = new Actuator(source.id, source.maxEffort,
        source.maxRate,
        Transmission.SimpleTransmission(shaftId, 1.0, 0.0));
      var steps = source.fullStepsPerRevolution, inertia = source.rotorInertia, holding = source.holdingTorque, curve = source.torqueSpeed;
      if (steps == null || inertia == null || holding == null || curve == null)
        throw "Axis motor must supply stepper ratings and a torque-speed curve";
      motor.drive = new StepperDrive(steps, inertia, holding, TorqueSpeedCurve.unflatten(curve));
      var assumptions = source.assumed;
      if (assumptions != null) motor.assumed = [for (label in assumptions) label];
      if (source.assumptions != null) motor.assumptions = [for (value in source.assumptions)
        {quantity: value.quantity, label: value.label}];
      robotkit.model.EngineeringAssumptions.add(motor.assumptions, "motor curve", "convenience compiler 24 V drive");
      robotkit.model.EngineeringAssumptions.add(motor.assumptions, "speed limit", "convenience compiler 24 V drive");
      motor.assumed.push("convenience compiler 24 V drive");
      motor.microsteps = 16;
      motor.maxStepRate = machinekit.motion.MotorDriver.catalog().get("GENERIC-DM542").maximumStepRate;
      motor.assumed.push("motor driver ratings");
      model.addActuator(motor);
      for (joint in model.joints) {
        if (joint.id == shaftId) joint.armature += inertia;
      }
    }
    model.materializeLimits();
    axes = [for (binding in bindings) {
      var limits = [for (joint in model.joints) if (joint.id == binding.travelJointId) joint.limits][0];
      new MotionAxisBlueprint(binding.id, [binding.travelJointId, binding.shaftJointId],
        0.0, binding.axis.stroke * MILLIMETRES_TO_METRES,
        limits.velocity == null ? maxVelocity : Math.min(maxVelocity, limits.velocity),
        limits.maxAcceleration == null ? maxAcceleration : Math.min(maxAcceleration, limits.maxAcceleration));
    }];
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
    requireAxis(axis, axisId);
    requireId(axisId);
    var assembly = new MachineAssembly();
    assembly.include(axisId, axis);
    return compilePhysicalAxes(assembly, [axisId], [axis], maxVelocity, maxAcceleration);
  }

  /** Compiles the same part-level assembly used by the complete motion blueprint. */
  public static function compileLinearAxisModel(axis:LinearAxis, axisId:String,
      ?maxVelocity:Float = DEFAULT_MAX_VELOCITY,
      ?maxAcceleration:Float = DEFAULT_MAX_ACCELERATION):RobotModel
    return compileLinearAxis(axis, axisId, maxVelocity, maxAcceleration).model;

  /** Compile actual members and joints; their resolved transmissions are the only screw ratios. */
  static function compilePhysicalAxes(assembly:MachineAssembly, ids:Array<String>, axes:Array<LinearAxis>,
      maxVelocity:Float, maxAcceleration:Float):MotionSystemBlueprint {
    // LinearAxis's grounded preview parts belong to its motor body when it is a moving stage.
    for (index in 0...ids.length) {
      var id = ids[index];
      var flat = axes[index].definition();
      var motorPose = [for (entry in flat.occurrences) if (entry.id == "motor") entry.initialPose][0];
      var children = [for (joint in flat.joints) if (joint.role == materia.assembly.AssemblyDefinition.AssemblyJointRole.Tree) joint.child];
      for (entry in flat.occurrences) if (entry.id != "motor" && children.indexOf(entry.id) < 0) {
        var connector = "root/" + entry.id;
        assembly.addMemberConnector(id + "/motor", connector,
          AssemblyFrames.compose(AssemblyFrames.inverse(motorPose), entry.initialPose));
        assembly.addMemberConnector(id + "/" + entry.id, "stageRoot", AssemblyFrames.identity());
        assembly.addMate(id + "/ground/" + entry.id, "fixed", id + "/motor", connector,
          id + "/" + entry.id, "stageRoot");
      }
    }
    var mechanical = new AssemblyModel();
    assembly.addTo(mechanical, "");
    var definition = mechanical.definition("machinekit-axes");
    var parts:Array<cadbridge.AssemblySimulationBridge.AssemblyPhysicalPart> = [];
    var components = assembly.components();
    for (entry in definition.definitions) {
      var occurrence = [for (candidate in definition.occurrences) if (candidate.definition == entry.id) candidate][0];
      var component = [for (candidate in components) if (candidate.id == occurrence.id) candidate.component][0];
      var mass = component.massProperties();
      var inertia = mass.inertia;
      if (inertia == null) throw 'Axis member "${occurrence.id}" needs inertia';
      // Express known kg and kg mm² as unit-density mm³ and mm⁵ for the bridge.
      parts.push({id: entry.id, materialId: component.materialSpec(), density: 1.0,
        volume: mass.mass * 1e9,
        centerOfMass: [mass.centreOfMass.x, mass.centreOfMass.y, mass.centreOfMass.z],
        inertia: [inertia.xx * 1e9, inertia.xy * 1e9, inertia.xz * 1e9,
          inertia.xy * 1e9, inertia.yy * 1e9, inertia.yz * 1e9,
          inertia.xz * 1e9, inertia.yz * 1e9, inertia.zz * 1e9]});
    }
    var physical = AssemblySimulationBridge.toRobotModel(definition, {metresPerUnit: MILLIMETRES_TO_METRES, parts: parts});
    // Put logical travels first for deterministic XYZ indexing; retain every physical motor joint.
    physical.model.joints.sort((a, b) -> {
      var first = ids.indexOf(a.id.split("/")[0]), second = ids.indexOf(b.id.split("/")[0]);
      if (a.type != JointType.Prismatic) first += ids.length;
      if (b.type != JointType.Prismatic) second += ids.length;
      return first == second ? Reflect.compare(a.id, b.id) : first - second;
    });
    var bindings:Array<AssemblyAxisBinding> = [];
    for (index in 0...ids.length) {
      var id = ids[index];
      var motor = physical.partLinks.get(id + "/motor");
      if (motor == null) throw 'Axis "$id" has no motor body';
      bindings.push({id: id, axis: axes[index], motorLinkId: physical.model.links[motor.link].id,
        shaftJointId: id + "/coupling", travelJointId: id + "/carriage-slide"});
    }
    return attachLinearAxisDrives(physical.model, bindings, maxVelocity, maxAcceleration);
  }

  /** Read the axis's resolved coupling, then convert its mm leader to SI. */
  static function assemblyScrewRatio(axis:LinearAxis):Float {
    var couplings = axis.definition().couplings;
    if (couplings != null) for (coupling in couplings)
      if (coupling.id == "lead-screw") return coupling.ratio / MILLIMETRES_TO_METRES;
    throw "Linear axis has no resolved lead-screw coupling";
  }

  static function requireAxis(axis:LinearAxis, id:String):Void {
    if (axis == null) throw 'MachineKit LinearAxis "$id" is required';
  }

  static function requireId(id:String):Void {
    if (id == null || StringTools.trim(id).length == 0)
      throw "MachineKit axis ID must be non-empty";
  }
}
