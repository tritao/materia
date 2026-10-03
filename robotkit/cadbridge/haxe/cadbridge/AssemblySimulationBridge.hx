package cadbridge;

import materia.assembly.AssemblyBodies;
import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinition.AssemblyJointRole;
import materia.assembly.AssemblyDefinition.AssemblyJointType;
import materia.assembly.AssemblyDefinition.AssemblyStateRecord;
import materia.assembly.AssemblyDefinitionCodec;
import materia.assembly.AssemblyDefinitionFlattener;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import robotkit.model.RobotModel;
import robotkit.model.Link;
import robotkit.model.Joint;
import robotkit.model.JointType;
import robotkit.model.JointLimits;
import robotkit.model.JointCoupling;
import robotkit.model.Transmission;
import robotkit.model.Actuator;
import robotkit.model.ActuatorDrive;
import robotkit.model.Encoder;
import robotkit.model.EncoderKind;
import robotkit.model.TorqueSpeedCurve;
import robotkit.model.RobotDriveConfiguration;
import robotkit.model.RobotMobileConfiguration;
import materia.project.SceneArtifact.SceneArtifactMobileBase;
import cadkit.modeling.AssemblyState;

/**
 * An assembly as one simulated robot. Parts that fixed joints hold together form one rigid body and
 * share a link; parts fixed to the world share the root link. Lengths are metres.
 */
typedef AssemblySimulationModel = {
  var model:RobotModel;
  /** Each part's convex hull, in its link's frame, for Simulation.addRobotAtPose. */
  var linkHulls:Array<AssemblyLinkHull>;
  /** Each simulated part's link and its frame within that link. */
  var partLinks:Map<String, AssemblyPartLink>;
  var closureIds:Array<String>;
  var closures:Array<AssemblySimulationClosure>;
}

/** One part's collision hull: XYZ triples in the frame of link `link`. */
typedef AssemblyLinkHull = {
  var link:Int;
  var part:String;
  var vertices:Array<Float>;
}

/** Where a part rides: link `link`, at `offset` (metres) in the link's frame. */
typedef AssemblyPartLink = {
  var link:Int;
  var offset:AssemblyFrame;
}

typedef AssemblySimulationClosure = {
  var id:String;
  /** The links the closure joins. */
  var parent:String;
  var child:String;
  var type:AssemblyJointType;
  var anchorParent:Array<Float>;
  var axisParent:Array<Float>;
}

typedef AssemblyPhysicalPart = {
  var id:String;
  var materialId:String;
  var volume:Float;
  var centerOfMass:Array<Float>;
  var inertia:Array<Float>;
  var density:Float;
  /** Bounded convex support hull, flattened XYZ triples in CAD units. */
  @:optional var collisionHull:Array<Float>;
  @:optional var collisionWarning:String;
  @:optional var collisionErrorRatio:Float;
}

typedef AssemblyPhysicalData = {
  var metresPerUnit:Float;
  var parts:Array<AssemblyPhysicalPart>;
}

/** Converts an assembly's tree joints and physical parts to a RobotKit model. */
class AssemblySimulationBridge {
  /**
   * End-stop room past a joint's limits when its assembly states no overtravel: 1 mm for a slide,
   * 1 degree for a rotary joint. A joint parked on its limit reads noise either side of it, and a
   * runtime that faulted exactly at the limit would stop the robot before any command.
   */
  public static inline final DEFAULT_PRISMATIC_OVERTRAVEL = 0.001;
  public static final DEFAULT_ROTARY_OVERTRAVEL = Math.PI / 180;

  /**
   * `freeOccurrences` are parts the simulation holds or moves on its own instead of bolting them to
   * the assembly: a workpiece, or a mobile robot's surroundings; they get no link, and no joint may
   * touch them.
   *
   * Each rigid body (`AssemblyBodies`) becomes one link, framed at its root part, with the combined
   * mass properties of its parts and one collision hull per part; bodies that no joint carries join
   * the root link. Only moving joints remain joints, so a machine of many bolted parts simulates as
   * a few links and its real axes. `massOf` may override a part's mass in kilograms (a part given
   * another material); its inertia scales with it.
   *
   * With `mobileBase` the assembly is a wheeled robot: its root link is the chassis that the drive
   * rolls over the floor, framed at the robot's `origin` on the floor (the assembly origin without
   * one), and the named wheel joints drive it.
   */
  public static function toRobotModel(definition:AssemblyDefinition,
      artifact:AssemblyPhysicalData, ?savedState:AssemblyStateRecord,
      ?freeOccurrences:Array<String>, ?massOf:String->Null<Float>,
      ?mobileBase:SceneArtifactMobileBase):AssemblySimulationModel {
    var free = new Map<String, Bool>();
    if (freeOccurrences != null) for (id in freeOccurrences) free.set(id, true);
    AssemblyDefinitionCodec.validate(definition);
    var sourceDefinition = definition;
    definition = AssemblyDefinitionFlattener.flatten(definition);
    if (artifact == null || artifact.parts == null || artifact.metresPerUnit <= 0)
      throw "Assembly simulation needs a valid scene artifact";
    var scale = artifact.metresPerUnit;
    var parts = new Map<String, AssemblyPhysicalPart>();
    for (part in artifact.parts) parts.set(part.id, part);
    var definitions = new Map<String, materia.assembly.AssemblyDefinition.AssemblyComponentDefinition>();
    for (component in definition.definitions) definitions.set(component.id, component);
    for (edge in definition.joints) if (free.exists(edge.parent) || free.exists(edge.child))
      throw 'Free part is joined by "${edge.id}"; a part the simulation holds or moves on its own cannot be joined';
    var placement = new AssemblyState(sourceDefinition, savedState);
    var rootFrame = mobileBase == null || mobileBase.origin == null ? AssemblyFrames.identity() :
      {x: mobileBase.origin.x / scale, y: mobileBase.origin.y / scale, z: 0.0, qx: 0.0, qy: 0.0,
        qz: Math.sin(mobileBase.origin.yaw / 2), qw: Math.cos(mobileBase.origin.yaw / 2)};
    var model = new RobotModel(definition.id);
    var root = model.addLink(new Link("assembly-root"));
    var links = [root];
    // Mass, centre of mass (link frame, metres) and inertia (about that centre, link axes) of each
    // part, per link, combined once every part is placed.
    var masses:Array<Array<{mass:Float, center:Array<Float>, inertia:Array<Float>}>> = [[
      {mass: 0.001, center: [0.0, 0.0, 0.0], inertia: [1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6]}]];
    var partLinks = new Map<String, AssemblyPartLink>();
    var offsets = new Map<String, AssemblyFrame>();
    var linkIndexOfPart = new Map<String, Int>();
    var linkHulls:Array<AssemblyLinkHull> = [];
    for (body in AssemblyBodies.of(definition)) {
      if (free.exists(body.id)) continue;
      var index = 0;
      var frame = rootFrame;
      if (body.joint != null) {
        index = links.length;
        links.push(model.addLink(new Link(body.id)));
        masses.push([]);
        frame = placement.worldPose(body.id);
      }
      var toLink = AssemblyFrames.inverse(frame);
      for (id in body.occurrences) {
        var occurrence = [for (item in definition.occurrences) if (item.id == id) item][0];
        var part = parts.get(occurrence.definition);
        if (part == null) throw 'Assembly occurrence "$id" has no physical part';
        if (!Math.isFinite(part.volume) || part.volume <= 0 || !Math.isFinite(part.density) ||
            part.density <= 0 || part.centerOfMass == null || part.centerOfMass.length != 3 ||
            part.inertia == null || part.inertia.length != 9)
          throw 'Assembly occurrence "$id" has invalid mass properties';
        for (coordinate in part.centerOfMass) if (!Math.isFinite(coordinate))
          throw 'Assembly occurrence "$id" has a non-finite centre of mass';
        for (component in part.inertia) if (!Math.isFinite(component))
          throw 'Assembly occurrence "$id" has a non-finite inertia';
        // The part's frame in its link's frame; fixed joints keep it constant.
        var offset = AssemblyFrames.compose(toLink, placement.worldPose(id));
        offsets.set(id, offset);
        linkIndexOfPart.set(id, index);
        partLinks.set(id, {link: index, offset: scaled(offset, scale)});
        var center = AssemblyFrames.transformPoint(offset, part.centerOfMass[0], part.centerOfMass[1],
          part.centerOfMass[2]);
        var baseMass = part.volume * part.density * scale * scale * scale;
        var chosen = massOf == null ? null : massOf(id);
        var mass = chosen == null ? baseMass : chosen;
        if (!Math.isFinite(mass) || mass <= 0) throw 'Assembly occurrence "$id" has an invalid mass';
        masses[index].push({mass: mass, center: [center.x * scale, center.y * scale, center.z * scale],
          inertia: rotated(part.inertia, offset, part.density * Math.pow(scale, 5) * mass / baseMass)});
        var hull = part.collisionHull;
        if (hull != null) {
          if (hull.length < 12 || hull.length > 64 * 3 || hull.length % 3 != 0)
            throw 'Assembly occurrence "$id" has an invalid collision hull';
          var vertices:Array<Float> = [];
          for (vertex in 0...Std.int(hull.length / 3)) {
            var x = hull[vertex * 3], y = hull[vertex * 3 + 1], z = hull[vertex * 3 + 2];
            if (!Math.isFinite(x) || !Math.isFinite(y) || !Math.isFinite(z))
              throw 'Assembly occurrence "$id" has a non-finite collision hull';
            var point = AssemblyFrames.transformPoint(offset, x, y, z);
            vertices.push(point.x * scale);
            vertices.push(point.y * scale);
            vertices.push(point.z * scale);
          }
          linkHulls.push({link: index, part: id, vertices: vertices});
        }
      }
    }
    for (index in 0...links.length) combine(links[index], masses[index]);
    function linkOf(id:String):Link {
      var index = linkIndexOfPart.get(id);
      if (index == null) throw 'Assembly occurrence "$id" has no simulated link';
      return links[index];
    }
    function offsetOf(id:String):AssemblyFrame {
      var offset = offsets.get(id);
      if (offset == null) throw 'Assembly occurrence "$id" has no simulated link';
      return offset;
    }
    var closures:Array<String> = [];
    var closureGeometry:Array<AssemblySimulationClosure> = [];
    for (edge in definition.joints) {
      if (edge.role == AssemblyJointRole.Closure) {
        closures.push(edge.id);
        var frame = AssemblyFrames.compose(offsetOf(edge.parent),
          connector(definitions.get(occurrenceDefinition(definition, edge.parent)), edge.parentConnector));
        var endpoint = AssemblyFrames.transformPoint(frame, edge.axis.x, edge.axis.y, edge.axis.z);
        closureGeometry.push({id: edge.id, parent: linkOf(edge.parent).id,
          child: linkOf(edge.child).id, type: edge.type,
          anchorParent: [frame.x * scale, frame.y * scale, frame.z * scale],
          axisParent: [endpoint.x - frame.x, endpoint.y - frame.y, endpoint.z - frame.z]});
        continue;
      }
      // Fixed joints hold parts inside one body; they are not joints of the robot.
      if (edge.type == AssemblyJointType.Fixed) continue;
      var kind:JointType = switch (edge.type) {
        case AssemblyJointType.Revolute: JointType.Revolute;
        case AssemblyJointType.Continuous: JointType.Continuous;
        case AssemblyJointType.Prismatic: JointType.Prismatic;
        default: throw 'Unsupported assembly joint type "${edge.type}"';
      };
      var joint = model.addJoint(new Joint(edge.id, kind, linkOf(edge.parent),
        linkOf(edge.child)));
      var parentFrame = AssemblyFrames.compose(offsetOf(edge.parent),
        connector(definitions.get(occurrenceDefinition(definition, edge.parent)), edge.parentConnector));
      var initial = placement.joint(edge.id);
      setFrame(joint, AssemblyFrames.compose(parentFrame,
        AssemblyFrames.axisMotion(edge.type, edge.axis, initial)), true, scale);
      setFrame(joint, AssemblyFrames.compose(offsetOf(edge.child),
        connector(definitions.get(occurrenceDefinition(definition, edge.child)), edge.childConnector)), false, scale);
      joint.axis = [edge.axis.x, edge.axis.y, edge.axis.z];
      var factor = edge.type == AssemblyJointType.Prismatic ? scale : 1.0;
      joint.limits = new JointLimits(edge.limits.lower == null ? -1e9 : (edge.limits.lower - initial) * factor,
        edge.limits.upper == null ? 1e9 : (edge.limits.upper - initial) * factor,
        edge.limits.velocity == null ? null : edge.limits.velocity * factor,
        edge.limits.effort);
      if (edge.limits.assumptions != null) joint.limits.assumptions = [for (value in edge.limits.assumptions)
        {quantity: value.quantity, label: value.label}];
      if (edge.limits.acceleration != null) joint.limits.maxAcceleration = edge.limits.acceleration * factor;
      joint.limits.overtravel = edge.limits.overtravel != null ? edge.limits.overtravel * factor :
        edge.type == AssemblyJointType.Prismatic ? DEFAULT_PRISMATIC_OVERTRAVEL : DEFAULT_ROTARY_OVERTRAVEL;
    }
    // A follower summing several couplings' terms has its placement taken off once, in its first.
    var offsetTargets = new Map<String, Bool>();
    if (definition.couplings != null) for (coupling in definition.couplings) {
      var leader:Null<materia.assembly.AssemblyDefinition.KinematicJoint> = null;
      var follower:Null<materia.assembly.AssemblyDefinition.KinematicJoint> = null;
      for (edge in definition.joints) {
        if (edge.id == coupling.source) leader = edge;
        if (edge.id == coupling.target) follower = edge;
      }
      if (leader == null || follower == null || leader.role == AssemblyJointRole.Closure ||
          follower.role == AssemblyJointRole.Closure)
        throw 'Assembly coupling "${coupling.id}" requires tree joints';
      var leaderScale = leader.type == AssemblyJointType.Prismatic ? scale : 1.0;
      var followerScale = follower.type == AssemblyJointType.Prismatic ? scale : 1.0;
      var ratio = coupling.ratio * followerScale / leaderScale;
      var placed = offsetTargets.exists(coupling.target) ? 0.0 : placement.joint(coupling.target);
      offsetTargets.set(coupling.target, true);
      var offset = (coupling.ratio * placement.joint(coupling.source) + coupling.offset - placed) * followerScale;
      var added = model.addCoupling(new JointCoupling(coupling.id, coupling.source,
        coupling.target, ratio, offset));
      if (coupling.efficiency != null) added.efficiency = coupling.efficiency;
      // Stiffness and backlash are in the leader's units, which a sliding leader measures in assembly units.
      var stiffness = coupling.stiffness, backlash = coupling.backlash, drag = coupling.drag;
      if (stiffness != null) added.stiffness = stiffness / leaderScale;
      if (backlash != null) added.backlash = backlash * leaderScale;
      if (drag != null) added.drag = drag * followerScale;
      if (coupling.assumptions != null) added.assumptions = [for (value in coupling.assumptions)
        {quantity: value.quantity, label: value.label}];
      if (coupling.assumed != null) added.assumed = [for (label in coupling.assumed) label];
    }
    // A motor on a joint: its effort and rate in robot units, and its rotor turning with the joint.
    if (definition.actuators != null) for (actuator in definition.actuators) {
      var driven:Null<Joint> = null;
      for (joint in model.joints) if (joint.id == actuator.joint) driven = joint;
      var edge:Null<materia.assembly.AssemblyDefinition.KinematicJoint> = null;
      for (candidate in definition.joints) if (candidate.id == actuator.joint) edge = candidate;
      if (driven == null || edge == null) throw 'Assembly actuator "${actuator.id}" drives no simulated joint';
      // The actuator works in assembly units from the joint's zero; the robot joint is in SI
      // units from its initial placement: joint = (actuator - initial) * factor.
      var factor = edge.type == AssemblyJointType.Prismatic ? scale : 1.0;
      var initial = placement.joint(actuator.joint);
      // A gearbox turns the motor `gearRatio` times for one unit of the joint: the actuator coordinate is that much more.
      var gear = actuator.gearRatio == null ? 1.0 : actuator.gearRatio;
      var added = new Actuator(actuator.id, actuator.maxEffort, actuator.maxRate,
        Transmission.SimpleTransmission(driven.id, gear / factor, -initial * factor));
      if (actuator.gearEfficiency != null) added.efficiency = actuator.gearEfficiency;
      if (actuator.assumed != null) added.assumed = [for (label in actuator.assumed) label];
      added.microsteps = actuator.microsteps;
      added.maxStepRate = actuator.maxStepRate;
      var steps = actuator.fullStepsPerRevolution;
      if (steps != null) added.fullStepsPerRevolution = steps;
      var inertia = actuator.rotorInertia == null ? 0.0 : actuator.rotorInertia;
      // The drive's speeds are in the actuator's own units, like `maxRate`.
      var curve = actuator.torqueSpeed == null ? null : TorqueSpeedCurve.unflatten(actuator.torqueSpeed);
      var holding = actuator.holdingTorque;
      var rated = actuator.ratedTorque, peak = actuator.peakTorque, ratedSpeed = actuator.ratedSpeed, topSpeed = actuator.maxSpeed;
      if (actuator.drive == "stepper" && steps != null && curve != null && holding != null)
        added.drive = new StepperDrive(steps, inertia, holding, curve);
      else if (actuator.drive == "servo" && rated != null && peak != null && ratedSpeed != null && topSpeed != null)
        added.drive = new ServoDrive(rated, peak, ratedSpeed, topSpeed, inertia,
          actuator.encoderCounts == null ? 0.0 : actuator.encoderCounts, curve);
      if (actuator.servoStiffness != null) added.servoStiffness = actuator.servoStiffness;
      if (actuator.servoDamping != null) added.servoDamping = actuator.servoDamping;
      // The encoder that reads the motor is its own sensor; a servo that names one does not also hold a count.
      if (actuator.encoder != null) added.encoder = actuator.encoder;
      if (actuator.assumptions != null) added.assumptions = [for (value in actuator.assumptions)
        {quantity: value.quantity, label: value.label}];
      model.addActuator(added);
      if (actuator.rotorInertia != null)
        // The rotor turns `gear` times as fast as the joint, so its inertia at the joint is `gear` squared times as much.
        driven.armature += edge.type == AssemblyJointType.Prismatic ? 0.0 : actuator.rotorInertia * gear * gear;
    }
    model.materializeLimits();
    // Encoders: sensors on joints. Counts per millimetre on a sliding joint, per revolution on a turning one.
    if (definition.encoders != null) for (encoder in definition.encoders) {
      var edge:Null<materia.assembly.AssemblyDefinition.KinematicJoint> = null;
      for (candidate in definition.joints) if (candidate.id == encoder.joint) edge = candidate;
      var onJoint:Null<Joint> = null;
      for (joint in model.joints) if (joint.id == encoder.joint) onJoint = joint;
      if (edge == null || onJoint == null) throw 'Assembly encoder "${encoder.id}" reads no simulated joint';
      var kind:EncoderKind = encoder.kind;
      var index = encoder.index == true;
      model.addEncoder(edge.type == AssemblyJointType.Prismatic
        ? Encoder.perMillimetre(encoder.id, onJoint.id, kind, encoder.counts, index)
        : Encoder.perRevolution(encoder.id, onJoint.id, kind, encoder.counts, index));
    }
    if (mobileBase != null) {
      for (wheel in [mobileBase.leftWheel, mobileBase.rightWheel])
        if ([for (joint in model.joints) if (joint.id == wheel) joint].length != 1)
          throw 'Mobile base wheel "$wheel" is not a moving joint of the assembly';
      model.mobileBase = new RobotMobileConfiguration(
        RobotDriveConfiguration.Differential(mobileBase.leftWheel, mobileBase.rightWheel,
          mobileBase.wheelRadius, mobileBase.trackWidth),
        mobileBase.maxLinearSpeed, mobileBase.maxAngularSpeed,
        mobileBase.maxLinearAcceleration, mobileBase.maxAngularAcceleration,
        mobileBase.footprintLength, mobileBase.footprintWidth);
    }
    return {model: model, linkHulls: linkHulls, partLinks: partLinks,
      closureIds: closures, closures: closureGeometry};
  }

  /** A frame with its translation scaled to metres. */
  static function scaled(frame:AssemblyFrame, scale:Float):AssemblyFrame
    return {x: frame.x * scale, y: frame.y * scale, z: frame.z * scale, qx: frame.qx, qy: frame.qy,
      qz: frame.qz, qw: frame.qw};

  /** A part's unit-density inertia, scaled by `factor` and turned into its link's axes. */
  static function rotated(inertia:Array<Float>, offset:AssemblyFrame, factor:Float):Array<Float> {
    var r = AssemblyFrames.toRotationMatrix(offset);
    var result = [for (_ in 0...9) 0.0];
    for (i in 0...3) for (j in 0...3) {
      var sum = 0.0;
      for (k in 0...3) for (l in 0...3) sum += r[i * 3 + k] * inertia[k * 3 + l] * r[j * 3 + l];
      result[i * 3 + j] = sum * factor;
    }
    return result;
  }

  /** Sets a link's mass properties from its parts', with the parallel-axis theorem. */
  static function combine(link:Link, pieces:Array<{mass:Float, center:Array<Float>, inertia:Array<Float>}>):Void {
    var mass = 0.0, center = [0.0, 0.0, 0.0];
    for (piece in pieces) {
      mass += piece.mass;
      for (axis in 0...3) center[axis] += piece.mass * piece.center[axis];
    }
    for (axis in 0...3) center[axis] /= mass;
    var inertia = [for (_ in 0...9) 0.0];
    for (piece in pieces) {
      var d = [for (axis in 0...3) piece.center[axis] - center[axis]];
      var squared = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
      for (i in 0...3) for (j in 0...3)
        inertia[i * 3 + j] += piece.inertia[i * 3 + j] + piece.mass * ((i == j ? squared : 0.0) - d[i] * d[j]);
    }
    link.mass = mass;
    link.centerOfMass = center;
    link.inertiaTensor = inertia;
  }

  static function occurrenceDefinition(definition:AssemblyDefinition, id:String):String {
    for (occurrence in definition.occurrences) if (occurrence.id == id) return occurrence.definition;
    throw 'Unknown assembly occurrence "$id"';
  }

  static function connector(component:materia.assembly.AssemblyDefinition.AssemblyComponentDefinition,
      name:String):AssemblyFrame {
    if (component == null) throw "Missing assembly component definition";
    for (item in component.connectors) if (item.name == name) return item.frame;
    throw 'Missing assembly connector "$name"';
  }

  static function setFrame(joint:Joint, frame:AssemblyFrame, parent:Bool, scale:Float):Void {
    var position = [frame.x * scale, frame.y * scale, frame.z * scale];
    var rotation = [frame.qx, frame.qy, frame.qz, frame.qw];
    if (parent) {
      joint.parentFramePosition = position;
      joint.parentFrameRotation = rotation;
    } else {
      joint.childFramePosition = position;
      joint.childFrameRotation = rotation;
    }
  }
}
