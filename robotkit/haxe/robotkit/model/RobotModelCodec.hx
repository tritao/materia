package robotkit.model;

import haxe.Json;
import haxe.io.Bytes;
import robotkit.model.ActuatorDrive;
import robotkit.model.CollisionShape;
import robotkit.model.Transmission;

/** Canonical, versioned JSON artifact for an editable RobotModel. */
class RobotModelCodec {
  public static inline final VERSION:Int = RobotModel.CURRENT_VERSION;

  public static function encode(model:RobotModel):Bytes {
    if (model == null) throw "RobotModel is required";
    requireText(model.name, "name");
    collisionName(model.collisionApproximation);
    var links = new Map<String, Bool>();
    for (link in model.links) {
      requireText(link.id, "link id");
      requireText(link.name, "link name");
      if (links.exists(link.id)) throw 'Duplicate robot link ${link.id}';
      links.set(link.id, true);
      vector(link.centerOfMass, 3, "link centerOfMass");
      vector(link.inertiaTensor, 9, "link inertiaTensor");
      finite(link.mass, "link mass");
      if (link.collisionShapes == null) throw 'Link ${link.id} has no collision shape list';
      for (shape in link.collisionShapes) {
        var error = shape == null ? "collision shape is null" : shape.validate();
        if (error != null) throw 'Link ${link.id}: $error';
      }
    }
    var frames = new Map<String, Bool>();
    for (frame in model.frames) {
      requireText(frame.id, "frame id");
      if (!links.exists(frame.link.id)) throw 'Frame ${frame.id} references an unknown link';
      if (frames.exists(frame.id)) throw 'Duplicate robot frame ${frame.id}';
      frames.set(frame.id, true);
      requireText(frame.name, "frame name");
      vector(frame.position, 3, "frame position");
      vector(frame.rotation, 4, "frame rotation");
    }
    var linkShapes = new Map<String, Int>();
    for (link in model.links) linkShapes.set(link.id, link.collisionShapes.length);
    for (pair in model.contactPairs) {
      if (pair == null) throw "Robot contact pair is null";
      if (!linkShapes.exists(pair.linkA) || !linkShapes.exists(pair.linkB) || pair.linkA == pair.linkB ||
          pair.shapeA < 0 || pair.shapeA >= linkShapes.get(pair.linkA) ||
          pair.shapeB < 0 || pair.shapeB >= linkShapes.get(pair.linkB))
        throw 'Contact pair ${pair.linkA}/${pair.shapeA} - ${pair.linkB}/${pair.shapeB} references no shape on two links';
      var error = pair.surface == null ? "contact pair surface is null" : pair.surface.validate();
      if (error != null) throw error;
    }
    var joints = new Map<String, Bool>();
    for (joint in model.joints) {
      requireText(joint.id, "joint id");
      requireText(joint.name, "joint name");
      if (joints.exists(joint.id)) throw 'Duplicate robot joint ${joint.id}';
      if (!links.exists(joint.parent.id) || !links.exists(joint.child.id))
        throw 'Joint ${joint.id} references an unknown link';
      joints.set(joint.id, true);
      vector(joint.parentFramePosition, 3, "joint parentFramePosition");
      vector(joint.parentFrameRotation, 4, "joint parentFrameRotation");
      vector(joint.childFramePosition, 3, "joint childFramePosition");
      vector(joint.childFrameRotation, 4, "joint childFrameRotation");
      vector(joint.axis, 3, "joint axis");
      nonNegative(joint.armature, "joint armature");
      nonNegative(joint.damping, "joint damping");
      nonNegative(joint.frictionLoss, "joint frictionLoss");
      nonNegative(joint.limitTimeConstant, "joint limitTimeConstant");
      nonNegative(joint.limitDampingRatio, "joint limitDampingRatio");
      vector(joint.limitImpedance, 5, "joint limitImpedance");
      finite(joint.limits.lower, "joint limits.lower");
      finite(joint.limits.upper, "joint limits.upper");
      finite(joint.limits.velocity, "joint limits.velocity");
      finite(joint.limits.effort, "joint limits.effort");
      finite(joint.limits.maxAcceleration, "joint limits.maxAcceleration");
      if (joint.limits.maxAcceleration < 0.0)
        throw "joint limits.maxAcceleration must be non-negative";
      finite(joint.limits.overtravel, "joint limits.overtravel");
      if (joint.limits.overtravel < 0.0)
        throw "joint limits.overtravel must be non-negative";
      jointTypeName(joint.type);
    }
    var actuators = new Map<String, Bool>();
    for (actuator in model.actuators) {
      validateActuator(actuator, joints);
      if (actuators.exists(actuator.id)) throw 'Duplicate robot actuator ${actuator.id}';
      actuators.set(actuator.id, true);
    }
    var encoderIds = new Map<String, Bool>();
    for (encoder in model.encoders) {
      if (encoder == null) throw "Robot encoder is null";
      requireText(encoder.id, "encoder ID");
      if (encoderIds.exists(encoder.id)) throw 'Duplicate robot encoder ${encoder.id}';
      encoderIds.set(encoder.id, true);
      if (!joints.exists(encoder.joint)) throw 'Encoder ${encoder.id} references unknown joint ${encoder.joint}';
      finite(encoder.countsPerUnit, "encoder countsPerUnit");
    }
    for (actuator in model.actuators)
      if (actuator.encoder != "" && !encoderIds.exists(actuator.encoder))
        throw 'Actuator ${actuator.id} references unknown encoder ${actuator.encoder}';
    var couplingIds = new Map<String, Bool>();
    var followerIds = new Map<String, Bool>();
    for (coupling in model.couplings) {
      if (coupling == null) throw "Robot joint coupling is null";
      if (couplingIds.exists(coupling.id)) throw 'Duplicate robot coupling ${coupling.id}';
      if (!joints.exists(coupling.leader) || !joints.exists(coupling.follower))
        throw 'Coupling ${coupling.id} references an unknown joint';
      if (followerIds.exists(coupling.follower))
        throw 'Joint ${coupling.follower} has multiple coupling leaders';
      if (!(coupling.efficiency > 0 && coupling.efficiency <= 1))
        throw 'Coupling ${coupling.id} has an invalid efficiency';
      couplingIds.set(coupling.id, true);
      followerIds.set(coupling.follower, true);
    }
    var sensors = new Map<String, Bool>();
    for (sensor in model.sensors) {
      requireText(sensor.id, "sensor id");
      if (sensors.exists(sensor.id)) throw 'Duplicate robot sensor ${sensor.id}';
      sensors.set(sensor.id, true);
      requireText(sensor.name, "sensor name");
      requireText(sensor.kind, "sensor kind");
      if (sensor.frame != null && !frames.exists(sensor.frame.id))
        throw 'Sensor ${sensor.id} references an unknown frame';
      finite(sensor.updateRate, "sensor updateRate");
      finite(sensor.maxRange, "sensor maxRange");
      finite(sensor.startAngleRadians, "sensor startAngleRadians");
      finite(sensor.fieldOfViewRadians, "sensor fieldOfViewRadians");
      finite(sensor.noiseStddev, "sensor noiseStddev");
    }
    if (model.mobileBase != null) validateMobile(model.mobileBase, joints);
    if (model.forkMechanism != null) validateFork(model.forkMechanism, joints);
    var mobile = model.mobileBase == null ? null : encodeMobile(model.mobileBase);
    var fork = model.forkMechanism == null ? null : encodeFork(model.forkMechanism);
    var document:Dynamic = {
      schemaVersion: VERSION,
      name: model.name,
      collisionApproximation: collisionName(model.collisionApproximation),
      floatingBase: model.floatingBase,
      links: [for (link in model.links) {
        id: link.id, name: link.name, mass: link.mass,
        centerOfMass: link.centerOfMass, inertiaTensor: link.inertiaTensor,
        visualGeometry: link.visualGeometry, collisionGeometry: link.collisionGeometry,
        collisionShapes: [for (shape in link.collisionShapes) encodeCollisionShape(shape)]
      }],
      joints: [for (joint in model.joints) {
        id: joint.id, name: joint.name, type: jointTypeName(joint.type),
        parentLink: joint.parent.id, childLink: joint.child.id,
        limits: encodeLimits(joint.limits),
        parentFramePosition: joint.parentFramePosition,
        parentFrameRotation: joint.parentFrameRotation,
        childFramePosition: joint.childFramePosition,
        childFrameRotation: joint.childFrameRotation,
        axis: joint.axis,
        dynamics: {armature: joint.armature, damping: joint.damping, frictionLoss: joint.frictionLoss,
          limitTimeConstant: joint.limitTimeConstant, limitDampingRatio: joint.limitDampingRatio,
          limitImpedance: joint.limitImpedance}
      }],
      actuators: [for (actuator in model.actuators) encodeActuator(actuator)],
      couplings: [for (coupling in model.couplings) encodeCoupling(coupling)],
      frames: [for (frame in model.frames) {
        id: frame.id, name: frame.name, link: frame.link.id,
        position: frame.position, rotation: frame.rotation
      }],
      sensors: [for (sensor in model.sensors) {
        id: sensor.id, name: sensor.name, kind: sensor.kind,
        updateRate: sensor.updateRate,
        frame: sensor.frame == null ? null : sensor.frame.id,
        rayCount: sensor.rayCount, maxRange: sensor.maxRange,
        startAngleRadians: sensor.startAngleRadians,
        fieldOfViewRadians: sensor.fieldOfViewRadians,
        noiseStddev: sensor.noiseStddev, noiseSeed: sensor.noiseSeed
      }],
      mobileBase: mobile,
      forkMechanism: fork,
      contactPairs: [for (pair in model.contactPairs) {
        linkA: pair.linkA, shapeA: pair.shapeA, linkB: pair.linkB, shapeB: pair.shapeB,
        surface: encodeSurface(pair.surface)
      }]
    };
    // Written only when there are some, so models without encoders keep their bytes.
    if (model.encoders.length > 0) document.encoders = [for (encoder in model.encoders) encodeEncoder(encoder)];
    return Bytes.ofString(Json.stringify(document));
  }

  static function encodeEncoder(encoder:Encoder):Dynamic {
    var record:Dynamic = {id: encoder.id, joint: encoder.joint, kind: encoder.kind, countsPerUnit: encoder.countsPerUnit};
    if (encoder.index) record.index = true;
    return record;
  }

  static function readEncoder(record:Dynamic):Encoder {
    var kind = text(record, "kind");
    if (kind != EncoderKind.Incremental && kind != EncoderKind.Absolute) throw 'Unsupported encoder kind $kind';
    var counts = number(record, "countsPerUnit");
    if (!(counts > 0.0)) throw "Encoder countsPerUnit must be positive";
    return new Encoder(text(record, "id"), text(record, "joint"), kind, counts,
      Reflect.hasField(record, "index") && Reflect.field(record, "index") == true);
  }

  public static function decode(bytes:Bytes):RobotModel {
    var root:Dynamic;
    try root = Json.parse(bytes.toString()) catch (_:Dynamic)
      throw "Malformed RobotModel artifact";
    var version = fieldInt(root, "schemaVersion");
    if (version != VERSION) throw 'Unsupported RobotModel schema version $version; expected $VERSION';

    var model = new RobotModel(text(root, "name"));
    model.collisionApproximation = readCollision(text(root, "collisionApproximation"));
    model.floatingBase = bool(root, "floatingBase");
    var links = new Map<String, Link>();
    for (record in array(root, "links")) {
      var id = text(record, "id");
      if (links.exists(id)) throw 'Duplicate robot link $id';
      var link = model.addLink(new Link(text(record, "name"), id));
      link.mass = number(record, "mass");
      link.centerOfMass = vectorField(record, "centerOfMass", 3);
      link.inertiaTensor = vectorField(record, "inertiaTensor", 9);
      link.visualGeometry = optionalText(record, "visualGeometry");
      link.collisionGeometry = optionalText(record, "collisionGeometry");
      for (shape in array(record, "collisionShapes")) {
        var decoded = readCollisionShape(shape);
        var error = decoded.validate();
        if (error != null) throw 'Link $id: $error';
        link.collisionShapes.push(decoded);
      }
      links.set(id, link);
    }

    var joints = new Map<String, Bool>();
    for (record in array(root, "joints")) {
      if (Reflect.hasField(record, "drive"))
        throw "RobotModel v5 does not accept joint.drive; use root actuators";
      var id = text(record, "id");
      if (joints.exists(id)) throw 'Duplicate robot joint $id';
      joints.set(id, true);
      var parentId = text(record, "parentLink");
      var childId = text(record, "childLink");
      var parent = links.get(parentId), child = links.get(childId);
      if (parent == null || child == null) throw 'Joint $id references an unknown link';
      var joint = model.addJoint(new Joint(text(record, "name"),
        readJointType(text(record, "type")), parent, child, id));
      var limits:Dynamic = required(record, "limits");
      var maxAcceleration = number(limits, "maxAcceleration");
      joint.limits = new JointLimits(number(limits, "lower"), number(limits, "upper"),
        number(limits, "velocity"), number(limits, "effort"), maxAcceleration);
      // Written only when a joint has some, so older models without it still load.
      if (Reflect.hasField(limits, "overtravel")) joint.limits.overtravel = number(limits, "overtravel");
      joint.parentFramePosition = vectorField(record, "parentFramePosition", 3);
      joint.parentFrameRotation = vectorField(record, "parentFrameRotation", 4);
      joint.childFramePosition = vectorField(record, "childFramePosition", 3);
      joint.childFrameRotation = vectorField(record, "childFrameRotation", 4);
      joint.axis = vectorField(record, "axis", 3);
      var dynamics:Dynamic = required(record, "dynamics");
      joint.armature = nonNegative(number(dynamics, "armature"), "joint armature");
      joint.damping = nonNegative(number(dynamics, "damping"), "joint damping");
      joint.frictionLoss = nonNegative(number(dynamics, "frictionLoss"), "joint frictionLoss");
      joint.limitTimeConstant = nonNegative(number(dynamics, "limitTimeConstant"), "joint limitTimeConstant");
      joint.limitDampingRatio = nonNegative(number(dynamics, "limitDampingRatio"), "joint limitDampingRatio");
      joint.limitImpedance = vectorField(dynamics, "limitImpedance", 5);
    }

    var couplingIds = new Map<String, Bool>();
    var followers = new Map<String, Bool>();
    for (record in array(root, "couplings")) {
      var coupling = new JointCoupling(text(record, "id"), text(record, "leader"),
        text(record, "follower"), number(record, "ratio"), number(record, "offset"));
      var efficiency = optionalNumber(record, "efficiency");
      if (efficiency != null) {
        if (!(efficiency > 0 && efficiency <= 1)) throw 'Coupling ${coupling.id} has an invalid efficiency';
        coupling.efficiency = efficiency;
      }
      if (Reflect.hasField(record, "stiffness")) coupling.stiffness = nonNegative(number(record, "stiffness"), "coupling stiffness");
      if (Reflect.hasField(record, "backlash")) coupling.backlash = nonNegative(number(record, "backlash"), "coupling backlash");
      if (Reflect.hasField(record, "drag")) coupling.drag = nonNegative(number(record, "drag"), "coupling drag");
      if (!joints.exists(coupling.leader) || !joints.exists(coupling.follower))
        throw 'Coupling ${coupling.id} references an unknown joint';
      if (couplingIds.exists(coupling.id) || followers.exists(coupling.follower))
        throw 'Duplicate robot coupling ${coupling.id} or follower';
      couplingIds.set(coupling.id, true);
      followers.set(coupling.follower, true);
      model.addCoupling(coupling);
    }

    var actuatorIds = new Map<String, Bool>();
    for (record in array(root, "actuators")) {
      var actuator = readActuator(record);
      if (actuatorIds.exists(actuator.id)) throw 'Duplicate robot actuator ${actuator.id}';
      actuatorIds.set(actuator.id, true);
      switch actuator.transmission {
        case SimpleTransmission(jointId, _, _):
          if (!joints.exists(jointId))
            throw 'Actuator ${actuator.id} references unknown joint $jointId';
      }
      model.addActuator(actuator);
    }
    if (Reflect.hasField(root, "encoders")) {
      var encoderIds = new Map<String, Bool>();
      for (record in array(root, "encoders")) {
        var encoder = readEncoder(record);
        if (encoderIds.exists(encoder.id)) throw 'Duplicate robot encoder ${encoder.id}';
        encoderIds.set(encoder.id, true);
        if (!joints.exists(encoder.joint)) throw 'Encoder ${encoder.id} references unknown joint ${encoder.joint}';
        model.addEncoder(encoder);
      }
      for (actuator in model.actuators)
        if (actuator.encoder != "" && !encoderIds.exists(actuator.encoder))
          throw 'Actuator ${actuator.id} references unknown encoder ${actuator.encoder}';
    }

    var frames = new Map<String, Frame>();
    for (record in array(root, "frames")) {
      var id = text(record, "id");
      if (frames.exists(id)) throw 'Duplicate robot frame $id';
      var linkId = text(record, "link"), link = links.get(linkId);
      if (link == null) throw 'Frame $id references an unknown link';
      var frame = model.addFrame(new Frame(text(record, "name"), link, id));
      frame.position = vectorField(record, "position", 3);
      frame.rotation = vectorField(record, "rotation", 4);
      frames.set(id, frame);
    }

    var sensors = new Map<String, Bool>();
    for (record in array(root, "sensors")) {
      var id = text(record, "id");
      if (sensors.exists(id)) throw 'Duplicate robot sensor $id';
      sensors.set(id, true);
      var sensor = model.addSensor(new Sensor(text(record, "name"), text(record, "kind"),
        number(record, "updateRate"), id));
      var frameId = optionalText(record, "frame");
      if (frameId != null) {
        sensor.frame = frames.get(frameId);
        if (sensor.frame == null) throw 'Sensor $id references an unknown frame';
      }
      sensor.rayCount = fieldInt(record, "rayCount");
      sensor.maxRange = number(record, "maxRange");
      sensor.startAngleRadians = number(record, "startAngleRadians");
      sensor.fieldOfViewRadians = number(record, "fieldOfViewRadians");
      sensor.noiseStddev = number(record, "noiseStddev");
      sensor.noiseSeed = fieldInt(record, "noiseSeed");
    }

    var mobile:Dynamic = required(root, "mobileBase");
    if (mobile != null) model.mobileBase = readMobile(mobile);
    for (record in array(root, "contactPairs")) {
      var linkA = text(record, "linkA"), linkB = text(record, "linkB");
      var shapeA = fieldInt(record, "shapeA"), shapeB = fieldInt(record, "shapeB");
      var a = links.get(linkA), b = links.get(linkB);
      if (a == null || b == null || linkA == linkB || shapeA < 0 || shapeA >= a.collisionShapes.length ||
          shapeB < 0 || shapeB >= b.collisionShapes.length)
        throw 'Contact pair $linkA/$shapeA - $linkB/$shapeB references no shape on two links';
      var surface = readSurface(required(record, "surface"));
      var error = surface.validate();
      if (error != null) throw error;
      model.contactPairs.push(new ContactPair(linkA, shapeA, linkB, shapeB, surface));
    }
    var fork:Dynamic = required(root, "forkMechanism");
    if (fork != null) model.forkMechanism = readFork(fork);
    return model;
  }

  public static function encodeCollisionShape(shape:CollisionShape):Dynamic {
    var kind:String, size:Array<Float>;
    switch shape.primitive {
      case Box(x, y, z): kind = "box"; size = [x, y, z];
      case Sphere(radius): kind = "sphere"; size = [radius];
      case Capsule(radius, half): kind = "capsule"; size = [radius, half];
      case Cylinder(radius, half): kind = "cylinder"; size = [radius, half];
    }
    var surface = shape.surface;
    return {kind: kind, size: size, position: shape.position, rotation: shape.rotation,
      surface: surface == null ? null : encodeSurface(surface),
      contact: switch shape.contact {
        case Layers: "layers";
        case PairsOnly: "pairs";
        case PairsAndEnvironment: "pairs-and-environment";
      }};
  }

  static function encodeSurface(surface:ContactSurface):Dynamic return {
    friction: surface.friction, frictionDimensions: surface.frictionDimensions,
    contactTimeConstant: surface.contactTimeConstant,
    contactDampingRatio: surface.contactDampingRatio
  };

  static function readSurface(record:Dynamic):ContactSurface {
    return new ContactSurface(vectorField(record, "friction", 3),
      fieldInt(record, "frictionDimensions"), number(record, "contactTimeConstant"),
      number(record, "contactDampingRatio"));
  }

  public static function readCollisionShape(record:Dynamic):CollisionShape {
    var primitive = switch text(record, "kind") {
      case "box":
        var size = vectorField(record, "size", 3);
        Box(size[0], size[1], size[2]);
      case "sphere": Sphere(vectorField(record, "size", 1)[0]);
      case "capsule":
        var size = vectorField(record, "size", 2);
        Capsule(size[0], size[1]);
      case "cylinder":
        var size = vectorField(record, "size", 2);
        Cylinder(size[0], size[1]);
      case other: throw 'Unsupported RobotModel collision shape kind $other';
    };
    var shape = new CollisionShape(primitive, vectorField(record, "position", 3),
      vectorField(record, "rotation", 4));
    var surface:Dynamic = required(record, "surface");
    if (surface != null) shape.surface = readSurface(surface);
    shape.contact = switch text(record, "contact") {
      case "layers": Layers;
      case "pairs": PairsOnly;
      case "pairs-and-environment": PairsAndEnvironment;
      case other: throw 'Unsupported RobotModel shape contact $other';
    };
    return shape;
  }

  /** Stiffness, backlash and drag are written only when a coupling has some, so other models keep their bytes. */
  static function encodeCoupling(coupling:JointCoupling):Dynamic {
    var record:Dynamic = {
      id: coupling.id, leader: coupling.leader, follower: coupling.follower,
      ratio: coupling.ratio, offset: coupling.offset, efficiency: coupling.efficiency
    };
    if (coupling.stiffness != 0.0) record.stiffness = nonNegative(coupling.stiffness, "coupling stiffness");
    if (coupling.backlash != 0.0) record.backlash = nonNegative(coupling.backlash, "coupling backlash");
    if (coupling.drag != 0.0) record.drag = nonNegative(coupling.drag, "coupling drag");
    return record;
  }

  static function encodeActuator(value:Actuator):Dynamic {
    var record:Dynamic = {
      id: value.id, maxEffort: value.maxEffort, maxRate: value.maxRate,
      servoStiffness: value.servoStiffness, servoDamping: value.servoDamping,
      fullStepsPerRevolution: value.fullStepsPerRevolution,
      transmission: switch value.transmission {
        case SimpleTransmission(jointId, ratio, offset):
          {kind: "simple", jointId: jointId, ratio: ratio, offset: offset};
      }
    };
    if (value.encoder != "") record.encoder = value.encoder;
    if (value.efficiency != 1.0) record.efficiency = value.efficiency;
    // A bare stepper is its steps alone, as before drive kinds; anything with ratings gets a drive.
    var drive = value.drive;
    if (drive != null) {
      if (Std.isOfType(drive, StepperDrive)) {
        var stepper:StepperDrive = cast drive;
        if (stepper.hasTorqueData()) record.drive = {kind: "stepper", fullStepsPerRevolution: stepper.fullStepsPerRevolution,
          rotorInertia: stepper.rotorInertia, holdingTorque: stepper.holdingTorque, curve: stepper.curve.flatten()};
      } else if (Std.isOfType(drive, ServoDrive)) {
        var servo:ServoDrive = cast drive;
        record.drive = {kind: "servo", ratedTorque: servo.ratedTorque, peakTorque: servo.peakTorqueValue,
          ratedSpeed: servo.ratedSpeed, maxSpeed: servo.maxSpeedValue, rotorInertia: servo.rotorInertia,
          encoderCounts: servo.encoderCounts, curve: servo.curve.flatten()};
      } else throw 'Actuator ${value.id} has an unsupported drive';
    }
    return record;
  }

  static function readActuatorDrive(record:Dynamic):ActuatorDrive {
    var curveValues:Dynamic = required(record, "curve");
    if (!Std.isOfType(curveValues, Array)) throw "Invalid RobotModel field curve";
    var curve = TorqueSpeedCurve.unflatten([for (entry in (cast curveValues : Array<Dynamic>)) {
      if (!Std.isOfType(entry, Int) && !Std.isOfType(entry, Float)) throw "Invalid RobotModel field curve";
      finite(entry, "curve");
    }]);
    return switch text(record, "kind") {
      case "stepper": new StepperDrive(number(record, "fullStepsPerRevolution"), nonNegative(number(record, "rotorInertia"), "drive rotorInertia"),
        nonNegative(number(record, "holdingTorque"), "drive holdingTorque"), curve);
      case "servo": new ServoDrive(number(record, "ratedTorque"), number(record, "peakTorque"), number(record, "ratedSpeed"),
        number(record, "maxSpeed"), nonNegative(number(record, "rotorInertia"), "drive rotorInertia"),
        nonNegative(number(record, "encoderCounts"), "drive encoderCounts"), curve);
      case kind: throw 'Unsupported actuator drive kind $kind';
    };
  }

  static function validateActuator(value:Actuator, joints:Map<String, Bool>):Void {
    if (value == null) throw "Robot actuator is null";
    nonNegative(value.servoStiffness, "actuator servoStiffness");
    nonNegative(value.servoDamping, "actuator servoDamping");
    if (!(value.efficiency > 0.0 && value.efficiency <= 1.0)) throw "Actuator efficiency must be in (0, 1]";
    nonNegative(value.fullStepsPerRevolution, "actuator fullStepsPerRevolution");
    if (value.drive != null && !Std.isOfType(value.drive, StepperDrive) && !Std.isOfType(value.drive, ServoDrive))
      throw 'Actuator ${value.id} has an unsupported drive';
    requireText(value.id, "actuator ID");
    finite(value.maxEffort, "actuator maxEffort");
    finite(value.maxRate, "actuator maxRate");
    if (value.maxEffort < 0.0 || value.maxRate < 0.0)
      throw "Actuator limits must be non-negative";
    if (value.transmission == null) throw "Actuator transmission is required";
    switch value.transmission {
      case SimpleTransmission(jointId, ratio, offset):
        if (!joints.exists(jointId)) throw 'Actuator ${value.id} references unknown joint $jointId';
        finite(ratio, "transmission ratio");
        finite(offset, "transmission offset");
        if (ratio == 0.0) throw "Transmission ratio must be nonzero";
    }
  }

  static function validateMobile(value:RobotMobileConfiguration,
      joints:Map<String, Bool>):Void {
    switch value.drive {
      case Differential(left, right, radius, track):
        requireJointReference(left, joints, "left wheel");
        requireJointReference(right, joints, "right wheel");
        finite(radius, "mobileBase wheelRadius");
        finite(track, "mobileBase trackWidth");
      case Ackermann(steering, drive, wheelBase, radius, angle):
        requireJointReference(steering, joints, "steering");
        requireJointReference(drive, joints, "drive wheel");
        finite(wheelBase, "mobileBase wheelBase");
        finite(radius, "mobileBase wheelRadius");
        finite(angle, "mobileBase maxSteeringAngle");
      case _: throw "Unsupported RobotModel drive configuration";
    }
    finite(value.maxLinearSpeed, "mobileBase maxLinearSpeed");
    finite(value.maxAngularSpeed, "mobileBase maxAngularSpeed");
    finite(value.maxLinearAcceleration, "mobileBase maxLinearAcceleration");
    finite(value.maxAngularAcceleration, "mobileBase maxAngularAcceleration");
    if (value.footprintLength != null) finite(value.footprintLength, "mobileBase footprintLength");
    if (value.footprintWidth != null) finite(value.footprintWidth, "mobileBase footprintWidth");
  }

  static function validateFork(value:RobotForkConfiguration,
      joints:Map<String, Bool>):Void {
    requireJointReference(value.liftJointId, joints, "fork lift");
    if (value.tiltJointId != null) requireJointReference(value.tiltJointId, joints, "fork tilt");
    if (value.spreadJointId != null) requireJointReference(value.spreadJointId, joints, "fork spread");
    finite(value.maxMassKg, "forkMechanism maxMassKg");
    finite(value.maxLoadMomentKgMeters, "forkMechanism maxLoadMomentKgMeters");
    finite(value.maxLiftHeightMeters, "forkMechanism maxLiftHeightMeters");
  }

  static function requireJointReference(id:String, joints:Map<String, Bool>, role:String):Void {
    requireText(id, '$role joint ID');
    if (!joints.exists(id)) throw 'RobotModel $role role references unknown joint $id';
  }

  static function readActuator(value:Dynamic):Actuator {
    var transmission = required(value, "transmission");
    var parsed:Transmission = switch text(transmission, "kind") {
      case "simple": SimpleTransmission(text(transmission, "jointId"),
        number(transmission, "ratio"), number(transmission, "offset"));
      case kind: throw 'Unsupported transmission kind $kind';
    };
    var actuator = new Actuator(text(value, "id"), number(value, "maxEffort"),
      number(value, "maxRate"), parsed);
    actuator.servoStiffness = nonNegative(number(value, "servoStiffness"), "actuator servoStiffness");
    actuator.servoDamping = nonNegative(number(value, "servoDamping"), "actuator servoDamping");
    // Absent in models saved before steppers were recorded: not a stepper.
    if (Reflect.hasField(value, "fullStepsPerRevolution"))
      actuator.fullStepsPerRevolution = nonNegative(number(value, "fullStepsPerRevolution"), "actuator fullStepsPerRevolution");
    // Models saved before drive kinds have no `drive`; a stepper's steps stand alone.
    if (Reflect.hasField(value, "drive") && Reflect.field(value, "drive") != null)
      actuator.drive = readActuatorDrive(Reflect.field(value, "drive"));
    if (Reflect.hasField(value, "encoder") && Reflect.field(value, "encoder") != null)
      actuator.encoder = text(value, "encoder");
    if (Reflect.hasField(value, "efficiency") && Reflect.field(value, "efficiency") != null) {
      actuator.efficiency = number(value, "efficiency");
      if (!(actuator.efficiency > 0.0 && actuator.efficiency <= 1.0)) throw "Actuator efficiency must be in (0, 1]";
    }
    return actuator;
  }

  static function encodeMobile(value:RobotMobileConfiguration):Dynamic return {
    drive: encodeDrive(value.drive),
    maxLinearSpeed: value.maxLinearSpeed, maxAngularSpeed: value.maxAngularSpeed,
    maxLinearAcceleration: value.maxLinearAcceleration,
    maxAngularAcceleration: value.maxAngularAcceleration,
    footprintLength: value.footprintLength, footprintWidth: value.footprintWidth
  };

  static function encodeDrive(value:RobotDriveConfiguration):Dynamic return switch value {
    case Differential(left, right, radius, track):
      {kind: "differential", leftWheelJoint: left, rightWheelJoint: right,
        wheelRadius: radius, trackWidth: track};
    case Ackermann(steering, drive, wheelBase, radius, maxAngle):
      {kind: "ackermann", steeringJoint: steering, driveWheelJoint: drive,
        wheelBase: wheelBase, wheelRadius: radius, maxSteeringAngle: maxAngle};
    case _: throw "Unsupported RobotModel drive configuration";
  };

  static function readMobile(value:Dynamic):RobotMobileConfiguration {
    return new RobotMobileConfiguration(readDrive(required(value, "drive")),
      number(value, "maxLinearSpeed"), number(value, "maxAngularSpeed"),
      number(value, "maxLinearAcceleration"), number(value, "maxAngularAcceleration"),
      optionalNumber(value, "footprintLength"), optionalNumber(value, "footprintWidth"));
  }

  static function readDrive(value:Dynamic):RobotDriveConfiguration return switch text(value, "kind") {
    case "differential": RobotDriveConfiguration.Differential(text(value, "leftWheelJoint"),
      text(value, "rightWheelJoint"), number(value, "wheelRadius"), number(value, "trackWidth"));
    case "ackermann": RobotDriveConfiguration.Ackermann(text(value, "steeringJoint"),
      text(value, "driveWheelJoint"), number(value, "wheelBase"), number(value, "wheelRadius"),
      number(value, "maxSteeringAngle"));
    case kind: throw 'Unsupported RobotModel drive configuration $kind';
  };

  static function encodeFork(value:RobotForkConfiguration):Dynamic return {
    liftJoint: value.liftJointId, tiltJoint: value.tiltJointId, spreadJoint: value.spreadJointId,
    maxMassKg: value.maxMassKg, maxLoadMomentKgMeters: value.maxLoadMomentKgMeters,
    maxLiftHeightMeters: value.maxLiftHeightMeters
  };

  static function readFork(value:Dynamic):RobotForkConfiguration {
    return new RobotForkConfiguration(text(value, "liftJoint"),
      number(value, "maxMassKg"), number(value, "maxLoadMomentKgMeters"),
      number(value, "maxLiftHeightMeters"), optionalText(value, "tiltJoint"),
      optionalText(value, "spreadJoint"));
  }

  static function collisionName(value:CollisionApproximation):String return switch value {
    case CollisionApproximation.None: "none";
    case CollisionApproximation.BoundsBox: "bounds-box";
    case _: throw "Unsupported RobotModel collision approximation";
  };

  static function readCollision(value:String):CollisionApproximation return switch value {
    case "none": CollisionApproximation.None;
    case "bounds-box": CollisionApproximation.BoundsBox;
    case _: throw 'Unsupported RobotModel collision approximation $value';
  };

  static function jointTypeName(value:JointType):String return switch value {
    case JointType.Fixed: "fixed";
    case JointType.Revolute: "revolute";
    case JointType.Continuous: "continuous";
    case JointType.Prismatic: "prismatic";
    case JointType.Floating: "floating";
    case _: throw "Unsupported RobotModel joint type";
  };

  static function readJointType(value:String):JointType return switch value {
    case "fixed": JointType.Fixed;
    case "revolute": JointType.Revolute;
    case "continuous": JointType.Continuous;
    case "prismatic": JointType.Prismatic;
    case "floating": JointType.Floating;
    case _: throw 'Unsupported RobotModel joint type $value';
  };

  static function required(value:Dynamic, name:String):Dynamic {
    if (value == null || !Reflect.hasField(value, name)) throw 'Missing RobotModel field $name';
    return Reflect.field(value, name);
  }

  static function text(value:Dynamic, name:String):String {
    var result:Dynamic = required(value, name);
    if (!Std.isOfType(result, String)) throw 'Invalid RobotModel field $name';
    requireText(result, name);
    return result;
  }

  /** Overtravel is written only when a joint has some, so models without it keep their bytes. */
  static function encodeLimits(limits:JointLimits):Dynamic {
    var result:Dynamic = {lower: limits.lower, upper: limits.upper, velocity: limits.velocity,
      effort: limits.effort, maxAcceleration: limits.maxAcceleration};
    if (limits.overtravel > 0.0) Reflect.setField(result, "overtravel", limits.overtravel);
    return result;
  }

  static function optionalText(value:Dynamic, name:String):Null<String> {
    var result:Dynamic = required(value, name);
    if (result == null) return null;
    if (!Std.isOfType(result, String)) throw 'Invalid RobotModel field $name';
    return result;
  }

  static function number(value:Dynamic, name:String):Float {
    var result:Dynamic = required(value, name);
    if (!Std.isOfType(result, Int) && !Std.isOfType(result, Float))
      throw 'Invalid RobotModel field $name';
    return finite(result, name);
  }

  static function optionalNumber(value:Dynamic, name:String):Null<Float> {
    var result:Dynamic = required(value, name);
    if (result == null) return null;
    if (!Std.isOfType(result, Int) && !Std.isOfType(result, Float))
      throw 'Invalid RobotModel field $name';
    return finite(result, name);
  }

  static function nonNegative(value:Float, name:String):Float {
    if (!Math.isFinite(value) || value < 0.0) throw 'RobotModel field $name must be finite and non-negative';
    return value;
  }

  static function bool(value:Dynamic, name:String):Bool {
    var result:Dynamic = required(value, name);
    if (!Std.isOfType(result, Bool)) throw 'Invalid RobotModel field $name';
    return result;
  }

  static function fieldInt(value:Dynamic, name:String):Int {
    var result:Dynamic = required(value, name);
    if (!Std.isOfType(result, Int)) throw 'Invalid RobotModel field $name';
    return result;
  }

  static function array(value:Dynamic, name:String):Array<Dynamic> {
    var result:Dynamic = required(value, name);
    if (!Std.isOfType(result, Array)) throw 'Invalid RobotModel field $name';
    return cast result;
  }

  static function vectorField(value:Dynamic, name:String, length:Int):Array<Float> {
    var values:Dynamic = required(value, name);
    if (!Std.isOfType(values, Array)) throw 'Invalid RobotModel field $name';
    var floatValues:Null<Array<Float>> = null;
    try floatValues = cast values catch (_:Dynamic) {}
    if (floatValues != null) {
      if (floatValues.length != length)
        throw 'RobotModel field $name must contain $length values';
      return [for (item in floatValues) finite(item, name)];
    }
    var intValues:Null<Array<Int>> = null;
    try intValues = cast values catch (_:Dynamic) {}
    if (intValues != null) {
      if (intValues.length != length)
        throw 'RobotModel field $name must contain $length values';
      return [for (item in intValues) item];
    }
    var dynamicValues = array(value, name);
    if (dynamicValues.length != length)
      throw 'RobotModel field $name must contain $length values';
    return [for (item in dynamicValues) {
      if (!Std.isOfType(item, Int) && !Std.isOfType(item, Float))
        throw 'Invalid RobotModel field $name';
      finite(item, name);
    }];
  }

  static function vector(values:Array<Float>, length:Int, name:String):Void {
    if (values == null || values.length != length) throw 'RobotModel $name must contain $length values';
    for (value in values) finite(value, name);
  }

  static function finite(value:Float, name:String):Float {
    if (!Math.isFinite(value)) throw 'RobotModel field $name must be finite';
    return value;
  }

  static function requireText(value:String, name:String):Void {
    if (value == null || StringTools.trim(value).length == 0)
      throw 'RobotModel field $name must not be empty';
  }
}
