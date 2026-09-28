package robotkit.model.urdf;

import robotkit.model.CollisionShape;
import robotkit.model.Transmission;

/** A loaded URDF robot and what the loader could not carry over. */
class UrdfImport {
  public final model:RobotModel;
  /** Human-readable notes on elements that were skipped or approximated. */
  public final warnings:Array<String>;

  public function new(model:RobotModel, warnings:Array<String>) {
    this.model = model;
    this.warnings = warnings;
  }
}

/**
 * Loads an expanded URDF document into a RobotModel (robotkit/plans/HUMANOID.md,
 * HU-D2). `xacro` must be expanded beforehand.
 *
 * URDF places each joint frame at its child link's origin: `origin` is the
 * child frame in the parent, `axis` is in the joint (child) frame, and inertia
 * is given in the `inertial` frame. A root link named `world` is dropped: a
 * floating joint from it sets `floatingBase`, a fixed one leaves the robot
 * fixed. Box, cylinder and sphere collision geometry becomes collision shapes;
 * mesh references are kept as geometry references. `dynamics` damping and
 * friction become joint damping and friction loss, `mimic` a joint coupling,
 * and a simple `transmission` an actuator.
 */
class UrdfLoader {
  /** Mass given to a link without an `inertial`, which the runtime requires to be positive. */
  public static inline var PLACEHOLDER_MASS = 1e-3;

  public static function load(source:String):UrdfImport {
    var document = Xml.parse(source);
    var robot = document.firstElement();
    if (robot == null || robot.nodeName != "robot") throw "URDF root element must be <robot>";
    var name = robot.get("name");
    var model = new RobotModel(name == null || name == "" ? "urdf-robot" : name);
    model.collisionApproximation = CollisionApproximation.None;
    var warnings:Array<String> = [];

    var linkElements:Array<Xml> = [for (element in robot.elementsNamed("link")) element];
    var jointElements:Array<Xml> = [for (element in robot.elementsNamed("joint")) element];
    var childLinks = new Map<String, Bool>();
    for (joint in jointElements) childLinks.set(required(child(joint, "child"), "link"), true);

    // A `world` root only anchors the robot; it is not a body.
    var world:Null<String> = null;
    for (link in linkElements) {
      var linkName = required(link, "name");
      if (linkName == "world" && !childLinks.exists(linkName)) world = linkName;
    }

    var links = new Map<String, Link>();
    for (element in linkElements) {
      var linkName = required(element, "name");
      if (linkName == world) continue;
      if (links.exists(linkName)) throw 'Duplicate URDF link $linkName';
      var link = model.addLink(new Link(linkName, "link/" + linkName));
      links.set(linkName, link);
      readInertial(element, link, warnings);
      var visual = element.elementsNamed("visual");
      var visualCount = 0;
      for (visualElement in visual) {
        visualCount++;
        var mesh = meshFile(visualElement);
        if (mesh != null && link.visualGeometry == null) link.visualGeometry = mesh;
      }
      if (visualCount > 1)
        warnings.push('link $linkName: kept the first of $visualCount visual meshes');
      for (collision in element.elementsNamed("collision"))
        readCollision(collision, link, warnings);
    }

    var joints = new Map<String, Joint>();
    var mimics:Array<{follower:String, leader:String, ratio:Float, offset:Float}> = [];
    for (element in jointElements) {
      var jointName = required(element, "name");
      var type = required(element, "type");
      var parentName = required(child(element, "parent"), "link");
      var childName = required(child(element, "child"), "link");
      if (parentName == world) {
        switch type {
          case "floating": model.floatingBase = true;
          case "fixed":
          case _: throw 'URDF joint $jointName attaches to world with unsupported type $type';
        }
        continue;
      }
      var parent = links.get(parentName), childLink = links.get(childName);
      if (parent == null || childLink == null)
        throw 'URDF joint $jointName references an unknown link';
      var jointType = switch type {
        case "revolute": JointType.Revolute;
        case "continuous": JointType.Continuous;
        case "prismatic": JointType.Prismatic;
        case "fixed": JointType.Fixed;
        case _: throw 'URDF joint $jointName has unsupported type $type';
      };
      if (joints.exists(jointName)) throw 'Duplicate URDF joint $jointName';
      var joint = model.addJoint(new Joint(jointName, jointType, parent, childLink, "joint/" + jointName));
      joints.set(jointName, joint);
      var origin = readOrigin(element);
      joint.parentFramePosition = origin.position;
      joint.parentFrameRotation = origin.rotation;
      joint.childFramePosition = [0.0, 0.0, 0.0];
      joint.childFrameRotation = [0.0, 0.0, 0.0, 1.0];
      var axis = child(element, "axis");
      joint.axis = axis == null ? [1.0, 0.0, 0.0] : normalize(vector(axis, "xyz", 3), jointName);
      var limit = child(element, "limit");
      var limits = new JointLimits();
      if (limit != null) {
        limits.effort = Math.abs(number(limit, "effort", 0.0));
        limits.velocity = Math.abs(number(limit, "velocity", 0.0));
        if (jointType == JointType.Revolute || jointType == JointType.Prismatic) {
          limits.lower = number(limit, "lower", 0.0);
          limits.upper = number(limit, "upper", 0.0);
        }
      } else if (jointType == JointType.Revolute || jointType == JointType.Prismatic)
        throw 'URDF joint $jointName needs <limit>';
      joint.limits = limits;
      var dynamics = child(element, "dynamics");
      if (dynamics != null) {
        joint.damping = Math.abs(number(dynamics, "damping", 0.0));
        joint.frictionLoss = Math.abs(number(dynamics, "friction", 0.0));
      }
      var mimic = child(element, "mimic");
      if (mimic != null)
        mimics.push({follower: jointName, leader: required(mimic, "joint"),
          ratio: number(mimic, "multiplier", 1.0), offset: number(mimic, "offset", 0.0)});
    }
    for (mimic in mimics) {
      var leader = joints.get(mimic.leader), follower = joints.get(mimic.follower);
      if (leader == null || follower == null)
        throw 'URDF joint ${mimic.follower} mimics unknown joint ${mimic.leader}';
      model.addCoupling(new JointCoupling("coupling/" + mimic.follower, leader.id,
        follower.id, mimic.ratio, mimic.offset));
    }

    for (element in robot.elementsNamed("transmission")) {
      var jointElement = child(element, "joint");
      var actuatorElement = child(element, "actuator");
      if (jointElement == null || actuatorElement == null) {
        warnings.push('transmission ${element.get("name")}: needs one joint and one actuator');
        continue;
      }
      var joint = joints.get(required(jointElement, "name"));
      if (joint == null) throw 'URDF transmission references unknown joint ${jointElement.get("name")}';
      var reduction = 1.0;
      var reductionElement = child(actuatorElement, "mechanicalReduction");
      if (reductionElement == null) reductionElement = child(element, "mechanicalReduction");
      if (reductionElement != null) reduction = parseNumber(text(reductionElement), "mechanicalReduction");
      if (reduction == 0.0) throw 'URDF transmission for ${joint.name} has zero reduction';
      var effort = joint.limits.effort / Math.abs(reduction);
      model.addActuator(new Actuator("actuator/" + required(actuatorElement, "name"), effort, 0.0,
        SimpleTransmission(joint.id, reduction, 0.0)));
    }
    return new UrdfImport(model, warnings);
  }

  static function readInertial(element:Xml, link:Link, warnings:Array<String>):Void {
    var inertial = child(element, "inertial");
    if (inertial == null) {
      link.mass = PLACEHOLDER_MASS;
      link.inertiaTensor = [1e-6, 0.0, 0.0, 0.0, 1e-6, 0.0, 0.0, 0.0, 1e-6];
      warnings.push('link ${link.name}: no <inertial>; given a ${PLACEHOLDER_MASS} kg placeholder mass');
      return;
    }
    var massElement = child(inertial, "mass");
    if (massElement == null) throw 'URDF link ${link.name} inertial needs <mass>';
    link.mass = number(massElement, "value", 0.0);
    var origin = readOrigin(inertial);
    link.centerOfMass = origin.position;
    var inertia = child(inertial, "inertia");
    if (inertia == null) throw 'URDF link ${link.name} inertial needs <inertia>';
    var ixx = number(inertia, "ixx", 0.0), ixy = number(inertia, "ixy", 0.0),
      ixz = number(inertia, "ixz", 0.0), iyy = number(inertia, "iyy", 0.0),
      iyz = number(inertia, "iyz", 0.0), izz = number(inertia, "izz", 0.0);
    var local = [ixx, ixy, ixz, ixy, iyy, iyz, ixz, iyz, izz];
    // Inertia is given in the inertial frame; the model keeps it in link axes: R I R^T.
    var r = rotationMatrix(origin.rotation);
    var result = [for (_ in 0...9) 0.0];
    for (row in 0...3) for (column in 0...3) {
      var sum = 0.0;
      for (a in 0...3) for (b in 0...3) sum += r[row * 3 + a] * local[a * 3 + b] * r[column * 3 + b];
      result[row * 3 + column] = sum;
    }
    link.inertiaTensor = result;
  }

  static function readCollision(element:Xml, link:Link, warnings:Array<String>):Void {
    var geometry = child(element, "geometry");
    if (geometry == null) throw 'URDF link ${link.name} collision needs <geometry>';
    var shape = geometry.firstElement();
    if (shape == null) throw 'URDF link ${link.name} collision geometry is empty';
    var origin = readOrigin(element);
    var primitive:Null<CollisionPrimitive> = switch shape.nodeName {
      case "box":
        var size = vector(shape, "size", 3);
        CollisionPrimitive.Box(size[0] / 2, size[1] / 2, size[2] / 2);
      case "sphere": CollisionPrimitive.Sphere(number(shape, "radius", 0.0));
      case "cylinder":
        CollisionPrimitive.Cylinder(number(shape, "radius", 0.0), number(shape, "length", 0.0) / 2);
      case "mesh":
        if (link.collisionGeometry == null) link.collisionGeometry = required(shape, "filename");
        warnings.push('link ${link.name}: collision mesh ${shape.get("filename")} is kept as a reference, not a collision shape');
        null;
      case other: throw 'URDF link ${link.name} has unsupported collision geometry $other';
    };
    if (primitive != null)
      link.collisionShapes.push(new CollisionShape(primitive, origin.position, origin.rotation));
  }

  static function meshFile(element:Xml):Null<String> {
    var geometry = child(element, "geometry");
    var mesh = geometry == null ? null : child(geometry, "mesh");
    return mesh == null ? null : mesh.get("filename");
  }

  static function readOrigin(element:Xml):{position:Array<Float>, rotation:Array<Float>} {
    var origin = child(element, "origin");
    if (origin == null) return {position: [0.0, 0.0, 0.0], rotation: [0.0, 0.0, 0.0, 1.0]};
    var position = origin.exists("xyz") ? vector(origin, "xyz", 3) : [0.0, 0.0, 0.0];
    var rpy = origin.exists("rpy") ? vector(origin, "rpy", 3) : [0.0, 0.0, 0.0];
    return {position: position, rotation: fromRollPitchYaw(rpy[0], rpy[1], rpy[2])};
  }

  /** URDF rpy turns about the fixed X, then Y, then Z axes: q = qz * qy * qx, in x, y, z, w order. */
  static function fromRollPitchYaw(roll:Float, pitch:Float, yaw:Float):Array<Float> {
    var cr = Math.cos(roll / 2), sr = Math.sin(roll / 2);
    var cp = Math.cos(pitch / 2), sp = Math.sin(pitch / 2);
    var cy = Math.cos(yaw / 2), sy = Math.sin(yaw / 2);
    return [
      sr * cp * cy - cr * sp * sy,
      cr * sp * cy + sr * cp * sy,
      cr * cp * sy - sr * sp * cy,
      cr * cp * cy + sr * sp * sy
    ];
  }

  static function rotationMatrix(q:Array<Float>):Array<Float> {
    var x = q[0], y = q[1], z = q[2], w = q[3];
    return [
      1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
      2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
      2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)
    ];
  }

  static function normalize(values:Array<Float>, jointName:String):Array<Float> {
    var length = Math.sqrt(values[0] * values[0] + values[1] * values[1] + values[2] * values[2]);
    if (!(length > 0.0)) throw 'URDF joint $jointName has a zero axis';
    return [for (value in values) value / length];
  }

  static function child(element:Xml, name:String):Null<Xml> {
    for (found in element.elementsNamed(name)) return found;
    return null;
  }

  static function required(element:Null<Xml>, attribute:String):String {
    if (element == null) throw 'URDF element is missing where "$attribute" is required';
    var value = element.get(attribute);
    if (value == null || value == "")
      throw 'URDF <${element.nodeName}> needs attribute "$attribute"';
    return value;
  }

  static function number(element:Xml, attribute:String, fallback:Float):Float {
    var value = element.get(attribute);
    return value == null ? fallback : parseNumber(value, attribute);
  }

  static function vector(element:Xml, attribute:String, length:Int):Array<Float> {
    var parts = [for (part in StringTools.trim(required(element, attribute)).split(" ")) if (part != "") part];
    if (parts.length != length)
      throw 'URDF attribute "$attribute" needs $length numbers';
    return [for (part in parts) parseNumber(part, attribute)];
  }

  static function text(element:Xml):String {
    var result = "";
    for (node in element) result += node.nodeValue;
    return StringTools.trim(result);
  }

  static function parseNumber(text:String, what:String):Float {
    var value = Std.parseFloat(StringTools.trim(text));
    if (!Math.isFinite(value)) throw 'URDF value "$text" for $what is not a number';
    return value;
  }
}
