package robotkit.collision;

import collisionkit.CollisionDescription;
import collisionkit.CollisionGeometry;
import collisionkit.CollisionPairRule;
import collisionkit.CollisionPairStatus;
import collisionkit.CollisionPose;
import collisionkit.kinematics.ModelBodies;
import kinematicskit.KinematicModel;
import kinematicskit.KinematicState;
import kinematicskit.Transform;
import robotkit.kinematics.RobotKinematics;
import robotkit.manipulation.ClearanceBodyData;
import robotkit.model.CollisionShape;
import robotkit.model.RobotModel;
import robotkit.spatial.Transform3;
import robotkit.tool.Tool;

/** How `RobotCollision.describe` places and groups a robot. */
class RobotCollisionOptions {
  public final name:String;
  /** Pair-class groups (CL-D4) of the links and of the tool. */
  public var linkGroup:Int = 0;
  public var toolGroup:Int = 1;
  /** A robot bolted down: its root link, and what is rigid to it, is fixed in the cell. */
  public var fixedBase:Bool = true;
  /** Where the robot's root sits in the cell (identity when null). */
  public var rootPose:Null<Transform3> = null;
  /** The reference configuration, as DOF values of the compiled model (zero, within limits, when null). */
  public var reference:Null<Array<Float>> = null;
  /** What the reference is called in the report (e.g. "home"). */
  public var referenceName:String = "zero";
  public var tool:Null<Tool> = null;
  /** The flange frame the tool's geometry is in; else `toolLink`'s frame. */
  public var toolFrame:Null<String> = null;
  public var toolLink:Null<String> = null;
  /** The runtime blueprint's self-collision flag: simulation-only, recorded when off (CL-D10). */
  public var selfCollision:Bool = true;

  public function new(name:String) {
    this.name = name;
  }
}

/** A robot in a `CollisionDescription`: its model bodies, and the described objects of its shapes, hulls and tool. */
class RobotCollisionBodies {
  public final robot:RobotModel;
  public final model:KinematicModel;
  public final bodies:ModelBodies;
  /** Per link, the described objects of its shapes, in `collisionShapes` order. */
  public final shapes:Array<Array<Int>>;
  public final hulls:Array<Int>;
  /** The tool's described body (following its flange link), or -1. */
  public final toolBody:Int;
  public final toolObjects:Array<Int>;

  public function new(robot:RobotModel, model:KinematicModel, bodies:ModelBodies, shapes:Array<Array<Int>>,
      hulls:Array<Int>, toolBody:Int, toolObjects:Array<Int>) {
    this.robot = robot;
    this.model = model;
    this.bodies = bodies;
    this.shapes = shapes;
    this.hulls = hulls;
    this.toolBody = toolBody;
    this.toolObjects = toolObjects;
  }

  /** The described body of a link. */
  public function linkBody(link:String):Int {
    var body = model.bodyIndex(link);
    if (body < 0) throw 'Robot "${bodies.name}" has no link "$link"';
    return bodies.bodies[body];
  }
}

/**
 * A RobotKit robot's collision geometry (COLLISION.md CL3a): one body per
 * link from the compiled model, the kit's rigid, adjacent and closure pairs
 * as rules, link primitives and hulls, the tool on a body of its own group
 * following its flange link (hull padding as inflation), and the robot as
 * one articulation at its reference configuration.
 *
 * Contact settings are mapped per CL-D10: `ShapeContact` and the
 * blueprint's self-collision opt-out are simulation-only and are recorded as
 * ignored; each `ContactPair` becomes an allow rule with the reason
 * "declared contact". A link's mesh reference (`collisionGeometry`) is
 * recorded and left out: meshes are not loaded.
 */
class RobotCollision {
  public static function describe(description:CollisionDescription, robot:RobotModel,
      options:RobotCollisionOptions):RobotCollisionBodies {
    var model = RobotKinematics.compile(robot);
    var reference = new KinematicState(model, options.reference == null ? zeroWithinLimits(model) : options.reference);
    if (options.rootPose != null) {
      var root = RobotKinematics.toTransform(options.rootPose);
      for (body in 0...model.bodyCount()) if (model.bodyParentJoint[body] < 0)
        reference.setRootPose(body, root.compose(model.bodyRootPoses[body]));
    }
    var name = options.name;
    var bodies = ModelBodies.describe(description, model, name, options.linkGroup, reference, options.referenceName,
      options.fixedBase);
    if (!options.selfCollision)
      description.note('Robot "$name": the simulation\'s self-collision opt-out is simulation-only and ignored (CL-D10)');

    var shapes:Array<Array<Int>> = [];
    var hulls:Array<Int> = [];
    for (link in robot.links) {
      var body = bodies.bodies[model.bodyIndex(link.id)];
      var objects:Array<Int> = [];
      for (index in 0...link.collisionShapes.length) {
        var shape = link.collisionShapes[index];
        var object = description.addObject(shape.name == "" ? '$name/${link.id}/shape$index' : '$name/${shape.name}',
          body, offsetOf(shape), shape.geometry, shape.inflation,
          switch shape.geometry { case Convex(_): "hull"; case _: "primitive"; });
        objects.push(object);
        switch shape.geometry { case Convex(_): hulls.push(object); case _: }
        var simulationOnly = switch shape.contact {
          case Layers: false;
          case _: true;
        }
        if (simulationOnly)
          description.note('Robot "$name": the contact setting ${shape.contact} of ${link.id} shape $index is simulation-only and ignored (CL-D10)');
      }
      shapes.push(objects);
      if (link.collisionGeometry != null)
        description.note('Robot "$name": link ${link.id} references mesh "${link.collisionGeometry}", which is not loaded');
    }
    function linkIndex(id:String):Int {
      for (i in 0...robot.links.length) if (robot.links[i].id == id) return i;
      throw 'Robot "$name" has no link "$id"';
    }
    for (pair in robot.contactPairs) {
      var a = shapes[linkIndex(pair.linkA)][pair.shapeA], b = shapes[linkIndex(pair.linkB)][pair.shapeB];
      description.setObjectRule(a, b, CollisionPairRule.Allow, CollisionPairStatus.DeclaredContact);
    }

    var toolBody = -1;
    var toolObjects:Array<Int> = [];
    var tool = options.tool;
    var toolShape = tool == null ? [] : tool.collision;
    var hasTool = toolShape.length > 0;
    if (hasTool && tool != null) {
      var flangeBody:Int, flange:Transform;
      if (options.toolFrame != null) {
        var frame = model.frameIndex(options.toolFrame);
        if (frame < 0) throw 'Robot "$name" has no tool frame "${options.toolFrame}"';
        flangeBody = model.frameBody[frame];
        flange = model.frameTransform(frame);
      } else {
        var link = options.toolLink == null ? robot.links[robot.links.length - 1].id : options.toolLink;
        flangeBody = model.bodyIndex(link);
        if (flangeBody < 0) throw 'Robot "$name" has no tool link "$link"';
        flange = Transform.identity();
      }
      toolBody = bodies.follow(flangeBody, 'tool/${tool.id}', options.toolGroup);
      var at = (local:Transform) -> ModelBodies.pose(flange.compose(local));
      for (i in 0...toolShape.length) {
        var shape = toolShape[i], p = shape.position, q = shape.rotation;
        var local = new Transform(p[0],p[1],p[2],q[0],q[1],q[2],q[3]);
        toolObjects.push(description.addObject('$name/tool/piece$i', toolBody, at(local),
          shape.geometry, shape.inflation, "tool shape"));
      }
    }
    bodies.declare();
    return new RobotCollisionBodies(robot, model, bodies, shapes, hulls, toolBody, toolObjects);
  }

  /**
   * The hulls `CollisionClearance` checks (`ClearanceBodyData`: simulation hulls,
   * assembly parts and deposited weld beads carried on links), as objects on
   * the robot's link bodies, so both worlds see the same scene (CL4a).
   * Tool hulls go on the tool body, which must exist when any is given.
   */
  public static function describeClearanceBodies(description:CollisionDescription, robot:RobotCollisionBodies,
      data:Array<ClearanceBodyData>):Array<Int> {
    return [for (body in data) {
      var on = body.tool ? robot.toolBody : robot.linkBody(body.link);
      if (body.tool && on < 0) throw 'Clearance body "${body.name}" is a tool hull, but the robot has no tool body';
      // A tool hull is given in its link's frame; the tool body shares that link's pose.
      if (body.tool && robot.model.bodyIndex(body.link) != robot.bodies.followed[robot.bodies.followers.indexOf(on)])
        throw 'Clearance tool hull "${body.name}" is not on the tool\'s link';
      description.addObject('${robot.bodies.name}/${body.name}', on, CollisionPose.identity(),
        CollisionGeometry.Convex(body.vertices), 0.0, "hull");
    }];
  }

  /**
   * The robot's link pairs the simulation should exclude (COLLISION.md CL3b,
   * CL-D10): link index pairs (two per pair) between which some object pair
   * is allowed in the built world (or that carry no objects), the tool
   * counting as its flange link. Passed to `Simulation.addRobotAtPose
   * (linkExcludes)`, the simulation excludes these pairs instead of computing
   * its own. Excludes are per link, rules per object: a link pair with one
   * allowed object pair is excluded whole, so the simulation may exclude
   * more than the clearance world, never fewer (CL-D10).
   */
  public static function simulationExcludes(description:CollisionDescription, build:collisionkit.CollisionBuild,
      robot:RobotCollisionBodies):Array<Int> {
    var links = robot.model.bodyCount();
    var objectsOf = [for (_ in 0...links) new Array<Int>()];
    for (index in 0...description.objects.length) {
      var link = linkOf(robot, description.objects[index].body);
      if (link >= 0) objectsOf[link].push(build.objects[index]);
    }
    var pairs:Array<Int> = [];
    for (a in 0...links) for (b in a + 1...links) {
      var allowed = objectsOf[a].length == 0 || objectsOf[b].length == 0;
      for (x in objectsOf[a]) for (y in objectsOf[b]) if (!allowed && build.world.pairStatus(x, y) != CollisionPairStatus.Checked)
        allowed = true;
      if (allowed) {
        pairs.push(a);
        pairs.push(b);
      }
    }
    return pairs;
  }

  /** The robot link (model body) of a described body, a follower (the tool) counting as the link it follows, or -1. */
  static function linkOf(robot:RobotCollisionBodies, body:Int):Int {
    var link = robot.bodies.bodies.indexOf(body);
    if (link >= 0) return link;
    var follower = robot.bodies.followers.indexOf(body);
    return follower >= 0 ? robot.bodies.followed[follower] : -1;
  }

  static function zeroWithinLimits(model:KinematicModel):Array<Float>
    return [for (dof in 0...model.dofCount()) Math.min(Math.max(0.0, model.dofLower[dof]), model.dofUpper[dof])];

  static function offsetOf(shape:CollisionShape):CollisionPose {
    var p = shape.position == null ? [0.0, 0.0, 0.0] : shape.position;
    var r = shape.rotation == null ? [0.0, 0.0, 0.0, 1.0] : shape.rotation;
    return new CollisionPose(p[0], p[1], p[2], r[0], r[1], r[2], r[3]);
  }

}
