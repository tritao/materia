import collisionkit.CollisionDescription;
import collisionkit.CollisionGeometry;
import collisionkit.CollisionPose;
import collisionkit.native.NativeCollisionWorld;
import robotkit.collision.RobotCollision;
import robotkit.collision.RobotCollision.RobotCollisionOptions;
import robotkit.collision.RobotCollision.RobotLinkHull;
import robotkit.model.CollisionShape;
import robotkit.model.Joint;
import robotkit.model.JointType;
import robotkit.model.Link;
import robotkit.model.RobotModel;
import robotkit.profile.RobotProfile;
import robotkit.runtime.RobotRuntimeCompiler;
import robotkit.runtime.SimulationHarness;
import robotkit.runtime.SimulationSpace;

/**
 * One collision description shared by the coal world and the MuJoCo
 * simulation (COLLISION.md CL3b): the simulation excludes exactly the link
 * pairs the description allows (a declared base–forearm allowance included)
 * instead of computing its own, gets the description's hulls, and at
 * sampled configurations every MuJoCo contact is a collision in the coal
 * world.
 */
class CollisionMuJoCoTests {
  static var assertions = 0;
  static final BOXES = [[1.4, 0.9, 0.0, 0.2, 0.2, 0.3], [-0.2, 1.3, 0.0, 0.25, 0.15, 0.3]];

  public static function main():Void {
    testSharedDescription();
    Sys.println('Collision MuJoCo tests passed ($assertions assertions)');
  }

  /** Base -> shoulder (z) -> upper -> elbow (z, x = 1) -> fore: boxes along the links, a hull on the upper arm. */
  static function arm():RobotModel {
    var model = new RobotModel("arm");
    var base = model.addLink(new Link("base"));
    var upper = model.addLink(new Link("upper"));
    var fore = model.addLink(new Link("fore"));
    var shoulder = model.addJoint(new Joint("shoulder", JointType.Revolute, base, upper));
    shoulder.axis = [0.0, 0, 1]; shoulder.limits.lower = -3.1; shoulder.limits.upper = 3.1;
    shoulder.limits.velocity = 2; shoulder.limits.effort = 100;
    var elbow = model.addJoint(new Joint("elbow", JointType.Revolute, upper, fore));
    elbow.axis = [0.0, 0, 1]; elbow.limits.lower = -3.1; elbow.limits.upper = 3.1;
    elbow.limits.velocity = 2; elbow.limits.effort = 100;
    elbow.parentFramePosition = [1.0, 0, 0];
    for (link in [base, upper, fore]) link.inertiaTensor = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01];
    base.collisionShapes.push(new CollisionShape(Box(0.2, 0.2, 0.1)));
    upper.collisionShapes.push(new CollisionShape(Box(0.4, 0.05, 0.05), [0.5, 0, 0]));
    fore.collisionShapes.push(new CollisionShape(Box(0.5, 0.05, 0.05), [0.55, 0, 0]));
    return model;
  }

  static function cube(x:Float, y:Float, z:Float, half:Float):Array<Float>
    return [for (i in 0...8) for (axis in 0...3) (axis == 0 ? x : axis == 1 ? y : z) + (((i >> axis) & 1) == 1 ? half : -half)];

  static function testSharedDescription():Void {
    var model = arm();
    var description = new CollisionDescription();
    var options = new RobotCollisionOptions("arm");
    options.hulls = [new RobotLinkHull(1, "upper-hull", cube(0.3, 0, 0.0, 0.06))];
    var robot = RobotCollision.describe(description, model, options);
    var boxes = [for (box in BOXES)
      description.addFixed('box', CollisionPose.translation(box[0], box[1], box[2]), CollisionGeometry.Box(box[3], box[4], box[5]), 1)];
    var world = new NativeCollisionWorld();
    var build = description.build(world);
    check(build.layoutErrors.length == 0, "the cell starts clear");
    var excludes = RobotCollision.simulationExcludes(description, build, robot);
    var pairs = [for (i in 0...Std.int(excludes.length / 2)) excludes[2 * i] + "-" + excludes[2 * i + 1]];
    check(pairs.join(",") == "0-1,1-2", 'the description allows base-upper and upper-fore, and checks base-fore (${pairs.join(",")})');
    var hulls = [for (hull in RobotCollision.simulationHulls(description, robot)) {link: hull.link, vertices: hull.vertices}];
    check(hulls.length == 1 && hulls[0].link == 1, "the description's hull feeds the simulation");
    var blueprint = RobotRuntimeCompiler.compile(model, new RobotProfile());

    /** MuJoCo's contacts with the robot at `q` (a fresh simulation, one step), with or without the shared excludes. */
    function contactsAt(q:Array<Float>, shared:Bool):Array<{link:Int, otherLink:Int, box:Int, distance:Float}> {
      var harness = new SimulationHarness(0.01, 1, SimulationSpace.MUJOCO);
      var simulation = harness.simulation;
      var runtime = simulation.addRobotAtPose(blueprint, [0, 0, 0], [0, 0, 0, 1], null, null, null, null, null, null, null,
        null, cast hulls, false, shared ? excludes : null);
      var objects = [for (box in BOXES) harness.spawnBox([box[0], box[1], box[2]], [box[3], box[4], box[5]])];
      simulation.setJointPositions(0, q);
      harness.step();
      var found = [for (c in simulation.robotContacts(runtime)) if (c.active && c.distance <= 0) {
        var box = -1;
        for (i in 0...objects.length) if (objects[i].handle.rawValue() == c.otherObject) box = i;
        {link: c.linkIndex, otherLink: c.otherKind == RobotLink ? c.otherLink : -1, box: box, distance: c.distance};
      }];
      harness.dispose();
      return found;
    }

    // Folded back, the forearm reaches over the base. SimKit's own rule excludes the pair (its
    // bounding-radius test calls them overlapping at rest); the description checks it, and with its
    // excludes MuJoCo reports the contact, as coal does.
    var folded = [0.0, 3.0];
    function baseFore(found:Array<{link:Int, otherLink:Int, box:Int, distance:Float}>):Int
      return found.filter(c -> (c.link == 0 && c.otherLink == 2) || (c.link == 2 && c.otherLink == 0)).length;
    check(baseFore(contactsAt(folded, false)) == 0, "SimKit's own rule excludes the base and the forearm");
    check(baseFore(contactsAt(folded, true)) > 0, "with the description's excludes MuJoCo reports their contact");
    robot.bodies.place(build, robot.bodies.state(folded));
    var baseBody = build.bodies[robot.linkBody("base")], foreBody = build.bodies[robot.linkBody("fore")];
    check(world.colliding(0.0).filter(p -> (p.bodyA == baseBody && p.bodyB == foreBody) || (p.bodyA == foreBody && p.bodyB == baseBody)).length > 0,
      "and coal reports it too");

    // Sampled configurations: every MuJoCo contact is a collision in the coal world.
    var rng = 7, contacts = 0;
    function next():Float {
      rng = (rng * 1103515245 + 12345) & 0x7fffffff;
      return rng / 2147483647.0 * 2.0 - 1.0;
    }
    for (_ in 0...60) {
      var q = [3.0 * next(), 3.0 * next()];
      robot.bodies.place(build, robot.bodies.state(q));
      var colliding = world.colliding(1e-4);
      for (contact in contactsAt(q, true)) {
        contacts++;
        var a = build.bodies[robot.bodies.bodies[contact.link]];
        var b = contact.otherLink >= 0 ? build.bodies[robot.bodies.bodies[contact.otherLink]]
          : contact.box >= 0 ? build.bodies[description.objects[boxes[contact.box]].body] : -2;
        var found = colliding.filter(p -> (p.bodyA == a && p.bodyB == b) || (p.bodyA == b && p.bodyB == a));
        check(found.length > 0, 'MuJoCo contact link ${contact.link} / ${contact.otherLink >= 0 ? "link " + contact.otherLink : "box " + contact.box} at q $q (depth ${contact.distance}) is a coal collision');
      }
    }
    check(contacts > 0, 'the sampled configurations touch ($contacts contacts)');
    Sys.println('CL3B cross-check: $contacts MuJoCo contacts over 60 configurations, all coal collisions');
    world.dispose();
  }

  static function check(value:Bool, message:String):Void {
    assertions++;
    if (!value) throw 'assertion failed: $message';
  }
}
