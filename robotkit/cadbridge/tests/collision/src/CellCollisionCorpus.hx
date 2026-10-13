import robotkit.collision.CollisionSceneBuilder;
import cadbridge.CellCollisionSnapshot;
import robotkit.collision.CollisionSceneBuilder.CollisionOwner;
import collisionkit.CollisionPairRule;
import collisionkit.CollisionPairStatus;
import collisionkit.native.NativeCollisionWorld;
import motionkit.robot.CollisionPlannerSpace;
import motionkit.robot.ManipulatorPlannerSpace;
import robotkit.manipulation.Manipulator;
import robotkit.model.Frame;
import robotkit.model.Joint;
import robotkit.model.JointType;
import robotkit.model.Link;
import robotkit.model.RobotModel;
import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;
import robotkit.spatial.Quat;

private typedef CellPairFixture = {
  var name:String;
  var first:String;
  var second:String;
  var gap:Float;
  var margin:Float;
  var touch:Bool;
  var clear:Bool;
  var welding:Bool;
  var foreign:Bool;
}

/** Independent cell interfaces and a thin obstacle between the usual joint samples. */
class CellCollisionCorpus {
  static var assertions = 0;

  public static function run():Int {
    var fixtures:Array<CellPairFixture> = [
      {name: "idle gripper B and neighbouring blank", first: "gripperB", second: "tray/blank", gap: 0.004, margin: 0.005, touch: false, clear: false, welding: false, foreign: false},
      {name: "vise seated contact", first: "blank", second: "vise/jaw", gap: 0, margin: 0, touch: true, clear: true, welding: false, foreign: false},
      {name: "vise penetration", first: "blank", second: "vise/jaw", gap: -0.000001, margin: 0, touch: true, clear: false, welding: false, foreign: false},
      {name: "tray neighbours", first: "tool", second: "tray/next-blank", gap: 0.006, margin: 0.005, touch: false, clear: true, welding: false, foreign: false},
      {name: "track designed interface", first: "track-arm", second: "track/support", gap: 0.004, margin: 0.003, touch: false, clear: true, welding: true, foreign: false},
      {name: "cross-robot door pair", first: "arm", second: "mill/door", gap: 0.004, margin: 0.005, touch: false, clear: false, welding: false, foreign: true}
    ];
    for (fixture in fixtures) pair(fixture);
    thinDoor();
    processContactIsNamed();
    snapshotOwnership();
    foreignGeometryWithNestedArmRoot();
    attachmentAndReset();
    releasedSeatAndOpenWindows();
    observedProcessJoint();
    sharedFixture();
    attachmentKeepsGeometricInflation();
    heldSupportIsStatic();
    compoundWorkMember();
    initialCompoundPayload();
    TrackWelderCollisionFixture.run(check);
    Sys.println('Cell collision corpus passed ($assertions assertions)');
    return assertions;
  }

  static function linkShapes(data:Array<{part:String, link:Int, vertices:Array<Float>}>):Array<robotkit.runtime.RobotRuntimeBlueprint.RobotRuntimeLinkShape> {
    return [for (row in data) {
      var shape = new robotkit.model.CollisionShape(collisionkit.CollisionGeometry.Convex(row.vertices));
      shape.name = row.part;
      new robotkit.runtime.RobotRuntimeBlueprint.RobotRuntimeLinkShape(row.link, shape);
    }];
  }

  static function arm():Manipulator {
    var robot = new RobotModel("corpus arm");
    var root = robot.addLink(new Link("root"));
    var moving = robot.addLink(new Link("moving"));
    var joint = robot.addJoint(new Joint("turn", JointType.Revolute, root, moving));
    joint.axis = [0.0, 0.0, 1.0]; joint.limits.lower = -Math.PI; joint.limits.upper = Math.PI;
    var tool = robot.addFrame(new Frame("tool", moving));
    tool.position = [1.0, 0.0, 0.0];
    return new Manipulator(robot, root.id, tool.id);
  }

  static function pair(fixture:CellPairFixture):Void {
    var arm = arm();
    var captured = new CellCollisionSnapshot("robot", Transform3.identity(), [], [
      CellCollisionSnapshot.observation(fixture.first, Mechanism("robot", "moving"), cube(0, 0, 0, 0.01), null, fixture.welding),
      CellCollisionSnapshot.observation(fixture.second, fixture.foreign ? Mechanism("mill", "moving") : Fixture,
        cube(0, 0, 0, 0.01), new Transform3(new Vec3(0.02 + fixture.gap, 0, 0), Quat.identity()))
    ]);
    var scene = captured.describe(arm, [0.0], 0.005);
    if (fixture.touch) processkit.HandlingCollisionPolicy.seatedStock(scene,
      scene.memberObjects([fixture.first]), scene.memberObjects([fixture.second]));
    else if (fixture.welding) processkit.WeldCollisionPolicy.apply(scene,
      scene.memberObjects([fixture.second]), fixture.margin, 0.0005);
    var build = scene.build(new NativeCollisionWorld());
    var planner = new ManipulatorPlannerSpace(new CollisionPlannerSpace(scene.description, build, scene.bodies, scene.margins()), arm, [0.0]);
    check((planner.violation([0.0]) == null) == fixture.clear, fixture.name + " conservative point verdict");
    var clearance = scene.clearance(() -> new NativeCollisionWorld());
    check((clearance.violation([0.0]) == null) == fixture.clear, fixture.name + " process point verdict");
    var copy = clearance.copyForGroup(arm);
    check((copy.violation([0.0]) == null) == fixture.clear, fixture.name + " worker verdict");
    var closest = clearance.closest([0.0]);
    check(closest != null && closest.a == fixture.first && closest.b == fixture.second &&
      Math.abs(closest.distance - Math.max(0.0, fixture.gap)) < 1e-8, fixture.name + " names and actual distance");
    clearance.world.dispose();
    copy.world.dispose();
    planner.dispose();
  }

  static function thinDoor():Void {
    var arm = arm(), centre = Math.sqrt(0.5);
    var captured = new CellCollisionSnapshot("robot", Transform3.identity(), [], [
      CellCollisionSnapshot.observation("arm/tool", Mechanism("robot", "moving"), cube(1, 0, 0, 0.003)),
      CellCollisionSnapshot.observation("mill/thin-door-edge", Fixture, cube(centre, centre, 0, 0.001))
    ]);
    var scene = captured.describe(arm, [0.0], 0.005);
    var build = scene.build(new NativeCollisionWorld());
    var planner = new ManipulatorPlannerSpace(new CollisionPlannerSpace(scene.description, build, scene.bodies, scene.margins()), arm, [0.0]);
    for (q in [0.0, Math.PI / 2, Math.PI]) check(planner.violation([q]) == null, "thin edge is clear at ends and midpoint");
    check(planner.violation([Math.PI / 4]) != null, "thin edge intersects inside the motion");
    check(!planner.edgesClear([0.0, Math.PI], 1)[0], "native continuous certificate refuses the hidden thin edge");
    planner.dispose();
  }

  static function processContactIsNamed():Void {
    var arm = arm();
    var captured = new CellCollisionSnapshot("robot", Transform3.identity(), [], [
      CellCollisionSnapshot.observation("torch", Mechanism("robot", "moving"), cube(0, 0, 0, 0.01), null, true),
      CellCollisionSnapshot.observation("work", Fixture, cube(0.021, 0, 0, 0.01)),
      CellCollisionSnapshot.observation("unrelated-door", Fixture, cube(0, 0.024, 0, 0.01))
    ]);
    var scene = captured.describe(arm, [0.0], 0.005);
    processkit.WeldCollisionPolicy.apply(scene, scene.memberObjects(["work"]), 0.005, 0.0005);
    var clearance = scene.clearance(() -> new NativeCollisionWorld());
    var found = clearance.violation([0.0], true);
    check(found != null && found.b == "unrelated-door" && found.required == 0.005,
      "work contact does not reduce an unrelated door's margin");
    clearance.apply(robotkit.manipulation.ClearanceChange.OpenWindow("torch", "unrelated-door", 0.003), [0.0]);
    check(clearance.violation([0.0], true) == null, "a named process window changes only its pair");
    clearance.apply(robotkit.manipulation.ClearanceChange.CloseWindow("torch", "unrelated-door"), [0.0]);
    check(clearance.violation([0.0], true) != null, "closing a window restores the original cell policy");
    clearance.reset();
    check(clearance.violation([0.0], true) != null, "reset restores the described snapshot");
    clearance.world.dispose();
  }

  static function snapshotOwnership():Void {
    var arm = arm();
    var captured = new CellCollisionSnapshot("arm", Transform3.identity(), []);
    captured.mechanism("arm", arm.robot, linkShapes([{part: "arm/tool", link: 1, vertices: cube(0, 0, 0, 0.01)}]),
      [Transform3.identity(), Transform3.identity()]);
    captured.mechanism("mill", arm.robot, linkShapes([{part: "mill/door", link: 1, vertices: cube(0, 0, 0, 0.01)}]),
      [Transform3.identity(), new Transform3(new Vec3(0.024, 0, 0), Quat.identity())]);
    var scene = captured.describe(arm, [0.0], 0.005);
    var clearance = scene.clearance(() -> new NativeCollisionWorld());
    check(clearance.violation([0.0]) != null, "a foreign controller's door never inherits arm reference exclusions");
    var foreign = false;
    for (id in scene.ownership.keys()) {
      var metadata = scene.ownership.get(id);
      if (metadata == null) throw "Missing collision ownership";
      switch metadata.owner {
      case Mechanism(controller, link): if (id == "mill/door") foreign = controller == "mill" && link == "moving";
      case _:
    }
    }
    check(foreign, "controller ownership survives the collision snapshot");
    var rebased = captured.at(new Transform3(new Vec3(1, 0, 0), Quat.identity()));
    check(Math.abs(rebased.fixedBounds()[0].minimum[0] - 0.014) < 1e-12,
      "changing a station frame keeps the actual door in its fixed world location");
    rebased.object("new-held-stock", [cube(0, 0, 0, 0.01)],
      new Transform3(new Vec3(1.5, 0, 0), Quat.identity()), "moving");
    var stock = rebased.hulls[rebased.hulls.length - 1];
    check(stock.id == "new-held-stock" && Math.abs(stock.pose.translation.x - 0.5) < 1e-12 && !stock.tool,
      "a rebased snapshot retains observed link poses for new held stock without classifying stock as a tool");
    clearance.world.dispose();
  }

  static function foreignGeometryWithNestedArmRoot():Void {
    var model = new RobotModel("nested arm root");
    var base = model.addLink(new Link("base"));
    var mount = model.addLink(new Link("mount"));
    var moving = model.addLink(new Link("moving"));
    var installed = model.addJoint(new Joint("installed", JointType.Fixed, base, mount));
    installed.parentFramePosition = [1.0, 0.0, 0.0];
    var turn = model.addJoint(new Joint("turn", JointType.Revolute, mount, moving));
    turn.limits.lower = -Math.PI; turn.limits.upper = Math.PI;
    var tool = model.addFrame(new Frame("tool", moving));
    var arm = new Manipulator(model, mount.id, tool.id);
    var one = new Transform3(new Vec3(1, 0, 0), Quat.identity());
    var captured = new CellCollisionSnapshot("arm", Transform3.identity(), []);
    captured.mechanism("arm", model, linkShapes([{part: "arm-object", link: 2, vertices: cube(0, 0, 0, 0.01)}]),
      [Transform3.identity(), one, one]);
    captured.mechanism("mill", model, linkShapes([{part: "foreign-door", link: 2, vertices: cube(0, 0, 0, 0.01)}]),
      [Transform3.identity(), one, new Transform3(new Vec3(1.024, 0, 0), Quat.identity())]);
    var clearance = captured.describe(arm, [0.0], 0.005).clearance(() -> new NativeCollisionWorld());
    var found = clearance.closest([0.0]);
    check(found != null && Math.abs(found.distance - 0.004) < 1e-10 && clearance.violation([0.0]) != null,
      "foreign geometry stays in the controller frame when the selected arm root is a descendant");
    clearance.world.dispose();
  }

  static function attachmentAndReset():Void {
    var arm = arm();
    var captured = new CellCollisionSnapshot("arm", Transform3.identity(), [], [
      CellCollisionSnapshot.observation("loose", Loose, cube(1, 0, 0, 0.01)),
      CellCollisionSnapshot.observation("obstacle", Fixture, cube(0, 1, 0, 0.01))
    ]);
    var scene = captured.describe(arm, [0.0], 0.005);
    var clearance = scene.clearance(() -> new NativeCollisionWorld());
    clearance.apply(robotkit.manipulation.ClearanceChange.Attach("loose", "moving", []), [0.0]);
    check(clearance.violation([0.0]) == null, "attachment preserves the part's current pose");
    check(clearance.violation([Math.PI / 2]) != null, "an attached part follows its observed carrier into an obstacle");
    var envelope = clearance.motionEnvelope([0.0], [Math.PI / 2]);
    var ratio = clearance.auditDisplacement([0.0], [0.001], [0.001], envelope);
    check(ratio > 0 && ratio <= 1, "described attached hulls keep conservative displacement bounds");
    clearance.apply(robotkit.manipulation.ClearanceChange.Detach("loose"), [Math.PI / 2]);
    check(clearance.movingNames().indexOf("loose") < 0, "a detached part is independent of arm motion");
    clearance.reset();
    clearance.apply(robotkit.manipulation.ClearanceChange.Attach("loose", "moving", []), [0.0]);
    check(clearance.violation([0.0]) == null && clearance.violation([Math.PI / 2]) != null,
      "reset restores geometry and ownership before a new attachment");
    clearance.world.dispose();
  }

  static function releasedSeatAndOpenWindows():Void {
    var arm = arm();
    var captured = new CellCollisionSnapshot("arm", Transform3.identity(), [], [
      CellCollisionSnapshot.observation("stock", Loose, cube(1, 0, 0, 0.01)),
      CellCollisionSnapshot.observation("seat-piece", Fixture, cube(1.02, 0, 0, 0.01), null, false, "vise-seat")
    ]);
    var scene = captured.describe(arm, [0.0], 0.005);
    processkit.HandlingCollisionPolicy.seatedStock(scene, scene.memberObjects(["stock"]), scene.memberObjects(["vise-seat"]));
    var clearance = scene.clearance(() -> new NativeCollisionWorld());
    clearance.apply(robotkit.manipulation.ClearanceChange.Attach("stock", "moving", ["vise-seat"]), [0.0]);
    check(clearance.violation([0.0]) == null, "attachment accepts a physical contact member as well as exact object names");
    clearance.apply(robotkit.manipulation.ClearanceChange.Detach("stock"), [0.0]);
    check(clearance.violation([0.0]) == null && Math.abs(clearance.distanceBetween([0.0], "stock", "vise-seat")) < 1e-8,
      "release onto the vise restores the authored zero-margin seat Touch rule");
    clearance.apply(robotkit.manipulation.ClearanceChange.OpenWindow("stock", "vise-seat", null), [0.0]);
    var refused = false;
    try clearance.apply(robotkit.manipulation.ClearanceChange.Attach("stock", "moving", []), [0.0]) catch (_:Dynamic) refused = true;
    check(refused && clearance.violation([0.0]) == null,
      "attachment during an open body window is refused without wiping its permission");
    refused = false;
    try clearance.apply(robotkit.manipulation.ClearanceChange.Detach("stock"), [0.0]) catch (_:Dynamic) refused = true;
    check(refused, "detach during an open body window is refused");
    clearance.apply(robotkit.manipulation.ClearanceChange.CloseWindow("stock", "vise-seat"), [0.0]);
    check(clearance.violation([0.0]) == null, "closing the surviving window restores the seat rule");
    clearance.world.dispose();
  }

  static function observedProcessJoint():Void {
    var robot = new RobotModel("arm with process jaw");
    var root = robot.addLink(new Link("root")), moving = robot.addLink(new Link("moving"));
    var jaw = robot.addLink(new Link("jaw"));
    var turn = robot.addJoint(new Joint("turn", JointType.Revolute, root, moving));
    turn.axis = [0.0, 0.0, 1.0]; turn.limits.lower = -Math.PI; turn.limits.upper = Math.PI;
    var slide = robot.addJoint(new Joint("jaw-slide", JointType.Prismatic, moving, jaw));
    slide.axis = [0.0, 1.0, 0.0]; slide.limits.lower = 0; slide.limits.upper = 0.05;
    var tool = robot.addFrame(new Frame("tool", moving));
    var arm = new Manipulator(robot, root.id, tool.id);
    var captured = new CellCollisionSnapshot("arm", Transform3.identity(), []);
    captured.mechanism("arm", robot, linkShapes([{part: "jaw/pad", link: 2, vertices: cube(0, 0, 0, 0.01)}]),
      [Transform3.identity(), Transform3.identity(), new Transform3(new Vec3(0, 0.02, 0), Quat.identity())],
      null, robotkit.kinematics.RobotKinematics.observedState(robot, [0.6, 0.02]));
    captured.fixture("door", cube(0, 0.044, 0, 0.01), Transform3.identity());
    var scene = captured.describe(arm, [0.0], 0.005);
    check(scene.bodies.reference.q[arm.model.dofIndex("turn")] == 0.0 &&
      scene.bodies.reference.q[arm.model.dofIndex("jaw-slide")] == 0.02,
      "planning starts at its selected command while the process joint keeps its observed position");
    var build = scene.build(new NativeCollisionWorld());
    var planner = new ManipulatorPlannerSpace(new CollisionPlannerSpace(scene.description, build, scene.bodies, scene.margins()),
      arm, [0.0], scene.bodies.reference.q);
    check(planner.violation([0.0]) != null, "conservative planning checks the observed jaw rather than its open default");
    var clearance = scene.clearance(() -> new NativeCollisionWorld());
    check(clearance.violation([0.0]) != null, "process checking retains the same observed jaw coordinate");
    var envelope = clearance.motionEnvelope([-0.01], [0.01]);
    var ratio = clearance.auditDisplacement([0.0], [0.001], [0.001], envelope);
    check(ratio > 0 && ratio <= 1, "motion bounds include the observed passive slide's actual lever arm");
    clearance.world.dispose(); planner.dispose();
  }

  static function sharedFixture():Void {
    var arm = arm(), captured = new CellCollisionSnapshot("arm", Transform3.identity(), []);
    var poses = [Transform3.identity(), Transform3.identity()];
    var floor = [{part: "cell/floor", link: 0, vertices: cube(0, 0, -1, 0.1)}];
    captured.mechanism("arm", arm.robot, linkShapes(floor), poses);
    captured.mechanism("mill", arm.robot, linkShapes(floor), poses);
    check(captured.hulls.length == 1, "shared root geometry enters the cell once");
    var fixture = switch captured.hulls[0].owner { case Fixture: true; case _: false; };
    check(fixture, "a shared fixed floor belongs to the environment rather than a controller");
    var rebased = captured.at(new Transform3(new Vec3(1, 0, 0), Quat.identity()));
    rebased.mechanism("mill", arm.robot, linkShapes(floor), poses);
    check(rebased.hulls.length == 1 && Math.abs(rebased.fixedBounds()[0].minimum[0] + 0.1) < 1e-12,
      "rebasing retains shared physical identity and the floor's world pose");
    var refused = false;
    try {
      var moving = new CellCollisionSnapshot("arm", Transform3.identity(), []);
      var member = [{part: "ambiguous-jaw", link: 1, vertices: cube(0, 0, 0, 0.01)}];
      moving.mechanism("arm", arm.robot, linkShapes(member), poses);
      moving.mechanism("mill", arm.robot, linkShapes(member), poses);
    } catch (_:Dynamic) { refused = true; }
    check(refused, "two controllers cannot claim one moving physical occurrence");
  }

  static function attachmentKeepsGeometricInflation():Void {
    var arm = arm();
    var captured = new CellCollisionSnapshot("arm", Transform3.identity(), [], [
      CellCollisionSnapshot.observation("loose", Held("arm", "moving"), cube(1, 0, 0, 0.01)),
      CellCollisionSnapshot.observation("obstacle", Fixture, cube(1.025, 0.025, 0, 0.01))
    ]);
    var scene = captured.describe(arm, [0.0], 0.005);
    var id = scene.object("loose"), original = scene.description.objects[id];
    scene.description.objects[id] = new collisionkit.DescribedObject(original.name, original.body, original.offset,
      original.geometry, 0.001, "independent geometric padding fixture", original.member);
    var clearance = scene.clearance(() -> new NativeCollisionWorld());
    var before = clearance.closest([0.0]);
    var expected = Math.sqrt(2) * 0.005 - 0.001;
    check(before != null && Math.abs(before.distance - expected) < 1e-8, "native fixture has the stated radial geometric inflation");
    clearance.apply(robotkit.manipulation.ClearanceChange.Attach("loose", "moving", []), [0.0]);
    var after = clearance.closest([0.0]);
    check(after != null && Math.abs(after.distance - expected) < 1e-8,
      "reattachment keeps radial inflation rather than baking its bounding cube into the collision shape");
    clearance.world.dispose();
  }

  static function heldSupportIsStatic():Void {
    var robot = new RobotModel("parked mobile arm");
    var root = robot.addLink(new Link("root")), moving = robot.addLink(new Link("moving"));
    var wheel = robot.addLink(new Link("wheel"));
    var shoulder = robot.addJoint(new Joint("turn", JointType.Revolute, root, moving));
    shoulder.axis = [0.0, 0.0, 1.0]; shoulder.limits.lower = -Math.PI; shoulder.limits.upper = Math.PI;
    var axle = robot.addJoint(new Joint("wheel-turn", JointType.Revolute, root, wheel));
    axle.axis = [0.0, 1.0, 0.0]; axle.parentFramePosition = [0.0, 0.0, -0.04];
    var frame = robot.addFrame(new Frame("tool", moving));
    var arm = new Manipulator(robot, root.id, frame.id);
    var captured = new CellCollisionSnapshot("arm", Transform3.identity(), [], [
      CellCollisionSnapshot.observation("arm/tool", Mechanism("arm", "moving"), cube(1, 0, 0, 0.01)),
      CellCollisionSnapshot.observation("wheel", Mechanism("arm", "wheel"), cube(0, 0, 0, 0.01)),
      CellCollisionSnapshot.observation("floor", Fixture, cube(0, 0, -0.059, 0.01))
    ]);
    var scene = captured.describe(arm, [0.0], 0.005);
    var clearance = scene.clearance(() -> new NativeCollisionWorld());
    check(clearance.violation([0.0]) == null && clearance.violation([0.5]) == null,
      "a held wheel touching its support does not become an arm-motion obstacle");
    var build = scene.build(new NativeCollisionWorld());
    var planner = new ManipulatorPlannerSpace(new CollisionPlannerSpace(scene.description, build, scene.bodies, scene.margins()),
      arm, [0.0], scene.bodies.reference.q);
    check(planner.edgesClear([0.0, 0.5], 1)[0], "the continuous planner uses the same selected-motion static mask");
    clearance.world.dispose(); planner.dispose();
  }

  static function compoundWorkMember():Void {
    var arm = arm(), captured = new CellCollisionSnapshot("arm", Transform3.identity(), []);
    captured.mechanism("arm", arm.robot, linkShapes([{part: "torch", link: 1, vertices: cube(0, 0, 0, 0.01)}]),
      [Transform3.identity(), Transform3.identity()], ["torch"]);
    captured.object("work", [cube(0.021, 0, 0, 0.01), cube(0, 0.021, 0, 0.01)], Transform3.identity(), null, Fixture);
    var scene = captured.describe(arm, [0.0], 0.005);
    var work = scene.memberObjects(["work"]);
    check(work.length == 2 && scene.description.objects[work[0]].body == scene.description.objects[work[1]].body,
      "all convex pieces retain one typed physical work member and body");
    processkit.WeldCollisionPolicy.apply(scene, work, 0.005, 0.0005);
    var clearance = scene.clearance(() -> new NativeCollisionWorld());
    check(clearance.violation([0.0]) != null && clearance.violation([0.0], true) == null,
      "the work margin applies to every piece without relaxing air clearance");
    check(scene.description.groupCount() <= 5, "compound workpieces use margin classes instead of one group per hull");
    clearance.world.dispose();
  }

  static function initialCompoundPayload():Void {
    var arm = arm(), captured = new CellCollisionSnapshot("arm", Transform3.identity(), []);
    captured.mechanism("arm", arm.robot, linkShapes([
      {part: "pad-A", link: 1, vertices: cube(1, 0, 0, 0.01)},
      {part: "pad-B", link: 1, vertices: cube(1.04, 0, 0, 0.01)},
      {part: "plate", link: 1, vertices: cube(1.1, 0, 0, 0.01)}
    ]), [Transform3.identity(), Transform3.identity()], ["pad-A", "pad-B", "plate"]);
    captured.object("stock", [cube(1.02, 0, 0, 0.01), cube(1.02, 0, 0, 0.01)], Transform3.identity(), "moving");
    var scene = captured.describe(arm, [0.0], 0.005), stock = scene.memberObjects(["stock"]);
    var pads = scene.contacts(["pad-A"]);
    scene.payload(scene.description.objects[stock[0]].name, pads,
      CollisionPairRule.Allow, CollisionPairStatus.ProcessContact, 0.0);
    check(scene.description.marginBetweenObjects(stock[1], scene.object("pad-A")) == 0.0 &&
      scene.description.marginBetweenObjects(stock[1], scene.object("pad-B")) == 0.005,
      "one payload declaration sets every piece's selected-contact margin and preserves other clearances");
    var build = scene.build(new NativeCollisionWorld());
    var metadata = scene.ownership.get(scene.description.objects[stock[0]].name);
    if (metadata == null) throw "Held stock lost its ownership fact";
    check(stock.length == 2 && !scene.description.objects[stock[0]].tool,
      "initial compound held stock keeps its physical identity without becoming tool geometry");
    check(build.world.pairStatus(build.objects[stock[0]], build.objects[stock[1]]) == CollisionPairStatus.Rigid,
      "overlapping hull pieces of the same held part do not check against each other");
    check(build.world.pairStatus(build.objects[stock[0]], build.objects[scene.object("pad-A")]) == CollisionPairStatus.ProcessContact &&
      build.world.pairStatus(build.objects[stock[0]], build.objects[scene.object("pad-B")]) == CollisionPairStatus.Checked &&
      build.world.pairStatus(build.objects[stock[0]], build.objects[scene.object("plate")]) == CollisionPairStatus.Checked,
      "named pad contacts never exempt a second gripper or the mounting plate on the same carrier");
    var clearance = scene.clearance(() -> new NativeCollisionWorld());
    check(clearance.violation([0.0]) != null, "the second gripper touching initially held stock is detected");
    clearance.apply(robotkit.manipulation.ClearanceChange.OpenWindow("stock", "pad-B", null), [0.0]);
    check(clearance.violation([0.0]) == null, "a physical-member window covers its compound pieces through the shared operation");
    clearance.apply(robotkit.manipulation.ClearanceChange.CloseWindow("stock", "pad-B"), [0.0]);
    check(clearance.violation([0.0]) != null, "closing the compound window restores held-stock checks");
    clearance.world.dispose(); build.world.dispose();
  }

  static function cube(x:Float, y:Float, z:Float, half:Float):Array<Float> {
    var result:Array<Float> = [];
    for (i in 0...8) {
      result.push(x + ((i & 1) == 0 ? -half : half));
      result.push(y + ((i & 2) == 0 ? -half : half));
      result.push(z + ((i & 4) == 0 ? -half : half));
    }
    return result;
  }

  static function check(value:Bool, message:String):Void {
    assertions++;
    if (!value) throw message;
  }
}
