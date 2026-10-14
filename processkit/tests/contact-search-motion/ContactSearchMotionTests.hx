import CollisionFixture;
import collisionkit.native.NativeCollisionWorld;
import haxe.Int64;
import processkit.ContactSearchRunner;
import processkit.ProbeMotionPlanner;
import processkit.ContactProbeRunner;
import processkit.ContactProbeRunner.ContactProbeRequest;
import processkit.ContactRegistrationRunner;
import processkit.WeldStowPlanner;
import motionkit.robot.ManipulatorMotion;
import processkit.WeldingPlanRunner;
import robotkit.spatial.Transform3;
import robotkit.spatial.Quat;
import robotkit.collision.CollisionClearance;
import processkit.tool.WeldSensor;
import processkit.tool.WeldArcModel;
import robotkit.model.RobotModel;
import robotkit.model.Link;
import robotkit.model.Joint;
import robotkit.model.JointType;
import robotkit.model.Frame;
import robotkit.manipulation.Manipulator;
import robotkit.runtime.RobotRuntimeCompiler;
import robotkit.runtime.RobotRuntimeSensorBlueprint;
import robotkit.runtime.SimulationHarness;
import robotkit.simulation.SimulatedRobot;
import robotkit.spatial.Vec3;
import motionkit.robot.ServoSession;

@:access(WeldPlanningTests)
class ContactSearchMotionTests {
  static var checks = 0;
  static function check(ok:Bool, message:String):Void { checks++; if (!ok) throw message; }
  static function axis(flip:Bool = false):{model:RobotModel, arm:Manipulator, base:Link, tool:Link} {
    var model = new RobotModel("contact-search-axis");
    var base = model.addLink(new Link("base"));
    var tool = model.addLink(new Link("tool"));
    var joint = model.addJoint(new Joint("probe", JointType.Prismatic, base, tool));
    joint.axis = [0.0, 0.0, 1.0];
    joint.limits.lower = -0.1; joint.limits.upper = 0.1;
    joint.limits.velocity = 0.1; joint.limits.maxAcceleration = 1.0;
    var flange = model.addFrame(new Frame("tip", tool));
    if (flip) flange.rotation = [1.0, 0.0, 0.0, 0.0];
    var arm = new Manipulator(model, base.id, flange.id);
    return {model: model, arm: arm, base: base, tool: tool};
  }
  static function run(dropFeedback:Bool, noTouch:Bool):Void {
    var fixture = axis();
    var model = fixture.model, arm = fixture.arm, base = fixture.base, tool = fixture.tool;
    if (!dropFeedback && !noTouch) {
      var planning = WeldingPlanRunner.planning(arm, 1.0);
      var source = planning.compiler;
      var solver = new ObservedProbeSolver(arm);
      var compiler = new motionkit.robot.ProgramCompiler(solver, source.limits, source.frameId,
        source.maxVelocity, source.maxAcceleration, source.maxJerk, source.startTolerances,
        source.timing, source.cartesianResolution, source.maxJointJump, source.positionTolerance,
        source.orientationTolerance, source.ikTolerance, null, source.perJointMaxJump);
      compiler.planCheck = source.planCheck;
      var probePlanner = new ProbeMotionPlanner(arm, compiler);
      var screening = new processkit.ProbePosePlanner(probePlanner);
      check(screening.hasClearApproachConfiguration(new Vec3(0, 0, -0.02), new Vec3(0, 0, -1), 0.02, [0.0]),
        "Candidate screening keeps a reachable aligned probe");
      check(!screening.hasClearApproachConfiguration(new Vec3(0, 0, -0.02), new Vec3(0, 1, 0), 0.02, [0.0]),
        "Candidate screening excludes unreachable torch orientations before pattern selection");
      check(screening.hasClearObservedApproachConfiguration(new Vec3(0, 0, -0.02), new Vec3(0, 0, -1), 0.02, [0.0]),
        "observed branch screening accepts a clear reachable normal");
      check(!screening.hasClearObservedApproachConfiguration(new Vec3(0, 0, -0.02), new Vec3(0, 1, 0), 0.02, [0.0]),
        "observed branch screening rejects an unreachable normal");
      solver.discoveries = 0;
      var preparedProbe = new processkit.ProbePosePlanner(probePlanner).prepare(new Vec3(0, 0, -0.02),
        new Vec3(0, 0, -1), 0.02, [0.0], WeldArcModel.TOUCH_TOLERANCE, 0.003, 1);
      check(Math.abs(preparedProbe.approach.translation.z + 0.043 + probePlanner.airPoseReserve) < 1e-9 &&
        Math.abs(preparedProbe.distance - 0.043 - probePlanner.airPoseReserve) < 1e-9, "Prepared search covers both signs of uncertainty and the IK air-clearance reserve in metres");
      check(preparedProbe.approachJoints != null, "Preparation retains the checked sensing configuration");
      check(solver.discoveries == 0, "Observed-branch preparation proves the sensing corridor without global IK discovery");
      var minimumZ = -0.02 - (0.02 + 0.003 + probePlanner.airPoseReserve);
      var liftedMotion = new StandoffObstacleProbePlanner(arm, source, minimumZ);
      check(liftedMotion.violation([minimumZ]) != null, "The test fixture blocks the minimum probe standoff");
      var lifted = new processkit.ProbePosePlanner(liftedMotion).prepareObserved(new Vec3(0, 0, -0.02),
        new Vec3(0, 0, -1), 0.02, [0.0], WeldArcModel.TOUCH_TOLERANCE, 0.003, 1);
      check(Math.abs(lifted.approach.translation.z - (minimumZ - 0.005)) < 1e-9,
        'A blocked minimum standoff selects the next checked clearance height: got ${lifted.approach.translation.z}, expected ${minimumZ - 0.005}');
      check(Math.abs(lifted.approach.translation.z + lifted.distance) < 1e-9,
        "Lifting the probe start preserves the far edge of its CAD uncertainty interval");
      var local = screening.prepareObserved(new Vec3(0, 0, -0.02), new Vec3(0, 0, -1),
        0.02, [0.0], WeldArcModel.TOUCH_TOLERANCE, 0.003, 1);
      check(local.approachJoints != null && solver.discoveries == 0, "Observed-only preparation returns a checked sensing goal");
      var localRejected = false;
      try screening.prepareObserved(new Vec3(0, 0, -0.02), new Vec3(0, 1, 0),
        0.02, [0.0], WeldArcModel.TOUCH_TOLERANCE, 0.003, 1) catch (_:Dynamic) localRejected = true;
      check(localRejected && solver.discoveries == 0, "A failed local refinement never invokes global IK discovery");
      var globalRejected = false;
      try screening.prepare(new Vec3(0, 0, -0.02), new Vec3(0, 1, 0),
        0.02, [0.0], WeldArcModel.TOUCH_TOLERANCE, 0.003, 1) catch (_:Dynamic) globalRejected = true;
      check(globalRejected && solver.discoveries > 0, "Full preparation retains global discovery after local failure");
      var locked = probePlanner.approachJoints(cast preparedProbe.approachJoints, [0.0]);
      check(arm.tcpPose(locked.endJoints).translation.sub(preparedProbe.approach.translation).norm() < 0.0001,
        "Joint-goal execution preserves the prepared TCP approach");
      check(!probePlanner.corridorReachable([0.09], new Vec3(0, 0, 1), 0.04),
        "Execution rejects a sensing corridor beyond mechanical travel");
      var direct = probePlanner.observedApproach(new Transform3(new Vec3(0, 0, 0.03), Quat.identity()), [0.0]);
      check(Math.abs(direct.endJoints[0] - 0.03) < 5e-5, "Direct approach reaches the checked goal");
      check(preparedProbe.approach.rotation.rotate(new Vec3(0, 0, 1)).sub(preparedProbe.direction).norm() < 1e-9,
        "Prepared torch wire points into the CAD plane");
      var corridorRejected = false;
      try new processkit.ProbePosePlanner(probePlanner).prepare(new Vec3(0, 0, 0.095), new Vec3(0, 0, -1),
        0.02, [0.0], 0, 0.003, 1) catch (error:Dynamic) corridorRejected = Std.string(error).indexOf("corridor") >= 0;
      check(corridorRejected, "A reachable nominal contact cannot hide an unreachable uncertainty corridor");
      var approach = probePlanner.approach(new Transform3(new Vec3(0, 0, 0.03), Quat.identity()), [0.0], 32);
      check(Math.abs(approach.endJoints[0] - 0.03) < 5e-5, 'Checked approach ends at the requested search pose (${approach.endJoints[0]})');
      var retreat = probePlanner.line(Transform3.identity(), approach.endJoints, 0.01);
      check(Math.abs(retreat.endJoints[0]) < 5e-5, "Checked straight retreat returns along the probe direction");
      var forbidden = false;
      try probePlanner.approach(new Transform3(new Vec3(0, 0, 0.2), Quat.identity()), [0.0], 32) catch (_:Dynamic) forbidden = true;
      check(forbidden, "Probe approach rejects a target outside the mechanical travel");
      var badStart = false;
      try probePlanner.approachJoints([0.01], [0.11]) catch (error:Dynamic)
        badStart = Std.string(error).indexOf("start joint probe=0.11") >= 0;
      check(badStart, "Probe approach identifies an observed start outside its finite joint bounds");
      var badGoal = false;
      try probePlanner.approachJoints([0.11], [0.0]) catch (error:Dynamic)
        badGoal = Std.string(error).indexOf("IK goal joint probe=0.11") >= 0;
      check(badGoal, "Probe approach identifies a discovered IK goal outside its finite joint bounds");
      var roundoffGoal = probePlanner.approachJoints([0.1000000000000005], [0.0]);
      check(Math.abs(roundoffGoal.endJoints[0] - 0.1) < 1e-12,
        "Probe approach clamps representation-sized IK overshoot to the exact joint limit");
      var roundoffStart = probePlanner.approachJoints([0.08], [0.1000000000000005]);
      check(Math.abs(roundoffStart.endJoints[0] - 0.08) < 1e-12,
        "Probe approach clamps representation-sized observed-start overshoot to the exact joint limit");
      function cube(z:Float):Array<Float> return [for (x in [-0.001, 0.001]) for (y in [-0.001, 0.001])
        for (height in [z - 0.001, z + 0.001]) for (value in [x, y, height]) value];
      var contactFixture = new CollisionFixture(arm, [0.0], 0.003, CollisionClearance.CONTACT_MARGIN);
      contactFixture.mechanism("tool", tool.id, cube(0), true);
      contactFixture.fixture("fixture", cube(0.014612), true);
      var probeContact = contactFixture.clearance();
      var blocked = probeContact;
      var checkedProbe = new ProbeMotionPlanner(arm, planning.compiler, probeContact);
      var brakingHit = checkedProbe.stoppingViolation([0.012], [0.005], 0.02);
      var clearCorridor = checkedProbe.sensingCorridorViolation([0.0], new Vec3(0, 0, 1), 0.012);
      var blockedCorridor = checkedProbe.sensingCorridorViolation([0.0], new Vec3(0, 0, 1), 0.0122);
      check(brakingHit != null && brakingHit.a == "tool" && brakingHit.b == "fixture" &&
          Math.abs(brakingHit.required - CollisionClearance.CONTACT_MARGIN) < 1e-12,
        "Probe braking retains the standard tool-body contact clearance");
      check(clearCorridor == null && blockedCorridor != null && blockedCorridor.a == "tool" &&
          blockedCorridor.b == "fixture" && Math.abs(blockedCorridor.required - CollisionClearance.CONTACT_MARGIN) < 1e-12,
        "A probe candidate proves its swept sensing corridor at the standard contact clearance");
      var uncertainProbe = new processkit.ContactProbeRunner.ContactProbeRequest(Transform3.identity(),
        new Vec3(0, 0, 1), 0.032, 0.01, 0.0005, 0.002, 0.0005);
      var precontact = uncertainProbe.precontactDistance(0.022, 0.01);
      check(Math.abs(precontact - 0.0115) < 1e-12 &&
        checkedProbe.sensingCorridorViolation([0.0], new Vec3(0, 0, 1), precontact) == null &&
        checkedProbe.sensingCorridorViolation([0.0], new Vec3(0, 0, 1), 0.022) != null,
        "A surface inside contact uncertainty does not invalidate the guaranteed pre-contact corridor");
      var noAir = false;
      try uncertainProbe.precontactDistance(0.01, 0.01) catch (_:Dynamic) noAir = true;
      check(noAir, "A probe with no clearance before possible touch cannot be prepared");
      var wireScene = new robotkit.collision.CollisionSceneBuilder(arm, [0.0], "probe", 0.003);
      wireScene.add("fixture", robotkit.collision.CollisionSceneBuilder.CollisionOwner.Fixture, cube(0.015));
      var wire = processkit.ProbeWire.describe(wireScene, arm, tool.id, Transform3.identity(), 0.001, 0.015,
        [], wireScene.memberObjects(["fixture"]));
      var wireClearance = wireScene.clearance(() -> new NativeCollisionWorld());
      var wireGuarded = new ProbeMotionPlanner(arm, planning.compiler, wireClearance, wire);
      check(switch wireScene.description.objects[wire.object].geometry {
        case Cylinder(radius, halfLength): Math.abs(radius - 0.0005) < 1e-12 && Math.abs(2 * halfLength - 0.015) < 1e-12;
        case _: false;
      }, "The shared flat-ended cylinder has the declared wire diameter and complete stickout");
      var wireGap = wireClearance.closest([0.0]);
      check(wireGap != null && Math.abs(wireGap.distance - 0.014) < 1e-9,
        "Native collision geometry puts the flat-ended cylinder's distal surface at the physical sensing tip");
      var safeSpeed = wireGuarded.sensingSpeed([0.0], new Vec3(0, 0, 1), 0.04, 0.02, 0.0005);
      check(safeSpeed > 0 && safeSpeed < 0.02, "Calibration and CAD wire extent bound requested sensing speed");
      check(safeSpeed * 0.04 + safeSpeed * safeSpeed / 2 <= 0.0005 - planning.compiler.ikTolerance.position + 1e-10,
        "The derived prismatic speed fits the entire deadline and braking travel inside touch stand-off");
      var rimScene = new robotkit.collision.CollisionSceneBuilder(arm, [0.0], "cut-wire", 0.003);
      rimScene.addGeometry("rim obstacle", robotkit.collision.CollisionSceneBuilder.CollisionOwner.Fixture,
        collisionkit.CollisionGeometry.Sphere(0.000005), new Transform3(new Vec3(0.000499, 0, -0.000001), Quat.identity()));
      var cutWire = processkit.ProbeWire.describe(rimScene, arm, tool.id, Transform3.identity(), 0.001, 0.015,
        [], rimScene.memberObjects(["rim obstacle"]));
      var rimClearance = rimScene.clearance(() -> new NativeCollisionWorld());
      check(rimClearance.violation([0.0], true) != null,
        "the flat cut-end rim collides where a rounded capsule would miss the obstacle");
      rimClearance.world.dispose();
      var localSolver = new StepLimitedProbeSolver(arm, 0.0005);
      var localCompiler = new motionkit.robot.ProgramCompiler(localSolver, source.limits, source.frameId,
        source.maxVelocity, source.maxAcceleration, source.maxJerk, source.startTolerances,
        source.timing, source.cartesianResolution, source.maxJointJump, source.positionTolerance,
        source.orientationTolerance, source.ikTolerance, null, source.perJointMaxJump);
      var localPlanner = new ProbeMotionPlanner(arm, localCompiler);
      var locallyContinuedSpeed = localPlanner.sensingSpeed([0.0], new Vec3(0, 0, 1), 0.004, 0.02, 0.0005);
      check(locallyContinuedSpeed > 0 && localSolver.largestRequest > 0.0005 &&
        localSolver.largestAcceptedStep <= 0.0005000001,
        'Sensing speed refines the Cartesian step while staying on the observed IK branch (${localSolver.largestRequest}/${localSolver.largestAcceptedStep}, $locallyContinuedSpeed)');
      var unreachableSolver = new StepLimitedProbeSolver(arm, 0.00001);
      var unreachableCompiler = new motionkit.robot.ProgramCompiler(unreachableSolver, source.limits, source.frameId,
        source.maxVelocity, source.maxAcceleration, source.maxJerk, source.startTolerances,
        source.timing, source.cartesianResolution, source.maxJointJump, source.positionTolerance,
        source.orientationTolerance, source.ikTolerance, null, source.perJointMaxJump);
      var boundedFailure = false;
      try new ProbeMotionPlanner(arm, unreachableCompiler).sensingSpeed([0.0], new Vec3(0, 0, 1), 0.004, 0.02, 0.0005)
        catch (error:Dynamic) boundedFailure = Std.string(error).indexOf("minimum continuation step") >= 0;
      check(boundedFailure, "Sensing speed fails closed when bounded local IK continuation cannot advance");
      check(wireGuarded.sensingSpeed([0.0], new Vec3(0, 0, 1), 0.04, 0.02, 0.0005, 0.08) < safeSpeed,
        "A longer command deadline reduces safe sensing speed");
      check(wireGuarded.sensingSpeed([0.0], new Vec3(0, 0, 1), 0.04, 0.02, 0.001) > safeSpeed,
        "A larger calibrated stand-off permits faster sensing");
      var missingReserve = false;
      try wireGuarded.sensingSpeed([0.0], new Vec3(0, 0, 1), 0.04, 0.02, 0)
        catch (_:Dynamic) missingReserve = true;
      check(missingReserve, "A physical wire cannot invent a sensing stand-off when calibration is absent");
      missingReserve = false;
      try wireGuarded.sensingSpeed([0.0], new Vec3(0, 0, 1), 0.04, 0.02, 0.000000001)
        catch (_:Dynamic) missingReserve = true;
      check(missingReserve, "Calibration smaller than the geometry and IK reserve cannot authorize sensing");
      check(wireGuarded.violation([0.0]) == null, "The CAD wire is initially clear of the fixture");
      check(wireGuarded.violation([0.012]) != null, "Air clearance includes the wire beyond the nozzle");
      check(wireGuarded.violation([0.0135], true) == null, "Calibrated near-contact sensing permits the wire to approach without penetration");
      check(wireGuarded.violation([0.0145], true) != null, "Touch sensing cannot authorize wire penetration");
      var wireBrakingHit = wireGuarded.stoppingViolation([0.011], [0.1], 0.02);
      check(wireBrakingHit != null && ((wireBrakingHit.a == "contact sensing wire" && wireBrakingHit.b == "fixture") ||
          (wireBrakingHit.b == "contact sensing wire" && wireBrakingHit.a == "fixture")),
        'A rejected deadline brake retains the obstructing CAD bodies: $wireBrakingHit');
      check(!wireGuarded.stoppingClear([0.011], [0.1], 0.02), "Predicted braking also guards the unconsumed wire");
      forbidden = false;
      try wireGuarded.approach(new Transform3(new Vec3(0, 0, 0.06), Quat.identity()), [0.0], 32)
        catch (_:Dynamic) forbidden = true;
      check(forbidden, "Clear approach endpoints cannot hide a wire collision between them");
      forbidden = false;
      try wireGuarded.line(new Transform3(new Vec3(0, 0, 0.06), Quat.identity()), [0.0], 0.01)
        catch (_:Dynamic) forbidden = true;
      check(forbidden, "Compiled straight air motion checks the protruding wire");
      var guarded = new ProbeMotionPlanner(arm, planning.compiler, blocked);
      forbidden = false;
      try guarded.observedApproach(new Transform3(new Vec3(0, 0, 0.03), Quat.identity()), [0.0]) catch (_:Dynamic) forbidden = true;
      check(forbidden, "Direct preference cannot authorize an intervening fixture collision");
      check(guarded.stoppingClear([0.0], [0.0], 0.02), "A stationary clear probe has a safe braking sweep");
      check(guarded.stoppingClear([0.1000000000000005], [0.0], 0.02),
        "Stationary braking tolerates representation-sized sensor overshoot at a joint limit");
      check(!guarded.stoppingClear([0.1000001], [0.0], 0.02),
        "Stationary braking rejects a measured joint position beyond numerical limit tolerance");
      check(!guarded.stoppingClear([0.008], [0.1], 0.02), "A clear current posture can still have an obstructed braking sweep");
      forbidden = false;
      try guarded.approach(new Transform3(new Vec3(0, 0, 0.03), Quat.identity()), [0.0], 32) catch (_:Dynamic) forbidden = true;
      check(forbidden, "A clear approach endpoint does not authorize travel through a fixture");
      forbidden = false;
      try guarded.line(new Transform3(new Vec3(0, 0, 0.03), Quat.identity()), [0.0], 0.01) catch (_:Dynamic) forbidden = true;
      check(forbidden, "Compiled straight probe motion checks intervening clearance");
    }
    var blueprint = RobotRuntimeCompiler.compile(model, new robotkit.profile.RobotProfile());
    blueprint.sensors.push(new RobotRuntimeSensorBlueprint("torch", WeldSensor.KIND, "tip", "tool", 1,
      [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]));
    var harness = new SimulationHarness(0.01);
    var runtime = harness.simulation.addRobot(blueprint);
    var robot = new SimulatedRobot("probe", runtime, model.name, ["base", "tool"], ["probe"]);
    var servo = new ServoSession(robot, arm);
    var runner = new ContactSearchRunner(servo, "torch", new Vec3(0, 0, -1), 0.04, 0.005,
      0.02, 0.002, WeldArcModel.TOUCH_TOLERANCE);
    var sensing = new WeldArcModel({maxCurrentA: 300.0, efficiency: 0.85, wireDiameterMm: 1.2, stickoutMm: 15.0});
    var tick = 0;
    var firstTouch = 0.0;
    var touched = false;
    var fastest = 0.0;
    while (tick < 1200 && !runner.stopped) {
      harness.step(Int64.ofInt(tick));
      tick++;
      var snapshot = robot.snapshot();
      var q = snapshot.positions.get(0);
      fastest = Math.max(fastest, Math.abs(snapshot.velocities.get(0)));
      if (!dropFeedback || tick < 20) {
        var gap = noTouch ? 1.0 : Math.max(0.0, q + 0.02);
        var reading = sensing.step(0.01, {arcCommanded: false, voltageSet: 20.0, wireSpeed: 0.0,
          supplyReady: true, tipDistance: gap, wireDistance: gap});
        if (reading.touch && !touched) { firstTouch = q; touched = true; }
        runtime.publishSensorFrame("torch", WeldSensor.values(reading), Int64.ofInt(tick),
          snapshot.sourceTimestampNs, snapshot.sourceClockId);
      }
      runner.update();
    }
    check(runner.stopped, "Probe completes braking within a bounded number of ticks");
    check(Math.abs(robot.snapshot().velocities.get(0)) < 1e-5, "Search handoff requires observed joint rest before reporting stopped");
    check(fastest > 0.001 && fastest <= 0.0051, "Probe executes bounded inward native motion");
    if (dropFeedback || noTouch) {
      check(!runner.completed() && runner.search.failure != null && runner.search.contact == null,
        "Missing contact or stale feedback fails without a registration point");
      if (dropFeedback) check(robot.snapshot().positions.get(0) > -0.002, "Stale feedback stops well before the material");
    } else {
      check(runner.completed() && runner.search.failure == null, 'Executed contact succeeds: ${runner.search.failure}');
      var contact:Vec3 = cast runner.search.contact;
      check(Math.abs(contact.z + 0.02) < 0.00006, "Calibrated measured contact recovers the physical plane within one servo period");
      check(Math.abs(contact.z - (firstTouch - WeldArcModel.TOUCH_TOLERANCE)) < 1e-12,
        "Contact is captured at detection rather than at the later braking endpoint");
    }
    for (_ in 0...10) harness.step(Int64.ofInt(tick++));
    check(Math.abs(robot.snapshot().velocities.get(0)) < 1e-6, "The native joint is at rest after probing");
    servo.dispose(); harness.dispose();
  }
  static function completeProbe(registration:Bool = false, precision:Bool = false):Void {
    var fixture = axis(true);
    var model = fixture.model, arm = fixture.arm;
    var blueprint = RobotRuntimeCompiler.compile(model, new robotkit.profile.RobotProfile());
    var channels = {arc: "torch.arc", wireSpeed: "torch.wire", voltage: "torch.voltage"};
    blueprint.addTool(new processkit.tool.WeldChannels(channels.arc, channels.wireSpeed, channels.voltage));
    blueprint.sensors.push(new RobotRuntimeSensorBlueprint("torch", WeldSensor.KIND, "tip", "tool", 1,
      [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]));
    var harness = new SimulationHarness(0.01);
    var runtime = harness.simulation.addRobot(blueprint);
    var robot = new SimulatedRobot("complete-probe", runtime, model.name, ["base", "tool"], ["probe"]);
    var planning = WeldingPlanRunner.planning(arm, 1.0);
    var motion = new ManipulatorMotion(robot, planning.compiler, (_) -> null, () -> runtime.pollEvents(), [0]);
    var probe = new ContactProbeRunner(motion, new ProbeMotionPlanner(arm, planning.compiler), channels, "torch",
      () -> new ServoSession(robot, arm));
    var prepared = new Transform3(new Vec3(0, 0, 0.01), arm.tcpPose([0.0]).rotation);
    var requested = new ContactProbeRequest(prepared, new Vec3(0, 0, -1), 0.04, 0.005, 0.0005, 0.002,
      WeldArcModel.TOUCH_TOLERANCE);
    if (precision) requested = requested.withPrecision(0.000001, ContactProbeRunner.SEARCH_AGE_SECONDS);
    var registrationRunner = new processkit.ContactRegistrationRunner(probe,
      new processkit.perception.ContactRegistrationSequence(
        new processkit.perception.ContactPoseEnvelope(Transform3.identity(), new Vec3(0, 0, 0.001), new Vec3(), 0.00001),
        (count, _, _) -> {
          if (count != 3) throw "No second independent reachable plane";
          return new processkit.perception.ContactRegistrationSequence.ContactRegistrationStage(new Vec3(0, 0, 1), -0.02,
            [requested, requested, requested]);
        }));
    if (!registration) {
      probe.start(new ContactProbeRequest(prepared, new Vec3(0, 0, -1), 0.04, 0.005, 0.0005, 0.002,
        WeldArcModel.TOUCH_TOLERANCE, [0.05]));
      check(probe.failure != null && probe.failure.indexOf("does not match") >= 0 && !motion.running,
        "A mismatched prepared joint goal fails before any movement");
      harness.step(Int64.ofInt(0));
      probe.start(new ContactProbeRequest(prepared, new Vec3(0, 0, -1), 0.2));
      check(probe.failure != null && probe.failure.indexOf("corridor") >= 0 && !motion.running,
        "A manual request also proves its executed sensing corridor before movement");
      harness.step(Int64.ofInt(1));
    }
    if (registration) registrationRunner.start(); else probe.start(requested);
    var sensing = new WeldArcModel({maxCurrentA: 300.0, efficiency: 0.85, wireDiameterMm: 1.2, stickoutMm: 15.0});
    var tick = 0, safe = true, touchEpisodes = 0, touching = false, fineRangeChecked = false;
    while (tick < 7200 && (registration ? registrationRunner.running() : probe.running())) {
      harness.step(Int64.ofInt(tick++));
      var snapshot = robot.snapshot();
      var on = switch runtime.channelValue(channels.arc) { case Digital(value): value; default: true; };
      var feed = switch runtime.channelValue(channels.wireSpeed) { case Analog(value): value; default: -1.0; };
      safe = safe && !on && feed == 0;
      var gap = Math.max(0.0, snapshot.positions.get(0) + 0.02);
      var reading = sensing.step(0.01, {arcCommanded: on, voltageSet: 20.0, wireSpeed: feed,
        supplyReady: true, tipDistance: gap, wireDistance: gap});
      if (reading.touch && !touching) touchEpisodes++;
      touching = reading.touch;
      runtime.publishSensorFrame("torch", WeldSensor.values(reading), Int64.ofInt(tick), snapshot.sourceTimestampNs, snapshot.sourceClockId);
      if (registration) registrationRunner.update(0.01); else probe.update(0.01);
      if (!registration && !fineRangeChecked && Std.string(Reflect.field(probe, "phase")) == "Fine") {
        var fine:ContactSearchRunner = cast Reflect.field(probe, "search");
        check(fine != null && Math.abs(fine.search.distance - requested.backoff) < 1e-12,
          "Fine search reaches the corrected material point without adding the calibrated contact offset twice");
        fineRangeChecked = true;
      }
    }
    if (registration) {
      check(!registrationRunner.running() && !registrationRunner.completed() && registrationRunner.workFrame == null &&
        registrationRunner.failure == "No second independent reachable plane", "Unreachable later registration stage fails without a work frame");
      check(touchEpisodes == 6, "Three registration contacts each execute coarse and fine touch episodes");
      check(safe && Math.abs(robot.snapshot().velocities.get(0)) < 1e-5,
        "Registration failure leaves arc and wire off and the observed arm at rest");
      harness.dispose(); return;
    }
    check(probe.completed() && probe.failure == null, 'The complete checked/refined probe succeeds: ${probe.failure}, tick=$tick, q=${robot.snapshot().positions.get(0)}, contacts=$touchEpisodes');
    check(fineRangeChecked, "Fine contact refinement executes with its bounded material distance");
    var point:Vec3 = cast probe.contact;
    check(point != null && Math.abs(point.z + 0.02) < 0.000006, "Fine probing improves the calibrated observation to within 6 micrometres");
    if (precision) check(Math.abs(point.z + 0.02) <= 0.000001,
      "A precision-limited native probe meets its micrometre contact budget after a shortened coarse backoff");
    check(touchEpisodes == 2, "The executed probe withdraws and measures a fresh second contact");
    check(safe, "Arc and wire remain off throughout approach, sensing, refinement and retreat");
    check(Math.abs(robot.snapshot().positions.get(0) - 0.01) < 0.0001, "Completed probing returns to the prepared air pose");
    check(Math.abs(robot.snapshot().velocities.get(0)) < 1e-5, "Completed retreat leaves the joint at rest");
    harness.dispose();
  }
  static function sixAxisPreparation():Void {
    var fixture = WeldPlanningTests.arm();
    var planning = WeldingPlanRunner.planning(fixture.arm, 1.0);
    var prepared = new processkit.ProbePosePlanner(new ProbeMotionPlanner(fixture.arm, planning.compiler));
    var start = [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0];
    for (normal in [new Vec3(0, 0, 1), new Vec3(0, 1, 0), new Vec3(1, 0, 0)]) {
      var request = prepared.prepare(new Vec3(0.35, 0.2, 0.15), normal, 0.02, start, 0.0005);
      check(request.approach.rotation.rotate(new Vec3(0, 0, 1)).sub(normal.scale(-1)).norm() < 1e-8,
        "Six-axis preparation aligns the wire with each independent CAD normal");
      var speed = prepared.motion.sensingSpeed(cast request.approachJoints, request.direction, request.distance, 0.01, request.contactOffset);
      check(speed > 0 && speed <= 0.01, "Each checked six-axis sensing branch has a positive bounded stopping speed");
      var move = prepared.motion.approachJoints(cast request.approachJoints, start);
      check(fixture.arm.tcpPose(move.endJoints).translation.sub(request.approach.translation).norm() < 0.0001,
        "Six-axis checked approach reaches the prepared uncertain contact pose");
      var near = move.endJoints.copy(); near[0] += 0.001;
      var returned = prepared.motion.retreat(request.approach, near, 0.01, move.endJoints);
      for (joint in 0...move.endJoints.length) check(Math.abs(returned.endJoints[joint] - move.endJoints[joint]) < 1e-8,
        "Retreat restores the original checked joint configuration after Cartesian withdrawal");
    }
  }
  static function resetEpochs():Void {
    for (mode in 0...3) resetEpoch(mode);
  }
  static function resetEpoch(mode:Int):Void {
    var fixture = axis(true);
    var blueprint = RobotRuntimeCompiler.compile(fixture.model, new robotkit.profile.RobotProfile());
    var channels = {arc:"torch.arc", wireSpeed:"torch.wire", voltage:"torch.voltage"};
    blueprint.addTool(new processkit.tool.WeldChannels(channels.arc, channels.wireSpeed, channels.voltage));
    var harness = new SimulationHarness(0.01);
    var runtime = harness.simulation.addRobot(blueprint);
    var robot = new SimulatedRobot("reset-probe", runtime, fixture.model.name, ["base", "tool"], ["probe"]);
    if (mode == 0) {
      var servo = new ServoSession(robot, fixture.arm);
      var search = new ContactSearchRunner(servo, "torch", new Vec3(0, 0, -1), 0.04, 0.005);
      harness.resetRobot(0);
      var rejected = false;
      try search.update() catch (error:Dynamic) rejected = Std.string(error).indexOf("changed epoch") >= 0;
      check(rejected && search.search.contact == null, "Search aborts a reset even when numeric joint time repeats");
      servo.dispose(); harness.dispose(); return;
    }
    var planning = WeldingPlanRunner.planning(fixture.arm, 1.0);
    var clear = new CollisionFixture(fixture.arm, [0.0]).clearance();
    var stow = WeldStowPlanner.plan(fixture.arm, planning.compiler, clear, [0.03], [0.0]);
    check(stow.ops.length == 1 && switch stow.ops[0] {
      case motionkit.program.MotionOp.MoveJ(motionkit.program.MoveTarget.JointTarget(goal), _, _): Math.abs(goal[0]) < 1e-12;
      case _: false;
    }, "Live weld stow compiles a checked return from the measured configuration to CAD ready");
    var motion = new ManipulatorMotion(robot, planning.compiler, (_) -> null, () -> runtime.pollEvents(), [0]);
    var probe = new ContactProbeRunner(motion, new ProbeMotionPlanner(fixture.arm, planning.compiler), channels, "torch",
      () -> new ServoSession(robot, fixture.arm));
    var requested = new ContactProbeRequest(new Transform3(new Vec3(0, 0, 0.01), fixture.arm.tcpPose([0.0]).rotation),
      new Vec3(0, 0, -1), 0.04);
    if (mode == 1) {
      probe.start(requested);
      check(probe.failure == null, 'Reset test starts a valid probe: ${probe.failure}');
      harness.resetRobot(0); probe.update(0.01);
      check(probe.failure != null && probe.failure.indexOf("changed epoch") >= 0 && probe.contact == null,
        "Probe rejects a reset across buffered approach/refinement/retreat ownership");
      harness.dispose(); return;
    }
    var registration = new processkit.ContactRegistrationRunner(probe,
      new processkit.perception.ContactRegistrationSequence(
        new processkit.perception.ContactPoseEnvelope(Transform3.identity(), new Vec3(0, 0, 0.001), new Vec3(), 0.00001),
        (_, _, _) -> new processkit.perception.ContactRegistrationSequence.ContactRegistrationStage(new Vec3(0, 0, 1), 0,
          [requested, requested, requested])));
    registration.start();
    check(registration.failure == null, 'Reset test starts a valid registration: ${registration.failure}');
    harness.resetRobot(0); registration.update(0.01);
    check(registration.failure == "Contact registration joint clock changed epoch" && registration.workFrame == null,
      'Registration never combines contact points across endpoint reset epochs: ${registration.failure}');
    harness.dispose();
  }
  public static function main():Void {
    fiveAxisPreparation();
    checkedJointRetreatFallback();
    run(false, false); run(true, false); run(false, true); completeProbe(); completeProbe(true); completeProbe(false, true);
    registrationReturnsToStart(); sixAxisPreparation(); resetEpochs();
    Sys.println('Contact search native motion: $checks assertions passed');
  }

  static function registrationReturnsToStart():Void {
    var fixture = axis();
    var blueprint = RobotRuntimeCompiler.compile(fixture.model, new robotkit.profile.RobotProfile());
    var channels = {arc: "torch.arc", wireSpeed: "torch.wire", voltage: "torch.voltage"};
    blueprint.addTool(new processkit.tool.WeldChannels(channels.arc, channels.wireSpeed, channels.voltage));
    var harness = new SimulationHarness(0.01);
    var runtime = harness.simulation.addRobot(blueprint);
    var robot = new SimulatedRobot("registration-return", runtime, fixture.model.name, ["base", "tool"], ["probe"]);
    var planning = WeldingPlanRunner.planning(fixture.arm, 1.0);
    var motion = new ManipulatorMotion(robot, planning.compiler, (_) -> null, () -> runtime.pollEvents(), [0]);
    var probe = new RegistrationReturnProbe(motion, new ProbeMotionPlanner(fixture.arm, planning.compiler), channels,
      "torch", () -> new ServoSession(robot, fixture.arm));
    var points = [new Vec3(-0.1, -0.1, 0), new Vec3(0.1, -0.1, 0), new Vec3(0.1, 0.1, 0),
      new Vec3(-0.1, 0.02, 0.05), new Vec3(0.1, 0.02, 0.05), new Vec3(0.04, -0.04, 0.05)];
    var normals = [new Vec3(0, 0, 1), new Vec3(0, 1, 0), new Vec3(1, 0, 0)];
    var selectionStarts:Array<Float> = [];
    var sequence = new processkit.perception.ContactRegistrationSequence(
      new processkit.perception.ContactPoseEnvelope(Transform3.identity(), new Vec3(0.1, 0.1, 0.1),
        new Vec3(0.1, 0.1, 0.1), 0.00001), (count, _, _) -> {
          selectionStarts.push(robot.snapshot().positions.get(0));
          var first = count == 3 ? 0 : count == 2 ? 3 : 5;
          var normal = normals[3 - count];
          return new processkit.perception.ContactRegistrationSequence.ContactRegistrationStage(normal,
            normal.dot(points[first]), [for (index in first...first + count)
              new ContactProbeRequest(new Transform3(points[index], Quat.identity()), new Vec3(0, 0, -1), 0.1)],
            count == 2 ? [0.03] : null);
        });
    var registration = new ContactRegistrationRunner(probe, sequence);
    var initial = robot.snapshot().positions.get(0);
    registration.start();
    var tick = 0;
    while (registration.running() && tick < 12000) {
      harness.step(Int64.ofInt(tick++));
      registration.update(0.01);
    }
    check(registration.completed() && registration.failure == null && registration.workFrame != null,
      'Accepted registration completes only after returning to its start: ${registration.failure}');
    check(probe.requests == 6, "Registration executes all 3–2–1 contacts before restoring the arm");
    check(selectionStarts.length == 3 && Math.abs(selectionStarts[0]) < 1e-9 &&
      Math.abs(selectionStarts[1] - 0.03) < 0.005 && Math.abs(selectionStarts[2] - 0.03) < 0.005,
      "Each next plane is selected from completed measurements before moving to its selected entry");
    check(probe.starts.length == 6 && Math.abs(probe.starts[0]) < 0.005 &&
      Math.abs(probe.starts[3] - 0.03) < 0.005 && Math.abs(probe.starts[5]) < 0.005,
      "Probes begin only after their checked stage entry, including return to the saved start");
    check(Math.abs(robot.snapshot().positions.get(0) - initial) < 0.005 &&
      Math.abs(robot.snapshot().velocities.get(0)) < 1e-5,
      "Accepted contact registration restores the measured starting posture at rest");
    harness.dispose();
  }

  static function fiveAxisPreparation():Void {
    var model = new RobotModel("five-axis-probe");
    var links = [for (i in 0...6) model.addLink(new Link('link-$i'))];
    var axes = [[1.0,0,0], [0.0,1,0], [0.0,0,1], [0.0,0,1], [0.0,1,0]];
    for (i in 0...5) {
      var joint = model.addJoint(new Joint('joint-$i', i < 3 ? JointType.Prismatic : JointType.Revolute,
        links[i], links[i+1]));
      joint.axis = axes[i];
      joint.limits.lower = i < 3 ? -1.0 : -Math.PI;
      joint.limits.upper = i < 3 ? 1.0 : Math.PI;
      joint.limits.velocity = 0.5;
      joint.limits.maxAcceleration = 1.0;
    }
    var flange = model.addFrame(new Frame("tip", links[5]));
    var arm = new Manipulator(model, links[0].id, flange.id);
    var planning = WeldingPlanRunner.planning(arm, 1.0);
    var source = planning.compiler;
    var solver = new ObservedProbeSolver(arm);
    var compiler = new motionkit.robot.ProgramCompiler(solver, source.limits, source.frameId,
      source.maxVelocity, source.maxAcceleration, source.maxJerk, source.startTolerances,
      source.timing, source.cartesianResolution, source.maxJointJump, source.positionTolerance,
      source.orientationTolerance, source.ikTolerance, null, source.perJointMaxJump);
    compiler.planCheck = source.planCheck;
    var motion = new ProbeMotionPlanner(arm, compiler);
    var preparation = new processkit.ProbePosePlanner(motion);
    var point = new Vec3(0.2, 0.1, 0.05);
    var outward = new Vec3(1,1,-1).normalized();
    var start = [0.0,0,0,0,0];
    check(preparation.hasClearObservedApproachConfiguration(point, outward, 0.01, start),
      "Five-axis contact screening constrains the wire axis without requiring independent tool spin");
    var prepared = preparation.prepareObserved(point, outward, 0.01, start, 0.0005, 0.003, 1);
    check(solver.discoveries == 0, "Five-axis preparation uses observed continuation only");
    var actual = arm.tcpPose(cast prepared.approachJoints);
    check(actual.rotation.rotate(new Vec3(0,0,1)).dot(outward.scale(-1)) >= Math.cos(compiler.ikTolerance.orientation),
      "Selected roll retains the inward wire axis tolerance");
    check(actual.translation.sub(prepared.approach.translation).norm() < 1e-9
      && actual.rotation.angularDistance(prepared.approach.rotation) < 1e-9,
      "Preparation locks the actual checked pose for execution");
    var spin = actual.rotation.multiply(Quat.fromAxisAngle(new Vec3(0,0,1), 0.3));
    var fixed = new motionkit.kinematics.Pose3(actual.translation.x, actual.translation.y,
      actual.translation.z, spin.x, spin.y, spin.z, spin.w);
    check(solver.solvePose(fixed, cast prepared.approachJoints, compiler.ikTolerance) == null,
      "An unreachable independent spin does not invalidate the reachable contact-axis task");
    var replay = motion.approachJoints(cast prepared.approachJoints, start);
    check(motion.corridorReachable(replay.endJoints, outward.scale(-1), prepared.distance),
      "The locked five-axis goal retains a fully checked sensing corridor");
  }

  static function checkedJointRetreatFallback():Void {
    var fixture = axis();
    var planning = WeldingPlanRunner.planning(fixture.arm, 1.0);
    var source = planning.compiler;
    var solver = new RetreatProbeSolver(fixture.arm);
    var compiler = new motionkit.robot.ProgramCompiler(solver, source.limits, source.frameId,
      source.maxVelocity, source.maxAcceleration, source.maxJerk, source.startTolerances,
      source.timing, source.cartesianResolution, source.maxJointJump, source.positionTolerance,
      source.orientationTolerance, source.ikTolerance, null, source.perJointMaxJump);
    var planner = new ProbeMotionPlanner(fixture.arm, compiler);
    var goal = [0.01], contact = [0.02];
    var returned = planner.retreat(fixture.arm.tcpPose(goal), contact, 0.01, goal);
    check(returned.program.ops.length == 1 && switch returned.program.ops[0] {
      case motionkit.program.MotionOp.MoveJ(_ , _, _): true;
      case _: false;
    }, "A checked joint retreat replaces a Cartesian path that the solver cannot compile");
    check(Math.abs(returned.endJoints[0] - goal[0]) < 5e-5,
      "The fallback restores the checked air configuration after sweeping the contact-aware joint path");
  }
}

private class ObservedProbeSolver extends motionkit.robot.ManipulatorKinematics {
  public var discoveries:Int = 0;
  public function new(arm:Manipulator) {
    super(arm, 1e-8);
    preferTargetOrientation = true;
  }
  override public function sampleCandidates(target:motionkit.kinematics.Pose3, maxCount:Int,
      tolerance:motionkit.kinematics.IkTolerance,
      ?freedom:motionkit.path.OrientationPolicy):Array<Array<Float>> {
    discoveries++;
    return super.sampleCandidates(target, maxCount, tolerance, freedom);
  }
}

private class StandoffObstacleProbePlanner extends ProbeMotionPlanner {
  final blockedZ:Float;
  public function new(arm:Manipulator, compiler:motionkit.robot.ProgramCompiler, blockedZ:Float) {
    super(arm, compiler);
    this.blockedZ = blockedZ;
  }
  override public function violation(q:Array<Float>, contact:Bool = false):Null<robotkit.manipulation.ClearanceViolation> {
    return q.length == 1 && Math.abs(q[0] - blockedZ) < 0.00015
      ? {a: "probe", b: "fixture", distance: 0.0, required: 0.003} : null;
  }
  override public function observedApproachJoints(goal:Array<Float>, start:Array<Float>):processkit.ProbeMotionPlanner.CheckedProbeMove {
    if (violation(goal) != null) throw "Probe approach goal is blocked by the fixture";
    var move = motionkit.program.MotionOp.MoveJ(motionkit.program.MoveTarget.JointTarget(goal),
      new motionkit.MotionOptions(), motionkit.program.Blend.ExactStop);
    return new processkit.ProbeMotionPlanner.CheckedProbeMove(new motionkit.program.MotionProgram([move]), goal);
  }
  override public function corridorReachable(start:Array<Float>, direction:Vec3, distance:Float):Bool return true;
}

private class RetreatProbeSolver extends motionkit.robot.ManipulatorKinematics {
  public function new(arm:Manipulator) {
    super(arm);
  }
  override public function solvePathWithRates(request:motionkit.kinematics.PathRequest):motionkit.kinematics.PathSolution
    return new motionkit.kinematics.PathSolution([for (_ in request.poses) null]);
}

private class StepLimitedProbeSolver extends motionkit.robot.ManipulatorKinematics {
  final arm:Manipulator;
  final maximumStep:Float;
  public var largestRequest(default, null):Float = 0.0;
  public var largestAcceptedStep(default, null):Float = 0.0;
  public function new(arm:Manipulator, maximumStep:Float) {
    super(arm, 1e-8);
    this.arm = arm;
    this.maximumStep = maximumStep;
  }
  override public function solvePose(target:motionkit.kinematics.Pose3, seed:Array<Float>,
      tolerance:motionkit.kinematics.IkTolerance,
      ?freedom:motionkit.path.OrientationPolicy):Null<Array<Float>> {
    var current = arm.tcpPose(seed).translation;
    var requested = new Vec3(target.x, target.y, target.z).sub(current).norm();
    largestRequest = Math.max(largestRequest, requested);
    if (requested > maximumStep + 1e-12) return null;
    largestAcceptedStep = Math.max(largestAcceptedStep, requested);
    return super.solvePose(target, seed, tolerance, freedom);
  }
}

private class RegistrationReturnProbe extends ContactProbeRunner {
  public var requests(default, null):Int = 0;
  public final starts:Array<Float> = [];
  var fakeRunning = false;
  var fakeCompleted = false;
  var measured:Null<Vec3> = null;

  public function new(motion:ManipulatorMotion, planner:ProbeMotionPlanner,
      channels:processkit.WelderProcessDevice.WelderChannels, sensor:String, makeServo:Void -> ServoSession) {
    super(motion, planner, channels, sensor, makeServo);
  }

  override public function start(request:ContactProbeRequest):Void {
    if (fakeRunning) throw "Fake registration probe is already running";
    starts.push(motion.robot.snapshot().positions.get(0));
    requests++;
    measured = request.approach.translation;
    Reflect.setField(this, "contact", null);
    Reflect.setField(this, "failure", null);
    fakeCompleted = false;
    if (requests == 1) {
      motion.reset();
      motion.run(new motionkit.program.MotionProgram([
        motionkit.program.MotionOp.MoveJ(motionkit.program.MoveTarget.JointTarget([0.03]),
          new motionkit.MotionOptions(), motionkit.program.Blend.ExactStop)]));
      fakeRunning = true;
    } else {
      Reflect.setField(this, "contact", measured);
      fakeCompleted = true;
    }
  }

  override public function update(dt:Float):Void {
    if (!fakeRunning) return;
    motion.update(dt);
    if (motion.failure != null) {
      Reflect.setField(this, "failure", motion.failure);
      fakeRunning = false;
    } else if (motion.completed) {
      Reflect.setField(this, "contact", measured);
      fakeRunning = false;
      fakeCompleted = true;
    }
  }

  override public function running():Bool return fakeRunning;
  override public function completed():Bool return fakeCompleted;

  override public function cancel():Void {
    fakeRunning = false;
    fakeCompleted = false;
    try motion.abort() catch (_:Dynamic) {}
  }
}
