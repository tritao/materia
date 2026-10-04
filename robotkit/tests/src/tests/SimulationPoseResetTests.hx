package tests;

import robotkit.runtime.RobotRuntimeBlueprint;
import robotkit.runtime.Simulation;
import robotkit.runtime.SimulationHarness;
import robotkit.runtime.RobotRuntimeCompiler;
import robotkit.model.RobotModel;
import robotkit.model.Link;
import robotkit.model.Joint;
import robotkit.model.JointType;
import robotkit.model.JointLimits;
import robotkit.model.JointCoupling;
import robotkit.model.Actuator;
import robotkit.model.ActuatorDrive.ServoDrive;
import robotkit.model.Transmission;
import haxe.Int64;
import robotkit.tool.ToolCollisionShape;
import robotkit.spatial.Vec3;

class SimulationPoseResetTests {
  public static function run(backend:Int):Void {
    var position = [1.25, -2.5, 0.75];
    var rotation = [Math.sin(0.3), 0.0, 0.0, Math.cos(0.3)];
    var simulationHarness = new SimulationHarness(0.01, 1, backend);

    var simulation = simulationHarness.simulation;
    var piece:Array<Float> = [];
    for (index in 0...8) {
      piece.push((index & 1) == 0 ? -0.01 : 0.01);
      piece.push((index & 2) == 0 ? -0.01 : 0.01);
      piece.push((index & 4) == 0 ? 0.0 : 0.02);
    }
    simulation.addRobotAtPose(new RobotRuntimeBlueprint(1, 0, 1), position, rotation,
      null, null, null, null, ToolCollisionShape.Hulls([piece], 0.005), 0);
    simulationHarness.teleportRobot(0, [4.0, 5.0, 6.0]);
    simulationHarness.resetRobot(0);
    checkPose(simulation, position, rotation, 'resetRobot on backend $backend');
    simulationHarness.teleportRobot(0, [7.0, 8.0, 9.0]);
    simulationHarness.reset();
    checkPose(simulation, position, rotation, 'reset on backend $backend');
    simulationHarness.dispose();
    var boxSimulationHarness = new SimulationHarness(0.01, 1, backend);

    var boxSimulation = boxSimulationHarness.simulation;
    boxSimulation.addRobotAtPose(new RobotRuntimeBlueprint(2, 0, 1), [0, 0, 0],
      [0, 0, 0, 1], null, null, null, null,
      ToolCollisionShape.Box(new Vec3(0.01, 0.01, 0.02), new Vec3(0, 0, 0.02)), 0);
    boxSimulationHarness.step(Int64.ofInt(0));
    boxSimulationHarness.dispose();
    if (backend == 1) toolProximity();
    driveReferenceCompilation();
    coupling(backend);
    stepperSlip(backend);
    if (backend == 1) servoCoupling();
  }

  /** Explicit gains opt a geared joint into its physical drive, without
   * reflecting its already-joint-coordinate inertia through the reduction twice.
   */
  static function driveReferenceCompilation():Void {
    var model = new RobotModel("explicit servo loop");
    var base = model.addLink(new Link("base"));
    var load = model.addLink(new Link("load"));
    load.mass = 1.0;
    load.inertiaTensor = [0.2, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.2];
    var joint = model.addJoint(new Joint("joint", JointType.Revolute, base, load));
    joint.armature = 0.05;
    var actuator = new Actuator("servo", 0, 0, Transmission.SimpleTransmission(joint.id, 100, 0));
    actuator.drive = new ServoDrive(1, 2, 100, 200, 5e-6, 4096);
    actuator.positionLoopRate = 4000;
    model.addActuator(actuator);
    var legacy = RobotRuntimeCompiler.compile(model, new robotkit.profile.RobotProfile());
    if (legacy.joints[0].servoStiffness != 0 || legacy.fastestPositionLoopRate() != 0)
      throw "Unspecified uncoupled gains must retain the existing tracking model";
    actuator.servoStiffness = 200;
    actuator.servoDamping = 1;
    var physical = RobotRuntimeCompiler.compile(model, new robotkit.profile.RobotProfile());
    if (physical.joints[0].servoStiffness != 2e6 || physical.joints[0].servoDamping != 1e4 ||
        physical.fastestPositionLoopRate() != 4000)
      throw "Explicit motor gains must be reflected to the joint drive";
    if (Math.abs(physical.joints[0].reflectedInertia - 0.25) > 1e-12)
      throw "An uncoupled geared joint must not reflect its load inertia twice";
  }

  /** A joint a stepper lost steps on sits its slip behind the command, and what it turns with goes along. */
  static function stepperSlip(backend:Int):Void {
    var model = new RobotModel("slipping pair");
    var base = model.addLink(new Link("base"));
    var first = model.addLink(new Link("first"));
    var second = model.addLink(new Link("second"));
    var source = model.addJoint(new Joint("source", JointType.Revolute, base, first));
    var follower = model.addJoint(new Joint("follower", JointType.Revolute, base, second));
    source.limits = new JointLimits(-2, 2);
    follower.limits = new JointLimits(-2, 2);
    model.addCoupling(new JointCoupling("gears", source.id, follower.id, -2.0, 0.0));
    var harness = new SimulationHarness(0.01, 1, backend);
    var runtime = harness.simulation.addRobot(RobotRuntimeCompiler.compile(model, new robotkit.profile.RobotProfile()));
    runtime.submitPosition(0, 0.3, 1);
    for (index in 0...100) harness.step(Int64.ofInt(index));
    var tolerance = backend == 0 ? 1e-9 : 0.02;
    var q = runtime.snapshot().q;
    if (Math.abs(q.get(0) - 0.3) > tolerance) throw 'pair did not reach its command on backend $backend: ${q.get(0)}';
    harness.simulation.setJointSlip(0, 0, -0.05);
    for (index in 100...300) harness.step(Int64.ofInt(index));
    q = runtime.snapshot().q;
    if (Math.abs(q.get(0) - 0.25) > tolerance || Math.abs(q.get(1) + 2.0 * q.get(0)) > tolerance)
      throw 'slip did not hold the joint behind its command on backend $backend: ${q.get(0)}, ${q.get(1)}';
    harness.simulation.setJointSlip(0, 0, 0.0);
    for (index in 300...500) harness.step(Int64.ofInt(index));
    if (Math.abs(runtime.snapshot().q.get(0) - 0.3) > tolerance)
      throw 'clearing the slip did not put the joint back on its command on backend $backend';
    harness.dispose();
  }

  /** A servo on a motor joint carries the joint coupled to it, which takes no commands of its own. */
  static function servoCoupling():Void {
    var build = (peak:Float) -> {
      var model = new RobotModel("servo pair");
      var base = model.addLink(new Link("base"));
      var load = model.addLink(new Link("load"));
      var rotor = model.addLink(new Link("rotor"));
      load.mass = 1.0;
      load.inertiaTensor = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01];
      rotor.mass = 0.1;
      rotor.inertiaTensor = [1e-4, 0.0, 0.0, 0.0, 1e-4, 0.0, 0.0, 0.0, 1e-4];
      var arm = model.addJoint(new Joint("arm", JointType.Revolute, base, load));
      var motor = model.addJoint(new Joint("motor", JointType.Continuous, base, rotor));
      arm.limits = new JointLimits(-2, 2);
      motor.limits = new JointLimits(-100, 100);
      // The arm turns once for five turns of the motor.
      model.addCoupling(new JointCoupling("gearbox", "arm", "motor", 5.0, 0.0));
      var actuator = new Actuator("servo", null, null, Transmission.SimpleTransmission("motor", 1.0, 0.0));
      actuator.drive = new ServoDrive(peak / 2.0, peak, 100.0, 200.0, 1e-4, 4096.0);
      model.addActuator(actuator);
      model.materializeLimits();
      return model;
    };
    var compiled = RobotRuntimeCompiler.compile(build(1.2), new robotkit.profile.RobotProfile());
    if (!(compiled.joints[1].servoStiffness > 0.0) || compiled.joints[0].servoStiffness != 0.0)
      throw 'only the motor joint is a servo: ${compiled.joints[0].servoStiffness}, ${compiled.joints[1].servoStiffness}';
    if (compiled.joints[1].maxEffort != 1.2) throw 'the servo limits its joint to its peak torque: ${compiled.joints[1].maxEffort}';
    var harness = new SimulationHarness(0.001, 1, 1);
    var runtime = harness.simulation.addRobot(compiled);
    // The plan commands the whole chain, as a trajectory does: the arm to 0.3, its motor to 1.5.
    runtime.submitPositions([0.3, 1.5], 1);
    for (index in 0...1500) harness.step(Int64.ofInt(index));
    var q = runtime.snapshot().q;
    if (Math.abs(q.get(0) - 0.3) > 0.02 || Math.abs(q.get(1) - 1.5) > 0.1)
      throw 'the servo did not carry its coupled joint to the command: ${q.get(0)}, ${q.get(1)}';
    harness.dispose();
    // The arm alone is not commanded: with no target for the motor it stays put.
    var idle = new SimulationHarness(0.001, 1, 1);
    var held = idle.simulation.addRobot(RobotRuntimeCompiler.compile(build(1.2), new robotkit.profile.RobotProfile()));
    held.submitPosition(0, 0.3, 1);
    for (index in 0...500) idle.step(Int64.ofInt(index));
    if (Math.abs(held.snapshot().q.get(0)) > 0.02)
      throw 'a joint moved only through its coupling must not be commanded: ${held.snapshot().q.get(0)}';
    idle.dispose();
  }

  static function toolProximity():Void {
    var simulationHarness = new SimulationHarness(0.01, 2, 1);

    var simulation = simulationHarness.simulation;
    var cup:Array<Float> = [];
    for (index in 0...8) {
      cup.push((index & 1) == 0 ? -0.01 : 0.01);
      cup.push((index & 2) == 0 ? -0.01 : 0.01);
      cup.push((index & 4) == 0 ? 0.0 : 0.02);
    }
    var runtime = simulation.addRobotAtPose(new RobotRuntimeBlueprint(1, 0, 1),
      [0, 0, 0], [0, 0, 0, 1], null, null, null, null,
      ToolCollisionShape.Hulls([cup], 0.03), 0);
    var obstacle = simulationHarness.spawnBox([0, 0, 0.05], [0.01, 0.01, 0.01]);
    simulationHarness.step(Int64.ofInt(0));
    var contacts = runtime.toolProximity();
    if (contacts.length == 0 || contacts[0].toolPieceIndex != 0 || contacts[0].active ||
        contacts[0].otherObject != obstacle.handle.rawValue())
      throw "Tool cup proximity was not reported";
    simulationHarness.dispose();
  }

  static function coupling(backend:Int):Void {
    var model = new RobotModel("coupled joints");
    var base = model.addLink(new Link("base"));
    var first = model.addLink(new Link("first"));
    var second = model.addLink(new Link("second"));
    var source = model.addJoint(new Joint("source", JointType.Revolute, base, first));
    var follower = model.addJoint(new Joint("follower", JointType.Revolute, base, second));
    source.limits = new JointLimits(-2, 2);
    follower.limits = new JointLimits(-2, 2);
    model.addCoupling(new JointCoupling("gears", source.id, follower.id, -2.0, 0.0));
    var simulationHarness = new SimulationHarness(0.01, 1, backend);

    var simulation = simulationHarness.simulation;
    var runtime = simulation.addRobot(RobotRuntimeCompiler.compile(model, new robotkit.profile.RobotProfile()));
    // A plan leaving the follower out turns it with its source, so the runtime names the pair.
    if (runtime.couplings.length != 1 || runtime.couplings[0].leader != 0 ||
        runtime.couplings[0].follower != 1 || runtime.couplings[0].ratio != -2.0)
      throw 'runtime did not report its coupling on backend $backend';
    runtime.submitPosition(0, 0.3, 1);
    for (index in 0...200) simulationHarness.step(Int64.ofInt(index));
    var q = runtime.snapshot().q;
    var tolerance = backend == 0 ? 1e-9 : 0.02;
    if (!(Math.abs(q.get(0)) > 0.1 && Math.abs(q.get(1) + 2.0 * q.get(0)) < tolerance))
      throw 'coupling did not follow source on backend $backend: ${q.get(0)}, ${q.get(1)}';
    simulationHarness.dispose();
  }

  /** A joint is as fast as every joint that turns with it allows: a lead screw caps its axis. */
  public static function coupledLimits():Void {
    var model = new RobotModel("screw axis");
    var base = model.addLink(new Link("base"));
    var carriage = model.addLink(new Link("carriage"));
    var screw = model.addLink(new Link("screw"));
    var pulley = model.addLink(new Link("pulley"));
    var axis = model.addJoint(new Joint("axis", JointType.Prismatic, base, carriage));
    var turn = model.addJoint(new Joint("turn", JointType.Revolute, base, screw));
    var belt = model.addJoint(new Joint("belt", JointType.Revolute, base, pulley));
    axis.limits = new JointLimits(0, 0.3, 0.08, 400, 0.5);
    axis.limits.overtravel = 0.005;
    // 2 mm lead: pi * 1000 rad per metre of travel. The screw turns at most 100 rad/s.
    turn.limits = new JointLimits(-1e9, 1e9, 100);
    // A belt off the screw at 1:2, limited to 150 rad/s and 2000 rad/s², with no limit of its own on speed below.
    belt.limits = new JointLimits(-1e9, 1e9, 150, null, 2000);
    model.addCoupling(new JointCoupling("lead", "axis", "turn", -Math.PI * 1000, 0.0));
    model.addCoupling(new JointCoupling("belt", "turn", "belt", 2.0, 0.0));
    var limits = model.coupledLimits("axis");
    var screwSpeed = 100 / (Math.PI * 1000), beltSpeed = 150 / (2 * Math.PI * 1000);
    if (Math.abs(limits.requireVelocity() - Math.min(screwSpeed, beltSpeed)) > 1e-12)
      throw 'coupled velocity limit ${limits.requireVelocity()}';
    if (Math.abs(limits.requireAcceleration() - Math.min(0.5, 2000 / (2 * Math.PI * 1000))) > 1e-12)
      throw 'coupled acceleration limit ${limits.requireAcceleration()}';
    if (limits.lower != 0 || limits.upper != 0.3 || limits.effort != 400 || limits.overtravel != 0.005)
      throw "coupled limits must keep the joint's own travel, effort and overtravel";
    if (axis.limits.requireVelocity() != 0.08) throw "coupled limits must not change the model";
    // A joint with no limit of its own takes its followers'.
    axis.limits.velocity = null;
    if (Math.abs(model.coupledLimits("axis").requireVelocity() - Math.min(screwSpeed, beltSpeed)) > 1e-12)
      throw "an unlimited joint should take its followers' limit";
    axis.limits.velocity = 0;
    if (model.coupledLimits("axis").requireVelocity() != 0) throw "a stated zero velocity must remain a stopped joint";
    axis.limits.velocity = null;
    if (model.coupledLimits("belt").requireVelocity() != 150) throw "a follower is not limited by its leader";

    // A motor on the screw: 0.63 N m up to 137 rad/s, a 3e-5 kg m² rotor, through a 40% screw.
    var motorModel = new RobotModel("motor axis");
    var frame = motorModel.addLink(new Link("frame"));
    var table = motorModel.addLink(new Link("table"));
    var rotor = motorModel.addLink(new Link("rotor"));
    table.mass = 10.0;
    rotor.mass = 0.2;
    rotor.centerOfMass = [0.0, 0.0, 0.0];
    rotor.inertiaTensor = [1e-5, 0.0, 0.0, 0.0, 1e-5, 0.0, 0.0, 0.0, 4e-6];
    var slide = motorModel.addJoint(new Joint("slide", JointType.Prismatic, frame, table));
    var screwJoint = motorModel.addJoint(new Joint("screw", JointType.Continuous, frame, rotor));
    slide.limits = new JointLimits(0, 0.3);
    screwJoint.limits = new JointLimits(-1e9, 1e9);
    screwJoint.axis = [0.0, 0.0, 1.0];
    screwJoint.armature = 3e-5;
    var screwLead = new JointCoupling("lead", "slide", "screw", Math.PI * 1000, 0.0);
    screwLead.efficiency = 0.4;
    motorModel.addCoupling(screwLead);
    motorModel.addActuator(new robotkit.model.Actuator("motor", 0.63, 137.0,
      robotkit.model.Transmission.SimpleTransmission("screw", 1.0, 0.0)));
    var driven = motorModel.coupledLimits("slide");
    var scale = Math.PI * 1000;
    if (Math.abs(driven.requireVelocity() - 137.0 / scale) > 1e-12)
      throw 'a motor caps its axis at its rate through the screw: ${driven.requireVelocity()}';
    // a = eta s T / (m + eta (J_screw + J_rotor) s²), with the screw's own 4e-6 about its axis.
    var expected = 0.4 * scale * 0.63 / (10.0 + 0.4 * (4e-6 + 3e-5) * scale * scale);
    if (Math.abs(driven.requireAcceleration() - expected) > expected * 1e-12)
      throw 'a motor accelerates its axis by its force over mass and turning inertia: ${driven.requireAcceleration()}, expected $expected';
    // Two motors, one per screw, as on a gantry's two sides: twice the force and twice the turning inertia.
    var second = motorModel.addLink(new Link("rotor2"));
    second.mass = 0.2;
    second.inertiaTensor = rotor.inertiaTensor.copy();
    var other = motorModel.addJoint(new Joint("screw2", JointType.Continuous, frame, second));
    other.limits = new JointLimits(-1e9, 1e9);
    other.armature = 3e-5;
    var otherLead = new JointCoupling("lead2", "slide", "screw2", -Math.PI * 1000, 0.0);
    otherLead.efficiency = 0.4;
    motorModel.addCoupling(otherLead);
    motorModel.addActuator(new robotkit.model.Actuator("motor2", 0.63, 137.0,
      robotkit.model.Transmission.SimpleTransmission("screw2", 1.0, 0.0)));
    var pair = 2 * 0.4 * scale * 0.63 / (10.0 + 2 * 0.4 * (4e-6 + 3e-5) * scale * scale);
    if (Math.abs(motorModel.coupledLimits("slide").requireAcceleration() - pair) > pair * 1e-12)
      throw "two motors add their force and their screws' turning inertia";
    // A servo with no explicit limits is held to its peak torque and maximum speed: 1.5 N m, 200 rad/s.
    motorModel.actuators.splice(0, motorModel.actuators.length);
    var servo = new robotkit.model.Actuator("servo", null, null, robotkit.model.Transmission.SimpleTransmission("screw", 1.0, 0.0));
    servo.drive = new robotkit.model.ActuatorDrive.ServoDrive(0.5, 1.5, 100.0, 200.0, 3e-5, 4096.0);
    motorModel.addActuator(servo);
    // Only the first screw is driven now; the second joins the axis as turning inertia all the same.
    var servoLimits = motorModel.coupledLimits("slide");
    var servoExpected = 0.4 * scale * 1.5 / (10.0 + 2 * 0.4 * (4e-6 + 3e-5) * scale * scale);
    if (Math.abs(servoLimits.requireVelocity() - 200.0 / scale) > 1e-12)
      throw 'a servo caps its axis at its maximum speed: ${servoLimits.requireVelocity()}';
    if (Math.abs(servoLimits.requireAcceleration() - servoExpected) > servoExpected * 1e-12)
      throw 'a servo accelerates its axis at its peak torque: ${servoLimits.requireAcceleration()}, expected $servoExpected';
  }

  static function checkPose(simulation:Simulation, position:Array<Float>, rotation:Array<Float>,
      operation:String):Void {
    var actual = simulation.robotPose(0);
    for (axis in 0...3)
      if (!(Math.abs(actual.position[axis] - position[axis]) < 1e-6))
        throw '$operation did not restore position axis $axis';
    for (axis in 0...4)
      if (!(Math.abs(actual.rotation[axis] - rotation[axis]) < 1e-6))
        throw '$operation did not restore rotation axis $axis';
  }
}
