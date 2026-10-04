import haxe.Int64;
import machinekit.assembly.LinearAxis;
import motionkit.MotionOptions;
import motionkit.program.Blend;
import motionkit.program.MotionOp;
import motionkit.program.MotionProgram;
import motionkit.program.MoveTarget;
import motionkit.robot.AxisKinematics;
import motionkit.robot.MachineKitRobotCompiler;
import motionkit.robot.ProgramCompiler;
import motionkit.robot.StartTolerances;
import motionkit.robot.PlanCheck;
import motionkit.robot.PlanCheck.PlanCheckOptions;
import motionkit.robot.PlanCheckSummary;
import motionkit.robot.StepperSlip;
import robotkit.core.CoupledJoint;
import motionkit.trajectory.ExecutionPlan;
import motionkit.trajectory.PlanDiagnostic;
import motionkit.trajectory.Trajectory;
import motionkit.trajectory.ValidationLimits;
import motionkit.robot.ManipulatorMotion;
import robotkit.model.Actuator;
import robotkit.model.Encoder;
import robotkit.model.EncoderKind;
import robotkit.runtime.EncoderMonitor;
import robotkit.runtime.SimulationHarness;
import robotkit.simulation.SimulatedRobot;
import robotkit.model.ActuatorDrive.ServoDrive;
import robotkit.model.ActuatorDrive.StepperDrive;
import robotkit.model.Joint;
import robotkit.model.JointCoupling;
import robotkit.model.JointLimits;
import robotkit.model.JointType;
import robotkit.model.Link;
import robotkit.model.RobotModel;
import robotkit.model.SteadyLoads;
import robotkit.model.TorqueSpeedCurve;
import robotkit.model.Transmission;

/**
 * The plan check against a one-axis machine: a 10 kg slide on a Tr10 x 2 screw (pi * 1000 rad per
 * metre at 40%, a 0.02 N m nut drag, a drive's stiffness and backlash only where stated) turned
 * by one motor. The numbers are worked from the model: motor torque = rotor inertia * alpha +
 * force / (ratio * efficiency) + drag.
 */
class PlanCheckTests extends MotionKitTestSupport {
  static inline var SCREW_RATIO = 3141.592653589793;

  /** The machine: `axis` is the slide's direction; `drive` is the motor's drive kind. */
  function machine(axis:Array<Float>, servo:Bool, stiffness:Float = 0.0, backlash:Float = 0.0):RobotModel {
    var model = new RobotModel("plan-check-axis");
    var frame = model.addLink(new Link("frame"));
    var table = model.addLink(new Link("table"));
    var rotor = model.addLink(new Link("rotor"));
    table.mass = 10.0;
    rotor.mass = 0.2;
    rotor.centerOfMass = [0.0, 0.0, 0.0];
    rotor.inertiaTensor = [1e-5, 0.0, 0.0, 0.0, 1e-5, 0.0, 0.0, 0.0, 4e-6];
    var slide = model.addJoint(new Joint("slide", JointType.Prismatic, frame, table));
    slide.axis = axis;
    slide.limits = new JointLimits(-1.0, 1.0);
    var screw = model.addJoint(new Joint("screw", JointType.Continuous, frame, rotor));
    screw.limits = new JointLimits(-1e9, 1e9);
    screw.armature = 3e-5;
    var lead = new JointCoupling("lead", "slide", "screw", SCREW_RATIO, 0.0);
    lead.efficiency = 0.4;
    lead.drag = 0.02;
    lead.stiffness = stiffness;
    lead.backlash = backlash;
    model.addCoupling(lead);
    var motor = new Actuator("motor", 0.63, 137.1, Transmission.SimpleTransmission("screw", 1.0, 0.0));
    if (servo) {
      motor.maxEffort = null;
      motor.maxRate = null;
      motor.drive = new ServoDrive(0.3, 1.2, 300.0, 500.0, 3e-5, 4096.0);
    } else {
      // A NEMA 23 on 24 V: 1.26 N m to 68.6 rad/s, then falling as 1 / speed.
      motor.microsteps = 16;
      motor.maxStepRate = 200000;
      motor.drive = new StepperDrive(200.0, 3e-5, 1.26, new TorqueSpeedCurve([0.0, 68.6, 137.2, 274.4], [1.26, 1.26, 0.63, 0.315]));
    }
    model.addActuator(motor);
    return model;
  }

  /** A plan of one joint moving `distance` within the given speed and acceleration, and its check. */
  function plan(distance:Float, velocity:Float, acceleration:Float, ?start:Float = 0.0):ExecutionPlan {
    var trajectory = Trajectory.generateStateToState([start], [0.0], [0.0], [start + distance], [velocity], [acceleration],
      [acceleration * 500.0]);
    var limits = new ValidationLimits(1, Int64.ofInt(1), Int64.ofInt(1));
    limits.velocity(0, velocity * 1.01);
    limits.acceleration(0, acceleration * 1.01);
    limits.jerk(0, acceleration * 600.0);
    var made = ExecutionPlan.create(trajectory, limits, Int64.ofInt(1), [start], [0.0], [0.0], [0.01], [0.01], [0.01]);
    trajectory.dispose();
    return made;
  }

  function kinds(result:PlanCheckResult, kind:PlanDiagnosticKind):Int {
    var count = 0;
    for (diagnostic in result.diagnostics) if (diagnostic.kind == kind) count++;
    return count;
  }

  /** Drive rejection occurs before the first chunk reaches the runtime, and a whole move is checked once. */
  public function testDirectMotionRunsPlanCheck():Void {
    var blueprint = MachineKitRobotCompiler.compileLinearAxis(new LinearAxis(23, 10, 80), "x", 0.05, 0.2);
    for (actuator in blueprint.model.actuators)
      actuator.drive = new StepperDrive(200.0, 3e-5, 0.0001,
        new TorqueSpeedCurve([0.0, 1000.0], [0.0001, 0.0001]));
    var harness = new SimulationHarness(0.01);
    var runtime = harness.simulation.addRobot(blueprint.runtime);
    var robot = new SimulatedRobot("checked-direct", runtime, blueprint.model.name,
      [for (link in blueprint.model.links) link.name], [for (joint in blueprint.model.joints) joint.name]);
    var recording = new robotkit.recording.RobotRecording();
    var machine = new motionkit.robot.MotionSystem(new robotkit.recording.RecordingRobot(robot, recording), blueprint);
    var directCheck:PlanCheck = machine.planCheck;
    directCheck.options.rejects = true;
    var rejected = false;
    try machine.moveAxes([new motionkit.AxisTarget("x", 0.07)], new MotionOptions(0.05, 0.2))
      catch (error:Dynamic) rejected = Std.string(error).indexOf("plan check") >= 0;
    check(rejected, "a direct move rejects a predicted stall");
    check(machine.planChecks.flagged == 1, "the direct rejection retains its findings");
    for (command in recording.commands) switch command {
      case ExecutionPlan(_): throw "a rejected drive check must not submit motion";
      case _:
    }
    machine.reset();
    harness.step(Int64.ofInt(1));
    directCheck.options.rejects = false;
    machine.planChecks.reset();
    machine.moveAxes([new motionkit.AxisTarget("x", 0.07)], new MotionOptions(0.05, 0.2));
    var tick = 0;
    while (machine.isMoving() && tick < 1000) {
      machine.update();
      harness.step(Int64.ofInt(++tick));
    }
    check(!machine.isMoving(), "a report-only direct move completes");
    check(machine.planChecks.plans == 1 && machine.planChecks.flagged == 1,
      "refilling a direct move checks its whole trajectory only once");
    harness.dispose();
  }

  public function testServoStreamRunsPlanCheck():Void {
    var fixture = buildContractArmFixture();
    for (joint in fixture.model.joints) {
      joint.limits.velocity = 2.0;
      joint.limits.maxAcceleration = 4.0;
    }
    var blueprint = robotkit.runtime.RobotRuntimeCompiler.compile(fixture.model);
    var actuator = new Actuator("weak-servo", null, null,
      Transmission.SimpleTransmission(fixture.model.joints[0].id, 1.0, 0.0));
    actuator.drive = new ServoDrive(0.00001, 0.00002, 100.0, 200.0, 1e-3, 4096.0);
    fixture.model.addActuator(actuator);
    var harness = new SimulationHarness(0.01);
    var runtime = harness.simulation.addRobot(blueprint);
    var robot = new SimulatedRobot("checked-servo", runtime, fixture.model.name,
      [for (link in fixture.model.links) link.name], [for (joint in fixture.model.joints) joint.name]);
    var recording = new robotkit.recording.RobotRecording();
    var session = new motionkit.robot.ServoSession(new robotkit.recording.RecordingRobot(robot, recording), fixture.arm,
      0.01, 1e-3, 1e-4, new motionkit.robot.ServoPlan.ServoPlanOptions(
        Int64.ofInt(blueprint.revision), Int64.ofInt(blueprint.calibrationRevision)));
    var servoCheck:PlanCheck = session.planCheck;
    servoCheck.options.rejects = true;
    check(session.command(new motionkit.kinematics.Twist6(0.0, 0.0, 0.0, 0.0, 0.0, 0.1),
      1, session.nowNs() + Int64.ofInt(100000000)) == null, "a live servo command is accepted");
    var rejected = false;
    try session.update() catch (error:Dynamic) rejected = Std.string(error).indexOf("plan check") >= 0;
    check(rejected && session.planChecks.flagged == 1, "a live servo chunk rejects a predicted overload");
    check(recording.commands.length == 0, "an overloaded servo chunk never reaches the runtime");
    session.dispose();
    harness.dispose();
  }

  public function testPlanCheck():Void {
    var partial = machine([1.0, 0.0, 0.0], false, 2000.0);
    var unrelated = partial.addJoint(new Joint("first", JointType.Prismatic,
      partial.links[0], partial.addLink(new Link("first-body"))));
    unrelated.limits = new JointLimits(-1.0, 1.0);
    partial.joints.remove(unrelated);
    partial.joints.unshift(unrelated);
    partial.addActuator(new Actuator("first-motor", 1.0, 1.0,
      Transmission.SimpleTransmission("first", 1.0, 0.0)));
    var one = new PlanCheck(partial, ["slide"]);
    this.check(one.axisLoads().length == 1 && one.axisLoads()[0].axis == "slide" &&
      one.axisLoads()[0].elastic != null && one.axisLoads()[0].elastic.deflections([10.0]).length == 1,
      "a partial plan uses its own driven axis even when another comes first in the model");
    var options = new PlanCheckOptions();
    options.steady = new SteadyLoads(5.0);
    var flat = machine([1.0, 0.0, 0.0], false);
    var upright = machine([0.0, 0.0, 1.0], false);

    // The planner's own limits, with the steady loads taken off, never ask for more than the drive gives.
    var limits = flat.coupledLimits("slide", options.steady);
    var free = flat.coupledLimits("slide");
    check(limits.requireAcceleration() < free.requireAcceleration(), "steady loads leave less force to accelerate with");
    // force = 0.4 * 0.63 * pi*1000 less the nut's drag and rail friction, over mass and rotor inertia.
    var force = 0.4 * (0.63 - 0.02) * SCREW_RATIO - 5.0;
    var expected = force / (10.0 + 0.4 * (4e-6 + 3e-5) * SCREW_RATIO * SCREW_RATIO);
    near(limits.requireAcceleration(), expected, "acceleration under steady loads is the force left over the inertia", 1e-9);
    var honest = plan(0.1, limits.requireVelocity(), limits.requireAcceleration());
    var result = new PlanCheck(flat, ["slide"], options).check(honest, 7, 0.0);
    check(result.diagnostics.length == 0, 'a plan at the planner\'s own limits passes: ${result.diagnostics}');
    check(result.worstTorqueRatio > 0.4 && result.worstTorqueRatio <= 1.0 + 1e-6,
      'and comes within the curve: ${result.worstTorqueRatio}');
    check(result.worstMotor == "motor", "the worst motor is named");
    honest.dispose();

    // Three times the acceleration is more than the curve gives: a stall, named by op, motor and axis.
    var rough = plan(0.1, limits.requireVelocity(), 3.0 * limits.requireAcceleration());
    var stalled = new PlanCheck(flat, ["slide"], options).check(rough, 12, 0.0);
    check(kinds(stalled, PlanDiagnosticKind.StepperStall) == 1, "a plan beyond the pull-out curve is flagged once per motor");
    var found = stalled.diagnostics[0];
    check(found.opIndex == 12 && found.subject == "motor" && found.axis == "slide" && found.value > found.limit &&
      found.samples > 0 && found.ratio() > 1.0, "the finding names its op, motor, axis and how far over it is");
    check(found.describe(40).indexOf("line 40, op 12") >= 0 && found.describe().indexOf("% over") > 0,
      'and reads as a sentence: ${found.describe(40)}');
    rough.dispose();

    // Gravity on a vertical axis: the motor holds the load against gravity, 10 kg through pi*1000 rad/m at 40%.
    var hold = new PlanCheck(upright, ["slide"], options);
    var loads = hold.axisLoads()[0];
    near(loads.gravityForce, 10.0 * 9.80665, "a vertical axis carries its weight", 1e-9);
    near(new PlanCheck(flat, ["slide"], options).axisLoads()[0].gravityForce, 0.0, "a horizontal axis carries none", 1e-9);
    var up = loads.motorTorque(loads.motors[0], 0.01, 0.0, 5.0), side = new PlanCheck(flat, ["slide"], options)
      .axisLoads()[0].motorTorque(loads.motors[0], 0.01, 0.0, 5.0);
    near(up - side, 10.0 * 9.80665 / (SCREW_RATIO * 0.4), "gravity adds its force through the ratio and the screw's efficiency", 1e-9);
    // Going down, the load drives the motor back through the screw: it needs less than it gave going up.
    var down = loads.motorTorque(loads.motors[0], -0.01, 0.0, 5.0);
    check(Math.abs(down) < Math.abs(up), "a load driving the motor needs less torque");
    // The same plan on the vertical axis comes nearer the limit.
    var gentle = plan(0.1, 0.02, 1.0);
    var onFlat = new PlanCheck(flat, ["slide"], options).check(gentle, 0, 0.0);
    var onUpright = new PlanCheck(upright, ["slide"], options).check(gentle, 0, 0.0);
    check(onUpright.worstTorqueRatio > onFlat.worstTorqueRatio, "a vertical axis asks more of its motor than a horizontal one");
    // A cutting force on feed moves adds against the motion, and only on moves at or below the feed limit.
    var cutting = options.copy();
    cutting.cuttingForce = 200.0;
    cutting.cuttingFeedLimit = 0.03;
    var feed = new PlanCheck(flat, ["slide"], cutting).check(gentle, 0, 0.02);
    var rapid = new PlanCheck(flat, ["slide"], cutting).check(gentle, 0, 0.5);
    check(feed.worstTorqueRatio > onFlat.worstTorqueRatio && Math.abs(rapid.worstTorqueRatio - onFlat.worstTorqueRatio) < 1e-12,
      "a cutting force applies to feed moves and not to rapids");
    gentle.dispose();

    // Past the speed the curve has torque for, a stepper cannot go at all.
    var fast = plan(0.5, 0.3, 1.0);
    var tooFast = new PlanCheck(flat, ["slide"], options).check(fast, 1, 0.0);
    check(kinds(tooFast, PlanDiagnosticKind.StepperStall) == 1 && tooFast.diagnostics[0].limit == 0.0,
      "a stepper asked for more speed than its curve reaches has no torque to give");
    fast.dispose();

    // A servo: peak torque bounds each sample, rated torque bounds the RMS over the plan.
    var servo = machine([1.0, 0.0, 0.0], true);
    var servoLimits = servo.coupledLimits("slide");
    near(servoLimits.requireVelocity(), 500.0 / SCREW_RATIO, "a servo's axis speed comes from its maximum speed", 1e-9);
    var brisk = plan(0.002, 0.1, 0.9 * servoLimits.requireAcceleration());
    var servoResult = new PlanCheck(servo, ["slide"], options).check(brisk, 3, 0.0);
    check(kinds(servoResult, PlanDiagnosticKind.ServoPeakTorque) == 0, "a servo under its peak torque is not flagged for it");
    check(kinds(servoResult, PlanDiagnosticKind.ServoRatedTorque) == 1,
      "but its RMS torque over a hard move is above the rated torque");
    var servoFinding = servoResult.diagnostics[0];
    check(servoFinding.value > servoFinding.limit && Math.abs(servoFinding.limit - 0.3) < 1e-12, "the RMS finding gives the rated torque as its limit");
    brisk.dispose();
    var violent = plan(0.002, 0.1, 4.0 * servoLimits.requireAcceleration());
    check(kinds(new PlanCheck(servo, ["slide"], options).check(violent, 3, 0.0), PlanDiagnosticKind.ServoPeakTorque) == 1,
      "a servo over its peak torque is flagged");
    violent.dispose();
    var easy = plan(0.1, 0.02, 0.3);
    check(new PlanCheck(servo, ["slide"], options).check(easy, 3, 0.0).diagnostics.length == 0, "an easy move passes on a servo");
    easy.dispose();

    // Accuracy: a drive 200 N/mm stiff stretches by force over stiffness; the nut's backlash adds on a reversal.
    var belt = machine([1.0, 0.0, 0.0], false, 200000.0, 5e-5);
    var tight = options.copy();
    tight.tolerance = 1e-4;
    var forward = plan(0.1, 0.02, 1.0);
    var accuracy = new PlanCheck(belt, ["slide"], tight);
    var first = accuracy.check(forward, 0, 0.0);
    // 10 kg at 1 m/s² plus 5 N of rail friction, over 200 N/mm.
    near(first.worstDeviation, (10.0 * 1.0 + 5.0) / 200000.0, "a drive stretches by force over stiffness", 1e-6);
    check(first.worstAxis == "slide" && first.diagnostics.length == 0, "within tolerance it passes");
    var back = plan(-0.1, 0.02, 1.0, 0.1);
    var second = accuracy.check(back, 1, 0.0);
    near(second.worstDeviation, (10.0 * 1.0 + 5.0) / 200000.0 + 5e-5, "a reversal between plans adds the nut's backlash", 1e-6);
    check(kinds(second, PlanDiagnosticKind.Accuracy) == 1 && second.diagnostics[0].subject == "slide",
      "and is flagged when the total passes the tolerance");
    var jolt = plan(0.1, 0.05, 5.0);
    var harder = accuracy.check(jolt, 2, 0.0);
    check(kinds(harder, PlanDiagnosticKind.Accuracy) == 1 && harder.worstDeviation > 2e-4, "a harder move stretches the drive further");
    forward.dispose();
    back.dispose();
    jolt.dispose();

    // The summary adds results up.
    var summary = new PlanCheckSummary();
    summary.add(stalled);
    summary.add(result);
    check(summary.plans == 2 && summary.flagged == 1 && summary.count(PlanDiagnosticKind.StepperStall) == 1 &&
      summary.worstMotor == "motor" && summary.worstTorqueRatio >= stalled.worstTorqueRatio, "the summary counts plans and findings");
  }

  /**
   * A stepper pushed over its pull-out curve loses sync: the check says how far the axis falls behind
   * its command, and `StepperSlip` turns that into offsets on the axis and the joints coupled to it.
   * A plan the check passes loses nothing.
   */
  public function testStepperSlip():Void {
    var options = new PlanCheckOptions();
    options.steady = new SteadyLoads(5.0);
    var flat = machine([1.0, 0.0, 0.0], false);
    var limits = flat.coupledLimits("slide", options.steady);
    var honest = plan(0.1, limits.requireVelocity(), limits.requireAcceleration());
    var passed = new PlanCheck(flat, ["slide"], options).check(honest, 0, 0.0);
    check(passed.diagnostics.length == 0 && passed.slips.length == 0, "a plan within the curve loses no steps");
    honest.dispose();

    var rough = plan(0.1, limits.requireVelocity(), 3.0 * limits.requireAcceleration());
    var stalled = new PlanCheck(flat, ["slide"], options).check(rough, 4, 0.0);
    check(stalled.slips.length == 1, "a plan over the curve loses steps on its axis");
    var slip = stalled.slips[0];
    check(slip.axis == "slide" && slip.motors.length == 1 && slip.motors[0] == "motor", "the slip names the axis and its motor");
    check(slip.total() > 0.0 && slip.total() < 0.1, 'the axis falls behind by part of the move: ${slip.total()}');
    near(slip.steps, slip.total() * SCREW_RATIO / (2.0 * Math.PI / 200.0), "lost distance is lost full steps through the ratio", 1e-9);
    check(slip.steps > 1.0, 'enough to be several full steps: ${slip.steps}');
    near(slip.lostAt(0.0), 0.0, "nothing is lost before the motor leaves its curve", 1e-12);
    near(slip.lostAt(1e3), slip.total(), "all of it is lost by the end", 1e-12);
    var mid = slip.lostAt(0.5 * (slip.times[0] + slip.times[slip.times.length - 1]));
    check(mid > 0.0 && mid < slip.total(), "and it builds up while the motor is over the curve");
    rough.dispose();

    // A servo over its peak is not a slip: only steppers lose steps.
    var servo = machine([1.0, 0.0, 0.0], true);
    var violent = plan(0.002, 0.1, 4.0 * servo.coupledLimits("slide").requireAcceleration());
    check(new PlanCheck(servo, ["slide"], options).check(violent, 0, 0.0).slips.length == 0, "a servo never slips");
    violent.dispose();

    // The offsets reach the axis and what turns with it, and are kept until reset.
    var offsets = new Map<Int, Float>();
    var carrying = new StepperSlip([new CoupledJoint(1, 0, SCREW_RATIO, 0.0)], ["slide" => 0], (joint, offset) -> offsets.set(joint, offset));
    carrying.start(stalled);
    carrying.update(0.0);
    check(!carrying.slipped(), "no slip before the plan is over the curve");
    carrying.update(1e3);
    near(carrying.lost("slide"), slip.total(), "the slip carries out the check's total", 1e-12);
    var axisOffset = offsets.get(0), screwOffset = offsets.get(1);
    check(axisOffset != null && screwOffset != null, "the axis and its screw both get an offset");
    if (axisOffset == null || screwOffset == null) return;
    near(axisOffset, -slip.total(), "the axis sits behind its command", 1e-12);
    near(screwOffset, -slip.total() * SCREW_RATIO, "the screw it turns follows through the coupling", 1e-9);
    carrying.start(null);
    carrying.update(5.0);
    near(carrying.lost("slide"), slip.total(), "the error is kept after the plan", 1e-12);
    carrying.start(stalled);
    carrying.update(1e3);
    near(carrying.lost("slide"), 2.0 * slip.total(), "and adds up over plans", 1e-12);
    carrying.reset();
    var axisBack = offsets.get(0), screwBack = offsets.get(1);
    check(!carrying.slipped() && axisBack == 0.0 && screwBack == 0.0, "reset puts the joints back on their command");
  }

  /**
   * Runs a program through a simulated XYZ gantry whose steppers are `weak` (they cannot hold the move, so the plan
   * check predicts slip) or strong, with an encoder on the X motor, and returns what the encoder monitor saw.
   */
  function gantryRun(weak:Bool):GantryRun {
    var blueprint = MachineKitRobotCompiler.compileXYZGantry(new LinearAxis(23, 10, 80), new LinearAxis(23, 10, 80),
      new LinearAxis(23, 10, 80), 0.1, 0.4);
    var model = blueprint.model;
    for (actuator in model.actuators) {
      actuator.maxEffort = weak ? 0.002 : 1e7;
      actuator.maxRate = weak ? 100.0 : 1e5;
      var torque = weak ? 0.004 : 1e7;
      actuator.drive = new StepperDrive(200.0, 3e-5, torque, new TorqueSpeedCurve([0.0, actuator.requireRate()], [torque, torque]));
    }
    // Explicit motor feedback detects lost steps; the load-side scale measures carriage error.
    model.addEncoder(Encoder.perRevolution("x.encoder", blueprint.axes[0].jointIds[1], EncoderKind.Incremental, 2000.0));
    model.addEncoder(Encoder.perMillimetre("x.scale", "x/carriage-slide", EncoderKind.Incremental, 200.0));
    var ids = [for (joint in model.joints) joint.id];
    var solver = new AxisKinematics(blueprint);
    var limits = new ValidationLimits(ids.length, Int64.ofInt(blueprint.runtime.revision),
      Int64.ofInt(blueprint.runtime.calibrationRevision));
    var scales = [for (_ in ids) 0.0];
    for (axis in blueprint.axes) for (slot in 0...axis.jointIds.length)
      scales[ids.indexOf(axis.jointIds[slot])] = Math.abs(axis.jointScales[slot]);
    var speeds = [for (joint in model.joints) joint.limits.requireVelocity()];
    var accelerations = [for (joint in model.joints) joint.limits.requireAcceleration()];
    var jerks = [for (scale in scales) 20.0 * scale];
    for (joint in 0...ids.length) {
      var bound = model.joints[joint].limits;
      limits.position(joint, bound.lower, bound.upper);
      limits.velocity(joint, speeds[joint]);
      limits.acceleration(joint, accelerations[joint]);
      limits.jerk(joint, jerks[joint]);
    }
    var compiler = new ProgramCompiler(solver, limits, "work", speeds, accelerations, jerks,
      new StartTolerances([for (scale in scales) 0.00001 * scale],
        [for (scale in scales) 0.02 * scale], [for (scale in scales) 0.02 * scale]));
    compiler.planCheck = new PlanCheck(model, ids, new PlanCheckOptions());
    var goal = [for (_ in ids) 0.0];
    for (slot in 0...blueprint.axes[0].jointIds.length)
      goal[ids.indexOf(blueprint.axes[0].jointIds[slot])] = blueprint.axes[0].jointScales[slot] * 0.05;
    var moving = new MotionProgram([MotionOp.MoveJ(MoveTarget.JointTarget(goal), new MotionOptions(), Blend.ExactStop)]);
    var harness = new SimulationHarness(0.01);
    var runtime = harness.simulation.addRobot(blueprint.runtime);
    var robot = new SimulatedRobot("gantry", runtime, model.name, [for (link in model.links) link.name],
      [for (joint in model.joints) joint.name]);
    var motion = new ManipulatorMotion(robot, compiler, function(_) return null, function() return {events: [], overflow: false}, [for (joint in 0...ids.length) joint]);
    var slip = new StepperSlip([for (coupling in model.couplings) new CoupledJoint(ids.indexOf(coupling.follower),
      ids.indexOf(coupling.leader), coupling.ratio, coupling.offset)], [for (joint in 0...ids.length) ids[joint] => joint], (joint, offset) -> harness.simulation.setJointSlip(0, joint, offset));
    motion.slip = slip;
    var monitor = new EncoderMonitor(model, [for (_ in ids) 0.0]);
    motion.run(moving);
    var tick = 0, guard = 0;
    var snapshot = robot.snapshot();
    while (!motion.completed && motion.failure == null && guard++ < 3000) {
      motion.update(0.01);
      harness.step(Int64.ofInt(++tick));
      snapshot = robot.snapshot();
      monitor.observe([for (joint in 0...ids.length) snapshot.positions.get(joint)], [for (joint in 0...ids.length) snapshot.setpointPositions.get(joint)],
        tick * 0.01);
    }
    var result = new GantryRun(monitor, slip, motion.completed, motion.failure, snapshot.positions.get(0),
      snapshot.setpointPositions.get(0));
    result.motorJoint = blueprint.axes[0].jointIds[1];
    result.motorRatio = blueprint.axes[0].jointScales[1] / blueprint.axes[0].jointScales[0];
    result.diagnostics = [for (diagnostic in motion.checks.diagnostics) diagnostic.toString()];
    harness.dispose();
    return result;
  }

  /**
   * A stepper pushed over its curve slips in the simulation, keeps the error, and a motor-side encoder sees it
   * against the command. A plan the motors can hold slips nowhere and the encoder stays quiet.
   */
  public function testEncoderSeesStepperSlip():Void {
    var weak = gantryRun(true);
    check(weak.completed, 'the weak machine runs its program (${weak.failure})');
    check(weak.slip.slipped() && weak.slip.lost("x/carriage-slide") > 0.0, 'its X axis lost distance: ${weak.slip.lost("x/carriage-slide")}');
    check(weak.position < weak.commanded - 1e-4, 'the simulated axis ends behind its command: ${weak.position} against ${weak.commanded}');
    near(weak.commanded - weak.position, weak.slip.lost("x/carriage-slide"), "by the distance the plan check predicted", 1e-5);
    check(weak.monitor.faults(), "the encoder faults");
    var found = weak.monitor.findings[0];
    check(found.kind == robotkit.runtime.EncoderMonitor.EncoderFindingKind.LostSteps && found.encoder == "x.encoder" && found.joint == weak.motorJoint,
      'and names lost steps on the encoder\'s joint: ${found}');
    check(found.motor.length > 0 && found.steps > robotkit.runtime.EncoderMonitor.STEPPER_BOUND_STEPS && found.error * weak.motorRatio < 0.0,
      'with the motor, how many steps, and the sign: ${found.describe()}');
    var seen = weak.monitor.pathError("x.scale");
    near(Math.abs(seen.last), weak.slip.lost("x/carriage-slide"), "the encoder reads the error that was kept", 5e-6);

    var strong = gantryRun(false);
    check(strong.completed && !strong.slip.slipped() && !strong.monitor.faults(),
      'motors that hold the move lose nothing and the encoder is quiet: ${strong.completed} ${strong.failure} ${strong.slip.lost("x")} ${strong.monitor.findings} ${strong.diagnostics}');
    near(strong.position, strong.commanded, "the axis ends on its command", 1e-6);
  }

  /**
   * A motor-side encoder cannot see what sits between the motor and the load, a load-side one can: the same
   * command with the load 0.3 mm short of it (belt stretch) is no fault for the motor and a path error for the scale.
   */
  public function testLoadSideEncoderReportsPathError():Void {
    var model = machine([1.0, 0.0, 0.0], false);
    model.addEncoder(Encoder.perRevolution("motor.encoder", "screw", EncoderKind.Incremental, 4096.0, true));
    model.addEncoder(Encoder.perMillimetre("axis.scale", "slide", EncoderKind.Absolute, 1000.0));
    var monitor = new EncoderMonitor(model, [0.0, 0.0]);
    check(monitor.motorSide(0) && !monitor.motorSide(1), "the encoder on the motor's joint is motor-side and the scale on the load's is load-side");
    // The slide is commanded to 20 mm and the screw to the matching turns, but the load only gets 19.7 mm.
    var turns = 0.02 * SCREW_RATIO;
    for (step in 1...11) {
      var fraction = step / 10.0;
      monitor.observe([0.0197 * fraction, turns * fraction], [0.02 * fraction, turns * fraction], step * 0.01);
    }
    check(!monitor.faults(), "a motor that is where it was told has not lost steps");
    var path = monitor.pathError("axis.scale");
    near(path.last, -0.0003, "the load-side scale reports the path error", 1e-6);
    check(path.peak >= Math.abs(path.last) - 1e-12 && path.rms > 0.0 && path.rms <= path.peak, "with its worst and RMS");
    near(monitor.pathError("motor.encoder").peak, 0.0, "while the motor-side encoder sees none of it", 1e-3);
    // Lost steps show on the motor-side encoder: the screw ends 40 full steps short of its command.
    var lost = 40.0 * 2.0 * Math.PI / 200.0;
    monitor.observe([0.02, turns - lost], [0.02, turns], 0.2);
    check(monitor.faults() && monitor.findings[0].encoder == "motor.encoder" && Math.abs(monitor.findings[0].steps - 40.0) < 1.0,
      'and names about 40 lost steps: ${monitor.findings}');
    // Counts are whole counts: the reading is the position to the nearest one.
    var reading = monitor.readings[1];
    near(reading.position(), Math.fround(0.02 * 1000.0 * 1000.0) / 1e6, "an absolute scale reports whole counts", 1e-12);
  }

  /**
   * The compiler runs the check on every plan it makes, reports it on the plan, and rejects the plan
   * only when asked to.
   */
  public function testCompilerRunsPlanCheck():Void {
    var blueprint = MachineKitRobotCompiler.compileXYZGantry(new LinearAxis(23, 10, 80), new LinearAxis(23, 10, 80),
      new LinearAxis(23, 10, 80), 0.1, 0.4);
    var model = blueprint.model;
    // Motors that cannot hold their axes: a stepper of 0.002 N m against a 0.4 m/s² axis.
    for (actuator in model.actuators) {
      actuator.maxEffort = 0.002;
      actuator.maxRate = 100.0;
      actuator.drive = new StepperDrive(200.0, 3e-5, 0.004, new TorqueSpeedCurve([0.0, 100.0], [0.004, 0.004]));
    }
    var ids = [for (joint in model.joints) joint.id];
    var solver = new AxisKinematics(blueprint);
    var limits = new ValidationLimits(ids.length, Int64.ofInt(blueprint.runtime.revision),
      Int64.ofInt(blueprint.runtime.calibrationRevision));
    for (joint in 0...ids.length) {
      limits.position(joint, -1e9, 1e9);
      limits.velocity(joint, 1e6);
      limits.acceleration(joint, 1e6);
      limits.jerk(joint, 1e9);
    }
    function compiler(rejects:Bool):ProgramCompiler {
      var made = new ProgramCompiler(solver, limits, "work", [for (_ in ids) 1e6], [for (_ in ids) 1e6], [for (_ in ids) 1e9],
        StartTolerances.uniform(ids.length, 0.00001, 0.02, 0.02));
      var options = new PlanCheckOptions();
      options.rejects = rejects;
      made.planCheck = new PlanCheck(model, ids, options);
      return made;
    }
    var start = [for (_ in ids) 0.0];
    var goal = start.copy();
    for (axis in blueprint.axes) for (slot in 0...axis.jointIds.length)
      goal[ids.indexOf(axis.jointIds[slot])] = axis.jointScales[slot] * 0.05;
    var moving = new MotionProgram([MotionOp.MoveJ(MoveTarget.JointTarget(goal), new MotionOptions(), Blend.ExactStop)]);
    var compiled = compiler(false).compile(moving, start, Int64.ofInt(1));
    var plan = compiled.blocks[0].plans[0];
    var result = plan.checked;
    check(result != null, "the compiler checks the plans it makes");
    if (result == null) return;
    check(result.diagnostics.length > 0 && result.diagnostics[0].opIndex == 0 && result.worstTorqueRatio > 1.0,
      "a plan the weak motors cannot follow is flagged, naming its op");
    var plain = new ProgramCompiler(solver, limits, "work", [for (_ in ids) 1e6], [for (_ in ids) 1e6], [for (_ in ids) 1e9],
      StartTolerances.uniform(ids.length, 0.00001, 0.02, 0.02));
    check(plain.compile(moving, start, Int64.ofInt(2)).blocks[0].plans[0].checked == null, "a compiler with no check leaves its plans unchecked");
    var rejected = false;
    try compiler(true).compile(moving, start, Int64.ofInt(3)) catch (error:Dynamic) rejected = Std.string(error).indexOf("plan check") >= 0;
    check(rejected, "a compiler set to reject refuses the plan");
  }
}

/** What a simulated gantry run left: the encoder monitor, the slip it suffered, and where its X axis ended against its command. */
private class GantryRun {
  public var motorJoint:String = "";
  public var motorRatio:Float = 1.0;
  public final monitor:EncoderMonitor;
  public final slip:StepperSlip;
  public final completed:Bool;
  public final failure:Null<String>;
  public final position:Float;
  public final commanded:Float;
  public var diagnostics:Array<String> = [];

  public function new(monitor:EncoderMonitor, slip:StepperSlip, completed:Bool, failure:Null<String>, position:Float, commanded:Float) {
    this.monitor = monitor;
    this.slip = slip;
    this.completed = completed;
    this.failure = failure;
    this.position = position;
    this.commanded = commanded;
  }
}
