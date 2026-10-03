import cadbridge.AssemblyPhysicalPartView;
import cadbridge.AssemblySimulationBridge;
import haxe.Int64;
import materia.project.SceneArtifact;
import motionkit.MotionOptions;
import motionkit.kinematics.IkTolerance;
import motionkit.kinematics.KinematicsSolver;
import motionkit.kinematics.Pose3;
import motionkit.kinematics.Twist6;
import motionkit.program.Blend;
import motionkit.program.MotionOp;
import motionkit.program.MotionProgram;
import motionkit.program.MoveTarget;
import motionkit.robot.ManipulatorMotion;
import motionkit.robot.PlanCheck;
import motionkit.robot.PlanCheck.PlanCheckOptions;
import motionkit.robot.ProgramCompiler;
import motionkit.robot.StartTolerances;
import motionkit.trajectory.ValidationLimits;
import motionkit.trajectory.ExecutionPlan;
import motionkit.trajectory.PlanDiagnostic;
import motionkit.trajectory.Trajectory;
import robotkit.model.Joint;
import robotkit.model.Link;
import robotkit.model.RobotModel;
import robotkit.model.SteadyLoads;
import robotkit.runtime.RobotRuntimeCompiler;
import robotkit.runtime.SimulationHarness;
import robotkit.world.SimulatedRobot;

/** The two axes of a Cartesian machine as a two-joint chain: the pose is where the joints are. */
class CartesianSolver implements KinematicsSolver {
  public function new() {}
  public function fork():KinematicsSolver return this;
  public function jointCount():Int return 2;
  public function forward(q:Array<Float>):Pose3 return new Pose3(q[0], q[1]);
  public function solvePose(target:Pose3, seed:Array<Float>, tolerance:IkTolerance):Null<Array<Float>> return [target.x, target.y];
  public function sampleCandidates(target:Pose3, maxCount:Int, tolerance:IkTolerance):Array<Array<Float>>
    return [[target.x, target.y]];
  public function solvePath(request:motionkit.kinematics.PathRequest):Array<Null<Array<Float>>>
    return request.followPointByPoint(this);
  public function solveDifferential(q:Array<Float>, twist:Twist6, ?preferredRate:Array<Float>):Null<Array<Float>>
    return [twist.linearX, twist.linearY];
}

/**
 * The CoreXY plotter example run through MotionKit: a program of moves along X and Y is planned for the two axes
 * alone, and the runtime turns the belts' pulleys, each motor with both axes, on the deterministic backend.
 */
class CoreXyTests extends MotionKitTestSupport {
  static function slot(index:Map<String, Int>, id:String):Int {
    var found = index.get(id);
    if (found == null) throw 'the plotter has no joint "$id"';
    return found;
  }

  public function testTwoBeltCompliance():Void {
    var model = new RobotModel("two-belts");
    var base = model.addLink(new Link("base"));
    for (id in ["x", "y", "a", "b"]) {
      var body = model.addLink(new Link(id + ".body"));
      model.addJoint(new Joint(id, id == "x" || id == "y" ? robotkit.model.JointType.Prismatic
        : robotkit.model.JointType.Continuous, base, body));
    }
    function axisStiffness():Float {
      var load = robotkit.model.DriveLoads.forAxis(model, "x");
      if (load == null) throw "The belt model lost its X drive";
      return load.stiffness;
    }
    var a = new robotkit.model.JointCoupling("xa", "x", "a", 1, 0);
    var b = new robotkit.model.JointCoupling("xb", "x", "b", 1, 0);
    a.stiffness = 1000;
    b.stiffness = 3000;
    model.addCoupling(a);
    model.addCoupling(b);
    model.addActuator(new robotkit.model.Actuator("a.motor", 1, 1, robotkit.model.Transmission.SimpleTransmission("a", 1, 0)));
    model.addActuator(new robotkit.model.Actuator("b.motor", 1, 1, robotkit.model.Transmission.SimpleTransmission("b", 1, 0)));
    near(axisStiffness(), 4000, "independent parallel belts add stiffness", 1e-9);
    model.addCoupling(new robotkit.model.JointCoupling("ya", "y", "a", 1, 0));
    model.addCoupling(new robotkit.model.JointCoupling("yb", "y", "b", -1, 0));
    near(axisStiffness(), 3000, "unequal CoreXY belts combine through the full stiffness matrix", 1e-9);
    var shared = robotkit.model.DriveLoads.of(model);
    var displaced = shared[0].elastic.deflections([1.0, 0.0]);
    near(displaced[0], 1.0 / 3000, "unequal belts deflect under X load", 1e-9);
    near(displaced[1], 1.0 / 6000, "unequal belts produce the derived cross-axis deflection", 1e-9);
    a.backlash = 0.002;
    b.backlash = 0.004;
    var lost = robotkit.model.DriveLoads.forAxis(model, "x");
    if (lost == null) throw "The shared drive lost its X axis";
    near(lost.backlash, 0.003,
      "independent motor lost motion has its worst Jacobian projection", 1e-9);
    a.stiffness = 0;
    near(axisStiffness(), 12000, "a rigid CoreXY belt leaves the other belt compliant", 1e-9);
    b.stiffness = 0;
    near(axisStiffness(), 0, "two rigid paths have no deflection", 1e-9);
  }

  public function testPlotterDrawsASquare():Void {
    var scene = SceneArtifact.decode(CoreXyPlotterPreview.plotter());
    var model = AssemblySimulationBridge.toRobotModel(scene.assemblyDefinition, AssemblyPhysicalPartView.fromSceneArtifact(scene),
      scene.assemblyState).model;
    var steady = new PlanCheckOptions().steady;
    var runtimeBlueprint = RobotRuntimeCompiler.compile(model, 1);
    var index = new Map<String, Int>();
    for (joint in 0...model.joints.length) index.set(model.joints[joint].id, joint);
    var radius = 1e-3 * CoreXyPlotter.TEETH * 2.0 / (2.0 * Math.PI);

    // The axes' limits come from both motors, and are the planner's.
    var planning = new RobotModel("plotter.axes");
    var parent = planning.addLink(new Link("plotter.base"));
    var driven:Array<Int> = [];
    for (id in ["x", "y"]) {
      var child = planning.addLink(new Link(id + ".carriage"));
      var source = model.joints[slot(index, id)];
      var joint = planning.addJoint(new Joint(id, source.type, parent, child, source.id));
      joint.axis = source.axis.copy();
      joint.limits = model.coupledLimits(id, steady);
      driven.push(slot(index, id));
      parent = child;
    }
    var velocities = [for (joint in planning.joints) joint.limits.requireVelocity()];
    var accelerations = [for (joint in planning.joints) joint.limits.requireAcceleration()];
    check(velocities[0] > 0.0 && accelerations[0] > 0.0 && accelerations[1] > 0.0, "the axes have limits from their motors");
    near(velocities[1], velocities[0], "X and Y are as fast as each other", 1e-12);
    check(accelerations[1] < accelerations[0], "Y accelerates less, carrying the gantry as well as the carriage");
    Sys.println('corexy axis limits: ${Math.round(velocities[0] * 1e4) / 10} mm/s, ${Math.round(accelerations[0] * 1e3) / 1e3} and ${Math.round(accelerations[1] * 1e3) / 1e3} m/s²');

    var limits = new ValidationLimits(2, Int64.ofInt(runtimeBlueprint.revision), Int64.ofInt(runtimeBlueprint.calibrationRevision));
    for (joint in 0...2) {
      limits.position(joint, -0.05, 0.05);
      limits.velocity(joint, velocities[joint] * 0.95);
      limits.acceleration(joint, accelerations[joint] * 0.95);
      limits.jerk(joint, accelerations[joint] * 40.0);
    }
    var ids = ["x", "y"];
    var compiler = new ProgramCompiler(new CartesianSolver(), limits, "work", [for (limit in velocities) limit * 0.95], [for (limit in accelerations) limit * 0.95],
      [for (limit in accelerations) limit * 20.0], StartTolerances.uniform(2, 1e-5, 0.02, 0.02));
    var options = new PlanCheckOptions();
    options.steady = steady;
    compiler.planCheck = new PlanCheck(model, ids, options);
    // A 30 mm square, then its diagonal both ways.
    var corners = [[0.03, 0.0], [0.03, 0.03], [0.0, 0.03], [0.0, 0.0], [0.03, 0.03], [0.0, 0.0]];
    var program = new MotionProgram([for (corner in corners)
      MotionOp.MoveJ(MoveTarget.JointTarget(corner), new MotionOptions(), Blend.ExactStop)]);
    var harness = new SimulationHarness(0.01);
    var runtime = harness.simulation.addRobot(runtimeBlueprint);
    var robot = new SimulatedRobot("plotter", runtime, model.name, [for (link in model.links) link.name], [for (joint in model.joints) joint.name]);
    var motion = new ManipulatorMotion(robot, compiler, function(_) return null, function() return {events: [], overflow: false}, driven);
    motion.run(program);
    var tick = 0, guard = 0;
    var peak = 0.0, worstSum = 0.0, worstDiagonal = 0.0;
    var alike = 0, opposite = 0;
    var motorSpeed = [0.0, 0.0];
    var previous = [0.0, 0.0, 0.0, 0.0];
    while (!motion.completed && motion.failure == null && guard++ < 6000) {
      motion.update(0.01);
      harness.step(Int64.ofInt(++tick));
      var snapshot = robot.snapshot();
      var x = snapshot.positions.get(slot(index, "x")), y = snapshot.positions.get(slot(index, "y"));
      // Every pulley is where its belts put it: the sum of its couplings' terms.
      for (follower in model.joints) {
        var wanted = 0.0, any = false;
        for (coupling in model.couplings) if (coupling.follower == follower.id) {
          any = true;
          wanted += coupling.ratio * snapshot.positions.get(slot(index, coupling.leader)) + coupling.offset;
        }
        if (any) worstSum = Math.max(worstSum, Math.abs(snapshot.positions.get(slot(index, follower.id)) - wanted));
      }
      var a = snapshot.positions.get(slot(index, "pulleyA-turn")), b = snapshot.positions.get(slot(index, "pulleyB-turn"));
      peak = Math.max(peak, Math.max(Math.abs(a), Math.abs(b)));
      // The motors are (x - y) / R and (x + y) / R, whatever the plan's timing.
      worstDiagonal = Math.max(worstDiagonal, Math.max(Math.abs(a - (x - y) / radius), Math.abs(b - (x + y) / radius)));
      var da = a - previous[0], db = b - previous[1], dx = x - previous[2], dy = y - previous[3];
      motorSpeed[0] = Math.max(motorSpeed[0], Math.abs(da) / 0.01);
      motorSpeed[1] = Math.max(motorSpeed[1], Math.abs(db) / 0.01);
      if (Math.abs(dy) < 1e-9 && Math.abs(dx) > 1e-5 && Math.abs(da - db) < 1e-6) alike++;
      if (Math.abs(dx) < 1e-9 && Math.abs(dy) > 1e-5 && Math.abs(da + db) < 1e-6) opposite++;
      previous = [a, b, x, y];
    }
    var finished = robot.snapshot();
    var endX = finished.positions.get(slot(index, "x")), endY = finished.positions.get(slot(index, "y"));
    check(motion.completed, 'the program completes (${motion.failure})');
    check([for (finding in motion.checks.diagnostics) if (finding.kind != motionkit.trajectory.PlanDiagnostic.PlanDiagnosticKind.Accuracy) finding].length == 0, 'a program at 95% of the axes\' limits asks no motor for more than it gives: ${[for (d in motion.checks.diagnostics) d.toString()]}');
    near(endX, 0.0, "the square ends where it began, x", 1e-6);
    near(endY, 0.0, "the square ends where it began, y", 1e-6);
    check(worstSum < 1e-9, 'every pulley is the sum of its couplings\' terms all along: $worstSum');
    check(worstDiagonal < 1e-9, 'and the motors are (x - y) / R and (x + y) / R: $worstDiagonal');
    near(peak, 0.06 / radius, "the motors turn furthest at the far corner, both axes at 30 mm: 60 mm of belt", 1e-6);
    check(alike > 10 && opposite > 10, 'x alone turns both motors alike ($alike ticks) and y alone against each other ($opposite)');
    // The axes' speed limit is the motors': both turning at their top speed along an axis.
    check(motorSpeed[0] < 204.0 && motorSpeed[1] < 204.0, 'motors stay under their 204 rad/s: ${motorSpeed}');
    Sys.println('corexy square: ${tick} ticks, motors at most ${Math.round(motorSpeed[0] * 10) / 10} and ${Math.round(motorSpeed[1] * 10) / 10} rad/s');
    harness.dispose();
  }

  /** A plan for the two axes from `from` to `to` within `scale` times their limits. */
  function plan(model:RobotModel, from:Array<Float>, to:Array<Float>, scale:Float, steady:SteadyLoads, coordinated:Bool = false):ExecutionPlan {
    var velocities = [for (id in ["x", "y"]) scale * model.coupledLimits(id, steady).requireVelocity()];
    var accelerations = [for (id in ["x", "y"]) scale * model.coupledLimits(id, steady).requireAcceleration()];
    if (coordinated) {
      velocities[0] = velocities[1] = Math.min(velocities[0], velocities[1]);
      accelerations[0] = accelerations[1] = Math.min(accelerations[0], accelerations[1]);
    }
    var trajectory = Trajectory.generateStateToState(from, [0.0, 0.0], [0.0, 0.0], to, velocities, accelerations,
      [for (limit in accelerations) limit * 500.0]);
    var limits = new ValidationLimits(2, Int64.ofInt(1), Int64.ofInt(1));
    for (joint in 0...2) {
      limits.velocity(joint, velocities[joint] * 1.01);
      limits.acceleration(joint, accelerations[joint] * 1.01);
      limits.jerk(joint, accelerations[joint] * 600.0);
    }
    var made = ExecutionPlan.create(trajectory, limits, Int64.ofInt(1), from, [0.0, 0.0], [0.0, 0.0], [0.01, 0.01], [0.01, 0.01], [0.01, 0.01]);
    trajectory.dispose();
    return made;
  }

  /**
   * Both motors serve both axes, so the plan check adds what each axis asks of a motor: along a diagonal one
   * motor stands almost still, and the other carries both axes' force.
   */
  public function testPlanCheckAddsTheAxesOnASharedMotor():Void {
    var scene = SceneArtifact.decode(CoreXyPlotterPreview.plotter());
    var model = AssemblySimulationBridge.toRobotModel(scene.assemblyDefinition, AssemblyPhysicalPartView.fromSceneArtifact(scene),
      scene.assemblyState).model;
    model = robotkit.model.RobotModelCodec.decode(robotkit.model.RobotModelCodec.encode(model));
    var options = new PlanCheckOptions();
    var check = new PlanCheck(model, ["x", "y"], options);
    var loads = check.axisLoads();
    this.check(loads.length == 2 && loads[0].motors.length == 2 && loads[1].motors.length == 2, "each axis is carried by both motors");
    for (load in loads) this.check(load.assumed.indexOf("belt stiffness") >= 0 &&
      load.assumed.indexOf("stepper inductance") >= 0 && load.assumed.indexOf("rotor inertia") >= 0,
      "part assumptions survive the assembly bridge and RobotModel codec into the axis load");

    var found = ["motorA", "motorB"];
    for (load in loads) for (motor in load.motors) this.check(found.indexOf(motor.actuator.id) >= 0, "and they are the plotter's two");
    near(loads[0].motors[0].share, 0.5, "each motor carries half of an axis's force", 1e-12);
    var worstDeviation = 0.0;
    // At the planner's motor limits, no move stalls; accuracy still reflects actual belt stretch.
    for (move in [[0.03, 0.0], [0.0, 0.03], [0.03, 0.03], [0.03, -0.03]]) {
      var honest = plan(model, [0.0, 0.0], move, 1.0, options.steady);
      var result = check.check(honest, 1, 0.0);
      worstDeviation = Math.max(worstDeviation, result.worstDeviation);
      this.check([for (finding in result.diagnostics) if (finding.kind != motionkit.trajectory.PlanDiagnostic.PlanDiagnosticKind.Accuracy) finding].length == 0, 'a move to ${move} at the axes\' limits passes: ${[for (d in result.diagnostics) d.toString()]} ratio ${result.worstTorqueRatio}');
      honest.dispose();
    }
    this.check(worstDeviation > 0.0, "the actual belts predict nonzero CoreXY deflection");
    Sys.println('corexy belt accuracy: ${Math.round(worstDeviation * 1e6) / 1000} mm worst deviation at planned limits');
    // Far above them, a move along one axis overloads both motors alike. On a
    // diagonal, motor A stands still but must hold the unequal X/Y inertial
    // loads; at ten times the allowed acceleration that holding load also
    // exceeds its torque ceiling.
    function flagged(move:Array<Float>):String {
      var rough = plan(model, [0.0, 0.0], move, 10.0, options.steady, move[0] == move[1]);
      var result = check.check(rough, 2, 0.0);
      for (diagnostic in result.diagnostics) {
        var accuracy = diagnostic.kind == motionkit.trajectory.PlanDiagnostic.PlanDiagnosticKind.Accuracy;
        this.check((diagnostic.assumed.indexOf("belt stiffness") >= 0) == accuracy,
          "only accuracy findings use the belt stiffness assumption");
        this.check(diagnostic.describe().indexOf("assumed:") >= 0, "findings expose the assumptions they use");
      }

      var names = [for (diagnostic in result.diagnostics) if (diagnostic.kind == motionkit.trajectory.PlanDiagnostic.PlanDiagnosticKind.StepperStall) diagnostic.subject];
      names.sort(Reflect.compare);
      rough.dispose();
      return names.join(",");
    }
    this.check(flagged([0.03, 0.0]) == "motorA,motorB", "x alone overloads both motors");
    this.check(flagged([0.0, 0.03]) == "motorA,motorB", "y alone overloads both motors");
    var diagonal = flagged([0.03, 0.03]);
    var motorASpeedRatio = 0.0;
    for (load in loads) for (motor in load.motors) if (motor.actuator.id == "motorA")
      motorASpeedRatio += motor.ratio;
    near(motorASpeedRatio, 0.0, "motor A stands still on an X=Y diagonal", 1e-9);
    this.check(diagonal == "motorA,motorB",
      'the unequal inertial loads also overload the stationary motor on a violent diagonal: $diagonal');
  }
}
