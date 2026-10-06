package processkit;

import motionkit.MotionOptions;
import motionkit.event.EventValue;
import motionkit.kinematics.IkTolerance;
import motionkit.kinematics.Pose3;
import motionkit.path.OrientationPolicy;
import motionkit.path.PoseLine;
import motionkit.path.PosePath;
import motionkit.path.PosePrimitive;
import motionkit.path.WeavePath;
import motionkit.path.FixedWeaveFrame;
import robotkit.spatial.Quat;
import motionkit.path.PoseWaypoint;
import motionkit.program.Blend;
import motionkit.program.InputPredicate;
import motionkit.program.MotionOp;
import motionkit.program.MotionProgram;
import motionkit.program.MoveTarget;
import motionkit.robot.ManipulatorKinematics;
import motionkit.robot.ManipulatorMotion;
import motionkit.robot.ProgramCompiler;
import motionkit.robot.PlanningLimits;
import processkit.WelderProcessDevice.WelderChannels;
import robotkit.manipulation.ArmClearance;
import robotkit.manipulation.KinematicGroup;
import processkit.skill.WeldPlan;
import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;
import processkit.tool.WeldFault;
import processkit.tool.WeldArcModel;
import processkit.tool.WeldSensor;
import processkit.tool.WeldSensor.WeldReading;
import robotkit.execution.FiredProcessEvent;
import robotkit.core.Robot;

private enum WeldingPhase {
  Idle;
  /** The run has started and the device is being prepared. */
  Preparing;
  /** The program is running. */
  Welding;
  /** The arc was lost: the arm is stopping and the device clearing, before the restart. */
  Stopping;
  Done;
  Failed;
}

/** The welder's latest reading, which the device and the program's input wait both read. */
private class LatestReading implements WelderFeedback {
  public var value:WeldReading = {arc: false, currentA: 0.0, voltageV: 0.0, touch: false, fault: WeldFault.None, powerW: 0.0};

  public function new() {}

  public function reading():WeldReading return value;
}

/** Complete welding motion checks shared by CAD station planning and runtime execution. */
class WeldPlanning {
  public final compiler:ProgramCompiler;
  public final wrist:WristLimits;
  public final planner:WeldPathPlanner;
  final clearance:Null<ArmClearance>;
  public function new(compiler:ProgramCompiler, wrist:WristLimits, planner:WeldPathPlanner,?clearance:ArmClearance) {
    this.clearance=clearance;
    this.compiler = compiler;
    this.planner = planner;
    this.wrist = wrist;
  }

  /** Offline station verification uses execution's geometric choice and the
   * same retained, timed program checks, without submitting a device command. */
  public function checked(requested:WeldPlan,start:Array<Float>):processkit.WeldPathProblem.WeldPathSelection {
    var selected=WeldingPlanRunner.selectWeld(compiler,wrist,clearance,requested,start);
    var group=cast(compiler.solver,ManipulatorKinematics).manipulator;
    var program=new WeldPathProgram(selected.problem,selected.curves,
      {arc:"station.arc",wireSpeed:"station.wire",voltage:"station.voltage"});
    var compiled=program.compile(compiler,group,start,haxe.Int64.ofInt(1),clearance);
    compiled.dispose();return selected;
  }
}

/**
 * Welds one seam with an arm: a MotionKit program, run by `ManipulatorMotion`, whose process is driven by a
 * `ProcessRun` over a `WelderProcessDevice`. It works through the robot alone: the torch's channels and the arm's
 * motion, and the welder's `tool_weld` reading, which the caller hands in each tick (`WeldSeam` takes it from the
 * robot's sensor), so the same runner welds on a fixed arm or a mobile base, simulated or real.
 *
 * The program is: a joint move to the approach pose (`approach` out along the wire from the start), a straight move
 * to the start, then the process run's - the arc strikes there (voltage and wire speed set, arc on) and the program
 * waits on the welder's established arc, or gives the seam up after `IGNITION_TIMEOUT`; it dwells `startDwell`; the
 * torch follows the seam at the travel speed with the wire speed that keeps the deposit per length constant; it dwells
 * `craterDwell` at the end with the arc up; the wire stops and the torch lifts `LIFT` over the burnback time while the
 * arc, with nothing feeding it, burns back, and the arc command ends; the torch retracts to the approach pose. The
 * process run's engagement (`ProcessEngagement`) carries the arc and the crater.
 *
 * If the welder faults mid-seam - the arc is lost, or never strikes - the process run interrupts, the arm stops, and
 * once the fault has cleared the run restarts `BACKOFF` metres before where it stopped, so the new bead overlaps the
 * old; at most `maxRestarts` times, after which the weld fails.
 */
class WeldingPlanRunner implements processkit.skill.WeldRunner {
  public static inline var FRAME:String = "arm-base";
  /** The name the program's input wait reads the established arc under. */
  public static inline var ARC_ESTABLISHED:String = "weld.arc_established";
  /** How long the program waits for the arc to establish, in seconds. */
  public static inline var IGNITION_TIMEOUT:Float = 2.0;
  /** How far before the stop a restart begins, in metres. */
  public static inline var BACKOFF:Float = 0.002;
  /** How far the torch lifts while the arc burns back, in metres. */
  public static inline var LIFT:Float = WeldPathPlanner.LIFT;
  /** Speed of the approach to the start and of the retract, in metres per second. */
  public static inline var APPROACH_SPEED:Float = 0.08;
  /** How long the welder may take to become ready before the weld is given up, in seconds. */
  public static inline var PREPARE_TIMEOUT:Float = 2.0;
  /** How long a fault may stay up after the arm has stopped before the weld is given up, in seconds. */
  public static inline var CLEAR_TIMEOUT:Float = 5.0;
  /** The most the torch turns within one straight piece of a corner's turn, in radians. */
  public static inline var TURN_STEP:Float = 0.2;
  /** Cartesian accuracy the seam is followed to, in metres. */
  public static inline var PATH_TOLERANCE:Float = 0.0005;
  public final motion:ManipulatorMotion;
  public var configuration:Null<motionkit.kinematics.SixAxisConfiguration> = null;
  public final channels:WelderChannels;
  public final maxRestarts:Int;
  /** The process run of the weld in progress. */
  public var current(default, null):Null<ProcessRun> = null;

  final wrist:WristLimits;
  final clearance:Null<ArmClearance>;
  var selectedProblem:Null<WeldPathProblem> = null;
  var selectedProgram:Null<WeldPathProgram> = null;
  var selectedCompilation:Null<motionkit.robot.CompiledProgram> = null;
  var selectedActive:Bool = false;
  final outputs:ChannelWelderOutputs;
  final latest:LatestReading;
  var phase:WeldingPhase = Idle;
  var failureMessage:Null<String> = null;
  var plan:Null<WeldPlan> = null;
  var restartCount:Int = 0;
  var waiting:Float = 0.0;
  /** Index, in the program, of the wait for the arc to establish. */
  var igniteIndex:Int = -1;
  var seam:Null<PosePath> = null;
  var planned:Null<PlannedWeld> = null;
  /** How long the last weld took to plan, in seconds, and what the plan is (for reports). */
  public var planningSeconds(default, null):Float = 0.0;
  /** Numeric pose queries used by the complete geometric and compiled-motion check. */
  public var planningIkSolves(default, null):Int = 0;
  public function lastPlan():Null<PlannedWeld> return planned;

  /** The weld's complete motion, validated without arc outputs or robot submission. Exact stops match the process program. */
  static function compileWeld(compiler:ProgramCompiler, wrist:WristLimits, planned:PlannedWeld,
      start:Array<Float>):motionkit.robot.CompiledProgram {
    var plan = planned.plan;
    var ops:Array<MotionOp> = [MotionOp.MoveJ(MoveTarget.JointTarget(planned.entry.joints), new MotionOptions(), Blend.ExactStop)];
    for (move in planned.entry.moves) ops.push(MotionOp.MoveL(move, FRAME, APPROACH_SPEED, Blend.ExactStop, OrientationPolicy.FreeAboutTool));
    ops.push(MotionOp.MoveL(pose(plan.start()), FRAME, APPROACH_SPEED, Blend.ExactStop, OrientationPolicy.FreeAboutTool));
    if (plan.parameters.startDwell > 0) ops.push(MotionOp.Dwell(plan.parameters.startDwell));
    ops.push(MotionOp.FollowPath(new PosePath(FRAME, pathOf(plan, plan.parameters.travelSpeed, wrist, planned.styles)), FRAME,
      plan.parameters.travelSpeed, []));
    if (plan.parameters.craterDwell > 0) ops.push(MotionOp.Dwell(plan.parameters.craterDwell));
    ops.push(MotionOp.MoveL(along(pose(plan.stop()), plan.stop(), -LIFT), FRAME,
      Math.max(LIFT / plan.parameters.burnback, 0.01), Blend.ExactStop, OrientationPolicy.FreeAboutTool));
    ops.push(MotionOp.MoveL(planned.retreat, FRAME, APPROACH_SPEED, Blend.ExactStop, OrientationPolicy.FreeAboutTool));
    return compiler.compile(new MotionProgram(ops), start, haxe.Int64.ofInt(1));
  }

  /** The same complete-motion planner used by execution, without a robot or channel owner. */
  public static function planning(manipulator:KinematicGroup, maxAcceleration:Float, ?clearance:ArmClearance):WeldPlanning {
    return planningWithLimits(manipulator, PlanningLimits.ofGroup(manipulator, new robotkit.model.SteadyLoads(), maxAcceleration), clearance);
  }

  static function planningWithLimits(manipulator:KinematicGroup, planning:PlanningLimits, ?clearance:ArmClearance):WeldPlanning {
    var count = manipulator.group.count();
    planning.requireGroup(manipulator);
    var limits = planning.validation();
    var solver = new ManipulatorKinematics(manipulator, 1e-8);
    solver.preferTargetOrientation = true;
    var compiler = new ProgramCompiler(solver, limits, FRAME,
      planning.velocity, planning.acceleration, planning.jerk,
      planning.startTolerances(0.005),
      null, 0.002, 0.2, PATH_TOLERANCE, 0.02, new IkTolerance(5e-5, 1e-3, 300, 0.03));
    compiler.planningAssumptions = planning.assumptions.copy();
    compiler.planCheck = planning.check();
    // Corner turns use the nearest rotary joints on the tool chain, in angular units.
    // A fixed torch has no angular cap to add; IK still rejects any change
    // of its hard wire axis. Linear drive limits are never angular limits.
    var wristSpeed = Math.POSITIVE_INFINITY;
    var wristAcceleration = Math.POSITIVE_INFINITY;
    for (joint in wristJointIndices(manipulator)) {
      wristSpeed = Math.min(wristSpeed, planning.velocity[joint]);
      wristAcceleration = Math.min(wristAcceleration, planning.acceleration[joint]);
    }
    var wrist:WristLimits = {angularSpeed: wristSpeed, angularAcceleration: wristAcceleration};
    var planner = new WeldPathPlanner(compiler.solver, compiler.ikTolerance, compiler.maxVelocity, wrist, clearance, compiler.perJointMaxJump,
      function(planned, start) return compileWeld(compiler, wrist, planned, start));
    return new WeldPlanning(compiler, wrist, planner,clearance);
  }

  /**
   * An arm's welding runner. `channels` are the torch's; `limits` supplies the drive-derived caps in joint order.
   * `clearance`, when given, is what the welds are planned to be clear of the work with (`WeldPathPlanner`).
   */
  public static function create(robot:Robot, manipulator:KinematicGroup,
      eventSource:Void -> {events:Array<FiredProcessEvent>, overflow:Bool}, channels:WelderChannels, limits:PlanningLimits,
      ?maxRestarts:Int = 3, ?clearance:ArmClearance):WeldingPlanRunner {
    var checked = planningWithLimits(manipulator, limits, clearance);
    var count = manipulator.group.count();
    var compiler = checked.compiler;
    var indices = [for (target in manipulator.toJointTargets([for (_ in 0...count) 0.0])) target.joint];
    // External axes bring the seam to the arm's working posture. Carry the
    // preference through point IK and the whole-path redundancy search.
    if (manipulator.external.indexOf(true) >= 0) {
      var positions = robot.snapshot().setpointPositions;
      cast(compiler.solver, ManipulatorKinematics).preferredPosture = [for (index in indices) positions.get(index)];
    }
    // The program waits on the established arc, which the welder's reading says.
    var latest = new LatestReading();
    var motion = new ManipulatorMotion(robot, compiler, function(channel) return channel == ARC_ESTABLISHED
      ? EventValue.Digital(latest.reading().arc) : null, eventSource, indices);
    return new WeldingPlanRunner(motion, channels, latest, maxRestarts, checked.wrist,clearance);
  }

  /** Up to three rotary drivers nearest the tool; work-positioner joints are excluded. */
  public static function wristJointIndices(manipulator:robotkit.manipulation.KinematicGroup):Array<Int> {
    var ids = [for (id in manipulator.jointIds()) Std.string(id)];
    var path = robotkit.manipulation.KinematicGroup.walk(manipulator.robot, manipulator.rootLink, manipulator.flangeLink);
    path.reverse();
    var indices:Array<Int> = [];
    for (joint in path) {
      if (joint.type != robotkit.model.JointType.Revolute && joint.type != robotkit.model.JointType.Continuous) continue;
      var index = ids.indexOf(Std.string(joint.id));
      if (index < 0) continue;
      indices.push(index);
      if (indices.length == 3) break;
    }
    return indices;
  }

  /** Over an existing motion, whose input wait reads the established arc from `latest`. */
  function new(motion:ManipulatorMotion, channels:WelderChannels, latest:LatestReading, maxRestarts:Int,
      wrist:WristLimits,?clearance:ArmClearance) {
    if (motion == null || channels == null || latest == null || maxRestarts < 0 || wrist == null)
      throw "WeldingPlanRunner needs motion, the torch's channels, a restart limit of zero or more, the wrist's limits";
    this.wrist = wrist;this.clearance=clearance;
    this.motion = motion;
    this.channels = channels;
    this.maxRestarts = maxRestarts;
    this.outputs = new ChannelWelderOutputs(channels);
    this.latest = latest;
  }

  public function restarts():Int return restartCount;
  public function running():Bool return phase == Preparing || phase == Welding || phase == Stopping;
  public function completed():Bool return phase == Done;
  public function failure():Null<String> return failureMessage;

  public function run(requested:WeldPlan):Void {
    if (running()) throw "A weld is already running";
    releaseSelected();
    var began=Sys.time();
    var group=cast(motion.compiler.solver,ManipulatorKinematics).manipulator;
    var before=group.numericSolveCount(),start=startPositions();
    var chosen=selectWeld(motion.compiler,wrist,clearance,requested,start,configuration);
    runSelected(chosen.problem,chosen.curves);
    planningSeconds=Sys.time()-began;planningIkSolves=group.numericSolveCount()-before;
  }

  /** Shared geometry and air policy for runtime and offline cell verification. */
  public static function selectWeld(compiler:ProgramCompiler,wrist:WristLimits,clearance:Null<ArmClearance>,
      requested:WeldPlan,start:Array<Float>,?configuration:motionkit.kinematics.SixAxisConfiguration):processkit.WeldPathProblem.WeldPathSelection {
    var problem=new WeldPathProblem(requested,wrist,[for(_ in requested.segments)WeldCorner.AROUND],FRAME,APPROACH_SPEED);
    var best:Null<processkit.WeldPathProblem.WeldPathSelection> = null;
    var reasons:Array<String> = [];
    // A checked cell needs an air departure before another run can cross the
    // work. The same geometry owns entry and retreat; both directions use DP.
    var airDistance=2*requested.parameters.approach;
    var alternatives=clearance==null ? [0.0] : [airDistance,0.0];
    for(clearanceDistance in alternatives){
      for(direction in [problem,problem.reversed()]){
        try {
          var alternative=clearanceDistance==0 ? direction : direction.withAirClearance(clearanceDistance);
          var selected=selectProblem(compiler,clearance,alternative,start,configuration);
          if(best==null || selected.cost<cast(best,processkit.WeldPathProblem.WeldPathSelection).cost)best=selected;
        }catch(error:Dynamic){reasons.push(Std.string(error));}
      }
      if(best!=null)break;
    }
    if(best==null)throw 'Cannot select either weld travel direction: ${reasons.join("; ")}';
    return cast(best,processkit.WeldPathProblem.WeldPathSelection);
  }

  /** Execute a global selection without running the legacy roll/entry search.
   * The selected geometry is compiled once before preparing the device. */
  public function runSelected(problem:WeldPathProblem,curves:Array<motionkit.planner.JointPathSamples>):Void {
    if(running())throw "A weld is already running";
    if(problem==null || problem.path.frameId!=FRAME)throw "Selected weld must use the runner's task frame";
    releaseSelected();
    var group=cast(motion.compiler.solver,ManipulatorKinematics).manipulator;
    var began=Sys.time(),before=group.numericSolveCount();
    var program=new WeldPathProgram(problem,curves,channels);
    var compiled=program.compile(motion.compiler,group,startPositions(),motion.compilationPlanId(),clearance,configuration);
    try {
      selectedProblem=problem;selectedProgram=program;selectedCompilation=compiled;
      var last=curves[curves.length-1],checked=0;
      for(curve in curves)checked+=curve.q.length;
      planned=new PlannedWeld(problem.plan,[],problem.cornerStyles(),
        new WeldEntry("globally selected joint entry",curves[0].q[0].copy(),[]),
        problem.retreat,"along the wire",checked,last.q[last.q.length-1],true);
      prepareWeld(problem.plan,problem.seam,problem.retreat);
      planningSeconds=Sys.time()-began;planningIkSolves=group.numericSolveCount()-before;
    }catch(error:Dynamic){releaseSelected();throw error;}
  }

  static function selectProblem(compiler:ProgramCompiler,clearance:Null<ArmClearance>,problem:WeldPathProblem,
      start:Array<Float>,configuration:Null<motionkit.kinematics.SixAxisConfiguration>):processkit.WeldPathProblem.WeldPathSelection {
    var solver=cast(compiler.solver,ManipulatorKinematics),group=solver.manipulator;
    var request=problem.request(start,compiler.ikTolerance,
      compiler.perJointMaxJump,compiler.maxVelocity);
    // Omitted ranges hold axes at the seed. Welds with a work positioner or
    // rail must search its physical range rather than freezing it there.
    var ranges:Array<motionkit.robot.ExternalAxisGrid.ExternalAxisRange> = [];
    // A Cartesian head solves all its axes directly; its linear axes are not
    // external coordinates to enumerate around a serial-arm solution.
    var cartesian=Std.isOfType(motionkit.robot.BranchIk.of(group),motionkit.robot.CartesianAnalyticIk);
    for(joint in 0...group.group.count())if(!cartesian && group.external[joint]){
      var limits=group.group.limitsOf(joint);
      if(!Math.isFinite(limits.lower) || !Math.isFinite(limits.upper))
        throw "Weld external-axis selection requires finite planning bounds";
      var span=limits.upper-limits.lower;
      var points=span==0 ? 1 : Std.int(Math.ceil(span/(request.maxJump[joint]*0.5)))+1;
      ranges.push(new motionkit.robot.ExternalAxisGrid.ExternalAxisRange(joint,limits.lower,limits.upper,points));
    }
    var sampling=new motionkit.robot.CandidateProblem.CandidateSamplingOptions(8,3,8,false,ranges,null,configuration);
    return problem.selectWithCost(group,request,sampling,clearance,(from,to)->{
      var entry=compiler.generateEntry(from,to);
      try {
        var violation=clearance==null ? null : motionkit.robot.TrajectoryClearance.violation(
          clearance,entry,false,0.01,q->problem.contact(solver.forward(q)));
        entry.dispose();return violation;
      }catch(error:Dynamic){entry.dispose();throw error;}
    },clearance==null ? null : q -> clearance.violation(q,false),null,null,0,solver);
  }

  function prepareWeld(prepared:WeldPlan,path:PosePath,exitPose:Pose3):Void {
    var plan=prepared;this.plan=plan;
    var parameters=plan.parameters,stop=pose(plan.stop()),travel=parameters.travelSpeed;
    seam=path;
    // Arc on with the wire at the weld speed, held until the arc is established, then the start dwell.
    var entry:Array<MotionOp> = [
      MotionOp.SetOutput(channels.wireSpeed, EventValue.Analog(parameters.wireSpeed)),
      MotionOp.SetOutput(channels.arc, EventValue.Digital(true)),
      MotionOp.WaitInput(ARC_ESTABLISHED, InputPredicate.Equals(EventValue.Digital(true)), IGNITION_TIMEOUT)
    ];
    if (parameters.startDwell > 0) entry.push(MotionOp.Dwell(parameters.startDwell));
    // A re-strike is over an existing bead: light at the stable minimum and omit the pooling dwell.
    var recoveryEntry:Array<MotionOp> = [
      MotionOp.SetOutput(channels.wireSpeed, EventValue.Analog(WeldArcModel.MIN_WIRE_SPEED)),
      MotionOp.SetOutput(channels.arc, EventValue.Digital(true)),
      MotionOp.WaitInput(ARC_ESTABLISHED, InputPredicate.Equals(EventValue.Digital(true)), IGNITION_TIMEOUT)
    ];
    // Crater fill with the arc up, then the wire stops; the torch lifts while the arc burns back, and the command ends.
    var exit:Array<MotionOp> = [];
    if (parameters.craterDwell > 0) exit.push(MotionOp.Dwell(parameters.craterDwell));
    exit.push(MotionOp.SetOutput(channels.wireSpeed, EventValue.Analog(0.0)));
    if (parameters.burnback > 0) {
      exit.push(MotionOp.MoveL(along(stop, plan.stop(), -LIFT), FRAME, Math.max(LIFT / parameters.burnback, 0.01), Blend.ExactStop, OrientationPolicy.FreeAboutTool));
    }
    exit.push(MotionOp.SetOutput(channels.arc, EventValue.Digital(false)));
    var recipe = new ProcessRecipe(travel * 0.5, travel * 2.0, travel, 0.0, OrientationPolicy.FreeAboutTool, 0.001,
      parameters.wireSpeed / travel, 0.0, BACKOFF, FeedChangePolicy.Reject, new ProcessEngagement(entry, exit, recoveryEntry), APPROACH_SPEED,
      PREPARE_TIMEOUT);
    var device = new WelderProcessDevice(outputs, latest, channels, {voltage: parameters.voltage});
    var process = new ProcessRun(recipe, seam, device, channels.wireSpeed, motion.session);
    current = process;
    process.start();
    restartCount = 0;
    failureMessage = null;
    waiting = 0.0;
    phase = Preparing;
  }

  public function update(dtSeconds:Float, reading:WeldReading):Void {
    latest.value = reading;
    var process = current;
    if (process == null) return;
    try {
      switch phase {
        case Idle | Done | Failed:
        case Preparing:
          // A welder with a fault it holds (a wire stuck to the work) never becomes ready: say so rather than wait for ever.
          process.update(0.0, dtSeconds);
          if (process.state == ProcessRunState.Ready) launch(true);
          else if (process.state == ProcessRunState.Failed) fail(cast(process.failure, String));
        case Welding:
          motion.update(dtSeconds);
          var problem = motion.failure;
          if (problem != null && !motion.running) {
            // The program failed on its own: an arc that never established (it failed waiting for it) is the welder's
            // to retry, anything else is not.
            if (motion.progress().op == igniteIndex || problem.indexOf(ARC_ESTABLISHED) >= 0) {
              process.interruptNow(travelled(), "the arc did not establish");
              beginStop();
            } else
              fail(problem);
          } else if (problem == null) {
            // Once past the path the seam is welded: a fault in the crater, the burnback or the lift is not a reason to
            // travel it again, only to finish ending the arc, which the rest of the program does and which clears it.
            if (!pastPath()) {
              process.update(travelled());
              if (process.state == ProcessRunState.ControlledInterruption) {
                beginStop();
                return;
              }
            }
            if (motion.completed) {
              process.finish();
              phase = Done;
            }
          }
        case Stopping:
          if (motion.running) motion.update(dtSeconds);
          waiting += dtSeconds;
          process.update(process.interruptedAt);
          if (process.state == ProcessRunState.Recovery && !motion.running) launch(false);
          else if (waiting > CLEAR_TIMEOUT)
            fail('the welder did not clear: ${WeldSensor.faultMessage(reading.fault)}');
      }
    } catch (error:Dynamic) {
      fail(Std.string(error));
    }
  }

  public function abort():Void {
    if (!running()) return;
    motion.abort();releaseSelected();
    phase = Idle;
  }

  /** The arm stops and the weld waits for the welder to clear, to restart. */
  function beginStop():Void {
    restartCount++;
    if (restartCount > maxRestarts) {
      fail('the arc could not be held in $maxRestarts restarts');
      return;
    }
    if (motion.running) motion.abort();
    waiting = 0.0;
    phase = Stopping;
  }

  /** Starts the process run's program, the first time from a joint move to the approach pose. */
  function launch(first:Bool):Void {
    var process = cast(current, ProcessRun);
    if(selectedProblem!=null){
      var start=process.preparedStart();
      if(!first){
        var problem=cast(selectedProblem,WeldPathProblem).recovery(start);
        var group=cast(motion.compiler.solver,ManipulatorKinematics).manipulator;
        // Recovery starts at the measured stopped state, never the prior program's endpoint.
        var positions=motion.robot.snapshot().positions;
        var stopped=[for(index in motion.jointIndices)positions.get(index)];
        var began=Sys.time(),before=group.numericSolveCount();
        var curves=selectProblem(motion.compiler,clearance,problem,stopped,configuration).curves;
        var program=new WeldPathProgram(problem,curves,channels,process.interruptedAt);
        var compiled=program.compile(motion.compiler,group,stopped,motion.compilationPlanId(),clearance,configuration);
        if(selectedCompilation!=null)selectedCompilation.dispose();
        selectedProgram=program;selectedCompilation=compiled;
        planningSeconds+=Sys.time()-began;planningIkSolves+=group.numericSolveCount()-before;
      }
      process.activatePrepared(start);outputs.drain();
      selectedActive=true;igniteIndex=cast(selectedProgram,WeldPathProgram).ignitionOp;waiting=0;
      motion.runCompiled(selectedCompilation);phase=Welding;return;
    }
    selectedActive=false;
    throw "Weld execution requires a selected problem and retained compilation";
  }

  /** The seam distance the torch has reached: where the program is on the path, before and after it the start and the end. */
  function travelled():Float {
    if(selectedActive)return selectedProgram.progress(selectedCompilation,motion.progress(),motion.completed).seamDistance;
    throw "Weld progress requires its selected compilation";
  }

  /** The program is past the path, in the crater fill, burnback, lift or retreat. */
  function pastPath():Bool {
    if(selectedActive)return switch selectedProgram.progress(selectedCompilation,motion.progress(),motion.completed).phase {
      case FillingCrater | BurningBack | Retreating | Complete:true;
      default:false;
    };
    return false;
  }

  function fail(message:String):Void {
    try motion.abort() catch (_:Dynamic) {}
    releaseSelected();failureMessage = message;
    phase = Failed;
  }

  function releaseSelected():Void {
    if(selectedCompilation!=null)selectedCompilation.dispose();
    selectedCompilation=null;selectedProgram=null;selectedProblem=null;selectedActive=false;
  }

  /** Where the arm's joints are now, or will be when the program running ends: where the next program starts. */
  function startPositions():Array<Float> {
    var commanded = motion.commandedPositions();
    if (commanded != null) return commanded;
    var positions = motion.robot.snapshot().positions;
    return [for (index in motion.jointIndices) positions.get(index)];
  }

  /**
   * The path the wire tip follows: a straight line per segment, the torch holding its orientation along it. Where
   * the next segment's orientation differs (a corner), the torch turns as it passes: along the last stretch of one
   * segment and the first of the next it turns half the way each, so that it meets the corner at the orientation midway
   * between the two and has the next segment's own orientation once past it. The turn is part of the travel, at the
   * travel speed, so the wire feed that keeps the deposit per length constant stays right, and no metal is piled
   * where the torch would otherwise stand still to turn. How long the stretch is comes from the angle and the wrist's
   * limits (`WeldCorner`), and a corner takes at most `WeldCorner.SHARE` of a segment.
   */
  public static function pathOf(plan:WeldPlan, travel:Float, wrist:WristLimits, styles:Array<Int>):Array<PosePrimitive> {
    var segments = plan.segments;
    var primitives:Array<PosePrimitive> = [];
    function waypoint(point:Vec3, rotation:Quat):PoseWaypoint
      return new PoseWaypoint(new Pose3(point.x, point.y, point.z, rotation.x, rotation.y, rotation.z, rotation.w), PATH_TOLERANCE, 0.01);
    function line(from:Vec3, fromRotation:Quat, to:Vec3, toRotation:Quat):Void
      primitives.push(new PoseLine(waypoint(from, fromRotation), waypoint(to, toRotation), OrientationPolicy.FreeAboutTool, 0.1, travel));
    /** A stretch of a turn from `r1` to `r2` of `angle` radians, taking it from fraction `s0` to `s1`, in steps of at most TURN_STEP radians. */
    function turn(from:Vec3, to:Vec3, r1:Quat, r2:Quat, s0:Float, s1:Float, angle:Float, travelA:Vec3, travelB:Vec3, style:Int):Void {
      var steps = Std.int(Math.max(1.0, Math.ceil(angle * (s1 - s0) / TURN_STEP)));
      for (step in 0...steps) {
        var f0 = step / steps, f1 = (step + 1) / steps;
        line(from.add(to.sub(from).scale(f0)), WeldCorner.orientationAt(r1, r2, s0 + (s1 - s0) * f0, travelA, travelB, style),
          from.add(to.sub(from).scale(f1)), WeldCorner.orientationAt(r1, r2, s0 + (s1 - s0) * f1, travelA, travelB, style));
      }
    }
    var progress = 0.0;
    for (index in 0...segments.length) {
      var firstPrimitive = primitives.length;
      var segment = segments[index];
      var a = segment.start.translation, b = segment.stop.translation;
      var length = segment.length();
      var direction = b.sub(a).scale(1.0 / length);
      var startRotation = segment.start.rotation, stopRotation = segment.stop.rotation;
      var beforeAngle = index > 0 ? segments[index - 1].stop.rotation.angularDistance(startRotation) : 0.0;
      var afterAngle = index + 1 < segments.length ? stopRotation.angularDistance(segments[index + 1].start.rotation) : 0.0;
      var before = beforeAngle > WeldCorner.STRAIGHT;
      var after = afterAngle > WeldCorner.STRAIGHT;
      var rampIn = before ? WeldCorner.given(WeldCorner.turnLength(beforeAngle, travel, wrist), length) : 0.0;
      var rampOut = after ? WeldCorner.given(WeldCorner.turnLength(afterAngle, travel, wrist), length) : 0.0;
      var from = a;
      if (before) {
        // The second half of the turn from the segment before.
        var end = a.add(direction.scale(rampIn));
        turn(a, end, segments[index - 1].stop.rotation, startRotation, 0.5, 1.0, beforeAngle, directionOf(segments[index - 1]), direction, styles[index]);
        from = end;
      }
      var until = after ? b.sub(direction.scale(rampOut)) : b;
      if (until.sub(from).norm() > 1e-6) line(from, startRotation, until, stopRotation);
      // The first half of the turn to the segment after.
      if (after) turn(until, b, stopRotation, segments[index + 1].start.rotation, 0.0, 0.5, afterAngle, direction, directionOf(segments[index + 1]), styles[index + 1]);
      var weave = plan.parameters.weave;
      if (weave != null && weave.amplitude > 0.0) {
        var open = segment.open;
        if (open == null) throw "A woven weld needs the CAD segment's material frame";
        var lateral = direction.cross(open).normalized();
        var base = new PosePath(FRAME, primitives.splice(firstPrimitive, primitives.length - firstPrimitive));
        var woven = WeavePath.apply(base, weave, new FixedWeaveFrame([lateral.x, lateral.y, lateral.z]), progress);
        for (primitive in woven.primitives) primitives.push(primitive);
      }
      progress += length;
    }
    return primitives;
  }

  /** The unit direction a segment runs in. */
  static function directionOf(segment:WeldSegment):Vec3
    return segment.stop.translation.sub(segment.start.translation).normalized();

  static function pose(frame:Transform3):Pose3 {
    var t = frame.translation, r = frame.rotation;
    return new Pose3(t.x, t.y, t.z, r.x, r.y, r.z, r.w);
  }

  /** `from` moved `distance` metres along its +Z, the wire. */
  static function along(from:Pose3, frame:Transform3, distance:Float):Pose3 {
    var wire = frame.rotation.rotate(new Vec3(0.0, 0.0, 1.0));
    return new Pose3(from.x + wire.x * distance, from.y + wire.y * distance, from.z + wire.z * distance, from.qx, from.qy, from.qz,
      from.qw);
  }
}
