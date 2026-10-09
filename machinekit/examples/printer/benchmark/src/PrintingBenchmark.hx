import cadbridge.AssemblyPhysicalPartView;
import cadbridge.AssemblySimulationBridge;
import camkit.additive.PrinterProfile;
import camkit.additive.Slicer;
import camkit.additive.SlicerSettings;
import camkit.additive.TestParts;
import cnckit.CncCompiler;
import cnckit.CncController;
import cnckit.printer.PrinterFlavour;
import cnckit.printer.PrinterWriter;
import haxe.Int64;
import machinekit.transmission.SpurGear;
import materia.project.SceneArtifact;
import motionkit.axis.MotionAxisBlueprint;
import motionkit.robot.ProgramBlocks;
import motionkit.robot.ProgramCompiler;
import motionkit.robot.ProgramSink;
import motionkit.robot.MotionSystemBlueprint;
import motionkit.robot.ProgramBarrier;
import motionkit.trajectory.ExecutionPlan;
import robotkit.device.DeviceBinding;
import robotkit.model.JointType;
import robotkit.model.SteadyLoads;
import toolpathkit.motion.MachineBinding;
import toolpathkit.motion.PrinterBinding;
import toolpathkit.motion.ToolpathMotion;
import toolpathkit.motion.ToolpathMotionBinding;
import toolpathkit.path.ToolpathOp;
import toolpathkit.path.ToolpathProgram;

/** The CoreXY printer's three axes as the planner sees them. */
class PrinterAxes {
  /**
    Joint values are relative to the assembly's initial pose, which has the nozzle over the bed's middle and 10 mm
    above it: the bed's front left corner at the surface is joint (-0.11, -0.11, -0.01), the work origin.
  **/
  public static inline var ORIGIN_X:Float = -0.11;
  public static inline var ORIGIN_Y:Float = -0.11;
  public static inline var ORIGIN_Z:Float = -0.01;
  public final binding:ToolpathMotionBinding;
  public final blueprint:MotionSystemBlueprint;
  public final speed:Array<Float> = [];
  public final acceleration:Array<Float> = [];
  public final start:Array<Float>;
  public final printer:PrinterBinding;

  public function new(?pressureAdvance:Float = 0.0) {
    var scene = SceneArtifact.decode(CoreXyPrinterPreview.printer());
    var model = AssemblySimulationBridge.toRobotModel(scene.assemblyDefinition, AssemblyPhysicalPartView.fromSceneArtifact(scene),
      scene.assemblyState).model;
    var planning = DeviceBinding.planningModel(model, 40000);
    var axes = [for (id in ["x", "y", "z"]) {
      var joint = planning.joints[[for (j in planning.joints) j.id].indexOf(id)];
      if (joint.type != JointType.Prismatic) throw 'printer joint $id is not an axis';
      var limits = planning.coupledLimits(id, new SteadyLoads());
      speed.push(limits.requireVelocity());
      acceleration.push(limits.requireAcceleration());
      new MotionAxisBlueprint(id, [id], joint.limits.lower, joint.limits.upper, limits.requireVelocity(), limits.requireAcceleration(),
        joint.limits.lower);
    }];
    blueprint = MotionSystemBlueprint.fromRobotModel(planning, axes);
    var machine = new MachineBinding("work", "x", "y", "z", 0.25, new toolpathkit.motion.ToolpathChannels(""), 1e-4);
    machine.setRapidsAtFeed(true);
    // The extruder: a hobbed gear of the printer's own extruder part, feeding the hotend's filament.
    var printed = new CoreXyPrinter();
    printer = new PrinterBinding("e", printed.hotend.spec.filamentDiameter * 1e-3, printed.extruder.spec.hobDiameter * 0.5e-3,
      printed.hotend.spec.maxVolumetricFlow * 1e-9, 0.0005, pressureAdvance);
    machine.setPrinter(printer);
    binding = new ToolpathMotionBinding(machine, blueprint);
    // Where homing leaves the nozzle (the slicer's profile).
    ids = [for (joint in planning.joints) joint.id];
    this.planning = planning;
    start = stateAt(0.0, 0.0, 0.01);
  }

  final planning:robotkit.model.RobotModel;
  var ids:Array<String> = [];

  /** Every joint's value with the nozzle at (x, y, z) of the bed's frame, each coupled joint where its leaders put it. */
  public function stateAt(x:Float, y:Float, z:Float):Array<Float> {
    var q = [for (_ in ids) 0.0];
    q[ids.indexOf("x")] = ORIGIN_X + x;
    q[ids.indexOf("y")] = ORIGIN_Y + y;
    q[ids.indexOf("z")] = ORIGIN_Z + z;
    var known = [for (id in ids) id == "x" || id == "y" || id == "z"];
    for (_ in 0...ids.length) for (follower in 0...ids.length) if (!known[follower]) {
      var sum = 0.0, any = false, ready = true;
      for (coupling in planning.couplings) if (coupling.follower == ids[follower]) {
        any = true;
        var leader = ids.indexOf(coupling.leader);
        if (!known[leader]) ready = false;
        else sum += coupling.ratio * q[leader] + coupling.offset;
      }
      if (any && ready) { q[follower] = sum; known[follower] = true; }
    }
    return q;
  }
}

typedef BenchmarkResult = {
  var failure:Null<String>;
  var planned:Float;
  var reference:Float;
  var wall:Float;
  /** Seconds of motion planned per second of planning, over the whole print and over its worst 10 s. */
  var overall:Float;
  var worstWindow:Float;
  var sections:Int;
}

/** Notes when each plan arrived and how long it runs, then releases it; barriers and notes go to a `ProgramBlocks`. */
class TimedSink implements ProgramSink {
  public final blocks:ProgramBlocks = new ProgramBlocks();
  /** Seconds of processor time (not wall time: other work on the machine does not count) planning spent on each plan, and the seconds that plan takes to run. */
  public final wall:Array<Float> = [];
  public final run:Array<Float> = [];
  var last:Float = Sys.cpuTime();
  final trace:Bool = Sys.getEnv("PRINTING_BENCHMARK_TRACE") != null;
  /** The largest garbage-collected heap seen as plans arrived, bytes. */
  public var peakHeap:Float = 0.0;

  public function new() {}

  public function reset():Void last = Sys.cpuTime();

  public function plan(plan:ExecutionPlan, opIndex:Int, length:Float, distances:Array<Float>, times:Array<Float>):Void {
    var now = Sys.cpuTime();
    wall.push(now - last);
    if (trace) Sys.println('TRACE plan ${run.length} op $opIndex: ${Math.round(plan.durationSeconds * 1000)} ms of motion, path ${Math.round(length * 1e5) / 100} mm, planned in ${Math.round((now - last) * 1000)} ms');
    last = now;
    run.push(plan.durationSeconds);
    peakHeap = Math.max(peakHeap, hl.Gc.heapBytes());
    // A plan is measured and let go, as an executor does once it has run it: a long print's plans do not fit in memory together.
    plan.dispose();
  }

  public function barrier(barrier:ProgramBarrier):Void blocks.barrier(barrier);
  public function note(text:String):Void blocks.note(text);
  public function finish(nextPlanId:Int64):Void blocks.finish(nextPlanId);
}

/**
 * Planning benchmark: a sliced job through lowering and the program compiler, planning only (no simulation, no plan
 * check), on the printer's axes. Prints moves, planned time, planning wall time and real-time ratio (overall and for
 * the worst 10 s of print time), peak memory, exact stops by reason and timing sections, next to the lower bound of a
 * look-ahead planner with Marlin's junction deviation at the same accelerations.
 *
 * Run with no arguments it is the gate. `PRINTING_BENCHMARK_ONLY` names a load instead: `gear`, `cube`, `gcode` (a tower
 * through G-code and back) or `all`.
 */
class PrintingBenchmark {
  /** The process's peak resident memory in megabytes (the kernel's high-water mark), or -1. */
  /** The planning-only view of a print: its moves, without heaters, fans, filament and markers. */
  static function motionOnly(program:ToolpathProgram):ToolpathProgram {
    // `PRINTING_BENCHMARK_LAYERS=n` plans only the first n layers, for profiling.
    var limit = Sys.getEnv("PRINTING_BENCHMARK_LAYERS");
    var layers = limit == null ? -1 : Std.parseInt(limit), seen = 0;
    var ops:Array<ToolpathOp> = [];
    for (op in program.ops) {
      switch op {
        case Move(_, _, _, _, _), MachineMove(_, _, _, _, _): ops.push(op);
        case Layer(_, _, _): if (layers >= 0 && ++seen > layers) break;
        case _:
      }
    }
    return new ToolpathProgram(ops, program.tools, [new toolpathkit.setup.Setup("1",
      new toolpathkit.path.Point3(PrinterAxes.ORIGIN_X, PrinterAxes.ORIGIN_Y, PrinterAxes.ORIGIN_Z))]);
  }

  /** What a load came to: the numbers `report` prints. */
  static function report(name:String, machine:PrinterAxes, program:ToolpathProgram, sliceSeconds:Float):BenchmarkResult {
    var moves = motionOnly(program);
    var moveCount = moves.ops.length;
    var estimate = PrintTimeEstimate.estimate(moves, machine.speed, machine.acceleration);
    var sections = 0;
    var loweringBegan = Sys.time();
    var lowered = ToolpathMotion.lower(moves, machine.binding.machine);
    var loweringSeconds = Sys.time() - loweringBegan;
    var motion = lowered.program;
    if (motion == null) throw '$name has no motion';
    var sink = new TimedSink();
    var heapBefore = hl.Gc.heapBytes(), allocatedBefore = hl.Gc.totalAllocated();
    var compileBegan = Sys.cpuTime();
    sink.reset();
    var failure:Null<String> = null;
    var compilation = machine.binding.compiler.begin(motion, machine.start, Int64.ofInt(1), 0, 1.0, sink);
    try {
      while (compilation.step()) {}
    } catch (error:Dynamic) { failure = Std.string(error); compilation.dispose(); }
    var compileSeconds = Sys.cpuTime() - compileBegan;
    var planned = 0.0;
    for (seconds in sink.run) planned += seconds;
    // The worst stretch of 10 s of print time: the plans in it, and what they cost to plan.
    var worstRatio = Math.POSITIVE_INFINITY, worstAt = 0.0;
    var cumulativeRun = [0.0], cumulativeWall = [0.0];
    for (i in 0...sink.run.length) { cumulativeRun.push(cumulativeRun[i] + sink.run[i]); cumulativeWall.push(cumulativeWall[i] + sink.wall[i]); }
    var window = 10.0;
    if (planned >= window) {
      var first = 0;
      for (last in 1...cumulativeRun.length) {
        while (cumulativeRun[last] - cumulativeRun[first + 1] >= window) first++;
        var span = cumulativeRun[last] - cumulativeRun[first];
        if (span < window) continue;
        var cost = cumulativeWall[last] - cumulativeWall[first];
        var ratio = span / Math.max(cost, 1e-9);
        if (ratio < worstRatio) { worstRatio = ratio; worstAt = cumulativeRun[first]; }
      }
    }
    var stops = [for (reason in lowered.stops.keys()) '$reason ${lowered.stops.get(reason)}'];
    stops.sort(Reflect.compare);
    var notes = sink.blocks.notes.length;
    Sys.println('printing benchmark: $name');
    Sys.println('  slicing: ${Math.round(sliceSeconds * 1000)} ms; $moveCount moves, ${estimate.segments} segments for the estimate');
    Sys.println('  lowering: ${Math.round(loweringSeconds * 1000)} ms to ${motion.ops.length} path ops');
    if (failure != null) Sys.println('  PLANNING FAILED after ${sink.run.length} plans (${Math.round(planned * 10) / 10} s planned): $failure');
    Sys.println('  planned: ${Math.round(planned * 10) / 10} s in ${sink.run.length} timing sections (plans); estimate lower bound ${Math.round(estimate.seconds * 10) / 10} s, ' +
      'planned is ${Math.round((planned / estimate.seconds - 1.0) * 1000) / 10} % above it');
    Sys.println('  planning: ${Math.round(compileSeconds * 100) / 100} s of processor time, ${Math.round(planned / compileSeconds * 100) / 100}x real time overall' +
      (worstRatio < Math.POSITIVE_INFINITY ? ', worst 10 s window ${Math.round(worstRatio * 100) / 100}x (from print time ${Math.round(worstAt * 10) / 10} s)' : ', print shorter than 10 s'));
    Sys.println('  exact stops by reason: ${stops.length == 0 ? "none" : stops.join(", ")}; $notes planner notes');
    Sys.println('  memory: managed heap peak ${Math.round(sink.peakHeap / 1048576)} MB (${Math.round(heapBefore / 1048576)} MB before planning), ' +
      '${Math.round((hl.Gc.totalAllocated() - allocatedBefore) / 1048576)} MB allocated while planning');
    sink.blocks.dispose();
    return {failure: failure, planned: planned, reference: estimate.seconds, wall: compileSeconds,
      overall: planned / Math.max(compileSeconds, 1e-9), worstWindow: worstRatio, sections: sink.run.length};
  }

  /**
   * AM5's gate on a thin gear (module 1.5, 40 teeth, 1.2 mm: six layers, 61 mm across, involute teeth that slice to
   * short segments): planning at least twice as fast as the print over the whole job and over its worst 10 s, a planned
   * time no more than 15 % above the look-ahead reference, and no planning failure (the compiler rejects any plan that
   * breaks a limit or strays from the path's tolerance).
   */
  static function gate(machine:PrinterAxes, settings:SlicerSettings, profile:PrinterProfile):Void {
    motorChecks(machine);
    extruderChecks(machine, settings, profile);
    // Pressure advance runs the extruder 40 ms ahead of the flow: the same filament in the end, and every plan still within its limits.
    extruderChecks(new PrinterAxes(0.04), settings, profile);
    var part = new SpurGear(1.5, 40, 1.2).geometry();
    var sliced = Slicer.slice(part, settings, profile, "gear");
    part.close();
    var result = report("gate gear (module 1.5, 40 teeth, 1.2 mm)", machine, sliced.program, sliced.report.seconds);
    var failures:Array<String> = [];
    if (result.failure != null) failures.push("planning failed: " + result.failure);
    if (result.overall < 2.0) failures.push('planning is ${result.overall}x real time overall, below 2x');
    if (result.worstWindow < 2.0) failures.push('planning is ${result.worstWindow}x real time over its worst 10 s, below 2x');
    if (result.planned > 1.15 * result.reference) failures.push('planned ${result.planned} s is more than 15 % above the reference ${result.reference} s');
    Sys.println(failures.length == 0 ? "PRINTING GATE PASSED" : "PRINTING GATE FAILED: " + failures.join("; "));
    if (failures.length > 0) Sys.exit(1);
  }

  /**
   * On a CoreXY machine both motors serve both axes, so a diagonal asks one of them for the sum of what the axes ask
   * (the slicer's infill runs at 45 and 135 degrees). Fast moves along x, y and both diagonals are planned with the
   * drives' plan check on, and sampled every millisecond: no motor turns faster than its own rate allows, none is asked
   * for more torque than it gives, and the axes' own limits are kept.
   */
  static function motorChecks(machine:PrinterAxes):Void {
    var model = machine.blueprint.model;
    var ids = [for (joint in model.joints) joint.id];
    var options = new motionkit.robot.PlanCheck.PlanCheckOptions();
    machine.binding.compiler.planCheck = new motionkit.robot.PlanCheck(model, ids, options);
    var motors = ["pulleyA-turn", "pulleyB-turn"];
    var rates = [for (id in motors) model.coupledLimits(id, null, true).requireVelocity()];
    var origin = new toolpathkit.path.Point3(PrinterAxes.ORIGIN_X, PrinterAxes.ORIGIN_Y, PrinterAxes.ORIGIN_Z);
    var report:Array<String> = [];
    var seconds = new Map<String, Float>();
    for (move in [{name: "x", dx: 1.0, dy: 0.0}, {name: "y", dx: 0.0, dy: 1.0}, {name: "45", dx: 1.0, dy: 1.0}, {name: "135", dx: -1.0, dy: 1.0}]) {
      var length = Math.sqrt(move.dx * move.dx + move.dy * move.dy);
      var from = new toolpathkit.path.Point3(0.11 - 0.05 * move.dx / length, 0.11 - 0.05 * move.dy / length, 0.01);
      var to = new toolpathkit.path.Point3(0.11 + 0.05 * move.dx / length, 0.11 + 0.05 * move.dy / length, 0.01);
      var program = new ToolpathProgram([ToolpathOp.Move(Cut, toolpathkit.path.PathGeometry.Line(from, to), 1.0, 0.0001,
        new toolpathkit.path.Provenance(1, 1, 0))], new toolpathkit.tool.ToolLibrary(), [new toolpathkit.setup.Setup("1", origin)]);
      var lowered = ToolpathMotion.lower(program, machine.binding.machine);
      var motion = lowered.program;
      if (motion == null) throw "the diagonal move lowers to nothing";
      // From where the previous move left the machine: at the start of this one.
      var start = machine.stateAt(from.x, from.y, 0.01);
      var compiled = machine.binding.compiler.compile(motion, start, Int64.ofInt(1));
      var peak = [0.0, 0.0], worstTorque = 0.0, findings = 0, duration = 0.0;
      for (block in compiled.blocks) for (plan in block.plans) {
        duration += plan.durationSeconds;
        var steps = Math.ceil(plan.durationSeconds / 0.001);
        for (step in 0...steps + 1) {
          var state = plan.evaluate(Math.min(plan.durationSeconds, step * 0.001));
          for (i in 0...2) peak[i] = Math.max(peak[i], Math.abs(state.velocities[ids.indexOf(motors[i])]));
        }
        var checked = plan.checked;
        if (checked != null) {
          worstTorque = Math.max(worstTorque, checked.worstTorqueRatio);
          for (finding in checked.diagnostics) if (finding.kind != motionkit.trajectory.PlanDiagnostic.PlanDiagnosticKind.Accuracy) {
            findings++;
            report.push('${move.name}: ${finding.describe(0)}');
          }
        }
      }
      for (i in 0...2) check(peak[i] <= rates[i] * 1.001, '${move.name}: ${motors[i]} turns ${peak[i]} rad/s, its limit is ${rates[i]}');
      check(findings == 0, '${move.name}: the plan check finds ${report.join("; ")}');
      seconds.set(move.name, duration);
      Sys.println('motors: ${move.name} move ${Math.round(duration * 1000)} ms, motors at most ${Math.round(peak[0])} and ${Math.round(peak[1])} rad/s of ${Math.round(rates[0])}, worst torque ${Math.round(worstTorque * 100)} %');
      compiled.dispose();
    }
    machine.binding.compiler.planCheck = null;
  }

  /**
   * A whole sliced job through lowering and the compiler with the extruder joint: the heater barriers lower to a
   * target and a wait, the retractions to extruder-only moves, and the filament the extruder joint takes over the whole
   * plan is what the program lays (its beads and its retractions) to within 0.2 %.
   */
  static function extruderChecks(machine:PrinterAxes, settings:SlicerSettings, profile:PrinterProfile):Void {
    var part = TestParts.smallTab();
    var sliced = Slicer.slice(part, settings, profile, "tab");
    part.close();
    var program = new ToolpathProgram(sliced.program.ops, sliced.program.tools, [new toolpathkit.setup.Setup("1",
      new toolpathkit.path.Point3(PrinterAxes.ORIGIN_X, PrinterAxes.ORIGIN_Y, PrinterAxes.ORIGIN_Z))]);
    var lowered = ToolpathMotion.lower(program, machine.binding.machine);
    var motion = lowered.program;
    if (motion == null) throw "the tab lowers to nothing";
    var targets = 0, waits = 0, offsets = 0, driven = 0;
    for (op in motion.ops) switch op {
      case SetOutput(channel, _) if (StringTools.startsWith(channel, "heater.")): targets++;
      case WaitInput(channel, _, _) if (channel == "heater.hotend.at_temperature" || channel == "heater.bed.at_temperature"): waits++;
      case MoveJ(JointOffset(_, _), _, _): offsets++;
      case FollowPath(path, _, _, _) if (path.drive != null): driven++;
      case _:
    }
    check(targets >= 2 && waits == 2, 'the job heats the bed and the hotend and waits for both: $targets targets, $waits waits');
    check(offsets >= 2 && driven >= 3, 'retractions are extruder moves and beads drive the extruder: $offsets moves, $driven driven paths');
    var compiled = machine.binding.compiler.compile(motion, machine.start, Int64.ofInt(1));
    var extruder = machine.printer.extruderIndex, taken = 0.0;
    var shortest = 1e9, under = 0, total = 0, seconds = 0.0, plans = 0;
    var all:Array<Float> = [];
    for (block in compiled.blocks) for (plan in block.plans) {
      var durations = plan.segmentDurations();
      plans++;
      for (i in 0...durations.length()) {
        var ms = Int64.toInt(durations.get(i)) / 1e6;
        total++; shortest = Math.min(shortest, ms); seconds += ms / 1000; all.push(ms);
        if (ms < 1.0) under++;
      }
    }
    all.sort(Reflect.compare);
    Sys.println('segments: $total in $plans plans over ${seconds} s, shortest ${shortest} ms, median ${all[all.length >> 1]} ms, 90th ${all[Std.int(all.length * 0.9)]} ms, $under under 1 ms');
    for (block in compiled.blocks) for (plan in block.plans) {
      taken += plan.evaluate(plan.durationSeconds).positions[extruder] - plan.evaluate(0.0).positions[extruder];
    }
    var filament = sliced.program.depositedVolume() / machine.printer.filamentArea();
    for (op in sliced.program.ops) switch op { case FilamentMove(length, _, _): filament += length; case _: }
    var expected = machine.printer.radians(filament);
    Sys.println('extruder: the joint turns ${Math.round(taken * 1000) / 1000} rad for ${Math.round(expected * 1000) / 1000} rad of filament (${Math.round((taken / expected - 1) * 1e5) / 1e3} %)');
    check(Math.abs(taken / expected - 1.0) < 0.002, 'the extruder takes what the program lays: $taken against $expected rad');
    compiled.dispose();
  }

  static function check(ok:Bool, message:String):Void {
    if (!ok) throw message;
  }

  public static function main():Void {
    // `PRINTING_BENCHMARK_GC=f` sets the fraction of the heap that may be allocated between collections.
    var threshold = Sys.getEnv("PRINTING_BENCHMARK_GC");
    if (threshold != null) hl.Gc.setMarkThreshold(Std.parseFloat(threshold));
    var census = Sys.getEnv("PRINTING_BENCHMARK_CENSUS");
    if (census != null) hl.Gc.censusStart(4096);
    mainLoads();
    if (census != null) {
      hl.Gc.censusStop();
      hl.Gc.censusDump(cast haxe.io.Bytes.ofString(census + "/census.json").getData());
    }
  }

  static function mainLoads():Void {
    var only = Sys.getEnv("PRINTING_BENCHMARK_ONLY");
    var machine = new PrinterAxes();
    for (axis in machine.blueprint.axes) Sys.println('  axis ${axis.id}: ${axis.lowerLimit} .. ${axis.upperLimit}');
    Sys.println('printer axes: x ${Math.round(machine.speed[0] * 1e4) / 10} mm/s ${Math.round(machine.acceleration[0] * 100) / 100} m/s2, ' +
      'y ${Math.round(machine.speed[1] * 1e4) / 10} mm/s ${Math.round(machine.acceleration[1] * 100) / 100} m/s2, ' +
      'z ${Math.round(machine.speed[2] * 1e4) / 10} mm/s ${Math.round(machine.acceleration[2] * 100) / 100} m/s2');
    var settings = new SlicerSettings();
    var profile = PrinterProfile.bed220();
    // Without a named load the run is the gate, which the suites run; the loads are for measuring.
    if (only == null && Sys.getEnv("PRINTING_BENCHMARK_GCODE") == null || only == "gate") {
      gate(machine, settings, profile);
      return;
    }
    var dump = Sys.getEnv("PRINTING_BENCHMARK_DUMP");
    function sliceNamed(name:String, part:cadkit.modeling.Part):camkit.additive.Slicer.SliceResult {
      var sliced = Slicer.slice(part, settings, profile, name);
      part.close();
      if (dump != null)
        sys.io.File.saveContent('$dump/$name.gcode', PrinterWriter.write(sliced.program, new CncController(Printer), PrinterFlavour.Klipper));
      return sliced;
    }
    /** G-code written earlier (`PRINTING_BENCHMARK_DUMP`) stands in for slicing: `PRINTING_BENCHMARK_GCODE=file`. */
    var file = Sys.getEnv("PRINTING_BENCHMARK_GCODE");
    if (file != null) {
      var began = Sys.time();
      var read = CncCompiler.compileDetailed(sys.io.File.getContent(file), new CncController(Printer), new toolpathkit.path.Point3(0.0, 0.0, 0.01));
      for (item in read.diagnostics) if (item.severity == cnckit.CncDiagnostic.CncSeverity.Error) throw "G-code: " + item.message;
      report(file, machine, read.program, Sys.time() - began);
      return;
    }
    if (only == "all" || only == "gear") {
      var sliced = sliceNamed("gear", new SpurGear(1.5, 40, 8).geometry());
      report("gear (module 1.5, 40 teeth, 8 mm)", machine, sliced.program, sliced.report.seconds);
    }
    if (only == "all" || only == "cube") {
      var sliced = sliceNamed("cube", TestParts.calibrationCube());
      report("calibration cube", machine, sliced.program, sliced.report.seconds);
    }
    if (only == "all" || only == "gcode") {
      var sliced = sliceNamed("tower", TestParts.testTower());
      var controller = new CncController(Printer);
      var text = PrinterWriter.write(sliced.program, controller, PrinterFlavour.Klipper);
      var began = Sys.time();
      var read = CncCompiler.compileDetailed(text, new CncController(Printer), new toolpathkit.path.Point3(0.0, 0.0, 0.01));
      for (item in read.diagnostics) if (item.severity == cnckit.CncDiagnostic.CncSeverity.Error) throw "G-code round trip: " + item.message;
      Sys.println('  G-code: ${text.split("\n").length} lines, read back in ${Math.round((Sys.time() - began) * 1000)} ms');
      report("test tower from G-code", machine, read.program, Sys.time() - began);
    }
  }
}
