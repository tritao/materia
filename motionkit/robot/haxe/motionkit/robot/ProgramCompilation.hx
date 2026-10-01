package motionkit.robot;

import haxe.Int64;
import motionkit.event.EventValue;
import motionkit.program.MotionProgram;

/**
  A program being planned one op at a time, so execution can start on the
  first plans while later ones are still to be made. Each step delivers what
  it plans to `sink`, which owns the plans from then on; a compilation owns
  only the motion it has not finished planning.
**/
@:allow(motionkit.robot.ProgramCompiler)
class ProgramCompilation {
  public final compiler:ProgramCompiler;
  public final program:MotionProgram;
  public final sink:ProgramSink;
  /** Whether every op is planned. */
  public var done(default, null):Bool = false;
  /** The speed of paths planned from now on, as a fraction of their programmed speed. */
  public var speedScale:Float;

  var q:Array<Float>;
  var nextId:Int64;
  var cursor:Int;
  var currentIndex:Int = -1;
  var skipNext:Bool = false;
  var pending:Null<ProgramCompiler.PendingMotion> = null;
  var leadingOutputs:Array<{channel:String, value:EventValue}> = [];

  public function new(compiler:ProgramCompiler, program:MotionProgram, initialQ:Array<Float>,
      firstPlanId:Int64, firstOp:Int, speedScale:Float, sink:ProgramSink) {
    if (sink == null) throw "Program compilation needs a sink";
    if (program == null || initialQ == null || initialQ.length != compiler.solver.jointCount())
      throw "Program compiler needs a program and complete start position";
    for (value in initialQ) if (!Math.isFinite(value)) throw "Non-finite program start position";
    if (!Math.isFinite(speedScale) || speedScale <= 0.0)
      throw "Program speed scale must be finite and positive";
    this.compiler = compiler;
    this.program = program;
    this.q = initialQ.copy();
    this.nextId = firstPlanId;
    this.cursor = firstOp;
    this.speedScale = speedScale;
    this.sink = sink;
  }

  /** Plans the next op. Returns false once the whole program is planned. */
  public function step():Bool return compiler.advance(this);

  /** The id the next plan will take. */
  public function nextPlanId():Int64 return nextId;

  /** Disposes the motion not yet delivered; delivered plans belong to the sink. */
  public function dispose():Void {
    var unfinished = pending;
    if (unfinished != null) unfinished.trajectory.dispose();
    pending = null;
  }
}
