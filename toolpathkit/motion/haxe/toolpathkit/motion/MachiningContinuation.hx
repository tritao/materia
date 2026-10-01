package toolpathkit.motion;

import motionkit.program.MotionProgram;

/**
  A program that resumes machining partway along op `opIndex` of the
  original, from `startDistance` along it: some approach ops, that op's
  remainder at `resumeOp`, then the original's later ops.
**/
class MachiningContinuation {
  public final program:MotionProgram;
  public final opIndex:Int;
  public final startDistance:Float;
  public final resumeOp:Int;

  public function new(program:MotionProgram, opIndex:Int, startDistance:Float, resumeOp:Int) {
    this.program = program;
    this.opIndex = opIndex;
    this.startDistance = startDistance;
    this.resumeOp = resumeOp;
  }

  /** The original op and distance along it for `op` and `distance` of this program; op -1 while approaching. */
  public function originalAt(op:Int, distance:Float):{op:Int, distance:Float} {
    if (op < resumeOp) return {op: -1, distance: 0.0};
    if (op == resumeOp) return {op: opIndex, distance: startDistance + distance};
    return {op: opIndex + op - resumeOp, distance: distance};
  }
}
