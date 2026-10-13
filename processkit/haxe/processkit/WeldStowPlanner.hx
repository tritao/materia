package processkit;

import haxe.Int64;
import motionkit.MotionOptions;
import motionkit.program.Blend;
import motionkit.program.MotionOp;
import motionkit.program.MotionProgram;
import motionkit.program.MoveTarget;
import motionkit.robot.ProgramCompiler;
import robotkit.manipulation.ClearanceWorld;
import robotkit.manipulation.JointRoute;
import robotkit.manipulation.Manipulator;

/** Compile a live collision-checked return to the arm's CAD ready configuration. */
class WeldStowPlanner {
  /**
   * Plan a stow from the observed joint state to the zero-relative CAD ready posture.
   * The caller supplies clearance built from the current cell poses, including deposited beads.
   */
  public static function plan(arm:Manipulator, compiler:ProgramCompiler, clearance:ClearanceWorld,
      from:Array<Float>, to:Array<Float>):MotionProgram {
    if (arm == null || compiler == null || clearance == null || from == null || to == null ||
        compiler.solver.jointCount() != arm.group.count() || from.length != arm.group.count() || to.length != from.length)
      throw "Weld stow needs matching arm, compiler, live clearance and endpoint configurations";
    var lower = [for (joint in 0...arm.group.count()) arm.group.limitsOf(joint).lower];
    var upper = [for (joint in 0...arm.group.count()) arm.group.limitsOf(joint).upper];

    function clearMove(start:Array<Float>, end:Array<Float>):Bool {
      if (clearance.violation(start) != null) return false;
      var program = new MotionProgram([MotionOp.MoveJ(MoveTarget.JointTarget(end), new MotionOptions(), Blend.ExactStop)]);
      var compiled:Null<motionkit.robot.CompiledProgram> = null;
      try {
        compiled = compiler.compile(program, start, Int64.ofInt(1));
        for (block in compiled.blocks) for (trajectory in block.plans) {
          var proof = motionkit.robot.TrajectoryClearanceProof.run(clearance, trajectory,
            compiler.maxVelocity, false);
          if (!proof.closed || proof.failure != null) throw "Weld stow trajectory has no continuous clearance certificate";
        }
        compiled.dispose();
        return true;
      } catch (_:Dynamic) {
        if (compiled != null) compiled.dispose();
        return false;
      }
    }

    var route = JointRoute.plan(from, to, lower, upper, clearMove);
    return new MotionProgram([for (index in 1...route.length)
      MotionOp.MoveJ(MoveTarget.JointTarget(route[index]), new MotionOptions(), Blend.ExactStop)]);
  }
}
