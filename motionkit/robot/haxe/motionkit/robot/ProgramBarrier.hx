package motionkit.robot;

import motionkit.program.InputPredicate;

/** Host-side condition between independently executable plan blocks. */
enum ProgramBarrier {
  Dwell(seconds:Float);
  WaitInput(channel:String, predicate:InputPredicate, timeoutSeconds:Float);
}
