package processkit;

import motionkit.program.MotionOp;

/**
 * What engages a process before its path and disengages it after: motion-program operations that run around the
 * process span, and that a restart runs again, since each restart re-engages. A welder strikes its arc and waits for
 * it to establish before the torch travels (`entry`), and fills the crater, ends the wire and the arc after (`exit`);
 * a sprayer or a dispenser needs neither, and has none.
 *
 * `entry` runs once the tool stands at the start of the span, so its outputs fire there; barriers such as a dwell
 * or an input wait may follow them. `exit` runs once the path is done, with the tool still at its end.
 *
 * With an `exit`, the process outputs the path's events drive are not zeroed at the path's end: the exit does that,
 * in its own order, and the run ends when its caller says the program is done (`ProcessRun.finish`) rather than when
 * the path is. Without one, the run ends at the path's end, and the process output goes to zero there.
 */
class ProcessEngagement {
  public final entry:Array<MotionOp>;
  public final exit:Array<MotionOp>;
  /** Recovery may re-engage gently instead of repeating the initial dwell. */
  public final recoveryEntry:Array<MotionOp>;

  public function new(entry:Array<MotionOp>, exit:Array<MotionOp>, ?recoveryEntry:Array<MotionOp>) {
    if (entry == null || exit == null) throw "Process engagement needs entry and exit operations (either may be empty)";
    this.entry = entry.copy();
    this.exit = exit.copy();
    this.recoveryEntry = recoveryEntry == null ? entry.copy() : recoveryEntry.copy();
  }
}
