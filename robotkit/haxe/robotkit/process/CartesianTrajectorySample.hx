package robotkit.process;

import robotkit.spatial.Transform3;

/** One time-parameterized TCP sample of a `CartesianTrajectory`. */
class CartesianTrajectorySample {
  public final work_T_tcp:Transform3;
  public final time:Float;
  public final processOn:Bool;
  /** Index of the toolpath point this sample's segment departs from. */
  public final segmentIndex:Int;

  public function new(work_T_tcp:Transform3, time:Float, processOn:Bool, segmentIndex:Int) {
    this.work_T_tcp = work_T_tcp;
    this.time = time;
    this.processOn = processOn;
    this.segmentIndex = segmentIndex;
  }
}
