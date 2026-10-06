package robotkit.manipulation;

import kinematicskit.KinematicSnapshot;
import kinematicskit.KinematicState;
import kinematicskit.SolverWorkspace;
import kinematicskit.SwivelTask;

/**
 * What evaluating a `KinematicGroup` writes: the model state, its snapshot and the solvers'
 * scratch. The group stays unchanged, so one group serves any number of threads, each evaluating
 * in data of its own (`KinematicGroup.threadLocalData`, or `newData` for a caller that keeps it).
 */
class KinematicGroupData {
  public final group:KinematicGroup;
  public final state:KinematicState;
  public final snapshot:KinematicSnapshot;
  public final workspace:SolverWorkspace;
  /** Numeric pose queries evaluated in this caller-owned or thread-local context. */
  public var numericSolves:Int = 0;
  /** Measures the swivel of a redundant arm (it keeps scratch of its own); null without one. */
  public final swivelProbe:Null<SwivelTask>;

  /** Made by `KinematicGroup.newData`, which knows the group's swivel. */
  public function new(group:KinematicGroup, swivelProbe:Null<SwivelTask>) {
    this.group = group;
    this.swivelProbe = swivelProbe;
    state = new KinematicState(group.model);
    snapshot = new KinematicSnapshot(group.model);
    workspace = new SolverWorkspace();
  }
}
