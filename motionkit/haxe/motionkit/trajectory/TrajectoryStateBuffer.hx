package motionkit.trajectory;

import TrajectoryCore;
#if (target.threaded && !eval)
import sys.thread.Mutex;
#end

/**
 * One shared native output buffer for trajectory and plan evaluations.
 *
 * A `mk_trajectory_state` is 2 KB because its arrays are sized for `MK_MAX_JOINTS`, however many joints a
 * robot has. Allocating and zeroing one per evaluation dominated a simulation tick, so evaluations borrow
 * this buffer instead: `acquire`, make the native call, `copyOut`, and always `release`. The native call
 * sets `joint_count` and writes only that many entries, and `copyOut` reads no more than that, so stale
 * entries beyond it are never observed.
 */
class TrajectoryStateBuffer {
  static final state = createState();

  #if (target.threaded && !eval)
  static final mutex = new Mutex();
  #end

  static function createState():mk_trajectory_state {
    var value = new mk_trajectory_state();
    value.set_struct_size(mk_trajectory_state.size());
    return value;
  }

  /** Locks the shared buffer. Every `acquire` needs exactly one `release`, including when the call throws. */
  public static function acquire():mk_trajectory_state {
    #if (target.threaded && !eval)
    mutex.acquire();
    #end
    state.set_struct_size(mk_trajectory_state.size());
    return state;
  }

  public static function release():Void {
    #if (target.threaded && !eval)
    mutex.release();
    #end
  }

  /** Copies the joint values the last native call wrote. Call between `acquire` and `release`. */
  public static function copyOut():TrajectoryState {
    var positions:Array<Float> = [];
    var velocities:Array<Float> = [];
    var accelerations:Array<Float> = [];
    var jerks:Array<Float> = [];
    for (joint in 0...state.get_joint_count()) {
      positions.push(state.get_position(joint));
      velocities.push(state.get_velocity(joint));
      accelerations.push(state.get_acceleration(joint));
      jerks.push(state.get_jerk(joint));
    }
    return new TrajectoryState(positions, velocities, accelerations, jerks);
  }
}
