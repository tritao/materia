package robotkit.behavior;

/** Haxeon behavior executed by robotd from published robot snapshots. */
interface RobotBehavior {
  function update(context:RobotContext):Void;
}
