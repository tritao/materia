package robotkit.behavior;

/** Application behavior that only sees RobotWorld's transport-neutral state. */
interface WorldBehavior {
  function update(context:WorldBehaviorContext):Void;
}
