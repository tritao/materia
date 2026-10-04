package robotkit.runtime;

/**
 * Bounded latest-value publication for runtime observations.
 *
 * Publishing replaces one reference; a slow consumer never grows a queue and
 * always receives the newest complete snapshot on its next read.
 */
class RobotSnapshotMailbox {
  var current:Null<RobotSnapshot> = null;

  public function new() {}

  public function publish(snapshot:RobotSnapshot):Void {
    if (snapshot == null)
      throw "RobotSnapshot mailbox cannot publish null";
    current = snapshot;
  }

  public function latest():Null<RobotSnapshot>
    return current;
}
