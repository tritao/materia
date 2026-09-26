package robotkit.skill;

import robotkit.world.RobotSnapshot;
import robotkit.perception.PointCloud;

/**
 * Dwells for a configured number of `update()` calls (a stationary scan
 * pass takes real time on a real sensor) then captures one `PointCloud` from
 * the supplied `scan` callback. `scan` carries every scanner-specific
 * parameter (which surface, sensor pose, noise, seed) as a closure so this
 * skill stays independent of any one scanning technique — a real LiDAR scan
 * accumulation, `robotkit.perception.SimulatedSurfaceScanner.scan`, or a
 * replayed cloud can all be supplied the same way. `scan` must be
 * deterministic given the observations already available to the caller (no
 * wall-clock or hidden randomness), so this skill replays identically.
 */
class ScanSurface implements Skill {
  public final scan:Void->PointCloud;
  public final dwellUpdates:Int;
  /** The captured cloud once this skill succeeds; null until then. */
  public var cloud(default, null):Null<PointCloud> = null;

  final lifecycle:SkillLifecycle = new SkillLifecycle();
  var remaining:Int = 0;

  public function new(scan:Void->PointCloud, ?dwellUpdates:Int = 1) {
    if (scan == null) throw "ScanSurface requires a scan callback";
    if (dwellUpdates < 0) throw "ScanSurface dwell updates cannot be negative";
    this.scan = scan;
    this.dwellUpdates = dwellUpdates;
  }

  public function start():Void {
    lifecycle.begin();
    cloud = null;
    remaining = dwellUpdates;
    if (remaining <= 0) complete();
  }

  public function update(snapshot:RobotSnapshot, durationSeconds:Float):SkillStatus {
    if (!lifecycle.isRunning()) return lifecycle.status();
    if (snapshot == null || !Math.isFinite(durationSeconds) || durationSeconds <= 0.0) {
      lifecycle.fail("ScanSurface update requires a robot snapshot and positive finite duration");
      return lifecycle.status();
    }
    remaining--;
    if (remaining <= 0) complete();
    return lifecycle.status();
  }

  public function cancel():Void {
    if (!lifecycle.isRunning()) return;
    lifecycle.cancel();
  }

  public function status():SkillStatus return lifecycle.status();
  public function result():Null<SkillResult> return lifecycle.result();

  function complete():Void {
    try {
      var value = scan();
      if (value == null) throw "scan callback returned no point cloud";
      cloud = value;
      lifecycle.succeed('captured ${value.size()} points');
    } catch (error:Dynamic) {
      lifecycle.fail(Std.string(error));
    }
  }
}
