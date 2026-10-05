package robotkit.skill;

import robotkit.localization.Localization;
import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;
import robotkit.spatial.Quat;
import robotkit.core.RobotSnapshot;

/**
 * Picks a part up with a vacuum tool, or sets the held part down. `contact` gives, in the map frame,
 * where the tool meets: the part's grasp point to pick, or where its contact must be for the held part
 * to rest on its seat to place. Optional `orientation` gives the tool rotation in the same map frame;
 * omission leaves spin free. Both are read when the skill starts, so they follow the part wherever it
 * is; the robot's place comes from `localization`, and its base must stand still meanwhile. The arm's
 * program switches the tool's channel on the robot, as it would a real ejector valve. With the tool's
 * vacuum sensor the skill tells the outcome as a real cell does: picking succeeds when the sensor reads
 * at least `holdThresholdKpa` afterwards, placing when it reads less; without one it trusts the program.
 */
class HandlePart implements Skill {
  public final runner:HandlingRunner;
  public final hold:Bool;
  public final vacuumSensor:Null<String>;
  public final holdThresholdKpa:Float;

  final localization:Localization;
  final contact:Void -> Vec3;
  final orientation:Null<Void -> Quat>;
  final lifecycle = new SkillLifecycle();

  public static function pick(runner:HandlingRunner, localization:Localization, grasp:Void -> Vec3,
      ?vacuumSensor:String, holdThresholdKpa:Float = 40.0, ?orientation:Void -> Quat):HandlePart
    return new HandlePart(runner, localization, grasp, true, vacuumSensor, holdThresholdKpa, orientation);

  public static function place(runner:HandlingRunner, localization:Localization, contact:Void -> Vec3,
      ?vacuumSensor:String, holdThresholdKpa:Float = 40.0, ?orientation:Void -> Quat):HandlePart
    return new HandlePart(runner, localization, contact, false, vacuumSensor, holdThresholdKpa, orientation);

  public function new(runner:HandlingRunner, localization:Localization, contact:Void -> Vec3, hold:Bool,
      ?vacuumSensor:String, holdThresholdKpa:Float = 40.0, ?orientation:Void -> Quat) {
    if (runner == null || localization == null || contact == null || !(holdThresholdKpa > 0))
      throw "HandlePart needs a runner, localization, contact point and a positive hold threshold";
    this.runner = runner;
    this.localization = localization;
    this.contact = contact;
    this.orientation = orientation;
    this.hold = hold;
    this.vacuumSensor = vacuumSensor;
    this.holdThresholdKpa = holdThresholdKpa;
  }

  public function start():Void {
    lifecycle.begin();
    try {
      var state = localization.state();
      if (state == null) throw "HandlePart has no robot pose";
      var base_T_map = Transform3.fromPose2(state.pose).inverse();
      var requested = orientation;
      runner.run(base_T_map.transformPoint(contact()), hold,
        requested == null ? null : base_T_map.rotation.multiply(requested()));
    } catch (error:Dynamic) {
      lifecycle.fail(Std.string(error));
    }
  }

  public function update(snapshot:RobotSnapshot, durationSeconds:Float):SkillStatus {
    if (!lifecycle.isRunning()) return lifecycle.status();
    try {
      // The program's channel events act on the robot's tool itself; nothing here consumes them.
      runner.update(durationSeconds);
      var failure = runner.failure();
      if (failure != null) fail(failure);
      else if (runner.completed()) {
        var sensed = holding(snapshot);
        if (sensed == null || sensed == hold) lifecycle.succeed(hold ? "part picked" : "part placed");
        else lifecycle.fail(hold ? "the vacuum found no seal" : "the vacuum still holds the part");
      }
    } catch (error:Dynamic) {
      fail(Std.string(error));
    }
    return lifecycle.status();
  }

  public function cancel():Void {
    if (!lifecycle.isRunning()) return;
    runner.abort();
    lifecycle.cancel();
  }

  public function status():SkillStatus return lifecycle.status();
  public function result():Null<SkillResult> return lifecycle.result();

  /** Whether the vacuum sensor reads a seal, or null without a sensor. */
  function holding(snapshot:RobotSnapshot):Null<Bool> {
    var sensor = vacuumSensor;
    if (sensor == null) return null;
    for (index in 0...snapshot.sensors.length) {
      var frame = snapshot.sensors.get(index);
      if (frame.sensorId == sensor) return frame.values.get(0) >= holdThresholdKpa;
    }
    throw 'Vacuum sensor "$sensor" has not reported';
  }

  function fail(message:String):Void {
    try runner.abort() catch (_:Dynamic) {}
    lifecycle.fail(message);
  }
}
