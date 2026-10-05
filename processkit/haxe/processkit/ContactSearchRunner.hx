package processkit;

import haxe.Int64;
import motionkit.kinematics.Twist6;
import motionkit.robot.ServoSession;
import processkit.perception.ContactSearch;
import robotkit.core.SensorFrame;
import robotkit.spatial.Vec3;

/** Executes one prepared arc-off probe through the arm's exclusive servo owner. */
class ContactSearchRunner {
  public final servo:ServoSession;
  public final search:ContactSearch;
  public final sensor:String;
  public var stopped(default, null):Bool = false;
  var sequence = 0;
  var previous:Null<Int64> = null;

  /** A fresh servo session takes over after the caller approaches the CAD region with arc and wire off. */
  public function new(servo:ServoSession, sensor:String, direction:Vec3, distance:Float, speed:Float,
      maxAgeSeconds:Float = 0.02, lateralTolerance:Float = 0.002, contactOffset:Float = 0.0) {
    if (servo == null || sensor == null || sensor.length == 0)
      throw "Contact search runner needs a servo owner and the torch sensor";
    if (servo.robot.snapshot().trajectoryActive)
      throw "Contact search cannot take over an active motion program";
    this.servo = servo; this.sensor = sensor;
    var snapshot = servo.robot.snapshot();
    var joints = [for (index in servo.manipulator.jointIndices()) snapshot.positions.get(index)];
    search = new ContactSearch(servo.manipulator.tcpPose(joints).translation, direction, distance, speed,
      maxAgeSeconds, lateralTolerance, contactOffset);
  }

  /** Contact is captured before braking from measured FK, rather than the stopping setpoint. */
  public function update():Void {
    if (stopped) return;
    var snapshot = servo.robot.snapshot();
    var now = snapshot.sourceTimestampNs;
    if (previous != null && now == cast(previous, Int64)) return;
    previous = now;
    try {
      var joints = [for (index in servo.manipulator.jointIndices()) snapshot.positions.get(index)];
      var point = servo.manipulator.tcpPose(joints).translation;
      var frame:Null<SensorFrame> = null;
      for (index in 0...snapshot.sensors.length) {
        var candidate = snapshot.sensors.get(index);
        if (candidate.sensorId == sensor) frame = candidate;
      }
      search.observe(now, snapshot.sourceClockId, point, frame);
      if (search.running()) {
        var velocity = search.velocity();
        sequence++;
        var rejected = servo.command(new Twist6(velocity.x, velocity.y, velocity.z, 0, 0, 0), sequence,
          now + search.maxAgeNs);
        if (rejected != null) throw 'Contact search servo command rejected: $rejected';
      } else servo.stop();
      var tick = servo.update();
      if (!search.running() && tick.atRest) stopped = true;
    } catch (error:Dynamic) {
      search.cancel();
      servo.stop();
      throw error;
    }
  }

  public function cancel():Void { search.cancel(); servo.stop(); }
  public function completed():Bool return stopped && search.contact != null && search.failure == null;
}
