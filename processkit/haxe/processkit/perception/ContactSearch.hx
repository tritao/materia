package processkit.perception;

import haxe.Int64;
import robotkit.core.SensorFrame;
import robotkit.spatial.Vec3;
import processkit.tool.WeldSensor;

/** Bounded arc-off probing policy. Motion and joint FK remain with the caller. */
class ContactSearch {
  public final origin:Vec3;
  public final direction:Vec3;
  public final distance:Float;
  public final speed:Float;
  public final maxAgeNs:Int64;
  public final lateralTolerance:Float;
  /** Calibrated distance from the sensing threshold to the material, along the search direction. */
  public final contactOffset:Float;
  public var contact(default, null):Null<Vec3> = null;
  public var failure(default, null):Null<String> = null;
  var began:Null<Int64> = null;
  var previous:Null<Int64> = null;
  var sequence:Null<Int64> = null;
  var sampleTime:Null<Int64> = null;
  var armed = false;

  public function new(origin:Vec3, direction:Vec3, distance:Float, speed:Float,
      maxAgeSeconds:Float = 0.02, lateralTolerance:Float = 0.002, contactOffset:Float = 0.0) {
    if (origin == null || direction == null || !Math.isFinite(direction.norm()) || direction.norm() < 1e-12 ||
        !Math.isFinite(distance) || !(distance > 0) || !Math.isFinite(speed) || !(speed > 0) ||
        !Math.isFinite(maxAgeSeconds) || !(maxAgeSeconds > 0) || !Math.isFinite(lateralTolerance) || !(lateralTolerance > 0) ||
        !Math.isFinite(contactOffset) || contactOffset < 0 || contactOffset > lateralTolerance)
      throw "Contact search needs finite geometry, positive bounds and a calibrated contact offset";
    this.origin = origin; this.direction = direction.normalized(); this.distance = distance; this.speed = speed;
    this.maxAgeNs = Int64.fromFloat(maxAgeSeconds * 1e9);
    this.lateralTolerance = lateralTolerance; this.contactOffset = contactOffset;
  }

  public function running():Bool return contact == null && failure == null;
  public function velocity():Vec3 return running() && armed ? direction.scale(speed) : new Vec3();
  public function cancel():Void if (running()) failure = "contact search cancelled";
  function fail(reason:String):Void { failure = reason; armed = false; }

  /** Point is measured joint FK on now's clock; the maximum sensor skew bounds contact-position error. */
  public function observe(now:Int64, clock:String, point:Vec3, frame:Null<SensorFrame>):Void {
    if (!running()) return;
    if (previous != null && now <= cast(previous, Int64)) { fail("contact search clock did not advance"); return; }
    previous = now;
    if (began == null) began = now;
    if (frame == null || frame.kind != WeldSensor.KIND || frame.sourceClockId != clock ||
        frame.sourceTimestampNs > now || now - frame.sourceTimestampNs > maxAgeNs) {
      fail("contact search needs fresh welding feedback on the joint clock"); return;
    }
    var values = [for (i in 0...frame.values.length) frame.values.get(i)];
    if (!WeldSensor.valid(values)) { fail("contact search received invalid welding feedback"); return; }
    var reading = WeldSensor.reading(values);
    if (reading.arc || reading.fault != 0 || reading.currentA > 0) {
      fail("contact search requires the arc off and the welder free of faults"); return;
    }
    if (point == null) { fail("contact search needs measured joint FK"); return; }
    var delta = point.sub(origin);
    var depth = delta.dot(direction);
    if (delta.sub(direction.scale(depth)).norm() > lateralTolerance || depth < -lateralTolerance || depth > distance) {
      fail("contact search left its bounded probe corridor"); return;
    }
    if (sequence != null && (frame.sequence < cast(sequence, Int64) || frame.sourceTimestampNs < cast(sampleTime, Int64))) {
      fail("contact search feedback moved backwards"); return;
    }
    if (now - cast(began, Int64) > Int64.fromFloat((distance / speed + 2.0) * 1e9)) {
      fail("contact search exhausted its time bound without touch"); return;
    }
    var fresh = sequence == null || frame.sequence > cast(sequence, Int64);
    if (fresh) {
      if (sampleTime != null && frame.sourceTimestampNs <= cast(sampleTime, Int64)) {
        fail("contact search feedback timestamp did not advance"); return;
      }
      sequence = frame.sequence; sampleTime = frame.sourceTimestampNs;
      if (!armed && reading.touch) { fail("contact search began with the wire already touching"); return; }
      if (reading.touch) {
        contact = point.add(direction.scale(contactOffset)); armed = false; return;
      }
      armed = true;
    }
    if (depth >= distance) fail("contact search exhausted its distance bound without touch");
  }
}
