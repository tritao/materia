package processkit.perception;

import haxe.Int64;
import robotkit.core.SensorFrame;
import robotkit.spatial.Vec3;
import processkit.tool.WeldSensor;
import robotkit.time.ClockMapping;
import robotkit.time.ClockMappings;
import robotkit.time.MappedTimestamp;

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
  var jointClock:Null<String> = null;
  var sensorClock:Null<String> = null;

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
  public function observe(now:Int64, clock:String, point:Vec3, frame:Null<SensorFrame>,
      ?mappings:ClockMappings):Void {
    if (!running()) return;
    if (clock == null || clock.length == 0 || clock == "unspecified" ||
        (jointClock != null && jointClock != clock)) {
      fail("contact search joint clock is unknown or changed epoch"); return;
    }
    jointClock = clock;
    if (previous != null && now <= cast(previous, Int64)) { fail("contact search clock did not advance"); return; }
    previous = now;
    if (began == null) began = now;
    if (frame == null || frame.kind != WeldSensor.KIND || frame.sourceTimestampNs < Int64.ofInt(0) ||
        frame.sourceClockId == "unspecified" ||
        frame.sourceClockId == null || frame.sourceClockId.length == 0 ||
        (sensorClock != null && sensorClock != frame.sourceClockId)) {
      fail("contact search needs fresh welding feedback on the joint clock"); return;
    }
    var mapped:Null<MappedTimestamp> = frame.sourceClockId == clock
      ? new MappedTimestamp(frame.sourceTimestampNs, Int64.ofInt(0))
      : mappings == null ? null : mappings.map(frame.sourceTimestampNs, frame.sourceClockId, clock);
    if (mapped == null) { fail("contact search needs a valid sensor-to-joint clock mapping"); return; }
    var earliest = ClockMapping.checkedSubtract(mapped.valueNs, mapped.errorBoundNs);
    var latest = ClockMapping.checkedAdd(mapped.valueNs, mapped.errorBoundNs);
    var age = earliest == null ? null : ClockMapping.checkedSubtract(now, earliest);
    // The whole timestamp interval must precede FK and lie within the age budget.
    // Thus mapping uncertainty counts toward the same speed*age contact-position bound.
    if (earliest == null || latest == null || age == null || latest > now || age > maxAgeNs) {
      fail("contact search needs fresh welding feedback within its clock uncertainty bound"); return;
    }
    sensorClock = frame.sourceClockId;
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
    if (sequence != null && (frame.sequence < cast(sequence, Int64) || mapped.valueNs < cast(sampleTime, Int64))) {
      fail("contact search feedback moved backwards"); return;
    }
    if (now - cast(began, Int64) > Int64.fromFloat((distance / speed + 2.0) * 1e9)) {
      fail("contact search exhausted its time bound without touch"); return;
    }
    var fresh = sequence == null || frame.sequence > cast(sequence, Int64);
    if (fresh) {
      if (sampleTime != null && mapped.valueNs <= cast(sampleTime, Int64)) {
        fail("contact search feedback timestamp did not advance"); return;
      }
      sequence = frame.sequence; sampleTime = mapped.valueNs;
      if (!armed && reading.touch) { fail("contact search began with the wire already touching"); return; }
      if (reading.touch) {
        contact = point.add(direction.scale(contactOffset)); armed = false; return;
      }
      armed = true;
    }
    if (depth >= distance) fail("contact search exhausted its distance bound without touch");
  }
}
