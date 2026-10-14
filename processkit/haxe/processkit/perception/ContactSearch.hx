package processkit.perception;

import haxe.Int64;
import robotkit.core.SensorFrame;
import robotkit.spatial.Vec3;
import processkit.tool.WeldSensor;
import robotkit.time.ClockMapping;
import robotkit.time.ClockMappings;
import robotkit.time.MappedTimestamp;

/** Bounded contact probing policy. Motion and joint FK remain with the caller. */
class ContactSearch {
  public final origin:Vec3;
  public final direction:Vec3;
  public final distance:Float;
  public final speed:Float;
  public final maxAgeNs:Int64;
  public final lateralTolerance:Float;
  /** Calibrated distance from the sensing threshold to the material, along the search direction. */
  public final contactOffset:Float;
  /** Maximum measured contact travel past the nominal endpoint while fresh feedback is stopping the servo. */
  public final maximumOvershoot:Float;
  public var contact(default, null):Null<Vec3> = null;
  public var measurement(default, null):Null<ContactMeasurement> = null;
  public var failure(default, null):Null<String> = null;
  final sensorKind:String;
  final coordinateFrame:String;
  final calibratedError:Float;
  var began:Null<Int64> = null;
  var previous:Null<Int64> = null;
  var sequence:Null<Int64> = null;
  var sampleTime:Null<Int64> = null;
  var endpointReachedAt:Null<Int64> = null;
  var armed = false;
  var awaitingEndpointContact = false;
  var jointClock:Null<String> = null;
  var sensorClock:Null<String> = null;

  public function new(origin:Vec3, direction:Vec3, distance:Float, speed:Float,
      maxAgeSeconds:Float = 0.02, lateralTolerance:Float = 0.002, contactOffset:Float = 0.0,
      sensorKind:String = WeldSensor.KIND, coordinateFrame:String = "root", calibratedError:Float = 0.0,
      maximumOvershoot:Float = 0.0) {
    if (origin == null || direction == null || !Math.isFinite(direction.norm()) || direction.norm() < 1e-12 ||
        !Math.isFinite(distance) || !(distance > 0) || !Math.isFinite(speed) || !(speed > 0) ||
        !Math.isFinite(maxAgeSeconds) || !(maxAgeSeconds > 0) || !Math.isFinite(lateralTolerance) || !(lateralTolerance > 0) ||
        !Math.isFinite(contactOffset) || contactOffset < 0 || contactOffset > lateralTolerance ||
        (sensorKind != WeldSensor.KIND && sensorKind != "tool_contact") || coordinateFrame == null || coordinateFrame.length == 0 ||
        !Math.isFinite(calibratedError) || calibratedError < 0 || !Math.isFinite(maximumOvershoot) || maximumOvershoot < 0 ||
        maximumOvershoot > speed * maxAgeSeconds)
      throw "Contact search needs finite geometry, positive bounds and a calibrated contact offset";
    this.origin = origin; this.direction = direction.normalized(); this.distance = distance; this.speed = speed;
    this.maxAgeNs = Int64.fromFloat(maxAgeSeconds * 1e9);
    this.lateralTolerance = lateralTolerance; this.contactOffset = contactOffset;
    this.maximumOvershoot = maximumOvershoot;
    this.sensorKind = sensorKind; this.coordinateFrame = coordinateFrame; this.calibratedError = calibratedError;
  }

  public function running():Bool return contact == null && failure == null;
  public function velocity():Vec3 return running() && armed && !awaitingEndpointContact ? direction.scale(speed) : new Vec3();
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
    if (frame == null || frame.kind != sensorKind || frame.sourceTimestampNs < Int64.ofInt(0) ||
        frame.sourceClockId == "unspecified" ||
        frame.sourceClockId == null || frame.sourceClockId.length == 0 ||
        (sensorClock != null && sensorClock != frame.sourceClockId)) {
      fail("contact search needs fresh feedback on its declared sensor and joint clocks"); return;
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
    if (earliest == null || latest == null || age == null) {
      fail("contact search sensor timestamp interval overflowed while checking freshness"); return;
    }
    if (latest > now || age > maxAgeNs) {
      var reason = latest > now ? "extends beyond joint FK time" : "exceeds the maximum age";
      fail('contact search sensor interval [${Int64.toStr(earliest)}, ${Int64.toStr(latest)}] ns $reason '
        + '(joint FK ${Int64.toStr(now)} ns, maximum age ${Int64.toStr(maxAgeNs)} ns)');
      return;
    }
    sensorClock = frame.sourceClockId;
    var values = [for (i in 0...frame.values.length) frame.values.get(i)];
    var touching = false;
    if (sensorKind == WeldSensor.KIND) {
      if (!WeldSensor.valid(values)) { fail("contact search received invalid welding feedback"); return; }
      var reading = WeldSensor.reading(values);
      if (reading.arc || reading.fault != 0 || reading.currentA > 0) {
        fail("contact search requires the arc off and the welder free of faults"); return;
      }
      touching = reading.touch;
    } else {
      if (frame.image != null || values.length != 1 || (values[0] != 0.0 && values[0] != 1.0)) {
        fail("contact search received invalid digital tool-contact feedback"); return;
      }
      touching = values[0] == 1.0;
    }
    if (point == null) { fail("contact search needs measured joint FK"); return; }
    var delta = point.sub(origin);
    var depth = delta.dot(direction);
    var lateral = delta.sub(direction.scale(depth)).norm();
    if (lateral > lateralTolerance || depth < -lateralTolerance || depth > distance + maximumOvershoot) {
      fail('contact search left its bounded probe corridor (lateral $lateral m / $lateralTolerance m, axial $depth m / ${distance + maximumOvershoot} m)'); return;
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
      if (!armed && touching) { fail("contact search began with the probe already touching"); return; }
      if (touching) {
        contact = point.add(direction.scale(contactOffset));
        var lagNs = now - mapped.valueNs;
        var uncertainty = calibratedError + speed * Int64.toFloat(Int64.add(lagNs, mapped.errorBoundNs)) * 1e-9;
        measurement = new ContactMeasurement(frame.sensorId, frame.frameId, frame.kind, coordinateFrame,
          frame.sequence, frame.sourceTimestampNs, frame.sourceClockId, mapped.valueNs, mapped.errorBoundNs,
          contact, uncertainty);
        armed = false; return;
      }
      armed = true;
    }
    if (depth >= distance) {
      if (maximumOvershoot <= 0) {
        fail('contact search exhausted its distance bound without touch (measured axial depth $depth m, nominal bound $distance m)');
        return;
      }
      if (endpointReachedAt == null) endpointReachedAt = now;
      awaitingEndpointContact = true;
      if (now - cast(endpointReachedAt, Int64) >= maxAgeNs)
        fail('contact search found no fresh touch within its bounded endpoint wait (measured axial depth $depth m, nominal bound $distance m, maximum contact stopping allowance $maximumOvershoot m)');
    }
  }
}
