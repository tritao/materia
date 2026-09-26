package motionkit.planner;

/**
 * One-dimensional profile from a moving start to rest at a target.
 *
 * The speed changes towards the cruise speed, cruises, then brakes, all at a
 * single acceleration. A start moving away from the target first brakes
 * through zero. When the target is closer than the start's stopping
 * distance, the profile ends where braking at the acceleration allows, past
 * the requested target; `endPosition` reports where it rests.
 */
class JogProfile {
  public final durationSeconds:Float;
  public final endPosition:Float;
  final startTimes:Array<Float> = [];
  final startPositions:Array<Float> = [];
  final startVelocities:Array<Float> = [];
  final accelerations:Array<Float> = [];

  public function new(start:Float, startVelocity:Float, target:Float, cruiseSpeed:Float,
      acceleration:Float) {
    if (!Math.isFinite(start) || !Math.isFinite(startVelocity) || !Math.isFinite(target))
      throw "Jog profile positions and velocity must be finite";
    if (!Math.isFinite(cruiseSpeed) || cruiseSpeed <= 0.0)
      throw "Jog profile cruise speed must be finite and positive";
    if (!Math.isFinite(acceleration) || acceleration <= 0.0)
      throw "Jog profile acceleration must be finite and positive";
    var time = 0.0;
    var position = start;
    var velocity = startVelocity;
    function phase(signedAcceleration:Float, duration:Float):Void {
      if (duration <= 0.0) return;
      startTimes.push(time);
      startPositions.push(position);
      startVelocities.push(velocity);
      accelerations.push(signedAcceleration);
      position += velocity * duration + 0.5 * signedAcceleration * duration * duration;
      velocity += signedAcceleration * duration;
      time += duration;
    }

    var direction = sign(target - position);
    if (velocity != 0.0 && sign(velocity) != direction) {
      // Moving away from the target, or it is already reached: brake to rest.
      phase(-sign(velocity) * acceleration, Math.abs(velocity) / acceleration);
      velocity = 0.0;
      direction = sign(target - position);
    }
    var speed = Math.abs(velocity);
    var distance = Math.abs(target - position);
    // The target may be closer than braking from this speed allows.
    distance = Math.max(distance, speed * speed / (2.0 * acceleration));
    if (direction == 0.0) direction = sign(velocity);
    if (speed > cruiseSpeed) {
      phase(-direction * acceleration, (speed - cruiseSpeed) / acceleration);
      var cruise = distance - speed * speed / (2.0 * acceleration);
      phase(0.0, cruise / cruiseSpeed);
      phase(-direction * acceleration, cruiseSpeed / acceleration);
    } else {
      var peak = Math.min(cruiseSpeed,
        Math.sqrt((2.0 * acceleration * distance + speed * speed) * 0.5));
      phase(direction * acceleration, (peak - speed) / acceleration);
      var cruise = distance - (peak * peak - speed * speed) / (2.0 * acceleration) -
        peak * peak / (2.0 * acceleration);
      if (peak > 0.0) phase(0.0, Math.max(0.0, cruise) / peak);
      phase(-direction * acceleration, peak / acceleration);
    }
    durationSeconds = time;
    endPosition = position;
  }

  /** Position at `timeSeconds`, clamped to the profile's span. */
  public function positionAt(timeSeconds:Float):Float {
    if (startTimes.length == 0 || timeSeconds >= durationSeconds) return endPosition;
    var index = phaseAt(timeSeconds);
    var elapsed = Math.max(0.0, timeSeconds - startTimes[index]);
    return startPositions[index] + startVelocities[index] * elapsed +
      0.5 * accelerations[index] * elapsed * elapsed;
  }

  /** Velocity at `timeSeconds`; zero once the profile has ended. */
  public function velocityAt(timeSeconds:Float):Float {
    if (startTimes.length == 0 || timeSeconds >= durationSeconds) return 0.0;
    var index = phaseAt(timeSeconds);
    return startVelocities[index] + accelerations[index] * Math.max(0.0, timeSeconds - startTimes[index]);
  }

  function phaseAt(timeSeconds:Float):Int {
    var index = 0;
    while (index + 1 < startTimes.length && startTimes[index + 1] <= timeSeconds) index++;
    return index;
  }

  static function sign(value:Float):Float return value > 0.0 ? 1.0 : value < 0.0 ? -1.0 : 0.0;
}
